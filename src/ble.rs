use defmt::{error, info};
use embassy::util::Signal;
use futures::future::Either;
use futures::pin_mut;

use nrf_softdevice::ble::{gatt_server, peripheral, Connection, FixedGattValue};
use nrf_softdevice::{raw, Softdevice};

use crate::plant::{Id as PlantId, Info as PlantInfo, StatusList as PlantStates};

#[nrf_softdevice::gatt_server(uuid = "c93d0001-8a71-016b-9c51-646db9d6a8fd")]
struct PlantService {
    #[characteristic(uuid = "c93d0002-8a71-016b-9c51-646db9d6a8fd", read, notify)]
    plant_info: PlantInfo,
    #[characteristic(uuid = "c93d0003-8a71-016b-9c51-646db9d6a8fd", read, write)]
    cmd: Command,
}

#[derive(defmt::Format)]
enum Update {
    Plant(PlantInfo),
}

#[derive(defmt::Format)]
#[repr(C, u8)]
pub enum Command {
    None,
    Measure,
    Watering(PlantId, u16, u16, u16),
}

pub struct Server {
    srv: PlantService,
    sd: &'static Softdevice,
}

impl FixedGattValue for PlantInfo {
    const SIZE: usize = core::mem::size_of::<Self>();

    fn from_gatt(data: &[u8]) -> Self {
        assert_eq!(data.len(), Self::SIZE);
        unsafe { core::ptr::read_unaligned(data.as_ptr() as *const _) }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts((self as *const Self) as *const u8, Self::SIZE) }
    }
}

impl FixedGattValue for Command {
    const SIZE: usize = 8;
    fn from_gatt(data: &[u8]) -> Self {
        assert_eq!(data.len(), Self::SIZE);
        unsafe { core::ptr::read_unaligned(data.as_ptr() as *const _) }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts((self as *const Self) as *const u8, Self::SIZE) }
    }
}

impl Server {
    pub fn new(sd: &'static Softdevice) -> Self {
        Self {
            srv: gatt_server::register(sd).unwrap(),
            sd,
        }
    }

    pub async fn advertise(&self) -> Connection {
        #[rustfmt::skip]
        let adv_data = &[
            0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
            0x03, 0x03, 0x09, 0x18,
            0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
        ];
        #[rustfmt::skip]
        let scan_data = &[
            0x03, 0x03, 0x09, 0x18,
        ];

        let config = peripheral::Config::default();
        defmt::unwrap!(
            peripheral::advertise(
                self.sd,
                peripheral::ConnectableAdvertisement::ScannableUndirected {
                    adv_data,
                    scan_data,
                },
                &config,
            )
            .await
        )
    }

    pub async fn serve(
        &self,
        conn: Connection,
        cmd_sig: &'static Signal<Command>,
        plant_states: &'static PlantStates<'static>,
    ) {
        let ref srv = self.srv;
        defmt::unwrap!(srv.plant_info_set(PlantInfo::new(0, 0, 0)));
        defmt::unwrap!(srv.cmd_set(Command::None));

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        let srvfut = gatt_server::run(&conn, srv, |e| match e {
            PlantServiceEvent::CmdWrite(cmd) => {
                info!("cmd {}", cmd);
                if let Err(e) = srv.cmd_set(cmd) {
                    info!("error: {:?}", e);
                }
            }

            PlantServiceEvent::PlantInfoNotificationsEnabled => {
                info!("notifications enabled");
                // cmd_sig.signal(Command::Measure);
            }
            PlantServiceEvent::PlantInfoNotificationsDisabled => {
                info!("notifications disabled");
                cmd_sig.signal(Command::None);
            }
        });

        futures::pin_mut!(srvfut);
        let mut upd_stamp = None;

        loop {
            let updfut = plant_states.next_update(&mut upd_stamp);
            pin_mut!(updfut);

            match futures::future::select(srvfut, updfut).await {
                Either::Left((res, _)) => {
                    if let Err(e) = res {
                        error!("gatt_server run exited with error: {:?}", e);
                    }
                    break;
                }
                Either::Right((iter, sfut)) => {
                    for upd in iter {
                        info!("update: {}", upd);

                        if let Err(e) = srv.plant_info_set(upd) {
                            error!("failed to set plant_info: {}", e);
                        }
                    }
                    srvfut = sfut;
                }
            };
        }
    }
}
