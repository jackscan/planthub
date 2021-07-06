#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(pin_static_ref)]
#![feature(type_alias_impl_trait)]
#![feature(generic_associated_types)]
#![feature(const_fn_trait_bound)]
#![allow(incomplete_features)]

use core::fmt::Write;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use defmt::{error, info, unwrap};
use defmt_rtt as _;
use embassy::executor::Spawner;
use embassy::io;
use embassy::io::{AsyncBufRead, AsyncWrite, AsyncWriteExt};
use embassy::time::{Duration, Instant, Timer};
use embassy::util::{Forever, Signal};
use futures::{pin_mut, Stream};
use futures_intrusive::channel::LocalChannel;
use futures_util::future::{poll_fn, Either};
use panic_probe as _;

use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, peripherals, rtc, uarte, Peripherals};
use hal::wdt::{self, Watchdog, WatchdogHandle};
use nrf52832_hal as hal;
use nrf_softdevice::{raw, Softdevice};

#[macro_use]
mod serial;
use serial::{SerialChannel, SerialSink};

mod ble;
mod plant;
mod twim;
mod weight_scale_drv;

use crate::weight_scale_drv::WeightScaleDrv;

const WD_TIMEOUT: Duration = Duration::from_millis(500);
static CLOCK_STARTED: AtomicBool = AtomicBool::new(false);

defmt::timestamp!("{=u64}", {
    if CLOCK_STARTED.load(Ordering::Acquire) {
        embassy::time::Instant::now().as_millis()
    } else {
        0
    }
});

trait AsyncReadWrite: AsyncBufRead + AsyncWrite {}

async fn writeln_uart_fmt(mut uart: Pin<&mut dyn AsyncReadWrite>, args: core::fmt::Arguments<'_>) {
    if let Err(_) = async {
        let mut buf = arrayvec::ArrayString::<128>::new();
        let res = buf.write_fmt(args);
        if let Err(_) = res {
            error!("formatting error");
            Err(io::Error::InvalidData)
        } else {
            info!("uart: {}", buf.as_str());
            uart.write_all(buf.as_bytes()).await?;
            uart.write_all(b"\r\n").await
        }
    }
    .await
    {
        error!("failed to write to uart");
    }
}

macro_rules! writeln_uart {
    ($dst:expr, $($arg:tt)*) => {
        writeln_uart_fmt($dst, core::format_args!($($arg)*)).await;
    }
}

#[embassy::task]
async fn serial_task(
    mut uart: Pin<&'static mut dyn AsyncReadWrite>,
    serial: &'static SerialChannel,
    cmd_sig: &'static Signal<serial::TwiCmd>,
) {
    info!("serial initialized");
    unwrap!(uart.write_all(b"Hello Serial!\r\n").await);

    type SerialOutBuffer = arrayvec::ArrayVec<u8, 64>;
    type SerialInBuffer = arrayvec::ArrayVec<u8, 64>;

    enum SerialTask {
        SerialInput(serial::Result<serial::TwiCmd>),
        ChannelClosed,
        InputTooLong,
    }

    let mut in_buf = SerialInBuffer::new();
    let mut out_buf = SerialOutBuffer::new();
    let out_stream = serial.stream();
    pin_mut!(out_stream);

    loop {
        // let mut n = 0usize;
        let task = poll_fn(|cx| loop {
            // move bytes from local buffer into uart buffer
            let out_pending = out_buf.is_empty()
                || match uart.as_mut().poll_write(cx, out_buf.as_ref()) {
                    Poll::Ready(Ok(num_written)) => {
                        // pop written bytes from front
                        out_buf.copy_within(num_written.., 0);
                        out_buf.truncate(out_buf.len() - num_written);
                        false
                    }
                    Poll::Ready(Err(e)) => {
                        error!("failed to write to uart: {}", e);
                        true
                    }
                    Poll::Pending => true,
                };

            // push next byte from channel stream into local buffer
            let stream_pending = out_buf.is_full()
                || match out_stream.as_mut().poll_next(cx) {
                    Poll::Ready(Some(b)) => {
                        // push next byte into local buffer
                        out_buf.push(b);
                        false
                    }
                    Poll::Ready(None) => {
                        // channel has been closed
                        return Poll::Ready(SerialTask::ChannelClosed);
                    }
                    Poll::Pending => true,
                };

            // helper to check for line break characters
            fn is_line_break(b: u8) -> bool {
                b == b'\r' || b == b'\n'
            }

            // parse uart input
            let in_pending = match uart.as_mut().poll_fill_buf(cx) {
                Poll::Ready(Ok(buf)) => {
                    let has_line_brk = buf.iter().position(|b| is_line_break(*b));
                    let n = if let Some(p) = has_line_brk {
                        p
                    } else {
                        buf.len()
                    };
                    let xr = in_buf.try_extend_from_slice(&buf[..n]);

                    let n = if let Some(p) = has_line_brk {
                        p + &buf[p..].iter().take_while(|b| is_line_break(**b)).count()
                    } else {
                        n
                    };
                    uart.as_mut().consume(n);

                    if let Err(_) = xr {
                        in_buf.clear();
                        // TODO: read until next new line before considering any input again
                        return Poll::Ready(SerialTask::InputTooLong);
                    } else if let Some(_) = has_line_brk {
                        let r = serial::parse_cmd(in_buf.as_ref()).transpose();
                        in_buf.clear();
                        if let Some(r) = r {
                            return Poll::Ready(SerialTask::SerialInput(r));
                        }
                    }

                    false
                }
                Poll::Ready(Err(e)) => {
                    error!("failed to read from uart: {}", e);
                    true
                }
                Poll::Pending => true,
            };

            if out_pending && stream_pending && in_pending {
                return Poll::Pending;
            }
        })
        .await;

        match task {
            SerialTask::SerialInput(Ok(cmd)) => {
                cmd_sig.signal(cmd);
            }
            SerialTask::SerialInput(Err(e)) => {
                writeln_uart!(uart.as_mut(), "parse error: {:?}", e);
            }
            SerialTask::InputTooLong => {
                writeln_uart!(uart.as_mut(), "input line exceeds limit");
            }
            SerialTask::ChannelClosed => break,
        }
    }

    writeln_uart!(uart, "Serial closed");
}

async fn run_serial_cmd<'a>(
    twim: &mut twim::Twim,
    serial: &SerialSink<'a>,
    cmd: serial::TwiCmd,
    wd_hndl: &mut WatchdogHandle<wdt::handles::HdlN>,
) {
    let (twim, stopper) = twim.borrow_stoppable();

    let total = cmd.timeout();
    let n = (2 * total.as_ticks() / WD_TIMEOUT.as_ticks()) as u32;
    let rem = total - WD_TIMEOUT * n / 2;
    info!("timeout: {} x {} + {}", WD_TIMEOUT / 2, n, rem);

    let transfer = cmd.run(twim, &serial);
    pin_mut!(transfer);

    for _ in 0..n {
        let timer = Timer::after(WD_TIMEOUT / 2);

        if let Either::Left((_, _)) = futures::future::select(transfer.as_mut(), timer).await {
            return;
        }

        wd_hndl.pet();
    }

    let timer = Timer::after(rem);
    match futures::future::select(transfer, timer).await {
        Either::Left((_, _)) => return,
        Either::Right((_, transfer)) => {
            stopper.stop();
            info!("stopping");
            writeln_serial!(serial, "timeout after {} ms", rem.as_millis());
            // need to await the stopped transfer
            transfer.await
        }
    };
}

async fn scan_twim<const CAP: usize>(
    twim: &mut twim::Twim,
    start: twim::Address,
    end: twim::Address,
    list: &mut arrayvec::ArrayVec<twim::Address, CAP>,
    wd_hndl: &mut WatchdogHandle<wdt::handles::HdlN>,
) -> usize {
    let mut count = 0;
    for addr in start..end {
        let mut drv = WeightScaleDrv::new(twim, addr);
        if drv.sleep().await.is_ok() {
            count += 1;
            if list.len() <= list.capacity() {
                list.push(addr);
            }
        }
        wd_hndl.pet();
    }
    count
}

async fn measure_plants(
    twim: &mut twim::Twim,
    list: &[twim::Address],
    plant_states: &plant::StatusList<'_>,
    wd_hndl: &mut WatchdogHandle<wdt::handles::HdlN>,
) {
    for &addr in list {
        let mut drv = WeightScaleDrv::new(twim, addr);
        match drv.read_temperature().await {
            Err(e) => error!("plant {:x}: error on reading temperature: {}", addr, e),
            Ok(t) => {
                info!("plant {:x}: temperature: {}", addr, t);
                if let Err(e) = drv.start_weight_measurement().await {
                    error!(
                        "plant {:x}: error on starting weight measurement: {}",
                        addr, e
                    );
                } else {
                    // wait for 7 seconds
                    const WAIT_TIME: Duration = Duration::from_millis(7000);
                    const NUM_WD_CYCLES: u64 = WAIT_TIME.as_ticks() * 2 / WD_TIMEOUT.as_ticks();
                    for _ in 0..NUM_WD_CYCLES {
                        Timer::after(WD_TIMEOUT / 2).await;
                        wd_hndl.pet();
                    }
                    // read weight
                    match drv.read_weight().await {
                        Ok(w) => {
                            info!("plant {:x}: weight: {}", addr, w);
                            plant_states
                                .update(plant::Info {
                                    id: addr as _,
                                    temperature: t,
                                    weight: core::cmp::max(w, 0xFFFF) as u16,
                                })
                                .await;
                        }
                        Err(e) => error! {"plant {:x}: error on reading weight: {}", addr, e},
                    }

                    if let Err(e) = drv.sleep().await {
                        error! {"plant {:x}: error sending sleep command: {}", addr, e};
                    }
                }
            }
        }
    }
}

#[embassy::task]
async fn worker_task(
    mut twim: twim::Twim,
    sercmd_sig: &'static Signal<serial::TwiCmd>,
    btcmd_sig: &'static Signal<ble::Command>,
    serial: &'static SerialChannel,
    mut wd_hndl: WatchdogHandle<wdt::handles::HdlN>,
    plant_states: &'static plant::StatusList<'static>,
) {
    let serial = SerialSink::new(serial);

    let mut upd_time = Instant::now() + Duration::from_secs(60);
    let mut plant_list = arrayvec::ArrayVec::<twim::Address, 8>::new();

    let n = scan_twim(&mut twim, 0x40, 0x60, &mut plant_list, &mut wd_hndl).await;

    info!("Found {} plant sensor(s)", n);

    loop {
        wd_hndl.pet();

        let sercmd_fut = sercmd_sig.wait();
        let btcmd_fut = btcmd_sig.wait();
        let cmd_fut = futures::future::select(sercmd_fut, btcmd_fut);
        let wd_timer = Timer::after(WD_TIMEOUT / 2);
        let upd_timer = Timer::at(upd_time);
        let time_fut = futures::future::select(upd_timer, wd_timer);

        // wait for cmd or time to pet watchdog
        let cmd = match futures::future::select(cmd_fut, time_fut).await {
            Either::Left((cmd, _)) => cmd,
            Either::Right((Either::Left(_), _)) => {
                measure_plants(&mut twim, &plant_list, plant_states, &mut wd_hndl).await;
                while upd_time < Instant::now() {
                    upd_time += Duration::from_secs(60);
                }
                continue;
            }
            Either::Right((Either::Right(_), _)) => continue,
        };

        wd_hndl.pet();

        match cmd {
            Either::Left((sercmd, _)) => {
                run_serial_cmd(&mut twim, &serial, sercmd, &mut wd_hndl).await;
            }
            Either::Right((btcmd, _)) => info!("TODO: dispatch {}", btcmd),
        };
    }
}

#[embassy::task]
async fn ble_task(
    sd: &'static Softdevice,
    cmd_sig: &'static Signal<ble::Command>,
    plant_states: &'static plant::StatusList<'static>,
) {
    let server = ble::Server::new(sd);

    loop {
        let conn = server.advertise().await;
        server.serve(conn, cmd_sig, plant_states).await;
    }
}

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

type Uarte = BufferedUarte<'static, peripherals::UARTE0, peripherals::TIMER1>;
impl<'a> AsyncReadWrite for Uarte {}

static EXECUTOR: Forever<embassy::executor::Executor> = Forever::new();
static RTC: Forever<rtc::RTC<peripherals::RTC1>> = Forever::new();
static ALARM: Forever<rtc::Alarm<peripherals::RTC1>> = Forever::new();
static UARTE: Forever<Uarte> = Forever::new();
static SERCMD_SIGNAL: Forever<Signal<serial::TwiCmd>> = Forever::new();
static SERIAL_CH: Forever<SerialChannel> = Forever::new();
static BTCMD_SIGNAL: Forever<Signal<ble::Command>> = Forever::new();
static PLANT_STATES: Forever<plant::StatusList> = Forever::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    let p = embassy_nrf::init(config);

    let sd_config = nrf_softdevice::Config {
        // LFCLK oscillator source.
        clock: Some(raw::nrf_clock_lf_cfg_t {
            // LFCLK crystal oscillator
            source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            // Only for ::NRF_CLOCK_LF_SRC_RC:
            // Calibration timer interval in 1/4 second units (nRF52: 1-32).
            rc_ctiv: 0,
            // Only for ::NRF_CLOCK_LF_SRC_RC:
            // How often (in number of calibration intervals) the RC oscillator shall be calibrated if the temperature hasn't changed. 0: Always calibrate even if the temperature hasn't changed. 1: Only calibrate if the temperature has changed (legacy - nRF51 only). 2-33: Check the temperature and only calibrate if it has changed, however calibration will take place every rc_temp_ctiv intervals in any case.
            rc_temp_ctiv: 0,
            // External clock accuracy used in the LL to compute timing windows.
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
        }),
        // BLE GAP connection configuration parameters.
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            // The number of concurrent connections the application can create with this configuration.
            conn_count: 3,
            // The time set aside for this connection on every connection interval in 1.25 ms units.
            event_length: raw::BLE_GAP_EVENT_LENGTH_DEFAULT as u16,
        }),
        // Maximum size of ATT packet the SoftDevice can send or receive.
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        // Attribute table size configuration.
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            // Attribute table size.
            attr_tab_size: 32768,
        }),
        // Configuration of maximum concurrent connections in the peripheral role.
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            // Maximum number of advertising sets.
            adv_set_count: 1,
            // Maximum number of connections concurrently acting as a peripheral.
            periph_role_count: 3,
            // central_role_count: 3,
            // central_sec_count: 0,
            // _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        // Device name and its properties.
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            // Pointer to where the value (device name) is stored or will be stored.
            p_value: b"HelloRust" as *const u8 as _,
            // Current length in bytes of the memory pointed to by p_value.
            current_len: 9,
            // Maximum length in bytes of the memory pointed to by p_value.
            max_len: 9,
            // Write permissions.
            write_perm: unsafe { core::mem::zeroed() },
            // Value location.
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                // Attribute Value is located in stack memory, no user memory is required.
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&sd_config);

    let rtc = RTC.put(rtc::RTC::new(
        unsafe { <peripherals::RTC1 as embassy::util::Steal>::steal() },
        interrupt::take!(RTC1),
    ));
    rtc.start();
    let alarm = ALARM.put(rtc.alarm0());
    unsafe { embassy::time::set_clock(rtc) };
    CLOCK_STARTED.store(true, Ordering::Release);

    let pp = hal::pac::Peripherals::take().unwrap();
    let mut wd = match Watchdog::try_new(pp.WDT) {
        Ok(wd) => wd,
        Err(_) => defmt::panic!("Watchdog already active"),
    };

    wd.set_lfosc_ticks(WD_TIMEOUT.as_ticks() as u32);

    let wdt::Parts {
        watchdog: _,
        handles: (wrk_wd_hndl,),
    } = wd.activate::<wdt::count::One>();

    let executor = EXECUTOR.put(embassy::executor::Executor::new());
    executor.set_alarm(alarm);

    executor.run(|spawner| {
        let mut config = uarte::Config::default();
        config.parity = uarte::Parity::EXCLUDED;
        config.baudrate = uarte::Baudrate::BAUD115200;
        static mut RX_BUF: [u8; 32] = [0u8; 32];
        static mut TX_BUF: [u8; 32] = [0u8; 32];

        let uart = UARTE.put(unsafe {
            BufferedUarte::new(
                p.UARTE0,
                p.TIMER1,
                p.PPI_CH0,
                p.PPI_CH1,
                interrupt::take!(UARTE0_UART0),
                p.P0_08,
                p.P0_06,
                NoPin,
                NoPin,
                config,
                &mut RX_BUF,
                &mut TX_BUF,
            )
        });

        let uart = Pin::static_mut(uart);

        static mut TWIM_BUF: [u8; 32] = [0u8; 32];
        let twim = twim::Twim::new(
            p.TWISPI0,
            interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0),
            p.P0_26,
            p.P0_25,
            twim::Frequency::K100,
            unsafe { &mut TWIM_BUF },
        );

        let serial_ch = SERIAL_CH.put(LocalChannel::new());
        let sercmd_sig = SERCMD_SIGNAL.put(Signal::new());
        let btcmd_sig = BTCMD_SIGNAL.put(Signal::new());

        static mut PLANT_STATUS_ARRAY: [plant::StampedInfo; 8] =
            [(0, plant::Info::new(0, 0, 0)); 8];
        let plant_states =
            PLANT_STATES.put(plant::StatusList::new(unsafe { &mut PLANT_STATUS_ARRAY }));

        unwrap!(spawner.spawn(softdevice_task(sd)));
        unwrap!(spawner.spawn(ble_task(sd, btcmd_sig, plant_states)));
        unwrap!(spawner.spawn(serial_task(uart, serial_ch, sercmd_sig)));
        unwrap!(spawner.spawn(worker_task(
            twim,
            sercmd_sig,
            btcmd_sig,
            serial_ch,
            wrk_wd_hndl.degrade(),
            plant_states,
        )));
    });
}
