use crate::twim;

use defmt::info;
use embassy::time;
use embassy_traits::delay::Delay;

pub use twim::{Address, Error};

pub struct WeightScaleDrv<'a> {
    twim: &'a mut twim::Twim,
    addr: twim::Address,
}

#[repr(u8)]
enum Command {
    Sleep = 0x00,
    MeasureWeight = 0x50,
    OpenValve = 0x51,
    CloseValve = 0x52,
    GetTemp = 0x53,
    SetTemp = 0x54,
    GetCalib = 0x55,
    SetCalib = 0x56,
    EnableWd = 0x57,
    CalibWrite = 0xA0,
    SetAddr = 0xA3,
    AddrWrite = 0xA6,
    DisableWd = 0xA9,
}

impl<'a> WeightScaleDrv<'a> {
    pub fn new(twim: &'a mut twim::Twim, addr: Address) -> Self {
        Self { twim, addr }
    }

    pub async fn read_temperature<'d>(&mut self) -> Result<u8, Error> {
        let mut buf = [Command::GetTemp as u8; 1];
        self.twim.write(self.addr, &buf).await?;
        time::Delay::new().delay_ms(2).await;
        let mut retries: usize = 0;
        let res = loop {
            match self.twim.read(self.addr, &mut buf).await {
                Err(twim::Error::AddressNack) if retries < 10 => retries += 1,
                res => break res,
            }
        };
        res.map(|()| buf[0])
    }
}
