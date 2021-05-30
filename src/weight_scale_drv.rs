use crate::twim;

use embassy::time::{self, Duration};
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

// Safety: Transfer futures must be awaited.
impl<'a> WeightScaleDrv<'a> {
    pub fn new(twim: &'a mut twim::Twim, addr: Address) -> Self {
        Self { twim, addr }
    }

    pub async fn set_valve(&mut self, open: bool) -> Result<(), Error> {
        self.twim
            .transfer()?
            .write_byte(if open {
                Command::OpenValve
            } else {
                Command::CloseValve
            } as u8)?
            .start(self.addr)
            .await
            .map(|_| ())
    }

    pub async fn set_watchdog(&mut self, enabled: bool) -> Result<(), Error> {
        self.twim
            .transfer()?
            .write_byte(if enabled {
                Command::EnableWd
            } else {
                Command::DisableWd
            } as u8)?
            .start(self.addr)
            .await
            .map(|_| ())
    }

    pub async fn sleep(&mut self) -> Result<(), Error> {
        self.twim
            .transfer()?
            .write_byte(Command::Sleep as u8)?
            .start(self.addr)
            .await
            .map(|_| ())
    }

    pub async fn read_temperature(&mut self) -> Result<u8, Error> {
        self.twim
            .transfer()?
            .write_byte(Command::GetTemp as u8)?
            .start(self.addr)
            .await?;
        time::Delay::new().delay_ms(2).await;
        let mut retries: usize = 0;
        Ok(loop {
            match self.twim.transfer()?.read(1)?.start(self.addr).await {
                Err(twim::Error::AddressNack) if retries < 10 => retries += 1,
                res => break res,
            }
        }?[0])
    }

    pub async fn measure_weight(&mut self, time: Duration) -> Result<u32, Error> {
        self.twim.transfer()?.write(&[Command::MeasureWeight as u8, 1])?.start(self.addr).await?;
        time::Timer::after(time).await;
        let buf = self.twim.transfer()?.read(5)?.start(self.addr).await?;
        let c = buf[0] as u32;
        Ok(((((buf[1] as u32) << 24)
            | ((buf[2] as u32) << 16)
            | ((buf[3] as u32) << 8)
            | (buf[4] as u32))
            + c / 2)
            / c)
    }
}
