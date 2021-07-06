use core::fmt::Write;
use core::str;

use defmt::{error, info};
use embassy::time::Duration;
use futures_intrusive::channel::LocalChannel;

use crate::twim;
use crate::weight_scale_drv::WeightScaleDrv;

type SerialBuffer = [u8; 64];
pub type SerialChannel = LocalChannel<u8, SerialBuffer>;
pub struct SerialSink<'a> {
    ch: &'a SerialChannel,
}

#[derive(Debug, defmt::Format)]
pub struct ParseError {
    pub msg: &'static str,
    pub pos: usize,
}

impl ParseError {
    fn new(msg: &'static str, pos: usize) -> Self {
        Self { msg, pos }
    }
}

impl From<str::Utf8Error> for ParseError {
    fn from(e: str::Utf8Error) -> Self {
        Self {
            msg: "invalid utf8 sequence",
            pos: e.valid_up_to(),
        }
    }
}

impl<'a> SerialSink<'a> {
    pub fn new(ch: &'a SerialChannel) -> Self {
        SerialSink { ch }
    }
    pub async fn write(
        &self,
        s: &str,
    ) -> core::result::Result<(), futures_intrusive::channel::ChannelSendError<u8>> {
        for &b in s.as_bytes() {
            self.ch.send(b).await?;
        }
        Ok(())
    }

    pub async fn writeln_fmt(&self, args: core::fmt::Arguments<'_>) {
        if let Err(_) = async {
            let mut buf = arrayvec::ArrayString::<128>::new();
            let res = buf.write_fmt(args);
            if let Err(_) = res {
                error!("formatting error");
                Err(futures_intrusive::channel::ChannelSendError(b'\0'))
            } else {
                info!("ch: {}", buf.as_str());
                self.write(buf.as_str()).await?;
                self.write("\r\n").await
            }
        }
        .await
        {
            error!("failed to write to serial");
        }
    }
}

macro_rules! writeln_serial {
    ($dst:expr, $($arg:tt)*) => {
            $dst.writeln_fmt(core::format_args!($($arg)*)).await;
    }
}

pub type Result<T> = core::result::Result<T, ParseError>;

pub type TwiAddr = twim::Address;

#[derive(defmt::Format, Debug, Default)]
pub struct TwiData {
    pub len: u8,
    pub buf: [u8; 16],
}

#[derive(defmt::Format, Debug)]
pub struct TwiWrite {
    pub addr: TwiAddr,
    pub data: TwiData,
}

#[derive(defmt::Format, Debug)]
pub enum TwiCmd {
    Write(TwiWrite),
    Read(TwiAddr, u8),
    WriteRead(TwiWrite, u8),
    ReadTemperature(TwiAddr),
    SetValve(TwiAddr, bool),
    SetWatchdog(TwiAddr, bool),
    MeasureWeight(TwiAddr, u16),
    GetVersion(TwiAddr),
    Sleep(TwiAddr),
}

impl TwiCmd {
    pub fn timeout(&self) -> Duration {
        Duration::from_millis(match self {
            TwiCmd::Write(_) => 10,
            TwiCmd::Read(_, _) => 10,
            TwiCmd::WriteRead(_, _) => 20,
            TwiCmd::ReadTemperature(_) => 20,
            TwiCmd::SetValve(_, _) => 10,
            TwiCmd::SetWatchdog(_, _) => 10,
            TwiCmd::MeasureWeight(_, ms) => (ms + 20) as u64,
            TwiCmd::GetVersion(_) => 10,
            TwiCmd::Sleep(_) => 10,
        })
    }

    pub async fn run(self, twim: &mut twim::Twim, serial: &SerialSink<'_>) {
        match &self.exec(twim, serial).await {
            Ok(()) => writeln_serial!(serial, "success"),
            Err(e) => writeln_serial!(serial, "failed: {:?}", e),
        }
    }

    async fn exec(
        &self,
        twim: &mut twim::Twim,
        serial: &SerialSink<'_>,
    ) -> core::result::Result<(), twim::Error> {
        match self {
            TwiCmd::Write(cmd) => {
                writeln_serial!(
                    serial,
                    "write({}, {:?})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize]
                );
                twim.transfer()?
                    .write(&cmd.data.buf[0..cmd.data.len as usize])?
                    .start(cmd.addr)
                    .await?;
            }
            &TwiCmd::Read(addr, count) => {
                writeln_serial!(serial, "read({}, #{})", addr, count);
                let buf = twim.transfer()?.read(count as usize)?.start(addr).await?;
                writeln_serial!(serial, "read: {:?}", buf);
            }
            TwiCmd::WriteRead(cmd, count) => {
                writeln_serial!(
                    serial,
                    "write({}, {:?}), read(#{})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize],
                    count
                );
                let buf = twim
                    .transfer()?
                    .write(&cmd.data.buf[0..cmd.data.len as usize])?
                    .read(*count as usize)?
                    .start(cmd.addr)
                    .await?;
                writeln_serial!(serial, "read: {:?}", buf);
            }
            &TwiCmd::SetValve(addr, open) => {
                writeln_serial!(serial, "{} valve", if open { "opening" } else { "closing" });
                let mut drv = WeightScaleDrv::new(twim, addr);
                drv.set_valve(open).await?;
            }
            &TwiCmd::SetWatchdog(addr, enabled) => {
                writeln_serial!(
                    serial,
                    "{} watchdog",
                    if enabled { "enabling" } else { "disabling" }
                );
                let mut drv = WeightScaleDrv::new(twim, addr);
                drv.set_watchdog(enabled).await?;
            }
            &TwiCmd::ReadTemperature(addr) => {
                writeln_serial!(serial, "reading temp");
                let mut drv = WeightScaleDrv::new(twim, addr);
                let temp = drv.read_temperature().await?;
                writeln_serial!(serial, "temperature: {}", temp);
            }
            &TwiCmd::MeasureWeight(addr, ms) => {
                writeln_serial!(serial, "measuring weight for {} ms", ms);
                let mut drv = WeightScaleDrv::new(twim, addr);
                let w = drv.measure_weight(Duration::from_millis(ms as u64)).await?;
                writeln_serial!(serial, "weight: {}", w);
                drv.sleep().await?;
            }
            &TwiCmd::GetVersion(addr) => {
                let mut drv = WeightScaleDrv::new(twim, addr);
                let ver = drv.read_version().await?;
                writeln_serial!(
                    serial,
                    "version {}.{}.{} {:x}",
                    ver.major,
                    ver.minor,
                    ver.patch,
                    ver.hash
                );
                drv.sleep().await?;
            }
            &TwiCmd::Sleep(addr) => {
                writeln_serial!(serial, "sleep");
                let mut drv = WeightScaleDrv::new(twim, addr);
                drv.sleep().await?;
            }
        }
        Ok(())
    }
}

struct TwiCmdParser<'a> {
    line: &'a str,
    pos: usize,
}

impl<'a> TwiCmdParser<'a> {
    fn from_str(line: &'a str) -> Self {
        TwiCmdParser { line, pos: 0 }
    }

    fn remains(&self) -> &str {
        &self.line[self.pos..]
    }

    fn len(&self) -> usize {
        self.line.len() - self.pos
    }

    fn advance(&mut self, n: usize) {
        self.pos = self.pos + n;
    }

    fn pop_front(&mut self, n: usize) -> &str {
        let p = self.pos;
        self.advance(n);
        &self.line[p..p + n]
    }

    fn next_word(&mut self) -> Option<(&str, usize)> {
        if let Some(i) = self.remains().find(|c: char| !c.is_whitespace()) {
            self.advance(i);
            let i = self
                .remains()
                .find(char::is_whitespace)
                .unwrap_or(self.len());
            let p = self.pos;
            Some((self.pop_front(i), p))
        } else {
            self.advance(self.len());
            None
        }
    }

    fn parse_byte(&mut self) -> Result<Option<u8>> {
        Ok(match self.next_word() {
            Some((s, p)) => Some(
                if let Some(s) = s.strip_prefix("0x") {
                    u8::from_str_radix(s, 16)
                } else if let Some(s) = s.strip_prefix("0") {
                    if s.len() > 0 {
                        u8::from_str_radix(s, 8)
                    } else {
                        Ok(0)
                    }
                } else {
                    u8::from_str_radix(s, 10)
                }
                .map_err(|_| ParseError::new("expected byte", p))?,
            ),
            None => None,
        })
    }

    fn parse_u16(&mut self) -> Result<Option<u16>> {
        Ok(match self.next_word() {
            Some((s, p)) => Some(
                if let Some(s) = s.strip_prefix("0x") {
                    u16::from_str_radix(s, 16)
                } else if let Some(s) = s.strip_prefix("0") {
                    if s.len() > 0 {
                        u16::from_str_radix(s, 8)
                    } else {
                        Ok(0)
                    }
                } else {
                    u16::from_str_radix(s, 10)
                }
                .map_err(|_| ParseError::new("expected 16bit unsigned", p))?,
            ),
            None => None,
        })
    }

    fn parse_addr(&mut self) -> Result<TwiAddr> {
        self.parse_byte()?
            .ok_or(ParseError::new("expected address", self.pos))
    }

    fn parse_count(&mut self) -> Result<u8> {
        self.parse_byte()?
            .ok_or(ParseError::new("expected count", self.pos))
    }

    fn parse_bool(&mut self) -> Result<bool> {
        match self
            .parse_byte()?
            .ok_or(ParseError::new("expected boolean", self.pos))?
        {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(ParseError::new("expected 0 or 1", self.pos)),
        }
    }

    fn parse_data(&mut self) -> Result<TwiData> {
        let mut data: TwiData = Default::default();
        while let Some(b) = self.parse_byte()? {
            if (data.len as usize) >= data.buf.len() {
                return Err(ParseError::new("too many data bytes", self.pos));
            }
            data.buf[data.len as usize] = b;
            data.len = data.len + 1;
        }
        Ok(data)
    }

    pub fn parse(&mut self) -> Result<Option<TwiCmd>> {
        Ok(if let Some((w, p)) = self.next_word() {
            Some(match w {
                "r" => {
                    let addr = self.parse_addr()?;
                    let count = self.parse_count()?;
                    TwiCmd::Read(addr, count)
                }
                "w" => TwiCmd::Write(TwiWrite {
                    addr: self.parse_addr()?,
                    data: self.parse_data()?,
                }),
                "wr" => {
                    let addr = self.parse_addr()?;
                    let count = self.parse_count()?;
                    TwiCmd::WriteRead(
                        TwiWrite {
                            addr,
                            data: self.parse_data()?,
                        },
                        count,
                    )
                }
                "t" => TwiCmd::ReadTemperature(self.parse_addr()?),
                "v" => {
                    let addr = self.parse_addr()?;
                    let open = self.parse_bool()?;
                    TwiCmd::SetValve(addr, open)
                }
                "wd" => {
                    let addr = self.parse_addr()?;
                    let enabled = self.parse_bool()?;
                    TwiCmd::SetWatchdog(addr, enabled)
                }
                "m" => {
                    let addr = self.parse_addr()?;
                    let ms = self
                        .parse_u16()?
                        .ok_or(ParseError::new("expected delay", self.pos))?;
                    TwiCmd::MeasureWeight(addr, ms)
                }
                "i" => TwiCmd::GetVersion(self.parse_addr()?),
                "s" => TwiCmd::Sleep(self.parse_addr()?),
                _ => return Err(ParseError::new("unknown command", p)),
            })
        } else {
            None
        })
    }
}

pub fn parse_cmd(buf: &[u8]) -> Result<Option<TwiCmd>> {
    match str::from_utf8(&buf) {
        Ok(s) => TwiCmdParser::from_str(s).parse(),
        Err(e) => Err(ParseError::new("invalid utf8 sequence", e.valid_up_to())),
    }
}
