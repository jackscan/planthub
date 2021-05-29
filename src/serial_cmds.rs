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
}

macro_rules! writeln_serial {
    ($dst:expr, $($arg:tt)*) => {
        if let Err(_) = async {
            let mut buf = arrayvec::ArrayString::<64>::new();
            let res = buf.write_fmt(core::format_args!($($arg)*));
            if let Err(_) = res {
                error!("formatting error");
                Err(futures_intrusive::channel::ChannelSendError(b'\0'))
            } else {
                info!("ch: {}", buf.as_str());
                $dst.write(buf.as_str()).await?;
                $dst.write("\r\n").await
            }
        }.await {
            error!("failed to write to serial");
        }
    }
}

pub type Result<T> = core::result::Result<T, ParseError>;

pub type TwiAddr = twim::Address;

#[derive(defmt::Format, Debug, Default)]
pub struct TwiData {
    pub len: u8,
    pub buf: [u8; 7],
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
}

impl TwiCmd {
    pub fn timeout(&self) -> Duration {
        Duration::from_millis(match self {
            TwiCmd::Write(_) => 1,
            TwiCmd::Read(_, _) => 1,
            TwiCmd::WriteRead(_, _) => 1,
            TwiCmd::ReadTemperature(_) => 3,
        })
    }

    pub async fn run(self, twim: &mut twim::Twim, serial: &SerialSink<'_>) {
        let mut buf = [0u8; 16];
        match self {
            TwiCmd::Write(cmd) => {
                writeln_serial!(
                    serial,
                    "write({}, {:?})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize]
                );
                let r = twim
                    .write(cmd.addr, &cmd.data.buf[0..cmd.data.len as usize])
                    .await;
                match &r {
                    Ok(()) => writeln_serial!(serial, "write succeeded"),
                    Err(e) => writeln_serial!(serial, "write failed: {:?}", e),
                }
            }
            TwiCmd::Read(addr, count) => {
                writeln_serial!(serial, "read({}, #{})", addr, count);
                let count = count as usize;
                if count > buf.len() {
                    error!("read buffer too small");
                    return;
                }
                let buf = &mut buf[0..count];
                let r = twim.read(addr, buf).await;
                match &r {
                    Ok(()) => writeln_serial!(serial, "read: {:?}", buf),
                    Err(e) => writeln_serial!(serial, "read failed: {:?}", e),
                }
            }
            TwiCmd::WriteRead(cmd, count) => {
                writeln_serial!(
                    serial,
                    "write({}, {:?}), read(#{})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize],
                    count
                );
                let count = count as usize;
                if count > buf.len() {
                    error!("read buffer too small");
                    return;
                }
                let buf = &mut buf[0..count];
                let r = twim
                    .write_read(cmd.addr, &cmd.data.buf[0..cmd.data.len as usize], buf)
                    .await;
                match &r {
                    Ok(()) => writeln_serial!(serial, "read: {:?}", buf),
                    Err(e) => writeln_serial!(serial, "write/read failed: {:?}", e),
                }
            }
            TwiCmd::ReadTemperature(addr) => {
                writeln_serial!(serial, "reading temp");
                let mut drv = WeightScaleDrv::new(twim, addr);
                match drv.read_temperature().await {
                    Ok(temp) => {
                        writeln_serial!(serial, "temperature: {}", temp);
                    }
                    Err(e) => {
                        writeln_serial!(serial, "reading temperature failed: {:?}", e);
                    }
                }
            }
        }
    }
}

pub struct TwiCmdParser<'a> {
    line: &'a str,
    pos: usize,
}

impl<'a> TwiCmdParser<'a> {
    pub fn from_str(line: &'a str) -> Self {
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

    fn parse_addr(&mut self) -> Result<TwiAddr> {
        self.parse_byte()?
            .ok_or(ParseError::new("expected address", self.pos))
    }

    fn parse_count(&mut self) -> Result<u8> {
        self.parse_byte()?
            .ok_or(ParseError::new("expected count", self.pos))
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
