use core::{str};

pub type TwiAddr = u8;

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

pub type Result<T> = core::result::Result<T, ParseError>;

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
                    u8::from_str_radix(s, 8)
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
                "t" => {
                    TwiCmd::ReadTemperature(self.parse_addr()?)
                }
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
