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
use core::task::Poll;

use defmt::{error, info, unwrap};
use defmt_rtt as _;
use embassy::executor::Spawner;
use embassy::io;
use embassy::io::{AsyncBufRead, AsyncWrite, AsyncWriteExt};
use embassy::util::{Forever, Signal, Steal};
use futures::{pin_mut, Stream};
use futures_intrusive::channel::LocalChannel;
use futures_util::future::poll_fn;
use panic_probe as _;

use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, peripherals, uarte, Peripherals};

mod serial_cmds;
use serial_cmds::TwiCmd;

mod twim;

trait AsyncReadWrite: AsyncBufRead + AsyncWrite {}

type SerialBuffer = [u8; 64];
type SerialChannel = LocalChannel<u8, SerialBuffer>;
struct SerialSink<'a> {
    ch: &'a SerialChannel,
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

macro_rules! writeln_uart {
    ($dst:expr, $($arg:tt)*) => {
        if let Err(_) = async {
            let mut buf = arrayvec::ArrayString::<64>::new();
            let res = buf.write_fmt(core::format_args!($($arg)*));
            if let Err(_) = res {
                error!("formatting error");
                Err(io::Error::InvalidData)
            } else {
                info!("uart: {}", buf.as_str());
                $dst.write_all(buf.as_bytes()).await?;
                $dst.write_all(b"\r\n").await
            }
        }.await {
            error!("failed to write to uart");
        }
    }
}

impl<'a> SerialSink<'a> {
    fn new(ch: &'a SerialChannel) -> Self {
        SerialSink { ch }
    }
    async fn write(&self, s: &str) -> Result<(), futures_intrusive::channel::ChannelSendError<u8>> {
        for &b in s.as_bytes() {
            self.ch.send(b).await?;
        }
        Ok(())
    }
}

#[embassy::task]
async fn serial_task(
    mut uart: Pin<&'static mut dyn AsyncReadWrite>,
    serial: &'static SerialChannel,
    cmd_sig: &'static Signal<TwiCmd>,
) {
    info!("serial initialized");
    unwrap!(uart.write_all(b"Hello Serial!\r\n").await);

    type SerialOutBuffer = arrayvec::ArrayVec<u8, 64>;
    type SerialInBuffer = arrayvec::ArrayVec<u8, 64>;

    enum SerialTask {
        SerialInput(serial_cmds::Result<TwiCmd>),
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
                        let r = serial_cmds::parse_cmd(in_buf.as_ref()).transpose();
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
                writeln_uart!(uart, "parse error: {:?}", e);
            }
            SerialTask::InputTooLong => {
                writeln_uart!(uart, "input line exceeds limit");
            }
            SerialTask::ChannelClosed => break,
        }
    }

    writeln_uart!(uart, "Serial closed");
}

async fn twi_transfer(
    twim: &mut twim::Twim,
    cmd: TwiCmd,
    buf: &mut [u8],
    serial: &SerialSink<'_>,
) -> Result<(), twim::Error> {
    match cmd {
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
                Ok((n, _)) => writeln_serial!(serial, "{} bytes written", n),
                Err(e) => writeln_serial!(serial, "write failed: {:?}", e),
                _ => {}
            }
            r.map(|_| ())
        }
        TwiCmd::Read(addr, count) => {
            writeln_serial!(serial, "read({}, #{})", addr, count);
            let count = count as usize;
            if count > buf.len() {
                error!("read buffer too small");
                return Err(twim::Error::Overrun);
            }
            let buf = &mut buf[0..count];
            let r = twim.read(addr, buf).await;
            match &r {
                Ok((_, n)) => writeln_serial!(serial, "read {} bytes: {:?}", n, buf),
                Err(e) => writeln_serial!(serial, "read failed: {:?}", e),
                _ => {}
            }
            r.map(|_| ())
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
                return Err(twim::Error::Overrun);
            }
            let buf = &mut buf[0..count];
            let r = twim
                .write_read(cmd.addr, &cmd.data.buf[0..cmd.data.len as usize], buf)
                .await;
            match &r {
                Ok((w, r)) => {
                    writeln_serial!(serial, "{} bytes written, {} bytes read: {:?}", w, r, buf)
                }
                Err(e) => writeln_serial!(serial, "write/read failed: {:?}", e),
                _ => {}
            }
            r.map(|_| ())
        }
    }
}

#[embassy::task]
async fn twi_task(
    mut twim: twim::Twim,
    cmd_sig: &'static Signal<TwiCmd>,
    serial: &'static SerialChannel,
) {
    let serial = SerialSink::new(serial);
    loop {
        let cmd = cmd_sig.wait().await;
        let mut buf = [0u8; 16];

        match twi_transfer(&mut twim, cmd, &mut buf, &serial).await {
            Ok(()) => {
                info!("transfer finished");
            }
            Err(e) => {
                info!("transfer failed: {}", e);
            }
        };
    }
}

type Uarte = BufferedUarte<'static, peripherals::UARTE0, peripherals::TIMER0>;
impl<'a> AsyncReadWrite for Uarte {}

static UARTE: Forever<Uarte> = Forever::new();
static TWICMD_SIGNAL: Forever<Signal<TwiCmd>> = Forever::new();
static SERIAL_CH: Forever<SerialChannel> = Forever::new();

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;
    static mut RX_BUF: [u8; 32] = [0u8; 32];
    static mut TX_BUF: [u8; 32] = [0u8; 32];

    let uart = UARTE.put(unsafe {
        BufferedUarte::new(
            p.UARTE0,
            p.TIMER0,
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

    let twim = twim::Twim::new(
        p.TWISPI0,
        interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0),
        p.P0_26,
        p.P0_25,
        twim::Frequency::K100,
    );

    let serial_ch = SERIAL_CH.put(LocalChannel::new());
    let cmd_sig = TWICMD_SIGNAL.put(Signal::new());
    unwrap!(spawner.spawn(serial_task(uart, serial_ch, cmd_sig)));
    unwrap!(spawner.spawn(twi_task(twim, cmd_sig, serial_ch)));
}
