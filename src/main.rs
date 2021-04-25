#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(pin_static_ref)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

use core::fmt::Write;
use core::pin::Pin;
use core::task::{Context, Poll};

use defmt::{error, info, unwrap};
use defmt_rtt as _;
use embassy::executor::Spawner;
use embassy::io::{AsyncBufRead, AsyncBufReadExt, AsyncWrite, AsyncWriteExt, Result};
use embassy::util::{Forever, Signal};
use panic_probe as _;

use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, peripherals, uarte, Peripherals};

mod serial_cmds;
use serial_cmds::TwiCmd;

trait AsyncReadWrite: AsyncBufRead + AsyncWrite {}

struct Serial<'a> {
    uart: Pin<&'a mut dyn AsyncReadWrite>,
}

impl<'a> AsyncBufRead for Serial<'a> {
    fn poll_fill_buf(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<&[u8]>> {
        self.get_mut().uart.as_mut().poll_fill_buf(cx)
    }
    fn consume(self: Pin<&mut Self>, amt: usize) {
        self.get_mut().uart.as_mut().consume(amt)
    }
}

impl<'a> AsyncWrite for Serial<'a> {
    fn poll_write(self: Pin<&mut Self>, cx: &mut Context<'_>, buf: &[u8]) -> Poll<Result<usize>> {
        let s = self.get_mut();
        let u = s.uart.as_mut();
        u.poll_write(cx, buf)
    }
}

#[embassy::task]
async fn serial_task(mut serial: Serial<'static>, cmd_sig: &'static Signal<TwiCmd>) {
    info!("serial initialized");

    unwrap!(serial.write_all(b"Hello Serial!\r\n").await);

    loop {
        fn is_line_break(b: u8) -> bool {
            b == b'\r' || b == b'\n'
        }
        unwrap!(serial.skip_while(is_line_break).await);
        let mut buf = [0u8; 32];
        let n = unwrap!(serial.read_while(&mut buf, |b| !is_line_break(b)).await);
        let buf = &buf[..n];

        unwrap!(serial.write_all(&buf).await);
        unwrap!(serial.write_all(b"\r\n").await);

        match serial_cmds::parse_cmd(&buf) {
            Ok(Some(cmd)) => {
                let mut resp_buf = arrayvec::ArrayString::<128>::new();
                if let Err(_) = write!(&mut resp_buf, "cmd: {:?}\n\r", cmd) {
                    error!("error formatting cmd: {}", (&resp_buf as &str));
                };
                unwrap!(serial.write_all(resp_buf.as_bytes()).await);
                cmd_sig.signal(cmd);
            }
            Ok(None) => info!("no cmd"),
            Err(e) => error!("err: {} at {}", e.msg, e.pos),
        }
    }
}

#[embassy::task]
async fn twi_task(cmd_sig: &'static Signal<TwiCmd>) {
    loop {
        let cmd = cmd_sig.wait().await;
        match cmd {
            TwiCmd::Write(cmd) => {
                info!(
                    "write({}, {})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize]
                );
            }
            TwiCmd::Read(addr, count) => {
                info!("read({}, #{})", addr, count)
            }
            TwiCmd::WriteRead(cmd, count) => {
                info!(
                    "write({}, {}), read(#{})",
                    cmd.addr,
                    &cmd.data.buf[0..cmd.data.len as usize],
                    count
                )
            }
        }
    }
}

type Uarte = BufferedUarte<'static, peripherals::UARTE0, peripherals::TIMER0>;
impl<'a> AsyncReadWrite for Uarte {}

static UARTE: Forever<Uarte> = Forever::new();
static TWICMD_SIGNAL: Forever<Signal<TwiCmd>> = Forever::new();

#[embassy::main]
async fn main(spawner: Spawner) {
    let p = Peripherals::take().unwrap();

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

    let serial = Serial {
        uart: Pin::static_mut(uart),
    };

    let cmd_sig = TWICMD_SIGNAL.put(Signal::new());
    unwrap!(spawner.spawn(serial_task(serial, cmd_sig)));
    unwrap!(spawner.spawn(twi_task(cmd_sig)));
}
