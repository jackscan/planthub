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
use embassy::time::{Duration, Timer};
use embassy::util::{Forever, Signal};
use futures::{pin_mut, Stream};
use futures_intrusive::channel::LocalChannel;
use futures_util::future::{poll_fn, Either};
use panic_probe as _;

use embassy_nrf::buffered_uarte::BufferedUarte;
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, peripherals, uarte, Peripherals};
use hal::wdt::{self, Watchdog, WatchdogHandle};
use nrf52832_hal as hal;

#[macro_use]
mod serial;
use serial::{SerialChannel, SerialSink};

mod twim;
mod weight_scale_drv;

const WD_TIMEOUT: Duration = Duration::from_millis(500);

defmt::timestamp!("{=u64}", { embassy::time::Instant::now().as_millis() });

trait AsyncReadWrite: AsyncBufRead + AsyncWrite {}

async fn writeln_uart_fmt(mut uart: Pin<&mut dyn AsyncReadWrite>, args: core::fmt::Arguments<'_>) {
    if let Err(_) = async {
        let mut buf = arrayvec::ArrayString::<64>::new();
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

#[embassy::task]
async fn worker_task(
    mut twim: twim::Twim,
    sercmd_sig: &'static Signal<serial::TwiCmd>,
    serial: &'static SerialChannel,
    mut wd_hndl: WatchdogHandle<wdt::handles::HdlN>,
) {
    let serial = SerialSink::new(serial);
    loop {
        wd_hndl.pet();

        // wait for cmd or time to pet watchdog
        let cmd = {
            let cmd_fut = sercmd_sig.wait();
            let wd_timer = Timer::after(WD_TIMEOUT / 2);
            match futures::future::select(cmd_fut, wd_timer).await {
                Either::Left((cmd, _)) => cmd,
                Either::Right((_, _)) => continue,
            }
        };

        wd_hndl.pet();

        run_serial_cmd(&mut twim, &serial, cmd, &mut wd_hndl).await;
    }
}

type Uarte = BufferedUarte<'static, peripherals::UARTE0, peripherals::TIMER0>;
impl<'a> AsyncReadWrite for Uarte {}

static UARTE: Forever<Uarte> = Forever::new();
static SERCMD_SIGNAL: Forever<Signal<serial::TwiCmd>> = Forever::new();
static SERIAL_CH: Forever<SerialChannel> = Forever::new();

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
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

    // let (wdh0, )

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
    unwrap!(spawner.spawn(serial_task(uart, serial_ch, sercmd_sig)));
    unwrap!(spawner.spawn(worker_task(twim, sercmd_sig, serial_ch, wrk_wd_hndl.degrade())));
}
