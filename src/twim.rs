use core::future::Future;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Context, Poll};

use defmt::trace;
use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::util::{AtomicWaker, DropBomb, Unborrow};
use embassy_extras::unborrow;
pub use embassy_traits::i2c::SevenBitAddress as Address;

use embassy_nrf::{gpio, interrupt, peripherals};
use hal::pac;
pub use hal::twim::Frequency;
use nrf52832_hal as hal;

const EASY_DMA_SIZE: usize = (1 << 8) - 1;

#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    TxBufferZeroLength,
    RxBufferZeroLength,
    Transmit,
    Receive,
    AddressNack,
    DataNack,
    Overrun,
    Busy,
}

pub struct Twim {
    regs: &'static pac::twim0::RegisterBlock,
    state: &'static sealed::State,
    buf: &'static mut [u8],
    txlen: usize,
    rxlen: usize,
}

pub struct StopSwitch<'a> {
    regs: &'a pac::twim0::RegisterBlock,
}

impl Twim {
    pub fn new<T: Instance>(
        _twim: impl Unborrow<Target = T>,
        irq: impl Unborrow<Target = T::Interrupt>,
        scl: impl Unborrow<Target = impl gpio::Pin>,
        sda: impl Unborrow<Target = impl gpio::Pin>,
        freq: Frequency,
        buf: &'static mut [u8],
    ) -> Self {
        unborrow!(irq, scl, sda);

        irq.disable();

        // configure pins before enabling TWIM according to product specification chapter 33.7
        scl.conf().write(|w| {
            w.dir()
                .input()
                .input()
                .connect()
                .pull()
                .pullup()
                .drive()
                .s0d1()
                .sense()
                .disabled()
        });
        sda.conf().write(|w| {
            w.dir()
                .input()
                .input()
                .connect()
                .pull()
                .pullup()
                .drive()
                .s0d1()
                .sense()
                .disabled()
        });

        let r = T::regs();

        // select pins
        r.psel.scl.write(|w| {
            let w = unsafe { w.pin().bits(scl.psel_bits() as u8) };
            w.connect().connected()
        });
        r.psel.sda.write(|w| {
            let w = unsafe { w.pin().bits(sda.psel_bits() as u8) };
            w.connect().connected()
        });

        // enable
        r.enable.write(|w| w.enable().enabled());
        // frequency
        r.frequency.write(|w| w.frequency().variant(freq));

        // interrupt handler
        fn on_interrupt<T: Instance>(_: *mut ()) {
            let r = T::regs();
            let s = T::state();

            if r.events_stopped.read().bits() != 0 {
                // Clearing interrupt can take up to four clock cycles.
                r.intenclr.write(|w| w.stopped().clear().error().clear());
                // wake up task
                s.end_waker.wake();
            } else if r.events_error.read().bits() != 0 {
                // Clearing interrupt can take up to four clock cycles.
                r.intenclr.write(|w| w.error().clear());
                // stop TWIM on error
                r.tasks_stop.write(|w| unsafe { w.bits(1) });
            }
        }

        irq.set_handler(on_interrupt::<T>);
        irq.unpend();
        irq.enable();

        Self {
            regs: T::regs(),
            state: T::state(),
            buf,
            txlen: 0,
            rxlen: 0,
        }
    }

    pub fn borrow_stoppable<'a>(&'a mut self) -> (&'a mut Self, StopSwitch<'a>) {
        let regs = self.regs;
        (self, StopSwitch { regs })
    }

    pub fn transfer<'a>(&'a mut self) -> Result<TransferSetup<'a>, Error> {
        // While interrupt for event STOPPED is enabled another transaction is in progress.
        if self.regs.inten.read().stopped().is_enabled() {
            Err(Error::Busy)
        } else {
            self.txlen = 0;
            self.rxlen = 0;
            Ok(TransferSetup { twim: self })
        }
    }

    fn init_transfer(&mut self, addr: Address) -> Result<(), Error> {
        let r = self.regs;
        // While interrupt for event STOPPED is enabled another transaction is in progress.
        if r.inten.read().stopped().is_enabled() {
            // TODO: Shall we await stop() instead of returning an error?
            // XXX: This should not happen since future can not be dropped without awaiting stopped.
            return Err(Error::Busy);
        }
        // set target address
        r.address.write(|w| unsafe { w.address().bits(addr) });
        // reset events
        r.events_stopped.reset();
        r.events_error.reset();
        // Write 1's to clear errorsrc bits.
        r.errorsrc
            .write(|w| w.overrun().set_bit().anack().set_bit().dnack().set_bit());
        // Enable interrupt on events STOPPED and ERROR.
        r.intenset.write(|w| w.stopped().set().error().set());
        Ok(())
    }

    /// Safety: We must quarantee that buffer is not read before TWIM is stopped.
    fn setup_rx(&mut self) {
        trace!("setup rx: {}", self.rxlen);
        let r = self.regs;
        r.events_lastrx.reset();
        r.rxd
            .ptr
            .write(|w| unsafe { w.ptr().bits(self.buf.as_mut_ptr() as _) });
        r.rxd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(self.rxlen as _) });
    }

    /// Safety: We must quarantee that buffer is not written before TWIM is stopped.
    fn setup_tx(&mut self) {
        trace!("setup tx: {}", self.txlen);
        let r = self.regs;
        r.events_lasttx.reset();
        r.txd
            .ptr
            .write(|w| unsafe { w.ptr().bits(self.buf.as_ptr() as _) });
        r.txd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(self.txlen as _) });
    }

    fn start(&mut self, addr: Address) -> Result<(), Error> {
        if self.txlen == 0 && self.rxlen == 0 {
            return Err(Error::RxBufferZeroLength);
        }

        let r = self.regs;
        self.init_transfer(addr)?;

        if self.txlen > 0 {
            self.setup_tx();
        }

        if self.rxlen > 0 {
            self.setup_rx();
        }

        r.shorts.write(|w| match (self.txlen, self.rxlen) {
            (0, _) => {
                // Enable shortcut between event LASTRX and task STOP.
                w.lastrx_stop().enabled()
            }
            (_, 0) => {
                // Enable shortcut between event LASTTX and task STOP.
                w.lasttx_stop().enabled()
            }
            (_, _) => {
                // Enable shortcuts between event LASTTX and STARTRX,
                // and event LASTRX and task STOP.
                w.lasttx_startrx().enabled().lastrx_stop().enabled()
            }
        });

        // Fence for dma transfer.
        compiler_fence(Ordering::Release);

        if self.txlen > 0 {
            // Start transmission.
            r.tasks_starttx.write(|w| unsafe { w.bits(1) });
        } else {
            // Start receive.
            r.tasks_startrx.write(|w| unsafe { w.bits(1) });
        }

        Ok(())
    }

    fn poll_end(&self, cx: &mut Context<'_>) -> Poll<Result<(usize, usize), Error>> {
        let r = self.regs;
        let s = self.state;

        s.end_waker.register(cx.waker());
        if r.events_stopped.read().bits() == 0 {
            trace!("transfer pending");
            return Poll::Pending;
        }

        // Fence for end of dma transfer.
        compiler_fence(Ordering::Acquire);

        Poll::Ready(if r.events_error.read().bits() == 0 {
            let txn = r.txd.amount.read().bits() as usize;
            let rxn = r.rxd.amount.read().bits() as usize;
            trace!("transfer finished");
            Ok((txn, rxn))
        } else {
            trace!("transfer error {:x}", r.errorsrc.read().bits());
            Err(match r.errorsrc.read() {
                e if e.anack().is_received() => Error::AddressNack,
                e if e.dnack().is_received() => Error::DataNack,
                e if e.overrun().is_received() => Error::Overrun,
                // According to product specification chapter 33.8.5 ERRORSRC this can never happen.
                _ => unreachable!(),
            })
        })
    }
}

impl<'a> StopSwitch<'a> {
    pub fn stop(&self) {
        self.regs.tasks_stop.write(|w| unsafe { w.bits(1) });
    }
}

pub struct TransferSetup<'a> {
    twim: &'a mut Twim,
}

impl<'a> TransferSetup<'a> {
    #[must_use]
    pub fn write_buf(&mut self, n: usize) -> Result<&mut [u8], Error> {
        let buf = &mut self.twim.buf;
        if n > buf.len() {
            Err(Error::Overrun)
        } else if n == 0 {
            Err(Error::TxBufferZeroLength)
        } else if n > EASY_DMA_SIZE {
            Err(Error::TxBufferTooLong)
        } else {
            self.twim.txlen = n;
            Ok(&mut buf[..n])
        }
    }

    pub fn read_len(&mut self, n: usize) -> Result<(), Error> {
        let buf = &mut self.twim.buf;
        if n > buf.len() {
            Err(Error::Overrun)
        } else if n == 0 {
            Err(Error::TxBufferZeroLength)
        } else if n > EASY_DMA_SIZE {
            Err(Error::RxBufferTooLong)
        } else {
            self.twim.rxlen = n;
            Ok(())
        }
    }

    #[must_use]
    pub fn start(self, addr: Address) -> Transfer<'a> {
        Transfer::new(self.twim, addr)
    }
}

enum TransferState<'a> {
    Empty,
    Setup(Address, &'a mut Twim),
    Running(DropBomb, &'a mut Twim),
}

pub struct Transfer<'a> {
    run_state: TransferState<'a>,
}

impl<'a> Transfer<'a> {
    fn new(twim: &'a mut Twim, addr: Address) -> Self {
        trace!("new transfer to {:x}", addr);
        Self {
            run_state: TransferState::Setup(addr, twim),
        }
    }
}

impl<'a> Future for Transfer<'a> {
    type Output = Result<&'a [u8], Error>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<<Self as futures::Future>::Output> {
        loop {
            match core::mem::replace(&mut self.as_mut().run_state, TransferState::Empty) {
                TransferState::Setup(addr, twim) => {
                    twim.start(addr)?;
                    self.as_mut().run_state = TransferState::Running(DropBomb::new(), twim)
                }
                TransferState::Running(bomb, twim) => {
                    break if let Poll::Ready(res) = twim.poll_end(cx) {
                        bomb.defuse();
                        let (txb, rxb) = (twim.txlen, twim.rxlen);
                        match res {
                            Ok((txn, _)) if txb != 0 && txn != txb => {
                                trace!("tx len:{} send:{}", txb, txn);
                                Poll::Ready(Err(Error::Transmit))
                            }
                            Ok((_, rxn)) if rxb != 0 && rxn != rxb => {
                                trace!("rx len:{} recv:{}", rxb, rxn);
                                Poll::Ready(Err(Error::Receive))
                            }
                            Ok((_, rxn)) => {
                                // let r = self.as_ref();
                                let r = &twim.buf[..rxn];
                                Poll::Ready(Ok(r))
                            }
                            Err(e) => Poll::Ready(Err(e)),
                        }
                    } else {
                        self.as_mut().run_state = TransferState::Running(bomb, twim);
                        Poll::Pending
                    };
                }
                TransferState::Empty => panic!("Polling empty transfer"),
            }
        }
    }
}

mod sealed {
    use super::*;

    pub struct State {
        pub end_waker: AtomicWaker,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                end_waker: AtomicWaker::new(),
            }
        }
    }

    pub trait Instance {
        fn regs() -> &'static pac::twim0::RegisterBlock;
        fn state() -> &'static State;
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: Interrupt;
}

macro_rules! impl_instance {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl sealed::Instance for peripherals::$type {
            fn regs() -> &'static pac::twim0::RegisterBlock {
                unsafe { &*pac::$pac_type::ptr() }
            }
            fn state() -> &'static sealed::State {
                static STATE: sealed::State = sealed::State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$type {
            type Interrupt = interrupt::$irq;
        }
    };
}

impl_instance!(TWISPI0, TWIM0, SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
impl_instance!(TWISPI1, TWIM1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
