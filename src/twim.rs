use core::future::Future;
use core::marker::PhantomData;
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
const SRAM_LOWER: usize = 0x2000_0000;
const SRAM_UPPER: usize = 0x3000_0000;

#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    TxBufferZeroLength,
    RxBufferZeroLength,
    // Transmit,
    // Receive,
    DMABufferNotInDataMemory,
    AddressNack,
    DataNack,
    Overrun,
    Busy,
}

pub struct Twim {
    regs: &'static pac::twim0::RegisterBlock,
    state: &'static sealed::State,
}

pub struct StopSwitch<'a> {
    regs: &'static pac::twim0::RegisterBlock,
    _pd: PhantomData<&'a ()>,
}

impl Twim {
    pub fn new<T: Instance>(
        _twim: impl Unborrow<Target = T>,
        irq: impl Unborrow<Target = T::Interrupt>,
        scl: impl Unborrow<Target = impl gpio::Pin>,
        sda: impl Unborrow<Target = impl gpio::Pin>,
        freq: Frequency,
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

            if r.events_error.read().bits() != 0 {
                // Clearing interrupt can take up to four clock cycles.
                r.intenclr.write(|w| w.error().clear());
                // stop TWIM on error
                r.tasks_stop.write(|w| unsafe { w.bits(1) });
            }
            if r.events_stopped.read().bits() != 0 {
                // Clearing interrupt can take up to four clock cycles.
                r.intenclr.write(|w| w.stopped().clear());
                // wake up task
                s.end_waker.wake();
            }
        }

        irq.set_handler(on_interrupt::<T>);
        irq.unpend();
        irq.enable();

        Self {
            regs: T::regs(),
            state: T::state(),
        }
    }

    pub fn borrow_stoppable<'a>(&'a mut self) -> (&'a mut Self, StopSwitch<'a>) {
        let r = self.regs;
        (
            self,
            StopSwitch {
                regs: r,
                _pd: PhantomData,
            },
        )
    }

    pub fn read<'a>(&'a mut self, address: Address, buffer: &'a mut [u8]) -> Transfer<'a> {
        Transfer::new(self, address, TransferSetup::Read(buffer))
    }

    pub fn write<'a>(&'a mut self, address: Address, buffer: &'a [u8]) -> Transfer<'a> {
        Transfer::new(self, address, TransferSetup::Write(buffer))
    }

    pub fn write_read<'a>(
        &'a mut self,
        address: Address,
        send_buffer: &'a [u8],
        recv_buffer: &'a mut [u8],
    ) -> Transfer<'a> {
        Transfer::new(
            self,
            address,
            TransferSetup::WriteRead(send_buffer, recv_buffer),
        )
    }
}

impl<'a> StopSwitch<'a> {
    pub fn stop(&mut self) {
        self.regs.tasks_stop.write(|w| unsafe { w.bits(1) });
    }
}

#[derive(defmt::Format)]
enum TransferSetup<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
    WriteRead(&'a [u8], &'a mut [u8]),
}

enum TransferState<'a> {
    Empty,
    Setup(Address, TransferSetup<'a>),
    Running(DropBomb),
}

pub struct Transfer<'a> {
    regs: &'static pac::twim0::RegisterBlock,
    state: &'static sealed::State,
    run_state: TransferState<'a>,
    _pd: PhantomData<&'a mut ()>,
}

impl<'a> Transfer<'a> {
    fn new(twim: &'a mut Twim, addr: Address, setup: TransferSetup<'a>) -> Self {
        trace!("new transfer to {:x}", addr);
        Self {
            regs: twim.regs,
            state: twim.state,
            run_state: TransferState::Setup(addr, setup),
            _pd: PhantomData,
        }
    }

    fn start(&mut self, addr: Address, setup: TransferSetup<'a>) -> Result<(), Error> {
        match setup {
            TransferSetup::Read(recv_buffer) => {
                check_rx_buffer(recv_buffer)?;
                self.init_transfer(addr)?;
                unsafe { self.setup_rx(recv_buffer) };

                let r = self.regs;
                // Enable shortcut between event LASTRX and task STOP.
                r.shorts.write(|w| w.lastrx_stop().enabled());
                // Start receive.
                r.tasks_startrx.write(|w| unsafe { w.bits(1) });
            }
            TransferSetup::Write(send_buffer) => {
                check_tx_buffer(send_buffer)?;
                self.init_transfer(addr)?;
                unsafe { self.setup_tx(send_buffer) };

                let r = self.regs;
                // Enable shortcut between event LASTTX and task STOP.
                r.shorts.write(|w| w.lasttx_stop().enabled());
                // Start transmission.
                r.tasks_starttx.write(|w| unsafe { w.bits(1) });
            }
            TransferSetup::WriteRead(send_buffer, recv_buffer) => {
                check_tx_buffer(send_buffer)?;
                check_rx_buffer(recv_buffer)?;
                self.init_transfer(addr)?;
                unsafe { self.setup_tx(send_buffer) };
                unsafe { self.setup_rx(recv_buffer) };

                let r = self.regs;
                // Enable shortcuts between event LASTTX and STARTRX,
                // and event LASTRX and task STOP.
                r.shorts
                    .write(|w| w.lasttx_startrx().enabled().lastrx_stop().enabled());
                // Start transmission.
                r.tasks_starttx.write(|w| unsafe { w.bits(1) });
            }
        }
        Ok(())
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

    /// Safety: This is only safe to be called when TWIM is stopped.
    /// Safety: We must quarantee that buffer is not used before TWIM is stopped.
    unsafe fn setup_rx(&mut self, buffer: &'a mut [u8]) {
        let r = self.regs;
        r.events_lastrx.reset();
        r.rxd
            .ptr
            .write(|w| unsafe { w.ptr().bits(buffer.as_mut_ptr() as _) });
        r.rxd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(buffer.len() as _) });
    }

    /// Safety: This is only safe to be called when TWIM is stopped.
    /// Safety: We must quarantee that buffer is not used before TWIM is stopped.
    unsafe fn setup_tx(&mut self, buffer: &'a [u8]) {
        let r = self.regs;
        r.events_lasttx.reset();
        r.txd
            .ptr
            .write(|w| unsafe { w.ptr().bits(buffer.as_ptr() as _) });
        r.txd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(buffer.len() as _) });
    }

    fn poll_end(&self, cx: &mut Context<'_>) -> Poll<Result<(), Error>> {
        let r = self.regs;
        let s = self.state;

        if r.events_stopped.read().bits() == 0 {
            s.end_waker.register(cx.waker());
            return Poll::Pending;
        }

        Poll::Ready(if r.events_error.read().bits() == 0 {
            Ok(())
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

impl<'a> Future for Transfer<'a> {
    type Output = Result<(), Error>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<<Self as futures::Future>::Output> {
        let this = self.get_mut();

        this.run_state = match core::mem::replace(&mut this.run_state, TransferState::Empty) {
            TransferState::Setup(addr, setup) => {
                trace!("transfer setup {}", setup);
                this.start(addr, setup)?;
                TransferState::Running(DropBomb::new())
            }
            TransferState::Empty => panic!("Polling empty transfer"),
            state => {
                trace!("transfer poll");
                state
            }
        };

        let pr = this.poll_end(cx);
        if let Poll::Ready(_) = pr {
            match core::mem::replace(&mut this.run_state, TransferState::Empty) {
                TransferState::Running(bomb) => bomb.defuse(),
                _ => unreachable!(),
            }
        }
        pr
    }
}

fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
}

fn check_rx_buffer(buffer: &[u8]) -> Result<(), Error> {
    // check if buffer is in RAM
    if !slice_in_ram(buffer) {
        return Err(Error::DMABufferNotInDataMemory);
    }

    // check buffer size
    if buffer.len() == 0 {
        Err(Error::RxBufferZeroLength)
    } else if buffer.len() > EASY_DMA_SIZE {
        Err(Error::RxBufferTooLong)
    } else {
        Ok(())
    }
}

fn check_tx_buffer(buffer: &[u8]) -> Result<(), Error> {
    // check if buffer is in RAM
    if !slice_in_ram(buffer) {
        return Err(Error::DMABufferNotInDataMemory);
    }

    // check buffer size
    if buffer.len() == 0 {
        Err(Error::TxBufferZeroLength)
    } else if buffer.len() > EASY_DMA_SIZE {
        Err(Error::TxBufferTooLong)
    } else {
        Ok(())
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
