#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{info};
use defmt_rtt as _;
use panic_probe as _;

#[entry]
fn main() -> ! {
    info!("Hello, world!");

    loop {
        cortex_m::asm::wfe();
    }
}
