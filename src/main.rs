#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use panic_probe as _;

#[embassy::task]
async fn periodic_task() {
    loop {
        info!("update");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy::main]
async fn main(spawner: Spawner) {
    unwrap!(spawner.spawn(periodic_task()));

}
