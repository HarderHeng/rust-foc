#![no_std]
#![no_main]

use embassy_stm32::pac as _;
use panic_probe as _;
use defmt_rtt as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
