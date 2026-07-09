#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m_rt::entry;
use embassy_boot_stm32::*;
use embassy_stm32::flash::Flash;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop { cortex_m::asm::wfi(); } }

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());
    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(layout.bank1_region));
    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let start = 0x0800_0000 + config.active.offset();
    let bl = BootLoader::prepare::<_, _, _, 2048>(config);
    unsafe { bl.load(start) }
}
