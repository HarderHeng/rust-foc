#![no_std]
#![no_main]

use core::ptr::read_volatile;
use cortex_m_rt::entry;
use foc_shared::{SlotConfig, SLOT_CONFIG_ADDR, SLOT_CONFIG_MAGIC, SLOT_A_BASE, SLOT_B_BASE, MAX_BOOT_ATTEMPTS};

const RCC_CSR: u32 = 0x4000_3800;
const PORRSTF: u32 = 1 << 26;
const IWDGRSTF: u32 = 1 << 29;
const RMVF: u32 = 1 << 24;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop { cortex_m::asm::wfi(); } }

#[entry]
fn main() -> ! {
    let raw: u64 = unsafe { read_volatile(SLOT_CONFIG_ADDR as *const u64) };
    let mut cfg = SlotConfig::from_u64(raw);

    let csr = unsafe { read_volatile(RCC_CSR as *const u32) };
    let is_por = csr & PORRSTF != 0;
    let is_wdg = csr & IWDGRSTF != 0;
    // Clear all reset flags
    unsafe { core::ptr::write_volatile(RCC_CSR as *mut u32, csr | RMVF); }

    if cfg.magic != SLOT_CONFIG_MAGIC {
        cfg = SlotConfig::default_config();
    }

    if is_por {
        cfg.boot_attempts = 0;
        cfg.flags = 0;
    }

    if is_wdg {
        cfg.boot_attempts = cfg.boot_attempts.saturating_add(1);
        cfg.flags |= SlotConfig::IWDG_RESET;
        if cfg.boot_attempts >= MAX_BOOT_ATTEMPTS {
            cfg.active_slot ^= 1;
            cfg.boot_attempts = 0;
        }
    }

    let slot_base = if cfg.active_slot == 0 { SLOT_A_BASE } else { SLOT_B_BASE };

    unsafe {
        let vt = slot_base as *const u32;
        let sp = read_volatile(vt);
        let rv = read_volatile(vt.offset(1));
        core::ptr::write_volatile(0xE000_ED08 as *mut u32, slot_base);
        core::arch::asm!(
            "msr MSP, {sp}",
            "bx {rv}",
            sp = in(reg) sp,
            rv = in(reg) rv,
            options(noreturn),
        );
    }
}
