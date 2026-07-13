//! C2: Independent Watchdog (IWDG) driver.
//!
//! The previous implementation called a single raw `feed_watchdog()` at
//! boot that started the IWDG with a ~32 s timeout, then refreshed it
//! exactly once. Nothing else ever touched the KR register, so the
//! IWDG would fire after the timeout and reset the device — the
//! firmware would appear to "reboot every 30 s" with no diagnostic.
//!
//! Fix: use `embassy_stm32::wdg::IndependentWatchdog`, store the
//! handle in a `critical_section::Mutex<RefCell<...>>`, and have the
//! heartbeat task pet it every cycle. The 5 s timeout gives the
//! system 10× the heartbeat period to recover from a stuck task
//! before resetting.

use core::cell::RefCell;
use critical_section::Mutex;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Peri;

/// 5 s timeout — 10× the heartbeat period. If the heartbeat task
/// stops ticking for any reason, the IWDG fires and the device
/// resets. The CRC32 OTA readback, the FOC control loop, and the
/// shell parser all complete in well under 500 ms, so a 5 s budget
/// is comfortable headroom.
const IWDG_TIMEOUT_US: u32 = 5_000_000;

/// Shared handle. `None` until `init()` is called from
/// `bsp::board_init`. After init, the heartbeat task locks this,
/// takes the handle, and calls `pet()` every 500 ms.
pub static IWDG: Mutex<RefCell<Option<IndependentWatchdog<'static, IWDG>>>> =
    Mutex::new(RefCell::new(None));

/// Start the IWDG. Idempotent: calling twice is a no-op.
pub fn init(iwdg: Peri<'static, IWDG>) {
    critical_section::with(|cs| {
        let mut slot = IWDG.borrow_ref_mut(cs);
        if slot.is_none() {
            let mut wdog = IndependentWatchdog::new(iwdg, IWDG_TIMEOUT_US);
            wdog.unleash();
            *slot = Some(wdog);
            defmt::info!(
                "IWDG: started with {} ms timeout (refreshed by heartbeat task)",
                IWDG_TIMEOUT_US / 1000
            );
        }
    });
}

/// Pet (refresh) the IWDG. Called by the heartbeat task. No-op if
/// the watchdog hasn't been started yet (e.g. during early boot
/// before `bsp::board_init`).
pub fn pet() {
    critical_section::with(|cs| {
        if let Some(wdog) = IWDG.borrow_ref_mut(cs).as_mut() {
            wdog.pet();
        }
    });
}
