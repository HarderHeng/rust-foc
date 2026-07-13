//! Heartbeat task — proves the system is alive by writing a defmt log
//! every 500 ms, and pets the IWDG (C2).
//!
//! The debug UART is now owned by `shell_task`, so this task uses
//! defmt-RTT only.

use defmt::info;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn heartbeat() {
    let mut tick: u32 = 0;
    loop {
        tick = tick.wrapping_add(1);
        info!("heartbeat tick={}", tick);
        // C2: refresh the IWDG. If the heartbeat task itself ever
        // wedges (e.g. an async lock held forever), the IWDG fires
        // after 5 s and the device resets — visible as a reboot
        // rather than a silent hang.
        crate::wdog::pet();
        Timer::after_millis(500).await;
    }
}
