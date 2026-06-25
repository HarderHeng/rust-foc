//! Heartbeat task — proves the system is alive by writing a defmt log
//! every 500 ms.  The debug UART is now owned by `shell_task`, so this
//! task uses defmt-RTT only.

use defmt::info;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn heartbeat() {
    let mut tick: u32 = 0;
    loop {
        tick = tick.wrapping_add(1);
        info!("heartbeat tick={}", tick);
        Timer::after_millis(500).await;
    }
}
