//! Heartbeat task — proves system is alive by writing a tick line
//! over the debug UART every 500ms. Also writes a defmt log for
//! RTT-side observability.

use defmt::info;
use embassy_time::Timer;

use crate::bsp::DebugUartSink;
use crate::drivers::debug_uart::DebugShellSink;

#[embassy_executor::task]
pub async fn heartbeat(mut sink: DebugUartSink) {
    let mut tick: u32 = 0;
    loop {
        tick = tick.wrapping_add(1);
        // Note: write_str is sync (synchronous into ringbuffer).
        // For 500ms cadence and small strings, this is fine.
        let _ = sink.write_str("[hb] tick\n");
        info!("heartbeat tick={}", tick);
        Timer::after_millis(500).await;
    }
}
