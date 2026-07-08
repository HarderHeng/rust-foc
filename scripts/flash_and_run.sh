#!/usr/bin/env bash
# Build, flash, and run the foc-rust app on the B-G431B-ESC1.
# Usage: ./scripts/flash_and_run.sh
set -euo pipefail

APP_BIN="target/thumbv7em-none-eabihf/release/foc-rust"

echo "=== Build app (release) ==="
cargo build --release

echo ""
echo "=== Flash app @ 0x08000000 ==="
probe-rs download --chip STM32G431CBUx --base-address 0x08000000 "$APP_BIN"

echo ""
echo "=== Run app (RTT + shell on USART2 @ 921600 baud) ==="
echo "In another terminal: screen /dev/ttyUSB0 921600"
probe-rs run --chip STM32G431CBUx "$APP_BIN"