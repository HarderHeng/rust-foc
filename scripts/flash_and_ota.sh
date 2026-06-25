#!/usr/bin/env bash
# End-to-end OTA flash + test.
# Usage: ./scripts/flash_and_ota.sh
set -euo pipefail

APP_BIN="target/thumbv7em-none-eabihf/release/foc-rust"
BOOT_BIN="target/thumbv7em-none-eabihf/release/bootloader"

echo "=== Build app + bootloader ==="
cargo build --release

echo ""
echo "=== Flash bootloader @ 0x08000000 ==="
probe-rs download --chip STM32G431CBUx --base-address 0x08000000 "$BOOT_BIN"

echo ""
echo "=== Flash app @ 0x08004000 ==="
probe-rs download --chip STM32G431CBUx --base-address 0x08004000 "$APP_BIN"

echo ""
echo "=== Run app (RTT + shell on USART2 @ 921600 baud) ==="
echo "In another terminal: screen /dev/ttyUSB0 921600"
probe-rs run --chip STM32G431CBUx "$APP_BIN"
