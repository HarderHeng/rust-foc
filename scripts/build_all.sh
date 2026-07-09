#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."

BIN_DIR="target/thumbv7em-none-eabihf/release"

echo "=== Building slot A ==="
SLOT=a cargo build -p app --release

echo "=== Building slot B ==="
SLOT=b cargo build -p app --release

cp "$BIN_DIR/app" "$BIN_DIR/app-a"
cp "$BIN_DIR/app" "$BIN_DIR/app-b"

echo ""
echo "=== bootloader ==="
arm-none-eabi-size "$BIN_DIR/bootloader" | tail -1
echo "=== app-a (0x08002000) ==="
arm-none-eabi-size "$BIN_DIR/app-a" | tail -1
echo "=== app-b (0x08010800) ==="
arm-none-eabi-size "$BIN_DIR/app-b" | tail -1

echo ""
echo "Flash layout:"
echo "  bootloader  : 0x08000000-0x08001FFF (8 KB)"
echo "  app-a       : 0x08002000-0x080107FF (58 KB)"
echo "  app-b       : 0x08010800-0x0801EFFF (58 KB)"
echo "  slot config : 0x0801FFF8-0x0801FFFF (8 B)"
echo ""
echo "Flash commands:"
echo "  probe-rs download --chip STM32G431CBUx --base-address 0x08000000 $BIN_DIR/bootloader"
echo "  probe-rs download --chip STM32G431CBUx --base-address 0x08002000 $BIN_DIR/app-a"
echo "  probe-rs download --chip STM32G431CBUx --base-address 0x08010800 $BIN_DIR/app-b"
