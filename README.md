# foc-rust

Field Oriented Control firmware in Rust on the **ST B-G431B-ESC1** dev board
(STM32G431CBU6, Cortex-M4F @ 170 MHz, 128 KB flash, 32 KB SRAM).

## Status

| Component | Status |
|-----------|--------|
| Workspace (app + common + bootloader) | ✅ |
| Board support (HSE 170 MHz, USART2 PB3/PB4 @ 921600 baud) | ✅ |
| Shell (help / version / info / reboot / ota_update) | ✅ |
| OTA bootloader (16 KB, y-modem, CRC-32/ISO-HDLC) | ✅ |
| OTA flag (flash-backed `FlashOtaFlag`) | ✅ |
| FOC — TBD | ⏳ |

## Workspace

```
foc-rust/                 # app binary (110 KB slot @ 0x0800_4000)
├── src/
│   ├── main.rs           # composition root: embassy-executor async
│   ├── bsp.rs            # board support (peripherals, clocks)
│   ├── drivers/
│   │   ├── debug_uart.rs # Uart2Sink + DebugShellSink + embedded-io adapters
│   │   └── flash.rs      # Stm32g4Flash (PAC-based, for OTA flag)
│   ├── commands/
│   │   ├── shell.rs      # ShellCommand enum + dispatch (5 commands)
│   │   └── ota.rs        # OtaUpdateCommand → set flag + reset
│   └── tasks/
│       ├── heartbeat.rs  # defmt tick every 500 ms
│       └── shell.rs      # async shell task via embedded-cli 0.2.1
│
common/                    # foc-common lib (shared types + OtaFlag trait)
├── src/
│   ├── lib.rs            # flash layout constants + re-exports
│   └── flag.rs           # FlashOtaFlag<F: NorFlash + ReadNorFlash>
│
bootloader/                # bootloader binary (16 KB @ 0x0800_0000)
├── src/
│   ├── main.rs           # entry: flag check → jump / y-modem / timeout
│   ├── flash.rs          # Stm32g4Flash (PAC-based, WRITE_SIZE=8)
│   ├── ymodem.rs         # y-modem CRC mode receiver (1 KB packets)
│   ├── crc.rs            # hardware CRC-32/ISO-HDLC
│   └── uart.rs           # raw USART2 TX/RX (blocking, PAC registers)
│
scripts/
└── flash_and_ota.sh      # build + flash bootloader + app
```

## Flash Layout

| Region | Address | Size | Content |
|--------|---------|------|---------|
| Bootloader | `0x0800_0000` | 16 KB | Bootloader code (y-modem receiver) |
| OTA flag | `0x0800_3F00` | 1 B | 0xAA = Pending, 0x00 = None |
| App | `0x0800_4000` | 110 KB | Application firmware |
| Metadata | `0x0801_F800` | 2 KB | Reserved (image CRC, version) |

## Toolchain

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs --features=cli
cargo install flip-link          # optional: stack overflow protection
```

## Build

```bash
cargo build                      # debug profile
cargo build --release            # release, size-optimized
cargo build -p bootloader        # bootloader only
cargo build -p foc-common        # shared lib only
```

## Flash + Run

Both bootloader and app must be flashed at their respective addresses:

```bash
probe-rs download --chip STM32G431CBUx --base-address 0x08000000 target/thumbv7em-none-eabihf/release/bootloader
probe-rs download --chip STM32G431CBUx --base-address 0x08004000 target/thumbv7em-none-eabihf/release/foc-rust
```

Or use the bundled script:

```bash
./scripts/flash_and_ota.sh
```

After flashing, connect a USB-TTL to PB3 (TX) / PB4 (RX) / GND:

```bash
screen /dev/ttyUSB0 921600
```

## Shell Commands

| Command | Action |
|---------|--------|
| `help` | List available commands |
| `version` | Firmware version + Git SHA |
| `info` | Chip + flash info |
| `reboot` | Reset MCU |
| `ota_update` | Set OTA flag → reset into bootloader |

## Clock Configuration

- HSE 8 MHz crystal → PLLN=85 → PLLR=DIV4 → **170 MHz sysclk**
- APB1 = DIV4 → 42.5 MHz (USART2 runs at 170 MHz / 184 ≈ 921600 baud)
- Bootloader and app share the **exact same** clock config so that USART2 baud
  stays consistent across resets.

## Key Design Decisions

- **Single-slot OTA** — no A/B swapping; 128 KB flash cannot spare a second slot.
  Bootloader receives y-modem over USART2, writes directly to the app region.
- **Custom bootloader** (not embassy-boot) — same reason: A/B partitions would
  halve the available app space.
- **PAC-level flash drivers** — both bootloader and app have their own
  `Stm32g4Flash` that drives the FLASH peripheral directly via `embassy_stm32::pac`,
  so the bootloader doesn't need a full HAL.
- **FlashOtaFlag is generic** — parameterized over `F: NorFlash + ReadNorFlash`.
  The implementor must ensure `WRITE_SIZE`/`READ_SIZE` alignment; the default
  implementation uses 8-byte buffers for STM32G4 compatibility.
- **Layered architecture** — `src/tasks/` and `src/commands/` **must not** import
  `embassy_stm32` directly; all HAL configuration goes through `bsp.rs`.

## CANopen / UDS (FDCAN1, OTA path)

See [`docs/superpowers/specs/2026-07-03-uds-rewrite-design.md`](docs/superpowers/specs/2026-07-03-uds-rewrite-design.md)
for the full design. The legacy 2026-07-02 spec is
**deprecated** (kept for history).

### Adding a new DID (ReadDataByIdentifier 0x22)

The UDS dispatcher is **table-driven** — adding a new DID is a single
`DidReadEntry` in `src/can/uds_config.rs`, no Rust dispatcher code change:

```rust
fn read_my_did(out: &mut [u8; 7]) -> Result<usize, Nrc> {
    out[0] = /* your byte */;
    Ok(1)
}

static READ_DIDS: &[DidReadEntry] = &[
    DidReadEntry {
        did: 0xF190,            // your DID
        session_access: 0b111,  // any session
        security_level: 0,       // no SAL required
        func: read_my_did,
    },
    // ... existing 0xF186 entry ...
];
```

Add a smoke-test scenario in `scripts/smoke_test.py` and run
`python3 scripts/smoke_test.py` to verify wire format. The firmware
build is `cargo build --release`.

### Adding a new UDS SID

`src/can/uds/`: each SID has its own submodule (`session.rs`, `dtc.rs`,
`security.rs`, ...). To add a new SID:

1. Pick a free SID (e.g. 0x30)
2. Add a `ServiceHandler::MySid` variant in `src/can/uds/config.rs`
3. Add the handler module `src/can/uds/my_sid.rs` with `pub fn handle(state, config, req)`
4. Wire the dispatch arm in `src/can/uds/mod.rs::dispatch`
5. Add a `ServiceEntry` in `src/can/uds_config.rs::SERVICES`
6. Add a smoke-test scenario

### SecurityAccess (0x27) — multi-SAL

The seed/key derivation is LFSR + bit-reversal (per the design doc §3.4,
matches MiniUds). Per-SAL `key_masks: [u32; 3]` lives in
`src/can/uds_config.rs::UDS_CONFIG`. To rotate the SAL1 mask:

```rust
key_masks: [0xNEW_MASK_SAL1, 0x524C_5E63, 0xA5C3_F11B],
```

Reference vector: seed=0xA5A5A5A5, mask=0x30002212 → key=0x497DFE82.
The Python smoke test `s_uds_lfsr_key_known` enforces this.

## License

TBD
