# foc-rust

Field-Oriented Control firmware in Rust on the **ST B-G431B-ESC1** dev board
(STM32G431CBU6, Cortex-M4F @ 170 MHz, 128 KB flash, 32 KB SRAM).

## Status

| Component | Status |
|-----------|--------|
| Workspace (`common` + `foc-algo` + `uds-core`) | ✅ |
| Board support (HSE → PLL → 170 MHz, USART2 PB3/PB4 @ 921600 baud) | ✅ |
| Shell (`help` / `version` / `info` / `reboot` / `spin` / `stop`) | ✅ |
| In-app OTA (UDS 0x34/0x36/0x37 over FDCAN1) | ✅ |
| UDS core (`uds-core` crate, AES-128-ECB SAL, 0x27 multi-SAL) | ✅ |
| FOC algorithm (`foc-algo` crate, libm only) | ✅ |
| Closed-loop torque/speed control on the B-G431B-ESC1 | ⏳ |

## Workspace

```
foc-rust/                 # app binary (124 KB slot @ 0x0800_0000)
├── src/
│   ├── main.rs           # composition root: embassy-executor async
│   ├── bsp.rs            # clock tree + peripheral init (TIM1, USART2, FDCAN1)
│   ├── drivers/
│   │   ├── debug_uart.rs # Uart2Sink + embedded-io adapters
│   │   ├── motor_pwm.rs  # TIM1 3-phase complementary PWM
│   │   └── can/          # FDCAN1 init + canopen NMT/heartbeat + uds_bridge
│   ├── shell/            # embedded-cli 0.2.1 — 6 commands
│   ├── tasks/            # heartbeat, motor, shell_task, canopen_task
│   ├── motor/            # OpenLoopCmd shared cell + open-loop generator
│   ├── uds/              # UDS statics + dispatch glue
│   ├── ota/              # in-app OTA state machine (UDS 0x34/0x36/0x37)
│   └── metadata.rs       # post-OTA metadata block reader
│
common/                    # foc-common (reserved; currently empty)
└── src/lib.rs
│
foc-algo/                  # foc-algo lib (no_std, libm only)
└── src/
    ├── lib.rs            # FocController, MotorParams
    ├── cascade.rs        # current + speed PI cascades
    ├── field_weakening.rs, mtpa.rs, decoupling.rs
    ├── observer.rs       # SmoObserver (sensorless angle)
    ├── protection.rs     # I2tLimiter, fault gating
    ├── state.rs, motor.rs
    ├── loops/            # inner / outer loop scaffolding
    └── math/             # vector ops, transforms
│
uds-core/                  # uds-core lib (no_std, zero platform deps)
└── src/
    ├── lib.rs            # crate root + porting guide
    ├── types.rs          # Session, SecurityLevel, SrvState, Nrc
    ├── state.rs          # UdsState + response buffer
    ├── table.rs          # UdsConfig schema + dispatch_sid()
    ├── crypto.rs         # AES-128-ECB key derivation
    ├── pending.rs        # 0x78 ResponsePending queue
    └── dtc.rs            # DTC storage
│
scripts/
├── flash_and_run.sh      # build + flash + probe-rs run
├── setup_vcan.sh         # bring up vcan0 for live smoke tests
├── firmware_emulator.py  # Python firmware model on a vcan bus
└── smoke_test.py         # 38 scenarios: sim + live modes
```

## Flash Layout

| Region | Address | Size | Content |
|--------|---------|------|---------|
| App | `0x0800_0000` | 124 KB | Application firmware |
| Metadata | `0x0801_F800` | 2 KB | Magic + image size + CRC32 + version + build TS |

In-app OTA writes the new image directly over the running app region
(0x0800_0000–0x0801_F7FF); the OTA handler is RAM-resident via
`#[link_section = ".data"]` so the controller cannot prefetch its
PC out from under itself mid-write.

## Toolchain

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs --features=cli
```

## Build

```bash
cargo build                          # debug profile
cargo build --release                # release, size-optimized
cargo build -p foc-algo              # portable FOC math crate only
cargo build -p uds-core              # portable UDS protocol crate only
```

## Flash + Run

```bash
probe-rs download --chip STM32G431CBUx --base-address 0x08000000 \
    target/thumbv7em-none-eabihf/release/foc-rust
```

Or use the bundled script:

```bash
./scripts/flash_and_run.sh
```

After flashing, connect a USB-TTL to PB3 (TX) / PB4 (RX) / GND:

```bash
screen /dev/ttyUSB0 921600
```

## Shell Commands

| Command | Action |
|---------|--------|
| `help` | List available commands |
| `version` | Firmware version |
| `info` | Chip + flash + SRAM info |
| `reboot` | Reset MCU |
| `spin <freq_hz> <voltage>` | Start open-loop rotating voltage vector |
| `stop` | Soft-stop the open-loop spin |

## Clock Configuration

- HSE 8 MHz crystal → PLLN=85 → PLLR=DIV4 → **170 MHz sysclk**
- APB1 = DIV4 → 42.5 MHz (USART2 clk = 170 MHz, baud = 170 MHz / 184 ≈ 921600)
- AHB = DIV1, APB2 = DIV1, `boost = true` (required for sysclk > 150 MHz per RM0440 §7.4.3)

## Key Design Decisions

- **Single-slot OTA** — no A/B swapping; 128 KB flash cannot spare a second slot.
- **In-app OTA via UDS 0x34/0x36/0x37 over FDCAN1** (not bootloader-mediated).
  No separate bootloader binary; the OTA handler runs out of the same flash
  it's writing, with the hot path pinned to `.data` to dodge prefetch.
- **PAC-level flash driver** — `src/ota/flash.rs` drives the FLASH peripheral
  directly via `embassy_stm32::pac` (no HAL); 8-byte aligned writes keep
  flash program time inside the controller's prefetch window.
- **`foc-algo` is a portable `no_std` math crate** (libm only);
  **`uds-core` is a portable `no_std` UDS engine** (aes + critical-section).
  Both compile on any target that supports `core` and are host-testable
  with plain `cargo test`.
- **Layered** — `src/canopen/` is gone; CANopen NMT/heartbeat lives in
  `src/drivers/can/canopen.rs`, and UDS is decoupled from CANopen entirely,
  with its own top-level modules:
  - `src/uds/` — application glue (statics, dispatch, tick)
  - `src/ota/` — application glue (transfer state machine, flash writes)
  - `src/drivers/` — HAL peripherals (USART2, TIM1, FDCAN1)

## UDS over FDCAN1

The full service dispatch table is in
[`src/uds/static_config.rs`](src/uds/static_config.rs) (`SERVICES`,
`READ_DIDS`, `WRITE_DIDS`, `ROUTINES_*`).
Key entries:

- `0x22` ReadDataByIdentifier — `0xF186` ActiveDiagSession
- `0x2E` WriteDataByIdentifier — `0xF180` KeyDataMasks (48 bytes, writable
  at runtime to rotate SAL1/2/3 keys; SAL2+ required)
- `0x27` SecurityAccess — multi-SAL, **AES-128-ECB** key derivation
  (`uds_core::crypto::generate_key`); seed is read from the STM32G4
  hardware RNG (`src/uds/static_config.rs::rng_seed`)
- `0x34` / `0x36` / `0x37` — in-app OTA over FDCAN1
- `0x78` ResponsePending — handled by `uds_core::pending`

See [`docs/superpowers/specs/2026-07-03-uds-rewrite-design.md`](docs/superpowers/specs/2026-07-03-uds-rewrite-design.md)
for the full design; the older `2026-07-02-can-ota-uds-design.md` is kept
for history.

### Adding a new DID (ReadDataByIdentifier 0x22)

Add a callback + one row in `READ_DIDS` — no Rust dispatcher change:

```rust
fn read_my_did(out: &mut [u8; 64]) -> Result<usize, Nrc> {
    out[0] = /* your byte */;
    Ok(1)
}

static READ_DIDS: &[DidReadEntry] = &[
    DidReadEntry { did: 0xF190, session_access: 0b111, security_level: 0, func: read_my_did },
    // ... existing 0xF186 entry ...
];
```

Writes follow the same pattern against `WRITE_DIDS`; routines go in
`ROUTINES_START` / `ROUTINES_STOP` / `ROUTINES_RESULT`. Then add a
scenario to `scripts/smoke_test.py` and run
`python3 scripts/smoke_test.py` to verify wire format.

### Adding a new SID

Pick an unused SID, add a `ServiceHandler::MySid` variant in
`uds_core::table`, write the handler logic (pure, no platform deps),
and append a `ServiceEntry` to `SERVICES` in `src/uds/static_config.rs`.

### SecurityAccess (0x27) — multi-SAL

AES-128-ECB: `key = AES_encrypt_ecb(seed, key_material)`. Default
key material per SAL lives in
`src/uds/static_config.rs::UDS_CONFIG::key_masks` (16 bytes each).
Rotate via DID `0xF180` at runtime. The NIST AES-128 ECB known-answer
test (`uds_core::crypto::tests::aes_kat`) enforces the algorithm.

## License

TBD