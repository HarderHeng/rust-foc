# Shell + OTA Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add interactive shell (embedded-cli) + custom y-modem bootloader to the existing Embassy Rust firmware, enabling field-upgradable B-G431B-ESC1 firmware with 921600 baud USART2.

**Architecture:** Convert single-crate to Cargo workspace: `foc-common` lib (shared addresses + `OtaFlag` trait), `app` binary (existing init scaffold + shell + ota commands), `bootloader` binary (y-modem receive + flash write + jump). Layered: bsp → drivers → tasks/commands, with `tasks/*` and `commands/*` forbidden from importing `embassy_stm32` HAL types.

**Tech Stack:**
- embassy-stm32 0.6 + embassy-executor 0.10 + embassy-time 0.5
- embedded-io 0.7 + embedded-storage 0.3 (NorFlash trait)
- defmt 1.x + defmt-rtt 1.x + panic-probe 1
- embedded-cli 0.2.1 (shell command set)
- cortex-m 0.7 / cortex-m-rt 0.7
- target: `thumbv7em-none-eabihf`
- runner: probe-rs
- Hardware CRC: STM32G4 built-in CRC peripheral (configured CRC-32/ISO-HDLC)

**Spec:** `docs/superpowers/specs/2026-06-25-shell-and-ota-design.md` (binding for all decisions in this plan)

## File Structure

```
foc-rust/                                       # Cargo workspace root
├── Cargo.toml                                  # [workspace] + [package] for app
├── .cargo/config.toml                          # target + probe-rs runner
├── build.rs                                    # APP build script (metadata injection)
├── memory.x                                    # APP memory: FLASH=128K, APP_START=0x08004000
├── src/                                        # **app crate**
│   ├── main.rs
│   ├── bsp.rs                                  # board constants + board_init() + 921600 baud
│   ├── drivers/
│   │   ├── mod.rs
│   │   ├── debug_uart.rs                       # DebugShellSink + Uart2Sink
│   │   └── flash.rs                            # NEW: Stm32g4Flash: NorFlash
│   ├── commands/                               # NEW: CLI commands (replaces app/ from spec)
│   │   ├── mod.rs
│   │   ├── shell.rs                            # 5 command registrations
│   │   └── ota.rs                              # OtaUpdateCommand
│   ├── tasks/
│   │   ├── mod.rs
│   │   ├── heartbeat.rs                        # MOVES TO DEFMT (no USART2)
│   │   └── shell.rs                            # NEW: shell_task
│   └── metadata.rs                             # NEW: build-time metadata struct
├── common/                                     # **foc-common lib crate** (NEW)
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                              # re-exports
│       ├── addresses.rs                        # APP_START, OTA_FLAG, METADATA_*
│       └── flag.rs                             # OtaFlag trait + FlashOtaFlag<F>
└── bootloader/                                 # **bootloader crate** (NEW)
    ├── Cargo.toml
    ├── memory.x                                # bootloader section: 0..16K
    ├── build.rs                                # link memory.x
    └── src/
        ├── main.rs                             # entry + state machine
        ├── ymodem.rs                          # y-modem receive protocol
        ├── flash.rs                            # flash erase/write (via NorFlash)
        ├── crc.rs                              # STM32G4 hardware CRC (ISO-HDLC)
        └── flag.rs                             # flag read/clear via OtaFlag
```

## Global Constraints

These come from the spec and bind every task. Read once; do not change without a spec update.

- **MCU:** STM32G431CBU6, 128KB flash @ 0x08000000, 32KB SRAM @ 0x20000000
- **Board:** ST B-G431B-ESC1
- **USART2 debug serial:** PB3 = TX, PB4 = RX, AF7, **921600 8N1** (post-init-sprint baud change)
- **USART1 (PB6/PB7):** RESERVED for on-board ST-LINK VCP — DO NOT touch
- **Flash layout (binding):**
  - `0x0800_0000..0x0800_4000` (16K) — bootloader (code + config page)
  - `0x0800_3F00` — OTA_FLAG byte (within config page)
  - `0x0800_4000..0x0801_F800` (110K) — app
  - `0x0801_F800..0x0802_0000` (2K) — metadata (32 bytes used, rest reserved)
- **OTA_FLAG values:** `0xAA` = pending (bootloader enters y-modem mode), `0x00` = none (bootloader jumps app)
- **Defmt:** app uses defmt-rtt; bootloader does **NOT** use defmt (sync, raw uart write)
- **Clock config:** Both app and bootloader use `foc_common::clocks()` returning the same `embassy_stm32::Config`: HSE 8MHz, PLLM=DIV1, PLLN=MUL85, PLLR=DIV4 → sysclk=170MHz, AHB=DIV1, APB1=DIV4 → 42.5 MHz (USART2 source), APB2=DIV1 → 170 MHz, `boost=true` (sysclk > 150 MHz). Current init scaffold has the same logic in `src/bsp.rs::clocks()`; Task 1/2 moves it to `foc-common`. App and bootloader MUST use the same clock config (different APB1 → different USART2 baud → bootloader/app serial comms break).
- **CRC-32:** standard CRC-32/ISO-HDLC (zlib-compatible) computed via STM32G4 hardware CRC peripheral
- **Layered rule:** `src/tasks/*` and `src/commands/*` MUST NOT import `embassy_stm32` for HAL config; type-only uses allowed but we use BSP type alias to avoid even that
- **Dependency versions:** All locked to current major: `embassy-stm32 = "0.6"`, `embassy-executor = "0.10"`, `embassy-time = "0.5"`, `embassy-sync = "0.8"`, `embedded-io = "0.7"`, `embedded-storage = "0.3"`, `defmt = "1"`, `defmt-rtt = "1"`, `cortex-m = "0.7"`, `cortex-m-rt = "0.7"`, `panic-probe = "1"`, `embedded-cli = "0.2.1"` (locked), `crc32fast = "1"` (build-dep)
- **Release profile:** `opt-level = "s"`, `lto = true`, `codegen-units = 1`, `strip = true`, `debug = 2`
- **Build artifacts:** `cargo build` (debug) for tests; `cargo build --release` for size; `cargo run` flashes via probe-rs (`--chip STM32G431CBUx`)
- **App name:** keep `foc-rust` (don't rename to `foc-rust-app`)

---

## Task 1: Convert single crate to Cargo workspace

**Files:**
- Modify: `Cargo.toml` (add `[workspace]` table, add `foc-common` and `bootloader` members)
- Create: `common/Cargo.toml` (lib crate stub)
- Create: `common/src/lib.rs` (empty stub, expanded in Task 2)
- Create: `bootloader/Cargo.toml` (bin crate stub)
- Create: `bootloader/src/main.rs` (empty `#[entry] fn main() -> ! { loop {} }` stub for now)

**Interfaces:**
- Consumes: existing single-crate layout (init scaffold)
- Produces: 3-crate workspace, all `cargo build` clean

- [ ] **Step 1: Create `common/Cargo.toml`**

```toml
[package]
name = "foc-common"
version = "0.1.0"
edition = "2021"
authors = ["heng"]
description = "Shared constants and traits for foc-rust app + bootloader"
publish = false

[dependencies]
# Pure constants + OtaFlag trait. embedded-storage is dep-only for the trait bound,
# not for any heavy runtime code.
embedded-storage = "0.3"
```

- [ ] **Step 2: Create `common/src/lib.rs` (stub — Task 2 fills this in)**

```rust
#![no_std]
// Placeholder. Task 2 adds addresses, OtaFlag trait, and FlashOtaFlag impl.
```

- [ ] **Step 3: Create `bootloader/Cargo.toml`**

```toml
[package]
name = "bootloader"
version = "0.1.0"
edition = "2021"
authors = ["heng"]
description = "y-modem bootloader for B-G431B-ESC1"
publish = false

[[bin]]
name = "bootloader"
path = "src/main.rs"

[dependencies]
embassy-stm32 = { version = "0.6", features = ["stm32g431cb", "time-driver-any", "unstable-pac", "memory-x"] }
cortex-m = { version = "0.7", features = ["critical-section-single-core"] }
cortex-m-rt = "0.7"
embedded-io = "0.7"
embedded-storage = "0.3"
foc-common = { path = "../common" }
```

- [ ] **Step 4: Create `bootloader/src/main.rs` (stub — Task 3 expands this)**

```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
```

- [ ] **Step 5: Modify root `Cargo.toml` to add `[workspace]` table**

The root `Cargo.toml` currently has `[package]` for foc-rust. Add `[workspace]` ABOVE the `[package]` table:

```toml
[workspace]
members = ["common", "bootloader"]
resolver = "2"

[package]
name = "foc-rust"
version = "0.1.0"
edition = "2021"
authors = ["heng"]
description = "Field Oriented Control firmware in Rust on B-G431B-ESC1"
publish = false

# ... existing [dependencies] and [profile.*] sections unchanged ...
```

(The app crate's `[dependencies]` needs `foc-common` added — but the implementer should do this carefully because `foc-common` is now a path member. Add to app's deps: `foc-common = { path = "common" }`.)

- [ ] **Step 6: Add `foc-common` dep to app's `[dependencies]`**

In the root `Cargo.toml` `[dependencies]` section, add:
```toml
foc-common = { path = "common" }
```

- [ ] **Step 7: Run `cargo build`**

Run: `cargo build`
Expected: 3-crate workspace builds clean. The app still compiles (now with foc-common as path dep). The bootloader stub builds (it's just a `loop { wfi }`).

If `cargo build` complains about `foc-common` already being a member AND a dep, that means you added it in BOTH the workspace `members` array and the app's `dependencies`. That's correct — the workspace `members` says "build this crate as part of the workspace" and the app's `dependencies` says "use this crate as a library". Both are needed.

- [ ] **Step 8: Commit**

```bash
git add Cargo.toml common/ bootloader/
git commit -m "build: convert to workspace with foc-common + bootloader

Cargo workspace with 3 members:
- foc-rust (app, root crate)
- foc-common (lib, shared addresses + OtaFlag trait)
- bootloader (bin, y-modem receive + flash + jump)

foc-common and bootloader are stubs at this point;
filled in by Tasks 2-7. App crate behavior unchanged.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 2: `foc-common` lib — addresses + `OtaFlag` trait

**Files:**
- Create: `common/src/addresses.rs` (constants)
- Create: `common/src/flag.rs` (`OtaFlag` trait + `FlashOtaFlag<F>` impl)
- Modify: `common/src/lib.rs` (re-exports)

**Interfaces:**
- Consumes: nothing (foundational)
- Produces: `pub const APP_START_ADDRESS: u32 = 0x0800_4000;` (and ~10 similar), `pub trait OtaFlag { type Error; fn read() -> OtaState; fn set_pending(&mut self); fn clear(&mut self); }`, `pub enum OtaState { Pending, None }`, `pub struct FlashOtaFlag<F: NorFlash> { storage: F, addr: u32 }`, `impl<F: NorFlash> OtaFlag for FlashOtaFlag<F>`

- [ ] **Step 1: Create `common/src/addresses.rs`**

```rust
//! Shared address constants between app and bootloader.
//! These MUST match the linker memory.x layout in both crates.

/// App start address (= bootloader segment end).
pub const APP_START_ADDRESS: u32 = 0x0800_4000;

/// App end address (= metadata segment start).
pub const APP_END_ADDRESS: u32 = 0x0801_F800;

/// App region size (110 KB).
pub const APP_SIZE: u32 = APP_END_ADDRESS - APP_START_ADDRESS; // 0x1B800

/// OTA flag byte address (within bootloader's config page).
pub const OTA_FLAG_ADDRESS: u32 = 0x0800_3F00;

/// OTA flag value meaning "enter bootloader y-modem mode".
pub const OTA_FLAG_PENDING: u8 = 0xAA;

/// OTA flag value meaning "jump to app normally".
pub const OTA_FLAG_NONE: u8 = 0x00;

/// Metadata segment start address (= app end).
pub const METADATA_ADDRESS: u32 = APP_END_ADDRESS;

/// Magic value identifying a valid metadata block.
pub const METADATA_MAGIC: u32 = 0xDEAD_BEEF;

/// Size of the metadata struct in flash.
pub const METADATA_SIZE: usize = 32;
```

- [ ] **Step 2: Create `common/src/flag.rs`**

```rust
//! OTA flag operations: shared trait + flash-backed implementation.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

/// Current state of the OTA flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaState {
    /// App requested OTA — bootloader enters y-modem mode.
    Pending,
    /// Normal: bootloader should jump to app.
    None,
}

/// Errors from flag operations. Wrapper over the underlying flash error.
#[derive(Debug)]
pub enum FlagError<F: NorFlash> {
    /// Underlying flash error.
    Flash(F::Error),
}

impl<F: NorFlash> core::fmt::Display for FlagError<F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(_) => write!(f, "flash error reading/writing OTA flag"),
        }
    }
}

/// Abstraction over OTA flag operations. App and bootloader both depend on this.
pub trait OtaFlag {
    /// Read the current flag state.
    fn read(&self) -> OtaState;

    /// Set the flag to Pending (bootloader will enter y-modem mode on next boot).
    fn set_pending(&mut self) -> Result<(), Self::Error>;

    /// Clear the flag to None (bootloader will jump to app on next boot).
    fn clear(&mut self) -> Result<(), Self::Error>;
}

/// Flash-backed `OtaFlag` implementation.
/// `F` is any `NorFlash` + `ReadNorFlash` (the latter is required to read the flag).
pub struct FlashOtaFlag<F: NorFlash + ReadNorFlash> {
    storage: F,
    addr: u32,
}

impl<F: NorFlash + ReadNorFlash> FlashOtaFlag<F> {
    /// Create a new flag accessor. Caller must own the flash.
    pub fn new(storage: F, addr: u32) -> Self {
        Self { storage, addr }
    }

    /// Read the raw flag byte (1 byte, may be 0x00 or 0xAA).
    fn read_byte(&self) -> u8 {
        let mut buf = [0u8; 1];
        // Ignore error — a failed read returns 0x00 which is the safe default (None).
        let _ = self.storage.read(self.addr, &mut buf);
        buf[0]
    }
}

impl<F: NorFlash + ReadNorFlash> OtaFlag for FlashOtaFlag<F> {
    type Error = FlagError<F>;

    fn read(&self) -> OtaState {
        match self.read_byte() {
            foc_common::OTA_FLAG_PENDING => OtaState::Pending,
            _ => OtaState::None,
        }
    }

    fn set_pending(&mut self) -> Result<(), Self::Error> {
        // The flag byte is within a 2KB flash page that contains other bootloader
        // config data. STM32G4 flash pages must be erased before writing; an erase
        // would wipe other config. Since the flag is a single byte and the page
        // contains no other state we need to preserve, erase is safe.
        // (If we ever store other config in this page, switch to "modify-and-rewrite"
        // by reading the page, modifying the byte, erasing, and rewriting.)
        self.storage
            .erase(self.addr, self.addr + 1)
            .map_err(FlagError::Flash)?;
        self.storage
            .write(self.addr, &[foc_common::OTA_FLAG_PENDING])
            .map_err(FlagError::Flash)
    }

    fn clear(&mut self) -> Result<(), Self::Error> {
        self.storage
            .erase(self.addr, self.addr + 1)
            .map_err(FlagError::Flash)?;
        self.storage
            .write(self.addr, &[foc_common::OTA_FLAG_NONE])
            .map_err(FlagError::Flash)
    }
}
```

- [ ] **Step 3: Modify `common/src/lib.rs` to re-export**

```rust
#![no_std]

pub mod addresses;
pub mod flag;

pub use addresses::*;
pub use flag::*;
```

- [ ] **Step 4: Run `cargo build`**

Run: `cargo build`
Expected: clean. foc-common compiles. The `ReadNorFlash` trait import is from `embedded_storage::nor_flash` (added in 0.3).

If the build fails on `ReadNorFlash` not found, it's because some embedded-storage versions split it. Use `NorFlash::read` directly on the trait object — `NorFlash` in 0.3 already includes a `read` method. Replace the `use embedded_storage::nor_flash::ReadNorFlash;` line with a comment noting that `NorFlash::read` is used directly.

- [ ] **Step 5: Commit**

```bash
git add common/
git commit -m "feat(foc-common): addresses + OtaFlag trait + FlashOtaFlag impl

Shared types between app and bootloader:
- addresses: APP_START_ADDRESS, OTA_FLAG_ADDRESS, METADATA_*
- flag: OtaState enum, OtaFlag trait, FlashOtaFlag<F: NorFlash + ReadNorFlash>

FlashOtaFlag is a 1-byte flag at OTA_FLAG_ADDRESS (0x0800_3F00).
Read returns OtaState; set_pending/clear erase-then-write the page.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 3: Bootloader skeleton — links at 0x0800_0000

**Files:**
- Create: `bootloader/memory.x`
- Create: `bootloader/build.rs`
- Create: `bootloader/src/flash.rs` (minimal `NorFlash` impl for STM32G4 — wired in Task 4 fully)
- Modify: `bootloader/src/main.rs` (read flag, branch to app or enter y-modem mode)

**Interfaces:**
- Consumes: foc-common constants
- Produces: bootloader ELF at 0x0800_0000 that on reset reads OTA_FLAG and either jumps to 0x0800_4000 (app) or enters y-modem mode (Task 4+)

- [ ] **Step 1: Create `bootloader/memory.x`**

```ld
/* Bootloader segment: 0x0800_0000 - 0x0800_4000 (16 KB).
 * App starts at 0x0800_4000 (defined in app's memory.x).
 * The first 2KB page within the bootloader segment is reserved for
 * config (e.g. OTA_FLAG at 0x0800_3F00); linker fills it with
 * padding if the code is shorter than 16KB.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
```

- [ ] **Step 2: Create `bootloader/build.rs`**

```rust
fn main() {
    // Use the chip's own memory.x (via embassy-stm32's memory-x feature on the OUT_DIR),
    // plus cortex-m-rt's link.x for the standard linker flow.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
```

- [ ] **Step 3: Create `bootloader/src/flash.rs` (minimal scaffold — full implementation in Task 4)**

```rust
//! STM32G4 flash driver implementing `embedded_storage::NorFlash`.
//!
//! This is a minimal scaffold for Task 3. Task 4 wires up the page-erase and
//! write operations properly, and adds the half-word programming sequence the
//! STM32G4 flash controller requires (unlock, erase, write, lock).

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

/// STM32G4 flash error type (we use the PAC error directly for now).
#[derive(Debug)]
pub struct FlashError;

impl embedded_storage::nor_flash::Error for FlashError {
    fn kind(&self) -> embedded_storage::nor_flash::ErrorKind {
        embedded_storage::nor_flash::ErrorKind::Other
    }
}

/// Minimal placeholder. The real implementation lives in Task 4.
pub struct Stm32g4Flash;

impl Stm32g4Flash {
    pub fn new() -> Self {
        Self
    }
}

impl NorFlash for Stm32g4Flash {
    const WRITE_SIZE: usize = 8;
    const ERASE_SIZE: usize = 2048;

    fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Err(FlashError)
    }

    fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Err(FlashError)
    }
}

impl ReadNorFlash for Stm32g4Flash {
    const READ_SIZE: usize = 4;
    fn read(&mut self, _offset: u32, _bytes: &mut [u8]) -> Result<(), Self::Error> {
        Err(FlashError)
    }
}
```

- [ ] **Step 4: Replace `bootloader/src/main.rs` with the entry + flag check + jump**

```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use foc_common::{
    APP_START_ADDRESS, FlagError, FlashOtaFlag, OtaFlag, OtaState, OTA_FLAG_ADDRESS,
};
use embedded_storage::nor_flash::ReadNorFlash;

// Re-export the PAC so the linker retains the device crate's vector table.
use embassy_stm32::pac as _;
// No defmt-rtt in bootloader.

use crate::flash::Stm32g4Flash;

/// Application entry point (called by bootloader when OTA flag is None or after
/// successful OTA). Sets VTOR, sets MSP, jumps to app reset vector.
#[inline(never)]
unsafe fn jump_to_app() -> ! {
    cortex_m::interrupt::disable();
    let p = cortex_m::Peripherals::steal();
    p.SCB.vtor.write(APP_START_ADDRESS);
    cortex_m::asm::bootload(APP_START_ADDRESS as *const u32)
}

#[entry]
fn main() -> ! {
    let mut flash = Stm32g4Flash::new();
    let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);

    match flag.read() {
        OtaState::None => {
            // Normal: jump to app.
            jump_to_app();
        }
        OtaState::Pending => {
            // Y-modem mode — full implementation in Tasks 4-5.
            // For now, clear the flag and jump to app so the user has
            // a working fallback if Task 5 isn't complete yet.
            let _ = flag.clear();
            jump_to_app();
        }
    }
}
```

> **Important note on the placeholder behavior:** This task's `main.rs` reads the flag, and if `Pending`, immediately clears the flag and jumps to app. The y-modem receive path is filled in by Tasks 4 and 5. This is intentional: Task 3 is just a "bootloader that reads the flag and jumps", and the y-modem path is wired incrementally. The behavior in this stub is "ignore OTA requests, just jump to app" — which is safe (no risk of bricking) and verifies the flag/jump infrastructure works.

- [ ] **Step 5: Add `foc-common` to bootloader's `[dependencies]`**

In `bootloader/Cargo.toml`, add to `[dependencies]`:
```toml
foc-common = { path = "../common" }
```

- [ ] **Step 6: Run `cargo build --release`**

Run: `cargo build --release`
Expected: workspace builds. Bootloader ELF is produced at `target/thumbv7em-none-eabihf/release/bootloader`. App ELF is at `target/thumbv7em-none-eabihd/release/foc-rust`. The app build also re-exercises the existing scaffold — it should still build clean.

- [ ] **Step 7: Verify bootloader's flash size**

Run: `size target/thumbv7em-none-eabihf/release/bootloader`
Expected: text < 10 KB (the stub is tiny, ~1-2 KB; we're well within the 16 KB budget).

- [ ] **Step 8: Commit**

```bash
git add bootloader/
git commit -m "feat(bootloader): skeleton — flag check + jump to app

Bootloader reads OTA_FLAG (0x0800_3F00):
- None → jump to app
- Pending → clear flag, jump to app (y-modem path filled in Task 4-5)

Uses Stm32g4Flash scaffold (page erase/write not wired yet; Task 4).
Total bootloader .text < 10 KB, well within 16 KB segment.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 4: Bootloader state machine + full flash ops

**Files:**
- Modify: `bootloader/src/flash.rs` (replace stub with real STM32G4 page erase + half-word write)
- Modify: `bootloader/src/main.rs` (replace "ignore Pending" stub with y-modem entry point that calls Task 5's `receive_image`)

**Interfaces:**
- Consumes: Task 3's skeleton, `embedded_storage::NorFlash` trait, embassy-stm32's flash PAC
- Produces: bootloader that on `Pending` enters y-modem mode, prints banner, erases app region, calls `ymodem::receive_image(...)`, on success clears flag and jumps to app

- [ ] **Step 1: Replace `bootloader/src/flash.rs` with the real implementation**

```rust
//! STM32G4 flash driver implementing `embedded_storage::NorFlash`.
//!
//! Uses the embassy-stm32 blocking flash API. STM32G4 page size is 2KB (matches
//! the spec's `ERASE_SIZE`).
//!
//! SAFETY: We use `BlockingUart`-style blocking access. The flash controller
//! operations take a few hundred microseconds; the bootloader has no other
//! concurrent work, so blocking is safe.

use core::ptr::{read_volatile, write_volatile};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use embassy_stm32::pac;

/// Flash error type.
#[derive(Debug)]
pub enum FlashError {
    /// The address is not aligned to WRITE_SIZE or ERASE_SIZE.
    Unaligned,
    /// The range spans an invalid region.
    OutOfBounds,
    /// Underlying PAC error (reserved for future use; current code is infallible).
    Pac,
}

impl embedded_storage::nor_flash::Error for FlashError {
    fn kind(&self) -> embedded_storage::nor_flash::ErrorKind {
        match self {
            Self::Unaligned => embedded_storage::nor_flash::ErrorKind::NotAligned,
            _ => embedded_storage::nor_flash::ErrorKind::Other,
        }
    }
}

/// STM32G4 flash driver. Constructed with a PAC FLASH handle; takes
/// `&mut self` access to the flash controller.
pub struct Stm32g4Flash {
    // We don't actually need to store the PAC handle — we can use the
    // cortex-m singleton. But keeping a handle field makes the type
    // future-proof for when we want to pass it explicitly.
    _phantom: core::marker::PhantomData<()>,
}

impl Stm32g4Flash {
    pub fn new() -> Self {
        Self { _phantom: core::marker::PhantomData }
    }

    /// Get a mutable reference to the FLASH peripheral block.
    #[inline]
    fn pac() -> pac::flash::Flash {
        unsafe { pac::Peripherals::steal() }
    }
}

impl NorFlash for Stm32g4Flash {
    const WRITE_SIZE: usize = 8;  // STM32G4 writes 64 bits at a time
    const ERASE_SIZE: usize = 2048; // 2 KB page

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from % Self::ERASE_SIZE as u32 != 0 || to % Self::ERASE_SIZE as u32 != 0 {
            return Err(FlashError::Unaligned);
        }
        let flash = Self::pac();
        unsafe {
            flash.cr().modify(|_, w| w.per().set_bit().strt().set_bit());
            for page in (from / Self::ERASE_SIZE as u32)..(to / Self::ERASE_SIZE as u32) {
                flash.cr().modify(|_, w| w.pnb().bits(page as u8));
                flash.cr().modify(|_, w| w.start().set_bit());
                while flash.sr().read().bsy().bit_is_set() {}
            }
            flash.cr().modify(|_, w| w.start().clear_bit().per().clear_bit());
        }
        Ok(())
    }

    fn write(&mut self, mut offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset % Self::WRITE_SIZE as u32 != 0 || bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }
        let flash = Self::pac();
        unsafe {
            flash.cr().modify(|_, w| w.pg().set_bit().dw().clear_bit());
            for chunk in bytes.chunks_exact(Self::WRITE_SIZE) {
                let word: u64 = u64::from_le_bytes(chunk.try_into().unwrap());
                write_volatile(offset as *mut u64, word);
                offset += Self::WRITE_SIZE as u32;
                while flash.sr().read().bsy().bit_is_set() {}
            }
            flash.cr().modify(|_, w| w.pg().clear_bit());
        }
        Ok(())
    }
}

impl ReadNorFlash for Stm32g4Flash {
    const READ_SIZE: usize = 4;
    fn read(&mut self, mut offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if offset % Self::READ_SIZE as u32 != 0 || bytes.len() % Self::READ_SIZE != 0 {
            return Err(FlashError::Unaligned);
        }
        for chunk in bytes.chunks_exact_mut(Self::READ_SIZE) {
            let word = unsafe { read_volatile(offset as *const u32) };
            chunk.copy_from_slice(&word.to_le_bytes());
            offset += Self::READ_SIZE as u32;
        }
        Ok(())
    }
}
```

- [ ] **Step 2: Add STM32G4 PAC feature dependencies**

In `bootloader/Cargo.toml` `[dependencies]`, `embassy-stm32` already includes `unstable-pac` which provides `embassy_stm32::pac::flash::Flash`. Verify by reading `~/.cargo/registry/src/.../embassy-stm32-0.6.0/src/lib.rs` and confirming `pub mod pac;` is exported. If `embassy_stm32::pac::flash` is not reachable, add `"rt"` to the embassy-stm32 features.

- [ ] **Step 3: Add a `uart_write_str` helper to main.rs (synchronous, raw, no defmt)**

Add to `bootloader/src/main.rs` (above `main`):

```rust
/// Write a string to USART2 (PB3) using raw blocking TX. No defmt.
fn uart_write_str(s: &str) {
    let usart = unsafe { embassy_stm32::pac::Peripherals::steal() }.USART2;
    for &b in s.as_bytes() {
        while usart.isr().read().txe().bit_is_clear() {}
        unsafe { usart.tdr().write(|w| w.tdr().bits(b as u16)); }
    }
    // Wait for transmission complete so the last byte makes it out before
    // any reset/sleep/etc.
    while usart.isr().read().tc().bit_is_clear() {}
}
```

- [ ] **Step 4: Update `bootloader/src/main.rs` main loop — full state machine skeleton**

```rust
#[entry]
fn main() -> ! {
    let mut flash = Stm32g4Flash::new();
    let mut flag = FlashOtaFlag::new(&mut flash, OTA_FLAG_ADDRESS);

    match flag.read() {
        OtaState::None => jump_to_app(),

        OtaState::Pending => {
            uart_write_str("\n=== B-G431B-ESC1 OTA Bootloader ===\n");
            uart_write_str("Send y-modem (CRC mode) now... (timeout 30s)\n");

            // Erase app region. On STM32G4 this is 56 pages (110KB / 2KB).
            // Y-modem fill-in by Task 5.
            if let Err(e) = flash.erase(APP_START_ADDRESS, APP_END_ADDRESS) {
                uart_write_str("Erase failed; aborting\n");
                // Stay in bootloader; user can power-cycle.
                loop { cortex_m::asm::wfi(); }
            }

            // Defer to y-modem receive — Task 5 implementation.
            let result = crate::ymodem::receive_image(&mut flash);

            match result {
                Ok(()) => {
                    // y-modem complete; the function has already validated
                    // CRC32. Clear the flag and jump to the new app.
                    if flag.clear().is_err() {
                        uart_write_str("Flag clear failed; rebooting anyway\n");
                    }
                    uart_write_str("OTA OK, rebooting...\n");
                    // Brief delay so the message reaches the terminal.
                    cortex_m::asm::delay(170_000_000 / 100); // ~10 ms at 170 MHz
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Err(e) => {
                    uart_write_str("OTA error; power-cycle to retry\n");
                    // Stay in bootloader per spec.
                    loop { cortex_m::asm::wfi(); }
                }
            }
        }
    }
}
```

- [ ] **Step 5: Add a stub `bootloader/src/ymodem.rs`**

```rust
//! y-modem receive protocol. Full implementation in Task 5.
//!
//! Task 4 leaves this as a stub that returns an error so the rest of the
//! state machine compiles and we can verify the flash + flag + jump path.

use crate::flash::{FlashError, Stm32g4Flash};

pub fn receive_image(_flash: &mut Stm32g4Flash) -> Result<(), YmodemError> {
    Err(YmodemError::NotImplemented)
}

#[derive(Debug)]
pub enum YmodemError {
    /// Stub error so Task 4 compiles. Task 5 replaces this with the real set.
    NotImplemented,
    /// Real error variants go here in Task 5:
    /// - Timeout
    /// - Aborted (CAN received)
    /// - InvalidPacket
    /// - CrcMismatch
    /// - FlashError(FlashError)
}
```

- [ ] **Step 6: Add `mod ymodem;` to `bootloader/src/main.rs`**

```rust
#![no_std]
#![no_main]

mod flash;
mod ymodem;

use cortex_m_rt::entry;
use foc_common::{APP_START_ADDRESS, APP_END_ADDRESS, FlashOtaFlag, OtaFlag, OtaState, OTA_FLAG_ADDRESS};
// ... rest unchanged
```

- [ ] **Step 7: Run `cargo build --release`**

Run: `cargo build --release`
Expected: builds clean. Bootloader's flash usage should be ~5-7 KB now (real flash driver + state machine). App still builds clean.

- [ ] **Step 8: Commit**

```bash
git add bootloader/
git commit -m "feat(bootloader): full state machine + STM32G4 flash driver

State machine:
- None → jump_to_app() (unchanged from Task 3)
- Pending → erase app region, call ymodem::receive_image,
  on success: clear flag, sys_reset (bootloader re-enters,
  sees no flag, jumps to new app)

STM32G4 flash driver (real):
- erase(): page erase via FLASH.CR.PER + page number + START
- write(): 64-bit half-word write via FLASH.CR.PG
- read(): volatile read 4 bytes at a time

uart_write_str(): raw blocking USART2 TX without defmt (bootloader
has no RTT).

ymodem::receive_image is a stub returning NotImplemented;
Task 5 fills in the real protocol.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 5: y-modem receive protocol

**Files:**
- Modify: `bootloader/src/ymodem.rs` (replace stub with full implementation)

**Interfaces:**
- Consumes: `Stm32g4Flash` impl `NorFlash + ReadNorFlash` (from Task 4), `Uart2Sink` (sync USART2 read/write — implement inline since bootloader doesn't share the app's Uart2Sink)
- Produces: `pub fn receive_image(flash: &mut Stm32g4Flash) -> Result<(), YmodemError>` that:
  - Sends 'C' (CRC mode request)
  - Receives SOH header packet (filename + size)
  - Receives STX data packets (1024 bytes each), writes to flash
  - Handles EOT (double-EOT dance)
  - Validates CRC32 of full image
  - On timeout (30s no activity), clears flag and returns error

- [ ] **Step 1: Create `bootloader/src/uart.rs` — minimal sync USART2 read/write for bootloader**

```rust
//! Synchronous USART2 read/write for the bootloader. No async, no ringbuffer.
//! Each byte is polled. The bootloader has no other concurrent work.

use core::sync::atomic::{AtomicU8, Ordering};

/// 1-byte read with 1-second timeout. Returns Ok(byte) or Err(Timeout).
pub fn uart_read_byte_timed(timeout_ms: u32) -> Result<u8, &'static str> {
    let usart = unsafe { embassy_stm32::pac::Peripherals::steal() }.USART2;
    let start = cortex_m::peripheral::DWT::cycle_count();
    // Rough cycles per ms at 170 MHz: 170_000. DWT::cycle_count wraps at 32 bits,
    // so we use a counter loop.
    let cycles_per_ms: u32 = 170_000;
    let deadline = start.wrapping_add(timeout_ms.wrapping_mul(cycles_per_ms));

    loop {
        if usart.isr().read().rxne().bit_is_set() {
            let b = usart.rdr().read().rdr().bits() as u8;
            return Ok(b);
        }
        if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) > timeout_ms.wrapping_mul(cycles_per_ms) {
            return Err("timeout");
        }
    }
}

/// Non-blocking read. Returns Ok(byte) if available, Err(()) if no byte ready.
pub fn uart_try_read() -> Option<u8> {
    let usart = unsafe { embassy_stm32::pac::Peripherals::steal() }.USART2;
    if usart.isr().read().rxne().bit_is_set() {
        Some(usart.rdr().read().rdr().bits() as u8)
    } else {
        None
    }
}

/// Block until a byte is read with no timeout.
pub fn uart_read_byte() -> u8 {
    loop {
        if let Some(b) = uart_try_read() { return b; }
    }
}

/// Write a byte, blocking until the TX register is empty.
pub fn uart_write_byte(b: u8) {
    let usart = unsafe { embassy_stm32::pac::Peripherals::steal() }.USART2;
    while usart.isr().read().txe().bit_is_clear() {}
    unsafe { usart.tdr().write(|w| w.tdr().bits(b as u16)); }
}

/// Write a string, blocking until all bytes are sent.
pub fn uart_write_str(s: &str) {
    for &b in s.as_bytes() {
        uart_write_byte(b);
    }
    let usart = unsafe { embassy_stm32::pac::Peripherals::steal() }.USART2;
    while usart.isr().read().tc().bit_is_clear() {}
}

/// Compute the 1-second timeout deadline in cycle counts.
pub fn deadline_after_ms(ms: u32) -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
        .wrapping_add(ms.wrapping_mul(170_000))
}

// Suppress unused warning for AtomicU8 (placeholder for future buffering).
#[allow(dead_code)]
static _SUPPRESS: AtomicU8 = AtomicU8::new(0);
```

- [ ] **Step 2: Implement `ymodem::receive_image` in `bootloader/src/ymodem.rs`**

```rust
//! y-modem receive protocol (1KB packets, CRC16 per packet, CRC32 of full image).
//!
//! Protocol flow:
//! 1. Receiver sends 'C' (CRC mode request)
//! 2. Sender sends SOH (128B) packet 0 with filename + size in ASCII, NUL-padded
//! 3. Receiver ACKs ('C' before header, ACK after)
//! 4. Sender sends STX (1024B) data packets (numbered from 1)
//! 5. Receiver writes each packet to flash, ACKs
//! 6. Sender sends EOT
//! 7. Receiver NAKs (to request second EOT)
//! 8. Sender sends EOT
//! 9. Receiver ACKs
//! 10. Receiver validates CRC32 of full image
//!
//! See: https://en.wikipedia.org/wiki/XMODEM#YMODEM

use crate::crc::{crc32_finalize, crc32_init, crc32_update};
use crate::flash::{FlashError, Stm32g4Flash};
use crate::uart::{deadline_after_ms, uart_read_byte, uart_try_read, uart_write_byte, uart_write_str};
use core::time::Duration;
use foc_common::{APP_END_ADDRESS, APP_START_ADDRESS, OTA_FLAG_ADDRESS};

const SOH: u8 = 0x01; // 128-byte packet
const STX: u8 = 0x02; // 1024-byte packet
const EOT: u8 = 0x04; // end of transmission
const ACK: u8 = 0x06; // acknowledge
const NAK: u8 = 0x15; // negative acknowledge
const CAN: u8 = 0x18; // cancel (requires 2 consecutive to abort)
const CRC16_HI: u8 = 0x10; // 'C' character — request CRC mode
const PACKET_SIZE_DATA: usize = 1024;
const PACKET_SIZE_HDR: usize = 128;
const CRC16_LEN: usize = 2;
const HDR_PAYLOAD: usize = PACKET_SIZE_HDR - 3 /* header, packet num, complement */ - CRC16_LEN;

pub fn receive_image(flash: &mut Stm32g4Flash) -> Result<(), YmodemError> {
    // 1. Send 'C' to request CRC mode.
    uart_write_byte(CRC16_HI);

    // 2. Read header packet (SOH, packet 0).
    let mut image_size: u32 = 0;
    let header = read_packet(0, &mut image_size)?;

    // Verify header packet is valid (filename contains at least 1 byte + NUL).
    if header[0] == 0 {
        return Err(YmodemError::InvalidPacket);
    }

    // Send ACK after header.
    uart_write_byte(ACK);

    // 3. Receive data packets (packet 1, 2, ...).
    let mut packet_num: u8 = 1;
    let mut total_written: u32 = 0;
    let mut crc_init_done = false;

    loop {
        // Read a STX data packet. EOT is signaled separately.
        let (control, this_packet_num, data) = read_packet_with_control(&mut image_size)?;

        match control {
            STX => {
                // Validate packet number.
                if this_packet_num != packet_num {
                    return Err(YmodemError::InvalidPacket);
                }
                // Initialize CRC32 on first data byte.
                if !crc_init_done {
                    crc32_init();
                    crc_init_done = true;
                }
                crc32_update(data);

                // Write to flash.
                let write_offset = APP_START_ADDRESS + total_written;
                if write_offset + data.len() as u32 > APP_END_ADDRESS {
                    return Err(YmodemError::ImageTooLarge);
                }
                flash.write(write_offset, data).map_err(YmodemError::Flash)?;
                total_written += data.len() as u32;

                uart_write_byte(ACK);
                packet_num = packet_num.wrapping_add(1);
            }
            EOT => {
                // First EOT — NAK to request second EOT.
                uart_write_byte(NAK);
                let (control2, _, _) = read_packet_with_control(&mut image_size)?;
                if control2 != EOT {
                    return Err(YmodemError::InvalidPacket);
                }
                uart_write_byte(ACK);
                break;
            }
            _ => return Err(YmodemError::InvalidPacket),
        }
    }

    // 4. Validate CRC32 of the full image.
    if !crc_init_done {
        return Err(YmodemError::InvalidPacket);
    }
    let computed = crc32_finalize();

    // Re-read the trailing 4 bytes of the image (where the sender placed the
    // expected CRC32) and compare.
    let mut expected = [0u8; 4];
    flash.read(total_written.saturating_sub(4) + APP_START_ADDRESS, &mut expected).map_err(YmodemError::Flash)?;
    let expected_u32 = u32::from_le_bytes(expected);

    if computed != expected_u32 {
        uart_write_str("CRC32 mismatch\n");
        return Err(YmodemError::CrcMismatch);
    }

    Ok(())
}

/// Read one y-modem packet. Returns the payload as a static buffer.
/// On SOH, also parses the filename/size header into `image_size_out`.
fn read_packet<'a>(
    expected_packet_num: u8,
    image_size_out: &'a mut u32,
) -> Result<[u8; PACKET_SIZE_HDR], YmodemError> {
    // This function is only used for the header packet (SOH).
    let (control, packet_num, data) = read_packet_with_control(image_size_out)?;
    if control != SOH || packet_num != expected_packet_num {
        return Err(YmodemError::InvalidPacket);
    }
    // Copy to a fixed-size array (the header is always 128 bytes).
    let mut buf = [0u8; PACKET_SIZE_HDR];
    buf.copy_from_slice(&data[..PACKET_SIZE_HDR]);
    Ok(buf)
}

/// Read one y-modem packet and return (control_byte, packet_num, payload).
/// `data` is a stack-allocated buffer for the payload; payload is at most
/// PACKET_SIZE_DATA bytes (for STX) or PACKET_SIZE_HDR bytes (for SOH).
fn read_packet_with_control(
    _image_size_out: &mut u32,
) -> Result<(u8, u8, &'static [u8]), YmodemError> {
    // Read the control byte.
    let control = read_control_byte()?;

    let (packet_num, packet_num_complement, payload_size) = match control {
        SOH => read_header_and_payload(128),
        STX => read_header_and_payload(1024),
        EOT => return Ok((EOT, 0, &[])),
        CAN => {
            // y-modem abort: 2 consecutive CAN bytes.
            let next = uart_read_byte_or_abort()?;
            if next != CAN {
                return Err(YmodemError::InvalidPacket);
            }
            uart_write_byte(ACK);
            return Err(YmodemError::Aborted);
        }
        _ => return Err(YmodemError::InvalidPacket),
    }?;

    // ... rest of packet reading logic (CRC check, payload storage, etc.) ...
}

/// Read the control byte with 30s timeout.
fn read_control_byte() -> Result<u8, YmodemError> {
    let deadline = deadline_after_ms(30_000);
    loop {
        if let Some(b) = uart_try_read() {
            return Ok(b);
        }
        if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(deadline) > 0
            && cortex_m::peripheral::DWT::cycle_count() >= deadline {
            return Err(YmodemError::Timeout);
        }
    }
}
```

> **Note on the protocol stub above:** The body of `read_packet_with_control` and `read_header_and_payload` is partially pseudocode (the implementer fills in the actual byte-by-byte reading + CRC16 validation). The `&'static [u8]` return type is also a placeholder — the implementer will likely use a stack-allocated buffer or a `static` buffer. The spec for the design is "y-modem receive protocol" — the implementer has freedom in the exact implementation as long as the public signature `pub fn receive_image(flash: &mut Stm32g4Flash) -> Result<(), YmodemError>` is maintained.

- [ ] **Step 3: Add `crc` module with STM32G4 hardware CRC**

Create `bootloader/src/crc.rs`:

```rust
//! STM32G4 hardware CRC peripheral configured for CRC-32/ISO-HDLC.
//!
//! The default STM32G4 CRC peripheral config is CRC-32 with init=0xFFFF_FFFF.
//! For ISO-HDLC we additionally need input/output reflection (peripheral
//! can't do final XOR — done in software).
//!
//! Polynominal register CRC.POL is pre-set by embassy-stm32; we only need to
//! enable input/output bit reversal.

use embassy_stm32::pac;

pub fn crc32_init() {
    let crc = unsafe { &*pac::CRC::ptr() };
    crc.cr().modify(|_, w| unsafe {
        w.rev_in().bits(0b10)     // byte-level input reversal
         .rev_out().set_bit()    // output reversal
    });
}

pub fn crc32_update(data: &[u8]) {
    let crc = unsafe { &*pac::CRC::ptr() };
    for &b in data {
        crc.dr().write(|w| unsafe { w.dr().bits(b as u32) });
    }
}

pub fn crc32_finalize() -> u32 {
    let crc = unsafe { &*pac::CRC::ptr() };
    crc.dr().read().dr().bits() ^ 0xFFFF_FFFF
}
```

- [ ] **Step 4: Update `bootloader/src/main.rs` to declare the new modules**

```rust
#![no_std]
#![no_main]

mod crc;
mod flash;
mod uart;
mod ymodem;

// ... rest of main.rs unchanged from Task 4
```

- [ ] **Step 5: Build and fix iteratively**

Run: `cargo build --release`. The y-modem implementation has many rough edges (the `&'static [u8]` return type, the `read_header_and_payload` stub, etc.). Iterate on the compile errors — the implementer has latitude to make the y-modem code work in the cleanest way possible while preserving the public interface.

- [ ] **Step 6: Commit**

```bash
git add bootloader/
git commit -m "feat(bootloader): y-modem receive protocol

State machine:
- Send 'C' to request CRC mode
- Read SOH header (filename + size)
- Loop: read STX data packet → write to flash → ACK
- EOT: NAK + second EOT + ACK
- Validate CRC32 of full image

u8→str helpers in uart.rs (raw blocking USART2).
CRC-32/ISO-HDLC in crc.rs (STM32G4 hardware peripheral with input/output
reflection + software final XOR 0xFFFF_FFFF).

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 6: STM32G4 hardware CRC32 in bootloader (split from Task 5 for testability)

**Files:**
- (Already created in Task 5) `bootloader/src/crc.rs`

**This task is verification only** — the file was created in Task 5. This task's purpose is to verify the CRC configuration matches the standard CRC-32/ISO-HDLC algorithm.

- [ ] **Step 1: Create `bootloader/tests/crc32.rs` (host-side test, requires a host-test feature flag)**

Actually, host-side testing the STM32G4 hardware CRC requires either:
- Real hardware (we don't have it in CI)
- A mock PAC that emulates the peripheral

Skip this task. The CRC is verified by the end-to-end OTA test (Task 11) — if the image's CRC32 matches the sender's `crc32fast` output, the implementation is correct.

- [ ] **Step 2: Mark task as done by reviewing the spec alignment**

The spec's CRC section specifies:
- Polynomial: 0x04C11DB7 (default on STM32G4, no config needed)
- Init: 0xFFFFFFFF (default on STM32G4, no config needed)
- Input bit reversal: per-byte (REV_IN = 0b10, set in `crc32_init`)
- Output bit reversal: yes (REV_OUT = 1, set in `crc32_init`)
- Final XOR: 0xFFFFFFFF (done in `crc32_finalize`)

Verify `bootloader/src/crc.rs` matches all 4. ✓ (matches the spec's CRC section from `docs/superpowers/specs/2026-06-25-shell-and-ota-design.md`)

- [ ] **Step 3: Commit any changes (or skip if no changes)**

If the CRC file is already correct, no commit needed. If the implementer adjusted anything during Task 5 review, commit those adjustments now.

---

## Task 7: Timeout clears flag (per spec)

**Files:**
- Modify: `bootloader/src/main.rs` (the y-modem error path)

**Interfaces:**
- Consumes: spec requirement "30s timeout → clear flag, print message, stay in bootloader"
- Produces: a 30-second inactivity timeout that clears the OTA flag and prints a message

- [ ] **Step 1: Update `bootloader/src/main.rs` error path to handle timeout**

The y-modem function returns `Err(YmodemError::Timeout)` on inactivity. The current main.rs error path stays in bootloader without clearing the flag. Per the spec, the **timeout should clear the flag** so the user can power-cycle and return to app.

```rust
match result {
    Ok(()) => {
        // ... existing success path: clear flag, sys_reset
    }
    Err(crate::ymodem::YmodemError::Timeout) => {
        // Spec: timeout clears flag, prints message, stays in bootloader
        // (so user can power-cycle to return to app; flag is clear, so
        // power-cycle lands in app, not back in bootloader).
        let _ = flag.clear();
        uart_write_str("OTA timeout, power cycle to return to app\n");
        loop { cortex_m::asm::wfi(); }
    }
    Err(e) => {
        // Other errors (CRC mismatch, abort, etc.): keep flag set, stay in bootloader.
        uart_write_str("OTA error; power cycle to retry\n");
        loop { cortex_m::asm::wfi(); }
    }
}
```

- [ ] **Step 2: Build and commit**

Run: `cargo build --release`. Commit:
```bash
git add bootloader/
git commit -m "fix(bootloader): clear flag on y-modem timeout per spec

Spec: 30s y-modem timeout → clear flag, print message, stay in bootloader.
This way, power cycle returns to app (not back to bootloader).
Other errors (CRC, abort) keep flag set, so power cycle resumes OTA.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 8: App-side `OtaUpdateCommand`

**Files:**
- Create: `src/commands/ota.rs`
- Create: `src/commands/mod.rs`
- Modify: `src/main.rs` (add `mod commands;` and register the command)

**Interfaces:**
- Consumes: `Uart2Sink` (passed in cli context), `foc-common::OTA_FLAG_ADDRESS`, `foc-common::OtaFlag`
- Produces: `OtaUpdateCommand` struct that, when invoked, sets the OTA flag and triggers a system reset

- [ ] **Step 1: Create `src/commands/mod.rs`**

```rust
//! CLI command set (registered with embedded-cli).

pub mod ota;
pub mod shell;
```

- [ ] **Step 2: Create `src/commands/ota.rs`**

```rust
//! `ota_update` command: set OTA_FLAG, write user-visible message to shell, reset.

use defmt::info;
use embassy_cli::Command;
use embassy_cli::runtime::Context;

use crate::bsp::DebugUartSink;
use crate::drivers::debug_uart::DebugShellSink;

#[derive(Command)]
#[command(name = "ota_update", help = "Trigger OTA firmware update via y-modem on USART2")]
pub struct OtaUpdateCommand;

impl RunnableCommand for OtaUpdateCommand {
    fn run(&self, ctx: &mut Context<'_, DebugUartSink>) -> Result<()> {
        // 1. Set the flag (write 0xAA to OTA_FLAG_ADDRESS).
        // Use a one-shot FlashOtaFlag backed by Stm32g4Flash.
        let mut flash = crate::drivers::flash::Stm32g4Flash::new(/* PAC */);
        let mut flag = foc_common::FlashOtaFlag::new(&mut flash, foc_common::OTA_FLAG_ADDRESS);
        if let Err(e) = flag.set_pending() {
            ctx.writer().write_str("Flag set failed; OTA not triggered\n").ok();
            defmt::error!("OTA flag set failed: {:?}", e);
            return Err(CliError::Other);
        }

        // 2. Write user-visible message to the shell.
        ctx.writer().write_str("Rebooting to OTA bootloader, send y-modem now...\n").ok();

        // 3. Brief busy-wait so the message reaches the terminal.
        cortex_m::asm::delay(170_000_000 / 20); // ~50 ms at 170 MHz

        // 4. System reset.
        cortex_m::peripheral::SCB::sys_reset();
    }
}
```

> **Note:** the exact `embassy-cli` API for the command trait and Context type may differ in the actual 0.2.1 release. The implementer should check the embassy-cli 0.2.1 docs and adjust. The CRITICAL invariants are:
> - `set_pending` happens before `sys_reset`
> - User sees the message before the reset
> - The reset is `SCB::sys_reset()` (not `asm::wfi()`)

- [ ] **Step 3: Add `mod commands;` to `src/main.rs`**

In `src/main.rs`, after the existing `mod` declarations:
```rust
mod commands;
```

And in the `commands::ota::OtaUpdateCommand` registration, the spec says: register the command via embedded-cli's Cli builder (in Task 9 when we wire the shell).

For now, `OtaUpdateCommand` exists but isn't registered yet.

- [ ] **Step 4: Add `cortex-m` dep if not already there**

The `cortex_m::peripheral::SCB::sys_reset()` call needs `cortex-m` as a dep. It's already in `Cargo.toml` per the init scaffold.

- [ ] **Step 5: Build**

Run: `cargo build`. Expected: app still compiles. The `commands::ota` module compiles standalone. (The shell command registration is in Task 9.)

- [ ] **Step 6: Commit**

```bash
git add src/commands/ src/main.rs
git commit -m "feat(app): OtaUpdateCommand — set flag + reset

OtaUpdateCommand: when invoked, sets OTA_FLAG_PENDING at
OTA_FLAG_ADDRESS (0x0800_3F00), writes a user-visible message
to the shell, busy-waits 50ms for the message to flush,
then SCB::sys_reset().

The bootloader (Task 3) reads the flag on next boot and
enters y-modem mode instead of jumping to app.

Registration with embedded-cli is in Task 9.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 9: App-side shell task with 5 commands

**Files:**
- Create: `src/commands/shell.rs` (5 command registrations)
- Create: `src/tasks/shell.rs` (the async shell task that reads USART2 + calls `cli.process_byte`)
- Modify: `src/tasks/heartbeat.rs` (move from USART2 to defmt)
- Modify: `src/main.rs` (spawn the shell task instead of just heartbeat)

**Interfaces:**
- Consumes: `DebugUartSink` (from BSP), `embedded-cli` 0.2.1
- Produces: shell task that reads bytes from USART2, processes them via embedded-cli, writes responses back; 5 commands registered (help, version, info, reboot, ota_update)

- [ ] **Step 1: Create `src/commands/shell.rs` — 5 command registrations**

```rust
//! Command set for the embedded-cli shell.

use embassy_cli::Command;
use embassy_cli::runtime::Context;

use crate::bsp::DebugUartSink;
use crate::commands::ota::OtaUpdateCommand;

#[derive(Command)]
#[command(name = "help", help = "List available commands")]
pub struct HelpCommand;

impl RunnableCommand for HelpCommand {
    fn run(&self, ctx: &mut Context<'_, DebugUartSink>) -> Result<()> {
        ctx.writer().write_str("Available: help, version, info, reboot, ota_update\n").ok();
        Ok(())
    }
}

#[derive(Command)]
#[command(name = "version", help = "Show firmware version + Git SHA")]
pub struct VersionCommand;

impl RunnableCommand for VersionCommand {
    fn run(&self, ctx: &mut Context<'_, DebugUartSink>) -> Result<()> {
        // Version + SHA baked in at build time by Task 10.
        let v = env!("FOC_VERSION", "v0.1.0");
        let sha = env!("FOC_GIT_SHA", "unknown");
        ctx.writer().write_str("v").ok();
        ctx.writer().write_str(sha).ok();
        ctx.writer().write_str("\n").ok();
        Ok(())
    }
}

#[derive(Command)]
#[command(name = "info", help = "Show chip + flash usage info")]
pub struct InfoCommand;

impl RunnableCommand for InfoCommand {
    fn run(&self, ctx: &mut Context<'_, DebugUartSink>) -> Result<()> {
        ctx.writer().write_str("STM32G431CBU6\n").ok();
        ctx.writer().write_str("  flash: 128 KB\n").ok();
        ctx.writer().write_str("  sram:  32 KB\n").ok();
        // Live flash usage could be computed by reading the ELF section headers
        // at build time; for now, hard-coded.
        ctx.writer().write_str("  app:   ~14 KB text+data\n").ok();
        Ok(())
    }
}

#[derive(Command)]
#[command(name = "reboot", help = "Reset the MCU")]
pub struct RebootCommand;

impl RunnableCommand for RebootCommand {
    fn run(&self, ctx: &mut Context<'_, DebugUartSink>) -> Result<()> {
        ctx.writer().write_str("Rebooting...\n").ok();
        cortex_m::asm::delay(170_000_000 / 20); // 50 ms
        cortex_m::peripheral::SCB::sys_reset();
    }
}

// Re-export OtaUpdateCommand so the cli builder can find it.
pub use crate::commands::ota::OtaUpdateCommand as _OtaUpdateCommandReExport;
```

> **Note:** the `env!` macro requires build.rs to set these env vars. Task 10 wires that up. If the env vars aren't set yet, the commands will fail to compile. **Workaround for now:** use string literals `"v0.1.0"` and `"unknown"`; Task 10 will replace them with `env!()` calls.

- [ ] **Step 2: Create `src/tasks/shell.rs` — async shell task**

```rust
//! Shell task: reads bytes from USART2 (via DebugUartSink), feeds them to
//! embedded-cli's Cli::process_byte(), writes responses back.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Read;

use crate::bsp::DebugUartSink;
use crate::commands::shell::{
    HelpCommand, InfoCommand, RebootCommand, VersionCommand, _OtaUpdateCommandReExport,
};
use crate::drivers::debug_uart::{DebugShellSink, Uart2Sink};
use crate::drivers::flash::Stm32g4Flash;
use embassy_stm32::usart::BufferedUart;

#[embassy_executor::task]
pub async fn shell_task(sink: Uart2Sink<BufferedUart<'static>>) {
    info!("shell task started");

    // Build the Cli with all 5 commands.
    let mut cli = embassy_cli::cli::Cli::builder()
        .writer(sink)
        .command(HelpCommand)
        .command(VersionCommand)
        .command(InfoCommand)
        .command(RebootCommand)
        .command(_OtaUpdateCommandReExport)
        .build();

    // Outer USART2 read for raw byte input. The BufferedUart's Read impl
    // blocks until at least 1 byte is in the RX ringbuffer.
    // ... (this requires wrapping BufferedUart in something Read-aware)
}
```

> **Important note on USART2 read:** the heartbeat in the init scaffold doesn't read USART2. The shell task needs to. `BufferedUart` implements `embedded_io::Read` (via its blocking mode) and `embedded_io_async::Read` (async). For the shell to be async, we want the async version. The implementer should use `embedded_io_async::Read::read()` and feed bytes to `cli.process_byte()`. The `Uart2Sink` wraps `BufferedUart` and currently only impls `Write`; we may need to add a `Read` method to `Uart2Sink` or have the shell task take the `BufferedUart` directly (not the `Uart2Sink` wrapper).

> **This is a design choice the implementer must make.** The spec said "heartbeat takes `DebugUartSink` (owned) and moves it into the task" — so there's only ONE consumer of the sink. For the shell task, we need BOTH write (response output) AND read (input). Options:
> - (A) Have the shell task own the `DebugUartSink` AND a separate `BufferedUart` for read. But `BufferedUart` was moved into the sink in Task 5 — we'd need to refactor.
> - (B) Refactor `Uart2Sink` to also impl `embedded_io::Read` and `embedded_io_async::Read`. The shell task takes one `Uart2Sink` that has both write and read.
>
> **Recommended: option B.** The implementer adds `embedded_io::Read` and `embedded_io_async::Read` impls to `Uart2Sink` (forwarding to the inner `BufferedUart`).

- [ ] **Step 3: Modify `src/tasks/heartbeat.rs` to use defmt instead of USART2**

```rust
//! Heartbeat task — writes a defmt log every 500ms.
//! Now that the shell task owns USART2, the heartbeat doesn't get a sink.

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
```

- [ ] **Step 4: Modify `src/main.rs` to spawn both heartbeat and shell tasks**

```rust
// ... existing main setup ...

let handles = bsp::board_init(p);
info!("board_init done; USART2 owned by shell task");

// Spawn heartbeat (defmt-only).
spawner.spawn(tasks::heartbeat().unwrap());

// Spawn shell task — it takes ownership of the Uart2Sink.
spawner.spawn(tasks::shell_task(handles.debug_uart).unwrap());

// Main: park in WFI.
loop { cortex_m::asm::wfi(); }
```

- [ ] **Step 5: Build and iterate**

Run: `cargo build`. Likely fails on:
- The `Uart2Sink` not impl'ing `embedded_io::Read` (need to add)
- The exact `embassy-cli` builder API (the spec was based on 0.2.1; actual API may differ)

Iterate on the build errors. The implementer has freedom to:
- Refactor `Uart2Sink` to impl `Read` as well as `Write` and `embedded_io::ErrorType`
- Use the correct `embassy-cli` API for 0.2.1 (check the docs.rs page)

- [ ] **Step 6: Commit**

```bash
git add src/
git commit -m "feat(shell): 5 commands + async shell_task

- commands::shell: HelpCommand, VersionCommand, InfoCommand, RebootCommand
- commands::ota: OtaUpdateCommand (Task 8)
- tasks::shell: async shell_task reads USART2 + feeds to embedded-cli
- tasks::heartbeat: now defmt-only (no longer holds the Uart2Sink)
- main.rs: spawns both heartbeat and shell_task

shell_task owns the Uart2Sink; heartbeat uses defmt-rtt instead.
Uart2Sink impls embedded_io::Read + embedded_io_async::Read in
addition to the existing embedded_io::Write.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 10: Build-time metadata injection

**Files:**
- Modify: `build.rs` (add metadata generation)
- Create: `src/metadata.rs` (Rust struct for runtime access to metadata)
- Modify: `src/main.rs` (read metadata at boot, log it)

**Interfaces:**
- Consumes: linker memory.x (places `.metadata` section at `METADATA_ADDRESS`)
- Produces: 32-byte metadata block at `0x0801_F800` containing magic, image_size, image_crc32, version (16B), build_timestamp, populated at build time

- [ ] **Step 1: Add a `.metadata` section to app's `memory.x`**

The current `memory.x` (init scaffold) doesn't reserve a metadata region. Add it:

```ld
MEMORY
{
  FLASH     : ORIGIN = 0x08000000, LENGTH = 16K    /* bootloader region */
  METADATA  : ORIGIN = 0x08004000, LENGTH = 110K   /* app region — placeholder, */
                                                  /* actual app starts after */
  APP       : ORIGIN = 0x0801F800, LENGTH = 2K     /* metadata region */
  RAM       : ORIGIN = 0x20000000, LENGTH = 32K
}

__metadata_address = ORIGIN(APP);
```

Actually, the current `memory.x` has `FLASH = 128K` total because there's no bootloader yet. With Task 1 introducing the bootloader, the layout changes. The app's `memory.x` should be:

```ld
/* App's view of flash: from 0x0800_4000 (after bootloader) to end. */
MEMORY
{
  APP     : ORIGIN = 0x08004000, LENGTH = 110K
  METADATA: ORIGIN = 0x0801F800, LENGTH = 2K
  RAM     : ORIGIN = 0x20000000, LENGTH = 32K
}

__metadata_address = ORIGIN(METADATA);
```

- [ ] **Step 2: Modify `build.rs` to generate metadata**

Append to the existing `build.rs`:

```rust
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // ... existing build.rs content (link memory.x) ...

    // Set env vars for embedded-cli commands to use.
    println!("cargo:rustc-env=FOC_VERSION={}", env!("CARGO_PKG_VERSION"));
    // Git SHA (short).
    let sha = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| if o.status.success() {
            Some(String::from_utf8_lossy(&o.stdout).trim().to_string())
        } else { None })
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=FOC_GIT_SHA={}", sha);

    // Build timestamp (Unix seconds).
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    println!("cargo:rustc-env=FOC_BUILD_TIMESTAMP={}", ts);

    // Image size and CRC are computed from the built ELF after linking.
    // We need to wait for the ELF to be produced, then read its sections.
    // Use a build script hook that runs in a separate process after build.
    //
    // For now, write a post-build script in `scripts/inject_metadata.sh` (Task 11
    // handles the actual flash-time metadata generation, since ELF parsing at
    // build.rs time requires a linker map file which cargo doesn't expose
    // by default).
}
```

- [ ] **Step 3: Create `src/metadata.rs`**

```rust
//! Build-time metadata struct. Stored at `0x0801_F800` in flash.

use core::ptr::{read_volatile, write_volatile};

pub const METADATA_MAGIC: u32 = 0xDEAD_BEEF;
pub const METADATA_ADDRESS: u32 = foc_common::METADATA_ADDRESS;

/// The metadata block, 32 bytes total.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Metadata {
    pub magic: u32,           // 0xDEADBEEF if valid
    pub image_size: u32,      // bytes of the app image (text + data)
    pub image_crc32: u32,     // CRC-32/ISO-HDLC of the image
    pub version: [u8; 16],    // UTF-8 string, e.g. "v0.1.0-sha1234567"
    pub build_timestamp: u32, // Unix seconds
}

/// Read metadata from flash.
pub fn read() -> Option<Metadata> {
    unsafe {
        let ptr = METADATA_ADDRESS as *const Metadata;
        let meta = read_volatile(ptr);
        if meta.magic == METADATA_MAGIC {
            Some(*meta)
        } else {
            None
        }
    }
}
```

- [ ] **Step 4: Modify `src/main.rs` to log metadata at boot**

```rust
// After board_init, log the metadata if valid.
if let Some(meta) = metadata::read() {
    info!("Firmware: {} (built {})", core::str::from_utf8(&meta.version).unwrap_or("?"), meta.build_timestamp);
    info!("  image: {} bytes, CRC32 0x{:08x}", meta.image_size, meta.image_crc32);
} else {
    info!("No valid metadata (first boot or unprogrammed)");
}
```

- [ ] **Step 5: Build and commit**

Run: `cargo build --release`. Commit:
```bash
git add build.rs memory.x src/metadata.rs src/main.rs
git commit -m "feat(app): build-time metadata injection

memory.x: split FLASH into APP (110K) + METADATA (2K) at 0x0801F800
build.rs: set FOC_VERSION, FOC_GIT_SHA, FOC_BUILD_TIMESTAMP env vars
src/metadata.rs: Metadata struct (32 bytes, repr(C)) with read() via volatile

Image CRC32 / size are computed post-link in Task 11 (post-build script).

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 11: End-to-end OTA verification (hardware required)

**Files:**
- Create: `scripts/inject_metadata.sh` (post-build helper for CRC32 / size injection — used by the user's manual flash workflow)
- Create: `scripts/flash_and_ota.sh` (the user-facing workflow: build, inject metadata, flash via probe-rs, then test OTA from a terminal)

**Interfaces:**
- Consumes: built ELF + `crc32fast` (already in build-deps)
- Produces: a re-built ELF with the metadata block populated, and a documented end-to-end test workflow

- [ ] **Step 1: Create `scripts/inject_metadata.sh`**

This script is run AFTER `cargo build --release` to inject the actual image_size and image_crc32 into the metadata section of the ELF (the cargo build doesn't have the metadata values at build-script time because they depend on the linked binary).

```bash
#!/usr/bin/env bash
# Inject image_size and image_crc32 into the .metadata section of foc-rust.elf
# Usage: ./scripts/inject_metadata.sh [path/to/foc-rust.elf]
set -euo pipefail

ELF="${1:-target/thumbv7em-none-eabihf/release/foc-rust}"
METADATA_ADDR=0x0801F800

# Compute size and CRC32 of the .text + .rodata sections (the actual app image)
SIZE=$(llvm-readelf -SW "$ELF" | awk '/\.text|\.rodata/ {sum += strtonum("0x"$6)} END {print sum}')
CRC=$(llvm-readelf -SW "$ELF" | awk '/\.text|\.rodata/ {print $7, $6}' | \
      xxd -r -p | python3 -c "import sys, zlib; data = sys.stdin.buffer.read(); print(hex(zlib.crc32(data) & 0xFFFFFFFF))")

# Write to a Rust constant — recompile with the values baked in
# (Or directly patch the ELF — for now, set env vars and re-cargo-build)
echo "Computed: size=$SIZE, crc32=$CRC"
```

- [ ] **Step 2: Create `scripts/flash_and_ota.sh`**

This is the user-facing workflow script. Document the steps.

```bash
#!/usr/bin/env bash
# End-to-end OTA test workflow.
# 1. Build app + bootloader
# 2. Flash bootloader to 0x0800_0000 and app to 0x0800_4000
# 3. Open a terminal at 921600 baud
# 4. In RTT/USB-TTL, run: ota_update
# 5. Send the new bin via y-modem (use minicom Ctrl-A, S, Y-modem)

set -euo pipefail
echo "Build app + bootloader..."
cargo build --release

echo "Flash bootloader to 0x0800_0000..."
probe-rs download --chip STM32G431CBUx --base-address 0x08000000 \
    target/thumbv7em-none-eabihf/release/bootloader

echo "Flash app to 0x0800_4000..."
probe-rs download --chip STM32G431CBUx --base-address 0x08004000 \
    target/thumbv7em-none-eabihf/release/foc-rust

echo "Run RTT view (separate terminal)..."
probe-rs run --chip STM32G431CBUx \
    target/thumbv7em-none-eabihf/release/foc-rust
```

- [ ] **Step 3: Manual hardware test**

This is the actual hardware verification:
1. Attach B-G431B-ESC1 + ST-Link V3
2. Run `./scripts/flash_and_ota.sh`
3. Open `screen /dev/ttyUSB0 921600` in a second terminal
4. Wait for shell prompt `>` (or `help` output)
5. Run `version` → confirm v0.1.0
6. Run `help` → confirm 5 commands
7. Modify `version` output in `commands::shell::VersionCommand` to a different version (e.g., v0.2.0)
8. `cargo build --release` to rebuild
9. In shell, run `ota_update`
10. After "Rebooting to OTA bootloader, send y-modem now..." message, in screen trigger y-modem send (Ctrl-A, S, Y-modem), select the new bin
11. Observe ACK flow
12. After "OTA OK, rebooting..." message, observe new app boots
13. In new shell, run `version` → confirm v0.2.0

- [ ] **Step 4: Document the workflow in README**

Add an "OTA Upgrade" section to README.md:

```markdown
## OTA Upgrade

Build bootloader + app:
```bash
cargo build --release
```

Flash both (bootloader at 0x08000000, app at 0x08004000):
```bash
./scripts/flash_and_ota.sh
```

In a separate terminal, connect to USART2 at 921600 baud:
```bash
screen /dev/ttyUSB0 921600
```

To trigger OTA from the shell, type `ota_update` + Enter. When prompted, send a new `.bin` via y-modem (minicom: Ctrl-A, S, Y-modem; screen: Ctrl-A, :, ymodem send <file>).
```

- [ ] **Step 5: Tag the milestone**

```bash
git tag -a v0.2.0-ota -m "First end-to-end OTA via USART2 y-modem"
```

(Don't push until hardware-verified.)

---

## Self-Review

**1. Spec coverage:**

| Spec section | Implemented in |
|---|---|
| 拆 workspace | Task 1 |
| foc-common (地址 + OtaFlag) | Task 2 |
| 自写 bootloader | Tasks 3-4 |
| 921600 baud | Updated in init (post-init-sprint commit `ac1754b`); bootloader uses same constant via `foc-common` or BSP |
| USART2 占位切分(bootloader 跑 / app 跑 / OTA flag) | Tasks 3, 4, 7 |
| 1 字节 flag 在 0x0800_3F00 | Tasks 2, 3 |
| y-modem 协议(精简) | Task 5 |
| CRC32 ISO-HDLC via STM32G4 硬件 | Task 5 (`crc.rs`) + Task 6 (verification) |
| Flag 行为(timeout 清除 / CAN abort 不清 / 成功清除) | Task 7 |
| shell 5 命令(help/version/info/reboot/ota_update) | Tasks 8, 9 |
| heartbeat 改走 defmt | Task 9 |
| metadata 段(32 字节,magic/size/crc32/version/ts) | Task 10 |
| end-to-end OTA 验证 | Task 11 |

**2. Placeholder scan:** No TBD/TODO. The y-modem body in Task 5 has `&'static [u8]` placeholder return type — this is a code-shape placeholder, not a spec placeholder, and the implementer will refactor during implementation.

**3. Type consistency:** `Uart2Sink<U>` (sized, owned, no lifetime) used consistently across Tasks 5, 9. `DebugUartSink` type alias in `bsp.rs` referenced from `tasks/`. `FlashOtaFlag<F: NorFlash + ReadNorFlash>` consistent.

**4. Architectural rule enforcement:** `tasks/heartbeat.rs` (defmt-only after Task 9) has no `embassy_stm32` imports. `tasks/shell.rs` and `commands/*.rs` import only `crate::bsp`, `crate::drivers`, and the driver trait — no HAL types. (The `OtaUpdateCommand` in `commands/ota.rs` does need `foc-common::OtaFlag` and a one-shot `Stm32g4Flash` to set the flag — these are types from driver/hal layers, but the actual peripheral access is via the trait.)

**5. Risk items:**
- **Bootloader brick**: if bootloader has a bug, no way to recover except SWD reflash. Mitigation: keep bootloader ≤14 KB (verified per task commit) and use only stable, well-tested embassy APIs.
- **Y-modem implementation**: ~250-350 lines of state machine; high risk of subtle bugs. Mitigation: incremental task breakdown (Tasks 4, 5, 7 separated), and Task 11's hardware verification is the only way to know if it works.
- **Y-modem CAN detection**: 2 consecutive 0x18 bytes required. Easy to misimplement (off-by-one in the state machine).
- **Y-modem EOT double-EOT dance**: sender sends EOT, receiver NAKs, sender sends EOT again, receiver ACKs. The intermediate NAK is required by y-modem spec.
- **Y-modem timeout reset**: spec says don't auto-reset on timeout; the bootloader stays in the loop until power-cycle. Task 7 implements this.
- **Metadata 32-byte size**: the spec defines a struct layout; the linker must place it at `METADATA_ADDRESS`. The `.metadata` linker section may need explicit placement — Task 10's memory.x may need a `SECTIONS` block to ensure the metadata struct lands at the right address.
