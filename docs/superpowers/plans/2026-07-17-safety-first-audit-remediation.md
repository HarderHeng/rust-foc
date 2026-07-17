# Safety-First Module Audit and Remediation Plan

> **For agentic workers:** Execute each task with a failing regression test where the behavior is host-testable. Do not commit changes unless explicitly requested.

**Goal:** Audit every production Rust module, repair confirmed correctness/safety defects with minimal changes, and leave an evidence-backed list of deferred hardware-only risks.

**Architecture:** Preserve the existing Embassy application, `foc-algo`, `uds-core`, and bootloader boundaries. Fix protocol/state/partition invariants at their owning layer; do not redesign closed-loop FOC, ISO-TP, or image authentication. Use host tests for pure protocol logic and target builds/link inspection for embedded code.

**Tech Stack:** Rust 2021, Embassy STM32G4, `thumbv7em-none-eabihf`, `uds-core`/`foc-algo` host tests, Python vCAN smoke harness.

## Global Constraints

- Safety-first, minimal necessary behavioral changes.
- No Git commits, rebases, pushes, or history rewrites.
- No `as any`/type suppression equivalent, no deleting failing tests.
- Do not implement closed-loop FOC, Ed25519 image signatures, or ISO-TP.
- Treat the bootloader plus `bootloader/build.rs`, `app-layout.x`, `app-memory.x`, and OTA constants as one partition contract.
- Canonical partition contract to verify and then encode: bootloader `0x0800_0000..0x0800_6000`, state `0x0800_6000..0x0800_7000`, unused gap `0x0800_7000..0x0800_9000`, ACTIVE `0x0800_9000..0x0801_4000` (44 KiB), DFU `0x0801_4000..0x0802_0000` (48 KiB), key store in the unused gap on a 2 KiB-aligned page.
- The key store must not be inside the DFU erase or bootloader swap range.

---

## Task 1: Establish the partition contract and add regression guards

**Files:**
- Modify: `app-memory.x`
- Modify: `app-layout.x` only if needed to declare the same regions
- Modify: `build.rs` only to copy the canonical `app-memory.x`
- Modify: `src/ota/mod.rs`
- Modify: `src/key_store.rs`
- Test: host-testable partition helper/tests in the owning modules or a pure test module

**Required behavior:**
- Correct `app-memory.x` from the stale 40 KiB/`0x0801_3000` layout to ACTIVE origin `0x0800_9000`, length `44K`, and DFU origin `0x0801_4000`, length `48K`.
- Keep bootloader symbols and OTA `DFU_START/DFU_END` consistent.
- Move the key store to a 2 KiB-aligned page in the unused `0x0800_7000..0x0800_9000` gap, preferably `0x0800_7000`; reserve/document that page so future layout edits cannot reuse it.
- Reject OTA image sizes above ACTIVE capacity, not merely DFU capacity.
- Do not move the key store to `0x0801_F000`; that address remains inside the canonical DFU range.

**Tests first:** assert ACTIVE/DFU boundaries, key-store page is outside both DFU and bootloader state, and sizes `43 KiB` accepted / `45 KiB` rejected.

**Verification:** `cargo test -p uds-core`, target debug/release builds, and ELF/linker symbol inspection for `.text`/ACTIVE end.

## Task 2: Make raw Flash operations atomic without silently freezing the system

**Files:** `src/drivers/flash.rs`, OTA call sites in `src/ota/mod.rs`.

**Required behavior:**
- Protect each 64-bit program sequence (`unlock → PG → write → BSY/error check → clear PG → lock`) with a PRIMASK critical section.
- Protect page erase controller register transitions and each start/check sequence as required by the STM32G4 reference manual.
- Do not claim that a critical section makes multi-page erase asynchronous. Document that OTA must have motor outputs disabled and that long erase latency is a system-level constraint. If source inspection proves the existing caller cannot safely erase while the motor is enabled, add the smallest safe gate; otherwise record the hardware-only risk rather than inventing an async flash driver.
- Preserve public unsafe signatures unless an owning caller requires a typed error change.

**Verification:** target build, diagnostics, and code review of all Flash callers; no host test may pretend to validate STM32 Flash timing.

## Task 3: Repair UDS pending response visibility

**Files:** `uds-core/src/pending.rs`, tests in the same module.

**Required behavior:** every emitted `0x78 ResponsePending` and terminal pending error sets the flag consumed by `src/uds/mod.rs::take_response_pending`, while preserving response-buffer ownership.

**Tests first:** P2 timeout exposes `[0x7F, sid, 0x78]`; P2* exposes the terminal NRC; completed pending work exposes its response exactly once.

**Verification:** `cargo test -p uds-core` and caller trace through `src/drivers/can/canopen.rs`.

## Task 4: Isolate pending request data and reject concurrent requests

**Files:** `uds-core/src/pending.rs`, `uds-core/src/state.rs`, `uds-core/src/table.rs`, OTA closures in `src/uds/static_config.rs`.

**Required behavior:**
- Store a fixed-size request snapshot (`[u8; 64]` plus length) in each pending job at enqueue time.
- Expose the snapshot through `UdsContext`; pending handlers must not read the shared last-request buffer.
- At dispatch entry, if the service state is `Pending`, reject the incoming request with `0x21 BusyRepeatRequest` using that request's SID.
- Keep the request buffer for synchronous dispatch compatibility.

**Tests first:** a queued job sees its original request after the shared buffer changes; a second request during Pending gets `0x21`; a new request after completion is accepted.

**Verification:** `cargo test -p uds-core`; inspect all `push_pending` callers and closure signatures.

## Task 5: Correct UDS request validation and functional addressing

**Files:** `uds-core/src/table.rs`, `src/drivers/can/uds_bridge.rs`, tests beside the pure helpers.

**Required behavior:**
- `0x34` wrong length returns `0x13`; correct length with unsupported format returns `0x31`.
- Apply the repository’s chosen ISO-14229 functional-addressing policy consistently: block state-changing/security/programming SIDs (`0x10` programming/extended subfunctions, `0x11`, `0x27`, `0x28`, `0x2E`, `0x31`, `0x34`, `0x36`, `0x37`) on functional requests; do not silently trigger reset, key writes, routines, or OTA from broadcast.
- Preserve the existing physical response ID and explicitly test positive-response suppression rules for functional requests, including `TesterPresent` suppress-bit behavior as supported by the current transport contract. Do not change response routing merely because the helper name is generic; verify against current smoke expectations and caller semantics first.
- Reorder NRC checks only if source/tests/spec evidence shows security precedence is required by this implementation; add a regression test if changed.

**Tests first:** wrong length/format, blocked functional SIDs, allowed read/diagnostic response, physical request response ID, and the selected suppression behavior.

## Task 6: Harden key-store persistence and runtime key rotation

**Files:** `src/key_store.rs`, `src/uds/static_config.rs`.

**Required behavior:**
- Read back every programmed double-word or the complete record after writing; return a programming error on mismatch.
- Keep RAM key masks unchanged if Flash persistence fails; only publish the new RAM masks after successful persistence and verification.
- Add an explicit validity rule that rejects erased/partial records rather than treating any non-all-`0xFF` bytes as valid. Do not claim power-loss atomicity without a record marker/version/CRC design; if implementing that is larger than a minimal patch, document it as a remaining high-risk item.

**Tests:** pure record-validation tests for erased, complete, and partial data; target build for the real Flash path.

## Task 7: Prevent cooperative Shell writes from starving real-time tasks

**Files:** `src/shell/task.rs`, `src/shell/commands.rs`.

**Required behavior:** use the actual `embedded_io_async::Write` implementation available for this Embassy UART and await all potentially blocking TX operations. Keep command parsing and wire output unchanged. If the target type does not implement async Write, stop and document the limitation rather than changing executor mode or inventing a buffering architecture.

**Tests:** parser and numeric-output host tests; target build; retain a hardware-only note for motor jitter/heartbeat timing because host tests cannot prove scheduler latency.

## Task 8: Full module audit and deferred findings

**Review units:** all `src/`, `foc-algo/src/`, `uds-core/src/`, `bootloader/src/`, and build/link files.

**Required output:** an audit report grouped by module with severity, exact file/symbol, evidence, whether fixed, and deferred reason. Explicitly cover FOC numerical boundaries, PWM safety gates, CANopen task timing, RNG/error paths, panic/reset paths, and dead/stale metadata code. Do not modify FOC closed-loop behavior; only add tests or comments if a confirmed issue is directly in scope.

## Task 9: Synchronize documentation only after code/layout is verified

**Files:** `README.md`, `src/ota/mod.rs` docs, `src/key_store.rs` docs, relevant UDS design text.

**Required behavior:** describe the actual bootloader/state/ACTIVE/DFU/key-store layout; remove false RAM-residency claims unless an actual linker section is implemented; correct file paths and OTA model. Do not delete `src/metadata.rs` or `app-memory.x` until reference searches and build evidence prove they are dead and removal is safe.

## Task 10: Final verification and independent review

**Commands:**
- `cargo fmt --all -- --check`
- `cargo test -p foc-algo --features libm-trig`
- `cargo test -p uds-core`
- `cargo build --target thumbv7em-none-eabihf --release`
- `cargo build -p bootloader --target thumbv7em-none-eabihf --release`
- `cargo clippy -p foc-algo -p uds-core --all-targets -- -D warnings` (record pre-existing target-specific failures separately)
- `python3 scripts/smoke_test.py` when the harness is available
- ELF/link inspection of ACTIVE/DFU bounds and target diagnostics for changed files

Before claiming completion, review the final diff against the task’s original scope and list any hardware-only risks that remain unverified.
