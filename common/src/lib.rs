#![no_std]

// Reserved for shared, hardware-independent helpers across the
// foc-rust workspace. Currently empty: the previous `flash`,
// `flag`, and `addresses` modules were removed when the y-modem
// bootloader was deleted (see
// `docs/superpowers/specs/2026-07-02-can-ota-uds-design.md`).
// Phase 4 (OTA via UDS) may add a metadata layout helper here
// if it ends up shared between the app and a future bootloader
// stub.
