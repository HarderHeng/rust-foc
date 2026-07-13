//! C5: per-device SAL key material.
//!
//! The default UDS SAL key masks in `static_config.rs` are
//! plaintext in the ELF — anyone with the source (or a flash
//! dump via SWD) knows the keys and can complete the
//! `0x27 RequestSeed` / `0x27 SendKey` handshake offline.
//!
//! This module replaces those defaults with per-device keys
//! generated from the STM32G4 hardware RNG on first boot, then
//! stored in a small flash region. The 192-byte region (3 ×
//! AES-128 keys) sits inside the existing 2 KB "metadata block"
//! at `0x0801_F800` (between the end of ACTIVE at `0x0801_4000`
//! and the start of DFU's swap-reserve at `0x0801_F800`). The
//! DFU partition is 4 KB larger than ACTIVE on purpose
//! (embassy-boot's swap algorithm needs the headroom), so this
//! region is in DFU's spare area.
//!
//! ## First-boot flow
//!
//!   1. `init()` reads the 192-byte region.
//!   2. If all bytes are 0xFF (factory-erased), generate 3
//!      random AES-128 keys from the RNG and write them.
//!   3. If the region has data, trust it (it was written by
//!      this firmware or by the `0xF180` DID-write path).
//!   4. Return the live keys to the caller.
//!
//! After `init()`, the caller (main) updates
//! `UDS_CONFIG.key_masks` with the live values. The runtime
//! DID 0xF180 write path also routes through `store()` so
//! runtime key rotation persists across reboots.

use crate::drivers::flash;
use embassy_stm32::pac::RNG;
use uds_core::crypto::AesBlock;

/// Address of the key store. Sits inside the existing 2 KB
/// metadata block at 0x0801_F800, which the README
/// documents as "reserved 2 KB metadata block" (the
/// post-OTA size + CRC32 metadata has been moved to the
/// STATE partition since the README was written).
pub const KEY_STORE_ADDR: u32 = 0x0801_F800;

/// 3 × 16-byte AES-128 keys.
pub const KEY_STORE_SIZE: usize = 48;

/// Read the per-device keys. If the region is erased (0xFF),
/// generate fresh keys from the RNG, write them to flash, and
/// return them. Otherwise just return what's there.
///
/// Idempotent: subsequent calls return the same keys (assuming
/// the region was written on first call).
pub fn init() -> [AesBlock; 3] {
    // Read the current contents.
    let mut raw = [0u8; KEY_STORE_SIZE];
    for (i, chunk) in raw.chunks_mut(4).enumerate() {
        let offset = KEY_STORE_ADDR + (i as u32) * 4;
        let word = unsafe { flash::read_u32(offset) };
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    // Detect "all erased" — 0xFF for every byte (factory fresh).
    let is_erased = raw.iter().all(|&b| b == 0xFF);
    if is_erased {
        // First boot on this device. Generate per-device keys.
        defmt::info!("C5: key store erased — generating per-device keys");
        let mut new_keys = [AesBlock([0u8; 16]); 3];
        for key in new_keys.iter_mut() {
            *key = generate_key_from_rng();
        }
        // Write to flash. 48 bytes = 6 × 8-byte u64 writes.
        // Use the existing flash driver. Erase the page first.
        let page = KEY_STORE_ADDR & !2047;
        if unsafe { flash::erase_region(page, page + 2048) }.is_err() {
            defmt::warn!("C5: key store erase failed — using defaults");
            return new_keys;
        }
        for (i, key) in new_keys.iter().enumerate() {
            let base = KEY_STORE_ADDR + (i as u32) * 16;
            // Split the 16-byte key into two 8-byte writes
            // (the flash driver's smallest granularity).
            let lo = u64::from_le_bytes(key.0[..8].try_into().unwrap());
            let hi = u64::from_le_bytes(key.0[8..].try_into().unwrap());
            if unsafe { flash::write_u64(base, KEY_STORE_ADDR, page + 2048, lo) }.is_err() {
                defmt::warn!("C5: key store write {} lo failed", i);
            }
            if unsafe { flash::write_u64(base + 8, KEY_STORE_ADDR, page + 2048, hi) }.is_err() {
                defmt::warn!("C5: key store write {} hi failed", i);
            }
        }
        new_keys
    } else {
        // Existing keys — decode.
        let mut keys = [AesBlock([0u8; 16]); 3];
        for (i, key) in keys.iter_mut().enumerate() {
            key.0.copy_from_slice(&raw[i * 16..(i + 1) * 16]);
        }
        defmt::info!("C5: loaded per-device keys from 0x{:08x}", KEY_STORE_ADDR);
        keys
    }
}

/// Persist a new set of keys to flash. Called by the 0xF180
/// DID-write path when the user rotates keys at runtime.
pub fn store(keys: &[AesBlock; 3]) -> Result<(), flash::FlashError> {
    let page = KEY_STORE_ADDR & !2047;
    unsafe { flash::erase_region(page, page + 2048) }?;
    for (i, key) in keys.iter().enumerate() {
        let base = KEY_STORE_ADDR + (i as u32) * 16;
        let lo = u64::from_le_bytes(key.0[..8].try_into().unwrap());
        let hi = u64::from_le_bytes(key.0[8..].try_into().unwrap());
        unsafe { flash::write_u64(base, KEY_STORE_ADDR, page + 2048, lo) }?;
        unsafe { flash::write_u64(base + 8, KEY_STORE_ADDR, page + 2048, hi) }?;
    }
    Ok(())
}

/// Generate one AES-128 key from the STM32G4 hardware RNG.
fn generate_key_from_rng() -> AesBlock {
    let mut key = [0u8; 16];
    for chunk in key.chunks_mut(4) {
        let mut timeout = 10_000u32;
        while !RNG.sr().read().drdy() {
            timeout -= 1;
            if timeout == 0 {
                // RNG didn't respond. Fall back to zeros — the
                // caller will detect the zero key via the
                // DID 0xF180 "all-zero rejected" check on the
                // next write, and the device will be locked out
                // of OTA until a working RNG cycle.
                defmt::warn!("C5: RNG timeout during key generation");
                return AesBlock([0u8; 16]);
            }
            core::hint::spin_loop();
        }
        let val = RNG.dr().read();
        chunk.copy_from_slice(&val.to_le_bytes());
    }
    AesBlock(key)
}
