//! Build-time metadata struct in flash.
//!
//! Last 2 KB of flash is reserved for this. Read at boot by
//! `main::main` to log the firmware version and image size /
//! CRC. Phase 4 (OTA via UDS) writes this region with the
//! post-OTA image's metadata.

use core::ptr::read_volatile;

/// Address of the metadata block. 2 KB at the top of flash, kept
/// out-of-band of the application image so a corrupt / in-progress
/// OTA never touches it. Re-exported from `crate::drivers::flash`
/// (the canonical writer-side home) under the historical name
/// `METADATA_ADDRESS` for backward compatibility with existing
/// readers.
pub use crate::drivers::flash::METADATA_ADDR as METADATA_ADDRESS;

/// Magic number that distinguishes a valid `Metadata` block from
/// erased flash (which reads as all-1s). Re-exported from
/// `crate::drivers::flash` (the canonical writer-side home).
pub use crate::drivers::flash::METADATA_MAGIC;

/// Layout of the metadata block, byte-for-byte. The struct fields
/// are the same as the wire layout (LE) so a single u32 / [u8; N]
/// `read_volatile` per field is naturally aligned. `build_timestamp`
/// sits between `image_crc32` and `version` so the two `u64` reads
/// for `version` land on 8-byte boundaries — a previous version
/// packed `version` right after `image_crc32`, putting the u64
/// reads at offsets 12 and 20 (not 8-aligned), which would hard-
/// fault on Cortex-M targets with `UNALIGN_TRP = 1`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Metadata {
    pub magic: u32,
    pub image_size: u32,
    pub image_crc32: u32,
    pub build_timestamp: u32,
    /// UTF-8 null-padded version string (e.g. `"0.1.0\0\0\0\0\0\0\0\0\0\0"`).
    pub version: [u8; 16],
}

/// Read the metadata block from flash.
///
/// Each field is read with its own `read_volatile` so the compiler
/// emits aligned scalar loads. A `read_volatile(*const Metadata)`
/// would copy the whole 32-byte struct via a sequence of
/// unaligned 32-bit loads — on Cortex-M4 that raises a hard fault
/// at `build_timestamp` (offset 12 is aligned, but the compiler
/// is allowed to emit non-aligned loads for the intermediate
/// copies). Reading field-by-field is implementation-defined-safe.
///
/// Wire layout (LE, must match `src/ota/flash.rs::write_metadata`):
///
///   0x00: magic           (u32)
///   0x04: image_size      (u32)
///   0x08: image_crc32     (u32)
///   0x0C: build_timestamp (u32)
///   0x10: version[0..8]   (u64) — 8-aligned
///   0x18: version[8..16]  (u64) — 8-aligned
pub fn read() -> Option<Metadata> {
    unsafe {
        // Each field is a scalar, so the compiler emits aligned reads.
        let magic = read_volatile((METADATA_ADDRESS + 0) as *const u32);
        if magic != METADATA_MAGIC {
            return None;
        }
        let image_size = read_volatile((METADATA_ADDRESS + 4) as *const u32);
        let image_crc32 = read_volatile((METADATA_ADDRESS + 8) as *const u32);
        let build_timestamp = read_volatile((METADATA_ADDRESS + 12) as *const u32);
        // The 16-byte version field is laid out across two u64s in
        // flash (little-endian). Both u64 reads land on 8-byte
        // boundaries (`build_timestamp` sits between the u32s and
        // the version block to make this happen). Read them
        // separately and byte-swap back into the struct's [u8; 16]
        // layout.
        let version_lo = read_volatile((METADATA_ADDRESS + 16) as *const u64);
        let version_hi = read_volatile((METADATA_ADDRESS + 24) as *const u64);
        let mut version = [0u8; 16];
        version[..8].copy_from_slice(&version_lo.to_le_bytes());
        version[8..].copy_from_slice(&version_hi.to_le_bytes());

        // Reject "fully-erased" or "all-zero" version/timestamp
        // blocks: a fresh chip with no OTA yet writes these as
        // 0xFF (erased flash) or 0x00 (unprogrammed); either
        // way the user-visible fields would be misleading.
        // "is the version empty" = "every byte is 0 or 0xFF".
        let version_meaningful = version.iter().any(|&b| b != 0x00 && b != 0xFF);
        if !version_meaningful || build_timestamp == 0xFFFF_FFFF {
            return None;
        }

        Some(Metadata { magic, image_size, image_crc32, build_timestamp, version })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metadata_address_is_8_aligned() {
        assert_eq!(METADATA_ADDRESS % 8, 0);
    }

    #[test]
    fn version_u64_reads_are_8_aligned() {
        // Mirror the offsets used in `read()`. The two u64 reads
        // for `version` MUST land on 8-byte boundaries — a
        // previous layout packed `version` right after the u32s
        // and emitted the reads at offsets 12 and 20, which is
        // not 8-aligned and would hard-fault on Cortex-M targets
        // with `UNALIGN_TRP = 1`.
        let version_lo_off = 16u32;
        let version_hi_off = 24u32;
        assert_eq!((METADATA_ADDRESS + version_lo_off) % 8, 0);
        assert_eq!((METADATA_ADDRESS + version_hi_off) % 8, 0);
    }
}
