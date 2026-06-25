//! Build-time metadata struct. Stored at `0x0801_F800` in flash.
//!
//! The full metadata block (magic, image_size, image_crc32, version, build
//! timestamp) is 32 bytes.  The image size and CRC-32 are computed
//! post-link (Task 11); until then they are left as zero-placeholders.
//! The version string and build timestamp are baked at compile time via
//! env vars set by `build.rs`.

use core::ptr::read_volatile;

pub const METADATA_MAGIC: u32 = foc_common::METADATA_MAGIC;
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
///
/// Returns `Some(Metadata)` when the magic is valid, `None` otherwise
/// (first boot or unprogrammed).
pub fn read() -> Option<Metadata> {
    unsafe {
        let ptr = METADATA_ADDRESS as *const Metadata;
        let meta = read_volatile(ptr);
        if meta.magic == METADATA_MAGIC {
            Some(meta)
        } else {
            None
        }
    }
}
