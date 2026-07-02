//! Build-time metadata struct in flash.
//!
//! Last 2 KB of flash is reserved for this. Read at boot by
//! `main::main` to log the firmware version and image size /
//! CRC. Phase 4 (OTA via UDS) will also write this region with
//! the post-OTA image's metadata.

use core::ptr::read_volatile;

/// Address of the metadata block. 2 KB at the top of flash, kept
/// out-of-band of the application image so a corrupt / in-progress
/// OTA never touches it.
pub const METADATA_ADDRESS: u32 = 0x0801_F800;

/// Magic number that distinguishes a valid `Metadata` block from
/// erased flash (which reads as all-1s).
pub const METADATA_MAGIC: u32 = 0xF0C1_001A;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Metadata {
    pub magic: u32,
    pub image_size: u32,
    pub image_crc32: u32,
    pub version: [u8; 16],
    pub build_timestamp: u32,
}

pub fn read() -> Option<Metadata> {
    unsafe {
        let ptr = METADATA_ADDRESS as *const Metadata;
        let meta = read_volatile(ptr);
        if meta.magic == METADATA_MAGIC { Some(meta) } else { None }
    }
}
