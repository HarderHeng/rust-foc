//! Build-time metadata struct in flash at `0x0801_F800`.

use core::ptr::read_volatile;

pub const METADATA_MAGIC: u32 = foc_common::METADATA_MAGIC;
pub const METADATA_ADDRESS: u32 = foc_common::METADATA_ADDRESS;

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
