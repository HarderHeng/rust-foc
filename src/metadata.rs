//! Per-slot metadata — last 32 bytes of each app slot.

use core::ptr::read_volatile;
use foc_shared::{SLOT_A_BASE, SLOT_B_BASE, SLOT_SIZE};

pub use crate::drivers::flash::METADATA_MAGIC;

const META_OFFSET: u32 = SLOT_SIZE - crate::drivers::flash::METADATA_SIZE;

#[repr(C)]
#[derive(defmt::Format, Clone, Copy)]
pub struct Metadata {
    pub magic: u32,
    pub image_size: u32,
    pub image_crc32: u32,
    pub build_timestamp: u32,
    pub version: [u8; 16],
}

fn slot_base() -> u32 {
    if crate::drivers::flash::current_slot() == 0 { SLOT_A_BASE } else { SLOT_B_BASE }
}

pub fn read() -> Option<Metadata> {
    let addr = slot_base() + META_OFFSET;
    unsafe {
        let magic = read_volatile(addr as *const u32);
        if magic != METADATA_MAGIC { return None; }
        let image_size = read_volatile((addr + 4) as *const u32);
        let image_crc32 = read_volatile((addr + 8) as *const u32);
        let build_timestamp = read_volatile((addr + 12) as *const u32);
        let version_lo = read_volatile((addr + 16) as *const u64);
        let version_hi = read_volatile((addr + 24) as *const u64);
        let mut version = [0u8; 16];
        version[..8].copy_from_slice(&version_lo.to_le_bytes());
        version[8..].copy_from_slice(&version_hi.to_le_bytes());
        let meaningful = version.iter().any(|&b| b != 0 && b != 0xFF);
        if !meaningful || build_timestamp == 0xFFFF_FFFF { return None; }
        Some(Metadata { magic, image_size, image_crc32, build_timestamp, version })
    }
}
