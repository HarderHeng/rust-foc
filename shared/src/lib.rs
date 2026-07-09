#![no_std]

// ── Physical ──
pub const FLASH_BASE: u32   = 0x0800_0000;
pub const FLASH_SIZE: u32   = 128 * 1024;
pub const PAGE_SIZE: u32    = 2048;

// ── Bootloader ──
pub const BOOTLOADER_BASE: u32 = FLASH_BASE;
pub const BOOTLOADER_SIZE: u32 = 0x2000; // 8 KB

// ── Slots ──
pub const SLOT_A_BASE: u32 = FLASH_BASE + BOOTLOADER_SIZE; // 0x0800_2000
pub const SLOT_B_BASE: u32 = 0x0801_0800;
pub const SLOT_SIZE: u32   = 58 * 1024;

// ── Slot Config (last u64 of flash) ──
pub const SLOT_CONFIG_ADDR: u32  = FLASH_BASE + FLASH_SIZE - 8;
pub const SLOT_CONFIG_MAGIC: u32 = 0x424F_4F54;
pub const MAX_BOOT_ATTEMPTS: u8  = 3;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SlotConfig {
    pub magic: u32,
    pub active_slot: u8,
    pub boot_attempts: u8,
    pub flags: u16,
}

impl SlotConfig {
    pub const IWDG_RESET: u16 = 1 << 0;

    pub const fn default_config() -> Self {
        Self { magic: SLOT_CONFIG_MAGIC, active_slot: 0, boot_attempts: 0, flags: 0 }
    }

    pub fn to_u64(self) -> u64 {
        (self.magic as u64)
            | ((self.active_slot as u64) << 32)
            | ((self.boot_attempts as u64) << 40)
            | ((self.flags as u64) << 48)
    }

    pub fn from_u64(raw: u64) -> Self {
        Self {
            magic: raw as u32,
            active_slot: ((raw >> 32) & 0xFF) as u8,
            boot_attempts: ((raw >> 40) & 0xFF) as u8,
            flags: ((raw >> 48) & 0xFFFF) as u16,
        }
    }
}
