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
