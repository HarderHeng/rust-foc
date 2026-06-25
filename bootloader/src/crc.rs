//! STM32G4 hardware CRC-32 (ISO-HDLC / PKZIP) driver.
//!
//! Configuration (CRC-32/ISO-HDLC):
//! - Polynomial: 0x04C1_1DB7 (default STM32G4, standard CRC-32)
//! - Init value: 0xFFFF_FFFF
//! - Input reversal: BYTE (each byte bit-reversed before shifting)
//! - Output reversal: REVERSED (32-bit result bit-reversed)
//! - Final XOR: 0xFFFF_FFFF (applied in software)
//!
//! Usage:
//! ```ignore
//! crc32_init();
//! crc32_update(data);  // call multiple times
//! let checksum = crc32_finalize();
//! ```

#![allow(dead_code)]

use embassy_stm32::pac;

/// Reference to the CRC peripheral (pub const in the PAC).
fn pac_crc() -> pac::crc::Crc {
    pac::CRC
}

/// Initialise the CRC peripheral for CRC-32/ISO-HDLC.
///
/// Call once before any `crc32_update()` calls.
pub fn crc32_init() {
    let crc = pac_crc();

    // Configure CRC for CRC-32/ISO-HDLC:
    //   POLYSIZE = 32-bit
    //   REV_IN = BYTE  (bit-reverse each input byte)
    //   REV_OUT = REVERSED
    //
    // Default INIT (0xFFFF_FFFF) and POL (0x04C1_1DB7) are already correct,
    // but we set INIT explicitly for clarity.
    crc.cr().modify(|w| {
        w.set_polysize(pac::crc::vals::Polysize::POLYSIZE32);
        w.set_rev_in(pac::crc::vals::RevIn::BYTE);
        w.set_rev_out(pac::crc::vals::RevOut::REVERSED);
    });

    // Set INIT register.
    crc.init().write_value(0xFFFF_FFFF);

    // Reset the CRC unit: loads INIT, ready for data.
    crc.cr().modify(|w| w.set_reset(true));
}

/// Feed a slice of bytes into the CRC engine.
///
/// Can be called multiple times to do a streaming CRC.
pub fn crc32_update(data: &[u8]) {
    let crc = pac_crc();
    for &byte in data {
        // Writing 8-bit data to the data register auto-sizes to 8 bits.
        crc.dr8().write_value(byte);
    }
}

/// Finalise the CRC and return the CRC-32/ISO-HDLC checksum.
///
/// The STM32G4 applies output bit-reversal (REV_OUT=REVERSED) automatically.
/// We still need the final XOR with 0xFFFF_FFFF per the CRC-32/ISO-HDLC spec.
pub fn crc32_finalize() -> u32 {
    let crc = pac_crc();
    let result = crc.dr32().read();
    result ^ 0xFFFF_FFFF
}
