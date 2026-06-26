//! STM32G4 hardware CRC-32 (ISO-HDLC).
//!
//! Poly 0x04C1_1DB7, init 0xFFFF_FFFF, input BYTE-rev, output bit-rev,
//! final XOR 0xFFFF_FFFF — standard CRC-32/ISO-HDLC.

use embassy_stm32::pac;

fn pac_crc() -> pac::crc::Crc { pac::CRC }

/// Initialise CRC peripheral. Call once before feeding data.
pub fn crc32_init() {
    let crc = pac_crc();
    crc.cr().modify(|w| {
        w.set_polysize(pac::crc::vals::Polysize::POLYSIZE32);
        w.set_rev_in(pac::crc::vals::RevIn::BYTE);
        w.set_rev_out(pac::crc::vals::RevOut::REVERSED);
    });
    crc.init().write_value(0xFFFF_FFFF);
    crc.cr().modify(|w| w.set_reset(true));
}

/// Feed data into the CRC engine. Can be called multiple times.
pub fn crc32_update(data: &[u8]) {
    let crc = pac_crc();
    for &byte in data { crc.dr8().write_value(byte); }
}

/// Finalise and return CRC-32/ISO-HDLC.
pub fn crc32_finalize() -> u32 {
    pac_crc().dr32().read() ^ 0xFFFF_FFFF
}
