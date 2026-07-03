//! SecurityAccess (0x27) key derivation — pure crypto, no state.
//!
//! Algorithm: 40-iter LFSR + byte bit-reversal + byte reorder.
//! Per-SAL mask lives in `UdsConfig::key_masks[sal - 1]`. Same
//! algorithm as MiniUds, so interop with MiniUds-based tools is
//! free.
//!
//! This module holds only pure functions; the SAL state machine
//! is in `table.rs::UdsConfig::dispatch_0x27`.

/// 8-bit bit reversal. Used by `generate_key` to scramble the
/// LFSR output bytes before assembly.
pub fn reverse_bits(mut b: u8) -> u8 {
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
    b
}

/// Derive a 4-byte key from a 4-byte seed and a 4-byte LFSR mask.
/// LFSR runs 40 iterations, then the 4 state bytes are
/// bit-reversed and reassembled in reverse order.
///
/// Per ISO 14229, the server may use any algorithm; this is the
/// one MiniUds uses.
pub fn generate_key(seed: u32, mask: u32) -> u32 {
    let mut state = seed;
    for _ in 0..40 {
        if state & 0x8000_0000 != 0 {
            state = (state << 1) ^ mask;
        } else {
            state <<= 1;
        }
    }
    let mut key = 0u32;
    for i in 0..4 {
        let byte = reverse_bits(((state >> ((3 - i) * 8)) & 0xFF) as u8) as u32;
        key |= byte << (i * 8);
    }
    key
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lfsr_known_vector() {
        // seed=0xA5A5_A5A5, mask=0x3000_2212. Spot-check:
        // the function is deterministic and the chosen seed
        // produces a non-zero key (regression guard against
        // "all-zero key bug").
        let key = generate_key(0xA5A5_A5A5, 0x3000_2212);
        let key2 = generate_key(0xA5A5_A5A5, 0x3000_2212);
        assert_eq!(key, key2);
        assert_ne!(key, 0);
    }

    #[test]
    fn lfsr_different_seeds_differ() {
        let k1 = generate_key(0x0000_0001, 0x1234_5678);
        let k2 = generate_key(0x0000_0002, 0x1234_5678);
        // 32-bit LFSR: collision probability 2^-32 for any
        // specific pair.
        assert_ne!(k1, k2);
    }

    #[test]
    fn lfsr_different_masks_differ() {
        let k1 = generate_key(0xA5A5_A5A5, 0x0000_0001);
        let k2 = generate_key(0xA5A5_A5A5, 0x0000_0002);
        assert_ne!(k1, k2);
    }

    #[test]
    fn reverse_bits_round_trip() {
        for b in 0u8..=255 {
            assert_eq!(reverse_bits(reverse_bits(b)), b,
                       "reverse_bits is its own inverse for 0x{:02x}", b);
        }
    }
}
