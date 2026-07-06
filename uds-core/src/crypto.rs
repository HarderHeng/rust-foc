//! SecurityAccess (0x27) key derivation — AES-128-ECB.
//!
//! Algorithm: AES-128 ECB encrypt(seed, key_material) → key.
//!   seed: 16 random bytes (generated from timing jitter in table.rs)
//!   key_material: 16-byte per-SAL mask (stored in UdsConfig::key_masks,
//!                  writable at runtime via DID 0xF180)
//!   result: 16-byte ciphertext = derived key
//!
//! This module holds only pure functions; the SAL state machine
//! and seed generation are in `table.rs`.

use aes::cipher::{generic_array::GenericArray, BlockEncrypt, KeyInit};
use aes::Aes128;
use core::fmt;

/// AES-128 key / seed / derived key size (16 bytes).
pub const AES_BLOCK_SIZE: usize = 16;

/// 16-byte value used as either seed or key material.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct AesBlock(pub [u8; AES_BLOCK_SIZE]);

#[cfg(feature = "defmt")]
impl defmt::Format for AesBlock {
    fn format(&self, f: defmt::Formatter) {
        for byte in &self.0 {
            defmt::write!(f, "{:02x}", byte);
        }
    }
}

#[allow(dead_code)]
impl AesBlock {
    pub const fn from_bytes(b: [u8; AES_BLOCK_SIZE]) -> Self { Self(b) }

    /// Read as a byte slice.
    pub fn as_bytes(&self) -> &[u8] { &self.0 }

    /// Write the block to a mutable slice starting at `offset`.
    /// Panics if the slice is too short.
    pub fn write_into(&self, out: &mut [u8], offset: usize) {
        out[offset..offset + AES_BLOCK_SIZE].copy_from_slice(&self.0);
    }
}

impl From<[u8; AES_BLOCK_SIZE]> for AesBlock {
    fn from(b: [u8; AES_BLOCK_SIZE]) -> Self { Self(b) }
}

impl fmt::Debug for AesBlock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in &self.0 {
            write!(f, "{:02x}", byte)?;
        }
        Ok(())
    }
}

/// Derive a 16-byte key from a 16-byte seed and 16-byte key material.
/// Uses AES-128-ECB: `key = AES_encrypt_ecb(seed, key_material)`.
pub fn generate_key(seed: &AesBlock, key_material: &AesBlock) -> AesBlock {
    let key = GenericArray::from_slice(&key_material.0);
    let cipher = Aes128::new(key);
    let mut block = GenericArray::clone_from_slice(&seed.0);
    cipher.encrypt_block(&mut block);
    AesBlock(block.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// NIST AES-128 ECB known-answer test (single block).
    /// Key: 000102030405060708090a0b0c0d0e0f
    /// Plaintext: 00112233445566778899aabbccddeeff
    /// Ciphertext: 69c4e0d86a7b0430d8cdb78070b4c55a
    #[test]
    fn aes_kat() {
        let key = AesBlock::from_bytes([
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        ]);
        let pt = AesBlock::from_bytes([
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
            0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
        ]);
        let ct = generate_key(&pt, &key);
        let expected: [u8; 16] = [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
            0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
        ];
        assert_eq!(ct.0, expected);
    }

    #[test]
    fn aes_deterministic() {
        let key = AesBlock::from_bytes([0x00; 16]);
        let seed = AesBlock::from_bytes([
            0xa5, 0xa5, 0xa5, 0xa5, 0xb0, 0xb1, 0xb2, 0xb3,
            0xc0, 0xc1, 0xc2, 0xc3, 0xd0, 0xd1, 0xd2, 0xd3,
        ]);
        let k1 = generate_key(&seed, &key);
        let k2 = generate_key(&seed, &key);
        assert_eq!(k1.0, k2.0);
    }

    #[test]
    fn aes_different_keys_differ() {
        let seed = AesBlock::from_bytes([0x00; 16]);
        let ka = AesBlock::from_bytes([0x01; 16]);
        let kb = AesBlock::from_bytes([0x02; 16]);
        assert_ne!(generate_key(&seed, &ka).0, generate_key(&seed, &kb).0);
    }
}
