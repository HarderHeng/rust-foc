//! In-app OTA via UDS TransferData (Phase 4 v1).
//!
//! Wire protocol (all under the SDO 0x2F00.0 gateway):
//!
//!   0x34 RequestDownload     [0x34, 0x00, size_lo, size_hi, size_hi2, size_hi3]
//!   0x34 positive response   [0x74, 0x00, 0x00, 0x02]
//!
//!   0x36 TransferData        [0x36, block_seq, b0, b1]     (2 bytes/block)
//!   0x36 positive response   [0x76, block_seq]
//!
//!   0x37 RequestTransferExit [0x37]
//!   0x37 positive response   [0x77]
//!
//! Phase 4 v1 simplifications (all documented in the spec at
//! `docs/superpowers/specs/2026-07-02-can-ota-uds-design.md`):
//!
//! - 2 bytes per TransferData call. For a 110 KB image that's
//!   ~55000 SDO round-trips at 500 kbps CAN, ~8 seconds total.
//!   Bumping to 6 bytes per call needs segmented SDO; deferred.
//! - No segmented transfer for the request payload — the
//!   RequestDownload fits in 7 bytes (SF) and TransferData
//!   transfers 2 bytes per call. Segmented SDO + UDS will land
//!   in a future revision.
//! - Block sequence counter is echoed but not checked. Per UDS,
//!   a wrong seq should return 0x73 wrongBlockSequenceNumber;
//!   v1 logs and ignores.
//! - The image is written **in place** at 0x0800_0000; the
//!   current UDS / canopen code is running from the same flash
//!   during the download. Standard STM32 flash semantics make
//!   this safe as long as the write pointer doesn't cross the
//!   current PC. We document the risk; a Phase 4+ v2 would
//!   relocate the OTA state machine to RAM.
//! - Single-bank flash layout: 0x0800_0000–0x0801_F7FF is the
//!   app region (124 KB). 0x0801_F800–0x0801_FFFF is the
//!   reserved 2 KB metadata block. The 0x37 handler writes the
//!   post-OTA image size + CRC32 to the metadata block, then
//!   triggers NVIC reset. On the next boot the metadata is
//!   already valid; no special "first-boot" branch needed.

use crate::drivers::flash;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use critical_section::Mutex;
use defmt::{info, warn};

// DFU partition (embassy-boot swap model)
const DFU_START:  u32 = 0x0801_3000;
const DFU_END:    u32 = 0x0801_F800; // 50 KB
const STATE_ADDR: u32 = 0x0800_6000;
const SWAP_MAGIC: u8  = 0xF0;


// ---- UDS service IDs (subset) -----------------------------------------

pub const SID_REQUEST_DOWNLOAD: u8 = 0x34;
pub const SID_TRANSFER_DATA: u8 = 0x36;
pub const SID_REQUEST_TRANSFER_EXIT: u8 = 0x37;

// ---- State ------------------------------------------------------------

/// State machine for the OTA transfer.
#[repr(u8)]
#[derive(defmt::Format, Copy, Clone, PartialEq, Eq)]
pub enum OtaState {
    Idle = 0,
    Receiving = 1,
    Done = 2,
}

impl OtaState {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Receiving,
            2 => Self::Done,
            _ => Self::Idle,
        }
    }
}

static OTA_STATE: AtomicU8 = AtomicU8::new(0);

/// Bytes still expected before the next TransferExit. Set at
/// RequestDownload; decremented by 2 per TransferData call.
static OTA_REMAINING: AtomicU32 = AtomicU32::new(0);

/// Total image size (in bytes) as declared by the master at
/// RequestDownload. Preserved across the download so the
/// 0x37 handler can write the actual size to the metadata
/// block (the offset has been rounded up to 8 bytes by the
/// trailing-pad write, so the offset alone is not enough).
static OTA_TOTAL_SIZE: AtomicU32 = AtomicU32::new(0);

/// Flash address the next TransferData write will land at.
static OTA_NEXT_OFFSET: AtomicU32 = AtomicU32::new(0);

/// Running CRC32 (ISO-HDLC, init 0xFFFF_FFFF, finalise XOR
/// 0xFFFF_FFFF). Reset at RequestDownload; updated as bytes
/// are written to flash.
static OTA_CRC32: AtomicU32 = AtomicU32::new(0);

/// Whether the post-OTA NVIC reset has been requested by the
/// 0x37 handler. Polled by the canopen task; clears on
/// observation.
static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Block sequence counter expected on the next TransferData.
/// v1 echoes the value but does not enforce the wrap.
static NEXT_BLOCK_SEQ: AtomicU8 = AtomicU8::new(1);

/// 8-byte alignment buffer for the 2-byte TransferData writes.
/// `OTA_BUFFER_LEN` is the number of valid bytes in
/// `OTA_BUFFER[0..OTA_BUFFER_LEN]`; whenever it reaches 8, the
/// canopen task flushes to flash via `flash::write_u64` and
/// resets the length to 0.
static OTA_BUFFER: Mutex<RefCell<([u8; 8], u8)>> =
    Mutex::new(RefCell::new(([0xFF; 8], 0)));

static OTA_TARGET_START: AtomicU32 = AtomicU32::new(0);
static OTA_TARGET_END: AtomicU32 = AtomicU32::new(0);

// ---- Public surface (called from src/can/uds.rs) ----------------------

/// True iff the canopen task should perform an NVIC system
/// reset on the next tick. Cleared once observed.
pub fn take_reset_request() -> bool {
    RESET_REQUESTED.swap(false, Ordering::Relaxed)
}

/// Handle 0x34 RequestDownload. Returns the response length
/// (1..=4) on success or 3 for negative responses.
///
/// **Lives in `.data` (RAM)** — see module-level docs. Calling
/// `flash::erase_app_region` while the erase sequence is running
/// out of flash that the controller is about to clobber is the
/// hazard this whole arrangement exists to prevent.
#[inline(never)]
pub fn handle_request_download(payload: &[u8]) -> usize {
    // Request: [0x00, size_lo, size_hi, size_hi2, size_hi3]
    // (dataFormatIdentifier=0x00 = no compression; 4-byte size
    // little-endian).
    if payload.len() != 5 || payload[0] != 0x00 {
        return store_uds_negative(SID_REQUEST_DOWNLOAD, NRC::IncorrectMessageLength);
    }
    let size = u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]) as usize;

    // Target the OTHER slot.
    if size == 0 || size > (DFU_END - DFU_START) as usize {
        return store_uds_negative(SID_REQUEST_DOWNLOAD, NRC::RequestOutOfRange);
    }

    // Reject if we're already mid-OTA. Per UDS, only one
    // transfer at a time per client.
    if OtaState::from_u8(OTA_STATE.load(Ordering::Relaxed)) != OtaState::Idle {
        return store_uds_negative(SID_REQUEST_DOWNLOAD, NRC::ConditionsNotCorrect);
    }

    // Erase the app region. 124 KB / 2 KB = 62 pages; each
    // erase is a few ms. Total ~50–100 ms.
    info!("OTA: erasing DFU partition ({} bytes)", size);
    if let Err(_e) = unsafe { flash::erase_region(DFU_START, DFU_END) } {
        warn!("OTA: erase failed");
        return store_uds_negative(SID_REQUEST_DOWNLOAD, NRC::GeneralProgrammingFailure);
    }

    // Initialise state.
    OTA_STATE.store(OtaState::Receiving as u8, Ordering::Relaxed);
    OTA_TOTAL_SIZE.store(size as u32, Ordering::Relaxed);
    OTA_REMAINING.store(size as u32, Ordering::Relaxed);
    OTA_NEXT_OFFSET.store(DFU_START, Ordering::Relaxed);
    OTA_TARGET_START.store(DFU_START, Ordering::Relaxed);
    OTA_TARGET_END.store(DFU_END, Ordering::Relaxed);
    OTA_CRC32.store(0xFFFF_FFFF, Ordering::Relaxed);
    NEXT_BLOCK_SEQ.store(1, Ordering::Relaxed);
    critical_section::with(|cs| {
        let (buf, len) = &mut *OTA_BUFFER.borrow_ref_mut(cs);
        *buf = [0xFF; 8];
        *len = 0;
    });

    info!("OTA: state → Receiving");
    // Response: [0x74, 0x00, 0x00, 0x02] — positive, no
    // compression, no address/size echo, 2-byte block size.
    store_uds_positive(&[0x74, 0x00, 0x00, 0x02])
}

/// Handle 0x36 TransferData. `payload` is the bytes after the
/// SID: `[block_seq, b0, b1]` (3 bytes for our 2-byte
/// granularity).
///
/// **Lives in `.data` (RAM)** — this is the hot path during OTA
/// and must not be running from flash while we're writing flash.
/// On STM32G4 the flash prefetch buffer keeps recent instructions
/// alive, but the moment a write reaches the cache line containing
/// our PC the controller would execute garbage.
#[inline(never)]
pub fn handle_transfer_data(payload: &[u8]) -> usize {
    if OtaState::from_u8(OTA_STATE.load(Ordering::Relaxed)) != OtaState::Receiving {
        return store_uds_negative(SID_TRANSFER_DATA, NRC::ConditionsNotCorrect);
    }
    if payload.len() != 3 {
        return store_uds_negative(SID_TRANSFER_DATA, NRC::IncorrectMessageLength);
    }
    // Reject if the declared image size is already fully delivered.
    // Without this check, `OTA_REMAINING` would saturate at zero
    // (the `fetch_update` below uses `saturating_sub`), the OTA
    // state would stay `Receiving`, and a misbehaving master (or a
    // replay loop) could keep writing flash at arbitrary offsets
    // through APP_START..APP_END. TransferExit would then arm the
    // NVIC reset against a half-written / corrupted image.
    if OTA_REMAINING.load(Ordering::Relaxed) == 0 {
        warn!("OTA: TransferData past declared size → 0x31");
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(SID_TRANSFER_DATA, NRC::RequestOutOfRange);
    }
    let seq = payload[0];
    let expected_seq = NEXT_BLOCK_SEQ.load(Ordering::Relaxed);
    // Per UDS, a wrong block sequence number ⇒ 0x73
    // wrongBlockSequenceNumber. Reject the call so the master
    // can re-sync; silently accepting lets blocks get lost
    // and the eventual CRC mismatch has no way to point at
    // the missing block.
    if seq != expected_seq {
        warn!(
            "OTA: block seq {} (expected {}) → 0x73",
            seq, expected_seq
        );
        return store_uds_negative(
            SID_TRANSFER_DATA,
            NRC::WrongBlockSequenceNumber,
        );
    }
    NEXT_BLOCK_SEQ.store(seq.wrapping_add(1), Ordering::Relaxed);

    let bytes: [u8; 2] = [payload[1], payload[2]];

    // Update the running CRC32 with the 2 image bytes.
    let mut crc = OTA_CRC32.load(Ordering::Relaxed);
    for &b in &bytes {
        crc = crc32_update(crc, b);
    }
    OTA_CRC32.store(crc, Ordering::Relaxed);

    // Append to the 8-byte alignment buffer; flush when full.
    let mut flush_error: Option<NRC> = None;
    let mut next_offset: u32 = OTA_NEXT_OFFSET.load(Ordering::Relaxed);
    critical_section::with(|cs| {
        let (buf, len) = &mut *OTA_BUFFER.borrow_ref_mut(cs);
        for &b in &bytes {
            buf[*len as usize] = b;
            *len += 1;
        }
        if *len == 8 {
            let word = u64::from_le_bytes(*buf);
            if unsafe { flash::write_u64(next_offset, OTA_TARGET_START.load(Ordering::Relaxed), OTA_TARGET_END.load(Ordering::Relaxed), word) }.is_err() {
                flush_error = Some(NRC::GeneralProgrammingFailure);
            } else {
                next_offset += 8;
            }
            *buf = [0xFF; 8];
            *len = 0;
        }
    });
    OTA_NEXT_OFFSET.store(next_offset, Ordering::Relaxed);

    // Decrement remaining. If the master over-sent (negative
    // remaining), the next TransferData will get
    // RequestOutOfRange from the state check (we'll be
    // negative and past the expected end). Accept the bytes
    // for now; the user can recover with a 0x37.
    let _ = OTA_REMAINING.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |r| {
        Some(r.saturating_sub(2))
    });

    if let Some(nrc) = flush_error {
        warn!("OTA: flash write failed at 0x{:08x}", next_offset);
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(SID_TRANSFER_DATA, nrc);
    }

    // Positive response: [0x76, seq].
    store_uds_positive(&[0x76, seq])
}

/// Handle 0x37 RequestTransferExit. Flushes any buffered bytes
/// (padded to 8 with 0xFF), writes the post-OTA metadata, sets
/// the reset flag, returns positive.
///
/// **Lives in `.data` (RAM)** — calls `flash::write_metadata` and
/// `flash::write_u64` (the trailing flush) on the OTA path, so
/// must be running from RAM by the time we get here.
#[inline(never)]
pub fn handle_transfer_exit(payload: &[u8]) -> usize {
    if OtaState::from_u8(OTA_STATE.load(Ordering::Relaxed)) != OtaState::Receiving {
        return store_uds_negative(
            SID_REQUEST_TRANSFER_EXIT,
            NRC::ConditionsNotCorrect,
        );
    }
    if !payload.is_empty() {
        return store_uds_negative(
            SID_REQUEST_TRANSFER_EXIT,
            NRC::IncorrectMessageLength,
        );
    }

    // Flush any remaining buffered bytes (padded with 0xFF).
    let next_offset = OTA_NEXT_OFFSET.load(Ordering::Relaxed);
    let bytes_remaining = OTA_REMAINING.load(Ordering::Relaxed);
    let total = OTA_TOTAL_SIZE.load(Ordering::Relaxed);
    let bytes_delivered = total - bytes_remaining;
    let mut flush_error: Option<NRC> = None;
    critical_section::with(|cs| {
        let (buf, len) = &mut *OTA_BUFFER.borrow_ref_mut(cs);
        if *len > 0 {
            let word = u64::from_le_bytes(*buf);
            if unsafe { flash::write_u64(next_offset, OTA_TARGET_START.load(Ordering::Relaxed), OTA_TARGET_END.load(Ordering::Relaxed), word) }.is_err() {
                flush_error = Some(NRC::GeneralProgrammingFailure);
            } else {
                // Successful flush — no need to bump next_offset
                // here (the offset already accounts for the bytes
                // that were buffered; we just finished the write).
            }
        }
        *buf = [0xFF; 8];
        *len = 0;
    });
    if let Some(nrc) = flush_error {
        warn!("OTA: trailing flush failed at 0x{:08x}", next_offset);
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(SID_REQUEST_TRANSFER_EXIT, nrc);
    }

    // Verify the actual bytes delivered matches the declared size.
    // Two failure modes to guard against:
    //
    // 1. Over-send — the master somehow delivered more bytes than
    //    declared (defensive: with the current `saturating_sub` in
    //    `handle_transfer_data` and the `OTA_REMAINING == 0`
    //    short-circuit this can't actually happen, but a future
    //    regression that loosens either guard would silently let
    //    extra bytes land past the declared end of image. Reject
    //    with 0x31 to make the overflow visible.
    //
    // 2. Short transfer — the master gave up early or dropped a
    //    packet. We allow a partial last block (the trailing flush
    //    pads with 0xFF to the next 8-byte boundary), but anything
    //    more than 7 bytes short of `total` means the image is
    //    incomplete. Refuse to mark it valid in metadata; the
    //    bootloader would refuse it on next boot, but we'd have
    //    wasted a flash write + NVIC reset cycle.
    if bytes_delivered > total {
        warn!(
            "OTA: over-send (delivered {} > {}) → 0x31",
            bytes_delivered, total
        );
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(
            SID_REQUEST_TRANSFER_EXIT,
            NRC::RequestOutOfRange,
        );
    }
    if bytes_delivered < total.saturating_sub(7) {
        warn!(
            "OTA: short transfer (delivered {} of {}) → 0x72",
            bytes_delivered, total
        );
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(
            SID_REQUEST_TRANSFER_EXIT,
            NRC::GeneralProgrammingFailure,
        );
    }

    // Finalise CRC32 (XOR with 0xFFFF_FFFF).
    let mut crc = OTA_CRC32.load(Ordering::Relaxed);
    crc ^= 0xFFFF_FFFF;
    let image_size = OTA_TOTAL_SIZE.load(Ordering::Relaxed);

    info!(
        "OTA: done. image_size={} crc32=0x{:08x}",
        image_size, crc
    );

    // CRC32 flash verification — read back every byte from the target
    // slot and compare against the running CRC. If flash cells didn't
    // program correctly (marginal voltage, worn page), this catches it
    // before we commit the slot switch.
    info!("OTA: verifying flash CRC32...");
    {
        let target_start = OTA_TARGET_START.load(Ordering::Relaxed);
        let mut verify_crc: u32 = 0xFFFF_FFFF;
        for offset in (0..image_size).step_by(4) {
            let word = unsafe { flash::read_u32(target_start + offset) };
            let bytes = word.to_le_bytes();
            let take = ((image_size - offset) as usize).min(4);
            for &b in &bytes[..take] {
                verify_crc = crc32_update(verify_crc, b);
            }
        }
        verify_crc ^= 0xFFFF_FFFF;
        if verify_crc != crc {
            warn!("OTA: flash CRC mismatch (expected 0x{:08x}, got 0x{:08x})", crc, verify_crc);
            OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
            return store_uds_negative(SID_REQUEST_TRANSFER_EXIT, NRC::GeneralProgrammingFailure);
        }
        info!("OTA: flash CRC OK");
    }

    // Set SWAP_MAGIC so embassy-boot swaps DFU → ACTIVE on next boot.
    if let Err(_) = unsafe { write_swap_magic() } {
        warn!("OTA: SWAP_MAGIC write failed");
        OTA_STATE.store(OtaState::Idle as u8, Ordering::Relaxed);
        return store_uds_negative(SID_REQUEST_TRANSFER_EXIT, NRC::GeneralProgrammingFailure);
    }
    info!("OTA: SWAP_MAGIC set → will swap on reset");

    // Mark done; arm the reset.
    OTA_STATE.store(OtaState::Done as u8, Ordering::Relaxed);
    RESET_REQUESTED.store(true, Ordering::Relaxed);
    info!("OTA: NVIC reset armed");
    store_uds_positive(&[0x77])
}

// ---- Helpers ---------------------------------------------------------

/// Negative response codes. Only the ones we actually send.
#[allow(dead_code)]
#[repr(u8)]
#[derive(defmt::Format, Copy, Clone, PartialEq, Eq)]
enum NRC {
    IncorrectMessageLength    = 0x13,
    ResponseTooLong           = 0x14,
    ConditionsNotCorrect      = 0x22,
    RequestOutOfRange         = 0x31,
    GeneralProgrammingFailure = 0x72,
    /// 0x73 wrongBlockSequenceNumber. Per UDS, the 0x36 master
    /// must send block seq 1, 2, 3, … wrapping at 0xFF→0x00.
    /// Mismatch ⇒ 0x73. Was previously logged-and-ignored,
    /// which let silent data loss through (CRC would later
    /// mismatch without a way to tell which block was missed).
    WrongBlockSequenceNumber  = 0x73,
}

/// Forward a UDS positive response into the shared response
/// buffer used by the UDS module. The `+ 0x40` is the standard
/// UDS positive-response offset.
///
/// Lives in `.data` (RAM) — called from the OTA handlers which
/// are themselves RAM-resident, so the call chain stays uniform.
#[inline(never)]
fn store_uds_positive(payload: &[u8]) -> usize {
    uds_core::state::store_response(payload)
}

/// Same but for negative responses (`[0x7F, SID, NRC]`).
#[inline(never)]
fn store_uds_negative(sid: u8, nrc: NRC) -> usize {
    uds_core::state::store_response(&[0x7F, sid, nrc as u8])
}

/// Standard CRC-32/ISO-HDLC (poly 0x04C1_1DB7), one byte at a time.
#[inline(never)]
fn crc32_update(crc: u32, byte: u8) -> u32 {
    let mut c = crc ^ byte as u32;
    for _ in 0..8 {
        c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
    }
    c
}

unsafe fn write_swap_magic() -> Result<(), flash::FlashError> {
    // Erase the STATE page (2 KB) then write SWAP_MAGIC (0xF0) as the
    // first byte. embassy-boot reads this on boot and swaps DFU→ACTIVE.
    let page_start = STATE_ADDR & !(2047u32);
    flash::erase_region(page_start, page_start + 2048)?;
    flash::write_u64(STATE_ADDR, page_start, page_start + 2048, SWAP_MAGIC as u64)
}


