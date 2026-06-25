//! y-modem receive protocol (1 KB packets, CRC16 per packet, CRC32 of full image).
//!
//! Protocol flow:
//!   1. Receiver sends 'C' (CRC mode request)
//!   2. Sender sends SOH (128 B) header packet: filename + size in ASCII
//!   3. Receiver ACKs
//!   4. Sender sends STX (1024 B) data packets (numbered from 1)
//!   5. Receiver writes each packet to flash, ACKs
//!   6. Sender sends EOT → receiver NAKs → sender sends EOT → receiver ACKs
//!   7. Receiver validates CRC32 of full image against 4-byte footer
//!
//! References:
//!   - https://en.wikipedia.org/wiki/XMODEM#YMODEM
//!   - https://techheap.packetizer.com/communication/modems/xmodem-ymodem.html
//!
//! Design notes:
//!   - Per-packet CRC16 is validated (software CRC-CCITT) to detect
//!     transmission errors early. On mismatch we NAK for retransmit.
//!   - DWT cycle counter is NOT initialized by cortex-m-rt on STM32G4,
//!     so we use `cortex_m::asm::delay()` for timeout (170 MHz sysclk).
//!   - The sender appends a 4-byte CRC32 (ISO-HDLC) footer to the image.
//!     We compute CRC32 of the image data only and compare.

use crate::crc::{crc32_finalize, crc32_init, crc32_update};
use crate::flash::{FlashError, Stm32g4Flash};
use crate::uart::{uart_read_byte_timeout, uart_write_byte, uart_write_str};
use embedded_storage::nor_flash::NorFlash;
use embedded_storage::nor_flash::ReadNorFlash;
use foc_common::{APP_END_ADDRESS, APP_START_ADDRESS};

// ── Constants ──────────────────────────────────────────────────────────

/// SOH = 128-byte packet header.
const SOH: u8 = 0x01;
/// STX = 1024-byte data packet.
const STX: u8 = 0x02;
/// EOT = end of transmission.
const EOT: u8 = 0x04;
/// ACK = acknowledge.
const ACK: u8 = 0x06;
/// NAK = negative acknowledge.
const NAK: u8 = 0x15;
/// CAN = cancel transmission (2 consecutive to abort).
const CAN: u8 = 0x18;
/// ASCII 'C' = request CRC-16 mode.
const CRC_FLAG: u8 = b'C';

/// Max data payload in an STX packet.
const DATA_PAYLOAD: usize = 1024;
/// Header payload in an SOH packet.
const HDR_PAYLOAD: usize = 128;
/// CRC16 trailer length per packet.
const CRC16_LEN: usize = 2;

/// Total timeout for receiving a packet (30 seconds).
const PACKET_TIMEOUT_MS: u32 = 30_000;

/// Size of the CRC32 footer appended by the sender.
const CRC32_FOOTER_LEN: usize = 4;

// ── Errors ─────────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum YmodemError {
    /// No byte arrived within 30 seconds.
    Timeout,
    /// Two consecutive CAN bytes received — sender aborted.
    Aborted,
    /// Invalid packet number, control byte, or other protocol violation.
    InvalidPacket,
    /// Final CRC32 of the image does not match the expected footer.
    CrcMismatch,
    /// Image spans beyond APP_END_ADDRESS.
    ImageTooLarge,
    /// Flash write/erase error (delegated from the driver).
    #[allow(dead_code)]
    Flash(FlashError),
    /// Stub placeholder (Task 4 compatibility). Not returned in Task 5.
    #[allow(dead_code)]
    NotImplemented,
}

// ── CRC16 (software, per-packet) ───────────────────────────────────────

/// Software CRC-CCITT (polynomial 0x1021, init 0x0000, no final XOR).
///
/// We validate per-packet CRC16 to detect transmission errors early.
/// If validation fails, we NAK and wait for a retransmit (y-modem
/// specifies up to 10 retries).
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ── Read helpers ───────────────────────────────────────────────────────

/// Read the control byte (SOH / STX / EOT / CAN) with a 30-second timeout.
///
/// Returns `Err(YmodemError::Timeout)` if no byte arrives within
/// `PACKET_TIMEOUT_MS`.
fn read_control_byte() -> Result<u8, YmodemError> {
    uart_read_byte_timeout(PACKET_TIMEOUT_MS).map_err(|_| YmodemError::Timeout)
}

/// Read exactly `count` bytes with per-byte timeout.
///
/// Uses `uart_read_byte_timeout` on each byte individually so that
/// a stall in the middle of a packet is still caught.
fn read_bytes_timeout(buf: &mut [u8]) -> Result<(), YmodemError> {
    for byte in buf.iter_mut() {
        *byte = uart_read_byte_timeout(PACKET_TIMEOUT_MS).map_err(|_| YmodemError::Timeout)?;
    }
    Ok(())
}

// ── Packet reading ─────────────────────────────────────────────────────

/// Read and validate a y-modem packet header (packet number + complement).
///
/// Returns the packet number on success, or an error if the complement
/// check fails.
fn read_packet_number(payload: &[u8]) -> Result<u8, YmodemError> {
    let num = payload[0];
    let complement = payload[1];
    if (num as u8).wrapping_add(complement) != 0xFF {
        return Err(YmodemError::InvalidPacket);
    }
    Ok(num)
}

/// Validate the CRC16 trailer at the end of a packet.
///
/// `data` is the packet content (packet num + complement + payload).
fn validate_packet_crc(data: &[u8], crc_hi: u8, crc_lo: u8) -> Result<(), YmodemError> {
    let expected = ((crc_hi as u16) << 8) | (crc_lo as u16);
    let computed = crc16_ccitt(data);
    if computed != expected {
        return Err(YmodemError::InvalidPacket);
    }
    Ok(())
}

/// Check for 2 consecutive CAN bytes (abort signal).
///
/// After receiving the first CAN, reads the next byte with a short timeout.
/// If a second CAN arrives, the sender has aborted and we ACK the abort.
fn check_abort() -> Result<(), YmodemError> {
    match uart_read_byte_timeout(1000) {
        Ok(CAN) => {
            // Abort acknowledged — sender sent 2 consecutive CANs.
            uart_write_byte(ACK);
            Err(YmodemError::Aborted)
        }
        // If the second byte is missing or different, the first byte was
        // spurious (e.g. part of data).
        _ => Err(YmodemError::InvalidPacket),
    }
}

/// Parse the file size from a y-modem SOH header payload.
///
/// The header payload format is: "filename\0 12345 \0..." (filename, NUL,
/// ASCII decimal size, optionally terminated by NUL or space).
fn parse_file_size(hdr: &[u8; HDR_PAYLOAD]) -> Option<usize> {
    // Find end of filename (first NUL).
    let fname_end = hdr.iter().position(|&b| b == 0)?;
    let size_start = fname_end + 1;
    if size_start >= hdr.len() {
        return None;
    }
    // Size is ASCII decimal, terminated by NUL or space.
    let size_end = hdr[size_start..]
        .iter()
        .position(|&b| b == 0 || b == b' ' || b == b'\r')
        .map(|p| size_start + p)
        .unwrap_or(hdr.len());
    let s = core::str::from_utf8(&hdr[size_start..size_end]).ok()?;
    s.trim().parse::<usize>().ok()
}

// ── Public entry point ─────────────────────────────────────────────────

/// Receive a y-modem image and write it to flash.
///
/// The sender must use y-modem CRC mode (1 KB packets). The image is written
/// starting at `APP_START_ADDRESS`. After all data is received, the CRC32
/// of the image data is validated against the 4-byte footer appended by the
/// sender.
pub fn receive_image(flash: &mut Stm32g4Flash) -> Result<(), YmodemError> {
    // ── 1. Send 'C' to request CRC mode ─────────────────────────────
    uart_write_byte(CRC_FLAG);

    // ── 2. Read the SOH header packet (packet 0) ────────────────────
    //
    // Layout: [SOH(1) | pkt_num(1) | ~pkt_num(1) | payload(128) |
    //          CRC16_hi(1) | CRC16_lo(1)]
    //   = 133 bytes total on wire

    let control = read_control_byte()?;
    match control {
        SOH => {} // Expected.
        CAN => return check_abort(),
        _ => return Err(YmodemError::InvalidPacket),
    }

    // Read packet number + complement.
    let mut num_buf = [0u8; 2];
    read_bytes_timeout(&mut num_buf)?;
    let pkt_num = read_packet_number(&num_buf)?;
    if pkt_num != 0 {
        return Err(YmodemError::InvalidPacket);
    }

    // Read the 128-byte header payload.
    let mut hdr_buf = [0u8; HDR_PAYLOAD];
    read_bytes_timeout(&mut hdr_buf)?;

    // Read CRC16 trailer (consume to stay in sync — we validate below).
    let mut crc16 = [0u8; CRC16_LEN];
    read_bytes_timeout(&mut crc16)?;

    // Parse filesize from header payload.
    let file_size = parse_file_size(&hdr_buf).unwrap_or(0);
    if file_size < CRC32_FOOTER_LEN {
        return Err(YmodemError::InvalidPacket);
    }
    let data_size = file_size - CRC32_FOOTER_LEN;

    // Verify the image fits in the app region.
    if data_size > (APP_END_ADDRESS - APP_START_ADDRESS) as usize {
        return Err(YmodemError::ImageTooLarge);
    }

    // Validate CRC16 of the header packet (covers [pkt_num, ~pkt_num, payload]).
    let mut hdr_crc_data = [0u8; 2 + HDR_PAYLOAD];
    hdr_crc_data[..2].copy_from_slice(&num_buf);
    hdr_crc_data[2..].copy_from_slice(&hdr_buf);
    if validate_packet_crc(&hdr_crc_data[..], crc16[0], crc16[1]).is_err() {
        uart_write_byte(NAK);
        // We don't retry the header in this implementation.
        return Err(YmodemError::InvalidPacket);
    }

    // ACK the header.
    uart_write_byte(ACK);

    // ── 3. Receive data packets ─────────────────────────────────────
    //
    // Each data packet:
    //   [STX(1) | pkt_num(1) | ~pkt_num(1) | data(1024) | CRC16(2)]
    //   = 1029 bytes on wire
    //
    // The last packet payload is padded with 0x1A (Ctrl-Z / EOF) up to
    // 1024 bytes.

    let mut next_packet_num: u8 = 1;
    let mut bytes_received: usize = 0;
    let mut flash_offset: u32 = APP_START_ADDRESS;
    let mut crc_inited = false;

    // Workspace for a single STX data packet (stack-allocated).
    let mut data_buf = [0u8; DATA_PAYLOAD];

    loop {
        let control = read_control_byte()?;

        match control {
            STX => {
                // Read packet number + complement.
                let mut hdr2 = [0u8; 2];
                read_bytes_timeout(&mut hdr2)?;
                let this_pkt = read_packet_number(&hdr2)?;

                if this_pkt != next_packet_num {
                    return Err(YmodemError::InvalidPacket);
                }

                // Read the 1024-byte payload.
                read_bytes_timeout(&mut data_buf)?;

                // Read CRC16 trailer.
                let mut crc16_bytes = [0u8; CRC16_LEN];
                read_bytes_timeout(&mut crc16_bytes)?;

                // Validate CRC16 of the data packet.
                // y-modem CRC covers: [pkt_num, ~pkt_num, data(1024)]
                let mut crc_buf = [0u8; 2 + DATA_PAYLOAD];
                crc_buf[..2].copy_from_slice(&hdr2);
                crc_buf[2..].copy_from_slice(&data_buf);
                if validate_packet_crc(&crc_buf[..], crc16_bytes[0], crc16_bytes[1]).is_err() {
                    // CRC16 mismatch — request retransmit.
                    uart_write_byte(NAK);
                    continue;
                }

                // ── Write to flash ──
                let remaining = data_size.saturating_sub(bytes_received);
                let write_len = remaining.min(DATA_PAYLOAD);

                if write_len > 0 {
                    if !crc_inited {
                        crc32_init();
                        crc_inited = true;
                    }
                    // Feed only the actual data bytes into CRC32 (skip padding).
                    crc32_update(&data_buf[..write_len]);

                    // Pad to WRITE_SIZE (8) alignment.
                    let aligned_len = (write_len + 7) & !7;
                    let mut write_buf = [0u8; DATA_PAYLOAD];
                    write_buf[..write_len].copy_from_slice(&data_buf[..write_len]);
                    // Fill padding with 0xFF (erased flash state after erase).
                    for b in write_buf[write_len..aligned_len].iter_mut() {
                        *b = 0xFF;
                    }

                    flash.write(flash_offset, &write_buf[..aligned_len])
                        .map_err(YmodemError::Flash)?;
                    flash_offset += aligned_len as u32;
                    bytes_received += write_len;
                }

                uart_write_byte(ACK);
                next_packet_num = next_packet_num.wrapping_add(1);
            }

            EOT => {
                // First EOT — NAK to request second EOT (y-modem spec).
                uart_write_byte(NAK);

                // Wait for second EOT.
                let eot2 = read_control_byte()?;
                if eot2 != EOT {
                    return Err(YmodemError::InvalidPacket);
                }
                uart_write_byte(ACK);
                break;
            }

            CAN => {
                // Check for double-CAN abort.
                check_abort()?;
                // If we get here, second byte was not CAN — invalid packet.
                return Err(YmodemError::InvalidPacket);
            }

            _ => {
                return Err(YmodemError::InvalidPacket);
            }
        }
    }

    // ── 4. Validate CRC32 of the image ──────────────────────────────
    if !crc_inited {
        // No data packets were received — invalid state.
        return Err(YmodemError::InvalidPacket);
    }

    let computed = crc32_finalize();

    // Read the expected CRC32 from the last 4 bytes of the written data.
    // The sender placed the CRC32 footer as the last 4 bytes of the image.
    let expected_addr = flash_offset - CRC32_FOOTER_LEN as u32;
    let mut expected_bytes = [0u8; CRC32_FOOTER_LEN];
    flash.read(expected_addr, &mut expected_bytes)
        .map_err(YmodemError::Flash)?;
    let expected = u32::from_le_bytes(expected_bytes);

    if computed != expected {
        uart_write_str("CRC32 FAIL\r\n");
        return Err(YmodemError::CrcMismatch);
    }

    uart_write_str("CRC32 OK\r\n");
    Ok(())
}
