//! y-modem CRC-mode receiver.
//!
//! Flow: send 'C' → SOH header (filename+size) → STX data packets → EOT.
//! Each packet CRC16-validated; full image CRC-32/ISO-HDLC validated at end.

use crate::crc::{crc32_finalize, crc32_init, crc32_update};
use crate::flash::Stm32g4Flash;
use crate::uart::{uart_read_byte_timeout, uart_write_byte, uart_write_str};
use embedded_storage::nor_flash::ReadNorFlash;
use embedded_storage::nor_flash::NorFlash;
use foc_common::{APP_END_ADDRESS, APP_START_ADDRESS};

const SOH: u8 = 0x01;
const STX: u8 = 0x02;
const EOT: u8 = 0x04;
const ACK: u8 = 0x06;
const NAK: u8 = 0x15;
const CAN: u8 = 0x18;
const CRC_FLAG: u8 = b'C';

const DATA_PAYLOAD: usize = 1024;
const HDR_PAYLOAD: usize = 128;
const CRC16_LEN: usize = 2;
const PACKET_TIMEOUT_MS: u32 = 30_000;
const CRC32_FOOTER_LEN: usize = 4;

#[derive(Debug)]
pub enum YmodemError {
    Timeout,
    Aborted,
    InvalidPacket,
    CrcMismatch,
    ImageTooLarge,
    /// Flash erase/programming error bubbled up from the driver.
    Flash,
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 { crc = (crc << 1) ^ 0x1021; }
            else { crc <<= 1; }
        }
    }
    crc
}

fn read_control_byte() -> Result<u8, YmodemError> {
    uart_read_byte_timeout(PACKET_TIMEOUT_MS).map_err(|_| YmodemError::Timeout)
}

fn read_bytes_timeout(buf: &mut [u8]) -> Result<(), YmodemError> {
    for byte in buf.iter_mut() {
        *byte = uart_read_byte_timeout(PACKET_TIMEOUT_MS).map_err(|_| YmodemError::Timeout)?;
    }
    Ok(())
}

fn read_packet_number(payload: &[u8]) -> Result<u8, YmodemError> {
    if payload[0].wrapping_add(payload[1]) != 0xFF {
        return Err(YmodemError::InvalidPacket);
    }
    Ok(payload[0])
}

fn validate_packet_crc(data: &[u8], crc_hi: u8, crc_lo: u8) -> Result<(), YmodemError> {
    let expected = ((crc_hi as u16) << 8) | (crc_lo as u16);
    if crc16_ccitt(data) != expected { Err(YmodemError::InvalidPacket) } else { Ok(()) }
}

fn check_abort() -> Result<(), YmodemError> {
    match uart_read_byte_timeout(1000) {
        Ok(CAN) => { uart_write_byte(ACK); Err(YmodemError::Aborted) }
        _ => Err(YmodemError::InvalidPacket),
    }
}

fn parse_file_size(hdr: &[u8; HDR_PAYLOAD]) -> Option<usize> {
    let fname_end = hdr.iter().position(|&b| b == 0)?;
    let size_start = fname_end + 1;
    if size_start >= hdr.len() { return None; }
    let size_end = hdr[size_start..]
        .iter()
        .position(|&b| b == 0 || b == b' ' || b == b'\r')
        .map(|p| size_start + p)
        .unwrap_or(hdr.len());
    let s = core::str::from_utf8(&hdr[size_start..size_end]).ok()?;
    s.trim().parse::<usize>().ok()
}

pub fn receive_image(flash: &mut Stm32g4Flash) -> Result<(), YmodemError> {
    uart_write_byte(CRC_FLAG);

    let control = read_control_byte()?;
    match control {
        SOH => {},
        CAN => return check_abort(),
        _ => return Err(YmodemError::InvalidPacket),
    }

    let mut num_buf = [0u8; 2];
    read_bytes_timeout(&mut num_buf)?;
    if read_packet_number(&num_buf)? != 0 { return Err(YmodemError::InvalidPacket); }

    let mut hdr_buf = [0u8; HDR_PAYLOAD];
    read_bytes_timeout(&mut hdr_buf)?;

    let mut crc16 = [0u8; CRC16_LEN];
    read_bytes_timeout(&mut crc16)?;

    let file_size = parse_file_size(&hdr_buf).unwrap_or(0);
    if file_size < CRC32_FOOTER_LEN { return Err(YmodemError::InvalidPacket); }
    let data_size = file_size - CRC32_FOOTER_LEN;

    if data_size > (APP_END_ADDRESS - APP_START_ADDRESS) as usize {
        return Err(YmodemError::ImageTooLarge);
    }

    let mut hdr_crc_data = [0u8; 2 + HDR_PAYLOAD];
    hdr_crc_data[..2].copy_from_slice(&num_buf);
    hdr_crc_data[2..].copy_from_slice(&hdr_buf);
    if validate_packet_crc(&hdr_crc_data[..], crc16[0], crc16[1]).is_err() {
        uart_write_byte(NAK);
        return Err(YmodemError::InvalidPacket);
    }
    uart_write_byte(ACK);

    let mut next_packet_num: u8 = 1;
    let mut bytes_received: usize = 0;
    let mut flash_offset: u32 = APP_START_ADDRESS;
    let mut crc_inited = false;
    let mut data_buf = [0u8; DATA_PAYLOAD];

    loop {
        let control = read_control_byte()?;

        match control {
            STX => {
                let mut hdr2 = [0u8; 2];
                read_bytes_timeout(&mut hdr2)?;
                if read_packet_number(&hdr2)? != next_packet_num { return Err(YmodemError::InvalidPacket); }

                read_bytes_timeout(&mut data_buf)?;

                let mut crc16_bytes = [0u8; CRC16_LEN];
                read_bytes_timeout(&mut crc16_bytes)?;

                let mut crc_buf = [0u8; 2 + DATA_PAYLOAD];
                crc_buf[..2].copy_from_slice(&hdr2);
                crc_buf[2..].copy_from_slice(&data_buf);
                if validate_packet_crc(&crc_buf[..], crc16_bytes[0], crc16_bytes[1]).is_err() {
                    uart_write_byte(NAK);
                    continue;
                }

                let remaining = data_size.saturating_sub(bytes_received);
                let write_len = remaining.min(DATA_PAYLOAD);

                if write_len > 0 {
                    if !crc_inited { crc32_init(); crc_inited = true; }
                    crc32_update(&data_buf[..write_len]);

                    let aligned_len = (write_len + 7) & !7;
                    let mut write_buf = [0u8; DATA_PAYLOAD];
                    write_buf[..write_len].copy_from_slice(&data_buf[..write_len]);
                    for b in write_buf[write_len..aligned_len].iter_mut() { *b = 0xFF; }

                    flash.write(flash_offset, &write_buf[..aligned_len])
                        .map_err(|_| YmodemError::Flash)?;
                    flash_offset += aligned_len as u32;
                    bytes_received += write_len;
                }

                uart_write_byte(ACK);
                next_packet_num = next_packet_num.wrapping_add(1);
            }

            EOT => {
                uart_write_byte(NAK);
                if read_control_byte()? != EOT { return Err(YmodemError::InvalidPacket); }
                uart_write_byte(ACK);
                break;
            }

            CAN => {
                check_abort()?;
                return Err(YmodemError::InvalidPacket);
            }

            _ => return Err(YmodemError::InvalidPacket),
        }
    }

    if !crc_inited { return Err(YmodemError::InvalidPacket); }
    let computed = crc32_finalize();

    let expected_addr = flash_offset - CRC32_FOOTER_LEN as u32;
    let mut expected_bytes = [0u8; CRC32_FOOTER_LEN];
    flash.read(expected_addr, &mut expected_bytes).map_err(|_| YmodemError::Flash)?;
    let expected = u32::from_le_bytes(expected_bytes);

    if computed != expected {
        uart_write_str("CRC32 FAIL\r\n");
        return Err(YmodemError::CrcMismatch);
    }

    uart_write_str("CRC32 OK\r\n");
    Ok(())
}
