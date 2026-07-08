#!/usr/bin/env python3
"""
Smoke test for the foc-rust Phase 1–4 OTA / UDS / CANopen stack.

Two run modes:

* `sim` (default) — in-process simulation. A Python firmware
  emulator mirrors the Rust firmware's wire format byte-for-byte
  (SDO server, OD, UDS services, OTA state machine, NMT,
  heartbeat). A Python master driver drives it via a `SimBus`
  abstraction (dicts with `id`, `data`, `dlc`). The driver parses
  the responses the same way a real CANopen master would and
  asserts on every byte. No hardware, no vcan required.

* `live` — runs against real frames on a real CAN interface via
  `python-can`. The bus is whatever the firmware is on: an
  actual STM32G431B-ESC1 connected via USB-CAN, or
  `scripts/firmware_emulator.py` listening on a vcan0. Use
  this mode to verify the wire format end-to-end without the
  Rust firmware, or to cross-check Rust + Python side-by-side.

The simulator path is the "spec test" — it verifies the firmware
implements the wire protocol correctly by comparing against an
independently-authored Python version that follows CiA 301 / ISO
14229 to the letter. If the Rust and Python implementations
agree on every byte for every scenario, a real master talking to
the real firmware should also agree.

Usage::

    # In-process sim (default, no hardware needed)
    python3 scripts/smoke_test.py
    python3 scripts/smoke_test.py --scenarios heartbeat,nmt
    python3 scripts/smoke_test.py --list

    # Live mode against vcan0
    sudo scripts/setup_vcan.sh                              # bring up vcan0
    python3 scripts/firmware_emulator.py vcan0 &            # in one shell
    python3 scripts/smoke_test.py --live vcan0              # in another

    # Live mode against real hardware on PCAN-USB
    python3 scripts/smoke_test.py --live pcan:PCAN_USBBUS1

Exit code: 0 on all-pass, 1 on any scenario failure, 2 if the
bus couldn't be opened (live mode).
"""

import argparse
import struct
import sys
import time


# ---- CiA 301 / ISO 14229 constants ----------------------------------

NODE_ID = 1

# COB-IDs
NMT_COB_ID = 0x000
HEARTBEAT_COB_ID = 0x700 + NODE_ID
SDO_RX_COB_ID = 0x600 + NODE_ID          # master → slave
SDO_TX_COB_ID = 0x580 + NODE_ID          # slave → master

# Phase 6: UDS has its own CAN-IDs, decoupled from CANopen SDO.
# Per ISO 14229-3 §7: functional broadcast 0x7DF, physical request
# 0x7E0-0x7E7 (per ECU address), physical response 0x7E8-0x7EF.
UDS_FUNCTIONAL_REQUEST_ID = 0x7DF
UDS_PHYSICAL_REQUEST_ID  = 0x7E0  # our ECU address = 1
UDS_PHYSICAL_RESPONSE_ID = 0x7E8  # = request + 8

# SDO command specifiers (top 3 bits of byte 0)
SDO_CMD_DOWNLOAD   = 0x20  # CCS=1 — Initiate Download Request
SDO_CMD_DOWNLOAD_SEG = 0x00 # CCS=0 — Download Segment Request
SDO_CMD_UPLOAD     = 0x40  # CCS=2 — Initiate Upload Request
SDO_CMD_UPLOAD_SEG = 0x60  # CCS=3 — Upload Segment Request
SDO_CMD_ABORT      = 0x80  # SCS=4 — Abort Transfer

SDO_MAX_SEGMENTED_SIZE = 18

# Initiate Download n-mask (bits 2-3 of byte 0). 0x0C = bits 2-3,
# NOT bit 4. See src/can/sdo.rs for the rationale.
SDO_N_MASK = 0x0C

# UDS service IDs (ISO 14229)
SID_DSC  = 0x10  # DiagnosticSessionControl
SID_ER   = 0x11  # ECUReset
SID_CDI  = 0x14  # ClearDiagnosticInformation
SID_RDTCI = 0x19 # ReadDTCInformation
SID_RDBI = 0x22  # ReadDataByIdentifier
SID_WDBI = 0x2E  # WriteDataByIdentifier
SID_SA   = 0x27  # SecurityAccess
SID_CC   = 0x28  # CommunicationControl
SID_RC   = 0x31  # RoutineControl
SID_TP   = 0x3E  # TesterPresent
SID_RD   = 0x34  # RequestDownload (OTA)
SID_TD   = 0x36  # TransferData (OTA)
SID_RTE  = 0x37  # RequestTransferExit (OTA)

# UDS negative response codes
NRC_SUB_FUNC_NOT_SUPPORTED       = 0x12
NRC_INCORRECT_MESSAGE_LENGTH     = 0x13
NRC_RESPONSE_TOO_LONG            = 0x14
NRC_CONDITIONS_NOT_CORRECT       = 0x22
NRC_REQUEST_OUT_OF_RANGE         = 0x31
NRC_SECURITY_ACCESS_DENIED       = 0x33
NRC_INVALID_KEY                  = 0x35
NRC_EXCEEDED_NUMBER_OF_ATTEMPTS  = 0x36
NRC_GENERAL_PROGRAMMING_FAILURE  = 0x72
NRC_WRONG_BLOCK_SEQUENCE_NUMBER  = 0x73
NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION = 0x7E
NRC_RESPONSE_PENDING             = 0x78
NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION = 0x7E

# SDO abort codes (CiA 301)
SDO_ABORT_TOGGLE_BIT_NOT_ALTERED = 0x0503_0000
SDO_ABORT_INVALID_COMMAND        = 0x0504_0001
SDO_ABORT_READ_ONLY              = 0x0601_0002
SDO_ABORT_OBJECT_DOES_NOT_EXIST  = 0x0602_0000
SDO_ABORT_LENGTH_MISMATCH        = 0x0607_0010

# OTA
APP_START = 0x0800_0000
APP_SIZE  = 0x1F800  # 124 KB app region (B-G431B-ESC1 has 128 KB)

# AES-128 single-block ECB for UDS seed-key.
# Pure Python implementation — no dependencies.
# Matches the Rust `generate_key` in src/uds/crypto.rs.
AES_SBOX = (
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5,
    0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc,
    0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a,
    0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b,
    0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85,
    0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17,
    0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
    0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9,
    0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6,
    0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94,
    0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68,
    0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
)
AES_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36)

def _aes_sub_word(w: int) -> int:
    return (AES_SBOX[(w >> 24) & 0xff] << 24 |
            AES_SBOX[(w >> 16) & 0xff] << 16 |
            AES_SBOX[(w >> 8) & 0xff] << 8 |
            AES_SBOX[w & 0xff])

def _aes_key_expansion(key: bytes) -> list[int]:
    nk, nb, nr = 4, 4, 10
    w = [0] * (nb * (nr + 1))
    for i in range(nk):
        w[i] = (key[i*4] << 24 | key[i*4+1] << 16 |
                key[i*4+2] << 8 | key[i*4+3])
    for i in range(nk, nb * (nr + 1)):
        temp = w[i - 1]
        if i % nk == 0:
            temp = _aes_sub_word((temp << 8) | (temp >> 24)) ^ (AES_RCON[i // nk - 1] << 24)
        w[i] = w[i - nk] ^ temp
    return w

def _aes_encrypt_block(plaintext: bytes, key: bytes) -> bytes:
    """AES-128 single-block ECB encryption. Returns 16 bytes."""
    w = _aes_key_expansion(key)
    state = list(plaintext)
    def add_round_key(r: int):
        for i in range(16):
            state[i] ^= (w[r * 4 + i // 4] >> (24 - (i % 4) * 8)) & 0xff
    def sub_bytes():
        for i in range(16):
            state[i] = AES_SBOX[state[i]]
    def shift_rows():
        state[1], state[5], state[9], state[13] = state[5], state[9], state[13], state[1]
        state[2], state[6], state[10], state[14] = state[10], state[14], state[2], state[6]
        state[3], state[7], state[11], state[15] = state[15], state[3], state[7], state[11]
    def mix_columns():
        for c in range(4):
            i = c * 4
            a0, a1, a2, a3 = state[i], state[i+1], state[i+2], state[i+3]
            state[i]   = _gmul(2, a0) ^ _gmul(3, a1) ^ a2 ^ a3
            state[i+1] = a0 ^ _gmul(2, a1) ^ _gmul(3, a2) ^ a3
            state[i+2] = a0 ^ a1 ^ _gmul(2, a2) ^ _gmul(3, a3)
            state[i+3] = _gmul(3, a0) ^ a1 ^ a2 ^ _gmul(2, a3)
    def _gmul(a: int, b: int) -> int:
        p = 0
        for _ in range(8):
            if b & 1: p ^= a
            hi = a & 0x80
            a = (a << 1) & 0xff
            if hi: a ^= 0x1b
            b >>= 1
        return p
    add_round_key(0)
    for r in range(1, 10):
        sub_bytes()
        shift_rows()
        mix_columns()
        add_round_key(r)
    sub_bytes()
    shift_rows()
    add_round_key(10)
    return bytes(state)

def _aes_key(seed: bytes, mask: bytes) -> bytes:
    """AES-128-ECB: key = AES_encrypt(seed, key_material)."""
    return _aes_encrypt_block(seed, mask)

# Default AES key masks (16 bytes each, matching Rust
# UDS_CONFIG.key_masks defaults in static_config.rs).
KEY_MASK_SAL1 = bytes([
    0x30, 0x00, 0x22, 0x12, 0xAB, 0xCD, 0xEF, 0x01,
    0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67,
])
KEY_MASK_SAL2 = bytes([
    0x52, 0x4C, 0x5E, 0x63, 0xDE, 0xAD, 0xBE, 0xEF,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
])
KEY_MASK_SAL3 = bytes([
    0xA5, 0xC3, 0xF1, 0x1B, 0xCA, 0xFE, 0xBA, 0xBE,
    0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12,
])

# 16-byte seed constant for emulator use.
SEED_16 = bytes([
    0xa5, 0xa5, 0xa5, 0xa5, 0xb0, 0xb1, 0xb2, 0xb3,
    0xc0, 0xc1, 0xc2, 0xc3, 0xd0, 0xd1, 0xd2, 0xd3,
])

# Pre-computed AES keys for the above seed + masks (used by emulator).
KEY       = _aes_key(SEED_16, KEY_MASK_SAL1)
KEY_SAL2  = _aes_key(SEED_16, KEY_MASK_SAL2)
KEY_SAL3  = _aes_key(SEED_16, KEY_MASK_SAL3)


# ---- CRC-32/ISO-HDLC (matches src/can/ota.rs::crc32_update) -------

def crc32_update(crc: int, byte: int) -> int:
    c = crc ^ byte
    for _ in range(8):
        c = ((c >> 1) ^ 0xEDB8_8320) if (c & 1) else (c >> 1)
    return c & 0xFFFF_FFFF


# ---- Mock firmware (Python mirror of the Rust firmware) ------------

class FirmwareEmulator:
    """Python mirror of the Rust firmware's protocol stack. Produces
    the exact same wire bytes for any given master frame so that
    spec-level assertions can verify the firmware's behavior."""

    def __init__(self) -> None:
        # Object Dictionary (initial values match src/can/od.rs)
        self.od: dict[int, dict[int, bytes]] = {
            0x1000: {0: b'\x00\x00\x00\x00'},                         # DeviceType
            0x1001: {0: b'\x00'},                                     # ErrorRegister
            0x1017: {0: struct.pack('<H', 1000)},                     # HeartbeatProducerTime
            0x1018: {                                                  # Identity
                1: struct.pack('<I', 0x0000_CAFE),                     # VendorId
                2: struct.pack('<I', 0x0000_00B0),                     # ProductCode (B-G431B-ESC1)
                3: struct.pack('<I', 0x0000_0001),                     # Revision
                4: struct.pack('<I', 0x0000_0000),                     # Serial
            },
        }
        self.last_response: bytes = b''  # returned by SDO read 0x2F00.0

        # CANopen state
        self.nmt_state = 'PreOperational'  # post boot-up

        # UDS state
        self.uds_session = 0x01   # Default
        self.uds_security = 0     # Locked
        self.uds_tx_disabled = False  # 0x28 CommControl state
        self.uds_rx_disabled = False
        # Key masks (matching Rust UDS_CONFIG.key_masks defaults).
        self.key_masks = [KEY_MASK_SAL1, KEY_MASK_SAL2, KEY_MASK_SAL3]

        # DTC storage: list of (code, status) tuples.
        self.dtcs: list[tuple[int, int]] = []

        # SecurityAccess fail count (matching Rust sa_fail_count).
        self.sa_fail_count: int = 0

        # OTA state
        self.ota_state = 'Idle'
        self.ota_total = 0
        self.ota_remaining = 0
        self.ota_offset = APP_START
        self.ota_crc32 = 0xFFFF_FFFF
        self.ota_next_block_seq = 1

        # Segmented SDO upload state
        self.seg_buf = bytearray(7)
        self.seg_len = 0
        self.seg_offset = 0
        self.seg_toggle = 0

        # Segmented SDO download state
        self.dl_seg_buf = bytearray(SDO_MAX_SEGMENTED_SIZE)
        self.dl_seg_len = 0
        self.dl_seg_offset = 0
        self.dl_seg_toggle = 0
        self.dl_seg_idx = 0
        self.dl_seg_sub = 0

    # ---- UDS (Phase 6: independent of CANopen) ----------------------

    def handle_uds(self, frame: dict) -> dict | None:
        """Phase 6: UDS requests come on 0x7DF (functional) or
        0x7E0 (physical). We dispatch the payload directly to
        `_dispatch_uds` (same code path as the legacy SDO tunnel
        used to take). The response frame is returned on
        0x7E8 (per ISO 14229-3 §7)."""
        if frame['id'] not in (UDS_FUNCTIONAL_REQUEST_ID,
                                UDS_PHYSICAL_REQUEST_ID):
            return None
        d = frame['data']
        # DLC might be < 8 if master sends short frame.
        dlc = frame['dlc']
        payload = bytes(d[:dlc])
        # Skip the empty frame case
        if not payload:
            return None
        self._dispatch_uds(payload)
        # Build response frame. DLC is the actual response
        # length (not 8) so the master can distinguish empty
        # (suppress positive) from short positive.
        resp_bytes = self.last_response
        if not resp_bytes:
            return None  # suppress positive response
        dlc = len(resp_bytes)
        # Pad to 64 bytes (CAN-FD). DLC conveys the actual
        # response length; master reads only `data[:dlc]`.
        resp_padded = (resp_bytes + b'\x00' * 64)[:64]
        return {
            'id': UDS_PHYSICAL_RESPONSE_ID,
            'data': resp_padded,
            'dlc': dlc,
        }

    # ---- NMT ---------------------------------------------------------

    def heartbeat_byte(self) -> int:
        return {
            'Booting':       0x00,
            'Stopped':       0x04,
            'Operational':   0x05,
            'PreOperational': 0x7F,
        }[self.nmt_state]

    def handle_nmt(self, frame: dict) -> str | None:
        if frame['id'] != NMT_COB_ID or frame['dlc'] < 2:
            return None
        cmd, node = frame['data'][0], frame['data'][1]
        if node not in (NODE_ID, 0):  # 0 = broadcast
            return None
        if cmd == 0x01:
            self.nmt_state = 'Operational'
        elif cmd == 0x02:
            self.nmt_state = 'Stopped'
        elif cmd == 0x80:
            self.nmt_state = 'PreOperational'
        else:
            return None  # 0x81/0x82 reset not supported in Phase 1
        return self.nmt_state

    # ---- SDO ---------------------------------------------------------

    def handle_sdo(self, frame: dict) -> dict | None:
        if frame['id'] != SDO_RX_COB_ID or frame['dlc'] < 8:
            return None
        d = frame['data']
        cmd = d[0]
        idx = struct.unpack_from('<H', d, 1)[0]
        sub = d[3]
        kind = cmd & 0xE0
        if kind == SDO_CMD_DOWNLOAD:
            return self._sdo_download(cmd, idx, sub, d)
        if kind == SDO_CMD_DOWNLOAD_SEG:
            return self._sdo_download_seg(cmd, d)
        if kind == SDO_CMD_UPLOAD:
            return self._sdo_upload_init(idx, sub)
        if kind == SDO_CMD_UPLOAD_SEG:
            return self._sdo_upload_seg(cmd)
        if kind == SDO_CMD_ABORT:
            # Client abort clears any in-flight transfer on either side.
            self.seg_len = 0
            self.dl_seg_len = 0
            return None
        return self._make_abort(0, 0, SDO_ABORT_INVALID_COMMAND)

    def _sdo_download(self, cmd: int, idx: int, sub: int, d: bytes) -> dict:
        e = cmd & 0x02
        s = cmd & 0x01
        if e and s:
            # Expedited Initiate Download.
            n = (cmd & SDO_N_MASK) >> 2
            if n > 3:
                return self._make_abort(idx, sub, SDO_ABORT_INVALID_COMMAND)
            num_bytes = 4 - n
            return self._apply_download(idx, sub, d[4:4 + num_bytes])
        if (not e) and s:
            # Segmented Initiate Download (0x21). Size in bytes 4-5.
            size = struct.unpack_from('<H', d, 4)[0]
            if size < 5 or size > SDO_MAX_SEGMENTED_SIZE:
                return self._make_abort(idx, sub, SDO_ABORT_INVALID_COMMAND)
            # Clear upload state to avoid races.
            self.seg_len = 0
            self.dl_seg_buf = bytearray(SDO_MAX_SEGMENTED_SIZE)
            self.dl_seg_len = size
            self.dl_seg_offset = 0
            self.dl_seg_toggle = 0
            self.dl_seg_idx = idx
            self.dl_seg_sub = sub
            return self._make_dl_ok()
        return self._make_abort(idx, sub, SDO_ABORT_INVALID_COMMAND)

    def _sdo_download_seg(self, cmd: int, d: bytes) -> dict:
        if self.dl_seg_len == 0:
            return self._make_abort(0, 0, SDO_ABORT_TOGGLE_BIT_NOT_ALTERED)
        toggle = (cmd >> 4) & 0x01
        if toggle != self.dl_seg_toggle:
            self.dl_seg_len = 0
            return self._make_abort(0, 0, SDO_ABORT_TOGGLE_BIT_NOT_ALTERED)
        n = (cmd >> 1) & 0x03
        last = (cmd & 0x01) != 0
        num_data = 7 - n
        new_offset = self.dl_seg_offset + num_data
        if new_offset > self.dl_seg_len:
            self.dl_seg_len = 0
            return self._make_abort(0, 0, SDO_ABORT_INVALID_COMMAND)
        self.dl_seg_buf[self.dl_seg_offset:new_offset] = d[1:1 + num_data]
        self.dl_seg_offset = new_offset
        self.dl_seg_toggle ^= 1
        if last:
            if new_offset != self.dl_seg_len:
                self.dl_seg_len = 0
                return self._make_abort(0, 0, SDO_ABORT_INVALID_COMMAND)
            idx = self.dl_seg_idx
            sub = self.dl_seg_sub
            value = bytes(self.dl_seg_buf[:self.dl_seg_len])
            self.dl_seg_len = 0
            return self._apply_download(idx, sub, value)
        return self._make_dl_ok()

    def _apply_download(self, idx: int, sub: int, value: bytes) -> dict:
        """Dispatch a fully-received download value to the OD."""
        if idx in (0x1000, 0x1001) or (idx == 0x1018 and sub in (0, 1, 2, 3, 4)):
            return self._make_abort(idx, sub, SDO_ABORT_READ_ONLY)
        if idx == 0x1017 and sub == 0:
            if len(value) != 2:
                return self._make_abort(idx, sub, SDO_ABORT_LENGTH_MISMATCH)
            self.od[0x1017][0] = value
            return self._make_dl_ok()
        if idx == 0x2F00 and sub == 0:
            self._dispatch_uds(value)
            return self._make_dl_ok()
        return self._make_abort(idx, sub, SDO_ABORT_OBJECT_DOES_NOT_EXIST)

    def _sdo_upload_init(self, idx: int, sub: int) -> dict:
        if idx == 0x2F00 and sub == 0:
            value = self.last_response
        elif idx in self.od and sub in self.od[idx]:
            value = self.od[idx][sub]
        else:
            return self._make_abort(idx, sub, SDO_ABORT_OBJECT_DOES_NOT_EXIST)
        return self._make_ul_response(idx, sub, value)

    def _sdo_upload_seg(self, cmd: int) -> dict:
        toggle = (cmd >> 4) & 0x01
        if self.seg_len == 0:
            return self._make_abort(0, 0, SDO_ABORT_TOGGLE_BIT_NOT_ALTERED)
        if toggle != self.seg_toggle:
            self.seg_len = 0
            self.seg_offset = 0
            return self._make_abort(0, 0, SDO_ABORT_TOGGLE_BIT_NOT_ALTERED)
        offset = self.seg_offset
        chunk = min(7, self.seg_len - offset)
        seg = bytes(self.seg_buf[offset:offset + chunk])
        last = (offset + chunk) == self.seg_len
        self.seg_offset += chunk
        self.seg_toggle ^= 1
        if last:
            self.seg_len = 0
            self.seg_offset = 0
        n = 7 - chunk
        c = 1 if last else 0
        b0 = 0xA0 | (toggle << 4) | (n << 1) | c
        return self._make_frame(SDO_TX_COB_ID, bytes([b0]) + seg + bytes(7 - chunk))

    def _make_ul_response(self, idx: int, sub: int, value: bytes) -> dict:
        payload = bytearray(8)
        payload[1:3] = struct.pack('<H', idx)
        payload[3] = sub
        if len(value) <= 4:
            cmds = {1: 0x8F, 2: 0x8E, 3: 0x8D, 4: 0x8C}
            payload[0] = cmds[len(value)]
            payload[4:4 + len(value)] = value
            return self._make_frame(SDO_TX_COB_ID, bytes(payload))
        # Segmented Initiate
        payload[0] = 0x82
        payload[4] = len(value)
        payload[5] = 0
        self.seg_buf[:len(value)] = value
        self.seg_len = len(value)
        self.seg_offset = 0
        self.seg_toggle = 0
        return self._make_frame(SDO_TX_COB_ID, bytes(payload))

    def _make_dl_ok(self) -> dict:
        # Initiate Download Response: byte 0 = 0x60 (scs = 0b011),
        # then index, sub, and 4 bytes of zero.
        return self._make_frame(SDO_TX_COB_ID, b'\x60\x00\x00\x00\x00\x00\x00\x00')

    def _make_abort(self, idx: int, sub: int, code: int) -> dict:
        idx_b = struct.pack('<H', idx)
        code_b = struct.pack('<I', code)
        payload = b'\x80' + idx_b + bytes([sub]) + code_b
        return self._make_frame(SDO_TX_COB_ID, payload)

    def _make_frame(self, cob_id: int, data: bytes) -> dict:
        return {'id': cob_id, 'data': bytes(data), 'dlc': len(data)}

    # ---- UDS ---------------------------------------------------------

    def _dispatch_uds(self, payload: bytes) -> None:
        if not payload:
            self.last_response = bytes([0x7F, 0, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sid = payload[0]
        # Phase 5c: tx_disabled flag — 0x28 0x03 (and the other
        # "disable" subfuncs) set this; non-0x28 requests return
        # 0x22 while disabled. The 0x28 0x00 enable bypasses
        # the check.
        if self.uds_tx_disabled and not (sid == SID_CC and len(payload) >= 2
                                          and payload[1] == 0x00):
            self.last_response = bytes([0x7F, sid, NRC_CONDITIONS_NOT_CORRECT])
            return
        rest = payload[1:]
        handlers = {
            SID_DSC:   self._uds_dsc,
            SID_ER:    self._uds_er,
            SID_CDI:   self._uds_cdi,
            SID_RDTCI: self._uds_rdtci,
            SID_RDBI:  self._uds_rdbi,
            SID_WDBI:  self._uds_wdbi,
            SID_SA:    self._uds_sa,
            SID_CC:    self._uds_cc,
            SID_RC:    self._uds_rc,
            SID_TP:    self._uds_tp,
            SID_RD:    self._uds_rd,
            SID_TD:    self._uds_td,
            SID_RTE:   self._uds_rte,
        }
        h = handlers.get(sid)
        if h is None:
            self.last_response = bytes([0x7F, sid, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        h(rest)

    def _uds_dsc(self, p: bytes) -> None:
        if len(p) != 1:
            self.last_response = bytes([0x7F, SID_DSC, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        if sub == 0x01:
            self.uds_session = 0x01
            self.uds_security = 0  # session change invalidates security
            self.sa_fail_count = 0
            self.last_response = bytes([SID_DSC + 0x40, 0x01])
        elif sub == 0x02:
            if self.uds_security == 0:
                self.last_response = bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED])
                return
            self.uds_session = 0x02
            self.uds_security = 0
            self.sa_fail_count = 0
            self.last_response = bytes([SID_DSC + 0x40, 0x02])
        elif sub == 0x03:
            if self.uds_security == 0:
                self.last_response = bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED])
                return
            self.uds_session = 0x03
            self.uds_security = 0
            self.sa_fail_count = 0
            self.last_response = bytes([SID_DSC + 0x40, 0x03])
        else:
            self.last_response = bytes([0x7F, SID_DSC, NRC_SUB_FUNC_NOT_SUPPORTED])

    def _uds_er(self, p: bytes) -> None:
        if len(p) != 1 or p[0] not in (0x01, 0x03):
            self.last_response = bytes([0x7F, SID_ER, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        self.sa_fail_count = 0
        self.last_response = bytes([SID_ER + 0x40, p[0]])

    def _uds_cdi(self, p: bytes) -> None:
        if len(p) != 3:
            self.last_response = bytes([0x7F, SID_CDI, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        group = (p[0] << 16) | (p[1] << 8) | p[2]
        group_high_nibble = (p[0] & 0xF0)
        if group == 0xFFFFFF:
            # Clear all (matches Rust clear_group 0xFFFFFF).
            self.dtcs.clear()
        else:
            # Clear by group high nibble (matches Rust clear_group logic).
            self.dtcs = [(c, s) for c, s in self.dtcs
                         if ((c >> 16) & 0xF0) != group_high_nibble]
        self.last_response = bytes([SID_CDI + 0x40])

    def _uds_rdtci(self, p: bytes) -> None:
        if len(p) < 2 or p[0] != 0x02:
            self.last_response = bytes([0x7F, SID_RDTCI, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        status_mask = p[1]
        matching = [(c, s) for c, s in self.dtcs if s & status_mask]
        count = len(matching)
        out = bytearray(5 + count * 4)
        out[0] = SID_RDTCI + 0x40
        out[1] = 0x02
        out[2] = 0xFE  # statusAvailability
        out[3] = (count >> 8) & 0xFF
        out[4] = count & 0xFF
        for i, (code, st) in enumerate(matching):
            off = 5 + i * 4
            code_bytes = code.to_bytes(3, 'big')
            out[off]     = code_bytes[0]
            out[off + 1] = code_bytes[1]
            out[off + 2] = code_bytes[2]
            out[off + 3] = st
        self.last_response = bytes(out)

    def _uds_rdbi(self, p: bytes) -> None:
        if len(p) != 2:
            self.last_response = bytes([0x7F, SID_RDBI, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        did = struct.unpack_from('<H', p, 0)[0]
        if did == 0xF186:
            self.last_response = bytes([SID_RDBI + 0x40, 0x86, 0xF1, self.uds_session])
        else:
            self.last_response = bytes([0x7F, SID_RDBI, NRC_REQUEST_OUT_OF_RANGE])

    def _uds_wdbi(self, p: bytes) -> None:
        if len(p) < 2:
            self.last_response = bytes([0x7F, SID_WDBI, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        did = struct.unpack_from('<H', p, 0)[0]
        if did == 0xF180:
            if len(p) != 14:  # 2 ID + 12 data
                self.last_response = bytes([0x7F, SID_WDBI, NRC_INCORRECT_MESSAGE_LENGTH])
                return
            masks = [
                struct.unpack_from('<I', p, 2)[0],
                struct.unpack_from('<I', p, 6)[0],
                struct.unpack_from('<I', p, 10)[0],
            ]
            if any(m == 0 for m in masks):
                self.last_response = bytes([0x7F, SID_WDBI, NRC_REQUEST_OUT_OF_RANGE])
                return
            self.key_masks = masks
            self.last_response = bytes([SID_WDBI + 0x40, 0x80, 0xF1])
        else:
            self.last_response = bytes([0x7F, SID_WDBI, NRC_REQUEST_OUT_OF_RANGE])

    def _uds_sa(self, p: bytes) -> None:
        if not p:
            self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        # ISO 14229-1 §7.22: lockout check (matches Rust dispatch_0x27).
        if self.sa_fail_count >= 3:
            self.last_response = bytes([0x7F, SID_SA, NRC_EXCEEDED_NUMBER_OF_ATTEMPTS])
            return
        # Per-SAL session gate (matches Rust sal_session_allowed).
        if sub in (0x01, 0x02) and self.uds_session not in (0x01, 0x02):
            self.last_response = bytes(
                [0x7F, SID_SA, NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION])
            return
        if sub in (0x03, 0x04) and self.uds_session not in (0x02, 0x03):
            self.last_response = bytes(
                [0x7F, SID_SA, NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION])
            return
        if sub in (0x05, 0x06) and self.uds_session != 0x03:
            self.last_response = bytes(
                [0x7F, SID_SA, NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION])
            return
        # Helper: zero seed response (already unlocked).
        zero_seed = lambda s: bytes([SID_SA + 0x40, s]) + b'\x00' * 16
        if sub == 0x01:
            if self.uds_security != 0:
                self.last_response = zero_seed(0x01)
                return
            self.last_response = bytes([SID_SA + 0x40, 0x01]) + SEED_16
        elif sub == 0x02:
            if len(p) != 17:
                self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
                return
            if p[1:17] == _aes_key(SEED_16, self.key_masks[0]):
                self.uds_security = 1
                self.sa_fail_count = 0
                self.last_response = bytes([SID_SA + 0x40, 0x02])
            else:
                self.sa_fail_count += 1
                self.last_response = bytes([0x7F, SID_SA, NRC_INVALID_KEY])
        elif sub == 0x03:
            if self.uds_security >= 2:
                self.last_response = zero_seed(0x03)
                return
            self.last_response = bytes([SID_SA + 0x40, 0x03]) + SEED_16
        elif sub == 0x04:
            if len(p) != 17:
                self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
                return
            if p[1:17] == _aes_key(SEED_16, self.key_masks[1]):
                self.uds_security = 2
                self.sa_fail_count = 0
                self.last_response = bytes([SID_SA + 0x40, 0x04])
            else:
                self.sa_fail_count += 1
                self.last_response = bytes([0x7F, SID_SA, NRC_INVALID_KEY])
        elif sub == 0x05:
            if self.uds_security >= 3:
                self.last_response = zero_seed(0x05)
                return
            self.last_response = bytes([SID_SA + 0x40, 0x05]) + SEED_16
        elif sub == 0x06:
            if len(p) != 17:
                self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
                return
            if p[1:17] == _aes_key(SEED_16, self.key_masks[2]):
                self.uds_security = 3
                self.sa_fail_count = 0
                self.last_response = bytes([SID_SA + 0x40, 0x06])
            else:
                self.sa_fail_count += 1
                self.last_response = bytes([0x7F, SID_SA, NRC_INVALID_KEY])
        else:
            self.last_response = bytes([0x7F, SID_SA, NRC_SUB_FUNC_NOT_SUPPORTED])

    def _uds_cc(self, p: bytes) -> None:
        # 0x28 CommunicationControl. [0x28, subfunc, network_type]
        if len(p) != 2:
            self.last_response = bytes([0x7F, SID_CC, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        if sub == 0x00:
            self.uds_tx_disabled = False
            self.uds_rx_disabled = False
        elif sub == 0x01:
            self.uds_tx_disabled = True
            self.uds_rx_disabled = False
        elif sub == 0x02:
            self.uds_tx_disabled = False
            self.uds_rx_disabled = True
        elif sub == 0x03:
            self.uds_tx_disabled = True
            self.uds_rx_disabled = True
        else:
            self.last_response = bytes([0x7F, SID_CC, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        self.last_response = bytes([SID_CC + 0x40, sub])

    def _uds_rc(self, p: bytes) -> None:
        # 0x31 RoutineControl. [0x31, subfunc, rid_hi, rid_lo, ...]
        if len(p) < 3:
            self.last_response = bytes([0x7F, SID_RC, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        rid = (p[1] << 8) | p[2] if len(p) >= 3 else 0
        if sub == 0x01 and rid == 0xFF00:
            # startRoutine 0xFF00 = checkProgrammingDependencies (stub)
            self.last_response = bytes([SID_RC + 0x40, sub, p[1], p[2]])
        elif sub == 0x03 and rid == 0xF001:
            # requestRoutineResults 0xF001 = checkPreConditions → 1 byte "0x00 OK"
            self.last_response = bytes([SID_RC + 0x40, sub, p[1], p[2], 0x00])
        elif sub == 0x01 and self.uds_session != 0x02 and rid == 0xFF00:
            # Wrong session — 0x7E
            self.last_response = bytes([0x7F, SID_RC,
                                        NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION])
        else:
            self.last_response = bytes([0x7F, SID_RC, NRC_REQUEST_OUT_OF_RANGE])

    def _uds_tp(self, p: bytes) -> None:
        if not p:
            self.last_response = bytes([0x7F, SID_TP, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        if sub & 0x7F == 0x00:
            if sub & 0x80:
                self.last_response = b''
            else:
                self.last_response = bytes([SID_TP + 0x40, 0x00])
        else:
            self.last_response = bytes([0x7F, SID_TP, NRC_SUB_FUNC_NOT_SUPPORTED])

    def _uds_rd(self, p: bytes) -> None:
        if self.uds_session != 0x02:
            self.last_response = bytes([0x7F, SID_RD, NRC_CONDITIONS_NOT_CORRECT])
            return
        if len(p) != 5 or p[0] != 0x00:
            self.last_response = bytes([0x7F, SID_RD, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        size = struct.unpack_from('<I', p, 1)[0]
        if size == 0 or size > APP_SIZE:
            self.last_response = bytes([0x7F, SID_RD, NRC_REQUEST_OUT_OF_RANGE])
            return
        if self.ota_state != 'Idle':
            self.last_response = bytes([0x7F, SID_RD, NRC_CONDITIONS_NOT_CORRECT])
            return
        self.ota_state = 'Receiving'
        self.ota_total = size
        self.ota_remaining = size
        self.ota_offset = APP_START
        self.ota_crc32 = 0xFFFF_FFFF
        self.ota_next_block_seq = 1
        self.last_response = bytes([0x74, 0x00, 0x00, 0x02])

    def _uds_td(self, p: bytes) -> None:
        if self.uds_session != 0x02:
            self.last_response = bytes([0x7F, SID_TD, NRC_CONDITIONS_NOT_CORRECT])
            return
        if self.ota_state != 'Receiving':
            self.last_response = bytes([0x7F, SID_TD, NRC_CONDITIONS_NOT_CORRECT])
            return
        if len(p) != 3:
            self.last_response = bytes([0x7F, SID_TD, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        seq = p[0]
        if seq != self.ota_next_block_seq:
            self.last_response = bytes([0x7F, SID_TD, NRC_WRONG_BLOCK_SEQUENCE_NUMBER])
            return
        self.ota_next_block_seq = (seq + 1) & 0xFF
        for b in p[1:3]:
            self.ota_crc32 = crc32_update(self.ota_crc32, b)
        self.ota_offset += 2
        self.ota_remaining = max(0, self.ota_remaining - 2)
        self.last_response = bytes([0x76, seq])

    def _uds_rte(self, p: bytes) -> None:
        if self.uds_session != 0x02:
            self.last_response = bytes([0x7F, SID_RTE, NRC_CONDITIONS_NOT_CORRECT])
            return
        if self.ota_state != 'Receiving':
            self.last_response = bytes([0x7F, SID_RTE, NRC_CONDITIONS_NOT_CORRECT])
            return
        if p:
            self.last_response = bytes([0x7F, SID_RTE, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        self.ota_crc32 ^= 0xFFFF_FFFF
        self.ota_state = 'Done'
        self.last_response = bytes([0x77])


# ---- Master driver (talks the same way a real CANopen master would)

class TestFailure(Exception):
    pass


def _assert(cond: bool, msg: str) -> None:
    if not cond:
        raise TestFailure(msg)


def assert_bytes(actual, expected, msg: str = "") -> None:
    a = bytes(actual)
    e = bytes(expected)
    if a != e:
        raise TestFailure(
            f"bytes mismatch: {a.hex(' ')} != {e.hex(' ')}  ({msg})"
        )


# ---- Bus abstraction (SimBus for in-process tests; CanBus for real CAN)

class Bus:
    """Send/receive a single CAN frame. Two implementations:

    * `SimBus` — wraps a `FirmwareEmulator`, returns frames in
      the same process. Default for the smoke test; no hardware
      needed.
    * `CanBus` — wraps `python-can`, sends/receives real frames
      on a real CAN interface (e.g. `socketcan:vcan0`, `pcan:PCAN_USBBUS1`).
      Used with `--live IFACE`.
    """

    def send(self, frame: dict) -> dict | None:
        """Send a frame, return the next matching response frame
        or `None` if no response is expected / received.

        For SDO request frames this returns the server's SDO
        response. For NMT frames this returns None (NMT has no
        response on the wire).
        """
        raise NotImplementedError

    def recv(self, timeout: float = 1.0) -> dict | None:
        """Receive any one frame from the bus, or None on timeout.
        Mainly used for heartbeat / non-SDO observations in live
        mode; sim mode delivers these inline via `send()`."""
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class SimBus(Bus):
    """In-process Bus backed by a `FirmwareEmulator`. Responses
    are returned synchronously from the same thread."""

    def __init__(self, fw: FirmwareEmulator) -> None:
        self.fw = fw

    def send(self, frame: dict) -> dict | None:
        if frame['id'] == SDO_RX_COB_ID:
            # Phase 6 backwards-compat: legacy SDO writes still
            # dispatch UDS via handle_sdo. New UDS path below.
            return self.fw.handle_sdo(frame)
        if frame['id'] in (UDS_FUNCTIONAL_REQUEST_ID, UDS_PHYSICAL_REQUEST_ID):
            return self.fw.handle_uds(frame)
        if frame['id'] == NMT_COB_ID:
            return self.fw.handle_nmt(frame)
        return None

    def recv(self, timeout: float = 1.0) -> dict | None:
        # Sim mode never spontaneously produces frames; the
        # emulator only responds on explicit send(). This is
        # here for API symmetry with CanBus.
        return None

    def close(self) -> None:
        pass


class CanBus(Bus):
    """Real CAN bus via `python-can`. Sends frames on the wire,
    waits for the matching server response.

    For SDO requests on `SDO_RX_COB_ID`, the response is the next
    frame on `SDO_TX_COB_ID`. For NMT, there is no response — we
    just send and return None. Heartbeats from `HEARTBEAT_COB_ID`
    are not picked up here; use `recv()` for those.
    """

    def __init__(self, interface: str, channel: str,
                 bitrate: int = 500_000, timeout: float = 1.0) -> None:
        try:
            import can
        except ImportError as e:
            raise TestFailure(
                f"python-can not installed (run `pip install python-can`); "
                f"needed for --live mode ({e})"
            )
        self._timeout = timeout
        try:
            self.bus = can.interface.Bus(
                interface=interface, channel=channel, bitrate=bitrate
            )
        except (can.CanError, OSError) as e:
            raise TestFailure(
                f"could not open CAN bus {interface}:{channel} @ {bitrate} bps: {e}. "
                f"For local testing without hardware, run `sudo "
                f"scripts/setup_vcan.sh` to bring up a vcan0 interface."
            )

    def send(self, frame: dict) -> dict | None:
        import can
        msg = can.Message(
            arbitration_id=frame['id'],
            data=list(frame['data']),
            is_extended_id=False,
        )
        try:
            self.bus.send(msg, timeout=self._timeout)
        except can.CanError as e:
            raise TestFailure(f"CAN send failed: {e}")
        # Decide which COB-ID to listen on for the response.
        if frame['id'] == SDO_RX_COB_ID:
            expect = SDO_TX_COB_ID
        elif frame['id'] == NMT_COB_ID:
            return None
        else:
            expect = None
        if expect is None:
            return None
        # Wait for the matching response.
        deadline = time.monotonic() + self._timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            r = self.bus.recv(timeout=remaining)
            if r is None:
                return None
            if r.arbitration_id == expect:
                return {
                    'id': r.arbitration_id,
                    'data': bytes(r.data),
                    'dlc': r.dlc,
                }
            # Skip frames we don't care about (heartbeats, etc.)

    def recv(self, timeout: float = 1.0) -> dict | None:
        r = self.bus.recv(timeout=timeout)
        if r is None:
            return None
        return {
            'id': r.arbitration_id,
            'data': bytes(r.data),
            'dlc': r.dlc,
        }

    def close(self) -> None:
        self.bus.shutdown()


class MasterDriver:
    """Builds SDO + UDS request frames, parses responses the
    same way a real master would. Asserts on every response
    byte. Phase 6: UDS frames go directly on 0x7E0/0x7E8
    (independent of CANopen SDO).

    Talks to a `Bus` (SimBus or CanBus) for actual frame transport;
    the frame builders are identical in both modes."""

    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        # Exposed for tests that want to poke at internal state
        # (e.g. trigger a segmented upload to check abort-on-replay).
        # Sim-only; CanBus returns None.
        self.fw = bus.fw if isinstance(bus, SimBus) else None

    # ---- low-level frame builders -----------------------------------

    def _frame_uds(self, payload: bytes) -> dict:
        """Phase 6: build a raw UDS frame on 0x7E0 (CAN-FD,
        supports up to 64 bytes payload)."""
        assert 1 <= len(payload) <= 64, f"UDS frame must be 1-64 bytes, got {len(payload)}"
        return {
            'id': UDS_PHYSICAL_REQUEST_ID,
            'data': (payload + b'\x00' * 64)[:64],
            'dlc': len(payload),
        }

    def send_uds(self, payload: bytes) -> bytes | None:
        """Phase 6: send a UDS request, return the response
        bytes (None on no response, e.g. 0x3E 0x80 suppress)."""
        frame = self._frame_uds(payload)
        resp = self.bus.send(frame)
        if resp is None or resp.get('dlc', 0) == 0:
            return None
        return bytes(resp['data'][:resp['dlc']])

    def _frame_sdo_write(self, idx: int, sub: int, value: bytes) -> dict:
        assert 1 <= len(value) <= 4
        n = 4 - len(value)
        cmds = {1: 0x2F, 2: 0x2B, 3: 0x27, 4: 0x23}
        payload = bytearray(8)
        payload[0] = cmds[len(value)]
        payload[1:3] = struct.pack('<H', idx)
        payload[3] = sub
        payload[4:4 + len(value)] = value
        return {'id': SDO_RX_COB_ID, 'data': bytes(payload), 'dlc': 8}

    def _frame_sdo_read(self, idx: int, sub: int) -> dict:
        return {
            'id': SDO_RX_COB_ID,
            'data': bytes([SDO_CMD_UPLOAD]) + struct.pack('<H', idx) + bytes([sub, 0, 0, 0, 0]),
            'dlc': 8,
        }

    def _frame_sdo_seg(self, toggle: int) -> dict:
        return {
            'id': SDO_RX_COB_ID,
            'data': bytes([SDO_CMD_UPLOAD_SEG | (toggle << 4)]) + bytes(7),
            'dlc': 8,
        }

    def _frame_sdo_dl_initiate(self, idx: int, sub: int, size: int) -> dict:
        """0x21 Initiate Download Request (segmented, with size)."""
        payload = bytearray(8)
        payload[0] = 0x21
        payload[1:3] = struct.pack('<H', idx)
        payload[3] = sub
        payload[4:6] = struct.pack('<H', size)
        return {'id': SDO_RX_COB_ID, 'data': bytes(payload), 'dlc': 8}

    def _frame_sdo_dl_seg(self, toggle: int, data: bytes, last: bool) -> dict:
        """0x00 Download Segment Request. data is ≤ 7 bytes; the
        segment frame carries num_data_bytes worth of payload and
        c=1 iff last."""
        assert len(data) <= 7
        n = 7 - len(data)
        b0 = SDO_CMD_DOWNLOAD_SEG | (toggle << 4) | (n << 1) | (1 if last else 0)
        payload = bytearray(8)
        payload[0] = b0
        payload[1:1 + len(data)] = data
        return {'id': SDO_RX_COB_ID, 'data': bytes(payload), 'dlc': 8}

    def _frame_nmt(self, cmd: int, node: int = NODE_ID) -> dict:
        return {'id': NMT_COB_ID, 'data': bytes([cmd, node]), 'dlc': 2}

    # ---- high-level ops ----------------------------------------------

    def nmt(self, cmd: int) -> str:
        new_state = self.bus.send(self._frame_nmt(cmd))
        _assert(new_state is not None, f"NMT cmd 0x{cmd:02x} ignored")
        return new_state

    def sdo_write(self, idx: int, sub: int, value: bytes) -> bool:
        resp = self.bus.send(self._frame_sdo_write(idx, sub, value))
        _assert(resp is not None, "no SDO response")
        return resp['data'][0] == 0x60

    def sdo_write_long(self, idx: int, sub: int, value: bytes) -> bool:
        """SDO download for values that exceed the expedited 4-byte
        ceiling. Uses segmented transfer (0x21 Initiate + one or
        more 0x00 Segments). Extended to 64 bytes for AES-128
        sendKey (18 bytes) and OTA RequestDownload.
        """
        assert 5 <= len(value) <= 64
        size = len(value)
        init = self.bus.send(self._frame_sdo_dl_initiate(idx, sub, size))
        _assert(init is not None, "no SDO initiate response")
        _assert(init['data'][0] == 0x60,
                f"expected 0x60 initiate response, got 0x{init['data'][0]:02x}")
        toggle = 0
        sent = 0
        while sent < size:
            chunk = min(7, size - sent)
            last = (sent + chunk) == size
            seg = self.bus.send(
                self._frame_sdo_dl_seg(toggle, value[sent:sent + chunk], last)
            )
            _assert(seg is not None, "no SDO segment response")
            _assert(seg['data'][0] == 0x60,
                    f"expected 0x60 segment response, got 0x{seg['data'][0]:02x}")
            toggle ^= 1
            sent += chunk
        return True

    def uds_dispatch_raw(self, payload: bytes) -> None:
        """Drive the UDS dispatcher directly, bypassing the SDO
        download layer. Sim mode only — useful for tests that want
        to verify UDS logic without exercising SDO framing."""
        _assert(self.fw is not None,
                "uds_dispatch_raw is sim-only; use sdo_write / sdo_write_long")
        self.fw._dispatch_uds(payload)

    def sdo_read(self, idx: int, sub: int) -> bytes:
        """SDO read handling both expedited (1–4 bytes) and
        segmented (5–7 bytes) responses. Returns the value bytes."""
        init = self.bus.send(self._frame_sdo_read(idx, sub))
        _assert(init is not None, "no SDO response")
        cmd = init['data'][0]
        if (cmd & 0xE0) == 0x80 and (cmd & 0x08):
            # Expedited Upload Response (e=1, s=1, n=bits 0-1).
            n = cmd & 0x03
            size = 4 - n
            return init['data'][4:4 + size]
        if cmd == 0x82:
            # Segmented Initiate Upload Response (s=1, e=0).
            size = struct.unpack_from('<H', init['data'], 4)[0]
            result = bytearray()
            toggle = 0
            while True:
                seg = self.bus.send(self._frame_sdo_seg(toggle))
                _assert(seg is not None, "no Upload Segment response")
                b0 = seg['data'][0]
                _assert((b0 & 0xE0) == 0xA0,
                        f"expected segment SCS 0xA0, got 0x{b0:02x}")
                chunk = 7 - ((b0 >> 1) & 0x03)
                last = b0 & 0x01
                result += seg['data'][1:1 + chunk]
                toggle ^= 1
                if last:
                    break
            _assert(len(result) == size,
                    f"segmented: got {len(result)} bytes, expected {size}")
            return bytes(result)
        raise TestFailure(f"unexpected SDO response cmd 0x{cmd:02x}")


# ---- Scenarios -------------------------------------------------------

def s_heartbeat(bus: Bus) -> None:
    """After boot-up the firmware reports PreOperational (0x7F).

    Sim mode checks the firmware's heartbeat byte directly (the
    emulator doesn't send spontaneous frames). Live mode receives
    a heartbeat frame on COB-ID 0x701 and checks its data byte.
    """
    if isinstance(bus, SimBus):
        assert_bytes([bus.fw.heartbeat_byte()], [0x7F],
                     "boot heartbeat should be PreOperational")
    else:
        msg = bus.recv(timeout=2.0)
        _assert(msg is not None, "no heartbeat frame received within 2s")
        _assert(msg['id'] == HEARTBEAT_COB_ID,
                f"got frame on 0x{msg['id']:03x}, expected heartbeat COB-ID 0x{HEARTBEAT_COB_ID:03x}")
        assert_bytes(msg['data'][:1], b'\x7F',
                     "boot heartbeat should be PreOperational")


def s_sdo_basic(bus: Bus) -> None:
    """SDO read of 0x1000 (DeviceType) — 4-byte expedited, SCS=0x8C."""
    drv = MasterDriver(bus)
    val = drv.sdo_read(0x1000, 0)
    assert_bytes(val, b'\x00\x00\x00\x00', "DeviceType = 0")


def s_sdo_write_heartbeat(bus: Bus) -> None:
    """Write 0x1017.0 (HeartbeatProducerTime) to 250 ms, read back."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x1017, 0, struct.pack('<H', 250)), "write heartbeat OK")
    val = drv.sdo_read(0x1017, 0)
    assert_bytes(val, struct.pack('<H', 250), "heartbeat now 250ms")


def s_sdo_ro_rejected(bus: Bus) -> None:
    """Write to a read-only entry returns an abort (SDO abort code
    0x06010002 ReadOnly)."""
    drv = MasterDriver(bus)
    _assert(not drv.sdo_write(0x1000, 0, b'\x01\x00\x00\x00'), "write to RO should fail")


def s_uds_tp(bus: Bus) -> None:
    """TesterPresent (0x3E 0x00) → positive (0x7E 0x00) via SDO."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x2F00, 0, bytes([SID_TP, 0x00])), "TP write OK")
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_TP + 0x40, 0x00]), "TP response")


def s_uds_tp_suppress(bus: Bus) -> None:
    """TesterPresent (0x3E 0x80) with suppressPositiveResponse bit
    set → SDO response payload is empty (server stays silent)."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x2F00, 0, bytes([SID_TP, 0x80])), "TP suppress write OK")
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, b'', "TP suppress → empty response")


def s_uds_cc_disable(bus: Bus) -> None:
    """0x28 0x03 disableNormalCommunication → positive. Subsequent
    non-0x28 requests return 0x22 ConditionsNotCorrect."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x2F00, 0, bytes([SID_CC, 0x03, 0x01])), "CC 0x03 write OK")
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_CC + 0x40, 0x03]), "CC 0x03 response")
    # Now TP should get 0x22.
    drv.sdo_write(0x2F00, 0, bytes([SID_TP, 0x00]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_TP, NRC_CONDITIONS_NOT_CORRECT]),
                 "TP during disable → 0x22")


def s_uds_cc_enable_bypass(bus: Bus) -> None:
    """0x28 0x03 disables, then 0x28 0x00 re-enables — even
    while disabled, 0x28 0x00 must work (the unlock path)."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_CC, 0x03, 0x01]))
    drv.sdo_read(0x2F00, 0)  # consume
    # The unlock: 0x28 0x00 must work even though we're disabled.
    drv.sdo_write(0x2F00, 0, bytes([SID_CC, 0x00, 0x01]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_CC + 0x40, 0x00]), "CC 0x00 enable bypasses disable")
    # TP now works again.
    drv.sdo_write(0x2F00, 0, bytes([SID_TP, 0x00]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_TP + 0x40, 0x00]), "TP after re-enable")


def s_uds_rc_start(bus: Bus) -> None:
    """0x31 0x01 startRoutine RID 0xFF00 (in ProgrammingSession)
    → positive with 4-byte result header."""
    drv = MasterDriver(bus)
    # Get into ProgrammingSession: SecurityAccess then DSC 0x02.
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    drv.sdo_read(0x2F00, 0)  # consume seed
    drv.sdo_write_long(0x2F00, 0, bytes([SID_SA, 0x02]) + KEY)
    drv.sdo_read(0x2F00, 0)  # consume key-accepted
    drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x02]))
    drv.sdo_read(0x2F00, 0)  # consume dsc-accepted
    # Now start 0xFF00.
    drv.sdo_write(0x2F00, 0, bytes([SID_RC, 0x01, 0xFF, 0x00]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_RC + 0x40, 0x01, 0xFF, 0x00]),
                 "Routine start 0xFF00")


def s_uds_rc_result(bus: Bus) -> None:
    """0x31 0x03 requestRoutineResults RID 0xF001 → 1-byte result
    0x00 (pre-conditions met)."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_RC, 0x03, 0xF0, 0x01]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_RC + 0x40, 0x03, 0xF0, 0x01, 0x00]),
                 "Routine result 0xF001")


def s_uds_session_gate_nrc(bus: Bus) -> None:
    """0x22 read DID 0xF186 in any session (allowed). But writing
    a non-existent DID in any session returns 0x31. Try the
    non-gated case (0xF186) vs the gated case to verify the
    dispatcher routes by SID+subfunc."""
    drv = MasterDriver(bus)
    # Read 0xF186 — should work in Default.
    drv.sdo_write(0x2F00, 0, bytes([SID_RDBI, 0x86, 0xF1]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_RDBI + 0x40, 0x86, 0xF1, 0x01]),
                 "ReadDID 0xF186 in Default session")
    # Read unknown DID — 0x31.
    drv.sdo_write(0x2F00, 0, bytes([SID_RDBI, 0x00, 0x00]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_RDBI, NRC_REQUEST_OUT_OF_RANGE]),
                 "ReadDID unknown → 0x31")


def s_uds_aes_kat(bus: Bus) -> None:
    """AES-128 known-answer test: verify the Python AES matches
    the NIST reference vector (the Rust side tests its own)."""
    key = bytes(range(16))  # 00..0f
    pt = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff])
    ct = _aes_key(pt, key)
    expected = bytes([
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
        0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
    ])
    assert_bytes(ct, expected, "AES-128 KAT")

def s_uds_security_unlock(bus: Bus) -> None:
    """Full security dance via direct UDS transport:

    1. Try ProgrammingSession without unlock → 0x33.
    2. RequestSeed (0x27 0x01) → 18-byte seed response.
    3. SendKey (0x27 0x02 + 16-byte key) → positive.
    4. ProgrammingSession (0x10 0x02) → 0x50 0x02.
    """
    drv = MasterDriver(bus)
    # 1. Locked → ProgrammingSession denied.
    resp = drv.send_uds(bytes([SID_DSC, 0x02]))
    assert_bytes(resp, bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED]),
                 "DSC 0x02 without unlock")
    # 2. RequestSeed.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    assert_bytes(resp[:2], bytes([SID_SA + 0x40, 0x01]), "seed response header")
    _assert(len(resp) == 18, f"seed response {len(resp)} bytes (expected 18)")
    seed = resp[2:]
    # 3. SendKey (compute key from the real seed).
    expected_key = _aes_key(seed, KEY_MASK_SAL1)
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + expected_key)
    assert_bytes(resp, bytes([SID_SA + 0x40, 0x02]), "key accepted")
    # 4. ProgrammingSession.
    resp = drv.send_uds(bytes([SID_DSC, 0x02]))
    assert_bytes(resp, bytes([SID_DSC + 0x40, 0x02]), "DSC 0x02 after unlock")


def s_uds_programming_needs_sal(bus: Bus) -> None:
    """0x10 0x02 ProgrammingSession without unlock → 0x33
    SecurityAccessDenied. Verifies the SAL gate on session
    transitions."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x02]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED]),
                 "ProgrammingSession without unlock → 0x33")


# ---- Phase 6: UDS raw transport scenarios (independent of CANopen) ----

def s_uds_transport_session(bus: Bus) -> None:
    """Phase 6: UDS request on 0x7E0 (raw transport, no SDO
    tunnel) returns the same positive response as the
    legacy 0x2F00 path. Verifies the can_id routing."""
    drv = MasterDriver(bus)
    resp = drv.send_uds(bytes([SID_TP, 0x00]))
    assert_bytes(resp, bytes([SID_TP + 0x40, 0x00]),
                 "TP via raw UDS 0x7E0")


def s_uds_transport_did(bus: Bus) -> None:
    """Phase 6: ReadDataByIdentifier on 0x7E0 returns
    0xF186 ActiveDiagSession via raw transport."""
    drv = MasterDriver(bus)
    resp = drv.send_uds(bytes([SID_RDBI, 0x86, 0xF1]))
    assert_bytes(resp, bytes([SID_RDBI + 0x40, 0x86, 0xF1, 0x01]),
                 "ReadDID 0xF186 via raw UDS")


def s_uds_transport_security_full(bus: Bus) -> None:
    """Phase 6: Full SecurityAccess dance via raw UDS
    (RequestSeed → SendKey → ProgrammingSession)."""
    drv = MasterDriver(bus)
    # RequestSeed
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    _assert(len(resp) == 18, f"seed response {len(resp)} bytes (expected 18)")
    seed = resp[2:]
    # SendKey (compute key from seed dynamically).
    expected_key = _aes_key(seed, KEY_MASK_SAL1)
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + expected_key)
    assert_bytes(resp, bytes([SID_SA + 0x40, 0x02]), "key accepted")
    # ProgrammingSession
    resp = drv.send_uds(bytes([SID_DSC, 0x02]))
    assert_bytes(resp, bytes([SID_DSC + 0x40, 0x02]), "DSC 0x02 after unlock")


def s_uds_transport_wrong_id(bus: Bus) -> None:
    """Phase 6: Unknown DID via raw UDS → 0x31 RequestOutOfRange.
    Verifies the dispatcher runs in raw transport mode."""
    drv = MasterDriver(bus)
    resp = drv.send_uds(bytes([SID_RDBI, 0xAB, 0xCD]))
    assert_bytes(resp, bytes([0x7F, SID_RDBI, NRC_REQUEST_OUT_OF_RANGE]),
                 "ReadDID unknown via raw UDS → 0x31")


def s_uds_transport_no_sdo_dependency(bus: Bus) -> None:
    """Phase 6 regression guard: send a UDS request on 0x7E0
    and verify the response COB-ID is 0x7E8 (NOT 0x581 which
    was the SDO tunnel). Confirms the can_id routing is
    really independent of the legacy SDO path."""
    drv = MasterDriver(bus)
    raw_resp = drv.bus.send(drv._frame_uds(bytes([SID_TP, 0x00])))
    if raw_resp is None:
        raise TestFailure("UDS transport: no response")
    if raw_resp['id'] != UDS_PHYSICAL_RESPONSE_ID:
        raise TestFailure(
            f"UDS response on COB-ID 0x{raw_resp['id']:03X} "
            f"(expected 0x{UDS_PHYSICAL_RESPONSE_ID:03X} = 0x7E8) — "
            f"still using SDO tunnel?")


def s_uds_dtc_basic(bus: Bus) -> None:
    """DTC read/clear cycle: inject → read → clear → read zero."""
    drv = MasterDriver(bus)
    # Inject a DTC directly into the emulator (sim mode only; in live
    # mode DTCs are triggered by the CAN timeout monitor in
    # canopen_task, which takes 10 s).
    if isinstance(bus, SimBus):
        bus.fw.dtcs.append((0x030100, 0x05))
    # Read by status mask 0xFF → expect 1 DTC.
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0xFF]))
    _assert(len(resp) == 9, f"ReadDTC resp len {len(resp)} (expected 9 = 5 header + 1×4)")
    assert_bytes(resp[:5], bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x01]),
                 "DTC report header (1 DTC)")
    assert_bytes(resp[5:9], bytes([0x03, 0x01, 0x00, 0x05]),
                 "DTC record: 0x030100 status 0x05")
    # Clear DTCs.
    resp = drv.send_uds(bytes([SID_CDI, 0xFF, 0xFF, 0xFF]))
    assert_bytes(resp, bytes([SID_CDI + 0x40]), "ClearDiagnosticInformation")
    # Read again → 0 DTCs.
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0xFF]))
    assert_bytes(resp, bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x00]),
                 "DTC report header (0 DTCs)")


def s_uds_dtc_multi(bus: Bus) -> None:
    """Multiple DTCs (P-group + U-group) + read by status mask + partial clear.
    The high nibble of the first DTC byte encodes the group per ISO 15031-6:
      0x0XXXXX = P (Powertrain)
      0x3XXXXX = U (Network)
    """
    drv = MasterDriver(bus)
    if isinstance(bus, SimBus):
        # 0x030100: first byte 0x03 → high nibble 0x00 = P-group
        bus.fw.dtcs.append((0x030100, 0x05))  # P-group, TEST_FAILED | CONFIRMED
        # 0x130200: first byte 0x13 → high nibble 0x10 = C-group (Chassis)
        bus.fw.dtcs.append((0x130200, 0x0D))  # C-group, different status bits
    # Read by status mask 0x0F (TEST_FAILED | TEST_FAILED_CURRENT | CONFIRMED | PENDING).
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0x0F]))
    _assert(len(resp) == 13, f"ReadDTC multiresp len {len(resp)} (expected 13 = 5 header + 2×4)")
    assert_bytes(resp[:5], bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x02]),
                 "DTC report header (2 DTCs)")
    # Read different mask → 0 matches (0x80 = NOT_AVAILABLE, not set).
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0x80]))
    assert_bytes(resp, bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x00]),
                 "DTC report header (0 DTCs for mask 0x80)")
    # Partial clear by C-group (0x10XXXX). The high nibble 0x10
    # matches DTC 0x130200 but not 0x030100.
    resp = drv.send_uds(bytes([SID_CDI, 0x10, 0xFF, 0xFF]))
    assert_bytes(resp, bytes([SID_CDI + 0x40]), "Partial clear by C-group")
    # Read all — only 0x030100 (P-group) should remain.
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0xFF]))
    _assert(len(resp) == 9, f"DTC after partial clear: len {len(resp)}")
    assert_bytes(resp[:5], bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x01]),
                 "DTC report header (1 DTC after C-group clear)")
    # Full clear.
    resp = drv.send_uds(bytes([SID_CDI, 0xFF, 0xFF, 0xFF]))
    resp = drv.send_uds(bytes([SID_RDTCI, 0x02, 0xFF]))
    assert_bytes(resp, bytes([SID_RDTCI + 0x40, 0x02, 0xFE, 0x00, 0x00]),
                 "DTC report header (0 DTCs after full clear)")


def s_uds_session_default(bus: Bus) -> None:
    """DiagnosticSessionControl 0x10 0x01 → 0x50 0x01."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x01])), "DSC write OK")
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_DSC + 0x40, 0x01]), "DefaultSession response")


def s_uds_security_sal2(bus: Bus) -> None:
    """SAL2 unlock via direct UDS transport. Uses mask KEY_MASK_SAL2,
    reachable from ProgrammingSession."""
    drv = MasterDriver(bus)
    # Unlock SAL1 first.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    sk = _aes_key(seed, KEY_MASK_SAL1)
    drv.send_uds(bytes([SID_SA, 0x02]) + sk)
    # Enter ProgrammingSession (clears SAL).
    drv.send_uds(bytes([SID_DSC, 0x02]))
    # RequestSeed SAL2.
    resp = drv.send_uds(bytes([SID_SA, 0x03]))
    _assert(len(resp) == 18, f"SAL2 seed response length {len(resp)}")
    seed2 = resp[2:]
    # SendKey SAL2.
    sk2 = _aes_key(seed2, KEY_MASK_SAL2)
    resp = drv.send_uds(bytes([SID_SA, 0x04]) + sk2)
    assert_bytes(resp, bytes([SID_SA + 0x40, 0x04]), "SAL2 unlocked")


def s_uds_security_sal3(bus: Bus) -> None:
    """SAL3 unlock via direct UDS transport. SAL3 is only
    reachable from ExtendedSession."""
    drv = MasterDriver(bus)
    # 1. Default session → SAL3 denied.
    resp = drv.send_uds(bytes([SID_SA, 0x05]))
    assert_bytes(resp, bytes(
        [0x7F, SID_SA, NRC_SUB_FUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION]),
        "SAL3 denied in Default session")
    # 2. Unlock SAL1, enter Extended.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    sk1 = _aes_key(seed, KEY_MASK_SAL1)
    drv.send_uds(bytes([SID_SA, 0x02]) + sk1)
    resp = drv.send_uds(bytes([SID_DSC, 0x03]))
    assert_bytes(resp, bytes([SID_DSC + 0x40, 0x03]), "ExtendedSession")
    # 3. RequestSeed SAL3.
    resp = drv.send_uds(bytes([SID_SA, 0x05]))
    _assert(len(resp) == 18, f"SAL3 seed response length {len(resp)}")
    seed3 = resp[2:]
    # 4. SendKey SAL3.
    sk3 = _aes_key(seed3, KEY_MASK_SAL3)
    resp = drv.send_uds(bytes([SID_SA, 0x06]) + sk3)
    assert_bytes(resp, bytes([SID_SA + 0x40, 0x06]), "SAL3 unlocked")


def s_uds_reset_soft(bus: Bus) -> None:
    """0x11 0x03 SoftReset → 0x51 0x03 (without firing the
    actual NVIC reset — the emulator just stashes the response).
    Also verifies 0x11 0x01 HardReset still works.
    """
    drv = MasterDriver(bus)
    # SoftReset.
    drv.sdo_write(0x2F00, 0, bytes([SID_ER, 0x03]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_ER + 0x40, 0x03]), "SoftReset response")
    # HardReset (regression — should still work).
    drv.sdo_write(0x2F00, 0, bytes([SID_ER, 0x01]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_ER + 0x40, 0x01]), "HardReset response")
    # Unknown subfunc → SubFunctionNotSupported.
    drv.sdo_write(0x2F00, 0, bytes([SID_ER, 0x02]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_ER, NRC_SUB_FUNC_NOT_SUPPORTED]),
                 "ER 0x02 rejected")


def s_uds_active_did(bus: Bus) -> None:
    """ReadDataByIdentifier 0xF186 → 0x62 0x86 0xF1 <session>."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_RDBI, 0x86, 0xF1]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_RDBI + 0x40, 0x86, 0xF1, 0x01]),
                 "F186 in Default session")


def s_uds_wrong_key(bus: Bus) -> None:
    """SendKey with the wrong value → 0x35 InvalidKey."""
    drv = MasterDriver(bus)
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    # Extract seed, ignore it — we'll send a bogus key.
    bogus_key = b'\x00' * 16
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + bogus_key)
    assert_bytes(resp, bytes([0x7F, SID_SA, NRC_INVALID_KEY]),
                 "wrong key rejected")


def s_uds_security_lockout(bus: Bus) -> None:
    """3 consecutive wrong SendKey attempts → 0x36 ExceededNumberOfAttempts.
    Session change resets the counter, allowing a fresh unlock."""
    drv = MasterDriver(bus)
    # 1. First wrong key.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + b'\x00' * 16)
    assert_bytes(resp, bytes([0x7F, SID_SA, NRC_INVALID_KEY]), "fail #1")
    # 2. Second wrong key (fresh seed each time).
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + b'\x01' * 16)
    assert_bytes(resp, bytes([0x7F, SID_SA, NRC_INVALID_KEY]), "fail #2")
    # 3. Third wrong key.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + b'\x02' * 16)
    assert_bytes(resp, bytes([0x7F, SID_SA, NRC_INVALID_KEY]), "fail #3")
    # 4. Fourth attempt → locked out (0x36).
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    assert_bytes(resp, bytes([0x7F, SID_SA, NRC_EXCEEDED_NUMBER_OF_ATTEMPTS]),
                 "lockout after 3 failures")
    # 5. Session change resets the counter.
    resp = drv.send_uds(bytes([SID_DSC, 0x01]))
    assert_bytes(resp, bytes([SID_DSC + 0x40, 0x01]), "session change resets lockout")
    # 6. Fresh unlock works.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    expected_key = _aes_key(seed, KEY_MASK_SAL1)
    resp = drv.send_uds(bytes([SID_SA, 0x02]) + expected_key)
    assert_bytes(resp, bytes([SID_SA + 0x40, 0x02]), "fresh unlock after session change")


def s_uds_request_seed_when_unlocked(bus: Bus) -> None:
    """ISO 14229: when SecurityAccess is already unlocked,
    RequestSeed must return a positive response with a zero
    seed (all bytes 0x00), not an NRC."""
    drv = MasterDriver(bus)
    # Get unlocked: seed + correct key.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    seed = resp[2:]
    expected_key = _aes_key(seed, KEY_MASK_SAL1)
    drv.send_uds(bytes([SID_SA, 0x02]) + expected_key)
    # Request seed again — zero seed.
    resp = drv.send_uds(bytes([SID_SA, 0x01]))
    _assert(len(resp) == 18, f"zero-seed response {len(resp)} bytes (expected 18)")
    assert_bytes(resp[:2], bytes([SID_SA + 0x40, 0x01]), "zero seed header")
    _assert(all(b == 0 for b in resp[2:]), "zero seed body")


def s_ota_block_seq(bus: Bus) -> None:
    """OTA flow: unlock → program session → RequestDownload (100 B)
    → TransferData seq=1 OK → TransferData seq=3 (wrong) →
    0x73 WrongBlockSequenceNumber.

    The 0x34 RequestDownload is 5 bytes — goes out via segmented
    SDO download. The 0x36 TransferData is 3 bytes, expedited."""
    drv = MasterDriver(bus)
    # Setup: unlock + enter programming
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    drv.sdo_read(0x2F00, 0)
    drv.sdo_write_long(0x2F00, 0,
                       bytes([SID_SA, 0x02]) + KEY)
    drv.sdo_read(0x2F00, 0)
    drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x02]))
    drv.sdo_read(0x2F00, 0)
    # RequestDownload 100 bytes — 5-byte UDS request, segmented SDO.
    drv.sdo_write_long(0x2F00, 0,
                       bytes([SID_RD, 0x00]) + struct.pack('<I', 100))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x74, 0x00, 0x00, 0x02]), "RD positive")
    # TransferData seq=1 OK
    drv.sdo_write(0x2F00, 0, bytes([SID_TD, 0x01, 0xAA, 0xBB]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x76, 0x01]), "TD seq=1 positive")
    # TransferData seq=3 (expected 2) → 0x73
    drv.sdo_write(0x2F00, 0, bytes([SID_TD, 0x03, 0xCC, 0xDD]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val,
                 bytes([0x7F, SID_TD, NRC_WRONG_BLOCK_SEQUENCE_NUMBER]),
                 "wrong block seq → 0x73")


def s_nmt_states(bus: Bus) -> None:
    """NMT transitions Operational / Stopped / PreOperational via
    addressed and broadcast frames."""
    drv = MasterDriver(bus)
    _assert(drv.nmt(0x01) == 'Operational', "→ Operational")
    _assert(drv.nmt(0x02) == 'Stopped', "→ Stopped")
    _assert(drv.nmt(0x80) == 'PreOperational', "→ PreOperational")
    # Broadcast (node = 0) applies to us.
    _assert(drv.nmt(0x01) == 'Operational', "broadcast → Operational")


def s_seg_toggle_mismatch(bus: Bus) -> None:
    """Upload Segment request with wrong toggle bit → abort."""
    drv = MasterDriver(bus)
    # Trigger a 6-byte response (SecurityAccess seed).
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    # The previous sdo_write consumed the seed (upload complete).
    # Re-trigger by writing again, then issue an Upload Initiate
    # ourselves and immediately send a wrong-toggle segment
    # without fetching the real one.
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    init = drv.bus.send(drv._frame_sdo_read(0x2F00, 0))
    _assert(init['data'][0] == 0x82,
            f"expected 0x82 (segmented), got 0x{init['data'][0]:02x}")
    # Wrong toggle (expected 0, send 1).
    bad = drv.bus.send(drv._frame_sdo_seg(1))
    _assert(bad is not None, "should respond with abort")
    assert_bytes(bad['data'][0:1], b'\x80', "abort SCS")


def s_seg_size_field(bus: Bus) -> None:
    """Segmented initiate response must have size field in bytes
    4–5 little-endian."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    # Re-trigger for the segmented path (the first sdo_write
    # already completed the previous upload).
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    init = drv.bus.send(drv._frame_sdo_read(0x2F00, 0))
    size = struct.unpack_from('<H', init['data'], 4)[0]
    _assert(size == 18, f"size field should be 18 (AES-128 seed), got {size}")


def s_seg_replay_after_done(bus: Bus) -> None:
    """After a segmented upload completes, a stale segment request
    should abort rather than replay old data."""
    drv = MasterDriver(bus)
    # Trigger + complete a segmented upload.
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    drv.sdo_read(0x2F00, 0)  # consumes all 6 bytes; clears state
    # Now a stray segment request.
    bad = drv.bus.send(drv._frame_sdo_seg(0))
    _assert(bad is not None and bad['data'][0] == 0x80,
            "stray segment after done → abort")


# ---- Segmented download scenarios -----------------------------------

def s_seg_dl_initiate(bus: Bus) -> None:
    """0x21 Initiate Download Response is 0x60 (success)."""
    drv = MasterDriver(bus)
    # 5-byte UDS request: SecurityAccess sendKey with wrong key.
    payload = bytes([SID_SA, 0x02]) + b'\x00\x00\x00\x00'
    init = drv.bus.send(drv._frame_sdo_dl_initiate(0x2F00, 0, len(payload)))
    _assert(init is not None, "no initiate response")
    assert_bytes(init['data'][0:1], b'\x60', "Initiate Download Response SCS")
    # Clean up the in-flight download so subsequent scenarios start clean.
    drv.bus.send(drv._frame_sdo_dl_seg(0, payload, last=True))


def s_seg_dl_one_segment(bus: Bus) -> None:
    """5-byte value fits in one 7-byte segment with c=1 and n=2."""
    drv = MasterDriver(bus)
    drv.sdo_write_long(0x2F00, 0,
                       bytes([SID_SA, 0x02]) + KEY)
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_SA + 0x40, 0x02]), "key accepted")


def s_seg_dl_toggle_mismatch(bus: Bus) -> None:
    """Download Segment with wrong toggle bit → abort."""
    drv = MasterDriver(bus)
    init = drv.bus.send(drv._frame_sdo_dl_initiate(0x2F00, 0, 5))
    _assert(init['data'][0] == 0x60, "init OK")
    # Wrong toggle (expected 0, send 1).
    bad = drv.bus.send(drv._frame_sdo_dl_seg(1, b'\xAA\xBB\xCC\xDD\xEE', last=True))
    _assert(bad is not None and bad['data'][0] == 0x80,
            "wrong toggle → abort")
    # State should be cleared — a subsequent segment also aborts.
    bad2 = drv.bus.send(drv._frame_sdo_dl_seg(0, b'\xAA\xBB\xCC\xDD\xEE', last=True))
    _assert(bad2 is not None and bad2['data'][0] == 0x80,
            "after abort, segment also aborts")


def s_seg_dl_stray_segment(bus: Bus) -> None:
    """A Download Segment without a preceding Initiate aborts."""
    drv = MasterDriver(bus)
    bad = drv.bus.send(drv._frame_sdo_dl_seg(0, b'\xAA\xBB\xCC\xDD', last=True))
    _assert(bad is not None and bad['data'][0] == 0x80,
            "stray segment → abort")


def s_seg_dl_size_mismatch(bus: Bus) -> None:
    """Segment's c=1 flag arrived but offset != total → abort."""
    drv = MasterDriver(bus)
    init = drv.bus.send(drv._frame_sdo_dl_initiate(0x2F00, 0, 5))
    _assert(init['data'][0] == 0x60, "init OK")
    # Send only 3 bytes with c=1; size says 5 → premature last.
    bad = drv.bus.send(drv._frame_sdo_dl_seg(0, b'\x01\x02\x03', last=True))
    _assert(bad is not None and bad['data'][0] == 0x80,
            "premature last → abort")


def s_seg_dl_two_segments(bus: Bus) -> None:
    """Skipped: Phase 3 v3 caps SDO segmented transfer at 7 bytes,
    so a single 7-byte segment carries every UDS request the
    firmware actually serves (sendKey=5, RequestDownload=5).
    Multi-segment transfers (>7 bytes) are deferred to a later
    phase — bumping SEGMENTED_MAX requires re-validating every
    other byte path that touches it."""
    raise TestFailure("multi-segment SDO download is deferred")


# ---- Scenario registry ----------------------------------------------

SCENARIOS: dict[str, callable] = {
    "heartbeat":           s_heartbeat,
    "sdo_basic":           s_sdo_basic,
    "sdo_write_heartbeat": s_sdo_write_heartbeat,
    "sdo_ro_rejected":     s_sdo_ro_rejected,
    "uds_tp":              s_uds_tp,
    # NOTE: 0x3E 0x80 (suppress positive response) is a known
    # gap — the SDO layer's `build_upload_response` doesn't
    # encode an empty payload, and `OdValue::len == 0` is
    # undefined. The Rust side honours it (per the existing
    # `store_response(&[])` path) but the simulator can't
    # round-trip an empty SDO read. Tracked as a known
    # limitation; full coverage needs a TP-only scenario that
    # bypasses SDO and uses a direct UDS dispatcher call.
    "uds_dtc":             s_uds_dtc_basic,
    "uds_dtc_multi":       s_uds_dtc_multi,
    "uds_security_lockout": s_uds_security_lockout,
    "uds_session":         s_uds_session_default,
    "uds_security":        s_uds_security_unlock,
    "uds_security_sal2":   s_uds_security_sal2,
    "uds_security_sal3":   s_uds_security_sal3,
    "uds_reset_soft":      s_uds_reset_soft,
    "uds_active_did":      s_uds_active_did,
    "uds_wrong_key":       s_uds_wrong_key,
    "uds_seed_when_unlocked": s_uds_request_seed_when_unlocked,
    "uds_aes_kat":         s_uds_aes_kat,
    "uds_session_gate_nrc": s_uds_session_gate_nrc,
    "uds_programming_needs_sal": s_uds_programming_needs_sal,
    "uds_cc_disable":      s_uds_cc_disable,
    "uds_cc_enable_bypass": s_uds_cc_enable_bypass,
    "uds_rc_start":        s_uds_rc_start,
    "uds_rc_result":       s_uds_rc_result,
    # Phase 6: raw UDS transport (independent of CANopen SDO).
    "uds_transport_session":      s_uds_transport_session,
    "uds_transport_did":          s_uds_transport_did,
    "uds_transport_security_full": s_uds_transport_security_full,
    "uds_transport_wrong_id":     s_uds_transport_wrong_id,
    "uds_transport_no_sdo":       s_uds_transport_no_sdo_dependency,
    "ota_block_seq":       s_ota_block_seq,
    "nmt":                 s_nmt_states,
    "seg_toggle_mismatch": s_seg_toggle_mismatch,
    "seg_size_field":      s_seg_size_field,
    "seg_replay_after":    s_seg_replay_after_done,
    "seg_dl_initiate":     s_seg_dl_initiate,
    "seg_dl_one_segment":  s_seg_dl_one_segment,
    "seg_dl_toggle":       s_seg_dl_toggle_mismatch,
    "seg_dl_stray":        s_seg_dl_stray_segment,
    "seg_dl_size_mismatch": s_seg_dl_size_mismatch,
    # "seg_dl_two_segments": s_seg_dl_two_segments,  # deferred: SEGMENTED_MAX=7
}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenarios", default=",".join(SCENARIOS.keys()),
                   help=f"Comma-separated list of scenarios to run "
                        f"(default: all). Known: {','.join(SCENARIOS.keys())}")
    p.add_argument("--list", action="store_true",
                   help="List scenario names and exit")
    p.add_argument("--live", metavar="IFACE", default=None,
                   help="Run against real CAN hardware on the given "
                        "python-can channel (e.g. `vcan0`, "
                        "`socketcan:vcan0`, `pcan:PCAN_USBBUS1`). "
                        "Each scenario gets a fresh firmware; "
                        "the firmware is whatever is on the bus "
                        "(real board, or `scripts/firmware_emulator.py` "
                        "listening on vcan0).")
    p.add_argument("--bitrate", type=int, default=500_000,
                   help="Bus bitrate for --live mode (default 500_000)")
    p.add_argument("--timeout", type=float, default=1.0,
                   help="Per-frame response timeout in --live mode (default 1.0s)")
    args = p.parse_args()

    if args.list:
        print("Scenarios:")
        for name in SCENARIOS:
            doc = SCENARIOS[name].__doc__ or ""
            print(f"  {name:<22} {doc.splitlines()[0] if doc else ''}")
        return 0

    selected = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    if args.live:
        # Channel string parsing: "vcan0" → socketcan / vcan0; full
        # form `interface:channel` passed through to python-can.
        if ":" in args.live:
            iface, chan = args.live.split(":", 1)
        else:
            iface, chan = "socketcan", args.live
        try:
            bus: Bus = CanBus(iface, chan, bitrate=args.bitrate, timeout=args.timeout)
        except TestFailure as e:
            print(f"  ERROR setup: {e}")
            return 2
        mode_label = f"live {iface}:{chan} @ {args.bitrate} bps"
    else:
        bus = None  # built per-scenario so each one is fresh
        mode_label = "sim (in-process)"

    print(f"Mode: {mode_label}")
    failed = 0
    for name in selected:
        if name not in SCENARIOS:
            print(f"  UNKNOWN  {name}")
            failed += 1
            continue
        if bus is None:
            # Sim mode: each scenario gets a fresh firmware.
            sim_bus = SimBus(FirmwareEmulator())
        else:
            sim_bus = bus  # shared across scenarios
        try:
            SCENARIOS[name](sim_bus)
            print(f"  PASS  {name}")
        except TestFailure as e:
            print(f"  FAIL  {name}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
            failed += 1

    print()
    if bus is not None:
        bus.close()
    if failed == 0:
        print(f"All {len(selected)} scenarios passed.")
        return 0
    print(f"{failed}/{len(selected)} scenarios failed.")
    return 1


if __name__ == "__main__":
    sys.exit(main())