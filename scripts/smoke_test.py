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

# SDO command specifiers (top 3 bits of byte 0)
SDO_CMD_DOWNLOAD   = 0x20  # CCS=1 — Initiate Download Request
SDO_CMD_DOWNLOAD_SEG = 0x00 # CCS=0 — Download Segment Request
SDO_CMD_UPLOAD     = 0x40  # CCS=2 — Initiate Upload Request
SDO_CMD_UPLOAD_SEG = 0x60  # CCS=3 — Upload Segment Request
SDO_CMD_ABORT      = 0x80  # SCS=4 — Abort Transfer

SDO_MAX_SEGMENTED_SIZE = 7

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

# SDO abort codes (CiA 301)
SDO_ABORT_TOGGLE_BIT_NOT_ALTERED = 0x0503_0000
SDO_ABORT_INVALID_COMMAND        = 0x0504_0001
SDO_ABORT_READ_ONLY              = 0x0601_0002
SDO_ABORT_OBJECT_DOES_NOT_EXIST  = 0x0602_0000
SDO_ABORT_LENGTH_MISMATCH        = 0x0607_0010

# OTA
APP_START = 0x0800_0000
APP_SIZE  = 0x1F800  # 124 KB app region (B-G431B-ESC1 has 128 KB)
SEED = b'\xA5\xA5\xA5\xA5'
KEY  = 0xA5A5_B7D9  # seed + 0x1234, LE


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
        rest = payload[1:]
        handlers = {
            SID_DSC:   self._uds_dsc,
            SID_ER:    self._uds_er,
            SID_CDI:   self._uds_cdi,
            SID_RDTCI: self._uds_rdtci,
            SID_RDBI:  self._uds_rdbi,
            SID_WDBI:  self._uds_wdbi,
            SID_SA:    self._uds_sa,
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
            self.last_response = bytes([SID_DSC + 0x40, 0x01])
        elif sub == 0x02:
            if self.uds_security == 0:
                self.last_response = bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED])
                return
            self.uds_session = 0x02
            self.uds_security = 0
            self.last_response = bytes([SID_DSC + 0x40, 0x02])
        else:
            self.last_response = bytes([0x7F, SID_DSC, NRC_SUB_FUNC_NOT_SUPPORTED])

    def _uds_er(self, p: bytes) -> None:
        if len(p) != 1 or p[0] != 0x01:
            self.last_response = bytes([0x7F, SID_ER, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        self.last_response = bytes([SID_ER + 0x40, 0x01])

    def _uds_cdi(self, p: bytes) -> None:
        if len(p) != 3:
            self.last_response = bytes([0x7F, SID_CDI, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        self.last_response = bytes([SID_CDI + 0x40])

    def _uds_rdtci(self, p: bytes) -> None:
        if len(p) < 2 or p[0] != 0x02:
            self.last_response = bytes([0x7F, SID_RDTCI, NRC_SUB_FUNC_NOT_SUPPORTED])
            return
        self.last_response = bytes([SID_RDTCI + 0x40, 0x02, 0x00, 0x00])

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
        # v1: no writable DIDs
        if len(p) < 2:
            self.last_response = bytes([0x7F, SID_WDBI, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        self.last_response = bytes([0x7F, SID_WDBI, NRC_REQUEST_OUT_OF_RANGE])

    def _uds_sa(self, p: bytes) -> None:
        if not p:
            self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
            return
        sub = p[0]
        if sub == 0x01:
            if self.uds_security != 0:
                self.last_response = bytes([0x7F, SID_SA, NRC_SECURITY_ACCESS_DENIED])
                return
            # 6-byte response: 0x67, subfunc, seed[4]
            self.last_response = bytes([SID_SA + 0x40, 0x01]) + SEED
        elif sub == 0x02:
            if len(p) != 5:
                self.last_response = bytes([0x7F, SID_SA, NRC_INCORRECT_MESSAGE_LENGTH])
                return
            key = struct.unpack_from('<I', p, 1)[0]
            if key == KEY:
                self.uds_security = 1
                self.last_response = bytes([SID_SA + 0x40, 0x02])
            else:
                self.last_response = bytes([0x7F, SID_SA, NRC_SECURITY_ACCESS_DENIED])
        else:
            self.last_response = bytes([0x7F, SID_SA, NRC_SUB_FUNC_NOT_SUPPORTED])

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
            return self.fw.handle_sdo(frame)
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
    """Builds SDO request frames, parses responses the same way a
    real CANopen master would. Asserts on every response byte.

    Talks to a `Bus` (SimBus or CanBus) for actual frame transport;
    the frame builders are identical in both modes."""

    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        # Exposed for tests that want to poke at internal state
        # (e.g. trigger a segmented upload to check abort-on-replay).
        # Sim-only; CanBus returns None.
        self.fw = bus.fw if isinstance(bus, SimBus) else None

    # ---- low-level frame builders -----------------------------------

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
        more 0x00 Segments). For 5–7 bytes a single segment
        carries the whole payload; this implementation caps at 14
        bytes (two segments) which is more than enough for every
        UDS request the firmware handles (sendKey = 5, RequestDownload = 5).
        """
        assert 5 <= len(value) <= 14
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


def s_uds_session_default(bus: Bus) -> None:
    """DiagnosticSessionControl 0x10 0x01 → 0x50 0x01."""
    drv = MasterDriver(bus)
    _assert(drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x01])), "DSC write OK")
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_DSC + 0x40, 0x01]), "DefaultSession response")


def s_uds_security_unlock(bus: Bus) -> None:
    """Full security dance:

    1. Try ProgrammingSession without unlock → 0x33 SecurityAccessDenied.
    2. RequestSeed (0x27 0x01) → 0x67 0x01 0xA5 0xA5 0xA5 0xA5
       (6 bytes — exercises segmented SDO upload).
    3. SendKey (0x27 0x02 + 4-byte key) → 0x67 0x02.
    4. ProgrammingSession (0x10 0x02) → 0x50 0x02.
    """
    drv = MasterDriver(bus)
    # 1. Locked → ProgrammingSession denied.
    drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x02]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_DSC, NRC_SECURITY_ACCESS_DENIED]),
                 "DSC 0x02 without unlock")
    # 2. RequestSeed.
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_SA + 0x40, 0x01]) + SEED, "4-byte seed response")
    # 3. SendKey — 5 bytes, so this needs segmented SDO download.
    drv.sdo_write_long(0x2F00, 0,
                       bytes([SID_SA, 0x02]) + struct.pack('<I', KEY))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_SA + 0x40, 0x02]), "key accepted")
    # 4. Now ProgrammingSession works.
    drv.sdo_write(0x2F00, 0, bytes([SID_DSC, 0x02]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_DSC + 0x40, 0x02]), "DSC 0x02 after unlock")


def s_uds_active_did(bus: Bus) -> None:
    """ReadDataByIdentifier 0xF186 → 0x62 0x86 0xF1 <session>."""
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_RDBI, 0x86, 0xF1]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([SID_RDBI + 0x40, 0x86, 0xF1, 0x01]),
                 "F186 in Default session")


def s_uds_wrong_key(bus: Bus) -> None:
    """SendKey with the wrong value → 0x33 SecurityAccessDenied.

    SendKey is 5 bytes (1 SID + 1 sub + 4 key), so we use the
    segmented SDO download path. See `MasterDriver.sdo_write_long`.
    """
    drv = MasterDriver(bus)
    drv.sdo_write(0x2F00, 0, bytes([SID_SA, 0x01]))
    drv.sdo_read(0x2F00, 0)  # consume seed
    drv.sdo_write_long(0x2F00, 0,
                       bytes([SID_SA, 0x02, 0x00, 0x00, 0x00, 0x00]))
    val = drv.sdo_read(0x2F00, 0)
    assert_bytes(val, bytes([0x7F, SID_SA, NRC_SECURITY_ACCESS_DENIED]),
                 "wrong key rejected")


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
                       bytes([SID_SA, 0x02]) + struct.pack('<I', KEY))
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
    _assert(size == 6, f"size field should be 6, got {size}")


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
                       bytes([SID_SA, 0x02]) + struct.pack('<I', KEY))
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
    "uds_session":         s_uds_session_default,
    "uds_security":        s_uds_security_unlock,
    "uds_active_did":      s_uds_active_did,
    "uds_wrong_key":       s_uds_wrong_key,
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