#!/usr/bin/env python3
"""
Run the Python firmware emulator on a real CAN interface.

Lets `scripts/smoke_test.py --live` exercise the wire format
end-to-end without the Rust firmware: the emulator binds to a
CAN interface (typically a `vcan0` virtual CAN), the smoke test
runs against the same interface, and the bytes exchanged are
real CAN frames.

This is useful when you want to:

* Verify the wire format without flashing real hardware.
* Cross-check the Rust firmware's wire output against an
  independent Python implementation (run both side-by-side on
  the same bus and compare traces — useful for finding protocol
  drift).
* Develop / debug the master side (python-canopen, custom
  scripts) against a known-good firmware that you can introspect
  from Python.

Usage::

    # Terminal 1 — set up vcan0 (requires root):
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

    # Terminal 2 — run the emulator:
    python3 scripts/firmware_emulator.py vcan0

    # Terminal 3 — run the smoke test:
    python3 scripts/smoke_test.py --live vcan0

Or use any python-can interface: `pcan:PCAN_USBBUS1`,
`socketcan:can0`, etc.

The emulator is byte-for-byte identical to the one inside
`scripts/smoke_test.py` (sim mode), so passing the smoke test
here is equivalent to passing it in-process.
"""

import argparse
import sys
import time

# Re-use the emulator from the smoke test.
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from smoke_test import FirmwareEmulator, HEARTBEAT_COB_ID, NMT_COB_ID, SDO_RX_COB_ID  # noqa: E402

import can  # noqa: E402,F401


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("channel",
                   help="python-can channel (e.g. `vcan0`, "
                        "`socketcan:can0`, `pcan:PCAN_USBBUS1`).")
    p.add_argument("--bitrate", type=int, default=500_000,
                   help="Bus bitrate (default 500_000).")
    p.add_argument("--node-id", type=int, default=1,
                   help="CANopen NodeId (default 1).")
    p.add_argument("--heartbeat-ms", type=int, default=1000,
                   help="Heartbeat period in ms (default 1000).")
    args = p.parse_args()

    if ":" in args.channel:
        iface, chan = args.channel.split(":", 1)
    else:
        iface, chan = "socketcan", args.channel

    bus = can.interface.Bus(interface=iface, channel=chan, bitrate=args.bitrate)
    fw = FirmwareEmulator()

    print(f"firmware emulator: {iface}:{chan} @ {args.bitrate} bps, "
          f"NodeId={args.node_id}, heartbeat={args.heartbeat_ms}ms")

    # Boot-up: send a single heartbeat with state=0x00, then go
    # to PreOperational (heartbeat byte 0x7F). Matches CiA 301.
    boot = can.Message(
        arbitration_id=HEARTBEAT_COB_ID,
        data=[fw.heartbeat_byte() & 0x00],  # 0x00 = boot-up
        is_extended_id=False,
    )
    # Re-fetch the boot-up byte (Booting -> 0x00).
    fw.nmt_state = 'Booting'
    boot.data = [fw.heartbeat_byte()]
    bus.send(boot, timeout=1.0)
    fw.nmt_state = 'PreOperational'
    next_hb = time.monotonic() + args.heartbeat_ms / 1000.0

    try:
        while True:
            now = time.monotonic()
            # Heartbeat tick.
            if now >= next_hb:
                hb = can.Message(
                    arbitration_id=HEARTBEAT_COB_ID,
                    data=[fw.heartbeat_byte()],
                    is_extended_id=False,
                )
                bus.send(hb, timeout=0.5)
                next_hb = now + args.heartbeat_ms / 1000.0

            # Block briefly waiting for an SDO / NMT frame.
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            arb_id = msg.arbitration_id
            frame = {'id': arb_id, 'data': bytes(msg.data), 'dlc': msg.dlc}
            if arb_id == SDO_RX_COB_ID:
                resp = fw.handle_sdo(frame)
                if resp is not None:
                    out = can.Message(
                        arbitration_id=resp['id'],
                        data=list(resp['data']),
                        is_extended_id=False,
                    )
                    bus.send(out, timeout=0.5)
            elif arb_id == NMT_COB_ID:
                fw.handle_nmt(frame)
            # Other COB-IDs (heartbeats from other nodes, etc.)
            # are silently dropped.
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())