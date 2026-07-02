#!/usr/bin/env bash
# Set up a Linux virtual CAN interface for smoke testing the
# foc-rust CAN stack without real hardware.
#
# Requires root (or CAP_NET_ADMIN). Linux kernel module `vcan`
# must be loaded: `sudo modprobe vcan`.
#
# Usage:
#   sudo ./scripts/setup_vcan.sh           # create + bring up vcan0
#   sudo ./scripts/setup_vcan.sh --down    # tear it down

set -euo pipefail

IFACE="${VCAN_IFACE:-vcan0}"

case "${1:-up}" in
  up)
    ip link show dev "$IFACE" >/dev/null 2>&1 || {
      echo "creating $IFACE"
      ip link add dev "$IFACE" type vcan
    }
    ip link set up "$IFACE"
    echo "$IFACE is up:"
    ip -d link show dev "$IFACE"
    ;;
  down)
    ip link show dev "$IFACE" >/dev/null 2>&1 || {
      echo "$IFACE does not exist"
      exit 0
    }
    ip link set down "$IFACE"
    ip link del dev "$IFACE"
    echo "$IFACE removed"
    ;;
  *)
    echo "usage: $0 [up|down]" >&2
    exit 2
    ;;
esac