#!/usr/bin/env bash
set -euo pipefail
IFACE="${1:-vcan0}"
if ! command -v ip >/dev/null 2>&1; then
  echo "Error: iproute2 (ip) is required." >&2
  exit 1
fi
sudo modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  sudo ip link add dev "$IFACE" type vcan
fi
sudo ip link set up "$IFACE"
echo "Virtual CAN interface $IFACE is ready."
echo "Run: ./build/autocore_socketcan_demo $IFACE"
