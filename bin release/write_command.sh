#!/usr/bin/env bash
# Writes a merged ESP32-S3 image at offset 0x0.
#
# Usage:
#   "bin release/write_command.sh" <merged.bin> [PORT]
#
# PORT defaults to the first /dev/cu.usbmodem* (macOS) or /dev/ttyACM* (Linux).
set -euo pipefail

IMAGE="${1:?Usage: write_command.sh <merged.bin> [PORT]}"
PORT="${2:-}"
if [[ -z "$PORT" ]]; then
  PORT="$(ls /dev/cu.usbmodem* /dev/ttyACM* 2>/dev/null | head -n1 || true)"
fi
if [[ -z "$PORT" ]]; then
  echo "No serial port found; pass it as the second argument." >&2
  exit 1
fi

ESPTOOL="${ESPTOOL:-esptool.py}"
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 460800 --after hard_reset write_flash 0x0 "$IMAGE"
