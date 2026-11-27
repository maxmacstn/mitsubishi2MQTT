#!/usr/bin/env bash
# Builds a single ESP32-S3 image (bootloader + partitions + boot_app0 + app)
# that can be written at offset 0x0 with write_command.sh or any flasher.
#
# Usage (from anywhere):
#   "bin release/merge_command.sh" [VERSION]
#
# VERSION defaults to the version in src/mitsubishi2mqtt/config.h, e.g.
# 2025.1.0 -> m2mqtt_2025-1-0-ESP32S3_merged.bin. Paths are derived from this
# repository and the PlatformIO home, so nothing is machine specific.
# Set SKIP_BUILD=1 to merge an existing build without running `pio run`.
set -euo pipefail

ENV_NAME="wifikit-serial-esp32-s3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$ROOT/.pio/build/$ENV_NAME"
PIO="${PIO:-pio}"
PIO_HOME="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"

VERSION="${1:-}"
if [[ -z "$VERSION" ]]; then
  VERSION="$(sed -n 's/.*m2mqtt_version = .*(\([0-9.]*\)).*/\1/p' "$ROOT/src/mitsubishi2mqtt/config.h" | head -n1)"
fi
if [[ -z "$VERSION" ]]; then
  echo "Could not read the version from config.h; pass it as the first argument." >&2
  exit 1
fi
VERSION_FILE="${VERSION//./-}"

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  (cd "$ROOT" && "$PIO" run -e "$ENV_NAME")
fi

APP_BIN="$BUILD_DIR/firmware.bin"
BOOTLOADER_BIN="$BUILD_DIR/bootloader.bin"
PARTITIONS_BIN="$BUILD_DIR/partitions.bin"
BOOT_APP0_BIN="$(ls "$PIO_HOME"/packages/framework-arduinoespressif32*/tools/partitions/boot_app0.bin 2>/dev/null | head -n1 || true)"

for f in "$APP_BIN" "$BOOTLOADER_BIN" "$PARTITIONS_BIN" "$BOOT_APP0_BIN"; do
  if [[ -z "$f" || ! -f "$f" ]]; then
    echo "Missing build output: ${f:-boot_app0.bin}" >&2
    exit 1
  fi
done

# Prefer PlatformIO's esptool run by PlatformIO's Python (which has
# pyserial), then an esptool on PATH. Override with PYTHON=/path/to/python.
ESPTOOL=()
PYTHON="${PYTHON:-$PIO_HOME/penv/bin/python}"
if [[ -x "$PYTHON" && -f "$PIO_HOME/packages/tool-esptoolpy/esptool.py" ]]; then
  ESPTOOL=("$PYTHON" "$PIO_HOME/packages/tool-esptoolpy/esptool.py")
elif command -v esptool.py >/dev/null 2>&1; then
  ESPTOOL=(esptool.py)
elif command -v esptool >/dev/null 2>&1; then
  ESPTOOL=(esptool)
else
  echo "esptool not found (install PlatformIO or 'pip install esptool')." >&2
  exit 1
fi

OUT_DIR="$SCRIPT_DIR"
OTA_BIN="$OUT_DIR/m2mqtt_${VERSION_FILE}-ESP32S3_OTA.bin"
MERGED_BIN="$OUT_DIR/m2mqtt_${VERSION_FILE}-ESP32S3_merged.bin"

# The OTA image (for /upgrade) is the app built in this run, so the merged
# image can never contain an older app than its file name says.
cp "$APP_BIN" "$OTA_BIN"

"${ESPTOOL[@]}" --chip esp32s3 merge_bin -o "$MERGED_BIN" \
  --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0000 "$BOOTLOADER_BIN" \
  0x8000 "$PARTITIONS_BIN" \
  0xe000 "$BOOT_APP0_BIN" \
  0x10000 "$OTA_BIN"

echo
echo "OTA image:    $OTA_BIN"
echo "Merged image: $MERGED_BIN"
