#!/usr/bin/env bash
# Builds and runs the host-side stability tests with AddressSanitizer and
# UndefinedBehaviorSanitizer.
#
#   test/run_host_tests.sh
#
# ArduinoJson 6 is taken from ARDUINOJSON_SRC if set, otherwise from the
# PlatformIO libdeps folder (run `pio pkg install -e esp07` first).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CXX="${CXX:-c++}"
OUT="${OUT:-$ROOT/.pio/host-tests}"

if [[ -z "${ARDUINOJSON_SRC:-}" ]]; then
  ARDUINOJSON_SRC="$ROOT/.pio/libdeps/esp07/ArduinoJson/src"
fi
if [[ ! -f "$ARDUINOJSON_SRC/ArduinoJson.h" ]]; then
  echo "ArduinoJson not found at $ARDUINOJSON_SRC" >&2
  echo "Run 'pio pkg install -e esp07' or set ARDUINOJSON_SRC." >&2
  exit 2
fi

mkdir -p "$OUT"

# -funsigned-char: both Xtensa toolchains (ESP8266, ESP32-S3) use unsigned
# char, which HeatPump::readPacket() relies on when comparing header bytes.

"$CXX" -std=c++17 -g -O1 -Wall -Wextra -Wno-unused-parameter \
  -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all \
  -funsigned-char \
  -DESP32 -DLOG_SIZE=4096 -DARDUINOJSON_ENABLE_ARDUINO_STRING=1 \
  -I "$ROOT/test/host" \
  -I "$ROOT/src/mitsubishi2mqtt" \
  -I "$ROOT/lib/HeatPump/src" \
  -I "$ARDUINOJSON_SRC" \
  "$ROOT/test/test_stability_host.cpp" \
  "$ROOT/src/mitsubishi2mqtt/logger.cpp" \
  "$ROOT/lib/HeatPump/src/HeatPump.cpp" \
  -o "$OUT/test_stability_host"

# LeakSanitizer is not available on macOS; keep it on elsewhere.
if [[ "$(uname)" == "Darwin" ]]; then
  export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}"
fi
"$OUT/test_stability_host"
