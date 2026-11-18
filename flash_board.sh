#!/usr/bin/env bash
# flash_board.sh — Compile & flash permanent SMART-LINK C++ firmware onto ESP32 or Arduino Uno
# Usage:
#   bash flash_board.sh esp1 [PORT] [WIFI_SSID] [WIFI_PASS]
#   bash flash_board.sh esp2 [PORT] [WIFI_SSID] [WIFI_PASS]
#   bash flash_board.sh uno  [PORT]

set -euo pipefail
export PATH="/home/cosmic/.local/bin:$PATH"

ROLE="${1:-}"
PORT="${2:-}"
SSID="${3:-YOUR_WIFI_SSID}"
PASS="${4:-YOUR_WIFI_PASS}"

ROOT="/home/cosmic/monorepo"
ACLI="/home/cosmic/.local/bin/arduino-cli"

if [[ -z "$ROLE" ]]; then
  echo "Usage: $0 {esp1|esp2|uno} [PORT] [WIFI_SSID] [WIFI_PASS]"
  exit 1
fi

# Auto-detect serial port if omitted
if [[ -z "$PORT" ]]; then
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$dev" ]]; then
      PORT="$dev"
      break
    fi
  done
fi

if [[ -z "$PORT" || ! -e "$PORT" ]]; then
  echo "[ERROR] No USB board found on /dev/ttyUSB* or /dev/ttyACM*."
  echo "Plug in the board with a data-capable USB cable and try again."
  exit 1
fi

echo "[INFO] Detected board port: $PORT"
"$ROOT/smartlink.sh" sync

if [[ "$ROLE" == "uno" ]]; then
  echo "=== Compiling & Flashing Arduino Uno (NODE_ID=0, 75 kHz, 38400 baud) on $PORT ==="
  "$ACLI" compile --fqbn arduino:avr:uno "$ROOT/firmware/uno_node"
  "$ACLI" upload -p "$PORT" --fqbn arduino:avr:uno "$ROOT/firmware/uno_node"
  echo "[SUCCESS] Arduino Uno flashed permanently! Auto-starts on every power-up."
  exit 0
fi

if [[ "$ROLE" == "esp1" ]]; then
  NID=1; NNAME="ESP32-1"; CIDX=1
elif [[ "$ROLE" == "esp2" ]]; then
  NID=2; NNAME="ESP32-2"; CIDX=2
else
  echo "[ERROR] Unknown role '$ROLE'. Use esp1, esp2, or uno."
  exit 1
fi

INO="$ROOT/firmware/esp32_node/esp32_node.ino"
python3 - "$INO" "$NID" "$NNAME" "$CIDX" "$SSID" "$PASS" <<'PYEOF'
import re, sys
path, nid, nname, cidx, ssid, pw = sys.argv[1:7]
s = open(path, "r", encoding="utf-8").read()
s = re.sub(r"(#define\s+NODE_ID\s+)\d+", rf"\g<1>{nid}", s)
s = re.sub(r'(#define\s+NODE_NAME\s+)"[^"]*"', rf'\g<1>"{nname}"', s)
s = re.sub(r"(#define\s+CARRIER_IDX\s+)\d+", rf"\g<1>{cidx}", s)
s = re.sub(r'(#define\s+WIFI_SSID\s+)"[^"]*"', rf'\g<1>"{ssid}"', s)
s = re.sub(r'(#define\s+WIFI_PASS\s+)"[^"]*"', rf'\g<1>"{pw}"', s)
open(path, "w", encoding="utf-8").write(s)
print(f"[CONFIG] Set NODE_ID={nid}, NODE_NAME=\"{nname}\", CARRIER_IDX={cidx}, WIFI_SSID=\"{ssid}\"")
PYEOF

echo "=== Compiling & Flashing $NNAME (NODE_ID=$NID) on $PORT ==="
"$ACLI" compile --fqbn esp32:esp32:esp32 "$ROOT/firmware/esp32_node"
"$ACLI" upload -p "$PORT" --fqbn esp32:esp32:esp32 --upload-property upload.speed=115200 "$ROOT/firmware/esp32_node"
echo "[SUCCESS] $NNAME flashed permanently to ESP32 Flash ROM! It will auto-start on every reboot/power-up."
