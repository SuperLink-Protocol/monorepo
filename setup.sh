#!/usr/bin/env bash
# setup.sh — One-command SMART-LINK setup for Raspberry Pi OS / Debian / Ubuntu
# Usage:
#   chmod +x setup.sh && ./setup.sh
#   ./setup.sh --with-arduino    # also installs arduino-cli + AVR/ESP32 cores for USB flashing

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WITH_ARDUINO=0

for arg in "$@"; do
  case "$arg" in
    --with-arduino) WITH_ARDUINO=1 ;;
    -h|--help)
      echo "Usage: ./setup.sh [--with-arduino]"
      exit 0
      ;;
  esac
done

echo "=== [1/5] Installing system dependencies (g++, cmake, alsa-utils) ==="
if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update -y
  sudo apt-get install -y build-essential cmake alsa-utils curl
elif command -v dnf >/dev/null 2>&1; then
  sudo dnf install -y gcc-c++ cmake alsa-utils curl
elif command -v pacman >/dev/null 2>&1; then
  sudo pacman -Sy --noconfirm base-devel cmake alsa-utils curl
else
  echo "[WARN] Unknown package manager. Ensure g++, cmake, and aplay/arecord are installed."
fi

if [[ "$WITH_ARDUINO" -eq 1 ]]; then
  echo "=== [Optional] Installing arduino-cli + AVR & ESP32 cores ==="
  if ! command -v arduino-cli >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
    export PATH="$HOME/.local/bin:$PATH"
  fi
  arduino-cli config init --overwrite || true
  arduino-cli config add board_manager.additional_urls \
    https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install arduino:avr esp32:esp32
fi

echo "=== [2/5] Granting USB-Serial permissions (dialout group) ==="
if getent group dialout >/dev/null 2>&1; then
  sudo usermod -aG dialout "$USER" || true
fi

echo "=== [3/5] Syncing shared C++ header & making scripts executable ==="
chmod +x "$ROOT/smartlink.sh" "$ROOT/setup.sh"
"$ROOT/smartlink.sh" sync

echo "=== [4/5] Building C++17 SMART-LINK Inference Engine (Release) ==="
"$ROOT/smartlink.sh" build

echo "=== [5/5] Running End-to-End Self-Test ==="
"$ROOT/smartlink.sh" test

echo ""
echo "================================================================"
echo "  SMART-LINK Setup Complete!"
echo "  Run as Relay (Raspberry Pi):  ./smartlink.sh relay"
echo "  Run as HQ (Terminal CLI):     ./smartlink.sh hq"
echo "  Attach USB board:             ./smartlink.sh relay --serial /dev/ttyUSB0"
echo "================================================================"
