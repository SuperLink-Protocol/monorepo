#!/usr/bin/env bash
# SMART-LINK C++17 Launcher & Build Script (Raspberry Pi OS / Linux)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$ROOT/engine/build/smartlink"

sync_core() {
  cp -f "$ROOT/firmware/common/smartlink_core.h" "$ROOT/firmware/esp32_node/smartlink_core.h"
  cp -f "$ROOT/firmware/common/smartlink_core.h" "$ROOT/firmware/uno_node/smartlink_core.h"
  echo "[SYNC] Copied smartlink_core.h to esp32_node and uno_node"
}

build_engine() {
  sync_core
  cmake -S "$ROOT/engine" -B "$ROOT/engine/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$ROOT/engine/build" -j"$(nproc 2>/dev/null || echo 2)"
  echo "[BUILD] Ready: $BIN"
}

CMD="${1:-relay}"
shift || true

case "$CMD" in
  sync)  sync_core ;;
  build) build_engine ;;
  test)
    [[ -x "$BIN" ]] || build_engine
    "$BIN" --self-test
    ;;
  relay|hq|chat)
    [[ -x "$BIN" ]] || build_engine
    exec "$BIN" --role "$CMD" --name "${NODE_NAME:-Pi-Node}" --id "${NODE_ID:-3}" "$@"
    ;;
  *)
    echo "Usage: $0 {sync|build|test|chat|relay|hq} [extra smartlink flags]"
    exit 1
    ;;
esac
