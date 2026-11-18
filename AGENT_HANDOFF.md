# AGENT HANDOFF: USB Deployment & Multi-Node Bring-Up

This document is for an automated coding/deployment agent tasked with flashing and running the **SMART-LINK C++** codebase on physically connected **Arduino Uno**, **ESP32 (×2)**, **Raspberry Pi**, and **HP ProBook / HP Victus** nodes via USB.

---

## 1. Repository Truth & Rules
1. **Single-source protocol header**: `firmware/common/smartlink_core.h`. Never edit copies inside `firmware/esp32_node/` or `firmware/uno_node/` directly. Always edit `firmware/common/smartlink_core.h` and run `.\smartlink.ps1 sync` (Windows) or `./smartlink.sh sync` (Linux).
2. **Protocol**: Binary `SL1` frames encoded as `SL1 <hex>\n` over USB-Serial (`38400` baud for Uno, `115200` baud for ESP32 USB, `38400` baud for ESP32 `Serial2`), UDP port `5050`, and FM audio subcarrier FSK.
3. **Success signal**: Delivered & HMAC-verified messages must emit `STATUS: SUCCESS 400`.

---

## 2. Step-by-Step USB Deployment Procedure

### Step 1: Sync & Verify Host C++17 Engine
On Windows (HP ProBook / HP Victus):
```powershell
.\smartlink.ps1 sync
.\smartlink.ps1 test
```
Expected output ends with:
`ALL SELF-TESTS PASSED -> STATUS: SUCCESS 400`

### Step 2: Detect USB-Connected Boards
- **Windows**:
  ```powershell
  .\smartlink.ps1 ports
  Get-PnpDevice -Class Ports -Status OK | Select-Object FriendlyName, InstanceId
  ```
- **Raspberry Pi / Linux**:
  ```bash
  ls -l /dev/ttyUSB* /dev/ttyACM*
  ```
- If `arduino-cli` is installed, prefer:
  ```bash
  arduino-cli board list
  ```

### Step 3: Flash Arduino Uno (`NODE_ID = 0`)
Sketch: `firmware/uno_node/uno_node.ino` (FQBN: `arduino:avr:uno`).
```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/uno_node
arduino-cli upload -p <UNO_PORT> --fqbn arduino:avr:uno firmware/uno_node
```
- Baud rate on USB `Serial`: `38400`.
- Verify by opening `<UNO_PORT>` at `38400` baud: you should see `SL1 5301...` heartbeat lines every 2 seconds.

### Step 4: Flash ESP32 #1 (`NODE_ID = 1`) & ESP32 #2 (`NODE_ID = 2`)
Sketch: `firmware/esp32_node/esp32_node.ino` (FQBN: `esp32:esp32:esp32`).
Before compiling each board, set these macros in `firmware/esp32_node/esp32_node.ino`:
- **ESP32 #1**: `NODE_ID 1`, `NODE_NAME "ESP32-1"`, `CARRIER_IDX 1` (`84 kHz`), plus `WIFI_SSID` / `WIFI_PASS`.
- **ESP32 #2**: `NODE_ID 2`, `NODE_NAME "ESP32-2"`, `CARRIER_IDX 2` (`95 kHz`), plus `WIFI_SSID` / `WIFI_PASS`.
- **Any additional ESP32**: assign unique `NODE_ID` (`6..250`), `NODE_NAME`, and `CARRIER_IDX` (`0..5`).

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_node
arduino-cli upload -p <ESP32_PORT> --fqbn esp32:esp32:esp32 firmware/esp32_node
```
- Baud rate on USB `Serial`: `115200`.

### Step 5: Connect & Launch the Live Mesh
1. Attach USB-connected boards to the host (`HP-ProBook`, `HP-Victus`, or `Pi-Node`) using `--serial`:
   ```powershell
   # Example: Uno on COM3, ESP32-1 on COM4
   .\smartlink.ps1 hq -Name HP-ProBook -Id 4 -Serial COM3,COM4
   ```
   *(Note: if Uno is on `38400` and ESP32 USB is on `115200`, connect Uno to `COMx` at `--baud 38400` and let ESP32s join over Wi-Fi UDP `5050` or `Serial2` at `38400`.)*
2. On **Raspberry Pi OS**:
   ```bash
   ./smartlink.sh relay --serial /dev/ttyACM0
   ```
3. **End-to-End Verification Checklist**:
   - Run `status` in `smartlink-hq>` and confirm all connected nodes show `[ONLINE]`.
   - Type a test message (e.g., `hello smart-link`) and verify **`STATUS: SUCCESS 400`** is printed.
   - Physically unplug one ESP32 USB/power cable; wait ~6 seconds for `[HEARTBEAT LOST] ... -> Inference Engine rerouting!`; send another message and confirm **`STATUS: SUCCESS 400`** still succeeds over the surviving path.
