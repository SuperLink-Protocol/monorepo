# SMART-LINK — C++17 Mesh, Inference Engine & FM Radio Playground

A **100% C++ / C++17, zero-external-dependency** implementation of the SMART-LINK resilient multi-hop network across heterogeneous hardware:

| Device | Role | Language / Build | Transports |
| :--- | :--- | :--- | :--- |
| **HP ProBook** | **HQ-1** (`ID=4`) | C++17 (`smartlink.exe` via `smartlink.ps1`) | Wi-Fi UDP (`5050`), USB-Serial, FM Audio (`winmm`) |
| **HP Victus** | **HQ-2** (`ID=5`) | C++17 (`smartlink.exe` via `smartlink.ps1`) | Wi-Fi UDP (`5050`), USB-Serial, FM Audio (`winmm`) |
| **Raspberry Pi** (Pi OS) | **Relay** (`ID=3`) | C++17 (`smartlink` via `smartlink.sh`) | Wi-Fi UDP (`5050`), USB-Serial, FM Audio (`aplay`/`arecord` or `rpitx`) |
| **ESP32 #1 & #2** | **Relays** (`ID=1, 2`) | Arduino C++ (`firmware/esp32_node`) | Wi-Fi UDP (`5050`), `Serial2` (`GPIO16/17`), USB Serial |
| **Arduino Uno** | **Origin** (`ID=0`) | AVR C++ (`firmware/uno_node`) | USB Serial (`38400`), `SoftwareSerial` (`D10/D11`), Piezo (`D9`) |
| **Any New Node** | **Dynamic** (`ID=6..250`) | Plug-and-play `HB` discovery | Auto-registered into $G=(V,E)$ when powered on! |

---

## 1. Repository Layout

- [firmware/common/smartlink_core.h](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/firmware/common/smartlink_core.h) — Single-source-of-truth header: SHA-256, HMAC-SHA256, RFC 5869 HKDF-SHA256, `SL1` binary frame + CRC16-CCITT, chained hop-audit tags $\tau_i$, SHA-256 CTR payload cipher, and FM subcarrier FSK modem.
- [engine/](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/engine/CMakeLists.txt) — C++17 host engine (`main.cpp`, `inference.cpp`, `transport.cpp`, `fm.cpp`) for HP ProBook, HP Victus, and Raspberry Pi OS.
- [firmware/esp32_node/esp32_node.ino](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/firmware/esp32_node/esp32_node.ino) — ESP32 relay firmware (with `GPIO13` live tamper button and `GPIO15` piezo carrier tone).
- [firmware/uno_node/uno_node.ino](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/firmware/uno_node/uno_node.ino) — Arduino Uno firmware (fits inside 2 KB SRAM).
- [smartlink.ps1](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/smartlink.ps1) — Windows PowerShell CLI launcher (`sync`, `build`, `test`, `hq`, `ports`).
- [smartlink.sh](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/smartlink.sh) — Raspberry Pi OS launcher (`sync`, `build`, `test`, `relay`, `hq`).
- [AGENT_HANDOFF.md](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/AGENT_HANDOFF.md) — Instructions for another AI agent to flash USB-connected Uno/ESP32 boards and bring up the mesh.

---

## 2. Quick Start (HP ProBook & HP Victus — Windows)

1. **Run the C++17 End-to-End Self-Test** (verifies HMAC chain, FM FSK modem, self-healing `E_rec` rerouting, and payload decryption):
   ```powershell
   .\smartlink.ps1 test
   ```
2. **Launch HQ Terminal Playground on HP ProBook** (with optional USB-connected Uno/ESP32 on `COM3`):
   ```powershell
   .\smartlink.ps1 hq -Name HP-ProBook -Id 4 -Serial COM3
   ```
3. **Launch HQ-2 on HP Victus** (listening on FM RX + UDP):
   ```powershell
   .\smartlink.ps1 hq -Name HP-Victus -Id 5 -FmRx
   ```

### HQ Playground CLI Commands
Once inside `smartlink-hq>`:
- Type any text (e.g. `Medical convoy moving to sector 4`) → encrypts in real time, assigns conflict-free carriers (`75, 84, 95, 108, 120, 135 kHz`), transmits over **FM audio subcarrier (`7.5–13.5 kHz` FSK)** + UDP + USB-Serial, and outputs **`[STATUS: SUCCESS 400]`**.
- `status` — View all online/offline nodes, active `T` (tree) / `B` (backup) / `R` (`E_rec`) links, and assigned kHz carriers.
- `unplug ESP32-1` / `plug ESP32-1` — Simulate unplugging a node (or physically unplug an ESP32 and wait 6 s for heartbeat timeout) to watch the Inference Engine automatically recompute routes and carriers.
- `fm on` / `fm off` — Toggle live FM audio subcarrier transmission.
- `ports` — List available COM/USB serial ports.

---

## 3. Quick Start (Raspberry Pi OS)

After cloning the repo, run the one-command setup script (installs `g++`, `cmake`, `alsa-utils`, syncs headers, builds Release, and runs self-test):
```bash
chmod +x setup.sh && ./setup.sh
# Optional: include arduino-cli + AVR/ESP32 cores for USB flashing:
# ./setup.sh --with-arduino
```

Then start the node:
```bash
NODE_NAME=Pi-Node NODE_ID=3 ./smartlink.sh relay --serial /dev/ttyUSB0
```
*(To transmit real RF on Raspberry Pi via an external FM tool like `rpitx`, pass `--fm-tx-cmd "sudo rpitx -m FM -f 100.1e6 -i {wav}"`.)*

---

## 4. Flashing ESP32 & Arduino Uno

1. Run `.\smartlink.ps1 sync` (or `./smartlink.sh sync`) so `smartlink_core.h` is copied into both `firmware/esp32_node/` and `firmware/uno_node/`.
2. **ESP32 #1 & #2**: Open [esp32_node.ino](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/firmware/esp32_node/esp32_node.ino) in Arduino IDE, set unique `NODE_ID` (`1`, `2`, or `6+` for new nodes), `NODE_NAME`, `CARRIER_IDX` (`0..5`), and `WIFI_SSID`/`WIFI_PASS`, then upload.
3. **Arduino Uno**: Open [uno_node.ino](file:///c:/Users/NACHIKETH/Desktop/SMART-LINK/firmware/uno_node/uno_node.ino) and upload.
4. **Wired UART1/2 Safety**: If wiring Uno `D11` (5V TX) directly to ESP32 `GPIO16` (3.3V RX), use a `1 kΩ + 2 kΩ` voltage divider and a common `GND`.

---

## 5. FM Communication Note
SMART-LINK maps its six carriers `{75, 84, 95, 108, 120, 135} kHz` to **audio-band FM subcarriers (`7.5–13.5 kHz`, 300-baud FSK, ±150 Hz deviation)**. Standard laptop speakers/line-out (`winmm`) and Pi audio (`aplay`) generate and decode this baseband FM signal directly; feeding the laptop's audio output into a **low-power FM transmitter module** (or SDR) radiates it over VHF FM radio, while a standard FM receiver plugged into the receiving laptop's mic/line-in (`--fm-rx`) demodulates all 6 channels in parallel.
