/*
 * esp32_node.ino — SMART-LINK C++ Firmware for ESP32 (Relays 1 & 2, or any new ESP32!)
 * ====================================================================================
 * Uses the shared single-source-of-truth header `smartlink_core.h` (copied into this
 * sketch folder by `smartlink.ps1 sync`).
 *
 * Transports (simultaneous):
 *   - USB Serial (115200 baud) — plug directly into HP ProBook / HP Victus / Pi via USB
 *   - UART2 (GPIO16 RX2 / GPIO17 TX2 @ 38400 baud) — wired daisy-chain
 *   - Wi-Fi UDP broadcast (port 5050) — wireless mesh with HP ProBook, HP Victus & Pi
 *
 * Peripherals:
 *   - GPIO15: Piezo buzzer / FM audio subcarrier tone (7.5..13.5 kHz or ÷10 audible)
 *   - GPIO13: Tamper button to GND (INPUT_PULLUP) — corrupts hop record after signing
 *   - GPIO2 : Status LED
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#define SL_NO_FM_RX
#include "smartlink_core.h"

// ==================== NODE CONFIGURATION ====================
#define NODE_ID     3          // 1 for ESP32-1, 2 for ESP32-2 (or any unique ID 1..250)
#define NODE_NAME   "ESP32-3"
#define NODE_ROLE   "relay"
#define CARRIER_IDX 3          // 0=75kHz, 1=84kHz, 2=95kHz, 3=108kHz, 4=120kHz, 5=135kHz

// Wi-Fi UDP mesh (auto-connects in background to NodeMCU v3 SMART-LINK-HQ AP)
#define WIFI_SSID   "SMART-LINK-HQ"
#define WIFI_PASS   "smartlink123"
#define MESH_PORT   5050
// ============================================================

#define RXD2_PIN    16
#define TXD2_PIN    17
#define TAMPER_PIN  13
#define BUZZER_PIN  15
#define LED_PIN     2

static WiFiUDP udp;
static bool udpStarted = false;
static SlLineDec usbDec, uart2Dec, udpDec;
static uint16_t mySeq = 1;
static uint32_t lastHbMs = 0;

struct SeenEntry { uint32_t key; uint32_t ms; };
static SeenEntry seen[32];
static uint8_t seenPos = 0;

static bool isDuplicate(uint8_t type, uint8_t src, uint16_t seq) {
  uint32_t k = ((uint32_t)type << 24) | ((uint32_t)src << 16) | seq;
  uint32_t now = millis();
  for (uint8_t i = 0; i < 32; i++) {
    if (seen[i].key == k && (now - seen[i].ms) < 12000) return true;
  }
  seen[seenPos] = {k, now};
  seenPos = (seenPos + 1) & 31;
  return false;
}

static void emitLine(const uint8_t* f, uint16_t len) {
  static char line[SL_MAX_FRAME * 2 + 8];
  memcpy(line, "SL1 ", 4);
  for (uint16_t i = 0; i < len; i++) {
    line[4 + 2 * i]     = sl_hexc(f[i] >> 4);
    line[4 + 2 * i + 1] = sl_hexc(f[i] & 15);
  }
  line[4 + 2 * len] = '\0';

  Serial.println(line);
  Serial2.println(line);
  if (WiFi.status() == WL_CONNECTED && udpStarted) {
    IPAddress bcast = WiFi.broadcastIP();
    if (bcast == IPAddress(0, 0, 0, 0)) bcast = IPAddress(255, 255, 255, 255);
    udp.beginPacket(bcast, MESH_PORT);
    udp.println(line);
    udp.endPacket();
  }
}

static void sendHeartbeat() {
  char pay[48];
  uint8_t caps = SL_CAP_UART | (WiFi.status() == WL_CONNECTED ? SL_CAP_WIFI : 0);
  int plen = snprintf(pay, sizeof(pay), "%s|%s|%u|", NODE_NAME, NODE_ROLE, (unsigned)caps);
  uint8_t f[SL_MAX_FRAME];
  uint16_t len = sl_build(f, sizeof(f), SL_T_HB, SL_HB_TTL, mySeq++, NODE_ID,
                          SL_BROADCAST, nullptr, 0, (const uint8_t*)pay, (uint8_t)plen);
  if (!len) return;
  len = sl_append_hop(f, len, sizeof(f), NODE_ID, CARRIER_IDX);
  if (len) {
    digitalWrite(LED_PIN, HIGH);
    emitLine(f, len);
    delay(40);
    digitalWrite(LED_PIN, LOW);
  }
}

static void handleFrame(const uint8_t* raw, uint16_t len) {
  SlView v;
  if (!sl_parse(raw, len, &v)) return;
  if (v.src == NODE_ID && v.H <= 1) return;
  if (isDuplicate(v.type, v.src, v.seq)) return;

  if (v.type == SL_T_HB) {
    if (v.flags > 1 && v.H < SL_MAX_HOPS) {
      uint8_t fwd[SL_MAX_FRAME];
      memcpy(fwd, raw, len);
      sl_set_flags(fwd, len, (uint8_t)(v.flags - 1));
      sl_set_tx(fwd, len, NODE_ID);
      uint16_t flen = sl_append_hop(fwd, len, sizeof(fwd), NODE_ID, CARRIER_IDX);
      if (flen) emitLine(fwd, flen);
    }
    return;
  }

  if (v.dst == NODE_ID || v.dst == SL_BROADCAST) {
    int ok = sl_verify(raw, &v);
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, (unsigned int)(SL_CARRIERS_KHZ[CARRIER_IDX] * 10), 60);
    if (v.type == SL_T_MSG) {
      uint8_t pt[SL_MAX_FRAME];
      memcpy(pt, raw + v.off_payload, v.L);
      sl_crypt(v.src, v.seq, pt, v.L);
      pt[v.L] = '\0';
      Serial.printf("[%s RX] seq=%u msg=\"%s\" | STATUS: %s %d\n",
                    NODE_NAME, v.seq, (const char*)pt,
                    ok ? "SUCCESS" : "TAMPER", ok ? SL_CODE_SUCCESS : SL_CODE_TAMPER);
      if (ok && v.src != NODE_ID) {
        uint8_t ackPay[4] = {(uint8_t)(v.seq >> 8), (uint8_t)v.seq,
                             (uint8_t)(SL_CODE_SUCCESS >> 8), (uint8_t)SL_CODE_SUCCESS};
        uint8_t rt[4] = {NODE_ID, CARRIER_IDX, v.src, CARRIER_IDX};
        uint8_t af[SL_MAX_FRAME];
        uint16_t alen = sl_build(af, sizeof(af), SL_T_ACK, 0, mySeq++, NODE_ID, v.src, rt, 2, ackPay, 4);
        alen = sl_append_hop(af, alen, sizeof(af), NODE_ID, CARRIER_IDX);
        if (alen) emitLine(af, alen);
      }
    }
    for (int b = 0; b < 3; b++) {
      digitalWrite(LED_PIN, HIGH);
      delay(140);
      digitalWrite(LED_PIN, LOW);
      delay(80);
    }
    return;
  }

  // Forward if we are on the source route
  int nextIdx = -1;
  uint8_t cidx = CARRIER_IDX;
  for (uint8_t i = 0; i + 1 < v.R; i++) {
    if (raw[v.off_route + 2 * i] == NODE_ID) {
      nextIdx = i + 1;
      cidx = raw[v.off_route + 2 * i + 1];
      break;
    }
  }
  if (nextIdx < 0) return;

  uint8_t fwd[SL_MAX_FRAME];
  memcpy(fwd, raw, len);
  sl_set_tx(fwd, len, NODE_ID);
  uint16_t flen = sl_append_hop(fwd, len, sizeof(fwd), NODE_ID, cidx);
  if (!flen) return;

  // Live MITM tamper demo: holding GPIO13 LOW corrupts the hop carrier AFTER HMAC signing
  if (digitalRead(TAMPER_PIN) == LOW) {
    SlView fv;
    if (sl_parse(fwd, flen, &fv) && fv.H > 0) {
      fwd[fv.off_hops + 2 * (fv.H - 1) + 1] = 5;  // corrupt carrier index without re-signing
      sl_fix_crc(fwd, flen);
    }
  }

  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, (unsigned int)(SL_CARRIERS_KHZ[cidx < SL_NUM_CARRIERS ? cidx : 0] * 10), 60);
  emitLine(fwd, flen);
  delay(15);
  digitalWrite(LED_PIN, LOW);
}

static bool wifiStarted = false;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // prevent false brownout resets on breadboard rails
  setCpuFrequencyMhz(80);                     // cuts idle/active current ~55% while staying fast

  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
  pinMode(TAMPER_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  sl_dec_init(&usbDec);
  sl_dec_init(&uart2Dec);
  sl_dec_init(&udpDec);

  sendHeartbeat();
}

void loop() {
  // Stagger Wi-Fi startup by NODE_ID (2.5s apart) so multiple ESP32s on one breadboard rail
  // never trigger simultaneous RF calibration spikes.
  if (!wifiStarted && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0 &&
      millis() >= (uint32_t)NODE_ID * 2500UL) {
    wifiStarted = true;
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  // low peak current, plenty of room-scale range
    WiFi.setSleep(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  if (wifiStarted && WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udp.begin(MESH_PORT);
      udpStarted = true;
      Serial.printf("[%s WIFI] Connected to %s (%s)\n",
                    NODE_NAME, WIFI_SSID, WiFi.localIP().toString().c_str());
      sendHeartbeat();
    }
  } else if (udpStarted) {
    udp.stop();
    udpStarted = false;
  }

  while (Serial.available() > 0) {
    uint16_t n = sl_dec_feed(&usbDec, (char)Serial.read());
    if (n) handleFrame(usbDec.buf, n);
  }
  while (Serial2.available() > 0) {
    uint16_t n = sl_dec_feed(&uart2Dec, (char)Serial2.read());
    if (n) handleFrame(uart2Dec.buf, n);
  }
  if (udpStarted) {
    int pktSize = udp.parsePacket();
    while (pktSize-- > 0) {
      int ch = udp.read();
      if (ch >= 0) {
        uint16_t n = sl_dec_feed(&udpDec, (char)ch);
        if (n) handleFrame(udpDec.buf, n);
      }
    }
  }
  if (millis() - lastHbMs >= 1500) {
    lastHbMs = millis();
    sendHeartbeat();
  }
}
