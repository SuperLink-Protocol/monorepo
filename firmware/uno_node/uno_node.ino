/*
 * uno_node.ino — SMART-LINK C++ Firmware for Arduino Uno (NODE_ID = 0)
 * =====================================================================
 * Fits inside 2 KB SRAM / 32 KB Flash using `smartlink_core.h` with
 * SL_NO_FM_RX and SL_MAX_FRAME=120.
 *
 * Transports:
 *   - USB Serial (38400 baud) — plug directly into HP ProBook / HP Victus / Pi via USB
 *   - SoftwareSerial D10 (RX) / D11 (TX) @ 38400 baud — wired daisy-chain to ESP32
 *     (Remember: 1kOhm + 2kOhm voltage divider on D11 -> ESP32 GPIO16!)
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#define SL_NO_FM_RX
#define SL_MAX_FRAME 120
#include "smartlink_core.h"

#define NODE_ID     0
#define NODE_NAME   "Arduino-Uno"
#define NODE_ROLE   "origin"
#define CARRIER_IDX 0          // 0 = 75 kHz (750 Hz piezo tone)

#define LINK_RX_PIN 10
#define LINK_TX_PIN 11
#define BUZZER_PIN  9
#define LED_PIN     13
#define LINK_BAUD   38400

static SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);
static SlLineDec usbDec, linkDec;
static uint8_t frameBuf[SL_MAX_FRAME];
static uint16_t mySeq = 1;
static uint32_t lastHbMs = 0;

static void emitLine(const uint8_t* f, uint16_t len) {
  Serial.print(F("SL1 "));
  linkSerial.print(F("SL1 "));
  for (uint16_t i = 0; i < len; i++) {
    char hi = sl_hexc(f[i] >> 4), lo = sl_hexc(f[i] & 15);
    Serial.write(hi); Serial.write(lo);
    linkSerial.write(hi); linkSerial.write(lo);
  }
  Serial.println();
  linkSerial.println();
}


static void sendHeartbeat() {
  char pay[36];
  int plen = snprintf(pay, sizeof(pay), "%s|%s|%u|", NODE_NAME, NODE_ROLE, (unsigned)SL_CAP_UART);
  uint16_t len = sl_build(frameBuf, sizeof(frameBuf), SL_T_HB, SL_HB_TTL, mySeq++, NODE_ID,
                          SL_BROADCAST, nullptr, 0, (const uint8_t*)pay, (uint8_t)plen);
  if (!len) return;
  len = sl_append_hop(frameBuf, len, sizeof(frameBuf), NODE_ID, CARRIER_IDX);
  if (len) emitLine(frameBuf, len);
}

static void handleFrame(const uint8_t* raw, uint16_t len) {
  SlView v;
  if (!sl_parse(raw, len, &v)) return;
  if (v.src == NODE_ID) return;

  if (v.type == SL_T_MSG && (v.dst == NODE_ID || v.dst == SL_BROADCAST)) {
    int ok = sl_verify(raw, &v);
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, (unsigned int)(SL_CARRIERS_KHZ[CARRIER_IDX] * 10), 60);
    if (ok) {
      // Decrypt payload in-place in frameBuf and print human-readable confirmation
      uint8_t mlen = v.L < 64 ? v.L : 64;
      memcpy(frameBuf, raw + v.off_payload, mlen);
      sl_crypt(v.src, v.seq, frameBuf, mlen);
      frameBuf[mlen] = '\0';
      Serial.print(F("[Arduino-Uno RX] seq="));
      Serial.print(v.seq);
      Serial.print(F(" msg=\""));
      Serial.print((const char*)frameBuf);
      Serial.println(F("\" | STATUS: SUCCESS 400"));

      // Send ACK back to source
      uint8_t ackPay[4] = {(uint8_t)(v.seq >> 8), (uint8_t)v.seq,
                           (uint8_t)(SL_CODE_SUCCESS >> 8), (uint8_t)SL_CODE_SUCCESS};
      uint8_t rt[4] = {NODE_ID, CARRIER_IDX, v.src, CARRIER_IDX};
      uint16_t alen = sl_build(frameBuf, sizeof(frameBuf), SL_T_ACK, 0, mySeq++, NODE_ID, v.src,
                               rt, 2, ackPay, 4);
      alen = sl_append_hop(frameBuf, alen, sizeof(frameBuf), NODE_ID, CARRIER_IDX);
      if (alen) emitLine(frameBuf, alen);
    } else {
      Serial.println(F("[Arduino-Uno RX] TAMPER | STATUS: ERROR 401"));
    }
    delay(20);
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.begin(LINK_BAUD);
  linkSerial.begin(LINK_BAUD);
  pinMode(LINK_RX_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  sl_dec_init(&usbDec);
  sl_dec_init(&linkDec);
  sendHeartbeat();
}

void loop() {
  while (Serial.available() > 0) {
    uint16_t n = sl_dec_feed(&usbDec, (char)Serial.read());
    if (n) handleFrame(usbDec.buf, n);
  }
  while (linkSerial.available() > 0) {
    uint16_t n = sl_dec_feed(&linkDec, (char)linkSerial.read());
    if (n) handleFrame(linkDec.buf, n);
  }
  if (millis() - lastHbMs >= 2000) {
    lastHbMs = millis();
    sendHeartbeat();
  }
}
