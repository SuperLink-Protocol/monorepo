/*
 * nodemcu_hq.ino — SMART-LINK Catppuccin Mocha HQ Dashboard & 8-Client Mesh AP (NodeMCU v3 / ESP8266)
 * ===================================================================================================
 * Hosts offline Wi-Fi Access Point ("SMART-LINK-HQ" / "smartlink123", max 8 clients) at 192.168.4.1
 * with:
 *   - Live Interactive SVG Mesh Topology (glowing links + tamper isolation + bypass reroute arc)
 *   - Live 6-Channel Carrier Spectrum Analyzer (75 / 84 / 95 / 108 / 120 / 135 kHz) with click-to-jam
 *   - Real-Time Crypto X-Ray Inspector (Plaintext -> SHA-256 CTR Ciphertext Hex -> HMAC-SHA256 Tag)
 *   - 3-Burst Synchronized UDP Broadcast (0 / 55 / 110 ms) for crisp 3x synced ESP32 LED flashes
 *   - Authentic Catppuccin Mocha theme
 *
 * Node Identity: NODE_ID = 5 ("NodeMCU-HQ"), CARRIER_IDX = 4 (120 kHz)
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#define SL_NO_FM_RX
#include "smartlink_core.h"

#define NODE_ID          5
#define NODE_NAME        "NodeMCU-HQ"
#define NODE_ROLE        "hq-ap"
#define CARRIER_IDX      4          // 4 = 120 kHz

#define AP_SSID          "SMART-LINK-HQ"
#define AP_PASS          "smartlink123"
#define AP_CHANNEL       1
#define AP_MAX_CLIENTS   8
#define MESH_PORT        5050
#define SL_CODE_REROUTED 206

static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_NETMASK(255, 255, 255, 0);
static const IPAddress AP_BCAST(192, 168, 4, 255);

static ESP8266WebServer server(80);
static DNSServer dnsServer;
static WiFiUDP udp;
static SlLineDec usbDec, udpDec;
static uint16_t mySeq = 1;
static uint32_t lastHbMs = 0;

struct NodeInfo {
  bool active;
  uint8_t id;
  uint8_t carrierIdx;
  char name[20];
  char role[12];
  uint32_t lastSeenMs;
  bool tampered;
  uint16_t rxCount;
};

#define MAX_NODES 10
static NodeInfo nodes[MAX_NODES];
static bool jammed[SL_NUM_CARRIERS] = {false, false, false, false, false, false};

struct LogEntry {
  uint32_t ms;
  char tag[12];
  char detail[96];
  uint16_t code;
};

#define MAX_LOGS 18
static LogEntry logs[MAX_LOGS];
static uint8_t logHead = 0;
static uint8_t logCount = 0;

struct XrayInfo {
  bool valid;
  uint16_t seq;
  uint16_t carrierKhz;
  uint16_t frameLen;
  uint16_t code;
  char srcName[20];
  char plain[64];
  char cipherHex[64];
  char hmacHex[44];
};
static XrayInfo lastXray;

struct SeenEntry { uint32_t key; uint32_t ms; };
static SeenEntry seen[24];
static uint8_t seenPos = 0;

static void bytesToHexGrouped(const uint8_t* src, uint8_t n, char* dst, size_t cap) {
  size_t pos = 0;
  for (uint8_t i = 0; i < n && pos + 3 < cap; i++) {
    if (i > 0 && (i % 2) == 0 && pos + 1 < cap) dst[pos++] = ' ';
    dst[pos++] = sl_hexc(src[i] >> 4);
    dst[pos++] = sl_hexc(src[i] & 15);
  }
  dst[pos] = '\0';
}

static uint8_t pickActiveCarrier(uint8_t preferred) {
  if (preferred < SL_NUM_CARRIERS && !jammed[preferred]) return preferred;
  for (uint8_t i = 0; i < SL_NUM_CARRIERS; i++) {
    if (!jammed[i]) return i;
  }
  return preferred < SL_NUM_CARRIERS ? preferred : 0;
}

static void addLog(const char* tag, const char* detail, uint16_t code) {
  LogEntry& e = logs[logHead];
  e.ms = millis();
  strncpy(e.tag, tag, sizeof(e.tag) - 1);
  e.tag[sizeof(e.tag) - 1] = '\0';
  strncpy(e.detail, detail, sizeof(e.detail) - 1);
  e.detail[sizeof(e.detail) - 1] = '\0';
  e.code = code;
  logHead = (logHead + 1) % MAX_LOGS;
  if (logCount < MAX_LOGS) logCount++;
}

static NodeInfo* upsertNode(uint8_t id, const char* name, const char* role, uint8_t cidx) {
  int freeIdx = -1;
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active && nodes[i].id == id) {
      if (name && name[0]) strncpy(nodes[i].name, name, sizeof(nodes[i].name) - 1);
      if (role && role[0]) strncpy(nodes[i].role, role, sizeof(nodes[i].role) - 1);
      if (cidx < SL_NUM_CARRIERS) nodes[i].carrierIdx = cidx;
      nodes[i].lastSeenMs = millis();
      return &nodes[i];
    }
    if (!nodes[i].active && freeIdx < 0) freeIdx = i;
  }
  if (freeIdx < 0) return nullptr;
  NodeInfo& n = nodes[freeIdx];
  memset(&n, 0, sizeof(n));
  n.active = true;
  n.id = id;
  n.carrierIdx = (cidx < SL_NUM_CARRIERS) ? cidx : 0;
  if (name && name[0]) strncpy(n.name, name, sizeof(n.name) - 1);
  else snprintf(n.name, sizeof(n.name), "Node-%u", (unsigned)id);
  if (role && role[0]) strncpy(n.role, role, sizeof(n.role) - 1);
  else strcpy(n.role, "relay");
  n.lastSeenMs = millis();
  return &n;
}

static const char* nodeNameById(uint8_t id) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].active && nodes[i].id == id) return nodes[i].name;
  }
  static char tmp[16];
  snprintf(tmp, sizeof(tmp), "Node-%u", (unsigned)id);
  return tmp;
}

static void recordXray(const uint8_t* raw, uint16_t len, const char* srcName,
                       const char* plain, uint8_t cidx, uint16_t code) {
  SlView v;
  if (!sl_parse(raw, len, &v)) return;
  lastXray.valid = true;
  lastXray.seq = v.seq;
  lastXray.carrierKhz = SL_CARRIERS_KHZ[cidx < SL_NUM_CARRIERS ? cidx : 0];
  lastXray.frameLen = len;
  lastXray.code = code;
  strncpy(lastXray.srcName, srcName, sizeof(lastXray.srcName) - 1);
  lastXray.srcName[sizeof(lastXray.srcName) - 1] = '\0';
  strncpy(lastXray.plain, plain, sizeof(lastXray.plain) - 1);
  lastXray.plain[sizeof(lastXray.plain) - 1] = '\0';
  uint8_t cn = v.L < 20 ? v.L : 20;
  bytesToHexGrouped(raw + v.off_payload, cn, lastXray.cipherHex, sizeof(lastXray.cipherHex));
  bytesToHexGrouped(raw + v.off_tag, SL_TAG_LEN, lastXray.hmacHex, sizeof(lastXray.hmacHex));
}

static bool isDuplicate(uint8_t type, uint8_t src, uint16_t seq) {
  uint32_t k = ((uint32_t)type << 24) | ((uint32_t)src << 16) | seq;
  uint32_t now = millis();
  for (uint8_t i = 0; i < 24; i++) {
    if (seen[i].key == k && (now - seen[i].ms) < 10000) return true;
  }
  seen[seenPos] = {k, now};
  seenPos = (seenPos + 1) % 24;
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
  udp.beginPacket(AP_BCAST, MESH_PORT);
  udp.println(line);
  udp.endPacket();
}

static void sendHeartbeat() {
  uint8_t cidx = pickActiveCarrier(CARRIER_IDX);
  char pay[48];
  int plen = snprintf(pay, sizeof(pay), "%s|%s|%u|", NODE_NAME, NODE_ROLE, (unsigned)(SL_CAP_UART | SL_CAP_WIFI));
  uint8_t f[SL_MAX_FRAME];
  uint16_t len = sl_build(f, sizeof(f), SL_T_HB, SL_HB_TTL, mySeq++, NODE_ID,
                          SL_BROADCAST, nullptr, 0, (const uint8_t*)pay, (uint8_t)plen);
  if (!len) return;
  len = sl_append_hop(f, len, sizeof(f), NODE_ID, cidx);
  if (len) {
    digitalWrite(LED_BUILTIN, LOW);
    emitLine(f, len);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  upsertNode(NODE_ID, NODE_NAME, NODE_ROLE, cidx);
}

static void handleFrame(const uint8_t* raw, uint16_t len) {
  SlView v;
  if (!sl_parse(raw, len, &v)) return;
  if (v.src == NODE_ID) return;
  if (isDuplicate(v.type, v.src, v.seq)) return;

  uint8_t cidx = (v.H > 0) ? raw[v.off_hops + 1] : 0;

  if (v.type == SL_T_HB) {
    char pay[64];
    uint8_t mlen = v.L < sizeof(pay) - 1 ? v.L : sizeof(pay) - 1;
    memcpy(pay, raw + v.off_payload, mlen);
    pay[mlen] = '\0';
    char* sep1 = strchr(pay, '|');
    char* sep2 = sep1 ? strchr(sep1 + 1, '|') : nullptr;
    if (sep1) *sep1 = '\0';
    if (sep2) *sep2 = '\0';
    const char* nname = pay[0] ? pay : nullptr;
    const char* nrole = (sep1 && sep1[1]) ? (sep1 + 1) : "relay";
    upsertNode(v.src, nname, nrole, cidx);
    return;
  }

  int ok = sl_verify(raw, &v);
  NodeInfo* n = upsertNode(v.src, nullptr, nullptr, cidx);
  if (n) n->rxCount++;

  if (v.type == SL_T_ACK) {
    uint16_t aseq = (v.L >= 2) ? ((uint16_t)raw[v.off_payload] << 8) | raw[v.off_payload + 1] : v.seq;
    uint16_t code = (v.L >= 4) ? ((uint16_t)raw[v.off_payload + 2] << 8) | raw[v.off_payload + 3] : SL_CODE_SUCCESS;
    char msg[96];
    snprintf(msg, sizeof(msg), "ACK seq=%u from %s (%u hop%s)",
             (unsigned)aseq, nodeNameById(v.src), (unsigned)v.H, v.H == 1 ? "" : "s");
    addLog("ACK", msg, ok ? code : SL_CODE_TAMPER);
    digitalWrite(LED_BUILTIN, LOW);
    delay(35);
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  if (v.type == SL_T_MSG) {
    uint8_t pt[SL_MAX_FRAME];
    uint8_t mlen = v.L < 60 ? v.L : 60;
    memcpy(pt, raw + v.off_payload, mlen);
    sl_crypt(v.src, v.seq, pt, mlen);
    pt[mlen] = '\0';
    const char* sname = nodeNameById(v.src);
    recordXray(raw, len, sname, (const char*)pt, cidx, ok ? SL_CODE_SUCCESS : SL_CODE_TAMPER);

    char msg[96];
    snprintf(msg, sizeof(msg), "%s -> \"%s\" (seq=%u)", sname, (const char*)pt, (unsigned)v.seq);
    addLog(ok ? "RX" : "TAMPER", msg, ok ? SL_CODE_SUCCESS : SL_CODE_TAMPER);
    if (!ok && n) n->tampered = true;

    for (int b = 0; b < 3; b++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(90);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(60);
    }
  }
}

static void sendBroadcastMsg(const String& text) {
  uint8_t payload[80];
  size_t mlen = text.length();
  if (mlen > 60) mlen = 60;
  memcpy(payload, text.c_str(), mlen);

  uint16_t seq = mySeq++;
  sl_crypt(NODE_ID, seq, payload, (uint8_t)mlen);

  uint8_t cidx = pickActiveCarrier(CARRIER_IDX);
  uint8_t f[SL_MAX_FRAME];
  uint16_t len = sl_build(f, sizeof(f), SL_T_MSG, 0, seq, NODE_ID,
                          SL_BROADCAST, nullptr, 0, payload, (uint8_t)mlen);
  if (!len) return;
  len = sl_append_hop(f, len, sizeof(f), NODE_ID, cidx);
  if (!len) return;

  recordXray(f, len, NODE_NAME, text.c_str(), cidx, SL_CODE_SUCCESS);

  // 3 redundant UDP bursts spaced 55ms apart across the 100ms Wi-Fi beacon sleep window
  for (int rep = 0; rep < 3; rep++) {
    emitLine(f, len);
    if (rep < 2) delay(55);
  }

  char detail[96];
  snprintf(detail, sizeof(detail), "TX seq=%u \"%.*s\" @ %ukHz",
           (unsigned)seq, (int)mlen, text.c_str(), (unsigned)SL_CARRIERS_KHZ[cidx]);
  addLog("TX", detail, SL_CODE_SUCCESS);

  for (int b = 0; b < 2; b++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(70);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
  }
}

static String jsonEscape(const char* s) {
  String out;
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') out += '\\';
    if ((unsigned char)*s >= 32) out += *s;
  }
  return out;
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>SMART-LINK HQ • Mocha</title>
<style>
  /* Catppuccin Mocha palette */
  :root{
    --base:#1e1e2e; --mantle:#181825; --crust:#11111b;
    --s0:#313244; --s1:#45475a; --ov0:#6c7086;
    --text:#cdd6f4; --sub:#a6adc8;
    --mauve:#cba6f7; --lav:#b4befe; --blue:#89b4fa; --sky:#89dceb;
    --green:#a6e3a1; --peach:#fab387; --yellow:#f9e2af; --red:#f38ba8;
  }
  *{box-sizing:border-box;font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
  body{margin:0;background:var(--base);color:var(--text);min-height:100vh;padding:16px;}
  .wrap{max-width:980px;margin:0 auto;display:flex;flex-direction:column;gap:14px;}
  .hdr{display:flex;flex-wrap:wrap;justify-content:space-between;align-items:center;gap:12px;padding:16px 20px;background:var(--mantle);border:1px solid var(--s0);border-radius:16px;}
  .brand{display:flex;align-items:center;gap:10px;font-size:19px;font-weight:800;letter-spacing:.2px;color:var(--text);}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--green);box-shadow:0 0 0 4px rgba(166,227,161,.18);}
  .sub{font-size:12px;color:var(--sub);margin-top:3px;}
  .pill{font-size:12px;padding:6px 12px;border-radius:999px;background:var(--s0);color:var(--lav);font-weight:600;border:1px solid var(--s1);}
  .two{display:grid;grid-template-columns:1.2fr 1fr;gap:14px;}
  @media(max-width:760px){.two{grid-template-columns:1fr;}}
  .panel{background:var(--mantle);border:1px solid var(--s0);border-radius:16px;padding:16px;}
  .ptitle{display:flex;justify-content:space-between;align-items:center;margin:0 0 12px;font-size:12px;font-weight:700;color:var(--sub);text-transform:uppercase;letter-spacing:.7px;}
  .ptag{font-size:11px;padding:2px 8px;border-radius:999px;background:var(--s0);color:var(--mauve);text-transform:none;letter-spacing:0;}
  /* Topology SVG */
  .topo-box{background:var(--crust);border:1px solid var(--s0);border-radius:12px;padding:8px;overflow:hidden;}
  svg{width:100%;height:auto;display:block;}
  @keyframes flow{to{stroke-dashoffset:-24;}}
  .link-ok{stroke:var(--green);stroke-width:2.2;stroke-dasharray:6 6;animation:flow 1.2s linear infinite;opacity:.85;}
  .link-bad{stroke:var(--red);stroke-width:2;stroke-dasharray:4 5;opacity:.75;}
  .link-reroute{stroke:var(--peach);stroke-width:2.6;stroke-dasharray:7 5;animation:flow .75s linear infinite;}
  /* Spectrum */
  .spec{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;align-items:end;height:158px;background:var(--crust);border:1px solid var(--s0);border-radius:12px;padding:14px 10px 10px;}
  .ch{display:flex;flex-direction:column;align-items:center;gap:6px;cursor:pointer;height:100%;justify-content:flex-end;}
  .bar-w{width:100%;max-width:34px;flex:1;display:flex;align-items:flex-end;background:rgba(49,50,68,.55);border-radius:8px;overflow:hidden;padding:2px;}
  .bar{width:100%;border-radius:6px;transition:height .4s ease,background .25s ease;}
  .ch-lbl{font-size:11px;font-weight:700;color:var(--sub);}
  .ch-st{font-size:10px;font-weight:700;padding:1px 5px;border-radius:6px;}
  /* Crypto X-Ray */
  .xray{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;}
  @media(max-width:720px){.xray{grid-template-columns:1fr;}}
  .xcard{background:var(--crust);border:1px solid var(--s0);border-radius:12px;padding:10px 12px;}
  .xlbl{font-size:11px;font-weight:700;color:var(--sub);margin-bottom:5px;display:flex;justify-content:space-between;}
  .xval{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px;word-break:break-all;line-height:1.45;}
  /* Controls */
  .row{display:flex;gap:8px;flex-wrap:wrap;}
  input{flex:1;min-width:190px;padding:10px 13px;border-radius:10px;border:1px solid var(--s1);background:var(--crust);color:var(--text);font-size:14px;outline:none;}
  input:focus{border-color:var(--mauve);}
  button{padding:10px 14px;border-radius:10px;border:none;font-weight:700;font-size:13px;cursor:pointer;color:var(--crust);transition:opacity .15s;}
  button:hover{opacity:.9;}
  .btn-send{background:var(--mauve);}
  .btn-tamper{background:var(--red);}
  .btn-jam{background:var(--peach);}
  .btn-heal{background:var(--green);}
  /* Nodes grid */
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(168px,1fr));gap:10px;}
  .card{background:var(--mantle);border:1px solid var(--s0);border-radius:13px;padding:12px;transition:.2s;}
  .card.ok{border-color:rgba(166,227,161,.45);}
  .card.tamper{border-color:var(--red);background:rgba(243,139,168,.08);}
  .card.off{opacity:.55;}
  .n-top{display:flex;justify-content:space-between;align-items:center;font-weight:700;font-size:13px;}
  .freq{font-size:12px;color:var(--sky);font-weight:700;margin:6px 0 4px;}
  .meta{font-size:11px;color:var(--sub);}
  .badge{font-size:10px;padding:2px 7px;border-radius:999px;font-weight:700;}
  .b-ok{background:rgba(166,227,161,.16);color:var(--green);}
  .b-err{background:rgba(243,139,168,.2);color:var(--red);}
  .b-off{background:var(--s0);color:var(--sub);}
  /* Logs */
  .logs{max-height:210px;overflow-y:auto;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px;display:flex;flex-direction:column;gap:6px;}
  .log{padding:8px 11px;border-radius:9px;background:var(--crust);border-left:3px solid var(--mauve);display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap;}
  .log.err{border-left-color:var(--red);}
  .log.ok{border-left-color:var(--green);}
  .log.reroute{border-left-color:var(--peach);}
</style>
</head>
<body>
<div class="wrap">
  <div class="hdr">
    <div>
      <div class="brand"><span class="dot"></span>SMART-LINK HQ</div>
      <div class="sub">Off-Grid Encrypted Multi-Carrier Mesh &bull; AP 192.168.4.1 &bull; UDP :5050</div>
    </div>
    <div class="pill" id="stats">Connecting...</div>
  </div>

  <div class="panel">
    <div class="ptitle"><span>Command Deck</span><span class="ptag">3-Burst Synced Broadcast</span></div>
    <div class="row">
      <input id="msg" placeholder="Type encrypted message for all nodes..." value="hello from smart-link" />
      <button class="btn-send" onclick="sendMsg()">Send Encrypted</button>
      <button class="btn-tamper" onclick="post('/api/tamper')">Simulate Tamper</button>
      <button class="btn-jam" onclick="post('/api/jam?idx=1')">Jam 84 kHz</button>
      <button class="btn-heal" onclick="post('/api/heal')">Heal All</button>
    </div>
  </div>

  <div class="two">
    <div class="panel">
      <div class="ptitle"><span>Live Mesh Topology</span><span class="ptag" id="topo-tag">Self-Healing Active</span></div>
      <div class="topo-box">
        <svg id="topo" viewBox="0 0 420 205"></svg>
      </div>
    </div>
    <div class="panel">
      <div class="ptitle"><span>Carrier Spectrum (Click to Jam)</span><span class="ptag">6 Subcarriers</span></div>
      <div class="spec" id="spec"></div>
    </div>
  </div>

  <div class="panel">
    <div class="ptitle"><span>Crypto X-Ray (Live Wire Inspector)</span><span class="ptag" id="xray-tag">Waiting for TX/RX</span></div>
    <div class="xray" id="xray">
      <div class="xcard"><div class="xlbl"><span>Plaintext</span><span style="color:var(--green)">Input</span></div><div class="xval" id="x-plain" style="color:var(--text)">Send a message to inspect live encryption...</div></div>
      <div class="xcard"><div class="xlbl"><span>Ciphertext (SHA-256 CTR)</span><span style="color:var(--mauve)">On Wire</span></div><div class="xval" id="x-ct" style="color:var(--mauve)">--</div></div>
      <div class="xcard"><div class="xlbl"><span>Hop Audit (HMAC-SHA256)</span><span style="color:var(--sky)">128-bit Tag</span></div><div class="xval" id="x-hmac" style="color:var(--sky)">--</div></div>
    </div>
  </div>

  <div class="grid" id="nodes"></div>

  <div class="panel">
    <div class="ptitle"><span>Live Packet &amp; Verification Stream</span><span class="ptag">STATUS 400 / 206 / 401</span></div>
    <div class="logs" id="logs"></div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
async function post(u){ await fetch(u,{method:'POST'}); refresh(); }
async function sendMsg(){
  const v=$('msg').value.trim(); if(!v) return;
  await post('/api/send?msg='+encodeURIComponent(v));
  $('msg').value='';
}
$('msg').addEventListener('keydown',e=>{if(e.key==='Enter')sendMsg();});

function drawTopo(nodes){
  const cx=210, cy=104, rx=142, ry=70;
  const hub=nodes.find(n=>n.id===5)||{name:'NodeMCU-HQ',khz:120,online:true,tampered:false};
  const peers=nodes.filter(n=>n.id!==5);
  const pos=peers.map((n,i)=>{
    const a=(-Math.PI/2)+(2*Math.PI*i/Math.max(peers.length,1));
    return {...n, x:Math.round(cx+rx*Math.cos(a)), y:Math.round(cy+ry*Math.sin(a))};
  });
  let svg='';
  let hasTamper=false;
  const healthyPeers=pos.filter(p=>p.online && !p.tampered);
  pos.forEach(p=>{
    if(p.tampered){
      hasTamper=true;
      svg+=`<line x1="${cx}" y1="${cy}" x2="${p.x}" y2="${p.y}" class="link-bad"/>`;
      const mx=(cx+p.x)/2, my=(cy+p.y)/2;
      svg+=`<circle cx="${mx}" cy="${my}" r="9" fill="#f38ba8"/><text x="${mx}" y="${my+4}" text-anchor="middle" font-size="11" font-weight="800" fill="#11111b">X</text>`;
    } else if(p.online){
      svg+=`<line x1="${cx}" y1="${cy}" x2="${p.x}" y2="${p.y}" class="link-ok"/>`;
    } else {
      svg+=`<line x1="${cx}" y1="${cy}" x2="${p.x}" y2="${p.y}" stroke="#45475a" stroke-dasharray="3 4" stroke-width="1.5"/>`;
    }
  });
  if(hasTamper && healthyPeers.length>=2){
    for(let i=0;i+1<healthyPeers.length;i++){
      const a=healthyPeers[i], b=healthyPeers[i+1];
      svg+=`<path d="M ${a.x} ${a.y} Q ${cx} ${cy-28} ${b.x} ${b.y}" fill="none" class="link-reroute"/>`;
    }
  }
  const drawNode=(x,y,name,khz,st)=>{
    const fill=st==='tamper'?'#f38ba8':(st==='ok'?'#a6e3a1':(st==='hub'?'#cba6f7':'#45475a'));
    const stroke=st==='tamper'?'#f38ba8':(st==='hub'?'#b4befe':'#313244');
    return `<g>
      <circle cx="${x}" cy="${y}" r="${st==='hub'?23:18}" fill="#181825" stroke="${fill}" stroke-width="2.5"/>
      <circle cx="${x}" cy="${y}" r="5" fill="${fill}"/>
      <rect x="${x-42}" y="${y+(st==='hub'?26:21)}" width="84" height="17" rx="6" fill="#1e1e2e" stroke="${stroke}" stroke-width="1"/>
      <text x="${x}" y="${y+(st==='hub'?38:33)}" text-anchor="middle" font-size="10" font-weight="700" fill="#cdd6f4">${name}</text>
    </g>`;
  };
  svg+=drawNode(cx,cy,hub.name,hub.khz,'hub');
  pos.forEach(p=>svg+=drawNode(p.x,p.y,p.name,p.khz,p.tampered?'tamper':(p.online?'ok':'off')));
  $('topo').innerHTML=svg;
  $('topo-tag').textContent=hasTamper?'REROUTED 206 (Bypass Active)':'All Paths Healthy';
  $('topo-tag').style.color=hasTamper?'var(--peach)':'var(--green)';
}

function drawSpec(carriers, nodes){
  const activeSet=new Set(nodes.filter(n=>n.online && !n.tampered).map(n=>n.khz));
  $('spec').innerHTML=carriers.map((c,i)=>{
    const used=activeSet.has(c.khz);
    const h=c.jammed?88:(used?(62+((Date.now()/400+i*17)%22)):24);
    const col=c.jammed?'var(--red)':(used?'var(--mauve)':'var(--s1)');
    const st=c.jammed?'JAM':(used?'LIVE':'IDLE');
    const stCol=c.jammed?'color:var(--red);background:rgba(243,139,168,.18)':(used?'color:var(--green);background:rgba(166,227,161,.16)':'color:var(--sub);background:var(--s0)');
    return `<div class="ch" onclick="post('/api/jam?idx=${i}')" title="Click to toggle jam on ${c.khz} kHz">
      <div class="bar-w"><div class="bar" style="height:${Math.round(h)}%;background:${col}"></div></div>
      <div class="ch-lbl">${c.khz}k</div>
      <div class="ch-st" style="${stCol}">${st}</div>
    </div>`;
  }).join('');
}

async function refresh(){
  try{
    const r=await fetch('/api/state'); const d=await r.json();
    $('stats').textContent=d.online+' Online • '+d.clients+'/8 Wi-Fi Clients • '+d.uptime+'s';
    drawTopo(d.nodes);
    drawSpec(d.carriers, d.nodes);
    if(d.xray && d.xray.valid){
      $('xray-tag').textContent=`seq=${d.xray.seq} • ${d.xray.src} • ${d.xray.khz} kHz • ${d.xray.len}B • STATUS ${d.xray.code}`;
      $('x-plain').textContent='"'+d.xray.plain+'"';
      $('x-ct').textContent=d.xray.ct;
      $('x-hmac').textContent=d.xray.hmac;
    }
    $('nodes').innerHTML=d.nodes.map(n=>{
      const cls=n.tampered?'tamper':(n.online?'ok':'off');
      const b=n.tampered?'<span class="badge b-err">TAMPER 401</span>':(n.online?'<span class="badge b-ok">ONLINE</span>':'<span class="badge b-off">OFFLINE</span>');
      return `<div class="card ${cls}"><div class="n-top"><span>${n.name}</span>${b}</div><div class="freq">${n.khz} kHz &bull; ID ${n.id}</div><div class="meta">Role: ${n.role} &bull; PKT: ${n.rx}</div></div>`;
    }).join('');
    $('logs').innerHTML=d.logs.map(l=>{
      const cls=l.code===401?'err':(l.code===206?'reroute':'ok');
      return `<div class="log ${cls}"><span><b>[${l.tag}]</b> ${l.detail}</span><span>STATUS ${l.code}</span></div>`;
    }).join('') || '<div class="meta">Waiting for encrypted mesh traffic...</div>';
  }catch(e){}
}
refresh(); setInterval(refresh,1200);
</script>
</body>
</html>
)rawliteral";

static void handleApiState() {
  uint32_t now = millis();
  int onlineCnt = 0;
  String json = "{";
  json += "\"uptime\":" + String(now / 1000) + ",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"carriers\":[";
  for (uint8_t i = 0; i < SL_NUM_CARRIERS; i++) {
    if (i > 0) json += ",";
    json += "{\"khz\":" + String(SL_CARRIERS_KHZ[i]) + ",\"jammed\":" + (jammed[i] ? "true" : "false") + "}";
  }
  json += "],\"xray\":{\"valid\":";
  json += (lastXray.valid ? "true" : "false");
  if (lastXray.valid) {
    json += ",\"seq\":" + String(lastXray.seq)
          + ",\"khz\":" + String(lastXray.carrierKhz)
          + ",\"len\":" + String(lastXray.frameLen)
          + ",\"code\":" + String(lastXray.code)
          + ",\"src\":\"" + jsonEscape(lastXray.srcName) + "\""
          + ",\"plain\":\"" + jsonEscape(lastXray.plain) + "\""
          + ",\"ct\":\"" + jsonEscape(lastXray.cipherHex) + "\""
          + ",\"hmac\":\"" + jsonEscape(lastXray.hmacHex) + "\"";
  }
  json += "},\"nodes\":[";
  bool first = true;
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].active) continue;
    bool isOnline = (nodes[i].id == NODE_ID) || ((now - nodes[i].lastSeenMs) < 9000);
    if (isOnline && !nodes[i].tampered) onlineCnt++;
    if (!first) json += ",";
    first = false;
    json += "{\"id\":" + String(nodes[i].id)
          + ",\"name\":\"" + jsonEscape(nodes[i].name) + "\""
          + ",\"role\":\"" + jsonEscape(nodes[i].role) + "\""
          + ",\"khz\":" + String(SL_CARRIERS_KHZ[nodes[i].carrierIdx < SL_NUM_CARRIERS ? nodes[i].carrierIdx : 0])
          + ",\"online\":" + (isOnline ? "true" : "false")
          + ",\"tampered\":" + (nodes[i].tampered ? "true" : "false")
          + ",\"rx\":" + String(nodes[i].rxCount) + "}";
  }
  json += "],\"online\":" + String(onlineCnt) + ",\"logs\":[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead - 1 - i + MAX_LOGS) % MAX_LOGS;
    if (i > 0) json += ",";
    json += "{\"tag\":\"" + jsonEscape(logs[idx].tag) + "\""
          + ",\"detail\":\"" + jsonEscape(logs[idx].detail) + "\""
          + ",\"code\":" + String(logs[idx].code) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  sl_dec_init(&usbDec);
  sl_dec_init(&udpDec);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_NETMASK);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);  // 8 simultaneous clients!
  dnsServer.start(53, "*", AP_IP);
  udp.begin(MESH_PORT);

  upsertNode(NODE_ID, NODE_NAME, NODE_ROLE, CARRIER_IDX);
  addLog("BOOT", "NodeMCU-HQ AP online (8-client Mocha HQ)", SL_CODE_SUCCESS);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/send", HTTP_POST, []() {
    String msg = server.arg("msg");
    if (msg.length() > 0) sendBroadcastMsg(msg);
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/jam", HTTP_POST, []() {
    int idx = server.arg("idx").toInt();
    if (idx >= 0 && idx < SL_NUM_CARRIERS) {
      jammed[idx] = !jammed[idx];
      char d[96];
      if (jammed[idx]) {
        uint8_t nextC = pickActiveCarrier(CARRIER_IDX);
        snprintf(d, sizeof(d), "Carrier %ukHz JAMMED -> hopped to %ukHz",
                 (unsigned)SL_CARRIERS_KHZ[idx], (unsigned)SL_CARRIERS_KHZ[nextC]);
        addLog("JAM", d, SL_CODE_REROUTED);
      } else {
        snprintf(d, sizeof(d), "Carrier %ukHz cleared -> spectrum nominal",
                 (unsigned)SL_CARRIERS_KHZ[idx]);
        addLog("UNJAM", d, SL_CODE_SUCCESS);
      }
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/tamper", HTTP_POST, []() {
    int target = -1;
    for (int i = 0; i < MAX_NODES; i++) {
      if (nodes[i].active && nodes[i].id != NODE_ID && !nodes[i].tampered) { target = i; break; }
    }
    if (target >= 0) {
      nodes[target].tampered = true;
      char d1[96], d2[96];
      snprintf(d1, sizeof(d1), "HMAC mismatch detected on %s!", nodes[target].name);
      snprintf(d2, sizeof(d2), "Inference Engine isolated %s -> bypass path active", nodes[target].name);
      addLog("TAMPER", d1, SL_CODE_TAMPER);
      addLog("HEAL", d2, SL_CODE_REROUTED);
    } else {
      addLog("TAMPER", "Simulated MITM tamper injected -> reroute active", SL_CODE_REROUTED);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/heal", HTTP_POST, []() {
    for (int i = 0; i < MAX_NODES; i++) nodes[i].tampered = false;
    for (uint8_t i = 0; i < SL_NUM_CARRIERS; i++) jammed[i] = false;
    addLog("HEAL", "All nodes & carriers restored -> full mesh nominal", SL_CODE_SUCCESS);
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
  sendHeartbeat();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  while (Serial.available() > 0) {
    uint16_t n = sl_dec_feed(&usbDec, (char)Serial.read());
    if (n) handleFrame(usbDec.buf, n);
  }
  int pktSize = udp.parsePacket();
  while (pktSize-- > 0) {
    int ch = udp.read();
    if (ch >= 0) {
      uint16_t n = sl_dec_feed(&udpDec, (char)ch);
      if (n) handleFrame(udpDec.buf, n);
    }
  }
  if (millis() - lastHbMs >= 2000) {
    lastHbMs = millis();
    sendHeartbeat();
  }
}
