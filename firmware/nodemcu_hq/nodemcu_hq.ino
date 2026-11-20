/*
 * nodemcu_hq.ino — SMART-LINK Wi-Fi Web Dashboard & Mesh Access Point (NodeMCU v3 / ESP8266)
 * ==========================================================================================
 * Creates an offline Wi-Fi Access Point ("SMART-LINK-HQ" / "smartlink123") at 192.168.4.1
 * so all ESP32 nodes + Laptop HQ + phones/tablets can join the encrypted SMART-LINK mesh
 * with ZERO data wires and ZERO external internet required.
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

#define NODE_ID     5
#define NODE_NAME   "NodeMCU-HQ"
#define NODE_ROLE   "hq-ap"
#define CARRIER_IDX 4          // 4 = 120 kHz

#define AP_SSID     "SMART-LINK-HQ"
#define AP_PASS     "smartlink123"
#define MESH_PORT   5050
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

struct SeenEntry { uint32_t key; uint32_t ms; };
static SeenEntry seen[24];
static uint8_t seenPos = 0;

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
      if (name && name[0]) { strncpy(nodes[i].name, name, sizeof(nodes[i].name) - 1); }
      if (role && role[0]) { strncpy(nodes[i].role, role, sizeof(nodes[i].role) - 1); }
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
  char pay[48];
  int plen = snprintf(pay, sizeof(pay), "%s|%s|%u|", NODE_NAME, NODE_ROLE, (unsigned)(SL_CAP_UART | SL_CAP_WIFI));
  uint8_t f[SL_MAX_FRAME];
  uint16_t len = sl_build(f, sizeof(f), SL_T_HB, SL_HB_TTL, mySeq++, NODE_ID,
                          SL_BROADCAST, nullptr, 0, (const uint8_t*)pay, (uint8_t)plen);
  if (!len) return;
  len = sl_append_hop(f, len, sizeof(f), NODE_ID, CARRIER_IDX);
  if (len) {
    digitalWrite(LED_BUILTIN, LOW);  // active LOW on ESP8266
    emitLine(f, len);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  upsertNode(NODE_ID, NODE_NAME, NODE_ROLE, CARRIER_IDX);
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
    uint8_t mlen = v.L < 72 ? v.L : 72;
    memcpy(pt, raw + v.off_payload, mlen);
    sl_crypt(v.src, v.seq, pt, mlen);
    pt[mlen] = '\0';
    char msg[96];
    snprintf(msg, sizeof(msg), "%s -> \"%s\" (seq=%u)",
             nodeNameById(v.src), (const char*)pt, (unsigned)v.seq);
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
  if (mlen > 72) mlen = 72;
  memcpy(payload, text.c_str(), mlen);

  uint16_t seq = mySeq++;
  sl_crypt(NODE_ID, seq, payload, (uint8_t)mlen);

  uint8_t f[SL_MAX_FRAME];
  uint16_t len = sl_build(f, sizeof(f), SL_T_MSG, 0, seq, NODE_ID,
                          SL_BROADCAST, nullptr, 0, payload, (uint8_t)mlen);
  if (!len) return;
  len = sl_append_hop(f, len, sizeof(f), NODE_ID, CARRIER_IDX);
  if (!len) return;

  // Transmit 3 redundant UDP bursts spaced 55ms apart to span the 100ms Wi-Fi modem-sleep
  // beacon window; ESP32 nodes deduplicate by (type, src, seq) so all 3 flash reliably in sync.
  for (int rep = 0; rep < 3; rep++) {
    emitLine(f, len);
    if (rep < 2) delay(55);
  }

  char detail[96];
  snprintf(detail, sizeof(detail), "TX seq=%u \"%.*s\" (SHA-256 CTR + HMAC)",
           (unsigned)seq, (int)mlen, text.c_str());
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
<title>SMART-LINK HQ</title>
<style>
  :root{--bg:#0b1120;--card:#111a2e;--border:#1e293b;--text:#f8fafc;--muted:#94a3b8;--accent:#38bdf8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;}
  *{box-sizing:border-box;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
  body{margin:0;background:radial-gradient(circle at top,#172554 0%,var(--bg) 55%);color:var(--text);min-height:100vh;padding:16px;}
  .wrap{max-width:860px;margin:0 auto;}
  .hdr{display:flex;flex-wrap:wrap;justify-content:space-between;align-items:center;gap:12px;margin-bottom:16px;padding:16px 20px;background:rgba(17,26,46,.85);border:1px solid var(--border);border-radius:14px;}
  .title{font-size:20px;font-weight:800;letter-spacing:.4px;display:flex;align-items:center;gap:10px;}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--ok);box-shadow:0 0 10px var(--ok);}
  .sub{font-size:12px;color:var(--muted);margin-top:3px;}
  .pill{font-size:12px;padding:5px 10px;border-radius:999px;background:#0f172a;border:1px solid var(--border);color:var(--accent);font-weight:600;}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px;margin-bottom:16px;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:12px;transition:.2s;}
  .card.ok{border-color:rgba(34,197,94,.45);}
  .card.tamper{border-color:rgba(239,68,68,.75);background:#2a1219;}
  .card.off{opacity:.55;}
  .n-top{display:flex;justify-content:space-between;align-items:center;font-weight:700;font-size:14px;}
  .freq{font-size:12px;color:var(--accent);font-weight:700;margin:6px 0;}
  .meta{font-size:12px;color:var(--muted);}
  .badge{font-size:11px;padding:2px 7px;border-radius:999px;font-weight:700;}
  .b-ok{background:rgba(34,197,94,.18);color:#4ade80;}
  .b-err{background:rgba(239,68,68,.22);color:#f87171;}
  .b-off{background:rgba(148,163,184,.18);color:#cbd5e1;}
  .panel{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px;margin-bottom:16px;}
  .panel h3{margin:0 0 10px;font-size:14px;color:var(--muted);text-transform:uppercase;letter-spacing:.6px;}
  .row{display:flex;gap:8px;flex-wrap:wrap;}
  input{flex:1;min-width:180px;padding:10px 12px;border-radius:10px;border:1px solid var(--border);background:#0b1222;color:var(--text);font-size:14px;outline:none;}
  input:focus{border-color:var(--accent);}
  button{padding:10px 14px;border-radius:10px;border:none;font-weight:700;font-size:13px;cursor:pointer;color:#0b1120;background:var(--accent);}
  button.warn{background:var(--warn);}
  button.ok{background:var(--ok);}
  .logs{max-height:260px;overflow-y:auto;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px;display:flex;flex-direction:column;gap:6px;}
  .log{padding:8px 10px;border-radius:8px;background:#0b1222;border-left:3px solid var(--accent);display:flex;justify-content:space-between;gap:8px;flex-wrap:wrap;}
  .log.err{border-left-color:var(--err);}
  .log.ok{border-left-color:var(--ok);}
  .log.reroute{border-left-color:var(--warn);}
</style>
</head>
<body>
<div class="wrap">
  <div class="hdr">
    <div>
      <div class="title"><span class="dot"></span>SMART-LINK HQ MESH</div>
      <div class="sub">Off-Grid Encrypted Multi-Carrier Control Panel &bull; AP 192.168.4.1 &bull; UDP :5050</div>
    </div>
    <div class="pill" id="stats">Loading mesh...</div>
  </div>

  <div class="grid" id="nodes"></div>

  <div class="panel">
    <h3>Encrypted Mesh Broadcast (SHA-256 CTR + HMAC)</h3>
    <div class="row">
      <input id="msg" placeholder="Type message to blink all connected ESP32s..." value="hello from smart-link hq" />
      <button onclick="sendMsg()">Send Encrypted</button>
      <button class="warn" onclick="simTamper()">Simulate Tamper</button>
      <button class="ok" onclick="healAll()">Heal Network</button>
    </div>
  </div>

  <div class="panel">
    <h3>Live Packet &amp; Verification Stream</h3>
    <div class="logs" id="logs"></div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
async function sendMsg(){
  const v=$('msg').value.trim(); if(!v) return;
  await fetch('/api/send?msg='+encodeURIComponent(v),{method:'POST'});
  $('msg').value=''; refresh();
}
async function simTamper(){
  await fetch('/api/tamper',{method:'POST'}); refresh();
}
async function healAll(){
  await fetch('/api/heal',{method:'POST'}); refresh();
}
$('msg').addEventListener('keydown',e=>{if(e.key==='Enter')sendMsg();});
async function refresh(){
  try{
    const r=await fetch('/api/state'); const d=await r.json();
    $('stats').textContent=d.online+' Online • '+d.clients+' Wi-Fi Clients • '+d.uptime+'s';
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
  json += "\"nodes\":[";
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
  WiFi.softAP(AP_SSID, AP_PASS);
  dnsServer.start(53, "*", AP_IP);
  udp.begin(MESH_PORT);

  upsertNode(NODE_ID, NODE_NAME, NODE_ROLE, CARRIER_IDX);
  addLog("BOOT", "NodeMCU-HQ AP online (SSID: SMART-LINK-HQ)", SL_CODE_SUCCESS);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/send", HTTP_POST, []() {
    String msg = server.arg("msg");
    if (msg.length() > 0) sendBroadcastMsg(msg);
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
      snprintf(d2, sizeof(d2), "Inference Engine isolated %s -> active path rerouted", nodes[target].name);
      addLog("TAMPER", d1, SL_CODE_TAMPER);
      addLog("HEAL", d2, SL_CODE_REROUTED);
    } else {
      addLog("TAMPER", "Simulated MITM tamper injected -> reroute active", SL_CODE_REROUTED);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/heal", HTTP_POST, []() {
    for (int i = 0; i < MAX_NODES; i++) nodes[i].tampered = false;
    addLog("HEAL", "All compromised nodes cleared -> full mesh restored", SL_CODE_SUCCESS);
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
