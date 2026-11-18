// main.cpp — SMART-LINK C++17 Node, Inference Engine, FM Radio PHY & HQ Terminal Playground
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "fm.hpp"
#include "inference.hpp"
#include "smartlink_core.h"
#include "transport.hpp"
#include "util.hpp"

namespace {

struct Config {
  uint8_t id = 4;
  std::string name = "HP-ProBook";
  std::string role = "hq";
  uint16_t udp_port = 5050;
  int baud = 38400;
  bool broadcast = true;
  bool fm_tx = true;
  bool fm_rx = false;
  bool self_test = false;
  std::string fm_tx_cmd;
  std::string fm_rx_cmd;
  std::vector<std::string> peers;
  std::vector<std::string> serial_devs;
};

struct App {
  Config cfg;
  sl::Engine eng;
  sl::UdpLink udp;
  std::vector<std::unique_ptr<sl::SerialLink>> serials;
  sl::FmRadio fm;
  std::mutex mu;
  std::map<uint32_t, int64_t> seen_frames;  // (src<<16|seq) -> timestamp
  std::map<uint16_t, int64_t> pending_ack;  // seq -> tx_start_ms
  uint16_t next_seq = 1;
  std::atomic<bool> running{true};
};

std::string carrier_label(int cidx) {
  if (cidx < 0 || cidx >= SL_NUM_CARRIERS) return "UNASSIGNED";
  return std::to_string(SL_CARRIERS_KHZ[cidx]) + "kHz";
}

void broadcast_line(App& app, const std::string& line) {
  app.udp.send_line(line);
  for (auto& s : app.serials) s->send_line(line);
}

std::string frame_to_sl1(const uint8_t* f, uint16_t len) {
  return "SL1 " + sl::to_hex(f, len);
}

// Parse a heartbeat payload: "name|role|caps|nbr1,nbr2,..."
void apply_hb_payload(App& app, uint8_t src, const uint8_t* p, uint8_t len, const std::string& via) {
  std::string s((const char*)p, len);
  std::vector<std::string> parts;
  size_t st = 0;
  while (st <= s.size()) {
    size_t bar = s.find('|', st);
    if (bar == std::string::npos) {
      parts.push_back(s.substr(st));
      break;
    }
    parts.push_back(s.substr(st, bar - st));
    st = bar + 1;
  }
  std::string nname = parts.size() > 0 && !parts[0].empty() ? parts[0] : ("node-" + std::to_string(src));
  std::string nrole = parts.size() > 1 && !parts[1].empty() ? parts[1] : "node";
  uint8_t ncaps = parts.size() > 2 && !parts[2].empty() ? (uint8_t)std::atoi(parts[2].c_str()) : 0;

  std::set<uint8_t> nbrs;
  if (parts.size() > 3 && !parts[3].empty()) {
    std::stringstream ss(parts[3]);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (!tok.empty()) nbrs.insert((uint8_t)std::atoi(tok.c_str()));
    }
  }

  std::lock_guard<std::mutex> g(app.mu);
  bool is_new = !app.eng.nodes.count(src);
  bool was_off = !is_new && !app.eng.nodes[src].online;

  auto& nr = app.eng.nodes[src];
  nr.id = src;
  nr.name = nname;
  nr.role = nrole;
  nr.via = via;
  nr.caps = ncaps;
  nr.last_seen = sl::now_ms();
  nr.online = true;

  // Ensure bidirectional neighbor visibility with self when heard directly
  nr.nbrs = nbrs;
  nr.nbrs.insert(app.cfg.id);
  app.eng.nodes[app.cfg.id].nbrs.insert(src);

  if (is_new || was_off) {
    app.eng.rebuild(false);
    sl::logf("%s[MESH] Node %s (ID=%u, role=%s) ONLINE via %s -> topology rebuilt (%zu links, %d bridges)%s",
             SL_GRN, nname.c_str(), src, nrole.c_str(), via.c_str(),
             app.eng.links.size(), app.eng.bridges, SL_RST);
  }
}

void send_heartbeat(App& app) {
  std::string nbr_list;
  uint8_t caps = SL_CAP_ENGINE | SL_CAP_WIFI | (app.cfg.fm_tx ? SL_CAP_FM_TX : 0) |
                 (app.cfg.fm_rx ? SL_CAP_FM_RX : 0) | (!app.serials.empty() ? SL_CAP_UART : 0);
  uint16_t seq = 0;
  {
    std::lock_guard<std::mutex> g(app.mu);
    auto& me = app.eng.nodes[app.cfg.id];
    me.last_seen = sl::now_ms();
    me.online = true;
    bool first = true;
    for (uint8_t nb : me.nbrs) {
      if (app.eng.alive(nb)) {
        if (!first) nbr_list += ",";
        nbr_list += std::to_string(nb);
        first = false;
      }
    }
    seq = app.next_seq++;
  }

  std::string payload = app.cfg.name + "|" + app.cfg.role + "|" + std::to_string(caps) + "|" + nbr_list;
  if (payload.size() > 180) payload.resize(180);

  uint8_t frame[SL_MAX_FRAME];
  uint16_t len = sl_build(frame, sizeof(frame), SL_T_HB, SL_HB_TTL, seq, app.cfg.id,
                          SL_BROADCAST, nullptr, 0, (const uint8_t*)payload.data(), (uint8_t)payload.size());
  if (!len) return;
  len = sl_append_hop(frame, len, sizeof(frame), app.cfg.id, 0);
  if (!len) return;
  broadcast_line(app, frame_to_sl1(frame, len));
}

void send_ack(App& app, uint8_t dst, uint16_t orig_seq, uint16_t status_code, const SlView& orig_v, const uint8_t* orig_f) {
  std::vector<uint8_t> rev_route;
  uint8_t R = 0;
  {
    std::lock_guard<std::mutex> g(app.mu);
    sl::Route rt;
    if (app.eng.route(app.cfg.id, dst, rt)) {
      R = (uint8_t)rt.ids.size();
      for (size_t i = 0; i < rt.ids.size(); i++) {
        rev_route.push_back(rt.ids[i]);
        rev_route.push_back(i < rt.cidx.size() ? rt.cidx[i] : 0);
      }
    }
  }
  if (R == 0) {
    // Fallback: reverse the observed hop list
    for (int i = (int)orig_v.H - 1; i >= 0; i--) {
      rev_route.push_back(orig_f[orig_v.off_hops + 2 * i]);
      rev_route.push_back(orig_f[orig_v.off_hops + 2 * i + 1]);
      R++;
    }
    rev_route.push_back(app.cfg.id);
    rev_route.push_back(0);
  }

  uint8_t ack_pay[4] = {(uint8_t)(orig_seq >> 8), (uint8_t)orig_seq,
                        (uint8_t)(status_code >> 8), (uint8_t)status_code};
  uint8_t frame[SL_MAX_FRAME];
  uint16_t seq;
  {
    std::lock_guard<std::mutex> g(app.mu);
    seq = app.next_seq++;
  }
  uint16_t len = sl_build(frame, sizeof(frame), SL_T_ACK, 0, seq, app.cfg.id, dst,
                          rev_route.data(), R, ack_pay, 4);
  if (!len) return;
  uint8_t c0 = (R > 0) ? rev_route[1] : 0;
  len = sl_append_hop(frame, len, sizeof(frame), app.cfg.id, c0);
  if (!len) return;
  broadcast_line(app, frame_to_sl1(frame, len));
}

void handle_frame(App& app, const uint8_t* raw, uint16_t len, const std::string& via) {
  SlView v;
  if (!sl_parse(raw, len, &v)) return;
  if (v.src == app.cfg.id && v.H <= 1) return;  // ignore our own immediate echo

  // Deduplicate (type, src, seq)
  {
    std::lock_guard<std::mutex> g(app.mu);
    uint32_t key = ((uint32_t)v.type << 24) | ((uint32_t)v.src << 16) | v.seq;
    int64_t now = sl::now_ms();
    if (app.seen_frames.count(key) && (now - app.seen_frames[key]) < 15000) return;
    app.seen_frames[key] = now;
  }

  if (v.type == SL_T_HB) {
    apply_hb_payload(app, v.src, raw + v.off_payload, v.L, via);
    if (v.flags > 1 && v.H < SL_MAX_HOPS) {
      uint8_t fwd[SL_MAX_FRAME];
      std::memcpy(fwd, raw, len);
      sl_set_flags(fwd, len, (uint8_t)(v.flags - 1));
      sl_set_tx(fwd, len, app.cfg.id);
      uint16_t flen = sl_append_hop(fwd, len, sizeof(fwd), app.cfg.id, 0);
      if (flen) broadcast_line(app, frame_to_sl1(fwd, flen));
    }
    return;
  }

  // Only process or forward SL_T_MSG / SL_T_ACK
  if (v.dst == app.cfg.id || v.dst == SL_BROADCAST) {
    int ok = sl_verify(raw, &v);
    if (v.type == SL_T_MSG) {
      std::vector<uint8_t> pt(raw + v.off_payload, raw + v.off_payload + v.L);
      sl_crypt(v.src, v.seq, pt.data(), (uint8_t)pt.size());
      std::string msg((const char*)pt.data(), pt.size());

      std::string hops_str;
      for (uint8_t h = 0; h < v.H; h++) {
        if (h) hops_str += " -> ";
        uint8_t nid = raw[v.off_hops + 2 * h];
        uint8_t cidx = raw[v.off_hops + 2 * h + 1];
        hops_str += app.eng.name_of(nid) + "(" + carrier_label(cidx) + ")";
      }

      if (ok) {
        sl::logf("\r%s[%s @ %s] \"%s\" (hops=%u | STATUS: SUCCESS %d)%s",
                 SL_GRN, app.eng.name_of(v.src).c_str(),
                 v.H > 0 ? carrier_label(raw[v.off_hops + 1]).c_str() : "75kHz",
                 msg.c_str(), v.H, SL_CODE_SUCCESS, SL_RST);
        if (v.src != app.cfg.id) send_ack(app, v.src, v.seq, SL_CODE_SUCCESS, v, raw);
      } else {
        sl::logf("%s[RX %s] from=%s seq=%u | HMAC CHAIN TAMPERED! | STATUS: ERROR %d%s",
                 SL_RED, via.c_str(), app.eng.name_of(v.src).c_str(), v.seq, SL_CODE_TAMPER, SL_RST);
        if (v.src != app.cfg.id) send_ack(app, v.src, v.seq, SL_CODE_TAMPER, v, raw);
      }
    } else if (v.type == SL_T_ACK && v.L >= 4) {
      uint16_t oseq = (uint16_t)((raw[v.off_payload] << 8) | raw[v.off_payload + 1]);
      uint16_t code = (uint16_t)((raw[v.off_payload + 2] << 8) | raw[v.off_payload + 3]);
      int64_t rtt = 0;
      {
        std::lock_guard<std::mutex> g(app.mu);
        if (app.pending_ack.count(oseq)) {
          rtt = sl::now_ms() - app.pending_ack[oseq];
          app.pending_ack.erase(oseq);
        }
      }
      if (ok && code == SL_CODE_SUCCESS) {
        sl::logf("%s[ACK] seq=%u from %s (%u hops, %lld ms) -> STATUS: SUCCESS 400%s",
                 SL_GRN, oseq, app.eng.name_of(v.src).c_str(), v.H, (long long)rtt, SL_RST);
      } else {
        sl::logf("%s[ACK] seq=%u from %s -> STATUS: ERROR %u (verify=%d)%s",
                 SL_RED, oseq, app.eng.name_of(v.src).c_str(), code, ok, SL_RST);
      }
    }
    return;
  }

  // Forward if we are on the source route
  int next_idx = -1;
  uint8_t cidx = 0;
  for (uint8_t i = 0; i + 1 < v.R; i++) {
    if (raw[v.off_route + 2 * i] == app.cfg.id) {
      next_idx = i + 1;
      cidx = raw[v.off_route + 2 * i + 1];
      break;
    }
  }
  if (next_idx < 0) return;

  uint8_t fwd[SL_MAX_FRAME];
  std::memcpy(fwd, raw, len);
  sl_set_tx(fwd, len, app.cfg.id);
  uint16_t flen = sl_append_hop(fwd, len, sizeof(fwd), app.cfg.id, cidx);
  if (!flen) return;

  sl::logf("%s[FWD] %s -> %s (seq=%u, carrier=%s)%s",
           SL_CYN, app.eng.name_of(v.src).c_str(), app.eng.name_of(v.dst).c_str(),
           v.seq, carrier_label(cidx).c_str(), SL_RST);

  broadcast_line(app, frame_to_sl1(fwd, flen));
  if (app.fm.tx()) {
    std::vector<uint8_t> fv(fwd, fwd + flen);
    app.fm.transmit(fv, cidx);
  }
}

void handle_line(App& app, const std::string& line, const std::string& origin) {
  if (line.rfind("SL1 ", 0) == 0) {
    std::vector<uint8_t> raw;
    if (sl::from_hex(line.substr(4), raw) && raw.size() <= SL_MAX_FRAME) {
      handle_frame(app, raw.data(), (uint16_t)raw.size(), origin);
    }
  } else if (sl::Log::verbose()) {
    sl::logf("%s[%s] %s%s", SL_DIM, origin.c_str(), line.c_str(), SL_RST);
  }
}

void print_status(App& app) {
  std::lock_guard<std::mutex> g(app.mu);
  std::printf("\n%s--- SMART-LINK Live Mesh Status (%s, ID=%u) ---%s\n",
              SL_BLD, app.cfg.name.c_str(), app.cfg.id, SL_RST);
  for (auto& kv : app.eng.nodes) {
    auto& n = kv.second;
    bool up = app.eng.alive(n.id);
    const char* col = up ? SL_GRN : SL_RED;
    const char* st = n.forced_off ? "UNPLUGGED" : (n.online ? "ONLINE" : "TIMEOUT");
    std::printf("  %s[%-9s]%s ID=%-3u %-14s role=%-8s via=%s\n",
                col, st, SL_RST, n.id, n.name.c_str(), n.role.c_str(),
                n.self ? "local" : n.via.c_str());
  }
  std::printf("Active Links (%zu) | Bridges=%d | Coverage=%.1f%%\n",
              app.eng.links.size(), app.eng.bridges, app.eng.coverage * 100.0);
  for (auto& l : app.eng.links) {
    std::printf("  %s <-> %s  [%c]  carrier=%s\n",
                app.eng.name_of(l.a).c_str(), app.eng.name_of(l.b).c_str(),
                l.kind, carrier_label(l.cidx).c_str());
  }
  std::printf("----------------------------------------------------\n\n");
  std::fflush(stdout);
}

void send_message(App& app, int target_id, const std::string& text) {
  if (text.empty()) return;
  uint16_t seq;
  std::vector<uint8_t> targets;
  {
    std::lock_guard<std::mutex> g(app.mu);
    seq = app.next_seq++;
    if (target_id >= 0) {
      targets.push_back((uint8_t)target_id);
    } else {
      for (auto& kv : app.eng.nodes) {
        if (kv.first != app.cfg.id && app.eng.alive(kv.first)) targets.push_back(kv.first);
      }
    }
  }

  if (targets.empty()) {
    // Local loopback verification if no remote nodes are online yet
    std::vector<uint8_t> ct(text.begin(), text.end());
    if (ct.size() > 140) ct.resize(140);
    sl_crypt(app.cfg.id, seq, ct.data(), (uint8_t)ct.size());
    uint8_t rt_bytes[2] = {app.cfg.id, 0};
    uint8_t frame[SL_MAX_FRAME];
    uint16_t flen = sl_build(frame, sizeof(frame), SL_T_MSG, 0, seq, app.cfg.id, app.cfg.id,
                             rt_bytes, 1, ct.data(), (uint8_t)ct.size());
    flen = sl_append_hop(frame, flen, sizeof(frame), app.cfg.id, 0);
    SlView v;
    int ok = flen && sl_parse(frame, flen, &v) && sl_verify(frame, &v);
    if (app.fm.tx() && flen) app.fm.transmit(std::vector<uint8_t>(frame, frame + flen), 0);
    sl::logf("%s[LOCAL-LOOPBACK] seq=%u | carrier=75kHz (FM audio TX=%s) | HMAC=%s | STATUS: SUCCESS %d%s",
             SL_GRN, seq, app.fm.tx() ? "ON" : "OFF", ok ? "VERIFIED" : "FAIL",
             ok ? SL_CODE_SUCCESS : SL_CODE_TAMPER, SL_RST);
    return;
  }

  for (uint8_t dst : targets) {
    sl::Route rt;
    {
      std::lock_guard<std::mutex> g(app.mu);
      if (!app.eng.route(app.cfg.id, dst, rt)) {
        sl::logf("%s[ROUTE FAIL] No path to %s (ID=%u) -> STATUS: ERROR 503%s",
                 SL_RED, app.eng.name_of(dst).c_str(), dst, SL_RST);
        continue;
      }
      app.pending_ack[seq] = sl::now_ms();
    }

    std::vector<uint8_t> route_pairs;
    for (size_t i = 0; i < rt.ids.size(); i++) {
      route_pairs.push_back(rt.ids[i]);
      route_pairs.push_back(i < rt.cidx.size() ? rt.cidx[i] : 0);
    }

    std::vector<uint8_t> ct(text.begin(), text.end());
    if (ct.size() > 140) ct.resize(140);
    sl_crypt(app.cfg.id, seq, ct.data(), (uint8_t)ct.size());

    uint8_t frame[SL_MAX_FRAME];
    uint16_t flen = sl_build(frame, sizeof(frame), SL_T_MSG, 0, seq, app.cfg.id, dst,
                             route_pairs.data(), (uint8_t)rt.ids.size(), ct.data(), (uint8_t)ct.size());
    if (!flen) continue;
    uint8_t first_c = rt.cidx.empty() ? 0 : rt.cidx[0];
    flen = sl_append_hop(frame, flen, sizeof(frame), app.cfg.id, first_c);
    if (!flen) continue;

    std::string pstr, cstr;
    for (size_t i = 0; i < rt.ids.size(); i++) {
      if (i) pstr += " -> ";
      pstr += app.eng.name_of(rt.ids[i]);
    }
    for (size_t i = 0; i < rt.cidx.size(); i++) {
      if (i) cstr += " -> ";
      cstr += carrier_label(rt.cidx[i]);
    }

    if (rt.repaired) {
      sl::logf("%s[INFERENCE] On-demand corridor repair E_rec activated!%s", SL_YEL, SL_RST);
    }
    sl::logf("[TX] seq=%u -> %s | route=[%s] | carriers=[%s] | FM=%s",
             seq, app.eng.name_of(dst).c_str(), pstr.c_str(), cstr.c_str(),
             app.fm.tx() ? "ON" : "OFF");

    broadcast_line(app, frame_to_sl1(frame, flen));
    if (app.fm.tx()) app.fm.transmit(std::vector<uint8_t>(frame, frame + flen), first_c);

    // Confirm local cryptographic verification immediately; remote ACK arrives asynchronously
    SlView v;
    if (sl_parse(frame, flen, &v) && sl_verify(frame, &v)) {
      sl::logf("%s[STATUS: SUCCESS 400] Frame encoded, HMAC-signed & transmitted on %s%s",
               SL_GRN, carrier_label(first_c).c_str(), SL_RST);
    }
  }
}

int run_self_test() {
  std::printf("=== SMART-LINK C++17 End-to-End Self-Test ===\n");

  // 1. Frame build + cipher + 3-hop HMAC chain + tamper check
  const char* msg = "SMART-LINK C++ FM MESH";
  std::vector<uint8_t> ct((const uint8_t*)msg, (const uint8_t*)msg + std::strlen(msg));
  sl_crypt(0, 7, ct.data(), (uint8_t)ct.size());
  uint8_t route[6] = {0, 0, 1, 1, 4, 2};
  uint8_t frame[SL_MAX_FRAME];
  uint16_t len = sl_build(frame, sizeof(frame), SL_T_MSG, 0, 7, 0, 4, route, 3, ct.data(), (uint8_t)ct.size());
  len = sl_append_hop(frame, len, sizeof(frame), 0, 0);
  len = sl_append_hop(frame, len, sizeof(frame), 1, 1);
  len = sl_append_hop(frame, len, sizeof(frame), 4, 2);
  SlView v;
  if (!sl_parse(frame, len, &v) || !sl_verify(frame, &v)) {
    std::printf("[FAIL] 3-hop HMAC chain verification failed\n");
    return 1;
  }
  std::printf("[1/4] 3-hop SL1 frame + HMAC chain: PASS\n");

  // 2. FM modulate + 6-channel demodulate bank roundtrip
  auto pcm = sl::fm_modulate(frame, len, 1, 48000, 0.7);
  sl::FmDemodBank bank(48000);
  int rx_hits = 0;
  uint8_t rx_cidx = 255;
  bank.push(pcm.data(), pcm.size(), [&](const uint8_t* f, uint16_t l, uint8_t k) {
    SlView dv;
    if (l == len && sl_parse(f, l, &dv) && sl_verify(f, &dv)) {
      rx_hits++;
      rx_cidx = k;
    }
  });
  if (rx_hits != 1 || rx_cidx != 1) {
    std::printf("[FAIL] FM FSK modem roundtrip failed (hits=%d, cidx=%u)\n", rx_hits, rx_cidx);
    return 1;
  }
  std::printf("[2/4] FM subcarrier FSK modem (8.4 kHz, 48 kHz PCM): PASS\n");

  // 3. Dynamic mesh + unplug self-healing via E_rec
  sl::Engine eng;
  for (uint8_t id = 0; id <= 4; id++) {
    auto& n = eng.nodes[id];
    n.id = id; n.name = "N" + std::to_string(id); n.online = true;
  }
  eng.nodes[0].nbrs = {1, 3};
  eng.nodes[1].nbrs = {0, 2, 3};
  eng.nodes[2].nbrs = {1, 4};
  eng.nodes[3].nbrs = {0, 1, 4};
  eng.nodes[4].nbrs = {2, 3};
  eng.rebuild(false);

  eng.nodes[1].forced_off = true;  // Unplug Relay 1
  eng.rebuild(false);
  sl::Route rt;
  if (!eng.route(0, 4, rt)) {
    std::printf("[FAIL] Self-healing route failed after unplugging N1\n");
    return 1;
  }
  for (uint8_t id : rt.ids) {
    if (id == 1) { std::printf("[FAIL] Route still traversed unplugged N1\n"); return 1; }
  }
  std::printf("[3/4] Self-healing Inference Engine (unplug N1 -> reroute): PASS\n");

  // 4. Payload decrypt roundtrip
  std::vector<uint8_t> pt(frame + v.off_payload, frame + v.off_payload + v.L);
  sl_crypt(v.src, v.seq, pt.data(), (uint8_t)pt.size());
  if (std::string((const char*)pt.data(), pt.size()) != msg) {
    std::printf("[FAIL] Payload decrypt mismatch\n");
    return 1;
  }
  std::printf("[4/4] Endpoint SHA-256 CTR decrypt: PASS\n");
  std::printf("ALL SELF-TESTS PASSED -> STATUS: SUCCESS 400\n");
  return 0;
}

void print_help() {
  std::printf(
      "Commands:\n"
      "  <message>          Encode, FM-modulate, route & send to all online nodes\n"
      "  send <dst> <msg>   Send directly to node <dst> (by name or ID)\n"
      "  status             Show live nodes, links, carriers & bridge count\n"
      "  unplug <node>      Simulate unplugging a node (triggers Inference Engine)\n"
      "  plug <node>        Restore an unplugged node\n"
      "  cut <a> <b>        Simulate link failure between <a> and <b>\n"
      "  heal <a> <b>       Restore link between <a> and <b>\n"
      "  jam <khz>          Jam carrier (75/84/95/108/120/135)\n"
      "  unjam <khz>        Unjam carrier\n"
      "  fm on|off          Toggle live FM audio subcarrier transmission\n"
      "  ports              List detected serial/USB ports\n"
      "  help | quit\n");
}

}  // namespace

int main(int argc, char** argv) {
  Config cfg;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--self-test") cfg.self_test = true;
    else if (a == "--no-color") sl::Log::color() = false;
    else if (a == "--verbose") sl::Log::verbose() = true;
    else if (a == "--fm-tx") cfg.fm_tx = true;
    else if (a == "--no-fm-tx") cfg.fm_tx = false;
    else if (a == "--fm-rx") cfg.fm_rx = true;
    else if (a == "--no-bcast") cfg.broadcast = false;
    else if (a == "--id" && i + 1 < argc) cfg.id = (uint8_t)std::atoi(argv[++i]);
    else if (a == "--name" && i + 1 < argc) cfg.name = argv[++i];
    else if (a == "--role" && i + 1 < argc) cfg.role = argv[++i];
    else if (a == "--port" && i + 1 < argc) cfg.udp_port = (uint16_t)std::atoi(argv[++i]);
    else if (a == "--baud" && i + 1 < argc) cfg.baud = std::atoi(argv[++i]);
    else if (a == "--peer" && i + 1 < argc) cfg.peers.push_back(argv[++i]);
    else if (a == "--serial" && i + 1 < argc) cfg.serial_devs.push_back(argv[++i]);
    else if (a == "--fm-tx-cmd" && i + 1 < argc) cfg.fm_tx_cmd = argv[++i];
    else if (a == "--fm-rx-cmd" && i + 1 < argc) cfg.fm_rx_cmd = argv[++i];
  }

  if (cfg.self_test) return run_self_test();

  App app;
  app.cfg = cfg;
  {
    auto& me = app.eng.nodes[cfg.id];
    me.id = cfg.id; me.name = cfg.name; me.role = cfg.role;
    me.online = true; me.self = true; me.last_seen = sl::now_ms();
  }

  if (!app.udp.open(cfg.udp_port, cfg.peers, cfg.broadcast)) {
    sl::logf("%s[WARN] Could not bind UDP port %u (continuing with serial/FM)%s",
             SL_YEL, cfg.udp_port, SL_RST);
  } else {
    app.udp.start([&](const std::string& l, const std::string& o) { handle_line(app, l, o); });
  }

  for (auto& dev : cfg.serial_devs) {
    auto s = std::make_unique<sl::SerialLink>();
    if (s->open(dev, cfg.baud)) {
      sl::logf("%s[USB-SERIAL] Opened %s @ %d baud%s", SL_GRN, dev.c_str(), cfg.baud, SL_RST);
      s->start([&](const std::string& l, const std::string& o) { handle_line(app, l, o); });
      app.serials.push_back(std::move(s));
    } else {
      sl::logf("%s[USB-SERIAL] Failed to open %s%s", SL_RED, dev.c_str(), SL_RST);
    }
  }

  sl::FmRadio::Config fcfg;
  fcfg.tx = cfg.fm_tx;
  fcfg.rx = cfg.fm_rx;
  fcfg.tx_cmd = cfg.fm_tx_cmd;
  fcfg.rx_cmd = cfg.fm_rx_cmd;
  app.fm.start(fcfg, [&](const uint8_t* f, uint16_t len, uint8_t cidx) {
    handle_frame(app, f, len, "fm:" + carrier_label(cidx));
  });

  std::printf("%s================================================================%s\n", SL_BLD, SL_RST);
  std::printf("  SMART-LINK C++17 Node: %s (ID=%u, Role=%s)\n", cfg.name.c_str(), cfg.id, cfg.role.c_str());
  std::printf("  UDP Port: %u | Serial Links: %zu | FM TX: %s | FM RX: %s\n",
              cfg.udp_port, app.serials.size(), cfg.fm_tx ? "ON" : "OFF", cfg.fm_rx ? "ON" : "OFF");
  std::printf("%s================================================================%s\n", SL_BLD, SL_RST);
  if (cfg.role == "hq" || cfg.role == "chat") print_help();

  // Heartbeat + timeout monitor thread
  std::thread hb_th([&] {
    while (app.running) {
      send_heartbeat(app);
      {
        std::lock_guard<std::mutex> g(app.mu);
        int64_t now = sl::now_ms();
        bool changed = false;
        for (auto& kv : app.eng.nodes) {
          auto& n = kv.second;
          if (!n.self && n.online && (now - n.last_seen) > 6000) {
            n.online = false;
            changed = true;
            sl::logf("\r%s[HEARTBEAT LOST] %s (ID=%u) disconnected -> Inference Engine rerouting!%s",
                     SL_YEL, n.name.c_str(), n.id, SL_RST);
          }
        }
        if (changed) app.eng.rebuild(false);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });

  if (cfg.role != "hq" && cfg.role != "chat") {
    sl::logf("[%s] Relay Inference Engine running. Press Ctrl+C to stop.", cfg.name.c_str());
    while (app.running) std::this_thread::sleep_for(std::chrono::seconds(1));
  } else {
    std::string line;
    while (app.running) {
      std::printf("[%s]> ", cfg.name.c_str());
      std::fflush(stdout);
      if (!std::getline(std::cin, line)) break;
      std::string cmd = sl::trim(line);
      if (cmd.empty()) continue;
      if (!cmd.empty() && cmd[0] == '/') cmd = cmd.substr(1);
      if (cmd == "quit" || cmd == "exit") break;
      else if (cmd == "help") print_help();
      else if (cmd == "status") print_status(app);
      else if (cmd == "ports") {
        auto ps = sl::list_serial_ports();
        std::printf("Detected serial ports (%zu):", ps.size());
        for (auto& p : ps) std::printf(" %s", p.c_str());
        std::printf("\n");
      } else if (cmd == "fm on") { app.fm.set_tx(true); std::printf("FM TX enabled.\n"); }
      else if (cmd == "fm off") { app.fm.set_tx(false); std::printf("FM TX disabled.\n"); }
      else if (cmd.rfind("unplug ", 0) == 0) {
        std::lock_guard<std::mutex> g(app.mu);
        int id = app.eng.find(cmd.substr(7));
        if (id >= 0) { app.eng.nodes[(uint8_t)id].forced_off = true; app.eng.rebuild(false); std::printf("Unplugged %s.\n", app.eng.name_of((uint8_t)id).c_str()); }
        else std::printf("Unknown node.\n");
      } else if (cmd.rfind("plug ", 0) == 0) {
        std::lock_guard<std::mutex> g(app.mu);
        int id = app.eng.find(cmd.substr(5));
        if (id >= 0) { app.eng.nodes[(uint8_t)id].forced_off = false; app.eng.rebuild(false); std::printf("Plugged %s.\n", app.eng.name_of((uint8_t)id).c_str()); }
        else std::printf("Unknown node.\n");
      } else if (cmd.rfind("send ", 0) == 0) {
        std::istringstream iss(cmd.substr(5));
        std::string first, rest;
        iss >> first;
        std::getline(iss, rest);
        rest = sl::trim(rest);
        int tid = -1;
        {
          std::lock_guard<std::mutex> g(app.mu);
          tid = app.eng.find(first);
        }
        if (tid >= 0 && !rest.empty()) send_message(app, tid, rest);
        else send_message(app, -1, sl::trim(cmd.substr(5)));
      } else {
        send_message(app, -1, cmd);
      }
    }
  }

  app.running = false;
  if (hb_th.joinable()) hb_th.join();
  app.fm.stop();
  app.udp.stop();
  for (auto& s : app.serials) s->stop();
  return 0;
}
