// inference.hpp — SMART-LINK Inference Engine (dynamic membership, backbone, backups, carriers, routing)
#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace sl {

struct NodeRec {
  uint8_t id = 0;
  std::string name, role, via;
  uint8_t caps = 0;
  std::set<uint8_t> nbrs;        // neighbours this node reports hearing directly
  int64_t last_seen = 0;
  bool online = false;
  bool forced_off = false;       // CLI "unplug" simulation
  bool self = false;
};

struct LinkRec {
  uint8_t a, b;
  double cost;
  char kind;                     // 'T' tree, 'B' backup, 'R' on-demand recovery (E_rec)
  int cidx = -1;                 // carrier index into SL_CARRIERS_KHZ, -1 = UNASSIGNED
};

struct Route {
  std::vector<uint8_t> ids;      // node ids src..dst
  std::vector<uint8_t> cidx;     // carrier of link ids[k] -> ids[k+1]
  bool repaired = false;
  std::vector<std::pair<uint8_t, uint8_t>> erec;
};

using Edge = std::pair<uint8_t, uint8_t>;  // always (min,max)
inline Edge mk(uint8_t a, uint8_t b) { return a < b ? Edge{a, b} : Edge{b, a}; }

class Engine {
 public:
  std::map<uint8_t, NodeRec> nodes;
  std::vector<LinkRec> links;    // active topology A = T ∪ S ∪ E_rec
  std::set<Edge> cut;            // simulated link failures
  std::set<int> jammed;          // jammed carrier indices
  int budget = 2;                // backup budget k
  int bridges = 0;
  double coverage = 0.0;         // fraction of tree links protected by a backup cycle
  std::set<Edge> erec;           // persistent recovery links (cleared on membership change)

  bool alive(uint8_t id) const;
  std::vector<std::pair<Edge, double>> candidates() const;  // G0 on live nodes
  void rebuild(bool keep_recovery = false);                 // MST + greedy backups + carriers
  bool route(uint8_t src, uint8_t dst, Route& out);         // Dijkstra + on-demand repair
  int carrier_of(uint8_t a, uint8_t b) const;
  std::string name_of(uint8_t id) const;
  int find(const std::string& q) const;                     // by name (case-insensitive) or id

 private:
  void assign_carriers();
  bool shortest(uint8_t s, uint8_t d, bool full, std::vector<uint8_t>& path) const;
};

}  // namespace sl
