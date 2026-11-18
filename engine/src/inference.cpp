// inference.cpp — implementation of the SMART-LINK engine (paper §Backbone, §Spectrum, §Recovery)
#include "inference.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <queue>

#include "smartlink_core.h"

namespace sl {

bool Engine::alive(uint8_t id) const {
  auto it = nodes.find(id);
  return it != nodes.end() && it->second.online && !it->second.forced_off;
}

std::string Engine::name_of(uint8_t id) const {
  auto it = nodes.find(id);
  return it == nodes.end() || it->second.name.empty() ? "node-" + std::to_string(id) : it->second.name;
}

int Engine::find(const std::string& q) const {
  std::string l = q;
  std::transform(l.begin(), l.end(), l.begin(), [](unsigned char c) { return (char)std::tolower(c); });
  for (auto& kv : nodes) {
    std::string n = kv.second.name;
    std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    if (n == l || std::to_string(kv.first) == l) return kv.first;
  }
  return -1;
}

// Candidate graph G0: an edge exists when either endpoint reports hearing the other directly.
std::vector<std::pair<Edge, double>> Engine::candidates() const {
  std::set<Edge> es;
  for (auto& kv : nodes) {
    if (!alive(kv.first)) continue;
    for (uint8_t n : kv.second.nbrs)
      if (n != kv.first && alive(n)) es.insert(mk(kv.first, n));
  }
  std::vector<std::pair<Edge, double>> out;
  for (auto& e : es)
    if (!cut.count(e)) out.push_back({e, 1.0});
  std::sort(out.begin(), out.end(), [](const std::pair<Edge, double>& x, const std::pair<Edge, double>& y) {
    if (x.second != y.second) return x.second < y.second;
    return x.first < y.first;
  });
  return out;
}

void Engine::rebuild(bool keep_recovery) {
  if (!keep_recovery) erec.clear();
  auto cand = candidates();
  links.clear();

  // --- Kruskal MST backbone T
  std::map<uint8_t, uint8_t> par;
  for (auto& kv : nodes) par[kv.first] = kv.first;
  std::function<uint8_t(uint8_t)> fnd = [&](uint8_t x) { while (par[x] != x) x = par[x] = par[par[x]]; return x; };
  std::vector<std::pair<Edge, double>> nontree;
  std::map<uint8_t, std::vector<uint8_t>> tadj;
  for (auto& c : cand) {
    uint8_t a = fnd(c.first.first), b = fnd(c.first.second);
    if (a != b) {
      par[a] = b;
      links.push_back({c.first.first, c.first.second, c.second, 'T', -1});
      tadj[c.first.first].push_back(c.first.second);
      tadj[c.first.second].push_back(c.first.first);
    } else {
      nontree.push_back(c);
    }
  }

  // --- Fundamental cycle of each non-tree edge = tree path between its endpoints
  auto tree_path = [&](uint8_t s, uint8_t d) {
    std::map<uint8_t, uint8_t> prev;
    std::queue<uint8_t> q;
    q.push(s); prev[s] = s;
    while (!q.empty()) {
      uint8_t u = q.front(); q.pop();
      if (u == d) break;
      for (uint8_t v : tadj[u]) if (!prev.count(v)) { prev[v] = u; q.push(v); }
    }
    std::set<Edge> p;
    if (!prev.count(d)) return p;
    for (uint8_t x = d; x != s; x = prev[x]) p.insert(mk(x, prev[x]));
    return p;
  };

  // --- Greedy budgeted submodular backup selection: max F(S) = |Cov(S)| (unit weights)
  std::set<Edge> covered;
  std::vector<std::set<Edge>> cyc;
  for (auto& e : nontree) cyc.push_back(tree_path(e.first.first, e.first.second));
  std::vector<bool> used(nontree.size(), false);
  for (int k = 0; k < budget; k++) {
    int best = -1; size_t bestg = 0;
    for (size_t i = 0; i < nontree.size(); i++) {
      if (used[i]) continue;
      size_t g = 0;
      for (auto& te : cyc[i]) if (!covered.count(te)) g++;
      if (g > bestg) { bestg = g; best = (int)i; }
    }
    if (best < 0) break;
    used[best] = true;
    for (auto& te : cyc[best]) covered.insert(te);
    links.push_back({nontree[best].first.first, nontree[best].first.second, nontree[best].second, 'B', -1});
  }
  size_t tree_n = 0;
  for (auto& l : links) if (l.kind == 'T') tree_n++;
  bridges = (int)(tree_n - covered.size());
  coverage = tree_n ? (double)covered.size() / tree_n : 1.0;

  // --- Persisting on-demand recovery links E_rec
  for (auto& e : erec) {
    bool present = false;
    for (auto& l : links) if (mk(l.a, l.b) == e) present = true;
    if (!present && alive(e.first) && alive(e.second) && !cut.count(e))
      links.push_back({e.first, e.second, 1.0, 'R', -1});
  }
  assign_carriers();
}

// Conflict graph H_c: links sharing a station conflict. Degree-ordered first-fit, recovery links first.
void Engine::assign_carriers() {
  size_t m = links.size();
  std::vector<std::vector<size_t>> conf(m);
  for (size_t i = 0; i < m; i++)
    for (size_t j = i + 1; j < m; j++) {
      auto& x = links[i]; auto& y = links[j];
      if (x.a == y.a || x.a == y.b || x.b == y.a || x.b == y.b) { conf[i].push_back(j); conf[j].push_back(i); }
    }
  std::vector<size_t> order(m);
  for (size_t i = 0; i < m; i++) order[i] = i;
  std::stable_sort(order.begin(), order.end(), [&](size_t i, size_t j) {
    bool ri = links[i].kind == 'R', rj = links[j].kind == 'R';
    if (ri != rj) return ri;
    return conf[i].size() > conf[j].size();
  });
  for (auto& l : links) l.cidx = -1;
  for (size_t i : order) {
    for (int c = 0; c < SL_NUM_CARRIERS; c++) {
      if (jammed.count(c)) continue;
      bool ok = true;
      for (size_t n : conf[i]) {
        int o = links[n].cidx;
        if (o >= 0 && std::abs((int)SL_CARRIERS_KHZ[c] - (int)SL_CARRIERS_KHZ[o]) < 9) { ok = false; break; }
      }
      if (ok) { links[i].cidx = c; break; }
    }
  }
}

int Engine::carrier_of(uint8_t a, uint8_t b) const {
  for (auto& l : links) if (mk(l.a, l.b) == mk(a, b)) return l.cidx;
  return -1;
}

bool Engine::shortest(uint8_t s, uint8_t d, bool full, std::vector<uint8_t>& path) const {
  std::map<uint8_t, std::vector<std::pair<uint8_t, double>>> adj;
  if (full) {
    for (auto& c : candidates()) {
      adj[c.first.first].push_back({c.first.second, c.second});
      adj[c.first.second].push_back({c.first.first, c.second});
    }
  } else {
    for (auto& l : links) {
      if (l.cidx < 0) continue;
      adj[l.a].push_back({l.b, l.cost});
      adj[l.b].push_back({l.a, l.cost});
    }
  }
  std::map<uint8_t, double> dist;
  std::map<uint8_t, uint8_t> prev;
  using QE = std::pair<double, uint8_t>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[s] = 0; pq.push({0, s});
  while (!pq.empty()) {
    auto [c, u] = pq.top(); pq.pop();
    if (c > dist[u]) continue;
    if (u == d) break;
    for (auto& [v, w] : adj[u]) {
      double nc = c + w;
      if (!dist.count(v) || nc < dist[v]) { dist[v] = nc; prev[v] = u; pq.push({nc, v}); }
    }
  }
  if (!dist.count(d)) return false;
  path.clear();
  for (uint8_t x = d; x != s; x = prev[x]) path.push_back(x);
  path.push_back(s);
  std::reverse(path.begin(), path.end());
  return true;
}

bool Engine::route(uint8_t src, uint8_t dst, Route& out) {
  out = Route{};
  if (!alive(src) || !alive(dst) || src == dst) return false;
  std::vector<uint8_t> p;
  if (!shortest(src, dst, false, p)) {
    // On-demand corridor repair: E_rec = Q0 \ (E_T ∪ S), activated outside budget k
    std::vector<uint8_t> corridor;
    if (!shortest(src, dst, true, corridor)) return false;
    for (size_t i = 0; i + 1 < corridor.size(); i++) {
      Edge e = mk(corridor[i], corridor[i + 1]);
      bool active = false;
      for (auto& l : links) if (mk(l.a, l.b) == e && l.cidx >= 0) active = true;
      if (!active) { erec.insert(e); out.erec.push_back(e); }
    }
    rebuild(true);
    if (!shortest(src, dst, false, p)) return false;
    out.repaired = true;
  }
  out.ids = p;
  for (size_t i = 0; i + 1 < p.size(); i++) out.cidx.push_back((uint8_t)carrier_of(p[i], p[i + 1]));
  return true;
}

}  // namespace sl
