// util.hpp — logging, time, hex helpers shared by the host engine
#pragma once
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace sl {

inline int64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

struct Log {
  static std::mutex& mu() { static std::mutex m; return m; }
  static bool& color() { static bool c = true; return c; }
  static bool& verbose() { static bool v = false; return v; }
};

// ANSI colour helpers (disabled with --no-color)
inline const char* C(const char* code) { return Log::color() ? code : ""; }
#define SL_RST   sl::C("\x1b[0m")
#define SL_DIM   sl::C("\x1b[2m")
#define SL_RED   sl::C("\x1b[31m")
#define SL_GRN   sl::C("\x1b[32m")
#define SL_YEL   sl::C("\x1b[33m")
#define SL_BLU   sl::C("\x1b[34m")
#define SL_MAG   sl::C("\x1b[35m")
#define SL_CYN   sl::C("\x1b[36m")
#define SL_BLD   sl::C("\x1b[1m")

template <typename... A>
inline void logf(const char* fmt, A... a) {
  std::lock_guard<std::mutex> g(Log::mu());
  std::printf(fmt, a...);
  std::printf("\n");
  std::fflush(stdout);
}
inline void logs(const std::string& s) {
  std::lock_guard<std::mutex> g(Log::mu());
  std::printf("%s\n", s.c_str());
  std::fflush(stdout);
}

inline std::string to_hex(const uint8_t* d, size_t n) {
  static const char* H = "0123456789abcdef";
  std::string s; s.reserve(n * 2);
  for (size_t i = 0; i < n; i++) { s.push_back(H[d[i] >> 4]); s.push_back(H[d[i] & 15]); }
  return s;
}
inline bool from_hex(const std::string& h, std::vector<uint8_t>& out) {
  if (h.size() % 2) return false;
  out.clear(); out.reserve(h.size() / 2);
  auto v = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  for (size_t i = 0; i < h.size(); i += 2) {
    int a = v(h[i]), b = v(h[i + 1]);
    if (a < 0 || b < 0) return false;
    out.push_back((uint8_t)((a << 4) | b));
  }
  return true;
}

inline std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

}  // namespace sl
