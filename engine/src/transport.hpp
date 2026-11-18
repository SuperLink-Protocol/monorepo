// transport.hpp — UDP broadcast mesh link + USB/UART serial link (Windows & Linux/Pi)
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sl {

// Called for every received text line. origin = "udp:1.2.3.4" | "serial:COM5" ...
using LineHandler = std::function<void(const std::string& line, const std::string& origin)>;

class UdpLink {
 public:
  ~UdpLink();
  // peers: "ip" or "ip:port". broadcast=false -> only unicast to peers.
  bool open(uint16_t port, const std::vector<std::string>& peers, bool broadcast);
  void start(LineHandler h);
  void send_line(const std::string& line);
  void stop();
  uint16_t port() const { return port_; }

 private:
  intptr_t sock_ = -1;
  uint16_t port_ = 0;
  bool bcast_ = true;
  std::vector<std::pair<std::string, uint16_t>> peers_;
  std::thread th_;
  std::atomic<bool> run_{false};
  std::mutex wmu_;
};

class SerialLink {
 public:
  ~SerialLink();
  bool open(const std::string& dev, int baud);
  void start(LineHandler h);
  void send_line(const std::string& line);
  void stop();
  const std::string& name() const { return dev_; }

 private:
#ifdef _WIN32
  void* h_ = nullptr;
#else
  int fd_ = -1;
#endif
  std::string dev_;
  std::thread th_;
  std::atomic<bool> run_{false};
  std::mutex wmu_;
};

// Enumerate serial ports present on this machine (best effort)
std::vector<std::string> list_serial_ports();

}  // namespace sl
