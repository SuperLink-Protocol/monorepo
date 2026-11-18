// transport.cpp — cross-platform UDP + serial implementation
#include "transport.hpp"

#include <cstring>

#include "util.hpp"

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  typedef int socklen_t;
  #define SL_CLOSESOCK closesocket
#else
  #include <arpa/inet.h>
  #include <dirent.h>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <sys/ioctl.h>
  #include <sys/select.h>
  #include <sys/socket.h>
  #include <termios.h>
  #include <unistd.h>
  #define SL_CLOSESOCK close
#endif

namespace sl {

// ------------------------------------------------------------------ UDP
static void net_init_once() {
#ifdef _WIN32
  static bool done = false;
  if (!done) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); done = true; }
#endif
}

UdpLink::~UdpLink() { stop(); }

bool UdpLink::open(uint16_t port, const std::vector<std::string>& peers, bool broadcast) {
  net_init_once();
  port_ = port;
  bcast_ = broadcast;
  for (auto& p : peers) {
    auto c = p.find(':');
    if (c == std::string::npos) peers_.push_back({p, port});
    else peers_.push_back({p.substr(0, c), (uint16_t)std::stoi(p.substr(c + 1))});
  }
  auto s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
  if (s == INVALID_SOCKET) return false;
#else
  if (s < 0) return false;
#endif
  int yes = 1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, (const char*)&yes, sizeof(yes));
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
#ifdef _WIN32
  DWORD tv = 300;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#else
  timeval tv{0, 300000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr*)&a, sizeof(a)) != 0) { SL_CLOSESOCK(s); return false; }
  sock_ = (intptr_t)s;
  return true;
}

void UdpLink::start(LineHandler h) {
  run_ = true;
  th_ = std::thread([this, h] {
    char buf[2048];
    while (run_) {
      sockaddr_in from{};
      socklen_t fl = sizeof(from);
      int n = recvfrom((decltype(socket(0, 0, 0)))sock_, buf, sizeof(buf) - 1, 0, (sockaddr*)&from, &fl);
      if (n <= 0) continue;
      buf[n] = 0;
      char ip[64] = {0};
      inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
      std::string origin = std::string("udp:") + ip;
      // a datagram may hold one or more lines
      std::string all(buf, n);
      size_t st = 0;
      while (st < all.size()) {
        size_t e = all.find('\n', st);
        if (e == std::string::npos) e = all.size();
        std::string line = trim(all.substr(st, e - st));
        if (!line.empty()) h(line, origin);
        st = e + 1;
      }
    }
  });
}

void UdpLink::send_line(const std::string& line) {
  if (sock_ == -1) return;
  std::lock_guard<std::mutex> g(wmu_);
  std::string msg = line + "\n";
  auto s = (decltype(socket(0, 0, 0)))sock_;
  if (bcast_) {
    sockaddr_in b{};
    b.sin_family = AF_INET;
    b.sin_port = htons(port_);
    b.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    sendto(s, msg.data(), (int)msg.size(), 0, (sockaddr*)&b, sizeof(b));
  }
  for (auto& p : peers_) {
    sockaddr_in u{};
    u.sin_family = AF_INET;
    u.sin_port = htons(p.second);
    inet_pton(AF_INET, p.first.c_str(), &u.sin_addr);
    sendto(s, msg.data(), (int)msg.size(), 0, (sockaddr*)&u, sizeof(u));
  }
}

void UdpLink::stop() {
  run_ = false;
  if (th_.joinable()) th_.join();
  if (sock_ != -1) { SL_CLOSESOCK((decltype(socket(0, 0, 0)))sock_); sock_ = -1; }
}

// ------------------------------------------------------------------ Serial
SerialLink::~SerialLink() { stop(); }

#ifdef _WIN32
bool SerialLink::open(const std::string& dev, int baud) {
  dev_ = dev;
  std::string path = dev.rfind("\\\\.\\", 0) == 0 ? dev : "\\\\.\\" + dev;
  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) return false;
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  GetCommState(h, &dcb);
  dcb.BaudRate = (DWORD)baud;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;   // Uno auto-resets on open; ESP32 boots normally
  dcb.fRtsControl = RTS_CONTROL_DISABLE;  // keep ESP32 out of reset/bootloader
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  SetCommState(h, &dcb);
  COMMTIMEOUTS to{};
  to.ReadIntervalTimeout = 20;
  to.ReadTotalTimeoutConstant = 200;
  to.ReadTotalTimeoutMultiplier = 0;
  to.WriteTotalTimeoutConstant = 500;
  SetCommTimeouts(h, &to);
  h_ = h;
  return true;
}

void SerialLink::start(LineHandler hnd) {
  run_ = true;
  th_ = std::thread([this, hnd] {
    std::string acc;
    char buf[512];
    std::string origin = "serial:" + dev_;
    while (run_) {
      DWORD n = 0;
      if (!ReadFile((HANDLE)h_, buf, sizeof(buf), &n, nullptr)) {
        logf("%s[%s] read error - device unplugged?%s", SL_RED, dev_.c_str(), SL_RST);
        Sleep(1000);
        continue;
      }
      for (DWORD i = 0; i < n; i++) {
        if (buf[i] == '\n') { std::string l = trim(acc); acc.clear(); if (!l.empty()) hnd(l, origin); }
        else if (acc.size() < 4096) acc.push_back(buf[i]);
      }
    }
  });
}

void SerialLink::send_line(const std::string& line) {
  if (!h_) return;
  std::lock_guard<std::mutex> g(wmu_);
  std::string m = line + "\n";
  DWORD w = 0;
  WriteFile((HANDLE)h_, m.data(), (DWORD)m.size(), &w, nullptr);
}

void SerialLink::stop() {
  run_ = false;
  if (th_.joinable()) th_.join();
  if (h_) { CloseHandle((HANDLE)h_); h_ = nullptr; }
}

std::vector<std::string> list_serial_ports() {
  std::vector<std::string> out;
  char target[512];
  for (int i = 1; i <= 64; i++) {
    std::string n = "COM" + std::to_string(i);
    if (QueryDosDeviceA(n.c_str(), target, sizeof(target))) out.push_back(n);
  }
  return out;
}

#else  // POSIX

static speed_t to_speed(int b) {
  switch (b) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 230400: return B230400;
    default: return B115200;
  }
}

static int64_t g_serial_open_ms = 0;

bool SerialLink::open(const std::string& dev, int baud) {
  dev_ = dev;
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) return false;
  termios t{};
  tcgetattr(fd, &t);
  cfmakeraw(&t);
  cfsetispeed(&t, to_speed(baud));
  cfsetospeed(&t, to_speed(baud));
  t.c_cflag |= (CLOCAL | CREAD);
  t.c_cflag &= ~CRTSCTS;
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 2;
  tcsetattr(fd, TCSANOW, &t);
  int dtr = TIOCM_DTR, rts = TIOCM_RTS;
  ioctl(fd, TIOCMBIS, &dtr);  // Enable DTR (required by Uno ATmega16U2 CDC bridge after avrdude)
  ioctl(fd, TIOCMBIC, &rts);  // Disable RTS (keeps ESP32 out of reset)
  fd_ = fd;
  g_serial_open_ms = now_ms();
  return true;
}

void SerialLink::start(LineHandler hnd) {
  run_ = true;
  th_ = std::thread([this, hnd] {
    std::string acc;
    char buf[512];
    std::string origin = "serial:" + dev_;
    while (run_) {
      ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n < 0) { usleep(200000); continue; }
      for (ssize_t i = 0; i < n; i++) {
        if (buf[i] == '\n') { std::string l = trim(acc); acc.clear(); if (!l.empty()) hnd(l, origin); }
        else if (acc.size() < 4096) acc.push_back(buf[i]);
      }
    }
  });
}

void SerialLink::send_line(const std::string& line) {
  if (fd_ < 0) return;
  if (now_ms() - g_serial_open_ms < 2300) return;  // Do not disturb Uno Optiboot during 1.8s DTR reset
  std::lock_guard<std::mutex> g(wmu_);
  std::string m = "\n" + line + "\n";
  for (size_t i = 0; i < m.size(); i += 16) {
    size_t chunk = std::min((size_t)16, m.size() - i);
    ssize_t r = ::write(fd_, m.data() + i, chunk);
    (void)r;
    tcdrain(fd_);
    usleep(6000);
  }
}

void SerialLink::stop() {
  run_ = false;
  if (th_.joinable()) th_.join();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

std::vector<std::string> list_serial_ports() {
  std::vector<std::string> out;
  DIR* d = opendir("/dev");
  if (!d) return out;
  while (auto* e = readdir(d)) {
    std::string n = e->d_name;
    if (n.rfind("ttyUSB", 0) == 0 || n.rfind("ttyACM", 0) == 0 || n == "serial0" || n == "ttyAMA0")
      out.push_back("/dev/" + n);
  }
  closedir(d);
  return out;
}
#endif

}  // namespace sl
