// fm.hpp — FM audio-subcarrier FSK PHY (modulator, demodulator bank, sound-card / radio I/O)
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sl {

using FrameCb = std::function<void(const uint8_t* f, uint16_t len, uint8_t cidx)>;

// Continuous-phase FSK on the audio subcarrier for carrier index cidx (see smartlink_core.h §7)
std::vector<int16_t> fm_modulate(const uint8_t* frame, uint16_t len, uint8_t cidx, int fs, double gain = 0.5);
bool write_wav(const std::string& path, const std::vector<int16_t>& s, int fs);
bool read_wav(const std::string& path, std::vector<int16_t>& s, int& fs);
double fm_airtime_s(uint16_t frame_len);

// Six parallel subcarrier decoders (one per SMART-LINK carrier)
class FmDemodBank {
 public:
  explicit FmDemodBank(int fs);
  ~FmDemodBank();
  void push(const int16_t* s, size_t n, const FrameCb& cb);

 private:
  void* ch_ = nullptr;  // SlFmChan[6]
};

class FmRadio {
 public:
  struct Config {
    int fs = 48000;
    bool tx = false, rx = false;
    std::string tx_cmd;  // optional external transmitter command, "{wav}" is replaced by a WAV path
    std::string rx_cmd;  // optional external capture command producing raw S16LE mono @ fs on stdout
    double gain = 0.5;
    std::string wav_dir = ".";
  };
  ~FmRadio();
  bool start(const Config& c, FrameCb on_frame);
  void transmit(const std::vector<uint8_t>& frame, uint8_t cidx);
  void stop();
  bool tx() const { return cfg_.tx; }
  bool rx() const { return cfg_.rx; }
  void set_tx(bool on) { cfg_.tx = on; }
  const Config& cfg() const { return cfg_; }
  bool play_blocking(const std::vector<int16_t>& s);

 private:
  void tx_loop();
  void rx_loop();
  Config cfg_;
  FrameCb cb_;
  std::thread txth_, rxth_;
  std::atomic<bool> run_{false};
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<std::pair<std::vector<uint8_t>, uint8_t>> q_;
};

}  // namespace sl
