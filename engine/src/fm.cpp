// fm.cpp — FM subcarrier FSK modem + audio I/O backends
//   Windows : winmm waveOut / waveIn (laptop speaker/line-out -> FM transmitter, FM receiver -> mic/line-in)
//   Linux/Pi: aplay / arecord, or an external radio command (e.g. rpitx/PiFmRds on the Pi GPIO4 antenna)
#include "fm.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "smartlink_core.h"
#include "util.hpp"

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
  #include <mmsystem.h>
  #define SL_POPEN _popen
  #define SL_PCLOSE _pclose
#else
  #include <unistd.h>
  #define SL_POPEN popen
  #define SL_PCLOSE pclose
#endif

namespace sl {

std::vector<int16_t> fm_modulate(const uint8_t* frame, uint16_t len, uint8_t cidx, int fs, double gain) {
  const uint32_t bits = sl_fm_air_bits(len);
  const double spb = (double)fs / SL_FM_BAUD;
  std::vector<int16_t> out;
  out.reserve((size_t)(bits * spb) + fs / 20);
  double phase = 0, t_next = 0;
  size_t n = 0;
  for (uint32_t k = 0; k < bits; k++) {
    const double f = (double)sl_fm_tone_hz(cidx, sl_fm_air_bit(frame, len, k));
    t_next += spb;
    for (; (double)n < t_next; n++) {
      phase += 2.0 * 3.14159265358979 * f / fs;
      if (phase > 6.283185307179586) phase -= 6.283185307179586;
      out.push_back((int16_t)(std::sin(phase) * gain * 32767.0));
    }
  }
  // 50 ms fade-out silence so sound cards don't clip the tail
  for (int i = 0; i < fs / 20; i++) out.push_back(0);
  return out;
}

double fm_airtime_s(uint16_t len) { return (double)sl_fm_air_bits(len) / SL_FM_BAUD + 0.05; }

bool write_wav(const std::string& path, const std::vector<int16_t>& s, int fs) {
  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  uint32_t data = (uint32_t)(s.size() * 2), riff = 36 + data, br = (uint32_t)fs * 2, fmtlen = 16, sr = (uint32_t)fs;
  uint16_t pcm = 1, ch = 1, ba = 2, bps = 16;
  std::fwrite("RIFF", 1, 4, f); std::fwrite(&riff, 4, 1, f); std::fwrite("WAVEfmt ", 1, 8, f);
  std::fwrite(&fmtlen, 4, 1, f); std::fwrite(&pcm, 2, 1, f); std::fwrite(&ch, 2, 1, f);
  std::fwrite(&sr, 4, 1, f); std::fwrite(&br, 4, 1, f); std::fwrite(&ba, 2, 1, f); std::fwrite(&bps, 2, 1, f);
  std::fwrite("data", 1, 4, f); std::fwrite(&data, 4, 1, f);
  std::fwrite(s.data(), 2, s.size(), f);
  std::fclose(f);
  return true;
}

bool read_wav(const std::string& path, std::vector<int16_t>& s, int& fs) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  std::vector<uint8_t> b;
  uint8_t tmp[4096];
  size_t n;
  while ((n = std::fread(tmp, 1, sizeof(tmp), f)) > 0) b.insert(b.end(), tmp, tmp + n);
  std::fclose(f);
  if (b.size() < 44 || std::memcmp(b.data(), "RIFF", 4) != 0) return false;
  size_t p = 12;
  uint16_t chans = 1;
  fs = 48000;
  while (p + 8 <= b.size()) {
    uint32_t sz;
    std::memcpy(&sz, &b[p + 4], 4);
    if (std::memcmp(&b[p], "fmt ", 4) == 0) {
      std::memcpy(&chans, &b[p + 10], 2);
      uint32_t sr; std::memcpy(&sr, &b[p + 12], 4); fs = (int)sr;
    } else if (std::memcmp(&b[p], "data", 4) == 0) {
      size_t end = std::min(b.size(), p + 8 + sz);
      for (size_t i = p + 8; i + 2 * chans <= end; i += 2 * chans) {
        int16_t v; std::memcpy(&v, &b[i], 2); s.push_back(v);
      }
      return true;
    }
    p += 8 + sz + (sz & 1);
  }
  return false;
}

// ------------------------------------------------------------------ demod bank
FmDemodBank::FmDemodBank(int fs) {
  auto* c = new SlFmChan[SL_NUM_CARRIERS];
  for (uint8_t i = 0; i < SL_NUM_CARRIERS; i++) sl_fm_chan_init(&c[i], i, (float)fs);
  ch_ = c;
}
FmDemodBank::~FmDemodBank() { delete[] (SlFmChan*)ch_; }

void FmDemodBank::push(const int16_t* s, size_t n, const FrameCb& cb) {
  auto* c = (SlFmChan*)ch_;
  for (size_t i = 0; i < n; i++) {
    float x = s[i] / 32768.0f;
    for (uint8_t k = 0; k < SL_NUM_CARRIERS; k++) {
      uint16_t l = sl_fm_push(&c[k], x);
      if (l) cb(c[k].fbuf, l, k);
    }
  }
}

// ------------------------------------------------------------------ radio I/O
FmRadio::~FmRadio() { stop(); }

bool FmRadio::start(const Config& c, FrameCb on_frame) {
  cfg_ = c;
  cb_ = on_frame;
  run_ = true;
  txth_ = std::thread([this] { tx_loop(); });
  if (cfg_.rx) rxth_ = std::thread([this] { rx_loop(); });
  return true;
}

void FmRadio::stop() {
  if (!run_) return;
  run_ = false;
  cv_.notify_all();
  if (txth_.joinable()) txth_.join();
  if (rxth_.joinable()) rxth_.detach();  // capture may block in a driver/pipe read
}

void FmRadio::transmit(const std::vector<uint8_t>& frame, uint8_t cidx) {
  if (!cfg_.tx || cidx >= SL_NUM_CARRIERS) return;
  std::lock_guard<std::mutex> g(mu_);
  q_.push_back({frame, cidx});
  cv_.notify_one();
}

bool FmRadio::play_blocking(const std::vector<int16_t>& s) {
  if (!cfg_.tx_cmd.empty()) {
    std::string wav = cfg_.wav_dir + "/smartlink_fm_tx.wav";
    write_wav(wav, s, cfg_.fs);
    std::string cmd = cfg_.tx_cmd;
    auto p = cmd.find("{wav}");
    if (p != std::string::npos) cmd.replace(p, 5, wav);
    return std::system(cmd.c_str()) == 0;
  }
#ifdef _WIN32
  WAVEFORMATEX wf{};
  wf.wFormatTag = WAVE_FORMAT_PCM; wf.nChannels = 1; wf.nSamplesPerSec = (DWORD)cfg_.fs;
  wf.wBitsPerSample = 16; wf.nBlockAlign = 2; wf.nAvgBytesPerSec = (DWORD)cfg_.fs * 2;
  HWAVEOUT wo;
  if (waveOutOpen(&wo, WAVE_MAPPER, &wf, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) return false;
  WAVEHDR h{};
  h.lpData = (LPSTR)s.data(); h.dwBufferLength = (DWORD)(s.size() * 2);
  waveOutPrepareHeader(wo, &h, sizeof(h));
  waveOutWrite(wo, &h, sizeof(h));
  while (!(h.dwFlags & WHDR_DONE)) Sleep(5);
  waveOutUnprepareHeader(wo, &h, sizeof(h));
  waveOutClose(wo);
  return true;
#else
  char cmd[160];
  std::snprintf(cmd, sizeof(cmd), "aplay -q -f S16_LE -c 1 -r %d -t raw 2>/dev/null", cfg_.fs);
  FILE* p = SL_POPEN(cmd, "w");
  if (!p) return false;
  std::fwrite(s.data(), 2, s.size(), p);
  return SL_PCLOSE(p) == 0;
#endif
}

void FmRadio::tx_loop() {
  while (run_) {
    std::pair<std::vector<uint8_t>, uint8_t> item;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [this] { return !q_.empty() || !run_; });
      if (!run_) return;
      item = std::move(q_.front());
      q_.pop_front();
    }
    auto pcm = fm_modulate(item.first.data(), (uint16_t)item.first.size(), item.second, cfg_.fs, cfg_.gain);
    if (!play_blocking(pcm))
      logf("%s[FM] TX failed (no audio device / transmitter command error)%s", SL_RED, SL_RST);
  }
}

void FmRadio::rx_loop() {
  FmDemodBank bank(cfg_.fs);
  auto deliver = [this](const uint8_t* f, uint16_t l, uint8_t k) { if (cb_) cb_(f, l, k); };
#ifdef _WIN32
  if (cfg_.rx_cmd.empty()) {
    WAVEFORMATEX wf{};
    wf.wFormatTag = WAVE_FORMAT_PCM; wf.nChannels = 1; wf.nSamplesPerSec = (DWORD)cfg_.fs;
    wf.wBitsPerSample = 16; wf.nBlockAlign = 2; wf.nAvgBytesPerSec = (DWORD)cfg_.fs * 2;
    HWAVEIN wi;
    if (waveInOpen(&wi, WAVE_MAPPER, &wf, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
      logf("%s[FM] RX: no capture device (connect FM receiver audio to mic/line-in)%s", SL_RED, SL_RST);
      return;
    }
    const int NB = 4, LEN = cfg_.fs / 20;
    std::vector<std::vector<int16_t>> bufs(NB, std::vector<int16_t>(LEN));
    std::vector<WAVEHDR> hs(NB);
    for (int i = 0; i < NB; i++) {
      hs[i] = WAVEHDR{};
      hs[i].lpData = (LPSTR)bufs[i].data(); hs[i].dwBufferLength = LEN * 2;
      waveInPrepareHeader(wi, &hs[i], sizeof(WAVEHDR));
      waveInAddBuffer(wi, &hs[i], sizeof(WAVEHDR));
    }
    waveInStart(wi);
    int cur = 0;
    while (run_) {
      if (hs[cur].dwFlags & WHDR_DONE) {
        bank.push(bufs[cur].data(), hs[cur].dwBytesRecorded / 2, deliver);
        hs[cur].dwFlags &= ~WHDR_DONE;
        waveInAddBuffer(wi, &hs[cur], sizeof(WAVEHDR));
        cur = (cur + 1) % NB;
      } else {
        Sleep(5);
      }
    }
    waveInStop(wi); waveInReset(wi);
    for (int i = 0; i < NB; i++) waveInUnprepareHeader(wi, &hs[i], sizeof(WAVEHDR));
    waveInClose(wi);
    return;
  }
#endif
  std::string cmd = cfg_.rx_cmd;
  if (cmd.empty()) {
    char b[160];
    std::snprintf(b, sizeof(b), "arecord -q -f S16_LE -c 1 -r %d -t raw 2>/dev/null", cfg_.fs);
    cmd = b;
  }
  FILE* p = SL_POPEN(cmd.c_str(), "r");
  if (!p) { logf("%s[FM] RX: cannot start capture command%s", SL_RED, SL_RST); return; }
  int16_t buf[2048];
  while (run_) {
    size_t n = std::fread(buf, 2, 2048, p);
    if (n == 0) break;
    bank.push(buf, n, deliver);
  }
  SL_PCLOSE(p);
}

}  // namespace sl
