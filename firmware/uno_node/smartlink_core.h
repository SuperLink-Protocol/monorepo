/*
 * smartlink_core.h — SMART-LINK shared protocol core (single source of truth)
 * ===========================================================================
 * Header-only, dependency-free C++ used by EVERY node:
 *   - HP ProBook / HP Victus / Raspberry Pi  (engine/, MSVC / g++)
 *   - ESP32                                    (firmware/esp32_node, Arduino)
 *   - Arduino Uno (ATmega328P, 2 KB SRAM)      (firmware/uno_node, Arduino)
 *
 * Contents
 *   1. SHA-256 (streaming, 16-word schedule -> low RAM), HMAC-SHA256, HKDF-SHA256 (RFC 5869)
 *   2. Key schedule: K_i = HKDF(master, "SMART-LINK|SL26", "relay-<id>")
 *   3. SL1 binary frame codec + CRC16-CCITT
 *   4. Chained hop-audit tag: tau_i = trunc16(HMAC_Ki(be32|tau_{i-1}| || tau_{i-1} || be32|R_i| || R_i))
 *   5. Endpoint payload cipher (SHA-256 CTR keystream)
 *   6. "SL1 <hex>\n" line codec (UART / USB serial / UDP)
 *   7. FM audio-subcarrier FSK modem parameters + streaming demodulator
 *
 * DO NOT fork this file per device. Edit here; `smartlink.ps1 sync` copies it
 * into each Arduino sketch folder.
 */
#ifndef SMARTLINK_CORE_H
#define SMARTLINK_CORE_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #define SL_PROGMEM PROGMEM
  #define SL_RD32(p) pgm_read_dword(p)
  #ifndef SL_MAX_FRAME
    #define SL_MAX_FRAME 120
  #endif
#else
  #define SL_PROGMEM
  #define SL_RD32(p) (*(p))
  #ifndef SL_MAX_FRAME
    #define SL_MAX_FRAME 255
  #endif
#endif

/* ------------------------------------------------------------------ */
/* Protocol constants                                                  */
/* ------------------------------------------------------------------ */
#define SL_SESSION       "SL26"
#define SL_MAGIC         0x53   /* 'S' */
#define SL_VERSION       0x01
#define SL_T_HB          1      /* heartbeat / discovery (flooded, TTL) */
#define SL_T_MSG         2      /* encrypted user message (source routed) */
#define SL_T_ACK         3      /* delivery receipt (source routed back) */
#define SL_BROADCAST     0xFF
#define SL_NO_CARRIER    0xFF
#define SL_TAG_LEN       16
#define SL_HB_TTL        3
#define SL_MAX_HOPS      8
#define SL_MIN_FRAME     30
#define SL_CODE_SUCCESS  400    /* project convention: "SUCCESS 400" = delivered + verified */
#define SL_CODE_TAMPER   401    /* hop-audit chain mismatch at destination */
#define SL_NUM_CARRIERS  6

/* Capability bits advertised in HB */
#define SL_CAP_ENGINE    0x01   /* full inference engine (laptop / Pi) */
#define SL_CAP_FM_TX     0x02
#define SL_CAP_FM_RX     0x04
#define SL_CAP_WIFI      0x08
#define SL_CAP_UART      0x10

static const uint16_t SL_CARRIERS_KHZ[SL_NUM_CARRIERS] = {75, 84, 95, 108, 120, 135};

/* Testbench master key — CHANGE FOR ANY REAL DEPLOYMENT (must match on all nodes) */
static const uint8_t SL_MASTER_KEY[32] = {
  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f
};

/* ------------------------------------------------------------------ */
/* 1. SHA-256 / HMAC / HKDF                                            */
/* ------------------------------------------------------------------ */
static const uint32_t SL_K256[64] SL_PROGMEM = {
  0x428a2f98UL,0x71374491UL,0xb5c0fbcfUL,0xe9b5dba5UL,0x3956c25bUL,0x59f111f1UL,0x923f82a4UL,0xab1c5ed5UL,
  0xd807aa98UL,0x12835b01UL,0x243185beUL,0x550c7dc3UL,0x72be5d74UL,0x80deb1feUL,0x9bdc06a7UL,0xc19bf174UL,
  0xe49b69c1UL,0xefbe4786UL,0x0fc19dc6UL,0x240ca1ccUL,0x2de92c6fUL,0x4a7484aaUL,0x5cb0a9dcUL,0x76f988daUL,
  0x983e5152UL,0xa831c66dUL,0xb00327c8UL,0xbf597fc7UL,0xc6e00bf3UL,0xd5a79147UL,0x06ca6351UL,0x14292967UL,
  0x27b70a85UL,0x2e1b2138UL,0x4d2c6dfcUL,0x53380d13UL,0x650a7354UL,0x766a0abbUL,0x81c2c92eUL,0x92722c85UL,
  0xa2bfe8a1UL,0xa81a664bUL,0xc24b8b70UL,0xc76c51a3UL,0xd192e819UL,0xd6990624UL,0xf40e3585UL,0x106aa070UL,
  0x19a4c116UL,0x1e376c08UL,0x2748774cUL,0x34b0bcb5UL,0x391c0cb3UL,0x4ed8aa4aUL,0x5b9cca4fUL,0x682e6ff3UL,
  0x748f82eeUL,0x78a5636fUL,0x84c87814UL,0x8cc70208UL,0x90befffaUL,0xa4506cebUL,0xbef9a3f7UL,0xc67178f2UL
};

typedef struct { uint32_t s[8]; uint8_t buf[64]; uint8_t n; uint32_t bits_lo, bits_hi; } SlSha256;

static inline uint32_t sl_rotr(uint32_t x, uint8_t n) { return (x >> n) | (x << (32 - n)); }

static inline void sl_sha_block(SlSha256* c, const uint8_t* d) {
  uint32_t w[16];
  for (uint8_t i = 0; i < 16; i++)
    w[i] = ((uint32_t)d[4*i] << 24) | ((uint32_t)d[4*i+1] << 16) | ((uint32_t)d[4*i+2] << 8) | d[4*i+3];
  uint32_t a=c->s[0], b=c->s[1], cc=c->s[2], dd=c->s[3], e=c->s[4], f=c->s[5], g=c->s[6], h=c->s[7];
  for (uint8_t i = 0; i < 64; i++) {
    if (i >= 16) {
      uint32_t x1 = w[(i+1) & 15], x14 = w[(i+14) & 15];
      uint32_t s0 = sl_rotr(x1, 7) ^ sl_rotr(x1, 18) ^ (x1 >> 3);
      uint32_t s1 = sl_rotr(x14, 17) ^ sl_rotr(x14, 19) ^ (x14 >> 10);
      w[i & 15] += s0 + w[(i+9) & 15] + s1;
    }
    uint32_t S1 = sl_rotr(e, 6) ^ sl_rotr(e, 11) ^ sl_rotr(e, 25);
    uint32_t ch = (e & f) ^ ((~e) & g);
    uint32_t t1 = h + S1 + ch + SL_RD32(&SL_K256[i]) + w[i & 15];
    uint32_t S0 = sl_rotr(a, 2) ^ sl_rotr(a, 13) ^ sl_rotr(a, 22);
    uint32_t mj = (a & b) ^ (a & cc) ^ (b & cc);
    uint32_t t2 = S0 + mj;
    h = g; g = f; f = e; e = dd + t1; dd = cc; cc = b; b = a; a = t1 + t2;
  }
  c->s[0]+=a; c->s[1]+=b; c->s[2]+=cc; c->s[3]+=dd; c->s[4]+=e; c->s[5]+=f; c->s[6]+=g; c->s[7]+=h;
}

static inline void sl_sha_init(SlSha256* c) {
  static const uint32_t iv[8] = {0x6a09e667UL,0xbb67ae85UL,0x3c6ef372UL,0xa54ff53aUL,
                                 0x510e527fUL,0x9b05688cUL,0x1f83d9abUL,0x5be0cd19UL};
  memcpy(c->s, iv, sizeof(iv)); c->n = 0; c->bits_lo = 0; c->bits_hi = 0;
}

static inline void sl_sha_update(SlSha256* c, const uint8_t* d, size_t len) {
  for (size_t i = 0; i < len; i++) {
    c->buf[c->n++] = d[i];
    uint32_t old = c->bits_lo; c->bits_lo += 8; if (c->bits_lo < old) c->bits_hi++;
    if (c->n == 64) { sl_sha_block(c, c->buf); c->n = 0; }
  }
}

static inline void sl_sha_final(SlSha256* c, uint8_t out[32]) {
  uint32_t lo = c->bits_lo, hi = c->bits_hi;
  uint8_t pad = 0x80; sl_sha_update(c, &pad, 1);
  uint8_t z = 0; while (c->n != 56) sl_sha_update(c, &z, 1);
  uint8_t L[8] = {(uint8_t)(hi>>24),(uint8_t)(hi>>16),(uint8_t)(hi>>8),(uint8_t)hi,
                  (uint8_t)(lo>>24),(uint8_t)(lo>>16),(uint8_t)(lo>>8),(uint8_t)lo};
  sl_sha_update(c, L, 8);
  for (uint8_t i = 0; i < 8; i++) {
    out[4*i] = (uint8_t)(c->s[i] >> 24); out[4*i+1] = (uint8_t)(c->s[i] >> 16);
    out[4*i+2] = (uint8_t)(c->s[i] >> 8); out[4*i+3] = (uint8_t)c->s[i];
  }
}

static inline void sl_sha256(const uint8_t* d, size_t len, uint8_t out[32]) {
  SlSha256 c; sl_sha_init(&c); sl_sha_update(&c, d, len); sl_sha_final(&c, out);
}

typedef struct { SlSha256 inner; uint8_t okey[64]; } SlHmac;

static inline void sl_hmac_init(SlHmac* h, const uint8_t* key, size_t klen) {
  uint8_t k[64]; memset(k, 0, 64);
  if (klen > 64) sl_sha256(key, klen, k); else memcpy(k, key, klen);
  uint8_t ipad[64];
  for (uint8_t i = 0; i < 64; i++) { ipad[i] = k[i] ^ 0x36; h->okey[i] = k[i] ^ 0x5c; }
  sl_sha_init(&h->inner); sl_sha_update(&h->inner, ipad, 64);
}
static inline void sl_hmac_update(SlHmac* h, const uint8_t* d, size_t len) { sl_sha_update(&h->inner, d, len); }
static inline void sl_hmac_final(SlHmac* h, uint8_t out[32]) {
  uint8_t ih[32]; sl_sha_final(&h->inner, ih);
  SlSha256 o; sl_sha_init(&o); sl_sha_update(&o, h->okey, 64); sl_sha_update(&o, ih, 32); sl_sha_final(&o, out);
}
static inline void sl_hmac(const uint8_t* key, size_t klen, const uint8_t* msg, size_t mlen, uint8_t out[32]) {
  SlHmac h; sl_hmac_init(&h, key, klen); sl_hmac_update(&h, msg, mlen); sl_hmac_final(&h, out);
}

/* RFC 5869 HKDF-SHA256, first 32 output bytes (T(1)) */
static inline void sl_hkdf32(const uint8_t* ikm, size_t ikm_len, const uint8_t* salt, size_t salt_len,
                             const uint8_t* info, size_t info_len, uint8_t out[32]) {
  uint8_t prk[32]; sl_hmac(salt, salt_len, ikm, ikm_len, prk);
  SlHmac h; sl_hmac_init(&h, prk, 32); sl_hmac_update(&h, info, info_len);
  uint8_t one = 0x01; sl_hmac_update(&h, &one, 1); sl_hmac_final(&h, out);
}

/* ------------------------------------------------------------------ */
/* 2. Key schedule                                                     */
/* ------------------------------------------------------------------ */
static inline void sl_node_key(uint8_t node_id, uint8_t out[32]) {
  const char* salt = "SMART-LINK|" SL_SESSION;
  char info[12]; int n = snprintf(info, sizeof(info), "relay-%u", (unsigned)node_id);
  sl_hkdf32(SL_MASTER_KEY, 32, (const uint8_t*)salt, strlen(salt), (const uint8_t*)info, (size_t)n, out);
}
static inline void sl_enc_key(uint8_t out[32]) {
  const char* salt = "SMART-LINK|" SL_SESSION; const char* info = "payload-enc";
  sl_hkdf32(SL_MASTER_KEY, 32, (const uint8_t*)salt, strlen(salt), (const uint8_t*)info, strlen(info), out);
}

/* ------------------------------------------------------------------ */
/* 3. Frame codec                                                      */
/*  [0]'S' [1]ver [2]type [3]flags(TTL) [4..5]seq [6]src [7]dst [8]tx  */
/*  [9]R  R*(id,cidx) route      (cidx = carrier of link id->next)     */
/*  [.]H  H*(id,cidx) hop records (appended by each transmitter)       */
/*  [.]L  L payload   [.]16 tag   [.]2 CRC16-CCITT(all previous)       */
/* ------------------------------------------------------------------ */
typedef struct {
  uint8_t type, flags, src, dst, tx, R, H, L;
  uint16_t seq, off_route, off_hops, off_payload, off_tag, len;
} SlView;

static inline uint16_t sl_crc16(const uint8_t* d, uint16_t n) {
  uint16_t c = 0xFFFF;
  for (uint16_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}
static inline void sl_fix_crc(uint8_t* f, uint16_t len) {
  uint16_t c = sl_crc16(f, len - 2); f[len-2] = (uint8_t)(c >> 8); f[len-1] = (uint8_t)c;
}

static inline int sl_parse(const uint8_t* f, uint16_t len, SlView* v) {
  if (len < SL_MIN_FRAME || len > SL_MAX_FRAME) return 0;
  if (f[0] != SL_MAGIC || f[1] != SL_VERSION) return 0;
  uint16_t c = sl_crc16(f, len - 2);
  if (f[len-2] != (uint8_t)(c >> 8) || f[len-1] != (uint8_t)c) return 0;
  v->type = f[2]; v->flags = f[3]; v->seq = (uint16_t)((f[4] << 8) | f[5]);
  v->src = f[6]; v->dst = f[7]; v->tx = f[8];
  uint16_t p = 9;
  v->R = f[p++]; v->off_route = p; p += 2 * v->R; if (p >= len) return 0;
  v->H = f[p++]; v->off_hops = p;  p += 2 * v->H; if (p >= len) return 0;
  v->L = f[p++]; v->off_payload = p; p += v->L;
  v->off_tag = p; p += SL_TAG_LEN;
  if ((uint16_t)(p + 2) != len) return 0;
  if (v->R > SL_MAX_HOPS + 1 || v->H > SL_MAX_HOPS) return 0;
  v->len = len;
  return 1;
}

/* Build a frame with H=0 (call sl_append_hop afterwards to sign hop 0). Returns length or 0. */
static inline uint16_t sl_build(uint8_t* f, uint16_t cap, uint8_t type, uint8_t flags, uint16_t seq,
                                uint8_t src, uint8_t dst, const uint8_t* route, uint8_t R,
                                const uint8_t* payload, uint8_t L) {
  uint16_t need = (uint16_t)(SL_MIN_FRAME + 2 * R + L);
  if (need > cap || need > SL_MAX_FRAME) return 0;
  uint16_t p = 0;
  f[p++] = SL_MAGIC; f[p++] = SL_VERSION; f[p++] = type; f[p++] = flags;
  f[p++] = (uint8_t)(seq >> 8); f[p++] = (uint8_t)seq; f[p++] = src; f[p++] = dst; f[p++] = src;
  f[p++] = R; if (R) { memcpy(f + p, route, 2 * R); p += 2 * R; }
  f[p++] = 0;
  f[p++] = L; if (L) { memcpy(f + p, payload, L); p += L; }
  memset(f + p, 0, SL_TAG_LEN); p += SL_TAG_LEN;
  p += 2; sl_fix_crc(f, p);
  return p;
}

static inline char sl_hexc(uint8_t n) { return (char)(n < 10 ? '0' + n : 'a' + n - 10); }
static inline int sl_hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* ct prefix = hex(SHA-256(payload)[0..8]) — binds the payload into every hop record */
static inline void sl_ct_prefix(const uint8_t* payload, uint8_t L, char out[17]) {
  uint8_t d[32]; sl_sha256(payload, L, d);
  for (uint8_t i = 0; i < 8; i++) { out[2*i] = sl_hexc(d[i] >> 4); out[2*i+1] = sl_hexc(d[i] & 15); }
  out[16] = 0;
}

/* ------------------------------------------------------------------ */
/* 4. Hop-audit chain                                                  */
/* R_i = "SL26|type|src|dst|seq|node|hop|carrier_hz|ctprefix"          */
/* ------------------------------------------------------------------ */
static inline void sl_chain_step(uint8_t type, uint8_t src, uint8_t dst, uint16_t seq, const char* ctp,
                                 const uint8_t* prev /* NULL => INIT */, uint8_t node, uint8_t hop,
                                 uint8_t cidx, uint8_t out[SL_TAG_LEN]) {
  uint8_t key[32]; sl_node_key(node, key);
  unsigned long chz = (cidx < SL_NUM_CARRIERS) ? (unsigned long)SL_CARRIERS_KHZ[cidx] * 1000UL : 0UL;
  char body[80];
  int bl = snprintf(body, sizeof(body), "%s|%u|%u|%u|%u|%u|%u|%lu|%s", SL_SESSION,
                    (unsigned)type, (unsigned)src, (unsigned)dst, (unsigned)seq,
                    (unsigned)node, (unsigned)hop, chz, ctp);
  static const char init[] = "INIT:" SL_SESSION;
  const uint8_t* p = prev ? prev : (const uint8_t*)init;
  uint32_t pl = prev ? SL_TAG_LEN : (uint32_t)(sizeof(init) - 1);
  uint8_t be[4];
  SlHmac h; sl_hmac_init(&h, key, 32);
  be[0]=(uint8_t)(pl>>24); be[1]=(uint8_t)(pl>>16); be[2]=(uint8_t)(pl>>8); be[3]=(uint8_t)pl;
  sl_hmac_update(&h, be, 4); sl_hmac_update(&h, p, pl);
  be[0]=0; be[1]=0; be[2]=(uint8_t)(bl>>8); be[3]=(uint8_t)bl;
  sl_hmac_update(&h, be, 4); sl_hmac_update(&h, (const uint8_t*)body, (size_t)bl);
  uint8_t full[32]; sl_hmac_final(&h, full);
  memcpy(out, full, SL_TAG_LEN);
}

/* Append hop record (node, cidx), re-sign tag, fix CRC. Returns new length or 0. */
static inline uint16_t sl_append_hop(uint8_t* f, uint16_t len, uint16_t cap, uint8_t node, uint8_t cidx) {
  SlView v; if (!sl_parse(f, len, &v)) return 0;
  if ((uint16_t)(len + 2) > cap || (uint16_t)(len + 2) > SL_MAX_FRAME || v.H >= SL_MAX_HOPS) return 0;
  char ctp[17]; sl_ct_prefix(f + v.off_payload, v.L, ctp);
  uint8_t tag[SL_TAG_LEN];
  sl_chain_step(v.type, v.src, v.dst, v.seq, ctp, v.H ? f + v.off_tag : (const uint8_t*)0, node, v.H, cidx, tag);
  uint16_t ins = (uint16_t)(v.off_hops + 2 * v.H);
  memmove(f + ins + 2, f + ins, len - ins);
  f[ins] = node; f[ins+1] = cidx; f[v.off_hops - 1] = (uint8_t)(v.H + 1);
  len += 2;
  memcpy(f + v.off_tag + 2, tag, SL_TAG_LEN);
  sl_fix_crc(f, len);
  return len;
}

/* Push one entry onto the route list (open-route extension). Returns new length or 0. */
static inline uint16_t sl_route_push(uint8_t* f, uint16_t len, uint16_t cap, uint8_t id, uint8_t cidx) {
  SlView v; if (!sl_parse(f, len, &v)) return 0;
  if ((uint16_t)(len + 2) > cap || (uint16_t)(len + 2) > SL_MAX_FRAME || v.R > SL_MAX_HOPS) return 0;
  uint16_t ins = (uint16_t)(v.off_route + 2 * v.R);
  memmove(f + ins + 2, f + ins, len - ins);
  f[ins] = id; f[ins+1] = cidx; f[9] = (uint8_t)(v.R + 1);
  len += 2; sl_fix_crc(f, len);
  return len;
}

static inline void sl_set_tx(uint8_t* f, uint16_t len, uint8_t tx) { f[8] = tx; sl_fix_crc(f, len); }
static inline void sl_set_flags(uint8_t* f, uint16_t len, uint8_t fl) { f[3] = fl; sl_fix_crc(f, len); }

/* Recompute the full chain over all hop records and compare (constant time). */
static inline int sl_verify(const uint8_t* f, const SlView* v) {
  if (v->H == 0) return 0;
  char ctp[17]; sl_ct_prefix(f + v->off_payload, v->L, ctp);
  uint8_t prev[SL_TAG_LEN], cur[SL_TAG_LEN];
  for (uint8_t h = 0; h < v->H; h++) {
    sl_chain_step(v->type, v->src, v->dst, v->seq, ctp, h ? prev : (const uint8_t*)0,
                  f[v->off_hops + 2*h], h, f[v->off_hops + 2*h + 1], cur);
    memcpy(prev, cur, SL_TAG_LEN);
  }
  uint8_t diff = 0;
  for (uint8_t i = 0; i < SL_TAG_LEN; i++) diff |= (uint8_t)(cur[i] ^ f[v->off_tag + i]);
  return diff == 0;
}

/* ------------------------------------------------------------------ */
/* 5. Endpoint payload cipher (keystream = SHA256(K_enc||src||0||seq||ctr)) */
/* ------------------------------------------------------------------ */
static inline void sl_crypt(uint8_t src, uint16_t seq, uint8_t* data, uint8_t L) {
  uint8_t k[32]; sl_enc_key(k);
  uint8_t blk[32];
  for (uint8_t i = 0; i < L; i++) {
    if ((i & 31) == 0) {
      uint8_t hdr[8] = {src, 0, (uint8_t)(seq >> 8), (uint8_t)seq, 0, 0, 0, (uint8_t)(i >> 5)};
      SlSha256 c; sl_sha_init(&c); sl_sha_update(&c, k, 32); sl_sha_update(&c, hdr, 8); sl_sha_final(&c, blk);
    }
    data[i] ^= blk[i & 31];
  }
}

/* ------------------------------------------------------------------ */
/* 6. Line codec: "SL1 <hex>\n" (incremental, no line buffer; MCU-friendly) */
/* ------------------------------------------------------------------ */
typedef struct { uint8_t buf[SL_MAX_FRAME]; uint16_t len; uint8_t state, nib, have; } SlLineDec;

static inline void sl_dec_init(SlLineDec* d) { d->len = 0; d->state = 0; d->nib = 0; d->have = 0; }

/* Feed one char. Returns frame length (>0) when a complete "SL1 ..." line ends, else 0. */
static inline uint16_t sl_dec_feed(SlLineDec* d, char c) {
  if (c == '\r') return 0;
  if (c == '\n') {
    uint16_t r = (d->state == 4 && !d->have) ? d->len : 0;
    d->state = 0; d->have = 0;
    return r;
  }
  if (d->state < 4) {
    char exp = (d->state == 0) ? 'S' : (d->state == 1) ? 'L' : (d->state == 2) ? '1' : ' ';
    if (c == exp) { if (d->state == 0) d->len = 0; d->state++; }
    else d->state = 5;
    return 0;
  }
  if (d->state == 5) return 0;
  int v = sl_hexv(c);
  if (v < 0) { d->state = 5; return 0; }
  if (!d->have) { d->nib = (uint8_t)v; d->have = 1; }
  else {
    if (d->len >= SL_MAX_FRAME) { d->state = 5; return 0; }
    d->buf[d->len++] = (uint8_t)((d->nib << 4) | v); d->have = 0;
  }
  return 0;
}

/* ------------------------------------------------------------------ */
/* 7. FM audio-subcarrier FSK modem                                    */
/* Each SMART-LINK carrier f (kHz) maps to an FM audio subcarrier of   */
/* f*100 Hz (75 kHz -> 7.5 kHz ... 135 kHz -> 13.5 kHz), all inside the */
/* 15 kHz FM broadcast audio band. mark = sub+150 Hz, space = sub-150. */
/* 300 baud, async bytes: start(0) + 8 data LSB-first + stop(1).       */
/* Air frame: 0.2 s mark preamble, 0x7E, LEN, frame bytes, 10 mark bits */
/* ------------------------------------------------------------------ */
#define SL_FM_BAUD          300
#define SL_FM_DEV_HZ        150
#define SL_FM_PREAMBLE_BITS 60
#define SL_FM_TRAILER_BITS  10
#define SL_FM_SYNC          0x7E

static inline uint32_t sl_fm_sub_hz(uint8_t cidx) {
  return (uint32_t)SL_CARRIERS_KHZ[cidx < SL_NUM_CARRIERS ? cidx : 0] * 100UL;
}
static inline uint32_t sl_fm_tone_hz(uint8_t cidx, int bit) {
  return bit ? sl_fm_sub_hz(cidx) + SL_FM_DEV_HZ : sl_fm_sub_hz(cidx) - SL_FM_DEV_HZ;
}
/* Total bits on air for a frame of `len` bytes */
static inline uint32_t sl_fm_air_bits(uint16_t len) {
  return SL_FM_PREAMBLE_BITS + (uint32_t)(len + 2) * 10 + SL_FM_TRAILER_BITS;
}
/* Bit k of the air sequence (0..sl_fm_air_bits-1) */
static inline int sl_fm_air_bit(const uint8_t* f, uint16_t len, uint32_t k) {
  if (k < SL_FM_PREAMBLE_BITS) return 1;
  k -= SL_FM_PREAMBLE_BITS;
  uint32_t byte_i = k / 10, bit_i = k % 10;
  if (byte_i >= (uint32_t)len + 2) return 1;
  uint8_t b = byte_i == 0 ? SL_FM_SYNC : byte_i == 1 ? (uint8_t)len : f[byte_i - 2];
  if (bit_i == 0) return 0;
  if (bit_i == 9) return 1;
  return (b >> (bit_i - 1)) & 1;
}

#ifndef SL_NO_FM_RX
#ifndef SL_FM_MAX_N
  #define SL_FM_MAX_N 160   /* samples per bit max (48 kHz / 300 baud) */
#endif
#ifndef SL_FM_MIN_POW
  #define SL_FM_MIN_POW 1e-5f
#endif

typedef struct { float zr, zi, wr, wi, sr, si; float rr[SL_FM_MAX_N], ri[SL_FM_MAX_N]; } SlTone;

typedef struct {
  SlTone mark, space;
  float re[SL_FM_MAX_N], se;
  uint16_t N, idx;
  uint8_t cidx;
  /* UART */
  uint8_t ust, ubyte; int32_t ucnt, idle;
  /* framer */
  uint8_t fst, flen; uint16_t fpos; int32_t gap;
  uint8_t fbuf[SL_MAX_FRAME];
} SlFmChan;

static inline void sl_tone_init(SlTone* t, float f, float fs) {
  memset(t, 0, sizeof(*t));
  t->zr = 1.0f; t->zi = 0.0f;
  t->wr = cosf(6.28318530718f * f / fs); t->wi = sinf(6.28318530718f * f / fs);
}

static inline void sl_fm_chan_init(SlFmChan* c, uint8_t cidx, float fs) {
  memset(c, 0, sizeof(*c));
  c->cidx = cidx;
  c->N = (uint16_t)(fs / SL_FM_BAUD + 0.5f);
  if (c->N > SL_FM_MAX_N) c->N = SL_FM_MAX_N;
  sl_tone_init(&c->mark, (float)sl_fm_tone_hz(cidx, 1), fs);
  sl_tone_init(&c->space, (float)sl_fm_tone_hz(cidx, 0), fs);
}

static inline void sl_tone_push(SlTone* t, float x, uint16_t idx) {
  float pr = x * t->zr, pi = -x * t->zi;
  t->sr += pr - t->rr[idx]; t->rr[idx] = pr;
  t->si += pi - t->ri[idx]; t->ri[idx] = pi;
  float nr = t->zr * t->wr - t->zi * t->wi;
  float ni = t->zr * t->wi + t->zi * t->wr;
  t->zr = nr; t->zi = ni;
}
static inline void sl_tone_resync(SlTone* t, uint16_t N) {
  float a = 0, b = 0;
  for (uint16_t i = 0; i < N; i++) { a += t->rr[i]; b += t->ri[i]; }
  t->sr = a; t->si = b;
  float m = sqrtf(t->zr * t->zr + t->zi * t->zi);
  if (m > 0) { t->zr /= m; t->zi /= m; }
}

static inline uint16_t sl_fm_byte(SlFmChan* c, uint8_t b) {
  c->gap = 0;
  switch (c->fst) {
    case 0: if (b == SL_FM_SYNC) c->fst = 1; break;
    case 1:
      if (b >= SL_MIN_FRAME && b <= SL_MAX_FRAME) { c->flen = b; c->fpos = 0; c->fst = 2; }
      else c->fst = (b == SL_FM_SYNC) ? 1 : 0;
      break;
    case 2:
      c->fbuf[c->fpos++] = b;
      if (c->fpos >= c->flen) { c->fst = 0; return c->flen; }
      break;
  }
  return 0;
}

/* Push one normalized sample (-1..1). Returns frame length when c->fbuf holds a complete frame. */
static inline uint16_t sl_fm_push(SlFmChan* c, float x) {
  uint16_t N = c->N, idx = c->idx;
  sl_tone_push(&c->mark, x, idx);
  sl_tone_push(&c->space, x, idx);
  float e = x * x; c->se += e - c->re[idx]; c->re[idx] = e;
  if (++c->idx >= N) {
    c->idx = 0;
    sl_tone_resync(&c->mark, N); sl_tone_resync(&c->space, N);
    float s = 0; for (uint16_t i = 0; i < N; i++) s += c->re[i]; c->se = s;
  }
  float pm = c->mark.sr * c->mark.sr + c->mark.si * c->mark.si;
  float ps = c->space.sr * c->space.sr + c->space.si * c->space.si;
  int present = (c->se > N * SL_FM_MIN_POW) && ((pm + ps) > 0.25f * N * c->se);
  int d = present ? (pm > ps ? 1 : 0) : -1;

  if (c->fst) { if (++c->gap > 12 * (int32_t)N) { c->fst = 0; c->gap = 0; } }

  uint16_t out = 0;
  if (c->ust == 0) {
    if (d == 1) c->idle++;
    else if (d == 0 && c->idle >= (int32_t)(N / 2)) { c->ust = 1; c->ucnt = 0; c->ubyte = 0; c->idle = 0; }
    else c->idle = 0;
  } else {
    c->ucnt++;
    int32_t h = N / 2;
    if (c->ucnt == h) { if (d != 0) c->ust = 0; }
    else if (c->ucnt > h && ((c->ucnt - h) % N) == 0) {
      int32_t k = (c->ucnt - h) / N;           /* 1..8 data, 9 stop */
      if (k <= 8) { if (d == 1) c->ubyte |= (uint8_t)(1 << (k - 1)); else if (d < 0) c->ust = 0; }
      else {
        c->ust = 0;
        if (d == 1) { out = sl_fm_byte(c, c->ubyte); c->idle = h; }
        else { c->fst = 0; c->idle = 0; }
      }
    }
  }
  return out;
}
#endif /* SL_NO_FM_RX */

#endif /* SMARTLINK_CORE_H */
