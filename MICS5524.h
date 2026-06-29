#pragma once
#include <Arduino.h>

class MICS5524 {
public:
  enum class State { WARMING_UP, CALIBRATING, READY };

  MICS5524(uint8_t pin,
           float vcc = 3.3f,
           float rl_ohm = 10000.0f,
           uint32_t warmup_ms = 30000UL,
           uint16_t cal_samples = 200,
           uint32_t cal_interval = 100UL);

  void begin();
  bool update();                 // panggil tiap loop, return true jika state berubah

  // ── Accessors ──────────────────────────────────────────────
  State state() const;
  float calProgress() const;     // 0.0 - 1.0
  float r0() const;
  void  setR0(float r0);        // langsung masuk ke READY
  float ratio() const;          // Rs/R0, -1 kalau belum siap
  float ppm() const;
  float rs() const;
  uint32_t warmupRemainSec() const;

private:
  uint8_t  _pin;
  float    _vcc, _rl;
  uint32_t _warmup_ms, _startMs, _lastCalMs;
  uint16_t _cal_samples;
  uint32_t _cal_interval;
  State    _state;

  float    _r0;
  double   _calSum;
  uint16_t _calCount;

  float _readRs() const;
};