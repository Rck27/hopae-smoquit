#include "MICS5524.h"

MICS5524::MICS5524(uint8_t pin, float vcc, float rl_ohm,
                   uint32_t warmup_ms, uint16_t cal_samples, uint32_t cal_interval)
  : _pin(pin), _vcc(vcc), _rl(rl_ohm),
    _warmup_ms(warmup_ms),
    _cal_samples(cal_samples), _cal_interval(cal_interval) {}

void MICS5524::begin() {
  analogReadResolution(12);       // 0-4095
  _state   = State::WARMING_UP;
  _startMs = millis();
  _r0      = -1.0f;
  _calSum  = 0.0;
  _calCount = 0;
  _lastCalMs = 0;
}

bool MICS5524::update() {
  bool changed = false;
  switch (_state) {
    case State::WARMING_UP:
      if (millis() - _startMs >= _warmup_ms) {
        _state = State::CALIBRATING;
        _lastCalMs = millis();
        changed = true;
      }
      break;

    case State::CALIBRATING:
      if (millis() - _lastCalMs >= _cal_interval) {
        _lastCalMs = millis();
        _calSum += _readRs();
        _calCount++;
        if (_calCount >= _cal_samples) {
          _r0 = static_cast<float>(_calSum / _calCount);
          _state = State::READY;
          changed = true;
        }
      }
      break;

    case State::READY:
      break;
  }
  return changed;
}

MICS5524::State MICS5524::state() const {
  return _state;
}

float MICS5524::calProgress() const {
  return (_cal_samples > 0) ? (float)_calCount / _cal_samples : 0.0f;
}

float MICS5524::r0() const {
  return _r0;
}

void MICS5524::setR0(float r0) {
  _r0 = r0;
  _state = State::READY;
}

float MICS5524::ratio() const {
  if (_state != State::READY || _r0 <= 0.0f) return -1.0f;
  return _readRs() / _r0;
}

float MICS5524::ppm() const {
  float r = ratio();
  if (r < 0.0f) return -1.0f;
  return 4.4638f * powf(r, -1.1487f);  // kurva datasheet CO
}

float MICS5524::rs() const {
  return _readRs();
}

uint32_t MICS5524::warmupRemainSec() const {
  if (_state != State::WARMING_UP) return 0;
  uint32_t elapsed = millis() - _startMs;
  return (elapsed < _warmup_ms) ? (_warmup_ms - elapsed) / 1000 : 0;
}

float MICS5524::_readRs() const {
  int raw = analogRead(_pin);
  if (raw >= 4095) raw = 4094;       // hindari pembagian nol
  float vout = (raw / 4095.0f) * _vcc;
  if (vout < 0.001f) return 1e6f;
  return _rl * (_vcc - vout) / vout;
}