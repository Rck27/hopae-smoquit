#pragma once
#include <Arduino.h>
#include <Wire.h>

// ── Register map ──────────────────────────────────────────────
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_XSB 0xFC
#define BMP280_REG_PRES_MSB 0xF7
#define BMP280_REG_PRES_LSB 0xF8
#define BMP280_REG_PRES_XSB 0xF9
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_MESCTL    0xF4
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_CHPID     0xD0
#define BMP280_REG_CAL_LO    0x88

#define BMP280_RESET_VEC     0xB6
#define BMP280_ID0           0x56
#define BMP280_ID1           0x57
#define BMP280_ID2           0x58

// ── Oversampling / filter / mode enum ─────────────────────────
typedef enum {
  BMP280_OVRS_SKIP = 0,
  BMP280_OVRS_1X   = 1,
  BMP280_OVRS_2X   = 2,
  BMP280_OVRS_4X   = 3,
  BMP280_OVRS_8X   = 4,
  BMP280_OVRS_16X  = 5,
} bmp280_oversampling_t;

typedef enum {
  BMP280_FILTER_OFF = 0,
  BMP280_FILTER_2   = 1,
  BMP280_FILTER_4   = 2,
  BMP280_FILTER_8   = 3,
  BMP280_FILTER_16  = 4,
} bmp280_filter_t;

typedef enum {
  BMP280_MODE_SLEEP  = 0,
  BMP280_MODE_FORCE  = 1,
  BMP280_MODE_NORMAL = 3,
} bmp280_mode_t;

// ── Struktur koefisien kalibrasi (dari datasheet) ─────────────
struct BMP280_Calib {
  uint16_t T1;
  int16_t  T2;
  int16_t  T3;
  uint16_t P1;
  int16_t  P2;
  int16_t  P3;
  int16_t  P4;
  int16_t  P5;
  int16_t  P6;
  int16_t  P7;
  int16_t  P8;
  int16_t  P9;
};

// ── Driver struct ─────────────────────────────────────────────
struct BMP280_t {
  uint8_t addr;
  uint8_t chip_id;
  int32_t t_fine;
  BMP280_Calib cmps;
};

// ── Public API (fungsi yang bisa dipanggil dari luar) ─────────
bool bmp280_init(BMP280_t *dev, int address);
bool bmp280_configure(BMP280_t *dev,
                      bmp280_oversampling_t t_os,
                      bmp280_oversampling_t p_os,
                      bmp280_filter_t filter);
bool bmp280_set_mode(BMP280_t *dev, bmp280_mode_t mode);
bool bmp280_is_sampling(BMP280_t *dev);
bool bmp280_readout(BMP280_t *dev, float *temperature_C, float *pressure_Pa);