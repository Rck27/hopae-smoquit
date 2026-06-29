#include "bmp280.h"

// ── Helper functions (static → hanya terlihat di file ini) ────
static bool bmp280_read(BMP280_t *dev, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(dev->addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)dev->addr, (uint8_t)len);
  for (size_t i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

static bool bmp280_write(BMP280_t *dev, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(dev->addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool bmp280_probe_address(BMP280_t *dev) {
  if (!bmp280_read(dev, BMP280_REG_CHPID, &dev->chip_id, 1)) return false;
  return (dev->chip_id == BMP280_ID0 ||
          dev->chip_id == BMP280_ID1 ||
          dev->chip_id == BMP280_ID2);
}

static bool bmp280_calibrate(BMP280_t *dev) {
  uint8_t buf[24];
  if (!bmp280_read(dev, BMP280_REG_CAL_LO, buf, sizeof(buf))) return false;

  dev->cmps.T1 = (uint16_t)(buf[0] | (buf[1] << 8));
  dev->cmps.T2 = (int16_t)(buf[2]  | (buf[3] << 8));
  dev->cmps.T3 = (int16_t)(buf[4]  | (buf[5] << 8));
  dev->cmps.P1 = (uint16_t)(buf[6] | (buf[7] << 8));
  dev->cmps.P2 = (int16_t)(buf[8]  | (buf[9] << 8));
  dev->cmps.P3 = (int16_t)(buf[10] | (buf[11] << 8));
  dev->cmps.P4 = (int16_t)(buf[12] | (buf[13] << 8));
  dev->cmps.P5 = (int16_t)(buf[14] | (buf[15] << 8));
  dev->cmps.P6 = (int16_t)(buf[16] | (buf[17] << 8));
  dev->cmps.P7 = (int16_t)(buf[18] | (buf[19] << 8));
  dev->cmps.P8 = (int16_t)(buf[20] | (buf[21] << 8));
  dev->cmps.P9 = (int16_t)(buf[22] | (buf[23] << 8));
  return true;
}

static int32_t compensate_T(BMP280_t *dev, int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->cmps.T1 << 1))) *
                  ((int32_t)dev->cmps.T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - (int32_t)dev->cmps.T1) *
                    ((adc_T >> 4) - (int32_t)dev->cmps.T1)) >> 12) *
                  (int32_t)dev->cmps.T3) >> 14;
  dev->t_fine = var1 + var2;
  return (dev->t_fine * 5 + 128) >> 8; // 0.01 °C
}

static uint32_t compensate_P(BMP280_t *dev, int32_t adc_P) {
  int64_t var1 = (int64_t)dev->t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dev->cmps.P6;
  var2 += (var1 * (int64_t)dev->cmps.P5) << 17;
  var2 += ((int64_t)dev->cmps.P4) << 35;
  var1 = ((var1 * var1 * (int64_t)dev->cmps.P3) >> 8) +
         ((var1 * (int64_t)dev->cmps.P2) << 12);
  var1 = ((((int64_t)1 << 47) + var1) * (int64_t)dev->cmps.P1) >> 33;
  if (var1 == 0) return 0;

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)dev->cmps.P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)dev->cmps.P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + ((int64_t)dev->cmps.P7 << 4);
  return (uint32_t)p; // Pa Q24.8 (bagi 256 untuk Pascal)
}

// ── Public API ─────────────────────────────────────────────────
bool bmp280_init(BMP280_t *dev, int address) {
  dev->addr = address;
  if (!bmp280_probe_address(dev)) return false;

  if (!bmp280_write(dev, BMP280_REG_RESET, BMP280_RESET_VEC)) return false;
  delay(10);

  return bmp280_calibrate(dev);
}

bool bmp280_configure(BMP280_t *dev,
                      bmp280_oversampling_t t_os,
                      bmp280_oversampling_t p_os,
                      bmp280_filter_t filter) {
  uint8_t ctrl = (t_os << 5) | (p_os << 2) | BMP280_MODE_SLEEP;
  if (!bmp280_write(dev, BMP280_REG_MESCTL, ctrl)) return false;

  uint8_t cfg = (filter << 2);
  return bmp280_write(dev, BMP280_REG_CONFIG, cfg);
}

bool bmp280_set_mode(BMP280_t *dev, bmp280_mode_t mode) {
  uint8_t ctrl;
  if (!bmp280_read(dev, BMP280_REG_MESCTL, &ctrl, 1)) return false;
  ctrl = (ctrl & ~0x03) | (uint8_t)mode;
  return bmp280_write(dev, BMP280_REG_MESCTL, ctrl);
}

bool bmp280_is_sampling(BMP280_t *dev) {
  uint8_t status;
  if (!bmp280_read(dev, BMP280_REG_STATUS, &status, 1)) return false;
  return (status & (1 << 3)) != 0;
}

bool bmp280_readout(BMP280_t *dev, float *temperature_C, float *pressure_Pa) {
  uint8_t buf[3];

  // Baca suhu dulu (agar t_fine terisi)
  if (!bmp280_read(dev, BMP280_REG_TEMP_MSB, buf, 3)) return false;
  int32_t adc_T = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
  int32_t T_raw = compensate_T(dev, adc_T);
  if (temperature_C) *temperature_C = T_raw / 100.0f;

  if (pressure_Pa) {
    if (!bmp280_read(dev, BMP280_REG_PRES_MSB, buf, 3)) return false;
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    uint32_t P_raw = compensate_P(dev, adc_P);
    *pressure_Pa = P_raw / 256.0f;
  }
  return true;
}