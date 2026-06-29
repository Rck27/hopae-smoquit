// ═══════════════════════════════════════════════════════════════════════════════
//  HOPAE — Breath CO + Flow Monitor  |  ESP32-C3
//  Fixed: pin conflict, mics_task, BMP init logic, result mutex,
//         oled_task overwrite, sumA uninit, status_buf race, VenturiResult_t init
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Pressure baseline (set to your local altitude-corrected value) ───────────
#define NOMINAL_PRESSURE 101279.78f

// ─── Includes ─────────────────────────────────────────────────────────────────
#include <Preferences.h>
#include <Wire.h>
#include "bmp280.h"
#include "Venturi.h"
#include "MICS5524.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_gap_ble_api.h"

// ─── OLED ─────────────────────────────────────────────────────────────────────
#define I2C_ADDRESS 0x3C
#define RST_PIN     -1

// ─── BLE ──────────────────────────────────────────────────────────────────────
#define DEVICE_NAME  "HOPAE-ESP32C3"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // ESP32 → Android
#define RX_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // Android → ESP32

// ─── Pin definitions ──────────────────────────────────────────────────────────
// FIX #1: GPIO 8 removed from LED use — it is the I2C SDA pin.
//         Use Wire.begin(8, 9): SDA=8, SCL=9. Do NOT drive GPIO 8 as output.
#define BTN_READ_PIN GPIO_NUM_3
#define MICS_PIN     0

// ─── Venturi geometry ─────────────────────────────────────────────────────────
#define VENTURI_D_INTAKE  0.015f      // 15 mm intake diameter (m)
#define VENTURI_D_NECK    0.0075f     // 7.5 mm neck diameter (m)
#define VENTURI_Cd        0.98f       // discharge coefficient
#define AIR_DENSITY       1.204f      // kg/m³ at ~20°C sea level
#define AIR_VISCOSITY     1.825e-5f   // Pa·s at ~20°C

// ─── Measurement logic ────────────────────────────────────────────────────────
#define THRESHOLD_IDLE    0.1f        // m/s — below this = no flow
#define MEASUREMENT_MS    500         // ms per measurement cycle
#define RESET_DELAY_MS    (MEASUREMENT_MS * 4)   // 2 s idle → maxhold reset

// ─── MICS5524 print interval ──────────────────────────────────────────────────
#define PRINT_MS 2000

// ─── BLE queue ────────────────────────────────────────────────────────────────
#define RX_BUF_SIZE    64
#define RX_QUEUE_DEPTH 8

// ═══════════════════════════════════════════════════════════════════════════════
//  Shared data structures
// ═══════════════════════════════════════════════════════════════════════════════

struct SensorPayload {
    VenturiResult_t out_venturi;   // maxhold venturi snapshot
    float           ppm;           // CO concentration (or -1 if not ready)
    char            status[20];    // "SAFE" / "DANGER" / "IDLE" / "WARMN" / etc.
};

// gSensor  : the "published" maxhold snapshot — owned by measurement_task,
//            read by oled_task and bleTxTask.
static SensorPayload     gSensor    = {};
static SemaphoreHandle_t gDataMutex = nullptr;   // FIX #4: protects gSensor

// gRawResult : latest raw venturi output from bmp_task — read by measurement_task.
// FIX #4 (continued): separate mutex for the raw result so bmp_task and
//         measurement_task never race on a partially-written struct.
static VenturiResult_t   gRawResult  = {};
static SemaphoreHandle_t gRawMutex   = nullptr;

// ─── FreeRTOS handles ─────────────────────────────────────────────────────────
static EventGroupHandle_t bleEvents    = nullptr;
#define BIT_CONNECTED BIT0

static SemaphoreHandle_t  btnSemaphore = nullptr;
static QueueHandle_t      rxQueue      = nullptr;

// ─── BLE objects ──────────────────────────────────────────────────────────────
static BLEServer         *pServer  = nullptr;
static BLECharacteristic *pTxChar  = nullptr;
static BLECharacteristic *pRxChar  = nullptr;

// ─── Peripheral objects ───────────────────────────────────────────────────────
SSD1306AsciiWire oled;

// FIX #2 (prereq): co object must exist so mics_task can call co.update()
MICS5524 co(MICS_PIN,
            3.3f,        // Vcc
            10000.0f,    // RL = 10 kΩ
            30000UL,     // 30 s warm-up
            200,         // 200 calibration samples
            100UL);      // 100 ms between samples → ~20 s cal phase

Preferences prefs;

BMP280_t  sensorA, sensorB;
Venturi_t venturi;

// Per-sensor absolute pressure offsets (from bmp_calibrate_offset)
float offsetA    = 0.0f;
float offsetB    = 0.0f;
// Differential pressure offset at zero flow (from splash_and_calibrate)
float dP_offset  = 0.0f;
bool  calibrated = false;

// Used only inside mics_task for its own print throttle
static uint32_t lastPrint = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  Thread-safe accessors
// ═══════════════════════════════════════════════════════════════════════════════

// Write entire gSensor payload (venturi snapshot + ppm + status)
void updateSensor(const VenturiResult_t &v, float ppm, const char *status) {
    xSemaphoreTake(gDataMutex, portMAX_DELAY);
    gSensor.out_venturi = v;
    gSensor.ppm         = ppm;
    strncpy(gSensor.status, status, sizeof(gSensor.status) - 1);
    gSensor.status[sizeof(gSensor.status) - 1] = '\0';
    xSemaphoreGive(gDataMutex);
}

// Update only the status string (used by mics_task for WARMN / R0CAL)
void updateStatus(const char *status) {
    xSemaphoreTake(gDataMutex, portMAX_DELAY);
    strncpy(gSensor.status, status, sizeof(gSensor.status) - 1);
    gSensor.status[sizeof(gSensor.status) - 1] = '\0';
    xSemaphoreGive(gDataMutex);
}

// Update only the ppm field (called every cycle by mics_task when READY)
void updatePPM(float ppm) {
    xSemaphoreTake(gDataMutex, portMAX_DELAY);
    gSensor.ppm = ppm;
    xSemaphoreGive(gDataMutex);
}

// Write raw venturi result from bmp_task
void writeRawResult(const VenturiResult_t &r) {
    xSemaphoreTake(gRawMutex, portMAX_DELAY);
    gRawResult = r;
    xSemaphoreGive(gRawMutex);
}

// Read raw venturi result into measurement_task
VenturiResult_t readRawResult() {
    VenturiResult_t r;
    xSemaphoreTake(gRawMutex, portMAX_DELAY);
    r = gRawResult;
    xSemaphoreGive(gRawMutex);
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Button ISR
// ═══════════════════════════════════════════════════════════════════════════════

static volatile TickType_t sLastBtnTick = 0;
#define DEBOUNCE_TICKS pdMS_TO_TICKS(250)

void IRAM_ATTR buttonISR() {
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - sLastBtnTick) < DEBOUNCE_TICKS) return;
    sLastBtnTick = now;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(btnSemaphore, &woken);
    portYIELD_FROM_ISR(woken);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE callbacks
// ═══════════════════════════════════════════════════════════════════════════════

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        xEventGroupSetBits(bleEvents, BIT_CONNECTED);
        Serial.println("[BLE] ✓ Client connected");
    }
    void onDisconnect(BLEServer *) override {
        xEventGroupClearBits(bleEvents, BIT_CONNECTED);
        Serial.println("[BLE] ✗ Disconnected — restarting advertising");
        BLEDevice::startAdvertising();
    }
};

class RxCharCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String val = pChar->getValue();
        if (val.isEmpty()) return;
        char buf[RX_BUF_SIZE] = {};
        size_t len = min(val.length(), (size_t)(RX_BUF_SIZE - 1));
        memcpy(buf, val.c_str(), len);
        if (xQueueSend(rxQueue, buf, 0) != pdTRUE) {
            Serial.println("[RX] ⚠ Queue full — dropped");
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE init
// ═══════════════════════════════════════════════════════════════════════════════

static void initBLE() {
    BLEDevice::init(DEVICE_NAME);

    BLESecurity *pSec = new BLESecurity();
    pSec->setAuthenticationMode(ESP_LE_AUTH_BOND);
    pSec->setCapability(ESP_IO_CAP_NONE);
    pSec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    BLEService *pSvc = pServer->createService(SERVICE_UUID);

    // TX: ESP32 → Android (notify)
    pTxChar = pSvc->createCharacteristic(
        TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pTxChar->addDescriptor(new BLE2902());

    // RX: Android → ESP32 (write)
    pRxChar = pSvc->createCharacteristic(
        RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pRxChar->setCallbacks(new RxCharCB());

    pSvc->start();

    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06);
    pAdv->setMaxPreferred(0x12);

    vTaskDelay(pdMS_TO_TICKS(500));   // let BLE stack settle
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] Advertising as \"%s\"\n", DEVICE_NAME);
    Serial.printf("      TX: %s\n", TX_CHAR_UUID);
    Serial.printf("      RX: %s\n", RX_CHAR_UUID);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE tasks
// ═══════════════════════════════════════════════════════════════════════════════

static void bleTxTask(void *) {
    SensorPayload snap;
    char txBuf[150];

    for (;;) {
        // Block until button pressed
        xSemaphoreTake(btnSemaphore, portMAX_DELAY);

        if (!(xEventGroupGetBits(bleEvents) & BIT_CONNECTED)) {
            Serial.println("[TX] No BLE client — skipping");
            continue;
        }

        // Snapshot under mutex
        xSemaphoreTake(gDataMutex, portMAX_DELAY);
        snap = gSensor;
        xSemaphoreGive(gDataMutex);

        // Format: v_intake(int), flow_Lmin(int), ppm(2dp), status
        snprintf(txBuf, sizeof(txBuf), "%d,%d,%.2f,%s",
                 (int)snap.out_venturi.v_intake,
                 (int)snap.out_venturi.flow_Lmin,
                 snap.ppm,
                 snap.status);

        pTxChar->setValue((uint8_t *)txBuf, strlen(txBuf));
        pTxChar->notify();
        Serial.printf("[TX] → \"%s\"\n", txBuf);
    }
}

static void bleRxTask(void *) {
    char rxBuf[RX_BUF_SIZE];

    for (;;) {
        if (xQueueReceive(rxQueue, rxBuf, portMAX_DELAY) != pdTRUE) continue;
        Serial.printf("[RX] ← \"%s\"\n", rxBuf);

        if (strcmp(rxBuf, "PING") == 0) {
            Serial.println("[RX] PONG");
        } else if (strcmp(rxBuf, "RESET") == 0) {
            Serial.println("[RX] Reset requested");
            // TODO: trigger sensor reset / clear baseline
        } else if (strncmp(rxBuf, "THRESHOLD:", 10) == 0) {
            float thr = atof(rxBuf + 10);
            Serial.printf("[RX] New threshold: %.2f ppm\n", thr);
            // TODO: store thr and use in measurement_task
        } else {
            Serial.printf("[RX] Unknown: \"%s\"\n", rxBuf);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Calibration helpers
// ═══════════════════════════════════════════════════════════════════════════════

// Calibrate per-sensor absolute offsets vs NOMINAL_PRESSURE.
// Call manually via serial 'c' command after dP offset is done.
void bmp_calibrate_offset(int samples = 500, int interval_ms = 15) {
    // FIX #6: both sumA and sumB explicitly initialized to 0
    float tA, pA, tB, pB;
    float sumA = 0.0f;
    float sumB = 0.0f;

    for (int i = 0; i < samples; i++) {
        if (bmp280_readout(&sensorA, &tA, &pA) &&
            bmp280_readout(&sensorB, &tB, &pB)) {
            sumA += pA;
            sumB += pB;
        }
        delay(interval_ms);
    }
    float meanA = sumA / samples;
    float meanB = sumB / samples;
    offsetA = meanA - NOMINAL_PRESSURE;
    offsetB = meanB - NOMINAL_PRESSURE;
    Serial.printf("[CAL] avgA=%.2f avgB=%.2f  offsetA=%.2f offsetB=%.2f Pa\n",
                  meanA, meanB, offsetA, offsetB);
}

// Calibrate differential pressure at zero flow.
// Called by serial 'c' command alongside bmp_calibrate_offset.
void venturi_calibrate_offset(int samples = 500, int interval_ms = 20) {
    Serial.println("[CAL] Calibrating dP offset — ensure NO airflow...");
    float sum = 0.0f;
    int   good = 0;

    for (int i = 0; i < samples; i++) {
        float tA, pA, tB, pB;
        if (bmp280_readout(&sensorA, &tA, &pA) &&
            bmp280_readout(&sensorB, &tB, &pB)) {
            sum += (pA - offsetA) - (pB - offsetB);
            good++;
        }
        delay(interval_ms);
    }

    if (good > 0) {
        dP_offset  = sum / good;
        calibrated = true;
        Serial.printf("[CAL] dP_offset = %.4f Pa  (%d samples)\n", dP_offset, good);
    } else {
        Serial.println("[CAL] FAILED — check BMP280 sensors!");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  OLED helpers
// ═══════════════════════════════════════════════════════════════════════════════

// Splash screen + initial dP calibration with progress bar.
// Runs in setup() (blocking) before FreeRTOS tasks start.
void splash_and_calibrate(SSD1306AsciiWire &oled,
                           int samples     = 500,
                           int interval_ms = 20) {
    const char   *title     = "HOPAE";
    uint8_t       title_px  = strlen(title) * 12;    // 2X: ~12px per char
    uint8_t       title_col = (128 - title_px) / 2;
    const uint8_t BAR_MAX   = 21;

    oled.home();
    oled.clear();
    oled.set2X();
    oled.setCol(title_col);
    oled.println(title);          // row 0-1
    oled.set1X();
    oled.setCursor(0, 2);
    oled.print("Calibrating...");  // row 2
    oled.setCursor(0, 3);
    for (uint8_t i = 0; i < BAR_MAX; i++) oled.print(' ');   // clear bar row

    Serial.println("[CAL] dP calibration — ensure NO airflow...");

    float   sum     = 0.0f;
    int     good    = 0;
    uint8_t bar_pos = 0;

    for (int i = 0; i < samples; i++) {
        float tA, pA, tB, pB;
        if (bmp280_readout(&sensorA, &tA, &pA) &&
            bmp280_readout(&sensorB, &tB, &pB)) {
            sum += (pA - pB);
            good++;
        }
        // Grow progress bar proportionally
        uint8_t bar_target = (uint8_t)((uint32_t)(i + 1) * BAR_MAX / samples);
        while (bar_pos < bar_target) {
            oled.setCursor(bar_pos * 6, 3);
            oled.print('#');
            bar_pos++;
        }
        delay(interval_ms);
    }

    oled.setCursor(0, 2);
    if (good > 0) {
        dP_offset  = sum / good;
        calibrated = true;
        Serial.printf("[CAL] dP_offset = %.4f Pa  (%d samples)\n", dP_offset, good);
        oled.print("Ready!        ");
    } else {
        Serial.println("[CAL] FAILED — check BMP280 sensors!");
        oled.print("FAILED!       ");
    }
    delay(600);
}

// Update OLED with current sensor snapshot.
// ppm < 0 (sensor not ready) shows "----" instead of a number.
void oled_update(SSD1306AsciiWire    &oled,
                 const VenturiResult_t &out_venturi,
                 float                  ppm,
                 const char            *status) {
    static char buf[12];

    // ── Row 0-1, Col LEFT: flow_Lmin 2X ──────────────────────────────────────
    snprintf(buf, sizeof(buf), "%-4.1f", out_venturi.flow_Lmin);
    oled.setCursor(0, 0);
    oled.set2X();
    oled.print(buf);
    const uint8_t COL_MID = oled.col();

    // ── Row 0, Col MID: unit "L/min" 1X ──────────────────────────────────────
    oled.setCursor(COL_MID, 0);
    oled.set1X();
    oled.print("L/min");

    // ── Row 1, Col MID: status string 1X (fixed width, trailing spaces) ───────
    snprintf(buf, sizeof(buf), "%-5s", status);
    oled.setCursor(COL_MID, 1);
    oled.set1X();
    oled.print(buf);

    // ── Row 0-1, Col RIGHT: CO ppm 2X ────────────────────────────────────────
    const uint8_t COL_RIGHT = 80;
    if (ppm < 0.0f) {
        snprintf(buf, sizeof(buf), "----");   // sensor warming up / not ready
    } else {
        snprintf(buf, sizeof(buf), "%-4.1f", ppm);
    }
    oled.setCursor(COL_RIGHT, 0);
    oled.set2X();
    oled.print(buf);

    // ── Row 2, Col RIGHT: label "PPM" 1X ─────────────────────────────────────
    oled.setCursor(COL_RIGHT, 2);
    oled.set1X();
    oled.print("PPM ");

    // ── Row 2-3, Col LEFT: v_intake 2X ───────────────────────────────────────
    snprintf(buf, sizeof(buf), "%-4.1f", out_venturi.v_intake);
    oled.setCursor(0, 2);
    oled.set2X();
    oled.print(buf);

    // ── Row 3, Col RIGHT: label "m/s" 1X ─────────────────────────────────────
    oled.setCursor(COL_RIGHT, 3);
    oled.set1X();
    oled.print("m/s ");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FreeRTOS tasks
// ═══════════════════════════════════════════════════════════════════════════════

// ─── mics_task ───────────────────────────────────────────────────────────────
// FIX #2: this task is now actually started in setup().
// Drives the MICS5524 state machine and keeps gSensor.ppm current.
void mics_task(void *) {
    for (;;) {
        bool changed = co.update();

        if (changed) {
            switch (co.state()) {
                case MICS5524::State::WARMING_UP:
                    // Only reaches here if somehow reset back — guard just in case
                    updateStatus("WARMN");
                    break;

                case MICS5524::State::CALIBRATING:
                    Serial.println("[MICS5524] Warm-up done — calibrating R0...");
                    updateStatus("R0CAL");
                    break;

                case MICS5524::State::READY:
                    Serial.printf("[MICS5524] Ready!  R0 = %.1f Ω\n", co.r0());
                    prefs.begin("mics5524", false);
                    prefs.putFloat("r0", co.r0());
                    prefs.end();
                    Serial.println("[MICS5524] R0 saved to flash.");
                    break;
            }
        }

        // Keep gSensor.ppm fresh every cycle when sensor is ready
        if (co.state() == MICS5524::State::READY) {
            updatePPM(co.ppm());
        }

        // Throttled serial log
        if (millis() - lastPrint >= PRINT_MS) {
            lastPrint = millis();
            switch (co.state()) {
                case MICS5524::State::WARMING_UP:
                    Serial.printf("[MICS5524] Warming up...  %lu s remaining\n",
                                  co.warmupRemainSec());
                    break;
                case MICS5524::State::CALIBRATING:
                    Serial.printf("[MICS5524] Calibrating R0...  %.0f%%\n",
                                  co.calProgress() * 100.0f);
                    break;
                case MICS5524::State::READY:
                    Serial.printf("[MICS5524] Rs=%.0f Ω  Rs/R0=%.3f  CO≈%.1f ppm\n",
                                  co.rs(), co.ratio(), co.ppm());
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ─── bmp_task ─────────────────────────────────────────────────────────────────
// Reads both BMP280s, computes dP and venturi result, publishes to gRawResult.
// FIX #1: no digitalWrite on GPIO 8 (SDA).
// FIX #4: writes result via writeRawResult() under gRawMutex.
void bmp_task(void *) {
    for (;;) {
        // Manual recalibration via serial 'c'/'C'
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'c' || c == 'C') {
                venturi_calibrate_offset();
                bmp_calibrate_offset();
            } else if (c == 'X') {
                debug_raw_mode();   // never returns
            }
        }

        float tempA, presA, tempB, presB;
        if (!bmp280_readout(&sensorA, &tempA, &presA)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (!bmp280_readout(&sensorB, &tempB, &presB)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Apply per-sensor absolute offset and differential offset
        presA -= offsetA;
        presB -= offsetB;
        float dP = (presA - presB) - dP_offset;

        VenturiResult_t raw = {};   // FIX #8: zero-init all fields
        if (venturi_calculate(&venturi, dP, &raw)) {
            writeRawResult(raw);
            Serial.printf("[BMP] dP=%.2f Pa | v_neck=%.3f m/s | v_intake=%.3f m/s | "
                          "Q=%.4f L/min | Re=%.0f\n",
                          dP, raw.v_neck, raw.v_intake, raw.flow_Lmin, raw.reynolds);
        } else {
            // Reverse flow or noise — publish zero result
            writeRawResult(VenturiResult_t{});   // FIX #8: zero-init
            Serial.printf("[BMP] dP=%.2f Pa — no flow / reverse\n", dP);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ─── oled_task ────────────────────────────────────────────────────────────────
// FIX #5: reads from gSensor (maxhold snapshot) — does NOT call updateSensor().
//         Previously it overwrote gSensor with raw result every 500 ms,
//         which destroyed the maxhold value that measurement_task maintains.
void oled_task(void *) {
    for (;;) {
        SensorPayload snap;
        xSemaphoreTake(gDataMutex, portMAX_DELAY);
        snap = gSensor;
        xSemaphoreGive(gDataMutex);

        oled_update(oled, snap.out_venturi, snap.ppm, snap.status);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ─── measurement_task ────────────────────────────────────────────────────────
// Maxhold logic: tracks peak v_intake during a breath, resets after idle.
// Determines SAFE / DANGER / WARMN based on peak + ppm.
// Owns gSensor (writes via updateSensor).
void measurement_task(void *) {
    float    max_intake = 0.0f;
    uint32_t idle_ms    = 0;

    for (;;) {
        // FIX #4: read raw result under mutex
        VenturiResult_t snap = readRawResult();

        // Read current ppm from gSensor (written by mics_task via updatePPM)
        float ppm;
        xSemaphoreTake(gDataMutex, portMAX_DELAY);
        ppm = gSensor.ppm;
        xSemaphoreGive(gDataMutex);

        float v = snap.v_intake;

        if (v > THRESHOLD_IDLE) {
            idle_ms = 0;
            Serial.printf("[MEAS] v=%.1f m/s\n", v);

            if (v > max_intake) {
                max_intake = v;

                // FIX: also guard against ppm = -1 (sensor not yet ready)
                const char *status;
                if (ppm < 0.0f) {
                    status = "WARMN";   // CO sensor still initialising
                } else if (ppm < 50.0f) {
                    status = "SAFE";
                } else {
                    status = "DANGER";
                }

                updateSensor(snap, ppm, status);
            }
        } else {
            idle_ms += MEASUREMENT_MS;
            if (idle_ms >= RESET_DELAY_MS) {
                max_intake = 0.0f;
                idle_ms    = 0;
                // FIX #8: VenturiResult_t{} zero-inits all 5 float fields
                updateSensor(VenturiResult_t{}, ppm, "IDLE");
                Serial.println("[MEAS] Idle — maxhold reset");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_MS));
    }
}

// ─── Task handles (untuk debug_raw_mode) ──────────────────────────────────────
static TaskHandle_t hMics = nullptr, hOled = nullptr, hMeas = nullptr,
                     hBleTx = nullptr, hBleRx = nullptr;


// ═══════════════════════════════════════════════════════════════════════════
//  DEBUG RAW MODE — trigger via Serial 'X'
//  Suspend semua task lain, lalu loop selamanya baca nilai mentah MICS + BMP.
//  TIDAK ADA EXIT — harus reset board untuk balik ke mode normal.
// ═══════════════════════════════════════════════════════════════════════════
void debug_raw_mode() {
    Serial.println("\n[DEBUG] === RAW SENSOR MODE ===");
    Serial.println("[DEBUG] Semua task disuspend. RESET board untuk keluar.");

    // bmp_task adalah caller (karena 'X' dibaca di dalam loop-nya),
    // jadi gak perlu disuspend — fungsi ini cukup gak pernah return.
    if (hMics)  vTaskSuspend(hMics);
    if (hOled)  vTaskSuspend(hOled);
    if (hMeas)  vTaskSuspend(hMeas);
    if (hBleTx) vTaskSuspend(hBleTx);
    if (hBleRx) vTaskSuspend(hBleRx);

    for (;;) {
        // ── Raw MICS5524 ────────────────────────────────────────────────────
        int   rawADC = analogRead(MICS_PIN);
        float vOut   = (rawADC / 4095.0f) * 3.3f;

        // ── Raw BMP280 (bypass semua offset/kalibrasi) ──────────────────────
        float tA = NAN, pA = NAN, tB = NAN, pB = NAN;
        bool  okA = bmp280_readout(&sensorA, &tA, &pA);
        bool  okB = bmp280_readout(&sensorB, &tB, &pB);

        Serial.printf("[DEBUG] MICS raw=%4d Vout=%.3fV | "
                      "A:%s T=%.2fC P=%.2fPa | B:%s T=%.2fC P=%.2fPa | dP_raw=%.2fPa\n",
                      rawADC, vOut,
                      okA ? "OK" : "FAIL", tA, pA,
                      okB ? "OK" : "FAIL", tB, pB,
                      (okA && okB) ? (pA - pB) : NAN);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // unreachable
}

// ═══════════════════════════════════════════════════════════════════════════════
//  setup
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);

    // ── FreeRTOS primitives ──────────────────────────────────────────────────
    bleEvents    = xEventGroupCreate();
    gDataMutex   = xSemaphoreCreateMutex();
    gRawMutex    = xSemaphoreCreateMutex();
    btnSemaphore = xSemaphoreCreateBinary();
    rxQueue      = xQueueCreate(RX_QUEUE_DEPTH, RX_BUF_SIZE);

    // ── Button (active-LOW, internal pull-up) ────────────────────────────────
    pinMode(BTN_READ_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_READ_PIN), buttonISR, FALLING);

    // ── BLE (must be before Wire.begin — uses its own I2C-unrelated stack) ───
    initBLE();
    xTaskCreatePinnedToCore(bleTxTask, "BLE_TX", 4096, nullptr, 2, &hBleTx, 0);
    xTaskCreatePinnedToCore(bleRxTask, "BLE_RX", 4096, nullptr, 2, &hBleRx, 0);



    // ── I2C + OLED ───────────────────────────────────────────────────────────
    // FIX #1: GPIO 8 = SDA, GPIO 9 = SCL — never set as digital output
    Wire.begin(8, 9);
    Wire.setClock(400000L);

#if RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else
    oled.begin(&Adafruit128x32, I2C_ADDRESS);
#endif
    oled.setFont(Adafruit5x7);

    // ── BMP280 init with retry ───────────────────────────────────────────────
    // FIX #3: use && so BOTH sensors must succeed, not just one.
    //         Also halt if all retries fail instead of silently continuing.
    bool bmpOk = false;
    for (int attempt = 1; attempt <= 5; attempt++) {
        if (bmp280_init(&sensorA, 0x77) && bmp280_init(&sensorB, 0x76)) {
            bmpOk = true;
            break;
        }
        Serial.printf("[BMP280] Init attempt %d/5 failed — retrying...\n", attempt);
        updateStatus("xBMP");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!bmpOk) {
        Serial.println("[BMP280] FATAL: both sensors failed to init after 5 attempts!");
        updateStatus("xBMP");
        // Halt safely via idle task delay (avoids WDT reset storm)
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    Serial.printf("[BMP280] A: addr=0x%02X  chip_id=0x%02X\n",
                  sensorA.addr, sensorA.chip_id);
    Serial.printf("[BMP280] B: addr=0x%02X  chip_id=0x%02X\n",
                  sensorB.addr, sensorB.chip_id);

    // 2× temp oversampling, 16× pressure oversampling, no IIR filter
    bmp280_configure(&sensorA, BMP280_OVRS_2X, BMP280_OVRS_16X, BMP280_FILTER_OFF);
    bmp280_set_mode(&sensorA, BMP280_MODE_NORMAL);
    bmp280_configure(&sensorB, BMP280_OVRS_2X, BMP280_OVRS_16X, BMP280_FILTER_OFF);
    bmp280_set_mode(&sensorB, BMP280_MODE_NORMAL);

    // ── Venturi geometry ─────────────────────────────────────────────────────
    venturi_init(&venturi, VENTURI_D_INTAKE, VENTURI_D_NECK);
    Serial.printf("[Venturi] A1=%.6f m²  A2=%.6f m²  A_ratio=%.4f\n",
                  venturi.A1, venturi.A2, venturi.A_ratio);

    // ── MICS5524 — try loading saved R0 from flash ───────────────────────────
    co.begin();
    prefs.begin("mics5524", true);
    float savedR0 = prefs.getFloat("r0", -1.0f);
    prefs.end();

    if (savedR0 > 0.0f) {
        co.setR0(savedR0);
        Serial.printf("[MICS5524] R0 loaded from flash: %.1f Ω  (skipping warm-up)\n",
                      savedR0);
    } else {
        Serial.println("[MICS5524] No stored R0 — will warm-up and calibrate.");
        updateStatus("WARMN");
    }

    // ── Splash + differential pressure calibration ───────────────────────────
    delay(200);
    splash_and_calibrate(oled);
    oled.clear();

    // ── FreeRTOS sensor tasks ────────────────────────────────────────────────
    // FIX #2: mics_task is now actually created and started
    // xTaskCreate()
    xTaskCreate(mics_task,        "mics_task", 4096, nullptr, 3, &hMics);
    xTaskCreate(bmp_task,         "bmp_task",  4096, nullptr, 4, nullptr); // bmp_task = caller, gak perlu disuspend diri sendiri
    xTaskCreate(oled_task,        "oled_task", 4096, nullptr, 3, &hOled);
    xTaskCreate(measurement_task, "measure",   4096, nullptr, 5, &hMeas);

    Serial.println("[RTOS] All tasks started.  Press button to transmit via BLE.");
    Serial.println("[RTOS] Send 'c' via Serial to recalibrate BMP280 offsets.");
}

// ─── loop ─────────────────────────────────────────────────────────────────────
// All work is handled in FreeRTOS tasks.
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
