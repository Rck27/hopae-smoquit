#define NOMINAL_PRESSURE 101279.78f  //Measure  from meters above sea level

#include <Preferences.h>

// #include "BluetoothSerial.h"
// BluetoothSerial sBT;


///////////////////////////// OLED
#include <Wire.h>

#include "bmp280.h"
#include "Venturi.h"
#include "MICS5524.h"

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1
////////////////////////////////





//////////////////////////////// BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_gap_ble_api.h"

#define DEVICE_NAME "HOPAE-ESP32C3"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 → Android
#define RX_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // Android → ESP32
//////////////////////////////////////////////////////


///////////////////////////////////// BUTTON
#define BTN_READ_PIN GPIO_NUM_3



////////////////////////////////////// MICS-5524
#define MICS_PIN 0     // ADC pin
#define PRINT_MS 2000  // print interval (ms)



struct SensorPayload {
  VenturiResult_t out_venturi;
  float ppm;
  char status[20];
};

VenturiResult_t result;

static SensorPayload gSensor = {};
static SemaphoreHandle_t gDataMutex;  // guards gSensor across tasks

// ─── FreeRTOS Handles ────────────────────────────────────────────────────────
static EventGroupHandle_t bleEvents;
#define BIT_CONNECTED BIT0

static SemaphoreHandle_t btnSemaphore;  // ISR → bleTxTask
static QueueHandle_t rxQueue;

#define RX_BUF_SIZE 64
#define RX_QUEUE_DEPTH 8

// ─── BLE Objects ─────────────────────────────────────────────────────────────
static BLEServer *pServer = nullptr;
static BLECharacteristic *pTxChar = nullptr;
static BLECharacteristic *pRxChar = nullptr;


// struct SensorPayload {
//   VenturiResult_t  result;
//   float co_ppm;
//   char  status[32];
// };

char status_buf[22];


// ─── Button ISR ──────────────────────────────────────────────────────────────
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

// ─── BLE Server Callbacks ─────────────────────────────────────────────────────
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

// ─── RX Characteristic Callbacks ─────────────────────────────────────────────
// Called from BLE stack task — push to queue, handle in bleRxTask
class RxCharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String val = pChar->getValue();
    if (val.isEmpty()) return;

    char buf[RX_BUF_SIZE] = {};
    size_t len = min(val.length(), (size_t)(RX_BUF_SIZE - 1));
    memcpy(buf, val.c_str(), len);

    if (xQueueSend(rxQueue, buf, 0) != pdTRUE) {
      Serial.println("[RX] ⚠ Queue full — feedback dropped");
    }
  }
};

// ─── BLE Init ────────────────────────────────────────────────────────────────
static void initBLE() {
  BLEDevice::init(DEVICE_NAME);
  BLESecurity *pSec = new BLESecurity();
  pSec->setAuthenticationMode(ESP_LE_AUTH_BOND);  // ← ini yang penting
  pSec->setCapability(ESP_IO_CAP_NONE);           // no screen/keyboard
  pSec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  // clearBonds();   // ← wipe stale Android bond keys

  BLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService *pSvc = pServer->createService(SERVICE_UUID);

  // TX: ESP32 → Android (NOTIFY, fire-and-forget)
  pTxChar = pSvc->createCharacteristic(
    TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  // RX: Android → ESP32 (WRITE or WRITE_NR for low-latency)
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
  vTaskDelay(pdMS_TO_TICKS(500));  // let BLE stack finish cleanup

  BLEDevice::startAdvertising();

  Serial.printf("[BLE] Advertising as \"%s\"\n", DEVICE_NAME);
  Serial.printf("      TX: %s\n", TX_CHAR_UUID);
  Serial.printf("      RX: %s\n", RX_CHAR_UUID);
}

static void bleTxTask(void * /*pvParams*/) {
  SensorPayload snap;
  char txBuf[150];

  for (;;) {
    // ── Wait for button press ─────────────────────────────────────────────
    xSemaphoreTake(btnSemaphore, portMAX_DELAY);

    // ── Guard: skip if nobody is listening ───────────────────────────────
    if (!(xEventGroupGetBits(bleEvents) & BIT_CONNECTED)) {
      Serial.println("[TX] Button pressed — no BLE client connected, skipping");
      snprintf(status_buf, sizeof(status_buf), "TX-X");

      continue;
    }

    // ── Snapshot sensor data under mutex ─────────────────────────────────
    xSemaphoreTake(gDataMutex, portMAX_DELAY);
    snap = gSensor;  // struct copy
    xSemaphoreGive(gDataMutex);

    // ── Format & notify ───────────────────────────────────────────────────
    snprintf(txBuf, sizeof(txBuf), "%d,%d,%.2f,%s",
             (int)snap.out_venturi.v_intake,
             (int)snap.out_venturi.flow_Lmin,
             snap.ppm,
             snap.status);

    pTxChar->setValue((uint8_t *)txBuf, strlen(txBuf));
    pTxChar->notify();  // Android can ignore — that's fine

    Serial.printf("[TX] → \"%s\"\n", txBuf);
    snprintf(status_buf, sizeof(status_buf), "TX-Y");
  }
}


    // ─── Task: BLE RX ─────────────────────────────────────────────────────────────
    /**
 * Processes feedback written by the Android app.
 * Extend the if/else chain with your own commands as needed.
 */
    static void bleRxTask(void * /*pvParams*/) {
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
          // TODO: store thr and use in your sensor logic

        } else {
          Serial.printf("[RX] Unknown command: \"%s\"\n", rxBuf);
        }
      }
    }





    void updateSensor(const VenturiResult_t &out_venturi, float ppm, const char *status) {
      xSemaphoreTake(gDataMutex, portMAX_DELAY);
      gSensor.out_venturi = out_venturi;
      gSensor.ppm = ppm;
      strncpy(gSensor.status, status, sizeof(gSensor.status) - 1);
      gSensor.status[sizeof(gSensor.status) - 1] = '\0';
      xSemaphoreGive(gDataMutex);
    }


   





    float dP_offset = 0.0f;
    bool calibrated = false;
    float offsetA = 0.0f;
    float offsetB = 0.0f;
    Venturi_t venturi;

    float dP = 0.0f;

    BMP280_t sensorA;
    BMP280_t sensorB;

    void bmp_calibrate_offset(int samples = 500, int interval_ms = 15) {
      //   float sumA, sumB = 0.0f;
      float tA, pA, tB, pB, sumA, sumB = { 0.0f };
      for (int i = 0; i < samples; i++) {
        if (bmp280_readout(&sensorA, &tA, &pA) && bmp280_readout(&sensorB, &tB, &pB)) {
          sumA += pA;
          sumB += pB;
        }
        delay(interval_ms);
      }
      float meanA = (sumA / samples);
      float meanB = (sumB / samples);
      offsetA = meanA - NOMINAL_PRESSURE;
      offsetB = meanB - NOMINAL_PRESSURE;
      Serial.printf("avg A %.2f avg B %.2f, offset A %.2f B %.2f \n", meanA, meanB, offsetA, offsetB);
    }


    void venturi_calibrate_offset(int samples = 500, int interval_ms = 20) {
      Serial.println("Calibrating offset — ensure NO airflow...");
      float sum = 0.0f;
      int good = 0;

      for (int i = 0; i < samples; i++) {

        float tA, pA, tB, pB;
        float sumA = 0.0f;
        float sumB = 0.0f;
        if (bmp280_readout(&sensorA, &tA, &pA) && bmp280_readout(&sensorB, &tB, &pB)) {
          sum += (pA - pB);
          good++;
        }
        delay(interval_ms);
      }

      if (good > 0) {
        dP_offset = sum / good;
        calibrated = true;
        Serial.printf("Offset = %.4f Pa (averaged over %d samples)\n",
                      dP_offset, good);
      } else {
        Serial.println("Calibration failed — check sensors!");
      }
    }

    void splash_screen(SSD1306AsciiWire & oled) {
      const char *title = "HOPAE";
      const char *msg = "";
      const uint8_t dots = 10;
      const uint16_t delay_char = 150;  // ms per dot


      uint8_t title_px = strlen(title) * 12;
      uint8_t title_col = (128 - title_px) / 2;

      oled.home();
      oled.set2X();
      oled.setCol(title_col);
      oled.println(title);

      oled.set1X();
      oled.print(msg);  // tulis dulu tanpa newline

      uint8_t bar_max = 21;      // max karakter di row 3
      uint8_t bar_steps = dots;  // bar tumbuh seiring jumlah dot
      oled.setCursor(0, 3);
      // print dots satu per satu, update bar tiap step
      for (uint8_t i = 0; i < bar_max; i++) {
        oled.print('#');
        delay(delay_char);
      }

      // delay(800);
      // oled.clear();
    }


    void splash_and_calibrate(SSD1306AsciiWire & oled, int samples = 500, int interval_ms = 20) {
      // ── Splash title (langsung tampil) ───────────────────────────────────────
      const char *title = "HOPAE";
      uint8_t title_px = strlen(title) * 12;  // 2X font: ~12px per char
      uint8_t title_col = (128 - title_px) / 2;

      oled.home();
      oled.clear();

      oled.set2X();
      oled.setCol(title_col);
      oled.println(title);  // row 0-1

      oled.set1X();
      oled.setCursor(0, 2);
      oled.print("Calibrating...");  // row 2

      // Pre-fill bar dengan spasi supaya overwrite bersih nanti
      const uint8_t BAR_MAX = 21;  // karakter di row 3
      oled.setCursor(0, 3);
      for (uint8_t i = 0; i < BAR_MAX; i++) oled.print(' ');

      Serial.println("Calibrating offset — ensure NO airflow...");

      // ── Kalibrasi + update bar ────────────────────────────────────────────────
      float sum = 0.0f;
      int good = 0;
      uint8_t bar_pos = 0;  // berapa '#' sudah tergambar

      for (int i = 0; i < samples; i++) {
        float tA, pA, tB, pB;
        if (bmp280_readout(&sensorA, &tA, &pA) && bmp280_readout(&sensorB, &tB, &pB)) {
          sum += (pA - pB);
          good++;
        }

        // Hitung berapa '#' yang seharusnya tergambar di titik ini
        uint8_t bar_target = (uint8_t)((uint32_t)(i + 1) * BAR_MAX / samples);
        while (bar_pos < bar_target) {
          oled.setCursor(bar_pos * 6, 3);  // 1X: 6px per char
          oled.print('#');
          bar_pos++;
        }

        delay(interval_ms);
      }

      // ── Hasil kalibrasi ───────────────────────────────────────────────────────
      oled.setCursor(0, 2);

      if (good > 0) {
        dP_offset = sum / good;
        calibrated = true;
        Serial.printf("Offset = %.4f Pa (averaged over %d samples)\n",
                      dP_offset, good);
        oled.print("Ready!        ");  // overwrite "Calibrating..."
      } else {
        Serial.println("Calibration failed — check sensors!");
        oled.print("FAILED!       ");  // lebar sama supaya bersih
      }

      delay(600);  // jeda sebentar biar user bisa baca status sebelum main screen
    }



    void oled_update(SSD1306AsciiWire & oled,
                     const VenturiResult_t &out_venturi,
                     float ppm,
                     const char *status) {
      static char buf[12];

      // ── Row 0-1, Col LEFT: flow_Lmin 2X ──────────────────────────────────────
      // "%-5.1f" → selalu 5 char (misal "12.3 "), trailing space overwrite sisa
      snprintf(buf, sizeof(buf), "%-4.1f", out_venturi.flow_Lmin);
      oled.setCursor(0, 0);
      oled.set2X();
      oled.print(buf);

      // Capture col sekali — karena format fixed-width, nilai ini konstan
      const uint8_t COL_MID = oled.col();

      // ── Row 0, Col MID: satuan "L/min" 1X ────────────────────────────────────
      oled.setCursor(COL_MID, 0);
      oled.set1X();
      oled.print("L/min");

      // ── Row 1, Col MID: status 1X, padded fixed-width ─────────────────────
      // "%-8s" → selalu 8 char, trailing space hapus sisa teks sebelumnya
      snprintf(buf, sizeof(buf), "%-5s", status);
      oled.setCursor(COL_MID, 1);
      oled.set1X();
      oled.print(buf);

      // ── Row 0-1, Col RIGHT: ppm 2X ───────────────────────────────────────────
      const uint8_t COL_RIGHT = 80;
      snprintf(buf, sizeof(buf), "%-3.1f", ppm);
      oled.setCursor(COL_RIGHT, 0);
      oled.set2X();
      oled.print(buf);

      // ── Row 2, Col RIGHT: label "PPM" 1X ─────────────────────────────────────
      oled.setCursor(COL_RIGHT, 2);
      oled.set1X();
      oled.print("PPM ");  // trailing space jaga-jaga

      // ── Row 2-3, Col LEFT: v_intake 2X ───────────────────────────────────────
      // BUG FIX: buf sebelumnya masih isi ppm — harus snprintf ulang dulu
      snprintf(buf, sizeof(buf), "%-4.1f", out_venturi.v_intake);
      oled.setCursor(0, 2);
      oled.set2X();
      oled.print(buf);

      // ── Row 3, Col RIGHT: label "m/s" 1X ─────────────────────────────────────
      oled.setCursor(COL_RIGHT, 3);
      oled.set1X();
      oled.print("m/s ");  // trailing space jaga-jaga
    }


// extern SemaphoreHandle_t result_mutex;
#define THRESHOLD_IDLE 0.1f                  // m/s — di bawah ini dianggap idle
#define MEASUREMENT_MS 500                   // ms per loop tick
#define RESET_DELAY_MS (MEASUREMENT_MS * 4)  // 2 s idle → reset maxhold \
                                             // ↑ pakai () biar operator precedence aman



    SSD1306AsciiWire oled;



    MICS5524 co(MICS_PIN,  // ADC pin
                3.3f,      // Vcc
                10000.0f,  // RL = 10 kΩ
                30000UL,   // 30 s warm-up (increase to 300000 for better accuracy)
                200,       // 200 calibration samples
                100UL);    // 100 ms between samples → ~20 s cal phase

    Preferences prefs;
    uint32_t lastPrint = 0;


    bool debug_mode = 0;
    void setup() {
      Serial.begin(115200);

      bleEvents = xEventGroupCreate();
      gDataMutex = xSemaphoreCreateMutex();
      btnSemaphore = xSemaphoreCreateBinary();
      rxQueue = xQueueCreate(RX_QUEUE_DEPTH, RX_BUF_SIZE);
      pinMode(8, OUTPUT);

      // Button — active LOW (built-in pull-up)
      pinMode(BTN_READ_PIN, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(BTN_READ_PIN), buttonISR, FALLING);

      initBLE();
    //   sBT.begin("HOPAE");

      xTaskCreatePinnedToCore(bleTxTask, "BLE_TX", 4096, nullptr, 2, nullptr, 0);
      xTaskCreatePinnedToCore(bleRxTask, "BLE_RX", 4096, nullptr, 2, nullptr, 0);

      Serial.println("[RTOS] Ready — press button to send a reading via BLE");

      Wire.begin(8, 9);  // SDA=21, SCL=22 on most ESP32 boards
      Wire.setClock(400000L);

#if RST_PIN >= 0
      oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else   // RST_PIN >= 0
      oled.begin(&Adafruit128x32, I2C_ADDRESS);
#endif  // RST_PIN >= 0

      oled.setFont(Adafruit5x7);

      for(int i = 0; i < 5; i++){

        if(bmp280_init(&sensorA, 0x77) || bmp280_init(&sensorB, 0x76)){ break;
        }
        else{
          Serial.println("BMP280A not found!");
          snprintf(status_buf, sizeof(status_buf), "xBMP");

        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
     
      Serial.printf("BMP280 found at 0x%02X, chip_id=0x%02X\n",
                    sensorA.addr, sensorA.chip_id);
      Serial.printf("BMP280 found at 0x%02X, chip_id=0x%02X\n",
                    sensorB.addr, sensorB.chip_id);

      // 2× oversampling on both channels, IIR filter coefficient 4
      bmp280_configure(&sensorA, BMP280_OVRS_2X, BMP280_OVRS_16X, BMP280_FILTER_OFF);
      bmp280_set_mode(&sensorA, BMP280_MODE_NORMAL);
      bmp280_configure(&sensorB, BMP280_OVRS_2X, BMP280_OVRS_16X, BMP280_FILTER_OFF);
      bmp280_set_mode(&sensorB, BMP280_MODE_NORMAL);

      venturi_init(&venturi, VENTURI_D_INTAKE, VENTURI_D_NECK);

      Serial.printf("A1=%.6f m²  A2=%.6f m²  A_ratio=%.4f\n",
                    venturi.A1, venturi.A2, venturi.A_ratio);

      co.begin();

      // --- restore R0 from flash so we skip calibration on reboot ----------
      prefs.begin("mics5524", true);  // read-only namespace
      float savedR0 = prefs.getFloat("r0", -1.0f);
      prefs.end();

      if (savedR0 > 0.0f) {
        co.setR0(savedR0);
        Serial.printf("[MICS5524] Loaded R0 from flash: %.1f Ω  → skipping calibration\n", savedR0);
      } else {
        Serial.println("[MICS5524] No stored R0. Will calibrate in clean air.");
      }


      delay(200);

      splash_and_calibrate(oled);

      oled.clear();

      //     xTaskCreate(
      //     mics_task,         // Task function
      //     "mics task",       // Task name
      //     4096,             // Stack size (bytes)
      //     NULL,              // Parameters
      //     3,                 // Priority
      //     NULL  // Task handle
      //   );

      xTaskCreate(
        bmp_task, "bmp_task", 4096, NULL, 4, NULL);
      xTaskCreate(
        oled_task, "oled task", 4096, NULL, 6, NULL);

      xTaskCreate(measurement_task, "measure", 4096, NULL, 6, NULL);
    }


    void mics_task(void *parameter) {
      for (;;) {
        bool stateChanged = co.update();  // <-- call every loop, non-blocking
        if (stateChanged) {
          switch (co.state()) {
            case MICS5524::State::WARMING_UP:
              snprintf(status_buf, sizeof(status_buf), "WARMN");


              break;

            case MICS5524::State::CALIBRATING:
              Serial.println("[MICS5524] Warm-up done. Calibrating R0...");
              break;

            case MICS5524::State::READY:
              Serial.printf("[MICS5524] Ready! R0 = %.1f Ω\n", co.r0());
              // --- save R0 to flash ---
              prefs.begin("mics5524", false);
              prefs.putFloat("r0", co.r0());
              prefs.end();
              Serial.println("[MICS5524] R0 saved to flash.");
              break;
          }
        }

        if (millis() - lastPrint >= PRINT_MS) {
          lastPrint = millis();

          switch (co.state()) {
            case MICS5524::State::WARMING_UP:
              Serial.printf("[MICS5524] Warming up... %lu s remaining\n",
                            co.warmupRemainSec());
              break;

            case MICS5524::State::CALIBRATING:
              Serial.printf("[MICS5524] Calibrating... %.0f%%\n",
                            co.calProgress() * 100.0f);
              break;

            case MICS5524::State::READY:
              {
                float r = co.ratio();
                float ppm = co.ppm();
                float rs = co.rs();
                Serial.printf("[MICS5524] Rs=%.0f Ω  Rs/R0=%.3f  CO≈%.1f ppm  \n",
                              rs, r, ppm);
                break;
              }
          }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }

    void bmp_task(void *parameter) {
      bool led_state = 0;
      for (;;) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'c' || c == 'C') {
            venturi_calibrate_offset();
            bmp_calibrate_offset();
          }
        }

        // if (!calibrated) continue;
        digitalWrite(8, led_state);
        led_state = !led_state;

        float tempA, presA, tempB, presB;
        if (!bmp280_readout(&sensorA, &tempA, &presA)) continue;
        if (!bmp280_readout(&sensorB, &tempB, &presB)) continue;
        presA -= offsetA;
        presB -= offsetB;

        dP = ((presA) - (presB));  // - dP_offset;  // intake − neck, should be positive during flow
        if (venturi_calculate(&venturi, dP, &result)) {
          Serial.printf(
            "dP=%.2f Pa | v_neck=%.3f m/s | v_intake=%.3f m/s | "
            "Q=%.4f L/min | Re=%.0f\n",
            dP,
            result.v_neck,
            result.v_intake,
            result.flow_Lmin,
            result.reynolds);
        } else {
          Serial.printf("dP=%.2f Pa — no flow / reverse\n", dP);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }

    void oled_task(void *parameter) {
      for (;;) {
        oled_update(oled, result, co.ppm(), status_buf);
        updateSensor(result, co.ppm(), status_buf);
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }


    // ─── Measurement Task ─────────────────────────────────────────────────────────
    void measurement_task(void *parameter) {
      float max_intake = 0.0f;
      uint32_t idle_ms = 0;

      for (;;) {
        // ── Snapshot result dari task lain (thread-safe) ───────────────────
        VenturiResult_t snap = {};
        float ppm = co.ppm();

        // xSemaphoreTake(result_mutex, portMAX_DELAY);
        snap = result;

        // xSemaphoreGive(result_mutex);

        float v = snap.v_intake;

        // ── Maxhold logic ──────────────────────────────────────────────────
        if (v > THRESHOLD_IDLE) {
          idle_ms = 0;
          Serial.printf("[V] %.1f ", v);
          if (v > max_intake) {
            max_intake = v;

            const char *status = (ppm < 50.0f) ? "SAFE" : "DANGER";
            updateSensor(snap, ppm, status);  // → gSensor, dibaca BLE TX
          }

        } else {
          idle_ms += MEASUREMENT_MS;

          if (idle_ms >= RESET_DELAY_MS) {
            max_intake = 0.0f;
            idle_ms = 0;
            updateSensor(VenturiResult_t{ 0.0f, 0.0f }, 0.0f, "IDLE");
            Serial.println("[MEAS] Idle — maxhold reset");
          }
        }

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_MS));
      }
    }




    void loop() {
    }