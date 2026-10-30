/*
 * ============================================================
 *  SAFEGAS - Deteksi Kebocoran Gas LPG Rumah Tangga
 *  Versi: v12 (WiFi + Safety + MQTT EMQX Cloud)
 *
 *  Ringkasan:
 *  - Safety : deteksi gas, katup servo, relay listrik, buzzer, LED.
 *  - WiFi   : berjalan di task terpisah (netTask, core 0).
 *  - MQTT   : EMQX Cloud (TLS, port 8883), telemetry & event.
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <stdarg.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// ============================================================
//  KONFIGURASI
// ============================================================

// ---------- WIFI ----------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// ---------- EMQX CLOUD ----------
const char* MQTT_HOST = "j0dcec1e.ala.asia-southeast1.emqxsl.com";
const int   MQTT_PORT = 8883;  // MQTT over TLS
const char* MQTT_USER = "simasrol";
const char* MQTT_PASS = "simas1234";

#define DEVICE_ID       "safegas01"
#define TOPIC_TELEMETRY "safegas/" DEVICE_ID "/telemetry"
#define TOPIC_EVENT     "safegas/" DEVICE_ID "/event"

// ---------- PIN ----------
#define PIN_GAS     34
#define PIN_BUZZER  23
#define PIN_SERVO   18
#define PIN_RELAY   19
#define PIN_BTN_RST 25
#define PIN_LED_G   13
#define PIN_LED_R   32
#define PIN_SDA     21
#define PIN_SCL     22

// ---------- PARAMETER LOGIKA ----------
const int GAS_ON       = 1500;
const int GAS_OFF      = 1300;
const int GAS_CRITICAL = 2800;

const unsigned long PERSIST_LOCK_MS = 15000;
const unsigned long TREND_WINDOW_MS = 1000;
const int           TREND_RISE_MIN  = 50;
const int           TREND_CONFIRM   = 2;

// ---------- WAKTU ----------
const unsigned long WARMUP_MS = 5000;
const unsigned long SAMPLE_MS = 250;
const unsigned long EVAL_MS   = 1000;
const unsigned long OLED_MS   = 500;

// ---------- SERVO ----------
const int SERVO_OPEN   = 90;
const int SERVO_CLOSED = 0;

// ============================================================
//  OBJEK & VARIABEL GLOBAL
// ============================================================

// ---------- MQTT ----------
// TLS client untuk EMQX Cloud (port 8883)
WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);

// Queue menjaga agar event dari loop() tidak mengakses MQTT client lintas-core.
struct MqttEventMessage {
  char msg[96];
};
QueueHandle_t mqttEventQueue = nullptr;

// ---------- STATUS ----------
enum Status { ST_WARMUP, ST_NORMAL, ST_ADA_GAS, ST_MEMBURUK, ST_BAHAYA };
const char* STATUS_STR[] = { "WARMUP", "NORMAL", "ADA_GAS", "MEMBURUK", "BAHAYA" };

bool warmup      = true;
bool gasDetected = false;
bool worsening   = false;
bool bahayaLock  = false;
bool valveOpen   = false;
bool powerOn     = false;
bool buzzerOn    = false;
int  riseCount   = 0;

unsigned long bootMs         = 0;
unsigned long gasStartTime   = 0;
unsigned long lastTrendTime  = 0;
unsigned long tSample = 0, tEval = 0, tOled = 0;

// ---------- SENSOR ----------
#define BUF_N 8
int gasBuf[BUF_N];
int bufIdx = 0, bufCount = 0;
int gasRaw = 0, gasAvg = 0;
int gasAtTrendStart = 0;

// ---------- PERANGKAT ----------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Servo valveServo;

// ---------- STATUS JARINGAN ----------
volatile bool wifiOk = false;
volatile bool mqttOk = false;  // true jika MQTT sudah connect

// ============================================================
//  UTIL & LOGIKA SAFETY
// ============================================================
Status currentStatus() {
  if (warmup)      return ST_WARMUP;
  if (bahayaLock)  return ST_BAHAYA;
  if (worsening)   return ST_MEMBURUK;
  if (gasDetected) return ST_ADA_GAS;
  return ST_NORMAL;
}

void logEvent(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  char buf[96];
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.printf("[EVENT] %s\n", buf);

  // Antrekan event; publish dilakukan oleh netTask agar MQTT client tidak
  // diakses bersamaan dari task/core yang berbeda.
  if (mqttEventQueue != nullptr) {
    MqttEventMessage eventMsg;
    snprintf(eventMsg.msg, sizeof(eventMsg.msg), "%s", buf);
    xQueueSend(mqttEventQueue, &eventMsg, 0);
  }
}

void sampleSensors() {
  gasRaw = analogRead(PIN_GAS);
  gasBuf[bufIdx] = gasRaw;
  bufIdx = (bufIdx + 1) % BUF_N;
  if (bufCount < BUF_N) bufCount++;

  long sum = 0;
  for (int i = 0; i < bufCount; i++) sum += gasBuf[i];
  gasAvg = sum / bufCount;
}

void setLock(const char* alasan) {
  if (bahayaLock) return;
  bahayaLock = true;
  logEvent("BAHAYA terkunci: %s (ADC=%d)", alasan, gasAvg);
}

bool tryReset(const char* src) {
  if (warmup) {
    logEvent("Reset diabaikan (%s): masih warm-up", src);
    return false;
  }

  if (!bahayaLock) {
    logEvent("Reset diabaikan (%s): sistem sedang normal", src);
    return false;
  }

  if (gasAvg >= GAS_OFF) {
    logEvent("Reset DITOLAK (%s): gas masih tinggi (ADC=%d)", src, gasAvg);
    return false;
  }

  bahayaLock      = false;
  gasDetected     = false;
  worsening       = false;
  riseCount       = 0;
  gasStartTime    = 0;
  gasAtTrendStart = 0;
  logEvent("Reset BERHASIL (%s): sistem normal kembali", src);
  return true;
}

void evaluate(unsigned long now) {
  // ---------- Warm-up ----------
  if (warmup) {
    if (now - bootMs >= WARMUP_MS) {
      warmup = false;
      logEvent("Warm-up selesai");
    }
  }

  // ---------- Evaluasi gas ----------
  if (!warmup) {
    bool adaGas = gasDetected ? (gasAvg >= GAS_OFF) : (gasAvg >= GAS_ON);

    if (!adaGas) {
      // Gas sudah hilang
      if (gasDetected) {
        logEvent("Gas hilang (ADC=%d)", gasAvg);
        gasDetected     = false;
        worsening       = false;
        riseCount       = 0;
        gasStartTime    = 0;
        gasAtTrendStart = 0;
        lastTrendTime   = now;
      }
    } else {
      // Gas baru terdeteksi
      if (!gasDetected) {
        gasDetected     = true;
        gasStartTime    = now;
        lastTrendTime   = now;
        gasAtTrendStart = gasAvg;
        riseCount       = 0;
        logEvent("Gas terdeteksi! ADC=%d", gasAvg);
      }

      // Kunci BAHAYA: kadar kritis atau bertahan terlalu lama
      if (gasAvg >= GAS_CRITICAL) setLock("kadar gas kritis");
      if (now - gasStartTime >= PERSIST_LOCK_MS) setLock("gas bertahan terlalu lama");

      // Analisis tren per jendela waktu
      if (now - lastTrendTime >= TREND_WINDOW_MS) {
        int delta = gasAvg - gasAtTrendStart;

        if (delta >= TREND_RISE_MIN) {
          riseCount++;
          if (!worsening) {
            worsening = true;
            logEvent("Tren memburuk: %d -> %d (delta %+d)", gasAtTrendStart, gasAvg, delta);
          }
          if (riseCount >= TREND_CONFIRM) setLock("tren gas naik terus");
        } else {
          riseCount = 0;
          if (delta < 0) worsening = false;
        }

        lastTrendTime   = now;
        gasAtTrendStart = gasAvg;
      }
    }
  }

  // ---------- Log serial ----------
  Serial.printf("[LOG] t=%lus | ADC: %4d | Status: %-8s | Katup: %-5s | Listrik: %-5s | Alarm: %s | Net: %s\n",
                now / 1000, gasAvg, STATUS_STR[currentStatus()],
                valveOpen ? "BUKA" : "TUTUP",
                powerOn   ? "NYALA" : "PUTUS",
                buzzerOn  ? "ON" : "OFF",
                mqttOk ? "MQTT" : (wifiOk ? "WIFI" : "OFF"));
}

void cekTombolReset() {
  static bool stable = HIGH, lastRead = HIGH;
  static unsigned long lastChange = 0;

  bool r = digitalRead(PIN_BTN_RST);
  if (r != lastRead) {
    lastChange = millis();
    lastRead   = r;
  }

  // Debounce 30 ms
  if ((millis() - lastChange) > 30 && r != stable) {
    stable = r;
    if (stable == LOW) tryReset("tombol");
  }
}

// ============================================================
//  UPDATE OUTPUTS (TIERED SAFETY RESPONSE)
// ============================================================
void updateOutputs(unsigned long now) {
  // ---------- LED ----------
  digitalWrite(PIN_LED_G, !gasDetected && !bahayaLock && !warmup);
  digitalWrite(PIN_LED_R, gasDetected || bahayaLock);

  // ---------- Katup gas (servo) ----------
  bool wantValveOpen = (!warmup && !bahayaLock && !gasDetected && !worsening);
  if (wantValveOpen != valveOpen) {
    valveOpen = wantValveOpen;
    valveServo.write(valveOpen ? SERVO_OPEN : SERVO_CLOSED);
    logEvent(valveOpen ? "Katup Gas DIBUKA" : "Katup Gas DITUTUP");
  }

  // ---------- Listrik (relay) ----------
  bool wantPowerOn = (!warmup && !bahayaLock);
  if (wantPowerOn != powerOn) {
    powerOn = wantPowerOn;
    digitalWrite(PIN_RELAY, powerOn ? HIGH : LOW);
    logEvent(powerOn ? "Listrik NYALA" : "Listrik DIPUTUS TOTAL (Darurat)");
  }

  // ---------- Buzzer ----------
  bool on = false;
  if (bahayaLock)       on = (now % 300)  < 150;
  else if (gasDetected) on = (now % 1000) < 100;

  if (on != buzzerOn) {
    buzzerOn = on;
    if (on) tone(PIN_BUZZER, bahayaLock ? 2500 : 1500);
    else    noTone(PIN_BUZZER);
  }
}

void drawOled(unsigned long now) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SAFEGAS");
  display.setCursor(66, 0);
  display.printf("W:%s M:%s", wifiOk ? "OK" : "--", mqttOk ? "OK" : "--");

  // Status utama
  display.setTextSize(2);
  display.setCursor(0, 12);
  switch (currentStatus()) {
    case ST_WARMUP:   display.print("WARMUP");   break;
    case ST_NORMAL:   display.print("NORMAL");   break;
    case ST_ADA_GAS:  display.print("ADA GAS");  break;
    case ST_MEMBURUK: display.print("MEMBURUK"); break;
    case ST_BAHAYA:   display.print("BAHAYA");   break;
  }

  // Detail
  display.setTextSize(1);
  display.setCursor(0, 33);
  display.printf("Gas: %d", gasAvg);
  display.setCursor(0, 43);
  display.printf("Katup:%s Listrik:%s", valveOpen ? "BUKA" : "TUTUP", powerOn ? "ON" : "OFF");

  // Info jaringan
  display.setCursor(0, 53);
  if (wifiOk) display.print(WiFi.localIP().toString());
  else        display.print("Mencari WiFi...");

  display.display();
}

// ============================================================
//  TASK JARINGAN (core 0)
// ============================================================
void netTask(void*) {
  Serial.println("[WIFI] Memulai koneksi ke Wokwi-GUEST...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);

  unsigned long lastPrint = 0;

  // Timer koneksi dan publish MQTT
  unsigned long lastMqttTry = 0, lastMqttPub = 0;
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(60);

  // TLS aktif untuk pengujian Wokwi, tetapi sertifikat server belum diverifikasi.
  // Untuk production, gunakan CA certificate EMQX dan verifikasi sertifikat.
  espClient.setInsecure();

  for (;;) {
    wifiOk = (WiFi.status() == WL_CONNECTED);

    if (!wifiOk) {
      // ---------- Menunggu WiFi ----------
      if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        Serial.printf("[WIFI] Status: %d (Mencoba...)\n", WiFi.status());
      }
    } else {
      // ---------- WiFi tersambung ----------
      static bool printed = false;
      if (!printed) {
        Serial.println("\n[WIFI] ========================");
        Serial.println("[WIFI] BERHASIL TERHUBUNG!");
        Serial.print  ("[WIFI] IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.println("[WIFI] ========================\n");
        printed = true;
      }

      // ---------- Koneksi EMQX Cloud & publish ----------
      if (!mqtt.connected()) {
        mqttOk = false;

        if (millis() - lastMqttTry > 5000) {
          lastMqttTry = millis();
          Serial.println("[MQTT] Mencoba connect ke EMQX Cloud...");

          String cid = String(DEVICE_ID) + "-" + String((uint32_t)esp_random(), HEX);
          if (mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS)) {
            Serial.println("[MQTT] Terhubung ke EMQX Cloud!");
            mqttOk = true;
          } else {
            Serial.printf("[MQTT] Gagal, rc=%d\n", mqtt.state());
          }
        }
      } else {
        mqtt.loop();
        mqttOk = true;

        // Kirim event yang diantrekan oleh logEvent()
        MqttEventMessage eventMsg;
        while (mqttEventQueue != nullptr &&
               xQueueReceive(mqttEventQueue, &eventMsg, 0) == pdTRUE) {
          char eventJson[150];
          snprintf(eventJson, sizeof(eventJson),
                   "{\"device\":\"%s\",\"uptime_s\":%lu,\"msg\":\"%s\"}",
                   DEVICE_ID, millis() / 1000, eventMsg.msg);
          mqtt.publish(TOPIC_EVENT, eventJson);
        }

        // Publish telemetry tiap 1 detik
        if (millis() - lastMqttPub > 1000) {
          lastMqttPub = millis();

          char json[250];
          snprintf(json, sizeof(json),
                   "{\"device\":\"%s\",\"uptime_s\":%lu,\"adc\":%d,\"adc_raw\":%d,"
                   "\"status\":\"%s\",\"valve\":\"%s\",\"power\":\"%s\","
                   "\"alarm\":%s,\"lock\":%s,\"threshold\":%d}",
                   DEVICE_ID, millis() / 1000, gasAvg, gasRaw,
                   STATUS_STR[currentStatus()],
                   valveOpen  ? "OPEN" : "CLOSED",
                   powerOn    ? "ON" : "OFF",
                   buzzerOn   ? "true" : "false",
                   bahayaLock ? "true" : "false",
                   GAS_ON);

          if (!mqtt.publish(TOPIC_TELEMETRY, json)) {
            Serial.println("[MQTT] Publish telemetry gagal");
          }
        }
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ============================================================
//  SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SafeGas v12 Booting ===");

  // ---------- GPIO ----------
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BTN_RST, INPUT_PULLUP);

  // ---------- ADC ----------
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_GAS, ADC_11db);

  // ---------- Relay & servo (kondisi awal: aman/tertutup) ----------
  digitalWrite(PIN_RELAY, LOW);
  ESP32PWM::allocateTimer(0);
  valveServo.setPeriodHertz(50);
  valveServo.attach(PIN_SERVO, 500, 2400);
  valveServo.write(SERVO_CLOSED);

  // ---------- OLED ----------
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ERR] OLED tidak ditemukan");
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SafeGas booting...");
  display.display();

  // ---------- Queue event MQTT & task jaringan ----------
  mqttEventQueue = xQueueCreate(10, sizeof(MqttEventMessage));
  if (mqttEventQueue == nullptr) {
    Serial.println("[MQTT] Gagal membuat event queue");
  }

  xTaskCreatePinnedToCore(netTask, "net", 4096, NULL, 1, NULL, 0);

  bootMs = millis();
  Serial.println("Warm-up MQ-2...");
}

void loop() {
  unsigned long now = millis();

  cekTombolReset();

  if (now - tSample >= SAMPLE_MS) { tSample = now; sampleSensors(); }
  if (now - tEval   >= EVAL_MS)   { tEval   = now; evaluate(now); }

  updateOutputs(now);

  if (now - tOled >= OLED_MS) { tOled = now; drawOled(now); }
}
