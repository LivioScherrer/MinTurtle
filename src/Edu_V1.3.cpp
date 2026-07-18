// =====================================================
// EduBot V1.3R – Firmware für Xiao ESP32-S3 + DRV8834
// =====================================================
// Hardware: Seeed Studio Xiao ESP32-S3
// Motortreiber: DRV8834 (STEP/DIR)
// Farbsensor: TCS34725 (I2C)
// Kommunikation: BLE (Web Bluetooth API)
// =====================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <driver/timer.h>
#include "soc/gpio_struct.h"
#include <esp_task_wdt.h>
#include "mesh.h"
#include <esp_wifi.h>

// Function prototypes (Arduino-typ)
bool IRAM_ATTR stepperTimerISR(void *arg);
void initStepperTimer();
void setMotorSpeed(int motorIdx, float speed);
void stopMotors();
int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal);
const char* detectColorName(int r, int g, int b);
void setupColorSensor();
void resetI2CBus();
void colorSensorTask(void* parameter);
void sendStatus(const char* msg);
void sendColorToWebsite();
void clearKeys();
void hardStop();
void handleCommand(String cmd);
void processBleEvents();
float rampSpeed(float current, float target, float dt);
void updateDrive();
void saveConfigIfDirty();
void validateConfig();
void setup();
void loop();

// Mesh-Forward-Deklarationen
void onMeshMessage(const uint8_t* fromMac, uint8_t opcode, const char* text, size_t len);
void updateFormation(); // NEU
char g_robotName[16] = "EduBot-01";

// =====================================================
// PIN-KONFIGURATION (Xiao ESP32-S3)
// =====================================================
// DRV8834 Anschlüsse:
//   STEP_L  -> GPIO 4
//   DIR_L   -> GPIO 3
//   STEP_R  -> GPIO 2
//   DIR_R   -> GPIO 1
//   ENABLE  -> GPIO 7 (optional, LOW = aktiv)
// TCS34725  -> I2C (SDA=GPIO5, SCL=GPIO6)

#define PIN_DIR_L      3
#define PIN_STEP_L     4
#define PIN_DIR_R      1
#define PIN_STEP_R     2
#define PIN_ENABLE     7

#define STEP_BIT_L     (1UL << PIN_STEP_L)
#define STEP_BIT_R     (1UL << PIN_STEP_R)
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
#define SDA_PIN        5
#define SCL_PIN        6

// =====================================================
// KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE = 115200;
constexpr const char* FIRMWARE_VERSION = "EduBot V1.3";
constexpr int STACK_COLOR_SENSOR = 8192;
constexpr int BLE_QUEUE_LENGTH = 16;
constexpr int TASK_PRIORITY_SENSOR = 2;
constexpr int CORE_SENSOR = 0;

constexpr uint16_t COLOR_LOW = 40;
constexpr uint16_t COLOR_HIGH = 200;
constexpr float RATIO_RED = 1.35f;
constexpr float RATIO_GB = 1.25f;

constexpr uint32_t MAX_SPEED_DEFAULT = 900;
constexpr uint32_t MAX_SPEED_MIN = 200;
constexpr uint32_t MAX_SPEED_MAX = 2500;
constexpr float TURN_DEFAULT = 0.25f;
constexpr float TURN_MIN = 0.0f;
constexpr float TURN_MAX = 0.9f;
constexpr float ACCEL_DEFAULT = 5000.0f;
constexpr float ACCEL_MIN = 100.0f;
constexpr float ACCEL_MAX = 10000.0f;

constexpr uint32_t STEP_TIMER_US = 10;
constexpr uint32_t MIN_STEP_INTERVAL = 50;
constexpr uint32_t MAX_STEP_INTERVAL = 50000;
constexpr uint32_t STEP_PERIOD_US = 1000000UL;

constexpr uint32_t FRAC_SCALE = 0x10000UL;

constexpr unsigned long COLOR_NOTIFY_MS = 200;
constexpr uint32_t LOOP_DELAY_MS = 5;

// =====================================================
// KONFIGURATION (wird in NVS gespeichert)
// =====================================================
struct Config {
  float maxSpeed = 900.0f;
  float turnFactor = 0.25f;
  float turnOnlySpeed = 550.0f;
  int8_t leftSign = -1;
  int8_t rightSign = -1;
  float accelRate = 5000.0f;
  uint32_t commandTimeoutMs = 500;
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
  uint16_t yellowRmin = 160, yellowGmin = 120, yellowBmax = 120;
};

Config config;
Preferences prefs;

// =====================================================
// HARDWARE-TIMER FÜR STEPPER (Bresenham)
// =====================================================
// Timer läuft mit 1 MHz (APB 80 MHz / 80 = 1 MHz)
// ISR feuert alle STEP_TIMER_US = 10 µs
// Q16.16 Bresenham-Akkumulator für maximale Präzision
// ISR setzt STEP HIGH, nächster Aufruf cleared → Pulsbreite = 10 µs

uint32_t stepFracAddL = 0;
uint32_t stepFracAddR = 0;
uint32_t stepFracAccumL = 0;
uint32_t stepFracAccumR = 0;
uint32_t stepIntervalL = 0;
uint32_t stepIntervalR = 0;

static uint8_t stepPhaseL = 0;
static uint8_t stepPhaseR = 0;

bool IRAM_ATTR stepperTimerISR(void *arg) {
  if (stepPhaseL == 1) { GPIO.out_w1tc = STEP_BIT_L; stepPhaseL = 0; }
  if (stepPhaseR == 1) { GPIO.out_w1tc = STEP_BIT_R; stepPhaseR = 0; }

  if (stepIntervalL > 0) {
    stepFracAccumL += stepFracAddL;
    if (stepFracAccumL >= FRAC_SCALE) {
      stepFracAccumL -= FRAC_SCALE;
      GPIO.out_w1ts = STEP_BIT_L;
      stepPhaseL = 1;
    }
  } else {
    stepFracAccumL = 0;
  }

  if (stepIntervalR > 0) {
    stepFracAccumR += stepFracAddR;
    if (stepFracAccumR >= FRAC_SCALE) {
      stepFracAccumR -= FRAC_SCALE;
      GPIO.out_w1ts = STEP_BIT_R;
      stepPhaseR = 1;
    }
  } else {
    stepFracAccumR = 0;
  }
  return false;
}

void initStepperTimer() {
  timer_config_t cfg = {};
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.counter_en = TIMER_PAUSE;
  cfg.intr_type = TIMER_INTR_LEVEL;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.divider = 80;

  timer_init(TIMER_GROUP_0, TIMER_0, &cfg);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, STEP_TIMER_US);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, stepperTimerISR, NULL, ESP_INTR_FLAG_IRAM);
  timer_start(TIMER_GROUP_0, TIMER_0);
}

// =====================================================
// MOTOR-ANSTEUERUNG (DRV8834)
// =====================================================

void setMotorSpeed(int motorIdx, float speed) {
  portENTER_CRITICAL(&stepperMux);
  const int dirPin = (motorIdx == 0) ? PIN_DIR_L : PIN_DIR_R;
  const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;

  float actualSpeed = speed * static_cast<float>(sign);

  if (actualSpeed > 0.5f) {
    digitalWrite(dirPin, HIGH);
  } else if (actualSpeed < -0.5f) {
    digitalWrite(dirPin, LOW);
    actualSpeed = -actualSpeed;
  } else {
    if (motorIdx == 0) { stepIntervalL = 0; stepFracAddL = 0; stepFracAccumL = 0; }
    else { stepIntervalR = 0; stepFracAddR = 0; stepFracAccumR = 0; }
    GPIO.out_w1tc = (motorIdx == 0) ? STEP_BIT_L : STEP_BIT_R;
    portEXIT_CRITICAL(&stepperMux);
    return;
  }

  uint32_t interval;
  uint32_t actualSpeedInt = static_cast<uint32_t>(actualSpeed);
  if (actualSpeedInt < 1) actualSpeedInt = 1;
  interval = STEP_PERIOD_US / actualSpeedInt;
  if (interval < MIN_STEP_INTERVAL) interval = MIN_STEP_INTERVAL;
  if (interval > MAX_STEP_INTERVAL) interval = MAX_STEP_INTERVAL;

  uint32_t fracAdd = (STEP_TIMER_US * FRAC_SCALE) / interval;

  if (motorIdx == 0) {
    stepIntervalL = interval;
    stepFracAddL = fracAdd;
    stepFracAccumL = 0;
  } else {
    stepIntervalR = interval;
    stepFracAddR = fracAdd;
    stepFracAccumR = 0;
  }
  portEXIT_CRITICAL(&stepperMux);
}

void stopMotors() {
  portENTER_CRITICAL(&stepperMux);
  stepIntervalL = 0;
  stepIntervalR = 0;
  stepFracAddL = 0;
  stepFracAddR = 0;
  stepFracAccumL = 0;
  stepFracAccumR = 0;

  GPIO.out_w1tc = STEP_BIT_L | STEP_BIT_R;
  portEXIT_CRITICAL(&stepperMux);
}

// =====================================================
// FARBSENSOR (TCS34725)
// =====================================================

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

volatile int sensorR = 0;
volatile int sensorG = 0;
volatile int sensorB = 0;
volatile uint16_t rawSensorR = 0, rawSensorG = 0, rawSensorB = 0, rawSensorC = 0;
volatile char sensorName[16] = "UNBEKANNT";

SemaphoreHandle_t colorSem = nullptr;
bool colorSensorOk = false;

// Kalibrierung: Rohwerte -> 0-255
int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal) {
  if (whiteVal <= blackVal) return 0;
  float value = (static_cast<float>(raw) - blackVal) * 255.0f / (whiteVal - blackVal);
  if (value < 0.0f) value = 0.0f;
  if (value > 255.0f) value = 255.0f;
  return static_cast<int>(value + 0.5f);
}

const char* detectColorName(int r, int g, int b) {
  if (r < static_cast<int>(COLOR_LOW) && g < static_cast<int>(COLOR_LOW) && b < static_cast<int>(COLOR_LOW)) return "SCHWARZ";
  if (r > static_cast<int>(COLOR_HIGH) && g > static_cast<int>(COLOR_HIGH) && b > static_cast<int>(COLOR_HIGH)) return "WEISS";
  if (r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  if (r > config.yellowRmin && g > config.yellowGmin && b < config.yellowBmax) return "GELB";
  return "UNBEKANNT";
}

void setupColorSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000); // Mehr Zeit zum Initialisieren
  
  if (tcs.begin()) {
    Serial.println("[SENSOR] TCS34725 gefunden");
    colorSensorOk = true;
  } else {
    Serial.println("[SENSOR] TCS34725 NICHT gefunden!");
    colorSensorOk = false;
  }
  colorSem = xSemaphoreCreateBinary();
  if (colorSem) xSemaphoreGive(colorSem);
}

void resetI2CBus() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SDA_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void colorSensorTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(120);
  int failCount = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);

      if (r == 0 && g == 0 && b == 0 && c == 0) {
        failCount++;
        if (failCount >= 3) {
          colorSensorOk = false;
          Serial.println("[SENSOR] 3x Null-Daten -> Sensor als offline markiert");
        }
      } else {
        failCount = 0;
        rawSensorR = r; rawSensorG = g; rawSensorB = b; rawSensorC = c;

        if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(10)) == pdTRUE) {
          int calR = calibrateColor(r, config.blackR, config.whiteR);
          int calG = calibrateColor(g, config.blackG, config.whiteG);
          int calB = calibrateColor(b, config.blackB, config.whiteB);

          sensorR = calR; sensorG = calG; sensorB = calB;
          strlcpy((char*)sensorName, detectColorName(calR, calG, calB), sizeof(sensorName));
          xSemaphoreGive(colorSem);
        }
      }
    } else {
      Serial.println("[SENSOR] Recovery attempt...");
      resetI2CBus();
      if (tcs.begin()) {
        colorSensorOk = true;
        failCount = 0;
        Serial.println("[SENSOR] Recovered!");
      } else {
        Serial.println("[SENSOR] Recovery fehlgeschlagen");
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
      lastWakeTime = xTaskGetTickCount();
      continue;
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// =====================================================
// BLE UUIDs (müssen mit HTML-Frontend übereinstimmen)
// =====================================================

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CMD_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define STATUS_UUID  "a9b1c2d3-1111-2222-3333-abcdefabcdef"

BLECharacteristic* statusChar = nullptr;
bool deviceConnected = false;

// =====================================================
// BLE EVENT-QUEUE (ISR-sicher)
// =====================================================

enum BleEventType : uint8_t {
  BLE_EVT_CONNECTED,
  BLE_EVT_DISCONNECTED,
  BLE_EVT_COMMAND
};

typedef struct {
  BleEventType type;
  char cmd[64];
} BleEvent;

QueueHandle_t bleEventQueue = nullptr;

// =====================================================
// STEUERUNGSZUSTAND
// =====================================================

bool keyW = false;
bool keyA = false;
bool keyS = false;
bool keyD = false;

float speedL = 0.0;
float speedR = 0.0;

bool formationMode = false; // RSSI-basiertes Zusammenfinden
bool isLeaderMode = false;
bool isFollowerMode = false;

bool configDirty = false; // Flag für NVS-Speicherung
unsigned long lastCommandTime = 0;
unsigned long lastColorNotify = 0;
unsigned long lastUpdateMicros = 0;

// =====================================================
// RACE / QUIZ Zustand
// =====================================================
int raceScore = 0;
unsigned long raceDriveUntil = 0;

enum QuizState { QUIZ_IDLE, QUIZ_DRIVE, QUIZ_READ, QUIZ_DONE };
QuizState quizState = QUIZ_IDLE;
String quizExpected[10];
int quizExpectedLen = 0;
int quizIndex = 0;
String quizReadColors[10];
unsigned long quizStepTime = 0;
int quizStepMs = 800; // ms pro Farbe

// =====================================================
// BLE-NACHRICHTEN
// =====================================================

void sendStatus(const char* msg) {
  Serial.printf("[BLE] %s\n", msg);
  if (statusChar != nullptr && deviceConnected) {
    statusChar->setValue(msg);
    statusChar->notify();
  }
}

void sendColorToWebsite() {
  int r = 0, g = 0, b = 0;
  char name[16] = {};

  if (colorSem == nullptr) return;
  if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(5)) == pdTRUE) {
    r = sensorR;
    g = sensorG;
    b = sensorB;
    strlcpy(name, (const char*)sensorName, sizeof(name));
    xSemaphoreGive(colorSem);
  } else {
    return;
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "COLOR:%d,%d,%d,%s", r, g, b, name);
  sendStatus(msg);

  // Optional: Farbereignis an Mesh weiterleiten (deaktivierbar)
  EduMesh::sendColor(r, g, b, name);
}

// =====================================================
// COMMAND-HANDLING
// =====================================================

void clearKeys() {
  keyW = false;
  keyA = false;
  keyS = false;
  keyD = false;
}

void hardStop() {
  clearKeys();
  speedL = 0.0;
  speedR = 0.0;
  stopMotors();
  sendStatus("STOP");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

  Serial.printf("[CMD] %s\n", cmd.c_str());

  if (cmd.startsWith("KEYS:")) {
    String data = cmd.substring(5);
    if (data.length() >= 4) {
      keyW = (data.charAt(0) == '1');
      keyA = (data.charAt(1) == '1');
      keyS = (data.charAt(2) == '1');
      keyD = (data.charAt(3) == '1');
    }
    if (isLeaderMode) EduMesh::sendCmd(cmd.c_str(), cmd.length());
    return;
  }

  if (cmd == "STOP") {
    hardStop();
    return;
  }

  if (cmd.startsWith("MAX:")) {
    float v = cmd.substring(4).toFloat();
    if (v >= MAX_SPEED_MIN && v <= MAX_SPEED_MAX) {
      config.maxSpeed = v;
      configDirty = true;
      char buf[32];
      snprintf(buf, sizeof(buf), "MAX=%.0f", config.maxSpeed);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("TURN:")) {
    float t = cmd.substring(5).toFloat();
    if (t >= TURN_MIN && t <= TURN_MAX) {
      config.turnFactor = t;
      configDirty = true;
      char buf[32];
      snprintf(buf, sizeof(buf), "TURN=%.2f", config.turnFactor);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("ACC:")) {
    float a = cmd.substring(4).toFloat();
    if (a >= ACCEL_MIN && a <= ACCEL_MAX) {
      config.accelRate = a;
      configDirty = true;
      char buf[32];
      snprintf(buf, sizeof(buf), "ACC=%.0f", config.accelRate);
      sendStatus(buf);
    }
    return;
  }

  if (cmd == "CAL:BLACK") {
    config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
    configDirty = true;
    sendStatus("CAL:BLACK OK");
    return;
  }

  if (cmd == "CAL:WHITE") {
    config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
    configDirty = true;
    sendStatus("CAL:WHITE OK");
    return;
  }

  // --- Mesh-Kommandos ---

  if (cmd.startsWith("MESH:NAME:")) {
    String name = cmd.substring(10);
    if (name.length() > 0 && name.length() < sizeof(g_robotName)) {
      strlcpy(g_robotName, name.c_str(), sizeof(g_robotName));
      prefs.begin("edubot", false);
      prefs.putString("robotName", String(g_robotName));
      prefs.end();
      char buf[48];
      snprintf(buf, sizeof(buf), "MESH:NAME=%s", g_robotName);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("MESHMSG:")) {
    String text = cmd.substring(8);
    bool ok = EduMesh::sendMsg(text.c_str(), text.length());
    sendStatus(ok ? "MESHMSG:OK" : "MESHMSG:FAIL");
    return;
  }

  if (cmd.startsWith("MESHSEND:")) {
    // Format: MESHSEND:AA:BB:CC:DD:EE:FF,Nachricht
    String rest = cmd.substring(9);
    int comma = rest.indexOf(',');
    if (comma < 0) { sendStatus("MESHSEND:FORMAT"); return; }
    String macStr = rest.substring(0, comma);
    String text = rest.substring(comma + 1);

    uint8_t mac[6];
    if (sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
      sendStatus("MESHSEND:BADMAC");
      return;
    }
    // Direkt an Peer senden - sicherstellen, dass Peer existiert
    if (!esp_now_is_peer_exist(mac)) {
      esp_now_peer_info_t info = {};
      memcpy(info.peer_addr, mac, 6);
      info.channel = EduMesh::MESH_CHANNEL;
      info.encrypt = false;
      esp_now_add_peer(&info);
    }
    bool ok = EduMesh::sendMsg(text.c_str(), text.length());
    sendStatus(ok ? "MESHSEND:OK" : "MESHSEND:FAIL");
    return;
  }

  if (cmd == "MESHPEERS") {
    EduMesh::Peer peers[EduMesh::MESH_MAX_PEERS];
    size_t count = 0;
    EduMesh::getPeers(peers, EduMesh::MESH_MAX_PEERS, &count);
    char buf[64];
    snprintf(buf, sizeof(buf), "MESHPEERS:%u", (unsigned)count);
    sendStatus(buf);
    for (size_t i = 0; i < count; i++) {
      char line[80];
      unsigned age = (millis() - peers[i].lastSeen) / 1000;
      snprintf(line, sizeof(line), "PEER:%s,%s,%us",
               EduMesh::macString(peers[i].mac).c_str(),
               peers[i].name, age);
      sendStatus(line);
    }
    return;
  }

  if (cmd == "MESHPING") {
    bool ok = EduMesh::sendPing(nullptr);
    sendStatus(ok ? "MESHPING:OK" : "MESHPING:FAIL");
    return;
  }

  if (cmd == "FORMATION:ON") {
    formationMode = true;
    sendStatus("FORMATION:ON");
    return;
  }

  if (cmd == "FORMATION:OFF") {
    formationMode = false;
    clearKeys();
    sendStatus("FORMATION:OFF");
    return;
  }

  if (cmd == "MODE:LEADER") {
    isLeaderMode = true; isFollowerMode = false; formationMode = false;
    clearKeys();
    sendStatus("MODE:LEADER");
    return;
  }
  if (cmd == "MODE:FOLLOWER") {
    isFollowerMode = true; isLeaderMode = false; formationMode = false;
    clearKeys();
    sendStatus("MODE:FOLLOWER");
    return;
  }
  if (cmd == "MODE:NORMAL") {
    isLeaderMode = false; isFollowerMode = false; formationMode = false;
    clearKeys();
    sendStatus("MODE:NORMAL");
    return;
  }

  // === MATHE-RENNEN ===
  if (cmd.startsWith("RACE:PROGRESS:")) {
    String scoreStr = cmd.substring(14);
    int s = scoreStr.toInt();
    if (s >= 0 && s <= 10) {
      raceScore = s;
      // Fahre proportional zur Punktzahl (0=nichts, 10=max)
      float fraction = raceScore / 10.0f;
      int driveMs = (int)(fraction * 2000);
      if (driveMs > 0) {
        keyW = true; keyA = false; keyS = false; keyD = false;
        raceDriveUntil = millis() + driveMs;
      }
      // An Mesh weiterleiten
      EduMesh::sendCmd(cmd.c_str(), cmd.length());
    }
    sendStatus(cmd.c_str());
    return;
  }

  if (cmd == "RACE:WIN") {
    raceScore = 10;
    EduMesh::sendCmd(cmd.c_str(), cmd.length());
    sendStatus("RACE:WIN");
    return;
  }

  // === FARBEN-QUIZ ===
  if (cmd.startsWith("QUIZ:CHECK:")) {
    String seq = cmd.substring(11);
    quizExpectedLen = 0;
    quizIndex = 0;
    // Parse: "ROT,GRUEN,BLAU,GELB,ROT"
    int start = 0;
    while (start < (int)seq.length() && quizExpectedLen < 10) {
      int comma = seq.indexOf(',', start);
      if (comma < 0) comma = seq.length();
      String c = seq.substring(start, comma);
      c.trim();
      if (c.length() > 0) {
        quizExpected[quizExpectedLen] = c;
        quizReadColors[quizExpectedLen] = "";
        quizExpectedLen++;
      }
      start = comma + 1;
    }
    if (quizExpectedLen > 0) {
      quizState = QUIZ_DRIVE;
      quizStepTime = millis();
      clearKeys();
      sendStatus("QUIZ:START");
    } else {
      sendStatus("QUIZ:EMPTY");
    }
    return;
  }

  if (cmd == "QUIZ:STOP") {
    quizState = QUIZ_IDLE;
    hardStop();
    sendStatus("QUIZ:STOP");
    return;
  }

  char buf[80];
  snprintf(buf, sizeof(buf), "UNKNOWN:%s", cmd.c_str());
  sendStatus(buf);
}

// =====================================================
// BLE CALBACKS
// =====================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    BleEvent evt = { BLE_EVT_CONNECTED, "" };
    if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
  }
  void onDisconnect(BLEServer* server) override {
    BleEvent evt = { BLE_EVT_DISCONNECTED, "" };
    if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string rxValue = characteristic->getValue().c_str();
    if (!rxValue.empty()) {
      BleEvent evt = { BLE_EVT_COMMAND, "" };
      strlcpy(evt.cmd, rxValue.c_str(), sizeof(evt.cmd));
      if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
    }
  }
};

// =====================================================
// EVENT-VERARBEITUNG (wird in loop() aufgerufen)
// =====================================================

void processBleEvents() {
  BleEvent evt;
  while (bleEventQueue && xQueueReceive(bleEventQueue, &evt, 0) == pdTRUE) {
    switch (evt.type) {
      case BLE_EVT_CONNECTED:
        deviceConnected = true;
        lastCommandTime = millis();
        Serial.println("[BLE] Verbunden");
        char buf[48];
        snprintf(buf, sizeof(buf), "%s bereit", FIRMWARE_VERSION);
        sendStatus(buf);
        break;
      case BLE_EVT_DISCONNECTED:
        deviceConnected = false;
        clearKeys();
        speedL = 0.0;
        speedR = 0.0;
        stopMotors();
        Serial.println("[BLE] Getrennt -> Werbung neu gestartet");
        BLEDevice::startAdvertising();
        break;
      case BLE_EVT_COMMAND:
        handleCommand(String(evt.cmd));
        break;
    }
  }
}

// =====================================================
// RAMPE (sanftes Beschleunigen)
// =====================================================

float rampSpeed(float current, float target, float dt) {
  float diff = target - current;
  if (diff < 1.0f && diff > -1.0f) {
    return target;
  }
  float maxChange = config.accelRate * dt;
  if (diff > 0) {
    current += maxChange;
    if (current > target) current = target;
  } else {
    current -= maxChange;
    if (current < target) current = target;
  }
  return current;
}

// =====================================================
// FAHRLOGIK
// =====================================================

void updateDrive() {
  float targetL = 0.0;
  float targetR = 0.0;

  bool forward  = keyW && !keyS;
  bool backward = keyS && !keyW;
  bool left     = keyA && !keyD;
  bool right    = keyD && !keyA;

  if (forward) {
    targetL = config.maxSpeed;
    targetR = config.maxSpeed;
  } else if (backward) {
    targetL = -config.maxSpeed;
    targetR = -config.maxSpeed;
  }

  if (forward || backward) {
    if (left)  targetL *= config.turnFactor;
    if (right) targetR *= config.turnFactor;
  }

  if (!forward && !backward) {
    if (left) {
      targetL = 0;
      targetR = config.turnOnlySpeed;
    } else if (right) {
      targetL = config.turnOnlySpeed;
      targetR = 0;
    }
  }

  // Timeout: Kein BLE-Befehl mehr -> Stop
  if (millis() - lastCommandTime > config.commandTimeoutMs) {
    targetL = 0.0;
    targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0;
      speedR = 0.0;
      clearKeys();
      stopMotors();
    }
    lastUpdateMicros = micros();
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastUpdateMicros) / 1000000.0;
  lastUpdateMicros = now;
  if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;

  float newSpeedL = rampSpeed(speedL, targetL, dt);
  if (newSpeedL != speedL) {
    speedL = newSpeedL;
    setMotorSpeed(0, speedL);
  }

  float newSpeedR = rampSpeed(speedR, targetR, dt);
  if (newSpeedR != speedR) {
    speedR = newSpeedR;
    setMotorSpeed(1, speedR);
  }
}

void validateConfig() {
  bool changed = false;

  if (config.maxSpeed < MAX_SPEED_MIN || config.maxSpeed > MAX_SPEED_MAX) {
    config.maxSpeed = static_cast<float>(MAX_SPEED_DEFAULT); changed = true;
  }
  if (config.turnFactor < TURN_MIN || config.turnFactor > TURN_MAX) {
    config.turnFactor = TURN_DEFAULT; changed = true;
  }
  if (config.accelRate < ACCEL_MIN || config.accelRate > ACCEL_MAX) {
    config.accelRate = ACCEL_DEFAULT; changed = true;
  }
  if (config.turnOnlySpeed < 50.0f || config.turnOnlySpeed > MAX_SPEED_MAX) {
    config.turnOnlySpeed = 550.0f; changed = true;
  }
  if (config.leftSign != -1 && config.leftSign != 1) {
    config.leftSign = -1; changed = true;
  }
  if (config.rightSign != -1 && config.rightSign != 1) {
    config.rightSign = -1; changed = true;
  }
  if (config.commandTimeoutMs < 100 || config.commandTimeoutMs > 10000) {
    config.commandTimeoutMs = 500; changed = true;
  }
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152;  config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }

  if (changed) {
    configDirty = true;
    Serial.println("[NVS] Invalid config corrected");
  }
}

void saveConfigIfDirty() {
  if (configDirty) {
    prefs.begin("edubot", false);
    prefs.putFloat("maxSpeed", config.maxSpeed);
    prefs.putFloat("turnFactor", config.turnFactor);
    prefs.putFloat("accelRate", config.accelRate);
    prefs.putUShort("blackR", config.blackR);
    prefs.putUShort("blackG", config.blackG);
    prefs.putUShort("blackB", config.blackB);
    prefs.putUShort("whiteR", config.whiteR);
    prefs.putUShort("whiteG", config.whiteG);
    prefs.putUShort("whiteB", config.whiteB);
    prefs.end();
    configDirty = false;
    Serial.println("[NVS] Config saved");
  }
}

// =====================================================
// MESH (ESP-NOW Inter-Roboter Kommunikation)
// =====================================================

void onMeshMessage(const uint8_t* fromMac, uint8_t opcode, const char* text, size_t len) {
  String fromStr = EduMesh::macString(fromMac);
  switch (opcode) {
    case EduMesh::MESH_OP_HEARTBEAT:
      // Stille Pflege; Peers werden in mesh.cpp verwaltet
      break;
    case EduMesh::MESH_OP_MSG: {
      char buf[96];
      snprintf(buf, sizeof(buf), "MESH_MSG:%s,%s", fromStr.c_str(), text);
      Serial.printf("[MESH] MSG von %s: %s\n", fromStr.c_str(), text);
      sendStatus(buf);
      break;
    }
    case EduMesh::MESH_OP_CMD: {
      // Weitergeleitetes Kommando lokal ausfuehren
      Serial.printf("[MESH] CMD von %s: %s\n", fromStr.c_str(), text);
      if (isFollowerMode && bleEventQueue) {
        BleEvent evt = { BLE_EVT_COMMAND, "" };
        strlcpy(evt.cmd, text, sizeof(evt.cmd));
        xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
      }
      break;
    }
    case EduMesh::MESH_OP_COLOR: {
      // Farbereignis weiterleiten
      Serial.printf("[MESH] COLOR von %s: %s\n", fromStr.c_str(), text);
      char buf[96];
      snprintf(buf, sizeof(buf), "MESH_COLOR:%s,%s", fromStr.c_str(), text);
      sendStatus(buf);
      break;
    }
    case EduMesh::MESH_OP_PING: {
      Serial.printf("[MESH] PING von %s\n", fromStr.c_str());
      char buf[48];
      snprintf(buf, sizeof(buf), "MESH_PING:%s", fromStr.c_str());
      sendStatus(buf);
      break;
    }
    case EduMesh::MESH_OP_PONG: {
      Serial.printf("[MESH] PONG von %s\n", fromStr.c_str());
      break;
    }
    default:
      Serial.printf("[MESH] unbekannt opcode=%u von %s\n", opcode, fromStr.c_str());
      break;
  }
}

// =====================================================
// FORMATIONS-LOGIK (Peer-basiert)
// =====================================================
void updateFormation() {
  // Wenn Tasten gedrückt sind, manuelle Steuerung priorisieren
  if (keyW || keyA || keyS || keyD) return;

  EduMesh::Peer peers[EduMesh::MESH_MAX_PEERS];
  size_t count = 0;
  EduMesh::getPeers(peers, EduMesh::MESH_MAX_PEERS, &count);

  if (count == 0) {
    // Keine Peers in Reichweite: stehend suchen (nicht fahren)
    keyW = false;
    return;
  }

  // Peer gefunden: wir sind "zusammen"
  keyW = false;
}

// =====================================================

void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V1.3 – Xiao ESP32-S3 + DRV8834");
  Serial.println("======================================");

    // esp_task_wdt_config_t wdt_config = {
  //   .timeout_ms = 10000,
  //   .idle_core_mask = (1 << CONFIG_NUM_CORES) - 1, 
  //   .trigger_panic = true,
  // };
  // esp_task_wdt_init(&wdt_config);
  // esp_task_wdt_add(NULL);
  
  esp_task_wdt_init(20, true);
  esp_task_wdt_add(NULL);

  // NVS-Konfiguration laden und validieren
  prefs.begin("edubot", false);
  config.maxSpeed      = prefs.getFloat("maxSpeed", static_cast<float>(MAX_SPEED_DEFAULT));
  config.turnFactor    = prefs.getFloat("turnFactor", TURN_DEFAULT);
  config.turnOnlySpeed = prefs.getFloat("turnOnlySpeed", 550.0f);
  config.accelRate     = prefs.getFloat("accelRate", ACCEL_DEFAULT);
  config.blackR        = prefs.getUShort("blackR", 152);
  config.blackG        = prefs.getUShort("blackG", 137);
  config.blackB        = prefs.getUShort("blackB", 98);
  config.whiteR        = prefs.getUShort("whiteR", 4143);
  config.whiteG        = prefs.getUShort("whiteG", 4258);
  config.whiteB        = prefs.getUShort("whiteB", 3036);
  String savedName    = prefs.getString("robotName", "EduBot-01");
  strlcpy(g_robotName, savedName.c_str(), sizeof(g_robotName));
  Serial.printf("[NVS] maxSpeed=%.0f accelRate=%.0f turnFactor=%.2f name=%s\n",
                config.maxSpeed, config.accelRate, config.turnFactor, g_robotName);
  validateConfig();
  saveConfigIfDirty();
  prefs.end();

  // Pins initialisieren
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  digitalWrite(PIN_STEP_L, LOW);
  digitalWrite(PIN_STEP_R, LOW);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);
  digitalWrite(PIN_ENABLE, LOW); // DRV8834 aktivieren

  GPIO.out_w1tc = STEP_BIT_L | STEP_BIT_R;

  lastCommandTime = millis();
  lastUpdateMicros = micros();

  // Hardware-Timer starten
  initStepperTimer();
  Serial.println("[TIMER] Hardware-Timer gestartet (10 µs)");

  // Farbsensor
  setupColorSensor();
  if (colorSensorOk) {
    xTaskCreatePinnedToCore(
      colorSensorTask,
      "ColorSensor",
      STACK_COLOR_SENSOR,
      NULL,
      TASK_PRIORITY_SENSOR,
      NULL,
      CORE_SENSOR
    );
    Serial.println("[TASK] ColorSensor-Task gestartet (Core 0)");
  }

  // BLE-Queue
  bleEventQueue = xQueueCreate(BLE_QUEUE_LENGTH, sizeof(BleEvent));
  if (bleEventQueue == NULL) {
    Serial.println("[BLE] Fehler: Queue konnte nicht erstellt werden!");
  }

  // BLE VOR WiFi initialisieren (Espressif-Empfehlung fuer Koexistenz)
  BLEDevice::init("EduBot");

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* cmdChar = service->createCharacteristic(
    CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  cmdChar->setCallbacks(new CommandCallbacks());

  statusChar = service->createCharacteristic(
    STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  statusChar->addDescriptor(new BLE2902());
  statusChar->setValue(FIRMWARE_VERSION);

  service->start();

  BLEAdvertising* advertising = server->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("[BLE] Werbung gestartet (EduBot)");

  // WiFi/Mesh NACH BLE starten
  WiFi.mode(WIFI_STA);
  delay(100);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

  if (EduMesh::begin(g_robotName, onMeshMessage)) {
    Serial.println("[MESH] ESP-NOW aktiv");
  } else {
    Serial.println("[MESH] ESP-NOW NICHT aktivierbar");
  }

  // WICHTIG: Advertising nach WiFi-Init neu starten
  // WiFi.mode() kann die BLE-Advertising auf ESP32-S3 stoeren
  advertising->start();
  Serial.println("[BLE] Advertising erneut gestartet nach WiFi-Init");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  esp_task_wdt_reset();

  // BLE-Events verarbeiten (wichtig: wurde in V1.2 vergessen!)
  processBleEvents();

  // Config in NVS speichern (nur wenn nötig, entkoppelt von BLE-Befehlen)
  if (configDirty) {
    saveConfigIfDirty();
  }

  // Fahrlogik aktualisieren
  updateDrive();

  // Farbsensor-Daten senden (~10 Hz)
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }

  // Mesh-Heartbeat und Peer-Pflege
  EduMesh::loop();

  // Formation: Roboter finden sich gegenseitig (RSSI-basiert)
  if (formationMode) {
    updateFormation();
  }

  // Quiz-Statusmaschine (non-blocking)
  if (quizState == QUIZ_DRIVE) {
    unsigned long elapsed = millis() - quizStepTime;
    if (elapsed < (unsigned long)quizStepMs) {
      // Vorwaerts fahren
      keyW = true; keyA = false; keyS = false; keyD = false;
    } else {
      // Anhalten und Farbe lesen
      keyW = false;
      char colorName[16];
      if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(5)) == pdTRUE) {
        strlcpy(colorName, (const char*)sensorName, sizeof(colorName));
        xSemaphoreGive(colorSem);
      } else {
        strlcpy(colorName, "UNBEKANNT", sizeof(colorName));
      }
      Serial.printf("[QUIZ] Tile %d: %s (erwartet: %s)\n", quizIndex, colorName, quizExpected[quizIndex].c_str());
      quizReadColors[quizIndex] = String(colorName);
      quizIndex++;
      quizStepTime = millis();
      if (quizIndex >= quizExpectedLen) {
        quizState = QUIZ_DONE;
        // Ergebnis auswerten
        int correct = 0;
        for (int i = 0; i < quizExpectedLen; i++) {
          if (quizReadColors[i] == quizExpected[i]) correct++;
        }
        char result[64];
        if (correct == quizExpectedLen) {
          snprintf(result, sizeof(result), "QUIZ:RESULT:OK,%d,%d", correct, quizExpectedLen);
        } else {
          int firstWrong = -1;
          for (int i = 0; i < quizExpectedLen; i++) {
            if (quizReadColors[i] != quizExpected[i]) { firstWrong = i; break; }
          }
          snprintf(result, sizeof(result), "QUIZ:RESULT:WRONG,%s,%s,%d,%d",
                   quizExpected[firstWrong].c_str(), quizReadColors[firstWrong].c_str(),
                   correct, quizExpectedLen);
        }
        sendStatus(result);
        EduMesh::sendCmd(result, strlen(result));
        hardStop();
      } else {
        quizState = QUIZ_DRIVE;
      }
    }
  }

  // Race-Drive-Timeout
  if (raceDriveUntil > 0 && millis() > raceDriveUntil) {
    raceDriveUntil = 0;
    if (quizState == QUIZ_IDLE && !formationMode) {
      clearKeys();
    }
  }

  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
