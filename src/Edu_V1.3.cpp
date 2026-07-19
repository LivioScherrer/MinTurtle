// =====================================================
// EduBot V1.3R - Firmware fuer Xiao ESP32-S3 + DRV8834
// =====================================================
// Hardware:       Seeed Studio Xiao ESP32-S3
// Motortreiber:   DRV8834 (STEP/DIR)
// Farbsensor:     TCS34725 (I2C)
// Kommunikation:  BLE (Web Bluetooth API)
// =====================================================
// BLE-Architektur: 1:1 uebernommen aus der funktionierenden
// .ino-Version. Event-Queue entkoppelt onWrite/onConnect/onDisconnect
// vom loop(). sendStatus macht notify direkt (wie im .ino).
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

// =====================================================
// PIN-KONFIGURATION (Xiao ESP32-S3)
// =====================================================
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

constexpr unsigned long COLOR_NOTIFY_MS = 100;
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
};

Config config;
Preferences prefs;

// =====================================================
// HARDWARE-TIMER FUER STEPPER (Bresenham)
// =====================================================
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
    if (stepFracAccumL >= 0x10000UL) {
      stepFracAccumL -= 0x10000UL;
      GPIO.out_w1ts = STEP_BIT_L;
      stepPhaseL = 1;
    }
  } else {
    stepFracAccumL = 0;
  }

  if (stepIntervalR > 0) {
    stepFracAccumR += stepFracAddR;
    if (stepFracAccumR >= 0x10000UL) {
      stepFracAccumR -= 0x10000UL;
      GPIO.out_w1ts = STEP_BIT_R;
      stepPhaseR = 1;
    }
  } else {
    stepFracAccumR = 0;
  }
  return false;
}

// =====================================================
// TIMER INIT
// =====================================================
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
// MOTOR-ANSTEUERUNG (DRV8834) - in IRAM wegen critical section
// =====================================================
void IRAM_ATTR setMotorSpeed(int motorIdx, float speed) {
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

  uint32_t speedInt = static_cast<uint32_t>(actualSpeed);
  if (speedInt < 1) speedInt = 1;    // Schutz gegen Division durch Null

  uint32_t interval = STEP_PERIOD_US / speedInt;
  if (interval < MIN_STEP_INTERVAL) interval = MIN_STEP_INTERVAL;
  if (interval > MAX_STEP_INTERVAL) interval = MAX_STEP_INTERVAL;  // BUG FIX: nie 0!

  uint32_t fracAdd = (STEP_TIMER_US * 0x10000UL) / interval;  // sicher: interval >= MIN

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

void IRAM_ATTR stopMotors() {
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

int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal) {
  if (whiteVal <= blackVal) return 0;
  long value = map(static_cast<long>(raw), static_cast<long>(blackVal), static_cast<long>(whiteVal), 0L, 255L);
  value = constrain(value, 0L, 255L);
  return static_cast<int>(value);
}

// Verbesserte Farbenkennung mit Schwellwert-Logik
const char* detectColorName(int r, int g, int b) {
  // Erst pruefen ob ueberhaupt eine Farbe erkannt wird (sehr dunkel)
  int total = r + g + b;
  if (total < 60) return "SCHWARZ";
  if (r > static_cast<int>(COLOR_HIGH) && g > static_cast<int>(COLOR_HIGH) && b > static_cast<int>(COLOR_HIGH)) return "WEISS";
  // dominant channel
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";  // zu dunkel fuer echte Farbe
  if (r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  if (r > 160 && g > 120 && b < 120) return "GELB";
  return "UNBEKANNT";
}

void setupColorSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  colorSensorOk = tcs.begin();
  if (colorSensorOk) {
    Serial.println("[SENSOR] TCS34725 gefunden");
  } else {
    Serial.println("[SENSOR] TCS34725 NICHT gefunden! (Trotzdem gestartet)");
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
  const TickType_t period = pdMS_TO_TICKS(150);
  int failCount = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
      tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
      rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;

      if (rawR == 0 && rawG == 0 && rawB == 0 && rawC == 0) {
        failCount++;
        if (failCount >= 3) {
          colorSensorOk = false;
        }
      } else {
        failCount = 0;

        int r = calibrateColor(rawR, config.blackR, config.whiteR);
        int g = calibrateColor(rawG, config.blackG, config.whiteG);
        int b = calibrateColor(rawB, config.blackB, config.whiteB);

        const char* detected = detectColorName(r, g, b);

        // Timeout statt portMAX_DELAY - verhindert Task-Hang
        if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
          sensorR = r;
          sensorG = g;
          sensorB = b;
          strlcpy((char*)sensorName, detected, sizeof(sensorName));
          xSemaphoreGive(colorSem);
        }
      }
    } else {
      Serial.println("[SENSOR] Recovery attempt...");
      resetI2CBus();
      colorSensorOk = tcs.begin();
      if (colorSensorOk) {
        Serial.println("[SENSOR] Recovered!");
        failCount = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
      lastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// =====================================================
// BLE UUIDs (muessen mit HTML-Frontend uebereinstimmen)
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

unsigned long lastCommandTime = 0;
unsigned long lastColorNotify = 0;
unsigned long lastUpdateMicros = 0;

// Spiel-Logik (Stub-Handler damit Frontend nicht haengt)
int  raceMyScore = 0;
int  raceOppScore = 0;
bool raceActive = false;
bool quizActive = false;

// =====================================================
// BLE-NACHRICHTEN (direktes notify, wie im .ino)
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

void handleCommand(const char* cmdStr) {
  String cmd(cmdStr);
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

  if (cmd.startsWith("KEYS:")) {
    String data = cmd.substring(5);
    if (data.length() >= 4) {
      keyW = (data.charAt(0) == '1');
      keyA = (data.charAt(1) == '1');
      keyS = (data.charAt(2) == '1');
      keyD = (data.charAt(3) == '1');
    }
    return;  // KEYS: nicht loggen - alle 50ms sonst spam
  }

  Serial.printf("[CMD] %s\n", cmd.c_str());

  if (cmd == "STOP") {
    hardStop();
    return;
  }

  // Stack-Buffer statt String heap alloc
  char buf[48];

  if (cmd.startsWith("MAX:")) {
    float v = cmd.substring(4).toFloat();
    if (v >= MAX_SPEED_MIN && v <= MAX_SPEED_MAX) {
      config.maxSpeed = v;
      prefs.begin("edubot", false);
      prefs.putFloat("maxSpeed", config.maxSpeed);
      prefs.end();
      snprintf(buf, sizeof(buf), "MAX=%.0f", config.maxSpeed);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("TURN:")) {
    float t = cmd.substring(5).toFloat();
    if (t >= TURN_MIN && t <= TURN_MAX) {
      config.turnFactor = t;
      prefs.begin("edubot", false);
      prefs.putFloat("turnFactor", config.turnFactor);
      prefs.end();
      snprintf(buf, sizeof(buf), "TURN=%.2f", config.turnFactor);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("ACC:")) {
    float a = cmd.substring(4).toFloat();
    if (a >= ACCEL_MIN && a <= ACCEL_MAX) {
      config.accelRate = a;
      prefs.begin("edubot", false);
      prefs.putFloat("accelRate", config.accelRate);
      prefs.end();
      snprintf(buf, sizeof(buf), "ACC=%.0f", config.accelRate);
      sendStatus(buf);
    }
    return;
  }

  if (cmd == "CAL:BLACK") {
    config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
    prefs.begin("edubot", false);
    prefs.putUShort("blackR", config.blackR);
    prefs.putUShort("blackG", config.blackG);
    prefs.putUShort("blackB", config.blackB);
    prefs.end();
    sendStatus("CAL:BLACK OK");
    return;
  }

  if (cmd == "CAL:WHITE") {
    config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
    prefs.begin("edubot", false);
    prefs.putUShort("whiteR", config.whiteR);
    prefs.putUShort("whiteG", config.whiteG);
    prefs.putUShort("whiteB", config.whiteB);
    prefs.end();
    sendStatus("CAL:WHITE OK");
    return;
  }

  // Mathe-Rennen Stub
  if (cmd.startsWith("RACE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      raceMyScore = 0; raceOppScore = 0; raceActive = true;
      sendStatus("RACE:STARTED");
    } else if (sub.startsWith("PROGRESS:")) {
      raceOppScore = sub.substring(9).toInt();
      snprintf(buf, sizeof(buf), "RACE:PROGRESS:%d", raceOppScore);
      sendStatus(buf);
    } else if (sub == "WIN") {
      raceOppScore = 10; raceActive = false;
      sendStatus("RACE:LOSE");
    } else if (sub == "LOSE") {
      raceActive = false;
      sendStatus("RACE:WIN");
    }
    return;
  }

  // Farben-Quiz Stub
  if (cmd.startsWith("QUIZ:")) {
    String sub = cmd.substring(5);
    if (sub.startsWith("CHECK:")) {
      quizActive = true;
      sendStatus("QUIZ:CHECKING");
    } else if (sub == "STOP") {
      quizActive = false;
    }
    return;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

// =====================================================
// BLE CALLBACKS
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
        lastUpdateMicros = micros();   // BUG FIX: dt-reset bei Connect
        Serial.println("[BLE] Verbunden");
        sendStatus("EduBot V1.3 bereit");
        break;
      case BLE_EVT_DISCONNECTED:
        deviceConnected = false;
        clearKeys();
        speedL = 0.0;
        speedR = 0.0;
        stopMotors();
        raceActive = false;
        quizActive = false;
        Serial.println("[BLE] Getrennt - Werbung gestartet");
        BLEDevice::startAdvertising();   // <-- direkt hier, wie im .ino
        break;
      case BLE_EVT_COMMAND:
        handleCommand(evt.cmd);
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

// =====================================================
// NVS-VALIDIERUNG
// =====================================================
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
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }
  if (changed) {
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
    Serial.println("[NVS] Invalid config corrected");
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V1.3 - Xiao ESP32-S3 + DRV8834");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
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
  // Schutz: zu kleine white-Werte -> default Kalibrierung
  if (config.whiteR < 200 || config.whiteG < 200 || config.whiteB < 200) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
  }
  Serial.printf("[NVS] maxSpeed=%.0f accelRate=%.0f turnFactor=%.2f\n", config.maxSpeed, config.accelRate, config.turnFactor);
  validateConfig();
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
  Serial.println("[TIMER] Hardware-Timer gestartet (10 us)");

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

  // BLE initialisieren
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
  statusChar->setValue("EduBot V1.3");

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("[BLE] Werbung gestartet (EduBot)");
  Serial.println("======================================\n");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  esp_task_wdt_reset();

  // BLE-Events verarbeiten
  processBleEvents();

  // Fahrlogik aktualisieren
  updateDrive();

  // Farbsensor-Daten senden (~10 Hz)
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }

  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}