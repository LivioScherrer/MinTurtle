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
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

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
  // WEISS: alle drei Kanaele hell und annaehernd gleich
  if (r > COLOR_HIGH && g > COLOR_HIGH && b > COLOR_HIGH &&
      abs(r - g) < 60 && abs(g - b) < 60 && abs(r - b) < 60) {
    return "WEISS";
  }
  // dominant channel
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";  // zu dunkel fuer echte Farbe

  // GELB: rot und gruen hoch, blau deutlich niedriger
  if (r > 100 && g > 100 && b < 80 && r > b * 1.4f && g > b * 1.4f) return "GELB";

  if (r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
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
volatile int  raceOppScore = 0;
volatile bool raceActive = false;
bool quizActive = false;

// =====================================================
// SPIEL-STATE-MACHINE
// =====================================================
// GameMode: welcher Spielmodus gerade aktiv ist
enum GameMode : uint8_t {
  GAME_NONE = 0,
  GAME_KNOBELN,    // Pfeil-Sequenz abfahren
  GAME_KORRIGIEREN, // Farb-Quiz Korrekturlauf
  GAME_RENNEN      // Multiplayer Wettrennen
};
GameMode currentGame = GAME_NONE;

// Gemeinsame Bewegungs-Engine fuer Spiele (blockierende Bewegungen)
constexpr uint32_t GAME_STEP_CM_MS = 60;     // Zeit pro "ein Schritt vorwaerts"
constexpr uint32_t GAME_TURN_MS = 280;       // Zeit pro 90 Grad Drehung
constexpr uint32_t GAME_COLOR_READ_MS = 250; // Pause ueber Plättchen

bool gameMoveBusy = false;

void gameDriveForward(uint32_t ms);
void gameTurnLeft(uint32_t ms);
void gameTurnRight(uint32_t ms);
void gameStop(uint32_t ms);

// --- Knobeln: Pfeil-Sequenz ---
#define KNOBELN_MAX 32
uint8_t knobelnSeq[KNOBELN_MAX];
uint8_t knobelnLen = 0;
uint8_t knobelnIdx = 0;

// --- Korrigieren: Farb-Quiz ---
#define KORRIGIEREN_MAX 16
char korrigierenExpected[KORRIGIEREN_MAX][16];
char korrigierenFound[KORRIGIEREN_MAX][16];
uint8_t korrigierenLen = 0;
uint8_t korrigierenIdx = 0;
bool korrigierenRunning = false;

// helper: aktuellen Farbnamen holen
void getCurrentColorName(char* out, size_t outLen) {
  if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(out, (const char*)sensorName, outLen);
    xSemaphoreGive(colorSem);
  } else {
    strlcpy(out, "UNBEKANNT", outLen);
  }
}

// --- Multiplayer Rennen (ESP-NOW) ---
#define RENNEN_TARGET_SCORE 10
uint8_t espNowPeer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // broadcast (default)
uint8_t pairedPeer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // spezifischer Gegner nach Pairing
bool espNowReady = false;
bool hasPairedPeer = false;
uint32_t raceStartTime = 0;
uint8_t myMac[6] = {0,0,0,0,0,0};

// flags Bit-Definitionen fuer RacePacket
#define RACE_FLAG_WINNER   0x01     // bit0: Sieger-Meldung
#define RACE_FLAG_HELLO    0x80     // bit7: Discovery-Hello
#define RACE_FLAG_PRESENT  0x40     // bit6: Antwort auf Hello ("ich bin da")

// Bekannte Peers (max 4 gefundene Roboter)
#define MAX_PEERS 4
struct PeerEntry {
  uint8_t mac[6];
  uint32_t lastSeen;
};
PeerEntry peerList[MAX_PEERS];
uint8_t peerCount = 0;

typedef struct __attribute__((packed)) {
  uint8_t magic;        // 0xA5
  uint8_t senderMac[6]; // MAC des Senders (zur Unterscheidung)
  uint8_t score;
  uint8_t flags;        // bitmask: bit0 winner, bit7 hello, bit6 present
} RacePacket;
static_assert(sizeof(RacePacket) == 9, "RacePacket muss 9 Bytes sein - kein Padding!");

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
  currentGame = GAME_NONE;
  knobelnLen = 0;
  knobelnIdx = 0;
  korrigierenLen = 0;
  korrigierenIdx = 0;
  korrigierenRunning = false;
  raceActive = false;
  sendStatus("STOP");
}

// =====================================================
// SPIEL-BEWEGUNGS-ENGINE
// Blockierende Bewegungen direkt ueber setMotorSpeed.
// KEYS: wird waehrenddessen ignoriert (currentGame != GAME_NONE).
// =====================================================
void gameDriveForward(uint32_t ms) {
  gameMoveBusy = true;
  speedL = config.maxSpeed * 0.6f;
  speedR = config.maxSpeed * 0.6f;
  setMotorSpeed(0, speedL);
  setMotorSpeed(1, speedR);
  vTaskDelay(pdMS_TO_TICKS(ms));
  stopMotors();
  speedL = 0; speedR = 0;
  gameMoveBusy = false;
}

void gameTurnLeft(uint32_t ms) {
  gameMoveBusy = true;
  speedL = -config.turnOnlySpeed;
  speedR = config.turnOnlySpeed;
  setMotorSpeed(0, speedL);
  setMotorSpeed(1, speedR);
  vTaskDelay(pdMS_TO_TICKS(ms));
  stopMotors();
  speedL = 0; speedR = 0;
  gameMoveBusy = false;
}

void gameTurnRight(uint32_t ms) {
  gameMoveBusy = true;
  speedL = config.turnOnlySpeed;
  speedR = -config.turnOnlySpeed;
  setMotorSpeed(0, speedL);
  setMotorSpeed(1, speedR);
  vTaskDelay(pdMS_TO_TICKS(ms));
  stopMotors();
  speedL = 0; speedR = 0;
  gameMoveBusy = false;
}

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  stopMotors();
  speedL = 0; speedR = 0;
  vTaskDelay(pdMS_TO_TICKS(ms));
  gameMoveBusy = false;
}

// =====================================================
// SPIEL: KNOBELN (Pfeil-Sequenz abfahren)
// Pfeile: 0=hoch 1=rechts 2=runter 3=links
// =====================================================
void knobelnExecute() {
  if (knobelnLen == 0) { sendStatus("KNOBELN:NONE"); return; }
  currentGame = GAME_KNOBELN;
  sendStatus("KNOBELN:RUNNING");
  Serial.printf("[KNOBELN] Start, len=%u\n", knobelnLen);

  for (uint8_t i = 0; i < knobelnLen; i++) {
    if (currentGame != GAME_KNOBELN) { sendStatus("KNOBELN:ABORT"); return; }
    uint8_t arrow = knobelnSeq[i];
    Serial.printf("[KNOBELN] step %u = %u\n", i, arrow);
    switch (arrow) {
      case 0: gameDriveForward(GAME_STEP_CM_MS * 4); break;
      case 1: gameTurnRight(GAME_TURN_MS); gameDriveForward(GAME_STEP_CM_MS * 4); break;
      case 2: gameStop(100); gameDriveForward(GAME_STEP_CM_MS * 4); break; // runter wie vorwaerts (einfach)
      case 3: gameTurnLeft(GAME_TURN_MS); gameDriveForward(GAME_STEP_CM_MS * 4); break;
      default: break;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
  }
  currentGame = GAME_NONE;
  sendStatus("KNOBELN:DONE");
  Serial.println("[KNOBELN] Done");
}

// =====================================================
// SPIEL: KORRIGIEREN (Roboter faehrt ueber Plättchen)
// =====================================================
void korrigierenExecute() {
  if (korrigierenLen == 0) { sendStatus("QUIZ:RESULT:OK,0,0"); return; }
  korrigierenRunning = true;
  currentGame = GAME_KORRIGIEREN;
  sendStatus("QUIZ:CHECKING");
  Serial.printf("[KORRIGIEREN] Start, len=%u\n", korrigierenLen);

  uint8_t okCount = 0;
  for (uint8_t i = 0; i < korrigierenLen; i++) {
    if (currentGame != GAME_KORRIGIEREN) { sendStatus("QUIZ:RESULT:ABORT"); return; }
    korrigierenIdx = i;
    // ein Stueck vorwaerts zum naechsten Plättchen
    gameDriveForward(GAME_STEP_CM_MS * 3);
    // kurz warten bis zur stabilen Messung
    gameStop(GAME_COLOR_READ_MS);
    char found[16] = "UNBEKANNT";
    getCurrentColorName(found, sizeof(found));
    strlcpy(korrigierenFound[i], found, sizeof(korrigierenFound[i]));
    Serial.printf("[KORRIGIEREN] %u: erwartet=%s gefunden=%s\n", i, korrigierenExpected[i], found);
    if (strcasecmp(korrigierenExpected[i], found) == 0) okCount++;
    char part[48];
    snprintf(part, sizeof(part), "QUIZ:STEP:%u,%s,%s", i, korrigierenExpected[i], found);
    sendStatus(part);
  }
  // am Ende stoppen
  stopMotors();
  speedL = 0; speedR = 0;
  korrigierenRunning = false;
  currentGame = GAME_NONE;
  char result[48];
  if (okCount == korrigierenLen) {
    snprintf(result, sizeof(result), "QUIZ:RESULT:OK,%u,%u", okCount, korrigierenLen);
  } else {
    snprintf(result, sizeof(result), "QUIZ:RESULT:WRONG,%u,%u", okCount, korrigierenLen);
  }
  sendStatus(result);
  Serial.printf("[KORRIGIEREN] Done: %u/%u\n", okCount, korrigierenLen);
}

// =====================================================
// ESP-NOW (Multiplayer Rennen)
// =====================================================
void espNowAddSpecificPeer(const uint8_t mac[6]);
void raceExecuteStart(bool remote);
void peerListAdd(const uint8_t mac[6]) {
  // Schon in Liste?
  for (uint8_t i = 0; i < peerCount; i++) {
    if (memcmp(peerList[i].mac, mac, 6) == 0) {
      peerList[i].lastSeen = millis();
      return;
    }
  }
  if (peerCount < MAX_PEERS) {
    memcpy(peerList[peerCount].mac, mac, 6);
    peerList[peerCount].lastSeen = millis();
    peerCount++;
    char msg[48];
    snprintf(msg, sizeof(msg), "RACE:PEER:%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sendStatus(msg);
  }
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(RacePacket)) return;
  RacePacket* pkt = (RacePacket*)data;
  if (pkt->magic != 0xA5) return;
  Serial.printf("[ESPNOW] pkt from %02X:%02X:%02X:%02X:%02X:%02X score=%u flags=%u\n",
    pkt->senderMac[0], pkt->senderMac[1], pkt->senderMac[2],
    pkt->senderMac[3], pkt->senderMac[4], pkt->senderMac[5],
    pkt->score, pkt->flags);
  // Eigenes Paket? (loopback - ESP-NOW macht das normalerweise nicht, aber sicherheitshalber)
  if (memcmp(pkt->senderMac, myMac, 6) == 0) return;

  // Discovery-Hello empfangen -> Antwort mit PRESENT + in Liste eintragen
  if (pkt->flags & RACE_FLAG_HELLO) {
    peerListAdd(pkt->senderMac);
    // Sender als Peer registrieren, damit wir gezielt antworten koennen
    espNowAddSpecificPeer(pkt->senderMac);
    // Mit PRESENT antworten (gezielt an Sender)
    RacePacket resp;
    resp.magic = 0xA5;
    memcpy(resp.senderMac, myMac, 6);
    resp.score = 0;
    resp.flags = (uint8_t)RACE_FLAG_PRESENT;
    esp_now_send(pkt->senderMac, (uint8_t*)&resp, sizeof(resp));
    Serial.println("[ESPNOW] PRESENT gesendet");
    return;
  }
  // PRESENT-Antwort -> nur in Liste eintragen
  if (pkt->flags & RACE_FLAG_PRESENT) {
    peerListAdd(pkt->senderMac);
    return;
  }

  // Wenn gepaart: nur Pakete vom Peer akzeptieren
  if (hasPairedPeer && memcmp(pkt->senderMac, pairedPeer, 6) != 0) return;

  // Normales Rennen-Paket
  if (pkt->flags & RACE_FLAG_WINNER || pkt->score >= RENNEN_TARGET_SCORE) {
    raceOppScore = pkt->score;
    raceActive = false;
    currentGame = GAME_NONE;   // Roboter wieder frei steuerbar
    stopMotors();
    speedL = 0; speedR = 0;
    sendStatus("RACE:OPP_WIN");
  } else {
    // Wenn das Rennen lokal noch nicht aktiv ist, der Peer aber schon spielt
    // -> automatisch starten (Gegner hat das Rennen begonnen)
    if (!raceActive) {
      raceExecuteStart(true);  // true = vom Gegner ausgeloest (setzt raceOppScore=0)
    }
    raceOppScore = pkt->score; // Erst nach dem Start-Reset zuweisen!
    char msg[32];
    snprintf(msg, sizeof(msg), "RACE:PROGRESS:%u", raceOppScore);
    sendStatus(msg);
  }
}

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[ESPNOW] send fehlgeschlagen status=%d\n", status);
  }
}

void espNowAddSpecificPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 1;          // FIXIERT - gleicher Channel wie broadcast
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] specific add_peer (vielleicht schon da)");
  }
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  // WiFi anhalten, Channel fixieren, dann WiFi wieder starten (ESP-NOW braucht das)
  // WICHTIG: ESP-NOW broadcast funktioniert zuverlaessig, wenn beide Roboter
  // auf demselben Primary-Channel sind. Wir fixieren Channel 1.
  esp_wifi_stop();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Eigene MAC-Adresse lesen (zur Unterscheidung im Paket)
  WiFi.macAddress(myMac);
  Serial.printf("[ESPNOW] meine MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);

  if (esp_now_init() != ESP_OK) { Serial.println("[ESPNOW] init failed"); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSend);

  // Broadcast-Peer explizit auf Channel 1
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, espNowPeer, 6);
  peer.channel = 1;            // FIXIERT - nicht 0 (auto)
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    espNowReady = true;
    Serial.println("[ESPNOW] bereit (broadcast peer auf channel 1)");
  } else {
    Serial.println("[ESPNOW] add_peer failed");
  }
}

// Sicherstellen, dass WiFi auf Channel 1 bleibt (BLE kann ihn verschieben).
// Nur aufrufen wenn noetig - NICHT periodisch im Loop, da das ESP-NOW stoert.
void espNowEnsureChannel() {
  if (!espNowReady) return;
  static uint8_t lastChannel = 255;
  uint8_t primary;
  wifi_second_chan_t second;
  if (esp_wifi_get_channel(&primary, &second) == ESP_OK && primary != lastChannel) {
    lastChannel = primary;
    Serial.printf("[ESPNOW] channel war %u -> setze auf 1\n", primary);
  }
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void raceSendPacket(uint8_t score, bool winner) {
  if (!espNowReady) return;
  RacePacket pkt;
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = score;
  pkt.flags = (uint8_t)(winner ? RACE_FLAG_WINNER : 0x00);
  // Wenn gepaart, direkt an Peer, sonst broadcast
  if (hasPairedPeer) {
    esp_now_send(pairedPeer, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  }
}

void raceSendHello() {
  if (!espNowReady) return;
  RacePacket pkt;
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = 0;
  pkt.flags = (uint8_t)RACE_FLAG_HELLO;
  // Liste leeren fuer frischen Scan
  peerCount = 0;
  esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  Serial.println("[ESPNOW] HELLO gesendet");
}

bool racePairWith(const uint8_t mac[6]) {
  espNowAddSpecificPeer(mac);
  memcpy(pairedPeer, mac, 6);
  hasPairedPeer = true;
  Serial.printf("[ESPNOW] Gepaart mit %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return true;
}

void raceUnpair() {
  if (hasPairedPeer) {
    esp_now_del_peer(pairedPeer);
    hasPairedPeer = false;
    pairedPeer[0]=0xFF; pairedPeer[1]=0xFF; pairedPeer[2]=0xFF;
    pairedPeer[3]=0xFF; pairedPeer[4]=0xFF; pairedPeer[5]=0xFF;
    Serial.println("[ESPNOW] Unpair");
  }
}

// =====================================================
// SPIEL: RENNEN (Multiplayer)
// =====================================================
void raceExecuteStart(bool remote = false) {
  currentGame = GAME_RENNEN;
  raceMyScore = 0;
  raceOppScore = 0;
  raceActive = true;
  raceStartTime = millis();
  // Remote-Start: teile Frontend mit, dass Gegner das Rennen begonnen hat
  sendStatus(remote ? "RACE:REMOTE_START" : "RACE:STARTED");
  Serial.println(remote ? "[RENNEN] Remote-Start" : "[RENNEN] Start");
}

void raceForwardStep() {
  // Ein "Schritt vorwaerts" pro richtig geloester Aufgabe
  // Rennen kann durch Gegner-Paket in der Zwischenzeit beendet worden sein
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; return; }
  gameDriveForward(GAME_STEP_CM_MS * 3);
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; }
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

  // ============================================
  // SPIEL: KNOBELN (Pfeil-Sequenz)
  // Format: KNOBELN:SET:<len>:<p0>,<p1>,...  (Pfeile 0-3)
  //         KNOBELN:RUN
  //         KNOBELN:STOP
  // ============================================
  if (cmd.startsWith("KNOBELN:")) {
    String sub = cmd.substring(8);
    if (sub.startsWith("SET:")) {
      String rest = sub.substring(4);
      int colon = rest.indexOf(':');
      if (colon < 0) return;
      uint8_t len = (uint8_t)rest.substring(0, colon).toInt();
      String arr = rest.substring(colon + 1);
      if (len == 0 || len > KNOBELN_MAX) { sendStatus("KNOBELN:BADLEN"); return; }
      knobelnLen = 0;
      int start = 0;
      while (knobelnLen < len) {
        int comma = arr.indexOf(',', start);
        String part = (comma < 0) ? arr.substring(start) : arr.substring(start, comma);
        int v = part.toInt();
        if (v < 0 || v > 3) { sendStatus("KNOBELN:BADARROW"); return; }
        knobelnSeq[knobelnLen++] = (uint8_t)v;
        if (comma < 0) break;
        start = comma + 1;
      }
      snprintf(buf, sizeof(buf), "KNOBELN:READY:%u", knobelnLen);
      sendStatus(buf);
    } else if (sub == "RUN") {
      knobelnIdx = 0;
      knobelnExecute();
    } else if (sub == "STOP") {
      currentGame = GAME_NONE;
      stopMotors();
      sendStatus("KNOBELN:STOPPED");
    }
    return;
  }

  // ============================================
  // SPIEL: KORRIGIEREN (Farb-Quiz)
  // Format: QUIZ:CHECK:<c0>,<c1>,...  (Farbnamen)
  //         QUIZ:STOP
  // ============================================
  if (cmd.startsWith("QUIZ:")) {
    String sub = cmd.substring(5);
    if (sub.startsWith("CHECK:")) {
      String arr = sub.substring(6);
      korrigierenLen = 0;
      int start = 0;
      while (korrigierenLen < KORRIGIEREN_MAX) {
        int comma = arr.indexOf(',', start);
        String part = (comma < 0) ? arr.substring(start) : arr.substring(start, comma);
        part.trim();
        part.toUpperCase();
        strlcpy(korrigierenExpected[korrigierenLen], part.c_str(), sizeof(korrigierenExpected[0]));
        korrigierenLen++;
        if (comma < 0) break;
        start = comma + 1;
      }
      korrigierenExecute();
    } else if (sub == "STOP") {
      currentGame = GAME_NONE;
      korrigierenRunning = false;
      stopMotors();
      sendStatus("QUIZ:STOPPED");
    }
    return;
  }

  // ============================================
  // SPIEL: RENNEN (Multiplayer via ESP-NOW)
  // Format: RACE:START         -> startet Rennen
  //         RACE:STEP          -> ich habe Aufgabe geloest, vorwaerts
  //         RACE:WIN           -> ich habe gewonnen
  //         RACE:LOSE          -> Gegner gewonnen
  //         RACE:PROGRESS:<n>  -> gegner fortschritt (alt)
  // ============================================
  if (cmd.startsWith("RACE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      raceExecuteStart();
      raceSendPacket(0, false);
    } else if (sub == "STEP") {
      if (!raceActive) return;
      raceMyScore++;
      raceForwardStep();
      // Rennen kann waehrend raceForwardStep durch Gegner beendet worden sein
      if (!raceActive) return;
      raceSendPacket((uint8_t)raceMyScore, raceMyScore >= RENNEN_TARGET_SCORE);
      snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
      sendStatus(buf);
      if (raceMyScore >= RENNEN_TARGET_SCORE) {
        raceActive = false;
        currentGame = GAME_NONE;   // Roboter wieder frei steuerbar
        stopMotors();
        speedL = 0; speedR = 0;
        sendStatus("RACE:YOU_WIN");
      }
    } else if (sub == "WIN") {
      // Frontend bestaetigt eigenen Sieg - nicht erneut ans Frontend senden
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors();
      speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)RENNEN_TARGET_SCORE, true);
    } else if (sub == "LOSE") {
      // Frontend meldet eigenen Verlust - info an Gegner
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors();
      speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)raceMyScore, false);
    } else if (sub == "SCAN") {
      // Peers suchen
      raceSendHello();
      sendStatus("RACE:SCAN_START");
    } else if (sub == "UNPAIR") {
      raceUnpair();
      sendStatus("RACE:UNPAIRED");
    } else if (sub.startsWith("PAIR:")) {
      // Erwartet MAC als XX:XX:XX:XX:XX:XX
      String macStr = sub.substring(5);
      uint8_t mac[6] = {0,0,0,0,0,0};
      int idx = 0;
      int pos = 0;
      while (idx < 6 && pos < macStr.length()) {
        String hexByte = macStr.substring(pos, pos + 2);
        mac[idx++] = (uint8_t)strtol(hexByte.c_str(), nullptr, 16);
        pos += 3;  // ueberspringt ':'
      }
      if (idx == 6) {
        racePairWith(mac);
        char conf[48];
        snprintf(conf, sizeof(conf), "RACE:PAIRED:%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        sendStatus(conf);
      } else {
        sendStatus("RACE:PAIR_FAIL");
      }
    } else if (sub.startsWith("PROGRESS:")) {
      raceOppScore = sub.substring(9).toInt();
      snprintf(buf, sizeof(buf), "RACE:PROGRESS:%d", raceOppScore);
      sendStatus(buf);
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
        currentGame = GAME_NONE;
        knobelnLen = 0;
        korrigierenLen = 0;
        korrigierenRunning = false;
        Serial.println("[BLE] Getrennt - Werbung gestartet");
        BLEDevice::startAdvertising();   // <-- direkt hier, wie im .ino
        // BLE kann den WiFi-Channel verschoben haben - wieder fixieren
        espNowEnsureChannel();
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
  // Waehrend ein Spiel aktiv ist, wird die Bewegung von der Spiel-Engine
  // blockierend gesteuert - KEYS: vom Frontend werden ignoriert.
  if (currentGame != GAME_NONE) {
    return;
  }

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

  // ESP-NOW fuer Multiplayer initialisieren
  espNowInit();

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

  // BLE kann den WiFi-Channel veraendern - nach BLE-Init sicherstellen,
  // dass wir wieder auf Channel 1 sind fuer ESP-NOW
  espNowEnsureChannel();
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