// =====================================================
// EduBot V1.3R - Main Entry Point
// Xiao ESP32-S3 + DRV8834 + TCS34725 + BLE + ESP-NOW
// =====================================================
// Modulare Firmware - alle Logik in separaten .h/.cpp Dateien
// =====================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_task_wdt.h>

#include "Config.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "BLEManager.h"
#include "Drive.h"
#include "EspNow.h"
#include "MemoryGame.h"
#include "GameEngine.h"

// =====================================================
// GLOBALE KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 5;

// =====================================================
// GAME STATE MACHINE
// =====================================================
GameMode currentGame = GAME_NONE;
bool driveLockedByGame() { return currentGame != GAME_NONE; }

// =====================================================
// SPIEL-BEWEGUNGS-ENGINE (blockierend)
// =====================================================
const uint32_t GAME_STEP_CM_MS   = 90;   // kuerzer: pro "Schritt" 90ms
const float     GAME_DRIVE_GAIN  = 0.65f;
constexpr uint32_t GAME_RAMP_TICK    = 10;
const uint32_t GAME_TURN_MS      = 320;  // kuerzere Drehung fuer 90 Grad
const uint32_t GAME_COLOR_READ_MS = 250;
const uint32_t GAME_SHORT_MS     = 220;
const uint32_t GAME_DRIVE_MS     = GAME_STEP_CM_MS * 3;  // ~270ms Vor/Rueck
const uint32_t GAME_KNOBELN_MS   = GAME_STEP_CM_MS * 4;  // ~360ms Pfeil-Knobeln
const uint32_t GAME_QUIZ_MS      = GAME_STEP_CM_MS * 3;  // ~270ms Korrigieren/Race

bool gameMoveBusy = false;

void gameDriveForward(uint32_t ms) {
  gameMoveBusy = true;
  const float target = config.maxSpeed * GAME_DRIVE_GAIN;
  const uint32_t rampMs = (ms / 3 > 120) ? 120 : ms / 3;
  uint32_t elapsed = 0;
  uint32_t lastMicros = micros();
  speedL = 0; speedR = 0;
  while (elapsed < ms) {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;
    if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;
    float curTarget;
    if (elapsed < rampMs) {
      curTarget = target * ((float)elapsed / (float)rampMs);
    } else if (elapsed < ms - rampMs) {
      curTarget = target;
    } else {
      float left = (float)(ms - elapsed) / (float)rampMs;
      if (left < 0.0f) left = 0.0f;
      curTarget = target * left;
    }
    speedL = rampSpeed(speedL, curTarget, dt);
    speedR = rampSpeed(speedR, curTarget, dt);
    setMotorSpeed(0, speedL);
    setMotorSpeed(1, speedR);
    vTaskDelay(pdMS_TO_TICKS(GAME_RAMP_TICK));
    elapsed += GAME_RAMP_TICK;
    if (currentGame == GAME_NONE) { stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false; return; }
  }
  stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false;
}

void gameDriveBackward(uint32_t ms) {
  gameMoveBusy = true;
  const float target = -config.maxSpeed * GAME_DRIVE_GAIN;
  const uint32_t rampMs = (ms / 3 > 120) ? 120 : ms / 3;
  uint32_t elapsed = 0;
  uint32_t lastMicros = micros();
  speedL = 0; speedR = 0;
  while (elapsed < ms) {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;
    if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;
    float curTarget;
    if (elapsed < rampMs) {
      curTarget = target * ((float)elapsed / (float)rampMs);
    } else if (elapsed < ms - rampMs) {
      curTarget = target;
    } else {
      float left = (float)(ms - elapsed) / (float)rampMs;
      if (left < 0.0f) left = 0.0f;
      curTarget = target * left;
    }
    speedL = rampSpeed(speedL, curTarget, dt);
    speedR = rampSpeed(speedR, curTarget, dt);
    setMotorSpeed(0, speedL);
    setMotorSpeed(1, speedR);
    vTaskDelay(pdMS_TO_TICKS(GAME_RAMP_TICK));
    elapsed += GAME_RAMP_TICK;
    if (currentGame == GAME_NONE) { stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false; return; }
  }
  stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false;
}

void gameTurnLeft(uint32_t ms) {
  gameMoveBusy = true;
  const float target = config.turnOnlySpeed;  // positive Geschwindigkeit, Richtung ueber Vorzeichen
  const uint32_t rampMs = (ms / 3 > 120) ? 120 : ms / 3;
  uint32_t elapsed = 0;
  uint32_t lastMicros = micros();
  speedL = 0; speedR = 0;
  while (elapsed < ms) {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;
    if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;
    float curTarget;
    if (elapsed < rampMs) {
      curTarget = target * ((float)elapsed / (float)rampMs);
    } else if (elapsed < ms - rampMs) {
      curTarget = target;
    } else {
      float left = (float)(ms - elapsed) / (float)rampMs;
      if (left < 0.0f) left = 0.0f;
      curTarget = target * left;
    }
    speedL = rampSpeed(speedL, -curTarget, dt);  // Links rueckwaerts
    speedR = rampSpeed(speedR,  curTarget, dt);  // Rechts vorwaerts -> Drehung links
    setMotorSpeed(0, speedL);
    setMotorSpeed(1, speedR);
    vTaskDelay(pdMS_TO_TICKS(GAME_RAMP_TICK));
    elapsed += GAME_RAMP_TICK;
    if (currentGame == GAME_NONE) { stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false; return; }
  }
  stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false;
}

void gameTurnRight(uint32_t ms) {
  gameMoveBusy = true;
  const float target = config.turnOnlySpeed;
  const uint32_t rampMs = (ms / 3 > 120) ? 120 : ms / 3;
  uint32_t elapsed = 0;
  uint32_t lastMicros = micros();
  speedL = 0; speedR = 0;
  while (elapsed < ms) {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;
    if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;
    float curTarget;
    if (elapsed < rampMs) {
      curTarget = target * ((float)elapsed / (float)rampMs);
    } else if (elapsed < ms - rampMs) {
      curTarget = target;
    } else {
      float left = (float)(ms - elapsed) / (float)rampMs;
      if (left < 0.0f) left = 0.0f;
      curTarget = target * left;
    }
    speedL = rampSpeed(speedL,  curTarget, dt);  // Links vorwaerts
    speedR = rampSpeed(speedR, -curTarget, dt);  // Rechts rueckwaerts -> Drehung rechts
    setMotorSpeed(0, speedL);
    setMotorSpeed(1, speedR);
    vTaskDelay(pdMS_TO_TICKS(GAME_RAMP_TICK));
    elapsed += GAME_RAMP_TICK;
    if (currentGame == GAME_NONE) { stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false; return; }
  }
  stopMotors(); speedL = 0; speedR = 0; gameMoveBusy = false;
}

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  stopMotors(); speedL = 0; speedR = 0;
  vTaskDelay(pdMS_TO_TICKS(ms));
  gameMoveBusy = false;
}

// =====================================================
// KNOBELN (Pfeil-Sequenz abfahren)
// =====================================================
constexpr uint8_t KNOBELN_MAX = 32;
uint8_t knobelnSeq[KNOBELN_MAX];
uint8_t knobelnLen = 0;

void knobelnExecute() {
  if (knobelnLen == 0) { sendStatus("KNOBELN:NONE"); return; }
  currentGame = GAME_KNOBELN;
  sendStatus("KNOBELN:RUNNING");
  for (uint8_t i = 0; i < knobelnLen; i++) {
    if (currentGame != GAME_KNOBELN) { sendStatus("KNOBELN:ABORT"); return; }
    switch (knobelnSeq[i]) {
      case 0: gameDriveForward(GAME_KNOBELN_MS); break;
      case 1: gameTurnRight(GAME_TURN_MS); break;
      case 2: gameDriveBackward(GAME_KNOBELN_MS); break;
      case 3: gameTurnLeft(GAME_TURN_MS); break;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
  }
  currentGame = GAME_NONE;
  sendStatus("KNOBELN:DONE");
}

// =====================================================
// KORRIGIEREN (Farb-Quiz)
// =====================================================
constexpr uint8_t KORRIGIEREN_MAX = 16;
char korrigierenExpected[KORRIGIEREN_MAX][16];
char korrigierenFound[KORRIGIEREN_MAX][16];
uint8_t korrigierenLen = 0;
bool korrigierenRunning = false;

void korrigierenExecute() {
  if (korrigierenLen == 0) { sendStatus("QUIZ:RESULT:OK,0,0"); return; }
  korrigierenRunning = true;
  currentGame = GAME_KORRIGIEREN;
  sendStatus("QUIZ:CHECKING");
  uint8_t okCount = 0;
  for (uint8_t i = 0; i < korrigierenLen; i++) {
    if (currentGame != GAME_KORRIGIEREN) { sendStatus("QUIZ:RESULT:ABORT"); return; }
    gameDriveForward(GAME_QUIZ_MS);
    gameStop(GAME_COLOR_READ_MS);
    char found[16] = "UNBEKANNT";
    getCurrentColorName(found, sizeof(found));
    strlcpy(korrigierenFound[i], found, sizeof(korrigierenFound[i]));
    if (strcasecmp(korrigierenExpected[i], found) == 0) okCount++;
    char part[48];
    snprintf(part, sizeof(part), "QUIZ:STEP:%u,%s,%s", i, korrigierenExpected[i], found);
    sendStatus(part);
  }
  stopMotors(); speedL = 0; speedR = 0;
  korrigierenRunning = false;
  currentGame = GAME_NONE;
  char result[48];
  if (okCount == korrigierenLen)
    snprintf(result, sizeof(result), "QUIZ:RESULT:OK,%u,%u", okCount, korrigierenLen);
  else
    snprintf(result, sizeof(result), "QUIZ:RESULT:WRONG,%u,%u", okCount, korrigierenLen);
  sendStatus(result);
}

// =====================================================
// RENNEN (Multiplayer)
// =====================================================
int  raceMyScore = 0;
volatile int  raceOppScore = 0;
volatile bool raceActive = false;
bool quizActive = false;

void raceExecuteStart(bool remote = false) {
  currentGame = GAME_RENNEN;
  raceMyScore = 0;
  raceOppScore = 0;
  raceActive = true;
  sendStatus(remote ? "RACE:REMOTE_START" : "RACE:STARTED");
}

void raceForwardStep() {
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; return; }
  gameDriveForward(GAME_QUIZ_MS);
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; }
}

void onRacePacket(const RacePacket* pkt) {
  if (pkt->flags & RACE_FLAG_WINNER || pkt->score >= RENNEN_TARGET_SCORE) {
    raceOppScore = pkt->score;
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    sendStatus("RACE:OPP_WIN");
  } else {
    if (!raceActive) raceExecuteStart(true);
    raceOppScore = pkt->score;
    char msg[32];
    snprintf(msg, sizeof(msg), "RACE:PROGRESS:%u", raceOppScore);
    sendStatus(msg);
  }
}

// =====================================================
// BLE-CONNECT/DISCONNECT-HANDLER
// =====================================================
void onBleConnected() {
  deviceConnected = true;
  lastCommandTime = millis();
  driveResetDt();
  Serial.println("[BLE] Verbunden");
  sendStatus("EduBot V1.3 bereit");
}

void onBleDisconnected() {
  deviceConnected = false;
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  raceActive = false;
  quizActive = false;
  currentGame = GAME_NONE;
  knobelnLen = 0;
  korrigierenLen = 0;
  korrigierenRunning = false;
  memoryStop();
  Serial.println("[BLE] Getrennt - Werbung gestartet");
  BLEDevice::startAdvertising();
  espNowEnsureChannel();
}

// =====================================================
// COMMAND-HANDLER
// =====================================================
void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  currentGame = GAME_NONE;
  knobelnLen = 0;
  korrigierenLen = 0;
  korrigierenRunning = false;
  raceActive = false;
  memoryStop();
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
    return;
  }

  Serial.printf("[CMD] %s\n", cmd.c_str());
  if (cmd == "STOP") { hardStop(); return; }

  char buf[48];

  if (cmd.startsWith("MAX:")) {
    float v = cmd.substring(4).toFloat();
    if (v >= MAX_SPEED_MIN && v <= MAX_SPEED_MAX) {
      config.maxSpeed = v;
      configSaveField("maxSpeed", v);
      snprintf(buf, sizeof(buf), "MAX=%.0f", config.maxSpeed);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("TURN:")) {
    float t = cmd.substring(5).toFloat();
    if (t >= TURN_MIN && t <= TURN_MAX) {
      config.turnFactor = t;
      configSaveField("turnFactor", t);
      snprintf(buf, sizeof(buf), "TURN=%.2f", config.turnFactor);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("ACC:")) {
    float a = cmd.substring(4).toFloat();
    if (a >= ACCEL_MIN && a <= ACCEL_MAX) {
      config.accelRate = a;
      configSaveField("accelRate", a);
      snprintf(buf, sizeof(buf), "ACC=%.0f", config.accelRate);
      sendStatus(buf);
    }
    return;
  }

  if (cmd == "CAL:BLACK") {
    config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
    configSaveUShort("blackR", config.blackR);
    configSaveUShort("blackG", config.blackG);
    configSaveUShort("blackB", config.blackB);
    sendStatus("CAL:BLACK OK");
    return;
  }
  if (cmd == "CAL:WHITE") {
    config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
    configSaveUShort("whiteR", config.whiteR);
    configSaveUShort("whiteG", config.whiteG);
    configSaveUShort("whiteB", config.whiteB);
    sendStatus("CAL:WHITE OK");
    return;
  }

  // KNOBELN
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
      knobelnExecute();
    } else if (sub == "STOP") {
      currentGame = GAME_NONE;
      stopMotors();
      sendStatus("KNOBELN:STOPPED");
    }
    return;
  }

  // KORRIGIEREN
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

  // RENNEN
  if (cmd.startsWith("RACE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      raceExecuteStart();
      raceSendPacket(0, false);
    } else if (sub == "STEP") {
      if (!raceActive) return;
      raceMyScore++;
      raceForwardStep();
      if (!raceActive) return;
      raceSendPacket((uint8_t)raceMyScore, raceMyScore >= RENNEN_TARGET_SCORE);
      snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
      sendStatus(buf);
      if (raceMyScore >= RENNEN_TARGET_SCORE) {
        raceActive = false;
        currentGame = GAME_NONE;
        stopMotors(); speedL = 0; speedR = 0;
        sendStatus("RACE:YOU_WIN");
      }
    } else if (sub == "WIN") {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)RENNEN_TARGET_SCORE, true);
    } else if (sub == "LOSE") {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)raceMyScore, false);
    } else if (sub == "SCAN") {
      raceSendHello();
      sendStatus("RACE:SCAN_START");
    } else if (sub == "UNPAIR") {
      raceUnpair();
      sendStatus("RACE:UNPAIRED");
    } else if (sub.startsWith("PAIR:")) {
      String macStr = sub.substring(5);
      uint8_t mac[6] = {0,0,0,0,0,0};
      int idx = 0;
      int pos = 0;
      while (idx < 6 && pos < macStr.length()) {
        String hexByte = macStr.substring(pos, pos + 2);
        mac[idx++] = (uint8_t)strtol(hexByte.c_str(), nullptr, 16);
        pos += 3;
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

  // MEMORY (A1)
  if (cmd.startsWith("MEMORY:")) {
    String sub = cmd.substring(7);
    if (sub == "START") {
      currentGame = GAME_MEMORY;
      memoryStart(3);
    } else if (sub.startsWith("START:")) {
      uint8_t n = (uint8_t)sub.substring(6).toInt();
      if (n < 3) n = 3;
      if (n > MEMORY_MAX_LEN) n = MEMORY_MAX_LEN;
      currentGame = GAME_MEMORY;
      memoryStart(n);
    } else if (sub == "STOP") {
      memoryStop();
      currentGame = GAME_NONE;
    } else if (sub.startsWith("INPUT:")) {
      uint8_t m = (uint8_t)sub.substring(6).toInt();
      memoryInput(m);
    }
    return;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V1.3 - Modular");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
  esp_task_wdt_add(NULL);

  configLoad();

  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_STEP_L, LOW);
  digitalWrite(PIN_STEP_R, LOW);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);
  digitalWrite(PIN_ENABLE, LOW);
  GPIO.out_w1tc = STEP_BIT_L | STEP_BIT_R;

  lastCommandTime = millis();
  driveResetDt();

  initStepperTimer();
  Serial.println("[TIMER] Hardware-Timer gestartet (10 us)");

  colorSetup();
  if (colorSensorOk) {
    xTaskCreatePinnedToCore(
      colorSensorTask, "ColorSensor",
      STACK_COLOR_SENSOR, NULL, TASK_PRIORITY_SENSOR, NULL, CORE_SENSOR
    );
    Serial.println("[TASK] ColorSensor-Task gestartet (Core 0)");
  }

  espNowInit();
  bleSetup();
  espNowEnsureChannel();
  Serial.println("======================================\n");
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastColorNotify = 0;
void loop() {
  esp_task_wdt_reset();
  bleProcessEvents();
  memoryTick();
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  configSaveIfDirty();
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
