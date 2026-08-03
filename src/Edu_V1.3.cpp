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
#include "LineFollower.h"

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
// Einheitliche Zeiten: jedes Spiel (Knobeln, Memory, Quiz, Rennen) nutzt
// dieselbe Vor/Zurueck- und Drehdauer -> konsistenter Bewegungsablauf.
// Beschleunigung/Bremsen uebernimmt AccelStepper (kein Wheelie, kein Schraeglauf).
const float     GAME_DRIVE_GAIN   = 1.0f;
constexpr uint32_t GAME_TICK_MS    = 5;    // Polling-Intervall fuer motorRun() waehrend blockierender Moves
const uint32_t GAME_MOVE_MS       = 360;  // Vor/Zurueck-Fahrt in ALLEN Spielen
const uint32_t GAME_COLOR_READ_MS = 250;  // Pause waehrend Farbsensor liest
constexpr uint8_t QUIZ_MAX_COLOR_USE = 4;  // jede Farbe max. so oft im Quiz verwendbar

bool gameMoveBusy = false;

// Wartet bis beide Stepper wirklich stehen (max 1s). motorRun() wird
// weiterhin aufgerufen, damit die Schritte waehrend der Verzoegerung
// noch ausgefuehrt werden.
static void waitForStop(uint32_t startMs, uint32_t maxExtraMs = 1000) {
  uint32_t extra = millis();
  while ((stepperL.isRunning() || stepperR.isRunning()) && millis() - extra < maxExtraMs) {
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

void gameDriveForward(uint32_t ms) {
  gameMoveBusy = true;
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  const float target = config.maxSpeed * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop(start);
  gameMoveBusy = false;
}

void gameDriveBackward(uint32_t ms) {
  gameMoveBusy = true;
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  const float target = -config.maxSpeed * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop(start);
  gameMoveBusy = false;
}

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  uint32_t start = millis();
  while (millis() - start < (uint32_t)ms) {
    motorRun();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Positionsbasiert: exakte Strecke fahren (z.B. 3cm pro Farbe im Quiz).
// Beruecksichtigt config.leftSign/rightSign fuer die Verkabelung.
void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  // Maximalgeschwindigkeit fuer diesen Move (benutzerkonfigurierbar)
  stepperL.setMaxSpeed(config.maxSpeed);
  stepperR.setMaxSpeed(config.maxSpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  // Relative Position: Roboter bewegt sich um 'steps' vorwaerts.
  // Durch config.leftSign/rightSign wird die Richtung korrigiert.
  stepperL.move(steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop();
      stepperR.stop();
      gameMoveBusy = false;
      return;
    }
    stepperL.run();
    stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: fahre n Schritte ohne currentGame-Abhaengigkeit, mit Timeout.
// Wird vom STEPS:-Befehl genutzt, um den richtigen Wert fuer 3cm zu finden.
void testDriveSteps(int32_t steps) {
  stepperL.setMaxSpeed(config.maxSpeed);
  stepperR.setMaxSpeed(config.maxSpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move(steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(steps * static_cast<int32_t>(config.rightSign));
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    stepperL.run();
    stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.stop();
    stepperR.stop();
  }
}

// Positionsbasierte Drehung: jeder Motor 'steps' in entgegengesetzte Richtung.
void gameTurnLeftSteps(int32_t steps) {
  gameMoveBusy = true;
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move(-steps * static_cast<int32_t>(config.leftSign));
  stepperR.move( steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}
void gameTurnRightSteps(int32_t steps) {
  gameMoveBusy = true;
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move( steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(-steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: drehe auf der Stelle (rechts) um 'steps' Schritte pro Rad.
void testTurnSteps(int32_t steps) {
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move( steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(-steps * static_cast<int32_t>(config.rightSign));
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.stop(); stepperR.stop();
  }
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
      case 0: gameDriveForward(GAME_MOVE_MS); break;
      case 1: gameTurnRightSteps(config.quizTurnSteps); break;
      case 2: gameDriveBackward(GAME_MOVE_MS); break;
      case 3: gameTurnLeftSteps(config.quizTurnSteps); break;
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
    gameDriveForwardSteps(config.quizForwardSteps);
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
  gameDriveForward(GAME_MOVE_MS);
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
  lineFollowerStop();
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
  lineFollowerStop();
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

  if (cmd == "STEPS?") {
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)config.quizForwardSteps);
    sendStatus(buf);
    return;
  }
  if (cmd.startsWith("STEPS:")) {
    int32_t n = (int32_t)cmd.substring(6).toInt();
    if (n < 1 || n > 2000) {
      sendStatus("STEPS:BAD");
      return;
    }
    config.quizForwardSteps = n;
    configSaveInt("quizSteps", n);
    // Sofort testen
    testDriveSteps(n);
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[STEPS] Test %ld Schritte gefahren (fuer 3cm-Quiz gespeichert)\n", (long)n);
    return;
  }

  if (cmd == "ANG?") {
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)config.quizTurnSteps);
    sendStatus(buf);
    return;
  }
  if (cmd.startsWith("ANG:")) {
    int32_t n = (int32_t)cmd.substring(4).toInt();
    if (n < 1 || n > 5000) {
      sendStatus("ANG:BAD");
      return;
    }
    config.quizTurnSteps = n;
    configSaveInt("quizTurnSteps", n);
    // Sofort testen (Drehung rechts auf der Stelle)
    testTurnSteps(n);
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[ANG] Test %ld Schritte/Rad gedreht (fuer 90-Grad-Spiel gespeichert)\n", (long)n);
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

  if (cmd == "CAL:RED" || cmd == "CAL:YELLOW" || cmd == "CAL:GREEN" || cmd == "CAL:BLUE") {
    ColorRef* ref = nullptr;
    const char* name = nullptr;
    const char* key  = nullptr;
    if      (cmd == "CAL:RED")    { ref = &config.calRot;   name = "RED";    key = "Rot";    }
    else if (cmd == "CAL:YELLOW") { ref = &config.calGelb;  name = "YELLOW"; key = "Gelb";   }
    else if (cmd == "CAL:GREEN")  { ref = &config.calGruen; name = "GREEN";  key = "Gruen";  }
    else                          { ref = &config.calBlau;  name = "BLUE";   key = "Blau";   }
    int r, g, b;
    if (colorSem == nullptr || xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) != pdTRUE) {
      sendStatus("CAL:BUSY");
      return;
    }
    r = sensorR; g = sensorG; b = sensorB;
    xSemaphoreGive(colorSem);
    ref->r = (uint8_t)constrain(r, 0, 255);
    ref->g = (uint8_t)constrain(g, 0, 255);
    ref->b = (uint8_t)constrain(b, 0, 255);
    ref->calibrated = true;
    char kR[16], kG[16], kB[16], kS[16];
    snprintf(kR, sizeof(kR), "cal%sR", key);
    snprintf(kG, sizeof(kG), "cal%sG", key);
    snprintf(kB, sizeof(kB), "cal%sB", key);
    snprintf(kS, sizeof(kS), "cal%sSet", key);
    configSaveUChar(kR, ref->r);
    configSaveUChar(kG, ref->g);
    configSaveUChar(kB, ref->b);
    configSaveBool(kS, true);
    char msg[48];
    snprintf(msg, sizeof(msg), "CAL:%s OK,%d,%d,%d", name, ref->r, ref->g, ref->b);
    sendStatus(msg);
    return;
  }

  if (cmd == "CAL:?") {
    char msg[96];
    snprintf(msg, sizeof(msg),
      "CAL:ROT=%s(%u,%u,%u) GELB=%s(%u,%u,%u) GRUEN=%s(%u,%u,%u) BLAU=%s(%u,%u,%u)",
      config.calRot.calibrated   ? "*" : "-", config.calRot.r,   config.calRot.g,   config.calRot.b,
      config.calGelb.calibrated  ? "*" : "-", config.calGelb.r,  config.calGelb.g,  config.calGelb.b,
      config.calGruen.calibrated ? "*" : "-", config.calGruen.r, config.calGruen.g, config.calGruen.b,
      config.calBlau.calibrated  ? "*" : "-", config.calBlau.r,  config.calBlau.g,  config.calBlau.b);
    sendStatus(msg);
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
      // Jede Farbe max. QUIZ_MAX_COLOR_USE mal
      for (uint8_t i = 0; i < korrigierenLen; i++) {
        uint8_t count = 0;
        for (uint8_t j = 0; j < korrigierenLen; j++) {
          if (strcasecmp(korrigierenExpected[i], korrigierenExpected[j]) == 0) count++;
        }
        if (count > QUIZ_MAX_COLOR_USE) {
          char err[48];
          snprintf(err, sizeof(err), "QUIZ:TOO_MANY:%s,%u", korrigierenExpected[i], count);
          sendStatus(err);
          korrigierenLen = 0;
          return;
        }
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

  // LINIE FOLGEN
  if (cmd.startsWith("LINE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      if (!lineFollowerActive) {
        currentGame = GAME_LINEFOLLOWER;
        lineFollowerStart();
        sendStatus("LINE:RUNNING");
      }
    } else if (sub == "STOP") {
      bool wasActive = lineFollowerActive;
      lineFollowerStop();
      currentGame = GAME_NONE;
      sendStatus(wasActive ? "LINE:STOPPED" : "LINE:STOPPED");
    }
    return;
  }

  // MEMORY (A1)
  if (cmd.startsWith("MEMORY:")) {    String sub = cmd.substring(7);
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

  initSteppers();
  Serial.println("[MOTOR] AccelStepper initialisiert (Beschleunigung aktiv)");
  playStartupBeep();

  lastCommandTime = millis();
  driveResetDt();

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
  if (currentGame == GAME_LINEFOLLOWER) {
    bool wasActive = lineFollowerActive;
    lineFollowerTick();
    if (wasActive && !lineFollowerActive) {
      // Linie endgueltig verloren -> Modus beenden, Meldung senden
      currentGame = GAME_NONE;
      sendStatus("LINE:LOST");
    }
  }
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  configSaveIfDirty();
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
