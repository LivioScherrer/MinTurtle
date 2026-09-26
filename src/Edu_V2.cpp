// =====================================================
// EduBot V2 - Main Entry Point
// Xiao ESP32-S3 + 2x TMC2209 (UART) + TCS34725 + BLE + ESP-NOW
// =====================================================
// Modulare Firmware - alle Logik in separaten .h/.cpp Dateien
// =====================================================

#include <Arduino.h>
#include <Wire.h>
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
#include "ProgGame.h"
#include "LedStrip.h"
#include "DistanceSensor.h"
#include "Slalom.h"

// =====================================================
// GLOBALE KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 1;

// Hindernis-Erkennung: VL53L0X (siehe DistanceSensor.h, OBSTACLE_DIST_MM).
// DIST-Meldungen kommen aus distTick() alle DIST_NOTIFY_MS.

// =====================================================
// GAME STATE MACHINE
// =====================================================
// THREADING: currentGame wird ausschliesslich auf Core 1 gelesen und
// geschrieben - die Main-Loop verarbeitet alle Eingaben (BLE-Events aus der
// Queue, ESP-NOW-Events aus der Queue, Serial) und ruft sich in den
// blockierenden Spiel-Moves ueber bleProcessEvents() selbst erneut auf.
// Motor-Task (Core 0) und ColorSensor-Task (Core 0) lesen currentGame NICHT.
// Deshalb ist hier KEIN volatile noetig; Motor.cpp sichert seinen eigenen
// Zustand separat per Mutex (siehe Motor.h).
GameMode currentGame = GAME_NONE;
bool driveLockedByGame() { return currentGame != GAME_NONE; }

// =====================================================
// SPIEL-BEWEGUNGS-ENGINE (blockierend)
// =====================================================
// Einheitliche Zeiten: jedes Spiel (Memory, Quiz, Rennen) nutzt
// dieselbe Vor/Zurueck- und Drehdauer -> konsistenter Bewegungsablauf.
// Feste Geschwindigkeit ohne Beschleunigung: der Motor-Task auf Core 0
// erzeugt die Pulse unabhaengig von dieser Loop.
constexpr uint32_t GAME_TICK_MS    = 1;    // Polling-Intervall in blockierenden Moves (Pulse macht der Motor-Task)
const uint32_t GAME_MOVE_MS       = 360;  // Vor/Zurueck-Fahrt in ALLEN Spielen
const uint32_t GAME_COLOR_READ_MS = 250;  // Pause waehrend Farbsensor liest
constexpr uint8_t QUIZ_MAX_COLOR_USE = 4;  // jede Farbe max. so oft im Quiz verwendbar

bool gameMoveBusy = false;

// Gemeinsame Warte-Schleife fuer alle blockierenden Bewegungen.
// bAbortOnNoGame=true bricht ab, sobald currentGame auf GAME_NONE geht
// (Notfall-Stop ueber STOP / BLE-Disconnect), sonst laeuft der Move durch.
static void waitWhileRunning(uint32_t ms, bool bAbortOnNoGame) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
    if (bAbortOnNoGame && currentGame == GAME_NONE) {
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Wartet bis beide Stepper wirklich stehen (max 1s). Die Pulse erzeugt der
// Motor-Task selbststaendig - hier wird nur auf das Ende gewartet.
static void waitForStop(uint32_t maxExtraMs = 1000) {
  const uint32_t extra = millis();
  while (motorStillRunning() && millis() - extra < maxExtraMs) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Vorwaerts/Zurueck-Fahrt fuer 'ms'. Vorzeichen = Richtung (setMotorSpeed
// ueberschreibt den Betrag ohnehin auf FIXED_DRIVE_SPEED).
void gameDrive(int32_t ms, bool bForward) {
  gameMoveBusy = true;
  const float target = bForward ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  waitWhileRunning((uint32_t)ms, true);
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop();
  gameMoveBusy = false;
}

void gameDriveForward(uint32_t ms)  { gameDrive((int32_t)ms, true); }
void gameDriveBackward(uint32_t ms) { gameDrive((int32_t)ms, false); }

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  // bAbortOnNoGame=false: ein bewusstes Pause-Fenster (z.B. waehrend der
  // Farbsensor liest) darf nicht durch einen Spielabbruch verkuerzt werden.
  waitWhileRunning(ms, false);
  gameMoveBusy = false;
}

// Wartet, bis beide Stepper stehen. bAbortOnNoGame=true bricht bei
// Spielabbruch ab (currentGame == GAME_NONE).
static void waitForStepsDone(bool bAbortOnNoGame) {
  while (motorStillRunning()) {
    esp_task_wdt_reset();
    if (bAbortOnNoGame && currentGame == GAME_NONE) {
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Positionsbasiert: exakte Strecke fahren (z.B. 3cm pro Farbe im Quiz).
// Beruecksichtigt config.leftSign/rightSign fuer die Verkabelung.
void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  prepPositionMove(0, steps * static_cast<long>(config.leftSign));
  prepPositionMove(1, steps * static_cast<long>(config.rightSign));
  waitForStepsDone(true);
  gameMoveBusy = false;
}

// Test-Helper: fahre n Schritte ohne currentGame-Abhaengigkeit, mit Timeout.
// Wird vom STEPS:-Befehl genutzt, um den richtigen Wert fuer 3cm zu finden.
void testDriveSteps(int32_t steps) {
  prepPositionMove(0, steps * static_cast<long>(config.leftSign));
  prepPositionMove(1, steps * static_cast<long>(config.rightSign));
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 5000) {
    esp_task_wdt_reset();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (motorStillRunning()) stopMotors();
}

// Positionsbasierte Drehung auf der Stelle: beide Motoren bekommen
// entgegengesetzte Richtungen, je 'steps' (vorzeichenkorrigiert).
// bLeft=true -> links rueckwaerts / rechts vorwaerts.
static void gameTurnSteps(int32_t steps, bool bLeft) {
  gameMoveBusy = true;
  const long l = bLeft ? -steps : steps;
  const long r = bLeft ? steps : -steps;
  prepPositionMove(0, l * static_cast<long>(config.leftSign));
  prepPositionMove(1, r * static_cast<long>(config.rightSign));
  waitForStepsDone(true);
  gameMoveBusy = false;
}

void gameTurnLeftSteps(int32_t steps)  { gameTurnSteps(steps, true); }
void gameTurnRightSteps(int32_t steps) { gameTurnSteps(steps, false); }

// Test-Helper: drehe auf der Stelle (rechts) um 'steps' Schritte pro Rad.
void testTurnSteps(int32_t steps) {
  prepPositionMove(0,  steps * static_cast<long>(config.leftSign));
  prepPositionMove(1, -steps * static_cast<long>(config.rightSign));
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 5000) {
    esp_task_wdt_reset();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (motorStillRunning()) stopMotors();
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
    bleProcessEvents();
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
int  raceOppScore = 0;
bool raceActive = false;
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

// Forward declarations for Slalom
void onSlalomPacket(const RacePacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

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
  Serial.println("[BLE] Verbunden");
  sendStatus("EduBot V2 bereit");
}

void onBleDisconnected() {
  deviceConnected = false;
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
   raceActive = false;
  quizActive = false;
  currentGame = GAME_NONE;
  korrigierenLen = 0;
  korrigierenRunning = false;
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  memoryStop();
  lineFollowerStop();
  progStop();
  Serial.println("[BLE] Getrennt - Werbung gestartet");
  BLEDevice::startAdvertising();
  espNowEnsureChannel();
}

// Forward declarations for Slalom
void onSlalomPacket(const RacePacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

// =====================================================
// COMMAND-HANDLER
// =====================================================
// Jede Befehlsgruppe hat einen eigenen Handler. Ein Handler bekommt den
// bereits getrimmten, GROSSBUCHSTABEN-Befehl und den Teil hinter dem
// Doppelpunkt (sub, nur bei Prefix-Befehlen belegt) und gibt true zurueck,
// wenn er den Befehl verarbeitet hat. handleCommand() ist dadurch nur noch
// die Reihenfolge der Pruefungen - jede Gruppe laesst sich einzeln lesen,
// aendern und testen.
void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  tmcDisable();   // Treiber ganz aus (kein Haltestrom, keine Waerme)
  currentGame = GAME_NONE;
  korrigierenLen = 0;
  korrigierenRunning = false;
  raceActive = false;
  memoryStop();
  lineFollowerStop();
  progStop();
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  ledStripShowAlert();
  sendStatus("STOP");
}

// ---- Tastatur-Status (KEYS:<W><A><S><D>) ----
static bool cmdKeys(const String& cmd) {
  String data = cmd.substring(5);
  if (data.length() >= 4) {
    keyW = (data.charAt(0) == '1');
    keyA = (data.charAt(1) == '1');
    keyS = (data.charAt(2) == '1');
    keyD = (data.charAt(3) == '1');
  }
  return true;
}

// ---- Distanzsensor: Messwert, Auto-Meldung, Diagnose, Schwellen, Offset ----
static bool cmdDist(const String& cmd) {
  char buf[48];
  if (cmd == "DIST?") {
    int16_t mm = distMeasureMM();
    if (mm >= 0) snprintf(buf, sizeof(buf), "DIST:%d", mm);
    else snprintf(buf, sizeof(buf), "DIST:--");
    sendStatus(buf);
    return true;
  }
  if (cmd == "DISTAUTO:ON") { distSetAuto(true); sendStatus("DISTAUTO:ON"); return true; }
  if (cmd == "DISTAUTO:OFF") { distSetAuto(false); sendStatus("DISTAUTO:OFF"); return true; }
  // Diagnose: kompakte Zeile nach BLE + ausfuehrlicher Status auf Serial.
  if (cmd == "DISTDIAG" || cmd == "DIST:DIAG") {
    distDiagPrint();
    char dbuf[96];
    distDiag(dbuf, sizeof(dbuf));
    sendStatus(dbuf);
    return true;
  }
  // Init-Debug: jeden Schritt mit Library-Debug nachvollziehen (nur Diagnose).
  if (cmd == "DISTDEBUG") {
    distDebugInit();
    sendStatus("DIST:DEBUG_DONE");
    return true;
  }
  // Bus-Zustand vom Boot-Zeitpunkt (beweist Power-Cycle-Wirkung).
  if (cmd == "DISTSCAN" || cmd == "DIST:SCAN") {
    distBootScan();
    sendStatus("DIST:SCAN_DONE");
    return true;
  }
  // Hindernisschwellen (NVS-persistent, Hysterese: Freigabe > Stopp).
  if (cmd == "OBSDIST?") {
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("OBSDIST:")) {
    int v = cmd.substring(8).toInt();
    if (v < 50 || v > 1200) { sendStatus("OBSDIST:BAD"); return true; }
    config.obsDistMM = (int16_t)v;
    configSaveInt("obsDist", v);
    if (config.obsRelMM <= config.obsDistMM) {
      config.obsRelMM = (int16_t)min(v + 30, 1200);
      configSaveInt("obsRel", config.obsRelMM);
      snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
      sendStatus(buf);
    }
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    return true;
  }
  if (cmd == "OBSREL?") {
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("OBSREL:")) {
    int v = cmd.substring(7).toInt();
    if (v < 50 || v > 1200) { sendStatus("OBSREL:BAD"); return true; }
    config.obsRelMM = (int16_t)v;
    configSaveInt("obsRel", v);
    if (config.obsRelMM <= config.obsDistMM) {
      config.obsDistMM = (int16_t)max(v - 30, 50);
      configSaveInt("obsDist", config.obsDistMM);
      snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
      sendStatus(buf);
    }
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
  // Offset-Kalibrierung (Datenblatt-Verfahren): weisses Target exakt bei
  // <mm> (50..500, empfohlen 100) ruhig halten, dann DISTCAL:<mm> senden.
  if (cmd == "DISTCAL?") {
    snprintf(buf, sizeof(buf), "DISTCAL=%d", distOffsetMM());
    sendStatus(buf);
    return true;
  }
  if (cmd == "DISTCAL:CLR") {
    distClearOffset();
    sendStatus("DISTCAL=0");
    return true;
  }
  if (cmd.startsWith("DISTCAL:")) {
    int v = cmd.substring(8).toInt();
    if (v < 50 || v > 500) { sendStatus("DISTCAL:BAD"); return true; }
    sendStatus("DISTCAL:RUN");
    if (distCalibrateOffset((int16_t)v)) {
      snprintf(buf, sizeof(buf), "DISTCAL=%d", distOffsetMM());
      sendStatus(buf);
    } else {
      sendStatus("DISTCAL:FAIL");
    }
    return true;
  }
  if (cmd == "I2C?") {
    // Bus-Scan 0x08..0x77. Gesund: 0x29 (TCS34725) + 0x30 (VL53L0X nach Umzug).
    // Nur 0x29 = VL53L0X fehlt/tot. Nichts = Bus/Wiring/Pull-ups pruefen.
    String out = "I2C:";
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { char h[8]; snprintf(h, sizeof(h), " %02X", a); out += h; }
    }
    sendStatus(out.c_str());
    Serial.printf("[I2C]%s\n", out.c_str() + 3);
    return true;
  }
  return false;
}

// ---- Farb-Kalibrierung: CAL:BLACK / WHITE / ROT / GELB / GRUEN / BLAU / ? ----
static bool cmdCal(const String& cmd) {
  char buf[48];
  if (cmd == "CAL:BLACK") {
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
      xSemaphoreGive(colorSem);
    }
    configSaveUShort("blackR", config.blackR);
    configSaveUShort("blackG", config.blackG);
    configSaveUShort("blackB", config.blackB);
    sendStatus("CAL:BLACK OK");
    return true;
  }
  if (cmd == "CAL:WHITE") {
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
      xSemaphoreGive(colorSem);
    }
    configSaveUShort("whiteR", config.whiteR);
    configSaveUShort("whiteG", config.whiteG);
    configSaveUShort("whiteB", config.whiteB);
    sendStatus("CAL:WHITE OK");
    return true;
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
      return true;
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
    return true;
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
    return true;
  }
  return false;
}

// ---- TMC2209-Treiber: Strom / Hold / Microsteps / Modus / EN / Diagnose ----
static bool cmdTmc(const String& cmd) {
  char buf[48];
  // Set-Befehle werden im NVS gespeichert und ueberleben einen Reboot.
  // Nach jeder Aenderung kommt zusaetzlich eine TMC:STAT:-Kompaktzeile.
  if (cmd == "TMC?") { tmcPrintStatus(); sendStatus("TMC:LOGGED"); tmcSendCompactStatus(); return true; }
  if (cmd == "UARTTEST") { tmcUartDiag(); sendStatus("UART:DIAG_DONE"); return true; }
  if (cmd == "UARTNET") { tmcNetDiag(); sendStatus("NETZ:DIAG_DONE"); return true; }
  if (cmd.startsWith("TMC:CURRENT:")) {
    int v = cmd.substring(12).toInt();
    if (tmcSetCurrent((uint16_t)v)) {
      config.tmcCurrent = (uint16_t)v; configSaveUShort("tmcCur", config.tmcCurrent);
      snprintf(buf, sizeof(buf), "TMC:CURRENT=%u", tmcGetCurrent()); sendStatus(buf);
      tmcSendCompactStatus();
    } else {
      sendStatus("TMC:BAD_CURRENT");
    }
    return true;
  }
  if (cmd.startsWith("TMC:HOLD:")) {
    int v = cmd.substring(9).toInt();
    if (tmcSetHold((uint8_t)v)) {
      config.tmcHold = (uint8_t)v; configSaveUChar("tmcHold", config.tmcHold);
      snprintf(buf, sizeof(buf), "TMC:HOLD=%u", tmcGetHold()); sendStatus(buf);
      tmcSendCompactStatus();
    } else {
      sendStatus("TMC:BAD_HOLD");
    }
    return true;
  }
  if (cmd.startsWith("TMC:MS:")) {
    int v = cmd.substring(7).toInt();
    if (tmcSetMicrosteps((uint16_t)v)) {
      config.tmcMicrosteps = (uint16_t)v; configSaveUShort("tmcMS", config.tmcMicrosteps);
      snprintf(buf, sizeof(buf), "TMC:MS=%u", tmcGetMicrosteps()); sendStatus(buf);
      tmcSendCompactStatus();
    } else {
      sendStatus("TMC:BAD_MS");
    }
    return true;
  }
  if (cmd == "TMC:MODE:SPREAD") {
    tmcSetSpreadCycle(true); config.tmcSpread = true; configSaveBool("tmcSpread", true);
    sendStatus("TMC:MODE=SPREAD"); tmcSendCompactStatus(); return true;
  }
  if (cmd == "TMC:MODE:STEALTH") {
    tmcSetSpreadCycle(false); config.tmcSpread = false; configSaveBool("tmcSpread", false);
    sendStatus("TMC:MODE=STEALTH"); tmcSendCompactStatus(); return true;
  }
  if (cmd == "TMC:EN:1") { tmcEnable(); sendStatus("TMC:EN=1"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:EN:0") { tmcDisable(); sendStatus("TMC:EN=0"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:INIT") { tmcReinit(); sendStatus("TMC:INIT_DONE"); tmcSendCompactStatus(); return true; }
  return false;
}

// ---- Geometrie-Einlernung: STEPS (Vorwaertstrecke) + ANG (Drehung) ----
static bool cmdSteps(const String& cmd) {
  char buf[48];
  if (cmd == "STEPS?") {
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)config.quizForwardSteps);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("STEPS:")) {
    int32_t n = (int32_t)cmd.substring(6).toInt();
    if (n < 1 || n > 2000) { sendStatus("STEPS:BAD"); return true; }
    config.quizForwardSteps = n;
    configSaveInt("quizSteps", n);
    // Sofort testen
    testDriveSteps(n);
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[STEPS] Test %ld Schritte gefahren (fuer 3cm-Quiz gespeichert)\n", (long)n);
    return true;
  }
  return false;
}

static bool cmdAng(const String& cmd) {
  char buf[48];
  if (cmd == "ANG?") {
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)config.quizTurnSteps);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("ANG:")) {
    int32_t n = (int32_t)cmd.substring(4).toInt();
    if (n < 1 || n > 5000) { sendStatus("ANG:BAD"); return true; }
    config.quizTurnSteps = n;
    configSaveInt("quizTurnSteps", n);
    // Sofort testen (Drehung rechts auf der Stelle)
    testTurnSteps(n);
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[ANG] Test %ld Schritte/Rad gedreht (fuer 90-Grad-Spiel gespeichert)\n", (long)n);
    return true;
  }
  return false;
}

// ---- Farb-Saettigungs-Schwelle (Boden-Differenzierung) ----
static bool cmdSat(const String& cmd) {
  char msg[32];
  if (cmd == "SAT?") {
    snprintf(msg, sizeof(msg), "SAT=%.2f", config.minColorSaturation);
    sendStatus(msg);
    return true;
  }
  if (cmd.startsWith("SAT:")) {
    float v = cmd.substring(4).toFloat();
    if (v < 0.0f || v > 1.0f) { sendStatus("SAT:BAD"); return true; }
    config.minColorSaturation = v;
    configSaveField("minSat", v);
    snprintf(msg, sizeof(msg), "SAT=%.2f", v);
    sendStatus(msg);
    return true;
  }
  return false;
}

// ---- gemeinsamer Helfer: MAC-String "AA:BB:CC:DD:EE:FF" -> 6 Bytes ----
// Erwartet genau 6 durch ':' getrennte Hex-Bytes. Gibt false zurueck, wenn
// das Format nicht passt (dann bleibt mac unveraendert).
static bool parseMac(const String& macStr, uint8_t mac[6]) {
  memset(mac, 0, 6);
  int idx = 0, pos = 0;
  while (idx < 6 && pos + 1 < macStr.length()) {
    char hex[3] = { macStr.charAt(pos), macStr.charAt(pos + 1), 0 };
    mac[idx++] = (uint8_t)strtol(hex, nullptr, 16);
    pos += 3;   // 2 Hex-Zeichen + Trenner
  }
  return idx == 6;
}

// ---- KORRIGIEREN (Farb-Quiz): QUIZ:CHECK:<ROT,GELB,...> / QUIZ:STOP ----
static bool cmdQuiz(const String& cmd) {
  if (!cmd.startsWith("QUIZ:")) return false;
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
        return true;
      }
    }
    korrigierenExecute();
  } else if (sub == "STOP") {
    currentGame = GAME_NONE;
    korrigierenRunning = false;
    stopMotors();
    sendStatus("QUIZ:STOPPED");
  }
  return true;
}

// ---- RENNEN (Multiplayer): RACE:START/STEP/WIN/LOSE/SCAN/UNPAIR/PAIR:/PROGRESS: ----
static bool cmdRace(const String& cmd) {
  if (!cmd.startsWith("RACE:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub == "START") {
    raceExecuteStart();
    raceSendPacket(0, false);
  } else if (sub == "STEP") {
    if (!raceActive) return true;
    raceMyScore++;
    raceForwardStep();
    if (!raceActive) return true;
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
    uint8_t mac[6];
    if (parseMac(sub.substring(5), mac)) {
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
  return true;
}

// ---- LINIE FOLGEN: LINE:START / LINE:STOP ----
static bool cmdLine(const String& cmd) {
  if (!cmd.startsWith("LINE:")) return false;
  String sub = cmd.substring(5);
  if (sub == "START") {
    if (!lineFollowerActive) {
      currentGame = GAME_LINEFOLLOWER;
      lineFollowerStart();
      sendStatus("LINE:RUNNING");
    }
  } else if (sub == "STOP") {
    lineFollowerStop();
    currentGame = GAME_NONE;
    sendStatus("LINE:STOPPED");
  }
  return true;
}

// ---- MEMORY: MEMORY:START / MEMORY:START:<n> / MEMORY:STOP / MEMORY:INPUT:<n> ----
static bool cmdMemory(const String& cmd) {
  if (!cmd.startsWith("MEMORY:")) return false;
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
  return true;
}

// ---- PROGRAMMIEREN (Code-Bausteine): PROG:SET:<enc> / PROG:RUN / PROG:STOP ----
static bool cmdProg(const String& cmd) {
  if (!cmd.startsWith("PROG:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub.startsWith("SET:")) {
    String enc = sub.substring(4);
    if (progSetFromString(enc.c_str())) {
      snprintf(buf, sizeof(buf), "PROG:READY:%u", progLen);
      sendStatus(buf);
    } else {
      sendStatus("PROG:BAD");
    }
  } else if (sub == "RUN") {
    progRun();
  } else if (sub == "STOP") {
    progStop();
  }
  return true;
}

// ---- SLALOM (Farben-Slalom Multiplayer) ----
// Eigenstaendiges Pairing (getrennt vom Race-Pairing), siehe Slalom.h.
static bool cmdSlalom(const String& cmd) {
  if (!cmd.startsWith("SLALOM:")) return false;
  String sub = cmd.substring(7);
  if (sub.startsWith("PROG:")) {
    String enc = sub.substring(5);
    slalomSetProgram(enc.c_str());
  } else if (sub.startsWith("INVITE:")) {
    // SLALOM:INVITE:<MAC> - Initiator (Host) schickt Einladung an Peer
    if (slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:ALREADY_PAIRED");
      return true;
    }
    uint8_t mac[6];
    if (!parseMac(sub.substring(7), mac)) {
      sendStatus("SLALOM:ERROR:BAD_MAC");
      return true;
    }
    // CMD_INVITE an den Peer senden (senderMac identifiziert uns)
    slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, mac);
    sendStatus("SLALOM:INVITE_SENT");
    Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else if (sub == "INVITE_ACCEPT") {
    // Web des Empfaengers hat das Pop-up bestaetigt
    if (!slalomInvitePending()) {
      sendStatus("SLALOM:ERROR:NO_INVITE");
      return true;
    }
    // CMD_INVITE_ACK an den Initiator schicken
    slalomSendPacketTo(SLALOM_CMD_INVITE_ACK, nullptr, slalomInviterMacPtr());
    // Wir sind Client (nicht Initiator)
    slalomPairWith(slalomInviterMacPtr(), false);
    slalomClearInvite();
    sendStatus("SLALOM:INVITE_ACCEPTED");
  } else if (sub == "INVITE_DECLINE") {
    // Einladung abgelehnt
    if (slalomInvitePending()) {
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomInviterMacPtr());
    }
    slalomClearInvite();
  } else if (sub == "UNPAIR") {
    // Slalom-spezifisches Unpair (nur Slalom-State, nicht das Race-Pairing)
    if (slalomHasPeer()) {
      // Dem Peer mitteilen, dass wir uns trennen
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomPeerMacPtr());
    }
    slalomUnpair();
  } else if (sub == "START_MATCH") {
    // Web-Befehl (Host): Spiel starten, Zufallsfarbe, Client in Bauphase holen
    if (!slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:NOT_PAIRED");
      return true;
    }
    if (!slalomIsHost) {
      sendStatus("SLALOM:ERROR:NOT_HOST");
      return true;
    }
    slalomStartGame();
  } else if (sub == "STOP") {
    // Spiel manuell abbrechen -> beide zurueck zu PAIRED
    slalomAbortGame();
  }
  return true;
}

void handleCommand(const char* cmdStr) {
  String cmd(cmdStr);
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

  if (cmd.startsWith("KEYS:")) { cmdKeys(cmd); return; }

  Serial.printf("[CMD] %s\n", cmd.c_str());
  if (cmd == "STOP") { hardStop(); return; }


  if (cmdSteps(cmd)) return;   // STEPS? / STEPS:<1..2000>  (Vorwaertstrecke, faehrt sofort)
  if (cmdAng(cmd))   return;   // ANG? / ANG:<1..5000>      (90-Grad-Drehung, faehrt sofort)
  if (cmdSat(cmd))   return;   // SAT? / SAT:<0..1>         (Mindest-Saettigung)


  // ---- Dispatch: erste passende Gruppe gewinnt ----
  if (cmdDist(cmd))  return;   // DIST / OBSDIST / OBSREL / DISTCAL / I2C / DISTDIAG
  if (cmdCal(cmd))   return;   // CAL:BLACK/WHITE/ROT/GELB/GREEN/BLUE/?
  if (cmdTmc(cmd))   return;   // TMC:* / UARTTEST / UARTNET
  if (cmdQuiz(cmd))  return;   // QUIZ:CHECK / QUIZ:STOP
  if (cmdRace(cmd))  return;   // RACE:START/STEP/WIN/... / PAIR: / PROGRESS:
  if (cmdSlalom(cmd))return;   // SLALOM:PROG/INVITE/START_MATCH/UNPAIR/STOP
  if (cmdLine(cmd))  return;   // LINE:START / LINE:STOP
  if (cmdMemory(cmd))return;   // MEMORY:START / STOP / INPUT:
  if (cmdProg(cmd))  return;   // PROG:SET / RUN / STOP
  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V2 - Modular (2x TMC2209 + VL53L0X)");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
  esp_task_wdt_add(NULL);

  configLoad();

  initSteppers();
  Serial.println("[MOTOR] Motor-Task auf Core 0 gestartet (feste Geschwindigkeit, keine Beschleunigung)");
  startupSpin();
  tmcBootDiag();

  lastCommandTime = millis();

  // VL53L0X VOR dem Farbsensor initialisieren: der Treiber braucht 0x29 fuer
  // sich allein, waehrend er seine NVM-Daten liest (siehe DistanceSensor.h).
  //
  // ACHTUNG: Diese Reihenfolge allein reicht NICHT, wenn beide Sensoren
  // physisch am Bus haengen. Ein auch noch nicht konfigurierter TCS34725
  // antwortet auf 0x29 und verfaelscht damit die NVM-Reads des VL53L0X
  // (Folge: -50 REF_SPAD_INIT und dann nur ~10 % brauchbare Messungen).
  // Abhilfe ohne Loeten ist nicht moeglich: der TCS34725 hat eine feste
  // Adresse und antwortet immer auf 0x29. Verifiziert funktionsfaehig:
  // 0x29 exklusiv (Farbsensor abgezogen) -> 46/46 gueltige Messungen,
  // 191-198 mm bei 20 cm Ziel, Range-Status 0.
  distSetup();
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
  ledStripSetup();
  Serial.println("======================================\n");
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastColorNotify = 0;
void loop() {
  esp_task_wdt_reset();
  bleProcessEvents();
  espNowProcessEvents();
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
  // ============================================================
  // Slalom-Timer:
  //   slalomGoPending  : 3s-Countdown nach BOTH_READY (HOST zaehlt,
  //                      Client nur Fallback 4.5s bei Paketverlust)
  //   slalomCheckRetry : Retry, wenn kein Treffer (beide MISS oder Timeout)
  // ============================================================

  // (1) GO-Timer: Host zaehlt 3s, sendet GO 3x und startet selbst.
  //     Client: Fallback, falls das GO-Paket verloren ging.
  if (slalomGoPending && !slalomRunning && slalomPhase == SLALOM_COUNTDOWN &&
      millis() >= slalomGoTimer) {
    slalomGoPending = false;
    if (slalomIsHost) {
      // GO 3x senden (Paketverlust-Retry). Der Client startet beim ersten
      // empfangenen Paket und ignoriert die Wiederholungen (Phase ist dann
      // RACING). So starten beide Roboter trotz ESP-NOW-Paketverlust
      // synchron statt mit 1.5s Client-Fallback-Versatz.
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      Serial.println("[SLALOM] HOST-GO-Timer: GO 3x gesendet, starte selbst");
    } else {
      // Fallback: GO-Paket verloren - trotzdem starten (leicht verspaetet)
      Serial.println("[SLALOM] Client-Fallback-GO: kein Paket erhalten, starte trotzdem");
    }
    slalomExecute();
  }
  // (2) Retry: beide Durchlaeufe ohne Treffer (oder 15s-Timeout) -> BUILD
  slalomCheckRetry();
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  // VL53L0X-Distanz: alle DIST_NOTIFY_MS als DIST:<mm> (nur bei BLE, nur wenn AUTO an)
  // Hindernis-Status fuer ProgGame/Slalom: distBlocked() (DIST < OBSTACLE_DIST_MM)
  distTick();
  // TMC-Auto-Standby: Treiber aus, wenn laenger als TMC_STANDBY_MS nichts faehrt
  tmcAutoStandbyTick();
  // LED-Streifen: Alert > Zielfarbe > Status (max. 4x/s aktualisieren)
  {
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate >= 250) {
      lastLedUpdate = millis();
      if (ledAlertActive) {
        ledStripTick();
      } else if (slalomTargetColor[0] != '\0') {
        ledStripShowTarget(slalomTargetColor);
      } else {
        ledStripShowStatus(deviceConnected);
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
