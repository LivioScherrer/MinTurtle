
#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

#include "Config.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "I2CBus.h"
#include "BLEManager.h"
#include "ApSignal.h"
#include "WebUi.h"
#include "WebUiConfig.h"
#include "Drive.h"
#include "EspNow.h"
#include "MemoryGame.h"
#include "GameEngine.h"
#include "LineFollower.h"
#include "ProgGame.h"
#include "LedStrip.h"
#include "DistanceSensor.h"
#include "Slalom.h"
#include "Lobby.h"
#include "GameCommon.h"
#include "AmpelGame.h"
#include "ZahlGame.h"
#include "GeheimGame.h"
#include "ParkGame.h"
#include "ZeitGame.h"
#include "Duel.h"

constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 1;

static bool claimGameSlot(const char* busyMsg, const char* what);


GameMode currentGame = GAME_NONE;
bool driveLockedByGame() { return currentGame != GAME_NONE && !gameAllowsManual(); }

constexpr uint32_t GAME_TICK_MS    = 1;
const uint32_t GAME_MOVE_MS       = 360;
const uint32_t GAME_COLOR_READ_MS = 250;
constexpr uint8_t QUIZ_MAX_COLOR_USE = 4;

bool gameMoveBusy = false;

static volatile bool s_cmdFromBle = false;

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

static void waitForStop(uint32_t maxExtraMs = 1000) {
  const uint32_t extra = millis();
  while (motorStillRunning() && millis() - extra < maxExtraMs) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

void gameDrive(int32_t ms, bool bForward) {
  gameMoveBusy = true;
  const float target = bForward ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED;
  setMotorSpeeds(target, target, false);
  waitWhileRunning((uint32_t)ms, true);
  setMotorSpeeds(0, 0, false);
  waitForStop();
  gameMoveBusy = false;
}

void gameDriveForward(uint32_t ms)  { gameDrive((int32_t)ms, true); }
void gameDriveBackward(uint32_t ms) { gameDrive((int32_t)ms, false); }

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  setMotorSpeeds(0, 0, false);
  waitWhileRunning(ms, false);
  gameMoveBusy = false;
}

static void waitForStepsDone(bool bAbortOnNoGame);

static void gameMoveSteps(int32_t physL, int32_t physR,
                          bool bAbortOnNoGame, int32_t budgetMs) {
  prepPositionMoves(physL, physR);
  if (budgetMs > 0) {
    const uint32_t start = millis();
    while (motorStillRunning() && (int32_t)(millis() - start) < budgetMs) {
      esp_task_wdt_reset();
      bleProcessEvents();
      vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
    }
    if (motorStillRunning()) stopMotors();
  } else {
    waitForStepsDone(bAbortOnNoGame);
  }
}

static void waitForStepsDone(bool bAbortOnNoGame) {
  static constexpr uint32_t WAIT_STEPS_TIMEOUT_MS = 20000;
  const uint32_t start = millis();
  while (motorStillRunning()) {
    esp_task_wdt_reset();
    if (bAbortOnNoGame && currentGame == GAME_NONE) {
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    if (millis() - start > WAIT_STEPS_TIMEOUT_MS) {
      Serial.printf("[GAME] Schritte nach %u ms nicht fertig - Abbruch\n",
                    (unsigned)WAIT_STEPS_TIMEOUT_MS);
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  gameMoveSteps(steps * static_cast<long>(config.leftSign),
                        steps * static_cast<long>(config.rightSign), true, 0);
  gameMoveBusy = false;
}

void gameDriveBackwardSteps(int32_t steps) {
  if (steps < 0) steps = -steps;
  gameMoveBusy = true;
  gameMoveSteps(-(long)steps * (long)config.leftSign,
                        -(long)steps * (long)config.rightSign, true, 0);
  gameMoveBusy = false;
}

void testDriveSteps(int32_t steps) {
  gameMoveSteps(steps * static_cast<long>(config.leftSign),
                        steps * static_cast<long>(config.rightSign), false, 5000);
}

static void gameTurnSteps(int32_t steps, bool bLeft) {
  gameMoveBusy = true;
  const long l = bLeft ? -steps : steps;
  const long r = bLeft ? steps : -steps;
  gameMoveSteps(l * static_cast<long>(config.leftSign),
                        r * static_cast<long>(config.rightSign), true, 0);
  gameMoveBusy = false;
}

void gameTurnLeftSteps(int32_t steps)  { gameTurnSteps(steps, true); }
void gameTurnRightSteps(int32_t steps) { gameTurnSteps(steps, false); }

void testTurnSteps(int32_t steps) {
  gameMoveSteps( steps * static_cast<long>(config.leftSign),
                        -steps * static_cast<long>(config.rightSign), false, 5000);
}

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
    if (currentGame != GAME_KORRIGIEREN) {
      korrigierenRunning = false;
      stopMotors();
      sendStatus("QUIZ:RESULT:ABORT");
      return;
    }
    gameDriveForwardSteps(configForwardSteps());
    gameStop(GAME_COLOR_READ_MS);
    ledStripClear();
    char found[16] = "UNBEKANNT";
    getCurrentColorName(found, sizeof(found));
    strlcpy(korrigierenFound[i], found, sizeof(korrigierenFound[i]));
    if (strcasecmp(korrigierenExpected[i], found) == 0) okCount++;
    char part[48];
    snprintf(part, sizeof(part), "QUIZ:STEP:%u,%s,%s", i, korrigierenExpected[i], found);
    sendStatus(part);
    ledStripShowProgress(i + 1, korrigierenLen,
                         (okCount == i + 1) ? LED_C_GREEN : LED_C_RED);
    ledStripTick();
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
  ledStripShowResultFlash(okCount == korrigierenLen);
  ledStripTick();
  sendStatus(result);
}

int  raceMyScore = 0;
int  raceOppScore = 0;
bool raceActive = false;
bool quizActive = false;
uint8_t raceTargetScore = RENNEN_TARGET_SCORE;
uint8_t raceStage = 0;
float raceMMPerPoint = 0.0f;
unsigned long raceLastOppMs = 0;

void raceExecuteStart(bool remote = false) {
  currentGame = GAME_RENNEN;
  raceMyScore = 0;
  raceOppScore = 0;
  raceActive = true;
  raceLastOppMs = 0;
  sendStatus(remote ? "RACE:REMOTE_START" : "RACE:STARTED");
}

void raceForwardStepMM(float mm) {
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; return; }
  if (mm > 0.0f) {
    const int32_t steps = geoStepsForMM(mm, config.tmcMicrosteps);
    if (steps < 1) return;
    gameDriveForwardSteps(steps);
  } else {
    gameDriveForwardSteps(configForwardSteps());
  }
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; }
}

void raceForwardStep() { raceForwardStepMM(raceMMPerPoint); }

void raceAbort(bool notifyPeer) {
  raceActive = false;
  currentGame = GAME_NONE;
  stopMotors(); speedL = 0; speedR = 0;
  if (notifyPeer) raceSendPacket((uint8_t)raceMyScore, false);
  sendStatus("RACE:STOPPED");
}

bool racePeerTimedOut() {
  if (!raceActive || !hasPairedPeer || raceLastOppMs == 0) return false;
  return (unsigned long)(millis() - raceLastOppMs) > RACE_PEER_TIMEOUT_MS;
}

void onSlalomPacket(const EduPacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

void onRacePacket(const EduPacket* pkt) {
  if (raceActive) raceLastOppMs = millis();
  if (pkt->flags & RACE_FLAG_WINNER || pkt->score >= raceTargetScore) {
    raceOppScore = pkt->score;
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    sendStatus("RACE:OPP_WIN");
  } else {
    if (!raceActive) {
      if (lobbyGameBusy()) {
        sendStatus("RACE:BUSY");
        Serial.println("[RACE] Score-0 verworfen: anderes Spiel laeuft");
        return;
      }
      raceExecuteStart(true);
    }
    raceOppScore = pkt->score;
    char msg[32];
    snprintf(msg, sizeof(msg), "RACE:PROGRESS:%d", raceOppScore);
    sendStatus(msg);
  }
}


static void clientGained() {
  lastCommandTime = millis();
  Serial.println("[BLE] Client verbunden");
  apSignalReset();
  edgeGuardReset();
  motorReleaseEmergency();
  if (colorSensorOk) {
    uint16_t rC = 0;
    int cR = 0, cG = 0, cB = 0;
    bool have = false;
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      rC = rawSensorC;
      cR = sensorR; cG = sensorG; cB = sensorB;
      have = true;
      xSemaphoreGive(colorSem);
    }
    if (have && rC >= 150) {
      floorLearn((uint8_t)constrain(cR, 0, 255), (uint8_t)constrain(cG, 0, 255),
                 (uint8_t)constrain(cB, 0, 255), rC);
      char fb[64];
      snprintf(fb, sizeof(fb), "FLOOR=%d,%d,%d,%u", cR, cG, cB, (unsigned)rC);
      sendStatus(fb);
    } else {
      Serial.println("[BODEN] keine gueltige Messung beim Verbinden - alter Boden bleibt");
    }
  }
  sendStatus("EduBot V2 bereit");
}

static void clientLost() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  tmcDisable();
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
  ampelStop();
  zahlStop();
  geheimStop();
  parkStop();
  zeitStop();
  duelStop();
  if (lobbyHasPeer()) lobbySendPacket(GAME_CMD_STOP, nullptr, true);
  Serial.println("[BLE] Letzter Client getrennt - Werbung gestartet");
  BLEDevice::startAdvertising();
  espNowEnsureChannel();
}

void onBleConnected() {
  deviceConnected = true;
  clientGained();
}

void onBleDisconnected() {
  deviceConnected = false;
  if (anyClientConnected()) {
    Serial.println("[BLE] Getrennt - Web-Client ist noch da, Spiel laeuft weiter");
    return;
  }
  clientLost();
}

void onWebClientConnected() {
  clientGained();
}

void onWebClientDisconnected() {
  if (anyClientConnected()) {
    Serial.println("[WEB] Getrennt - Bluetooth-Client ist noch da, Spiel laeuft weiter");
    return;
  }
  clientLost();
}

void onSlalomPacket(const EduPacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  tmcDisable();
  currentGame = GAME_NONE;
  korrigierenLen = 0;
  korrigierenRunning = false;
  raceActive = false;
  memoryStop();
  lineFollowerStop();
  progStop();
  ampelStop();
  zahlStop();
  geheimStop();
  parkStop();
  zeitStop();
  duelStop();
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  if (lobbyHasPeer()) lobbySendPacket(GAME_CMD_STOP, nullptr, true);
  ledStripShowAlert();
  sendStatus("STOP");
}

static void cmdHalt() {
  driveClearKeys();
  speedL = 0.0f; speedR = 0.0f;
  stopMotors();
  gameHaltSet();
  if (currentGame == GAME_NONE) gameHaltClear();
  sendStatus("HALT:OK");
}

static bool cmdIsGameStop(const String& cmd) {
  return cmd == "LINE:STOP"  || cmd == "MEMORY:STOP" || cmd == "QUIZ:STOP"
      || cmd == "RACE:STOP"  || cmd == "PROG:STOP"  || cmd == "SLALOM:STOP"
      || cmd == "AMPEL:STOP" || cmd == "ZAHL:STOP" || cmd == "GEHEIM:STOP"
      || cmd == "PARK:STOP"  || cmd == "ZEIT:STOP";
}

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


  if (cmd == "EDGE:ON") {
    edgeGuardSet(true);
    sendStatus("EDGE:ON");
    return true;
  }
  if (cmd == "EDGE:OFF") {
    edgeGuardSet(true);
    sendStatus("EDGE:ON");
    Serial.println("[KANTE] EDGE:OFF ignoriert - Waechter ist immer aktiv");
    return true;
  }
  if (cmd == "EDGE:RESET") {
    edgeGuardReset();
    motorReleaseEmergency();
    sendStatus("EDGE:RESET");
    Serial.println("[KANTE] Zaehler zurueckgesetzt - Roboter wieder freigegeben");
    return true;
  }
  if (cmd == "EDGE?") {
    char eb[48];
    snprintf(eb, sizeof(eb), "EDGE=%u,%u,%u,%u",
             edgeGuardActive() ? 1u : 0u, (unsigned)rawSensorC, (unsigned)rawSensorR,
             motorEmergencyActive() ? 1u : 0u);
    sendStatus(eb);
    Serial.printf("[KANTE] %s | Clear=%u (Kante < %u, frei ab %u) | Rot=%u%s\n",
                  edgeGuardActive() ? "aktiv" : "aus",
                  (unsigned)rawSensorC, (unsigned)edgeGuardMaxNow(),
                  (unsigned)edgeGuardReleaseNow(), (unsigned)rawSensorR,
                  motorEmergencyActive() ? " | NOT-HALT aktiv" : "");
    return true;
  }
  if (cmd == "FLOOR?") {
    char fb[64];
    snprintf(fb, sizeof(fb), "FLOOR=%u,%u,%u,%u,%u",
             config.floorCalibrated ? 1u : 0u, config.floorR, config.floorG,
             config.floorB, (unsigned)config.floorC);
    sendStatus(fb);
    Serial.printf("[BODEN] %s rgb(%u,%u,%u) Clear=%u\n",
                  config.floorCalibrated ? "gesetzt" : "keiner",
                  config.floorR, config.floorG, config.floorB,
                  (unsigned)config.floorC);
    return true;
  }
  if (cmd == "DISTDIAG" || cmd == "DIST:DIAG") {
    distDiagPrint();
  char dbuf[128];
  distDiag(dbuf, sizeof(dbuf));
    sendStatus(dbuf);
    return true;
  }
  if (cmd == "DISTREF") {
    if (distRefSpadRequest()) {
      sendStatus("DISTREF:RUN");
    } else {
      sendStatus("DISTREF:BUSY");
    }
    return true;
  }
  if (cmd == "DISTDEBUG") {
    distDebugInit();
    return true;
  }
  if (cmd == "DISTSCAN" || cmd == "DIST:SCAN") {
    distBootScan();
    sendStatus("DIST:SCAN_DONE");
    return true;
  }
  if (cmd == "OBSDIST?") {
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("OBSDIST:")) {
    int v = cmd.substring(8).toInt();
    if (v < 50 || v > 1200) { sendStatus("OBSDIST:BAD"); return true; }
    int rel = min(v + 30, 1200);
    if (rel <= v) { sendStatus("OBSDIST:BAD"); return true; }
    config.obsDistMM = (int16_t)v;
    config.obsRelMM  = (int16_t)rel;
    configSaveInt("obsDist", v);
    configSaveInt("obsRel", rel);
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
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
    int st = max(v - 30, 50);
    if (st >= v) { sendStatus("OBSREL:BAD"); return true; }
    config.obsRelMM  = (int16_t)v;
    config.obsDistMM = (int16_t)st;
    configSaveInt("obsRel", v);
    configSaveInt("obsDist", st);
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
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
  if (cmd.startsWith("DISTMEAS:")) {
    int v = cmd.substring(9).toInt();
    if (v < 20 || v > 500) { sendStatus("DISTMEAS:BAD"); return true; }
    int16_t raw = -1;
    if (distMeasureRawAvg(raw)) {
      snprintf(buf, sizeof(buf), "DISTMEAS=%d,%d", v, (int)raw);
    } else {
      snprintf(buf, sizeof(buf), "DISTMEAS:FAIL");
    }
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("DISTCAL3:")) {
    int r[3] = { 0, 0, 0 };
    int idx = 0;
    bool ok = true;
    const char* p = cmd.c_str() + 9;
    while (*p != '\0' && idx < 3) {
      char* end = nullptr;
      long val = strtol(p, &end, 10);
      if (end == p || val <= 0 || val > 2000) { ok = false; break; }
      r[idx++] = (int)val;
      p = end;
      if (*p == ',') p++;
    }
    if (!ok || idx != 3) { sendStatus("DISTCAL3:BAD"); return true; }
    if (distCalibrate3((int16_t)r[0], (int16_t)r[1], (int16_t)r[2])) {
      snprintf(buf, sizeof(buf), "DISTCAL3=%.3f,%d",
               (double)config.obsScale, (int)config.obsOffsetMM);
      sendStatus(buf);
    } else {
      sendStatus("DISTCAL3:BAD");
    }
    return true;
  }
  if (cmd == "I2C?") {
    static constexpr size_t I2C_SCAN_MAX_LEN = 200;
    char out[I2C_SCAN_MAX_LEN + 1];
    static constexpr size_t I2C_TAIL_RESERVE = 12;
    static constexpr size_t I2C_LIST_MAX     = I2C_SCAN_MAX_LEN - I2C_TAIL_RESERVE;
    memcpy(out, "I2C:", 4);
    size_t outLen = 4;
    uint16_t found = 0;
    uint16_t listed = 0;
    if (!i2cBusLock(500)) {
      Serial.println("[I2C] Scan nicht moeglich: Bus belegt");
      sendStatus("I2C:BUSY");
      return true;
    }
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        found++;
        if (outLen + 3 > I2C_LIST_MAX) continue;
        snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen, " %02X", a);
        outLen += 3;
        listed++;
      }
    }
    i2cBusUnlock();
    if (found > listed) {
      snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen,
               " +%u", (unsigned)(found - listed));
    } else {
      snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen,
               " :n=%u", (unsigned)found);
    }
    sendStatus(out);
    Serial.printf("[I2C] %u Adressen (%u gelistet): %s\n",
                  (unsigned)found, (unsigned)listed, out + 4);
    return true;
  }
  return false;
}

static bool cmdWebUi(const String& cmd) {
  if (cmd == "WEB:?") {
    char buf[96];
    if (webUiIsUp()) {
      snprintf(buf, sizeof(buf), "WEB:ON:%u,%s", (unsigned)webUiClientCount(),
               WiFi.channel() == WEBUI_AP_CHANNEL ? "CH1" : "CH?");
      sendStatus(buf);
      Serial.printf("[WEB] WLAN AN: Kanal %u, %u Client(s), SSID %s\n",
                    (unsigned)WiFi.channel(), (unsigned)webUiClientCount(),
                    webUiSsid().c_str());
    } else {
      snprintf(buf, sizeof(buf), "WEB:OFF:0");
      sendStatus(buf);
      Serial.printf("[WEB] WLAN AUS: bereit per Farbfolge, SSID waere %s\n",
                    webUiSsid().c_str());
    }
    return true;
  }
  if (cmd == "WEBAP:AUTO:?") {
    sendStatus(webUiAutoEnabled() ? "WEBAP:AUTO:ON" : "WEBAP:AUTO:OFF");
    return true;
  }
  if (cmd == "WEBAP:AUTO:ON" || cmd == "WEBAP:AUTO:OFF") {
    const bool wantOn = (cmd == "WEBAP:AUTO:ON");
    if (!webUiSetAutoEnabled(wantOn)) {
      sendStatus("WEBAP:AUTO:FAILED");
      return true;
    }
    sendStatus(wantOn ? "WEBAP:AUTO:ON" : "WEBAP:AUTO:OFF");
    Serial.printf("[WEB] Autostart %s gespeichert - gilt ab dem naechsten Boot\n",
                  wantOn ? "eingeschaltet" : "ausgeschaltet");
    return true;
  }
  if (cmd == "WEBPW:GET") {
    if (s_cmdFromBle) {
      sendStatus("WEBPW:LOCKED");
      Serial.println("[WEBPW] GET ueber Web-UI abgelehnt - nur ueber Serial");
      return true;
    }
    String pw = webUiPassword();
    Serial.printf("[WEBPW] WLAN \"%s\", Passwort: %s\n", webUiSsid().c_str(), pw.c_str());
    Serial.println("[WEBPW] QR-Inhalt zum Aufkleber:");
    Serial.printf("WIFI:T:WPA;S:%s;P:%s;;\n", webUiSsid().c_str(), pw.c_str());
    sendStatus("WEBPW:SENT");
    return true;
  }
  if (cmd == "WEBPW:RESET") {
    if (s_cmdFromBle) {
      sendStatus("WEBPW:LOCKED");
      Serial.println("[WEBPW] RESET ueber Web-UI abgelehnt - nur ueber Serial");
      return true;
    }
    String pw = webUiResetPassword();
    Serial.printf("[WEBPW] Neues Passwort: %s\n", pw.c_str());
    Serial.println("[WEBPW] Der Roboter muss neu gestartet werden, damit der");
    Serial.println("[WEBPW] Access Point das neue Passwort verwendet.");
    Serial.printf("WIFI:T:WPA;S:%s;P:%s;;\n", webUiSsid().c_str(), pw.c_str());
    sendStatus("WEBPW:RESET");
    return true;
  }
  if (cmd == "WEBAP:ON") {
    if (s_cmdFromBle) {
      sendStatus("WEBAP:LOCKED");
      Serial.println("[WEBAP] ON ueber Web-UI abgelehnt - nur ueber Serial");
      return true;
    }
    webUiStart();
    sendStatus(webUiIsUp() ? "WEBAP:ON" : "WEBAP:FAILED");
    return true;
  }
  if (cmd == "WEBAP:OFF") {
    if (s_cmdFromBle) {
      sendStatus("WEBAP:LOCKED");
      Serial.println("[WEBAP] OFF ueber Web-UI abgelehnt - nur ueber Serial");
      return true;
    }
    webUiStop();
    sendStatus("WEBAP:OFF");
    return true;
  }
  if (cmd == "WEBDIAG") {
    webUiPrintDiag();
    sendStatus("WEBDIAG:DONE");
    return true;
  }
  return false;
}

static bool cmdCal(const String& cmd) {
  const bool isWrite = (cmd == "CAL:BLACK" || cmd == "CAL:WHITE" ||
                        cmd == "CAL:RED" || cmd == "CAL:YELLOW" ||
                        cmd == "CAL:GREEN" || cmd == "CAL:BLUE");
  if (isWrite && s_cmdFromBle) {
    sendStatus("CAL:LOCKED");
    Serial.printf("[CAL] %s ueber Web-UI abgelehnt - Kalibrierung ist gesperrt "
                  "(geht nur ueber Serial)\n", cmd.c_str());
    return true;
  }
  if (cmd == "CAL:BLACK") {
    if (!colorSem || xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      sendStatus("CAL:BLACK BUSY");
      Serial.println("[CAL] BLACK: Farb-Sperre nicht bekommen - nicht gespeichert");
      return true;
    }
    config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
    xSemaphoreGive(colorSem);
    configSaveUShort("blackR", config.blackR);
    configSaveUShort("blackG", config.blackG);
    configSaveUShort("blackB", config.blackB);
    sendStatus("CAL:BLACK OK");
    return true;
  }
  if (cmd == "CAL:WHITE") {
    if (!colorSem || xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      sendStatus("CAL:WHITE BUSY");
      Serial.println("[CAL] WHITE: Farb-Sperre nicht bekommen - nicht gespeichert");
      return true;
    }
    config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
    xSemaphoreGive(colorSem);
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
if (cmd == "CAL:COLOR" || cmd == "CAL:FARBE") {
    char nb[16] = {};
    int sr = 0, sg = 0, sb = 0;
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      sr = sensorR; sg = sensorG; sb = sensorB;
      strlcpy(nb, (const char*)sensorName, sizeof(nb));
      xSemaphoreGive(colorSem);
    }
    Serial.printf("[CAL] erkannt: %-10s rgb(%d,%d,%d)\n", nb, sr, sg, sb);
    char cm[48];
    snprintf(cm, sizeof(cm), "CAL:COLOR=%d,%d,%d,%s", sr, sg, sb, nb);
    sendStatus(cm);
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

static bool cmdTmc(const String& cmd) {
  char buf[48];
  if (cmd == "TMC?") { tmcPrintStatus(); sendStatus("TMC:LOGGED"); tmcSendCompactStatus(); return true; }
    if (cmd == "UARTTEST") { sendStatus("UARTTEST:GESPERRT"); return true; }
  if (cmd == "UARTNET") { tmcNetDiag(); sendStatus("NETZ:DIAG_DONE"); return true; }
    if (cmd == "MOT:SYNC") {
      if (motorSyncMicrosteps()) sendStatus("MOT:SYNC:OK");
      else                        sendStatus("MOT:SYNC:SKIP");
      return true;
    }
    if (cmd == "MOT:ALIGN") {
      if (motorAlignRequest()) sendStatus("MOT:ALIGN:OK");
      else                       sendStatus("MOT:ALIGN:BUSY");
      return true;
    }
    if (cmd == "MOT:SPD?") {
      if (!motorDiagSteadySpeed()) sendStatus("MOT:SPD:NO_TMC");
      return true;
    }
    if (cmd == "MOT:DIAG") {
      if (!motorDiagRequest()) {
        sendStatus(motorDiagRunning() ? "MOT:DIAG:BUSY" : "MOT:DIAG:NO_TMC");
        Serial.printf("[CMD] MOT:DIAG abgelehnt: %s\n",
                      motorDiagRunning() ? "Messlauf laeuft bereits" : "Treiber aus (kein EN)");
      }
      return true;
    }
  if (cmd.startsWith("TMC:CURRENT:") || cmd.startsWith("TMC:HOLD:")) {
    snprintf(buf, sizeof(buf), "TMC:FEST=%u", tmcGetCurrent()); sendStatus(buf);
    Serial.printf("[CMD] %s abgelehnt - der Strom ist fest %u mA\n",
                  cmd.c_str(), (unsigned)tmcGetCurrent());
    return true;
  }
  if (cmd.startsWith("TMC:MS:")) {
    sendStatus("TMC:FIX-TEST-64");
    Serial.println("[FIX-TEST] TMC:MS blockiert - fest 64 Microsteps");
    return true;
  }
  if (cmd == "TMC:MODE:SPREAD" || cmd == "TMC:MODE:STEALTH") {
    sendStatus("TMC:FIX-TEST-SPREAD");
    Serial.println("[FIX-TEST] TMC:MODE blockiert - fest SpreadCycle");
    return true;
  }
  if (cmd == "TMC:EN:1") { tmcEnable(); sendStatus("TMC:EN=1"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:EN:0") { tmcDisable(); sendStatus("TMC:EN=0"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:INIT") { tmcReinit(); sendStatus("TMC:INIT_DONE"); tmcSendCompactStatus(); return true; }
    if (cmd == "TMC:REG") { tmcRegDump(); sendStatus("TMC:REG_DONE"); return true; }
  return false;
}

static bool cmdSteps(const String& cmd) {
  char buf[64];
  if (cmd == "STEPS?") {
    snprintf(buf, sizeof(buf), "STEPS=%.1f,%ld", (double)config.quizForwardMM,
             (long)configForwardSteps());
    sendStatus(buf);
    return true;
  }
  if (cmd == "GEO?") {
    snprintf(buf, sizeof(buf), "GEO:%u,%.2f,%.1f,%.1f",
             (unsigned)(MOTOR_FULL_STEPS_PER_REV * config.tmcMicrosteps),
             (double)configStepsPerMM(), (double)config.quizForwardMM,
             (double)config.quizTurnDeg);
    sendStatus(buf);
    return true;
  }
  if (cmd == "GEO:RESET") {
    configResetGeometry();
    snprintf(buf, sizeof(buf), "GEO:RESET=%.1f,%.1f",
             (double)config.quizForwardMM, (double)config.quizTurnDeg);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("STEPS:")) {
    float mm = cmd.substring(6).toFloat();
    if (!inRange(mm, FORWARD_MM_MIN, FORWARD_MM_MAX)) { sendStatus("STEPS:BAD"); return true; }
    config.quizForwardMM = mm;
    configSaveField("quizMM", mm);
    testDriveSteps(configForwardSteps());
    snprintf(buf, sizeof(buf), "STEPS=%.1f,%ld", (double)mm, (long)configForwardSteps());
    sendStatus(buf);
    Serial.printf("[STEPS] %.1f mm = %ld Schritte (bei %u Microsteps) gefahren und gespeichert\n",
                  (double)mm, (long)configForwardSteps(), config.tmcMicrosteps);
    return true;
  }
  return false;
}

static bool cmdAng(const String& cmd) {
  char buf[64];
  if (cmd == "ANG?") {
    snprintf(buf, sizeof(buf), "ANG=%.1f,%ld", (double)config.quizTurnDeg,
             (long)configTurnSteps());
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("ANG:")) {
    float deg = cmd.substring(4).toFloat();
    if (!inRange(deg, TURN_DEG_MIN, TURN_DEG_MAX)) { sendStatus("ANG:BAD"); return true; }
    config.quizTurnDeg = deg;
    configSaveField("quizDeg", deg);
    testTurnSteps(configTurnSteps());
    snprintf(buf, sizeof(buf), "ANG=%.1f,%ld", (double)deg, (long)configTurnSteps());
    sendStatus(buf);
    Serial.printf("[ANG] %.1f Grad = %ld Schritte/Rad gedreht und gespeichert\n",
                  (double)deg, (long)configTurnSteps());
    return true;
  }
  return false;
}

static bool cmdSpeed(const String& cmd) {
  char buf[64];
  if (cmd == "SPEED?") {
    snprintf(buf, sizeof(buf), "SPEED=%.0f,%.0f", (double)config.driveSpeedMM,
             (double)FIXED_DRIVE_SPEED);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("SPEED:")) {
    sendStatus("SPEED:FIX-TEST-150");
    Serial.println("[FIX-TEST] SPEED blockiert - fest 150 mm/s");
    return true;
  }
  return false;
}

static bool cmdMotorFeel(const String& cmd) {
  char buf[64];
  if (cmd == "RAMP?") {
    snprintf(buf, sizeof(buf), "RAMP=%u,%u", (unsigned)config.rampMs,
             motorRampActive() ? 1u : 0u);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("RAMP:")) {
    long v = cmd.substring(5).toInt();
    if (v < RAMP_MS_MIN || v > RAMP_MS_MAX) { sendStatus("RAMP:BAD"); return true; }
    config.rampMs = (uint16_t)v;
    configSaveUShort("rampMs", config.rampMs);
    motorUpdateSpeeds();
    snprintf(buf, sizeof(buf), "RAMP=%u,%u", (unsigned)config.rampMs,
             motorRampActive() ? 1u : 0u);
    sendStatus(buf);
    Serial.printf("[RAMP] %u ms (0 = keine Rampe)\n", (unsigned)config.rampMs);
    return true;
  }
  if (cmd == "SPIN?") {
    snprintf(buf, sizeof(buf), "SPIN=%u,%u", (unsigned)config.spinSpeed,
             (unsigned)config.spinMs);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("SPIN:")) {
    const char* arg = cmd.c_str() + 5;
    int s1 = 0, ms = 0;
    const int n = sscanf(arg, "%d,%d", &s1, &ms);
    if (n == 1) {
      ms = s1; s1 = (int)config.spinSpeed;
    } else if (n != 2) {
      sendStatus("SPIN:BAD"); return true;
    }
    if (s1 < SPIN_SPEED_MIN || s1 > SPIN_SPEED_MAX ||
        ms < SPIN_MS_MIN   || ms > SPIN_MS_MAX)               { sendStatus("SPIN:BAD"); return true; }
    config.spinSpeed = (uint16_t)s1;
    config.spinMs    = (uint16_t)ms;
    configSaveUShort("spinSpeed", config.spinSpeed);
    configSaveUShort("spinMs", config.spinMs);
    snprintf(buf, sizeof(buf), "SPIN=%u,%u", (unsigned)config.spinSpeed,
             (unsigned)config.spinMs);
    sendStatus(buf);
    Serial.printf("[SPIN] Einschalt-Drehung: %u Schritte/s fuer %u ms%s\n",
                  (unsigned)config.spinSpeed, (unsigned)config.spinMs,
                  (config.spinSpeed == 0 || config.spinMs == 0) ? " (aus)" : "");
    return true;
  }
  return false;
}

static bool cmdSat(const String& cmd) {
  char msg[32];
  if (cmd == "SAT?") {
    snprintf(msg, sizeof(msg), "SAT=%.2f", config.minColorSaturation);
    sendStatus(msg);
    return true;
  }
  if (cmd.startsWith("SAT:")) {
    float v = cmd.substring(4).toFloat();
    if (!inRange(v, 0.0f, 1.0f)) { sendStatus("SAT:BAD"); return true; }
    config.minColorSaturation = v;
    configSaveField("minSat", v);
    snprintf(msg, sizeof(msg), "SAT=%.2f", v);
    sendStatus(msg);
    return true;
  }
  return false;
}


static bool cmdQuiz(const String& cmd) {
  if (!cmd.startsWith("QUIZ:")) return false;
  String sub = cmd.substring(5);
  if (sub.startsWith("CHECK:")) {
    if (korrigierenRunning) {
      sendStatus("QUIZ:BUSY");
      Serial.println("[QUIZ] Laufend - CHECK abgelehnt");
      return true;
    }
    if (claimGameSlot("QUIZ:BUSY", "QUIZ")) return true;
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
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

static bool cmdRace(const String& cmd) {
  if (!cmd.startsWith("RACE:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub == "START") {
    raceTargetScore = RENNEN_TARGET_SCORE;
    raceStage = 0;
    raceMMPerPoint = 0.0f;
  } else if (sub.startsWith("START:")) {
    const String params = sub.substring(6);
    const int b = params.indexOf(':');
    long target = params.toInt();
    long stage = (b >= 0) ? params.substring(b + 1).toInt() : 0;
    if (target < RACE_TARGET_MIN || target > RACE_TARGET_MAX) target = RENNEN_TARGET_SCORE;
    if (stage < 0 || stage > 6) stage = 0;
    raceTargetScore = (uint8_t)target;
    raceStage = (uint8_t)stage;
    raceMMPerPoint = 0.0f;
  }
  if (sub == "START" || sub.startsWith("START:")) {
    if (claimGameSlot("RACE:BUSY", "RACE")) return true;
    raceExecuteStart();
    raceSendPacket(0, false);
    snprintf(buf, sizeof(buf), "RACE:TARGET:%u,%u", raceTargetScore, raceStage);
    sendStatus(buf);
  } else if (sub == "STEP") {
    if (!raceActive) return true;
    raceMyScore++;
    raceForwardStep();
    if (!raceActive) return true;
    raceSendPacket((uint8_t)raceMyScore, raceMyScore >= raceTargetScore);
    snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
    sendStatus(buf);
    if (raceMyScore >= raceTargetScore) {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      sendStatus("RACE:YOU_WIN");
    }
  } else if (sub.startsWith("STEP:")) {
    const float mm = sub.substring(5).toFloat();
    if (!inRange(mm, 1.0f, 300.0f)) { sendStatus("RACE:BAD_MM"); return true; }
    raceMMPerPoint = mm;
    if (!raceActive) return true;
    raceMyScore++;
    raceForwardStepMM(mm);
    if (!raceActive) return true;
    raceSendPacket((uint8_t)raceMyScore, raceMyScore >= raceTargetScore);
    snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
    sendStatus(buf);
    if (raceMyScore >= raceTargetScore) {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      sendStatus("RACE:YOU_WIN");
    }
  } else if (sub == "WIN") {
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    raceSendPacket((uint8_t)raceTargetScore, true);
  } else if (sub == "LOSE") {
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    raceSendPacket((uint8_t)raceMyScore, false);
  } else if (sub == "STOP") {
    raceAbort(true);
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
    raceLastOppMs = millis();
    snprintf(buf, sizeof(buf), "RACE:PROGRESS:%d", raceOppScore);
    sendStatus(buf);
  } else if (sub == "?") {
    snprintf(buf, sizeof(buf), "RACE:TARGET:%u,%u", raceTargetScore, raceStage);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "RACE:STATE:%d,%d,%u,%u", raceActive ? 1 : 0,
             currentGame == GAME_RENNEN ? 1 : 0, (unsigned)raceMyScore,
             (unsigned)raceOppScore);
    sendStatus(buf);
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

static bool claimGameSlot(const char* busyMsg, const char* what) {
  if (!driveLockedByGame()) return false;
  sendStatus(busyMsg);
  Serial.printf("[%s] Spiel laeuft (game=%d) - Start abgelehnt\n",
                what, (int)currentGame);
  return true;
}

static bool cmdLine(const String& cmd) {
  if (!cmd.startsWith("LINE:")) return false;
  String sub = cmd.substring(5);
  if (sub == "START") {
    if (claimGameSlot("LINE:BUSY", "LINE")) return true;
    if (!lineFollowerActive) {
      currentGame = GAME_LINEFOLLOWER;
      lineFollowerStart();
      sendStatus("LINE:RUNNING");
    } else {
      sendStatus("LINE:BUSY");
    }
  } else if (sub == "STOP") {
    lineFollowerStop();
    currentGame = GAME_NONE;
    sendStatus("LINE:STOPPED");
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

static bool cmdMemory(const String& cmd) {
  if (!cmd.startsWith("MEMORY:")) return false;
  String sub = cmd.substring(7);
  if (sub == "START") {
    if (claimGameSlot("MEMORY:BUSY", "MEMORY")) return true;
    currentGame = GAME_MEMORY;
    memoryStart(3);
  } else if (sub.startsWith("START:")) {
    if (claimGameSlot("MEMORY:BUSY", "MEMORY")) return true;
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
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

static bool cmdProg(const String& cmd) {
  if (!cmd.startsWith("PROG:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub.startsWith("SET:")) {
    if (progActive) {
      sendStatus("PROG:BUSY");
      Serial.println("[PROG] Laufend - SET abgelehnt");
      return true;
    }
    String enc = sub.substring(4);
    if (progSetFromString(enc.c_str())) {
      snprintf(buf, sizeof(buf), "PROG:READY:%u", progLen);
      sendStatus(buf);
    } else {
      sendStatus("PROG:BAD");
    }
  } else if (sub == "RUN") {
    if (slalomPhase == SLALOM_BUILD || slalomPhase == SLALOM_COUNTDOWN ||
        slalomPhase == SLALOM_RACING) {
      sendStatus("PROG:BUSY:SLALOM");
      Serial.println("[PROG] Slalom laeuft - Solo-Start abgelehnt");
      return true;
    }
    if (claimGameSlot("PROG:BUSY", "PROG")) return true;
    progRun();
  } else if (sub == "STOP") {
    progStop();
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

static bool cmdSlalom(const String& cmd) {
  if (!cmd.startsWith("SLALOM:")) return false;
  String sub = cmd.substring(7);
  if (sub.startsWith("PROG:")) {
    String enc = sub.substring(5);
    slalomSetProgram(enc.c_str());
  } else if (sub.startsWith("INVITE:")) {
    if (slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:ALREADY_PAIRED");
      return true;
    }
    uint8_t mac[6];
    if (!parseMac(sub.substring(7), mac)) {
      sendStatus("SLALOM:ERROR:BAD_MAC");
      return true;
    }
    slalomInviteSend(mac);
    sendStatus("SLALOM:INVITE_SENT");
    Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else if (sub == "INVITE_ACCEPT") {
    if (!slalomInvitePending()) {
      sendStatus("SLALOM:ERROR:NO_INVITE");
      return true;
    }
    slalomSendPacketTo(SLALOM_CMD_INVITE_ACK, nullptr, slalomInviterMacPtr(), true);
    slalomPairWith(slalomInviterMacPtr(), false);
    slalomClearInvite();
    sendStatus("SLALOM:INVITE_ACCEPTED");
  } else if (sub == "INVITE_DECLINE") {
    if (slalomInvitePending()) {
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomInviterMacPtr(), true);
    }
    slalomClearInvite();
  } else if (sub == "UNPAIR") {
    if (slalomHasPeer()) {
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomPeerMacPtr(), true);
    }
    slalomUnpair();
  } else if (sub == "START_MATCH") {
    if (!slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:NOT_PAIRED");
      return true;
    }
    if (!slalomIsHost) {
      sendStatus("SLALOM:ERROR:NOT_HOST");
      return true;
    }
    if (claimGameSlot("SLALOM:BUSY", "SLALOM")) return true;
    slalomStartGame();
  } else if (sub == "STOP") {
    slalomAbortGame();
  } else {
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

void handleCommand(const char* cmdStr);
void handleBleCommand(const char* cmdStr) {
  s_cmdFromBle = true;
  handleCommand(cmdStr);
  s_cmdFromBle = false;
}
void handleCommand(const char* cmdStr) {
  String cmd(cmdStr);
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

    if (cmd.startsWith("KEYS:")) { cmdKeys(cmd); return; }
    if (cmd == "DRV?") {
      char drv[224];
      driveDebug(drv, sizeof(drv));
      sendStatus(drv);
      return;
    }

  Serial.printf("[CMD] %s\n", cmd.c_str());
  if (cmd == "STOP") { hardStop(); return; }
  if (cmd == "HALT") { cmdHalt(); return; }

  if (cmdIsGameStop(cmd)) {
    driveClearKeys();
    speedL = 0.0; speedR = 0.0;
  }


  if (cmdSteps(cmd)) return;
  if (cmdAng(cmd))   return;
  if (cmdSpeed(cmd)) return;
  if (cmdMotorFeel(cmd)) return;
  if (cmdSat(cmd))   return;


  if (cmdDist(cmd))  return;
  if (cmdWebUi(cmd)) return;
  if (cmdCal(cmd))   return;
  if (cmdTmc(cmd))   return;
  if (cmdQuiz(cmd))  return;
  if (cmdRace(cmd))  return;
  if (cmdSlalom(cmd))return;
  if (lobbyCmd(cmd))  return;
  if (cmdLine(cmd))  return;
  if (cmdMemory(cmd))return;
  if (cmdProg(cmd))  return;
  if (ampelCmd(cmd)) return;
  if (zahlCmd(cmd))  return;
  if (geheimCmd(cmd)) return;
  if (parkCmd(cmd))  return;
  if (zeitCmd(cmd))  return;
  if (duelCmd(cmd))  return;
  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V2 - Modular (2x TMC2209 + VL53L0X)");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
  esp_task_wdt_add(NULL);

configLoad();

edgeGuardSet(true);

initSteppers();
  Serial.println("[MOTOR] Motor-Task auf Core 0 gestartet (feste Geschwindigkeit, sanftes Anfahren)");
  startupSpin();
  tmcBootDiag();

  lastCommandTime = millis();

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
  webUiSetup();
  if (webUiAutoEnabled()) {
    Serial.println("[WEB] Autostart aktiv - Access Point wird beim Boot gestartet");
    webUiStart();
  }
  espNowEnsureChannel();
  ledStripSetup();
  Serial.println("======================================\n");
}

static void ledPatternUpdate() {
  if (edgeGuardBlocked()) {
    ledStripShowBlink(LED_C_RED, 500);
    return;
  }
  if (tmcBatteryLow()) {
    ledStripShowBlink(LED_C_RED, 500);
    return;
  }
  if (apSignalActive()) {
    switch (apSignalStep()) {
      case APSIG_WANT_BLUE:  ledStripShowBreathe(LED_C_BLUE,  900); return;
      case APSIG_WANT_GREEN: ledStripShowBreathe(LED_C_GREEN, 900); return;
      case APSIG_ANNOUNCED:  ledStripShowBlink(LED_C_GREEN, 260);  return;
      default: break;
    }
  }
  if (slalomPhase == SLALOM_COUNTDOWN) {
    const uint32_t now = millis();
    const uint32_t left = (slalomGoTimer > now) ? (slalomGoTimer - now) : 0;
    ledStripShowCountdown(left, slalomIsHost ? 3000 : 4500);
    return;
  }
  if (slalomPhase == SLALOM_DONE) {
    ledStripShowResult(slalomIWonLast);
    return;
  }
  if (slalomTargetColor[0] != '\0') {
    ledStripShowTarget(slalomTargetColor);
    return;
  }
  if (slalomPhase == SLALOM_PAIRED) {
    ledStripShowPaired();
    return;
  }
  if (memoryActive) {
    if (memoryAwaitInput) ledStripShowProgress(memoryInputIdx, memoryLen, LED_C_GREEN);
    else                  ledStripShowMemoryShow();
    return;
  }
  if (ampelActive()) {
    ledStripShowBreathe(ampelRed() ? LED_C_RED : (ampelCheat() ? LED_C_YELLOW : LED_C_GREEN), 1600);
    return;
  }
  if (zahlActive()) {
    ledStripShowBreathe(LED_C_BLUE, 1600);
    return;
  }
  if (geheimActive()) {
    ledStripShowBreathe(LED_C_CYAN, 1600);
    return;
  }
  if (parkActive()) {
    ledStripShowBreathe(LED_C_ORANGE, 1600);
    return;
  }
  if (zeitActive()) {
    ledStripShowBreathe(LED_C_WHITE, 1600);
    return;
  }
  if (currentGame == GAME_LINEFOLLOWER) {
    ledStripShowLine(lineFollowerLedState());
    return;
  }
  if (currentGame == GAME_RENNEN) {
    ledStripShowRace();
    return;
  }
  if (speedL != 0.0f || speedR != 0.0f) {
    LedDriveDir dir = LED_DIR_IDLE;
    if (keyW && !keyS)      dir = LED_DIR_FWD;
    else if (keyS && !keyW) dir = LED_DIR_BACK;
    ledStripShowDrive(dir);
    return;
  }
  ledStripShowStatus(anyClientConnected());
}

unsigned long lastColorNotify = 0;
void loop() {
  esp_task_wdt_reset();
  bleProcessEvents();
  webUiFlush();
  apSignalTick();
  espNowProcessEvents();
  memoryTick();
  ampelTick();
  zahlTick();
  geheimTick();
  parkTick();
  zeitTick();
  duelTick();
  if (currentGame == GAME_LINEFOLLOWER) {
    bool wasActive = lineFollowerActive;
    lineFollowerTick();
    if (wasActive && !lineFollowerActive) {
      currentGame = GAME_NONE;
      sendStatus("LINE:LOST");
    }
  }

  if (slalomGoPending && !slalomRunning && slalomPhase == SLALOM_COUNTDOWN &&
      millis() >= slalomGoTimer) {
    slalomGoPending = false;
    if (slalomIsHost) {
      slalomSendPacket(SLALOM_CMD_GO, nullptr, true);
      Serial.println("[SLALOM] HOST-GO-Timer: GO gesendet (reliable), starte selbst");
    } else {
      Serial.println("[SLALOM] Client-Fallback-GO: kein Paket erhalten, starte trotzdem");
    }
    slalomExecute();
  }
  slalomCheckRetry();
  slalomTick();
  lobbyTick();
  if (!driveLockedByGame()) driveUpdate();
  if (anyClientConnected() && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  distTick();
  distRefSpadService();
  tmcAutoStandbyTick();
  motorDiagTick();
  tmcBatteryGuardTick();
  if (racePeerTimedOut()) {
    Serial.println("[RACE] Gegner timeout -> Rennen beendet");
    raceAbort(false);
  }
  {
    static unsigned long lastLedMode = 0;
    if (millis() - lastLedMode >= 50) {
      lastLedMode = millis();
      ledPatternUpdate();
    }
    ledStripTick();
  }
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
