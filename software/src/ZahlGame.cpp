#include "ZahlGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

constexpr float   ZAHL_SPEED_MM_S = 100.0f;
constexpr float   ZAHL_SPEED_PER_STAGE = 20.0f;
constexpr uint8_t ZAHL_STAGE_EVERY = 3;
constexpr uint8_t ZAHL_STAGE_MAX   = 5;
constexpr int32_t ZAHL_OVERRUN_MM = 500;

static bool    s_active = false;
static bool    s_driving = false;
static int16_t s_targetMM = 0;
static uint8_t s_success = 0;

bool zahlActive() { return s_active && s_driving; }
static uint8_t zahlStage() {
  uint8_t st = 1 + s_success / ZAHL_STAGE_EVERY;
  return (st > ZAHL_STAGE_MAX) ? ZAHL_STAGE_MAX : st;
}
static float zahlSpeed() {
  return ZAHL_SPEED_MM_S + (float)(zahlStage() - 1) * ZAHL_SPEED_PER_STAGE;
}
static uint8_t zahlScoreForMM(int32_t abw) {
  int score = 100 - (int)((abw * 5 + 5) / 10);
  if (score < 0) score = 0;
  if (score > 100) score = 100;
  return (uint8_t)score;
}

void zahlStop() {
  if (!s_active) return;
  s_active  = false;
  s_driving = false;
  s_success = 0;
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_ZAHL) {
    gameModeEnd("ZAHL:STOPPED");
  } else {
    gameHaltClear();
  }
}

void zahlTick() {
  if (!s_active) return;
  if (currentGame != GAME_ZAHL) {
    s_active  = false;
    s_driving = false;
    return;
  }
  if (!s_driving) return;

  if (probeMM() > (int32_t)s_targetMM + ZAHL_OVERRUN_MM) {
    s_driving = false;
    stopMotors();
    gameSpeedRestore();
    gameHaltClear();
    duelAddPoints(0);
    sendStatus("ZAHL:OVERRUN");
    return;
  }

  if (gameHaltRequested()) {
    gameHaltClear();
    s_driving = false;
    const int32_t driven = probeMM();
    const int32_t delta  = driven - s_targetMM;
    const int32_t abw    = (delta < 0) ? -delta : delta;
    const uint8_t score = zahlScoreForMM(abw);
    if (score >= 80) s_success++;
    duelAddPoints(score >= 80 ? 2 : (score >= 40 ? 1 : 0));
    gameSpeedRestore();
    char msg[48];
    snprintf(msg, sizeof(msg), "ZAHL:RESULT:%ld,%u", (long)delta, (unsigned)score);
    sendStatus(msg);
    ledStripShowResultFlash(score >= 80);
    ledStripTick();
  }
}

bool zahlCmd(const String& cmd) {
  if (!cmd.startsWith("ZAHL:")) return false;
  const String sub = cmd.substring(5);

  if (sub.startsWith("START:")) {
    if (currentGame != GAME_NONE && currentGame != GAME_ZAHL) {
      sendStatus("ZAHL:BUSY");
      return true;
    }
    if (s_active && s_driving) {
      sendStatus("ZAHL:BUSY");
      return true;
    }
    const String arg = sub.substring(6);
    int v = 0;
    if (arg.startsWith("RANDOM:")) {
      int maxCm = arg.substring(7).toInt();
      if (maxCm != 20) maxCm = 40;
      v = random(5, maxCm + 1);
    } else {
      v = arg.toInt();
      if (v < 5)   v = 5;
      if (v > 100) v = 100;
    }
    s_targetMM = (int16_t)(v * 10);
    s_active  = true;
    s_driving = true;
    currentGame = GAME_ZAHL;
    gameHaltClear();
    probeBegin();
    gameSpeedOverride(zahlSpeed());
    setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
    char msg[32];
    snprintf(msg, sizeof(msg), "ZAHL:GO:%d", v);
    sendStatus(msg);
    sendStatusF("ZAHL:STAGE:%u", (unsigned)zahlStage());
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      zahlStop();
    } else {
      sendStatus("ZAHL:STOPPED");
    }
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
