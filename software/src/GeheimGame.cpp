#include "GeheimGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Config.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

static int16_t geheimRangeMin(uint8_t st) { return (st <= 1) ? 200 : 100; }
static int16_t geheimRangeMax(uint8_t st) { return (st <= 1) ? 500 : (st == 2 ? 800 : 1000); }
static int32_t geheimTolPerfect(uint8_t st) { return (st <= 1) ? 20 : (st == 2 ? 15 : 10); }
static int32_t geheimTolGood(uint8_t st)    { return (st <= 1) ? 50 : (st == 2 ? 40 : 30); }
constexpr uint8_t GEHEIM_STAGE_MAX = 3;

static bool    s_active = false;
static bool    s_driving = false;
static bool    s_haveSecret = false;
static int16_t s_secretMM = 0;
static int16_t s_guessMM = 0;
static uint8_t s_wins = 0;

bool geheimActive() { return s_active && (s_driving || s_haveSecret); }
static uint8_t geheimStage() {
  uint8_t st = 1 + s_wins;
  return (st > GEHEIM_STAGE_MAX) ? GEHEIM_STAGE_MAX : st;
}

static void geheimNewSecret() {
  const uint8_t st = geheimStage();
  s_secretMM = (int16_t)random(geheimRangeMin(st), geheimRangeMax(st) + 1);
  s_haveSecret = true;
  sendStatusF("GEHEIM:STAGE:%u", (unsigned)st);
  sendStatus("GEHEIM:READY");
}

void geheimStop() {
  if (!s_active) return;
  s_active = false;
  s_driving = false;
  s_haveSecret = false;
  s_wins = 0;
  stopMotors();
  if (currentGame == GAME_GEHEIM) {
    gameModeEnd("GEHEIM:STOPPED");
  } else {
    gameHaltClear();
  }
}

void geheimTick() {
  if (!s_active || !s_driving) return;
  if (currentGame != GAME_GEHEIM) {
    s_active = false;
    s_driving = false;
    return;
  }
  if (motorStillRunning()) return;
  s_driving = false;
  const int32_t diff = (int32_t)s_secretMM - (int32_t)s_guessMM;
  const int32_t abw = (diff < 0) ? -diff : diff;
  const uint8_t st = geheimStage();
  uint8_t score = 0;
  if (abw <= geheimTolPerfect(st))      score = 2;
  else if (abw <= geheimTolGood(st))    score = 1;
  if (score == 2) s_wins++;
  duelAddPoints(score);
  char msg[48];
  snprintf(msg, sizeof(msg), "GEHEIM:RESULT:%ld,%u", (long)diff, (unsigned)score);
  sendStatus(msg);
  ledStripShowResultFlash(score > 0);
  ledStripTick();
}

bool geheimCmd(const String& cmd) {
  if (!cmd.startsWith("GEHEIM:")) return false;
  const String sub = cmd.substring(7);

  if (sub == "START") {
    if (currentGame != GAME_NONE && currentGame != GAME_GEHEIM) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    if (s_driving) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    s_active = true;
    currentGame = GAME_GEHEIM;
    gameHaltClear();
    geheimNewSecret();
    return true;
  }

  if (sub.startsWith("START:")) {
    if (currentGame != GAME_NONE && currentGame != GAME_GEHEIM) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    if (s_active && s_driving) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    int v = sub.substring(6).toInt();
    if (v < 10)  v = 10;
    if (v > 100) v = 100;
    s_guessMM = (int16_t)(v * 10);
    if (!s_haveSecret) {
      s_active = true;
      currentGame = GAME_GEHEIM;
      geheimNewSecret();
    }
    s_active = true;
    currentGame = GAME_GEHEIM;
    gameHaltClear();
    const int32_t steps = geoStepsForMM((float)s_secretMM, config.tmcMicrosteps);
    prepPositionMoves(steps * (long)config.leftSign,
                      steps * (long)config.rightSign);
    s_driving = true;
    sendStatus("GEHEIM:GO");
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      geheimStop();
    } else {
      sendStatus("GEHEIM:STOPPED");
    }
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
