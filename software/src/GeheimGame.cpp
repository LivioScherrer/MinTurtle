#include "GeheimGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Config.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

constexpr uint8_t GEHEIM_STAGE_MAX = 3;
constexpr int16_t GEHEIM_MIN_MM     = 50;

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

static void geheimNewSecret(uint8_t maxCm) {
  if (maxCm != 20 && maxCm != 100) maxCm = 40;
  s_secretMM = (int16_t)(random(GEHEIM_MIN_MM / 10, maxCm + 1) * 10);
  s_haveSecret = true;
  sendStatusF("GEHEIM:STAGE:%u", (unsigned)geheimStage());
  sendStatus("GEHEIM:READY");
}
static uint8_t geheimScoreForMM(int32_t abw) {
  int score = 100 - (int)((abw * 5 + 5) / 10);
  if (score < 0) score = 0;
  if (score > 100) score = 100;
  return (uint8_t)score;
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
  const uint8_t score = geheimScoreForMM(abw);
  if (score >= 80) s_wins++;
  duelAddPoints(score >= 80 ? 2 : (score >= 40 ? 1 : 0));
  char msg[48];
  snprintf(msg, sizeof(msg), "GEHEIM:RESULT:%ld,%u", (long)diff, (unsigned)score);
  sendStatus(msg);
  ledStripShowResultFlash(score >= 80);
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
    geheimNewSecret(40);
    return true;
  }

  if (sub.startsWith("START:RANDOM:")) {
    if (currentGame != GAME_NONE && currentGame != GAME_GEHEIM) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    if (s_driving) {
      sendStatus("GEHEIM:BUSY");
      return true;
    }
    const int maxCm = sub.substring(13).toInt();
    s_active = true;
    currentGame = GAME_GEHEIM;
    gameHaltClear();
    geheimNewSecret((uint8_t)maxCm);
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
    if (!s_haveSecret) {
      sendStatus("GEHEIM:ERROR:NO_SECRET");
      return true;
    }
    int v = sub.substring(6).toInt();
    if (v < 5)   v = 5;
    if (v > 100) v = 100;
    s_guessMM = (int16_t)(v * 10);
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
