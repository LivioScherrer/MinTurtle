#include "ZeitGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"
#include "ColorSensor.h"

constexpr float   ZEIT_SPEED_MM_S = 120.0f;
constexpr uint32_t ZEIT_TIMEOUT_MS = 20000;
constexpr uint32_t ZEIT_POLL_MS    = 20;
constexpr uint8_t  ZEIT_HITS_NEED  = 2;

static const char* const ZEIT_COLORS[4] = { "ROT", "GELB", "GRUEN", "BLAU" };

static bool     s_active = false;
static bool     s_driving = false;
static uint8_t  s_target = 0;
static uint32_t s_startMs = 0;
static uint32_t s_lastPoll = 0;
static uint8_t  s_hits = 0;
static uint32_t s_bestMs = 0;

bool zeitActive() { return s_active && s_driving; }

void zeitStop() {
  if (!s_active) return;
  s_active  = false;
  s_driving = false;
  s_bestMs  = 0;
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_ZEIT) {
    gameModeEnd("ZEIT:STOPPED");
  } else {
    gameHaltClear();
  }
}

void zeitTick() {
  if (!s_active) return;
  if (currentGame != GAME_ZEIT) {
    s_active  = false;
    s_driving = false;
    return;
  }
  if (!s_driving) return;

  if (gameHaltRequested()) {
    gameHaltClear();
    duelAddPoints(0);
    zeitStop();
    return;
  }

  const uint32_t now = millis();
  if (now - s_startMs > ZEIT_TIMEOUT_MS) {
    s_driving = false;
    stopMotors();
    gameSpeedRestore();
    duelAddPoints(0);
    sendStatus("ZEIT:NOTFOUND");
    ledStripShowResultFlash(false);
    ledStripTick();
    return;
  }
  if (now - s_lastPoll < ZEIT_POLL_MS) return;
  s_lastPoll = now;
  char name[16] = "--";
  if (colorSensorOk) getCurrentColorName(name, sizeof(name));
  if (strcasecmp(name, ZEIT_COLORS[s_target]) == 0) s_hits++;
  else                                               s_hits = 0;
  if (s_hits < ZEIT_HITS_NEED) return;

  s_driving = false;
  const uint32_t ms = now - s_startMs;
  const bool isBest = (s_bestMs == 0 || ms < s_bestMs);
  const uint8_t score = isBest ? 2 : 1;
  if (isBest) s_bestMs = ms;
  duelAddPoints(score);
  gameSpeedRestore();
  char msg[48];
  snprintf(msg, sizeof(msg), "ZEIT:RESULT:%lu,%u", (unsigned long)ms, (unsigned)score);
  stopMotors();
  sendStatus(msg);
  ledStripShowResultFlash(true);
  ledStripTick();
}

bool zeitCmd(const String& cmd) {
  if (!cmd.startsWith("ZEIT:")) return false;
  const String sub = cmd.substring(5);

  if (sub.startsWith("START:")) {
    if (currentGame != GAME_NONE && currentGame != GAME_ZEIT) {
      sendStatus("ZEIT:BUSY");
      return true;
    }
    if (s_active && s_driving) {
      sendStatus("ZEIT:BUSY");
      return true;
    }
    int v = sub.substring(6).toInt();
    if (v < 0) v = 0;
    if (v > 3) v = 3;
    s_target = (uint8_t)v;
    s_active  = true;
    s_driving = true;
    s_hits = 0;
    currentGame = GAME_ZEIT;
    gameHaltClear();
    gameSpeedOverride(ZEIT_SPEED_MM_S);
    setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
    s_startMs = millis();
    s_lastPoll = 0;
    sendStatus("ZEIT:GO");
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      zeitStop();
    } else {
      sendStatus("ZEIT:STOPPED");
    }
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
