#include "ParkGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"
#include "DistanceSensor.h"

constexpr float   PARK_SPEED_MM_S = 60.0f;
constexpr int16_t PARK_TOUCH_MM   = 45;
constexpr int32_t PARK_EMPTY_MM   = 1500;
constexpr uint8_t PARK_STAGE_EVERY = 2;
constexpr uint8_t PARK_STAGE_MAX   = 3;

static bool    s_active = false;
static bool    s_driving = false;
static uint8_t s_success = 0;

bool parkActive() { return s_active && s_driving; }
static uint8_t parkStage() {
  uint8_t st = 1 + s_success / PARK_STAGE_EVERY;
  return (st > PARK_STAGE_MAX) ? PARK_STAGE_MAX : st;
}
static int16_t parkTolPerfect(uint8_t st) { return (st <= 1) ? 100 : (st == 2 ? 80 : 60); }
static int16_t parkTolGood(uint8_t st)    { return (st <= 1) ? 200 : (st == 2 ? 150 : 120); }

void parkStop() {
  if (!s_active) return;
  s_active  = false;
  s_driving = false;
  s_success = 0;
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_PARK) {
    gameModeEnd("PARK:STOPPED");
  } else {
    gameHaltClear();
  }
}

static void parkFinish(const char* token) {
  s_driving = false;
  stopMotors();
  gameSpeedRestore();
  if (token != nullptr) sendStatus(token);
}

void parkTick() {
  if (!s_active) return;
  if (currentGame != GAME_PARK) {
    s_active  = false;
    s_driving = false;
    return;
  }
  if (!s_driving) return;

  const int16_t mm = distLastMM();
  if (mm >= 0 && mm < PARK_TOUCH_MM) {
    duelAddPoints(0);
    parkFinish("PARK:TOUCH");
    ledStripShowResultFlash(false);
    ledStripTick();
    return;
  }
  if (probeMM() > PARK_EMPTY_MM) {
    duelAddPoints(0);
    parkFinish("PARK:EMPTY");
    return;
  }

  if (gameHaltRequested()) {
    gameHaltClear();
    if (mm < 0) {
      duelAddPoints(0);
      parkFinish("PARK:STOPPED");
      return;
    }
    const uint8_t st = parkStage();
    uint8_t score = 0;
    if (mm < parkTolPerfect(st))      score = 2;
    else if (mm < parkTolGood(st))    score = 1;
    if (score > 0) s_success++;
    duelAddPoints(score);
    gameSpeedRestore();
    char msg[48];
    snprintf(msg, sizeof(msg), "PARK:RESULT:%d,%u", (int)mm, (unsigned)score);
    parkFinish(msg);
    ledStripShowResultFlash(score > 0);
    ledStripTick();
  }
}

bool parkCmd(const String& cmd) {
  if (!cmd.startsWith("PARK:")) return false;
  const String sub = cmd.substring(5);

  if (sub == "START") {
    if (currentGame != GAME_NONE && currentGame != GAME_PARK) {
      sendStatus("PARK:BUSY");
      return true;
    }
    if (s_active && s_driving) {
      sendStatus("PARK:BUSY");
      return true;
    }
    s_active  = true;
    s_driving = true;
    currentGame = GAME_PARK;
    gameHaltClear();
    probeBegin();
    gameSpeedOverride(PARK_SPEED_MM_S);
    setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
    sendStatus("PARK:GO");
    sendStatusF("PARK:STAGE:%u", (unsigned)parkStage());
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      parkStop();
    } else {
      sendStatus("PARK:STOPPED");
    }
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
