#include "AmpelGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

constexpr float    AMPEL_ROLL_MM_S    = 60.0f;
constexpr uint32_t AMPEL_RED_MIN_MS   = 2000;
constexpr uint32_t AMPEL_RED_MAX_MS   = 6000;
constexpr uint32_t AMPEL_CHEAT_WINDOW = 3000;
constexpr uint32_t AMPEL_STAR_MS      = 600;
constexpr uint32_t AMPEL_GOOD_MS      = 1200;
constexpr int32_t  AMPEL_MISS_MM      = 1500;
constexpr uint32_t AMPEL_MISS_MS      = 20000;
constexpr uint8_t  AMPEL_CHEAT_DENOM  = 6;
constexpr uint8_t  AMPEL_STAGE_EVERY  = 3;
constexpr uint8_t  AMPEL_STAGE_MAX    = 5;
constexpr uint32_t AMPEL_RED_SHRINK   = 1000;

enum AmpelPhase : uint8_t {
  AMPEL_IDLE = 0,
  AMPEL_ARMING,
  AMPEL_RED,
  AMPEL_CHEAT,
  AMPEL_WAIT_NEXT
};

static bool     s_active    = false;
static uint8_t  s_phase     = AMPEL_IDLE;
static uint32_t s_phaseAt   = 0;
static uint32_t s_redAfter  = 0;
static bool     s_cheatRound = false;
static bool     s_cheatPressed = false;
static uint8_t  s_success   = 0;

bool ampelActive() { return s_active; }
bool ampelRed()    { return s_active && s_phase == AMPEL_RED; }
bool ampelCheat()  { return s_active && s_phase == AMPEL_CHEAT; }
uint8_t ampelStage() {
  uint8_t st = 1 + s_success / AMPEL_STAGE_EVERY;
  return (st > AMPEL_STAGE_MAX) ? AMPEL_STAGE_MAX : st;
}

static void ampelBeginRound() {
  s_cheatRound   = (random(0, AMPEL_CHEAT_DENOM) == 0);
  s_cheatPressed = false;
  s_phase        = AMPEL_ARMING;
  s_phaseAt      = millis();
  const uint32_t st = ampelStage();
  uint32_t redMax = (AMPEL_RED_MAX_MS > (uint32_t)(st - 1) * AMPEL_RED_SHRINK)
      ? AMPEL_RED_MAX_MS - (uint32_t)(st - 1) * AMPEL_RED_SHRINK
      : AMPEL_RED_MIN_MS;
  if (redMax < AMPEL_RED_MIN_MS) redMax = AMPEL_RED_MIN_MS;
  s_redAfter = (redMax <= AMPEL_RED_MIN_MS)
      ? AMPEL_RED_MIN_MS
      : random(AMPEL_RED_MIN_MS, redMax + 1);
  probeBegin();
  gameSpeedOverride(AMPEL_ROLL_MM_S);
  setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
  sendStatus("AMPEL:ARMED");
  sendStatusF("AMPEL:STAGE:%u", (unsigned)ampelStage());
}

static void ampelFinishRound(const char* token) {
  stopMotors();
  gameSpeedRestore();
  s_phase = AMPEL_IDLE;
  if (token != nullptr) sendStatus(token);
}

void ampelStop() {
  if (!s_active) return;
  s_active = false;
  s_phase  = AMPEL_IDLE;
  s_success = 0;
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_AMPEL) {
    gameModeEnd("AMPEL:STOPPED");
  } else {
    gameHaltClear();
  }
}

void ampelTick() {
  if (!s_active) return;
  if (currentGame != GAME_AMPEL) {
    s_active = false;
    s_phase  = AMPEL_IDLE;
    return;
  }

  const uint32_t now = millis();

  switch (s_phase) {
    case AMPEL_ARMING: {
      if (gameHaltRequested()) {
        gameHaltClear();
        ampelFinishRound("AMPEL:EARLY");
        return;
      }
      if (now - s_phaseAt < s_redAfter) return;
      if (s_cheatRound) {
        stopMotors();
        s_phase   = AMPEL_CHEAT;
        s_phaseAt = now;
        sendStatus("AMPEL:CHEAT");
      } else {
        probeBegin();
        s_phase   = AMPEL_RED;
        s_phaseAt = now;
        sendStatus("AMPEL:RED");
      }
      return;
    }

    case AMPEL_RED: {
      if (gameHaltRequested()) {
        gameHaltClear();
        const uint32_t ms = now - s_phaseAt;
        const int32_t mm = probeMM();
        uint8_t score = 0;
        if (ms < AMPEL_STAR_MS)      score = 2;
        else if (ms < AMPEL_GOOD_MS) score = 1;
        if (score > 0) s_success++;
        duelAddPoints(score);
        char msg[64];
        snprintf(msg, sizeof(msg), "AMPEL:RESULT:%ld,%u,%lu",
                 (long)mm, (unsigned)score, (unsigned long)ms);
        ampelFinishRound(msg);
        ledStripShowResultFlash(score > 0);
        ledStripTick();
        return;
      }
      if (probeMM() > AMPEL_MISS_MM || now - s_phaseAt > AMPEL_MISS_MS) {
        duelAddPoints(0);
        ampelFinishRound("AMPEL:MISSED");
        ledStripShowResultFlash(false);
        ledStripTick();
      }
      return;
    }

    case AMPEL_CHEAT: {
      if (s_cheatPressed) {
        s_success++;
        ampelFinishRound("AMPEL:CHEATWIN");
        ledStripShowResultFlash(true);
        ledStripTick();
        return;
      }
      if (gameHaltRequested() || now - s_phaseAt > AMPEL_CHEAT_WINDOW) {
        gameHaltClear();
        duelAddPoints(0);
        ampelFinishRound("AMPEL:CHEATLATE");
        ledStripShowResultFlash(false);
        ledStripTick();
      }
      return;
    }

    case AMPEL_WAIT_NEXT:
    default:
      return;
  }
}

bool ampelCmd(const String& cmd) {
  if (!cmd.startsWith("AMPEL:")) return false;
  const String sub = cmd.substring(6);

  if (sub == "START" || sub == "NEXT") {
    if (s_active && s_phase != AMPEL_IDLE) {
      sendStatus("AMPEL:BUSY");
      return true;
    }
    if (!s_active) {
      if (currentGame != GAME_NONE) {
        sendStatus("AMPEL:BUSY");
        return true;
      }
      s_active = true;
    }
    currentGame = GAME_AMPEL;
    gameHaltClear();
    ampelBeginRound();
    return true;
  }

  if (sub == "CHEATPRESS") {
    if (s_active && s_phase == AMPEL_CHEAT) s_cheatPressed = true;
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      ampelStop();
    } else {
      sendStatus("AMPEL:STOPPED");
    }
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
