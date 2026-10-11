// EduBot V2 - B9 "Einparken" - Implementierung. Siehe ParkGame.h.
#include "ParkGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"
#include "DistanceSensor.h"

constexpr float   PARK_SPEED_MM_S = 60.0f;    // langsames Heranrollen
constexpr int16_t PARK_TOUCH_MM   = 45;       // darunter: angestossen (Sensor-Blindzone)
constexpr int32_t PARK_EMPTY_MM   = 1500;     // ohne Hindernis: Abbruch
constexpr uint8_t PARK_STAGE_EVERY = 2;       // Erfolge pro Stufe
constexpr uint8_t PARK_STAGE_MAX   = 3;

static bool    s_active = false;
static bool    s_driving = false;
static uint8_t s_success = 0;    // erfolgreiche Runden (Stufen-Zaehler)

bool parkActive() { return s_active && s_driving; }
static uint8_t parkStage() {
  uint8_t st = 1 + s_success / PARK_STAGE_EVERY;
  return (st > PARK_STAGE_MAX) ? PARK_STAGE_MAX : st;
}
// Perfekt-/Gut-Band je Stufe (mm Restabstand).
static int16_t parkTolPerfect(uint8_t st) { return (st <= 1) ? 100 : (st == 2 ? 80 : 60); }
static int16_t parkTolGood(uint8_t st)    { return (st <= 1) ? 200 : (st == 2 ? 150 : 120); }

void parkStop() {
  if (!s_active) return;
  s_active  = false;
  s_driving = false;
  s_success = 0;   // neues Spiel beginnt wieder auf Stufe 1 (fair)
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_PARK) {
    gameModeEnd("PARK:STOPPED");
  } else {
    gameHaltClear();
  }
}

// Runde beenden (halten die Motoren bereits? restore immer).
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

  // Angestossen: von selbst gestoppt, Runde verloren (0 Punkte).
  const int16_t mm = distLastMM();
  if (mm >= 0 && mm < PARK_TOUCH_MM) {
    duelAddPoints(0);
    parkFinish("PARK:TOUCH");
    ledStripShowResultFlash(false);
    ledStripTick();
    return;
  }
  // Sicherheitsnetz: weit und breit kein Hindernis.
  if (probeMM() > PARK_EMPTY_MM) {
    parkFinish("PARK:EMPTY");
    return;
  }

  if (gameHaltRequested()) {
    gameHaltClear();
    if (mm < 0) {   // Sensor meldet nichts - keine faire Wertung moeglich
      parkFinish("PARK:STOPPED");
      return;
    }
    const uint8_t st = parkStage();
    uint8_t score = 0;
    if (mm < parkTolPerfect(st))      score = 2;
    else if (mm < parkTolGood(st))    score = 1;
    if (score > 0) s_success++;   // Erfolg zaehlt fuer die naechste Stufe
    duelAddPoints(score);   // Duell: Rundenpunkte an den Gegner melden
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
    // Eigenes Spiel darf neu starten, alles andere ist belegt.
    if (currentGame != GAME_NONE && currentGame != GAME_PARK) {
      sendStatus("PARK:BUSY");
      return true;
    }
    if (s_active && s_driving) {   // rollt gerade aufs Hindernis zu
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
