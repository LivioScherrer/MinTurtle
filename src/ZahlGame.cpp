// EduBot V2 - B1 "Zahlentreffer" - Implementierung. Siehe ZahlGame.h.
#include "ZahlGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

constexpr float   ZAHL_SPEED_MM_S = 100.0f;   // ruhiges Messtempo auf Stufe 1
constexpr float   ZAHL_SPEED_PER_STAGE = 20.0f;   // +20 mm/s je Stufe
constexpr uint8_t ZAHL_STAGE_EVERY = 3;          // Erfolge pro Stufe
constexpr uint8_t ZAHL_STAGE_MAX   = 5;
// Stufen (Blockly-Prinzip): je 3 erfolgreiche Runden steigt die Stufe
// (max. 5), der Roboter rollt schneller. Ziel und Wertung bleiben immer
// gleich - nur das Treffen wird schwerer.
constexpr int16_t ZAHL_TARGET_MIN = 100;      // 10 cm
constexpr int16_t ZAHL_TARGET_MAX = 1000;     // 100 cm
constexpr int32_t ZAHL_OVERRUN_MM = 500;      // Sicherheitsnetz hinter dem Ziel

static bool    s_active = false;
static bool    s_driving = false;
static int16_t s_targetMM = 0;
static uint8_t s_success = 0;    // erfolgreiche Runden (Stufen-Zaehler)

bool zahlActive() { return s_active && s_driving; }
static uint8_t zahlStage() {
  uint8_t st = 1 + s_success / ZAHL_STAGE_EVERY;
  return (st > ZAHL_STAGE_MAX) ? ZAHL_STAGE_MAX : st;
}
static float zahlSpeed() {
  return ZAHL_SPEED_MM_S + (float)(zahlStage() - 1) * ZAHL_SPEED_PER_STAGE;
}

void zahlStop() {
  if (!s_active) return;
  s_active  = false;
  s_driving = false;
  s_success = 0;   // neues Spiel beginnt wieder auf Stufe 1 (fair)
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

  // Sicherheitsnetz: weit hinter dem Ziel noch nicht gebremst.
  if (probeMM() > (int32_t)s_targetMM + ZAHL_OVERRUN_MM) {
    s_driving = false;
    stopMotors();
    gameSpeedRestore();
    gameHaltClear();
    sendStatus("ZAHL:OVERRUN");
    return;
  }

  if (gameHaltRequested()) {
    gameHaltClear();
    s_driving = false;
    // HALT hat die Motoren bereits gestoppt; hier nur auswerten.
    // VORZEICHENBEHAFTET senden: + = zu weit gefahren, - = zu kurz.
    // Die UI zeigt damit die Richtung an und kann die Marke setzen.
    const int32_t driven = probeMM();
    const int32_t delta  = driven - s_targetMM;
    const int32_t abw    = (delta < 0) ? -delta : delta;
    const uint8_t score = probeScoreMM(abw);
    if (score > 0) s_success++;   // Erfolg zaehlt fuer die naechste Stufe
    duelAddPoints(score);   // Duell: Rundenpunkte an den Gegner melden
    gameSpeedRestore();
    char msg[48];
    snprintf(msg, sizeof(msg), "ZAHL:RESULT:%ld,%u", (long)delta, (unsigned)score);
    sendStatus(msg);
    ledStripShowResultFlash(score > 0);
    ledStripTick();
  }
}

bool zahlCmd(const String& cmd) {
  if (!cmd.startsWith("ZAHL:")) return false;
  const String sub = cmd.substring(5);

  if (sub.startsWith("START:")) {
    // Eigenes Spiel darf neu starten (Nochmal), alles andere ist belegt.
    if (currentGame != GAME_NONE && currentGame != GAME_ZAHL) {
      sendStatus("ZAHL:BUSY");
      return true;
    }
    if (s_active && s_driving) {   // es rollt gerade eine Fahrt
      sendStatus("ZAHL:BUSY");
      return true;
    }
    int v = sub.substring(6).toInt();
    v = ((v + 5) / 10) * 10;                       // auf 10er-Schritte runden
    if (v < 10)  v = 10;
    if (v > 100) v = 100;
    s_targetMM = (int16_t)(v * 10);
    s_active  = true;
    s_driving = true;
    currentGame = GAME_ZAHL;
    gameHaltClear();
    probeBegin();
    // Ruhiges Messtempo als RAM-Override (setMotorSpeeds ignoriert sonst den
    // Betrag), je Stufe schneller. Restore bei HALT/OVERRUN/STOP.
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
