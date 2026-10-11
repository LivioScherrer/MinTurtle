// EduBot V2 - A5 "Ampel" - Implementierung. Siehe AmpelGame.h.
#include "AmpelGame.h"
#include "GameCommon.h"
#include "GameEngine.h"
#include "Probe.h"
#include "Motor.h"
#include "BLEManager.h"
#include "LedStrip.h"
#include "Duel.h"

// ---- Parameter (eine Stelle; die Punkteschwellen bewusst hier, damit die
// Schwellen der Probe (10/30 mm fuer B1) davon unabhaengig bleiben) ----
constexpr float    AMPEL_ROLL_MM_S    = 60.0f;   // Rollgeschwindigkeit
constexpr uint32_t AMPEL_RED_MIN_MS   = 2000;    // Rot kommt nach 2..6 s
constexpr uint32_t AMPEL_RED_MAX_MS   = 6000;
constexpr uint32_t AMPEL_CHEAT_WINDOW = 3000;    // Zeit zum Schummler-Entdecken
constexpr int32_t  AMPEL_STAR_MM      = 50;      // < 50 mm Nachrollweg = Stern
constexpr int32_t  AMPEL_GOOD_MM      = 150;     // < 150 mm = gut
constexpr int32_t  AMPEL_MISS_MM      = 1500;    // Sicherheitsnetz
constexpr uint32_t AMPEL_MISS_MS      = 20000;   // Sicherheitsnetz
constexpr uint8_t  AMPEL_CHEAT_DENOM  = 6;       // 1 von 6 Runden ist Schummler
// Stufen (Blockly-Prinzip: gleiche Regeln, wachsender Anspruch): je 3
// erfolgreiche Runden steigt die Stufe (max. 5), das Rot-Fenster wird enger.
// Die Wertung bleibt immer gleich - nur das Kommen von Rot wird fieser.
constexpr uint8_t  AMPEL_STAGE_EVERY  = 3;       // Erfolge pro Stufe
constexpr uint8_t  AMPEL_STAGE_MAX    = 5;
constexpr uint32_t AMPEL_RED_SHRINK   = 1000;    // ms weniger MAX pro Stufe

enum AmpelPhase : uint8_t {
  AMPEL_IDLE = 0,
  AMPEL_ARMING,     // rollt, Rot kommt noch
  AMPEL_RED,        // Rot gezeigt, wartet auf HALT
  AMPEL_CHEAT,      // Roboter hat "geschummelt", wartet auf CHEATPRESS
  AMPEL_WAIT_NEXT   // Ergebnis gezeigt, wartet auf NEXT
};

static bool     s_active    = false;
static uint8_t  s_phase     = AMPEL_IDLE;
static uint32_t s_phaseAt   = 0;
static uint32_t s_redAfter  = 0;      // Zufallszeit bis Rot in dieser Runde
static bool     s_cheatRound = false;
static bool     s_cheatPressed = false;
static uint8_t  s_success   = 0;      // erfolgreiche Runden (Stufen-Zaehler)

bool ampelActive() { return s_active; }
bool ampelRed()    { return s_active && s_phase == AMPEL_RED; }
bool ampelCheat()  { return s_active && s_phase == AMPEL_CHEAT; }
uint8_t ampelStage() {
  uint8_t st = 1 + s_success / AMPEL_STAGE_EVERY;
  return (st > AMPEL_STAGE_MAX) ? AMPEL_STAGE_MAX : st;
}

// Runde beginnen: Phase setzen, Probe nullen, losrollen.
static void ampelBeginRound() {
  s_cheatRound   = (random(0, AMPEL_CHEAT_DENOM) == 0);
  s_cheatPressed = false;
  s_phase        = AMPEL_ARMING;
  s_phaseAt      = millis();
  // Rot-Fenster je Stufe: MIN bleibt, MAX schrumpft.
  const uint32_t st = ampelStage();
  uint32_t redMax = (AMPEL_RED_MAX_MS > (uint32_t)(st - 1) * AMPEL_RED_SHRINK)
      ? AMPEL_RED_MAX_MS - (uint32_t)(st - 1) * AMPEL_RED_SHRINK
      : AMPEL_RED_MIN_MS;
  if (redMax < AMPEL_RED_MIN_MS) redMax = AMPEL_RED_MIN_MS;
  s_redAfter = (redMax <= AMPEL_RED_MIN_MS)
      ? AMPEL_RED_MIN_MS
      : random(AMPEL_RED_MIN_MS, redMax + 1);
  probeBegin();
  // Langsames Rolltempo: RAM-Override (setMotorSpeeds haelt den Betrag sonst
  // immer auf FIXED_DRIVE_SPEED). Restore beim Runden-Ende/Abbruch.
  gameSpeedOverride(AMPEL_ROLL_MM_S);
  setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
  sendStatus("AMPEL:ARMED");
  sendStatusF("AMPEL:STAGE:%u", (unsigned)ampelStage());
}

static void ampelFinishRound(const char* token) {
  stopMotors();
  gameSpeedRestore();     // Modus rollt nicht mehr - altes Tempo zurueck
  s_phase = AMPEL_IDLE;   // Ergebnis steht; NEXT startet die naechste Runde
  if (token != nullptr) sendStatus(token);
}

void ampelStop() {
  if (!s_active) return;
  s_active = false;
  s_phase  = AMPEL_IDLE;
  s_success = 0;   // neues Spiel beginnt wieder auf Stufe 1 (fair)
  stopMotors();
  gameSpeedRestore();
  if (currentGame == GAME_AMPEL) {
    gameModeEnd("AMPEL:STOPPED");
  } else {
    // hardStop()/Disconnect war schon dran: nur aufraeumen, nichts melden.
    gameHaltClear();
  }
}

void ampelTick() {
  if (!s_active) return;
  // Not-Aus oder Modewechsel von aussen: still aufraeumen.
  if (currentGame != GAME_AMPEL) {
    s_active = false;
    s_phase  = AMPEL_IDLE;
    return;
  }

  const uint32_t now = millis();

  switch (s_phase) {
    case AMPEL_ARMING: {
      // Zu frueh gebremst: Runde verloren (klare Rueckmeldung, kein stiller No-Op).
      if (gameHaltRequested()) {
        gameHaltClear();
        ampelFinishRound("AMPEL:EARLY");
        return;
      }
      if (now - s_phaseAt < s_redAfter) return;
      if (s_cheatRound) {
        // Schummler: Roboter stoppt von selbst, obwohl GRUEN ist.
        stopMotors();
        s_phase   = AMPEL_CHEAT;
        s_phaseAt = now;
        sendStatus("AMPEL:CHEAT");
      } else {
        // Rot: Probe ab genau diesem Moment, Roboter rollt weiter.
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
        const int32_t mm = probeMM();
        uint8_t score = 0;
        if (mm < AMPEL_STAR_MM)      score = 2;
        else if (mm < AMPEL_GOOD_MM) score = 1;
        if (score > 0) s_success++;   // Erfolg zaehlt fuer die naechste Stufe
        duelAddPoints(score);   // Duell: Rundenpunkte an den Gegner melden
        char msg[48];
        snprintf(msg, sizeof(msg), "AMPEL:RESULT:%ld,%u", (long)mm, (unsigned)score);
        ampelFinishRound(msg);
        ledStripShowResultFlash(score > 0);
        ledStripTick();
        return;
      }
      // Sicherheitsnetz: nie gebremst -> Runde vorbei.
      if (probeMM() > AMPEL_MISS_MM || now - s_phaseAt > AMPEL_MISS_MS) {
        ampelFinishRound("AMPEL:MISSED");
        ledStripShowResultFlash(false);
        ledStripTick();
      }
      return;
    }

    case AMPEL_CHEAT: {
      // CHEATPRESS gewinnt, HALT (nicht erkannt) oder Zeitablauf verlieren.
      if (s_cheatPressed) {
        s_success++;   // entdeckter Schummler zaehlt wie ein Erfolg
        ampelFinishRound("AMPEL:CHEATWIN");
        ledStripShowResultFlash(true);
        ledStripTick();
        return;
      }
      if (gameHaltRequested() || now - s_phaseAt > AMPEL_CHEAT_WINDOW) {
        gameHaltClear();
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
      // Laeuft schon eine Runde: kein Neustart mitten hinein.
      sendStatus("AMPEL:BUSY");
      return true;
    }
    if (!s_active) {
      // Anderes Spiel am Laufen? (eigener Modus ist aus, sonst oben abgefangen)
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
