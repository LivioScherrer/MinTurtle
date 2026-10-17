#include "GameCommon.h"
#include "GameEngine.h"
#include "Motor.h"
#include "BLEManager.h"
#include "Config.h"
#include "ProgGame.h"

static bool s_haltRequested = false;
static bool s_manualDrive   = false;
static bool s_edgeSuspend   = false;

static float s_savedSpeed  = 150.0f;
static bool  s_savedValid  = false;

void gameHaltSet()     { s_haltRequested = true; }
bool gameHaltRequested() { return s_haltRequested; }
void gameHaltClear()   { s_haltRequested = false; }

void gameSetManualDrive(bool on) { s_manualDrive = on; }
bool gameAllowsManual()          { return s_manualDrive; }

void gameSuspendEdgeGuard(bool on) {
  if (on && !s_edgeSuspend) {
    edgeGuardReset();
    motorReleaseEmergency();
  }
  s_edgeSuspend = on;
}

bool gameEdgeGuardSuspended() { return s_edgeSuspend; }

void gameSpeedOverride(float mmS) {
  if (!s_savedValid) {
    s_savedSpeed = config.driveSpeedMM;
    s_savedValid = true;
  }
  float want = mmS;
  if (want < SPEED_MM_S_MIN) want = SPEED_MM_S_MIN;
  if (want > SPEED_MM_S_MAX) want = SPEED_MM_S_MAX;
  const float maxReach = MAX_DRIVE_STEPS_S / configStepsPerMM();
  if (want > maxReach) want = floorf(maxReach);
  if (config.driveSpeedMM != want) {
    config.driveSpeedMM = want;
    motorUpdateSpeeds();
    Serial.printf("[GAME] Tempo-Override %.0f mm/s (ohne NVS)\n", (double)want);
  }
}

void gameSpeedRestore() {
  if (!s_savedValid) return;
  s_savedValid = false;
  if (config.driveSpeedMM != s_savedSpeed) {
    config.driveSpeedMM = s_savedSpeed;
    motorUpdateSpeeds();
    Serial.printf("[GAME] Tempo zurueck auf %.0f mm/s\n", (double)s_savedSpeed);
  }
}

void gameModeEnd(const char* token) {
  stopMotors();
  currentGame = GAME_NONE;
  if (token != nullptr) sendStatus(token);
}
