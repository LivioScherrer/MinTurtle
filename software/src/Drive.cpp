#include "Drive.h"
#include "Motor.h"
#include "Config.h"
#include "GameEngine.h"

bool keyW = false;
bool keyA = false;
bool keyS = false;
bool keyD = false;

float speedL = 0.0;
float speedR = 0.0;

unsigned long lastCommandTime = 0;

void driveClearKeys() {
  keyW = false; keyA = false; keyS = false; keyD = false;
}

void driveHardStop() {
  driveClearKeys();
  speedL = 0.0;
  speedR = 0.0;
  stopMotors();
}

void driveUpdate() {
  if (driveLockedByGame()) return;

  static bool     s_wasMoving = false;
  static long     s_startL = 0, s_startR = 0;
  const bool      wantMove = (keyW || keyS || keyA || keyD);
  if (wantMove && !s_wasMoving) {
    motorStepPositions(s_startL, s_startR);
  } else if (!wantMove && s_wasMoving) {
    long endL = 0, endR = 0;
    motorStepPositions(endL, endR);
    const long dL = endL - s_startL, dR = endR - s_startR;
    const long aL = labs(dL), aR = labs(dR);
    const long mx = aL > aR ? aL : aR;
    const int diffPromille = (mx > 0) ? (int)(labs(aL - aR) * 1000L / mx) : 0;
    const long diff = labs(aL - aR);
    const float spmm = configStepsPerMM();
    const float mmL = (spmm > 0.0f) ? (float)aL / spmm : 0.0f;
    const float mmR = (spmm > 0.0f) ? (float)aR / spmm : 0.0f;
    Serial.printf("[DRV] Fahrt beendet: Schritte L=%ld R=%ld (Diff %ld) = %.1f / %.1f mm\n",
                  aL, aR, diff, (double)mmL, (double)mmR);
    Serial.printf("[DRV]   %s | Unterschied %ld Schritte = %.2f mm (%.1f Promille)\n",
                  diffPromille <= 10 ? "Zaehler praktisch gleich -> GLEICHE Pulse an beide Raeder"
                                     : "Zaehler unterscheiden sich -> Software faehrt die Raeder ungleich an",
                  diff, (spmm > 0.0f) ? (float)diff / spmm : 0.0f, (double)diffPromille);
  }
  s_wasMoving = wantMove;

  float targetL = 0.0;
  float targetR = 0.0;

  bool forward  = keyW && !keyS;
  bool backward = keyS && !keyW;
  bool left     = keyA && !keyD;
  bool right    = keyD && !keyA;

  if (forward)  { targetL = FIXED_DRIVE_SPEED; targetR = FIXED_DRIVE_SPEED; }
  else if (backward) { targetL = -FIXED_DRIVE_SPEED; targetR = -FIXED_DRIVE_SPEED; }
  if (!forward && !backward) {
    if (left)  { targetL = 0; targetR = FIXED_DRIVE_SPEED; }
    else if (right) { targetL = FIXED_DRIVE_SPEED; targetR = 0; }
  }


  const bool bHardStop = (millis() - lastCommandTime > config.commandTimeoutMs);
  if (bHardStop) {
    targetL = 0.0; targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0;
      speedR = 0.0;
    }
  }

  setMotorSpeeds(targetL, targetR, !bHardStop);
  speedL = targetL;
  speedR = targetR;
}

void driveDebug(char* out, size_t n) {
  const bool locked = driveLockedByGame();
  const bool forward  = keyW && !keyS;
  const bool backward = keyS && !keyW;
  const bool left     = keyA && !keyD;
  const bool right    = keyD && !keyA;
  char want[8] = "----";
  if      (forward)  { want[0] = 'F'; want[1] = 'F'; }
  else if (backward) { want[0] = 'B'; want[1] = 'B'; }
  else if (left)     { want[0] = 'L'; want[1] = 'R'; }
  else if (right)    { want[0] = 'R'; want[1] = 'L'; }
  const unsigned long age = millis() - lastCommandTime;
  float curL = 0.0f, curR = 0.0f;
  motorRampSpeeds(curL, curR);
  long posL = 0, posR = 0;
  motorStepPositions(posL, posR);
  snprintf(out, n,
    "DRV:keys=%d%d%d%d want=%s spd=%.0f/%.0f fix=%.0f lst=%.0f/%.0f "
    "pos=%ld/%ld d=%ld lock=%d game=%d cmdAlter=%lums timeout=%lums laeuft=%d "
    "sign=%d/%d en=%d",
    keyW, keyA, keyS, keyD, want,
    (double)speedL, (double)speedR, (double)FIXED_DRIVE_SPEED,
    (double)curL, (double)curR,
    posL, posR, posL - posR,
    locked ? 1 : 0, (int)currentGame, age,
    (unsigned long)config.commandTimeoutMs,
    motorStillRunning() ? 1 : 0,
    (int)config.leftSign, (int)config.rightSign,
    tmcIsEnabled() ? 1 : 0);
}
