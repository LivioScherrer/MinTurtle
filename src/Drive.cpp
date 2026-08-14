// EduBot V1.3 - Drive-Implementierung (WASD / Touch)
// Feste Geschwindigkeit aus Motor.h - keine Einstellung moeglich.
#include "Drive.h"
#include "Motor.h"
#include "Config.h"

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

void driveResetDt() {
}

void driveUpdate() {
  // Waehrend ein Spiel aktiv ist, wird die Bewegung von der Spiel-Engine
  // blockierend gesteuert - KEYS: vom Frontend werden ignoriert.
  if (driveLockedByGame()) return;

  // Feste Geschwindigkeit - keine Einstellung irgendwo.
  // Drehen auf der Stelle: ein Motor still, der andere mit voller Geschwindigkeit.
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

  // Timeout: kein BLE-Befehl mehr -> Stop
  if (millis() - lastCommandTime > config.commandTimeoutMs) {
    targetL = 0.0; targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0; speedR = 0.0;
      driveClearKeys();
    }
  }

  // Feste Geschwindigkeit setzen - Motor.cpp nutzt immer FIXED_DRIVE_SPEED.
  setMotorSpeed(0, targetL);
  setMotorSpeed(1, targetR);
  speedL = targetL;
  speedR = targetR;

  motorRun();
}
