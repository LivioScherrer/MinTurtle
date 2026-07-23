// EduBot V1.3 - Drive-Implementierung
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
unsigned long lastUpdateMicros = 0;

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
  lastUpdateMicros = micros();
}

void driveUpdate() {
  // Waehrend ein Spiel aktiv ist, wird die Bewegung von der Spiel-Engine
  // blockierend gesteuert - KEYS: vom Frontend werden ignoriert.
  // (Der Aufrufer in main entscheidet, ob driveUpdate() aufgerufen wird.)
  extern bool driveLockedByGame();
  if (driveLockedByGame()) return;

  float targetL = 0.0;
  float targetR = 0.0;

  bool forward  = keyW && !keyS;
  bool backward = keyS && !keyW;
  bool left     = keyA && !keyD;
  bool right    = keyD && !keyA;

  if (forward) { targetL = config.maxSpeed; targetR = config.maxSpeed; }
  else if (backward) { targetL = -config.maxSpeed; targetR = -config.maxSpeed; }
  if (forward || backward) {
    if (left)  targetL *= config.turnFactor;
    if (right) targetR *= config.turnFactor;
  }
  if (!forward && !backward) {
    if (left)  { targetL = 0; targetR = config.turnOnlySpeed; }
    else if (right) { targetL = config.turnOnlySpeed; targetR = 0; }
  }

  if (millis() - lastCommandTime > config.commandTimeoutMs) {
    targetL = 0.0; targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0; speedR = 0.0;
      driveClearKeys();
      stopMotors();
    }
    lastUpdateMicros = micros();
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastUpdateMicros) / 1000000.0;
  lastUpdateMicros = now;
  if (dt < 0.0001f || dt > 1.0f) dt = 0.001f;

  float newSpeedL = rampSpeed(speedL, targetL, dt);
  if (newSpeedL != speedL) { speedL = newSpeedL; setMotorSpeed(0, speedL); }
  float newSpeedR = rampSpeed(speedR, targetR, dt);
  if (newSpeedR != speedR) { speedR = newSpeedR; setMotorSpeed(1, speedR); }
}
