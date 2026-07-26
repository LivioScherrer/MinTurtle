// EduBot V1.3 - Drive-Implementierung (WASD / Touch, AccelStepper)
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
  extern bool driveLockedByGame();
  if (driveLockedByGame()) return;

  // Freifahren: vom Benutzer einstellbare Beschleunigung (ACC:-Befehl).
  // Spiele/LineFollower setzen ihre eigene Beschleunigung davor.
  // Beim Lenken waehrend der Fahrt: deutlich hoehere Beschleunigung,
  // damit das Innenrad SOFORT runterfaehrt -> man kann waehrend der
  // Fahrt lenken, ohne das Gas wegzunehmen.
  float targetL = 0.0;
  float targetR = 0.0;

  bool forward  = keyW && !keyS;
  bool backward = keyS && !keyW;
  bool left     = keyA && !keyD;
  bool right    = keyD && !keyA;

  const bool moving   = (forward || backward);
  const bool steering = (left || right) && moving;
  const float driveAccel = steering ? 20000.0f : config.accelRate;
  stepperL.setMaxSpeed(config.maxSpeed);
  stepperR.setMaxSpeed(config.maxSpeed);
  stepperL.setAcceleration(driveAccel);
  stepperR.setAcceleration(driveAccel);

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

  // Timeout: kein BLE-Befehl mehr -> Stop
  if (millis() - lastCommandTime > config.commandTimeoutMs) {
    targetL = 0.0; targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0; speedR = 0.0;
      driveClearKeys();
    }
  }

  // AccelStepper erledigt die Rampe intern - nur Ziel setzen.
  setMotorSpeed(0, targetL);
  setMotorSpeed(1, targetR);
  speedL = targetL;
  speedR = targetR;

  motorRun();
}
