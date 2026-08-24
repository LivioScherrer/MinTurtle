// EduBot V1.3 - Motor-Implementierung mit AccelStepper
// Feste Geschwindigkeit (FIXED_DRIVE_SPEED) - keine Einstellung moeglich.
// Verhaelt sich wie direktes digitalWrite: sobald setMotorSpeed != 0, laeuft
// der Motor sofort mit fester Geschwindigkeit.
#include "Motor.h"
#include "Config.h"

AccelStepper stepperL(AccelStepper::DRIVER, PIN_STEP_L, PIN_DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, PIN_STEP_R, PIN_DIR_R);

void initSteppers() {
  // AccelStepper braucht eine Beschleunigung fuer move()/run(). Wir setzen
  // sie extrem hoch, damit keine Rampe entsteht (sofort volle Geschwindigkeit).
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperL.setAcceleration(10000.0f);
  stepperR.setAcceleration(10000.0f);
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  digitalWrite(PIN_STEP_L, LOW);
  digitalWrite(PIN_STEP_R, LOW);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);
}

// Start-Signal: nach dem Booten drehen beide Motoren kurz (2000 steps/s, 500ms).
// Wichtig: setSpeed() wird intern auf setMaxSpeed() begrenzt - deshalb hier
// vorher setMaxSpeed hochsetzen und danach wieder auf FIXED_DRIVE_SPEED zurueck.
void startupSpin() {
  stepperL.setMaxSpeed(2000.0f);
  stepperR.setMaxSpeed(2000.0f);
  stepperL.setSpeed(2000.0f);
  stepperR.setSpeed(2000.0f);
  uint32_t start = millis();
  while (millis() - start < 500) {
    stepperL.runSpeed();
    stepperR.runSpeed();
  }
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
}

// Geschwindigkeit: Richtung ueber Vorzeichen, Betrag immer FIXED_DRIVE_SPEED.
// Vorzeichen-Korrektur ueber config.leftSign/rightSign (manche Motoren sind verkehrt).
// speed == 0.0f bedeutet explizit STOPP (Motor bleibt stehen).
void setMotorSpeed(int motorIdx, float speed) {
  // Erstmal: wenn 0 uebergeben wird, Motor wirklich stoppen
  if (speed == 0.0f) {
    if (motorIdx == 0) stepperL.setSpeed(0.0f);
    else               stepperR.setSpeed(0.0f);
    return;
  }
  const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;
  // Richtung kommt vom Vorzeichen, Geschwindigkeit ist immer FIXED_DRIVE_SPEED
  const float actual = (speed > 0.0f ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED) * static_cast<float>(sign);
  if (motorIdx == 0) stepperL.setSpeed(actual);
  else               stepperR.setSpeed(actual);
}

void stopMotors() {
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
}

void motorRun() {
  stepperL.runSpeed();
  stepperR.runSpeed();
}

// Position-Move sauber vorbereiten: AccelStepper-Accel-State (_n/_cn),
// _speed und _direction werden zurueckgesetzt, damit run() sofort in
// die Richtung von physicalSteps startet (kein Fehltritt rueckwaerts,
// kein Kriechstart aus veralteter Rampe).
void prepPositionMove(AccelStepper& s, long physicalSteps, float speed) {
  s.setCurrentPosition(s.currentPosition());   // _n=0, _speed=0, _stepInterval=0
  s.setMaxSpeed(speed);
  s.setAcceleration(10000.0f);                 // schnelle Rampe (~33ms statt 1s)
  s.move(physicalSteps);
  s.setSpeed(physicalSteps >= 0 ? speed : -speed);  // Richtung = Ziel-Richtung
}

