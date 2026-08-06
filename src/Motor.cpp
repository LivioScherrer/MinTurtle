// EduBot V1.3 - Motor-Implementierung mit AccelStepper
// Konstante Beschleunigung in steps/s^2 -> gleichmaessiges Anfahren/Bremsen
// beider Stepper -> kein Wheelie, kein Schraeglauf mehr.
#include "Motor.h"
#include "Config.h"

AccelStepper stepperL(AccelStepper::DRIVER, PIN_STEP_L, PIN_DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, PIN_STEP_R, PIN_DIR_R);

void initSteppers() {
  stepperL.setMaxSpeed(MOTOR_MAX_SPEED);
  stepperR.setMaxSpeed(MOTOR_MAX_SPEED);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
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

// Geschwindigkeit in steps/s; Richtung ueber Vorzeichen.
// Vorzeichen-Korrektur ueber config.leftSign/rightSign (manche Motoren sind verkehrt).
void setMotorSpeed(int motorIdx, float speed) {
  const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;
  float actual = speed * static_cast<float>(sign);
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

// Kurzer Piepton direkt nach dem Booten: beide Stepper ~600ms auf
// einer festen Frequenz fahren -> hörbarer Ton. Danach sofort auf 0.
void playStartupBeep() {
  constexpr uint32_t BEEP_MS = 600;
  constexpr float    BEEP_SPEED = 1200.0f;  // etwas unter MOTOR_MAX_SPEED -> angenehmerer Ton
  stepperL.setMaxSpeed(MOTOR_MAX_SPEED);
  stepperR.setMaxSpeed(MOTOR_MAX_SPEED);
  stepperL.setSpeed(BEEP_SPEED);
  stepperR.setSpeed(BEEP_SPEED);
  uint32_t start = millis();
  while (millis() - start < BEEP_MS) {
    stepperL.runSpeed();
    stepperR.runSpeed();
  }
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
}
