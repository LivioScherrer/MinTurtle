// EduBot V1.3 - Motor-Modul (DRV8834 Stepper, AccelStepper)
#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

// Stepper-Pins (Xiao ESP32-S3)
#define PIN_DIR_L   3
#define PIN_STEP_L  4
#define PIN_DIR_R   1
#define PIN_STEP_R   2
#define PIN_ENABLE   7

// Maximale Step-Rate, die der Roboter je fahren soll (steps/s)
constexpr float MOTOR_MAX_SPEED    = 2500.0f;
// Feste Beschleunigung (steps/s^2). 1500 -> 0->380 steps/s in ca. 250ms.
// Wird vom Freifahren via setAcceleration(config.accelRate) ueberschrieben.
constexpr float GLOBAL_ACCEL_RATE  = 1500.0f;

// Stepper-Instanzen (in Motor.cpp definiert)
extern AccelStepper stepperL;
extern AccelStepper stepperR;

// Geschwindigkeit setzen (steps/s, Vorzeichen = Richtung).
// Ueber AccelStepper intern beschleunigt/abgebremst -> kein Wheelie.
void setMotorSpeed(int motorIdx, float speed);
// Beide Motoren sanft auf 0 (AccelStepper rampt runter).
void stopMotors();
// Jede Loop aufrufen: fuehrt die geplanten Schritte aus.
void motorRun();
// Initialisierung (Pins + Stepper-Defaults)
void initSteppers();
// Kurzer Startton: beide Motoren kurz mit Maximalgeschwindigkeit fahren
void playStartupBeep();
