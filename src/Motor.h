// EduBot V1.3 - Motor-Modul (DRV8834 Stepper, AccelStepper)
// Feste Geschwindigkeit - keine Einstellung irgendwo im Code.
#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

// Stepper-Pins (Xiao ESP32-S3)
#define PIN_DIR_L   3
#define PIN_STEP_L  4
#define PIN_DIR_R   1
#define PIN_STEP_R   2

// Feste Fahrgeschwindigkeit in steps/s.
// Einzige Geschwindigkeits-Konstante im gesamten Projekt.
// 333 steps/s = 3ms zwischen den STEP-Pulsen.
constexpr float FIXED_DRIVE_SPEED  = 450.0f;

// Stepper-Instanzen (in Motor.cpp definiert)
extern AccelStepper stepperL;
extern AccelStepper stepperR;

// Geschwindigkeit setzen (steps/s, Vorzeichen = Richtung).
// Die Geschwindigkeit wird auf FIXED_DRIVE_SPEED begrenzt - egal was uebergeben wird.
void setMotorSpeed(int motorIdx, float speed);
// Beide Motoren sofort stoppen.
void stopMotors();
// Jede Loop aufrufen: fuehrt die geplanten Schritte aus.
void motorRun();
// Position-Move sauber vorbereiten (AccelStepper-State + Richtung):
// setzt _n/_speed zurueck, damit run() ohne Fehltritt und ohne
// Rest-Rampen-Zustand in die richtige Richtung startet.
// physicalSteps: vorzeichenkorrigierte Schritte (positiv = CW).
void prepPositionMove(AccelStepper& s, long physicalSteps, float speed);
// Initialisierung (Pins + Stepper-Defaults, feste Geschwindigkeit)
void initSteppers();
// Start-Signal nach dem Booten: beide Motoren drehen kurz (2000 steps/s, 500ms)
void startupSpin();
