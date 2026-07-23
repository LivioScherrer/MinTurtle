// EduBot V1.3 - Motor-Modul (DRV8834 Stepper, Bresenham-Timer)
#pragma once

#include <Arduino.h>
#include <driver/timer.h>
#include "soc/gpio_struct.h"

// Stepper-Pins (Xiao ESP32-S3)
#define PIN_DIR_L   3
#define PIN_STEP_L  4
#define PIN_DIR_R   1
#define PIN_STEP_R  2
#define PIN_ENABLE  7

#define STEP_BIT_L  (1UL << PIN_STEP_L)
#define STEP_BIT_R  (1UL << PIN_STEP_R)

// Stepper-Timing
constexpr uint32_t STEP_TIMER_US      = 10;
constexpr uint32_t MIN_STEP_INTERVAL  = 50;
constexpr uint32_t MAX_STEP_INTERVAL  = 50000;
constexpr uint32_t STEP_PERIOD_US     = 1000000UL;

extern portMUX_TYPE stepperMux;

// Geschwindigkeiten (fuer Drive/GameLogic lesbar)
extern volatile uint32_t stepIntervalL, stepIntervalR;

// Motor ansteuern (Geschwindigkeit in "steps/s", Vorzeichen = Richtung)
void setMotorSpeed(int motorIdx, float speed);
// Beide Motoren sofort stoppen
void stopMotors();
// Hardware-Timer initialisieren
void initStepperTimer();
// Rampen-Funktion (sanftes Anfahren/Bremsen) - delta-t in Sekunden
float rampSpeed(float current, float target, float dt);
