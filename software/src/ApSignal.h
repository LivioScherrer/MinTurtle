#pragma once

#include <Arduino.h>

#define APSIG_IDLE       0
#define APSIG_WANT_RED   1
#define APSIG_WANT_BLUE  2
#define APSIG_WANT_GREEN 3
#define APSIG_ANNOUNCED  4

void apSignalTick();

uint8_t apSignalStep();

bool apSignalActive();

void apSignalReset();