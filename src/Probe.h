#pragma once

#include <Arduino.h>

void probeBegin();

int32_t probeMM();

uint8_t probeScoreMM(int32_t mm);

constexpr int32_t PROBE_PERFECT_MM = 10;
constexpr int32_t PROBE_GOOD_MM    = 30;
