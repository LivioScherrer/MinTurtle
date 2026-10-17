#pragma once

#include <Arduino.h>

#define DIST_I2C_ADDR 0x30

constexpr uint32_t DIST_NOTIFY_MS = 500;

constexpr uint32_t DIST_VALUE_MAX_AGE_MS = 1000;



constexpr uint32_t DIST_SINGLE_PERIOD_MS = 200;
constexpr uint32_t DIST_SINGLE_LOCK_MS   = 120;
constexpr uint8_t  DIST_SINGLE_RETRIES   = 3;

bool distBlocked();

void distSetup();

bool distOk();

int16_t distMeasureMM();

bool distCalibrateOffset(int16_t trueMM);

bool distCalibrate3(int16_t raw50, int16_t raw100, int16_t raw200);

bool distMeasureRawAvg(int16_t& outRaw, uint8_t want = 8, uint8_t minOk = 3);

int16_t distOffsetMM();
void distClearOffset();

int distRefSpadCalibrate();

bool distRefSpadRequest();

void distRefSpadService();



int16_t distLastMM();

uint32_t distLastAgeMs();

void distTick();

void distDiag(char* buf, size_t bufLen);
void distDiagPrint();

void distDebugInit();

void distBootScan();

void distSetAuto(bool on);
bool distAuto();

