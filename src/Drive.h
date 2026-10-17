#pragma once

#include <Arduino.h>

extern bool keyW, keyA, keyS, keyD;
extern float speedL, speedR;
extern unsigned long lastCommandTime;

void driveClearKeys();
void driveHardStop();
void driveUpdate();

void driveDebug(char* out, size_t n);
