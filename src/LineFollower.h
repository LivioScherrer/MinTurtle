#pragma once

#include <Arduino.h>
#include <LedStrip.h>

extern bool lineFollowerActive;

LedLineState lineFollowerLedState();

void lineFollowerStart();
void lineFollowerStop();
void lineFollowerTick();
