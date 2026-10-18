#pragma once

#include <Arduino.h>

bool parkCmd(const String& cmd);
void parkTick();
void parkStop();

bool parkActive();
