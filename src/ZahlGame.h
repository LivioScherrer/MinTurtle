#pragma once

#include <Arduino.h>

bool zahlCmd(const String& cmd);
void zahlTick();
void zahlStop();

bool zahlActive();
