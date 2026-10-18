#pragma once

#include <Arduino.h>

bool zeitCmd(const String& cmd);
void zeitTick();
void zeitStop();

bool zeitActive();
