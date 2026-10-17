#pragma once

#include <Arduino.h>

bool ampelCmd(const String& cmd);

void ampelTick();

void ampelStop();

bool ampelActive();
bool ampelRed();
bool ampelCheat();
