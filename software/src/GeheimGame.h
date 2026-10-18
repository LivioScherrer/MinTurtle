#pragma once

#include <Arduino.h>

bool geheimCmd(const String& cmd);
void geheimTick();
void geheimStop();

bool geheimActive();
