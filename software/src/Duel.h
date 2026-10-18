#pragma once

#include <Arduino.h>
#include "EspNow.h"

bool duelCmd(const String& cmd);

void duelTick();

void duelAddPoints(int points);

void duelOnPacket(const EduPacket* pkt);

void duelStop();

bool duelActive();

bool duelBusy();
