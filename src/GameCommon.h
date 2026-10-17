#pragma once

#include <Arduino.h>

void gameHaltSet();
bool gameHaltRequested();
void gameHaltClear();

void gameSetManualDrive(bool on);
bool gameAllowsManual();

void gameSuspendEdgeGuard(bool on);
bool gameEdgeGuardSuspended();

void gameSpeedOverride(float mmS);
void gameSpeedRestore();

void gameModeEnd(const char* token);
