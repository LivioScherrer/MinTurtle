// EduBot V1.3 - Drive-Modul
// Frei-Fahren Modus: WASD/Touch -> sanftes Anfahren via Rampe
#pragma once

#include <Arduino.h>

extern bool keyW, keyA, keyS, keyD;
extern float speedL, speedR;
extern unsigned long lastCommandTime;
extern unsigned long lastUpdateMicros;

// Tasten loeschen
void driveClearKeys();
// Alles auf Stopp + Spielmodus aus
void driveHardStop();
// Drive-Tick (in loop() aufrufen, ausserhalb von Spielen)
void driveUpdate();
// Connected-Handler: dt zuruecksetzen
void driveResetDt();
