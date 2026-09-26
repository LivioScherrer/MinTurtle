// EduBot V2 - Drive-Modul
// Frei-Fahren-Modus: WASD/Touch -> linke/rechte Zielgeschwindigkeit.
// Die Motoren fahren immer mit fester Geschwindigkeit (FIXED_DRIVE_SPEED);
// eine Rampe/Beschleunigung gibt es bewusst nicht.
#pragma once

#include <Arduino.h>

// Tastenstatus (von "KEYS:<W><A><S><D>" als '0'/'1' gesetzt)
extern bool keyW, keyA, keyS, keyD;
// Letzte gesetzte Zielgeschwindigkeiten (nur Status/Anzeige)
extern float speedL, speedR;
// Zeitstempel des letzten eingehenden Befehls (Basis fuer config.commandTimeoutMs)
extern unsigned long lastCommandTime;

// Tasten loeschen
void driveClearKeys();
// Alles auf Stopp (Keys + Geschwindigkeiten + Motoren)
void driveHardStop();
// Drive-Tick (in loop() aufrufen, ausserhalb von Spielen).
// Ueberfuehrt den Tastenstatus in Zielgeschwindigkeiten und setzt nach
// config.commandTimeoutMs ohne Befehl sicher auf Stopp.
void driveUpdate();
