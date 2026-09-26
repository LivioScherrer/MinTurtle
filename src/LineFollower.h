// EduBot V2 - Linienfolge-Modul (Follow-Line via TCS34725 Schwarz-Erkennung)
#pragma once

#include <Arduino.h>

// Status des Linienfolge-Modus
extern bool lineFollowerActive;

// Linienfolge starten (setzt driveLock via currentGame in main)
void lineFollowerStart();
// Linienfolge stoppen (motoren aus, flag zurueck)
void lineFollowerStop();
// Tick - in loop() aufrufen, wenn currentGame == GAME_LINEFOLLOWER
void lineFollowerTick();
