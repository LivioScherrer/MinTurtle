// EduBot V2 - Linienfolge-Modul (Follow-Line via TCS34725 Schwarz-Erkennung)
#pragma once

#include <Arduino.h>
#include <LedStrip.h>   // LedLineState (Zustand fuer die LED-Statusanzeige)

// Status des Linienfolge-Modus
extern bool lineFollowerActive;

// Grober Zustand fuer die LED-Statusanzeige (LED_LINE_*).
LedLineState lineFollowerLedState();

// Linienfolge starten (setzt driveLock via currentGame in main)
void lineFollowerStart();
// Linienfolge stoppen (motoren aus, flag zurueck)
void lineFollowerStop();
// Tick - in loop() aufrufen, wenn currentGame == GAME_LINEFOLLOWER
void lineFollowerTick();
