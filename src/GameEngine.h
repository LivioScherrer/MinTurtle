// EduBot V1.3 - Spiel-Bewegungs-Engine (blockierend)
// Gemeinsam genutzt von Pfeil-Knobeln, Korrigieren, Rennen und Memory
#pragma once

#include <Arduino.h>

// Aktueller Spiel-Modus (gesetzt von handleCommand, rueckgesetzt wenn Spiel endet)
enum GameMode : uint8_t {
  GAME_NONE = 0,
  GAME_KNOBELN,
  GAME_KORRIGIEREN,
  GAME_RENNEN,
  GAME_MEMORY,
  GAME_LINEFOLLOWER,
  GAME_PROG,
  GAME_SLALOM
};
extern GameMode currentGame;

// Bewegungs-Timings (in Edu_V1.3.cpp definiert).
// Einheitlich fuer alle Spiele (Knobeln, Memory, Quiz, Rennen).
extern const uint32_t GAME_MOVE_MS;     // Vor/Rueck-Fahrt Dauer
extern const float    GAME_DRIVE_GAIN;  // Geschwindigkeits-Faktor

// Blockierende Bewegungsfunktionen (mit Rampe / Smooth)
void gameDriveForward(uint32_t ms);
void gameDriveBackward(uint32_t ms);
void gameTurnLeftSteps(int32_t steps);
void gameTurnRightSteps(int32_t steps);
void gameDriveForwardSteps(int32_t steps);
void gameStop(uint32_t ms);
