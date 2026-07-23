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
  GAME_MEMORY
};
extern GameMode currentGame;

// Bewegungs-Timings (in Edu_V1.3.cpp definiert)
extern const uint32_t GAME_KNOBELN_MS;  // Vor/Rueck-Fahrt Dauer
extern const uint32_t GAME_TURN_MS;     // 90-Grad-Drehung Dauer
extern const float    GAME_DRIVE_GAIN;  // Geschwindigkeits-Faktor

// Blockierende Bewegungsfunktionen (mit Rampe / Smooth)
void gameDriveForward(uint32_t ms);
void gameDriveBackward(uint32_t ms);
void gameTurnLeft(uint32_t ms);
void gameTurnRight(uint32_t ms);
void gameStop(uint32_t ms);
