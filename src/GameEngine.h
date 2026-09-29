// EduBot V2 - Spiel-Bewegungs-Engine (blockierend)
// Gemeinsam genutzt von Quiz, Rennen, Memory, Linienfolger und Programmierung.
#pragma once

#include <Arduino.h>
#include <stdint.h>

// Aktueller Spiel-Modus (gesetzt von handleCommand, rueckgesetzt wenn Spiel endet).
// != GAME_NONE sperrt die freie WASD-Steuerung (driveLockedByGame()).
enum GameMode : uint8_t {
  GAME_NONE = 0,
  GAME_KORRIGIEREN,
  GAME_RENNEN,
  GAME_MEMORY,
  GAME_LINEFOLLOWER,
  GAME_PROG,
  GAME_SLALOM
};
extern GameMode currentGame;

// true, solange ein Spiel die Bewegung steuert (KEYS: werden ignoriert).
bool driveLockedByGame();

// Bewegungs-Timings (in Edu_V2.cpp definiert).
// Einheitlich fuer alle Spiele, damit der Ablauf konsistent bleibt.
extern const uint32_t GAME_MOVE_MS;       // Vor/Rueck-Fahrt Dauer
extern const uint32_t GAME_COLOR_READ_MS; // Pause, waehrend der Farbsensor liest

// true, solange ein blockierender Spiel-Move laeuft.
extern bool gameMoveBusy;

// ---- Blockierende Bewegungs-Primitiven ----
// Die wegsatzlichen Primitiven (Vorwaerts fuer x mm) fahren mit fester
// Geschwindigkeit (FIXED_DRIVE_SPEED) und - sofern config.rampMs > 0 -
// sanfter Rampe (AccelStepper run()/setAcceleration). Die zeitlich
// begrenzten (fahre 400 ms) fahren BEWUSST ohne Rampe: dort ist die Zeit das
// Kommando, und 150 ms Anlaufzeit waeren ein Drittel der Fahrzeit.
// Die Step-Pulse erzeugt der Motor-Task; hier wird nur darauf gewartet.
// Ein Spielabbruch (currentGame -> GAME_NONE, z.B. Notfall-STOP) bricht
// jeden Move ab, damit der Robrometer nicht weiterfaehrt.

// Vorwaerts/Zurueck fuer 'ms' Millisekunden.
void gameDriveForward(uint32_t ms);
void gameDriveBackward(uint32_t ms);

// Drehung auf der Stelle: 'steps' Schritte pro Rad, entgegengesetzte Richtungen.
void gameTurnLeftSteps(int32_t steps);
void gameTurnRightSteps(int32_t steps);

// Positionsfahrt vorwaerts: exakte Strecke in Schritten (z.B. 3cm im Quiz).
void gameDriveForwardSteps(int32_t steps);
void gameDriveBackwardSteps(int32_t steps);

// Motoren stehen fuer 'ms' Millisekunden (Pause-Fenster, z.B. Farb-Lesezeit).
void gameStop(uint32_t ms);
