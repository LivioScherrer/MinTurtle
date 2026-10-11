// EduBot V2 - Spiel-Bewegungs-Engine (blockierend)
// Gemeinsam genutzt von Quiz, Rennen, Memory, Linienfolger, Programmierung
// und den Spiel-Modi (A5/B1/B8/B9/B10).
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
  GAME_SLALOM,
  // Uebrige Spiel-Modi (Ampel, Zahlentreffer). Die frueheren Eintraege
  // GAME_WINKEL (B2), GAME_HAND (A6), GAME_FORM (B4/C1), GAME_SUN (B5),
  // GAME_MINE (A9), GAME_FOOD (B6) und GAME_EXPLORE (C2) sind gestrichen;
  // die Werte von GAME_AMPEL/GAME_ZAHL bleiben dadurch unveraendert.
  GAME_AMPEL,   // A5 "Ampel" (Gruenes Licht, rotes Licht)
  GAME_ZAHL,    // B1 "Zahlentreffer" (Praezisions-Parcours)
  GAME_GEHEIM,  // B8 "Geheim-Strecke" (Schaetzen + Kalibrieren)
  GAME_PARK,    // B9 "Einparken" (Distanz-Bremsung)
  GAME_ZEIT,    // B10 "Zeitfahren" (Tempo gegen die Uhr)
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
