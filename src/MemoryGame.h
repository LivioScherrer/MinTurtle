// EduBot V1.3 - Memory-Spiel (A1)
// Roboter zeigt Sequenz aus Bewegungen (gleich wie Pfeil-Knobeln), User wiederholt per Web-Buttons
#pragma once

#include <Arduino.h>

// Max. Sequenzlaenge und Sequenz-Schritte
constexpr uint8_t MEMORY_MAX_LEN = 16;
// Bewegungs-Codes
constexpr uint8_t MEM_FWD   = 0;
constexpr uint8_t MEM_BACK  = 1;
constexpr uint8_t MEM_LEFT  = 2;
constexpr uint8_t MEM_RIGHT = 3;

// State
extern uint8_t memorySeq[MEMORY_MAX_LEN];
extern uint8_t memoryLen;
extern uint8_t memoryInputIdx;  // Welche Eingabe erwartet
extern bool    memoryActive;     // Laeuft ein Spiel?
extern bool    memoryAwaitInput;// Wartet auf Usereingabe (Show-Phase vorbei)

// Spiel starten mit Anfangsschritten (z.B. 3) - baut intern eine neue Sequenz
// und zeigt sie blockierend vor (gleiches Bewegungs-Set wie Pfeil-Knobeln)
void memoryStart(uint8_t startLen = 3);
// Abbruch
void memoryStop();
// Ein User-Klick (0..3) - true wenn korrekt
bool memoryInput(uint8_t move);
// Tick (in loop() aufrufen) - steuert Timeout der Usereingabe
void memoryTick();
