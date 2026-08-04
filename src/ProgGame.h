// EduBot V1.3 - Programmieren-Spiel (Code-Bausteine, Scratch-artig)
// User baut auf der Webseite per Drag & Drop ein Programm aus Bloecken:
// Fahren (cm), Drehen (Grad), Warten (s), Wiederhole-n-mal, Wiederhole-bis-Farbe.
#pragma once

#include <Arduino.h>

// Grenzen (kodiertes Programm laeuft ueber BLE -> Webseite begrenzt Laenge)
constexpr uint8_t PROG_MAX_STEPS      = 20;  // Top-Level-Bloecke
constexpr uint8_t PROG_LOOP_BODY_MAX  = 8;   // Bloecke pro Loop
constexpr uint8_t PROG_UNTIL_MAX_ITER = 60;  // Sicherheits-Limit fuer Bis-Farbe-Loops

// Baustein-Op-Codes
enum ProgOp : uint8_t {
  OP_FWD = 0,   // F<n>  vorwaerts n cm
  OP_BACK,      // B<n>  rueckwaerts n cm
  OP_LEFT,      // L<n>  links drehen n Grad
  OP_RIGHT,     // R<n>  rechts drehen n Grad
  OP_WAIT,      // W<n>  warten n Sekunden
  OP_REPEAT,    // U<n>(body)  Body n-mal wiederholen
  OP_UNTIL      // C<c>(body)  Body wiederholen bis Farbe c sichtbar
};

// Farb-Indizes fuer OP_UNTIL (Namen in ProgGame.cpp)
constexpr uint8_t PROG_COLOR_ROT   = 0;
constexpr uint8_t PROG_COLOR_GELB  = 1;
constexpr uint8_t PROG_COLOR_GRUEN = 2;
constexpr uint8_t PROG_COLOR_BLAU  = 3;

// Einfacher Baustein (auch Loop-Body: keine verschachtelten Loops)
struct ProgBodyStep {
  uint8_t op;    // OP_FWD..OP_WAIT
  int16_t val;   // cm / Grad / Sekunden
};

struct ProgStep {
  uint8_t  op;        // ProgOp
  int16_t  val;       // cm / Grad / s / Wiederhol-Anzahl
  uint8_t  color;     // nur OP_UNTIL: Ziel-Farbe
  uint8_t  bodyLen;   // nur OP_REPEAT/OP_UNTIL
  ProgBodyStep body[PROG_LOOP_BODY_MAX];
};

extern ProgStep progSteps[PROG_MAX_STEPS];
extern uint8_t  progLen;
extern bool     progActive;

// Programm aus kompaktem String parsen.
// Format: Top-Level durch Kommas getrennt.
//   F10 = 10cm vorwaerts, R90 = 90 Grad rechts, W2 = 2s warten
//   U3(F10,R90) = 3x wiederholen, C2(F5) = wiederholen bis GRUEN
// true bei Erfolg, false bei Syntaxfehler / leerem Programm
bool progSetFromString(const char* s);

// Programm ausfuehren (blockierend, wie Pfeil-Knobeln).
// Setzt currentGame = GAME_PROG und sendet PROG:* Statusmeldungen.
void progRun();

// Abbruch (aus handleCommand / hardStop / Disconnect)
void progStop();
