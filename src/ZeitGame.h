// EduBot V2 - B10 "Zeitfahren"
//
// Der Roboter faehrt los bis zur gewaehlten Zielfarbe (ROT/GELB/GRUEN/BLAU
// als Klebeband am Boden). Die Zeit wird gemessen: Bestzeit knacken ist das
// Spiel - Tempo gegen die Uhr statt Praezision. Zwei Treffer in Folge
// (Entprellung) zaehlen als Ziel, Timeout ohne Farbe bricht ab.
//
// Steigerung ueber Bestzeit-Jagd (keine Stufen): Neue Bestzeit gibt
// 2 Duell-Punkte, Ankommen 1 Punkt. STOP loescht die Bestzeit (fairer
// Neustart).
//
// Kommandos: ZEIT:START:<0-3> (Zielfarbe), ZEIT:STOP. HALT bricht die Fahrt
//   als Aufgabe ab (ZEIT:STOPPED).
// Meldungen: ZEIT:GO, ZEIT:RESULT:<ms>,<0-2>, ZEIT:NOTFOUND,
//   ZEIT:STOPPED, ZEIT:BUSY.
#pragma once

#include <Arduino.h>

bool zeitCmd(const String& cmd);
void zeitTick();
void zeitStop();

// Fuer die LED-Prioritaetenkette.
bool zeitActive();
