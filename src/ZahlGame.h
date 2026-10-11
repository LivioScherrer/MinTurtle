// EduBot V2 - B1 "Zahlentreffer" (Praezisions-Parcours)
//
// Der Roboter faehrt geradeaus auf eine Zieldistanz zu (z.B. 70 cm). Das Kind
// bremst mit HALT (weicher Spiel-Stopp) genau auf der Markierung. Die
// Probe-Routine vergleicht die gefahrene Strecke mit dem Ziel:
//   < 10 mm Abweichung = perfekt, < 30 mm = gut, sonst daneben.
//
// Stufen: je 3 erfolgreiche Runden steigt die Stufe (max. 5, gemeldet als
// ZAHL:STAGE:<1-5>); der Roboter rollt pro Stufe 20 mm/s schneller
// (100..180 mm/s). Ziel und Wertung bleiben immer gleich. STOP setzt
// zurueck auf Stufe 1.
//
// Kommandos: ZAHL:START:<cm> (10..100, 10er-Schritte), ZAHL:STOP, HALT.
// Meldungen: ZAHL:GO:<cm>, ZAHL:STAGE:<1-5>, ZAHL:RESULT:<mmAbw>,<0-2>,
//   ZAHL:OVERRUN, ZAHL:STOPPED, ZAHL:BUSY.
#pragma once

#include <Arduino.h>

bool zahlCmd(const String& cmd);
void zahlTick();
void zahlStop();

// Fuer die LED-Prioritaetenkette.
bool zahlActive();
