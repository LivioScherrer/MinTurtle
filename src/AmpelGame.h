// EduBot V2 - A5 "Ampel" (Gruenes Licht, rotes Licht) - Einzelmodus
//
// Der Roboter rollt langsam; zu einer Zufallszeit (2-6 s) zeigt die Ampel
// ROT (UI + LED). Das Kind bremst mit HALT (NICHT Not-Aus-STOP). Die
// Probe-Routine misst den Nachrollweg: < 50 mm = Stern, < 150 mm = gut.
//
// Schummler-Runde (1 von 6): Die Ampel bleibt GRUEN, der Roboter stoppt nach
// 2-6 s von selbst und meldet AMPEL:CHEAT. Wer binnen 3 s den Schummler-Knopf
// drueckt (AMPEL:CHEATPRESS), gewinnt die Runde (AMPEL:CHEATWIN).
//
// Stufen: je 3 erfolgreiche Runden steigt die Stufe (max. 5, gemeldet als
// AMPEL:STAGE:<1-5> zu Rundenbeginn); das Rot-Fenster schrumpft pro Stufe um
// 1 s (2-6 s, 2-5 s, ... bis 2 s fix). Die Wertung bleibt immer gleich.
// STOP setzt zurueck auf Stufe 1.
//
// Kommandos (UI -> Roboter): AMPEL:START, AMPEL:NEXT, AMPEL:CHEATPRESS,
//   AMPEL:STOP; dazu HALT (weicher Stopp, GameCommon).
// Meldungen (Roboter -> UI): AMPEL:STAGE:<1-5>, AMPEL:ARMED, AMPEL:RED,
//   AMPEL:RESULT:<mm>,<0-2>,
//   AMPEL:EARLY, AMPEL:MISSED, AMPEL:CHEAT, AMPEL:CHEATWIN, AMPEL:CHEATLATE,
//   AMPEL:STOPPED, AMPEL:BUSY.
#pragma once

#include <Arduino.h>

// Kommando-Handler (Dispatch in Edu_V2.cpp). true = Befehl gehoert AMPEL:.
bool ampelCmd(const String& cmd);

// Nicht-blockierender Tick (aus loop()).
void ampelTick();

// Abbruch (hardStop / BLE-Disconnect / AMPEL:STOP).
void ampelStop();

// Fuer die LED-Prioritaetenkette in ledPatternUpdate().
bool ampelActive();
bool ampelRed();
bool ampelCheat();
