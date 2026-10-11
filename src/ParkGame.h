// EduBot V2 - B9 "Einparken"
//
// Der Roboter rollt langsam auf ein Hindernis zu. Das Kind bremst mit HALT
// (weicher Spiel-Stopp) so nah wie moeglich - ohne anzustossen. Der
// Distanzsensor misst den Restabstand: einziger Modus, der den ToF-Sensor
// als Spielgeraet nutzt (sonst nur Sicherheit).
// Faellt der Abstand unter die Tast-Schwelle, stoppt der Roboter von selbst
// (TOUCH). Ohne Hindernis in Reichweite meldet er EMPTY.
//
// Stufen: je 2 erfolgreiche Runden steigt die Stufe (max. 3); die
// Perfekt-/Gut-Baender werden enger (100/80/60 bzw. 200/150/120 mm).
// STOP setzt zurueck auf Stufe 1.
//
// Kommandos: PARK:START, PARK:STOP, HALT.
// Meldungen: PARK:GO, PARK:STAGE:<1-3>, PARK:RESULT:<mm>,<0-2>,
//   PARK:TOUCH, PARK:EMPTY, PARK:STOPPED, PARK:BUSY.
#pragma once

#include <Arduino.h>

bool parkCmd(const String& cmd);
void parkTick();
void parkStop();

// Fuer die LED-Prioritaetenkette.
bool parkActive();
