// EduBot V2 - B8 "Geheim-Strecke"
//
// Das Kind waehlt eine Schaetzung (cm). Der Roboter faehrt danach eine
// GEHEIME Strecke (pro Spiel neu gewuerfelt) und meldet die Abweichung:
// zu kurz oder zu weit geraten? Schaetzen + Kalibrieren statt Reagieren -
// eine andere Faehigkeit als Ampel (Reaktion) und Zahlentreffer (Bremsen).
// Geknackt (<= Toleranz) gibt es eine neue Geheim-Strecke.
//
// Stufen: je geknackter Strecke waechst der Bereich und die Toleranz wird
// enger (max. 3, gemeldet als GEHEIM:STAGE:<1-3>). STOP setzt zurueck.
//
// Kommandos: GEHEIM:START (neue Strecke), GEHEIM:START:<cm> (10..100,
//   Ratesrunde), GEHEIM:STOP.
// Meldungen: GEHEIM:READY, GEHEIM:STAGE:<1-3>, GEHEIM:GO, GEHEIM:RESULT:
//   <mmAbw>,<0-2> (+ = weiter als geraten, - = kuerzer), GEHEIM:STOPPED,
//   GEHEIM:BUSY.
#pragma once

#include <Arduino.h>

bool geheimCmd(const String& cmd);
void geheimTick();
void geheimStop();

// Fuer die LED-Prioritaetenkette.
bool geheimActive();
