// EduBot V2 - Praezisions-Pruefroutine (Probe)
//
// Misst, wie weit der Roboter seit einem definierten Moment gefahren ist -
// Quelle sind die Schrittzaehler beider Raeder (motorStepPositions(), dieselben
// Werte, die [DRV] und DRV? melden). Kein neuer Sensor, keine Blockade.
//
// Genutzt von den Praezisions-Spielen (A5 Ampel: Nachrollweg nach Rot,
// B1 Zahlentreffer: Abweichung von der Zielmarkierung). Die Toleranzen
// (10 mm / 30 mm) stehen NUR hier - damit sind alle Modi gleich fair.
#pragma once

#include <Arduino.h>

// Referenz setzen: ab jetzt wird gemessen (Moment des Signals).
void probeBegin();

// Zurueckgelegte Strecke seit probeBegin() in mm (vorwaerts positiv).
int32_t probeMM();

// Punktzahl fuer eine Abweichung: 2 = |mm| < 10 (perfekt), 1 = < 30 (gut),
// sonst 0. Nimmt den BETRAG selbst - der Aufrufer darf mit Vorzeichen rufen.
uint8_t probeScoreMM(int32_t mm);

// Toleranzschwellen (fuer Anzeige/Tests, Werte an EINER Stelle).
constexpr int32_t PROBE_PERFECT_MM = 10;
constexpr int32_t PROBE_GOOD_MM    = 30;
