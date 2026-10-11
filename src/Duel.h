// EduBot V2 - Duell (1:1-Punkte-Austausch fuer die Spiel-Modi)
//
// Ein leichter, generischer Wettkampf-Kanal auf der bestehenden 1:1-ESP-NOW-
// Pairing-Infrastruktur (RACE:SCAN / RACE:PAIR, siehe cmdRace):
//   * Beide Roboter spielen ihren Modus wie im Einzelspieler.
//   * Jeder Modus meldet seine Rundenpunkte per duelAddPoints() (A5/B1/B8/B9:
//     0-2 Punkte je Runde, B10: 2 fuer Bestzeit, 1 fuer Ankommen).
//   * Der Punktestand geht per ESP-NOW an den gepaarten Gegner (SLALOM_CMD_DUEL).
//   * Wer zuerst DUEL_TARGET Punkte hat, gewinnt. Die UI zeigt "Du x : y Gegner".
//
// Ohne aktives Duell ist duelAddPoints() ein No-Op - Einzelspieler bleibt
// unveraendert. Kein Kollisions-Spiel: die Roboter spielen ihre Runden
// unabhaengig voneinander, nur die Punkte werden verglichen.
//
// Kommandos: DUEL:START, DUEL:STOP, DUEL:?.
// Meldungen: DUEL:STARTED:10, DUEL:ME:<n>, DUEL:OPP:<n>, DUEL:WON, DUEL:LOST,
//   DUEL:NOPAIR, DUEL:TIMEOUT, DUEL:STOPPED.
#pragma once

#include <Arduino.h>
#include "EspNow.h"

// Kommando-Handler (Dispatch in Edu_V2.cpp). true = Befehl gehoert DUEL:.
bool duelCmd(const String& cmd);

// Tick (aus loop()): Gegner-Timeout bemerken.
void duelTick();

// Punkte fuer die eigene Runde melden (No-Op ohne aktives Duell).
// negative Punkte (Giftpilz) erlaubt, Gesamtstand bleibt >= 0.
void duelAddPoints(int points);

// Eingehendes Duell-Paket (aus espNowProcessEvents, bereits gepaart-geprueft
// wird hier noch einmal gegen pairedPeer).
void duelOnPacket(const RacePacket* pkt);

// Abbruch (hardStop / BLE-Disconnect).
void duelStop();

bool duelActive();
