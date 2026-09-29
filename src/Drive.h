// EduBot V2 - Drive-Modul
// Frei-Fahren-Modus: WASD/Touch -> linke/rechte Zielgeschwindigkeit.
// Die Motoren fahren immer mit fester Geschwindigkeit (FIXED_DRIVE_SPEED).
// Das Anfahren und Abbremsen beim Tastendruck laufen ueber die Rampe aus
// config.rampMs (Software-Rampe im Motor-Task). Das STOPP selbst ist NICHT
// gerampt: setMotorSpeed(i, 0) ist immer sofort - Not-Aus, losgelassene
// Taste und Befehls-Timeout duessen nicht warten.
// Der Linienfolger faehrt dagegen bewusst OHNE Rampe (setMotorSpeed(..., false)):
// er schaltet seine Suchdrehungen alle 140 ms um, die Rampe wuerde sie
// verschlucken.
#pragma once

#include <Arduino.h>

// Tastenstatus (von "KEYS:<W><A><S><D>" als '0'/'1' gesetzt)
extern bool keyW, keyA, keyS, keyD;
// Letzte gesetzte Zielgeschwindigkeiten (nur Status/Anzeige)
extern float speedL, speedR;
// Zeitstempel des letzten eingehenden Befehls (Basis fuer config.commandTimeoutMs)
extern unsigned long lastCommandTime;

// Tasten loeschen
void driveClearKeys();
// Alles auf Stopp (Keys + Geschwindigkeiten + Motoren)
void driveHardStop();
// Drive-Tick (in loop() aufrufen, ausserhalb von Spielen).
// Ueberfuehrt den Tastenstatus in Zielgeschwindigkeiten und setzt nach
// config.commandTimeoutMs ohne Befehl sicher auf Stopp.
void driveUpdate();

// Diagnose fuer "WASD tut nichts": schreibt den kompletten Zustand der
// Kette KEYS -> driveUpdate -> setMotorSpeed in einen Puffer, damit sich
// jeder Abschnitt einzeln pruefen laesst (per Befehl DRV? abrufbar).
// Reihenfolge im Text: Keys | Zielgeschwindigkeiten | Sperrgrund | Treiber.
void driveDebug(char* out, size_t n);
