// EduBot V2 - Gemeinsame Spiel-Helfer (ModeBase)
//
// Baut die Querschnitts-Mechanismen fuer die neuen Spiel-Modi an EINER Stelle:
//   * HALT  - weicher Spiel-Stopp: Motoren aus, Spiel bleibt aktiv.
//             Im Gegensatz zu STOP (Not-Aus, hardStop()) schaltet HALT weder
//             die Treiber ab noch beendet es das laufende Spiel. Der Modus
//             konsumiert das Latch in seinem Tick und wertet aus.
//   * Manual-Freigabe - im Spiel darf ein Kind wieder per KEYS fahren
//             (fuer A4/C3; der Mechanismus existiert ab W0, derzeit ungenutzt).
//   * Kanten-Suspend - der Tischkanten-Waechter wird fuer die Dauer eines
//             Modus stumm geschaltet (Session-Flag, NIE NVS - sonst bliebe er
//             nach einem Spiel dauerhaft aus). Gedacht fuer Modi, die schwarze
//             Flaechen als Spiel-Element lesen (derzeit ungenutzt).
//   * gameModeEnd() - Standard-Ende: Motoren aus, currentGame = GAME_NONE,
//             kurzes Token an die UI (64-Zeichen-Grenze von sendStatus).
#pragma once

#include <Arduino.h>

// ---- HALT-Latch (weicher Stopp) ----
void gameHaltSet();          // aus cmdHalt() in Edu_V2.cpp
bool gameHaltRequested();    // true, solange das Latch nicht konsumiert wurde
void gameHaltClear();        // Modus hat ausgewertet

// ---- Manual-Freigabe (W2; Mechanismus ab W0) ----
// true: KEYS: werden trotz laufendem Spiel an driveUpdate() durchgereicht.
void gameSetManualDrive(bool on);
bool gameAllowsManual();

// ---- Kanten-Waechter aussetzen (Session-Flag) ----
// Wird in edgeGuardTick() (ProgGame.cpp) gelesen. gameSuspendEdgeGuard(true)
// raeumt zusaetzlich einen evtl. stehenden Kanten-Halt weg
// (edgeGuardReset + motorReleaseEmergency).
void gameSuspendEdgeGuard(bool on);
bool gameEdgeGuardSuspended();

// ---- Tempo-Override (RAM only, KEIN NVS) ----
// setMotorSpeeds() legt den Betrag IMMER auf FIXED_DRIVE_SPEED - fuer die
// langsamen Praezisions-Spiele (A5: 60 mm/s, B1: 100 mm/s) wird das globale
// Spiel-Tempo deshalb waehrend des Modus umgestellt (gleiches Muster wie der
// T-Block in ProgGame). Immer paarweise: Override beim Start, Restore beim
// Modus-Ende/Abbruch. Ohne NVS - ein Reboot stellt den konfigurierten Wert her.
void gameSpeedOverride(float mmS);
void gameSpeedRestore();

// ---- Standard-Modus-Ende ----
// Motoren aus, currentGame = GAME_NONE, sendStatus(token).
void gameModeEnd(const char* token);
