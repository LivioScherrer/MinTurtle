// EduBot V2 - Distanzsensor-Modul (VL53L0X, I2C an J3/J4)
// Verdrahtung Edu_V2: SDA=GPIO5, SCL=GPIO6 (gemeinsamer Bus mit TCS34725).
//
// ------------------------------------------------------------------
// WARUM DAS MODUL SO VORSICHTIG INITIALISIERT (auf Hardware erklaert)
// ------------------------------------------------------------------
// TCS34725 und VL53L0X haben BEIDE die Default-I2C-Adresse 0x29. Der
// Adafruit-Treiber initialisiert den VL53L0X so:
//     begin(TARGET):  DataInit @ 0x29 (im Treiber hart kodiert)
//                     -> setAddress(TARGET)
//                     -> GetDeviceInfo @ TARGET
//                     -> StaticInit @ TARGET   <- liest die NVM-Daten
// Der NVM-Zugriff in StaticInit (ReferenceSpadCount, Good-SPAD-Map) laeuft also
// auf TARGET. Antwortet dort der TCS34725 ebenfalls, sind die I2C-Reads
// verfaelscht (zwei Geraete auf einer Adresse). Folge:
//   * ReferenceSpadCount wird ungueltig (real: 0x7FFFFF77, also "alle Bits")
//   * StaticInit startet die optische Ref-SPAD-Kalibrierung und scheitert
//     mit -50 (VL53L0X_ERROR_REF_SPAD_INIT)
//   * die Phasenkalibrierung wird nie ausgefuehrt
//   * danach meldet jede Messung Range-Status 4 (PhaseFail)
//
// ZWEI REGELN DARAUS (beide muessen eingehalten werden):
//  1. Waehrend der Initialisierung darf 0x29 dem VL53L0X allein gehoeren.
//     distSetup() prueft das und nimmt je nach Lage den 0x29- oder den
//     0x30-Pfad (siehe distInitAtTarget).
//  2. setMeasurementTimingBudgetMicroSeconds() darf NICHT aufgerufen werden:
//     die Funktion fuehrt intern perform_phase_calibration() aus und SCHREIBT
//     das Ergebnis ins NVM. Mit verdraechtigen Ausgangsdaten wird damit
//     dauerhaft eine kaputte Phasenkalibrierung abgelegt. Der Standardwert
//     der Library (33 ms) ist zu verwenden.
//
// ZUSAETZLICH: lox ist ein statisches Objekt, und get_info_from_device() liest
// die NVM-Daten nur EIN MAL pro Instanz (ReadDataFromDeviceDone). Ein einmal
// eingefrorener Fehlwert bliebe sonst die ganze Sitzung bestehen. Deshalb wird
// vor jedem begin() eine frische Instanz per Placement-New erzeugt
// (loxResetInstance() - dort steht auch, warum keine Zuweisung erlaubt ist).
//
// ZUSTAENDIG (auf dem Geraet verifiziert):
//   Farbsensor ausgesteckt -> 0x29 exklusiv -> Init erfolgreich,
//   Continuous-Modus, 46/46 gueltige Messungen, 191-198 mm bei 20 cm Ziel.
//   Farbsensor angesteckt -> 0x29 doppelt belegt -> nur Notpfad, ~10 % Ausbeute
//   (siehe README: I2C-Multiplexer oder getrennte Busse noetig).
#pragma once

#include <Arduino.h>

// I2C-Adresse nach Umadressierung (TCS34725 bleibt auf 0x29)
#define DIST_I2C_ADDR 0x30

// ------------------------------------------------------------------
// Intervall fuer automatische Web-Meldungen (DIST:<mm>)
constexpr uint32_t DIST_NOTIFY_MS = 500;

// Defaults (aktuelle Werte stehen in config.obsDistMM/obsRelMM, NVS-persistent).
constexpr int16_t OBSTACLE_DIST_MM = 100;
constexpr int16_t OBSTACLE_RELEASE_MM = 130;

// ACHTUNG: KEIN setMeasurementTimingBudgetMicroSeconds() aufrufen.
// Diese Adafruit/ST-Funktion fuehrt intern VL53L0X_perform_phase_calibration()
// aus und SCHREIBT das Ergebnis in das NVM des Sensors. Wird sie aufgerufen,
// waehrend der I2C-Lesepfad durch den Adresskonflikt mit dem TCS34725
// verfaelschte Werte liefert, wird dauerhaft eine kaputte Phasenkalibrierung
// abgelegt - danach meldet jede Messung Range-Status 4 (PhaseFail).
// Die frueher funktionierende Firmware rief die Funktion gar nicht auf und
// nutzte den Library-Standard (33 ms). Siehe backup_vor-revert/DistanceSensor.cpp.

// --- Single-Shot-Fallback (siehe DistanceSensor.cpp readRawMM) ---
// readRange() ist blockierend (es wartet auf das Messende). Damit die
// Main-Loop/BLE nicht zu lange blockiert, wird hoechstens alle
// DIST_SINGLE_PERIOD_MS gemessen.
constexpr uint32_t DIST_SINGLE_PERIOD_MS = 200;
// Wartezeit fuer den I2C-Lock pro Messversuch beim blockierenden readRange().
constexpr uint32_t DIST_SINGLE_LOCK_MS   = 120;
// Anzahl Messversuche, bis ein gueltiger Wert (Range-Status 0) akzeptiert wird.
constexpr uint8_t  DIST_SINGLE_RETRIES   = 3;

// true wenn Hindernis erkannt (Hysterese aus config.obsDistMM/obsRelMM:
// < Stopp blockiert, > Freigabe gibt frei). Kein Messwert / ausser
// Reichweite (-1) = frei. Non-blocking.
bool distBlocked();

// Initialisierung (setzt distOk). Muss VOR colorSetup() laufen.
void distSetup();

// true wenn Sensor bereit
bool distOk();

// Einmalige Messung in mm (offset-korrigiert); <0 bei Fehler/ausser Reichweite
int16_t distMeasureMM();

// Offset-Kalibrierung nach Datenblatt-Verfahren: weisses Target exakt bei
// trueMM (50..500, empfohlen 100) ruhig halten, mittelt ~10 Messungen und
// speichert offset = roh - wahr (NVS). false bei zu wenig/gueltigen Samples.
bool distCalibrateOffset(int16_t trueMM);

// Aktueller Offset / zuruecksetzen
int16_t distOffsetMM();
void distClearOffset();

// Letzter gueltiger Messwert (Cache aus distTick), -1 = kein Wert
int16_t distLastMM();

// In loop() aufrufen: meldet alle DIST_NOTIFY_MS "DIST:<mm>" per sendStatus (nur bei BLE).
// Braucht Funktionspointer-ähnlichen Callback nicht: nutzt sendStatus aus BLEManager.
void distTick();

// Diagnose (nur lesend, kein Verhaltenseinfluss):
// distDiag() schreibt eine kompakte Zeile nach buf (fuer BLE),
// distDiagPrint() gibt ausfuehrlich alles auf Serial aus (I2C-Scan, Modus,
// letzter Wert, Range-Status-Text, Fehlerzaehler + Hinweise).
void distDiag(char* buf, size_t bufLen);
void distDiagPrint();

// Fehlersuche: jeden Init-Schritt mit Library-Debug nachvollziehen
// (veraendert I2C-Zustand - nur zur Diagnose verwenden).
void distDebugInit();

// Bus-Zustand VOR der Distanz-Initialisierung (Boot-Zeitpunkt) anzeigen.
void distBootScan();

// Automatik an/aus (Default an)
void distSetAuto(bool on);
bool distAuto();

