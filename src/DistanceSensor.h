// EduBot V2 - Distanzsensor-Modul (VL53L0X, I2C an J3/J4)
// Verdrahtung Edu_V2: SDA=GPIO5, SCL=GPIO6 (gemeinsamer Bus mit TCS34725).
//
// ------------------------------------------------------------------
// WARUM DAS MODUL SO VORSICHTIG INITIALISIERT WIRD
// ------------------------------------------------------------------
// TCS34725 und VL53L0X haben BEIDE die Default-I2C-Adresse 0x29. Der
// Adafruit-Treiber wertet seinen Adressparameter aber zu spaet aus:
//     begin(TARGET):  DataInit @ 0x29
//                     -> setAddress(TARGET)          <- erst HIER wird umgezogen
//                     -> GetDeviceInfo @ TARGET
//                     -> StaticInit @ TARGET
// DataInit ist der Teil, der die NVM-Kalibrierdaten AUSLESET. Laeuft das
// noch auf 0x29, antwortet dort der TCS34725 mit und die Reads kommen
// verfaelscht zurueck. Folge:
//   * ReferenceSpadCount wird ungueltig (real: 0x7FFFFF77, also "alle Bits")
//   * StaticInit startet die optische Ref-SPAD-Kalibrierung und scheitert
//     mit -50 (VL53L0X_ERROR_REF_SPAD_INIT)
//   * die Phasenkalibrierung wird nie ausgefuehrt
//   * danach meldet jede Messung Range-Status 4 (PhaseFail)
//
// LOESUNG (siehe patch_vl53l0x.py): der Patch verschiebt den Adresswechsel
// VOR DataInit. Damit laeuft der komplette Init einschliesslich aller
// NVM-Zugriffe auf 0x30, wo der VL53L0X allein antwortet. Der TCS34725
// bleibt unangetastet auf 0x29. Ein Umstecken oder I2C-Multiplexer ist
// dafuer nicht noetig.
//
// ZWEI WEITERE REGELN:
//  1. setMeasurementTimingBudgetMicroSeconds() darf NICHT aufgerufen werden:
//     die Funktion fuehrt intern perform_phase_calibration() aus und SCHREIBT
//     das Ergebnis ins NVM. Der Standardwert der Library (33 ms) ist zu
//     verwenden.
//  2. Nach fehlgeschlagenem StaticInit darf NICHT gemessen werden: die
//     Skalierungsparameter des Treibers sind dann 0 und die Messfunktionen
//     teilen durch sie (Guru Meditation / IntegerDivideByZero). Deshalb
//     setzen distInvalidate() und die Pflicht von s_ok false jeden Zustand,
//     in dem lox ohne gueltiges StaticInit weiterlaueft.
//
// ZUSAETZLICH: lox ist ein statisches Objekt, und get_info_from_device() liest
// die NVM-Daten nur EIN MAL pro Instanz (ReadDataFromDeviceDone). Ein einmal
// eingefrorener Fehlwert bliebe sonst die ganze Sitzung bestehen. Deshalb wird
// vor jedem begin() eine frische Instanz per Placement-New erzeugt
// (loxResetInstance() - dort steht auch, warum keine Zuweisung erlaubt ist).
//
// ZUSTAENDIG (auf dem Geraet verifiziert):
//   Init erfolgreich ueber 0x30, Continuous-Modus, 46/46 gueltige Messungen,
//   191-198 mm bei 20 cm Ziel, Farbsensor gleichzeitig an 0x29 aktiv.
//   Sollte begin() mit -50 scheitern (NVM beschaedigt), DISTREF senden.
#pragma once

#include <Arduino.h>

// I2C-Adresse nach Umadressierung (TCS34725 bleibt auf 0x29)
#define DIST_I2C_ADDR 0x30

// ------------------------------------------------------------------
// Intervall fuer automatische Web-Meldungen (DIST:<mm>)
constexpr uint32_t DIST_NOTIFY_MS = 500;

// Die Hindernisschwellen liegen NICHT hier, sondern in
// config.obsDistMM / config.obsRelMM (NVS-persistent, per OBSDIST:/OBSREL:)
// - siehe Config.h und distBlocked(). Frueher standen hier zwei Konstanten
// (OBSTACLE_DIST_MM / OBSTACLE_RELEASE_MM), die aber nirgends ausgewertet
// wurden: wer sie aenderte, aenderte nichts.

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

// Ref-SPAD-Kalibrierung gezielt erneut ausfuehren (Befehl DISTREF).
//
// Noetig, wenn begin() an -50 (REF_SPAD_INIT) scheitert: die NVM-Ref-SPAD-
// Daten sind dann unbrauchbar, und genau diese Kalibrierung schreibt sie neu.
// Sie braucht ein geeignetes Ziel - weisse, matte Flaeche auf ~20-30 cm,
// nicht in direkter Sonne - und dauert einige Sekunden. War sie erfolgreich,
// wird der Sensor danach vollstaendig neu initialisiert und ist wieder
// messbereit. Rueckgabe: 0 = erfolgreich, sonst ST-Fehlercode.
//
// ACHTUNG: blockiert sekundenlang und haelt den I2C-Bus. Nicht direkt aus
// dem Kommando-Pfad rufen - dort distRefSpadRequest() benutzen. Grund ist
// nicht der Watchdog (esp_task_wdt_init(10000,...) sind 10000 SEKUNDEN,
// kein 10-Sekunden-Timeout), sondern die responsiveness: waehrend der
// Kalibrierung laufen Motorschritte und Spiel-Logik nicht weiter.
int distRefSpadCalibrate();

// DISTREF asynchron anstossen. Die Kalibrierung dauert mehrere Sekunden;
// direkt aus handleCommand() aufgerufen wuerde den Loop-Task fuer diese
// Zeit vollstaendig blockieren (Motorschritte, Spiel-Logik, BLE-Events).
// Stattdessen startet distRefSpadRequest() einen eigenen Task und kehrt
// sofort zurueck. distRefSpadService() (in loop() aufrufen) meldet das
// Ergebnis per sendStatus, sobald es fertig ist.
// Rueckgabe: false, wenn bereits eine Kalibrierung laeuft.
bool distRefSpadRequest();

// Ergebnis einer per distRefSpadRequest() angestossenen Kalibrierung
// auswerten und per sendStatus melden. Blockiert nicht.
void distRefSpadService();



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

