// EduBot V2 - Programmieren-Spiel (Code-Bausteine, Scratch-artig)
// Die Webseite baut per Drag & Drop ein Programm aus Bloecken und schickt
// es als kompakten String zum Roboter.
//
// Programm-Aufbau: FLACHE Instruktionsliste mit Sprungzielen.
//   Ein Loop belegt zwei Eintraege: den Kopf (OP_REPEAT/OP_UNTIL) und einen
//   OP_END. Der Kopf merkt sich in endIdx, wo sein OP_END steht. Dadurch
//   sind Schleifen beliebig verschachtelbar - im Gegensatz zur alten
//   Struct-mit-Innenarray-Loesung, die nur eine Ebene erlaubte.
#pragma once

#include <Arduino.h>

// Grenzen (kodiertes Programm laeuft ueber BLE -> Webseite begrenzt Laenge)
constexpr uint8_t PROG_MAX_STEPS      = 40;  // flache Liste inkl. aller OP_END
constexpr uint8_t PROG_MAX_DEPTH      = 3;   // Verschachtelungstiefe der Schleifen
constexpr uint8_t PROG_UNTIL_MAX_ITER = 60;  // Sicherheits-Limit fuer Bis-Loops
constexpr uint8_t PROG_MAX_ENC        = 200; // Max. Laenge des kodierten Strings

// Baustein-Op-Codes
enum ProgOp : uint8_t {
  // --- vorhanden ---
  OP_FWD = 0,   // F<n>        vorwaerts n cm
  OP_BACK,      // B<n>        rueckwaerts n cm
  OP_LEFT,      // L<n>        links drehen n Grad
  OP_RIGHT,     // R<n>        rechts drehen n Grad
  OP_WAIT,      // W<n>        warten n Sekunden (W E <c>: bis Sensor)
  OP_REPEAT,    // U<n>(body)  n-mal (U E <c>(body): solange Sensor)
  OP_UNTIL,     // C<c>(body)  body wiederholen bis Farbe c sichtbar
  // --- neu: millimetergenaue Fahrten ---
  OP_MM_FWD,    // M<n>        vorwaerts n mm (exakt, unabhaengig von Microsteps)
  OP_MM_BACK,   // N<n>        rueckwaerts n mm (exakt)
  // --- neu: Distanzsensor mit eingetragenem Wert ---
  OP_DRIVE_DIST,// D<n>        fahre vorwaerts bis Abstand <= n mm
  OP_WAIT_DIST, // X<n>        warte bis Abstand <= n mm (max. 5 s)
  OP_WAIT_FREE, // G<n>        warte bis Abstand >= n mm (max. 5 s)
  // --- "bis" als eigener Baustein (der verstaendliche Weg zu Farbe/Distanz) ---
  OP_DRIVE_UNTIL,// P<c>       fahre vorwaerts BIS Farbe/Sensor c sichtbar ist
  OP_WAIT_UNTIL, // V<c>       warte BIS Farbe/Sensor c sichtbar ist
  // --- neu: Anzeige und Feinsteuerung ---
  OP_LED,       // S<c>        LED auf Slot c setzen (0-3 Farben, 4/5 IR, 6 aus)
  OP_PAUSE,     // Z<n>        Pause n x 100 ms
  OP_STOP,      // Q           Vollstopp
  // --- nur vom Parser erzeugt ---
  OP_END        // Schliesst eine Schleife
};

// Sensor-/Slot-Indizes fuer E/C/S-Slots: 0-3 Farben, 4 = Distanz NAH, 5 = Distanz FREI
constexpr uint8_t PROG_COLOR_ROT   = 0;
constexpr uint8_t PROG_COLOR_GELB  = 1;
constexpr uint8_t PROG_COLOR_GRUEN = 2;
constexpr uint8_t PROG_COLOR_BLAU  = 3;
constexpr uint8_t PROG_SENSOR_IR_HIGH = 4;  // Hindernis erkannt (distBlocked())
constexpr uint8_t PROG_SENSOR_IR_LOW  = 5;  // Weg frei (DIST >= Schwelle / kein Wert)
constexpr uint8_t PROG_SLOT_OFF       = 6;  // nur LED: ausschalten

// Sicherheitsnetze fuer "fahre bis ...": ohne sie faehrt der Roboter bei
// defektem oder blindem Sensor endlos weiter. Beide Grenzen greifen
// unabhaengig voneinander - eine verletzte genuegt zum Halt.
constexpr uint32_t PROG_DRIVE_UNTIL_MAX_MS = 20000UL;   // hoechstens 20 s
constexpr int32_t  PROG_DRIVE_UNTIL_MAX_MM = 1500;      // hoechstens 1,5 m
// Haltezeit fuer eine erkannte Farbe waehrend "fahre bis". Kurz genug, dass
// der Roboter nicht weit ueber die Farblinie hinausfaehrt, lang genug, dass
// ein einzelner Fehlwert beim Fahren nicht stoppt.
constexpr uint32_t PROG_DRIVE_UNTIL_HOLD_MS = 60;

// Ein Eintrag der flachen Programmliste.
struct ProgStep {
  uint8_t op;      // ProgOp
  int16_t val;     // cm / Grad / s / mm / Anzahl; bei W und U auch negativer Sensor-Marker
  int8_t  color;   // Zielfarbe fuer F/B (=-1 wenn keine); bei C die Farbe des Bis-Loops
  uint8_t endIdx;  // nur OP_REPEAT/OP_UNTIL: Index des zugehoerigen OP_END
  uint8_t sibIdx;  // Index innerhalb der Eltern-Liste (Top-Level: 0.., Loop-Body: 0..). 0xFF = OP_END.
};

extern ProgStep progSteps[PROG_MAX_STEPS];
extern uint8_t  progLen;
extern bool     progActive;

// Programm aus kompaktem String parsen (Schleifen beliebig verschachtelt).
//   F10 = 10 cm vorwaerts, R90 = 90 Grad rechts, W2 = 2 s warten
//   M45 = 45 mm vorwaerts, D120 = fahre bis Abstand <= 120 mm
//   P2  = fahre bis GRUEN sichtbar ist,  V2 = warte bis GRUEN sichtbar ist
//   U3(F10,R90) = 3x, C2(F5) = bis GRUEN
//   U2(U3(F5))  = verschachtelt
// true bei Erfolg, false bei Syntaxfehler / leerem Programm.
bool progSetFromString(const char* s);

// Programm ausfuehren (blockierend, wie Pfeil-Knobeln).
// Setzt currentGame = GAME_PROG und sendet PROG:* Statusmeldungen.
void progRun();

// Programm mit Farb-Slalom-Ablauf ausfuehren: currentGame = GAME_SLALOM.
// Target: Zielfarbe (z.B. "ROT"). Stoppt, wenn die Farbe stabil
// (Haltezeit ~100ms) erkannt wird, oder das Programm endet.
// Sendet SLALOM:TARGET:<color> | SLALOM:FINISH:<color> | SLALOM:DONE.
void progRunSlalom(const char* targetColor);

// Abbruch (aus handleCommand / hardStop / Disconnect)
void progStop();

// ---- Tischkanten-Waechter ----
// Der Roboter soll nicht vom Tisch fallen. Der TCS34725 sitzt nach unten;
// ueber der Kante faellt sein Clear-Kanal (Gesamthelligkeit) auf fast 0.
// Ausgewertet wird im Sensor-Task (ColorSensor.cpp, ~alle 150 ms), NICHT im
// Fahrpfad - der 50-us-Puls-Task bleibt davon unberuehrt.
//
// Drei Messungen in Folge unter der Schwelle, dann sofortiger Halt. Drei
// deshalb: eine schwarze Linie auf dem Tisch sieht genauso dunkel aus wie der
// Abgrund, ist aber nur kurz. Dauer statt Einzelwert trennt die beiden Faelle.
void edgeGuardTick(uint16_t rawClear);   // aus dem Sensor-Task
void edgeGuardSet(bool on);               // Web-UI-Schalter
bool edgeGuardActive();
bool edgeGuardTripped();                  // true einmalig, danach wieder false
void edgeGuardReset();                    // Zaehler und Merker loeschen
