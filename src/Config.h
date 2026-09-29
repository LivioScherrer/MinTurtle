// EduBot V2 - Config-Modul
// Konfiguration mit NVS-Persistenz und Sanity-Validierung
#pragma once

#include <Arduino.h>
#include <Preferences.h>

struct ColorRef {
  uint8_t r = 0, g = 0, b = 0;
  bool    calibrated = false;
};

// Drehrichtung pro Motor - feste Konstanten, passend zur Verdrahtung.
// Bewusst KEINE NVS-Werte und KEINE Web-UI-Einstellung: ein einmal
// falsch gespeicherter Wert wuerde jeden spaeteren korrigierten Default
// bei jedem Build ueberschreiben. Die Quelle ist die einzige Wahrheit.
//
// Wirkt an JEDER Stelle genau einmal: setMotorSpeed() multipliziert intern
// mit dem Vorzeichen, die prepPositionMove()-Aufrufer ebenfalls. Nicht
// nochmals an anderer Stelle umkehren.
// Vorzeichen der jeweiligen Motorplatte fuer die Fahrtrichtung "vorwaerts".
// Gehoert zu der Verdrahtung in Motor.h: links ist U4/J5 (D1/D0), rechts
// ist U2/J1 (D8/D2). Beide Werte nur an einer Stelle aendern.
constexpr int8_t LEFT_MOTOR_SIGN  = +1;   // Motor links  (U4/J5, D0/D1)
constexpr int8_t RIGHT_MOTOR_SIGN = -1;   // Motor rechts (U2/J1, D2/D8)

// Die beiden MUESSEN unterschiedlich sein. Sind sie gleich, drehen beide
// Raeder gleichsinnig und der Roboter dreht sich auf der Stelle statt zu
// fahren. Das ist kein gueltiger Zustand -> lieber den Build abbrechen
// als ein Roboter, der im Kreis fährt.
static_assert(LEFT_MOTOR_SIGN != RIGHT_MOTOR_SIGN,
              "Beide Motoren mit gleichem Vorzeichen: der Roboter wuerde sich "
              "nur im Kreis drehen. Siehe LEFT_MOTOR_SIGN / RIGHT_MOTOR_SIGN.");
static_assert(LEFT_MOTOR_SIGN == 1 || LEFT_MOTOR_SIGN == -1,
              "LEFT_MOTOR_SIGN muss +1 oder -1 sein.");
static_assert(RIGHT_MOTOR_SIGN == 1 || RIGHT_MOTOR_SIGN == -1,
              "RIGHT_MOTOR_SIGN muss +1 oder -1 sein.");

// =====================================================
// GEOMETRIE / KINEMATIK - einzige Quelle der Wahrheit
// =====================================================
// Grundsatz: GESPEICHERT werden physikalische Groessen (mm, Grad, mm/s),
// NIEMALS Schrittzahlen. Schrittzahlen haengen an den Microsteps - sie werden
// bei jedem Bedarf aus der aktuellen Microstep-Zahl berechnet. Dadurch stimmen
// alle Laengen, Drehungen und das Tempo unabhaengig davon, ob 1, 16 oder 256
// Microsteps eingestellt sind. Wer Motor oder Rad tauscht, aendert HIER; wer
// nur die Microsteps aendert, muss gar nichts tun.
//
// Motor: NEMA8, Schrittwinkel 1,8 Grad -> 200 VOLLschritte pro Umdrehung.
// Das ist die einzige Motor-Konstante im Projekt. Jede Strecke und jede
// Drehung wird daraus abgeleitet - es gibt keine fest verdrahteten
// Soll-Distanzen mehr.
constexpr uint16_t MOTOR_FULL_STEPS_PER_REV = 200;

// ---------- Motorgstrom: 200 mA sind das ECHTE Maximum ----------
// Der NEMA8 8HS11-0204S hat laut Datenblatt 0,2 A Nennstrom. Mehr laesst den
// Motor vibrieren, heizt ihn auf und kostet Schritte - das ist keine
// Tuning-Option, sondern eine Spezifikationsgrenze.
//
// Diese beiden Werte sind die EINZIGE Quelle der Wahrheit. Alles, was den
// Strom setzt, haelt sich daran:
//   * tmcSetCurrent()      -> Befehl TMC:CURRENT: (BLE und Serial-Monitor)
//   * configValidate()     -> NVS-Wert beim Boot, wird NOTFALLS auch
//                             nach unten korrigiert
//   * die Web-Einstellseite (max/min-Attribute + Pruefung)
// Frueher stand hier "100...800" - das waren 4x Nennstrom, und ein einmal so
// gespeicherter Wert blieb in der NVS erhalten.
//
// Warum die Grenze echt ist und nicht nur Anzeige: configureOne() setzt
// I_scale_analog(false), der Strom laeuft also digital ueber IHOLD_IRUN und
// wird von tmcSetCurrent() geschrieben. Nachgerechnet fuer R_sense 0,56 Ohm:
//     Soll 200 mA -> CS 28 -> 198,9 mA tatsaechlich (0,99-fach Nennstrom)
// Die 5-Bit-Quantisierung UEBERSCHREITET den Sollwert dabei nie; die
// Saettigung CS 31 entspraeche 396 mA und ist durch die Grenze ausgeschlossen.
constexpr uint16_t TMC_CURRENT_MA_MIN     = 100;  // darunter haelt der Motor nicht
constexpr uint16_t TMC_CURRENT_MA_MAX     = 200;  // = Nennstrom laut Datenblatt
constexpr uint16_t TMC_CURRENT_MA_DEFAULT = TMC_CURRENT_MA_MAX;
// Rad-Durchmesser 35 mm (am Roboter gemessen). Wird nur noch fuer die
// Umrechnung Schritte <-> mm gebraucht; Drehungen kommen ohne den Radwert aus.
constexpr float WHEEL_DIAMETER_MM = 35.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265358979f * WHEEL_DIAMETER_MM;
// Strecken-Trim aus der Fahrprobe: auf 10 cm fuhr der Roboter 1 mm zu weit.
// Die tatsaechlich gerollte Schrittlaenge ist also ~1 % groesser als die
// Rechnung aus dem gemessenen Radumfang (Messfehler/effektiver Rollradius).
// EINMAL zentral korrigiert - alle mm<->Schritt-Rechnungen haengen daran
// (F/M-Bloecke, Quiz-Einlernung, Tempo). Drehungen sind NICHT betroffen.
// Nachjustieren: DRIVE_TRIM *= tatsaechlich gefahrene mm / Soll-mm,
// z.B. 1,0101 * (99,5 / 100), wenn 10 cm nur 99,5 mm werden.
constexpr float DRIVE_TRIM = 1.0101f;
// Effektiver (gerollter) Radumfang fuer die Schritt-Rechnung.
constexpr float EFFECTIVE_CIRCUMFERENCE_MM = WHEEL_CIRCUMFERENCE_MM * DRIVE_TRIM;
// Weg pro EINEM Vollschritt: 0,5553 mm (effektiv, inkl. Trim).
constexpr float MM_PER_FULL_STEP = EFFECTIVE_CIRCUMFERENCE_MM / (float)MOTOR_FULL_STEPS_PER_REV;

// Spielfeld-Raster: ein Viertel Radumdrehung = 50 Vollschritte = 27,77 mm.
// Das ist bewusst derselbe Wert wie eine 90-Grad-Drehung, denn die ist
// ebenfalls genau MOTOR_FULL_STEPS_PER_REV/4 = 50 Vollschritte. Vorwaerts
// und Drehen sind damit geometrisch gleich - reines Motorraster, kein
// Altmotor-Wert.
constexpr int32_t QUIZ_STEP_FULL_STEPS = MOTOR_FULL_STEPS_PER_REV / 4;   // 50
constexpr float   QUIZ_STEP_MM  = (float)QUIZ_STEP_FULL_STEPS * MM_PER_FULL_STEP;
// QUIZ_TURN_DEG ist NUR die nominelle Referenz (90 Grad) fuer die Skalierung
// der L/R-Bloecke (degToSteps in ProgGame.cpp: Faktor quizTurnDeg/QUIZ_TURN_DEG).
// Sie darf NICHT auf den gemessenen Wert gesetzt werden, sonst wuerden die
// Block-Drehungen nicht mehr mitskalieren.
constexpr float   QUIZ_TURN_DEG = 90.0f;   // = 50 Vollschritte pro Rad
// Werks-Default der Drehungs-Einlernung (NVS quizDeg, GEO:RESET). Am Roboter
// gemessen: 195 Kommando-Grad ergeben 90 Grad real.
constexpr float   TURN_DEG_DEFAULT = 195.0f;
// Wertebereiche fuer die Einlernung (NVS).
constexpr float FORWARD_MM_MIN = 5.0f,  FORWARD_MM_MAX = 150.0f;
constexpr float TURN_DEG_MIN   = 10.0f,  TURN_DEG_MAX   = 360.0f;
constexpr float SPEED_MM_S_MIN = 20.0f,  SPEED_MM_S_MAX = 600.0f;

// ---------- Anfahrzeit (Beschleunigungsrampe) ----------
// Zeit, die der Motor von 0 auf das eingestellte Tempo braucht - und
// symmetrisch wieder herunter. Eine Zahl, eine Bedeutung, an beiden Stellen
// an denen sie wirkt: freies Fahren (WASD, anfahren UND abbremsen beim
// Loslassen) und Fahrten nach Strecke.
// 0 = KEINE Rampe: dann faehrt er ueberall sofort mit voller Geschwindigkeit
// (das Verhalten vor dem Rampen-Umbau - lauter, aber ohne jede Verzoegerung).
//
// NICHT von dieser Zahl betroffen sind die harten Stopps: Not-Aus,
// STOP-Knopf, Befehls-Timeout, TMC:EN:0. Eine Bremse, die hier so lange
// dauert, wuerde den Roboter bei Verbindungsausfall noch sekundenlang
// rollen lassen.
//
// Warum ZEIT und nicht Beschleunigung in mm/s^2: die Zeit ist die einzige
// Angabe, die unabhaengig von Microstep-Zahl und Tempo dasselbe Gefahr
// ergibt. Eine mm/s^2-Angabe muesste bei jedem Wechsel der Microsteps neu
// umgerechnet werden, sonst faehrt derselbe Wert je nach Motor-Einstellung
// schneller oder langsamer - genau der Fehler, den die ganze Geometrie-
// Umstellung in diesem Projekt vermeiden soll.
constexpr uint16_t RAMP_MS_MIN = 0, RAMP_MS_MAX = 2000, RAMP_MS_DEFAULT = 150;

// ---------- Einschalt-Drehung nach dem Boot ----------
// Kurze Drehung auf der Stelle direkt nach dem Start: zeigt "der Roboter
// lebt" und holt die Motoren an ihre exakte Ausgangsposition. Dauer 0
// schaltet sie ab - dann wartet der Roboter auch nicht.
//
// Die Dauer ist der einzige Wert, den die Web-UI anbietet ("Einschalten
// (ms)"): eine Drehzahl in Schritten/s ist auf einer Grundschul-Seite nur
// unlesbar. Die Drehzahl bleibt ein reiner Firmwarewert und ist ueber
// SPIN:<schritte/s>,<ms> am Serial Monitor erreichbar.
constexpr uint16_t SPIN_SPEED_MIN = 0, SPIN_SPEED_MAX = 20000, SPIN_SPEED_DEFAULT = 6000;
constexpr uint16_t SPIN_MS_MIN = 0, SPIN_MS_MAX = 3000, SPIN_MS_DEFAULT = 500;

// NaN-sichere Bereichspruefung. IMMER statt "x < MIN || x > MAX" benutzen
// und mit !inRange(...) auswerten.
//
// Grund: jeder Vergleich mit NaN ist false. Ein "STEPS:NAN" (toFloat() ist
// atof() und parst "nan"/"inf") passierte mit der alten Form daher jede
// Pruefung, wurde in config UND in die NVS geschrieben und kam schliesslich
// bei geoStepsForMM() als (int32_t)NaN an - undefiniertes Verhalten, daraus
// ein Schrittzahl-Wert, dessen labs() nie 0 erreicht, und damit ein Roboter,
// der ewig weiterfaehrt.
//
// Beide Formen sind bei NaN "durchgelassen", aber mit umgekehrtem Ergebnis:
//   alt:  x < lo || x > hi    -> bei NaN false  -> "gueltig"   (BUG)
//   neu:  !(x >= lo && x <=hi)-> bei NaN true   -> "ungueltig" (korrekt)
constexpr bool inRange(float x, float lo, float hi) {
  return (x >= lo) && (x <= hi);
}
// Obergrenze fuer die Schrittrate: der Motor-Puls-Task tickt alle 50us, das
// sind 20 000 Schritte/s. Es wird jeder Tick benutzt - ein Schritt pro Tick ist
// der schnellste saubere Takt, denn der naechste waere nur 25us und damit
// anfällig gegen Jitter der anderen Tasks.
// Folge: die Strecke ist auch bei voller Rate noch richtig, ein spaeter
// Aufwachen kostet einen Schritt (AccelStepper holt nicht nach, aber der
// naechste Puls kommt entsprechend spaeter) - im schlimmsten Fall wird die
// Fahrt einen Tick zu langsam, sie wird nicht ungenau.
// WICHTIG: Schritte/s sind der eigentlichen Limit. Bei 128 Microsteps ergibt
// 20 000 Schritte/s nur ~74 mm/s, bei 16 Microsteps dagegen ~589 mm/s.
// Wer mehr Tempo will, muss Microsteps senken (siehe Motor & TMC).
constexpr float MAX_DRIVE_STEPS_S = 20000.0f;

// Schritte inkl. Microsteps fuer eine Anzahl VOLLschritte. Das ist die
// praezise Form des Fahrens: der Motor zaehlt seine eigenen Vollschritte.
constexpr int32_t geoStepsForFullSteps(int32_t fullSteps, uint16_t microsteps) {
  return fullSteps * (int32_t)microsteps;
}
// Vollschritte <-> mm.
constexpr float fullStepsToMM(int32_t fullSteps) {
  return (float)fullSteps * MM_PER_FULL_STEP;
}
// Schritte fuer eine Strecke in mm (Microsteps einbezogen).
constexpr int32_t geoStepsForMM(float mm, uint16_t microsteps) {
  return (int32_t)((mm < 0.0f ? -mm : mm)
                   * ((float)MOTOR_FULL_STEPS_PER_REV * (float)microsteps / EFFECTIVE_CIRCUMFERENCE_MM)
                   + 0.5f);
}
// Schritte fuer eine Drehung in Grad (pro Rad). Braucht den Radwert NICHT.
constexpr int32_t geoStepsForDeg(float deg, uint16_t microsteps) {
  return (int32_t)((deg < 0.0f ? -deg : deg) / 360.0f
                   * ((float)MOTOR_FULL_STEPS_PER_REV * (float)microsteps)
                   + 0.5f);
}

// Die static_asserts halten die Rechnung fest, damit ein Tausch von Motor
// oder Rad nicht still zu falschen Distanzen fuehrt. Geprueft wird an
// ZWEI Microstep-Zahlen - wenn 16 und 128 dasselbe Ergebnis skalieren,
// ist die Strecke nachweislich unabhaengig von der Microstep-Einstellung.
static_assert(MM_PER_FULL_STEP > 0.555f && MM_PER_FULL_STEP < 0.556f,
              "Ein Vollschritt muss 0,5553 mm ergeben (Rad 35mm / 200 Schritte, inkl. DRIVE_TRIM).");
static_assert(geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 16) == 800
           && geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 128) == 6400,
              "50 Vollschritte muessen 16 Microsteps -> 800 und 128 -> 6400 ergeben.");
static_assert(geoStepsForDeg(QUIZ_TURN_DEG, 16) == 800
           && geoStepsForDeg(QUIZ_TURN_DEG, 128) == 6400,
              "90 Grad muessen 16 Microsteps -> 800 und 128 -> 6400 ergeben.");
static_assert(geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 16)
           == geoStepsForDeg(QUIZ_TURN_DEG, 16),
              "Ein Rasterfeld (Vorwaerts) muss genau einer 90-Grad-Drehung entsprechen.");

// Kennzahl aus Motor + Rad (Microsteps sind ABSICHTLICH nicht dabei: sie aendern
// nur die Schrittzahl, nicht die Strecke). Weicht sie von "geoSig" im NVS ab,
// stammt die Einlernung noch von anderer Hardware -> dann wird sie verworfen.
// GEO_DEF_VERSION bitte erhoehen, wenn sich die Motor-/Rad-Definition aendert:
// das invalidiert die alte Einlernung beim naechsten Boot automatisch.
constexpr uint32_t GEO_DEF_VERSION = 4;
constexpr uint32_t geoSignature() {
  return ((uint32_t)GEO_DEF_VERSION << 28)
       ^ ((uint32_t)MOTOR_FULL_STEPS_PER_REV * 1000000u)
       ^ ((uint32_t)WHEEL_DIAMETER_MM * 1000u);
}

struct Config {
  int8_t leftSign     = LEFT_MOTOR_SIGN;
  int8_t rightSign    = RIGHT_MOTOR_SIGN;

  uint32_t commandTimeoutMs = 500;
  // TMC2209-Laufzeitwerte (NVS: tmcCur/tmcHold/tmcMS/tmcSpread, per TMC:*-Befehl setzbar)
  // 200 mA sind der Nennstrom des NEMA8 laut Datenblatt - HARTE Obergrenze,
  // definiert als TMC_CURRENT_MA_MAX weiter oben in dieser Datei. Wer den Wert
  // "optimiert", laeuft den Motor ueber seine Spezifikation: er vibriert,
  // heizt sich auf und verliert Schritte. Der Bereich ist bewusst eng.
  uint16_t tmcCurrent = TMC_CURRENT_MA_DEFAULT;  // mA RMS, 100...200
  uint8_t  tmcHold = 16;        // 0...31 (halber Haltestrom: kuehl, reicht flach)
  uint16_t tmcMicrosteps = 16;  // 1,2,4,8,16,32,64,128,256
  bool     tmcSpread = false;   // false=StealthChop (leise), true=SpreadCycle
  // Einlernung in PHYSIKALISCHEN Einheiten - dadurch von der Microstep-Zahl
  // unabhaengig (NVS: quizMM/quizDeg, per STEPS:<mm>/ANG:<grad>).
  float quizForwardMM  = QUIZ_STEP_MM;    // 5..150 mm, Werkseinstellung 27,8 (ein Rasterfeld)
  float quizTurnDeg    = TURN_DEG_DEFAULT; // 10..360 Grad, Werkseinstellung 195 (gemessen)
  // Fahrgeschwindigkeit in mm/s (NVS: driveSpeed, per SPEED:<mm/s>).
  // Die Schrittrate wird daraus und aus der Microstep-Zahl berechnet.
  float driveSpeedMM   = 150.0f;          // 20..300 mm/s
  // Beschleunigungsrampe fuer Positionsfahrten (NVS: rampMs, per RAMP:<ms>).
  // 0 = keine Rampe. Bereich RAMP_MS_MIN..RAMP_MS_MAX.
  uint16_t rampMs      = RAMP_MS_DEFAULT;
  // Anlaufdrehung nach dem Boot (NVS: spinSpeed/spinMs, per SPIN:<s>,<ms>).
  // 0 bei beiden Werten = aus. Bereiche SPIN_SPEED_MIN/MAX, SPIN_MS_MIN/MAX.
  uint16_t spinSpeed   = SPIN_SPEED_DEFAULT;
  uint16_t spinMs      = SPIN_MS_DEFAULT;
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
  // Mindest-Saettigung fuer warme Farbtoene (Boden-Differenzierung):
  // unterhalb gilt die Messung als zu blass (Holzboden wirkt gelblich)
  // -> UNBEKANNT. Per SAT:-Befehl einstellbar.
  float minColorSaturation = 0.40f;
  // VL53L0X-Hindernisschwellen mit Hysterese (mm, per OBSDIST:/OBSREL: einstellbar):
  // blockiert ab < obsDistMM, wieder frei erst ab > obsRelMM.
  //
  // Diese Schwellen gelten fuer den LINIENFOLGER und die PROGRAMM-BAUSTEINE
  // "fahre bis ...", nicht fuer das Freifahren. Das Freifahren fragt den
  // Sensor bewusst NICHT ab: der VL53L0X sitzt niedrig und sieht beim Fahren
  // haeufig Tischkante, Wand oder Boden, wodurch der Roboter gar nicht erst
  // loskomme. In Drive.cpp steht die Begruendung ausfuehrlich.
  int16_t obsDistMM = 100;
  int16_t obsRelMM  = 130;
  // VL53L0X-Korrektur: richtig = obsScale * roh - obsOffsetMM.
  // Einzelpunkt (DISTCAL:<wahr>, Datenblatt bei 100 mm): obsScale = 1,
  // obsOffsetMM = roh - wahr. 3-Punkt (DISTCAL3, 5/10/20 cm): Gerade aus
  // drei Roh-Mittelwerten (Least-Squares), obsScale = Steigung.
  int16_t obsOffsetMM = 0;
  float   obsScale = 1.0f;   // 0.5..2.0 (NVS: obsScale)
  ColorRef calRot;
  ColorRef calGelb;
  ColorRef calGruen;
  ColorRef calBlau;
};

extern Config config;
extern Preferences prefs;

// ---- Umrechnung physikalisch <-> Schritte (immer mit der aktuellen Microstep-Zahl) ----
// Aufrufer brauchen NIE eine Schrittzahl zu pflegen: sie fragen einfach hier.
int32_t configForwardSteps();   // Schritte fuer config.quizForwardMM
int32_t configTurnSteps();      // Schritte fuer config.quizTurnDeg
float   configStepsPerMM();     // Schritte pro mm (Microsteps eingerechnet)
float   configStepsPerS();      // Schrittrate fuer config.driveSpeedMM (Motor.h)

// Lade aus NVS, wende Defaults an, validiere
void configLoad();
// Sanity-Check: wenn ausserhalb Bereich -> Default setzen + speichern
void configValidate();
// Einlernung (mm/Grad) auf die Werkseinstellungen zuruecksetzen und speichern
// (nur noetig, wenn Motor oder Rad getauscht wurden).
void configResetGeometry();
// Direkt speichern (z.B. nach STEPS:-Befehl)
void configSaveField(const char* key, float value);
void configSaveInt(const char* key, int32_t value);
void configSaveUShort(const char* key, uint16_t value);
void configSaveUChar(const char* key, uint8_t value);
void configSaveBool(const char* key, bool value);
