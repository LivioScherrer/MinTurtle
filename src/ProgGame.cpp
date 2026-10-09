// EduBot V1.3 - Programmieren-Spiel Implementierung
// Block-Programm (Scratch-artig, mit Parametern) parsen + blockierend ausfuehren.
#include "ProgGame.h"
#include "GameEngine.h"
#include "Config.h"
#include "BLEManager.h"
#include "Motor.h"
#include "Drive.h"
#include "ColorSensor.h"
#include "EspNow.h"
#include "DistanceSensor.h"
#include "LedStrip.h"
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern bool slalomFinished;  // Flag in Edu_V1.3.cpp

ProgStep progSteps[PROG_MAX_STEPS];
uint8_t  progLen    = 0;
bool     progActive = false;

constexpr uint32_t PROG_STEP_GAP_MS = 120;  // Pause zwischen Top-Level-Bloecken
constexpr uint32_t PROG_LOOP_GAP_MS = 100;  // Pause zwischen Loop-Durchlaeufen
constexpr uint32_t SLALOM_TARGET_HOLD_MS = 250;  // Zielfarbe stabil sehen, bevor der Slalom stoppt

// Baustein-Op-Codes
static const char* const PROG_COLOR_NAMES[6] = { "ROT", "GELB", "GRUEN", "BLAU", "NAH", "FREI" };

// Distanz-Hindernis: distBlocked() (VL53L0X, Schwellen in config.obsDistMM/obsRelMM).
// -> distBlocked() = true bedeutet: Hindernis erkannt (wie NAH im Web-UI)
static inline bool irBlocked() {
  return distBlocked();
}

// Laufender Programmschritt (fuer die Statusanzeige PROG:STEP / PROG:SUB).
static volatile uint8_t progCurStep = 0;
static volatile uint8_t progCurSub  = 0;
static volatile uint8_t progCurLoop = 0;

// =====================================================
// Farb-Erkennung mit Haltezeit
// (Sensor-Task auf Core 0 aktualisiert den Farbnamen laufend)
// =====================================================
static uint32_t colMatchStart = 0;
static int8_t   colMatchIdx   = -1;

// ---- Tischkanten-Erkennung ueber den Clear-Kanal des Farbsensors ----
// Der TCS34725 sitzt nach unten. Ueber der Tischkante kommt kein Licht zurueck,
// der Clear-Kanal (rawSensorC, Gesamthelligkeit ohne Farbfilter) faellt auf fast 0.
//
// EDGE_RAW_C_MAX: Schwelle in Rohwerten. Der Sensor liefert auf dem Tisch je nach
// Beleuchtung dreistellige Werte; ueber der Kante bleiben ein bis zwei zweistellige
// Werte uebrig. Die Schwelle liegt bewusst tief, damit eine dunkle Stelle auf dem
// Tisch (schwarze Linie, Schatten) NICHT als Kante gilt - zaehlt wird die Dauer.
// GEMESSEN (COM9, 2026-10, Sensor in ca. X mm Hoehe):
//   sicher auf dem Tisch, 20 cm von der Kante .... Clear = 2092 (stabil)
//   Roboter an der Tischkante ................... Clear =  85 .. 124
//
// Das ist Faktor 20 zwischen beiden Zustaenden. EDGE_RAW_C_MAX liegt mit 500
// mittig im Logarithmus: 4x ueber dem hoechsten Kantenwert, 4x unter dem
// Tischwert. Nach oben bleibt damit genug Luft fuer Schatten und dunkle
// Tischkanten, nach unten genug Abstand zur Streuung an der Kante.
//
// WICHTIG - Grenzen, die man kennen muss:
//  - Die Werte gelten fuer diesen Sensor in dieser Hoehe auf diesem Tisch.
//    Auf dunklem Teppich oder bei starker Abschattung liegt der Tischwert
//    niedriger. Im Zweifel EDGE? lesen und die Schwelle nachziehen.
//  - Eine schwarze Linie auf dem Tisch liest sich vermutlich aehnlich dunkel
//    wie die Kante. Deshalb EDGE_CONFIRM_N: es zaehlt die Dauer, nicht den
//    Einzelwert.
// Alle Werte GEMESSEN (COM9, 2026-10, 38-mm-Roboter in Betriebshoehe):
//
//   Tischkante, Abgrund ............  85 ..  124
//   schwarzes Papier .............. 489
//   Tisch, nichts darunter ........ 849 / 823 / 3928
//   rotes Plaettchen .............. 12349
//   gruenes Plaettchen ............ 12365
//   blaues Plaettchen ............. ~16000
//   gelbes Plaettchen ............. 16129
//   weisses Papier ................ 16159
//
// Die Luecke zwischen echter Kante (124) und schwarzem Papier (489) betraegt
// 365 Rohwerte. EDGE_RAW_C_MAX liegt mit 300 mittig darin: schwarzes Papier und
// alles Bunte bleiben sicher darueber, echter Abgrund sicher darunter.
//
// Die erste Fassung stand auf 500 und war damit ZU HOCH - sie lag ueber dem
// schwarzen Papier. Gemessen wurde damals 124 gegen 2092, aber die 2092 stammten
// von einem hellen Blatt; der Tisch selbst liefert nur 849. Der Waechter hat
// daraufhin bei jedem dunklen Plaettchen angeschlagen und den Roboter
// mitten in der Strecke gestoppt - das war die Ursache der zu kurzen Strecken.
//
// HYSTERESE: Nach einem Halt muss die Helligkeit erst wieder deutliche
// EDGE_RAW_C_RELEASE uebersteigen, bevor ueberhaupt gefahren wird. Ohne das
// wuerde der Waechter direkt nach dem Anfahren wieder anschlagen, weil die
// Kante noch im Sichtfeld liegt.
constexpr uint16_t EDGE_RAW_C_MAX   = 300;   // darunter = Kante
constexpr uint16_t EDGE_RAW_C_RELEASE = 600; // darueber = wieder frei
constexpr uint8_t  EDGE_CONFIRM_N   = 2;    // 2 Messungen in Folge (~300 ms)

static volatile uint8_t s_edgeCount = 0;    // aufeinanderfolgende Treffer
static volatile bool    s_edgeLatched = false;   // Kante erkannt, gemeldet
static volatile bool    s_edgeGuardOn = false;   // Schalter aus der NVS
static volatile bool    s_edgeBlocked = false;   // Hysterese: gesperrt bis zum Boden

// Einmalige Kantenmeldung: der Motor-Stopp darf nicht bei jedem Sensor-Tick
// erneut in den Log schreiben, aber die UEBERGANG in den Zustand "Kante"
// ist gemeldenswert - sie ist der Moment, in dem der Roboter stehenbleibt.
bool edgeGuardTripped() {
  if (!s_edgeLatched) return false;
  s_edgeLatched = false;
  return true;
}
bool edgeGuardActive()   { return s_edgeGuardOn; }
void edgeGuardReset()    { s_edgeCount = 0; s_edgeLatched = false; s_edgeBlocked = false; }
void edgeGuardSet(bool on) {
  s_edgeGuardOn = on;
  edgeGuardReset();
  if (on) Serial.println("[KANTE] Waechter aktiv - stoppt an der Tischkante");
  else    Serial.println("[KANTE] Waechter aus");
}

static void colorPollReset() { colMatchStart = 0; colMatchIdx = -1; }

// true wenn die Sensor-Bedingung (Farbe 0-3 oder Distanz 4/5) erfuellt ist.
// Farbe: seit >= 40ms stabil sichtbar (kurze Haltezeit, damit selbst kurze
// Fahrten im Bis-Modus genug Zeit haben zum Erkennen).
// Distanz: distBlocked() (letzter VL53L0X-Wert, keine Haltezeit noetig).
static bool sensorPoll(uint8_t idx) {
  if (idx >= PROG_SENSOR_IR_HIGH) return (irBlocked() == (idx == PROG_SENSOR_IR_HIGH));
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  if (strcasecmp(name, PROG_COLOR_NAMES[idx]) != 0) { colorPollReset(); return false; }
  if (colMatchIdx != (int8_t)idx) { colMatchIdx = (int8_t)idx; colMatchStart = millis(); return false; }
  return (millis() - colMatchStart) >= 40;
}

// Sofort-Check ohne Haltezeit: 1x Lesung reicht.
// Genutzt fuer Sensor-Bedingung waehrend der Bewegung (Sensor muss MOTOREN
// stoppen BEVOR der Roboter darueber hinausfaehrt -> Haltezeit sei 0.
// Distanz: letzter VL53L0X-Wert.
static bool sensorHitImmediate(uint8_t idx) {
  if (idx >= PROG_SENSOR_IR_HIGH) return (irBlocked() == (idx == PROG_SENSOR_IR_HIGH));
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  return strcasecmp(name, PROG_COLOR_NAMES[idx]) == 0;
}

// Tischkante: der Sensor sitzt nach unten und sieht ueber der Tischkante
// nichts mehr. Der Clear-Kanal (rawSensorC) ist die Gesamthelligkeit ohne
// Farbfilter - ueber der Kante faellt er auf nahezu 0, das ist unabhaengig von
// der Farbe des Tisches. Ein zweiter Sensor nach unten waere das robustere
// Prinzip, vorhanden ist aber nur der TCS34725.
//
// Bewusst KEINE Farbauswertung: schwarze Linien auf hellem Tisch sehen genauso
// dunkel aus wie der Abgrund. Mit Clear-Kanal und Helligkeitsschwelle ist eine
// schwarze Linie aber nur kurz dunkel, waehrend die Kante dauerhaft dunkel
// bleibt - deshalb zaehlen hier aufeinanderfolgende Messungen, nicht eine.
static bool tableEdgeDetected() {
  if (!colorSensorOk) return false;      // Sensor tot: nicht als Kante werten
  if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  const uint16_t clear = rawSensorC;
  xSemaphoreGive(colorSem);
  return clear < EDGE_RAW_C_MAX;
}

// Vom Sensor-Task alle ~150 ms aufgerufen (ColorSensor.cpp). Zaehlt
// aufeinanderfolgende Messungen unterhalb der Schwelle und loest genau einmal
// aus. Wird hier - nicht im Fahrpfad - ausgewertet, damit der 50-us-Puls-Task
// nichts auf dem Weg hat.
//
// s_edgeBlocked ist die Hysterese: einmal ausgeloest, bleibt der Roboter
// gesperrt, bis die Helligkeit wieder ueber EDGE_RAW_C_RELEASE liegt. Ohne
// das wuerde der Waechter nach dem Anfahren sofort wieder anschlagen, weil die
// Kante noch im Sichtfeld des nachlaufenden Sensors ist - der Roboter stuende
// dann still und koennte nie wieder losfahren.
void edgeGuardTick(uint16_t rawClear) {
  if (!s_edgeGuardOn) { s_edgeCount = 0; s_edgeBlocked = false; return; }

  if (s_edgeBlocked) {
    if (rawClear > EDGE_RAW_C_RELEASE) {
      s_edgeBlocked = false;
      s_edgeCount = 0;
      Serial.printf("[KANTE] Boden wieder da (Clear=%u > %u) - freigegeben\n",
                    rawClear, EDGE_RAW_C_RELEASE);
    }
    return;
  }

  if (rawClear < EDGE_RAW_C_MAX) {
    if (s_edgeCount < 255) s_edgeCount++;
    if (s_edgeCount >= EDGE_CONFIRM_N) {
      stopMotors();                       // sofort und ohne Rueckfrage
      if (!s_edgeLatched) {
        Serial.printf("[KANTE] Tischkante nach %u Messungen (Clear=%u < %u) - "
                      "Motoren gestoppt\n", s_edgeCount, rawClear, EDGE_RAW_C_MAX);
      }
      s_edgeLatched = true;
      s_edgeBlocked = true;               // erst bei Clear > RELEASE wieder frei
    }
  } else {
    s_edgeCount = 0;
  }
}

// Slalom-Zielfarb-Check waehrend der Fahrt (debounced + Flanken-Logik).
// - Debounce: Farbe muss `holdMs` lang stabil sichtbar sein (spannt mehrere
//   Sensor-Ticks a ~150ms), damit ein einzelner Bewegungs-Ausreisser nicht stoppt.
// - Flanke: war die Farbe schon beim Start sichtbar (armed=false), wird sie
//   ignoriert, bis sie einmal verschwunden war (armed=true) und wieder auftaucht.
static uint32_t slalomHitStart = 0;
static int8_t  slalomHitIdx   = -1;
static bool    slalomHitArmed = true;

static void slalomHitReset(bool armed) {
  slalomHitStart = 0;
  slalomHitIdx   = -1;
  slalomHitArmed = armed;
}

static bool slalomHitPoll(uint8_t idx, uint32_t holdMs) {
  if (idx >= PROG_SENSOR_IR_HIGH) return false;
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  if (strcasecmp(name, PROG_COLOR_NAMES[idx]) != 0) {
    slalomHitArmed = true;   // Farbe weg -> naechstes Auftauchen ist eine Flanke
    slalomHitStart = 0;
    slalomHitIdx   = -1;
    return false;
  }
  if (!slalomHitArmed) return false;  // war von Anfang an da -> ignorieren
  if (slalomHitIdx != (int8_t)idx) { slalomHitIdx = (int8_t)idx; slalomHitStart = millis(); return false; }
  return (millis() - slalomHitStart) >= holdMs;
}

// Hilfsfunktion: laeuft gerade ein Spiel, das die Stepper nutzt?
// GAME_PROG (Code-Bausteine solo) und GAME_SLALOM (Slalom-Multiplayer)
// muessen die Motoren kontrollieren koennen.
static inline bool gameAllowsMotorRun() {
  return currentGame == GAME_PROG || currentGame == GAME_SLALOM;
}

// =====================================================
// Umrechnung Benutzer-Einheiten -> Stepper-Schritte
// Immer aus der Geometrie (config.quizForwardMM / quizTurnDeg sind in mm
// bzw. Grad eingelernt) - dadurch stimmen die Distanzen bei jeder Microstep-
// Einstellung. Die Einlernung wirkt als Korrektur-Faktor fuer alle Werte.
// =====================================================
static int32_t cmToSteps(int16_t cm) {
  // EXAKT Zentimeter. Kein Korrekturfaktor.
  //
  // "Vorwaerts 10 cm" muss 10 cm fahren - Punkt. Ein Korrekturfaktor
  // (config.quizForwardMM / QUIZ_STEP_MM) ist hier bewusst NICHT
  // angewendet: sobald per STEPS:<mm> eingelernt wurde, fuhr der Baustein
  // dadurch ein Vielfaches der angegebenen Strecke. Bei einem eingelernten
  // Wert von 30 mm ergibt F10 dann 127 mm statt 100 mm. Fuer ein Kind ist
  // ein Baustein, der etwas anderes faehrt als er sagt, das schlimmste
  // denkbare Verhalten.
  //
  // Der eingelernte Wert wirkt weiterhin dort, wo er hingehort: beim Quiz
  // und beim Rennen ueber configForwardSteps() (siehe Config.cpp). Die
  // Einlernung ist damit KEIN Geheimnis-Faktor fuer die Spiele, sondern die
  // Einstellgroesse fuer den Quiz-Rasterweg.
  //
  // Die Schrittzahl ist ohnehin exakt, weil Rad-Durchmesser und
  // Vollschritte pro Umdrehung feste Konstanten sind (Config.h) - Microsteps
  // aendern nur die Aufloesung, nie die Strecke. Der mm-Baustein (OP_MM_FWD)
  // rechnet genauso direkt, F10 und M100 liefern denselben Weg.
  const float mm = (float)cm * 10.0f;
  return geoStepsForMM(mm, config.tmcMicrosteps);
}
static int32_t degToSteps(int16_t deg) {
  const float d = (float)deg * (config.quizTurnDeg / QUIZ_TURN_DEG);
  return geoStepsForDeg(d, config.tmcMicrosteps);
}

// =====================================================
// Bewegung mit Abbruch + optionalem Farb-Check waehrend der Fahrt
// sL/sR: logische Rad-Schritte (vorwaerts positiv), Vorzeichen-Verkabelung
// wird ueber config.leftSign/rightSign korrigiert.
// colorCheck: Sensor-Slot (Farbe 0-3), bei dem die Fahrt stoppt; -1 = keiner.
// Kein globaler ROT-Stop: ROT haelt nur an, wenn das Programm es verlangt
// ("bis ROT" / "fahre bis ROT" -> colorCheck). Ohne roten Baustein ist ROT
// eine normale Farbe auf dem Boden und stoppt nichts.
// Rueckgabe: true wenn die Ziel-Farbe die Fahrt beendet hat.
// =====================================================
static bool progMove(int32_t sL, int32_t sR, float speed, int8_t colorCheck) {
  (void)speed;   // Feste Geschwindigkeit - der Motor-Task faehrt immer FIXED_DRIVE_SPEED.
  // EIN Aufruf fuer beide Seiten: der Blockprogramm-Befehl ist die Stelle, an
// der ein Zeitversatz zwischen den Raedern am auffaelligsten waere - die
// Schrittzahlen beider Seiten stimmen zwar, aber bei unterschiedlichem Start
// wirkt das Drehmoment kurzzeitig nur einseitig und der Roboter steht
// schief. Genau das war vorher moeglich, weil zwischen den beiden Aufrufen der
// 50-us-Puls-Task den Mutex bekommen konnte.
prepPositionMoves(sL * (long)config.leftSign, sR * (long)config.rightSign);
  // Slalom: merken, ob wir schon AUF der Zielfarbe starten. Dann erst nach
  // einem Weg-Gang wieder triggern (Flanken-Logik, siehe slalomHitPoll).
  if (colorCheck >= 0 && colorCheck < (int8_t)PROG_SENSOR_IR_HIGH) {
    char nm[16];
    getCurrentColorName(nm, sizeof(nm));
    slalomHitReset(strcasecmp(nm, PROG_COLOR_NAMES[(uint8_t)colorCheck]) != 0);
  } else {
    slalomHitReset(true);
  }
  uint32_t lastPoll = 0;
  bool colorHit = false;
  while (motorStillRunning()) {   // Pulse erzeugt der Motor-Task auf Core 0
    esp_task_wdt_reset();
    if (!gameAllowsMotorRun()) { stopMotors(); break; }
    if (millis() - lastPoll >= 20) {
      lastPoll = millis();
      // Optionaler Sensor-Check waehrend der Fahrt.
      // Slalom: debounced + Flanken-Logik (nur "neu" auftauchende Zielfarbe),
      // damit ein schon beim Start sichtbarer Wert (Startflaeche / LED-Licht)
      // nicht sofort stoppt.
      // Solo (OP_UNTIL): Sofort-Hit (1x Lesung) - stoppen BEVOR drueber hinweg.
      if (colorCheck >= 0) {
        bool hit = (currentGame == GAME_SLALOM)
                     ? slalomHitPoll((uint8_t)colorCheck, SLALOM_TARGET_HOLD_MS)
                     : sensorHitImmediate((uint8_t)colorCheck);
        if (hit) {
          Serial.printf("[PROG] sensorHit %s -> STOP\n", PROG_COLOR_NAMES[(uint8_t)colorCheck]);
          colorHit = true;
          stopMotors();
          break;
        }
      }
    }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return colorHit;
}

// =====================================================
// "Fahre bis ..." - eine Routine fuer alle drei Faelle
// =====================================================
//
// D (bis Abstand), P (bis Farbe) und die alte Form "F..E.." (bis Farbe mit
// fester Suchstrecke) laufen alle hier durch. Das ist Absicht: vorher gab es
// zwei voellig verschiedene Implementierungen, und die eine davon fuhr in
// 10-mm-Etappen mit jeweils eigener Rampe. Bei 150 mm/s und 150 ms Anfahrzeit
// kostet jede Etappe mehr Zeit im Anfahren als im Fahren - der Roboter kam
// hardly vom Fleck, und der Abstandssensor bekam kaum eine brauchbare Messung.
//
// Jetzt: eine einzige sanfte Rampe, danach durchgehende Fahrt und alle
// ~10 ms eine Pruefung. Das ist fuers Reagieren schnell genug und laesst den
// Roboter wirklich fahren.
//
// slot   >= 0 : Farbe (0..3) oder Hindernis (4) / Weg frei (5)
// targetMM >  0 : Stopp, sobald der gemessene Abstand <= targetMM ist
// maxMM       : Strecken-Sicherheitsnetz (0 = PROG_DRIVE_UNTIL_MAX_MM)
//
// Rueckgabe true = Bedingung hat den Lauf beendet, false = Sicherheitsnetz.
static bool progDriveUntil(int8_t slot, int16_t targetMM, int32_t maxMM) {
  // Ohne Bedingung waere das ein "fahr einfach los bis zum Sicherheitsnetz".
  // Bewusst zurueckweisen statt still weiterzufahren - ein Aufruferfehler
  // darf nicht wie ein geplanter Block aussehen.
  if (slot < 0 && targetMM <= 0) {
    Serial.println("[PROG] fahre-bis ohne Bedingung - nicht ausgefuehrt");
    return false;
  }
  if (maxMM <= 0 || maxMM > PROG_DRIVE_UNTIL_MAX_MM) maxMM = PROG_DRIVE_UNTIL_MAX_MM;

  // Flanken-Logik fuer Farben: steht der Roboter schon AUF der Zielfarbe,
  // darf er nicht sofort stoppen - sonst waere "fahre bis Rot" auf einem roten
  // Feld ein Block ohne Wirkung. Erst wenn die Farbe einmal verschwunden war,
  // zaehlt das naechste Auftaucht als Ziel. Genau dieselbe Regel wie im
  // Slalom (slalomHitPoll), deshalb wird deren Zustand hier benutzt.
  if (slot >= 0 && slot < (int8_t)PROG_SENSOR_IR_HIGH) {
    char nm[16];
    getCurrentColorName(nm, sizeof(nm));
    slalomHitReset(strcasecmp(nm, PROG_COLOR_NAMES[(uint8_t)slot]) != 0);
  } else {
    slalomHitReset(true);
  }

  const float stepsPerMM = configStepsPerMM();
  float drivenMM = 0.0f;
  uint32_t lastMs = millis();
  const uint32_t t0 = lastMs;
  bool hit = false;
  const char* reason = "Sicherheitsnetz";

  char was[24];
  if (targetMM > 0) snprintf(was, sizeof(was), "Abstand <= %d mm", targetMM);
  else              snprintf(was, sizeof(was), "%s", PROG_COLOR_NAMES[(uint8_t)slot]);
  Serial.printf("[PROG] fahre bis %s (Sicherheitsnetz: %.1f m / %lu s)\n",
                was, (double)((float)maxMM / 1000.0f), (unsigned long)PROG_DRIVE_UNTIL_MAX_MS);

  setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);   // beide Seiten, eine Rampe
  for (;;) {
    if (!gameAllowsMotorRun()) break;
    const uint32_t now = millis();
    uint32_t dt = now - lastMs;
    lastMs = now;
    // Gefahrene Strecke aus dem BEFEHL abrechnen (nicht messen): fuer das
    // Sicherheitsnetz reicht das, und es ist unabhaengig von Radschlupf.
    // dt wird auf 500 ms begrenzt statt uebersprungen: ein ausgesetzter
    // Durchlauf darf NICHT als "gefahren 0 mm" gelten, sonst waere das
    // Sicherheitsnetz genau dann am schwaechsten, wenn der Roboter ohnehin
    // schon stoert.
    if (dt > 500UL) dt = 500UL;
    if (stepsPerMM > 0.0f) {
      drivenMM += FIXED_DRIVE_SPEED * (float)dt / 1000.0f / stepsPerMM;
    }

    bool done = false;
    if (targetMM > 0) {
      const int16_t d = distMeasureMM();
      if (d >= 0 && d <= targetMM) { done = true; reason = "Abstand erreicht"; }
    }
    if (!done && slot >= 0) {
      // Farbe: Haltezeit gegen Einzel-Fehlwerte. Hindernis/Weg-frei: sofort.
      const bool h = (slot < (int8_t)PROG_SENSOR_IR_HIGH)
                       ? slalomHitPoll((uint8_t)slot, PROG_DRIVE_UNTIL_HOLD_MS)
                       : sensorHitImmediate((uint8_t)slot);
      if (h) { done = true; reason = PROG_COLOR_NAMES[(uint8_t)slot]; }
    }
    if (done) {
      hit = true;
      break;
    }
    if (drivenMM >= (float)maxMM) { reason = "Suchstrecke verbraucht"; break; }
    if (now - t0 >= PROG_DRIVE_UNTIL_MAX_MS) { reason = "Zeitlimit"; break; }

    bleProcessEvents();
    espNowProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  stopMotors();
  Serial.printf("[PROG] fahre-bis beendet (%s): %.0f mm, Restabstand %d mm\n",
                reason, (double)drivenMM, (int)distLastMM());
  return hit;
}

// "Warte bis ..." - der stehende Zwilling. slot >= 0, max. 30 s.
static bool progWaitUntil(int8_t slot) {
  colorPollReset();
  const uint32_t t0 = millis();
  while (millis() - t0 < 30000UL) {
    if (!gameAllowsMotorRun()) return false;
    if (slot >= 0 && sensorPoll((uint8_t)slot)) return true;
    bleProcessEvents();
    espNowProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

// Beim Start einer Strecken-Fahrt ins Log schreiben, WELCHE Strecke in mm
// tatsaechlich befohlen wird.
//
// Grund: F/B nehmen ZENTIMETER, M/N nehmen MILLIMETER - dieselbe Zahl,
// zehnfache Strecke. Genau daran ist es beim Nachmessen gescheitert: wer im
// Block "10" eintippt, bekommt je nach Baustein 100 mm oder 10 mm, ohne
// dass man es der Fahrt ansieht. Mit dieser Zeile steht die Wahrheit im Log.
static void progLogDist(const char* op, int16_t val, float mm, int32_t steps) {
  Serial.printf("[PROG] %s%ld -> %.1f mm (%.2f cm) = %ld Schritte (%u Microsteps)\n",
                op, (long)val, (double)mm, (double)(mm / 10.0), (long)steps,
                (unsigned)config.tmcMicrosteps);
}

// Einfachen Baustein ausfuehren (kein Loop). Rueckgabe: true wenn Farbe ausgeloest.
static bool progExecSimple(uint8_t op, int16_t val, int8_t colorCheck) {
  int32_t steps;
// Feste Geschwindigkeit - keine Einstellung irgendwo.
  switch (op) {
    case OP_FWD:
      steps = cmToSteps(val);
      progLogDist("F", val, (float)val * 10.0f, steps);
      return progMove(steps, steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_BACK:
      steps = cmToSteps(val);
      progLogDist("B", val, (float)val * 10.0f, steps);
      return progMove(-steps, -steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_DRIVE_UNTIL:
      // "fahre bis Farbe/Sensor". val > 0 = Suchstrecke in mm (kommt aus der
      // alten Schreibweise F50E2, im Parser bereits hierher umgewandelt),
      // val == 0 = nur das Sicherheitsnetz.
      return progDriveUntil(colorCheck, -1,
                            val > 0 ? (int32_t)val : PROG_DRIVE_UNTIL_MAX_MM);
    case OP_WAIT_UNTIL:
      return progWaitUntil(colorCheck);
    case OP_MM_FWD:
      // Millimetergenau: der Wert wird direkt in Schritte gerechnet und ist
      // dadurch unabhaengig von der Microstep-Einstellung.
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      progLogDist("M", val, (float)val, steps);
      return progMove(steps, steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_MM_BACK:
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      progLogDist("N", val, (float)val, steps);
      return progMove(-steps, -steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_LEFT:
      steps = degToSteps(val);
      progMove(-steps, steps, FIXED_DRIVE_SPEED, -1);
      return false;
    case OP_RIGHT:
      steps = degToSteps(val);
      progMove(steps, -steps, FIXED_DRIVE_SPEED, -1);
      return false;
    case OP_DRIVE_DIST:
      // "Fahre, bis der Abstand <= val ist." Dieselbe Routine wie "bis
      // Farbe": eine Rampe, dann durchgehend fahren und alle ~10 ms pruefen.
      // Bewusst KEIN zusaetzlicher Halt auf der Hindernis-Schwelle
      // (distBlocked): die steht woanders und wuerde den Baustein vorzeitig
      // beenden, obwohl der eingetragene Wert noch nicht erreicht ist - der
      // Roboter wuerde anhalten, ohne zu wissen warum. Das Sicherheitsnetz
      // (1,5 m / 20 s) faengt den Fall "Sensor blind" ab.
      return progDriveUntil(-1, val, PROG_DRIVE_UNTIL_MAX_MM);
    case OP_WAIT_DIST: {
      // "Warte, bis der Abstand <= val ist" (hoechstens 5 s).
      const uint32_t t0 = millis();
      while (millis() - t0 < 5000UL) {
        if (!gameAllowsMotorRun()) return false;
        const int16_t d = distMeasureMM();
        if (d >= 0 && d <= val) return false;
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(30));
        esp_task_wdt_reset();
      }
      return false;
    }
    case OP_WAIT_FREE: {
      // "Warte, bis der Abstand >= val ist" - also bis der Weg frei ist.
      const uint32_t t0 = millis();
      while (millis() - t0 < 5000UL) {
        if (!gameAllowsMotorRun()) return false;
        const int16_t d = distMeasureMM();
        if (d >= val || d < 0) return false;        // d<0 = keine Messung -> nicht blockiert
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(30));
        esp_task_wdt_reset();
      }
      return false;
    }
    case OP_LED: {
      if (val == PROG_SLOT_OFF) ledStripClear();
      else ledStripShowTarget(PROG_COLOR_NAMES[val]);
      ledStripTick();   // sofort zeichnen (der Lauf blockiert die Main-Loop)
      bleProcessEvents();
      espNowProcessEvents();
      vTaskDelay(pdMS_TO_TICKS(60));
      return false;
    }
    case OP_PAUSE: {
      // Feine Wartezeit: val x 100 ms (W wartet ganze Sekunden).
      const uint32_t ms = (uint32_t)val * 100UL;
      const uint32_t t0 = millis();
      while (millis() - t0 < ms) {
        if (!gameAllowsMotorRun()) break;
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(30));
        esp_task_wdt_reset();
      }
      return false;
    }
    case OP_STOP: {
      stopMotors(); speedL = 0; speedR = 0;
      vTaskDelay(pdMS_TO_TICKS(150));
      return false;
    }
    case OP_WAIT: {
      if (val < 0) {
        // Bedingtes Warten: bis Sensor == target (|val|-1), Farbe oder Distanz (4/5)
        uint8_t target = (uint8_t)(-(int)val - 1);
        colorPollReset();
        uint32_t start = millis();
        while (millis() - start < 60000) {  // max 60s warten
          if (!gameAllowsMotorRun()) return false;
          if (sensorPoll(target)) return false;  // keine colorHit-Semantik hier
          bleProcessEvents();
          espNowProcessEvents();
          vTaskDelay(pdMS_TO_TICKS(30));
        }
        return false;
      }
      uint32_t ms = (uint32_t)val * 1000;
      uint32_t start = millis();
      while (millis() - start < ms) {
        if (!gameAllowsMotorRun()) break;
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      return false;
    }
    default:
      return false;
  }
}

// Prueft ob der Sensor-Wert-Slot erfuellt ist (1) oder nicht (0).
// Wird fuer OP_REPEAT im Sensor-Modus genutzt. Distanz: 4=NAH/blockiert, 5=FREI.
static bool sensorMatches(uint8_t idx) {
  if (idx >= PROG_SENSOR_IR_HIGH) return (irBlocked() == (idx == PROG_SENSOR_IR_HIGH));
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  return strcasecmp(name, PROG_COLOR_NAMES[idx]) == 0;
}

// =====================================================
// Parser
// =====================================================
static bool parseUInt(const char*& p, long& out) {
  if (*p < '0' || *p > '9') return false;
  long v = 0;
  while (*p >= '0' && *p <= '9') {
    v = v * 10 + (*p - '0');
    p++;
    if (v > 100000) return false;
  }
  out = v;
  return true;
}

// Marker-Konvention: bei OP_WAIT/OP_REPEAT bedeutet val < 0 einen
// Sensor-Slot. Der Slot steht in st.color, der Betrag |val|-1 ist der Index.
static int16_t clampVal(uint8_t op, long v) {
  long lo = 0, hi = 0;
  switch (op) {
    case OP_FWD: case OP_BACK:      lo = 1;   hi = 200;   break; // cm
    case OP_LEFT: case OP_RIGHT:    lo = 5;   hi = 360;   break; // Grad
    case OP_WAIT:                   lo = 1;   hi = 10;    break; // Sekunden
    case OP_REPEAT:                 lo = 1;   hi = 20;    break; // Anzahl
    case OP_MM_FWD: case OP_MM_BACK: lo = 1;  hi = 2000;  break; // mm
    case OP_DRIVE_DIST:             lo = 20;  hi = 1500;  break; // mm Schwellwert
    case OP_WAIT_DIST:              lo = 10;  hi = 1500;  break; // mm
    case OP_WAIT_FREE:              lo = 10;  hi = 1500;  break; // mm
    case OP_PAUSE:                  lo = 1;   hi = 30;    break; // x 100 ms
    case OP_LED:                    lo = 0;   hi = 6;     break; // Slot
    default: break;
  }
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int16_t)v;
}

static bool opFromChar(char c, uint8_t& op) {
  switch (c) {
    case 'F': op = OP_FWD;        return true;
    case 'B': op = OP_BACK;       return true;
    case 'L': op = OP_LEFT;       return true;
    case 'R': op = OP_RIGHT;      return true;
    case 'W': op = OP_WAIT;       return true;
    case 'M': op = OP_MM_FWD;     return true;
    case 'N': op = OP_MM_BACK;    return true;
    case 'D': op = OP_DRIVE_DIST; return true;
    case 'X': op = OP_WAIT_DIST;  return true;
    case 'G': op = OP_WAIT_FREE;  return true;
    case 'P': op = OP_DRIVE_UNTIL; return true;
    case 'V': op = OP_WAIT_UNTIL;  return true;
    case 'S': op = OP_LED;        return true;
    case 'Z': op = OP_PAUSE;      return true;
    case 'Q': op = OP_STOP;       return true;
    default: return false;
  }
}

// Liest ein optionales E<slot> und liefert true, wenn eins da war.
// slotOut bekommt den Slot (0..5).
static bool parseESlot(const char*& p, int8_t& slotOut) {
  if (*p != 'E') return false;
  p++;
  if (*p < '0' || *p > '5') return false;
  slotOut = (int8_t)(*p - '0');
  p++;
  return true;
}

// Parst eine Folge von Bloecken.
//   topLevel = true : bis Stringende
//   topLevel = false: bis ')' (p wird dahinter positioniert)
// Schreibt in progSteps[pc..] und laesst pc hinter dem letzten Eintrag stehen.
static bool progParseRange(const char*& p, uint8_t& pc, uint8_t depth, bool topLevel) {
  while (*p != '\0') {
    if (!topLevel && *p == ')') { p++; return pc > 0 ? true : false; }
    char c = *p;
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c == ',' || c == ' ' || c == ';') { p++; continue; }
    p++;

    ProgStep st;
    memset(&st, 0, sizeof(st));
    st.color = -1;
    st.endIdx = 0;

    // ---------- Schleifen: U...(...) und C...(...) ----------
    if (c == 'U' || c == 'C') {
      if (depth >= PROG_MAX_DEPTH) { Serial.println("[PROG] Verschachtelung zu tief"); return false; }
      if (pc + 1 >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
      int8_t slot = -1;
      if (c == 'U') {
        st.op = OP_REPEAT;
        if (parseESlot(p, slot)) {
          st.color = slot;
          st.val = -(int16_t)(slot + 1);   // negativer Marker: solange Sensor
        } else {
          long v;
          if (!parseUInt(p, v)) return false;
          st.val = clampVal(OP_REPEAT, v);
        }
      } else {
        long v;
        if (!parseUInt(p, v)) return false;
        if (v < 0 || v > 5) return false;
        st.op = OP_UNTIL;
        st.color = (int8_t)v;
      }
      if (*p != '(') return false;
      p++;
      const uint8_t headIdx = pc;
      progSteps[pc++] = st;
      if (!progParseRange(p, pc, (uint8_t)(depth + 1), false)) return false;
      if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
      progSteps[headIdx].endIdx = pc;            // Zielmarke merken
      ProgStep e;                                  // OP_END anhaengen
      memset(&e, 0, sizeof(e));
      e.op = OP_END;
      e.color = -1;
      progSteps[pc++] = e;
      continue;
    }

    // ---------- einfache Bloecke ----------
    uint8_t op;
    if (!opFromChar(c, op)) { Serial.printf("[PROG] unbekannter Baustein '%c'\n", c); return false; }
    st.op = op;

    // P/V: der Wert ist der SENSOR-SLOT (0-5), keine Zahl zum Fahren.
    // Das ist der ganze Unterschied zu den anderen Bloecken und deshalb hier
    // VOR der allgemeinen Zahlenbehandlung - sonst wuerde P2 als "2 cm fahren"
    // gelesen und der Baustein waere genau das, was er nicht soll.
    if (op == OP_DRIVE_UNTIL || op == OP_WAIT_UNTIL) {
      if (*p < '0' || *p > '5') { Serial.printf("[PROG] %c braucht eine Farbe/Sensor 0-5\n", c); return false; }
      st.color = (int8_t)(*p - '0');
      st.val = 0;
      p++;
      if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
      progSteps[pc++] = st;
      continue;
    }

    int8_t slot = -1;
    const bool hasSlot = parseESlot(p, slot);

// F mit E-Slot: die alte Schreibweise "FE<slot>" (nur aus Programmen, die
    // vor dem P-Baustein gespeichert wurden). Wird HIER gleich auf OP_DRIVE_UNTIL
    // // umgewandelt, statt es in zwei Interpretationen mitzuschleppen:
    // "F10 mit Zielfarbe aus einer Bis-Schleife" und "bis Farbe mit 50 cm
    // Suchstrecke" sind im Executor sonst nicht mehr zu unterscheiden - beide
    // kommen als (F, val, colorCheck) an.
    if (hasSlot && op == OP_FWD) {
      st.op      = OP_DRIVE_UNTIL;
      st.color   = slot;
      st.val     = 500;                  // 50 cm Suchstrecke, in mm
    } else if (hasSlot && op == OP_BACK) {
      // Rueckwaerts "bis Farbe" ergibt keinen Sinn (der Sensor schaut nach
      // vorn) und wurde nie in der Palette angeboten. Der Bestand bleibt
      // unveraendert, damit alte Programme nicht anders laufen.
      st.color = slot;
      st.val = 50;                       // max. Suchstrecke in cm
    } else if (hasSlot && op == OP_WAIT) {
      st.val = -(int16_t)(slot + 1);     // warte bis Sensor == slot
    } else if (hasSlot) {
      return false;                      // E nur bei F/B/W/U erlaubt
    } else if (op == OP_STOP) {
      st.val = 0;                        // Q braucht keinen Wert
    } else {
      long v;
      if (!parseUInt(p, v)) return false;
      st.val = clampVal(op, v);
    }
    if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
    progSteps[pc++] = st;
  }
  // Top-Level: Ende erreicht. Bei Unterebene muss jetzt ')' stehen.
  if (!topLevel) { Serial.println("[PROG] Klammer fehlt"); return false; }
  return true;
}

bool progSetFromString(const char* s) {
  if (s == nullptr) return false;
  const int len = (int)strlen(s);
  if (len <= 0 || len > PROG_MAX_ENC) {
    Serial.printf("[PROG] Laenge %d unzulässig (max %d)\n", len, PROG_MAX_ENC);
    return false;
  }
  Serial.printf("[PROG] Parse '%s'\n", s);
  memset(progSteps, 0, sizeof(progSteps));
  uint8_t pc = 0;
  const char* p = s;
  if (!progParseRange(p, pc, 0, true) || pc == 0) { progLen = 0; return false; }
  progLen = pc;
  Serial.printf("[PROG] %u Instruktionen\n", progLen);
  return true;
}


// =====================================================
// Ausfuehrung: rekursiver Bereichs-Laeufer
// =====================================================
// Slalom-Helfer stehen weiter unten - der Bereichs-Laeufer braucht sie fuer
// die Zielpruefung nach jedem Baustein.
static bool slalomTargetStable(uint8_t targetIdx);
static bool slalomFinish(uint8_t targetIdx);

// Kontext fuer einen Programmlauf. Wird pro Schleife kopiert, damit sich
// "bis Farbe" auf die inneren Ebenen nicht verliert.
struct ProgCtx {
  bool    slalom      = false;   // Slalom-Modus
  uint8_t slalomTarget = 0;      // Zielfarbe im Slalom
  int8_t  untilColor  = -1;      // Farbe, bei der ein Bis-Loop endet (-1 = keiner)
  int8_t  defaultColor = -1;     // Farbe fuer F/B ohne eigenes Ziel
  uint8_t headIndex   = 0;       // Index des umgebenden Loop-Kopfs (fuer SUB-Meldung)
};

// Fuehrt progSteps[from..to) aus (to ist exklusiv).
// Rueckgabe: true, wenn ctx.untilColor gesehen wurde (beendet den Bis-Loop).
static bool progExecRange(uint8_t from, uint8_t to, ProgCtx ctx) {
  bool seen = false;
  for (uint8_t i = from; i < to && gameAllowsMotorRun() && !seen; i++) {
    ProgStep& s = progSteps[i];
    if (s.op == OP_END) continue;                 // gehoert zum umgebenden Loop
    esp_task_wdt_reset();

    // ---------- Schleife ----------
    if (s.op == OP_REPEAT || s.op == OP_UNTIL) {
      const uint8_t end = s.endIdx;
      if (end <= i || end > progLen) {           // Sicherheit gegen kaputte Sprungmarken
        Serial.printf("[PROG] kaputte Sprungmarke bei %u (end=%u)\n", i, end);
        break;
      }
      ProgCtx inner = ctx;
      inner.headIndex = i;
      if (s.op == OP_UNTIL) {
        inner.untilColor = s.color;
        inner.defaultColor = s.color;
        colorPollReset();
        Serial.printf("[PROG] Bis %s, %u..%u\n", PROG_COLOR_NAMES[s.color], i + 1, end);
      }

      if (s.op == OP_REPEAT && s.val < 0) {
        // "solange Sensor == Slot"
        const uint8_t target = (uint8_t)(-(int)s.val - 1);
        for (uint8_t k = 0; k < PROG_UNTIL_MAX_ITER && gameAllowsMotorRun(); k++) {
          if (!sensorMatches(target)) break;
          if (progExecRange(i + 1, end, inner)) { seen = true; break; }
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
      } else {
        const uint16_t reps = (s.op == OP_REPEAT) ? (uint16_t)s.val : PROG_UNTIL_MAX_ITER;
        for (uint16_t k = 0; k < reps && gameAllowsMotorRun() && !seen; k++) {
          if (progExecRange(i + 1, end, inner)) { seen = true; break; }
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
        if (s.op == OP_UNTIL && !seen && gameAllowsMotorRun()) {
          sendStatusF("PROG:MAXITER:%s", PROG_COLOR_NAMES[s.color]);
        }
      }
      if (s.op == OP_UNTIL && seen) sendStatusF("PROG:COLOR:%s", PROG_COLOR_NAMES[s.color]);
      i = end;                                   // ueber das OP_END springen
      continue;
    }

    // ---------- einfacher Baustein ----------
    // LED ticken: haelt einen Not-Aus-Alarm waehrend des Laufs sichtbar
    // (und animiert ein per S-Block gesetztes Licht weiter).
    ledStripTick();
    sendStatusF("PROG:STEP:%u", i);
    int8_t cc = -1;
    if (s.op == OP_FWD || s.op == OP_BACK || s.op == OP_MM_FWD || s.op == OP_MM_BACK) {
      cc = (s.color >= 0) ? s.color : ctx.defaultColor;
    } else if (s.op == OP_DRIVE_UNTIL || s.op == OP_WAIT_UNTIL) {
      // P/V tragen ihre Bedingung IMMER selbst im Block - sie sind genau die
      // Bausteine, bei denen der Aufrufer die Farbe ausdruecklich waehlt.
      // Deshalb NICHT ctx.defaultColor: ein "P3" in einer "bis GELB"-Schleife
      // muss auf Blau reagieren, nicht auf Gelb.
      cc = s.color;
    }
    if (progExecSimple(s.op, s.val, cc)) seen = true;

    // Slalom: Zielfarbe nach jedem Baustein pruefen
    if (ctx.slalom && gameAllowsMotorRun() && slalomTargetStable(ctx.slalomTarget)) {
      slalomFinish(ctx.slalomTarget);
    }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(PROG_STEP_GAP_MS));
  }
  return seen;
}

void progRun() {
  // Re-Entry-Schutz. handleCommand() ist aus jeder blockierenden
  // Spiel-Warteschleife heraus re-entrant (bleProcessEvents()). Ohne diese
  // Zeile koennte ein zweites PROG:RUN waehrend des Laufs eine zweite
  // Ausfuehrung starten - rekursiv und ohne Tiefenbegrenzung, also bis zum
  // Stack-Ueberlauf. Ein laufendes Programm wird nie verschachtelt.
  if (progActive) {
    sendStatus("PROG:BUSY");
    Serial.println("[PROG] Laufend - Start ignoriert");
    return;
  }
  if (progLen == 0) { sendStatus("PROG:NONE"); return; }
  progActive = true;
  currentGame = GAME_PROG;
  colorPollReset();
  sendStatus("PROG:RUNNING");
  Serial.printf("[PROG] Start, %u Instruktionen\n", progLen);

  // LED-Streifen waehrend des Laufs dunkel: das Programm kann Farben lesen
  // ("bis Farbe"), eigenes LED-Licht wuerde den Sensor verfaelschen. Der
  // Not-Aus-Alarm blinkt trotzdem, weil progExecRange ledStripTick() aufruft.
  ledStripClear();

  ProgCtx ctx;                  // kein Slalom, keine Until-Farbe
  progExecRange(0, progLen, ctx);

  stopMotors(); speedL = 0; speedR = 0;
  const bool aborted = (currentGame != GAME_PROG);
  progActive = false;
  currentGame = GAME_NONE;
  sendStatus(aborted ? "PROG:ABORT" : "PROG:DONE");
  Serial.printf("[PROG] %s\n", aborted ? "Abgebrochen" : "Fertig");
}

// Zielfarbe mit kurzer Haltezeit (sensorPoll: 40ms) stabil erkennen.
// Kehrt sofort mit false zurueck, wenn die Farbe gerade nicht sichtbar ist,
// damit der Slalom nach jedem Block nicht kuenstlich wartet.
static bool slalomTargetStable(uint8_t targetIdx) {
  uint32_t start = millis();
  while (millis() - start < 300) {
    if (!gameAllowsMotorRun()) return false;
    char name[16];
    getCurrentColorName(name, sizeof(name));
    if (strcasecmp(name, PROG_COLOR_NAMES[targetIdx]) != 0) {
      colorPollReset();
      return false;
    }
    if (sensorPoll(targetIdx)) return true;
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

// Slalom-Zielfarbe erreicht: Roboter stoppen, Flags setzen und FINISH melden.
// Rueckgabe immer true (Aufrufer kann direkt return).
static bool slalomFinish(uint8_t targetIdx) {
  stopMotors(); speedL = 0; speedR = 0;
  progActive = false;
  currentGame = GAME_NONE;
  slalomFinished = true;
  char fmsg[48];
  snprintf(fmsg, sizeof(fmsg), "SLALOM:FINISH:%s", PROG_COLOR_NAMES[targetIdx]);
  sendStatus(fmsg);
  return true;
}

// =====================================================
// SLALOM: gleicher Executor, aber nach jedem Baustein wird die Zielfarbe
// geprueft. slalomFinish() setzt currentGame = GAME_NONE, worueber sich
// alle Schleifen abwickeln - ein eigenes Rueckgabeflag braucht es nicht.
// =====================================================
void progRunSlalom(const char* targetColor) {
  if (progLen == 0) { sendStatus("SLALOM:NONE"); return; }
  progActive = true;
  currentGame = GAME_SLALOM;
  colorPollReset();
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", targetColor ? targetColor : "ROT");
  sendStatus(msg);
  Serial.printf("[SLALOM] Start target=%s, %u Instruktionen\n", targetColor, progLen);

  // Farb-Index des Ziels ermitteln (fuer colorPoll)
  uint8_t targetIdx = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (strcasecmp(targetColor, PROG_COLOR_NAMES[i]) == 0) { targetIdx = i; break; }
  }

  ProgCtx ctx;
  ctx.slalom = true;
  ctx.slalomTarget = targetIdx;
  ctx.defaultColor = (int8_t)targetIdx;   // F/B ohne eigenes Ziel -> Slalomziel
  progExecRange(0, progLen, ctx);

  if (slalomFinished) return;              // Ziel wurde im Executor erreicht

  // Programmende ohne Treffer: nicht gewonnen, Runde weiter
  stopMotors(); speedL = 0; speedR = 0;
  progActive = false;
  currentGame = GAME_NONE;
  sendStatus("SLALOM:DONE");
  Serial.println("[SLALOM] Programmende ohne Treffer");
}

void progStop() {
  if (!progActive && currentGame != GAME_PROG) return;
  progActive = false;
  currentGame = GAME_NONE;   // laufende Bewegungen brechen selbst ab
  stopMotors(); speedL = 0; speedR = 0;
  sendStatus("PROG:ABORT");
  Serial.println("[PROG] Stop angefordert");
}
