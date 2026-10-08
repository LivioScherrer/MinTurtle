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
  prepPositionMove(0, sL * (long)config.leftSign);
  prepPositionMove(1, sR * (long)config.rightSign);
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

// Einfachen Baustein ausfuehren (kein Loop). Rueckgabe: true wenn Farbe ausgeloest.
static bool progExecSimple(uint8_t op, int16_t val, int8_t colorCheck) {
  int32_t steps;
// Feste Geschwindigkeit - keine Einstellung irgendwo.
  switch (op) {
    case OP_FWD:
      steps = cmToSteps(val);
      return progMove(steps, steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_BACK:
      steps = cmToSteps(val);
      return progMove(-steps, -steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_MM_FWD:
      // Millimetergenau: der Wert wird direkt in Schritte gerechnet und ist
      // dadurch unabhaengig von der Microstep-Einstellung.
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      return progMove(steps, steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_MM_BACK:
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      return progMove(-steps, -steps, FIXED_DRIVE_SPEED, colorCheck);
    case OP_LEFT:
      steps = degToSteps(val);
      progMove(-steps, steps, FIXED_DRIVE_SPEED, -1);
      return false;
    case OP_RIGHT:
      steps = degToSteps(val);
      progMove(steps, -steps, FIXED_DRIVE_SPEED, -1);
      return false;
    case OP_DRIVE_DIST: {
      // "Fahre, bis der Abstand <= val ist." In kleinen Etappen, damit der
      // Sensor frueh genug reagiert. Sicherheitsnetze: max. Strecke und Zeit.
      const int16_t target = val;
      const int16_t stepMM = 10;
      const int16_t maxMM  = 1500;
      const uint32_t t0 = millis();
      int16_t driven = 0;
      while (driven < maxMM && millis() - t0 < 20000UL) {
        if (!gameAllowsMotorRun()) return false;
        const int16_t d = distMeasureMM();
        if (d >= 0 && d <= target) break;          // Ziel erreicht
        if (irBlocked()) break;                    // Hindernis-Schwelle
        steps = geoStepsForMM((float)stepMM, config.tmcMicrosteps);
        progMove(steps, steps, FIXED_DRIVE_SPEED, -1);
        driven = (int16_t)(driven + stepMM);
        bleProcessEvents();
        espNowProcessEvents();
        esp_task_wdt_reset();
      }
      Serial.printf("[PROG] D%u gefahren %u mm (Restabstand %d)\n",
                    target, driven, (int)distLastMM());
      return false;
    }
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

    int8_t slot = -1;
    const bool hasSlot = parseESlot(p, slot);

    // F/B mit E-Slot: fahre bis zur Farbe (bzw. bis IR-Bedingung)
    if (hasSlot && (op == OP_FWD || op == OP_BACK)) {
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

static inline bool progGameActive() {
  return currentGame == GAME_PROG || currentGame == GAME_SLALOM;
}

// Fuehrt progSteps[from..to) aus (to ist exklusiv).
// Rueckgabe: true, wenn ctx.untilColor gesehen wurde (beendet den Bis-Loop).
static bool progExecRange(uint8_t from, uint8_t to, ProgCtx ctx) {
  bool seen = false;
  for (uint8_t i = from; i < to && progGameActive() && !seen; i++) {
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
        for (uint8_t k = 0; k < PROG_UNTIL_MAX_ITER && progGameActive(); k++) {
          if (!sensorMatches(target)) break;
          if (progExecRange(i + 1, end, inner)) { seen = true; break; }
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
      } else {
        const uint16_t reps = (s.op == OP_REPEAT) ? (uint16_t)s.val : PROG_UNTIL_MAX_ITER;
        for (uint16_t k = 0; k < reps && progGameActive() && !seen; k++) {
          if (progExecRange(i + 1, end, inner)) { seen = true; break; }
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
        if (s.op == OP_UNTIL && !seen && progGameActive()) {
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
    }
    if (progExecSimple(s.op, s.val, cc)) seen = true;

    // Slalom: Zielfarbe nach jedem Baustein pruefen
    if (ctx.slalom && progGameActive() && slalomTargetStable(ctx.slalomTarget)) {
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
