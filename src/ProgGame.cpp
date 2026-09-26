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

static const char* const PROG_COLOR_NAMES[6] = { "ROT", "GELB", "GRUEN", "BLAU", "NAH", "FREI" };

// Distanz-Hindernis: VL53L0X-Messwert < OBSTACLE_DIST_MM (siehe DistanceSensor.h).
// -> distBlocked() = true bedeutet: Hindernis erkannt (wie NAH im Web-UI)
static inline bool irBlocked() {
  return distBlocked();
}

// =====================================================
// Farb-Erkennung mit Haltezeit
// (Sensor-Task auf Core 0 aktualisiert den Farbnamen laufend)
// =====================================================
static uint32_t colMatchStart = 0;
static int8_t   colMatchIdx   = -1;

// Schneller Check fuer Notfall-Stop (50ms Haltezeit) – eigener State
static uint32_t colQuickStart = 0;
static int8_t   colQuickIdx   = -1;

static void colorPollReset() { colMatchStart = 0; colMatchIdx = -1; }
static void colorQuickReset() { colQuickStart = 0; colQuickIdx = -1; }

// true wenn die Sensor-Bedingung (Farbe 0-3 oder Distanz 4/5) erfuellt ist.
// Farbe: seit >= 40ms stabil sichtbar (kurze Haltezeit, damit selbst kurze
// Fahrten im Bis-Modus genug Zeit haben zum Erkennen).
// Distanz: letzter VL53L0X-Wert gegen OBSTACLE_DIST_MM (keine Haltezeit noetig).
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

// Schnellerer Notfall-Stop-Check (50ms Haltezeit, unabhaengiger State)
static bool colorPollQuick(uint8_t idx) {
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  if (strcasecmp(name, PROG_COLOR_NAMES[idx]) != 0) {
    colorQuickReset(); return false;
  }
  if (colQuickIdx != (int8_t)idx) { colQuickIdx = (int8_t)idx; colQuickStart = millis(); return false; }
  return (millis() - colQuickStart) >= 50;
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

// Kurz warten bis Farbe stabil sichtbar (max timeoutMs), mit Abbruch-Check
static bool colorSettled(uint8_t idx, uint32_t timeoutMs = 250) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!gameAllowsMotorRun()) return false;
    if (sensorPoll(idx)) return true;
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

// =====================================================
// Umrechnung Benutzer-Einheiten -> Stepper-Schritte
// quizForwardSteps = Schritte pro 3cm, quizTurnSteps = Schritte pro 90 Grad
// =====================================================
static int32_t cmToSteps(int16_t cm) {
  return (int32_t)(((float)cm * (float)config.quizForwardSteps) / 3.0f + 0.5f);
}
static int32_t degToSteps(int16_t deg) {
  return (int32_t)(((float)deg * (float)config.quizTurnSteps) / 90.0f + 0.5f);
}

// =====================================================
// Bewegung mit Abbruch + optionalem Farb-Check waehrend der Fahrt
// sL/sR: logische Rad-Schritte (vorwaerts positiv), Vorzeichen-Verkabelung
// wird ueber config.leftSign/rightSign korrigiert.
// Zusaetzlich: globaler ROT-Notfall-Stop. Sobald der Sensor ROT erkennt,
// werden die Motoren sofort gestoppt und das Programm abgebrochen
// (currentGame = GAME_NONE). Reaktion in <= 50ms (kurze Haltezeit).
// Rueckgabe: true wenn die Ziel-Farbe die Fahrt beendet hat ODER ROT gesehn.
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
      // Globaler ROT-Stop: NUR im Solo-Programmiermodus (GAME_PROG).
      // Im Slalom ist ROT eine der 4 normalen Zielfarben - dort uebernimmt
      // der Ziel-Check (sensorHitImmediate / sensorPoll) das Anhalten. Ein
      // pauschaler ROT-Stop wuerde sonst schon direkt nach dem Losfahren
      // stoppen (ROT-Wert der Startflaeche / Heuristik) und die Runde ohne
      // Treffer (slalomFinished=false) abbrechen.
      if (currentGame != GAME_SLALOM && colorPollQuick(PROG_COLOR_ROT)) {
        colorHit = true;
        stopMotors();
        // Programm komplett abbrechen
        progActive = false;
        currentGame = GAME_NONE;
        speedL = 0; speedR = 0;
        sendStatus("PROG:STOP_RED");
        break;
      }
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
    case OP_LEFT:
      steps = degToSteps(val);
      progMove(-steps, steps, FIXED_DRIVE_SPEED, -1);
      return false;
    case OP_RIGHT:
      steps = degToSteps(val);
      progMove(steps, -steps, FIXED_DRIVE_SPEED, -1);
      return false;
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
// EQ_COLOR-Slot. Die Farbe steht in st.color, der Betrag |val|-1 ist der Index.
// So bleibt die Struct-Groesse unveraendert.
static int16_t clampVal(uint8_t op, long v) {
  long lo = 0, hi = 0;
  switch (op) {
    case OP_FWD: case OP_BACK:  lo = 1; hi = 100; break;  // cm
    case OP_LEFT: case OP_RIGHT: lo = 5; hi = 360; break; // Grad
    case OP_WAIT:               lo = 1; hi = 10;  break;  // Sekunden
    case OP_REPEAT:             lo = 1; hi = 20;  break;  // Anzahl
    default: break;
  }
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int16_t)v;
}

static bool opFromChar(char c, uint8_t& op) {
  switch (c) {
    case 'F': op = OP_FWD;   return true;
    case 'B': op = OP_BACK;  return true;
    case 'L': op = OP_LEFT;  return true;
    case 'R': op = OP_RIGHT; return true;
    case 'W': op = OP_WAIT;  return true;
    default: return false;
  }
}

// Loop-Body parsen: "F10,R90" bis zur schliessenden Klammer.
// p steht hinter '(' und wird hinter ')' positioniert.
// Body kann nur F/B/L/R/W enthalten (keine verschachtelten Loops).
// W kann im Body als WE<color> vorkommen (warte bis Sensor == color).
static bool parseBody(const char*& p, ProgStep& st) {
  uint8_t bl = 0;
  while (*p != '\0' && *p != ')') {
    char c = *p;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if (c == ',' || c == ' ') { p++; continue; }
    p++;
    uint8_t op;
    if (!opFromChar(c, op)) return false;   // keine verschachtelten Loops
    if (bl >= PROG_LOOP_BODY_MAX) return false;
    st.body[bl].op = op;
    st.body[bl].color = -1;
    // Spezialfall W mit eingebettetem E<color> (Wait-for-Color im Body)
    if (op == OP_WAIT && *p == 'E') {
      p++;
      if (*p < '0' || *p > '3') return false;
      st.body[bl].val = -(int16_t)((uint8_t)(*p - '0') + 1);
      p++;
    } else if ((op == OP_FWD || op == OP_BACK) && *p == 'E') {
      p++;
      if (*p < '0' || *p > '3') return false;
      st.body[bl].color = (int8_t)(*p - '0');
      st.body[bl].val = 50; // max search distance in cm
      p++;
    } else {
      long v;
      if (!parseUInt(p, v)) return false;
      st.body[bl].val = clampVal(op, v);
    }
    bl++;
  }
  if (*p != ')') return false;
  p++;
  st.bodyLen = bl;
  return bl > 0;  // leerer Loop macht keinen Sinn
}

bool progSetFromString(const char* s) {
  if (s == nullptr) return false;
  Serial.printf("[PROG] Parse '%s'\n", s);
  uint8_t n = 0;
  const char* p = s;
  while (*p != '\0' && n < PROG_MAX_STEPS) {
    char c = *p;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    if (c == ',' || c == ' ') { p++; continue; }

    ProgStep st;
    memset(&st, 0, sizeof(st));
    st.color = -1;
    p++;

    if (c == 'U') {
      // Slot: Zahl ODER E<slot> (bedingte Wiederholung solange Sensor == slot)
      if (*p == 'E') {
        p++;
        if (*p < '0' || *p > '5') return false;
        st.op = OP_REPEAT;
        st.color = (uint8_t)(*p - '0');
        st.val = -(int16_t)(st.color + 1);   // negativer Marker
        p++;
      } else {
        long v;
        if (!parseUInt(p, v)) return false;
        st.op = OP_REPEAT;
        st.val = clampVal(OP_REPEAT, v);
      }
      if (*p != '(') return false;
      p++;
      if (!parseBody(p, st)) return false;
    } else if (c == 'W') {
      // Slot: Zahl ODER E<slot> (warte bis Sensor == slot)
      if (*p == 'E') {
        p++;
        if (*p < '0' || *p > '5') return false;
        st.op = OP_WAIT;
        st.color = (uint8_t)(*p - '0');
        st.val = -(int16_t)(st.color + 1);
        p++;
      } else {
        long v;
        if (!parseUInt(p, v)) return false;
        st.op = OP_WAIT;
        st.val = clampVal(OP_WAIT, v);
      }
    } else if (c == 'C') {
      long v;
      if (!parseUInt(p, v)) return false;
      if (v < 0 || v > 5) return false;
      st.op = OP_UNTIL;
      st.color = (uint8_t)v;
      if (*p != '(') return false;
      p++;
      if (!parseBody(p, st)) return false;
    } else {
      uint8_t op;
      if (!opFromChar(c, op)) return false;
      st.op = op;
      st.color = -1;
      if ((op == OP_FWD || op == OP_BACK) && *p == 'E') {
        p++;
        if (*p < '0' || *p > '5') return false;
        st.color = (int8_t)(*p - '0');
        st.val = 50; // max search distance in cm
        p++;
      } else {
        long v;
        if (!parseUInt(p, v)) return false;
        st.val = clampVal(op, v);
      }
    }
    progSteps[n++] = st;
  }
  progLen = n;
  return n > 0;
}

// =====================================================
// Ausfuehrung
// =====================================================
void progRun() {
  if (progLen == 0) { sendStatus("PROG:NONE"); return; }
  progActive = true;
  currentGame = GAME_PROG;
  colorPollReset();
  sendStatus("PROG:RUNNING");
  Serial.printf("[PROG] Start, %u Bloecke\n", progLen);

  for (uint8_t i = 0; i < progLen && currentGame == GAME_PROG; i++) {
    esp_task_wdt_reset();
    sendStatusF("PROG:STEP:%u", i);
    ProgStep& st = progSteps[i];

    if (st.op == OP_REPEAT) {
      if (st.val < 0) {
        // Bedingte Wiederholung: solange Sensor == color
        uint8_t target = (uint8_t)(-(int)st.val - 1);
        for (uint8_t iter = 0; iter < PROG_UNTIL_MAX_ITER && currentGame == GAME_PROG; iter++) {
          if (!sensorMatches(target)) break;
          esp_task_wdt_reset();
          for (uint8_t j = 0; j < st.bodyLen && currentGame == GAME_PROG; j++) {
            sendStatusF("PROG:SUB:%u,%u", i, j);
            progExecSimple(st.body[j].op, st.body[j].val, st.body[j].color);
          }
          bleProcessEvents();
          espNowProcessEvents();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
        }
      } else {
        // n-mal wiederholen
        for (int16_t k = 0; k < st.val && currentGame == GAME_PROG; k++) {
          for (uint8_t j = 0; j < st.bodyLen && currentGame == GAME_PROG; j++) {
            sendStatusF("PROG:SUB:%u,%u", i, j);
            progExecSimple(st.body[j].op, st.body[j].val, st.body[j].color);
          }
          bleProcessEvents();
          espNowProcessEvents();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
      }
    } else if (st.op == OP_UNTIL) {
      // wiederholen bis Farbe sichtbar (wird waehrend der Fahrt geprueft,
      // kein colorSettled davor -> Roboter MUSS sich bewegen, sonst faehrt er
      // 0cm und bricht sofort ab falls die Farbe zufaellig schon sichtbar ist).
      Serial.printf("[PROG] OP_UNTIL color=%s bodyLen=%u\n", PROG_COLOR_NAMES[st.color], st.bodyLen);
      for (uint8_t k = 0; k < st.bodyLen; k++) {
        Serial.printf("[PROG]   body[%u] op=%u val=%d color=%d\n", k, st.body[k].op, st.body[k].val, (int)st.body[k].color);
      }
      bool seen = false;
      colorPollReset();
      for (uint8_t iter = 0; iter < PROG_UNTIL_MAX_ITER && !seen && currentGame == GAME_PROG; iter++) {
        for (uint8_t j = 0; j < st.bodyLen && !seen && currentGame == GAME_PROG; j++) {
          sendStatusF("PROG:SUB:%u,%u", i, j);
          // Farb-Check nur waehrend Fahr-Bloecken sinnvoll
          int8_t cc = (st.body[j].op == OP_FWD || st.body[j].op == OP_BACK)
                      ? (st.body[j].color >= 0 ? st.body[j].color : (int8_t)st.color) : (int8_t)-1;
          seen = progExecSimple(st.body[j].op, st.body[j].val, cc);
        }
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
        esp_task_wdt_reset();
      }
      Serial.printf("[PROG] OP_UNTIL done seen=%d iter\n", (int)seen);
      if (seen) sendStatusF("PROG:COLOR:%s", PROG_COLOR_NAMES[st.color]);
      else if (currentGame == GAME_PROG) sendStatus("PROG:MAXITER");
    } else {
      progExecSimple(st.op, st.val, st.color);
    }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(PROG_STEP_GAP_MS));
  }

   stopMotors(); speedL = 0; speedR = 0;
  bool aborted = (currentGame != GAME_PROG);
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
// SLALOM: Farbziel während der Fahrt passiv beobachten
// (ähnelt progRun, aber stoppt sofort, wenn die Zielfarbe
//  stabil erkannt wird, und meldet SLALOM:FINISH)
// =====================================================
void progRunSlalom(const char* targetColor) {
  if (progLen == 0) { sendStatus("SLALOM:NONE"); return; }
  progActive = true;
  currentGame = GAME_SLALOM;
  colorPollReset();
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", targetColor ? targetColor : "ROT");
  sendStatus(msg);
  Serial.printf("[SLALOM] Start target=%s, %u Bloecke\n", targetColor, progLen);

  // Farb-Index des Ziels ermitteln (fuer colorPoll)
  uint8_t targetIdx = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (strcasecmp(targetColor, PROG_COLOR_NAMES[i]) == 0) { targetIdx = i; break; }
  }

  for (uint8_t i = 0; i < progLen && currentGame == GAME_SLALOM; i++) {
    esp_task_wdt_reset();
    ProgStep& st = progSteps[i];

    if (st.op == OP_REPEAT) {
      uint16_t reps = (st.val < 0) ? PROG_UNTIL_MAX_ITER : (uint16_t)st.val;
      for (uint16_t k = 0; k < reps && currentGame == GAME_SLALOM; k++) {
        for (uint8_t j = 0; j < st.bodyLen && currentGame == GAME_SLALOM; j++) {
          // F/B-Bloecke stoppen sofort, sobald die Zielfarbe sichtbar ist
          int8_t cc = (st.body[j].op == OP_FWD || st.body[j].op == OP_BACK)
                        ? (st.body[j].color >= 0 ? st.body[j].color : (int8_t)targetIdx)
                        : (int8_t)-1;
          progExecSimple(st.body[j].op, st.body[j].val, cc);
          if (slalomTargetStable(targetIdx)) { slalomFinish(targetIdx); return; }
        }
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
        esp_task_wdt_reset();
        if (slalomTargetStable(targetIdx)) { slalomFinish(targetIdx); return; }
      }
    } else if (st.op == OP_UNTIL) {
      // Für Slalom: Wiederhole bis Zielfarbe erreicht (oder max iter)
      for (uint8_t iter = 0; iter < PROG_UNTIL_MAX_ITER && currentGame == GAME_SLALOM; iter++) {
        for (uint8_t j = 0; j < st.bodyLen && currentGame == GAME_SLALOM; j++) {
          int8_t cc = (st.body[j].op == OP_FWD || st.body[j].op == OP_BACK)
                        ? (st.body[j].color >= 0 ? st.body[j].color : (int8_t)targetIdx) : (int8_t)-1;
          progExecSimple(st.body[j].op, st.body[j].val, cc);
          if (slalomTargetStable(targetIdx)) { slalomFinish(targetIdx); return; }
        }
        bleProcessEvents();
        espNowProcessEvents();
        vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
        esp_task_wdt_reset();
        if (slalomTargetStable(targetIdx)) { slalomFinish(targetIdx); return; }
      }
    } else {
      int8_t cc = (st.op == OP_FWD || st.op == OP_BACK)
                    ? (st.color >= 0 ? st.color : (int8_t)targetIdx)
                    : (int8_t)-1;
      progExecSimple(st.op, st.val, cc);
      if (slalomTargetStable(targetIdx)) { slalomFinish(targetIdx); return; }
    }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(PROG_STEP_GAP_MS));
  }

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
