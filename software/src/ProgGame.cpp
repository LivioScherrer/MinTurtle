#include "ProgGame.h"
#include "GameCommon.h"
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

extern bool slalomFinished;

ProgStep progSteps[PROG_MAX_STEPS];
uint8_t  progLen    = 0;
bool     progActive = false;

static float s_progSavedSpeed = 150.0f;
static bool  s_progSavedValid = false;
static void progSpeedSave() {
  if (!s_progSavedValid) { s_progSavedSpeed = config.driveSpeedMM; s_progSavedValid = true; }
}
static void progSpeedRestore() {
  if (!s_progSavedValid) return;
  s_progSavedValid = false;
  if (config.driveSpeedMM != s_progSavedSpeed) {
    config.driveSpeedMM = s_progSavedSpeed;
    motorUpdateSpeeds();
    Serial.printf("[PROG] Tempo zurueck auf %.0f mm/s\n", (double)s_progSavedSpeed);
  }
}
static void progSpeedSet(int16_t mms) {
  float want = (float)mms;
  if (want < SPEED_MM_S_MIN) want = SPEED_MM_S_MIN;
  if (want > SPEED_MM_S_MAX) want = SPEED_MM_S_MAX;
  const float maxReach = MAX_DRIVE_STEPS_S / configStepsPerMM();
  if (want > maxReach) want = floorf(maxReach);
  config.driveSpeedMM = want;
  motorUpdateSpeeds();
  Serial.printf("[PROG] Tempo T%d -> %.0f mm/s (gilt bis Programmende)\n",
                (int)mms, (double)want);
}

constexpr uint32_t PROG_STEP_GAP_MS = 120;
constexpr uint32_t PROG_LOOP_GAP_MS = 100;
constexpr uint32_t SLALOM_TARGET_HOLD_MS = 250;

static const char* const PROG_COLOR_NAMES[6] = { "ROT", "GELB", "GRUEN", "BLAU", "NAH", "FREI" };

static inline bool irBlocked() {
  return distBlocked();
}

static volatile uint8_t progPath[4] = {0, 0, 0, 0};
static volatile uint8_t progPathLen = 0;

static uint32_t colMatchStart = 0;
static int8_t   colMatchIdx   = -1;

constexpr uint16_t EDGE_RAW_C_MAX   = 300;
constexpr uint16_t EDGE_RAW_C_RELEASE = 400;
constexpr uint8_t  EDGE_CONFIRM_N   = 2;

static volatile uint8_t s_edgeCount = 0;
static volatile bool    s_edgeGuardOn = false;
static volatile bool    s_edgeBlocked = false;

bool edgeGuardActive()   { return true; }
bool edgeGuardBlocked()  { return s_edgeBlocked; }
void edgeGuardReset()    { s_edgeCount = 0; s_edgeBlocked = false; }
void edgeGuardSet(bool on) {
  (void)on;
  const bool changed = !s_edgeGuardOn;
  s_edgeGuardOn = true;
  config.edgeGuard = true;
  if (changed) configSaveBool("edgeGuard", true);
  edgeGuardReset();
  Serial.println("[KANTE] Waechter aktiv - stoppt an der Tischkante (alle Modi)");
}

uint16_t edgeGuardMaxNow() {
  if (config.floorCalibrated && config.floorC > 0) {
    uint16_t dyn = config.floorC / 2;
    if (dyn > EDGE_RAW_C_MAX) dyn = EDGE_RAW_C_MAX;
    if (dyn < 40) dyn = 40;
    return dyn;
  }
  return EDGE_RAW_C_MAX;
}
uint16_t edgeGuardReleaseNow() {
  if (config.floorCalibrated && config.floorC > 0) {
    uint16_t dyn = (uint16_t)(((uint32_t)config.floorC * 2) / 3);
    if (dyn > EDGE_RAW_C_RELEASE) dyn = EDGE_RAW_C_RELEASE;
    const uint16_t mx = edgeGuardMaxNow();
    if (dyn <= mx) dyn = (uint16_t)(mx + 20);
    return dyn;
  }
  return EDGE_RAW_C_RELEASE;
}

void floorLearn(uint8_t r, uint8_t g, uint8_t b, uint16_t c) {
  const bool neu = !config.floorCalibrated ||
                   config.floorR != r || config.floorG != g ||
                   config.floorB != b || config.floorC != c;
  config.floorR = r; config.floorG = g; config.floorB = b;
  config.floorC = c; config.floorCalibrated = true;
  if (neu) {
    configSaveUChar("floorR", r);
    configSaveUChar("floorG", g);
    configSaveUChar("floorB", b);
    configSaveUShort("floorC", c);
    configSaveBool("floorSet", true);
  }
  edgeGuardReset();
  Serial.printf("[BODEN] gelernt: rgb(%u,%u,%u) Clear=%u%s -> Kante<%u frei>%u\n",
                r, g, b, c, neu ? "" : " (unveraendert)",
                edgeGuardMaxNow(), edgeGuardReleaseNow());
}
static void colorPollReset() { colMatchStart = 0; colMatchIdx = -1; }

static bool sensorPoll(uint8_t idx) {
  if (idx >= PROG_SENSOR_IR_HIGH) return (irBlocked() == (idx == PROG_SENSOR_IR_HIGH));
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  if (strcasecmp(name, PROG_COLOR_NAMES[idx]) != 0) { colorPollReset(); return false; }
  if (colMatchIdx != (int8_t)idx) { colMatchIdx = (int8_t)idx; colMatchStart = millis(); return false; }
  return (millis() - colMatchStart) >= 40;
}

static bool sensorHitImmediate(uint8_t idx) {
  if (idx >= PROG_SENSOR_IR_HIGH) return (irBlocked() == (idx == PROG_SENSOR_IR_HIGH));
  if (!colorSensorOk) return false;
  char name[16];
  getCurrentColorName(name, sizeof(name));
  return strcasecmp(name, PROG_COLOR_NAMES[idx]) == 0;
}

void edgeGuardTick(uint16_t rawClear) {
  if (gameEdgeGuardSuspended()) { s_edgeCount = 0; s_edgeBlocked = false; return; }

  if (s_edgeBlocked) {
    const uint16_t rel = edgeGuardReleaseNow();
    if (rawClear > rel) {
      s_edgeBlocked = false;
      s_edgeCount = 0;
      motorReleaseEmergency();
      Serial.printf("[KANTE] Boden wieder da (Clear=%u > %u) - freigegeben\n",
                    rawClear, rel);
    } else {
      stopMotors();
    }
    return;
  }

  const uint16_t mx = edgeGuardMaxNow();
  if (rawClear < mx) {
    if (s_edgeCount < 255) s_edgeCount++;
    if (s_edgeCount == EDGE_CONFIRM_N) {
      stopMotors();
      motorEmergencyStop();
      Serial.printf("[KANTE] Tischkante nach %u Messungen (Clear=%u < %u) - "
                    "Motoren gestoppt, gesperrt bis zum Boden\n",
                    s_edgeCount, rawClear, mx);
      s_edgeBlocked = true;
    }
  } else {
    s_edgeCount = 0;
  }
}

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
    slalomHitArmed = true;
    slalomHitStart = 0;
    slalomHitIdx   = -1;
    return false;
  }
  if (!slalomHitArmed) return false;
  if (slalomHitIdx != (int8_t)idx) { slalomHitIdx = (int8_t)idx; slalomHitStart = millis(); return false; }
  return (millis() - slalomHitStart) >= holdMs;
}

static inline bool gameAllowsMotorRun() {
  return currentGame == GAME_PROG || currentGame == GAME_SLALOM;
}

static int32_t cmToSteps(int16_t cm) {
  const float mm = (float)cm * 10.0f;
  return geoStepsForMM(mm, config.tmcMicrosteps);
}
static int32_t degToSteps(int16_t deg) {
  const float d = (float)deg * (config.quizTurnDeg / QUIZ_TURN_DEG);
  return geoStepsForDeg(d, config.tmcMicrosteps);
}

static bool progMove(int32_t sL, int32_t sR, int8_t colorCheck) {
  const int32_t physL = sL * (long)config.leftSign;
  const int32_t physR = sR * (long)config.rightSign;
  prepPositionMoves(physL, physR);
  if (colorCheck >= 0 && colorCheck < (int8_t)PROG_SENSOR_IR_HIGH) {
    char nm[16];
    getCurrentColorName(nm, sizeof(nm));
    slalomHitReset(strcasecmp(nm, PROG_COLOR_NAMES[(uint8_t)colorCheck]) != 0);
  } else {
    slalomHitReset(true);
  }
  uint32_t lastPoll = 0;
  bool colorHit = false;
  while (motorStillRunning()) {
    esp_task_wdt_reset();
    if (!gameAllowsMotorRun()) { stopMotors(); break; }
    if (millis() - lastPoll >= 20) {
      lastPoll = millis();
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

static bool progDriveUntil(int8_t slot, int16_t targetMM, int32_t maxMM) {
  if (slot < 0 && targetMM <= 0) {
    Serial.println("[PROG] fahre-bis ohne Bedingung - nicht ausgefuehrt");
    return false;
  }
  if (maxMM <= 0 || maxMM > PROG_DRIVE_UNTIL_MAX_MM) maxMM = PROG_DRIVE_UNTIL_MAX_MM;

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

  setMotorSpeeds(FIXED_DRIVE_SPEED, FIXED_DRIVE_SPEED, true);
  for (;;) {
    if (!gameAllowsMotorRun()) break;
    const uint32_t now = millis();
    uint32_t dt = now - lastMs;
    lastMs = now;
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

static void progLogDist(const char* op, int16_t val, float mm, int32_t steps) {
  Serial.printf("[PROG] %s%ld -> %.1f mm (%.2f cm) = %ld Schritte (%u Microsteps)\n",
                op, (long)val, (double)mm, (double)(mm / 10.0), (long)steps,
                (unsigned)config.tmcMicrosteps);
}

static bool progExecSimple(uint8_t op, int16_t val, int8_t colorCheck) {
  int32_t steps;
  switch (op) {
    case OP_FWD:
      steps = cmToSteps(val);
      progLogDist("F", val, (float)val * 10.0f, steps);
      return progMove(steps, steps, colorCheck);
    case OP_BACK:
      steps = cmToSteps(val);
      progLogDist("B", val, (float)val * 10.0f, steps);
      return progMove(-steps, -steps, colorCheck);
    case OP_DRIVE_UNTIL:
      return progDriveUntil(colorCheck, -1,
                            val > 0 ? (int32_t)val : PROG_DRIVE_UNTIL_MAX_MM);
    case OP_WAIT_UNTIL:
      return progWaitUntil(colorCheck);
    case OP_MM_FWD:
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      progLogDist("M", val, (float)val, steps);
      return progMove(steps, steps, colorCheck);
    case OP_MM_BACK:
      steps = geoStepsForMM((float)val, config.tmcMicrosteps);
      progLogDist("N", val, (float)val, steps);
      return progMove(-steps, -steps, colorCheck);
    case OP_LEFT:
      steps = degToSteps(val);
      progMove(-steps, steps, -1);
      return false;
    case OP_RIGHT:
      steps = degToSteps(val);
      progMove(steps, -steps, -1);
      return false;
    case OP_DRIVE_DIST:
      return progDriveUntil(-1, val, PROG_DRIVE_UNTIL_MAX_MM);
    case OP_WAIT_DIST: {
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
      const uint32_t t0 = millis();
      while (millis() - t0 < 5000UL) {
        if (!gameAllowsMotorRun()) return false;
        const int16_t d = distMeasureMM();
        if (d >= val || d < 0) return false;
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
      ledStripTick();
      bleProcessEvents();
      espNowProcessEvents();
      vTaskDelay(pdMS_TO_TICKS(60));
      return false;
    }
    case OP_PAUSE: {
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
    case OP_SPEED: {
      progSpeedSet(val);
      return false;
    }
    case OP_WAIT: {
      if (val < 0) {
        uint8_t target = (uint8_t)(-(int)val - 1);
        colorPollReset();
        uint32_t start = millis();
        while (millis() - start < 60000) {
          if (!gameAllowsMotorRun()) return false;
          if (sensorPoll(target)) return false;
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

static int16_t clampVal(uint8_t op, long v) {
  long lo = 0, hi = 0;
  switch (op) {
    case OP_FWD: case OP_BACK:      lo = 1;   hi = 200;   break;
    case OP_LEFT: case OP_RIGHT:    lo = 5;   hi = 360;   break;
    case OP_WAIT:                   lo = 1;   hi = 10;    break;
    case OP_REPEAT:                 lo = 1;   hi = 20;    break;
    case OP_MM_FWD: case OP_MM_BACK: lo = 1;  hi = 2000;  break;
    case OP_DRIVE_DIST:             lo = 20;  hi = 1500;  break;
    case OP_WAIT_DIST:              lo = 10;  hi = 1500;  break;
    case OP_WAIT_FREE:              lo = 10;  hi = 1500;  break;
    case OP_PAUSE:                  lo = 1;   hi = 30;    break;
    case OP_LED:                    lo = 0;   hi = 6;     break;
    case OP_SPEED:                  lo = (long)SPEED_MM_S_MIN; hi = (long)SPEED_MM_S_MAX; break;
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
    case 'T': op = OP_SPEED;      return true;
    default: return false;
  }
}

static bool parseESlot(const char*& p, int8_t& slotOut) {
  if (*p != 'E') return false;
  p++;
  if (*p < '0' || *p > '5') return false;
  slotOut = (int8_t)(*p - '0');
  p++;
  return true;
}

  static bool progParseRange(const char*& p, uint8_t& pc, uint8_t depth, bool topLevel) {
    uint8_t sib = 0;
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
      st.sibIdx = 0xFF;

      if (c == 'U' || c == 'C') {
        if (depth >= PROG_MAX_DEPTH) { Serial.println("[PROG] Verschachtelung zu tief"); return false; }
        if (pc + 1 >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
        int8_t slot = -1;
        if (c == 'U') {
          st.op = OP_REPEAT;
          if (parseESlot(p, slot)) {
            st.color = slot;
            st.val = -(int16_t)(slot + 1);
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
        st.sibIdx = sib++;
        progSteps[pc++] = st;
        if (!progParseRange(p, pc, (uint8_t)(depth + 1), false)) return false;
        if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
        progSteps[headIdx].endIdx = pc;
        ProgStep e;
        memset(&e, 0, sizeof(e));
        e.op = OP_END;
        e.color = -1;
        e.sibIdx = 0xFF;
        progSteps[pc++] = e;
        continue;
      }

      uint8_t op;
      if (!opFromChar(c, op)) { Serial.printf("[PROG] unbekannter Baustein '%c'\n", c); return false; }
      st.op = op;

      if (op == OP_DRIVE_UNTIL || op == OP_WAIT_UNTIL) {
        if (*p < '0' || *p > '5') { Serial.printf("[PROG] %c braucht eine Farbe/Sensor 0-5\n", c); return false; }
        st.color = (int8_t)(*p - '0');
        st.val = 0;
        p++;
        if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
        st.sibIdx = sib++;
        progSteps[pc++] = st;
        continue;
      }

      int8_t slot = -1;
      const bool hasSlot = parseESlot(p, slot);

      if (hasSlot && op == OP_FWD) {
        st.op      = OP_DRIVE_UNTIL;
        st.color   = slot;
        st.val     = 500;
      } else if (hasSlot && op == OP_BACK) {
        st.color = slot;
        st.val = 50;
      } else if (hasSlot && op == OP_WAIT) {
        st.val = -(int16_t)(slot + 1);
      } else if (hasSlot) {
        return false;
      } else if (op == OP_STOP) {
        st.val = 0;
      } else {
        long v;
        if (!parseUInt(p, v)) return false;
        st.val = clampVal(op, v);
      }
      if (pc >= PROG_MAX_STEPS) { Serial.println("[PROG] Programm zu lang"); return false; }
      st.sibIdx = sib++;
      progSteps[pc++] = st;
    }
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


static bool slalomTargetStable(uint8_t targetIdx);
static bool slalomFinish(uint8_t targetIdx);

struct ProgCtx {
  bool    slalom      = false;
  uint8_t slalomTarget = 0;
  int8_t  untilStack[PROG_MAX_DEPTH];
  uint8_t untilTop    = 0;
  int8_t  defaultColor = -1;
  uint8_t headIndex   = 0;
  uint8_t path[PROG_MAX_DEPTH + 1];
  uint8_t pathLen     = 0;
bool bodyHadDrivable = false;
};

static int8_t progExecRange(uint8_t from, uint8_t to, ProgCtx& ctx) {
  int8_t hitSlot = -1;
  for (uint8_t i = from; i < to && gameAllowsMotorRun(); i++) {
    ProgStep& s = progSteps[i];
    if (s.op == OP_END) continue;
    esp_task_wdt_reset();

    uint8_t msgPath[PROG_MAX_DEPTH + 1];
    uint8_t msgPathLen = ctx.pathLen;
    memcpy(msgPath, ctx.path, ctx.pathLen);
    if (s.sibIdx != 0xFF && msgPathLen < PROG_MAX_DEPTH + 1) {
      msgPath[msgPathLen++] = s.sibIdx;
    }
    char pathBuf[32];
    pathBuf[0] = '\0';
    for (uint8_t pi = 0; pi < msgPathLen; pi++) {
      char tmp[8];
      snprintf(tmp, sizeof(tmp), "%s%u", (pi ? "." : ""), (unsigned)msgPath[pi]);
      strcat(pathBuf, tmp);
    }
    sendStatusF("PROG:AT:%s", pathBuf);

    if (s.op == OP_REPEAT || s.op == OP_UNTIL) {
      const uint8_t end = s.endIdx;
      if (end <= i || end > progLen) {
        Serial.printf("[PROG] kaputte Sprungmarke bei %u (end=%u)\n", i, end);
        break;
      }
      ProgCtx inner = ctx;
      inner.headIndex = i;
      if (inner.pathLen < PROG_MAX_DEPTH + 1) {
        inner.path[inner.pathLen++] = s.sibIdx;
      }
      if (s.op == OP_UNTIL) {
        if (inner.untilTop < PROG_MAX_DEPTH) {
          inner.untilStack[inner.untilTop++] = s.color;
        }
        inner.defaultColor = s.color;
        colorPollReset();
        Serial.printf("[PROG] Bis %s, %u..%u\n", PROG_COLOR_NAMES[s.color], i + 1, end);
      }
      inner.bodyHadDrivable = false;

      if (s.op == OP_REPEAT && s.val < 0) {
        const uint8_t target = (uint8_t)(-(int)s.val - 1);
        for (uint8_t k = 0; k < PROG_UNTIL_MAX_ITER && gameAllowsMotorRun(); k++) {
          if (!sensorHitImmediate(target)) break;
          int8_t subHit = progExecRange(i + 1, end, inner);
          if (subHit >= 0) hitSlot = subHit;
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
      } else {
        const uint16_t reps = (s.op == OP_REPEAT) ? (uint16_t)s.val : PROG_UNTIL_MAX_ITER;
        for (uint16_t k = 0; k < reps && gameAllowsMotorRun(); k++) {
          int8_t subHit = progExecRange(i + 1, end, inner);
          if (subHit >= 0) hitSlot = subHit;
          if (s.op == OP_UNTIL && hitSlot == s.color) break;
          bleProcessEvents(); espNowProcessEvents();
          ledStripTick();
          vTaskDelay(pdMS_TO_TICKS(PROG_LOOP_GAP_MS));
          esp_task_wdt_reset();
        }
        if (s.op == OP_UNTIL && !inner.bodyHadDrivable && gameAllowsMotorRun()) {
          sendStatusF("PROG:MAXITER:%s", PROG_COLOR_NAMES[s.color]);
        }
      }
      if (s.op == OP_UNTIL) {
        if (inner.untilTop > 0) inner.untilTop--;
        if (hitSlot == s.color) {
          sendStatusF("PROG:COLOR:%s", PROG_COLOR_NAMES[s.color]);
          return hitSlot;
        }
      }
      i = end;
      continue;
    }

    ledStripTick();

    int8_t cc = -1;
    bool readsColor = false;
    if (s.op == OP_FWD || s.op == OP_BACK || s.op == OP_MM_FWD || s.op == OP_MM_BACK) {
      cc = (s.color >= 0) ? s.color : (ctx.untilTop > 0 ? ctx.untilStack[ctx.untilTop - 1] : ctx.defaultColor);
      if (cc >= 0 && cc < (int8_t)PROG_SENSOR_IR_HIGH) readsColor = true;
    } else if (s.op == OP_DRIVE_UNTIL || s.op == OP_WAIT_UNTIL) {
      cc = s.color;
      if (cc >= 0 && cc < (int8_t)PROG_SENSOR_IR_HIGH) readsColor = true;
    } else if (s.op == OP_UNTIL) {
      if (ctx.untilTop > 0) cc = ctx.untilStack[ctx.untilTop - 1];
    }
    if (readsColor) ledStripClear();

    bool drivable = (s.op == OP_FWD || s.op == OP_BACK || s.op == OP_MM_FWD || s.op == OP_MM_BACK);
    if (ctx.untilTop > 0 && drivable) ctx.bodyHadDrivable = true;

    if (s.op == OP_STOP) {
      progActive = false;
      currentGame = GAME_NONE;
      stopMotors();
      return -1;
    }

    int8_t blockHit = progExecSimple(s.op, s.val, cc) ? cc : -1;
    if (blockHit >= 0) hitSlot = blockHit;

    if (ctx.slalom && gameAllowsMotorRun() && slalomTargetStable(ctx.slalomTarget)) {
      slalomFinish(ctx.slalomTarget);
      return -1;
    }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(PROG_STEP_GAP_MS));
  }
  return hitSlot;
}

void progRun() {
  if (progActive) {
    sendStatus("PROG:BUSY");
    Serial.println("[PROG] Laufend - Start ignoriert");
    return;
  }
  if (progLen == 0) { sendStatus("PROG:NONE"); return; }
  progActive = true;
  currentGame = GAME_PROG;
  colorPollReset();
  progSpeedSave();
  sendStatus("PROG:RUNNING");
  Serial.printf("[PROG] Start, %u Instruktionen\n", progLen);

  ledStripClear();

  ProgCtx ctx;
  progExecRange(0, progLen, ctx);

  stopMotors(); speedL = 0; speedR = 0;
  progSpeedRestore();
  const bool aborted = (currentGame != GAME_PROG);
  progActive = false;
  currentGame = GAME_NONE;
  sendStatus(aborted ? "PROG:ABORT" : "PROG:DONE");
  Serial.printf("[PROG] %s\n", aborted ? "Abgebrochen" : "Fertig");
}

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

void progRunSlalom(const char* targetColor) {
  if (progLen == 0) { sendStatus("SLALOM:NONE"); return; }
  progActive = true;
  currentGame = GAME_SLALOM;
  colorPollReset();
  progSpeedSave();
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", targetColor ? targetColor : "ROT");
  sendStatus(msg);
  Serial.printf("[SLALOM] Start target=%s, %u Instruktionen\n", targetColor, progLen);

  uint8_t targetIdx = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (strcasecmp(targetColor, PROG_COLOR_NAMES[i]) == 0) { targetIdx = i; break; }
  }

  ProgCtx ctx;
  ctx.slalom = true;
  ctx.slalomTarget = targetIdx;
  ctx.defaultColor = (int8_t)targetIdx;
  progExecRange(0, progLen, ctx);

  if (slalomFinished) { progSpeedRestore(); return; }

  stopMotors(); speedL = 0; speedR = 0;
  progActive = false;
  currentGame = GAME_NONE;
  progSpeedRestore();
  sendStatus("SLALOM:DONE");
  Serial.println("[SLALOM] Programmende ohne Treffer");
}

void progStop() {
  if (!progActive && currentGame != GAME_PROG) return;
  progActive = false;
  currentGame = GAME_NONE;
  stopMotors(); speedL = 0; speedR = 0;
  progSpeedRestore();
  sendStatus("PROG:ABORT");
  Serial.println("[PROG] Stop angefordert");
}
