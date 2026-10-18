#include "ApSignal.h"
#include "ColorSensor.h"
#include "BLEManager.h"
#include "Config.h"
#include "ProgGame.h"
#include "WebUi.h"

#define APSIG_STEP_TIMEOUT_MS  3000

#define APSIG_ANNOUNCE_MS      15000

#define APSIG_POLL_MS          40

#define APSIG_STABLE_READINGS  4

static uint8_t  s_confirmed     = 0;
static uint32_t s_stepSince     = 0;
static uint32_t s_lastPoll      = 0;
static uint32_t s_announceUntil = 0;
static bool     s_announcing    = false;
static char     s_lastName[16]  = {0};
static uint8_t  s_stableCount   = 0;

static uint8_t stepOfColor(const char* name) {
  if (strcmp(name, "ROT") == 0)   return 1;
  if (strcmp(name, "BLAU") == 0)  return 2;
  if (strcmp(name, "GRUEN") == 0) return 3;
  return 0;
}

uint8_t apSignalStep() {
  if (s_announcing && millis() < s_announceUntil) return APSIG_ANNOUNCED;
  if (s_confirmed == 0) return APSIG_IDLE;
  if (s_confirmed == 1) return APSIG_WANT_BLUE;
  return APSIG_WANT_GREEN;
}

bool apSignalActive() {
  if (s_announcing) return millis() < s_announceUntil;
  return s_confirmed > 0;
}

void apSignalReset() {
  s_confirmed = 0;
  s_stableCount = 0;
  s_lastName[0] = '\0';
  s_announcing = false;
  s_announceUntil = 0;
}

static void readColorName(char* out, size_t outLen) {
  out[0] = '\0';
  if (!colorSem) return;
  if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(5)) != pdTRUE) return;
  int r = sensorR, g = sensorG, b = sensorB;
  xSemaphoreGive(colorSem);
  if (r + g + b <= 0) return;
  strlcpy(out, detectColorName(r, g, b), outLen);
}

static void announce() {
  edgeGuardReset();

  webUiStart();

  if (!webUiIsUp()) {
    Serial.println("[AP-SIGNAL] WLAN-Start fehlgeschlagen - kein Access Point sichtbar");
    apSignalReset();
    return;
  }

  s_announcing = true;
  s_announceUntil = millis() + APSIG_ANNOUNCE_MS;
  s_stableCount = 0;
  s_lastName[0] = '\0';

  Serial.println("");
  Serial.println("=====================================================");
  Serial.println("  WLAN EINGESCHALTET - auf dem iPad so beitreten:");
  Serial.println("=====================================================");
  Serial.printf("   1. Netz      : %s\n", webUiSsid().c_str());
  Serial.printf("   2. Passwort  : %s\n", webUiPassword().c_str());
  Serial.println("   3. Browser   : http://192.168.4.1");
  Serial.println("=====================================================");
  Serial.println("");
  Serial.printf("WIFI:T:WPA;S:%s;P:%s;;\n", webUiSsid().c_str(),
                webUiPassword().c_str());
}

void apSignalTick() {
  const uint32_t now = millis();
  if (now - s_lastPoll < APSIG_POLL_MS) return;
  s_lastPoll = now;

  if (!colorSensorOk) return;

  if (anyClientConnected()) {
    if (s_confirmed > 0 || s_announcing) apSignalReset();
    return;
  }

  if (s_announcing) {
    if (now >= s_announceUntil) apSignalReset();
    return;
  }

  if (s_confirmed > 0 && now - s_stepSince > APSIG_STEP_TIMEOUT_MS) {
    Serial.println("[AP-SIGNAL] Folge abgerissen - neu beginnen mit ROT");
    apSignalReset();
  }

  char name[16];
  readColorName(name, sizeof(name));
  if (name[0] == '\0') return;

  if (strncmp(name, s_lastName, sizeof(s_lastName)) == 0) {
    if (s_stableCount < APSIG_STABLE_READINGS) s_stableCount++;
  } else {
    strlcpy(s_lastName, name, sizeof(s_lastName));
    s_stableCount = 1;
  }
  if (s_stableCount < APSIG_STABLE_READINGS) return;

  const uint8_t seen = stepOfColor(name);
  if (seen == 0) return;

  if (seen > s_confirmed + 1) return;
  if (seen <= s_confirmed) return;

  s_confirmed = seen;
  s_stepSince = now;
  s_stableCount = 0;
  s_lastName[0] = '\0';

  if (seen == 1) {
    Serial.println("[AP-SIGNAL] ROT gesehen - jetzt BLAU zeigen");
  } else if (seen == 2) {
    Serial.println("[AP-SIGNAL] BLAU gesehen - jetzt GRUEN zeigen");
  } else {
    Serial.println("[AP-SIGNAL] GRUEN gesehen - Folge vollstaendig");
    announce();
  }
}