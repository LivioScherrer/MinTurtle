#include "Config.h"

Config config;
Preferences prefs;

float configStepsPerMM() {
  return (float)MOTOR_FULL_STEPS_PER_REV * (float)config.tmcMicrosteps / EFFECTIVE_CIRCUMFERENCE_MM;
}
int32_t configForwardSteps() { return geoStepsForMM(config.quizForwardMM, config.tmcMicrosteps); }
int32_t configTurnSteps()    { return geoStepsForDeg(config.quizTurnDeg, config.tmcMicrosteps); }
float configStepsPerS() {
  float sps = config.driveSpeedMM * configStepsPerMM();
  if (!(sps >= 0.0f)) return 0.0f;
  return sps > MAX_DRIVE_STEPS_S ? MAX_DRIVE_STEPS_S : sps;
}

uint16_t configOptimalMicrosteps(float driveSpeedMM) {
  static const uint16_t microstepTable[] = { 256, 128, 64, 32, 16, 8, 4, 2, 1 };
  for (uint16_t ms : microstepTable) {
    float stepsPerMM = (float)MOTOR_FULL_STEPS_PER_REV * (float)ms / EFFECTIVE_CIRCUMFERENCE_MM;
    float sps = driveSpeedMM * stepsPerMM;
    if (sps <= MAX_DRIVE_STEPS_S) {
      return ms;
    }
  }
  return 1;
}

void configLoad() {
  prefs.begin("edubot", false);
  config.tmcMicrosteps = prefs.getUShort("tmcMS", 64);
  config.tmcSpread     = prefs.getBool("tmcSpread", false);

  config.tmcMicrosteps = 64;
  config.tmcSpread = true;
  config.driveSpeedMM = 150.0f;
  prefs.putUShort("tmcMS", config.tmcMicrosteps);
  prefs.putBool("tmcSpread", config.tmcSpread);
  prefs.putFloat("driveSpeed", config.driveSpeedMM);
  Serial.println("[FIX-TEST] 150 mm/s, 64 Microsteps, SpreadCycle (fest verdrahtet)");

  const uint32_t sig = geoSignature();
  const bool     geoChanged = (prefs.getUInt("geoSig", 0) != sig);
  config.quizForwardMM = prefs.getFloat("quizMM", QUIZ_STEP_MM);
  config.quizTurnDeg   = prefs.getFloat("quizDeg", TURN_DEG_DEFAULT);
  if (geoChanged) {
    config.quizForwardMM = QUIZ_STEP_MM;
    config.quizTurnDeg   = TURN_DEG_DEFAULT;
  }
  (void)prefs.getFloat("driveSpeed", 150.0f);

  config.rampMs    = prefs.getUShort("rampMs", RAMP_MS_DEFAULT);
  config.spinSpeed = prefs.getUShort("spinSpeed", SPIN_SPEED_DEFAULT);
  config.spinMs    = prefs.getUShort("spinMs", SPIN_MS_DEFAULT);

  config.blackR        = prefs.getUShort("blackR", 222);
  config.blackG        = prefs.getUShort("blackG", 216);
  config.blackB        = prefs.getUShort("blackB", 146);
  config.whiteR        = prefs.getUShort("whiteR", 4857);
  config.whiteG        = prefs.getUShort("whiteG", 5574);
  config.whiteB        = prefs.getUShort("whiteB", 3811);
  if (prefs.isKey("minSat")) config.minColorSaturation = prefs.getFloat("minSat", 0.40f);
  else { config.minColorSaturation = 0.40f; prefs.putFloat("minSat", config.minColorSaturation); }
  config.obsDistMM = (int16_t)prefs.getInt("obsDist", 100);
  config.obsRelMM  = (int16_t)prefs.getInt("obsRel", 130);
  config.obsOffsetMM = (int16_t)prefs.getInt("obsOff", 0);
  config.obsScale    = prefs.getFloat("obsScale", 1.0f);
  config.calRot.r      = prefs.getUChar("calRotR", 120);
  config.calRot.g      = prefs.getUChar("calRotG", 31);
  config.calRot.b      = prefs.getUChar("calRotB", 33);
  config.calRot.calibrated    = prefs.getBool("calRotSet", true);
  config.calGelb.r     = prefs.getUChar("calGelbR", 234);
  config.calGelb.g     = prefs.getUChar("calGelbG", 212);
  config.calGelb.b     = prefs.getUChar("calGelbB", 104);
  config.calGelb.calibrated   = prefs.getBool("calGelbSet", true);
  config.calGruen.r    = prefs.getUChar("calGruenR", 40);
  config.calGruen.g    = prefs.getUChar("calGruenG", 69);
  config.calGruen.b    = prefs.getUChar("calGruenB", 47);
  config.calGruen.calibrated  = prefs.getBool("calGruenSet", true);
  config.calBlau.r     = prefs.getUChar("calBlauR", 18);
  config.calBlau.g     = prefs.getUChar("calBlauG", 53);
  config.calBlau.b     = prefs.getUChar("calBlauB", 104);
  config.calBlau.calibrated   = prefs.getBool("calBlauSet", true);
  config.edgeGuard      = prefs.getBool("edgeGuard", true);
  config.floorR         = prefs.getUChar("floorR", 0);
  config.floorG         = prefs.getUChar("floorG", 0);
  config.floorB         = prefs.getUChar("floorB", 0);
  config.floorC         = prefs.getUShort("floorC", 0);
  config.floorCalibrated = prefs.getBool("floorSet", false);
  if (config.floorCalibrated && config.floorC == 0) config.floorCalibrated = false;
  if (config.whiteR < 200 || config.whiteG < 200 || config.whiteB < 200) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
  }
    Serial.printf("[NVS] einlernung: vorwaerts=%.1f mm drehung=%.1f Grad "
                "tempo=%.0f mm/s tmc=%u/%u geoSig=0x%08lX (soll 0x%08lX)%s\n",
                (double)config.quizForwardMM, (double)config.quizTurnDeg,
                (double)config.driveSpeedMM,
                config.tmcMicrosteps,
                config.tmcSpread ? 1 : 0,
                (unsigned long)prefs.getUInt("geoSig", 0), (unsigned long)sig,
                geoChanged ? " -> NEU GESETZT" : "");
  Serial.printf("[GEO] %u Schritte/Kreis (%u Vollschritte x %u Microsteps), "
                "Rad %.0f mm = %.1f Schritte/mm -> %.1f mm = %ld Schritte, "
                "%.1f Grad = %ld Schritte\n",
                (unsigned)(MOTOR_FULL_STEPS_PER_REV * config.tmcMicrosteps),
                (unsigned)MOTOR_FULL_STEPS_PER_REV, config.tmcMicrosteps,
                (double)WHEEL_DIAMETER_MM, (double)configStepsPerMM(),
                (double)config.quizForwardMM, (long)configForwardSteps(),
                (double)config.quizTurnDeg, (long)configTurnSteps());
  if (geoChanged) {
    Serial.printf("[GEO] Motor/Rad geaendert (Sig 0x%08lX) -> Einlernung auf "
                  "Werkwerte zurueckgesetzt: %.1f mm = %ld Schritte, %.1f Grad = %ld Schritte\n",
                  (unsigned long)prefs.getUInt("geoSig", 0), (double)config.quizForwardMM,
                  (long)configForwardSteps(), (double)config.quizTurnDeg,
                  (long)configTurnSteps());
    prefs.putFloat("quizMM", config.quizForwardMM);
    prefs.putFloat("quizDeg", config.quizTurnDeg);
    prefs.putUInt("geoSig", sig);
  }
  configValidate();
  prefs.end();
}

void configResetGeometry() {
  config.quizForwardMM = QUIZ_STEP_MM;
  config.quizTurnDeg   = TURN_DEG_DEFAULT;
  prefs.begin("edubot", false);
  prefs.putFloat("quizMM", config.quizForwardMM);
  prefs.putFloat("quizDeg", config.quizTurnDeg);
  prefs.putUInt("geoSig", geoSignature());
  prefs.end();
  Serial.printf("[GEO] Einlernung zurueckgesetzt: %.1f mm = %ld Schritte, "
                "%.1f Grad = %ld Schritte\n",
                (double)config.quizForwardMM, (long)configForwardSteps(),
                (double)config.quizTurnDeg, (long)configTurnSteps());
}

void configValidate() {
  bool changed = false;
  if (!inRange(config.quizForwardMM, FORWARD_MM_MIN, FORWARD_MM_MAX)) {
    config.quizForwardMM = QUIZ_STEP_MM; changed = true;
  }
  if (!inRange(config.quizTurnDeg, TURN_DEG_MIN, TURN_DEG_MAX)) {
    config.quizTurnDeg = TURN_DEG_DEFAULT; changed = true;
  }
  if (!inRange(config.driveSpeedMM, SPEED_MM_S_MIN, SPEED_MM_S_MAX)) {
    config.driveSpeedMM = 150.0f; changed = true;
  }
  if (config.driveSpeedMM > MAX_DRIVE_STEPS_S / configStepsPerMM()) {
    config.driveSpeedMM = floorf(MAX_DRIVE_STEPS_S / configStepsPerMM());
    changed = true;
  }
  if (config.rampMs > RAMP_MS_MAX) {
    config.rampMs = RAMP_MS_DEFAULT; changed = true;
  }
  if (config.spinSpeed > SPIN_SPEED_MAX) { config.spinSpeed = SPIN_SPEED_DEFAULT; changed = true; }
  if (config.spinMs > SPIN_MS_MAX) { config.spinMs = SPIN_MS_DEFAULT; changed = true; }
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }
  if (!inRange(config.minColorSaturation, 0.0f, 1.0f)) {
    config.minColorSaturation = 0.40f;
    changed = true;
  }
  if (config.obsDistMM < 50 || config.obsDistMM > 1200) {
    config.obsDistMM = 100; changed = true;
  }
  if (config.obsRelMM <= config.obsDistMM || config.obsRelMM > 1200) {
    int32_t rel = (int32_t)config.obsDistMM + 30;
    if (rel > 1200) {
      config.obsDistMM = 1170;
      rel = 1200;
      changed = true;
    }
    config.obsRelMM = (int16_t)rel;
    changed = true;
  }
  if (config.obsOffsetMM < -400 || config.obsOffsetMM > 400) {
    config.obsOffsetMM = 0; config.obsScale = 1.0f; changed = true;
  }
  if (!inRange(config.obsScale, 0.5f, 2.0f)) {
    config.obsScale = 1.0f; changed = true;
  }
  if (config.tmcMicrosteps != 1 && config.tmcMicrosteps != 2 &&
      config.tmcMicrosteps != 4 && config.tmcMicrosteps != 8 &&
      config.tmcMicrosteps != 16 && config.tmcMicrosteps != 32 &&
      config.tmcMicrosteps != 64 && config.tmcMicrosteps != 128 &&
      config.tmcMicrosteps != 256) {
    config.tmcMicrosteps = 16;
    changed = true;
  }
  if (changed) {
    prefs.begin("edubot", false);
    prefs.putUShort("blackR", config.blackR);
    prefs.putUShort("blackG", config.blackG);
    prefs.putUShort("blackB", config.blackB);
    prefs.putUShort("whiteR", config.whiteR);
    prefs.putUShort("whiteG", config.whiteG);
    prefs.putUShort("whiteB", config.whiteB);
    prefs.putFloat("minSat", config.minColorSaturation);
  prefs.putInt("obsDist", config.obsDistMM);
  prefs.putInt("obsRel", config.obsRelMM);
    prefs.putInt("obsOff", config.obsOffsetMM);
    prefs.putFloat("obsScale", config.obsScale);
    prefs.putUShort("tmcMS", config.tmcMicrosteps);
    prefs.putBool("tmcSpread", config.tmcSpread);
    prefs.putFloat("quizMM", config.quizForwardMM);
    prefs.putFloat("quizDeg", config.quizTurnDeg);
    prefs.putFloat("driveSpeed", config.driveSpeedMM);
    prefs.putUShort("rampMs", config.rampMs);
    prefs.putUShort("spinSpeed", config.spinSpeed);
    prefs.putUShort("spinMs", config.spinMs);
    prefs.putUInt("geoSig", geoSignature());
    prefs.end();
    Serial.println("[NVS] Invalid config corrected");
  }
}

void configSaveField(const char* key, float value) {
  prefs.begin("edubot", false);
  prefs.putFloat(key, value);
  prefs.end();
}

void configSaveInt(const char* key, int32_t value) {
  prefs.begin("edubot", false);
  prefs.putInt(key, value);
  prefs.end();
}

void configSaveUShort(const char* key, uint16_t value) {
  prefs.begin("edubot", false);
  prefs.putUShort(key, value);
  prefs.end();
}

void configSaveUChar(const char* key, uint8_t value) {
  prefs.begin("edubot", false);
  prefs.putUChar(key, value);
  prefs.end();
}

void configSaveBool(const char* key, bool value) {
  prefs.begin("edubot", false);
  prefs.putBool(key, value);
  prefs.end();
}
