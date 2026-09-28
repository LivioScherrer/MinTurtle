// EduBot V2 - Config-Implementierung
#include "Config.h"

Config config;
Preferences prefs;

// ---- Umrechnung physikalisch <-> Schritte (Microsteps stecken nur hier drin) ----
float configStepsPerMM() {
  return (float)MOTOR_FULL_STEPS_PER_REV * (float)config.tmcMicrosteps / WHEEL_CIRCUMFERENCE_MM;
}
int32_t configForwardSteps() { return geoStepsForMM(config.quizForwardMM, config.tmcMicrosteps); }
int32_t configTurnSteps()    { return geoStepsForDeg(config.quizTurnDeg, config.tmcMicrosteps); }
float configStepsPerS() {
  float sps = config.driveSpeedMM * configStepsPerMM();
  // NaN-sicher klemmen: bei NaN waere "sps > MAX" false, also liefe NaN
  // durch und landete als setMaxSpeed(NaN) im AccelStepper (_maxSpeed = 0)
  // - der Motor stuende dann nie, ohne dass irgendwo ein Fehler sichtbar wird.
  if (!(sps >= 0.0f)) return 0.0f;
  return sps > MAX_DRIVE_STEPS_S ? MAX_DRIVE_STEPS_S : sps;
}

void configLoad() {
  prefs.begin("edubot", false);
  // TMC-Werte ZUERST: die Microsteps gehen in jede Umrechnung ein.
  config.tmcCurrent    = prefs.getUShort("tmcCur", 200);
  config.tmcHold       = prefs.getUChar("tmcHold", 16);
  config.tmcMicrosteps = prefs.getUShort("tmcMS", 16);
  config.tmcSpread     = prefs.getBool("tmcSpread", false);

  // Einlernung in mm/Grad. Signatur nur aus Motor + Rad: Microsteps sind
  // ABSICHTLICH nicht Teil davon, denn sie aendern nur die Schrittzahl, nicht
  // die Strecke. Weicht die Signatur ab, stammt die Einlernung noch von
  // anderer Hardware (Rad-/Motortausch) -> verwerfen.
  const uint32_t sig = geoSignature();
  const bool     geoChanged = (prefs.getUInt("geoSig", 0) != sig);
  config.quizForwardMM = prefs.getFloat("quizMM", QUIZ_STEP_MM);
  config.quizTurnDeg   = prefs.getFloat("quizDeg", TURN_DEG_DEFAULT);
  if (geoChanged) {
    config.quizForwardMM = QUIZ_STEP_MM;
    config.quizTurnDeg   = TURN_DEG_DEFAULT;
  }
  if (prefs.isKey("driveSpeed")) config.driveSpeedMM = prefs.getFloat("driveSpeed", 150.0f);
  else { config.driveSpeedMM = 150.0f; prefs.putFloat("driveSpeed", config.driveSpeedMM); }

  config.blackR        = prefs.getUShort("blackR", 152);
  config.blackG        = prefs.getUShort("blackG", 137);
  config.blackB        = prefs.getUShort("blackB", 98);
  config.whiteR        = prefs.getUShort("whiteR", 4143);
  config.whiteG        = prefs.getUShort("whiteG", 4258);
  config.whiteB        = prefs.getUShort("whiteB", 3036);
  if (prefs.isKey("minSat")) config.minColorSaturation = prefs.getFloat("minSat", 0.40f);
  else { config.minColorSaturation = 0.40f; prefs.putFloat("minSat", config.minColorSaturation); }
  config.obsDistMM = (int16_t)prefs.getInt("obsDist", 100);
  config.obsRelMM  = (int16_t)prefs.getInt("obsRel", 130);
  config.obsOffsetMM = (int16_t)prefs.getInt("obsOff", 0);
  config.calRot.r      = prefs.getUChar("calRotR", 0);
  config.calRot.g      = prefs.getUChar("calRotG", 0);
  config.calRot.b      = prefs.getUChar("calRotB", 0);
  config.calRot.calibrated    = prefs.getBool("calRotSet", false);
  config.calGelb.r     = prefs.getUChar("calGelbR", 0);
  config.calGelb.g     = prefs.getUChar("calGelbG", 0);
  config.calGelb.b     = prefs.getUChar("calGelbB", 0);
  config.calGelb.calibrated   = prefs.getBool("calGelbSet", false);
  config.calGruen.r    = prefs.getUChar("calGruenR", 0);
  config.calGruen.g    = prefs.getUChar("calGruenG", 0);
  config.calGruen.b    = prefs.getUChar("calGruenB", 0);
  config.calGruen.calibrated  = prefs.getBool("calGruenSet", false);
  config.calBlau.r     = prefs.getUChar("calBlauR", 0);
  config.calBlau.g     = prefs.getUChar("calBlauG", 0);
  config.calBlau.b     = prefs.getUChar("calBlauB", 0);
  config.calBlau.calibrated   = prefs.getBool("calBlauSet", false);
  // Schutz: zu kleine white-Werte -> default Kalibrierung
  if (config.whiteR < 200 || config.whiteG < 200 || config.whiteB < 200) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
  }
    Serial.printf("[NVS] einlernung: vorwaerts=%.1f mm drehung=%.1f Grad "
                "tempo=%.0f mm/s tmc=%u/%u/%u/%d geoSig=0x%08lX (soll 0x%08lX)%s\n",
                (double)config.quizForwardMM, (double)config.quizTurnDeg,
                (double)config.driveSpeedMM,
                config.tmcCurrent, config.tmcHold, config.tmcMicrosteps,
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
    // Einlernung von der alten Hardware verwerfen und die Werkwerte speichern.
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
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }
  if (!inRange(config.minColorSaturation, 0.0f, 1.0f)) {
    config.minColorSaturation = 0.40f;
    changed = true;
  }
  // Hindernisschwellen: 50..1200 mm (Sensor blind unter ~50mm),
  // Freigabe strikt ueber Stopp (Hysterese).
  if (config.obsDistMM < 50 || config.obsDistMM > 1200) {
    config.obsDistMM = 100; changed = true;
  }
  if (config.obsRelMM <= config.obsDistMM || config.obsRelMM > 1200) {
    // Der Nachlauf muss in den Bereich passen UND ueber dem Stopp-Wert
    // bleiben. Bei einem Stopp-Wert nahe 1200 ist obsDistMM+30 zu gross -
    // dann den Stopp-Wert so weit absenken, dass 30 mm Abstand passen.
    int32_t rel = (int32_t)config.obsDistMM + 30;
    if (rel > 1200) {
      config.obsDistMM = 1170;
      rel = 1200;
      changed = true;
    }
    config.obsRelMM = (int16_t)rel;
    changed = true;
  }
  if (config.obsOffsetMM < -200 || config.obsOffsetMM > 200) {
    config.obsOffsetMM = 0; changed = true;
  }
  // Motorgstrom: 100..200 mA, harte Obergrenze = Nennstrom des NEMA8.
  // Wichtig: der Bereich MUSS mit TMC_CURRENT_MA_MIN/MAX aus Motor.h
  // uebereinstimmen. Stand hier vorher "100..800", dann waere ein per altem
  // Build gespeicherter 800-mA-Wert weiterhin gueltig - der Befehlsweg
  // (tmcSetCurrent) lehnt zwar ab, aber die NVS haette den Wert behalten und
  // initSteppers() haette ihn beim Boot erneut eingelesen. Genau damit waere
  // die Grenze nicht "echt" gewesen, sondern nur im UI.
  if (config.tmcCurrent < TMC_CURRENT_MA_MIN || config.tmcCurrent > TMC_CURRENT_MA_MAX) {
    config.tmcCurrent = TMC_CURRENT_MA_MAX; changed = true;
  }
  if (config.tmcHold > 31) {
    config.tmcHold = 31; changed = true;
  }
  if (config.tmcMicrosteps != 1 && config.tmcMicrosteps != 2 &&
      config.tmcMicrosteps != 4 && config.tmcMicrosteps != 8 &&
      config.tmcMicrosteps != 16 && config.tmcMicrosteps != 32 &&
      config.tmcMicrosteps != 64 && config.tmcMicrosteps != 128 &&
      config.tmcMicrosteps != 256) {
    // Ungueltige Microsteps -> Default. Abhaengige Schrittzahlen werden
    // nirgends gespeichert, also gibt es hier nichts mitzuziehen.
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
    prefs.putUShort("tmcCur", config.tmcCurrent);
    prefs.putUChar("tmcHold", config.tmcHold);
    prefs.putUShort("tmcMS", config.tmcMicrosteps);
    prefs.putBool("tmcSpread", config.tmcSpread);
    prefs.putFloat("quizMM", config.quizForwardMM);
    prefs.putFloat("quizDeg", config.quizTurnDeg);
    prefs.putFloat("driveSpeed", config.driveSpeedMM);
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
