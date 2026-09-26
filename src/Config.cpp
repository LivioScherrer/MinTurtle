// EduBot V2 - Config-Implementierung
#include "Config.h"

Config config;
Preferences prefs;

void configLoad() {
  prefs.begin("edubot", false);
  config.quizForwardSteps = prefs.getInt("quizSteps", 42);
  config.quizTurnSteps    = prefs.getInt("quizTurnSteps", 88);
  config.blackR        = prefs.getUShort("blackR", 152);
  config.blackG        = prefs.getUShort("blackG", 137);
  config.blackB        = prefs.getUShort("blackB", 98);
  config.whiteR        = prefs.getUShort("whiteR", 4143);
  config.whiteG        = prefs.getUShort("whiteG", 4258);
  config.whiteB        = prefs.getUShort("whiteB", 3036);
  config.minColorSaturation = prefs.getFloat("minSat", 0.40f);
  config.obsDistMM = (int16_t)prefs.getInt("obsDist", 100);
  config.obsRelMM  = (int16_t)prefs.getInt("obsRel", 130);
  config.obsOffsetMM = (int16_t)prefs.getInt("obsOff", 0);
  config.tmcCurrent    = prefs.getUShort("tmcCur", 200);
  config.tmcHold       = prefs.getUChar("tmcHold", 16);
  config.tmcMicrosteps = prefs.getUShort("tmcMS", 16);
  config.tmcSpread     = prefs.getBool("tmcSpread", false);
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
  Serial.printf("[NVS] quizSteps=%ld quizTurnSteps=%ld tmc=%u/%u/%u/%d\n",
                (long)config.quizForwardSteps, (long)config.quizTurnSteps,
                config.tmcCurrent, config.tmcHold, config.tmcMicrosteps,
                config.tmcSpread ? 1 : 0);
  configValidate();
  prefs.end();
}

void configValidate() {
  bool changed = false;
  if (config.quizForwardSteps < 1 || config.quizForwardSteps > 2000) {
    config.quizForwardSteps = 42; changed = true;
  }
  if (config.quizTurnSteps < 1 || config.quizTurnSteps > 5000) {
    config.quizTurnSteps = 88; changed = true;
  }
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }
  if (config.minColorSaturation < 0.0f || config.minColorSaturation > 1.0f) {
    config.minColorSaturation = 0.40f;
    changed = true;
  }
  // Hindernisschwellen: 50..1200 mm (Sensor blind unter ~50mm),
  // Freigabe strikt ueber Stopp (Hysterese).
  if (config.obsDistMM < 50 || config.obsDistMM > 1200) {
    config.obsDistMM = 100; changed = true;
  }
  if (config.obsRelMM <= config.obsDistMM || config.obsRelMM > 1200) {
    config.obsRelMM = (int16_t)(config.obsDistMM + 30); changed = true;
  }
  if (config.obsOffsetMM < -200 || config.obsOffsetMM > 200) {
    config.obsOffsetMM = 0; changed = true;
  }
  if (config.tmcCurrent < 100 || config.tmcCurrent > 800) {
    config.tmcCurrent = 200; changed = true;
  }
  if (config.tmcHold > 31) {
    config.tmcHold = 31; changed = true;
  }
  if (config.tmcMicrosteps != 1 && config.tmcMicrosteps != 2 &&
      config.tmcMicrosteps != 4 && config.tmcMicrosteps != 8 &&
      config.tmcMicrosteps != 16 && config.tmcMicrosteps != 32 &&
      config.tmcMicrosteps != 64 && config.tmcMicrosteps != 128 &&
      config.tmcMicrosteps != 256) {
    config.tmcMicrosteps = 16; changed = true;
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
