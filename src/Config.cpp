// EduBot V1.3 - Config-Implementierung
#include "Config.h"

Config config;
Preferences prefs;
static bool configDirty = false;

void configLoad() {
  prefs.begin("edubot", false);
  config.maxSpeed      = prefs.getFloat("maxSpeed", static_cast<float>(MAX_SPEED_DEFAULT));
  config.turnFactor    = prefs.getFloat("turnFactor", TURN_DEFAULT);
  config.turnOnlySpeed = prefs.getFloat("turnOnlySpeed", 320.0f);
  config.accelRate     = prefs.getFloat("accelRate", ACCEL_DEFAULT);
  config.quizForwardSteps = prefs.getInt("quizSteps", 42);
  config.quizTurnSteps    = prefs.getInt("quizTurnSteps", 88);
  config.blackR        = prefs.getUShort("blackR", 152);
  config.blackG        = prefs.getUShort("blackG", 137);
  config.blackB        = prefs.getUShort("blackB", 98);
  config.whiteR        = prefs.getUShort("whiteR", 4143);
  config.whiteG        = prefs.getUShort("whiteG", 4258);
  config.whiteB        = prefs.getUShort("whiteB", 3036);
  // Schutz: zu kleine white-Werte -> default Kalibrierung
  if (config.whiteR < 200 || config.whiteG < 200 || config.whiteB < 200) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
  }
  Serial.printf("[NVS] maxSpeed=%.0f accelRate=%.0f turnFactor=%.2f\n",
                config.maxSpeed, config.accelRate, config.turnFactor);
  configValidate();
  prefs.end();
}

void configValidate() {
  bool changed = false;
  if (config.maxSpeed < MAX_SPEED_MIN || config.maxSpeed > MAX_SPEED_MAX) {
    config.maxSpeed = static_cast<float>(MAX_SPEED_DEFAULT); changed = true;
  }
  if (config.turnFactor < TURN_MIN || config.turnFactor > TURN_MAX) {
    config.turnFactor = TURN_DEFAULT; changed = true;
  }
  // Migration: alter Default 0.25 (car-like, kaum Drehung) -> 0.0 (Innenrad stoppt)
  if (config.turnFactor == 0.25f) {
    config.turnFactor = TURN_DEFAULT; changed = true;
  }
  if (config.quizForwardSteps < 1 || config.quizForwardSteps > 2000) {
    config.quizForwardSteps = 42; changed = true;
  }
  if (config.quizTurnSteps < 1 || config.quizTurnSteps > 5000) {
    config.quizTurnSteps = 88; changed = true;
  }
  if (config.accelRate < ACCEL_MIN || config.accelRate > ACCEL_MAX) {
    config.accelRate = ACCEL_DEFAULT; changed = true;
  }
  if (config.blackR >= config.whiteR || config.blackG >= config.whiteG || config.blackB >= config.whiteB) {
    config.blackR = 152; config.blackG = 137; config.blackB = 98;
    config.whiteR = 4143; config.whiteG = 4258; config.whiteB = 3036;
    changed = true;
  }
  if (changed) {
    prefs.begin("edubot", false);
    prefs.putFloat("maxSpeed", config.maxSpeed);
    prefs.putFloat("turnFactor", config.turnFactor);
    prefs.putFloat("accelRate", config.accelRate);
    prefs.putUShort("blackR", config.blackR);
    prefs.putUShort("blackG", config.blackG);
    prefs.putUShort("blackB", config.blackB);
    prefs.putUShort("whiteR", config.whiteR);
    prefs.putUShort("whiteG", config.whiteG);
    prefs.putUShort("whiteB", config.whiteB);
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

void configSaveIfDirty() {
  if (!configDirty) return;
  configDirty = false;
  // Platzhalter fuer zukuenftige "dirty"-basierte Persistenz
}
