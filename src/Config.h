// EduBot V1.3 - Config-Modul
// Konfiguration mit NVS-Persistenz und Sanity-Validierung
#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Konfigurationsgrenzen (constexpr, hier zentral)
constexpr uint32_t MAX_SPEED_DEFAULT = 900;
constexpr uint32_t MAX_SPEED_MIN     = 200;
constexpr uint32_t MAX_SPEED_MAX     = 2500;
constexpr float    TURN_DEFAULT      = 0.25f;
constexpr float    TURN_MIN          = 0.0f;
constexpr float    TURN_MAX          = 0.9f;
constexpr float    ACCEL_DEFAULT     = 5000.0f;
constexpr float    ACCEL_MIN         = 100.0f;
constexpr float    ACCEL_MAX         = 10000.0f;

struct Config {
  float maxSpeed      = 900.0f;
  float turnFactor    = 0.25f;
  float turnOnlySpeed = 550.0f;
  int8_t leftSign     = -1;
  int8_t rightSign    = -1;
  float accelRate     = 5000.0f;
  uint32_t commandTimeoutMs = 500;
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
};

extern Config config;
extern Preferences prefs;

// Lade aus NVS, wende Defaults an, validiere
void configLoad();
// Sanity-Check: wenn ausserhalb Bereich -> Default setzen + speichern
void configValidate();
// Direkt speichern (z.B. nach MAX:/TURN:/ACC: Befehl)
void configSaveField(const char* key, float value);
void configSaveUShort(const char* key, uint16_t value);
// Sanftes Speichern wenn dirty
void configSaveIfDirty();
