// EduBot V1.3 - Config-Modul
// Konfiguration mit NVS-Persistenz und Sanity-Validierung
#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Konfigurationsgrenzen (constexpr, hier zentral)
constexpr uint32_t MAX_SPEED_DEFAULT = 380;
constexpr uint32_t MAX_SPEED_MIN     = 200;
constexpr uint32_t MAX_SPEED_MAX     = 2500;
constexpr float    TURN_DEFAULT      = 0.0f;
constexpr float    TURN_MIN          = 0.0f;
constexpr float    TURN_MAX          = 0.9f;
constexpr float    ACCEL_DEFAULT     = 5000.0f;
constexpr float    ACCEL_MIN         = 100.0f;
constexpr float    ACCEL_MAX         = 10000.0f;

struct ColorRef {
  uint8_t r = 0, g = 0, b = 0;
  bool    calibrated = false;
};

struct Config {
  float maxSpeed      = 380.0f;
  float turnFactor    = 0.25f;
  float turnOnlySpeed = 320.0f;
  int8_t leftSign     = -1;
  int8_t rightSign    = -1;
  float accelRate     = 5000.0f;
  uint32_t commandTimeoutMs = 500;
  int32_t quizForwardSteps  = 42;    // Schritte pro 3cm im Quiz (Half-Step, Rad 37mm)
  int32_t quizTurnSteps     = 88;    // Schritte pro 90-Grad-Drehung (354/4 empirisch, je Rad)
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
  ColorRef calRot;
  ColorRef calGelb;
  ColorRef calGruen;
  ColorRef calBlau;
};

extern Config config;
extern Preferences prefs;

// Lade aus NVS, wende Defaults an, validiere
void configLoad();
// Sanity-Check: wenn ausserhalb Bereich -> Default setzen + speichern
void configValidate();
// Direkt speichern (z.B. nach MAX:/TURN:/ACC: Befehl)
void configSaveField(const char* key, float value);
void configSaveInt(const char* key, int32_t value);
void configSaveUShort(const char* key, uint16_t value);
void configSaveUChar(const char* key, uint8_t value);
void configSaveBool(const char* key, bool value);
// Sanftes Speichern wenn dirty
void configSaveIfDirty();
