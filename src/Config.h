// EduBot V1.3 - Config-Modul
// Konfiguration mit NVS-Persistenz und Sanity-Validierung
#pragma once

#include <Arduino.h>
#include <Preferences.h>

// IR-Sensor (Hindernis-Erkennung) auf Pin D8 = GPIO7 (LM393-Modul, aktiv-low)
constexpr uint8_t  IR_PIN            = 7;

struct ColorRef {
  uint8_t r = 0, g = 0, b = 0;
  bool    calibrated = false;
};

struct Config {
  int8_t leftSign     = -1;
  int8_t rightSign    = -1;
  uint32_t commandTimeoutMs = 500;
  int32_t quizForwardSteps  = 42;    // Schritte pro 3cm im Quiz (Half-Step, Rad 37mm)
  int32_t quizTurnSteps     = 88;    // Schritte pro 90-Grad-Drehung (354/4 empirisch, je Rad)
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
  // Mindest-Saettigung fuer warme Farbtoene (Boden-Differenzierung):
  // unterhalb gilt die Messung als zu blass (Holzboden wirkt gelblich)
  // -> UNBEKANNT. Per SAT:-Befehl einstellbar.
  float minColorSaturation = 0.40f;
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
// Direkt speichern (z.B. nach STEPS:-Befehl)
void configSaveField(const char* key, float value);
void configSaveInt(const char* key, int32_t value);
void configSaveUShort(const char* key, uint16_t value);
void configSaveUChar(const char* key, uint8_t value);
void configSaveBool(const char* key, bool value);
