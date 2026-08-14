// EduBot V1.3 - LED-Streifen-Modul (WS2812, 7 LEDs an GPIO 44)
// Boot-Pattern, Status-LEDs und Zielfarben-Anzeige
#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_STRIP_PIN   44
#define LED_STRIP_COUNT 7

extern Adafruit_NeoPixel ledStrip;
extern volatile bool ledAlertActive;

// Setup: NeoPixel init + Boot-Pattern (blockierend, ~2s)
void ledStripSetup();
// In loop() aufrufen: verarbeitet Alert-Flashes und Status
void ledStripTick();
// Zielfarbe (Name wie "ROT", "GELB", "GRUEN", "BLAU", "WEISS", "SCHWARZ") zeigen
void ledStripShowTarget(const char* colorName);
// Status: blau = BLE verbunden, gruen = bereit, sonst aus
void ledStripShowStatus(bool bleConnected);
// Rot blinken (Notfall-Stop / Fehler), wird nach ledAlertDurationMs deaktiviert
void ledStripShowAlert();
// Alle LEDs ausschalten
void ledStripClear();
