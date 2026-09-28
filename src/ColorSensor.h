// EduBot V2 - Farbsensor-Modul (TCS34725)
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// I2C-Pins (Xiao ESP32-S3)
#define SDA_PIN 5
#define SCL_PIN 6

// Schwellwerte
// COLOR_LOW ist als "sehr dunkel"-Schwelle deklariert, wird aber nirgends
// ausgewertet: detectColorName() benutzt dafuer das Literal 60. Bewusst so
// belassen, damit eine Aenderung hier nicht unbemerkt das Farbverhalten
// aendert - siehe README / detectColorName().
constexpr uint16_t COLOR_LOW  = 40;
constexpr uint16_t COLOR_HIGH = 200;
constexpr float    RATIO_RED  = 1.35f;
constexpr float    RATIO_GB   = 1.25f;

extern Adafruit_TCS34725 tcs;
extern volatile int sensorR, sensorG, sensorB;
extern volatile uint16_t rawSensorR, rawSensorG, rawSensorB, rawSensorC;
extern volatile char sensorName[16];
extern SemaphoreHandle_t colorSem;
// Wird auf Core 0 vom Farb-Task geschrieben und auf Core 1 gelesen (loop(),
// ProgGame-Hot-Path). Ohne volatile sieht der Compiler die Aenderung nicht
// garantiert - byteweise gibt es kein Abreissen, aber die Sichtbarkeit war
// nicht garantiert.
extern volatile bool colorSensorOk;

// I2C + Sensor initialisieren
void colorSetup();
// FreeRTOS-Task
void colorSensorTask(void* parameter);
// I2C-Bus-Recovery (9-clock-Toggle)
// I2C-Bus per 9 Clock-Pulsen zuruecksetzen und neu initialisieren.
// false = Bus war belegt, es wurde NICHT angefasst (Aufrufer soll den
// Versuch nicht als Fehlversuch werten).
bool resetI2CBus();
// Aktuellen Farbnamen lesen (blocking, mit Timeout)
void getCurrentColorName(char* out, size_t outLen);
// Kalibrierung helper
int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal);
// Farbklassifikation
const char* detectColorName(int r, int g, int b);
