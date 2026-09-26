// EduBot V2 - Farbsensor-Modul (TCS34725)
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// I2C-Pins (Xiao ESP32-S3)
#define SDA_PIN 5
#define SCL_PIN 6

// Schwellwerte
constexpr uint16_t COLOR_LOW  = 40;
constexpr uint16_t COLOR_HIGH = 200;
constexpr float    RATIO_RED  = 1.35f;
constexpr float    RATIO_GB   = 1.25f;

extern Adafruit_TCS34725 tcs;
extern volatile int sensorR, sensorG, sensorB;
extern volatile uint16_t rawSensorR, rawSensorG, rawSensorB, rawSensorC;
extern volatile char sensorName[16];
extern SemaphoreHandle_t colorSem;
extern bool colorSensorOk;

// I2C + Sensor initialisieren
void colorSetup();
// FreeRTOS-Task
void colorSensorTask(void* parameter);
// I2C-Bus-Recovery (9-clock-Toggle)
void resetI2CBus();
// Aktuellen Farbnamen lesen (blocking, mit Timeout)
void getCurrentColorName(char* out, size_t outLen);
// Kalibrierung helper
int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal);
// Farbklassifikation
const char* detectColorName(int r, int g, int b);
