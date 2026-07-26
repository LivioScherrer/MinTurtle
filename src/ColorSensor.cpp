// EduBot V1.3 - Farbsensor-Implementierung
#include "ColorSensor.h"
#include "Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <semphr.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

volatile int sensorR = 0;
volatile int sensorG = 0;
volatile int sensorB = 0;
volatile uint16_t rawSensorR = 0, rawSensorG = 0, rawSensorB = 0, rawSensorC = 0;
volatile char sensorName[16] = "UNBEKANNT";

SemaphoreHandle_t colorSem = nullptr;
bool colorSensorOk = false;

int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal) {
  if (whiteVal <= blackVal) return 0;
  long value = map(static_cast<long>(raw), static_cast<long>(blackVal), static_cast<long>(whiteVal), 0L, 255L);
  value = constrain(value, 0L, 255L);
  return static_cast<int>(value);
}

const char* detectColorName(int r, int g, int b) {
  int total = r + g + b;
  if (total < 60) return "SCHWARZ";
  if (r > COLOR_HIGH && g > COLOR_HIGH && b > COLOR_HIGH &&
      abs(r - g) < 60 && abs(g - b) < 60 && abs(r - b) < 60) {
    return "WEISS";
  }
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";
  // Gelb: R und G beide deutlich hoeher als B, R und G aehnlich hoch.
  // Schwellen bewusst locker, damit auch dunklere / blaustichige Gelbtoene erkannt werden.
  if (r > 70 && g > 70 && b < 100 && r > b * 1.3f && g > b * 1.3f && abs(r - g) < 110) return "GELB";
  if (r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  return "UNBEKANNT";
}

void colorSetup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  colorSensorOk = tcs.begin();
  if (colorSensorOk) {
    Serial.println("[SENSOR] TCS34725 gefunden");
  } else {
    Serial.println("[SENSOR] TCS34725 NICHT gefunden! (Trotzdem gestartet)");
  }
  colorSem = xSemaphoreCreateBinary();
  if (colorSem) xSemaphoreGive(colorSem);
}

void resetI2CBus() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SDA_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void getCurrentColorName(char* out, size_t outLen) {
  if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(out, (const char*)sensorName, outLen);
    xSemaphoreGive(colorSem);
  } else {
    strlcpy(out, "UNBEKANNT", outLen);
  }
}

void colorSensorTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(150);
  int failCount = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
      tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
      rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;

      if (rawR == 0 && rawG == 0 && rawB == 0 && rawC == 0) {
        failCount++;
        if (failCount >= 3) {
          colorSensorOk = false;
        }
      } else {
        failCount = 0;
        int r = calibrateColor(rawR, config.blackR, config.whiteR);
        int g = calibrateColor(rawG, config.blackG, config.whiteG);
        int b = calibrateColor(rawB, config.blackB, config.whiteB);
        const char* detected = detectColorName(r, g, b);
        if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
          sensorR = r;
          sensorG = g;
          sensorB = b;
          strlcpy((char*)sensorName, detected, sizeof(sensorName));
          xSemaphoreGive(colorSem);
        }
      }
    } else {
      Serial.println("[SENSOR] Recovery attempt...");
      resetI2CBus();
      colorSensorOk = tcs.begin();
      if (colorSensorOk) {
        Serial.println("[SENSOR] Recovered!");
        failCount = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
      lastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
