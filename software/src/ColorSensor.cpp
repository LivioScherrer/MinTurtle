#include "ColorSensor.h"
#include "Config.h"
#include "I2CBus.h"
#include "ProgGame.h"
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
volatile bool colorSensorOk = false;

int calibrateColor(uint16_t raw, uint16_t blackVal, uint16_t whiteVal) {
  if (whiteVal <= blackVal) return 0;
  long value = map(static_cast<long>(raw), static_cast<long>(blackVal), static_cast<long>(whiteVal), 0L, 255L);
  value = constrain(value, 0L, 255L);
  return static_cast<int>(value);
}

static float chromaDist(int r, int g, int b, uint8_t fr, uint8_t fg, uint8_t fb) {
  const float sm = (float)(r + g + b);
  const float sr = (float)(fr + fg + fb);
  if (sm < 1.0f || sr < 1.0f) return 99.0f;
  return fabsf((float)r / sm - (float)fr / sr)
       + fabsf((float)g / sm - (float)fg / sr)
       + fabsf((float)b / sm - (float)fb / sr);
}

const char* detectColorName(int r, int g, int b) {
  int total = r + g + b;
  if (total < 60) return "SCHWARZ";
  if (r > COLOR_HIGH && g > COLOR_HIGH && b > COLOR_HIGH &&
      abs(r - g) < 60 && abs(g - b) < 60 && abs(r - b) < 60) {
    return "WEISS";
  }
  const char* const refNames[4] = { "ROT", "GELB", "GRUEN", "BLAU" };
  const ColorRef* const refPtrs[4] = { &config.calRot, &config.calGelb,
                                       &config.calGruen, &config.calBlau };
  const char* bestRef = nullptr;
  float bestRefD = 99.0f;
  for (int i = 0; i < 4; i++) {
    if (!refPtrs[i]->calibrated) continue;
    const float d = chromaDist(r, g, b, refPtrs[i]->r, refPtrs[i]->g, refPtrs[i]->b);
    if (d < bestRefD) { bestRefD = d; bestRef = refNames[i]; }
  }
  float floorD = 99.0f;
  if (config.floorCalibrated)
    floorD = chromaDist(r, g, b, config.floorR, config.floorG, config.floorB);
  const bool refHit = (bestRef != nullptr && bestRefD <= 0.18f);
  const bool floorHit = (floorD <= 0.15f);
  if (floorHit && refHit) {
    if (bestRefD + 0.05f < floorD) return bestRef;
    return "BODEN";
  }
  if (refHit) return bestRef;
  if (floorHit) return "BODEN";
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";

  const int minC = r < g ? (r < b ? r : b) : (g < b ? g : b);
  const float sat = (float)(maxC - minC) / (float)maxC;
  const bool warmEnough = (sat >= config.minColorSaturation);

  if (warmEnough && r > 100 && g > 100 && b < r * 0.60f &&
      r > b * 1.4f && g > b * 1.4f) return "GELB";

  if (warmEnough && r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  return "UNBEKANNT";
}

void colorSetup() {
  i2cBusInit();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!i2cBusLock(100)) {
    Serial.println("[SENSOR] I2C-Bus belegt -> colorSetup übersprungen");
    colorSem = xSemaphoreCreateBinary();
    if (colorSem) xSemaphoreGive(colorSem);
    return;
  }
  colorSensorOk = tcs.begin();
  i2cBusUnlock();
  if (colorSensorOk) {
    Serial.println("[SENSOR] TCS34725 gefunden");
  } else {
    Serial.println("[SENSOR] TCS34725 NICHT gefunden! (Trotzdem gestartet)");
  }
  colorSem = xSemaphoreCreateBinary();
  if (colorSem) xSemaphoreGive(colorSem);
}

bool resetI2CBus() {
  if (!i2cBusLock(200)) {
    Serial.println("[SENSOR] resetI2CBus: I2C-Bus belegt -> Bus nicht angefasst");
    return false;
  }
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
  i2cBusUnlock();
  return true;
}

void getCurrentColorName(char* out, size_t outLen) {
  if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(out, (const char*)sensorName, outLen);
    xSemaphoreGive(colorSem);
  } else {
    strlcpy(out, "UNBEKANNT", outLen);
  }
}

static constexpr uint8_t COLOR_RECOVERY_TRIES = 3;

static bool colorSensorAlive() {
  if (!i2cBusLock(30)) return true;
  uint8_t id = 0;
  Wire.beginTransmission(TCS34725_ADDRESS);
  Wire.write(TCS34725_COMMAND_BIT | 0x12);
  bool ok = (Wire.endTransmission(false) == 0);
  if (ok) ok = (Wire.requestFrom((int)TCS34725_ADDRESS, (int)1) == 1);
  if (ok) id = Wire.read();
  i2cBusUnlock();
  return ok && (id == 0x44 || id == 0x4D);
}

void colorSensorTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(150);
  int failCount = 0;
  uint8_t recoveries = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
      if (!i2cBusLock(30)) { vTaskDelayUntil(&lastWakeTime, period); continue; }
      tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
      i2cBusUnlock();

      if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
        rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;
        xSemaphoreGive(colorSem);
      }

      if (rawR == 0 && rawG == 0 && rawB == 0 && rawC == 0) {
        if (colorSensorAlive()) {
          failCount = 0;
          if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
            sensorR = 0; sensorG = 0; sensorB = 0;
            strlcpy((char*)sensorName, "SCHWARZ", sizeof(sensorName));
            xSemaphoreGive(colorSem);
          }
        } else {
          failCount++;
          if (failCount >= 3) {
            colorSensorOk = false;
            Serial.println("[SENSOR] Farbsensor antwortet nicht (Device-ID weg) -> Recovery");
          }
        }
      } else {
        failCount = 0;
        edgeGuardTick(rawC);

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
    } else if (recoveries < COLOR_RECOVERY_TRIES) {
      if (!resetI2CBus()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, period);
        continue;
      }
      recoveries++;
      Serial.printf("[SENSOR] Recovery-Versuch %d/%d\n", recoveries, COLOR_RECOVERY_TRIES);
      if (i2cBusLock(50)) {
        colorSensorOk = tcs.begin();
        i2cBusUnlock();
      }
      if (colorSensorOk) {
        Serial.println("[SENSOR] Recovered!");
        failCount = 0;
        recoveries = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      lastWakeTime = xTaskGetTickCount();
    } else {
      vTaskDelay(pdMS_TO_TICKS(2000));
      lastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
