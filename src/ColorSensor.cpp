// EduBot V1.3 - Farbsensor-Implementierung
#include "ColorSensor.h"
#include "Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <semphr.h>
#include <math.h>

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

  struct HSV { float h; float s; float v; };
  auto toHsv = [](int ri, int gi, int bi) -> HSV {
    float rf = ri / 255.0f, gf = gi / 255.0f, bf = bi / 255.0f;
    float cmax = rf > gf ? (rf > bf ? rf : bf) : (gf > bf ? gf : bf);
    float cmin = rf < gf ? (rf < bf ? rf : bf) : (gf < bf ? gf : bf);
    float delta = cmax - cmin;
    HSV o;
    o.v = cmax;
    o.s = (cmax > 0.0001f) ? (delta / cmax) : 0.0f;
    if (delta < 0.0001f) { o.h = 0.0f; return o; }
    if (cmax == rf) {
      o.h = 60.0f * fmodf((gf - bf) / delta, 6.0f);
      if (o.h < 0) o.h += 360.0f;
    } else if (cmax == gf) {
      o.h = 60.0f * ((bf - rf) / delta + 2.0f);
    } else {
      o.h = 60.0f * ((rf - gf) / delta + 4.0f);
    }
    return o;
  };
  auto hueDist = [](float a, float b) -> float {
    float d = fabsf(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
  };

  HSV cur = toHsv(r, g, b);
  if (cur.v < 0.15f) return "SCHWARZ";
  if (cur.s < 0.15f && cur.v > 0.6f) return "WEISS";

  struct Cand { const char* name; const ColorRef* ref; };
  const Cand cands[] = {
    { "ROT",   &config.calRot   },
    { "GELB",  &config.calGelb  },
    { "GRUEN", &config.calGruen },
    { "BLAU",  &config.calBlau  },
  };
  const char* bestName = nullptr;
  float      bestDist  = 1e9f;
  for (const auto& c : cands) {
    if (!c.ref->calibrated) continue;
    HSV ref = toHsv(c.ref->r, c.ref->g, c.ref->b);
    if (ref.s < 0.10f) continue;
    float d = hueDist(cur.h, ref.h) + 0.3f * fabsf(cur.s - ref.s) * 50.0f;
    if (d < bestDist) { bestDist = d; bestName = c.name; }
  }

  if (bestName) {
    if (bestDist > 35.0f) return "UNBEKANNT";
    return bestName;
  }

  // Keine Farbe kalibriert -> Default-Hue-Heuristik als Fallback.
  float h = cur.h;
  if (h < 15.0f || h >= 345.0f) return "ROT";
  if (h < 70.0f)  return "GELB";
  if (h < 170.0f) return "GRUEN";
  if (h < 260.0f) return "BLAU";
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
  colorSem = xSemaphoreCreateMutex();
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
      // rawSensor* sind volatile (gelesen ausserhalb des Tasks ohne Semaphore)
      // -> atomar ueber Mutex setzen
      if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
        rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;
        xSemaphoreGive(colorSem);
      }

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
