// EduBot V2 - Farbsensor-Implementierung
// Stand: exakte Funktionsweise aus Edu V1.3 (hat funktioniert).
#include "ColorSensor.h"
#include "Config.h"
#include "I2CBus.h"
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
  // Erst pruefen ob ueberhaupt eine Farbe erkannt wird (sehr dunkel)
  int total = r + g + b;
  if (total < 60) return "SCHWARZ";
  // WEISS: alle drei Kanaele hell und annaehernd gleich
  if (r > COLOR_HIGH && g > COLOR_HIGH && b > COLOR_HIGH &&
      abs(r - g) < 60 && abs(g - b) < 60 && abs(r - b) < 60) {
    return "WEISS";
  }
  // dominant channel
  int maxC = r > g ? (r > b ? r : b) : (g > b ? g : b);
  if (maxC < 50) return "UNBEKANNT";  // zu dunkel fuer echte Farbe

  // GELB: rot und gruen hoch, blau deutlich niedriger
  if (r > 100 && g > 100 && b < 80 && r > b * 1.4f && g > b * 1.4f) return "GELB";

  if (r > g * RATIO_RED && r > b * RATIO_RED) return "ROT";
  if (g > r * RATIO_GB && g > b * RATIO_GB) return "GRUEN";
  if (b > r * RATIO_GB && b > g * RATIO_GB) return "BLAU";
  return "UNBEKANNT";
}

void colorSetup() {
  // Bus-Mutex zuerst: colorSetup() und distSetup() teilen sich denselben Bus.
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

void resetI2CBus() {
  // Sperre MUSS halten: nine Clock-Pulse und das Wire.begin() wuerden
  // sonst mitten in einer VL53L0X-Transaktion den Bus umbauen. Ohne
  // Sperre zu arbeiten (Rueckgabewert ignorieren) ist genau der Fehler,
  // den I2CBus eigentlich verhindern soll - und ausgerechnet dann, wenn
  // der Distanzsensor den Bus laenger haelt, wird er praktisch.
  if (!i2cBusLock(200)) {
    return;
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
}

void getCurrentColorName(char* out, size_t outLen) {
  if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(out, (const char*)sensorName, outLen);
    xSemaphoreGive(colorSem);
  } else {
    strlcpy(out, "UNBEKANNT", outLen);
  }
}

// Anzahl der Recovery-Versuche, danach ruht der Sensor (kein Log-Spam und
// kein wiederholtes resetI2CBus(), das den VL53L0X mit zerstoeren wuerde).
static constexpr uint8_t COLOR_RECOVERY_TRIES = 3;

void colorSensorTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(150);
  int failCount = 0;
  uint8_t recoveries = 0;

  while (true) {
    if (colorSensorOk) {
      uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
      // I2C-Sequenz gegen den VL53L0X (Core 1) absichern.
      if (!i2cBusLock(30)) { vTaskDelayUntil(&lastWakeTime, period); continue; }
      tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
      i2cBusUnlock();

      rawSensorR = rawR; rawSensorG = rawG; rawSensorB = rawB; rawSensorC = rawC;

      if (rawR == 0 && rawG == 0 && rawB == 0 && rawC == 0) {
        failCount++;
        if (failCount >= 3) {
          colorSensorOk = false;
          Serial.println("[SENSOR] Farbsensor liefert 0-Werte -> Recovery");
        }
      } else {
        failCount = 0;

        int r = calibrateColor(rawR, config.blackR, config.whiteR);
        int g = calibrateColor(rawG, config.blackG, config.whiteG);
        int b = calibrateColor(rawB, config.blackB, config.whiteB);

        const char* detected = detectColorName(r, g, b);

        // Timeout statt portMAX_DELAY - verhindert Task-Hang
        if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) == pdTRUE) {
          sensorR = r;
          sensorG = g;
          sensorB = b;
          strlcpy((char*)sensorName, detected, sizeof(sensorName));
          xSemaphoreGive(colorSem);
        }
      }
    } else if (recoveries < COLOR_RECOVERY_TRIES) {
      recoveries++;
      Serial.printf("[SENSOR] Recovery-Versuch %d/%d\n", recoveries, COLOR_RECOVERY_TRIES);
      resetI2CBus();
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
      // Recovery erschoepft: nicht weiter den Bus anfassen. Der
      // Distanzsensor bleibt so ueber lebensfaehig.
      vTaskDelay(pdMS_TO_TICKS(2000));
      lastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWakeTime, period);
  }
}
