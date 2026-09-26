// EduBot V2 - Gemeinsamer I2C-Bus-Lock (Implementierung)
#include "I2CBus.h"

static SemaphoreHandle_t s_bus = nullptr;

void i2cBusInit() {
  if (s_bus) return;
  s_bus = xSemaphoreCreateMutex();
}

bool i2cBusLock(uint32_t timeoutMs) {
  if (!s_bus) return true;   // noch nicht initialisiert: nicht blockieren
  return xSemaphoreTake(s_bus, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void i2cBusUnlock() {
  if (s_bus) xSemaphoreGive(s_bus);
}
