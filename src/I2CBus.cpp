// EduBot V2 - Gemeinsamer I2C-Bus-Lock (Implementierung)
#include "I2CBus.h"

static SemaphoreHandle_t s_bus = nullptr;

void i2cBusInit() {
  if (s_bus) return;
  s_bus = xSemaphoreCreateMutex();
  if (!s_bus) Serial.println("[I2C] FATAL: Bus-Mutex nicht anlegbar");
}

bool i2cBusLock(uint32_t timeoutMs) {
  // Fail-CLOSED: ohne vorhandene Sperre wird der Zugriff verweigert, nicht
  // erlaubt. Vorher stand hier "return true" - ein Sicherheits-Lock, der
  // genau dann Zugriff gewaehrt, wenn er gar nicht existiert. Ein Aufrufer
  // vor i2cBusInit() haette damit ungesperrt pinMode + Wire.begin() gemacht,
  // also genau die Bus-Zerstoerung, die dieses Modul verhindern soll.
  if (!s_bus) return false;
  return xSemaphoreTake(s_bus, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void i2cBusUnlock() {
  if (s_bus) xSemaphoreGive(s_bus);
}
