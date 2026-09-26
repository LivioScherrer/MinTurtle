// EduBot V2 - BLE Manager Implementierung 
#include "BLEManager.h"
#include "ColorSensor.h"
#include <stdarg.h>

BLECharacteristic* statusChar = nullptr;
bool deviceConnected = false;
QueueHandle_t bleEventQueue = nullptr;

// Forward-Declaration: Command-Handler aus main
extern void handleCommand(const char* cmdStr);

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    BleEvent evt = { BLE_EVT_CONNECTED, "" };
    if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
  }
  void onDisconnect(BLEServer* server) override {
    BleEvent evt = { BLE_EVT_DISCONNECTED, "" };
    if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string rxValue = characteristic->getValue().c_str();
    if (!rxValue.empty()) {
      BleEvent evt = { BLE_EVT_COMMAND, "" };
      strlcpy(evt.cmd, rxValue.c_str(), sizeof(evt.cmd));
      if (bleEventQueue) xQueueSend(bleEventQueue, &evt, pdMS_TO_TICKS(10));
    }
  }
};

void bleSetup() {
  bleEventQueue = xQueueCreate(BLE_QUEUE_LENGTH, sizeof(BleEvent));
  if (bleEventQueue == NULL) {
    Serial.println("[BLE] Fehler: Queue konnte nicht erstellt werden!");
  }

  BLEDevice::init("EduBot");
  // MTU aushandeln. Ohne das bleibt der Default 23 Byte, und
  // sendStatus() schneidet jede laengere Zeile ab - die
  // DIST:DIAG:-Zeile (9 Felder, bis ~71 Zeichen) käme unbrauchbar an,
  // genauso alle SLALOM:-/TMC:-Statuszeilen. Der Browser (Web Bluetooth)
  // verhandelt die MTU beim Verbinden mit; 247 ist der uebliche Wert.
  BLEDevice::setMTU(247);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  BLECharacteristic* cmdChar = service->createCharacteristic(
    CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  cmdChar->setCallbacks(new CommandCallbacks());

  statusChar = service->createCharacteristic(
    STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  statusChar->addDescriptor(new BLE2902());
  statusChar->setValue("EduBot V2");

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.println("[BLE] Werbung gestartet (EduBot)");
}

// USB Serial Monitor: gleiche Befehle wie per BLE (non-blocking,
// daher auch in blockierenden Spiel-Moves sicher aufrufbar).
static void serialPoll() {
  static char buf[256];
  static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handleCommand(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // Ueberlauf: Zeile verwerfen
    }
  }
}

void bleProcessEvents() {
  // Forward-Declaration: externe Handler
  extern void onBleConnected();
  extern void onBleDisconnected();

  BleEvent evt;
  while (bleEventQueue && xQueueReceive(bleEventQueue, &evt, 0) == pdTRUE) {
    switch (evt.type) {
      case BLE_EVT_CONNECTED:
        onBleConnected();
        break;
      case BLE_EVT_DISCONNECTED:
        onBleDisconnected();
        break;
      case BLE_EVT_COMMAND:
        handleCommand(evt.cmd);
        break;
    }
  }

  // Befehle vom Serial Monitor (USB) – gleiche Syntax wie per BLE
  serialPoll();
}

void sendStatus(const char* msg) {
  Serial.printf("[BLE] %s\n", msg);
  if (statusChar != nullptr && deviceConnected) {
    statusChar->setValue(msg);
    statusChar->notify();
  }
}

void sendStatusF(const char* fmt, ...) {
  char buf[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  sendStatus(buf);
}

void sendColorToWebsite() {
  int r = 0, g = 0, b = 0;
  char name[16] = {};
  if (colorSem == nullptr) return;
  if (xSemaphoreTake(colorSem, pdMS_TO_TICKS(5)) == pdTRUE) {
    r = sensorR;
    g = sensorG;
    b = sensorB;
    strlcpy(name, (const char*)sensorName, sizeof(name));
    xSemaphoreGive(colorSem);
  } else {
    return;
  }
  char msg[64];
  snprintf(msg, sizeof(msg), "COLOR:%d,%d,%d,%s", r, g, b, name);
  sendStatus(msg);
}
