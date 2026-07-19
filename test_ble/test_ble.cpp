// =====================================================
// EduBot BLE TEST - NUR VERBINDUNG
// =====================================================
// Minimal-Firmware: Nur BLE. Nichts sonst.
// Kein Farbsensor, keine Motoren, keine Queue.
// =====================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CMD_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define STATUS_UUID  "a9b1c2d3-1111-2222-3333-abcdefabcdef"

BLECharacteristic* statusChar;
bool connected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    connected = true;
    Serial.println("[BLE] +++ VERBUNDEN +++");
    statusChar->setValue("VERBUNDEN");
    statusChar->notify();
  }
  void onDisconnect(BLEServer*) override {
    connected = false;
    Serial.println("[BLE] --- GETRENNT ---");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    Serial.printf("[CMD] '%s' (len=%d)\n", v.c_str(), v.length());
    String echo = "ECHO:" + String(v.c_str());
    statusChar->setValue(echo.c_str());
    statusChar->notify();
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EduBot BLE-TEST ===");

  BLEDevice::init("EduBot-Test");

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* cmdChar = service->createCharacteristic(
    CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  cmdChar->setCallbacks(new CmdCallbacks());

  statusChar = service->createCharacteristic(
    STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  statusChar->addDescriptor(new BLE2902());
  statusChar->setValue("BEREIT");

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[BLE] Werbung aktiv. Warte auf Verbindung...");
}

void loop() {
  static unsigned long lastBeat = 0;
  if (connected && millis() - lastBeat > 2000) {
    lastBeat = millis();
    String msg = "PING:" + String(millis() / 1000);
    statusChar->setValue(msg.c_str());
    statusChar->notify();
    Serial.printf("[BLE] Heartbeat: %s\n", msg.c_str());
  }
  delay(10);
}