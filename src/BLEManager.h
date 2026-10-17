#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/queue.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CMD_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define STATUS_UUID  "a9b1c2d3-1111-2222-3333-abcdefabcdef"

constexpr int BLE_QUEUE_LENGTH = 32;

enum BleEventType : uint8_t {
  BLE_EVT_CONNECTED,
  BLE_EVT_DISCONNECTED,
  BLE_EVT_COMMAND,
  BLE_EVT_WEB_CONNECTED,
  BLE_EVT_WEB_DISCONNECTED
};

struct BleEvent {
  BleEventType type;
  char cmd[256];
};

extern BLECharacteristic* statusChar;
extern bool deviceConnected;
extern QueueHandle_t bleEventQueue;

bool anyClientConnected();

void bleSetup();
void bleProcessEvents();
void sendStatus(const char* msg);
void sendStatusF(const char* fmt, ...);
void sendColorToWebsite();
