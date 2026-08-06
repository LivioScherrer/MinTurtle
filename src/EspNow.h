// EduBot V1.3 - ESP-NOW (Multiplayer Rennen)
#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

constexpr uint8_t  RENNEN_TARGET_SCORE = 10;
constexpr uint8_t  SLALOM_TARGET_WINS  = 3;
constexpr uint8_t  MAX_PEERS = 4;
constexpr uint8_t  RACE_FLAG_WINNER  = 0x01;
constexpr uint8_t  RACE_FLAG_HELLO   = 0x80;
constexpr uint8_t  RACE_FLAG_PRESENT = 0x40;

// RacePacket: kompatibel zum urspruenglichen 9-Byte-Paket, aber mit
// zusaetzlichem 'slalomCmd' + 'slalomData' fuer den Farben-Slalom.
struct __attribute__((packed)) RacePacket {
  uint8_t magic;
  uint8_t senderMac[6];
  uint8_t score;
  uint8_t flags;
  uint8_t slalomCmd;     // 0 = Rennscore, sonst SLALOM_CMD_*
  uint8_t slalomLen;     // Laenge der slalomData
  char    slalomData[83]; // max. 83 Zeichen fuer Programmdaten / Status
};
static_assert(sizeof(RacePacket) == 94, "RacePacket muss 94 Bytes sein - kein Padding!");

// Slalom-Befehle (ueber ESP-NOW)
constexpr uint8_t SLALOM_CMD_NONE    = 0;
constexpr uint8_t SLALOM_CMD_PROG    = 1;  // Programm (encoded string) vom Gegner
constexpr uint8_t SLALOM_CMD_READY   = 2;  // Gegner ist bereit (Programm empfangen)
constexpr uint8_t SLALOM_CMD_GO      = 3;  // Startsignal (beide bereit)
constexpr uint8_t SLALOM_CMD_FINISH  = 4;  // Gegner hat Farbe getrofffen (Score=Sieg)
constexpr uint8_t SLALOM_CMD_RESET   = 5;  // Neustart/Runde neu
constexpr uint8_t SLALOM_CMD_TARGET  = 6;  // Zielfarbe (Host -> Client) - locked erst nach ACK
constexpr uint8_t SLALOM_CMD_OPP_FINISHED = 7;  // Gegner hat Farbe erreicht (10-Sek-Extension)
constexpr uint8_t SLALOM_CMD_TIMEOUT     = 8;  // Runde abgelaufen (niemand hat Farbe erreicht)
constexpr uint8_t SLALOM_CMD_TARGET_ACK  = 9;  // Client bestaetigt Zielfarbe -> Host broadcastet SLALOM:TARGET
struct PeerEntry {
  uint8_t mac[6];
  uint32_t lastSeen;
};

extern PeerEntry peerList[MAX_PEERS];
extern uint8_t peerCount;
extern SemaphoreHandle_t peerMutex;
// Events aus Wi-Fi-Callback -> Main-Loop (thread-sicher verarbeitet)
struct EspNowEvent {
  enum Type : uint8_t { PEER_ADD, RACE_PKT } type;
  uint8_t mac[6];
  RacePacket pkt;
};
extern QueueHandle_t espNowEventQueue;
extern uint8_t espNowPeer[6];
extern uint8_t pairedPeer[6];
extern bool espNowReady;
extern bool hasPairedPeer;
extern uint8_t myMac[6];

// WiFi/ESP-NOW initialisieren (Broadcast-Peer auf Channel 1)
void espNowInit();
// Channel nach BLE-Aktionen wiederherstellen
void espNowEnsureChannel();
// Queue-Events verarbeiten (in loop() aufrufen, thread-sicher)
void espNowProcessEvents();
// Discovery-Hello broadcast
void raceSendHello();
// Score an Peer senden (winner=true setzt WIN-Flag)
void raceSendPacket(uint8_t score, bool winner);
// Pairing mit spezifischem Peer
void racePairWith(const uint8_t mac[6]);
void raceUnpair();
