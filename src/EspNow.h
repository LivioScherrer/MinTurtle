// EduBot V1.3 - ESP-NOW (Multiplayer Rennen)
#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

constexpr uint8_t  RENNEN_TARGET_SCORE = 10;
constexpr uint8_t  MAX_PEERS = 4;
constexpr uint8_t  RACE_FLAG_WINNER  = 0x01;
constexpr uint8_t  RACE_FLAG_HELLO   = 0x80;
constexpr uint8_t  RACE_FLAG_PRESENT = 0x40;

struct PeerEntry {
  uint8_t mac[6];
  uint32_t lastSeen;
};

extern PeerEntry peerList[MAX_PEERS];
extern uint8_t peerCount;
extern uint8_t espNowPeer[6];
extern uint8_t pairedPeer[6];
extern bool espNowReady;
extern bool hasPairedPeer;
extern uint8_t myMac[6];

struct __attribute__((packed)) RacePacket {
  uint8_t magic;
  uint8_t senderMac[6];
  uint8_t score;
  uint8_t flags;
};
static_assert(sizeof(RacePacket) == 9, "RacePacket muss 9 Bytes sein - kein Padding!");

// WiFi/ESP-NOW initialisieren (Broadcast-Peer auf Channel 1)
void espNowInit();
// Channel nach BLE-Aktionen wiederherstellen
void espNowEnsureChannel();
// Discovery-Hello broadcast
void raceSendHello();
// Score an Peer senden (winner=true setzt WIN-Flag)
void raceSendPacket(uint8_t score, bool winner);
// Pairing mit spezifischem Peer
void racePairWith(const uint8_t mac[6]);
void raceUnpair();
