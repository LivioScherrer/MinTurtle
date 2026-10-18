#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

constexpr uint8_t  RENNEN_TARGET_SCORE = 10;
constexpr uint8_t  RACE_TARGET_MIN  = 1;
constexpr uint8_t  RACE_TARGET_MAX  = 99;
constexpr uint32_t RACE_PEER_TIMEOUT_MS = 45000;
constexpr uint8_t  MAX_PEERS = 4;

constexpr uint8_t  FRAME_MAGIC   = 0xA5;
constexpr uint8_t  FRAME_VERSION = 2;
constexpr size_t   FRAME_DATA_MAX = 83;

struct __attribute__((packed)) EduPacket {
  uint8_t magic;
  uint8_t version;
  uint8_t senderMac[6];
  uint8_t seq;
  uint8_t flags;
  uint8_t cmd;
  uint8_t score;
  uint8_t len;
  char    data[FRAME_DATA_MAX];
};
static_assert(sizeof(EduPacket) == 96, "EduPacket muss 96 Bytes sein - kein Padding!");

constexpr uint8_t RACE_FLAG_WINNER   = 0x01;
constexpr uint8_t RACE_FLAG_RELIABLE = 0x08;
constexpr uint8_t RACE_FLAG_PRESENT  = 0x40;
constexpr uint8_t RACE_FLAG_HELLO    = 0x80;

constexpr uint8_t SLALOM_CMD_NONE    = 0;
constexpr uint8_t SLALOM_CMD_PROG    = 1;
constexpr uint8_t SLALOM_CMD_GO      = 2;
constexpr uint8_t SLALOM_CMD_START   = 3;
constexpr uint8_t SLALOM_CMD_WIN     = 4;
constexpr uint8_t SLALOM_CMD_MISS    = 5;
constexpr uint8_t SLALOM_CMD_ABORT   = 6;
constexpr uint8_t SLALOM_CMD_INVITE      = 10;
constexpr uint8_t SLALOM_CMD_INVITE_ACK  = 11;
constexpr uint8_t SLALOM_CMD_SLALOM_UNPAIR = 12;
constexpr uint8_t SLALOM_CMD_DUEL        = 21;

constexpr uint8_t LOBBY_CMD_INVITE   = 30;
constexpr uint8_t LOBBY_CMD_ACCEPT   = 31;
constexpr uint8_t LOBBY_CMD_DECLINE  = 32;
constexpr uint8_t LOBBY_CMD_LEAVE    = 33;
constexpr uint8_t LOBBY_CMD_PING     = 34;
constexpr uint8_t LOBBY_CMD_ACK      = 35;
constexpr uint8_t GAME_CMD_START     = 40;
constexpr uint8_t GAME_CMD_STOP      = 41;

struct PeerEntry {
  uint8_t mac[6];
  uint32_t lastSeen;
};

extern PeerEntry peerList[MAX_PEERS];
extern uint8_t peerCount;
extern SemaphoreHandle_t peerMutex;

struct EspNowEvent {
  enum Type : uint8_t { PEER_ADD, PKT } type;
  uint8_t mac[6];
  EduPacket pkt;
};
extern QueueHandle_t espNowEventQueue;
extern uint8_t espNowPeer[6];
extern uint8_t pairedPeer[6];
extern bool espNowReady;
extern bool hasPairedPeer;
extern uint8_t myMac[6];

bool parseMac(const String& macStr, uint8_t mac[6]);

struct EspNowTxStats {
  uint16_t sent;
  uint16_t acked;
  uint16_t dropped;
  uint16_t dupRx;
};

void espNowInit();
void espNowEnsureChannel();
void espNowProcessEvents();
void espNowTick();
bool espNowPeerAlive();
uint32_t espNowPeerSilentMs();
void espNowGetTxStats(EspNowTxStats& out);

bool espNowSendTo(const uint8_t dest[6], uint8_t cmd, const char* data,
                  uint8_t flags = 0, uint8_t score = 0, bool reliable = false);

void raceSendHello();
void raceSendPacket(uint8_t score, bool winner);
void racePairWith(const uint8_t mac[6]);
void raceUnpair();
void espNowAddSpecificPeer(const uint8_t mac[6]);
