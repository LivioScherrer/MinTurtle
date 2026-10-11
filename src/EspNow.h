// EduBot V2 - ESP-NOW (Multiplayer Rennen)
#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

// Zielpunktzahl des Rennens. Nur noch der DEFAULT: die Web-UI setzt sie pro
// Stufe ueber RACE:START:<ziel>:<stufe> (RACE_TARGET_MIN/MAX begrenzen sie).
// Ohne Parameter gilt weiterhin 10 -> kompatibel zu aelterer Web-UI/Firmware.
constexpr uint8_t  RENNEN_TARGET_SCORE = 10;
constexpr uint8_t  RACE_TARGET_MIN  = 1;
constexpr uint8_t  RACE_TARGET_MAX  = 99;
// Kein Renn-Schritt des Gegners in dieser Zeit -> Rennen sauber beenden.
// Der Zaehler startet erst mit dem ersten Schritt des Gegners, deshalb darf
// das Fenster grosszuegig sein: 45 s reichen fuer eine knappe Denkpause,
// ohne dass ein abgestuerzter Gegner ewig haengen bleibt.
constexpr uint32_t RACE_PEER_TIMEOUT_MS = 45000;
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

// Slalom-Befehle (ueber ESP-NOW) - vereinfacht, KEINE Punkte.
// Wer zuerst die Zielfarbe erreicht, gewinnt sofort das Spiel.
constexpr uint8_t SLALOM_CMD_NONE    = 0;
constexpr uint8_t SLALOM_CMD_PROG    = 1;  // "Ich bin bereit" + Programm (encoded string)
constexpr uint8_t SLALOM_CMD_GO      = 2;  // Startsignal (Host -> Client), beide fahren los
constexpr uint8_t SLALOM_CMD_START   = 3;  // Spiel starten + Zielfarbe (Host -> Client)
constexpr uint8_t SLALOM_CMD_WIN     = 4;  // Ich habe die Zielfarbe erreicht -> Spielende
constexpr uint8_t SLALOM_CMD_MISS    = 5;  // Mein Durchlauf ist zu Ende, Farbe nicht erreicht
constexpr uint8_t SLALOM_CMD_ABORT   = 6;  // Spiel abgebrochen -> beide zurueck zu PAIRED
constexpr uint8_t SLALOM_CMD_INVITE      = 10; // Einladung (Initiator -> Gegner)
constexpr uint8_t SLALOM_CMD_INVITE_ACK  = 11; // Einladung angenommen (Gegner -> Initiator)
constexpr uint8_t SLALOM_CMD_SLALOM_UNPAIR = 12; // Slalom-spezifisches Unpair (nur dieses Spiel)
// Duell-Kanal (Duel.cpp): Punkte-Austausch fuer die Spiel-Modi (A5/B1/B8/B9/B10).
// Nutzlast in slalomData: "<punkte>;<S|W>" (S = Stand, W = Sieg).
constexpr uint8_t SLALOM_CMD_DUEL        = 21;
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
// Peer fuer beliebigen Zweck (Slalom-Invite) registrieren
void espNowAddSpecificPeer(const uint8_t mac[6]);
