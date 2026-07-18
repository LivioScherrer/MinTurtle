// =====================================================
// EduBot Mesh – Inter-Roboter Kommunikation via ESP-NOW
// =====================================================
// Hardware: Seeed Studio Xiao ESP32-S3
// Protokoll: ESP-NOW (verbindungslos, Broadcast an Peers)
// Nutzung:  Freie Text-Nachrichten zwischen mehreren EduBots
// =====================================================

#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>

namespace EduMesh {

// --- Konstanten ---------------------------------------------------------

constexpr size_t MESH_MAX_PAYLOAD   = 200;   // max Bytes pro Nachricht
constexpr size_t MESH_MAX_PEERS     = 8;     // maximalePeer-Anzahl
constexpr uint32_t MESH_CHANNEL     = 6;     // WiFi Kanal (muss fuer alle gleich sein)
constexpr unsigned long MESH_PEER_EXPIRE_MS = 60000; // Peers ohne Heartbeat entfernen
constexpr unsigned long MESH_HEARTBEAT_MS   = 2000;  // eigene Heartbeat-Rate
constexpr unsigned long MESH_FORWARD_HOP_LIMIT = 4;  // max Weiterleitungs-Spruenge

// Opcodes (1 Byte payload[0])
enum MeshOpcode : uint8_t {
  MESH_OP_HEARTBEAT = 1,   // zwangsveraendert
  MESH_OP_MSG       = 2,   // freie Text-Nachricht
  MESH_OP_CMD       = 3,   // weiterzuleitendes Kommando
  MESH_OP_COLOR     = 4,   // Farb-Ereignis weiterleiten
  MESH_OP_PING      = 5,
  MESH_OP_PONG      = 6
};

// --- Peer-Tabelle -------------------------------------------------------

struct Peer {
  uint8_t mac[6];
  char    name[16];
  unsigned long lastSeen;
  bool    active;
  int     rssi; // RSSI hinzugefügt
};

// --- Callback-Typen -----------------------------------------------------

typedef void (*MeshMessageCallback)(const uint8_t* fromMac, uint8_t opcode, const char* text, size_t len);

// --- API ----------------------------------------------------------------

bool begin(const char* robotName, MeshMessageCallback onMessage);
void loop();
bool sendMsg(const char* text, size_t len);
bool sendCmd(const char* text, size_t len);
bool sendColor(int r, int g, int b, const char* name);
bool sendPing(const uint8_t mac[6]);
size_t peerCount();
void getPeers(Peer* outPeers, size_t maxCount, size_t* outCount);
String macString(const uint8_t mac[6]);

} // namespace EduMesh
