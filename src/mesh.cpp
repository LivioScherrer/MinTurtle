// =====================================================
// EduBot Mesh – Inter-Roboter Kommunikation via ESP-NOW
// =====================================================
// Hardware: Seeed Studio Xiao ESP32-S3
// Protokoll: ESP-NOW (verbindungslos, Broadcast an Peers)
// Nutzung:  Freie Text-Nachrichten zwischen mehreren EduBots
// =====================================================

#include "mesh.h"
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_now.h>

namespace EduMesh {

// --- Interne Zustände --------------------------------------------------

static char s_robotName[16] = "EduBot";
static MeshMessageCallback s_onMessage = nullptr;
static Peer s_peers[MESH_MAX_PEERS];
static unsigned long s_lastHeartbeat = 0;
static unsigned long s_lastPeerSweep = 0;

// --- Hilfsfunktionen ----------------------------------------------------

String macString(const uint8_t mac[6]) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static Peer* findPeer(const uint8_t mac[6]) {
  for (size_t i = 0; i < MESH_MAX_PEERS; i++) {
    if (!s_peers[i].active) continue;
    if (memcmp(s_peers[i].mac, mac, 6) == 0) return &s_peers[i];
  }
  return nullptr;
}

static Peer* addPeer(const uint8_t mac[6], const char* name) {
  Peer* p = findPeer(mac);
  if (p) {
    p->lastSeen = millis();
    return p;
  }
  for (size_t i = 0; i < MESH_MAX_PEERS; i++) {
    if (!s_peers[i].active) {
      memcpy(s_peers[i].mac, mac, 6);
      if (name && name[0]) strlcpy(s_peers[i].name, name, sizeof(s_peers[i].name));
      s_peers[i].lastSeen = millis();
      s_peers[i].active   = true;
      return &s_peers[i];
    }
  }
  return nullptr;
}

size_t peerCount() {
  size_t n = 0;
  for (size_t i = 0; i < MESH_MAX_PEERS; i++) if (s_peers[i].active) n++;
  return n;
}

void getPeers(Peer* outPeers, size_t maxCount, size_t* outCount) {
  size_t n = 0;
  for (size_t i = 0; i < MESH_MAX_PEERS && n < maxCount; i++) {
    if (s_peers[i].active) {
      memcpy(&outPeers[n], &s_peers[i], sizeof(Peer));
      n++;
    }
  }
  if (outCount) *outCount = n;
}

// --- Sende-Hilfsfunktion ----------------------------------------------

static bool sendRaw(const uint8_t* destMac, uint8_t opcode, const uint8_t* payload, size_t len) {
  if (len > MESH_MAX_PAYLOAD - 1) len = MESH_MAX_PAYLOAD - 1;

  uint8_t buf[MESH_MAX_PAYLOAD];
  buf[0] = opcode;
  if (len > 0) memcpy(&buf[1], payload, len);
  size_t total = 1 + len;

  esp_err_t result = esp_now_send(destMac, buf, total);
  if (result != ESP_OK) {
    Serial.printf("[MESH] sende fehlgeschlagen op=%u err=%d\n", opcode, result);
    return false;
  }
  return true;
}

bool sendMsg(const char* text, size_t len) {
  return sendRaw((const uint8_t*)"\xff\xff\xff\xff\xff\xff", MESH_OP_MSG, (const uint8_t*)text, len);
}

bool sendCmd(const char* text, size_t len) {
  return sendRaw((const uint8_t*)"\xff\xff\xff\xff\xff\xff", MESH_OP_CMD, (const uint8_t*)text, len);
}

bool sendColor(int r, int g, int b, const char* name) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%d,%d,%d,%s", r, g, b, name ? name : "");
  return sendRaw((const uint8_t*)"\xff\xff\xff\xff\xff\xff", MESH_OP_COLOR, (const uint8_t*)buf, strlen(buf));
}

bool sendPing(const uint8_t mac[6]) {
  if (mac == nullptr) mac = (const uint8_t*)"\xff\xff\xff\xff\xff\xff";
  return sendRaw(mac, MESH_OP_PING, (const uint8_t*)s_robotName, strlen(s_robotName));
}

// --- Peer-Registrierung (ESP-NOW) --------------------------------------

static bool ensurePeerRegistered(const uint8_t mac[6]) {
  if (findPeer(mac) != nullptr) return true; // schon in der Peer-Tabelle
  if (esp_now_is_peer_exist(mac)) return true; // schon bei ESP-NOW registriert

  esp_now_peer_info_t info = {};
  memcpy(info.peer_addr, mac, 6);
  info.channel = MESH_CHANNEL;
  info.encrypt = false;
  esp_err_t err = esp_now_add_peer(&info);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("[MESH] add_peer fehlgeschlagen: %d\n", err);
    return false;
  }
  return true;
}

// --- ESP-NOW Empfangs-Callback -----------------------------------------

static void IRAM_ATTR onDataRecvStatic(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1 || mac == nullptr || data == nullptr) return;

  uint8_t opcode = data[0];
  const uint8_t* payload = &data[1];
  size_t payloadLen = static_cast<size_t>(len) - 1;

  // Peer registrieren (Broadcast-Empfaenger immer vorhanden)
  ensurePeerRegistered(mac);
  
  // Heartbeat enthaelt den Roboter-Namen im payload
  char peerName[16] = {};
  if ((opcode == MESH_OP_HEARTBEAT) && payloadLen > 0 && payloadLen < sizeof(peerName)) {
    memcpy(peerName, payload, payloadLen);
    peerName[payloadLen] = '\0';
  }

  Peer* p = addPeer(mac, peerName);
  if (p) {
    // RSSI in dieser API-Version nicht direkt verfuegbar.
    // Wir nutzen einen Platzhalter: Empfang = Peer in Reichweite.
    p->rssi = -60; // Approximation: Empfangen => ca. -60 dBm Bereich
  }

  // User-Callback
  if (s_onMessage) {
    char text[MESH_MAX_PAYLOAD] = {};
    size_t copyLen = (payloadLen < sizeof(text) - 1) ? payloadLen : sizeof(text) - 1;
    memcpy(text, payload, copyLen);
    text[copyLen] = '\0';
    s_onMessage(mac, opcode, text, copyLen);
  }
}

// --- Send-Callback (optional Logging) ----------------------------------

static void onDataSentStatic(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[MESH] send status fail\n");
  }
}

// --- Loop / Heartbeat --------------------------------------------------

void loop() {
  unsigned long now = millis();

  // Heartbeat (Broadcast an alle)
  if (now - s_lastHeartbeat >= MESH_HEARTBEAT_MS) {
    s_lastHeartbeat = now;
    sendRaw((const uint8_t*)"\xff\xff\xff\xff\xff\xff", MESH_OP_HEARTBEAT,
            (const uint8_t*)s_robotName, strlen(s_robotName));
  }

  // Abgelaufene Peers entfernen
  if (now - s_lastPeerSweep >= 5000) {
    s_lastPeerSweep = now;
    for (size_t i = 0; i < MESH_MAX_PEERS; i++) {
      if (!s_peers[i].active) continue;
      if (now - s_peers[i].lastSeen > MESH_PEER_EXPIRE_MS) {
        Serial.printf("[MESH] Peer entfernt: %s\n", macString(s_peers[i].mac).c_str());
        if (esp_now_is_peer_exist(s_peers[i].mac)) {
          esp_now_del_peer(s_peers[i].mac);
        }
        s_peers[i].active = false;
        s_peers[i].name[0] = '\0';
      }
    }
  }
}

// --- begin -------------------------------------------------------------

bool begin(const char* robotName, MeshMessageCallback onMessage) {
  strlcpy(s_robotName, robotName ? robotName : "EduBot", sizeof(s_robotName));
  s_onMessage = onMessage;
  memset(s_peers, 0, sizeof(s_peers));

  // WiFi.mode() wird vor BLE init in Edu_V1.3.cpp gesetzt
  // Nur Kanal setzen - WiFi ist bereits im STA-Modus
  esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[MESH] esp_now_init fehlgeschlagen");
    return false;
  }
  esp_now_register_recv_cb(onDataRecvStatic);
  esp_now_register_send_cb(onDataSentStatic);

  // Broadcast-Peer anlegen, damit wir senden koennen
  esp_now_peer_info_t broadcast = {};
  memset(broadcast.peer_addr, 0xFF, 6);
  broadcast.channel = MESH_CHANNEL;
  broadcast.encrypt = false;
  esp_now_add_peer(&broadcast);

  String mac = WiFi.macAddress();
  Serial.printf("[MESH] gestartet: name=%s channel=%u mac=%s\n",
                s_robotName, MESH_CHANNEL, mac.c_str());
  return true;
}

} // namespace EduMesh
