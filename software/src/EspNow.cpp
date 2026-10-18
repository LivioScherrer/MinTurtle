#include "EspNow.h"
#include "BLEManager.h"
#include "Drive.h"

PeerEntry peerList[MAX_PEERS];
uint8_t peerCount = 0;
uint8_t espNowPeer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t pairedPeer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
bool espNowReady = false;
bool hasPairedPeer = false;
uint8_t myMac[6] = {0,0,0,0,0,0};

SemaphoreHandle_t peerMutex = nullptr;
QueueHandle_t espNowEventQueue = nullptr;

extern void onRacePacket(const EduPacket* pkt);
extern void onSlalomPacket(const EduPacket* pkt);
extern void duelOnPacket(const EduPacket* pkt);
extern void lobbyOnPacket(const EduPacket* pkt);

namespace {

constexpr uint32_t HEARTBEAT_INTERVAL_MS = 2000;
constexpr uint32_t PEER_LOST_MS          = 6000;
uint32_t s_lastPingMs = 0;
uint32_t s_lastPeerSeenMs = 0;

constexpr uint8_t  REL_SLOTS        = 4;
constexpr uint8_t  REL_MAX_ATTEMPTS = 3;
constexpr uint32_t REL_BACKOFF_MS[REL_MAX_ATTEMPTS] = {50, 150, 450};

struct RelSlot {
  bool      used;
  uint8_t   dest[6];
  EduPacket pkt;
  uint8_t   attempts;
  uint32_t  nextMs;
  uint32_t  queuedMs;
};
RelSlot s_rel[REL_SLOTS];

EspNowTxStats s_stats = {};
uint8_t s_nextSeq = 0;

struct PeerSeq {
  uint8_t mac[6];
  uint8_t seq;
  bool    seen;
};
PeerSeq s_peerSeq[MAX_PEERS];

PeerSeq* peerSeqFor(const uint8_t mac[6]) {
  int freeIdx = -1;
  for (uint8_t i = 0; i < MAX_PEERS; i++) {
    if (memcmp(s_peerSeq[i].mac, mac, 6) == 0) return &s_peerSeq[i];
    if (!s_peerSeq[i].seen && freeIdx < 0) freeIdx = i;
  }
  if (freeIdx < 0) freeIdx = 0;
  memset(&s_peerSeq[freeIdx], 0, sizeof(PeerSeq));
  memcpy(s_peerSeq[freeIdx].mac, mac, 6);
  return &s_peerSeq[freeIdx];
}

void fillPacket(EduPacket& pkt, uint8_t cmd, const char* data,
                uint8_t flags, uint8_t score, uint8_t seq) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic   = FRAME_MAGIC;
  pkt.version = FRAME_VERSION;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.seq   = seq;
  pkt.flags = flags;
  pkt.cmd   = cmd;
  pkt.score = score;
  if (data) {
    size_t len = strlen(data);
    if (len > FRAME_DATA_MAX - 1) len = FRAME_DATA_MAX - 1;
    memcpy(pkt.data, data, len);
    pkt.data[len] = '\0';
    pkt.len = (uint8_t)len;
  }
}

bool rawSend(const uint8_t dest[6], const EduPacket& pkt) {
  if (!espNowReady) return false;
  s_stats.sent++;
  esp_err_t e = esp_now_send(dest, (const uint8_t*)&pkt, sizeof(pkt));
  if (e != ESP_OK) {
    Serial.printf("[ESPNOW] send error=%d\n", (int)e);
    return false;
  }
  return true;
}

void sendAck(const uint8_t dest[6], uint8_t ackedSeq) {
  EduPacket pkt;
  fillPacket(pkt, LOBBY_CMD_ACK, nullptr, 0, 0, 0);
  pkt.data[0] = (char)ackedSeq;
  pkt.len = 1;
  esp_now_send(dest, (uint8_t*)&pkt, sizeof(pkt));
}

void ackSlot(const uint8_t mac[6], uint8_t ackedSeq) {
  for (uint8_t i = 0; i < REL_SLOTS; i++) {
    if (s_rel[i].used && s_rel[i].pkt.seq == ackedSeq &&
        memcmp(s_rel[i].dest, mac, 6) == 0) {
      s_stats.acked++;
      s_rel[i].used = false;
      return;
    }
  }
}

void notePeerSeen(const uint8_t mac[6]) {
  if (hasPairedPeer && memcmp(mac, pairedPeer, 6) == 0) s_lastPeerSeenMs = millis();
}

void postEvent(const EspNowEvent& e) {
  if (!espNowEventQueue) return;
  if (xQueueSend(espNowEventQueue, &e, 0) != pdTRUE) {
    static uint32_t lastLogMs = 0;
    uint32_t now = millis();
    if (now - lastLogMs > 2000) {
      lastLogMs = now;
      Serial.println("[ESPNOW] Event-Queue voll - Paket verworfen");
    }
  }
}

int relFreeSlot() {
  for (uint8_t i = 0; i < REL_SLOTS; i++) if (!s_rel[i].used) return i;
  return -1;
}

}


void peerListAdd(const uint8_t mac[6]) {
  if (peerMutex && xSemaphoreTake(peerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    for (uint8_t i = 0; i < peerCount; i++) {
      if (memcmp(peerList[i].mac, mac, 6) == 0) {
        peerList[i].lastSeen = millis();
        xSemaphoreGive(peerMutex);
        return;
      }
    }
    if (peerCount < MAX_PEERS) {
      memcpy(peerList[peerCount].mac, mac, 6);
      peerList[peerCount].lastSeen = millis();
      peerCount++;
      char msg[48];
      snprintf(msg, sizeof(msg), "RACE:PEER:%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      sendStatus(msg);
    }
    xSemaphoreGive(peerMutex);
  }
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != (int)sizeof(EduPacket)) return;
  EduPacket pkt;
  memcpy(&pkt, data, sizeof(EduPacket));
  if (pkt.magic != FRAME_MAGIC) return;
  if (pkt.version != FRAME_VERSION) {
    static uint32_t lastLogMs = 0;
    if (millis() - lastLogMs > 5000) {
      lastLogMs = millis();
      Serial.printf("[ESPNOW] fremde Version %u ignoriert (erwartet %u)\n",
                    pkt.version, FRAME_VERSION);
    }
    return;
  }
  if (memcmp(pkt.senderMac, myMac, 6) == 0) return;

  memcpy(pkt.senderMac, mac, 6);
  if (memcmp(mac, myMac, 6) == 0) return;

  notePeerSeen(mac);

  if (pkt.len > FRAME_DATA_MAX - 1) pkt.len = FRAME_DATA_MAX - 1;
  pkt.data[pkt.len] = '\0';

  if (pkt.flags & RACE_FLAG_PRESENT) {
    EspNowEvent e = {};
    e.type = EspNowEvent::PEER_ADD;
    memcpy(e.mac, pkt.senderMac, 6);
    postEvent(e);
    return;
  }
  if (pkt.flags & RACE_FLAG_HELLO) {
    EspNowEvent e = {};
    e.type = EspNowEvent::PEER_ADD;
    memcpy(e.mac, pkt.senderMac, 6);
    postEvent(e);
    EduPacket resp;
    fillPacket(resp, SLALOM_CMD_NONE, nullptr, RACE_FLAG_PRESENT, 0, 0);
    esp_now_send(mac, (uint8_t*)&resp, sizeof(resp));
    return;
  }
  if (pkt.cmd == LOBBY_CMD_ACK) {
    if (pkt.len >= 1) ackSlot(mac, (uint8_t)pkt.data[0]);
    return;
  }

  if (pkt.flags & RACE_FLAG_RELIABLE) {
    espNowAddSpecificPeer(mac);
    PeerSeq* ps = peerSeqFor(mac);
    bool fresh = true;
    if (ps->seen) {
      uint8_t d = (uint8_t)(pkt.seq - ps->seq);
      fresh = (d > 0 && d < 128);
      if (!fresh) s_stats.dupRx++;
    }
    ps->seq  = pkt.seq;
    ps->seen = true;
    sendAck(mac, pkt.seq);
    if (!fresh) return;
  }

  if (pkt.cmd == LOBBY_CMD_PING) {
    return;
  }

  EspNowEvent e = {};
  e.type = EspNowEvent::PKT;
  memcpy(&e.pkt, &pkt, sizeof(EduPacket));
  postEvent(e);
}

void espNowProcessEvents() {
  if (!espNowEventQueue) { espNowTick(); return; }
  EspNowEvent e;
  while (xQueueReceive(espNowEventQueue, &e, 0) == pdTRUE) {
    switch (e.type) {
      case EspNowEvent::PEER_ADD:
        peerListAdd(e.mac);
        espNowAddSpecificPeer(e.mac);
        break;
      case EspNowEvent::PKT:
        if (e.pkt.cmd == SLALOM_CMD_DUEL) {
          duelOnPacket(&e.pkt);
        } else if (e.pkt.cmd >= LOBBY_CMD_INVITE && e.pkt.cmd <= GAME_CMD_STOP) {
          lobbyOnPacket(&e.pkt);
        } else if (e.pkt.cmd != SLALOM_CMD_NONE) {
          onSlalomPacket(&e.pkt);
        } else {
          if (!hasPairedPeer) break;
          if (memcmp(e.pkt.senderMac, pairedPeer, 6) != 0) break;
          onRacePacket(&e.pkt);
        }
        break;
    }
  }
  espNowTick();
}

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[ESPNOW] send fehlgeschlagen status=%d\n", status);
  }
}

namespace {
void relService() {
  int head = -1;
  uint32_t oldest = 0xFFFFFFFFUL;
  for (uint8_t i = 0; i < REL_SLOTS; i++) {
    if (s_rel[i].used && s_rel[i].queuedMs < oldest) {
      oldest = s_rel[i].queuedMs;
      head = i;
    }
  }
  if (head < 0) return;
  RelSlot& s = s_rel[head];
  if (s.attempts >= REL_MAX_ATTEMPTS) {
    s.used = false;
    s_stats.dropped++;
    Serial.printf("[ESPNOW] zuverlaessiges Paket cmd=%u nach %u Versuchen "
                  "aufgegeben (Seq %u)\n", s.pkt.cmd, REL_MAX_ATTEMPTS, s.pkt.seq);
    return;
  }
  uint32_t now = millis();
  if (now < s.nextMs) return;
  s_stats.sent++;
  if (esp_now_send(s.dest, (uint8_t*)&s.pkt, sizeof(s.pkt)) != ESP_OK) {
    Serial.println("[ESPNOW] Retry-Versand fehlgeschlagen");
  }
  s.attempts++;
  s.nextMs = now + REL_BACKOFF_MS[s.attempts - 1];
}
}

void espNowTick() {
  if (!espNowReady) return;
  relService();

  if (hasPairedPeer) {
    uint32_t now = millis();
    if (now - s_lastPingMs >= HEARTBEAT_INTERVAL_MS) {
      s_lastPingMs = now;
      espNowSendTo(pairedPeer, LOBBY_CMD_PING, nullptr,
                   RACE_FLAG_RELIABLE, 0, false);
    }
  }
}

bool espNowPeerAlive() {
  if (!hasPairedPeer) return true;
  if (s_lastPeerSeenMs == 0) return true;
  return (millis() - s_lastPeerSeenMs) <= PEER_LOST_MS;
}

uint32_t espNowPeerSilentMs() {
  if (!hasPairedPeer || s_lastPeerSeenMs == 0) return 0;
  return millis() - s_lastPeerSeenMs;
}

void espNowGetTxStats(EspNowTxStats& out) { out = s_stats; }

bool espNowSendTo(const uint8_t dest[6], uint8_t cmd, const char* data,
                  uint8_t flags, uint8_t score, bool reliable) {
  if (!espNowReady) return false;
  if (!dest) return false;

  static const uint8_t BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  if (reliable && memcmp(dest, BROADCAST, 6) == 0) {
    reliable = false;
  }

  if (reliable) flags |= RACE_FLAG_RELIABLE;

  EduPacket pkt;
  uint8_t seq = s_nextSeq++;
  fillPacket(pkt, cmd, data, flags, score, seq);

  if (!reliable) return rawSend(dest, pkt);

  int slot = relFreeSlot();
  if (slot < 0) {
    s_stats.dropped++;
    Serial.printf("[ESPNOW] Reliability-Slots voll - cmd=%u verworfen\n", cmd);
    return false;
  }
  RelSlot& s = s_rel[slot];
  s.used     = true;
  memcpy(s.dest, dest, 6);
  s.pkt      = pkt;
  s.attempts = 0;
  s.queuedMs = millis();
  s.nextMs   = 0;

  bool othersOpen = false;
  for (uint8_t i = 0; i < REL_SLOTS; i++) {
    if (i != (uint8_t)slot && s_rel[i].used) { othersOpen = true; break; }
  }
  if (!othersOpen) {
    s.attempts = 1;
    s.nextMs   = millis() + REL_BACKOFF_MS[0];
    s_stats.sent++;
    if (esp_now_send(dest, (uint8_t*)&s.pkt, sizeof(s.pkt)) != ESP_OK) {
      Serial.println("[ESPNOW] erster Versand fehlgeschlagen");
    }
  }
  return true;
}

void espNowAddSpecificPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] specific add_peer fehlgeschlagen");
  }
}

bool parseMac(const String& macStr, uint8_t mac[6]) {
  memset(mac, 0, 6);
  int idx = 0, pos = 0;
  while (idx < 6 && pos + 1 < macStr.length()) {
    char hex[3] = { macStr.charAt(pos), macStr.charAt(pos + 1), 0 };
    mac[idx++] = (uint8_t)strtol(hex, nullptr, 16);
    pos += 3;
  }
  return idx == 6;
}

void espNowInit() {
  peerMutex = xSemaphoreCreateMutex();
  espNowEventQueue = xQueueCreate(32, sizeof(EspNowEvent));
  if (!espNowEventQueue) {
    Serial.println("[ESPNOW] Queue-Erzeugung fehlgeschlagen");
    return;
  }

  const wifi_mode_t curMode = WiFi.getMode();
  const bool apActive = (curMode == WIFI_AP || curMode == WIFI_AP_STA);
  if (!apActive) {
    WiFi.mode(WIFI_STA);
    esp_wifi_stop();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  } else {
    Serial.println("[ESPNOW] Access Point laeuft bereits - WLAN-Kanal nicht angefasst");
  }

  WiFi.macAddress(myMac);
  Serial.printf("[ESPNOW] meine MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);

  esp_err_t initErr = esp_now_init();
  if (initErr != ESP_OK) {
    Serial.printf("[ESPNOW] init failed err=%d\n", (int)initErr);
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSend);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, espNowPeer, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_err_t addErr = esp_now_add_peer(&peer);
  if (addErr == ESP_OK) {
    espNowReady = true;
    Serial.println("[ESPNOW] bereit (broadcast peer auf channel 1)");
  } else {
    Serial.printf("[ESPNOW] add_peer failed err=%d\n", (int)addErr);
  }
}

void espNowEnsureChannel() {
  if (!espNowReady) return;
  uint8_t primary = 0;
  wifi_second_chan_t second;
  if (esp_wifi_get_channel(&primary, &second) != ESP_OK) return;
  if (primary == 1) return;
  Serial.printf("[ESPNOW] Kanal war %u -> setze auf 1\n", primary);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void raceSendHello() {
  if (!espNowReady) return;
  if (peerMutex && xSemaphoreTake(peerMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    peerCount = 0;
    xSemaphoreGive(peerMutex);
  }
  EduPacket pkt;
  fillPacket(pkt, SLALOM_CMD_NONE, nullptr, RACE_FLAG_HELLO, 0, 0);
  esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  Serial.println("[ESPNOW] HELLO gesendet");
}

void raceSendPacket(uint8_t score, bool winner) {
  if (!espNowReady) return;
  const uint8_t* dest = hasPairedPeer ? pairedPeer : espNowPeer;
  espNowSendTo(dest, SLALOM_CMD_NONE, nullptr,
               winner ? RACE_FLAG_WINNER : 0, score, false);
}

void racePairWith(const uint8_t mac[6]) {
  espNowAddSpecificPeer(mac);
  memcpy(pairedPeer, mac, 6);
  hasPairedPeer = true;
  s_lastPeerSeenMs = millis();
  s_lastPingMs     = millis();
  for (uint8_t i = 0; i < REL_SLOTS; i++) s_rel[i].used = false;
  Serial.printf("[ESPNOW] Gepaart mit %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void raceUnpair() {
  if (hasPairedPeer) {
    esp_now_del_peer(pairedPeer);
    hasPairedPeer = false;
    memset(pairedPeer, 0xFF, 6);
    s_lastPeerSeenMs = 0;
    for (uint8_t i = 0; i < REL_SLOTS; i++) s_rel[i].used = false;
    Serial.println("[ESPNOW] Unpair");
  }
}
