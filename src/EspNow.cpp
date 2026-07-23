// EduBot V1.3 - ESP-NOW Implementierung
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

// Forward-Declaration: Race-Logik in main
extern void onRacePacket(const RacePacket* pkt);
extern void espNowAddSpecificPeer(const uint8_t mac[6]);

void peerListAdd(const uint8_t mac[6]) {
  for (uint8_t i = 0; i < peerCount; i++) {
    if (memcmp(peerList[i].mac, mac, 6) == 0) {
      peerList[i].lastSeen = millis();
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
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(RacePacket)) return;
  RacePacket* pkt = (RacePacket*)data;
  if (pkt->magic != 0xA5) return;
  Serial.printf("[ESPNOW] pkt score=%u flags=%u\n", pkt->score, pkt->flags);
  if (memcmp(pkt->senderMac, myMac, 6) == 0) return;

  if (pkt->flags & RACE_FLAG_HELLO) {
    peerListAdd(pkt->senderMac);
    espNowAddSpecificPeer(pkt->senderMac);
    RacePacket resp;
    resp.magic = 0xA5;
    memcpy(resp.senderMac, myMac, 6);
    resp.score = 0;
    resp.flags = RACE_FLAG_PRESENT;
    esp_now_send(pkt->senderMac, (uint8_t*)&resp, sizeof(resp));
    return;
  }
  if (pkt->flags & RACE_FLAG_PRESENT) {
    peerListAdd(pkt->senderMac);
    return;
  }
  if (hasPairedPeer && memcmp(pkt->senderMac, pairedPeer, 6) != 0) return;
  onRacePacket(pkt);
}

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[ESPNOW] send fehlgeschlagen status=%d\n", status);
  }
}

void espNowAddSpecificPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] specific add_peer (vielleicht schon da)");
  }
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  esp_wifi_stop();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  WiFi.macAddress(myMac);
  Serial.printf("[ESPNOW] meine MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);

  if (esp_now_init() != ESP_OK) { Serial.println("[ESPNOW] init failed"); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSend);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, espNowPeer, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    espNowReady = true;
    Serial.println("[ESPNOW] bereit (broadcast peer auf channel 1)");
  } else {
    Serial.println("[ESPNOW] add_peer failed");
  }
}

void espNowEnsureChannel() {
  if (!espNowReady) return;
  static uint8_t lastChannel = 255;
  uint8_t primary;
  wifi_second_chan_t second;
  if (esp_wifi_get_channel(&primary, &second) == ESP_OK && primary != lastChannel) {
    lastChannel = primary;
    Serial.printf("[ESPNOW] channel war %u -> setze auf 1\n", primary);
  }
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void raceSendPacket(uint8_t score, bool winner) {
  if (!espNowReady) return;
  RacePacket pkt;
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = score;
  pkt.flags = winner ? RACE_FLAG_WINNER : 0x00;
  if (hasPairedPeer) {
    esp_now_send(pairedPeer, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  }
}

void raceSendHello() {
  if (!espNowReady) return;
  RacePacket pkt;
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = 0;
  pkt.flags = RACE_FLAG_HELLO;
  peerCount = 0;
  esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  Serial.println("[ESPNOW] HELLO gesendet");
}

void racePairWith(const uint8_t mac[6]) {
  espNowAddSpecificPeer(mac);
  memcpy(pairedPeer, mac, 6);
  hasPairedPeer = true;
  Serial.printf("[ESPNOW] Gepaart mit %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void raceUnpair() {
  if (hasPairedPeer) {
    esp_now_del_peer(pairedPeer);
    hasPairedPeer = false;
    pairedPeer[0]=0xFF; pairedPeer[1]=0xFF; pairedPeer[2]=0xFF;
    pairedPeer[3]=0xFF; pairedPeer[4]=0xFF; pairedPeer[5]=0xFF;
    Serial.println("[ESPNOW] Unpair");
  }
}
