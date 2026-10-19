#include "Slalom.h"

#include "BLEManager.h"
#include "EspNow.h"
#include "GameEngine.h"
#include "LedStrip.h"
#include "ProgGame.h"

SlalomPhase slalomPhase = SLALOM_IDLE;
static const char* const SLALOM_PHASE_NAMES[6] = {
  "IDLE", "PAIRED", "BUILD", "COUNTDOWN", "RACING", "DONE"
};

char slalomTargetColor[16] = "";
char slalomMyProgram[PROG_MAX_ENC + 1] = "";
bool slalomRunning   = false;
bool slalomSoloTime  = false;
bool slalomIsHost    = false;
bool slalomReady     = false;
bool slalomOppReady  = false;
bool slalomFinished  = false;
bool slalomWon       = false;
bool slalomIWonLast  = false;

bool slalomMyRunDone  = false;
bool slalomOppRunDone = false;
unsigned long slalomOppWaitStart = 0;

unsigned long slalomGoTimer = 0;
bool slalomGoPending = false;

static const char* const SLALOM_COLOR_NAMES[4] = { "ROT", "GELB", "GRUEN", "BLAU" };

static bool    slalomHasPeer_     = false;
static uint8_t slalomPeerMac[6]  = {0,0,0,0,0,0};
static uint8_t slalomInviterMac[6] = {0,0,0,0,0,0};
static bool    slalomInvitePending_ = false;
static bool    slalomInviteSent_     = false;
static uint8_t slalomInviteSentMac[6] = {0,0,0,0,0,0};
static unsigned long slalomInviteSentMs = 0;
static uint8_t slalomInviteRetries    = 0;

static constexpr uint32_t SLALOM_INVITE_RETRY_MS = 2000;
static constexpr uint8_t  SLALOM_INVITE_MAX_RETRY = 3;
static constexpr uint32_t SLALOM_INVITE_TIMEOUT_MS = 10000;

bool slalomHasPeer() { return slalomHasPeer_; }
uint8_t* slalomPeerMacPtr() { return slalomPeerMac; }
bool slalomInvitePending() { return slalomInvitePending_; }
uint8_t* slalomInviterMacPtr() { return slalomInviterMac; }
void slalomClearInvite() {
  slalomInvitePending_ = false;
  memset(slalomInviterMac, 0, 6);
}

void slalomInviteSend(const uint8_t destMac[6]) {
  memcpy(slalomInviteSentMac, destMac, 6);
  espNowAddSpecificPeer(destMac);
  slalomInviteSent_ = true;
  slalomInviteSentMs = millis();
  slalomInviteRetries = 1;
  slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, destMac, true);
  Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet (Versuch 1/%d)\n",
    destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5],
    SLALOM_INVITE_MAX_RETRY);
}

void slalomTick() {
  if (!slalomInviteSent_ || slalomHasPeer_) return;
  const unsigned long age = millis() - slalomInviteSentMs;
  if (age < SLALOM_INVITE_RETRY_MS) return;

  if (slalomInviteRetries < SLALOM_INVITE_MAX_RETRY) {
    slalomInviteRetries++;
    slalomInviteSentMs = millis();
    slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, slalomInviteSentMac, true);
    Serial.printf("[SLALOM] Einladung erneut gesendet (Versuch %d/%d)\n",
                  slalomInviteRetries, SLALOM_INVITE_MAX_RETRY);
    return;
  }
  if (age >= SLALOM_INVITE_TIMEOUT_MS) {
    slalomInviteSent_ = false;
    sendStatus("SLALOM:INVITE_TIMEOUT");
    Serial.println("[SLALOM] Einladung unbeantwortet - aufgegeben");
  }
}

void slalomSetPhase(SlalomPhase p) {
  if (slalomPhase == p) return;
  Serial.printf("[SLALOM] Phase %s -> %s\n", SLALOM_PHASE_NAMES[slalomPhase], SLALOM_PHASE_NAMES[p]);
  slalomPhase = p;
}

void slalomResetRuntime() {
  slalomRunning = false;
  slalomReady = false;
  slalomOppReady = false;
  slalomFinished = false;
  slalomWon = false;
  slalomMyRunDone = false;
  slalomOppRunDone = false;
  slalomOppWaitStart = 0;
  slalomGoPending = false;
  slalomGoTimer = 0;
  slalomMyProgram[0] = '\0';
}

void slalomPairWith(const uint8_t mac[6], bool initiator) {
  memcpy(slalomPeerMac, mac, 6);
  slalomHasPeer_ = true;
  slalomIsHost  = initiator;
  slalomInviteSent_ = false;
  espNowAddSpecificPeer(mac);
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_PAIRED);
  sendStatus(initiator ? "SLALOM:INVITE_PAIRED:HOST" : "SLALOM:INVITE_PAIRED:CLIENT");
  Serial.printf("[SLALOM] Pairing mit %02X:%02X:%02X:%02X:%02X:%02X (Initiator=%d)\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], initiator);
}

void slalomUnpair() {
  if (slalomHasPeer_) {
    const bool isRacePeer = hasPairedPeer && memcmp(pairedPeer, slalomPeerMac, 6) == 0;
    if (!isRacePeer) {
      esp_now_del_peer(slalomPeerMac);
    } else {
      Serial.println("[SLALOM] Peer bleibt als Rennen-Peer in der ESP-NOW-Tabelle");
    }
  }
  slalomHasPeer_       = false;
  slalomIsHost        = false;
  slalomSoloTime      = false;
  slalomInvitePending_ = false;
  slalomInviteSent_   = false;
  memset(slalomPeerMac, 0, 6);
  memset(slalomInviterMac, 0, 6);
  memset(slalomInviteSentMac, 0, 6);
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  sendStatus("SLALOM:UNPAIRED");
  Serial.println("[SLALOM] Slalom-Unpair");
}

void slalomSendPacket(uint8_t cmd, const char* data, bool reliable) {
  const uint8_t* dest;
  if (slalomHasPeer_) {
    dest = slalomPeerMac;
  } else if (hasPairedPeer) {
    dest = pairedPeer;
  } else {
    dest = espNowPeer;
  }
  espNowSendTo(dest, cmd, data, 0, 0, reliable);
}

void slalomSendPacketTo(uint8_t cmd, const char* data, const uint8_t destMac[6],
                        bool reliable) {
  if (memcmp(destMac, myMac, 6) == 0) return;
  espNowSendTo(destMac, cmd, data, 0, 0, reliable);
}

void slalomEndGame(bool iWon) {
  if (slalomRunning || progActive) {
    progStop();
    slalomRunning = false;
    currentGame = GAME_NONE;
  }
  slalomSoloTime = false;
  slalomResetRuntime();
  slalomWon = true;
  slalomIWonLast = iWon;
  slalomSetPhase(SLALOM_DONE);
  sendStatus(iWon ? "SLALOM:WIN:ME" : "SLALOM:WIN:OPP");
  Serial.printf("[SLALOM] Spielende: %s\n", iWon ? "ICH habe gewonnen!" : "Gegner hat gewonnen");
}

void slalomAbortGame(bool notifyPeer) {
  if (slalomRunning || progActive) {
    progStop();
    slalomRunning = false;
    currentGame = GAME_NONE;
  }
  const bool wasSolo = slalomSoloTime;
  slalomSoloTime = false;
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(wasSolo ? SLALOM_IDLE : SLALOM_PAIRED);
  sendStatus("SLALOM:STOPPED");
  Serial.println("[SLALOM] Spiel abgebrochen");
  if (notifyPeer && !wasSolo && slalomHasPeer_) {
    slalomSendPacket(SLALOM_CMD_ABORT, nullptr, true);
  }
}

void slalomStartSoloTime(const char* color) {
  if (slalomRunning || currentGame != GAME_NONE) {
    sendStatus("SLALOM:BUSY");
    return;
  }
  if (!color || color[0] == '\0') {
    sendStatus("SLALOM:ERROR:NO_TARGET");
    return;
  }
  bool known = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (strcasecmp(color, SLALOM_COLOR_NAMES[i]) == 0) { known = true; break; }
  }
  if (!known) {
    sendStatus("SLALOM:ERROR:NO_TARGET");
    return;
  }
  slalomSoloTime = true;
  slalomResetRuntime();
  strlcpy(slalomTargetColor, color, sizeof(slalomTargetColor));
  slalomSetPhase(SLALOM_BUILD);
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor);
  sendStatus(msg);
  sendStatus("SLALOM:MATCH_STARTED");
  Serial.printf("[SLALOM] Zeitfahrt gestartet, Zielfarbe %s\n", slalomTargetColor);
}

void slalomStartGame() {
  if (!slalomHasPeer_) {
    Serial.println("[SLALOM] Start blockiert: kein Peer gepaart");
    return;
  }
  slalomSoloTime = false;
  slalomResetRuntime();
  slalomSetPhase(SLALOM_BUILD);
  uint8_t idx = random(0, 4);
  strlcpy(slalomTargetColor, SLALOM_COLOR_NAMES[idx], sizeof(slalomTargetColor));
  slalomSendPacket(SLALOM_CMD_START, slalomTargetColor, true);
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor);
  sendStatus(msg);
  sendStatus("SLALOM:MATCH_STARTED");
  Serial.printf("[SLALOM] Host startete Spiel, Zielfarbe %s (zuverlaessig gesendet)\n", slalomTargetColor);
}

void slalomOnStartReceived(const char* col) {
  if (slalomPhase == SLALOM_BUILD || slalomPhase == SLALOM_COUNTDOWN ||
      slalomPhase == SLALOM_RACING) return;
  slalomSoloTime = false;
  slalomResetRuntime();
  if (col && col[0] != '\0') {
    strlcpy(slalomTargetColor, col, sizeof(slalomTargetColor));
  }
  slalomSetPhase(SLALOM_BUILD);
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor[0] ? slalomTargetColor : "?");
  sendStatus(msg);
  sendStatus("SLALOM:MATCH_STARTED");
  Serial.printf("[SLALOM] Client: Spiel gestartet, Zielfarbe %s\n", slalomTargetColor);
}

void slalomSetProgram(const char* enc) {
  strlcpy(slalomMyProgram, enc, sizeof(slalomMyProgram));
  slalomReady = true;
  sendStatus("SLALOM:READY");
  if (slalomSoloTime) {
    slalomOppReady = true;
  } else {
    slalomSendPacket(SLALOM_CMD_PROG, slalomMyProgram, true);
  }
  slalomMaybeStart();
}

void slalomMaybeStart() {
  if (slalomPhase != SLALOM_BUILD) return;
  if (slalomRunning || slalomGoPending) return;
  if (!slalomReady || !slalomOppReady) return;
  if (slalomTargetColor[0] == '\0') return;
  slalomSetPhase(SLALOM_COUNTDOWN);
  sendStatus("SLALOM:BOTH_READY");
  if (slalomSoloTime) {
    slalomGoTimer = millis() + 3000;
  } else if (slalomIsHost) {
    slalomGoTimer = millis() + 3000;
  } else {
    slalomGoTimer = millis() + 4500;
  }
  slalomGoPending = true;
  Serial.printf("[SLALOM] %s bereit. %s-Countdown 3s bis GO\n",
                slalomSoloTime ? "Zeitfahrt" : "Beide",
                slalomIsHost ? "Host" : (slalomSoloTime ? "Solo" : "Client"));
}

void slalomExecute() {
  if (slalomTargetColor[0] == '\0') {
    Serial.println("[SLALOM] Abbruch: keine Zielfarbe gesetzt");
    sendStatus("SLALOM:ERROR:NO_TARGET");
    return;
  }
  currentGame = GAME_SLALOM;
  slalomRunning = true;
  slalomMyRunDone = false;
  slalomOppRunDone = false;
  slalomOppWaitStart = 0;
  slalomSetPhase(SLALOM_RACING);
  sendStatus("SLALOM:RUNNING");
  ledStripClear();
  Serial.printf("[SLALOM] RACE! Ziel: %s\n", slalomTargetColor);
  progSetFromString(slalomMyProgram);
  progRunSlalom(slalomTargetColor);
  currentGame = GAME_NONE;
  slalomRunning = false;
  if (slalomWon) return;

  if (slalomFinished) {
    slalomSendPacket(SLALOM_CMD_WIN, slalomTargetColor, true);
    slalomEndGame(true);
  } else {
    slalomMyRunDone = true;
    slalomSendPacket(SLALOM_CMD_MISS, nullptr, true);
    if (slalomOppRunDone) {
      slalomResetRuntime();
      slalomSetPhase(SLALOM_BUILD);
      sendStatus("SLALOM:RETRY");
    } else {
      slalomOppWaitStart = millis();
    }
  }
}

void slalomExecuteSoloTime() {
  if (!slalomSoloTime) return;
  if (slalomTargetColor[0] == '\0') {
    sendStatus("SLALOM:ERROR:NO_TARGET");
    slalomSetPhase(SLALOM_BUILD);
    return;
  }
  slalomFinished = false;
  slalomWon = false;
  slalomRunning = true;
  currentGame = GAME_SLALOM;
  slalomSetPhase(SLALOM_RACING);
  sendStatus("SLALOM:RUNNING");
  ledStripClear();
  progSetFromString(slalomMyProgram);
  const uint32_t startMs = millis();
  progRunSlalom(slalomTargetColor);
  const uint32_t elapsedMs = millis() - startMs;
  const bool hit = slalomFinished;
  slalomFinished = false;
  slalomRunning = false;
  currentGame = GAME_NONE;
  if (!slalomSoloTime) return;
  slalomIWonLast = hit;
  slalomSetPhase(SLALOM_DONE);
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TIME_RESULT:%lu,%u",
           (unsigned long)elapsedMs, hit ? 1u : 0u);
  sendStatus(msg);
  ledStripShowResultFlash(hit);
  ledStripTick();
  Serial.printf("[SLALOM] Zeitfahrt %.2f s, Treffer=%d\n",
                elapsedMs / 1000.0f, hit ? 1 : 0);
}

void slalomCheckRetry() {
  if (slalomRunning || slalomPhase != SLALOM_RACING) return;
  if (slalomWon) return;
  if (slalomMyRunDone && slalomOppRunDone) {
    slalomResetRuntime();
    slalomSetPhase(SLALOM_BUILD);
    sendStatus("SLALOM:RETRY");
  } else if (slalomMyRunDone && slalomOppWaitStart != 0 &&
             millis() - slalomOppWaitStart > 15000) {
    Serial.println("[SLALOM] 15s-Timeout: Gegner-Ergebnis fehlt -> Retry");
    slalomResetRuntime();
    slalomSetPhase(SLALOM_BUILD);
    sendStatus("SLALOM:RETRY");
  }
}

void onSlalomPacket(const EduPacket* pkt) {
  switch (pkt->cmd) {
    case SLALOM_CMD_INVITE:
      break;
    case SLALOM_CMD_INVITE_ACK:
      break;
    case SLALOM_CMD_SLALOM_UNPAIR:
      if (slalomInviteSent_ &&
          memcmp(pkt->senderMac, slalomInviteSentMac, 6) == 0) {
        slalomInviteSent_ = false;
        memset(slalomInviteSentMac, 0, 6);
        sendStatus("SLALOM:INVITE_DECLINED");
        Serial.println("[SLALOM] Einladung abgelehnt");
        return;
      }
      break;
    default:
      if (!slalomHasPeer_) {
        Serial.println("[SLALOM] Paket verworfen: kein Slalom-Peer");
        return;
      }
      if (memcmp(pkt->senderMac, slalomPeerMac, 6) != 0) {
        Serial.println("[SLALOM] Paket verworfen: Absender ist nicht der Slalom-Peer");
        return;
      }
      break;
  }

  if (slalomSoloTime && pkt->cmd != SLALOM_CMD_SLALOM_UNPAIR &&
      pkt->cmd != SLALOM_CMD_START) {
    return;
  }

  switch (pkt->cmd) {
    case SLALOM_CMD_PROG:
      if (pkt->len > 0) {
        slalomOppReady = true;
        sendStatus("SLALOM:OPP_PROG");
        slalomMaybeStart();
      }
      break;

    case SLALOM_CMD_GO: {
      slalomGoPending = false;
      if (slalomPhase == SLALOM_COUNTDOWN && !slalomRunning) {
        slalomExecute();
      }
      break;
    }

    case SLALOM_CMD_START: {
      if (slalomIsHost) break;
      char buf[16] = {0};
      size_t len = pkt->len < 15 ? pkt->len : 15;
      memcpy(buf, pkt->data, len);
      slalomOnStartReceived(buf);
      break;
    }

    case SLALOM_CMD_WIN:
      if (slalomWon) break;
      slalomEndGame(false);
      break;

    case SLALOM_CMD_MISS:
      slalomOppRunDone = true;
      sendStatus("SLALOM:OPP_MISS");
      if (slalomMyRunDone) {
        slalomCheckRetry();
      }
      break;

    case SLALOM_CMD_ABORT:
      slalomAbortGame(false);
      break;

    case SLALOM_CMD_INVITE:
      if (slalomPhase != SLALOM_IDLE) {
        Serial.println("[SLALOM] INVITE ignoriert: nicht in IDLE");
        return;
      }
      {
        memcpy(slalomInviterMac, pkt->senderMac, 6);
        slalomInvitePending_ = true;
        char invMsg[48];
        snprintf(invMsg, sizeof(invMsg), "SLALOM:INVITED_BY:%02X:%02X:%02X:%02X:%02X:%02X",
                 pkt->senderMac[0], pkt->senderMac[1], pkt->senderMac[2],
                 pkt->senderMac[3], pkt->senderMac[4], pkt->senderMac[5]);
        sendStatus(invMsg);
        Serial.printf("[SLALOM] Einladung empfangen von %02X:%02X:%02X:%02X:%02X:%02X\n",
          pkt->senderMac[0], pkt->senderMac[1], pkt->senderMac[2],
          pkt->senderMac[3], pkt->senderMac[4], pkt->senderMac[5]);
      }
      break;

    case SLALOM_CMD_INVITE_ACK:
      if (!slalomHasPeer_) {
        slalomPairWith(pkt->senderMac, true);
      }
      break;

    case SLALOM_CMD_SLALOM_UNPAIR:
      if (slalomHasPeer_ && memcmp(pkt->senderMac, slalomPeerMac, 6) == 0) {
        Serial.println("[SLALOM] Peer hat Slalom-Unpair gesendet");
        slalomUnpair();
      }
      break;

    default:
      break;
  }
}
