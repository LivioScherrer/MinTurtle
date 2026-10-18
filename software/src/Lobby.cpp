#include "Lobby.h"

#include "BLEManager.h"
#include "Duel.h"
#include "GameEngine.h"
#include "Slalom.h"

extern void handleCommand(const char* cmdStr);
extern void raceAbort(bool notifyPeer);

LobbyPhase lobbyPhase = LOBBY_IDLE;
bool lobbyIsHost = false;

static bool  lobbyHasPeer_ = false;
static uint8_t lobbyPeerMac[6] = {0};
static uint8_t lobbyInviterMac[6] = {0};

static bool    lobbyInviteSent_ = false;
static uint8_t lobbyInviteSentMac[6] = {0};
static unsigned long lobbyInviteSentMs = 0;
static uint8_t lobbyInviteRetries = 0;

static uint32_t lobbyTeardownMs = 0;
static uint8_t  lobbyTeardownMac[6] = {0};
static constexpr uint32_t LOBBY_TEARDOWN_MS = 750;

static const char* const LOBBY_PHASE_NAMES[4] = {
  "IDLE", "INVITING", "INVITED", "JOINED"
};

static constexpr uint32_t LOBBY_INVITE_RETRY_MS    = 2000;
static constexpr uint8_t  LOBBY_INVITE_MAX_RETRY   = 3;
static constexpr uint32_t LOBBY_INVITE_TIMEOUT_MS  = 10000;

static bool  lobbyStopLocal();
static void  lobbyRunLocalStart(const char* spiel);

const char* lobbyPhaseName() { return LOBBY_PHASE_NAMES[lobbyPhase]; }

static void lobbySetPhase(LobbyPhase p) {
  if (lobbyPhase == p) return;
  Serial.printf("[LOBBY] Phase %s -> %s\n",
                LOBBY_PHASE_NAMES[lobbyPhase], LOBBY_PHASE_NAMES[p]);
  lobbyPhase = p;
}

bool lobbyHasPeer() { return lobbyHasPeer_; }
uint8_t* lobbyPeerMacPtr() { return lobbyPeerMac; }
uint8_t* lobbyInviterMacPtr() { return lobbyInviterMac; }

static void lobbySendTo(const uint8_t dest[6], uint8_t cmd, const char* data,
                        bool reliable) {
  if (!dest) return;
  espNowSendTo(dest, cmd, data, 0, 0, reliable);
}

void lobbySendPacket(uint8_t cmd, const char* data, bool reliable) {
  lobbySendTo(lobbyHasPeer_ ? lobbyPeerMac : espNowPeer, cmd, data, reliable);
}

static void lobbyJoin(const uint8_t mac[6]) {
  memcpy(lobbyPeerMac, mac, 6);
  lobbyHasPeer_ = true;
  lobbyTeardownMs = 0;
  lobbyInviteSent_ = false;
  memset(lobbyInviteSentMac, 0, 6);
  memset(lobbyInviterMac, 0, 6);
  lobbySetPhase(LOBBY_JOINED);
  lobbyIsHost = (memcmp(myMac, mac, 6) < 0);
  sendStatus(lobbyIsHost ? "LOBBY:JOINED:HOST" : "LOBBY:JOINED:CLIENT");
  Serial.printf("[LOBBY] verbunden mit %02X:%02X:%02X:%02X:%02X:%02X (%s)\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                lobbyIsHost ? "ich bin HOST" : "ich bin CLIENT");

  slalomPairWith(mac, lobbyIsHost);
  racePairWith(mac);
}

static void lobbyLeave(bool notifyPeer) {
  if (notifyPeer && lobbyHasPeer_) {
    lobbySendPacket(LOBBY_CMD_LEAVE, nullptr, true);
  }
  lobbyStopLocal();

  memcpy(lobbyTeardownMac, lobbyPeerMac, 6);
  lobbyTeardownMs = millis() + LOBBY_TEARDOWN_MS;

  lobbyHasPeer_   = false;
  lobbyIsHost     = false;
  lobbyInviteSent_ = false;
  memset(lobbyPeerMac, 0, 6);
  memset(lobbyInviteSentMac, 0, 6);
  memset(lobbyInviterMac, 0, 6);
  lobbySetPhase(LOBBY_IDLE);
  sendStatus("LOBBY:LEFT");
  Serial.println("[LOBBY] Lobby verlassen");
}

void lobbyInviteSend(const uint8_t destMac[6]) {
  memcpy(lobbyInviteSentMac, destMac, 6);
  espNowAddSpecificPeer(destMac);
  lobbyInviteSent_   = true;
  lobbyInviteSentMs  = millis();
  lobbyInviteRetries = 1;
  lobbySetPhase(LOBBY_INVITING);
  lobbySendTo(destMac, LOBBY_CMD_INVITE, nullptr, true);
  Serial.printf("[LOBBY] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet "
                "(Versuch 1/%d)\n",
                destMac[0], destMac[1], destMac[2],
                destMac[3], destMac[4], destMac[5], LOBBY_INVITE_MAX_RETRY);
}

void lobbyTick() {
  if (lobbyTeardownMs != 0 && (int32_t)(millis() - lobbyTeardownMs) >= 0) {
    lobbyTeardownMs = 0;
    if (hasPairedPeer && memcmp(pairedPeer, lobbyTeardownMac, 6) == 0) {
      if (slalomHasPeer()) slalomUnpair();
      raceUnpair();
      Serial.println("[LOBBY] Spiel-Pairings aufgeloest");
    }
    memset(lobbyTeardownMac, 0, 6);
  }

  if (!lobbyInviteSent_ || lobbyHasPeer_) return;
  const unsigned long age = millis() - lobbyInviteSentMs;
  if (age < LOBBY_INVITE_RETRY_MS) return;

  if (lobbyInviteRetries < LOBBY_INVITE_MAX_RETRY) {
    lobbyInviteRetries++;
    lobbyInviteSentMs = millis();
    lobbySendTo(lobbyInviteSentMac, LOBBY_CMD_INVITE, nullptr, true);
    Serial.printf("[LOBBY] Einladung erneut gesendet (Versuch %d/%d)\n",
                  lobbyInviteRetries, LOBBY_INVITE_MAX_RETRY);
    return;
  }
  if (age >= LOBBY_INVITE_TIMEOUT_MS) {
    lobbyInviteSent_ = false;
    memset(lobbyInviteSentMac, 0, 6);
    lobbySetPhase(LOBBY_IDLE);
    sendStatus("LOBBY:INVITE_TIMEOUT");
    Serial.println("[LOBBY] Einladung unbeantwortet - aufgegeben");
  }
}


bool lobbyGameBusy() {
  return currentGame != GAME_NONE || duelBusy() ||
         slalomPhase == SLALOM_BUILD || slalomPhase == SLALOM_COUNTDOWN ||
         slalomPhase == SLALOM_RACING;
}

static bool lobbyGameRunning(const char* s) {
  if (!strncmp(s, "DUEL", 4)) return duelActive();
  if (!strcmp(s, "RACE"))     return currentGame == GAME_RENNEN;
  if (!strcmp(s, "SLALOM"))
    return currentGame == GAME_SLALOM || slalomPhase == SLALOM_BUILD ||
           slalomPhase == SLALOM_COUNTDOWN || slalomPhase == SLALOM_RACING;
  return false;
}

static bool lobbyIsGame(const char* s) {
  if (!strcmp(s, "SLALOM") || !strcmp(s, "RACE") || !strcmp(s, "DUEL"))
    return true;
  if (strncmp(s, "DUEL:", 5) != 0) return false;
  const char* code = s + 5;
  return !strcmp(code, "AMP") || !strcmp(code, "ZAH") || !strcmp(code, "GEH") ||
         !strcmp(code, "PAR") || !strcmp(code, "ZEI");
}

static void lobbyRunLocalStart(const char* spiel) {
  if (lobbyGameBusy()) return;
  if (duelActive()) duelStop();
  if (!strcmp(spiel, "SLALOM")) {
    if (!lobbyIsHost) return;
    handleCommand("SLALOM:START_MATCH");
  } else if (!strcmp(spiel, "RACE")) {
    handleCommand("RACE:START");
  } else if (!strcmp(spiel, "DUEL") || !strncmp(spiel, "DUEL:", 5)) {
    handleCommand("DUEL:START");
  }
}

static bool lobbyStopLocal() {
  bool stopped = false;
  if (duelActive()) { duelStop(); stopped = true; }
  if (currentGame == GAME_RENNEN) { raceAbort(false); stopped = true; }
  if (currentGame == GAME_SLALOM || slalomPhase == SLALOM_BUILD ||
      slalomPhase == SLALOM_COUNTDOWN || slalomPhase == SLALOM_RACING) {
    slalomAbortGame(false);
    stopped = true;
  }
  if (stopped) sendStatus("GAME:STOPPED");
  return stopped;
}

void lobbyOnPacket(const EduPacket* pkt) {
  switch (pkt->cmd) {
    case LOBBY_CMD_INVITE:
      break;
    case LOBBY_CMD_ACCEPT:
    case LOBBY_CMD_DECLINE:
      if (!lobbyInviteSent_ ||
          memcmp(pkt->senderMac, lobbyInviteSentMac, 6) != 0) {
        Serial.println("[LOBBY] ACCEPT/DECLINE verworfen: "
                       "keine eigene Einladung an diesen Absender");
        return;
      }
      break;
    case LOBBY_CMD_LEAVE:
      if (!lobbyHasPeer_ ||
          memcmp(pkt->senderMac, lobbyPeerMac, 6) != 0) {
        Serial.println("[LOBBY] LEAVE verworfen: Absender ist nicht der Lobby-Peer");
        return;
      }
      break;
    case GAME_CMD_START:
      if (lobbyIsHost || !lobbyHasPeer_ ||
          memcmp(pkt->senderMac, lobbyPeerMac, 6) != 0) {
        Serial.println("[LOBBY] GAME_CMD_START verworfen: nicht vom Lobby-Client");
        return;
      }
      break;
    case GAME_CMD_STOP:
      if (!lobbyHasPeer_ ||
          memcmp(pkt->senderMac, lobbyPeerMac, 6) != 0) {
        Serial.println("[LOBBY] GAME_CMD_STOP verworfen: nicht vom Lobby-Peer");
        return;
      }
      break;
    default:
      return;
  }

  switch (pkt->cmd) {
    case LOBBY_CMD_INVITE: {
      if (lobbyPhase == LOBBY_JOINED) {
        Serial.println("[LOBBY] INVITE ignoriert: bereits verbunden");
        return;
      }
      if (lobbyPhase == LOBBY_INVITED &&
          memcmp(pkt->senderMac, lobbyInviterMac, 6) == 0) {
        return;
      }
      memcpy(lobbyInviterMac, pkt->senderMac, 6);
      lobbySetPhase(LOBBY_INVITED);
      sendStatusF("LOBBY:INVITED_BY:%02X:%02X:%02X:%02X:%02X:%02X",
                  pkt->senderMac[0], pkt->senderMac[1], pkt->senderMac[2],
                  pkt->senderMac[3], pkt->senderMac[4], pkt->senderMac[5]);
      Serial.printf("[LOBBY] Einladung empfangen von %02X:%02X:%02X:%02X:%02X:%02X\n",
                    pkt->senderMac[0], pkt->senderMac[1], pkt->senderMac[2],
                    pkt->senderMac[3], pkt->senderMac[4], pkt->senderMac[5]);
      break;
    }

    case LOBBY_CMD_ACCEPT:
      lobbyJoin(pkt->senderMac);
      break;

    case LOBBY_CMD_DECLINE:
      lobbyInviteSent_ = false;
      memset(lobbyInviteSentMac, 0, 6);
      lobbySetPhase(LOBBY_IDLE);
      sendStatus("LOBBY:INVITE_DECLINED");
      Serial.println("[LOBBY] Einladung abgelehnt");
      break;

    case LOBBY_CMD_LEAVE:
      lobbyLeave(false);
      break;

    case GAME_CMD_START: {
      char spiel[16];
      strlcpy(spiel, pkt->data, sizeof(spiel));
      if (!lobbyIsGame(spiel)) {
        sendStatus("LOBBY:ERROR:BAD_GAME");
        return;
      }
      if (lobbyGameBusy() && !lobbyGameRunning(spiel)) {
        sendStatus("GAME:REFUSED:BUSY");
        Serial.printf("[LOBBY] GAME_CMD_START abgelehnt: anderes Spiel laeuft (%s)\n",
                      spiel);
        return;
      }
      sendStatusF("GAME:STARTED:%s", spiel);
      lobbyRunLocalStart(spiel);
      break;
    }

    case GAME_CMD_STOP:
      lobbyStopLocal();
      break;
  }
}

bool lobbyCmd(const String& cmd) {
  if (!cmd.startsWith("LOBBY:")) return false;
  String sub = cmd.substring(6);

  if (sub.startsWith("INVITE:")) {
    if (lobbyPhase == LOBBY_JOINED) {
      sendStatus("LOBBY:ERROR:ALREADY_JOINED");
      return true;
    }
    if (lobbyPhase == LOBBY_INVITING) {
      sendStatus("LOBBY:ERROR:INVITE_PENDING");
      return true;
    }
    uint8_t mac[6];
    if (!parseMac(sub.substring(7), mac)) {
      sendStatus("LOBBY:ERROR:BAD_MAC");
      return true;
    }
    if (memcmp(mac, myMac, 6) == 0) {
      sendStatus("LOBBY:ERROR:SELF");
      return true;
    }
    lobbyInviteSend(mac);
    sendStatus("LOBBY:INVITE_SENT");
    return true;
  }

  if (sub == "ACCEPT") {
    if (lobbyPhase != LOBBY_INVITED) {
      sendStatus("LOBBY:ERROR:NOT_INVITED");
      return true;
    }
    uint8_t mac[6];
    memcpy(mac, lobbyInviterMac, 6);
    lobbyJoin(mac);
    lobbySendTo(mac, LOBBY_CMD_ACCEPT, nullptr, true);
    return true;
  }

  if (sub == "DECLINE") {
    if (lobbyPhase != LOBBY_INVITED) {
      sendStatus("LOBBY:ERROR:NOT_INVITED");
      return true;
    }
    lobbySendTo(lobbyInviterMac, LOBBY_CMD_DECLINE, nullptr, true);
    memset(lobbyInviterMac, 0, 6);
    lobbySetPhase(LOBBY_IDLE);
    sendStatus("LOBBY:DECLINED");
    Serial.println("[LOBBY] Einladung abgelehnt");
    return true;
  }

  if (sub == "LEAVE") {
    if (lobbyPhase == LOBBY_IDLE) {
      sendStatus("LOBBY:ERROR:NOT_JOINED");
      return true;
    }
    lobbyLeave(true);
    return true;
  }

  if (sub.startsWith("START:")) {
    if (lobbyPhase != LOBBY_JOINED || !lobbyIsHost) {
      sendStatus("LOBBY:ERROR:NOT_HOST");
      return true;
    }
    String spiel = sub.substring(6);
    spiel.toUpperCase();
    if (!lobbyIsGame(spiel.c_str())) {
      sendStatus("LOBBY:ERROR:BAD_GAME");
      return true;
    }
    if (lobbyGameBusy()) {
      sendStatus("LOBBY:ERROR:BUSY");
      return true;
    }
    lobbyRunLocalStart(spiel.c_str());
    espNowSendTo(lobbyPeerMac, GAME_CMD_START, spiel.c_str(), 0, 0, true);
    sendStatusF("GAME:STARTED:%s", spiel.c_str());
    return true;
  }

  if (sub == "STOP") {
    if (lobbyPhase != LOBBY_JOINED || !lobbyIsHost) {
      sendStatus("LOBBY:ERROR:NOT_HOST");
      return true;
    }
    lobbyStopLocal();
    espNowSendTo(lobbyPeerMac, GAME_CMD_STOP, nullptr, 0, 0, true);
    return true;
  }

  if (sub == "?") {
    sendStatusF("LOBBY:STATE:%s,%s", lobbyPhaseName(),
                lobbyPhase == LOBBY_JOINED ? (lobbyIsHost ? "HOST" : "CLIENT")
                                           : "-");
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
