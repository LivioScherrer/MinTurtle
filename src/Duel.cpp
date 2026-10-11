// EduBot V2 - Duell - Implementierung. Siehe Duel.h.
#include "Duel.h"
#include "BLEManager.h"

// Zielpunktzahl und Gegner-Timeout: gleiche Grosszuegigkeit wie beim Rennen
// (RACE_PEER_TIMEOUT_MS = 45 s), damit eine kurze Denkpause keinen Abbruch
// erzeugt, ein abgestuerzter Gegner aber sichtbar wird.
constexpr int16_t  DUEL_TARGET     = 10;
constexpr uint32_t DUEL_TIMEOUT_MS = 45000;

static bool     s_active = false;
static int16_t  s_my = 0;
static int16_t  s_opp = 0;
static bool     s_won = false;
static bool     s_lost = false;
static uint32_t s_lastOppMs = 0;
static bool     s_timeoutSent = false;

bool duelActive() { return s_active; }

// Punktestand senden. winner=true setzt das W-Flag (Spiel entschieden).
static void duelSend(bool winner) {
  if (!espNowReady || !hasPairedPeer) return;
  RacePacket pkt = {};
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.slalomCmd = SLALOM_CMD_DUEL;
  snprintf(pkt.slalomData, sizeof(pkt.slalomData), "%d;%c",
           (int)s_my, winner ? 'W' : 'S');
  pkt.slalomLen = (uint8_t)strlen(pkt.slalomData);
  esp_now_send(pairedPeer, (uint8_t*)&pkt, sizeof(pkt));
}

void duelStop() {
  if (!s_active) return;
  s_active = false;
  sendStatus("DUEL:STOPPED");
}

void duelAddPoints(int points) {
  if (!s_active || s_won || s_lost) return;
  s_my += (int16_t)points;
  if (s_my < 0) s_my = 0;
  if (s_my >= DUEL_TARGET) {
    s_won = true;
    duelSend(true);
    sendStatus("DUEL:WON");
  } else {
    duelSend(false);
    sendStatusF("DUEL:ME:%d", (int)s_my);
  }
}

void duelOnPacket(const RacePacket* pkt) {
  // Nur der gepaarte Gegner darf Punkte melden - gleiche Regel wie beim
  // Rennen (kein Fremdgeraet kann ein Duell "gewinnen").
  if (!hasPairedPeer) return;
  if (memcmp(pkt->senderMac, pairedPeer, 6) != 0) return;
  if (!s_active) return;
  int score = 0;
  char flag = 'S';
  if (sscanf(pkt->slalomData, "%d;%c", &score, &flag) != 2) return;
  s_opp = (int16_t)score;
  s_lastOppMs = millis();
  s_timeoutSent = false;
  sendStatusF("DUEL:OPP:%d", (int)s_opp);
  if (flag == 'W' && !s_won) {
    s_lost = true;
    sendStatus("DUEL:LOST");
  }
}

void duelTick() {
  if (!s_active || s_won || s_lost || s_timeoutSent) return;
  if (millis() - s_lastOppMs > DUEL_TIMEOUT_MS) {
    s_timeoutSent = true;
    sendStatus("DUEL:TIMEOUT");
  }
}

bool duelCmd(const String& cmd) {
  if (!cmd.startsWith("DUEL:")) return false;
  const String sub = cmd.substring(5);

  if (sub == "START") {
    if (!hasPairedPeer) { sendStatus("DUEL:NOPAIR"); return true; }
    s_active = true;
    s_my = 0;
    s_opp = 0;
    s_won = false;
    s_lost = false;
    s_lastOppMs = millis();
    s_timeoutSent = false;
    duelSend(false);
    sendStatusF("DUEL:STARTED:%d", (int)DUEL_TARGET);
    return true;
  }

  if (sub == "STOP") {
    if (s_active) {
      duelStop();
    } else {
      sendStatus("DUEL:STOPPED");
    }
    return true;
  }

  if (sub == "?") {
    sendStatusF("DUEL:STATE:%d,%d,%d", s_active ? 1 : 0, (int)s_my, (int)s_opp);
    return true;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
  return true;
}
