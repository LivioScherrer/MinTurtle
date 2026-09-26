// EduBot V2 - Farbmarker-Slalom: Implementierung
// Logik 1:1 aus Edu_V2.cpp uebernommen (kein Verhaltenswechsel).
#include "Slalom.h"

#include "BLEManager.h"   // sendStatus
#include "EspNow.h"       // esp_now_send, espNowAddSpecificPeer, myMac, pairedPeer...
#include "GameEngine.h"   // currentGame
#include "LedStrip.h"     // ledStripClear
#include "ProgGame.h"     // progSetFromString, progRunSlalom, progStop

// =====================================================
// Zustand
// =====================================================
SlalomPhase slalomPhase = SLALOM_IDLE;
static const char* const SLALOM_PHASE_NAMES[6] = {
  "IDLE", "PAIRED", "BUILD", "COUNTDOWN", "RACING", "DONE"
};

char slalomTargetColor[16] = "";
char slalomMyProgram[PROG_MAX_ENC + 1] = "";
bool slalomRunning   = false;   // faehrt gerade (RACE)
bool slalomIsHost    = false;   // Initiator = Host
bool slalomReady     = false;   // ich habe Bereit gedrueckt
bool slalomOppReady  = false;   // Gegner hat Bereit gedrueckt (PROG empfangen)
bool slalomFinished  = false;   // mein Roboter hat die Zielfarbe erreicht (von progRunSlalom)
bool slalomWon       = false;   // das Spiel ist entschieden (jemand hat getroffen)

// Retry-Logik (niemand hat getroffen -> neue Runde mit gleicher Farbe)
bool slalomMyRunDone  = false;  // mein Durchlauf ist zu Ende (ohne Treffer)
bool slalomOppRunDone = false;  // Gegner-Durchlauf zu Ende (MISS empfangen)
unsigned long slalomOppWaitStart = 0; // seit wann warte ich auf das Gegner-Ergebnis

// GO-Sync: NUR der HOST ist der Zeitgeber (3s nachdem beide bereit sind).
unsigned long slalomGoTimer = 0;   // Absoluter GO-Zeitpunkt (millis)
bool slalomGoPending = false;      // GO-Timer laeuft

// Farbnamen für den Slalom-Modus (4 Farben)
static const char* const SLALOM_COLOR_NAMES[4] = { "ROT", "GELB", "GRUEN", "BLAU" };

// =====================================================
// SLALOM PAIRING (eigenstaendig vom Race-Pairing)
// Einladungs-Flow:
//   1. Web A sendet SLALOM:INVITE:<MAC_B>  -> wir schicken CMD_INVITE an B
//   2. Web B empfaengt SLALOM:INVITED_BY:.. -> Pop-up, B klickt "Annehmen"
//   3. Web B sendet SLALOM:INVITE_ACCEPT   -> wir schicken CMD_INVITE_ACK an A
//   4. Beide Seiten markieren sich als gepaart; Initiator (A) = Host
// =====================================================
static bool    slalomHasPeer_     = false;     // haben wir einen Slalom-Peer?
static uint8_t slalomPeerMac[6]  = {0,0,0,0,0,0}; // MAC des Slalom-Peers
// MAC des Initiators (von dem die Einladung kam) - bei Empfang von CMD_INVITE
static uint8_t slalomInviterMac[6] = {0,0,0,0,0,0};
// Pending: wir haben eine Einladung bekommen, aber Web-Accept steht noch aus
static bool    slalomInvitePending_ = false;

bool slalomHasPeer() { return slalomHasPeer_; }
uint8_t* slalomPeerMacPtr() { return slalomPeerMac; }
bool slalomInvitePending() { return slalomInvitePending_; }
uint8_t* slalomInviterMacPtr() { return slalomInviterMac; }
void slalomClearInvite() {
  slalomInvitePending_ = false;
  memset(slalomInviterMac, 0, 6);
}

// Phase-Setzer (Logging + Status-Update zentral)
void slalomSetPhase(SlalomPhase p) {
  if (slalomPhase == p) return;
  Serial.printf("[SLALOM] Phase %s -> %s\n", SLALOM_PHASE_NAMES[slalomPhase], SLALOM_PHASE_NAMES[p]);
  slalomPhase = p;
}

// Laufzeit-Zustand zuruecksetzen (ohne Pairing/Phase anzufassen).
// WICHTIG: slalomTargetColor bleibt ERHALTEN - bei einem Retry (niemand
// getroffen) wird mit DERSELBEN Zielfarbe weitergespielt.
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

// Slalom-Pairing aufbauen (intern, von beiden Seiten gerufen)
void slalomPairWith(const uint8_t mac[6], bool initiator) {
  memcpy(slalomPeerMac, mac, 6);
  slalomHasPeer_ = true;
  slalomIsHost  = initiator;  // Initiator = Host
  // ESP-NOW Peer registrieren (falls noch nicht)
  espNowAddSpecificPeer(mac);
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';  // frisches Spiel, noch keine Zielfarbe
  slalomSetPhase(SLALOM_PAIRED);
  // Status ans Web
  sendStatus(initiator ? "SLALOM:INVITE_PAIRED:HOST" : "SLALOM:INVITE_PAIRED:CLIENT");
  Serial.printf("[SLALOM] Pairing mit %02X:%02X:%02X:%02X:%02X:%02X (Initiator=%d)\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], initiator);
}

// Slalom-spezifisches Unpair (nur Slalom-State, Race-Pairing unberuehrt)
void slalomUnpair() {
  if (slalomHasPeer_) {
    esp_now_del_peer(slalomPeerMac);
  }
  slalomHasPeer_       = false;
  slalomIsHost        = false;
  slalomInvitePending_ = false;
  memset(slalomPeerMac, 0, 6);
  memset(slalomInviterMac, 0, 6);
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  sendStatus("SLALOM:UNPAIRED");
  Serial.println("[SLALOM] Slalom-Unpair");
}

void slalomSendPacket(uint8_t cmd, const char* data) {
  RacePacket pkt = {};
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = 0;
  pkt.flags = 0;
  pkt.slalomCmd = cmd;
  if (data) {
    size_t len = strlen(data);
    if (len > sizeof(pkt.slalomData) - 1) len = sizeof(pkt.slalomData) - 1;
    memcpy(pkt.slalomData, data, len);
    pkt.slalomLen = len;
  } else {
    pkt.slalomLen = 0;
  }
  if (slalomHasPeer_) {
    esp_now_send(slalomPeerMac, (uint8_t*)&pkt, sizeof(pkt));
  } else if (hasPairedPeer) {
    esp_now_send(pairedPeer, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  }
}

// An eine bestimmte MAC senden (fuer Slalom-Invite vor dem Pairing)
void slalomSendPacketTo(uint8_t cmd, const char* data, const uint8_t destMac[6]) {
  if (memcmp(destMac, myMac, 6) == 0) return;  // nicht an uns selbst
  RacePacket pkt = {};
  pkt.magic = 0xA5;
  memcpy(pkt.senderMac, myMac, 6);
  pkt.score = 0;
  pkt.flags = 0;
  pkt.slalomCmd = cmd;
  if (data) {
    size_t len = strlen(data);
    if (len > sizeof(pkt.slalomData) - 1) len = sizeof(pkt.slalomData) - 1;
    memcpy(pkt.slalomData, data, len);
    pkt.slalomLen = len;
  } else {
    pkt.slalomLen = 0;
  }
  esp_now_send(destMac, (uint8_t*)&pkt, sizeof(pkt));
}

// Spiel vorbei, weil JEMAND die Zielfarbe erreicht hat: stoppen, DONE-Phase.
void slalomEndGame(bool iWon) {
  if (slalomRunning) {
    progStop();
    slalomRunning = false;
    currentGame = GAME_NONE;
  }
  slalomResetRuntime();
  slalomWon = true;   // Spiel ist entschieden (verhindert z.B. spaetes MISS)
  slalomSetPhase(SLALOM_DONE);
  sendStatus(iWon ? "SLALOM:WIN:ME" : "SLALOM:WIN:OPP");
  Serial.printf("[SLALOM] Spielende: %s\n", iWon ? "ICH habe gewonnen!" : "Gegner hat gewonnen");
}

// Spiel abgebrochen (manuell): zurueck zu PAIRED, Peer informieren.
void slalomAbortGame() {
  if (slalomRunning) {
    progStop();
    slalomRunning = false;
    currentGame = GAME_NONE;
  }
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';  // keine alte Zielfarbe mehr anzeigen
  slalomSetPhase(SLALOM_PAIRED);
  sendStatus("SLALOM:STOPPED");
  Serial.println("[SLALOM] Spiel abgebrochen");
  if (slalomHasPeer_) {
    slalomSendPacket(SLALOM_CMD_ABORT, nullptr);
  }
}

// NUR der Host ruft dies auf (Web-Befehl SLALOM:START_MATCH):
// Zufallsfarbe waehlen und den Client in die Bauphase holen (CMD_START 3x,
// damit ein einzelnes verlorenes ESP-NOW-Paket nicht den Client haengen
// laesst). Die Farbe wird dem Web ueber SLALOM:TARGET gemeldet.
void slalomStartGame() {
  if (!slalomHasPeer_) {
    Serial.println("[SLALOM] Start blockiert: kein Peer gepaart");
    return;
  }
  slalomResetRuntime();
  slalomSetPhase(SLALOM_BUILD);
  uint8_t idx = random(0, 4);
  strlcpy(slalomTargetColor, SLALOM_COLOR_NAMES[idx], sizeof(slalomTargetColor));
  for (int i = 0; i < 3; i++) {
    slalomSendPacket(SLALOM_CMD_START, slalomTargetColor);
    delay(40);
  }
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor);
  sendStatus(msg);
  sendStatus("SLALOM:MATCH_STARTED");
  Serial.printf("[SLALOM] Host startete Spiel, Zielfarbe %s (3x gesendet)\n", slalomTargetColor);
}

// Client: Zielfarbe vom Host empfangen (CMD_START) -> Phase BUILD, Farbe anzeigen.
void slalomOnStartReceived(const char* col) {
  // Nur annehmen, wenn gerade KEIN Spiel laeuft (PAIRED, DONE oder IDLE).
  // So stoeren verspaetete/doppelte START-Pakete kein laufendes Spiel.
  if (slalomPhase == SLALOM_BUILD || slalomPhase == SLALOM_COUNTDOWN ||
      slalomPhase == SLALOM_RACING) return;
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

// Bereit gedrueckt: Programm speichern, Peer informieren, ggf. gemeinsam starten.
void slalomSetProgram(const char* enc) {
  strlcpy(slalomMyProgram, enc, sizeof(slalomMyProgram));
  slalomReady = true;
  sendStatus("SLALOM:READY");
  slalomSendPacket(SLALOM_CMD_PROG, slalomMyProgram);
  slalomMaybeStart();
}

// Beide Spieler bereit? Dann zaehlt der HOST einen 3s-Countdown in der
// Firmware (nicht im Web - dort haetten beide Seiten unabhaengige
// BLE-Jitter) und sendet danach SLALOM_CMD_GO an den Client. Der Client
// startet sofort bei Empfang des GO; als Fallback (Paketverlust) startet er
// nach 4.5s eigenstaendig.
void slalomMaybeStart() {
  if (slalomPhase != SLALOM_BUILD) return;
  if (slalomRunning || slalomGoPending) return;
  if (!slalomReady || !slalomOppReady) return;
  if (slalomTargetColor[0] == '\0') return;
  slalomSetPhase(SLALOM_COUNTDOWN);
  sendStatus("SLALOM:BOTH_READY");
  if (slalomIsHost) {
    slalomGoTimer = millis() + 3000;
  } else {
    slalomGoTimer = millis() + 4500;  // Fallback, falls GO verloren geht
  }
  slalomGoPending = true;
  Serial.printf("[SLALOM] Beide bereit. %s-Countdown 3s bis GO\n", slalomIsHost ? "Host" : "Client");
}

// Rennen ausfuehren: Programm laden und abfahren. Blockiert bis der Lauf
// fertig ist (oder die Zielfarbe stabil erkannt wurde -> slalomFinished).
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
  // LED-Ring leeren: die Zielfarbe auf dem Ring wuerde den Farbsensor
  // am Boden verwirren (das Ziel liegt ja auf dem Boden).
  ledStripClear();
  Serial.printf("[SLALOM] RACE! Ziel: %s\n", slalomTargetColor);
  progSetFromString(slalomMyProgram);
  progRunSlalom(slalomTargetColor);
  currentGame = GAME_NONE;
  slalomRunning = false;   // wichtig: sonst blockiert slalomCheckRetry im loop()
  if (slalomWon) return;   // Gegner hat waehrenddessen gewonnen -> kein MISS senden

  if (slalomFinished) {
    // Treffer! Spiel ist sofort vorbei.
    slalomSendPacket(SLALOM_CMD_WIN, slalomTargetColor);
    slalomEndGame(true);
  } else {
    // Kein Treffer: Durchlauf-Ergebnis melden und auf Gegner warten.
    slalomMyRunDone = true;
    slalomSendPacket(SLALOM_CMD_MISS, nullptr);
    if (slalomOppRunDone) {
      // Gegner war schon fertig -> zurueck in die Bauphase
      slalomResetRuntime();
      slalomSetPhase(SLALOM_BUILD);
      sendStatus("SLALOM:RETRY");
    } else {
      slalomOppWaitStart = millis();
    }
  }
}

// 15s-Sicherheits-Timeout in der Retry-Phase (loop()):
// Haben BEIDE gemeldet, dass sie die Farbe nicht erreicht haben, oder wartet
// ein Spieler laenger als 15s auf das Gegner-Ergebnis, geht es zurueck in die
// Bauphase. Gleiche Zielfarbe - die Spieler verbessern ihr Programm.
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

void onSlalomPacket(const RacePacket* pkt) {
  switch (pkt->slalomCmd) {
    case SLALOM_CMD_PROG:
      if (pkt->slalomLen > 0 && slalomPhase == SLALOM_BUILD) {
        slalomOppReady = true;
        sendStatus("SLALOM:OPP_PROG");
        slalomMaybeStart();
      }
      break;

    case SLALOM_CMD_GO: {
      // Host-Startsignal: perfekter Sync. Der Host hat seinen 3s-Countdown
      // in der Firmware abgeschlossen und uns per ESP-NOW gestartet.
      slalomGoPending = false;
      if (slalomPhase == SLALOM_COUNTDOWN && !slalomRunning) {
        slalomExecute();
      }
      break;
    }

    case SLALOM_CMD_START: {
      // Host: Spiel gestartet + Zielfarbe (3x gesendet gegen Paketverlust).
      if (slalomIsHost) break;   // nur der Client verarbeitet das
      char buf[16] = {0};
      size_t len = pkt->slalomLen < 15 ? pkt->slalomLen : 15;
      memcpy(buf, pkt->slalomData, len);
      slalomOnStartReceived(buf);
      break;
    }

    case SLALOM_CMD_WIN:
      // Gegner hat die Zielfarbe erreicht -> Spiel ist SOFORT vorbei.
      // Guard: habe ich (fast gleichzeitig) selbst gewonnen, nicht ueberschreiben.
      if (slalomWon) break;
      slalomEndGame(false);
      break;

    case SLALOM_CMD_MISS:
      // Gegner-Durchlauf ohne Treffer beendet -> Retry-Check (wenn beide fertig).
      slalomOppRunDone = true;
      sendStatus("SLALOM:OPP_MISS");
      if (slalomMyRunDone) {
        slalomCheckRetry();
      }
      break;

    case SLALOM_CMD_ABORT:
      // Gegner hat das Spiel abgebrochen -> zurueck zu PAIRED.
      slalomAbortGame();
      break;

    case SLALOM_CMD_INVITE:
      // Einladung empfangen - Slalom-Phase muss IDLE sein
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
      // Initiator: Empfaenger hat die Einladung angenommen
      if (!slalomHasPeer_) {
        slalomPairWith(pkt->senderMac, true);  // Initiator = Host
      }
      break;

    case SLALOM_CMD_SLALOM_UNPAIR:
      // Peer hat das Slalom-Match beendet - bei uns auch unpairen
      if (slalomHasPeer_ && memcmp(pkt->senderMac, slalomPeerMac, 6) == 0) {
        Serial.println("[SLALOM] Peer hat Slalom-Unpair gesendet");
        slalomUnpair();
      }
      break;

    default:
      break;
  }
}
