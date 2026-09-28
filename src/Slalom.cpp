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
bool slalomIWonLast  = false;   // Ergebnis der letzten Entscheidung (LED-Anzeige in DONE)

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
// Wir selbst haben eine Einladung GESENDET und warten auf das ACK. Ohne diesen
// Zustand konnte JEDES beliebige Geraet ein INVITE_ACK schicken und sich mit
// uns paaren - sogar als Host. Der Zustand macht das ACK beantwortbar.
static bool    slalomInviteSent_     = false;
static uint8_t slalomInviteSentMac[6] = {0,0,0,0,0,0};
static unsigned long slalomInviteSentMs = 0;
static uint8_t slalomInviteRetries    = 0;
// Wer unsere Einladung abgelehnt hat. Solange das gesetzt ist, wird nicht
// erneut eingeladen (sonst poppt das Popup mehrfach wieder auf).
static uint8_t slalomDeclinedBy_[6]   = {0,0,0,0,0,0};

// Einladungs-Timeout: wie lange warten wir auf ein ACK, und wie oft senden wir
// die Einladung erneut? Vorher wurde die Einladung GENAU EINMAL geschickt -
// ein einziges verlorenes ESP-NOW-Paket liess die Web-UI dauerhaft in
// "Einladung gesendet! Warte auf Antwort..." stehen, ohne jeden Ausweg.
static constexpr uint32_t SLALOM_INVITE_RETRY_MS = 2000;  // Abstand zweier Versuche
static constexpr uint8_t  SLALOM_INVITE_MAX_RETRY = 3;    // Versuche insgesamt
static constexpr uint32_t SLALOM_INVITE_TIMEOUT_MS = 10000; // danach aufgeben

bool slalomHasPeer() { return slalomHasPeer_; }
uint8_t* slalomPeerMacPtr() { return slalomPeerMac; }
bool slalomInvitePending() { return slalomInvitePending_; }
uint8_t* slalomInviterMacPtr() { return slalomInviterMac; }
void slalomClearInvite() {
  slalomInvitePending_ = false;
  memset(slalomInviterMac, 0, 6);
}

// Einladung senden und den Wartezustand-armed: Retry in slalomTick().
void slalomInviteSend(const uint8_t destMac[6]) {
  // Alte Ablehnung loeschen! Sonst blockiert eine fruehere Ablehnung genau
  // diese neue Einladung: beim naechsten slalomTick() passt
  // slalomDeclinedBy_ weiterhin zu slalomInviteSentMac, der Retry wird
  // abgebrochen, slalomInviteSent_ faellt auf false - und weil dann auch der
  // Timeout-Zweig nie erreicht wird (er prueft slalomInviteSent_), blieb die
  // Web-UI dauerhaft bei "Warte auf Antwort..." stehen.
  memset(slalomDeclinedBy_, 0, 6);
  memcpy(slalomInviteSentMac, destMac, 6);
  espNowAddSpecificPeer(destMac);   // sicherstellen, dass der Peer in der Tabelle ist
  slalomInviteSent_ = true;
  slalomInviteSentMs = millis();
  slalomInviteRetries = 1;
  slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, destMac);
  Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet (Versuch 1/%d)\n",
    destMac[0], destMac[1], destMac[2], destMac[3], destMac[4], destMac[5],
    SLALOM_INVITE_MAX_RETRY);
}

// Einladungs-Timeout / Retry (aus loop() aufrufen, nicht-blockierend).
void slalomTick() {
  if (!slalomInviteSent_ || slalomHasPeer_) return;
  const unsigned long age = millis() - slalomInviteSentMs;
  if (age < SLALOM_INVITE_RETRY_MS) return;

  if (slalomInviteRetries < SLALOM_INVITE_MAX_RETRY) {
    slalomInviteRetries++;
    slalomInviteSentMs = millis();
    // Ein Retry, nachdem die Gegenseite abgelehnt hat, wuerde das
    // Einladungs-Pop-up dort erneut aufpoppen lassen - bis zu
    // SLALOM_INVITE_MAX_RETRY Mal. Deshalb die Ablehnung im
    // SLALOM_CMD_SLALOM_UNPAIR-Handler protokollieren und den Retry
    // abbrechen.
    if (slalomDeclinedBy_) {
      if (memcmp(slalomDeclinedBy_, slalomInviteSentMac, 6) == 0) {
        slalomInviteSent_ = false;
        memset(slalomDeclinedBy_, 0, 6);
        // Auch der Web-UI Bescheid geben: sie steht sonst weiter auf
        // "Warte auf Antwort...", obwohl hier abgebrochen wurde.
        sendStatus("SLALOM:INVITE_DECLINED");
        Serial.println("[SLALOM] Einladung abgelehnt - kein weiterer Versuch");
        return;
      }
    }
    slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, slalomInviteSentMac);
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
  slalomInviteSent_ = false;  // ACK ist da, wartezustand beendet
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
    // ACHTUNG: der ESP-NOW-Peer-Eintrag wird nur geloescht, wenn dieser
    // Geraet nicht zugleich der Rennen-Gegner ist. slalomAutoScan findet den
    // Slalom-Partner ueber den RACE-Scan, d.h. beide Pairings zeigen
    // normalerweise auf dieselbe MAC. Ein pauschales esp_now_del_peer()
    // entfernte dort den Peer aus der Tabelle, waehrend hasPairedPeer im
    // EspNow-Modul weiter true blieb - jedes raceSendPacket() scheiterte
    // danach still mit ESP_ERR_NO_PEER und das Rennen war ohne Neustart
    // tot. Jetzt bleibt der Eintrag erhalten, wenn er fuer das Rennen
    // gebraucht wird.
    const bool isRacePeer = hasPairedPeer && memcmp(pairedPeer, slalomPeerMac, 6) == 0;
    if (!isRacePeer) {
      esp_now_del_peer(slalomPeerMac);
    } else {
      Serial.println("[SLALOM] Peer bleibt als Rennen-Peer in der ESP-NOW-Tabelle");
    }
  }
  slalomHasPeer_       = false;
  slalomIsHost        = false;
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
  // Nicht nur slalomRunning pruefen: es gibt auch den Fall, dass waehrend des
  // Slaloms ein SOLO-Programm (progActive, ohne slalomRunning) laeuft. Ein
  // WIN/ABORT-Paket beendete dann nur den Slalom-Zustand, progStop() wurde
  // nie gerufen - der Roboter fuhr das Solo-Programm weiter, waehrend die
  // Web-UI schon "Gegner hat gewonnen" anzeigte.
  if (slalomRunning || progActive) {
    progStop();
    slalomRunning = false;
    currentGame = GAME_NONE;
  }
  slalomResetRuntime();
  slalomWon = true;   // Spiel ist entschieden (verhindert z.B. spaetes MISS)
  // Fuer die LED-Anzeige in Phase DONE: resetRuntime() loescht slalomFinished,
  // deshalb das Ergebnis hier separat und stabil ablegen.
  slalomIWonLast = iWon;
  slalomSetPhase(SLALOM_DONE);
  sendStatus(iWon ? "SLALOM:WIN:ME" : "SLALOM:WIN:OPP");
  Serial.printf("[SLALOM] Spielende: %s\n", iWon ? "ICH habe gewonnen!" : "Gegner hat gewonnen");
}

// Spiel abgebrochen (manuell): zurueck zu PAIRED, Peer informieren.
void slalomAbortGame() {
  // Siehe slalomEndGame(): auch ein laufendes Solo-Programm muss stoppen.
  if (slalomRunning || progActive) {
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
  // ---- ZENTRALE ABSENDER-PRUEFUNG ---------------------------------
  // Vorher pruefte NUR der UNPAIR-Fall den Absender. Jedes beliebige
  // ESP-NOW-Geraet in Reichweite konnte dadurch:
  //   WIN     -> das Spiel sofort beenden und sich zum Gewinner erklaeren
  //   ABORT   -> das laufende Spiel abbrechen
  //   START   -> dem Client eine fremde Zielfarbe aufzwingen
  //   PROG    -> den Countdown erzwingen
  //   INVITE_ACK -> sich paaren, OHNE jemals eingeladen worden zu sein -
  //                und dabei sogar zum HOST werden
  // Zwei Nachrichten sind davon ausgenommen, und nur aus guten Gruenden:
  //   * INVITE      - kommt per Definition von einem noch Ungepaarten.
  //   * INVITE_ACK  - nur wenn WIR selbst eine Einladung an genau diese MAC
  //                   gesendet haben (slalomInviteSent_). Das beweist, dass
  //                   das ACK eine Antwort auf unsere eigene Einladung ist.
  // Hinweis: pkt->senderMac ist hier bereits die vom Wi-Fi-Stack gemeldete
  // MAC (siehe onEspNowRecv) und damit nicht mehr frei waehlbar.
  switch (pkt->slalomCmd) {
    case SLALOM_CMD_INVITE:
      break;  // Absender wird weiter unten (Phase-Pruefung) bewertet
    case SLALOM_CMD_INVITE_ACK:
      break;  // eigener Ast weiter unten (nur nach eigener Einladung gueltig)
    case SLALOM_CMD_SLALOM_UNPAIR:
      // Gilt fuer GEGNER unseres Slalom-Peers UND fuer eine Ablehnung, wenn
      // wir selbst gerade eine Einladung offen haben. Vorher brauchte der
      // Handler zwingend einen gepaarten Peer - eine Ablehnung kam also gar
      // nicht an, und die Gegenseite blieb in IDLE, waehrend wir ewig auf
      // eine Antwort warteten.
      if (slalomInviteSent_ &&
          memcmp(pkt->senderMac, slalomInviteSentMac, 6) == 0) {
        slalomInviteSent_ = false;
        memcpy(slalomDeclinedBy_, pkt->senderMac, 6);
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

  switch (pkt->slalomCmd) {
    case SLALOM_CMD_PROG:
      // Absender ist jetzt garantiert der Slalom-Peer. "Bereit" wird in
      // JEDER Phase gemerkt, nicht nur in BUILD: kam das PROG waehrend des
      // eigenen Countdowns an, ging es sonst verloren und nach dem Retry
      // warteten beide vergeblich aufeinander. slalomMaybeStart() ist
      // selbststaendig no-op, solange wir nicht in BUILD sind.
      if (pkt->slalomLen > 0) {
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
      // Absender ist hier bereits als das Geraet bestaetigt, auf dessen
      // Einladung wir gewartet haben (Pruefung weiter oben). Initiator = Host.
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
