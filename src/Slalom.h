// EduBot V2 - Farbmarker-Slalom (Multiplayer)
// =====================================================
// Ausgelagert aus Edu_V2.cpp: vollstaendige Zustandsmaschine inkl. ESP-NOW-
// Sync, Pairing und Retry-Logik. Die main ruft nur noch die oeffentlichen
// Funktionen auf (siehe Slalom.h).
// =====================================================
// Regeln:
//   * KEINE Punkte / Runden. Es geht nur darum, wer ZUERST die Zielfarbe erreicht.
//   * Sobald ein Roboter die Zielfarbe stabil erkennt, ist das Spiel SOFORT
//     vorbei: dieser Roboter gewinnt, beide stoppen (Phase DONE).
//   * Hat NIEMAND getroffen (beide Programme ohne Treffer zu Ende), geht es
//     automatisch zurueck in die Bauphase (gleiche Zielfarbe) - die Spieler
//     duerfen ihr Programm verbessern. So lange, bis jemand trifft.
//
// Ablauf / Sync (bewusst simpel + robust, KEINE ACK-Kette):
//   1. Pairing per Invite/Accept (Initiator = Host).
//   2. Host drueckt "Spiel starten" -> waehlt Zufallsfarbe, sendet
//      SLALOM_CMD_START (+Farbe) 3x an den Client (Paketverlust), zeigt die
//      Farbe lokal an. Beide gehen in die Bauphase (BUILD).
//   3. Jeder Spieler baut sein Programm und drueckt "Bereit" -> sendet
//      SLALOM_CMD_PROG an den Peer. Sobald BEIDE bereit sind (COUNTDOWN):
//      * Host zaehlt einen 3s-Countdown in der Firmware und sendet danach
//        SLALOM_CMD_GO (3x) an den Client; beide fahren (fast) gleichzeitig.
//      * Client startet sofort bei Empfang von GO; Fallback nach 4.5s.
//   4. RACE: beide fahren ihr Programm. Wer die Farbe erreicht, sendet
//      SLALOM_CMD_WIN -> beide stoppen, Gewinner wird angezeigt (DONE).
//      Wer ohne Treffer fertig wird, sendet SLALOM_CMD_MISS. Haben BEIDE
//      "miss" gemeldet (oder 15s Sicherheits-Timeout), geht es zurueck zu BUILD.
#pragma once

#include <Arduino.h>
#include <EspNow.h>   // RacePacket, SLALOM_CMD_*, espNowSend
#include <ProgGame.h> // PROG_MAX_ENC

enum SlalomPhase : uint8_t {
  SLALOM_IDLE       = 0,   // nicht gepaart
  SLALOM_PAIRED     = 1,   // gepaart, wartet auf Spielstart (Host)
  SLALOM_BUILD      = 2,   // Zielfarbe bekannt, Programm bauen
  SLALOM_COUNTDOWN  = 3,   // beide bereit, Countdown laeuft
  SLALOM_RACING     = 4,   // Roboter fahren
  SLALOM_DONE       = 5    // Gewinner gefunden, Spiel vorbei
};

// ---- Laufzeit-Zustand (von ProgGame/loop beobachtet) ----
extern bool slalomRunning;     // fährt gerade (RACE)
extern bool slalomFinished;    // mein Roboter hat die Zielfarbe erreicht
extern bool slalomWon;         // Spiel ist entschieden (jemand hat getroffen)
extern bool slalomMyRunDone;   // mein Durchlauf zu Ende (ohne Treffer)
extern bool slalomOppRunDone;  // Gegner-Durchlauf zu Ende (MISS empfangen)
extern bool slalomIsHost;      // Initiator = Host
extern bool slalomReady;       // ich habe Bereit gedrueckt
extern bool slalomOppReady;    // Gegner hat Bereit gedrueckt
extern char  slalomTargetColor[16];   // Zielfarbe (leer = keine)
extern char  slalomMyProgram[PROG_MAX_ENC + 1];
extern SlalomPhase slalomPhase;
// GO-Sync: nur der HOST ist Zeitgeber (3s nach "beide bereit").
extern unsigned long slalomGoTimer;   // absoluter GO-Zeitpunkt (millis)
extern bool slalomGoPending;          // GO-Timer läuft

// ---- Phasen-Steuerung / Reset ----
void slalomSetPhase(SlalomPhase p);
void slalomResetRuntime();   // Laufzeit-Zustand zuruecksetzen (Pairing/Phase bleiben)

// ---- Pairing (eigenstaendig vom Race-Pairing) ----
bool slalomHasPeer();
uint8_t* slalomPeerMacPtr();
void slalomPairWith(const uint8_t mac[6], bool initiator);
void slalomUnpair();

// ---- ESP-NOW ----
void slalomSendPacket(uint8_t cmd, const char* data);
void slalomSendPacketTo(uint8_t cmd, const char* data, const uint8_t destMac[6]);
void onSlalomPacket(const RacePacket* pkt);

// ---- Spiel-Ablauf ----
void slalomStartGame();            // nur Host: Zufallsfarbe, Client in BUILD
void slalomOnStartReceived(const char* col);  // Client: Zielfarbe empfangen
void slalomSetProgram(const char* enc);       // Bereit gedrueckt
void slalomMaybeStart();                     // beide bereit? -> COUNTDOWN
void slalomExecute();                        // Lauf starten (blockierend)
void slalomCheckRetry();                     // Retry-Timeout (aus loop)
void slalomEndGame(bool iWon);
void slalomAbortGame();

// ---- Pending-Invite (Web-Pop-up) ----
bool slalomInvitePending();
uint8_t* slalomInviterMacPtr();
void slalomClearInvite();
