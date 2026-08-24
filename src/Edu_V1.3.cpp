// =====================================================
// EduBot V1.3R - Main Entry Point
// Xiao ESP32-S3 + DRV8834 + TCS34725 + BLE + ESP-NOW
// =====================================================
// Modulare Firmware - alle Logik in separaten .h/.cpp Dateien
// =====================================================

#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_task_wdt.h>

#include "Config.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "BLEManager.h"
#include "Drive.h"
#include "EspNow.h"
#include "MemoryGame.h"
#include "GameEngine.h"
#include "LineFollower.h"
#include "ProgGame.h"
#include "LedStrip.h"

// =====================================================
// GLOBALE KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 1;

// IR-Sensor: IR_PIN kommt aus Config.h (D8 = GPIO7)
constexpr unsigned long IR_NOTIFY_MS      = 250;  // alle 250ms Wert an Web senden

// =====================================================
// GAME STATE MACHINE
// =====================================================
GameMode currentGame = GAME_NONE;
bool driveLockedByGame() { return currentGame != GAME_NONE; }

// =====================================================
// SPIEL-BEWEGUNGS-ENGINE (blockierend)
// =====================================================
// Einheitliche Zeiten: jedes Spiel (Memory, Quiz, Rennen) nutzt
// dieselbe Vor/Zurueck- und Drehdauer -> konsistenter Bewegungsablauf.
// Beschleunigung/Bremsen uebernimmt AccelStepper (kein Wheelie, kein Schraeglauf).
constexpr float GAME_DRIVE_GAIN   = 1.0f;
constexpr uint32_t GAME_TICK_MS    = 1;    // Polling-Intervall fuer motorRun() waehrend blockierender Moves
const uint32_t GAME_MOVE_MS       = 360;  // Vor/Zurueck-Fahrt in ALLEN Spielen
const uint32_t GAME_COLOR_READ_MS = 250;  // Pause waehrend Farbsensor liest
constexpr uint8_t QUIZ_MAX_COLOR_USE = 4;  // jede Farbe max. so oft im Quiz verwendbar

bool gameMoveBusy = false;

// Wartet bis beide Stepper wirklich stehen (max 1s). motorRun() wird
// weiterhin aufgerufen, damit die Schritte waehrend der Verzoegerung
// noch ausgefuehrt werden.
static void waitForStop(uint32_t startMs, uint32_t maxExtraMs = 1000) {
  uint32_t extra = millis();
  while ((stepperL.isRunning() || stepperR.isRunning()) && millis() - extra < maxExtraMs) {
    esp_task_wdt_reset();
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

void gameDriveForward(uint32_t ms) {
  gameMoveBusy = true;
  const float target = FIXED_DRIVE_SPEED * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop(start);
  gameMoveBusy = false;
}

void gameDriveBackward(uint32_t ms) {
  gameMoveBusy = true;
  const float target = -FIXED_DRIVE_SPEED * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop(start);
  gameMoveBusy = false;
}

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  uint32_t start = millis();
  while (millis() - start < (uint32_t)ms) {
    esp_task_wdt_reset();
    motorRun();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Positionsbasiert: exakte Strecke fahren (z.B. 3cm pro Farbe im Quiz).
// Beruecksichtigt config.leftSign/rightSign fuer die Verkabelung.
void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  // Feste Geschwindigkeit fuer diesen Move
  prepPositionMove(stepperL, steps * static_cast<int32_t>(config.leftSign), FIXED_DRIVE_SPEED);
  prepPositionMove(stepperR, steps * static_cast<int32_t>(config.rightSign), FIXED_DRIVE_SPEED);
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) {
      stepperL.stop();
      stepperR.stop();
      gameMoveBusy = false;
      return;
    }
    stepperL.run();
    stepperR.run();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: fahre n Schritte ohne currentGame-Abhaengigkeit, mit Timeout.
// Wird vom STEPS:-Befehl genutzt, um den richtigen Wert fuer 3cm zu finden.
void testDriveSteps(int32_t steps) {
  prepPositionMove(stepperL, steps * static_cast<int32_t>(config.leftSign), FIXED_DRIVE_SPEED);
  prepPositionMove(stepperR, steps * static_cast<int32_t>(config.rightSign), FIXED_DRIVE_SPEED);
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    esp_task_wdt_reset();
    stepperL.run();
    stepperR.run();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.stop();
    stepperR.stop();
  }
}

// Positionsbasierte Drehung: jeder Motor 'steps' in entgegengesetzte Richtung.
void gameTurnLeftSteps(int32_t steps) {
  gameMoveBusy = true;
  prepPositionMove(stepperL, -steps * static_cast<int32_t>(config.leftSign), FIXED_DRIVE_SPEED);
  prepPositionMove(stepperR,  steps * static_cast<int32_t>(config.rightSign), FIXED_DRIVE_SPEED);
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}
void gameTurnRightSteps(int32_t steps) {
  gameMoveBusy = true;
  prepPositionMove(stepperL,  steps * static_cast<int32_t>(config.leftSign), FIXED_DRIVE_SPEED);
  prepPositionMove(stepperR, -steps * static_cast<int32_t>(config.rightSign), FIXED_DRIVE_SPEED);
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: drehe auf der Stelle (rechts) um 'steps' Schritte pro Rad.
void testTurnSteps(int32_t steps) {
  prepPositionMove(stepperL,  steps * static_cast<int32_t>(config.leftSign), FIXED_DRIVE_SPEED);
  prepPositionMove(stepperR, -steps * static_cast<int32_t>(config.rightSign), FIXED_DRIVE_SPEED);
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    esp_task_wdt_reset();
    stepperL.run(); stepperR.run();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.stop(); stepperR.stop();
  }
}

// =====================================================
// KORRIGIEREN (Farb-Quiz)
// =====================================================
constexpr uint8_t KORRIGIEREN_MAX = 16;
char korrigierenExpected[KORRIGIEREN_MAX][16];
char korrigierenFound[KORRIGIEREN_MAX][16];
uint8_t korrigierenLen = 0;
bool korrigierenRunning = false;

void korrigierenExecute() {
  if (korrigierenLen == 0) { sendStatus("QUIZ:RESULT:OK,0,0"); return; }
  korrigierenRunning = true;
  currentGame = GAME_KORRIGIEREN;
  sendStatus("QUIZ:CHECKING");
  uint8_t okCount = 0;
  for (uint8_t i = 0; i < korrigierenLen; i++) {
    if (currentGame != GAME_KORRIGIEREN) { sendStatus("QUIZ:RESULT:ABORT"); return; }
    gameDriveForwardSteps(config.quizForwardSteps);
    gameStop(GAME_COLOR_READ_MS);
    char found[16] = "UNBEKANNT";
    getCurrentColorName(found, sizeof(found));
    strlcpy(korrigierenFound[i], found, sizeof(korrigierenFound[i]));
    if (strcasecmp(korrigierenExpected[i], found) == 0) okCount++;
    char part[48];
    snprintf(part, sizeof(part), "QUIZ:STEP:%u,%s,%s", i, korrigierenExpected[i], found);
    sendStatus(part);
    bleProcessEvents();
  }
  stopMotors(); speedL = 0; speedR = 0;
  korrigierenRunning = false;
  currentGame = GAME_NONE;
  char result[48];
  if (okCount == korrigierenLen)
    snprintf(result, sizeof(result), "QUIZ:RESULT:OK,%u,%u", okCount, korrigierenLen);
  else
    snprintf(result, sizeof(result), "QUIZ:RESULT:WRONG,%u,%u", okCount, korrigierenLen);
  sendStatus(result);
}

// =====================================================
// RENNEN (Multiplayer)
// =====================================================
int  raceMyScore = 0;
volatile int  raceOppScore = 0;
volatile bool raceActive = false;
bool quizActive = false;

void raceExecuteStart(bool remote = false) {
  currentGame = GAME_RENNEN;
  raceMyScore = 0;
  raceOppScore = 0;
  raceActive = true;
  sendStatus(remote ? "RACE:REMOTE_START" : "RACE:STARTED");
}

void raceForwardStep() {
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; return; }
  gameDriveForward(GAME_MOVE_MS);
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; }
}

// Forward declarations for Slalom
void onSlalomPacket(const RacePacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

void onRacePacket(const RacePacket* pkt) {
  if (pkt->flags & RACE_FLAG_WINNER || pkt->score >= RENNEN_TARGET_SCORE) {
    raceOppScore = pkt->score;
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    sendStatus("RACE:OPP_WIN");
  } else {
    if (!raceActive) raceExecuteStart(true);
    raceOppScore = pkt->score;
    char msg[32];
    snprintf(msg, sizeof(msg), "RACE:PROGRESS:%u", raceOppScore);
    sendStatus(msg);
  }
}

// =====================================================
// FARBMARKER-SLALOM (Multiplayer) - VEREINFACHTE Logik
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
// =====================================================
enum SlalomPhase : uint8_t {
  SLALOM_IDLE       = 0,   // nicht gepaart
  SLALOM_PAIRED     = 1,   // gepaart, wartet auf Spielstart (Host)
  SLALOM_BUILD      = 2,   // Zielfarbe bekannt, Programm bauen
  SLALOM_COUNTDOWN  = 3,   // beide bereit, 3s bis Start
  SLALOM_RACING     = 4,   // Roboter fahren
  SLALOM_DONE       = 5    // Gewinner gefunden, Spiel vorbei
};
static const char* const SLALOM_PHASE_NAMES[6] = {
  "IDLE", "PAIRED", "BUILD", "COUNTDOWN", "RACING", "DONE"
};
SlalomPhase slalomPhase = SLALOM_IDLE;

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
// Er sendet SLALOM_CMD_GO per ESP-NOW an den Client und startet gleichzeitig
// selbst -> beide Roboter fahren mit minimalem Versatz (ESP-NOW-Latenz) los.
// Der Client startet sofort bei Empfang von GO; als Fallback (Paketverlust)
// startet er nach 4.5s eigenstaendig.
unsigned long slalomGoTimer = 0;   // Absoluter GO-Zeitpunkt (millis)
bool slalomGoPending = false;      // GO-Timer laeuft

// Farbnamen für den Slalom-Modus (4 Farben)
static const char* const SLALOM_COLOR_NAMES[4] = { "ROT", "GELB", "GRUEN", "BLAU" };

// Phase-Setzer (Logging + Status-Update zentral)
static void slalomSetPhase(SlalomPhase p) {
  if (slalomPhase == p) return;
  Serial.printf("[SLALOM] Phase %s -> %s\n", SLALOM_PHASE_NAMES[slalomPhase], SLALOM_PHASE_NAMES[p]);
  slalomPhase = p;
}

// =====================================================
// SLALOM PAIRING (eigenstaendig vom Race-Pairing)
// Einladungs-Flow:
//   1. Web A sendet SLALOM:INVITE:<MAC_B>  -> wir schicken CMD_INVITE an B
//   2. Web B empfaengt SLALOM:INVITED_BY:.. -> Pop-up, B klickt "Annehmen"
//   3. Web B sendet SLALOM:INVITE_ACCEPT   -> wir schicken CMD_INVITE_ACK an A
//   4. Beide Seiten markieren sich als gepaart; Initiator (A) = Host
// =====================================================
static bool    slalomHasPeer     = false;     // haben wir einen Slalom-Peer?
static uint8_t slalomPeerMac[6]  = {0,0,0,0,0,0}; // MAC des Slalom-Peers
// MAC des Initiators (von dem die Einladung kam) - bei Empfang von CMD_INVITE
static uint8_t slalomInviterMac[6] = {0,0,0,0,0,0};
// Pending: wir haben eine Einladung bekommen, aber Web-Accept steht noch aus
static bool    slalomInvitePending = false;

// Laufzeit-Zustand zuruecksetzen (ohne Pairing/Phase anzufassen).
// WICHTIG: slalomTargetColor bleibt ERHALTEN - bei einem Retry (niemand
// getroffen) wird mit DERSELBEN Zielfarbe weitergespielt.
static void slalomResetRuntime() {
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
static void slalomPairWith(const uint8_t mac[6], bool initiator) {
  memcpy(slalomPeerMac, mac, 6);
  slalomHasPeer = true;
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
static void slalomUnpair() {
  if (slalomHasPeer) {
    esp_now_del_peer(slalomPeerMac);
  }
  slalomHasPeer       = false;
  slalomIsHost        = false;
  slalomInvitePending = false;
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
  if (slalomHasPeer) {
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
  if (slalomHasPeer) {
    slalomSendPacket(SLALOM_CMD_ABORT, nullptr);
  }
}

// NUR der Host ruft dies auf (Web-Befehl SLALOM:START_MATCH):
// Zufallsfarbe waehlen und den Client in die Bauphase holen (CMD_START 3x,
// damit ein einzelnes verlorenes ESP-NOW-Paket nicht den Client haengen
// laesst). Die Farbe wird dem Web ueber SLALOM:TARGET gemeldet.
void slalomStartGame() {
  if (!slalomHasPeer) {
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
        slalomInvitePending = true;
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
      if (!slalomHasPeer) {
        slalomPairWith(pkt->senderMac, true);  // Initiator = Host
      }
      break;

    case SLALOM_CMD_SLALOM_UNPAIR:
      // Peer hat das Slalom-Match beendet - bei uns auch unpairen
      if (slalomHasPeer && memcmp(pkt->senderMac, slalomPeerMac, 6) == 0) {
        Serial.println("[SLALOM] Peer hat Slalom-Unpair gesendet");
        slalomUnpair();
      }
      break;

    default:
      break;
  }
}

// =====================================================
// BLE-CONNECT/DISCONNECT-HANDLER
// =====================================================
void onBleConnected() {
  deviceConnected = true;
  lastCommandTime = millis();
  driveResetDt();
  Serial.println("[BLE] Verbunden");
  sendStatus("EduBot V1.3 bereit");
}

void onBleDisconnected() {
  deviceConnected = false;
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
   raceActive = false;
  quizActive = false;
  currentGame = GAME_NONE;
  korrigierenLen = 0;
  korrigierenRunning = false;
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  memoryStop();
  lineFollowerStop();
  progStop();
  Serial.println("[BLE] Getrennt - Werbung gestartet");
  BLEDevice::startAdvertising();
  espNowEnsureChannel();
}

// Forward declarations for Slalom
void onSlalomPacket(const RacePacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

// =====================================================
// COMMAND-HANDLER
// =====================================================
void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  currentGame = GAME_NONE;
  korrigierenLen = 0;
  korrigierenRunning = false;
   raceActive = false;
  memoryStop();
  lineFollowerStop();
  progStop();
  slalomResetRuntime();
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
  ledStripShowAlert();
  sendStatus("STOP");
}

void handleCommand(const char* cmdStr) {
  String cmd(cmdStr);
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

  if (cmd.startsWith("KEYS:")) {
    String data = cmd.substring(5);
    if (data.length() >= 4) {
      keyW = (data.charAt(0) == '1');
      keyA = (data.charAt(1) == '1');
      keyS = (data.charAt(2) == '1');
      keyD = (data.charAt(3) == '1');
    }
    return;
  }

  Serial.printf("[CMD] %s\n", cmd.c_str());
  if (cmd == "STOP") { hardStop(); return; }

  char buf[48];

  if (cmd == "STEPS?") {
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)config.quizForwardSteps);
    sendStatus(buf);
    return;
  }
  if (cmd.startsWith("STEPS:")) {
    int32_t n = (int32_t)cmd.substring(6).toInt();
    if (n < 1 || n > 2000) {
      sendStatus("STEPS:BAD");
      return;
    }
    config.quizForwardSteps = n;
    configSaveInt("quizSteps", n);
    // Sofort testen
    testDriveSteps(n);
    snprintf(buf, sizeof(buf), "STEPS=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[STEPS] Test %ld Schritte gefahren (fuer 3cm-Quiz gespeichert)\n", (long)n);
    return;
  }

  if (cmd == "ANG?") {
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)config.quizTurnSteps);
    sendStatus(buf);
    return;
  }
  if (cmd.startsWith("ANG:")) {
    int32_t n = (int32_t)cmd.substring(4).toInt();
    if (n < 1 || n > 5000) {
      sendStatus("ANG:BAD");
      return;
    }
    config.quizTurnSteps = n;
    configSaveInt("quizTurnSteps", n);
    // Sofort testen (Drehung rechts auf der Stelle)
    testTurnSteps(n);
    snprintf(buf, sizeof(buf), "ANG=%ld", (long)n);
    sendStatus(buf);
    Serial.printf("[ANG] Test %ld Schritte/Rad gedreht (fuer 90-Grad-Spiel gespeichert)\n", (long)n);
    return;
  }

  if (cmd == "SAT?") {
    char msg[32];
    snprintf(msg, sizeof(msg), "SAT=%.2f", config.minColorSaturation);
    sendStatus(msg);
    return;
  }
  if (cmd.startsWith("SAT:")) {
    float v = cmd.substring(4).toFloat();
    if (v < 0.0f || v > 1.0f) { sendStatus("SAT:BAD"); return; }
    config.minColorSaturation = v;
    configSaveField("minSat", v);
    char msg[32];
    snprintf(msg, sizeof(msg), "SAT=%.2f", v);
    sendStatus(msg);
    return;
  }

  if (cmd == "CAL:BLACK") {
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
      xSemaphoreGive(colorSem);
    }
    configSaveUShort("blackR", config.blackR);
    configSaveUShort("blackG", config.blackG);
    configSaveUShort("blackB", config.blackB);
    sendStatus("CAL:BLACK OK");
    return;
  }
  if (cmd == "CAL:WHITE") {
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
      xSemaphoreGive(colorSem);
    }
    configSaveUShort("whiteR", config.whiteR);
    configSaveUShort("whiteG", config.whiteG);
    configSaveUShort("whiteB", config.whiteB);
    sendStatus("CAL:WHITE OK");
    return;
  }

  if (cmd == "CAL:RED" || cmd == "CAL:YELLOW" || cmd == "CAL:GREEN" || cmd == "CAL:BLUE") {
    ColorRef* ref = nullptr;
    const char* name = nullptr;
    const char* key  = nullptr;
    if      (cmd == "CAL:RED")    { ref = &config.calRot;   name = "RED";    key = "Rot";    }
    else if (cmd == "CAL:YELLOW") { ref = &config.calGelb;  name = "YELLOW"; key = "Gelb";   }
    else if (cmd == "CAL:GREEN")  { ref = &config.calGruen; name = "GREEN";  key = "Gruen";  }
    else                          { ref = &config.calBlau;  name = "BLUE";   key = "Blau";   }
    int r, g, b;
    if (colorSem == nullptr || xSemaphoreTake(colorSem, pdMS_TO_TICKS(100)) != pdTRUE) {
      sendStatus("CAL:BUSY");
      return;
    }
    r = sensorR; g = sensorG; b = sensorB;
    xSemaphoreGive(colorSem);
    ref->r = (uint8_t)constrain(r, 0, 255);
    ref->g = (uint8_t)constrain(g, 0, 255);
    ref->b = (uint8_t)constrain(b, 0, 255);
    ref->calibrated = true;
    char kR[16], kG[16], kB[16], kS[16];
    snprintf(kR, sizeof(kR), "cal%sR", key);
    snprintf(kG, sizeof(kG), "cal%sG", key);
    snprintf(kB, sizeof(kB), "cal%sB", key);
    snprintf(kS, sizeof(kS), "cal%sSet", key);
    configSaveUChar(kR, ref->r);
    configSaveUChar(kG, ref->g);
    configSaveUChar(kB, ref->b);
    configSaveBool(kS, true);
    char msg[48];
    snprintf(msg, sizeof(msg), "CAL:%s OK,%d,%d,%d", name, ref->r, ref->g, ref->b);
    sendStatus(msg);
    return;
  }

  if (cmd == "CAL:?") {
    char msg[96];
    snprintf(msg, sizeof(msg),
      "CAL:ROT=%s(%u,%u,%u) GELB=%s(%u,%u,%u) GRUEN=%s(%u,%u,%u) BLAU=%s(%u,%u,%u)",
      config.calRot.calibrated   ? "*" : "-", config.calRot.r,   config.calRot.g,   config.calRot.b,
      config.calGelb.calibrated  ? "*" : "-", config.calGelb.r,  config.calGelb.g,  config.calGelb.b,
      config.calGruen.calibrated ? "*" : "-", config.calGruen.r, config.calGruen.g, config.calGruen.b,
      config.calBlau.calibrated  ? "*" : "-", config.calBlau.r,  config.calBlau.g,  config.calBlau.b);
    sendStatus(msg);
    return;
  }

  // KORRIGIEREN
  if (cmd.startsWith("QUIZ:")) {
    String sub = cmd.substring(5);
    if (sub.startsWith("CHECK:")) {
      String arr = sub.substring(6);
      korrigierenLen = 0;
      int start = 0;
      while (korrigierenLen < KORRIGIEREN_MAX) {
        int comma = arr.indexOf(',', start);
        String part = (comma < 0) ? arr.substring(start) : arr.substring(start, comma);
        part.trim();
        part.toUpperCase();
        strlcpy(korrigierenExpected[korrigierenLen], part.c_str(), sizeof(korrigierenExpected[0]));
        korrigierenLen++;
        if (comma < 0) break;
        start = comma + 1;
      }
      // Jede Farbe max. QUIZ_MAX_COLOR_USE mal
      for (uint8_t i = 0; i < korrigierenLen; i++) {
        uint8_t count = 0;
        for (uint8_t j = 0; j < korrigierenLen; j++) {
          if (strcasecmp(korrigierenExpected[i], korrigierenExpected[j]) == 0) count++;
        }
        if (count > QUIZ_MAX_COLOR_USE) {
          char err[48];
          snprintf(err, sizeof(err), "QUIZ:TOO_MANY:%s,%u", korrigierenExpected[i], count);
          sendStatus(err);
          korrigierenLen = 0;
          return;
        }
      }
      korrigierenExecute();
    } else if (sub == "STOP") {
      currentGame = GAME_NONE;
      korrigierenRunning = false;
      stopMotors();
      sendStatus("QUIZ:STOPPED");
    }
    return;
  }

  // RENNEN
  if (cmd.startsWith("RACE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      raceExecuteStart();
      raceSendPacket(0, false);
    } else if (sub == "STEP") {
      if (!raceActive) return;
      raceMyScore++;
      raceForwardStep();
      if (!raceActive) return;
      raceSendPacket((uint8_t)raceMyScore, raceMyScore >= RENNEN_TARGET_SCORE);
      snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
      sendStatus(buf);
      if (raceMyScore >= RENNEN_TARGET_SCORE) {
        raceActive = false;
        currentGame = GAME_NONE;
        stopMotors(); speedL = 0; speedR = 0;
        sendStatus("RACE:YOU_WIN");
      }
    } else if (sub == "WIN") {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)RENNEN_TARGET_SCORE, true);
    } else if (sub == "LOSE") {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      raceSendPacket((uint8_t)raceMyScore, false);
    } else if (sub == "SCAN") {
      raceSendHello();
      sendStatus("RACE:SCAN_START");
    } else if (sub == "UNPAIR") {
      raceUnpair();
      sendStatus("RACE:UNPAIRED");
    } else if (sub.startsWith("PAIR:")) {
      String macStr = sub.substring(5);
      uint8_t mac[6] = {0,0,0,0,0,0};
      int idx = 0;
      int pos = 0;
      while (idx < 6 && pos < macStr.length()) {
        String hexByte = macStr.substring(pos, pos + 2);
        mac[idx++] = (uint8_t)strtol(hexByte.c_str(), nullptr, 16);
        pos += 3;
      }
      if (idx == 6) {
        racePairWith(mac);
        char conf[48];
        snprintf(conf, sizeof(conf), "RACE:PAIRED:%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        sendStatus(conf);
      } else {
        sendStatus("RACE:PAIR_FAIL");
      }
    } else if (sub.startsWith("PROGRESS:")) {
      raceOppScore = sub.substring(9).toInt();
      snprintf(buf, sizeof(buf), "RACE:PROGRESS:%d", raceOppScore);
      sendStatus(buf);
    }
    return;
  }

  // SLALOM (Farben-Slalom Multiplayer)
  if (cmd.startsWith("SLALOM:")) {
    String sub = cmd.substring(7);
    if (sub.startsWith("PROG:")) {
      String enc = sub.substring(5);
      slalomSetProgram(enc.c_str());
    } else if (sub.startsWith("INVITE:")) {
      // SLALOM:INVITE:<MAC> - Initiator (Host) schickt Einladung an Peer
      // Wichtig: hat nichts mit dem Race-Pairing zu tun, ist komplett
      // eigenstaendiges Slalom-Pairing.
      if (slalomHasPeer) {
        sendStatus("SLALOM:ERROR:ALREADY_PAIRED");
        return;
      }
      String macStr = sub.substring(7);
      uint8_t mac[6] = {0,0,0,0,0,0};
      int idx = 0, pos = 0;
      while (idx < 6 && pos < macStr.length()) {
        String hexByte = macStr.substring(pos, pos + 2);
        mac[idx++] = (uint8_t)strtol(hexByte.c_str(), nullptr, 16);
        pos += 3;
      }
      if (idx != 6) {
        sendStatus("SLALOM:ERROR:BAD_MAC");
        return;
      }
      // CMD_INVITE an den Peer senden (senderMac identifiziert uns)
      slalomSendPacketTo(SLALOM_CMD_INVITE, nullptr, mac);
      sendStatus("SLALOM:INVITE_SENT");
      Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else if (sub == "INVITE_ACCEPT") {
      // Web des Empfaengers hat das Pop-up bestaetigt
      if (!slalomInvitePending) {
        sendStatus("SLALOM:ERROR:NO_INVITE");
        return;
      }
      // CMD_INVITE_ACK an den Initiator schicken
      slalomSendPacketTo(SLALOM_CMD_INVITE_ACK, nullptr, slalomInviterMac);
      // Wir sind Client (nicht Initiator)
      slalomPairWith(slalomInviterMac, false);
      slalomInvitePending = false;
      sendStatus("SLALOM:INVITE_ACCEPTED");
    } else if (sub == "INVITE_DECLINE") {
      // Einladung abgelehnt
      if (slalomInvitePending) {
        slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomInviterMac);
      }
      slalomInvitePending = false;
      memset(slalomInviterMac, 0, 6);
    } else if (sub == "UNPAIR") {
      // Slalom-spezifisches Unpair (nur Slalom-State, nicht das Race-Pairing)
      if (slalomHasPeer) {
        // Dem Peer mitteilen, dass wir uns trennen
        slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomPeerMac);
      }
      slalomUnpair();
    } else if (sub == "START_MATCH") {
      // Web-Befehl (Host): Spiel starten, Zufallsfarbe, Client in Bauphase holen
      if (!slalomHasPeer) {
        sendStatus("SLALOM:ERROR:NOT_PAIRED");
        return;
      }
      if (!slalomIsHost) {
        sendStatus("SLALOM:ERROR:NOT_HOST");
        return;
      }
      slalomStartGame();
    } else if (sub == "STOP") {
      // Spiel manuell abbrechen -> beide zurueck zu PAIRED
      slalomAbortGame();
    }
    return;
  }

  // LINIE FOLGEN
  if (cmd.startsWith("LINE:")) {
    String sub = cmd.substring(5);
    if (sub == "START") {
      if (!lineFollowerActive) {
        currentGame = GAME_LINEFOLLOWER;
        lineFollowerStart();
        sendStatus("LINE:RUNNING");
      }
    } else if (sub == "STOP") {
      bool wasActive = lineFollowerActive;
      lineFollowerStop();
      currentGame = GAME_NONE;
      sendStatus(wasActive ? "LINE:STOPPED" : "LINE:STOPPED");
    }
    return;
  }

  // MEMORY (A1)
  if (cmd.startsWith("MEMORY:")) {    String sub = cmd.substring(7);
    if (sub == "START") {
      currentGame = GAME_MEMORY;
      memoryStart(3);
    } else if (sub.startsWith("START:")) {
      uint8_t n = (uint8_t)sub.substring(6).toInt();
      if (n < 3) n = 3;
      if (n > MEMORY_MAX_LEN) n = MEMORY_MAX_LEN;
      currentGame = GAME_MEMORY;
      memoryStart(n);
    } else if (sub == "STOP") {
      memoryStop();
      currentGame = GAME_NONE;
    } else if (sub.startsWith("INPUT:")) {
      uint8_t m = (uint8_t)sub.substring(6).toInt();
      memoryInput(m);
    }
    return;
  }

  // PROGRAMMIEREN (Code-Bausteine)
  if (cmd.startsWith("PROG:")) {
    String sub = cmd.substring(5);
    if (sub.startsWith("SET:")) {
      String enc = sub.substring(4);
      if (progSetFromString(enc.c_str())) {
        snprintf(buf, sizeof(buf), "PROG:READY:%u", progLen);
        sendStatus(buf);
      } else {
        sendStatus("PROG:BAD");
      }
    } else if (sub == "RUN") {
      progRun();
    } else if (sub == "STOP") {
      progStop();
    }
    return;
  }

  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V1.3 - Modular");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
  esp_task_wdt_add(NULL);

  configLoad();

  initSteppers();
  Serial.println("[MOTOR] AccelStepper initialisiert (Beschleunigung aktiv)");

  pinMode(IR_PIN, INPUT);  // D8 = IR-Sensor (digital)
  Serial.println("[IR] Sensor auf D8 (GPIO7, INPUT) initialisiert");

  lastCommandTime = millis();
  driveResetDt();

  colorSetup();
  if (colorSensorOk) {
    xTaskCreatePinnedToCore(
      colorSensorTask, "ColorSensor",
      STACK_COLOR_SENSOR, NULL, TASK_PRIORITY_SENSOR, NULL, CORE_SENSOR
    );
    Serial.println("[TASK] ColorSensor-Task gestartet (Core 0)");
  }

  espNowInit();
  bleSetup();
  espNowEnsureChannel();
  ledStripSetup();
  Serial.println("======================================\n");
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastColorNotify = 0;
unsigned long lastIrNotify = 0;
int lastIrValue = -1;  // Edge-Erkennung, nur bei Wechsel senden
void loop() {
  esp_task_wdt_reset();
  bleProcessEvents();
  espNowProcessEvents();
  memoryTick();
  if (currentGame == GAME_LINEFOLLOWER) {
    bool wasActive = lineFollowerActive;
    lineFollowerTick();
    if (wasActive && !lineFollowerActive) {
      // Linie endgueltig verloren -> Modus beenden, Meldung senden
      currentGame = GAME_NONE;
      sendStatus("LINE:LOST");
    }
  }
  // ============================================================
  // Slalom-Timer:
  //   slalomGoPending  : 3s-Countdown nach BOTH_READY (HOST zaehlt,
  //                      Client nur Fallback 4.5s bei Paketverlust)
  //   slalomCheckRetry : Retry, wenn kein Treffer (beide MISS oder Timeout)
  // ============================================================

  // (1) GO-Timer: Host zaehlt 3s, sendet GO 3x und startet selbst.
  //     Client: Fallback, falls das GO-Paket verloren ging.
  if (slalomGoPending && !slalomRunning && slalomPhase == SLALOM_COUNTDOWN &&
      millis() >= slalomGoTimer) {
    slalomGoPending = false;
    if (slalomIsHost) {
      // GO 3x senden (Paketverlust-Retry). Der Client startet beim ersten
      // empfangenen Paket und ignoriert die Wiederholungen (Phase ist dann
      // RACING). So starten beide Roboter trotz ESP-NOW-Paketverlust
      // synchron statt mit 1.5s Client-Fallback-Versatz.
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      slalomSendPacket(SLALOM_CMD_GO, nullptr);
      Serial.println("[SLALOM] HOST-GO-Timer: GO 3x gesendet, starte selbst");
    } else {
      // Fallback: GO-Paket verloren - trotzdem starten (leicht verspaetet)
      Serial.println("[SLALOM] Client-Fallback-GO: kein Paket erhalten, starte trotzdem");
    }
    slalomExecute();
  }
  // (2) Retry: beide Durchlaeufe ohne Treffer (oder 15s-Timeout) -> BUILD
  slalomCheckRetry();
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  // LED-Streifen: Alert > Zielfarbe > Status (max. 4x/s aktualisieren)
  {
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate >= 250) {
      lastLedUpdate = millis();
      if (ledAlertActive) {
        ledStripTick();
      } else if (slalomTargetColor[0] != '\0') {
        ledStripShowTarget(slalomTargetColor);
      } else {
        ledStripShowStatus(deviceConnected);
      }
    }
  }
  // IR-Sensor: LM393-Modul ist aktiv-low (OUT = LOW bei Hindernis).
  // Daher invertieren: irVal = 1 bedeutet Hindernis, 0 bedeutet frei.
  // Nur senden, wenn sich der Wert geaendert hat
  int irVal = !digitalRead(IR_PIN);
  if (irVal != lastIrValue) {
    lastIrValue = irVal;
    char msg[8];
    snprintf(msg, sizeof(msg), "IR:%d", irVal);
    sendStatus(msg);
  }
  // Optional: periodischer Heartbeat (alle 5s), damit das Web weiss
  // dass die Verbindung lebt (auch wenn IR-Wert stabil ist)
  static unsigned long lastIrHeartbeat = 0;
  if (deviceConnected && millis() - lastIrHeartbeat > 5000) {
    lastIrHeartbeat = millis();
    char msg[16];
    snprintf(msg, sizeof(msg), "IR:HB:%d", irVal);
    sendStatus(msg);
  }
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
