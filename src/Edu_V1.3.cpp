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

// =====================================================
// GLOBALE KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 5;

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
// Einheitliche Zeiten: jedes Spiel (Knobeln, Memory, Quiz, Rennen) nutzt
// dieselbe Vor/Zurueck- und Drehdauer -> konsistenter Bewegungsablauf.
// Beschleunigung/Bremsen uebernimmt AccelStepper (kein Wheelie, kein Schraeglauf).
const float     GAME_DRIVE_GAIN   = 1.0f;
constexpr uint32_t GAME_TICK_MS    = 5;    // Polling-Intervall fuer motorRun() waehrend blockierender Moves
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
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

void gameDriveForward(uint32_t ms) {
  gameMoveBusy = true;
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  const float target = config.maxSpeed * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
  waitForStop(start);
  gameMoveBusy = false;
}

void gameDriveBackward(uint32_t ms) {
  gameMoveBusy = true;
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  const float target = -config.maxSpeed * GAME_DRIVE_GAIN;
  setMotorSpeed(0, target);
  setMotorSpeed(1, target);
  uint32_t start = millis();
  while (millis() - start < ms) {
    motorRun();
    if (currentGame == GAME_NONE) { stopMotors(); gameMoveBusy = false; return; }
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
    motorRun();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Positionsbasiert: exakte Strecke fahren (z.B. 3cm pro Farbe im Quiz).
// Beruecksichtigt config.leftSign/rightSign fuer die Verkabelung.
void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  // Maximalgeschwindigkeit fuer diesen Move (benutzerkonfigurierbar)
  stepperL.setMaxSpeed(config.maxSpeed);
  stepperR.setMaxSpeed(config.maxSpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  // Relative Position: Roboter bewegt sich um 'steps' vorwaerts.
  // Durch config.leftSign/rightSign wird die Richtung korrigiert.
  stepperL.move(steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop();
      stepperR.stop();
      gameMoveBusy = false;
      return;
    }
    stepperL.run();
    stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: fahre n Schritte ohne currentGame-Abhaengigkeit, mit Timeout.
// Wird vom STEPS:-Befehl genutzt, um den richtigen Wert fuer 3cm zu finden.
void testDriveSteps(int32_t steps) {
  stepperL.setMaxSpeed(config.maxSpeed);
  stepperR.setMaxSpeed(config.maxSpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move(steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(steps * static_cast<int32_t>(config.rightSign));
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    stepperL.run();
    stepperR.run();
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
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move(-steps * static_cast<int32_t>(config.leftSign));
  stepperR.move( steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}
void gameTurnRightSteps(int32_t steps) {
  gameMoveBusy = true;
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move( steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(-steps * static_cast<int32_t>(config.rightSign));
  while (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    if (currentGame == GAME_NONE) {
      stepperL.stop(); stepperR.stop(); gameMoveBusy = false; return;
    }
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  gameMoveBusy = false;
}

// Test-Helper: drehe auf der Stelle (rechts) um 'steps' Schritte pro Rad.
void testTurnSteps(int32_t steps) {
  stepperL.setMaxSpeed(config.turnOnlySpeed);
  stepperR.setMaxSpeed(config.turnOnlySpeed);
  stepperL.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperR.setAcceleration(GLOBAL_ACCEL_RATE);
  stepperL.move( steps * static_cast<int32_t>(config.leftSign));
  stepperR.move(-steps * static_cast<int32_t>(config.rightSign));
  uint32_t start = millis();
  while ((stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) && millis() - start < 5000) {
    stepperL.run(); stepperR.run();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (stepperL.distanceToGo() != 0 || stepperR.distanceToGo() != 0) {
    stepperL.stop(); stepperR.stop();
  }
}

// =====================================================
// KNOBELN (Pfeil-Sequenz abfahren)
// =====================================================
constexpr uint8_t KNOBELN_MAX = 32;
uint8_t knobelnSeq[KNOBELN_MAX];
uint8_t knobelnLen = 0;

void knobelnExecute() {
  if (knobelnLen == 0) { sendStatus("KNOBELN:NONE"); return; }
  currentGame = GAME_KNOBELN;
  sendStatus("KNOBELN:RUNNING");
  for (uint8_t i = 0; i < knobelnLen; i++) {
    if (currentGame != GAME_KNOBELN) { sendStatus("KNOBELN:ABORT"); return; }
    switch (knobelnSeq[i]) {
      case 0: gameDriveForward(GAME_MOVE_MS); break;
      case 1: gameTurnRightSteps(config.quizTurnSteps); break;
      case 2: gameDriveBackward(GAME_MOVE_MS); break;
      case 3: gameTurnLeftSteps(config.quizTurnSteps); break;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
  }
  currentGame = GAME_NONE;
  sendStatus("KNOBELN:DONE");
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
void slalomMaybeRun();

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
// FARBMARKER-SLALOM (Multiplayer)
//
// =====================================================
// 5-Phasen State Machine (v2.0)
//   IDLE       : Keine Pairing-Verbindung
//   SETUP      : Gepaart, Host hat Spiel gestartet, Zielfarbe noch nicht synchronisiert
//   BUILDING   : Zielfarbe ist gelockt (Host hat ACK bekommen), Spieler bauen Programm
//   READY_WAIT : Mindestens ein Spieler hat "Bereit" gedrueckt, 10s-Grace laeuft
//   RACING     : Countdown vorbei, Roboter fuehren das Programm aus
//   ROUND_END  : Punkte angezeigt, 3s Uebergangspause vor naechster Runde
//
// ESP-NOW Protokoll:
//   CMD_START_MATCH (Web -> Host)         : SLALOM:START_MATCH
//   CMD_TARGET       (Host -> Client)     : SLALOM_CMD_TARGET  (nur Broadcast, kein Web-Update)
//   CMD_TARGET_ACK   (Client -> Host)     : SLALOM_CMD_TARGET_ACK
//   CMD_BEREIT       (Peer  -> Peer)      : SLALOM_CMD_PROG  (Programm/Bereit-Signal)
//   CMD_GO           (Host -> Client)     : SLALOM:GO (nach 3s Web-Countdown)
//
// Host wartet auf CMD_TARGET_ACK bevor er SLALOM:TARGET an das WEB broadcastet.
// Dadurch ist die Zielfarbe auf beiden Seiten garantiert "locked" wenn das Spiel startet.
// =====================================================
enum SlalomPhase : uint8_t {
  SLALOM_IDLE       = 0,
  SLALOM_SETUP      = 1,
  SLALOM_BUILDING   = 2,
  SLALOM_READY_WAIT = 3,
  SLALOM_RACING     = 4,
  SLALOM_ROUND_END  = 5
};
static const char* const SLALOM_PHASE_NAMES[6] = {
  "IDLE", "SETUP", "BUILDING", "READY_WAIT", "RACING", "ROUND_END"
};
SlalomPhase slalomPhase = SLALOM_IDLE;

char slalomTargetColor[16] = "";
char slalomMyProgram[PROG_MAX_ENC + 1] = "";
char slalomOppProgram[PROG_MAX_ENC + 1] = "";
bool slalomReady = false;        // Eigenes Programm ist gesetzt (Bereit gedrückt)
bool slalomOppReady = false;     // Gegner-Programm empfangen (Gegner hat Bereit gedrückt)
bool slalomRunning = false;
bool slalomIsHost = false;       // Bin ich Host (Spiel-Einlader)?
bool slalomBereitPhase = false;  // Warte auf beide Spieler-Bereit-Drücke
bool slalomReadyToRun = false;   // Beide bereit → Web zeigt Countdown
bool slalomWaitingForTarget = false; // Nicht-Host: warte auf CMD_TARGET für neue Runde
bool slalomTargetLocked = false;     // Zielfarbe ist zwischen Host+Client synchronisiert (ACK erhalten)
unsigned long slalomBereitStart = 0; // Startzeitpunkt 10s-Grace nach erstem Bereit
bool slalomBereitActive = false;     // 10s-Grace läuft (für 2. Spieler)

// Runden-Tracking
int slalomRound = 0;
int slalomScoreMe = 0;
int slalomScoreOpp = 0;
bool slalomFinished = false;       // Hat dieses Board die Zielfarbe erreicht?
bool slalomOppFinished = false;    // Hat der Gegner die Zielfarbe erreicht?
unsigned long slalomExtStart = 0;  // Startzeit der 10-Sek-Extension
bool slalomExtActive = false;      // 10-Sek-Extension läuft
unsigned long slalomRoundTimeout = 0; // Timeout für gesamte Runde (60 Sek)
unsigned long slalomTransitionStart = 0; // 3-Sek-Pause zwischen Runden
bool slalomTransitionActive = false;     // Wartet auf 3-Sek-Pause vor neuer Runde

// Farbnamen für den Slalom-Modus (4 Farben)
static const char* const SLALOM_COLOR_NAMES[4] = { "ROT", "GELB", "GRUEN", "BLAU" };

// Phase-Setzer (Logging + Status-Update zentral)
static void slalomSetPhase(SlalomPhase p) {
  if (slalomPhase == p) return;
  Serial.printf("[SLALOM] Phase %s -> %s\n", SLALOM_PHASE_NAMES[slalomPhase], SLALOM_PHASE_NAMES[p]);
  slalomPhase = p;
}

// ACK-Timeout: merkt sich wann der Host das letzte CMD_TARGET gesendet hat.
// Falls nach 3s kein ACK vom Client kommt (z.B. ESP-NOW Paketverlust),
// wird die Zielfarbe automatisch gelockt, damit das Match nicht haengt.
unsigned long slalomAckDeadline = 0;

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
  if (hasPairedPeer) {
    esp_now_send(pairedPeer, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    esp_now_send(espNowPeer, (uint8_t*)&pkt, sizeof(pkt));
  }
}

// CMD_TARGET_ACK: Client bestaetigt dem Host den Erhalt der Zielfarbe.
//   - Auf dem Client direkt: nach Empfang von CMD_TARGET, slalomTargetColor setzen + ACK senden
//   - Auf dem Host: nur den ACK-Path (slalomLockTarget) aufrufen
// Strikt getrennt von slalomSetTargetColor: kein doppeltes Web-Broadcast.

// NUR der Host ruft dies auf. Waehlt eine Zufallsfarbe, sendet CMD_TARGET
// an den Client, aber NOCH NICHT SLALOM:TARGET an das Web. Das passiert erst
// in slalomLockTarget() nach erfolgreichem ACK-Handshake.
void slalomPickTarget() {
  if (!slalomIsHost) {
    Serial.println("[SLALOM] slalomPickTarget() blockiert: ich bin nicht Host");
    return;
  }
  uint8_t idx = random(0, 4);
  strlcpy(slalomTargetColor, SLALOM_COLOR_NAMES[idx], sizeof(slalomTargetColor));
  slalomTargetLocked = false;
  // ACK-Timeout starten: falls in 3s kein ACK kommt, selbst locken
  slalomAckDeadline = millis() + 3000;
  // NUR ESP-NOW an Client. Web-Update kommt spaeter (slalomLockTarget)
  slalomSendPacket(SLALOM_CMD_TARGET, slalomTargetColor);
  Serial.printf("[SLALOM] Host hat Ziel %s gewaehlt, warte auf Client-ACK (Timeout 3s)\n", slalomTargetColor);
}

// Client setzt die Zielfarbe (nach CMD_TARGET vom Host), schickt ACK
// zurueck, und schickt SLALOM:TARGET an das lokale Web.
void slalomSetTargetColor(const char* col) {
  if (!col || strlen(col) == 0) return;
  strlcpy(slalomTargetColor, col, sizeof(slalomTargetColor));
  // Lokales Lock-Flag: Client weiss, dass das Ziel angekommen ist.
  // Der finale ACK-Lock passiert serverseitig (Host) durch slalomLockTarget.
  // Setzen wir hier dennoch auf true, damit der Client nicht durch ein
  // verfruehtes SLALOM:GO aus dem Tritt kommt (slalomExecute prueft dies).
  slalomTargetLocked = true;
  // Phase: SETUP -> BUILDING (lokal weiss der Client: Ziel ist da)
  if (slalomPhase == SLALOM_SETUP) {
    slalomSetPhase(SLALOM_BUILDING);
  }
  // ACK an Host schicken (Color mit Prefix, damit Host verifizieren kann)
  slalomSendPacket(SLALOM_CMD_TARGET_ACK, slalomTargetColor);
  // Lokales Web informieren
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor);
  sendStatus(msg);
  Serial.printf("[SLALOM] Client hat Ziel %s empfangen + ACK gesendet\n", slalomTargetColor);
}

// Host bekommt ACK vom Client: Zielfarbe ist jetzt auf BEIDEN Seiten gesetzt.
// Erst jetzt darf der Host SLALOM:TARGET an sein Web broadcasten -> beide UIs
// zeigen die grosse Zielfarbe, Phase wechselt zu BUILDING.
void slalomLockTarget() {
  if (!slalomIsHost) return;
  slalomTargetLocked = true;
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:TARGET:%s", slalomTargetColor);
  sendStatus(msg);
  slalomSetPhase(SLALOM_BUILDING);
  Serial.printf("[SLALOM] Host hat ACK erhalten, Ziel %s ist LOCKED\n", slalomTargetColor);
}

void slalomSetProgram(const char* enc) {
  strlcpy(slalomMyProgram, enc, sizeof(slalomMyProgram));
  slalomReady = true;
  // Phase READY_WAIT (vorher BUILDING, jetzt wartet man auf Gegner)
  if (slalomPhase == SLALOM_BUILDING) {
    slalomSetPhase(SLALOM_READY_WAIT);
  }
  sendStatus("SLALOM:READY");
  slalomSendPacket(SLALOM_CMD_PROG, slalomMyProgram);
  // Prüfe, ob beide bereit sind; falls nicht, 10s-Grace-Timer starten
  if (slalomOppReady) {
    slalomMaybeRun();
  } else {
    // Wir haben zuerst gedrückt: 10s-Grace für Gegner starten
    slalomBereitStart = millis();
    slalomBereitActive = true;
  }
}

// Wird aufgerufen wenn beide Spieler bereit sind (oder 10s-Grace abgelaufen).
// Signalisiert dem Web, dass nun ein 3-Sekunden-Countdown angezeigt werden soll.
void slalomMaybeRun() {
  if (!slalomBereitPhase) return;
  if (slalomRunning) return;
  if (!slalomReady || !slalomOppReady) return;
  if (slalomTargetColor[0] == '\0') {
    // Sollte nicht passieren: Zielfarbe muss bereits in SETUP gelockt sein
    if (slalomIsHost) slalomPickTarget();
    else return;
  }
  slalomBereitActive = false;
  slalomReadyToRun = true;
  sendStatus("SLALOM:BOTH_READY");
}

void slalomStartNextRound() {
  slalomTransitionActive = false;
  slalomRound++;
  slalomFinished = false;
  slalomOppFinished = false;
  slalomExtActive = false;
  slalomReady = false;
  slalomOppReady = false;
  slalomReadyToRun = false;
  // Bereit-Phase aktivieren
  slalomBereitPhase = true;
  slalomBereitActive = false;
  slalomBereitStart = 0;
  // Phase: SETUP (Zielfarbe noch nicht synchronisiert)
  slalomSetPhase(SLALOM_SETUP);
  // Zuerst ROUND_START senden (UI schaltet Programmierbereich frei)
  char msg[48];
  snprintf(msg, sizeof(msg), "SLALOM:ROUND_START:%d", slalomRound);
  sendStatus(msg);
  // Zielfarbe nur in der ERSTEN Runde picken; danach gleiche Farbe beibehalten.
  // So hat jeder Spieler die Chance, sein Programm iterativ zu verbessern
  // und es mehrfach zu versuchen, bis die Zielfarbe erreicht wird.
  if (slalomIsHost) {
    slalomWaitingForTarget = false;
    if (slalomTargetColor[0] == '\0') {
      // Erste Runde: Zufallsfarbe picken
      slalomPickTarget();
      // Phase bleibt SETUP bis slalomLockTarget() ACK bekommt (oder Timeout)
    } else {
      // Folge-Runden: gleiche Farbe beibehalten, nur neu synchronisieren
      slalomTargetLocked = false;
      slalomAckDeadline = millis() + 3000;
      slalomSendPacket(SLALOM_CMD_TARGET, slalomTargetColor);
      Serial.printf("[SLALOM] Runde %d: gleiche Zielfarbe %s, re-synchronisiere mit Client\n",
                    slalomRound, slalomTargetColor);
    }
  } else {
    slalomWaitingForTarget = true;
    if (slalomTargetColor[0] == '\0') {
      // Erste Runde: Zielfarbe vom Host abwarten
      sendStatus("SLALOM:WAITING_TARGET");
    } else {
      // Folge-Runden: gleiche Farbe erwarten, warten auf neue CMD_TARGET-Bestaetigung
      sendStatus("SLALOM:WAITING_TARGET");
    }
  }
  Serial.printf("[SLALOM] Neue Runde %d gestartet (Phase SETUP, Farbe=%s)\n",
                slalomRound, slalomTargetColor);
}

// Host ruft dies auf, um das Spiel zu starten (initial).
void slalomStartMatch() {
  slalomIsHost = true;
  slalomRound = 1;
  slalomScoreMe = 0;
  slalomScoreOpp = 0;
  slalomStartNextRound();
  sendStatus("SLALOM:MATCH_STARTED");
  Serial.println("[SLALOM] Match vom Host gestartet");
}

void slalomRoundEnd() {
  slalomExtActive = false;
  slalomFinished = false;
  slalomOppFinished = false;
  slalomSetPhase(SLALOM_ROUND_END);
  char msg[64];
  snprintf(msg, sizeof(msg), "SLALOM:ROUND_END:%d:%d:%d", slalomRound, slalomScoreMe, slalomScoreOpp);
  sendStatus(msg);
  Serial.printf("[SLALOM] Runde %d beendet. Score: Ich=%d Gegner=%d\n", slalomRound, slalomScoreMe, slalomScoreOpp);
  // Nicht-blockierende 3-Sek-Pause via Timer (loop() prüft ab)
  slalomTransitionStart = millis();
  slalomTransitionActive = true;
}

void slalomExecute() {
  currentGame = GAME_SLALOM;
  slalomRunning = true;
  slalomFinished = false;
  slalomOppFinished = false;
  slalomExtActive = false;
  slalomRoundTimeout = millis() + 60000; // 60 Sek Gesamttimeout pro Runde
  slalomSetPhase(SLALOM_RACING);
  // Zielfarbe MUSS gesetzt sein (sonst kein sinnvoller Lauf)
  if (slalomTargetColor[0] == '\0') {
    Serial.println("[SLALOM] Abbruch: keine Zielfarbe gesetzt");
    slalomRunning = false;
    currentGame = GAME_NONE;
    sendStatus("SLALOM:ERROR:NO_TARGET");
    return;
  }
  // Warnung (kein Abbruch): falls der ACK-Handshake noch nicht durch ist,
  // laeuft das Rennen trotzdem. ESP-NOW kann einzelne Pakete verlieren -
  // wir wollen das Match nicht blockieren, nur weil ein ACK fehlt.
  if (!slalomTargetLocked) {
    Serial.println("[SLALOM] WARNUNG: Zielfarbe noch nicht gelockt (ACK fehlt) - starte trotzdem");
    slalomTargetLocked = true;  // nachtraeglich locken
  }
  sendStatus("SLALOM:RUNNING");
  progSetFromString(slalomMyProgram);
  progRunSlalom(slalomTargetColor);
  slalomRunning = false;
  currentGame = GAME_NONE;

  // Runden-Ergebnis per ESP-NOW an Gegner senden
  if (slalomFinished) {
    slalomScoreMe++;
    slalomSendPacket(SLALOM_CMD_OPP_FINISHED, slalomTargetColor);
    // 10-Sek-Extension starten: warten ob Gegner auch noch trifft
    slalomExtStart = millis();
    slalomExtActive = true;
  } else if (slalomOppFinished) {
    // Gegner hat getroffen, wir nicht: 10-Sek-Extension läuft bereits
    // (vom Gegner angestossen); wir können nicht mehr punkten
    // (Nichts zu tun, Extension-Timer läuft)
  } else {
    // Weder ich noch Gegner haben getroffen (oder 10s-Grace-Timeout)
    if (slalomExtActive) {
      // Extension läuft: nichts tun
    } else {
      // Keine Extension: Runde beenden und nächste starten
      slalomSendPacket(SLALOM_CMD_TIMEOUT, nullptr);
      slalomRoundEnd();
    }
  }
}

void onSlalomPacket(const RacePacket* pkt) {
  if (pkt->slalomCmd == SLALOM_CMD_PROG && pkt->slalomLen > 0) {
    char buf[PROG_MAX_ENC + 1] = {0};
    size_t len = pkt->slalomLen;
    if (len > PROG_MAX_ENC) len = PROG_MAX_ENC;
    memcpy(buf, pkt->slalomData, len);
    buf[len] = '\0';
    strlcpy(slalomOppProgram, buf, sizeof(slalomOppProgram));
    slalomOppReady = true;
    // Phase READY_WAIT aktivieren (sobald mind. 1 Spieler Bereit gedrueckt hat)
    if (slalomPhase == SLALOM_BUILDING) {
      slalomSetPhase(SLALOM_READY_WAIT);
    }
    sendStatus("SLALOM:OPP_PROG");
    // Gegner hat Bereit gedrückt. Wenn ich auch bereit bin, sofort starten.
    // Sonst: 10s-Grace-Timer starten (für mich).
    if (slalomReady) {
      slalomMaybeRun();
    } else if (slalomBereitPhase) {
      slalomBereitStart = millis();
      slalomBereitActive = true;
    }
  } else if (pkt->slalomCmd == SLALOM_CMD_TARGET && pkt->slalomLen > 0) {
    // Host sendet CMD_TARGET an Client. Client setzt Farbe + schickt ACK.
    // (In slalomSetTargetColor passiert beides in einem Schritt.)
    char buf[16] = {0};
    size_t len = pkt->slalomLen < 15 ? pkt->slalomLen : 15;
    memcpy(buf, pkt->slalomData, len);
    // Wenn wir noch nicht in einer Runde sind, hat der Host das Match gestartet
    if (slalomRound == 0) {
      slalomRound = 1;
      slalomScoreMe = 0;
      slalomScoreOpp = 0;
      slalomBereitPhase = true;
      slalomBereitActive = false;
      slalomBereitStart = 0;
      slalomWaitingForTarget = false;
      slalomIsHost = false;
      slalomSetPhase(SLALOM_SETUP);
      // Für das Web: Match gestartet + Runde starten
      sendStatus("SLALOM:MATCH_STARTED");
      char rmsg[48];
      snprintf(rmsg, sizeof(rmsg), "SLALOM:ROUND_START:%d", slalomRound);
      sendStatus(rmsg);
      Serial.println("[SLALOM] Match vom Gegner (Host) gestartet");
    } else {
      // Neue Runde: Ziel von Host erhalten
      slalomWaitingForTarget = false;
    }
    // ACK an Host + lokales Web updaten
    slalomSetTargetColor(buf);
    // Phase bleibt SETUP bis slalomSetTargetColor ACK gesendet hat;
    // das eigentliche Web-SLALOM:TARGET (das die grosse Anzeige triggert) wird
    // in slalomSetTargetColor gesendet. Phase BUILDING setzt der HOST.
  } else if (pkt->slalomCmd == SLALOM_CMD_TARGET_ACK && pkt->slalomLen > 0) {
    // Host empfaengt ACK vom Client. Color-Verifikation: ACK-Farbe muss
    // mit der eigenen uebereinstimmen, sonst verwerfen (Race condition).
    if (!slalomIsHost) return;
    if (slalomTargetLocked) return;   // bereits gelockt, kein 2. ACK noetig
    char buf[16] = {0};
    size_t len = pkt->slalomLen < 15 ? pkt->slalomLen : 15;
    memcpy(buf, pkt->slalomData, len);
    if (strcasecmp(buf, slalomTargetColor) != 0) {
      Serial.printf("[SLALOM] ACK-Farbe %s passt nicht zu %s, verwerfe\n", buf, slalomTargetColor);
      return;
    }
    slalomAckDeadline = 0;  // ACK erhalten, Timeout deaktivieren
    slalomLockTarget();
  } else {
    switch (pkt->slalomCmd) {
      case SLALOM_CMD_FINISH:
        slalomRunning = false;
        currentGame = GAME_NONE;
        stopMotors();
        speedL = 0;
        speedR = 0;
        sendStatus("SLALOM:OPP_FINISH");
        break;
      case SLALOM_CMD_OPP_FINISHED:
        // Gegner hat Farbe erreicht: Roboter stoppen, 10-Sek-Extension
        slalomRunning = false;
        currentGame = GAME_NONE;
        stopMotors();
        speedL = 0;
        speedR = 0;
        slalomOppFinished = true;
        slalomExtActive = true;
        slalomExtStart = millis();
        sendStatus("SLALOM:OPP_FINISH");
        break;
      case SLALOM_CMD_TIMEOUT:
        // Gegner-Runde abgelaufen (kein Treffer)
        slalomRunning = false;
        currentGame = GAME_NONE;
        stopMotors();
        speedL = 0;
        speedR = 0;
        sendStatus("SLALOM:TIMEOUT");
        break;
      case SLALOM_CMD_RESET:
  slalomReady = false;
  slalomOppReady = false;
  slalomRunning = false;
  slalomIsHost = false;
  slalomBereitPhase = false;
  slalomReadyToRun = false;
  slalomWaitingForTarget = false;
  slalomBereitActive = false;
  slalomExtActive = false;
  slalomTransitionActive = false;
  slalomTargetLocked = false;
  slalomTargetColor[0] = '\0';
  slalomRound = 0;
  slalomScoreMe = 0;
  slalomScoreOpp = 0;
        slalomTargetColor[0] = '\0';
        slalomSetPhase(SLALOM_IDLE);
        sendStatus("SLALOM:RESET");
        break;
      default:
        break;
    }
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
  knobelnLen = 0;
  korrigierenLen = 0;
  korrigierenRunning = false;
  slalomReady = false;
  slalomOppReady = false;
  slalomRunning = false;
  slalomBereitPhase = false;
  slalomBereitActive = false;
  slalomReadyToRun = false;
  slalomExtActive = false;
  slalomTransitionActive = false;
  slalomTargetLocked = false;
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
void slalomSendPacket(uint8_t cmd, const char* data);

// =====================================================
// COMMAND-HANDLER
// =====================================================
void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  currentGame = GAME_NONE;
  knobelnLen = 0;
  korrigierenLen = 0;
  korrigierenRunning = false;
   raceActive = false;
  memoryStop();
  lineFollowerStop();
  progStop();
  slalomReady = false;
  slalomOppReady = false;
  slalomRunning = false;
  slalomIsHost = false;
  slalomBereitPhase = false;
  slalomReadyToRun = false;
  slalomWaitingForTarget = false;
  slalomBereitActive = false;
  slalomExtActive = false;
  slalomTransitionActive = false;
  slalomTargetLocked = false;
  slalomTargetColor[0] = '\0';
  slalomSetPhase(SLALOM_IDLE);
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

  if (cmd.startsWith("MAX:")) {
    float v = cmd.substring(4).toFloat();
    if (v >= MAX_SPEED_MIN && v <= MAX_SPEED_MAX) {
      config.maxSpeed = v;
      configSaveField("maxSpeed", v);
      snprintf(buf, sizeof(buf), "MAX=%.0f", config.maxSpeed);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("TURN:")) {
    float t = cmd.substring(5).toFloat();
    if (t >= TURN_MIN && t <= TURN_MAX) {
      config.turnFactor = t;
      configSaveField("turnFactor", t);
      snprintf(buf, sizeof(buf), "TURN=%.2f", config.turnFactor);
      sendStatus(buf);
    }
    return;
  }

  if (cmd.startsWith("ACC:")) {
    float a = cmd.substring(4).toFloat();
    if (a >= ACCEL_MIN && a <= ACCEL_MAX) {
      config.accelRate = a;
      configSaveField("accelRate", a);
      snprintf(buf, sizeof(buf), "ACC=%.0f", config.accelRate);
      sendStatus(buf);
    }
    return;
  }

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

  // KNOBELN
  if (cmd.startsWith("KNOBELN:")) {
    String sub = cmd.substring(8);
    if (sub.startsWith("SET:")) {
      String rest = sub.substring(4);
      int colon = rest.indexOf(':');
      if (colon < 0) return;
      uint8_t len = (uint8_t)rest.substring(0, colon).toInt();
      String arr = rest.substring(colon + 1);
      if (len == 0 || len > KNOBELN_MAX) { sendStatus("KNOBELN:BADLEN"); return; }
      knobelnLen = 0;
      int start = 0;
      while (knobelnLen < len) {
        int comma = arr.indexOf(',', start);
        String part = (comma < 0) ? arr.substring(start) : arr.substring(start, comma);
        int v = part.toInt();
        if (v < 0 || v > 3) { sendStatus("KNOBELN:BADARROW"); return; }
        knobelnSeq[knobelnLen++] = (uint8_t)v;
        if (comma < 0) break;
        start = comma + 1;
      }
      snprintf(buf, sizeof(buf), "KNOBELN:READY:%u", knobelnLen);
      sendStatus(buf);
    } else if (sub == "RUN") {
      knobelnExecute();
    } else if (sub == "STOP") {
      currentGame = GAME_NONE;
      stopMotors();
      sendStatus("KNOBELN:STOPPED");
    }
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
    } else if (sub == "START_MATCH") {
      // CMD_START_MATCH (Web -> Host): initiiert den Host-Prozess
      if (!hasPairedPeer) {
        sendStatus("SLALOM:ERROR:NOT_PAIRED");
        return;
      }
      slalomStartMatch();
    } else if (sub == "GO") {
      // CMD_GO (Web nach 3s Countdown): startet das Programm
      // Strikt: nur in Phase READY_WAIT (BOTH_READY), niemals frueh.
      if (slalomReadyToRun && !slalomRunning) {
        slalomReadyToRun = false;
        slalomBereitPhase = false;
        slalomBereitActive = false;
        slalomExecute();
      } else {
        Serial.printf("[SLALOM] GO ignoriert: readyToRun=%d running=%d phase=%s\n",
          slalomReadyToRun, slalomRunning, SLALOM_PHASE_NAMES[slalomPhase]);
      }
    } else if (sub == "STOP") {
      slalomRunning = false;
      currentGame = GAME_NONE;
      stopMotors();
      speedL = 0;
      speedR = 0;
      slalomSendPacket(SLALOM_CMD_RESET, nullptr);
      slalomReady = false;
      slalomOppReady = false;
      slalomBereitPhase = false;
      slalomReadyToRun = false;
      slalomBereitActive = false;
      slalomExtActive = false;
      slalomTransitionActive = false;
      slalomTargetLocked = false;
      slalomTargetColor[0] = '\0';
      slalomSetPhase(SLALOM_IDLE);
      sendStatus("SLALOM:STOPPED");
    } else if (sub == "FINISH") {
      // Eigenes Treffer-Signal an Gegner
      slalomSendPacket(SLALOM_CMD_FINISH, nullptr);
      char msg[48];
      snprintf(msg, sizeof(msg), "SLALOM:FINISH:%s", slalomTargetColor);
      sendStatus(msg);
    } else if (sub == "EXT_RETRY") {
      // 10-Sek-Extension: Programm nochmal starten
      if (slalomExtActive && !slalomRunning) {
        // Wie slalomExecute(), aber Ziel ist bereits bekannt (aus Gegner-Sync)
        currentGame = GAME_SLALOM;
        slalomRunning = true;
        slalomFinished = false;
        slalomRoundTimeout = millis() + 60000;
        slalomSetPhase(SLALOM_RACING);
        sendStatus("SLALOM:RUNNING");
        progSetFromString(slalomMyProgram);
        progRunSlalom(slalomTargetColor);
        slalomRunning = false;
        currentGame = GAME_NONE;
        if (slalomFinished) {
          slalomScoreMe++;
          slalomSendPacket(SLALOM_CMD_OPP_FINISHED, slalomTargetColor);
          // Extension neu starten
          slalomExtStart = millis();
        }
        // Wenn weder ich noch Gegner im Ziel: 10-Sek-Timer in loop() beendet
      }
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
  playStartupBeep();

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
  // Slalom: 3 strikt getrennte Timer + ACK-Timeout
  //   slalomBereitActive      : 10s-Grace fuer 2. Bereit-Druck (Phase READY_WAIT)
  //   slalomExtActive         : 10s-Extension nach Treffer (Phase RACING)
  //   slalomTransitionActive  : 3s-Pause nach Rundenende (Phase ROUND_END)
  //   slalomAckDeadline       : 3s-Timeout fuer CMD_TARGET_ACK (sonst manuell locken)
  // ============================================================

  // (0) ACK-Timeout: falls Client nicht antwortet, Zielfarbe selbst locken
  if (slalomIsHost && !slalomTargetLocked && slalomAckDeadline > 0 &&
      slalomTargetColor[0] != '\0' && millis() >= slalomAckDeadline) {
    Serial.println("[SLALOM] ACK-Timeout (3s) - Client hat nicht geantwortet, locke selbst");
    slalomAckDeadline = 0;
    slalomLockTarget();
  }

  // (1) READY_WAIT: 10s-Grace-Timer fuer 2. Bereit-Druck
  if (slalomBereitActive && millis() - slalomBereitStart >= 10000) {
    slalomBereitActive = false;
    if (slalomBereitPhase && !slalomRunning && slalomPhase == SLALOM_READY_WAIT) {
      Serial.printf("[SLALOM] 10s-Grace READY_WAIT abgelaufen. Bereit: ich=%d opp=%d\n", slalomReady, slalomOppReady);
      if (!slalomReady) {
        // Wir haben nicht gedrückt: leeres Programm
        slalomMyProgram[0] = '\0';
        slalomReady = true;
      }
      if (!slalomOppReady) {
        // Gegner hat nicht gedrückt: leeres Programm ist OK
        // (Wir müssen nicht warten, das Rennen startet)
      }
      // Jetzt "beide bereit" → Web zeigt Countdown + dann fahren
      slalomMaybeRun();
    }
  }
  // (2) RACING: 10s-Extension Timer (nach Treffer, warten auf Gegner)
  if (slalomExtActive && millis() - slalomExtStart >= 10000) {
    slalomExtActive = false;
    if (slalomPhase == SLALOM_RACING) {
      // 10 Sekunden abgelaufen: Runde beenden
      if (!slalomFinished) {
        // Ich habe nicht rechtzeitig getroffen: Gegner bekommt keinen Extra-Punkt
        slalomSendPacket(SLALOM_CMD_TIMEOUT, nullptr);
      }
      slalomRoundEnd();
    }
  }
  // (3) ROUND_END: 3s-Uebergangspause vor naechster Runde
  if (slalomTransitionActive && millis() - slalomTransitionStart >= 3000) {
    slalomTransitionActive = false;
    slalomStartNextRound();
  }
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
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
