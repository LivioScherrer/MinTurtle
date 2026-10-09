// =====================================================
// EduBot V2 - Main Entry Point
// Xiao ESP32-S3 + 2x TMC2209 (UART) + TCS34725 + BLE + ESP-NOW
// =====================================================
// Modulare Firmware - alle Logik in separaten .h/.cpp Dateien
// =====================================================

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <esp_task_wdt.h>

#include "Config.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "I2CBus.h"      // i2cBusLock fuer den I2C?-Scan
#include "BLEManager.h"
#include "Drive.h"
#include "EspNow.h"
#include "MemoryGame.h"
#include "GameEngine.h"
#include "LineFollower.h"
#include "ProgGame.h"
#include "LedStrip.h"
#include "DistanceSensor.h"
#include "Slalom.h"

// =====================================================
// GLOBALE KONSTANTEN
// =====================================================
constexpr unsigned long BAUD_RATE          = 115200;
constexpr int          STACK_COLOR_SENSOR = 8192;
constexpr int          TASK_PRIORITY_SENSOR = 2;
constexpr int          CORE_SENSOR        = 0;
constexpr unsigned long COLOR_NOTIFY_MS    = 100;
constexpr uint32_t     LOOP_DELAY_MS      = 1;

// Zentrale Start-Sperre fuer alle Spiel-Modi. Definiert weiter unten, neben
// cmdLine(); die Vorwaertsdeklaration ist noetig, weil cmdQuiz() und
// cmdRace() davor stehen. Siehe ausfuehrliche Begruendung dort.
static bool claimGameSlot(const char* busyMsg, const char* what);

// Hindernis-Erkennung: VL53L0X (distBlocked(), Schwellen in config.obsDistMM/obsRelMM).
// DIST-Meldungen kommen aus distTick() alle DIST_NOTIFY_MS.

// =====================================================
// GAME STATE MACHINE
// =====================================================
// THREADING: currentGame wird ausschliesslich auf Core 1 gelesen und
// geschrieben - die Main-Loop verarbeitet alle Eingaben (BLE-Events aus der
// Queue, ESP-NOW-Events aus der Queue, Serial) und ruft sich in den
// blockierenden Spiel-Moves ueber bleProcessEvents() selbst erneut auf.
// Motor-Task (Core 0) und ColorSensor-Task (Core 0) lesen currentGame NICHT.
// Deshalb ist hier KEIN volatile noetig; Motor.cpp sichert seinen eigenen
// Zustand separat per Mutex (siehe Motor.h).
GameMode currentGame = GAME_NONE;
bool driveLockedByGame() { return currentGame != GAME_NONE; }

// =====================================================
// SPIEL-BEWEGUNGS-ENGINE (blockierend)
// =====================================================
// Einheitliche Zeiten: jedes Spiel (Memory, Quiz, Rennen) nutzt
// dieselbe Vor/Zurueck- und Drehdauer -> konsistenter Bewegungsablauf.
// Feste Geschwindigkeit; die Anfahr-Rampe (config.rampMs) faehrt der
// AccelStepper im Motor-Task auf Core 0, unabhaengig von dieser Loop.
constexpr uint32_t GAME_TICK_MS    = 1;    // Polling-Intervall in blockierenden Moves (Pulse macht der Motor-Task)
// Spiel-Fahrten laufen inzwischen POSITIONSBASIERT (mm -> Schritte), nicht
// zeitbasiert, damit die Strecke unabhaengig von der Microstep-Zahl bleibt.
// GAME_MOVE_MS wird nur noch als Reserve fuer Zeitfahrten (Pause/Puppe) genutzt.
const uint32_t GAME_MOVE_MS       = 360;  // Reserve-Zeitfahrt, siehe oben
const uint32_t GAME_COLOR_READ_MS = 250;  // Pause waehrend Farbsensor liest
constexpr uint8_t QUIZ_MAX_COLOR_USE = 4;  // jede Farbe max. so oft im Quiz verwendbar

bool gameMoveBusy = false;

// Herkunft des Befehls: true = Web-UI ueber BLE, false = Serial-Monitor.
// Wird von handleBleCommand() gesetzt, von cmdCal() gelesen (Kalibrier-Sperre).
static volatile bool s_cmdFromBle = false;

// Gemeinsame Warte-Schleife fuer alle blockierenden Bewegungen.
// bAbortOnNoGame=true bricht ab, sobald currentGame auf GAME_NONE geht
// (Notfall-Stop ueber STOP / BLE-Disconnect), sonst laeuft der Move durch.
static void waitWhileRunning(uint32_t ms, bool bAbortOnNoGame) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
    if (bAbortOnNoGame && currentGame == GAME_NONE) {
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Wartet bis beide Stepper wirklich stehen (max 1s). Die Pulse erzeugt der
// Motor-Task selbststaendig - hier wird nur auf das Ende gewartet.
static void waitForStop(uint32_t maxExtraMs = 1000) {
  const uint32_t extra = millis();
  while (motorStillRunning() && millis() - extra < maxExtraMs) {
    esp_task_wdt_reset();
    if (currentGame == GAME_NONE) { stopMotors(); return; }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Vorwaerts/Zurueck-Fahrt fuer 'ms'. Vorzeichen = Richtung (setMotorSpeed
// ueberschreibt den Betrag ohnehin auf FIXED_DRIVE_SPEED).
//
// useRamp = false: hier ist die ZEIT das Kommando. Eine Rampe von z. B.
// 150 ms wuerde die ersten 150 ms der gewaehnten Fahrzeit auffressen und der
// Roboter kaeme nie auf die volle Geschwindigkeit - "fahre 400 ms" waere
// dann eine 250-ms-Fahrt plus Anlauf. Wegstrecken ueber prepPositionMove()
// (GameEngine) fahren dagegen mit Rampe, weil dort die Strecke das Kommando ist.
void gameDrive(int32_t ms, bool bForward) {
  gameMoveBusy = true;
  const float target = bForward ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED;
  // Beide Raeder in EINEM Aufruf: hier ist der Versatz zwischen zwei
  // Aufrufen besonders teuer, weil anschliessend gewartet wird - ein
  // verzoegertes zweites Ziel heisst, der Roboter faehrt die ersten
  // Millisekunden einseitig an und die vorgegebene Fahrzeit stimmt nicht mehr.
  setMotorSpeeds(target, target, false);
  waitWhileRunning((uint32_t)ms, true);
  setMotorSpeeds(0, 0, false);
  waitForStop();
  gameMoveBusy = false;
}

void gameDriveForward(uint32_t ms)  { gameDrive((int32_t)ms, true); }
void gameDriveBackward(uint32_t ms) { gameDrive((int32_t)ms, false); }

void gameStop(uint32_t ms) {
  gameMoveBusy = true;
  // Hart, nicht weich: Spiel-Pausen stehen ueblicherweise vor Aktionen, die
  // einen STILLSTEHENDEN Roboter brauchen (Farbe messen, Drehung ansetzen,
  // warten bis etwas da ist). Ein ausrollender Roboter wuerde die Farbe im
  // Vorbeifahren lesen und das Ergebnis waere Muell.
  setMotorSpeeds(0, 0, false);
  // bAbortOnNoGame=false: ein bewusstes Pause-Fenster (z.B. waehrend der
  // Farbsensor liest) darf nicht durch einen Spielabbruch verkuerzt werden.
  waitWhileRunning(ms, false);
  gameMoveBusy = false;
}

// Wartet, bis beide Stepper stehen. bAbortOnNoGame=true bricht bei
// Spielabbruch ab (currentGame == GAME_NONE).
static void waitForStepsDone(bool bAbortOnNoGame) {
  // Harte Obergrenze. Die Schleife wartet darauf, dass der Motor-Task die
  // Positionsschritte auf 0 bringt - und motorStillRunning() liest genau die
  // Variablen, die dieser Task beschreibt. Sollte dort etwas klemmen (z.B. ein
  // dauerhaft gehaltener motorMutex, oder eine Stellung, deren Restzaehler
  // nie 0 wird), wartete diese Schleife uneendlich: der Watchdog greift nicht,
  // denn esp_task_wdt_init(10000, true) sind 10000 SEKUNDEN. Der Roboter
  // stand dann still, ohne eine Meldung, und die BLE-Verbindung blieb offen.
  // 20 s sind fuer die laengste faellige Fahrt (Programm-Block OP_DRIVE_DIST
  // bis 1500 mm) deutlich ueberdimensioniert.
  static constexpr uint32_t WAIT_STEPS_TIMEOUT_MS = 20000;
  const uint32_t start = millis();
  while (motorStillRunning()) {
    esp_task_wdt_reset();
    if (bAbortOnNoGame && currentGame == GAME_NONE) {
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    if (millis() - start > WAIT_STEPS_TIMEOUT_MS) {
      Serial.printf("[GAME] Schritte nach %u ms nicht fertig - Abbruch\n",
                    (unsigned)WAIT_STEPS_TIMEOUT_MS);
      stopMotors();
      gameMoveBusy = false;
      return;
    }
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
}

// Positionsbasiert: exakte Strecke fahren (z.B. 3cm pro Farbe im Quiz).
// Beruecksichtigt config.leftSign/rightSign fuer die Verkabelung.
void gameDriveForwardSteps(int32_t steps) {
  gameMoveBusy = true;
  prepPositionMoves(steps * static_cast<long>(config.leftSign),
                    steps * static_cast<long>(config.rightSign));
  waitForStepsDone(true);
  gameMoveBusy = false;
}

// Rueckwaerts in Schritten - Gegenstueck zu gameDriveForwardSteps, damit
// auch das Memory-Spiel unabhaengig von der Microstep-Zahl dieselbe Strecke
// faehrt (negative Schrittzahl = zurueck).
void gameDriveBackwardSteps(int32_t steps) {
  if (steps < 0) steps = -steps;
  gameMoveBusy = true;
  prepPositionMoves(-(long)steps * (long)config.leftSign,
                    -(long)steps * (long)config.rightSign);
  waitForStepsDone(true);
  gameMoveBusy = false;
}

// Test-Helper: fahre n Schritte ohne currentGame-Abhaengigkeit, mit Timeout.
// Wird vom STEPS:-Befehl genutzt, um den richtigen Wert fuer 3cm zu finden.
void testDriveSteps(int32_t steps) {
  prepPositionMoves(steps * static_cast<long>(config.leftSign),
                    steps * static_cast<long>(config.rightSign));
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 5000) {
    esp_task_wdt_reset();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (motorStillRunning()) stopMotors();
}

// Positionsbasierte Drehung auf der Stelle: beide Motoren bekommen
// entgegengesetzte Richtungen, je 'steps' (vorzeichenkorrigiert).
// bLeft=true -> links rueckwaerts / rechts vorwaerts.
//
// prepPositionMoves() statt zwei Aufrufe: bei einer Drehung ist ein
// zeitlicher Versatz zwischen den Seiten der schlimmste denkbare Fehler -
// die Schrittzahlen stimmen beide, aber das Drehmoment wirkt kurzzeitig nur
// auf ein Rad, und der Roboter steht danach schief.
static void gameTurnSteps(int32_t steps, bool bLeft) {
  gameMoveBusy = true;
  const long l = bLeft ? -steps : steps;
  const long r = bLeft ? steps : -steps;
  prepPositionMoves(l * static_cast<long>(config.leftSign),
                    r * static_cast<long>(config.rightSign));
  waitForStepsDone(true);
  gameMoveBusy = false;
}

void gameTurnLeftSteps(int32_t steps)  { gameTurnSteps(steps, true); }
void gameTurnRightSteps(int32_t steps) { gameTurnSteps(steps, false); }

// Test-Helper: drehe auf der Stelle (rechts) um 'steps' Schritte pro Rad.
void testTurnSteps(int32_t steps) {
  prepPositionMoves( steps * static_cast<long>(config.leftSign),
                    -steps * static_cast<long>(config.rightSign));
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 5000) {
    esp_task_wdt_reset();
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(GAME_TICK_MS));
  }
  if (motorStillRunning()) stopMotors();
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
    if (currentGame != GAME_KORRIGIEREN) {
      // korrigierenRunning MUSS hier zurueckgesetzt werden. Der Abbruch
      // erfolgt praktisch immer ueber einen anderen Spiel-Befehl, der
      // currentGame auf GAME_NONE setzt (RACE:STOP, PROG:STOP, ...), ohne
      // dieses Flag zu kennen. Bliebe es true, wuerde jedes weitere
      // QUIZ:CHECK an meinem Guard scheitern (QUIZ:BUSY) - das Quiz waere
      // bis zum naechsten QUIZ:STOP, BLE-Reconnect oder Reboot tot.
      korrigierenRunning = false;
      stopMotors();
      sendStatus("QUIZ:RESULT:ABORT");
      return;
    }
    gameDriveForwardSteps(configForwardSteps());
    gameStop(GAME_COLOR_READ_MS);
    // Waehrend des Farblesens bleibt der Streifen dunkel: eigenes LED-Licht
    // (rot/gruen) koennte sonst als Farbe auf dem Boden erkannt werden.
    ledStripClear();
    char found[16] = "UNBEKANNT";
    getCurrentColorName(found, sizeof(found));
    strlcpy(korrigierenFound[i], found, sizeof(korrigierenFound[i]));
    if (strcasecmp(korrigierenExpected[i], found) == 0) okCount++;
    char part[48];
    snprintf(part, sizeof(part), "QUIZ:STEP:%u,%s,%s", i, korrigierenExpected[i], found);
    sendStatus(part);
    // LED-Fortschritt: gruen solange alles stimmt, rot sobald ein Fehler dabei ist.
    ledStripShowProgress(i + 1, korrigierenLen,
                         (okCount == i + 1) ? LED_C_GREEN : LED_C_RED);
    ledStripTick();
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
  // Endergebnis zusaetzlich am Roboter: gruener Blitz = alles richtig.
  ledStripShowResultFlash(okCount == korrigierenLen);
  ledStripTick();
  sendStatus(result);
}

// =====================================================
// RENNEN (Multiplayer)
// =====================================================
int  raceMyScore = 0;
int  raceOppScore = 0;
bool raceActive = false;
bool quizActive = false;
// Zielpunktzahl des laufenden Rennens. Default RENNEN_TARGET_SCORE (10), damit
// ein Partner mit aelterer Firmware (die nur RACE:START kennt) weiterhin
// identisch funktioniert. Ueber RACE:START:<ziel>:<stufe> setzbar.
uint8_t raceTargetScore = RENNEN_TARGET_SCORE;
uint8_t raceStage = 0;
// Strecke je richtigem Punkt in mm, gesetzt ueber RACE:STEP:<mm>.
// 0 = keine Angabe: dann gilt die eingelernte Quiz-Strecke (configForwardSteps).
float raceMMPerPoint = 0.0f;
// Letztes Paket vom Gegner (fuer den Peer-Timeout).
// 0 = noch kein Gegner-Schritt gesehen, Timeout ist also noch nicht scharf.
unsigned long raceLastOppMs = 0;

void raceExecuteStart(bool remote = false) {
  currentGame = GAME_RENNEN;
  raceMyScore = 0;
  raceOppScore = 0;
  raceActive = true;
  // Der Timeout startet absichtlich NICHT hier: bis zum ersten Schritt des
  // Gegners darf beliebig viel Zeit vergehen (die erste Aufgabe wird gerechnet).
  raceLastOppMs = 0;
  sendStatus(remote ? "RACE:REMOTE_START" : "RACE:STARTED");
}

// Einen Punkt vorwaerts: feste Strecke in mm (RACE:STEP:<mm>). Alte Web-UIs
// ohne Parameter fallen auf die eingelernte Quiz-Strecke zurueck - auch dann
// wird positionsgefahren, damit die Strecke nie von der Microstep-Zahl haengt.
void raceForwardStepMM(float mm) {
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; return; }
  if (mm > 0.0f) {
    // Positionsfahrt: exakte Strecke, unabhaengig von Tempo und Microsteps.
    const int32_t steps = geoStepsForMM(mm, config.tmcMicrosteps);
    if (steps < 1) return;
    gameDriveForwardSteps(steps);
  } else {
    // Alte Web-UI ohne Parameter: eine feste Strecke statt Zeitbetrieb, damit
    // auch dieser Pfad unabhaengig von der Microstep-Zahl gleich weit faehrt.
    gameDriveForwardSteps(configForwardSteps());
  }
  if (!raceActive) { stopMotors(); speedL = 0; speedR = 0; }
}

void raceForwardStep() { raceForwardStepMM(raceMMPerPoint); }

// Rennen sauber beenden: Motoren aus, Spiel freigeben. Erlaubt das freie
// Fahren wieder - vorher blieb GAME_RENNEN aktiv, bis ein STOP kam.
void raceAbort(bool notifyPeer) {
  raceActive = false;
  currentGame = GAME_NONE;
  stopMotors(); speedL = 0; speedR = 0;
  if (notifyPeer) raceSendPacket((uint8_t)raceMyScore, false);
  sendStatus("RACE:STOPPED");
}

// Prueft, ob der Gegner laenger als RACE_PEER_TIMEOUT_MS nichts gesendet hat.
// true = Rennen muss beendet werden.
// Solange der Gegner noch keinen einzigen Schritt geschickt hat, wird nichts
// abgebrochen - sonst wuerde ein Kind, das die erste Aufgabe noch rechnet,
// nach RACE_PEER_TIMEOUT_MS faelschlich als verloren gewertet.
bool racePeerTimedOut() {
  if (!raceActive || !hasPairedPeer || raceLastOppMs == 0) return false;
  return (unsigned long)(millis() - raceLastOppMs) > RACE_PEER_TIMEOUT_MS;
}

// Forward declarations for Slalom
void onSlalomPacket(const RacePacket* pkt);
void slalomExecute();
void slalomMaybeStart();
void slalomCheckRetry();

void onRacePacket(const RacePacket* pkt) {
  // Timeout erst ab dem ersten echten Renn-Schritt des Gegners zaehlen
  // (wenn das Rennen schon laeuft) - Scan-/Start-Pakete scharfmachen nichts.
  if (raceActive) raceLastOppMs = millis();
  if (pkt->flags & RACE_FLAG_WINNER || pkt->score >= raceTargetScore) {
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
// BLE-CONNECT/DISCONNECT-HANDLER
// =====================================================
void onBleConnected() {
  deviceConnected = true;
  lastCommandTime = millis();
  Serial.println("[BLE] Verbunden");
  sendStatus("EduBot V2 bereit");
}

void onBleDisconnected() {
  deviceConnected = false;
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  // Gegenstueck zu hardStop(): der EN-Pin ist fuer BEIDE Treiber gemeinsam,
  // also schaltet ein HIGH den Robotor auch dann hart ab, wenn stopMotors()
  // den Mutex nicht bekommen haette. Ohne das bliebe ein Disconnect waehrend
  // einer Fahrt ohne Not-Aus-Backstop.
  tmcDisable();
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
// Jede Befehlsgruppe hat einen eigenen Handler. Ein Handler bekommt den
// bereits getrimmten, GROSSBUCHSTABEN-Befehl und den Teil hinter dem
// Doppelpunkt (sub, nur bei Prefix-Befehlen belegt) und gibt true zurueck,
// wenn er den Befehl verarbeitet hat. handleCommand() ist dadurch nur noch
// die Reihenfolge der Pruefungen - jede Gruppe laesst sich einzeln lesen,
// aendern und testen.
void hardStop() {
  driveClearKeys();
  speedL = 0.0; speedR = 0.0;
  stopMotors();
  tmcDisable();   // Treiber ganz aus (kein Haltestrom, keine Waerme)
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

// Erkennt die Spiel-STOP-Befehle. Wird ZENTRAL in handleCommand() benutzt:
// ein Spiel-STOP muss auch die WASD-Tasten loeschen. Andernfalls nimmt die
// Main-Loop im naechsten Tick die noch gesetzten Tasten wieder auf und der
// Roboter faehrt mit voller Geschwindigkeit weiter, obwohl das Spiel
// gestoppt wurde. Weil die WebUI zusaetzlich alle ~50 ms ein KEYS:-
// Heartbeat schickt, greift der Timeout in driveUpdate() dabei nie.
// Zentral statt in jedem Handler: ein einzelner vergessener Handler waere
// sofort wieder derselbe Aerger.
static bool cmdIsGameStop(const String& cmd) {
  return cmd == "LINE:STOP"  || cmd == "MEMORY:STOP" || cmd == "QUIZ:STOP"
      || cmd == "RACE:STOP"  || cmd == "PROG:STOP"  || cmd == "SLALOM:STOP";
}

// ---- Tastatur-Status (KEYS:<W><A><S><D>) ----
static bool cmdKeys(const String& cmd) {
  String data = cmd.substring(5);
  if (data.length() >= 4) {
    keyW = (data.charAt(0) == '1');
    keyA = (data.charAt(1) == '1');
    keyS = (data.charAt(2) == '1');
    keyD = (data.charAt(3) == '1');
  }
  return true;
}

// ---- Distanzsensor: Messwert, Auto-Meldung, Diagnose, Schwellen, Offset ----
static bool cmdDist(const String& cmd) {
  char buf[48];
  if (cmd == "DIST?") {
    int16_t mm = distMeasureMM();
    if (mm >= 0) snprintf(buf, sizeof(buf), "DIST:%d", mm);
    else snprintf(buf, sizeof(buf), "DIST:--");
    sendStatus(buf);
    return true;
  }
  if (cmd == "DISTAUTO:ON") { distSetAuto(true); sendStatus("DISTAUTO:ON"); return true; }
  if (cmd == "DISTAUTO:OFF") { distSetAuto(false); sendStatus("DISTAUTO:OFF"); return true; }

  // Tischkanten-Schwellen fuer die Anzeige in EDGE?. Die WAECHTER-LOGik und
  // damit die Wahrheit liegt in ProgGame.cpp (EDGE_RAW_C_MAX,
  // EDGE_RAW_C_RELEASE, EDGE_CONFIRM_N, edgeGuardTick). Hier nur gespiegelt.
constexpr uint16_t EDGE_RAW_C_MAX_DISP    = 300;   // darunter = Kante
constexpr uint16_t EDGE_RAW_C_RELEASE_DISP = 400;  // darueber = wieder frei

  // Tischkanten-Waechter. Der Sensor sitzt nach unten; ueber der Kante faellt
  // die Gesamthelligkeit auf fast 0, zwei Messungen in Folge -> sofortiger Halt.
  // Standardmaessig EINGESCHALTET (config.edgeGuard, NVS-Schluessel "edgeGuard").
  // Ausgeschaltet wird er nur in den Farbsensor-Einstellungen der Web-UI.
  if (cmd == "EDGE:ON") {
    edgeGuardSet(true);
    sendStatus("EDGE:ON");
    return true;
  }
  // EDGE:OFF gibt es nicht mehr: der Wächter ist immer an. Abgelehnt statt
  // ignoriert, damit eine alte Web-UI eine klare Antwort bekommt.
  if (cmd == "EDGE:OFF") {
    sendStatus("EDGE:ALWAYS_ON");
    Serial.println("[KANTE] EDGE:OFF abgelehnt - der Wächter ist immer an");
    return true;
  }
  if (cmd == "EDGE:RESET") {
    edgeGuardReset();
    motorReleaseEmergency();   // Not-Halt aufheben - sonst fahren die Motoren nie wieder
    sendStatus("EDGE:RESET");
    Serial.println("[KANTE] Zaehler zurueckgesetzt - Roboter wieder freigegeben");
    return true;
  }
  if (cmd == "EDGE?") {
    char eb[48];
    snprintf(eb, sizeof(eb), "EDGE=%u,%u,%u,%u",
             edgeGuardActive() ? 1u : 0u, (unsigned)rawSensorC, (unsigned)rawSensorR,
             motorEmergencyActive() ? 1u : 0u);
    sendStatus(eb);
    Serial.printf("[KANTE] %s | Clear=%u (Kante < %u, frei ab %u) | Rot=%u%s\n",
                  edgeGuardActive() ? "aktiv" : "aus",
                  (unsigned)rawSensorC, (unsigned)EDGE_RAW_C_MAX_DISP,
                  (unsigned)EDGE_RAW_C_RELEASE_DISP, (unsigned)rawSensorR,
                  motorEmergencyActive() ? " | NOT-HALT aktiv" : "");
    return true;
  }
  // Diagnose: kompakte Zeile nach BLE + ausfuehrlicher Status auf Serial.
  if (cmd == "DISTDIAG" || cmd == "DIST:DIAG") {
    distDiagPrint();
  // 128 statt 96. Die Zeile besteht aus 9 Feldern, vier davon sind
  // unsigned-long-Zaehler (je 10 Stellen). Rechnerisch schlimmster Fall:
  //   "DIST:DIAG:" 10 + ok 1 + lastMM 4 + lastRaw 4 + status 3 + offset 4
  //   + 4 Zaehler a 10 = 40 + 8 Komma  =>  rund 74 Zeichen + NUL
  // Der alte 96-Byte-Puffer haette also gerade noch gepasst - aber nur
  // knapp, und ohne Reserve. 128 gibt Luft und bleibt deutlich unter der
  // BLE-Nutzlastgrenze (MTU 247 -> 244 Byte), sodass die Diagnose auch bei
  // sehr grossen Zaehlern nicht mittendrin abgeschnitten wird.
  char dbuf[128];
  distDiag(dbuf, sizeof(dbuf));
    sendStatus(dbuf);
    return true;
  }
  // Ref-SPAD-Kalibrierung: einziger Weg, einen Sensor mit unbrauchbaren
  // NVM-Ref-SPAD-Daten wiederzubeleben. Laeuft asynchron, weil sie den
  // Loop-Task sekundenlang blockieren wuerde (Motor, Spiele, BLE-Retries).
  if (cmd == "DISTREF") {
    if (distRefSpadRequest()) {
      sendStatus("DISTREF:RUN");
    } else {
      sendStatus("DISTREF:BUSY");
    }
    return true;
  }
  // Init-Debug: jeden Schritt mit Library-Debug nachvollziehen (nur Diagnose).
  if (cmd == "DISTDEBUG") {
    distDebugInit();
    return true;
  }
  // Bus-Zustand vom Boot-Zeitpunkt (beweist Power-Cycle-Wirkung).
  if (cmd == "DISTSCAN" || cmd == "DIST:SCAN") {
    distBootScan();
    sendStatus("DIST:SCAN_DONE");
    return true;
  }
  // Hindernisschwellen (NVS-persistent, Hysterese: Freigabe > Stopp).
  if (cmd == "OBSDIST?") {
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("OBSDIST:")) {
    int v = cmd.substring(8).toInt();
    if (v < 50 || v > 1200) { sendStatus("OBSDIST:BAD"); return true; }
    // Die Hysterese braucht obsRelMM > obsDistMM. Der Mitlauf-Frei-Wert
    // darf deshalb nicht unter den Stopp-Wert kippen und nicht ueber
    // 1200 hinauslaufen (sonst waere er selbst ausserhalb des Bereichs,
    // den die WebUI und configValidate zulassen). Ist kein gueltiger
    // Mitlauf-Wert moeglich, wird der Widerspruch abgelehnt statt ein
    // kollabiertes Band zu speichern - sonst laesst sich distBlocked()
    // nie wieder loesen und der Roboter blockiert dauerhaft.
    int rel = min(v + 30, 1200);
    if (rel <= v) { sendStatus("OBSDIST:BAD"); return true; }
    config.obsDistMM = (int16_t)v;
    config.obsRelMM  = (int16_t)rel;
    configSaveInt("obsDist", v);
    configSaveInt("obsRel", rel);
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
  if (cmd == "OBSREL?") {
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("OBSREL:")) {
    int v = cmd.substring(7).toInt();
    if (v < 50 || v > 1200) { sendStatus("OBSREL:BAD"); return true; }
    // Spiegelbild zu OBSDIST: Stopp-Wert muss unter dem Frei-Wert bleiben
    // und selbst im Bereich liegen. Sonst kollabiert das Hystereseband.
    int st = max(v - 30, 50);
    if (st >= v) { sendStatus("OBSREL:BAD"); return true; }
    config.obsRelMM  = (int16_t)v;
    config.obsDistMM = (int16_t)st;
    configSaveInt("obsRel", v);
    configSaveInt("obsDist", st);
    snprintf(buf, sizeof(buf), "OBSDIST=%d", config.obsDistMM);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "OBSREL=%d", config.obsRelMM);
    sendStatus(buf);
    return true;
  }
  // Offset-Kalibrierung (Datenblatt-Verfahren): weisses Target exakt bei
  // <mm> (50..500, empfohlen 100) ruhig halten, dann DISTCAL:<mm> senden.
  if (cmd == "DISTCAL?") {
    snprintf(buf, sizeof(buf), "DISTCAL=%d", distOffsetMM());
    sendStatus(buf);
    return true;
  }
  if (cmd == "DISTCAL:CLR") {
    distClearOffset();
    sendStatus("DISTCAL=0");
    return true;
  }
  if (cmd.startsWith("DISTCAL:")) {
    int v = cmd.substring(8).toInt();
    if (v < 50 || v > 500) { sendStatus("DISTCAL:BAD"); return true; }
    sendStatus("DISTCAL:RUN");
    if (distCalibrateOffset((int16_t)v)) {
      snprintf(buf, sizeof(buf), "DISTCAL=%d", distOffsetMM());
      sendStatus(buf);
    } else {
      sendStatus("DISTCAL:FAIL");
    }
    return true;
  }
  // 3-Punkt-Kalibrierung (Web-UI): erst an den drei Abstaenden roh messen
  // (DISTMEAS:<mm>), dann die drei Roh-Mittelwerte zusammen senden
  // (DISTCAL3:<roh50>,<roh100>,<roh200>). Die Messung blockiert nur kurz
  // (~1 s) und verarbeitet dabei BLE-Events.
  if (cmd.startsWith("DISTMEAS:")) {
    int v = cmd.substring(9).toInt();
    if (v < 20 || v > 500) { sendStatus("DISTMEAS:BAD"); return true; }
    int16_t raw = -1;
    if (distMeasureRawAvg(raw)) {
      snprintf(buf, sizeof(buf), "DISTMEAS=%d,%d", v, (int)raw);
    } else {
      snprintf(buf, sizeof(buf), "DISTMEAS:FAIL");
    }
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("DISTCAL3:")) {
    // Format: DISTCAL3:<roh50>,<roh100>,<roh200> (jeweils > 0, <= 2000 mm)
    int r[3] = { 0, 0, 0 };
    int idx = 0;
    bool ok = true;
    const char* p = cmd.c_str() + 9;
    while (*p != '\0' && idx < 3) {
      char* end = nullptr;
      long val = strtol(p, &end, 10);
      if (end == p || val <= 0 || val > 2000) { ok = false; break; }
      r[idx++] = (int)val;
      p = end;
      if (*p == ',') p++;
    }
    if (!ok || idx != 3) { sendStatus("DISTCAL3:BAD"); return true; }
    if (distCalibrate3((int16_t)r[0], (int16_t)r[1], (int16_t)r[2])) {
      snprintf(buf, sizeof(buf), "DISTCAL3=%.3f,%d",
               (double)config.obsScale, (int)config.obsOffsetMM);
      sendStatus(buf);
    } else {
      sendStatus("DISTCAL3:BAD");
    }
    return true;
  }
  if (cmd == "I2C?") {
    // Bus-Scan 0x08..0x77. Gesund: 0x29 (TCS34725) + 0x30 (VL53L0X nach Umzug).
    // Nur 0x29 = VL53L0X fehlt/tot. Nichts = Bus/Wiring/Pull-ups pruefen.
    // Der Scan MUSS den Bus sperren: er laeuft zur Laufzeit, und der
    // Farbsensor liest im selben Moment ueber dasselbe TwoWire-Objekt.
    // TwoWire serialisiert nur Einzelaufrufe, nicht die Dreierfolge des
    // Farbsensors - dazwischen einzuschieben verschiebt dessen
    // Registerzeiger.
    //
    // Laengenbegrenzung: sendStatus() geht per BLE-Notify raus, und bei
    // MTU 247 sind das 247-3 = 244 Byte Nutzlast. Der Scan ueber 0x08..0x77
    // kann bis zu 112 Adressen finden und mit " %02X" rund 450 Zeichen
    // ergeben - bei einem halb defekten Bus antworten viele Adressen, und
    // die Antwort wurde dann vom BLE-Stack stillschweigend abgeschnitten
    // bzw. verworfen. Die Web-UI zeigte danach gar nichts. Es wird daher
    // auf 200 Zeichen Nutzlast gekappt und die Anzahl gefundener
    // Adressen mitgeschickt, damit die Diagnose vollstaendig bleibt.
    static constexpr size_t I2C_SCAN_MAX_LEN = 200;
    // Platz fuer die Schlussangabe reservieren, damit die Diagnose NIEMALS
    // unmarkiert gekuerzt wird: ohne "n=<Anzahl>" bzw. "+<Anzahl>" waere eine
    // abgeschnittene Liste nicht als solche erkennbar und man haette eine
    // vollstaendige, aber falsche Liste vor sich.
    char out[I2C_SCAN_MAX_LEN + 1];
    // Platzbedarf am Ende: " :n=65535" = 9 Zeichen, " +65535" = 7, Plus NUL.
    static constexpr size_t I2C_TAIL_RESERVE = 12;
    static constexpr size_t I2C_LIST_MAX     = I2C_SCAN_MAX_LEN - I2C_TAIL_RESERVE;
    memcpy(out, "I2C:", 4);
    size_t outLen = 4;
    uint16_t found = 0;    // ALLE antwortenden Adressen, auch die nicht gelisteten
    uint16_t listed = 0;   // davon tatsaechlich in die Liste geschriebene
    if (!i2cBusLock(500)) {
      Serial.println("[I2C] Scan nicht moeglich: Bus belegt");
      sendStatus("I2C:BUSY");
      return true;
    }
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        found++;
        if (outLen + 3 > I2C_LIST_MAX) continue;   // Liste voll -> nur noch zaehlen
        snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen, " %02X", a);
        outLen += 3;
        listed++;
      }
    }
    i2cBusUnlock();
    if (found > listed) {
      // Gekuerzt: die Gesamtzahl steht mit da, damit klar ist, dass die Liste
      // unvollstaendig ist und wie viel fehlt.
      snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen,
               " +%u", (unsigned)(found - listed));
    } else {
      snprintf(out + outLen, (I2C_SCAN_MAX_LEN + 1) - outLen,
               " :n=%u", (unsigned)found);
    }
    sendStatus(out);
    Serial.printf("[I2C] %u Adressen (%u gelistet): %s\n",
                  (unsigned)found, (unsigned)listed, out + 4);
    return true;
  }
  return false;
}

// ---- Farb-Kalibrierung: CAL:BLACK / WHITE / ROT / GELB / GRUEN / BLAU / ? ----
// Einmal kalibriert, dann gilt es: Schreibzugriffe (BLACK/WHITE/ROT/GELB/
// GRUEN/BLAU) sind ueber die Web-UI (BLE) GESPERRT und antworten CAL:LOCKED.
// Ueber Serial (USB, Service-Weg) bleiben sie offen. Lesen (CAL:?, CAL:COLOR)
// geht auf beiden Wegen. Grund: versehentliches Neukalibrieren per Knopfdruck
// hat schon einmal die funktionierende Einlernung mit falschen Werten
// ueberschrieben - das darf nicht wieder passieren.
static bool cmdCal(const String& cmd) {
  char buf[48];
  const bool isWrite = (cmd == "CAL:BLACK" || cmd == "CAL:WHITE" ||
                        cmd == "CAL:RED" || cmd == "CAL:YELLOW" ||
                        cmd == "CAL:GREEN" || cmd == "CAL:BLUE");
  if (isWrite && s_cmdFromBle) {
    sendStatus("CAL:LOCKED");
    Serial.printf("[CAL] %s ueber Web-UI abgelehnt - Kalibrierung ist gesperrt "
                  "(geht nur ueber Serial)\n", cmd.c_str());
    return true;
  }
  if (cmd == "CAL:BLACK") {
    // Sperre wirklich noetig: gelingt sie nicht, waeren rawSensorR/G/B stale
    // oder - schlimmer - ein halb geschriebener Satz. Vorher speicherte der
    // Code in diesem Fall einfach die ALTEN Werte und meldete trotzdem
    // "CAL:BLACK OK" - eine Fehlkalibrierung, die erst beim Spielen auffiel
    // und sich nicht mehr zuordnen liess.
    if (!colorSem || xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      sendStatus("CAL:BLACK BUSY");
      Serial.println("[CAL] BLACK: Farb-Sperre nicht bekommen - nicht gespeichert");
      return true;
    }
    config.blackR = rawSensorR; config.blackG = rawSensorG; config.blackB = rawSensorB;
    xSemaphoreGive(colorSem);
    configSaveUShort("blackR", config.blackR);
    configSaveUShort("blackG", config.blackG);
    configSaveUShort("blackB", config.blackB);
    sendStatus("CAL:BLACK OK");
    return true;
  }
  if (cmd == "CAL:WHITE") {
    if (!colorSem || xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) != pdTRUE) {
      sendStatus("CAL:WHITE BUSY");
      Serial.println("[CAL] WHITE: Farb-Sperre nicht bekommen - nicht gespeichert");
      return true;
    }
    config.whiteR = rawSensorR; config.whiteG = rawSensorG; config.whiteB = rawSensorB;
    xSemaphoreGive(colorSem);
    configSaveUShort("whiteR", config.whiteR);
    configSaveUShort("whiteG", config.whiteG);
    configSaveUShort("whiteB", config.whiteB);
    sendStatus("CAL:WHITE OK");
    return true;
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
      return true;
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
    return true;
  }
  // Sensor-Farbe auf Serial. Die Auswertung detectColorName() passiert im
  // Sensor-Task und liegt nur im Speicher; ueber BLE kommt COLOR:..., aber
  // BLE ist im seriellen Monitor nicht sichtbar. Fuer die Kalibrierung wird
  // deshalb hier derselbe Wert ausgegeben - sonst laesst sich am Serial-Monitor
  // nicht pruefen, was der Sensor gerade erkennt.
if (cmd == "CAL:COLOR" || cmd == "CAL:FARBE") {
    char nb[16] = {};
    int sr = 0, sg = 0, sb = 0;
    if (colorSem && xSemaphoreTake(colorSem, pdMS_TO_TICKS(50)) == pdTRUE) {
      sr = sensorR; sg = sensorG; sb = sensorB;
      strlcpy(nb, (const char*)sensorName, sizeof(nb));
      xSemaphoreGive(colorSem);
    }
    Serial.printf("[CAL] erkannt: %-10s rgb(%d,%d,%d)\n", nb, sr, sg, sb);
    char cm[48];
    snprintf(cm, sizeof(cm), "CAL:COLOR=%d,%d,%d,%s", sr, sg, sb, nb);
    sendStatus(cm);
    return true;
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
    return true;
  }
  return false;
}

// ---- TMC2209-Treiber: Strom / Hold / Microsteps / Modus / EN / Diagnose ----
static bool cmdTmc(const String& cmd) {
  char buf[48];
  // Set-Befehle werden im NVS gespeichert und ueberleben einen Reboot.
  // Nach jeder Aenderung kommt zusaetzlich eine TMC:STAT:-Kompaktzeile.
  if (cmd == "TMC?") { tmcPrintStatus(); sendStatus("TMC:LOGGED"); tmcSendCompactStatus(); return true; }
    // UARTTEST ist absichtlich gesperrt: der Befehl schreibt an die
    // TMC-Adressen 0..3. Da ein TMC2209 seine UART-Adresse pro Datagram
    // hochzaehlt, wandern die beiden Treiber danach auf Adresse 2/3 und
    // antworten nicht mehr - bis ein Neustart die OTP-Adressen zuruecksetzt.
    // UARTNET (nur lesend) bleibt erlaubt.
    if (cmd == "UARTTEST") { sendStatus("UARTTEST:GESPERRT"); return true; }
  if (cmd == "UARTNET") { tmcNetDiag(); sendStatus("NETZ:DIAG_DONE"); return true; }
    // MOT:DIAG: liest GSTAT, SG_RESULT, VACTUAL, TSTEP, MSCNT und DRV_STATUS
    // von BEIDEN Treibern. Reine Beobachtung - es wird nichts geregelt und
    // der Roboter nicht angehalten. Der Lauf braucht 16 Loop-Durchlaeufe
    // (ein Register pro Durchlauf, ~2 ms Lesen), danach kommt die Auswertung
    // im Log und als MOT:MEAS:-Zeile fuer die Web-UI.
    if (cmd == "MOT:SYNC") {
      // Beide Motoren auf denselben Mikroschritt bringen. Laeuft bei jedem
      // Moduswechsel automatisch; dieser Befehl macht es auf Abruf.
      const bool korrigiert = motorSyncMicrosteps();
      sendStatus(korrigiert ? "MOT:SYNC:OK" : "MOT:SYNC:OK");
      return true;
    }
    if (cmd == "MOT:SPD?") {
      // Messung mitten in der Fahrt: VACTUAL beider Treiber (= wie schnell
      // drehen die Raeder wirklich) plus MSCNT als Referenz. Genau damit
      // laesst sich ein dauerhafter Drift von "die Raeder kommen
      // unterschiedlich weit" (mechanisch) trennen.
      if (!motorDiagSteadySpeed()) sendStatus("MOT:SPD:NO_TMC");
      return true;
    }
    if (cmd == "MOT:DIAG") {
      if (!motorDiagRequest()) {
        // Grund: Treiber aus (Auto-Standby) oder ein Lauf laeuft bereits.
        sendStatus(motorDiagRunning() ? "MOT:DIAG:BUSY" : "MOT:DIAG:NO_TMC");
        Serial.printf("[CMD] MOT:DIAG abgelehnt: %s\n",
                      motorDiagRunning() ? "Messlauf laeuft bereits" : "Treiber aus (kein EN)");
      }
      return true;
    }
  // TMC:CURRENT: und TMC:HOLD: gibt es nicht mehr. Der Motorstrom ist fest
  // 200 mA (Nennstrom des NEMA8, Motor.h TMC_CURRENT_MA_FIXED), IHOLD ist der
  // halbe Fahrstrom - beides wird in configureOne() abgeleitet.
  //
  // Die Befehle werden abgelehnt statt ignoriert: eine alte Web-UI oder ein
  // altes Skript, das sie noch schickt, soll eine klare Antwort bekommen und
  // nicht stillschweigend denken, der Strom waere jetzt so eingestellt.
  if (cmd.startsWith("TMC:CURRENT:") || cmd.startsWith("TMC:HOLD:")) {
    snprintf(buf, sizeof(buf), "TMC:FEST=%u", tmcGetCurrent()); sendStatus(buf);
    Serial.printf("[CMD] %s abgelehnt - der Strom ist fest %u mA\n",
                  cmd.c_str(), (unsigned)tmcGetCurrent());
    return true;
  }
  if (cmd.startsWith("TMC:MS:")) {
    int v = cmd.substring(7).toInt();
    if (tmcSetMicrosteps((uint16_t)v)) {
      // tmcSetMicrosteps() setzt config.tmcMicrosteps und die Schrittrate
      // bereits selbst - und zwar VOR dem Readback, mit Rollback bei
      // Misserfolg. Hier steht deshalb nur noch das Persistieren.
      configSaveUShort("tmcMS", config.tmcMicrosteps);
      Serial.printf("[GEO] %u Microsteps: %.1f mm = %ld Schritte, %.1f Grad = %ld Schritte\n",
                    config.tmcMicrosteps, (double)config.quizForwardMM,
                    (long)configForwardSteps(), (double)config.quizTurnDeg,
                    (long)configTurnSteps());
      snprintf(buf, sizeof(buf), "TMC:MS=%u", tmcGetMicrosteps()); sendStatus(buf);
      tmcSendCompactStatus();
    } else {
      sendStatus("TMC:BAD_MS");
    }
    return true;
  }
  if (cmd == "TMC:MODE:SPREAD") {
    tmcSetSpreadCycle(true); config.tmcSpread = true; configSaveBool("tmcSpread", true);
    sendStatus("TMC:MODE=SPREAD"); tmcSendCompactStatus(); return true;
  }
  if (cmd == "TMC:MODE:STEALTH") {
    tmcSetSpreadCycle(false); config.tmcSpread = false; configSaveBool("tmcSpread", false);
    sendStatus("TMC:MODE=STEALTH"); tmcSendCompactStatus(); return true;
  }
  if (cmd == "TMC:EN:1") { tmcEnable(); sendStatus("TMC:EN=1"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:EN:0") { tmcDisable(); sendStatus("TMC:EN=0"); tmcSendCompactStatus(); return true; }
  if (cmd == "TMC:INIT") { tmcReinit(); sendStatus("TMC:INIT_DONE"); tmcSendCompactStatus(); return true; }
    // TMC:REG: Register-Dump mit Schreibtest - damit laesst sich von aussen
    // pruefen, ob der Treiber wirklich konfiguriert ist (und ob Schreibzugriffe
    // ueberhaupt ankommen). Nur lesend, ausser dem einen GCONF-Bit im Test,
    // das sofort wieder zurueckgeschrieben wird.
    if (cmd == "TMC:REG") { tmcRegDump(); sendStatus("TMC:REG_DONE"); return true; }
  return false;
}

// ---- Geometrie-Einlernung + Tempo (alles in physikalischen Einheiten) ----
// Gespeichert werden mm / Grad / mm-s, NIEMALS Schrittzahlen. Die Schritte
// werden bei jedem Aufruf aus der aktuellen Microstep-Zahl berechnet - die
// Einlernung gilt also fuer JEDE Microstep-Einstellung.
//   STEPS:<mm>    Strecke einer Farb-Raste (Werkwert 30)
//   ANG:<grad>    Drehung im Spiel (Werkwert 90)
//   SPEED:<mm/s>  Fahrgeschwindigkeit (Werkwert 150)
// (Rampe und Anlaufdrehung: RAMP: / SPIN:, siehe cmdMotorFeel weiter unten)
static bool cmdSteps(const String& cmd) {
  char buf[64];
  if (cmd == "STEPS?") {
    snprintf(buf, sizeof(buf), "STEPS=%.1f,%ld", (double)config.quizForwardMM,
             (long)configForwardSteps());
    sendStatus(buf);
    return true;
  }
  if (cmd == "GEO?") {
    snprintf(buf, sizeof(buf), "GEO:%u,%.2f,%.1f,%.1f",
             (unsigned)(MOTOR_FULL_STEPS_PER_REV * config.tmcMicrosteps),
             (double)configStepsPerMM(), (double)config.quizForwardMM,
             (double)config.quizTurnDeg);
    sendStatus(buf);
    return true;
  }
  if (cmd == "GEO:RESET") {
    configResetGeometry();
    snprintf(buf, sizeof(buf), "GEO:RESET=%.1f,%.1f",
             (double)config.quizForwardMM, (double)config.quizTurnDeg);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("STEPS:")) {
    float mm = cmd.substring(6).toFloat();
    if (!inRange(mm, FORWARD_MM_MIN, FORWARD_MM_MAX)) { sendStatus("STEPS:BAD"); return true; }
    config.quizForwardMM = mm;
    configSaveField("quizMM", mm);
    // Sofort in der neuen Distanz testen
    testDriveSteps(configForwardSteps());
    snprintf(buf, sizeof(buf), "STEPS=%.1f,%ld", (double)mm, (long)configForwardSteps());
    sendStatus(buf);
    Serial.printf("[STEPS] %.1f mm = %ld Schritte (bei %u Microsteps) gefahren und gespeichert\n",
                  (double)mm, (long)configForwardSteps(), config.tmcMicrosteps);
    return true;
  }
  return false;
}

static bool cmdAng(const String& cmd) {
  char buf[64];
  if (cmd == "ANG?") {
    snprintf(buf, sizeof(buf), "ANG=%.1f,%ld", (double)config.quizTurnDeg,
             (long)configTurnSteps());
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("ANG:")) {
    float deg = cmd.substring(4).toFloat();
    if (!inRange(deg, TURN_DEG_MIN, TURN_DEG_MAX)) { sendStatus("ANG:BAD"); return true; }
    config.quizTurnDeg = deg;
    configSaveField("quizDeg", deg);
    // Sofort testen (Drehung rechts auf der Stelle)
    testTurnSteps(configTurnSteps());
    snprintf(buf, sizeof(buf), "ANG=%.1f,%ld", (double)deg, (long)configTurnSteps());
    sendStatus(buf);
    Serial.printf("[ANG] %.1f Grad = %ld Schritte/Rad gedreht und gespeichert\n",
                  (double)deg, (long)configTurnSteps());
    return true;
  }
  return false;
}

// ---- Fahrgeschwindigkeit (mm/s) ----
static bool cmdSpeed(const String& cmd) {
  char buf[64];
  if (cmd == "SPEED?") {
    snprintf(buf, sizeof(buf), "SPEED=%.0f,%.0f", (double)config.driveSpeedMM,
             (double)FIXED_DRIVE_SPEED);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("SPEED:")) {
    float mmS = cmd.substring(6).toFloat();
    if (!inRange(mmS, SPEED_MM_S_MIN, SPEED_MM_S_MAX)) { sendStatus("SPEED:BAD"); return true; }
    config.driveSpeedMM = mmS;
    configSaveField("driveSpeed", mmS);
    motorUpdateSpeeds();
    snprintf(buf, sizeof(buf), "SPEED=%.0f,%.0f", (double)mmS, (double)FIXED_DRIVE_SPEED);
    sendStatus(buf);
    Serial.printf("[SPEED] %.0f mm/s = %.0f Schritte/s (%u Microsteps)%s\n",
                  (double)mmS, (double)FIXED_DRIVE_SPEED, config.tmcMicrosteps,
                  (double)mmS * configStepsPerMM() > MAX_DRIVE_STEPS_S
                    ? " - am Puls-Task begrenzt" : "");
    return true;
  }
  return false;
}

// ---- Anfahrzeit + Einschalt-Drehung ----
// Beide kommen aus der NVS und sind in der Web-UI einstellbar. Sie sind
// bewusst NICHT im TMC:-Bereich: sie gehoeren nicht zum Treiber, sondern zur
// Laufzeit des Motor-Tasks, und ein TMC:RESET soll sie nicht mit zuruecksetzen.
//   RAMP:<ms>            Anfahrzeit bis zum vollen Tempo, 0 = aus (Werkwert 150)
//   SPIN:<ms>            Dauer der Einschalt-Drehung, 0 = aus
//   SPIN:<schritte/s>,<ms>  beides zusammen (Serial-Monitor)
static bool cmdMotorFeel(const String& cmd) {
  char buf[64];
  if (cmd == "RAMP?") {
    snprintf(buf, sizeof(buf), "RAMP=%u,%u", (unsigned)config.rampMs,
             motorRampActive() ? 1u : 0u);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("RAMP:")) {
    // toInt() statt toFloat(): die Angabe ist eine Zeit in ganzen ms, und ein
    // "RAMP:1e9" soll nicht als 1000000000 durchrutschen.
    long v = cmd.substring(5).toInt();
    if (v < RAMP_MS_MIN || v > RAMP_MS_MAX) { sendStatus("RAMP:BAD"); return true; }
    config.rampMs = (uint16_t)v;
    configSaveUShort("rampMs", config.rampMs);
    // motorUpdateSpeeds() setzt das Flag, das der Puls-Task liest - deshalb
    // muss es hier ueber den normalen Weg laufen, nicht direkt am Config-Feld.
    motorUpdateSpeeds();
    snprintf(buf, sizeof(buf), "RAMP=%u,%u", (unsigned)config.rampMs,
             motorRampActive() ? 1u : 0u);
    sendStatus(buf);
    Serial.printf("[RAMP] %u ms (0 = keine Rampe)\n", (unsigned)config.rampMs);
    return true;
  }
  if (cmd == "SPIN?") {
    snprintf(buf, sizeof(buf), "SPIN=%u,%u", (unsigned)config.spinSpeed,
             (unsigned)config.spinMs);
    sendStatus(buf);
    return true;
  }
  if (cmd.startsWith("SPIN:")) {
    // Zwei Formen, absichtlich:
    //   SPIN:<ms>            nur die Dauer, die Drehzahl bleibt wie sie ist
    //   SPIN:<schritte/s>,<ms>  beide (Serial-Monitor, Feinabstimmung)
    // Die Kurzform gibt es, weil die Web-UI nur EINE verstaendliche Zahl
    // zeigen soll: "Schritte/s" in einer Grundschul-Einstellungsseite war
    // unlesbar, und eine hartkodierte 6000 im JS haette den Firmware-Wert
    // SPIN_SPEED_DEFAULT dupliziert - bei einer Aenderung dort wuerde die
    // Seite stillschweigend zurueckschreiben.
    //
    // Drehzahl und Dauer muessen ZUSAMMEN gesetzt werden: ein halb gesetzter
    // Anlaufdreher (Drehzahl 0, Dauer 500) waere eine Fahrt, die nie
    // stattfindet, aber den Boot um 500 ms verzoegert.
    const char* arg = cmd.c_str() + 5;
    int s1 = 0, ms = 0;
    const int n = sscanf(arg, "%d,%d", &s1, &ms);
    if (n == 1) {
      // Nur Dauer - aktuelle Drehzahl unveraendert lassen.
      ms = s1; s1 = (int)config.spinSpeed;
    } else if (n != 2) {
      sendStatus("SPIN:BAD"); return true;
    }
    if (s1 < SPIN_SPEED_MIN || s1 > SPIN_SPEED_MAX ||
        ms < SPIN_MS_MIN   || ms > SPIN_MS_MAX)               { sendStatus("SPIN:BAD"); return true; }
    config.spinSpeed = (uint16_t)s1;
    config.spinMs    = (uint16_t)ms;
    configSaveUShort("spinSpeed", config.spinSpeed);
    configSaveUShort("spinMs", config.spinMs);
    snprintf(buf, sizeof(buf), "SPIN=%u,%u", (unsigned)config.spinSpeed,
             (unsigned)config.spinMs);
    sendStatus(buf);
    Serial.printf("[SPIN] Einschalt-Drehung: %u Schritte/s fuer %u ms%s\n",
                  (unsigned)config.spinSpeed, (unsigned)config.spinMs,
                  (config.spinSpeed == 0 || config.spinMs == 0) ? " (aus)" : "");
    return true;
  }
  return false;
}

// ---- Farb-Saettigungs-Schwelle (Boden-Differenzierung) ----
static bool cmdSat(const String& cmd) {
  char msg[32];
  if (cmd == "SAT?") {
    snprintf(msg, sizeof(msg), "SAT=%.2f", config.minColorSaturation);
    sendStatus(msg);
    return true;
  }
  if (cmd.startsWith("SAT:")) {
    float v = cmd.substring(4).toFloat();
    if (!inRange(v, 0.0f, 1.0f)) { sendStatus("SAT:BAD"); return true; }
    config.minColorSaturation = v;
    configSaveField("minSat", v);
    snprintf(msg, sizeof(msg), "SAT=%.2f", v);
    sendStatus(msg);
    return true;
  }
  return false;
}

// ---- gemeinsamer Helfer: MAC-String "AA:BB:CC:DD:EE:FF" -> 6 Bytes ----
// Erwartet genau 6 durch ':' getrennte Hex-Bytes. Gibt false zurueck, wenn
// das Format nicht passt (dann bleibt mac unveraendert).
static bool parseMac(const String& macStr, uint8_t mac[6]) {
  memset(mac, 0, 6);
  int idx = 0, pos = 0;
  while (idx < 6 && pos + 1 < macStr.length()) {
    char hex[3] = { macStr.charAt(pos), macStr.charAt(pos + 1), 0 };
    mac[idx++] = (uint8_t)strtol(hex, nullptr, 16);
    pos += 3;   // 2 Hex-Zeichen + Trenner
  }
  return idx == 6;
}

// ---- KORRIGIEREN (Farb-Quiz): QUIZ:CHECK:<ROT,GELB,...> / QUIZ:STOP ----
static bool cmdQuiz(const String& cmd) {
  if (!cmd.startsWith("QUIZ:")) return false;
  String sub = cmd.substring(5);
  if (sub.startsWith("CHECK:")) {
    // Kein neues Quiz waehrend eines laufenden: der Parser loescht
    // korrigierenLen und schreibt korrigierenExpected[] neu, waehrend
    // korrigierenExecute() genau ueber dieses Array iteriert. handleCommand()
    // ist re-entrant, zwei Quizlaeufe wuerden sich also verschränken.
    if (korrigierenRunning) {
      sendStatus("QUIZ:BUSY");
      Serial.println("[QUIZ] Laufend - CHECK abgelehnt");
      return true;
    }
    // Siehe claimGameSlot(): korrigierenExecute() bricht ueber
    // currentGame != GAME_KORRIGIEREN ab und setzt dabei nichts zurueck, was
    // den fremden Spielzustand wieder freigeraeumt haette.
    if (claimGameSlot("QUIZ:BUSY", "QUIZ")) return true;
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
        return true;
      }
    }
    korrigierenExecute();
  } else if (sub == "STOP") {
    currentGame = GAME_NONE;
    korrigierenRunning = false;
    stopMotors();
    sendStatus("QUIZ:STOPPED");
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// ---- RENNEN (Multiplayer): RACE:START[/:<ziel>:<stufe>]/STEP[/:<mm>]/WIN/LOSE/
//      STOP/SCAN/UNPAIR/PAIR:/PROGRESS: ---------------------------------------------------------
// Rueckwaertskompatibilitaet (Partner mit aelterer Firmware bzw. aelterer Web-UI):
//   * RACE:START ohne Parameter  -> Ziel 10 (wie bisher)
//   * RACE:STEP ohne Parameter  -> Zeitbetrieb 360 ms (wie bisher)
// Das ESP-NOW-Paket bleibt unveraendert - die Uebertragung zwischen zwei
// Robotern ist bit-identisch zum bisherigen Protokoll.
static bool cmdRace(const String& cmd) {
  if (!cmd.startsWith("RACE:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub == "START") {
    raceTargetScore = RENNEN_TARGET_SCORE;   // Default, falls keine Parameter
    raceStage = 0;
    raceMMPerPoint = 0.0f;                    // Zeitbetrieb bis zur STEP-Angabe
  } else if (sub.startsWith("START:")) {
    // START:<ziel>[:<stufe>] - Ziel 1..99, Stufe 0..6 (nur Anzeige/Protokoll)
    const String params = sub.substring(6);
    const int b = params.indexOf(':');
    long target = params.toInt();
    long stage = (b >= 0) ? params.substring(b + 1).toInt() : 0;
    if (target < RACE_TARGET_MIN || target > RACE_TARGET_MAX) target = RENNEN_TARGET_SCORE;
    if (stage < 0 || stage > 6) stage = 0;
    raceTargetScore = (uint8_t)target;
    raceStage = (uint8_t)stage;
    raceMMPerPoint = 0.0f;
  }
  if (sub == "START" || sub.startsWith("START:")) {
    // Siehe claimGameSlot(): ohne diese Sperre startete das Rennen mitten in
    // einem anderen Spiel und hinterliess einen Zustand currentGame=GAME_NONE
    // bei raceActive=true. RACE:STEP zaehlte dann Punkte, obwohl die Fahrt
    // sofort wieder abgebrochen wurde - das Kind sah Punkte, der Roboter stand.
    if (claimGameSlot("RACE:BUSY", "RACE")) return true;
    raceExecuteStart();
    raceSendPacket(0, false);
    snprintf(buf, sizeof(buf), "RACE:TARGET:%u,%u", raceTargetScore, raceStage);
    sendStatus(buf);
  } else if (sub == "STEP") {
    if (!raceActive) return true;
    raceMyScore++;
    raceForwardStep();
    if (!raceActive) return true;
    raceSendPacket((uint8_t)raceMyScore, raceMyScore >= raceTargetScore);
    snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
    sendStatus(buf);
    if (raceMyScore >= raceTargetScore) {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      sendStatus("RACE:YOU_WIN");
    }
  } else if (sub.startsWith("STEP:")) {
    // STEP:<mm> - Strecke je Punkt in Millimetern (ersetzt den 360-ms-Betrieb).
    const float mm = sub.substring(5).toFloat();
    if (!inRange(mm, 1.0f, 300.0f)) { sendStatus("RACE:BAD_MM"); return true; }
    raceMMPerPoint = mm;
    if (!raceActive) return true;
    raceMyScore++;
    raceForwardStepMM(mm);
    if (!raceActive) return true;
    raceSendPacket((uint8_t)raceMyScore, raceMyScore >= raceTargetScore);
    snprintf(buf, sizeof(buf), "RACE:ME:%d", raceMyScore);
    sendStatus(buf);
    if (raceMyScore >= raceTargetScore) {
      raceActive = false;
      currentGame = GAME_NONE;
      stopMotors(); speedL = 0; speedR = 0;
      sendStatus("RACE:YOU_WIN");
    }
  } else if (sub == "WIN") {
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    raceSendPacket((uint8_t)raceTargetScore, true);
  } else if (sub == "LOSE") {
    raceActive = false;
    currentGame = GAME_NONE;
    stopMotors(); speedL = 0; speedR = 0;
    raceSendPacket((uint8_t)raceMyScore, false);
  } else if (sub == "STOP") {
    raceAbort(true);
  } else if (sub == "SCAN") {
    raceSendHello();
    sendStatus("RACE:SCAN_START");
  } else if (sub == "UNPAIR") {
    raceUnpair();
    sendStatus("RACE:UNPAIRED");
  } else if (sub.startsWith("PAIR:")) {
    uint8_t mac[6];
    if (parseMac(sub.substring(5), mac)) {
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
    raceLastOppMs = millis();
    snprintf(buf, sizeof(buf), "RACE:PROGRESS:%d", raceOppScore);
    sendStatus(buf);
  } else if (sub == "?") {
    snprintf(buf, sizeof(buf), "RACE:TARGET:%u,%u", raceTargetScore, raceStage);
    sendStatus(buf);
    snprintf(buf, sizeof(buf), "RACE:STATE:%d,%d,%u,%u", raceActive ? 1 : 0,
             currentGame == GAME_RENNEN ? 1 : 0, (unsigned)raceMyScore,
             (unsigned)raceOppScore);
    sendStatus(buf);
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// ---- LINIE FOLGEN: LINE:START / LINE:STOP ----
// Darf der Befehl "<prefix>BUSY" die aktuell laufende Spiel-Sperre beanspruchen?
//
// Hintergrund: handleCommand() ist aus jeder blockierenden Spiel-Warte-
// schleife heraus re-entrant (bleProcessEvents()). Ein zweiter Start-Befehl
// kann also mitten in einem laufenden Spiel ankommen. Ohne diese Pruefung
// passierte Folgendes:
//
//   PROG laeuft (progActive, currentGame=GAME_PROG)
//     -> MEMORY:START kommt rein
//     -> setzt currentGame = GAME_MEMORY, memoryActive = true, hebt eine Fahrt auf
//     -> das laufende progMove() erkennt currentGame != GAME_PROG, bricht ab
//        und setzt am Ende currentGame = GAME_NONE   <-- ueberschreibt das gerade
//                                                          gestartete Memory
//   Endzustand: memoryActive == true, currentGame == GAME_NONE
//     -> Memory ist tot: memoryStart() hat "if (memoryActive) return;"
//     -> das Freifahren ist entsperrt, die WebUI zeigt aber "Memory laeuft"
//     -> Rettung nur ueber MEMORY:STOP
//
// Dieselbe Mechanik galt fuer RACE:START (Punkte wurden gezaehlt, ohne zu
// fahren) und QUIZ:CHECK. Deshalb pruefen jetzt ALLE Startbefehle hier -
// die bisherigen Einzelpruefungen (progActive, korrigierenRunning,
// memoryActive, raceActive) sehen jeweils nur sich selbst, nicht currentGame.
//
// true = gesperrt, der Befehl wurde abgelehnt.
static bool claimGameSlot(const char* busyMsg, const char* what) {
  if (!driveLockedByGame()) return false;
  sendStatus(busyMsg);
  Serial.printf("[%s] Spiel laeuft (game=%d) - Start abgelehnt\n",
                what, (int)currentGame);
  return true;
}

static bool cmdLine(const String& cmd) {
  if (!cmd.startsWith("LINE:")) return false;
  String sub = cmd.substring(5);
  if (sub == "START") {
    // Der Linienfolger darf kein laufendes Spiel uebernehmen. Siehe
    // claimGameSlot() fuer den Grund.
    if (claimGameSlot("LINE:BUSY", "LINE")) return true;
    if (!lineFollowerActive) {
      currentGame = GAME_LINEFOLLOWER;
      lineFollowerStart();
      sendStatus("LINE:RUNNING");
    } else {
      // Bisher stiller No-Op: kein Befehl wurde gesendet, der Knopf
      // schien defekt. Jetzt gibt es eine Rueckmeldung.
      sendStatus("LINE:BUSY");
    }
  } else if (sub == "STOP") {
    lineFollowerStop();
    currentGame = GAME_NONE;
    sendStatus("LINE:STOPPED");
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// ---- MEMORY: MEMORY:START / MEMORY:START:<n> / MEMORY:STOP / MEMORY:INPUT:<n> ----
static bool cmdMemory(const String& cmd) {
  if (!cmd.startsWith("MEMORY:")) return false;
  String sub = cmd.substring(7);
  if (sub == "START") {
    // Siehe claimGameSlot(). Besonders folgenreich war das hier, weil
    // memoryStart() "if (memoryActive) return;" hat und sein eigenes Flag
    // gesetzt laesst: ein fremder Start hinterliess memoryActive=true bei
    // currentGame=GAME_NONE. Memory war danach tot und das Freifahren offen.
    if (claimGameSlot("MEMORY:BUSY", "MEMORY")) return true;
    currentGame = GAME_MEMORY;
    memoryStart(3);
  } else if (sub.startsWith("START:")) {
    if (claimGameSlot("MEMORY:BUSY", "MEMORY")) return true;
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
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// ---- PROGRAMMIEREN (Code-Bausteine): PROG:SET:<enc> / PROG:RUN / PROG:STOP ----
static bool cmdProg(const String& cmd) {
  if (!cmd.startsWith("PROG:")) return false;
  char buf[48];
  String sub = cmd.substring(5);
  if (sub.startsWith("SET:")) {
    // Nicht waehrend eines laufenden Programms: progSetFromString() macht
    // memset(progSteps) und setzt progLen - der laufende Interpreter haelt
    // aber eine Referenz genau auf dieses Array. handleCommand() ist aus
    // jeder Spiel-Warteschleife re-entrant, ein zweites PROG:SET wuerde dem
    // Programm also unter sich selbst die Instruktionen austauschen.
    if (progActive) {
      sendStatus("PROG:BUSY");
      Serial.println("[PROG] Laufend - SET abgelehnt");
      return true;
    }
    String enc = sub.substring(4);
    if (progSetFromString(enc.c_str())) {
      snprintf(buf, sizeof(buf), "PROG:READY:%u", progLen);
      sendStatus(buf);
    } else {
      sendStatus("PROG:BAD");
    }
  } else if (sub == "RUN") {
    // Auch waehrend eines laufenden Slalom-Matches kein Solo-Start. Der
    // Slalom-Executor (slalomExecute) ruft progSetFromString() auf, und das
    // macht memset(progSteps) - faellt ein GO-Paket waehrend eines hier
    // gestarteten Solo-Laufs ein, wuerde das laufende Programm unter sich
    // selbst durch die Schrittliste des Gegners ersetzt.
    if (slalomPhase == SLALOM_BUILD || slalomPhase == SLALOM_COUNTDOWN ||
        slalomPhase == SLALOM_RACING) {
      // Eigener String, weil "PROG:BUSY" sonst als "es laeuft schon ein
      // Programm" gelesen wuerde - es laeuft aber gar kein Programm, sondern
      // ein Slalom-Match. Die Meldung muss zum tatsaechlichen Zustand passen,
      // sonst sucht das Kind vergeblich nach einem Stop-Knopf.
      sendStatus("PROG:BUSY:SLALOM");
      Serial.println("[PROG] Slalom laeuft - Solo-Start abgelehnt");
      return true;
    }
    // Siehe claimGameSlot(): progRun() setzt am Ende currentGame = GAME_NONE.
    // Ein fremder Start, der waehrend des Laufs currentGame belegt haette,
    // wuerde dadurch wieder freigeraeumt, waehrend das neue Spiel weiterlaeuft.
    if (claimGameSlot("PROG:BUSY", "PROG")) return true;
    // Kein Einrasten mehr: es hat 30...200 ms blockiert und konnte im
    // Timeout-Fall den Start verhindern, ohne den Schiefstand zu beheben
    // (motorSettle war winkelneutral - siehe Motor.cpp).
    progRun();
  } else if (sub == "STOP") {
    progStop();
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// ---- SLALOM (Farben-Slalom Multiplayer) ----
// Eigenstaendiges Pairing (getrennt vom Race-Pairing), siehe Slalom.h.
static bool cmdSlalom(const String& cmd) {
  if (!cmd.startsWith("SLALOM:")) return false;
  String sub = cmd.substring(7);
  if (sub.startsWith("PROG:")) {
    String enc = sub.substring(5);
    slalomSetProgram(enc.c_str());
  } else if (sub.startsWith("INVITE:")) {
    // SLALOM:INVITE:<MAC> - Initiator (Host) schickt Einladung an Peer
    if (slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:ALREADY_PAIRED");
      return true;
    }
    uint8_t mac[6];
    if (!parseMac(sub.substring(7), mac)) {
      sendStatus("SLALOM:ERROR:BAD_MAC");
      return true;
    }
    // CMD_INVITE an den Peer senden. slalomInviteSend() merkt sich Ziel-MAC
    // und Zeitpunkt und wiederholt die Einladung ueber slalomTick() - vorher
    // ging genau EIN Paket raus, und bei Paketverlust blieb die Web-UI
    // dauerhaft in "Einladung gesendet! Warte auf Antwort..." stehen.
    // Das Merken ist zugleich die Berechtigung fuer ein spaeteres
    // INVITE_ACK: nur darauf antworten wir.
    slalomInviteSend(mac);
    sendStatus("SLALOM:INVITE_SENT");
    Serial.printf("[SLALOM] Einladung an %02X:%02X:%02X:%02X:%02X:%02X gesendet\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else if (sub == "INVITE_ACCEPT") {
    // Web des Empfaengers hat das Pop-up bestaetigt
    if (!slalomInvitePending()) {
      sendStatus("SLALOM:ERROR:NO_INVITE");
      return true;
    }
    // CMD_INVITE_ACK an den Initiator schicken
    slalomSendPacketTo(SLALOM_CMD_INVITE_ACK, nullptr, slalomInviterMacPtr());
    // Wir sind Client (nicht Initiator)
    slalomPairWith(slalomInviterMacPtr(), false);
    slalomClearInvite();
    sendStatus("SLALOM:INVITE_ACCEPTED");
  } else if (sub == "INVITE_DECLINE") {
    // Einladung abgelehnt
    if (slalomInvitePending()) {
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomInviterMacPtr());
    }
    slalomClearInvite();
  } else if (sub == "UNPAIR") {
    // Slalom-spezifisches Unpair (nur Slalom-State, nicht das Race-Pairing)
    if (slalomHasPeer()) {
      // Dem Peer mitteilen, dass wir uns trennen
      slalomSendPacketTo(SLALOM_CMD_SLALOM_UNPAIR, nullptr, slalomPeerMacPtr());
    }
    slalomUnpair();
  } else if (sub == "START_MATCH") {
    // Web-Befehl (Host): Spiel starten, Zufallsfarbe, Client in Bauphase holen
    if (!slalomHasPeer()) {
      sendStatus("SLALOM:ERROR:NOT_PAIRED");
      return true;
    }
    if (!slalomIsHost) {
      sendStatus("SLALOM:ERROR:NOT_HOST");
      return true;
    }
    // Siehe claimGameSlot(): slalomExecute() setzt am Ende currentGame auf
    // GAME_NONE und wuerde damit ein mitten im Spiel gestartetes anderes
    // Programm wieder entsperren.
    if (claimGameSlot("SLALOM:BUSY", "SLALOM")) return true;
    slalomStartGame();
  } else if (sub == "STOP") {
    // Spiel manuell abbrechen -> beide zurueck zu PAIRED
    slalomAbortGame();
  } else {
    // Unbekannter Sub-Befehl. Muss ein ELSE der Kette sein und darf nicht
    // unbedingt davor stehen: die Erfolgszweige fallen in den Abschluss der
    // Funktion durch. Ein unbedingtes sendStatus() schoebe JEDEM gueltigen
    // Befehl ein "UNKNOWN:..." nach - und da BLE-Notify FIFO ist, wuerde die
    // Web-UI die gerade gezeigte Erfolgsmeldung zuverlaessig mit einer gelben
    // "Unbekannter Befehl"-Warnung ueberschreiben.
    sendStatus((String("UNKNOWN:") + cmd).c_str());
  }
  return true;
}

// Herkunft des Befehls: true = Web-UI ueber BLE, false = Serial-Monitor (USB).
// Die Kalibrierung ist ueber die Web-UI GESPERRT (einmal kalibriert, dann
// passt das) - ueber Serial bleibt sie als Service-Weg offen. Deshalb gibt es
// zwei Einstiege statt einem Parameter durch ein Dutzend Funktionen.
void handleCommand(const char* cmdStr);   // unten definiert
void handleBleCommand(const char* cmdStr) {
  s_cmdFromBle = true;
  handleCommand(cmdStr);
  s_cmdFromBle = false;
}
void handleCommand(const char* cmdStr) {
  String cmd(cmdStr);
  cmd.trim();
  cmd.toUpperCase();
  lastCommandTime = millis();

    if (cmd.startsWith("KEYS:")) { cmdKeys(cmd); return; }
    // DRV? zeigt die komplette Freifahren-Kette (Keys, Geschwindigkeit,
    // Spiel-Sperre, Treiber). Nur Diagnose - veraendert nichts.
    if (cmd == "DRV?") {
      // 224 statt 160: die Zeile enthaelt jetzt auch die Schrittzaehler beider
      // Raeder (pos=/d=) fuer die Drift-Diagnose. BLE verhandelt MTU 247, und
      // sendStatus() schneidet nichts ab - ein zu kleiner Puffer wuerde die
      // Diagnose genau dort kappen, wo sie gebraucht wird.
      char drv[224];
      driveDebug(drv, sizeof(drv));
      sendStatus(drv);
      return;
    }

  Serial.printf("[CMD] %s\n", cmd.c_str());
  if (cmd == "STOP") { hardStop(); return; }

  // Spiel-STOP: Tasten + Geschwindigkeits-Buchfuehrung mitraeumen, damit die
  // Main-Loop nicht sofort wieder anzieht. (Siehe cmdIsGameStop.)
  if (cmdIsGameStop(cmd)) {
    driveClearKeys();
    speedL = 0.0; speedR = 0.0;
  }


  if (cmdSteps(cmd)) return;   // STEPS?/STEPS:<mm>  + GEO? / GEO:RESET
  if (cmdAng(cmd))   return;   // ANG?/ANG:<grad>
  if (cmdSpeed(cmd)) return;   // SPEED?/SPEED:<mm/s>
  if (cmdMotorFeel(cmd)) return;  // RAMP?/RAMP:<ms>  + SPIN?/SPIN:<s>,<ms>
  if (cmdSat(cmd))   return;   // SAT? / SAT:<0..1>         (Mindest-Saettigung)


  // ---- Dispatch: erste passende Gruppe gewinnt ----
  if (cmdDist(cmd))  return;   // DIST / OBSDIST / OBSREL / DISTCAL / I2C / DISTDIAG
  if (cmdCal(cmd))   return;   // CAL:BLACK/WHITE/ROT/GELB/GREEN/BLUE/?
  if (cmdTmc(cmd))   return;   // TMC:* / UARTTEST / UARTNET
  if (cmdQuiz(cmd))  return;   // QUIZ:CHECK / QUIZ:STOP
  if (cmdRace(cmd))  return;   // RACE:START/STEP/WIN/... / PAIR: / PROGRESS:
  if (cmdSlalom(cmd))return;   // SLALOM:PROG/INVITE/START_MATCH/UNPAIR/STOP
  if (cmdLine(cmd))  return;   // LINE:START / LINE:STOP
  if (cmdMemory(cmd))return;   // MEMORY:START / STOP / INPUT:
  if (cmdProg(cmd))  return;   // PROG:SET / RUN / STOP
  sendStatus((String("UNKNOWN:") + cmd).c_str());
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(BAUD_RATE);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n======================================");
  Serial.println("EduBot V2 - Modular (2x TMC2209 + VL53L0X)");
  Serial.println("======================================");

  esp_task_wdt_init(10000, true);
  esp_task_wdt_add(NULL);

configLoad();

// Tischkanten-Waechter: immer an, ohne Schalter und ohne NVS. Muss VOR dem
  // Start des Farbsensor-Tasks passieren: edgeGuardTick() wird von dort
  // aufgerufen und wuerde sonst die ersten Messungen ignorieren - beim
  // Anfahren direkt nach dem Einschalten waere die Kante genau dann
  // ungeschuetzt, wenn es gefaehrlich wird.
edgeGuardSet(true);

initSteppers();
  Serial.println("[MOTOR] Motor-Task auf Core 0 gestartet (feste Geschwindigkeit, sanftes Anfahren)");
  startupSpin();
  tmcBootDiag();

  lastCommandTime = millis();

  // VL53L0X VOR dem Farbsensor initialisieren: der Treiber liest beim
  // Start seine NVM-Kalibrierdaten, und das muss auf einer Adresse passieren,
  // an der nur er selbst antwortet (siehe DistanceSensor.h).
  //
  // Die Reihenfolge allein reicht nicht - haengt der TCS34725 mit am Bus,
  // verfaelscht er die NVM-Reads des VL53L0X (Folge: -50 REF_SPAD_INIT).
  // Geloest ueber den Library-Patch (patch_vl53l0x.py): begin(0x30) zieht
  // den Sensor VOR DataInit auf 0x30, sodass alle NVM-Zugriffe dort laufen
  // und der Farbsensor auf 0x29 bleiben kann. Beides gleichzeitig verifiziert:
  // Continuous-Modus, 46/46 gueltige Messungen, 191-198 mm bei 20 cm Ziel.
  distSetup();
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
// LED-STATUSANZEIGE
// =====================================================
// Waehlt genau EIN Muster nach Prioritaet (oben gewinnt):
//   1. Slalom-Countdown  - Startampel: rot fuellt sich, gruen = GO
//   2. Slalom-Ergebnis   - Regenbogen (gewonnen) / rotes Atmen (verloren)
//   3. Slalom-Bauphase   - Zielfarbe pulsiert
//   4. Memory            - Show-Lauflicht / gruener Fortschrittsbalken
//   5. Linienfolger      - gruener Scanner / Such- und Hindernis-Blinken
//   6. Rennen            - gelbes Lauflicht
//   7. Freifahren        - Lauflicht in Fahrtrichtung
//   8. Ruhezustand       - blau atmen (App verbunden) / orange atmen (wartet)
// Alarm (Not-Aus) und Ergebnisblitze uebersteuern die Kette intern in LedStrip.
static void ledPatternUpdate() {
  if (slalomPhase == SLALOM_COUNTDOWN) {
    const uint32_t now = millis();
    const uint32_t left = (slalomGoTimer > now) ? (slalomGoTimer - now) : 0;
    // Host zaehlt 3 s bis GO, Client-Fallback 4,5 s (siehe slalomMaybeStart).
    ledStripShowCountdown(left, slalomIsHost ? 3000 : 4500);
    return;
  }
  if (slalomPhase == SLALOM_DONE) {
    ledStripShowResult(slalomIWonLast);
    return;
  }
  if (slalomTargetColor[0] != '\0') {   // Bauphase (auch nach MISS-Retry)
    ledStripShowTarget(slalomTargetColor);
    return;
  }
  if (slalomPhase == SLALOM_PAIRED) {   // Gegner verbunden, wartet auf Start
    ledStripShowPaired();
    return;
  }
  if (memoryActive) {
    // Show-Phase: Roboter zeigt die Sequenz. Eingabe: wie weit ist das Kind?
    if (memoryAwaitInput) ledStripShowProgress(memoryInputIdx, memoryLen, LED_C_GREEN);
    else                  ledStripShowMemoryShow();
    return;
  }
  if (currentGame == GAME_LINEFOLLOWER) {
    ledStripShowLine(lineFollowerLedState());
    return;
  }
  if (currentGame == GAME_RENNEN) {
    ledStripShowRace();
    return;
  }
  if (speedL != 0.0f || speedR != 0.0f) {
    // Freifahren: Lauflicht in Fahrtrichtung, Drehen = zwei Punkte nach aussen.
    uint8_t dir = LED_DIR_IDLE;
    if (keyW && !keyS)      dir = LED_DIR_FWD;
    else if (keyS && !keyW) dir = LED_DIR_BACK;
    ledStripShowDrive(dir);
    return;
  }
  ledStripShowStatus(deviceConnected);
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastColorNotify = 0;
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
  // (2b) Einladungs-Wartezustand: Retry der Einladung bzw. Timeout.
  slalomTick();
  if (!driveLockedByGame()) driveUpdate();
  if (deviceConnected && millis() - lastColorNotify > COLOR_NOTIFY_MS) {
    lastColorNotify = millis();
    sendColorToWebsite();
  }
  // VL53L0X-Distanz: alle DIST_NOTIFY_MS als DIST:<mm> (nur bei BLE, nur wenn AUTO an)
  // Hindernis-Status fuer ProgGame/Slalom: distBlocked()
  distTick();
  // Ergebnis einer asynchronen Ref-SPAD-Kalibrierung (DISTREF) auswerten.
  // Muss unbedingt im Loop bleiben: der Kalibrierungs-Task sperrt den
  // I2C-Bus, und distTick() findet bis zum Abschluss keine Messung.
  distRefSpadService();
  // TMC-Auto-Standby: Treiber aus, wenn laenger als TMC_STANDBY_MS nichts faehrt
  tmcAutoStandbyTick();
  // Treiber-Messung fortschreiben: liest bewusst nur EIN Register pro
  // Durchlauf. Muss hier stehen bleiben - die Messung braucht viele
  // Durchlaeufe, und ein blockierender Aufruf wuerde die Fahrt stoeren.
  motorDiagTick();
  // Rennen: hat der Gegner zu lange geschwiegen, sauber beenden. Sonst bleibt
  // der Roboter bis zum naechsten STOP im Renn-Modus und die Steuerung gesperrt.
  if (racePeerTimedOut()) {
    Serial.println("[RACE] Gegner timeout -> Rennen beendet");
    raceAbort(false);
  }
  // LED-Streifen: Musterwahl 20x/s (Prioritaetenkette), Zeichnen in jedem
  // Durchlauf. Alarm/Ergebnisblitze uebersteuern die Kette intern.
  {
    static unsigned long lastLedMode = 0;
    if (millis() - lastLedMode >= 50) {
      lastLedMode = millis();
      ledPatternUpdate();
    }
    ledStripTick();
  }
  vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
}
