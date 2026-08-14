// EduBot V1.3 - Linienfolge-Implementierung
// Folgt einer farbigen Linie (alles ausser WEISS) auf hellem Untergrund mit dem TCS34725.
// Da nur ein einzelner Farbsensor vorhanden ist, wird eine Such-Strategie
// verwendet: fahre geradeaus solange Linie erkannt wird; beim Verlust
// der Linie: kurzes Stoppen, dann abwechselnd rechts/links Such-Drehungen.
// Laeuft nicht-blockierend ueber lineFollowerTick() in der Main-Loop.
// Beschleunigung/Bremsen uebernimmt AccelStepper (GLOBAL_ACCEL_RATE) ->
// kein Wheelie, kein Schraeglauf, beide Stepper bleiben synchron.
#include "LineFollower.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "Config.h"
#include "Drive.h"
#include "BLEManager.h"

bool lineFollowerActive = false;

namespace {
  // Zustands-Automat
  enum LineState : uint8_t {
    LS_IDLE = 0,   // nicht aktiv
    LS_FORWARD,   // Linie erkannt -> geradeaus
    LS_LOST,       // Linie verloren -> Such-Phase
    LS_SEARCH_L,   // Suche links (Drehung)
    LS_SEARCH_R    // Suche rechts (Drehung)
  };
  LineState lsState = LS_IDLE;

  // Timing
  uint32_t lsStateEnterMs = 0;  // wann der aktuelle Zustand begann
  uint32_t lsLastSeenMs = 0;    // wann Linie zuletzt erkannt wurde

  // Konfiguration
  // Geschwindigkeit beim Folgen (geradeaus)
  const float LINE_FOLLOW_SPEED = 380.0f;
  // Geschwindigkeit beim Suchen (Drehen)
  const float LINE_SEARCH_SPEED = 320.0f;
  // Wie lange ohne Linie geradeaus weiterfahren, bevor Suche beginnt
  const uint32_t LINE_GRACE_MS = 80;
  // Such-Drehungen: Dauer und maximale Versuche pro Seite
  const uint32_t LINE_SEARCH_TURN_MS = 140;
  const uint8_t  LINE_SEARCH_MAX_TRIES = 4;   // pro Seite
  uint8_t lsSearchTries = 0;                  // Zaehler der Such-Versuche

  // Gepufferte Sensorablesung (nicht blockierend)
  bool lineDetected = false;

  void changeState(LineState next) {
    lsState = next;
    lsStateEnterMs = millis();
  }

  bool readLineDetected() {
    // Linie = jede Farbe ausser WEISS (und UNBEKANNT)
    char nm[16] = "UNBEKANNT";
    getCurrentColorName(nm, sizeof(nm));
    if (strcasecmp(nm, "WEISS") == 0)    return false;
    if (strcasecmp(nm, "UNBEKANNT") == 0) return false;
    return true;
  }

  // Setzt nur die Ziel-Geschwindigkeiten. AccelStepper macht die Rampe intern.
  void motorsForward(float v) {
    setMotorSpeed(0, v);
    setMotorSpeed(1, v);
  }
  void motorsTurnLeft(float v) {
    setMotorSpeed(0, -v);
    setMotorSpeed(1,  v);
  }
  void motorsTurnRight(float v) {
    setMotorSpeed(0,  v);
    setMotorSpeed(1, -v);
  }
  void motorsOff() {
    setMotorSpeed(0, 0);
    setMotorSpeed(1, 0);
  }
}

void lineFollowerStart() {
  if (lineFollowerActive) return;
  lineFollowerActive = true;
  lsSearchTries = 0;
  lsLastSeenMs = millis();
  // Feste Beschleunigung (keine Einstellung irgendwo)
  motorsForward(LINE_FOLLOW_SPEED);
  changeState(LS_FORWARD);
  Serial.println("[LINE] Folge starten");
}

void lineFollowerStop() {
  if (!lineFollowerActive) return;
  // Sanft herunter rampen (AccelStepper), blockierend max 800ms
  motorsOff();
  uint32_t start = millis();
  while ((stepperL.isRunning() || stepperR.isRunning()) && millis() - start < 800) {
    motorRun();
    bleProcessEvents();
    delay(10);
  }
  lineFollowerActive = false;
  lsState = LS_IDLE;
  lsSearchTries = 0;
  Serial.println("[LINE] Gestoppt");
}

void lineFollowerTick() {
  if (!lineFollowerActive) return;

  // Sensordaten aktualisieren (eine Ablesung pro Tick)
  lineDetected = readLineDetected();
  uint32_t now = millis();

  if (lineDetected) {
    lsLastSeenMs = now;
    lsSearchTries = 0;
  }

  // Zustands-Automat -> setzt die Ziel-Geschwindigkeiten
  switch (lsState) {
    case LS_IDLE:
      motorsOff();
      break;

    case LS_FORWARD: {
      if (lineDetected) {
        // Linie da -> vorwaerts (idempotent: setSpeed auf gleichen Wert ist OK)
        motorsForward(LINE_FOLLOW_SPEED);
      } else {
        // Linie verloren -> kurz geradeaus (Grace), dann Suche beginnen
        if (now - lsLastSeenMs > LINE_GRACE_MS) {
          motorsOff();
          lsSearchTries = 0;
          changeState(LS_LOST);
        }
      }
      break;
    }

    case LS_LOST: {
      // Kurz stoppen, dann abwechselnd links/rechts suchen
      if (now - lsStateEnterMs > 60) {
        lsSearchTries = 0;
        motorsTurnLeft(LINE_SEARCH_SPEED);
        changeState(LS_SEARCH_L);
      }
      break;
    }

    case LS_SEARCH_L: {
      if (lineDetected) {
        motorsForward(LINE_FOLLOW_SPEED);
        changeState(LS_FORWARD);
      } else if (now - lsStateEnterMs > LINE_SEARCH_TURN_MS) {
        lsSearchTries++;
        if (lsSearchTries >= LINE_SEARCH_MAX_TRIES) {
          // Wechsel zur rechten Suche: lsSearchTries NICHT zuruecksetzen,
          // damit R weitere MAX_TRIES Versuche erhaelt (symmetrisch zu L).
          motorsTurnRight(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_R);
        } else {
          motorsTurnLeft(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_L);
        }
      }
      break;
    }

    case LS_SEARCH_R: {
      if (lineDetected) {
        motorsForward(LINE_FOLLOW_SPEED);
        changeState(LS_FORWARD);
      } else if (now - lsStateEnterMs > LINE_SEARCH_TURN_MS) {
        lsSearchTries++;
        // 4 Versuche links + 4 Versuche rechts = 8 insgesamt (symmetrisch)
        if (lsSearchTries >= LINE_SEARCH_MAX_TRIES * 2) {
          motorsOff();
          lineFollowerActive = false;
          lsState = LS_IDLE;
          Serial.println("[LINE] Linie verloren - Stopp");
        } else {
          motorsTurnRight(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_R);
        }
      }
      break;
    }
  }

  // Stepper ausfuehren (ein Schritt wenn faellig, mit Rampe)
  motorRun();
}
