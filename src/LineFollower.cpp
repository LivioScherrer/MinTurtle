// EduBot V2 - Linienfolge-Implementierung
// Folgt einer farbigen Linie (alles ausser WEISS) auf hellem Untergrund mit dem TCS34725.
// Da nur ein einzelner Farbsensor vorhanden ist, wird eine Such-Strategie
// verwendet: fahre geradeaus solange Linie erkannt wird; beim Verlust
// der Linie: kurzes Stoppen, dann abwechselnd rechts/links Such-Drehungen.
// Laeuft nicht-blockierend ueber lineFollowerTick() in der Main-Loop.
// Feste Geschwindigkeit OHNE Rampe (die Suchdrehungen schalten alle 140 ms um -
// eine Rampe wuerde sie verschlucken): die Pulse erzeugt der Motor-Task
// auf Core 0 selbststaendig, hier werden nur Ziel-Geschwindigkeiten gesetzt.
#include "LineFollower.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "Config.h"
#include "Drive.h"
#include "BLEManager.h"
#include "DistanceSensor.h"  // distBlocked() - Hindernis-Schutz

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
  // Zustand, in den nach einem Hindernis-Schturz zurueckgekehrt wird.
  // Siehe lineFollowerTick() - ohne das verliert der Roboter nach jedem
  // Hindernis sein Such-Budget und kann endlos drehen.
  bool    lsStoppedForObstacle = false;
  LineState lsResumeState = LS_FORWARD;

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

  // Setzt nur die Ziel-Geschwindigkeiten. Die Pulse macht der Motor-Task.
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

// Zustand fuer die LED-Statusanzeige: Linie erkannt, Suche, Hindernis oder
// inaktiv. Wird von der Prioritaetenkette in Edu_V2.cpp gelesen.
// Bewusst der Zustands-Automat (nicht das rohe lineDetected): kurze
// Sensor-Aussetzer in der Grace-Phase sollen die Anzeige nicht flackern lassen.
LedLineState lineFollowerLedState() {
  if (!lineFollowerActive) return LED_LINE_IDLE;
  switch (lsState) {
    case LS_IDLE:    return LED_LINE_OBSTACLE;   // wartet auf freie Bahn
    case LS_FORWARD: return LED_LINE_FOLLOW;     // auch kurze Aussetzer (Grace)
    default:         return LED_LINE_SEARCH;     // LS_LOST / LS_SEARCH_L/R
  }
}

void lineFollowerStart() {
  if (lineFollowerActive) return;
  lineFollowerActive = true;
  lsSearchTries = 0;
  lsStoppedForObstacle = false;   // frischer Lauf: kein offener Hindernis-Zustand
  lsLastSeenMs = millis();
  // ERST messen, DANN fahren. Vorher wurde hier blind mit voller
  // Geschwindigkeit gestartet: readLineDetected() liefert erst nach der
  // ersten Farb-Task-Ablesung (~150 ms) ein Ergebnis, LS_FORWARD lueft die
  // 80-ms-Grace also sofort ab und der Roboter fuhr die ersten Zentimeter
  // ohne zu wissen, ob ueberhaupt eine Linie da ist. Jetzt wird der erste
  // Zustand aus der Messung abgeleitet: Linie da -> fahren, keine Linie ->
  // direkt in die Suchphase statt blind loszurollen.
  lineDetected = readLineDetected();
  if (lineDetected) {
    motorsForward(LINE_FOLLOW_SPEED);
    changeState(LS_FORWARD);
  } else {
    motorsOff();
    changeState(LS_LOST);
  }
  Serial.println("[LINE] Folge starten");
}

void lineFollowerStop() {
  if (!lineFollowerActive) return;
  // Motoren stoppen, kurz warten bis der Motor-Task alles verarbeitet (max 800ms)
  motorsOff();
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 800) {
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  lineFollowerActive = false;
  lsState = LS_IDLE;
  lsSearchTries = 0;
  lsStoppedForObstacle = false;
  Serial.println("[LINE] Gestoppt");
}

void lineFollowerTick() {
  if (!lineFollowerActive) return;

  // Sensordaten aktualisieren (eine Ablesung pro Tick)
  lineDetected = readLineDetected();
  uint32_t now = millis();

  // HINDERNIS-SCHUTZ: hat der Distanzsensor ein Hindernis gemeldet, wird
  // angehalten und in LS_IDLE gewartet, bis wieder frei ist. Bewusst VOR dem
  // Zustandsautomaten: eine Such-Drehung in ein Hindernis hinein waere genau
  // das, was der Sensor verhindern soll. distBlocked() liefert bei fehlender
  // Messung "frei" - ohne Sensor verhaelt sich der Linienfolger also exakt
  // wie vorher.
  if (distBlocked()) {
    motorsOff();
    if (lsState != LS_IDLE) {
      // Merken, in welchem Zustand wir unterbrochen wurden, um dort
      // zurueckzukehren - und dass es ein Hindernis war.
      lsStoppedForObstacle = true;
      lsResumeState = lsState;
    }
    changeState(LS_IDLE);
    return;
  }
  // Hindernis ist frei: aus dem Wartezustand wieder aufnehmen.
  if (lsState == LS_IDLE) {
    if (lsStoppedForObstacle) {
      lsStoppedForObstacle = false;
      // Nach einem Hindernis nicht in LS_FORWARD einsteigen: dort waere
      // lsLastSeenMs so alt, dass der Roboter sofort wieder "Linie verloren"
      // erklaert, die Such-Drehungen starten und lsSearchTries auf 0
      // gesetzt wird. Das 8-Versuche-Budget (4 links + 4 rechts) faengt dann
      // nach JEDEM Hindernis neu an - steht das Hindernis auf der Linie, dreht
      // der Roboter endlos, statt mit LINE:LOST aufzugeben. Stattdessen den
      // Zustand fortsetzen, in dem wir unterbrochen wurden: das Budget bleibt
      // erhalten, der naechste Tick laeuft ganz normal weiter.
      changeState(lsResumeState);
    } else {
      changeState(LS_FORWARD);
    }
  }

  if (lineDetected) {
    lsLastSeenMs = now;
    lsSearchTries = 0;
  }

  // Zustands-Automat -> setzt die Ziel-Geschwindigkeiten
  switch (lsState) {
    case LS_IDLE:
      // Wartezustand "Hindernis blockiert" - Motoren bleiben aus, bis
      // distBlocked() wieder false meldet (siehe-oben).
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
          lsStoppedForObstacle = false;   // Lauf beendet, nichts offen
          Serial.println("[LINE] Linie verloren - Stopp");
        } else {
          motorsTurnRight(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_R);
        }
      }
      break;
    }
  }
}
