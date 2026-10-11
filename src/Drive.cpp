// EduBot V2 - Drive-Implementierung (WASD / Touch)
// Zielgeschwindigkeit aus config.driveSpeedMM (per SPEED:<mm/s> einstellbar).
#include "Drive.h"
#include "Motor.h"
#include "Config.h"
#include "GameEngine.h"  // driveLockedByGame()

bool keyW = false;
bool keyA = false;
bool keyS = false;
bool keyD = false;

float speedL = 0.0;
float speedR = 0.0;

unsigned long lastCommandTime = 0;

void driveClearKeys() {
  keyW = false; keyA = false; keyS = false; keyD = false;
}

void driveHardStop() {
  driveClearKeys();
  speedL = 0.0;
  speedR = 0.0;
  stopMotors();
}

void driveUpdate() {
  // Waehrend ein Spiel aktiv ist, wird die Bewegung von der Spiel-Engine
  // blockierend gesteuert - KEYS: vom Frontend werden ignoriert.
  if (driveLockedByGame()) return;

  // Ist der Roboter von "faehrt" auf "steht" uebergegangen? Dann FAHRT-BERICHT
  // ausgeben: einer pro Manoever, ohne dass der User irgendetwas ablesen muss.
  //
  // Warum: driftet der Roboter beim Rueckwaertsfahren seitlich, ist die erste
  // Frage immer "Software oder Hardware?". Die Schrittzaehler beider Stepper
  // beantworten sie: sie zaehlen JEDEN erzeugten Puls, unabhaengig davon, ob
  // das Rad rollt. Gleiche Zaehler bei sichtbarem Drift = mechanisch (Winkel,
  // Reibung im Getriebe, Spiel). Unterschiedliche Zaehler = die Software fährt
  // die Raeder ungleich an. Diese Unterscheidung spart jede Vermutung.
  static bool     s_wasMoving = false;
  static long     s_startL = 0, s_startR = 0;
  const bool      wantMove = (keyW || keyS || keyA || keyD);
  if (wantMove && !s_wasMoving) {
    motorStepPositions(s_startL, s_startR);   // Startmarke fuer diese Fahrt
  } else if (!wantMove && s_wasMoving) {
    long endL = 0, endR = 0;
    motorStepPositions(endL, endR);
    const long dL = endL - s_startL, dR = endR - s_startR;
    // ACHTUNG: die beiden Zaehler sind ABSICHTLICH gegenlaeufig - die Raeder
    // drehen gegeneinander, damit der Roboter geradeaus faehrt. Vergleichen
    // muss man deshalb die BETRAEGE. Die rohe Differenz ist per Bauweise
    // etwa doppelt so gross wie die Fahrstrecke und sieht sonst immer wie ein
    // Fehler aus.
    const long aL = labs(dL), aR = labs(dR);
    // Toleranz relativ, nicht absolut: 10 Schritte Unterschied sind bei 10000
    // Schritten 0,1 % und voellig normal (Rundung im Zeitraster), bei 300
    // Schritten waeren dieselben 10 Schritte ein echter Unterschied. Eine
    // feste Grenze hat die Diagnose entweder zu empfindlich oder zu
    // unempfindlich gemacht, je nachdem, wie lang gefahren wurde.
    const long mx = aL > aR ? aL : aR;
    const int diffPromille = (mx > 0) ? (int)(labs(aL - aR) * 1000L / mx) : 0;
    const long diff = labs(aL - aR);
    const float spmm = configStepsPerMM();
    const float mmL = (spmm > 0.0f) ? (float)aL / spmm : 0.0f;
    const float mmR = (spmm > 0.0f) ? (float)aR / spmm : 0.0f;
    Serial.printf("[DRV] Fahrt beendet: Schritte L=%ld R=%ld (Diff %ld) = %.1f / %.1f mm\n",
                  aL, aR, diff, (double)mmL, (double)mmR);
    Serial.printf("[DRV]   %s | Unterschied %ld Schritte = %.2f mm (%.1f Promille)\n",
                  diffPromille <= 10 ? "Zaehler praktisch gleich -> GLEICHE Pulse an beide Raeder"
                                     : "Zaehler unterscheiden sich -> Software faehrt die Raeder ungleich an",
                  diff, (spmm > 0.0f) ? (float)diff / spmm : 0.0f, (double)diffPromille);
  }
  s_wasMoving = wantMove;

  // Geschwindigkeit aus config.driveSpeedMM (per SPEED: einstellbar).
  // Drehen auf der Stelle: ein Motor still, der andere mit voller Geschwindigkeit.
  float targetL = 0.0;
  float targetR = 0.0;

  bool forward  = keyW && !keyS;
  bool backward = keyS && !keyW;
  bool left     = keyA && !keyD;
  bool right    = keyD && !keyA;

  if (forward)  { targetL = FIXED_DRIVE_SPEED; targetR = FIXED_DRIVE_SPEED; }
  else if (backward) { targetL = -FIXED_DRIVE_SPEED; targetR = -FIXED_DRIVE_SPEED; }
  if (!forward && !backward) {
    if (left)  { targetL = 0; targetR = FIXED_DRIVE_SPEED; }
    else if (right) { targetL = FIXED_DRIVE_SPEED; targetR = 0; }
  }

  // KEIN Hindernisschutz im Freifahren - bewusst so.
  //
  // Es gab hier kurz eine Sperre ("if (distBlocked()) { targetL = targetR = 0; }")
  // samt Web-UI-Schalter. Beides ist wieder entfernt. Grund: der VL53L0X sitzt
  // niedrig und zeigt beim Fahren oft auf Tischkante, Wand, Schrankkante oder
  // den Boden. Unter der Stoppschwelle galt dann dauerhaft "Hindernis", die
  // W/A/S/D-Tasten hatten keinerlei Wirkung - ohne jede Meldung im
  // Fahr-Bildschirm. Ueber OBSDIST:/OBSREL: war das nicht abzuschalten, deren
  // Untergrenze liegt bei 50 mm. Fuer ein Kind, das frei fahren will, ist
  // "faehrt gar nicht" das schlechteste Verhalten.
  //
  // Der Distanzsensor wird weiterhin genutzt, aber nur dort, wo es wirklich
  // sinnvoll ist: im Linienfolger (LineFollower.cpp) und in den
  // Programm-Bausteinen "fahre bis ..." / "warte nah" / "warte frei"
  // (ProgGame.cpp). Dort ist die Nutzung gewollt und sichtbar.

  // Timeout: kein BLE-Befehl mehr -> HARTER Stopp.
  // WICHTIG: hier nur die Geschwindigkeit auf 0 setzen, die Tasten NICHT
  // loeschen. BLE verliert gelegentlich ein Paket; wuerde man dabei auch die
  // Keys loeschen, stuende der Roboter bei jedem einzelnen Aussetzer mitten
  // im Fahren. Das naechste Heartbeat-Paket (alle 50 ms) stellt den Zustand
  // ohnehin sofort wieder her. Beim echten Verbindungsverlust raeumt
  // onBleDisconnected() die Tasten auf.
  //
  // bHardStop merkt sich diesen Fall: hier wird ausdruecklich NICHT weich
  // gebremst. Eine Bremse, die config.rampMs lang dauert, wuerde den Roboter
  // bei einer 2000er-Anfahrzeit noch zwei Sekunden rollen lassen - genau
  // dann, wenn niemand mehr da ist, um ihn anzuhalten.
  const bool bHardStop = (millis() - lastCommandTime > config.commandTimeoutMs);
  if (bHardStop) {
    targetL = 0.0; targetR = 0.0;
    if (speedL != 0.0 || speedR != 0.0) {
      speedL = 0.0;
      speedR = 0.0;
    }
  }

  // Feste Geschwindigkeit setzen - Motor.cpp nutzt immer FIXED_DRIVE_SPEED.
  // useRamp = !bHardStop: also weiches Ausrollen, wenn eine Taste losgelassen
  // wird (speed == 0, Rampe aktiv), und harter Stopp im Timeout-Fall.
  //
  // EIN Aufruf fuer beide Raeder, nicht zwei: sonst koennte der Motor-Task
  // den Mutex zwischen den beiden Aufrufen bekommen und der zweite Befehl
  // verzoegert ankommen - der Roboter faehrt dann kurz einseitig an. Genau
  // das war vorher moeglich, weil driveUpdate() in jedem Loop-Durchlauf läuft.
  setMotorSpeeds(targetL, targetR, !bHardStop);
  speedL = targetL;
  speedR = targetR;
}

// Diagnose: zeigt fuer jeden Abschnitt der Kette, was tatsaechlich ankommt.
void driveDebug(char* out, size_t n) {
  const bool locked = driveLockedByGame();
  // Was der naechste Tick aus den Keys rechnen wuerde:
  const bool forward  = keyW && !keyS;
  const bool backward = keyS && !keyW;
  const bool left     = keyA && !keyD;
  const bool right    = keyD && !keyA;
  char want[8] = "----";
  if      (forward)  { want[0] = 'F'; want[1] = 'F'; }
  else if (backward) { want[0] = 'B'; want[1] = 'B'; }
  else if (left)     { want[0] = 'L'; want[1] = 'R'; }
  else if (right)    { want[0] = 'R'; want[1] = 'L'; }
  const unsigned long age = millis() - lastCommandTime;
  // lst=/lst= sind die TATSAECHLICHEN Geschwindigkeiten der Freifahr-Rampe
  // (Schritte/s). spd= zeigt nur, was befohlen wurde - bei der Rampe sind die
  // beiden nicht dasselbe: beim Anfahren sind sie kurzzeitig verschieden, und
  // genau so laesst sich ein schiefer Start von einem schiefen Aufbau
  // unterscheiden.
  float curL = 0.0f, curR = 0.0f;
  motorRampSpeeds(curL, curR);
  // posL=/posR= sind die Schrittzaehler der beiden Stepper, d= die Differenz.
  // Das ist die entscheidende Groesse fuer "Drift: Software oder Hardware?":
  // der Zaehler zaehlt JEDEN erzeugten Puls, unabhaengig davon, ob das Rad
  // rollt. Gleiche Zaehler + Drift = mechanisch (Winkel, Reibung, Spiel).
  // Unterschiedliche Zaehler = die Software faehrt die Raeder ungleich an.
  long posL = 0, posR = 0;
  motorStepPositions(posL, posR);
  snprintf(out, n,
    "DRV:keys=%d%d%d%d want=%s spd=%.0f/%.0f fix=%.0f lst=%.0f/%.0f "
    "pos=%ld/%ld d=%ld lock=%d game=%d cmdAlter=%lums timeout=%lums laeuft=%d "
    "sign=%d/%d en=%d",
    keyW, keyA, keyS, keyD, want,
    (double)speedL, (double)speedR, (double)FIXED_DRIVE_SPEED,
    (double)curL, (double)curR,
    posL, posR, posL - posR,
    locked ? 1 : 0, (int)currentGame, age,
    (unsigned long)config.commandTimeoutMs,
    motorStillRunning() ? 1 : 0,
    (int)config.leftSign, (int)config.rightSign,
    tmcIsEnabled() ? 1 : 0);
}
