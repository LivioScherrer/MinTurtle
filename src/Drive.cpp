// EduBot V2 - Drive-Implementierung (WASD / Touch)
// Feste Geschwindigkeit aus Motor.h - keine Einstellung moeglich.
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

  // Feste Geschwindigkeit - keine Einstellung irgendwo.
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
  setMotorSpeed(0, targetL, !bHardStop);
  setMotorSpeed(1, targetR, !bHardStop);
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
  snprintf(out, n,
    "DRV:keys=%d%d%d%d want=%s spd=%.0f/%.0f fix=%.0f lock=%d game=%d "
    "cmdAlter=%lums timeout=%ums laeuft=%d sign=%d/%d en=%d",
    keyW, keyA, keyS, keyD, want,
    (double)speedL, (double)speedR, (double)FIXED_DRIVE_SPEED,
    locked ? 1 : 0, (int)currentGame, age,
    (unsigned long)config.commandTimeoutMs,
    motorStillRunning() ? 1 : 0,
    (int)config.leftSign, (int)config.rightSign,
    tmcIsEnabled() ? 1 : 0);
}
