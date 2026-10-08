// EduBot V2 - Motor-Implementierung (2x TMC2209 via UART + STEP/DIR)
//
// Architektur: Die zeitkritische Puls-Erzeugung laeuft in einem eigenen
// FreeRTOS-Task auf Core 0. Ein 50us-Hardware-Timer (esp_timer) schickt per
// Task-Notification einen Tick an den Task; der Task prueft dort (ueber
// AccelStepper.runSpeed() mit micros()-Deadline-Check) ob ein Puls faellig ist.
// Dadurch:
//  - Pulse sind unabhaengig von BLE/ESP-NOW/Serial in der Main-Loop (kein Ruckeln)
//  - praktisch keine CPU-Last (Task blockiert zwischen Ticks, kein busy-wait,
//    ColorSensor-Task und Idle bleiben unberuehrt)
//  - ~20 kHz Abtastung >> benoetigte 5 kHz -> Jitter im Bereich von Mikrosekunden
//
// Beschleunigung (config.rampMs): an ZWEI Stellen, mit derselben Bedeutung -
// "wie lange von 0 bis zum vollen Tempo".
//   * Freies Fahren (WASD): Software-Rampe im Task. AccelStepper kann eine
//     Rampe nur MIT einem Ziel (computeNewSpeed() braucht distanceToGo), beim
//     Freifahren gibt es keins - also wandert die Ist-Geschwindigkeit s_cur
//     pro Tick um s_accel*dt Richtung s_tgt.
//   * Positions-Moves: AccelStepper-Rampe (run()/setAcceleration), die
//     zusaetzlich vor dem Ziel abbremst.
// Bei config.rampMs == 0 ist beides aus (sofort volle Geschwindigkeit).
// Harter Stopp (useRamp=false): Not-Aus, STOP-Knopf, Befehls-Timeout,
// TMC:EN:0, Linienfolger, zeitlich begrenzte Fahrten.
// Der eigene Rest-Schritt-Zaehler gilt nur fuer Positions-Moves; er zaehlt die
// echten Pulse (Positionsaenderung) mit, erkennt das Ende und ist gleichzeitig
// die Anzeige fuer motorStillRunning().
#include "Motor.h"
#include "Config.h"
#include "BLEManager.h"
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

namespace {
  // KEIN Einrasten mehr. Es gab einmal motorSettle(): zwei Vollschritte hin
  // UND exakt zwei zurueck vor dem Programmstart, "damit die Raeder auf
  // derselben Microstep-Index stehen".
  //
  // Das ist winkelneutral - am Ende steht jeder Rotor genau wieder auf der
  // Index, auf der er vorher stand, nur eben um den Schiefstand gegen das
  // jeweils andere Rad versetzt. Der gewuenschte Effekt tritt genau dann NICHT
  // ein, wenn er gebraucht wird, und kostet dafuer 30...200 ms blockiert bei
  // jedem Start (mit dem Risiko eines Timeouts, in dem gar nichts mehr faehrt).
  //
  // Hintergrund, warum es den Schiefstand nicht geben kann, ihn aber auch nicht
  // wegbekommt: bei einem direkt gekoppelten Stepper (Motor -> Rad ohne
  // Getriebe) haengt die Fahrweg-Richtung an der Rad-/Achs-Montage am Chassis,
  // nicht an der Rotorposition. Beide Raeder rollen exakt gleich weit, der
  // Fahrweg ist eine exakt gerade Linie - sie liegt nur um h schraeg zur
  // Roboterachse. Rotor-Schlupf verschiebt die Speichen, nicht die Radebene.
  // "Die Raeder stehen anders" ist eine Baugroesse, keine Firmware-Groesse.

  // Stepper-Treiber - bewusst NUR hier sichtbar: Ausserhalb dieses Moduls darf
  // niemand direkt run()/runSpeed()/stop() o.ae. aufrufen (Race Conditions).
  AccelStepper stepperL(AccelStepper::DRIVER, PIN_STEP_L, PIN_DIR_L);
  AccelStepper stepperR(AccelStepper::DRIVER, PIN_STEP_R, PIN_DIR_R);

  // Mutex fuer alle AccelStepper-State-Aenderungen (setSpeed, MaxSpeed...)
  SemaphoreHandle_t motorMutex = nullptr;

  // Motor bereit? Bleibt false, wenn initSteppers() den Mutex nicht anlegen
  // konnte (praktisch immer Speichermangel beim Boot).
  //
  // WICHTIG: Ohne dieses Flag verlagert ein blosses "return" in
  // initSteppers() den Absturz nur nach hinten. startupSpin(),
  // tmcAutoStandbyTick(), setMotorSpeed() und stopMotors() werden unbedingt
  // weiter aufgerufen und wuerden xSemaphoreTake(nullptr, ...) ausfuehren -
  // also einen Null-Dereferenzierungs-Crash, teils erst Sekunden nach dem
  // Boot (tmcAutoStandbyTick greift nach TMC_STANDBY_MS zu). Jede oeffentliche
  // Motor-API prueft deshalb zuerst hier und tut dann nichts.
  bool motorReady = false;

  // Task-Kommunikation: esp_timer-Tick -> Notification an den Motor-Task
  TaskHandle_t motorTaskHandle = nullptr;
  esp_timer_handle_t pulseTimer = nullptr;

  // Positions-Move-Zustand pro Motor (true = Rest-Zaehler aktiv)
  volatile bool posActiveL = false;
  volatile bool posActiveR = false;
  volatile long remainingL = 0;   // offene Schritte im Position-Modus
  volatile long remainingR = 0;

  // Anfahr-Rampe an/aus (aus config.rampMs, in motorUpdateSpeedsLocked()
  // berechnet). Hier bewusst eine Kopie und nicht direkt config.rampMs: der
  // Puls-Task laeuft auf Core 0 und darf config nicht lesen, waehrend Core 1
  // es ueber einen RAMP:-Befehl schreibt. volatile bool = ein Byte = ein
  // atomarer Zugriff auf dem ESP32.
  volatile bool s_rampOn = true;
  // Beschleunigung in Schritte/s^2, ebenfalls von motorUpdateSpeedsLocked()
  // aus config.rampMs und FIXED_DRIVE_SPEED berechnet. Der Task liest nur
  // diese Zahl, nie config.
  volatile float s_accel = 0.0f;

  // Software-Rampe fuer das FREIE Fahren. AccelStepper kann eine Rampe nur
  // zusammen mit einem Ziel (computeNewSpeed() braucht distanceToGo) - beim
  // Freifahren gibt es kein Ziel, es geht nur um "wie schnell gerade".  // Also: Ist-Geschwindigkeit s_cur wird im Task um s_accel*dt pro Tick
  // Richtung s_tgt gezogen; setSpeed() bekommt jeden Tick den neuen Wert.
  //
  // s_tgt wird von setMotorSpeed() gesetzt, s_cur nur hier. Beide werden
  // ausschliesslich unter motorMutex gelesen und geschrieben.
  volatile float s_curL = 0.0f, s_tgtL = 0.0f;
  volatile float s_curR = 0.0f, s_tgtR = 0.0f;

  // Motor wirklich stilllegen: setCurrentPosition() loescht neben der
  // Geschwindigkeit auch ein (ggf. veraltetes) Ziel. Ohne das wuerde
  // computeNewSpeed() - aufgerufen von setAcceleration()/setMaxSpeed() -
  // aus einem alten Ziel wieder eine Schrittfolge planen, die runSpeed()
  // im Speed-Modus dann ausfuehrt: der Roboter liefe ohne Befehl los.
  void stepperHalt(AccelStepper& s) {
    s.setCurrentPosition(s.currentPosition());
  }

  // Rampe einen Tick weiter. maxDelta = s_accel * dt in Schritte/s.
  // maxDelta <= 0 heisst "Rampe aus" (RAMP:0, oder accel kam nicht zustande,
  // z. B. bei Tempo 0) - dann wird ohne Verzoegerung auf das Ziel gesprungen.
  void rampStep(AccelStepper& s, volatile float& cur, float tgt, float maxDelta) {
    if (cur == tgt) return;               // steht schon: kein setSpeed noetig
    if (!(maxDelta > 0.0f)) {             // auch NaN-sicher
      cur = tgt; s.setSpeed(tgt); return;
    }
    const float diff = tgt - cur;
    // In den letzten beiden Faellen wird cur EXAKT auf tgt gesetzt statt
    // cur += diff: sonst schaukelt der Motor im letzten Bit um das Ziel.
    if (diff > maxDelta)       cur += maxDelta;
    else if (diff < -maxDelta) cur -= maxDelta;
    else                       cur = tgt;
    s.setSpeed(cur);
  }

  // Motor-Task: wartet blockierend auf Timer-Ticks und erzeugt faellige Pulse.
  void motorTaskLoop(void* param) {
    // Zeitbasis fuer die Rampe. Der Task rechnet die echte verstrichene Zeit,
    // nicht die Tick-Anzahl: bei Mutex-Miss faellt schon mal ein Tick aus, und
    // die Rampe darf dadurch nicht langsamer werden.
    int64_t lastUs = esp_timer_get_time();
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // naechster 50us-Tick
      // motorMutex ist hier garantiert gesetzt: initSteppers() legt ihn an
      // und bricht VOR xTaskCreatePinnedToCore ab, falls das Anlegen
      // scheitert. Dieser Task kann also nie mit nullptr laufen. Ein
      // motorReady-Check ist hier deshalb nicht noetig (und waere in der
      // 50-us-Schleife auch unnoetiger Overhead).
      if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        // Einmal pro 50-us-Tick lesen, beide Motoren nutzen denselben Wert.
        const bool ramp = s_rampOn;
        const int64_t nowUs = esp_timer_get_time();
        float dt = (float)(nowUs - lastUs) / 1000000.0f;
        lastUs = nowUs;
        // Obergrenze: nach einem langen Stall (Task-Verzugszeit, Debugger)
        // wuerde ein ungebremster Nachhol-Schritt sonst die ganze Rampe auf
        // einmal nachholen und den Roboter einen Meter vorschiessen.
        if (!(dt > 0.0f))      dt = 0.0f;
        else if (dt > 0.05f)   dt = 0.05f;   // 50 ms
        const float maxDelta = s_accel * dt;
        if (posActiveL) {
          // Position-Modus. WICHTIG: run() meldet true, solange Speed ODER
          // Restziel != 0 - NICHT pro Puls. Der Rest-Zaehler zaehlt deshalb
          // die echten Positionsaenderungen (currentPosition() springt nur
          // bei einem Puls), was fuer run() und runSpeed() gleichermassen gilt.
          const long beforeL = stepperL.currentPosition();
          if (ramp) stepperL.run(); else stepperL.runSpeed();
          if (stepperL.currentPosition() != beforeL) {
            if (--remainingL <= 0) {
              remainingL = 0;
              posActiveL = false;
              stepperL.setSpeed(0.0f);   // Ziel erreicht -> hart stoppen
            }
          }
        } else {
          // Speed-Modus: erst die Rampe weiterdrehen, dann die Pulse.
          // Ohne Rampe sind cur und tgt gleich, rampStep() ist dann ein
          // No-op und es bleibt exakt das alte Verhalten.
          rampStep(stepperL, s_curL, s_tgtL, maxDelta);
          stepperL.runSpeed();
        }
        if (posActiveR) {
          const long beforeR = stepperR.currentPosition();
          if (ramp) stepperR.run(); else stepperR.runSpeed();
          if (stepperR.currentPosition() != beforeR) {
            if (--remainingR <= 0) {
              remainingR = 0;
              posActiveR = false;
              stepperR.setSpeed(0.0f);
            }
          }
        } else {
          rampStep(stepperR, s_curR, s_tgtR, maxDelta);
          stepperR.runSpeed();
        }
        xSemaphoreGive(motorMutex);
      }
      // Bei Mutex-Miss: nichts tun, der naechste Tick kommt eh in 50us.
    }
  }

  // esp_timer-Callback (esp_timer-Task-Kontext): nackten Tick an Motor-Task geben.
  void pulseTimerCb(void* arg) {
    if (motorTaskHandle != nullptr) {
      xTaskNotifyGive(motorTaskHandle);
    }
  }

  // ---- TMC2209 (BEIDE Treiber am gemeinsamen Single-Wire-UART) ----
  // TMC-UART wird NUR in init/Befehlen/Diagnose angefasst (Core 1),
  // nie im Puls-Task. Konfiguration 1:1 aus dem verifizierten
  // Xiao-Testsketch (TMC2209_Test) uebernommen.
  TMC2209Stepper tmcL(&Serial1, TMC_R_SENSE, TMC_ADDR_L);
  TMC2209Stepper tmcR(&Serial1, TMC_R_SENSE, TMC_ADDR_R);

  // ---- TMC-Laufzeit-State (nur Core 1, kein Mutex noetig) ----
  uint16_t s_currentMa = TMC_CURRENT_MA_DEFAULT;
  uint8_t  s_hold = TMC_HOLD_DEFAULT;
  uint16_t s_microsteps = TMC_MICROSTEPS_DEFAULT;
bool s_spread = true;
bool s_enabled = false;
// Letzte echte Fahrt (Speed != 0 oder Positionsfahrt). Nur Core 1 schreibt
// (setMotorSpeed/prepPositionMove), Tick liest -> volatile reicht.
volatile unsigned long s_lastMotionMs = 0;
  bool s_msConfirmed = false;

  bool validMicrosteps(uint16_t v) {
    return v == 1 || v == 2 || v == 4 || v == 8 || v == 16 ||
           v == 32 || v == 64 || v == 128 || v == 256;
  }

  uint8_t mresCodeFor(uint16_t ms) {
    switch (ms) {
      case 256: return 0;
      case 128: return 1;
      case 64:  return 2;
      case 32:  return 3;
      case 16:  return 4;
      case 8:   return 5;
      case 4:   return 6;
      case 2:   return 7;
      case 1:   return 8;
      default:  return 4;
    }
  }

  bool applyMicrosteps(TMC2209Stepper& d, uint16_t ms) {
    const uint8_t want = mresCodeFor(ms);
    for (uint8_t i = 0; i < 3; i++) {
      d.mstep_reg_select(true);
      d.mres(want);
      if (d.mres() == want) return true;
      delay(2);
    }
    return false;
  }

  void configureOne(TMC2209Stepper& d, bool& msOkOut) {
    d.begin();
    d.pdn_disable(true);       // UART statt PDN
    d.mstep_reg_select(true);  // Microsteps per UART
    d.I_scale_analog(false);   // Strom per UART
    d.toff(5);
    d.rms_current(s_currentMa);
    d.ihold(s_hold);
    msOkOut = applyMicrosteps(d, s_microsteps);
    d.intpol(true);
    d.pwm_autoscale(true);
    d.pwm_autograd(true);
    d.en_spreadCycle(s_spread);
    d.TPOWERDOWN(20);
    d.TPWMTHRS(0);
    // SLAVECONF auf den Auslieferungswert: Antwort sofort ("always send").
    // Der Treiber behaelt seine Register ueber einen MCU-Reset hinweg - ohne
    // diesen Reset koennte eine fremde SENDDELAY-Einstellung vom vorherigen
    // Lauf den UART-Bus unnoetig verlangsamen.
    d.SLAVECONF(0);
  }
}  // namespace

// Laufzeit-Schrittrate (Schritte/s). Nicht selbst setzen: motorUpdateSpeeds()
// rechnet sie aus config.driveSpeedMM, Rad und Microsteps.
float FIXED_DRIVE_SPEED = 0.0f;

// Beschleunigung fuer Positionsfahrten (Schritte/s^2): von 0 auf
// FIXED_DRIVE_SPEED in config.rampMs. Bewusst mild - kein Ruck, aber auch
// keine spuerbare Verzoegerung auf kurzen Strecken (27,8-mm-Feld erreicht
// praktisch noch die volle Geschwindigkeit). Haengt an FIXED_DRIVE_SPEED,
// damit die Rampe ihre Zeit in mm/s^2 trotzdem einhaelt, egal wie die
// Microsteps stehen.
static float motorAccelStepsS2() {
  if (!(FIXED_DRIVE_SPEED > 0.0f)) return 0.0f;   // NaN-sicher, s.u.
  const uint16_t ms = config.rampMs;
  if (ms == 0) return 0.0f;   // keine Rampe -> siehe motorUpdateSpeedsLocked()
  return FIXED_DRIVE_SPEED * 1000.0f / (float)ms;
}

// Schrittrate fuer die eingestellte Geschwindigkeit. configStepsPerS() ist in
// Config.cpp und rechnet mm/s x Schritte/mm; begrenzt durch MAX_DRIVE_STEPS_S.
// Die eigentliche Rechnung. Voraussetzung: motorMutex ist entweder noch nicht
// angelegt (initSteppers, der Motor-Task laeuft noch gar nicht) oder bereits
// gehalten.
static void motorUpdateSpeedsLocked() {
  FIXED_DRIVE_SPEED = configStepsPerS();
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  // Der Puls-Task liest nur dieses Flag und s_accel, nicht config.rampMs
  // (Core 0 gegen Core 1). Beide werden deshalb HIER gesetzt - jede Aenderung
  // an der Rampe muss ueber motorUpdateSpeeds() laufen.
  s_rampOn = (config.rampMs > 0);
  s_accel = motorAccelStepsS2();
  // setAcceleration() ignoriert 0 und ruft intern computeNewSpeed() - bei
  // noch stehendem Motor (Ziel == Position) ist das ein sicherer No-op. Bei
  // ausgeschalteter Rampe ist der Wert sowieso 0 und damit auch ein No-op:
  // dann fahren beide Wege ohne Rampe (Positions-Modus ueber runSpeed()).
  stepperL.setAcceleration(s_accel);
  stepperR.setAcceleration(s_accel);
  // computeNewSpeed() rechnet mit distanceToGo() = _targetPos - _currentPos.
  // Beim Freifahren laeuft _currentPos mit jedem Puls weiter, _targetPos
  // nicht - der "Abstand zum Ziel" waechst also ins Unendliche. computeNewSpeed
  // wuerde daraufhin die AccelStepper-Rampe starten und die aktuelle
  // Geschwindigkeit auf deren ersten Wert ueberschreiben (bei Tempo 150 und
  // Rampe 150 ms etwa 190 statt 2500 Schritte/s). Genau das passiert, wenn
  // jemand waehrend der Fahrt SPEED: oder RAMP: schickt. Deshalb danach
  // die vom Task gesetzte Geschwindigkeit wiederherstellen; der Task rammt
  // sie im naechsten Tick auf das neue Ziel.
  if (s_curL != 0.0f) stepperL.setSpeed(s_curL);
  if (s_curR != 0.0f) stepperR.setSpeed(s_curR);
  const float wanted = config.driveSpeedMM * configStepsPerMM();
  Serial.printf("[MOTOR] Tempo: %.0f mm/s -> %.0f Schritte/s (%u Schritte/Kreis, %u Microsteps)%s\n",
                (double)config.driveSpeedMM, (double)FIXED_DRIVE_SPEED,
                (unsigned)(MOTOR_FULL_STEPS_PER_REV * s_microsteps), s_microsteps,
                wanted > MAX_DRIVE_STEPS_S ? " - am 50us-Puls-Task begrenzt" : "");
  if (s_rampOn) {
    Serial.printf("[MOTOR] Anfahren: %u ms (0 -> %.0f Schritte/s = %.0f Schritte/s^2)\n",
                  (unsigned)config.rampMs, (double)FIXED_DRIVE_SPEED, (double)s_accel);
  } else {
    Serial.println("[MOTOR] Anfahren: aus (sofort volle Geschwindigkeit)");
  }
}

// Oeffentliche API. Sperrt den Mutex, weil setMaxSpeed() den AccelStepper-
// Zustand aendert, den der Motor-Task auf Core 0 parallel liest - dieselbe
// Klasse wie in setMotorSpeed()/. Ohne Sperre ist das formal ein Data Race.
void motorUpdateSpeeds() {
  // Noch kein Mutex -> initSteppers() vor der Anlage. Zu diesem Zeitpunkt
  // laeuft der Motor-Task nicht und Core 1 ist der einzige Schreiber, es
  // gibt also nichts zu serialisieren.
  if (!motorMutex) { motorUpdateSpeedsLocked(); return; }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
  motorUpdateSpeedsLocked();
  xSemaphoreGive(motorMutex);
}

bool motorRampActive() { return s_rampOn; }

// Beide IST-Geschwindigkeiten der Freifahr-Rampe (Schritte/s, mit
// Vorzeichen), unter EINEM Mutex-Halt gelesen - zwei Einzelaufrufe koennten
// sonst zwei verschiedene 50-us-Ticks zeigen und genau den Versatz vortaeuschen,
// den man eigentlich sucht.
// Zweck: Diagnose. Ohne das sieht man am Serial-Monitor nur die BEFEHLE
// (DRV:spd=...), nicht was die Motoren wirklich tun. Wenn die Rampe die
// Motoren unterschiedlich schnell hochzieht, sieht man es hier.
void motorRampSpeeds(float& outL, float& outR) {
  outL = s_curL;
  outR = s_curR;
  if (!motorMutex) return;   // noch nicht initialisiert
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
  outL = s_curL;
  outR = s_curR;
  xSemaphoreGive(motorMutex);
}

void initSteppers() {
  // Laufzeitwerte zuerst: die Schrittrate haengt an den Microsteps.
  s_currentMa = config.tmcCurrent;
  s_hold = config.tmcHold;
  s_microsteps = config.tmcMicrosteps;
  s_spread = config.tmcSpread;
  motorUpdateSpeeds();
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  // Rampenwert fuer Positionsfahrten steht schon (motorUpdateSpeeds() oben).
  // Hier nur der Ausgangszustand: stehende Motoren.
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // erst aus, nach TMC-Konfig einschalten
  s_enabled = false;
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  digitalWrite(PIN_STEP_L, LOW);
  digitalWrite(PIN_STEP_R, LOW);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);

  // TMC-UART PCB Edu_V2.2: ESP-TX(43) --1k (R9)--> PDN-Netz + ESP-RX(44) direkt an PDN.
  // Adresszuordnung siehe TMC_ADDR_L / TMC_ADDR_R in Motor.h (links = U4/J5).
  // Laufzeitwerte stehen schon oben (aus dem NVS uebernommen).
  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(50);
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  tmcEnable();

  motorMutex = xSemaphoreCreateMutex();
  // Jede dieser drei Anlagen kann scheitern, und die Folgen sind jeweils
  // still: ein nuller Mutex liess jede xSemaphoreTake-Dereferenzierung
  // crashen, ein fehlgeschlagener Task hiess "kein Motor", und ein
  // gescheiterter Timer hiess, dass motorTaskLoop fuer immer in
  // ulTaskNotifyTake(portMAX_DELAY) blockiert - der Roboter schaltete dann
  // nie einen Schritt und meldete keinen Fehler. Deshalb pruefen und laut
  // sagen, statt still weiterzulaufen.
  if (!motorMutex) {
    Serial.println("[MOTOR] FATAL: Mutex konnte nicht angelegt werden");
    // Hier bewusst NICHT weiterlaufen: der Motor-Task wuerde gleich in
    // xSemaphoreTake(nullptr, ...) greifen und crashen, und jede
    // setMotorSpeed()/stopMotors()-Aufruf ebenfalls. Besser: der Roboter
    // faehrt gar nicht, meldet es laut, und die Ursache (praktisch immer
    // Speichermangel beim Boot) steht im Log.
    return;
  }

  bool taskOk = (xTaskCreatePinnedToCore(
        motorTaskLoop, "MotorTask",
        4096, NULL,
        5,             // hoeher als ColorSensor (2), unter den System-Tasks
        &motorTaskHandle,
        0              // Core 0 (Main-Loop + BLE laufen auf Core 1)
      ) == pdPASS);
  if (!taskOk) {
    Serial.println("[MOTOR] FATAL: Motor-Task konnte nicht erstellt werden");
    return;
  }

  const esp_timer_create_args_t timerArgs = {
    .callback            = &pulseTimerCb,
    .arg                 = nullptr,
    .dispatch_method     = ESP_TIMER_TASK,
    .name                = "motorPulse",
    .skip_unhandled_events = true
  };
  if (esp_timer_create(&timerArgs, &pulseTimer) != ESP_OK) {
    Serial.println("[MOTOR] FATAL: Puls-Timer konnte nicht erstellt werden - Motor bleibt still");
    return;
  }
  // 50us-Periode (~20 kHz): 4x Oversampling ueber der 5 kHz Puls-Rate.
  if (esp_timer_start_periodic(pulseTimer, MOTOR_TICK_US) != ESP_OK) {
    Serial.println("[MOTOR] FATAL: Puls-Timer startete nicht - Motor bleibt still");
    return;
  }
  // Erst jetzt ist die Motoranlage vollstaendig: Mutex, Task und laufender
  // Puls-Timer vorhanden. Ab hier sind die oeffentlichen APIs erlaubt.
  motorReady = true;
}

void startupSpin() {
  if (!motorReady) {
    Serial.println("[MOTOR] Startup-Spin uebersprungen (Motor nicht bereit)");
    return;
  }
  // Beide Werte kommen aus der NVS (Web-UI). 0 bei einem davon heisst "aus" -
  // dann wird NICHT gewartet, sonst stuende der Roboter nach dem Boot
  // sinnlos eine halbe Sekunde still.
  if (config.spinMs == 0 || config.spinSpeed == 0) {
    Serial.println("[MOTOR] Anlaufdrehung aus (Drehzahl oder Dauer = 0)");
    return;
  }
  // Echter Spin AUF DER STELLE: linkes Rad vorwaerts, rechtes rueckwaerts.
  // Beide gleichsinnig zu fahren (wie vorher) hat den Roboter bei jedem Boot
  // rund 11 mm nach vorn geschoben - auf einem Tisch nicht ungefahrlich und
  // ausserdem sinnlos, denn es sah wie eine Drehung aus, war aber eine Fahrt.
  // Die Vorzeichen kommen aus config, damit es bei einer other Verdrahtung
  // korrekt bleibt.
  const long spinL = (long)config.spinSpeed * (long)config.leftSign;
  const long spinR = -(long)config.spinSpeed * (long)config.rightSign;
  Serial.printf("[MOTOR] Anlaufdrehung: %u Schritte/s fuer %u ms\n",
                (unsigned)config.spinSpeed, (unsigned)config.spinMs);
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    stepperL.setMaxSpeed((float)config.spinSpeed);
    stepperR.setMaxSpeed((float)config.spinSpeed);
    stepperL.setSpeed((float)spinL);
    stepperR.setSpeed((float)spinR);
    xSemaphoreGive(motorMutex);
  }
  // Warten - die Pulse erzeugt der Motor-Task selbststaendig.
  vTaskDelay(pdMS_TO_TICKS(config.spinMs));
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // Im Speed-Modus zaehlt runSpeed() _currentPos mit, das Ziel bleibt aber
    // stehen. Ohne stepperHalt() waere es nach dem Spin veraltet und
    // setAcceleration() (z.B. bei SPEED:) koennte daraus wieder eine Fahrt
    // planen.
    stepperHalt(stepperL);
    stepperHalt(stepperR);
    stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
    stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
    xSemaphoreGive(motorMutex);
  }
}

void setMotorSpeed(int motorIdx, float speed, bool useRamp) {
  if (!motorReady) return;   // siehe motorReady
  if (speed == 0.0f) {
    // ---- WEICHES ABBREMSEN (Taste losgelassen) ----
    // NUR das Ziel auf 0. Der Puls-Task laesst die Ist-Geschwindigkeit dann
    // ueber config.rampMs ausrollen. stepperHalt() wird hier bewusst NICHT
    // aufgerufen: das setzt _stepInterval = 0, der Motor stuende sofort.
    // Voraussetzung: die Rampe ist ueberhaupt an (sonst waere "weich"
    // ohnehin "sofort") und es laeuft keine Positionsfahrt - die hat einen
    // festen Zielpunkt und eine Schrittzahl, die kann man nicht weich
    // abbrechen. Eine laufende Positionsfahrt faellt deshalb unten in den
    // harten Zweig.
    if (useRamp && s_rampOn && !(motorIdx == 0 ? posActiveL : posActiveR)) {
      volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
      volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
      // Die Schleife ruft das in jedem Durchlauf auf: nur anfassen, wenn
      // wirklich noch etwas laeuft.
      if (cur != 0.0f || tgt != 0.0f) {
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
          xSemaphoreTake(motorMutex, portMAX_DELAY);
        }
        // s_lastMotionMs wird hier weitergepflegt: der Roboter rollt ja
        // noch aus, und tmcAutoStandbyTick() schaltet die Treiber nach
        // TMC_STANDBY_MS ab. Ohne das wuerde der Treiber mitten im
        // Ausrollen weggeschaltet.
        s_lastMotionMs = millis();
        tgt = 0.0f;
        xSemaphoreGive(motorMutex);
      }
      return;
    }
    // ---- HALTER STOPP (Not-Aus, STOP-Knopf, Timeout, TMC:EN:0) ----
    // Fail-safe: erst kurz warten, bei Fehlschlag blockieren, bis der Mutex
    // frei ist. Ein still gescheiterter Stopp hiesse "der Roboter faehrt
    // weiter" - das darf hier nicht passieren. (Die kritischen Abschnitte
    // selbst sind winzig: nur GPIO.)
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
      xSemaphoreTake(motorMutex, portMAX_DELAY);
    }
    // Expliziter Stopp fuer diesen Motor (auch eine offene Positionsfahrt abbrechen)
    // cur UND tgt auf 0, sonst zoege der Puls-Task nach dem Stopp noch auf
    // ein altes Ziel hin.
    if (motorIdx == 0) {
      posActiveL = false; remainingL = 0; s_curL = 0.0f; s_tgtL = 0.0f;
      stepperHalt(stepperL);
    } else {
      posActiveR = false; remainingR = 0; s_curR = 0.0f; s_tgtR = 0.0f;
      stepperHalt(stepperR);
    }
    xSemaphoreGive(motorMutex);
    return;
  }
  // Fail-Safe: nachhaken, statt das Ziel stillschweigend zu verwerfen.
  // Das ist hier keine Theorie. Der Mutex wird alle 50 us vom Motor-Task
  // genommen; der ColorSensor-Task laeuft auf DEMSELBEN Core 0 mit Prioritaet 2
  // und damit HOEHER als der Motor-Task (5). Er darf ihn also mitten in der
  // kritischen Sektion verdraengen - und der Farbsensor macht I2C, also
  // Millisekunden. In diesem Fenster ist der Mutex nicht zu bekommen.
  //
  // Ein still verworfenes Ziel ist genau der Fehler "ein Motor faehrt erst
  // spaeter los als der andere": driveUpdate() ruft die beiden Motoren
  // nacheinander auf, das Zeitfenster liegt mitten dazwischen, und der
  // zweite Aufruf faellt schon mal aus. Genau wie bei den Stopp-Zweigen
  // weiter unten: erst kurz warten, dann blockieren. Ein blockierendes
  // Warten ist hier unkritisch - der Motor-Task gibt den Mutex nach spaeestens
  // einem Tick (50 us) wieder frei.
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
  {
    // Jede echte Fahrt schaltet den Treiber automatisch ein
    // (STOP/TMC:EN:0 schalten ihn aus). Flag-Check ist billig.
    if (!s_enabled) tmcEnable();
    s_lastMotionMs = millis();
    // Richtung kommt vom Vorzeichen, Betrag immer FIXED_DRIVE_SPEED.
    // Vorzeichen-Korrektur ueber config.leftSign/rightSign (manche Motoren verkehrt verkabelt).
    const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;
    const float actual = (speed > 0.0f ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED) * static_cast<float>(sign);
    // Altes Ziel der abgebrochenen Positionsfahrt loeschen, dann fahren.
    AccelStepper& s = (motorIdx == 0) ? stepperL : stepperR;
    volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
    volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
    if (motorIdx == 0) { posActiveL = false; remainingL = 0; }
    else               { posActiveR = false; remainingR = 0; }
    stepperHalt(s);
    tgt = actual;
    if (useRamp && s_rampOn) {
      // WICHTIG: stepperHalt() hat oben auch das Schrittintervall genullt
      // (setCurrentPosition() setzt _stepInterval = 0). Ohne dieses
      // setSpeed() hier wuerde runSpeed() dauerhaft "nichts faellig" melden
      // und der Motor stuende still - die Rampe haette dann als einzigen
      // Effekt, dass gar nichts mehr passiert. Der Puls-Task uebernimmt den
      // Wert danach ohnehin pro Tick neu.
      //
      // cur wird hier bewusst NICHT auf actual gesetzt: setMotorSpeed() ruft
      // driveUpdate() in jedem Loop-Durchlauf auf, und jedes Anspringen auf
      // die volle Geschwindigkeit wuerde die Rampe wirkungslos machen.
      s.setSpeed(cur);
    } else {
      cur = actual;
      s.setSpeed(actual);
    }
  }
  xSemaphoreGive(motorMutex);
}

void stopMotors() {
  // Fail-safe wie beim Stopp-Zweig in setMotorSpeed(): ein Stopp darf nie
  // still scheitern, sonst laeuft der Roboter weiter.
  if (!motorReady) return;   // siehe motorReady
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
  posActiveL = false; remainingL = 0; s_curL = 0.0f; s_tgtL = 0.0f; stepperHalt(stepperL);
  posActiveR = false; remainingR = 0; s_curR = 0.0f; s_tgtR = 0.0f; stepperHalt(stepperR);
  xSemaphoreGive(motorMutex);
}

void prepPositionMove(int motorIdx, long physicalSteps) {
  if (!motorReady) return;   // siehe motorReady
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    AccelStepper& s = (motorIdx == 0) ? stepperL : stepperR;
    volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
    volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
    // Die Freifahr-Rampe wird hier bewusst stillgelegt (cur = tgt = 0).
    // Sonst zoege der Puls-Task nach dem Ende der Positionsfahrt, sobald
    // posActive wieder false ist, auf das Ziel eines ALTEN Freifahr-Befehls
    // nach - der Roboter wuerde ohne neuen Befehl wieder losfahren.
    cur = 0.0f; tgt = 0.0f;
    if (physicalSteps == 0) {
      if (motorIdx == 0) { posActiveL = false; remainingL = 0; stepperHalt(s); }
      else               { posActiveR = false; remainingR = 0; stepperHalt(s); }
    } else {
      // Jede echte Fahrt schaltet den Treiber automatisch ein.
      if (!s_enabled) tmcEnable();
      s_lastMotionMs = millis();
      // Altes Ziel und Rampenreste verwerfen, dann fahren.
      // Mit Rampe (config.rampMs > 0): move() setzt das Ziel und startet die
      // AccelStepper-Rampe, der Task ruft run(), der Motor beschleunigt und
      // bremst selbst ab. Ohne Rampe: sofort volle Geschwindigkeit, der Task
      // ruft dann runSpeed() (s. motorTaskLoop). Die beiden Wege sind nicht
      // kombinierbar - move() ruft intern computeNewSpeed() auf und das
      // wuerde ein vorangestelltes setSpeed() sofort ueberschreiben.
      stepperHalt(s);
      s.setMaxSpeed(FIXED_DRIVE_SPEED);
      if (motorIdx == 0) { posActiveL = true;  remainingL = labs(physicalSteps); }
      else               { posActiveR = true;  remainingR = labs(physicalSteps); }
      if (s_rampOn) {
        s.setAcceleration(motorAccelStepsS2());
        s.move(physicalSteps);
      } else {
        s.setSpeed(physicalSteps >= 0 ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED);
      }
    }
    xSemaphoreGive(motorMutex);
  }
}

bool motorStillRunning() {
  return (posActiveL && remainingL != 0) || (posActiveR && remainingR != 0);
}

// ---- motorSettle() geloescht ----
// Die Funktion fuhr vor dem Programmstart zwei Vollschritte hin UND exakt
// zwei zurueck, "damit die Raeder auf derselben Microstep-Index stehen".
// Das ist winkelneutral - am Ende steht jeder Rotor genau wieder auf der
// Index, auf der er vorher stand, nur eben um den Schiefstand gegen das
// jeweils andere Rad versetzt. Also genau dann wirkungslos, wenn es gebraucht
// wuerde. Sie hat dafuer 30...200 ms bei jedem Start blockiert und konnte im
// Timeout-Fall den Programmstart komplett verhindern (PROG:NO_SETTLE).

// ---- TMC2209-Laufzeit-API (Adresse siehe TMC_ADDR_L / TMC_ADDR_R) ----

void tmcEnable() {
  digitalWrite(PIN_EN, LOW);
  s_enabled = true;
}

void tmcDisable() {
  stopMotors();
  digitalWrite(PIN_EN, HIGH);
  s_enabled = false;
}

bool tmcIsEnabled() { return s_enabled; }

// Auto-Standby fuer loop(): Treiber aus, wenn seit TMC_STANDBY_MS keine
// Fahrt mehr lief UND nichts mehr offen ist (keine Positionsfahrt, Speed 0).
// Alle Fahrten sind relativ (kein absolutes Positionswissen) -> Wiederein-
// schalten bei der naechsten Fahrt ist verlustfrei.
void tmcAutoStandbyTick() {
  if (!motorReady) return;   // siehe motorReady
  if (!s_enabled) return;
  if (millis() - s_lastMotionMs < TMC_STANDBY_MS) return;
  bool idle = false;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    idle = !posActiveL && !posActiveR &&
           stepperL.speed() == 0.0f && stepperR.speed() == 0.0f;
    xSemaphoreGive(motorMutex);
  }
  if (idle) {
    tmcDisable();
    Serial.println("[TMC] Auto-Standby: Treiber aus (EN)");
  }
}

void tmcReinit() {
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  Serial.printf("[TMC] Reinit: L=%s R=%s\n", okL ? "OK" : "FEHLER", okR ? "OK" : "FEHLER");
}

bool tmcSetCurrent(uint16_t mA) {
  // HARTE Grenze 100..200 mA, siehe die Begruendung bei TMC_CURRENT_MA_MAX
  // in Motor.h. Auch der Serial-Monitor kann nichts groesseres setzen: der
  // Aufrufer (cmdTmc) speichert config.tmcCurrent nur bei true, ein
  // abgelehnter Wert erreicht die NVS also gar nicht erst.
  if (mA < TMC_CURRENT_MA_MIN || mA > TMC_CURRENT_MA_MAX) {
    Serial.printf("[TMC] Strom %u mA abgelehnt (erlaubt %u..%u mA)\n",
                  mA, TMC_CURRENT_MA_MIN, TMC_CURRENT_MA_MAX);
    return false;
  }
  s_currentMa = mA;
  tmcL.rms_current(s_currentMa);
  tmcR.rms_current(s_currentMa);
  tmcL.ihold(s_hold);
  tmcR.ihold(s_hold);
  return true;
}

bool tmcSetHold(uint8_t ihold) {
  if (ihold > 31) return false;
  s_hold = ihold;
  tmcL.ihold(s_hold);
  tmcR.ihold(s_hold);
  return true;
}

bool tmcSetMicrosteps(uint16_t ms) {
  if (!validMicrosteps(ms)) return false;
  // Reihenfolge und Rollback sind hier wichtig, weil drei Ebenen betroffen
  // sind: der Treiber (MRES im Silizium), der Laufzeit-Schatten
  // s_microsteps und config.tmcMicrosteps (die NVS-Quelle fuer configStepsPerS).
  // Vorher wurde s_microsteps VOR dem Readback gesetzt und motorUpdateSpeeds()
  // rechnete noch mit dem ALTEN config.tmcMicrosteps. Scheiterte der Readback,
  // blieb s_microsteps auf dem neuen Wert stehen, obwohl im Silizium der alte
  // lag - Shadow, NVS und Hardware liefen dauerhaft auseinander, und zwar
  // ueber einen Neustart hinweg (der Aufrufer speichert config nur bei true).
  // Genau die Art Fehler, die die Architektur in Config.h ("speichere mm,
  // nie Schrittzahlen") eigentlich ausschliessen soll.
  const uint16_t prevShadow = s_microsteps;
  const uint16_t prevConfig = config.tmcMicrosteps;
  config.tmcMicrosteps = ms;   // VOR dem Tempo: motorUpdateSpeeds() braucht den neuen Wert
  s_microsteps = ms;
  motorUpdateSpeeds();
  const bool okL = applyMicrosteps(tmcL, ms);
  const bool okR = applyMicrosteps(tmcR, ms);
  s_msConfirmed = okL && okR;
  if (!s_msConfirmed) {
    // Readback fehlgeschlagen: alles zurueckdrehen, damit Shadow und Config
    // wieder das beschreiben, was tatsaechlich im Treiber steht.
    s_microsteps = prevShadow;
    config.tmcMicrosteps = prevConfig;
    motorUpdateSpeeds();
    Serial.printf("[TMC] Microsteps %u nicht bestaetigt (L=%d R=%d) - auf %u zurueckgesetzt\n",
                  ms, okL ? 1 : 0, okR ? 1 : 0, prevShadow);
  }
  return s_msConfirmed;
}

void tmcSetSpreadCycle(bool spread) {
  s_spread = spread;
  tmcL.en_spreadCycle(s_spread);
  tmcR.en_spreadCycle(s_spread);
  if (!s_spread) {
    tmcL.pwm_autoscale(true); tmcL.pwm_autograd(true);
    tmcR.pwm_autoscale(true); tmcR.pwm_autograd(true);
  }
}

bool tmcSpreadCycle() { return s_spread; }
uint16_t tmcGetCurrent() { return s_currentMa; }
uint8_t tmcGetHold() { return s_hold; }
uint16_t tmcGetMicrosteps() { return s_microsteps; }
bool tmcMicrostepsConfirmed() { return s_msConfirmed; }

// Kompakt-Status fuer BLE/Web-UI (wird nach TMC? und jedem Set-Befehl gesendet)
void tmcSendCompactStatus() {
  const uint8_t cL = tmcL.test_connection(), cR = tmcR.test_connection();
  char m[64];
  snprintf(m, sizeof(m), "TMC:STAT:%u,%u,%u,%u,%u,%u,%u,%u",
    cL == 0 ? 1 : 0, cR == 0 ? 1 : 0,
    tmcL.version(), tmcR.version(),
    s_currentMa, s_microsteps, s_spread ? 1 : 0, s_enabled ? 1 : 0);
  sendStatus(m);
}

void tmcPrintStatus() {
  const uint8_t connL = tmcL.test_connection();
  const uint8_t connR = tmcR.test_connection();
  const uint32_t stL = tmcL.DRV_STATUS();
  const uint32_t stR = tmcR.DRV_STATUS();
  Serial.println();
  Serial.println("========== TMC2209 STATUS (Edu V2) ==========");
  Serial.printf("UART L(addr %u): %s | R(addr %u): %s\n",
    TMC_ADDR_L, connL == 0 ? "OK" : "FEHLER",
    TMC_ADDR_R, connR == 0 ? "OK" : "FEHLER");
  Serial.printf("Version L=0x%X R=0x%X (erwartet 0x21)\n", tmcL.version(), tmcR.version());
  Serial.printf("Treiber-EN (GPIO%d): %s\n", PIN_EN, s_enabled ? "EIN" : "AUS");
  Serial.printf("Strom: %u mA RMS, Hold: %u\n", s_currentMa, s_hold);
  Serial.printf("Microsteps: %u %s\n", s_microsteps, s_msConfirmed ? "(OK)" : "(NICHT UEBERNOMMEN!)");
  Serial.printf("Modus: %s\n", s_spread ? "SpreadCycle" : "StealthChop");
  Serial.printf("IFCNT: L=%u R=%u\n", tmcL.IFCNT(), tmcR.IFCNT());
  Serial.printf("SG_RESULT L=%lu R=%lu\n",
    (unsigned long)(stL & 0x03FF), (unsigned long)(stR & 0x03FF));
  Serial.println("=============================================");
  Serial.println();
}

void tmcBootDiag() {
  Serial.println("========== TMC BOOT-DIAG ==========");
  const uint8_t connL = tmcL.test_connection();
  const uint8_t connR = tmcR.test_connection();
  Serial.printf("UART L: %s (Code %u) | R: %s (Code %u)\n",
    connL == 0 ? "OK" : "FEHLER", connL, connR == 0 ? "OK" : "FEHLER", connR);
  Serial.printf("Version L=0x%X R=0x%X (erwartet 0x21)\n", tmcL.version(), tmcR.version());
  const uint8_t ifBefore = tmcL.IFCNT();
  tmcL.mstep_reg_select(true);
  const uint8_t ifAfter = tmcL.IFCNT();
  Serial.printf("IFCNT L: %u -> %u %s\n", ifBefore, ifAfter,
    ifAfter != ifBefore ? "(zaehlt = Austausch OK)" : "(steht = KEIN Austausch!)");
  Serial.printf("MRES L: %u | Strom: %u mA | Hold: %u | Modus: %s | EN: %s\n",
    tmcL.mres(), s_currentMa, s_hold,
    s_spread ? "SpreadCycle" : "StealthChop", s_enabled ? "EIN" : "AUS");
  Serial.println("===================================");
}

// "UARTNET": Charakterisiert das PDN-Netz elektrisch (ohne UART-Verkehr).
void tmcNetDiag() {
  Serial.println("========== NETZ-DIAG ==========");
  Serial1.end();
  pinMode(TMC_UART_TX, INPUT);
  pinMode(TMC_UART_RX, INPUT);
  delayMicroseconds(20);
  const int idle = digitalRead(TMC_UART_RX);
  Serial.printf("[a] Netzpegel ohne ESP-Treiber: %s\n",
    idle ? "HIGH (Pull-up vorhanden)" : "LOW (kein externer Pull-up)");

  pinMode(TMC_UART_TX, OUTPUT_OPEN_DRAIN);
  digitalWrite(TMC_UART_TX, LOW);
  delayMicroseconds(50);
  const int pulled = digitalRead(TMC_UART_RX);
  digitalWrite(TMC_UART_TX, HIGH);   // Open-Drain HIGH = hochohmig -> Release
  const int s0 = digitalRead(TMC_UART_RX);
  delayMicroseconds(1);   const int s1 = digitalRead(TMC_UART_RX);
  delayMicroseconds(9);   const int s10 = digitalRead(TMC_UART_RX);
  delayMicroseconds(90);  const int s100 = digitalRead(TMC_UART_RX);
  delay(1);               const int s1000 = digitalRead(TMC_UART_RX);
  Serial.printf("[b] Open-Drain LOW gezogen: Netz=%s\n", pulled ? "HIGH?!" : "LOW");
  Serial.printf("[c] Nach Release: 0us=%d 1us=%d 10us=%d 100us=%d 1ms=%d (1=HIGH)\n",
    s0, s1, s10, s100, s1000);

  pinMode(TMC_UART_TX, OUTPUT);
  digitalWrite(TMC_UART_TX, LOW);    // simuliert Startbit durch 1k (PCB R9)
  delayMicroseconds(50);
  const int ll = digitalRead(TMC_UART_RX);
  digitalWrite(TMC_UART_TX, HIGH);
  pinMode(TMC_UART_TX, INPUT);
  Serial.printf("[d] TX-PushPull LOW durch R9(%u Ohm): ESP liest %s\n",
    (unsigned)R9_SERIES_OHM, ll ? "HIGH" : "LOW");

  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(2);
  Serial.println("================================");
}
