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
// KEINE Beschleunigung: Es gibt nur FIXED_DRIVE_SPEED. Auch Positions-Moves
// laufen mit konstanter Geschwindigkeit via runSpeed() und einem eigenen
// Rest-Schritt-Zaehler (kein AccelStepper-Ramp-Werk, kein setAcceleration).
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
  // Stepper-Treiber - bewusst NUR hier sichtbar: Ausserhalb dieses Moduls darf
  // niemand direkt run()/runSpeed()/stop() o.ae. aufrufen (Race Conditions).
  AccelStepper stepperL(AccelStepper::DRIVER, PIN_STEP_L, PIN_DIR_L);
  AccelStepper stepperR(AccelStepper::DRIVER, PIN_STEP_R, PIN_DIR_R);

  // Mutex fuer alle AccelStepper-State-Aenderungen (setSpeed, MaxSpeed...)
  SemaphoreHandle_t motorMutex = nullptr;

  // Task-Kommunikation: esp_timer-Tick -> Notification an den Motor-Task
  TaskHandle_t motorTaskHandle = nullptr;
  esp_timer_handle_t pulseTimer = nullptr;

  // Positions-Move-Zustand pro Motor (true = Rest-Zaehler aktiv)
  volatile bool posActiveL = false;
  volatile bool posActiveR = false;
  volatile long remainingL = 0;   // offene Schritte im Position-Modus
  volatile long remainingR = 0;

  // Motor-Task: wartet blockierend auf Timer-Ticks und erzeugt faellige Pulse.
  void motorTaskLoop(void* param) {
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // naechster 50us-Tick
      if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        if (posActiveL) {
          // Position-Modus: konstante Geschwindigkeit, Rest-Zaehler dekrementieren.
          if (stepperL.runSpeed()) {
            if (--remainingL <= 0) {
              remainingL = 0;
              posActiveL = false;
              stepperL.setSpeed(0.0f);   // Ziel erreicht -> hart stoppen
            }
          }
        } else {
          // Speed-Modus: runSpeed() macht nur was bei speed != 0.
          stepperL.runSpeed();
        }
        if (posActiveR) {
          if (stepperR.runSpeed()) {
            if (--remainingR <= 0) {
              remainingR = 0;
              posActiveR = false;
              stepperR.setSpeed(0.0f);
            }
          }
        } else {
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
  }
}  // namespace

void initSteppers() {
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  // Keine Beschleunigung - nicht gesetzt, wird nirgends gebraucht.
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
  // Links = U2/J1 (Addr 0), rechts = U4/J5 (Addr 1).
  // Laufzeitwerte kommen aus dem NVS (per TMC:*-Befehl einstellbar).
  s_currentMa = config.tmcCurrent;
  s_hold = config.tmcHold;
  s_microsteps = config.tmcMicrosteps;
  s_spread = config.tmcSpread;
  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(50);
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  tmcEnable();

  motorMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    motorTaskLoop, "MotorTask",
    4096, NULL,
    5,             // hoeher als ColorSensor (2), unter den System-Tasks
    &motorTaskHandle,
    0              // Core 0 (Main-Loop + BLE laufen auf Core 1)
  );

  const esp_timer_create_args_t timerArgs = {
    .callback            = &pulseTimerCb,
    .arg                 = nullptr,
    .dispatch_method     = ESP_TIMER_TASK,
    .name                = "motorPulse",
    .skip_unhandled_events = true
  };
  esp_timer_create(&timerArgs, &pulseTimer);
  // 50us-Periode (~20 kHz): 4x Oversampling ueber der 5 kHz Puls-Rate.
  esp_timer_start_periodic(pulseTimer, MOTOR_TICK_US);
}

void startupSpin() {
  // StartupSpin: kurz mit hoher Drehzahl drehen. setMaxSpeed vorher hochsetzen,
  // weil manche AccelStepper-Versionen setSpeed auf maxSpeed begrenzen.
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    stepperL.setMaxSpeed(STARTUP_SPIN_SPEED);
    stepperR.setMaxSpeed(STARTUP_SPIN_SPEED);
    stepperL.setSpeed(STARTUP_SPIN_SPEED);
    stepperR.setSpeed(STARTUP_SPIN_SPEED);
    xSemaphoreGive(motorMutex);
  }
  // Warten - die Pulse erzeugt der Motor-Task selbststaendig.
  vTaskDelay(pdMS_TO_TICKS(STARTUP_SPIN_MS));
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    stepperL.setSpeed(0.0f);
    stepperR.setSpeed(0.0f);
    stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
    stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
    xSemaphoreGive(motorMutex);
  }
}

void setMotorSpeed(int motorIdx, float speed) {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    if (speed == 0.0f) {
      // Expliziter Stopp fuer diesen Motor (auch eine offene Positionsfahrt aborten)
      if (motorIdx == 0) { posActiveL = false; remainingL = 0; stepperL.setSpeed(0.0f); }
      else               { posActiveR = false; remainingR = 0; stepperR.setSpeed(0.0f); }
      xSemaphoreGive(motorMutex);
      return;
    }
    // Jede echte Fahrt schaltet den Treiber automatisch ein
    // (STOP/TMC:EN:0 schalten ihn aus). Flag-Check ist billig.
    if (!s_enabled) tmcEnable();
    s_lastMotionMs = millis();
    // Richtung kommt vom Vorzeichen, Betrag immer FIXED_DRIVE_SPEED.
    // Vorzeichen-Korrektur ueber config.leftSign/rightSign (manche Motoren verkehrt verkabelt).
    const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;
    const float actual = (speed > 0.0f ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED) * static_cast<float>(sign);
    if (motorIdx == 0) { posActiveL = false; remainingL = 0; stepperL.setSpeed(actual); }
    else               { posActiveR = false; remainingR = 0; stepperR.setSpeed(actual); }
    xSemaphoreGive(motorMutex);
  }
}

void stopMotors() {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    posActiveL = false; remainingL = 0; stepperL.setSpeed(0.0f);
    posActiveR = false; remainingR = 0; stepperR.setSpeed(0.0f);
    xSemaphoreGive(motorMutex);
  }
}

void prepPositionMove(int motorIdx, long physicalSteps) {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    AccelStepper& s = (motorIdx == 0) ? stepperL : stepperR;
    if (physicalSteps == 0) {
      if (motorIdx == 0) { posActiveL = false; remainingL = 0; s.setSpeed(0.0f); }
      else               { posActiveR = false; remainingR = 0; s.setSpeed(0.0f); }
    } else {
      // Jede echte Fahrt schaltet den Treiber automatisch ein.
      if (!s_enabled) tmcEnable();
      s_lastMotionMs = millis();
      // Rampen-/Richtungsreste zuruecksetzen, dann sofort volle Geschwindigkeit
      // in Zielrichtung - keine Beschleunigungsrampe.
      s.setCurrentPosition(s.currentPosition());
      s.setMaxSpeed(FIXED_DRIVE_SPEED);
      if (motorIdx == 0) { posActiveL = true;  remainingL = labs(physicalSteps); }
      else               { posActiveR = true;  remainingR = labs(physicalSteps); }
      s.setSpeed(physicalSteps >= 0 ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED);
    }
    xSemaphoreGive(motorMutex);
  }
}

bool motorStillRunning() {
  return (posActiveL && remainingL != 0) || (posActiveR && remainingR != 0);
}

// ---- TMC2209-Laufzeit-API (linker Treiber / Adresse 0) ----

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
  if (mA < 100 || mA > 800) return false;
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
  s_microsteps = ms;
  const bool okL = applyMicrosteps(tmcL, s_microsteps);
  const bool okR = applyMicrosteps(tmcR, s_microsteps);
  s_msConfirmed = okL && okR;
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

// ---- UART-Diagnose (per Serial-Befehl "UARTTEST" triggern) ----
// [2] Raw-Read an Adresse 0/1: Wir senden ein Read-Request und dumpen ALLE
//     empfangenen Bytes. Da TX ueber 1k (PCB R9) auf dasselbe Netz wie RX geht,
//     erwarten wir zunaechst ein Echo unserer 4 Bytes (Kontinuitaets-Test),
//     danach 8 Bytes TMC-Antwort. Interpretation:
//       4 Bytes            -> Leitung OK, TMC antwortet nicht (Strom/Addr/PDN-Pad)
//       12 Bytes           -> alles perfekt
//       0 Bytes            -> Bus offen (Widerstand fehlt / falscher Pin)
// [3] Absolute Baudrate messen (ein blockierender 600-Byte-Write).
// [4] Gleiches Protokoll auf Serial2 (UART2) auf denselben Pins testen.
static uint8_t tmcCrc8(const uint8_t* d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 0x01)) crc = (crc << 1) ^ 0x07;
      else crc = (crc << 1);
      crc &= 0xFF;
      b >>= 1;
    }
  }
  return crc;
}

static uint32_t measureBaud(HardwareSerial& port, const char* name) {
  while (port.available()) port.read();
  static uint8_t burst[600];
  memset(burst, 0x55, sizeof(burst));
  const uint32_t t0 = micros();
  port.write(burst, sizeof(burst));
  const uint32_t tBlock = micros() - t0;
  const uint32_t tDrain0 = micros();
  while (port.available() && (micros() - tDrain0) < 100000UL) port.read();
  uint32_t baud = 0;
  if (tBlock > 1000) {
    baud = (uint32_t)(((uint64_t)(sizeof(burst) - 128) * 10ULL * 1000000ULL) / tBlock);
  }
  Serial.printf("[3] %s: write(600) blockierte %lu us -> ~%lu Baud (115200: ~41000us)\n",
    name, (unsigned long)tBlock, (unsigned long)baud);
  return baud;
}

void tmcUartDiag() {
  Serial.println("========== UART-DIAG ==========");
  pinMode(TMC_UART_RX, INPUT_PULLDOWN);
  delay(2);
  Serial.printf("[1] RX-Netz Ruhepegel (mit Pulldown): %s (HIGH = externer Pullup/PDN verbunden)\n",
    digitalRead(TMC_UART_RX) ? "HIGH" : "LOW");

  struct PinTest { const char* name; int8_t rx; int8_t tx; };
  const PinTest tests[2] = {
    { "normal   RX=44 TX=43", TMC_UART_RX, TMC_UART_TX },
    { "getauscht RX=43 TX=44", TMC_UART_TX, TMC_UART_RX }
  };

  for (const PinTest& t : tests) {
    Serial1.end();
    Serial1.begin(TMC_BAUD, SERIAL_8N1, t.rx, t.tx);
    delay(2);
    for (uint8_t addr = 0; addr <= 3; addr++) {
      while (Serial1.available()) Serial1.read();
      // Read-Request GCONF(0x00): [0x05, (addr<<1)|0x01, 0x00, CRC]
      uint8_t req[4] = { 0x05, (uint8_t)((addr << 1) | 0x01), 0x00, 0x00 };
      req[3] = tmcCrc8(req, 3);
      Serial1.write(req, 4);
      Serial1.flush();
      delay(8);
      Serial.printf("[2] %s Addr%u: ", t.name, addr);
      int n = 0;
      while (Serial1.available() && n < 16) { Serial.printf("%02X ", Serial1.read()); n++; }
      if (n == 0)      Serial.print("(kein Byte -> Bus offen/falscher Pin)");
      else if (n <= 4) Serial.print("(nur Echo -> Leitung OK, TMC antwortet nicht)");
      else             Serial.print("(TMC-Antwort empfangen!)");
      Serial.println();
    }
    measureBaud(Serial1, "UART1");
  }
  Serial1.end();
  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(2);

  Serial2.end();
  Serial2.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(2);
  for (uint8_t addr = 0; addr <= 1; addr++) {
    while (Serial2.available()) Serial2.read();
    uint8_t req[4] = { 0x05, (uint8_t)((addr << 1) | 0x01), 0x00, 0x00 };
    req[3] = tmcCrc8(req, 3);
    Serial2.write(req, 4);
    Serial2.flush();
    delay(8);
    Serial.printf("[4] UART2 Addr%u: ", addr);
    int n = 0;
    while (Serial2.available() && n < 16) { Serial.printf("%02X ", Serial2.read()); n++; }
    if (n == 0)      Serial.print("(kein Byte)");
    else if (n <= 4) Serial.print("(nur Echo, TMC schweigt)");
    else             Serial.print("(TMC-Antwort empfangen!)");
    Serial.println();
  }
  measureBaud(Serial2, "UART2");
  Serial2.end();
  Serial2.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(2);

  Serial.println("================================");
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
