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
#include <esp_task_wdt.h>

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

// 50-us-Schleife, setMotorSpeedLocked() und prepPositionMoveLocked() ebenfalls.
static volatile bool s_emergency = false;

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
      //
      // Bewusst kein Not-Halt-Zweig im Puls-Task: der Tischkanten-Waechter
      // (edgeGuardTick im Sensor-Task) haelt ueber motorEmergencyStop() an.
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

  // Diagnose-Zugang zu den Registern ohne Getter.
  //
  // TMCStepper bietet die UART-Mechanik nur protected an (TMC2208Stepper:
  // "protected: void write(uint8_t,uint32_t); uint32_t read(uint8_t)").
  // Fuer GSTAT (0x01), TSTEP (0x12), VACTUAL (0x22) und MSCNT (0x6A) gibt es
  // keinen Getter - ohne read() koennte die Firmware diese Register gar nicht
  // auslesen, und genau an ihnen erkennt man einen Schrittverlust.
  //
  // Warum eine Ableitung und nicht eigener UART-Code: die frueher vorhandene
  // tmcUartDiag() hat den Bus selbst manipuliert und dabei die
  // UART-Adresse verschoben, sodass die Treiberadressen dauerhaft verrutscht
  // sind (Kommentar bei tmcNetDiag()). read()/write() der Bibliothek setzen
  // dieses Auto-Inkrement-Bit nicht und pruefen die CRC mit Wiederholung.
  // Die Diagnose erbt damit genau den erprobten Code, statt einen zweiten zu
  // erfinden, der dasselbe falsch machen koennte.
  class TMC2209Diag : public TMC2209Stepper {
   public:
    TMC2209Diag(Stream* serial, float rs, uint8_t addr)
      : TMC2209Stepper(serial, rs, addr) {}
    using TMC2208Stepper::read;
    using TMC2208Stepper::write;
  };

  TMC2209Diag tmcL(&Serial1, TMC_R_SENSE, TMC_ADDR_L);
  TMC2209Diag tmcR(&Serial1, TMC_R_SENSE, TMC_ADDR_R);

  // ---- TMC-Laufzeit-State (nur Core 1, kein Mutex noetig) ----
  uint16_t s_microsteps = TMC_MICROSTEPS_DEFAULT;
// Off-time und Standstill-Leistungsreduktion. Beide gehoeren in CHOPCONF
// (0x08) bzw. TPOWER (0x11) - im alten Code ueber TMCStepper gesetzt, was
// an den falschen Adressen landete (siehe configureOne()).
uint8_t  s_toff   = 5;      // 2..15; 1 nur mit TBL>=2, 0 schaltet den Treiber ab
uint8_t  s_tpower = 20;     // TPOWERDOWN: Standstill-Reduktion nach ~2^20/n Schritten
bool     motorDiagQuiet = false;   // Boot: nur eine Zeile statt sechs Pruefzeilen
bool     s_initPhase = false;          // waehrend initSteppers(): nicht doppelt konfigurieren
bool s_spread = true;
bool s_enabled = false;
// Letzte echte Fahrt (Speed != 0 oder Positionsfahrt). Nur Core 1 schreibt
// (setMotorSpeed/prepPositionMove), Tick liest -> volatile reicht.
volatile unsigned long s_lastMotionMs = 0;
  bool s_msConfirmed = false;
  // Drossel fuer die Konfigurations-Selbstheilung in motorWakeDrivers():
  // hoechstens ein Nachhol-Versuch pro Sekunde.
  uint32_t s_lastCfgRetryMs = 0;
  // Beruhigungszeit nach EN=LOW, bevor der TMC2209 konfiguriert wird.
  // Der Treiber nimmt Register-Schreibzugriffe erst an, wenn er nach dem
  // EN-Wechsel bereit ist - direkt nach dem Wechsel ging die Konfiguration
  // gelegentlich ins Leere (Symptom: "erst TMC:MS senden, dann faehrt er").
  constexpr uint32_t TMC_EN_SETTLE_MS = 10;

  // Erlaubt sind alle Zweierpotenzen 1..256. Feine Werte brauchen mehr
  // Schritte/s pro mm - das Tempo wird in Edu_V2.cpp (SPEED:/TMC:MS:)
  // auf MAX_DRIVE_STEPS_S/configStepsPerMM() gekappt, damit Anzeige ==
  // Realitaet gilt. 256 ist damit bei langsamem Tempo fahrbar.
  bool validMicrosteps(uint16_t v) {
    return v == 1 || v == 2 || v == 4 || v == 8 || v == 16 || v == 32 ||
           v == 64 || v == 128 || v == 256;
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

  // STEP-Perioden-Schwelle aus einer Geschwindigkeit. Der Registerwert ist
  // laut Datenblatt immer auf 1/256 Schritt bezogen und damit unabhaengig von
  // der Microstep-Zahl; die Herleitung steht bei TMC_SPREADCYCLE_AB_MM_S in
  // Motor.h (und ist gegen die OTP-Stufen des Herstellers geprueft).
  uint32_t tmcPeriodForMmS(float v_mm_s) {
    if (!(v_mm_s > 1.0f)) return 0xFFFFF;          // nie umschalten
    const float p = 16777216.0f * EFFECTIVE_CIRCUMFERENCE_MM / (51200.0f * v_mm_s);
    return (p >= 1048575.0f) ? 0xFFFFF : (uint32_t)p;
  }

  // Register des TMC2209, die TMCStepper NICHT richtig adressiert.
//
// GEMESSEN auf dem Roboter (2026-10, COM9): nach TMC:INIT las das Datenblatt-
// Readback TPWMTHRS=0, TCOOLTHRS=0, SGTHRS=0 - die Werte waren nie im
// Silizium. Grund: der namespace TMC2209_n kennt nur IOIN, SGTHRS, SG_RESULT
// und COOLCONF. Fuer TCOOLTHRS und TPWMTHRS benutzt TMCStepper die
// Basisklassen-Register vom TMC2130 - und die tragen dort die Adressen 0x16
// bzw. 0x15. Auf dem TMC2209 ist 0x16 das COOLCONF und 0x15 das THIGH. Der
// Aufruf sieht erfolgreich aus, schreibt aber an die falsche Stelle.
//
// Deshalb werden alle drei Schwellen hier ueber write() mit der Adresse aus
// dem Datenblatt gesetzt. Die Kontrolle steht in tmcPrintStatus() /
// tmcBootDiag(): dort wird direkt aus den Registern zurueckgelesen.
// Register des TMC2209 nach Datenblatt Rev. 1.09 - NICHT nach der Karte des
// TMC2130, die TMCStepper verwendet.
//
// GEMESSEN auf dem Roboter (COM9, 2026-10): nach der Konfiguration las das
// Readback TPWMTHRS=0, TCOOLTHRS=0, SGTHRS=0, TPOWER=0, und CHOPCONF war
// unlesbar. Grund: der namespace TMC2209_n der Bibliothek kennt nur IOIN,
// SGTHRS, SG_RESULT und COOLCONF; alles andere laeuft ueber die Basisklasse
// TMCStepper und damit ueber die Karte des TMC2130 - dort liegt CHOPCONF auf
// 0x06, beim TMC2209 ist 0x06 aber IOIN und damit nur lesbar. Der Aufruf sah
// erfolgreich aus und schrieb ins Leere.
//
// Deshalb: alle Register ueber write() mit der Adresse aus dem Datenblatt und
// jedes Schreiben sofort per Readback pruefen. Ein Register, das nicht
// ankommt, sagt es im Log - das ist der Unterschied zu vorher, wo ein
// stillschweigender Fehler als "Motor faehrt komisch" auftrat.
// ACHTUNG, beim Readback: Der TMC2209 kennzeichnet 0x03, 0x08, 0x11, 0x13,
// 0x14 und 0x40 als "W" = NUR SCHREIBBAR. Ein Readback liefert dort 0, und
// das beweist gar nichts. Genau aus dieser Fehldiagnose heraus ("Schwellen
// kommen nicht an") wurde faelschlich auf falsche Adressen geschlossen. Die
// Adressen sind richtig; beweisen kann man sie nur indirekt - ueber das
// Fahrverhalten und ueber IFCNT (zaehlt erfolgreiche Schreibzugriffe).
static void tmcWriteChecked(TMC2209Diag& d, uint8_t reg, uint32_t val,
                            uint32_t mask, const char* name) {
  d.write(reg, val);
  const uint32_t back = d.read(reg);
  if ((back & mask) == (val & mask)) {
    Serial.printf("[TMC] %-9s 0x%02X = 0x%08lX  per Readback bestaetigt\n",
                  name, reg, (unsigned long)(back & mask));
  } else {
    Serial.printf("[TMC] %-9s 0x%02X gesetzt 0x%08lX (nur schreibbar, Readback ohne Aussage)\n",
                  name, reg, (unsigned long)(val & mask));
  }
}

// Motorstrom: Datenblatt S. 28 (IHOLD_IRUN, UART-Adresse 0x10) und S. 53-54
// (Stromrechnung). Der Strom IST ueber UART einstellbar - ueber IRUN (Lauf)
// und IHOLD (Standstill), nicht ueber die Widerstaende selbst.
//
// IRUN/IHOLD sind Strom-SKALEN 0..31, wobei 0 = 1/32 und 31 = 32/32 des
// durch den Fuehlerwiderstand bestimmten Maximalstroms. Widerstand und VREF
// legen also die Obergrenze fest, IRUN/IHOLD skalieren nur darunter.
//
// Aus IRUN wird der Effektivstrom so berechnet, dass der eingestellte
// mA-Wert auch wirklich eingestellt wird:
//     I_RMS = (CS+1)/32 * VFS/(Rsense+0.02) / sqrt(2)
//     =>   CS = I_RMS * 32 * (Rsense+0.02) * sqrt(2) / VFS - 1
// mit VFS = 0,325 V bei vsense = 0 (GCONF-Standard auf dieser Platine).
static uint8_t irunForMilliAmp(uint16_t mA) {
  const float vfs = 0.325f;                      // vsense = 0
  float cs = (float)mA / 1000.0f * 32.0f * (TMC_R_SENSE + 0.02f) * 1.41421356f / vfs - 1.0f;
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  // KLEMME auf 16, kein Abrunden. Mit 560 mOhm ergibt sich fuer 200 mA
  // Sollstrom cs = 15,15 -> IRUN 16 -> 210 mA.
  //
  // Das ist Absicht und GEMESSEN, nicht geraten: IRUN = 15 laesst den Roboter
  // stehen (Bisect 2026-10: 15/7 steht still, 16/7 faehrt - IHOLD war egal).
  // Das Datenblatt bestaetigt die Richtung: "IRUN is 16 to 31 for best
  // microstep performance". Unter 16 faellt die Mikroschritt-Qualitaet so
  // weit ab, dass der Motor bei unseren Raedern nicht mehr anfaehrt - die
  // 6 % Nennstrom sind nicht das Problem, die zerfallende Sinusform ist es.
  // Der fruehere Versuch "abrunden, damit nie ueberschritten wird" (IRUN 15)
  // hat genau diesen Stillstand erzeugt und wurde zurueckgenommen.
  return (uint8_t)(cs < 16.0f ? 16.0f : cs);
}

// Reihenfolge-Vorbereitung: configureOne() steht weiter unten im Text und
// wird von hier aus gebraucht. tmcConfigureBothDrivers() ist in Motor.h
// deklariert und wird weiter unten definiert.
static void configureOne(TMC2209Diag& d, bool& msOkOut);

// Effektivstrom, den eine IRUN/IHOLD-Stufe ergibt (nur fuer die Anzeige).
static float milliAmpForIrun(uint8_t irun) {
  const float vfs = 0.325f;
  return (float)((int)irun + 1) / 32.0f * vfs / (TMC_R_SENSE + 0.02f) / 1.41421356f * 1000.0f;
}

void configureOne(TMC2209Diag& d, bool& msOkOut) {
  d.begin();
  // Erfolgsnachweis fuer die ganze Konfiguration.
  //
  // CHOPCONF, IHOLD_IRUN, TPOWER, TPWMTHRS, TCOOLTHRS, SGTHRS und NODECONF
  // sind laut Datenblatt NUR SCHREIBBAR - ein Readback liefert dort 0 und
  // beweist nichts (daraus ist die Fehldiagnose "Writes werden ignoriert"
  // entstanden). Was stattdessen stimmt, ist IFCNT: "This register becomes
  // incremented with each successful UART interface write access." Zaehlt
  // dieser Zaehler nach der Konfiguration um die Anzahl der Schreibzugriffe
  // hoch, hat der Treiber sie alle angenommen.
  const uint8_t ifBefore = (uint8_t)d.read(TMC_REG_IFCNT);
  // ---- GCONF (0x00): Modi und Flags ----
  // pdn_disable = UART statt PDN-Pin. mstep_reg_select = MRES aus dem
  // Register statt aus den MS-Pins - siehe den MRES-Hinweis am Ende.
  // multistep_filt = 1 (Glattung des Puffilings ueber 750 Hz).
  //
  // i_scale_analog (Bit 0) = 1: Der Roboter faehrt damit, mit Bit 0 = 0
  // steht er (Zaehler laufen voll, Silizium fuehrt nur einen Bruchteil aus,
  // Motoren pfeifen, v = 0). Der Aufdruck auf den Stromfuehlern ist R560,
  // also stimmen die 560 mOhm aus HARDWARE_FAKTEN.md - warum Bit 0 = 1
  // trotzdem das fahrende Bit ist, ist physikalisch OFFEN (vermutlich ist
  // die Skalierungsrichtung andersherum als angenommen). Es gilt das
  // gemessene Verhalten. Falls die Motoren heiss werden, ist das die
  // erste Stelle zum Nachsehen.
  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT
                 | 0x01u;   // i_scale_analog = 1: faehrt (GEMESSEN)
  if (s_spread) gconf |= GCONF_EN_SPREADCYCLE;
  tmcWriteChecked(d, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");

  // ---- CHOPCONF (0x08): MRES, Interpolation, toff ----
  const uint32_t mresField = ((uint32_t)mresCodeFor(s_microsteps) << CHOP_MRES_SHIFT) & CHOP_MRES_MASK;
  uint32_t chop = mresField | CHOP_INTPOL | ((uint32_t)(s_toff & 7) << CHOP_TOFF_SHIFT);
  tmcWriteChecked(d, TMC_REG_CHOPCONF, chop,
                  CHOP_MRES_MASK | CHOP_INTPOL | CHOP_TOFF_MASK, "CHOPCONF");

  // ---- IHOLD_IRUN (0x10): Motorstrom ----
  // Der Strom ist FEST und nicht einstellbar: TMC_CURRENT_MA_FIXED = 200 mA,
  // der Nennstrom des NEMA8 laut Datenblatt. Nicht "einstellbar, aber
  // versteckt" - es gibt keinen Befehl, kein NVS-Feld und kein UI-Feld mehr.
  // Wer ihn aendern will, aendert die Konstante im Code, und dann sieht man
  // es beim Booten im Log.
  //
  // IHOLD ist der halbe Fahrstrom. Ein Haltestrom gleich dem Fahrstrom laesst
  // den Motor im Stillstand heissen, ohne dass er mehr Moment haelt als im
  // Stand - er haelt nur mehr Waerme.
  const uint8_t irun  = irunForMilliAmp(TMC_CURRENT_MA_FIXED);
  const uint8_t ihold = (uint8_t)(irun / 2u);
  uint32_t iholdIrun = ((uint32_t)irun << 8) & 0x0001F000u;   // IRUN  bits 12..8
  iholdIrun |= ((uint32_t)ihold & 0x1Fu);                     // IHOLD bits 4..0
  iholdIrun |= ((uint32_t)TMC_IHOLDDELAY << 16) & 0x000F0000u; // IHOLDDELAY 19..16
  tmcWriteChecked(d, TMC_REG_IHOLD_IRUN, iholdIrun, 0x001FFFFFu, "IHOLD/IRUN");
  if (motorDiagQuiet) {
    Serial.printf("[TMC] Strom fest: IRUN=%u -> %.0f mA RMS (Soll %u mA), "
                  "IHOLD=%u -> %.0f mA, moeglich bis %.0f mA\n",
                  irun, (double)milliAmpForIrun(irun), (unsigned)TMC_CURRENT_MA_FIXED,
                  ihold, (double)milliAmpForIrun(ihold),
                  (double)milliAmpForIrun(31));
  }


  // ---- TPOWER (0x11) ----
  tmcWriteChecked(d, TMC_REG_TPOWER, s_tpower, 0xFFu, "TPOWER");

  // ---- TPWMTHRS (0x13): ab hier auf SpreadCycle ----
  // 0 bedeutet laut Datenblatt "Disabled" - der Treiber bliebe bei JEDER
  // Geschwindigkeit in StealthChop, also auch dort, wo StealthChop die
  // wenigste Drehmomentreserve hat. Der Wert ist unabhaengig von der
  // Microstep-Zahl (er bezieht sich immer auf 1/256 Schritt).
  tmcWriteChecked(d, TMC_REG_TPWMTHRS, tmcPeriodForMmS(TMC_SPREADCYCLE_AB_MM_S),
                  0x000FFFFFu, "TPWMTHRS");

  // ---- TCOOLTHRS (0x14) + SGTHRS (0x40): StallGuard ----
  // TCOOLTHRS: unterhalb dieser Geschwindigkeit ist StallGuard nicht
  // verlaesslich (Datenblatt), also erst oberhalb auswerten.
  tmcWriteChecked(d, TMC_REG_TCOOLTHRS, tmcPeriodForMmS(TMC_STALLGUARD_AB_MM_S),
                  0x000FFFFFu, "TCOOLTHRS");
  // SGTHRS ist anwendungsabhaengig; NICHTS in der Firmware handelt auf dem
  // Stall-Flag, der Wert dient nur der Anzeige.
  tmcWriteChecked(d, TMC_REG_SGTHRS, TMC_SGTHRS_DEFAULT, 0xFFu, "SGTHRS");

  // ---- NODECONF (0x03): Antworttoleranz ----
  // SENDDELAY 0 = der Treiber antwortet sofort. Ohne das koennte eine fremde
  // Einstellung vom vorherigen Lauf den UART-Bus unnoetig verlangsamen.
  tmcWriteChecked(d, TMC_REG_NODECONF, 0x0Fu, 0x0Fu, "NODECONF");

  // ---- Erfolgskontrolle ueber IFCNT ----
  // Acht Register wurden geschrieben (GCONF, CHOPCONF, IHOLD_IRUN, TPOWER,
  // TPWMTHRS, TCOOLTHRS, SGTHRS, NODECONF). Zaehlt IFCNT mindestens so weit,
  // sind alle angekommen. Die Schwelle stand vorher bei 7 und ist damit zu
  // schwach: drei Register duerften stillschweigend ausfallen, ohne dass der
  // Erfolgspfad s_msConfirmed verliert.
  const uint8_t ifAfter = (uint8_t)d.read(TMC_REG_IFCNT);
  const uint8_t sent   = (uint8_t)(ifAfter - ifBefore);
  msOkOut = (sent >= TMC_CFG_WRITE_COUNT);
  if (motorDiagQuiet) {
    Serial.printf("[TMC] Konfiguration: %u Schreibzugriffe angenommen (erwartet %u) - %s\n",
                  sent, (unsigned)TMC_CFG_WRITE_COUNT,
                  msOkOut ? "ALLE REGISTER GESETZT" : "NICHT VOLLSTAENDIG");
  }

  // ---- MRES-Hinweis: der wichtigste dieser ganzen Sektion ----
  // UND - solange mstep_reg_select = 0 - die Microstep-Aufloesung (Datenblatt
  // 4.3 und 5.5.1). Diese Platine nutzt sie fuer die Adresse (links 0b01,
  // rechts 0b00). Ohne mstep_reg_select waeren die Raeder also auf 128 bzw.
  // 256 Microsteps gelaufen, waehrend die Firmware fuer beide denselben Weg
  // berechnet - der Roboter waere geschwungen statt gerade zu fahren. Das war
  // der Drift, und es ist kein Hardware-Defekt.
}

// Beide Treiber ueber denselben Pfad konfigurieren - die einzige Stelle, die
// nach aussen wirkt. tmcSetCurrent(), tmcSetHold() und tmcReinit() gehen
// darueber, damit Register 0x10 und CHOPCONF nie auf dem Umweg ueber die
// TMCStepper-Schattenregisters geschrieben werden.
static bool tmcConfigureBothDrivers() {
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  return s_msConfirmed;
}

  // ---- Treiber-Messung (GSTAT / SG_RESULT / VACTUAL / TSTEP / MSCNT) ----
  //
  // Zweck: einen Schrittverlust bzw. Schlupf BEOBACHTEN, statt ihn zu
  // raten. Wichtig ist die Begrenzung: es wird nichts geregelt und nichts
  // nachgestellt. Kein Korrekturwert, kein Nachlaufen, kein Ausgleich - die
  // Messung liefert Zahlen und Hinweise, das Fahrverhalten aendert sich nicht.
  //
  // Ablauf: motorDiagRequest() startet einen Lauf, motorDiagTick() liest
  // danach JEDESMAL genau EIN Register und wird aus dem Loop genau einmal pro
  // Durchlauf gerufen. Ein Lauf ist damit 16 Read-Aufrufe a ~2 ms
  // (replyDelay der Bibliothek) ueber 16 Loop-Durchlaeufe verteilt und
  // blockiert nirgends. Ein einziger blockierender Aufruf von 16 Reads
  // (rund 30 ms) wuerde dagegen die Fahrt sichtbar stoeren - und ein Read im
  // 50-us-Puls-Task wuerde 40 Ticks verschlucken, also genau die Schritte
  // zerstoeren, die man messen will.
  struct TmcMeas {
    uint8_t  ifcntA[2];   // Bus-Zaehler VOR der Messung
    uint8_t  ifcntB[2];   // und danach: muss weitergezaehlt haben
    uint16_t sg[2];       // SG_RESULT (0x41) - Drehmomentreserve
    int32_t  vactual[2];  // VACTUAL (0x22) - gemessene Drehzahl, 24 Bit
    uint32_t tstep[2];    // TSTEP (0x12) - gemessene STEP-Periode, 20 Bit
    uint16_t mscnt[2];    // MSCNT (0x6A) - Sequenzerposition, 10 Bit
    uint8_t  gstat[2];    // GSTAT (0x01) - bit1 drv_err, bit2 uv_cp
    uint32_t drv[2];      // DRV_STATUS (0x6F)
  };

  TmcMeas s_meas;
  uint16_t s_mscntPrev[2] = {0, 0};
  int32_t  s_mscntAcc[2]  = {0, 0};   // seit der letzten Meldung zurueckgelegt
  uint8_t  s_diagStep = 0;            // 0 = kein laufender Messlauf
  bool     s_diagAuto = false;        // true = automatisch nach dem Anfahren
  bool     s_diagWindow = false;      // Messung lag im sicheren Tempo-Fenster
  uint16_t s_diagFlags = 0;           // Bitmaske der Befunde (siehe Bitliste)
  bool     s_diagArmed = false;       // "nach dem naechsten Anfahren messen"
  bool     s_mscntPrimed = false;     // MSCNT-Vorreffer schon genommen?
  uint32_t s_steadyLastMs = 0;        // letzte Momentaufnahme waehrend der Fahrt
  uint32_t s_steadyBusyMs  = 0;       // wie lange sie gedauert hat (Stoerungs-mass)

  // Befund-Bits der Messung. Bewusst klein gehalten: die Deutung steht in
  // Klartext im Log und in der Web-UI, die Bitmaske ist nur die Kurzfassung.
  enum DiagFlag : uint16_t {
    DIAG_UV_CP        = 1u << 0,   // GSTAT bit2: Ladepumpen-Unterspannung
    DIAG_DRV_ERR      = 1u << 1,   // GSTAT bit1: Treiberfehler
    DIAG_MODE_DIFF    = 1u << 2,   // einer StealthChop, einer SpreadCycle
    DIAG_OPEN_LOAD    = 1u << 3,   // OLA/OLB
    DIAG_SHORT        = 1u << 4,   // Kurzschluss-Flags
    DIAG_OVERTEMP     = 1u << 5,   // OT/OTPW
    DIAG_CURRENT_DIFF = 1u << 6,   // CS_ACTUAL weit auseinander
    DIAG_TSTEP_DIFF   = 1u << 7,   // STEP-Periode der Treiber weit auseinander
    DIAG_TSTEP_SAT    = 1u << 8,   // TSTEP am Anschlag = kaum Pulse gesehen
    DIAG_MSCNT_DIFF   = 1u << 9,   // MSCNT-Betraege ungleich = Schrittverlust
    DIAG_SG_DIFF      = 1u << 10,  // Drehmomentreserve stark verschieden
    DIAG_VACTUAL_DIFF = 1u << 11,  // gemessene Drehzahl weit auseinander
    DIAG_READ_ERROR   = 1u << 12,  // CRC-Fehler / Bus lieb nicht
  };

  // MSCNT-Vorreffer. Wird nach jeder Konfiguration neu genommen, weil der
  // Zaehlerstand selbst keine Aussage hat - entscheidend ist nur, wie weit
  // die beiden Treiber seitdem VORGERUECKT sind. Bei Microstep-Wechsel muss
  // das neu erfolgen: der Vorschub pro Puls haengt an MRES.
  void motorDiagPrimeMscnt() {
    for (uint8_t i = 0; i < 2; i++) {
      TMC2209Diag& d = (i == 0) ? tmcL : tmcR;
      s_mscntPrev[i] = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x03FF);
    }
    s_mscntAcc[0] = 0;
    s_mscntAcc[1] = 0;
    s_mscntPrimed = true;
  }

  // Vorreck je Treiber, vorzeichenrichtig und ueber den 10-Bit-Ueberlauf
  // hinweg. Vorzeichenrichtig ist wichtig, weil die Raeder gegenlaeufig
  // angesteuert werden: der Sequenzer laeuft bei einem Rad vorwaerts, beim
  // anderen rueckwaerts durch die Tabelle.
  int16_t mscntDelta(uint16_t now, uint16_t prev) {
    int16_t d = (int16_t)((now - prev) & 0x03FF);   // 0..1023
    return (d > 511) ? (int16_t)(d - 1024) : d;      // -> -512..511
  }
}  // namespace

// Laufzeit-Schrittrate (Schritte/s). Nicht selbst setzen: motorUpdateSpeeds()
// rechnet sie aus config.driveSpeedMM, Rad und Microsteps.
float FIXED_DRIVE_SPEED = 0.0f;

// Beschleunigung fuer Positionsfahrten (Schritte/s^2): von 0 auf
// FIXED_DRIVE_SPEED in config.rampMs. Bewusst mild - kein Ruck, aber auch
// keine spuerbare Verzoegerung auf kurzen Strecken (30,15-mm-Feld erreicht
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

// Fail-Safe-Mutex holen (Definition weiter unten, dort mit Erklaerung).
static void takeMotorMutex();

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
  takeMotorMutex();
  outL = s_curL;
  outR = s_curR;
  xSemaphoreGive(motorMutex);
}

// Schrittzaehler beider Stepper unter EINEM Mutex-Halt: zwei Einzelaufrufe
// koennten zwei verschiedene 50-us-Ticks zeigen und genau den Versatz
// vortaeuschen, den man eigentlich sucht.
void motorStepPositions(long& outL, long& outR) {
  outL = 0;
  outR = 0;
  if (!motorMutex) return;   // noch nicht initialisiert
  takeMotorMutex();
  outL = stepperL.currentPosition();
  outR = stepperR.currentPosition();
  xSemaphoreGive(motorMutex);
}

void initSteppers() {
  // Laufzeitwerte zuerst: die Schrittrate haengt an den Microsteps.
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
  s_initPhase = true;
  // WICHTIG: EN muss VOR der Konfiguration low sein. Ein TMC2209 nimmt
  // Register-Schreibzugriffe nur im eingeschalteten Zustand an - bei EN high
  // werden sie stillschweigend verworfen, und alle Registerwerte lesen sich
  // wie die Auslieferungswerte (0). Genau das ist auf dem Roboter passiert:
  // nach TMC:INIT lasen TPWMTHRS/TCOOLTHRS/SGTHRS zurueck alle 0, obwohl der
  // Aufruf keine Fehlermeldung erzeugt hat.
  tmcEnable();
  bool okL = false, okR = false;
  motorDiagQuiet = true;          // Boot: eine Zusammenfassungszeile statt 7 Zeilen
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  motorDiagQuiet = false;
  s_initPhase = false;
  s_msConfirmed = okL && okR;
  // MSCNT-Vorreffer erst NACH der Konfiguration: der Vorruck pro Puls haengt
  // an MRES, und vorher ist der Zaehlerstand nicht bedeutsam.
  motorDiagPrimeMscnt();

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

// Fail-Safe: erst kurz warten, bei Fehlschlag blockieren, bis der Mutex frei
// ist. Ein still gescheiterter Zugriff waere "der Roboter faehrt weiter".
static void takeMotorMutex() {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
}

// setMotorSpeedLocked(): setzt EINEN Motor. Voraussetzung: motorMutex ist
// gehalten. Genau deshalb gibt es setMotorSpeeds() - dort werden beide Motoren
// unter EINEM Mutex-Halt gesetzt, sodass kein Zeitfenster zwischen zwei
// Befehlen entstehen kann.
static void setMotorSpeedLocked(int motorIdx, float speed, bool useRamp) {
  // Not-Halt (Tischkante): kein Anfahren, bis motorReleaseEmergency().
  // SCHRITT 2 des Wächter-Rückbaus. Vom Bisect belegt ungefährlich, solange
  // der Halt nicht klemmt: false verzweigt hier nie hinein. Falls der Roboter
  // trotzdem steht, EDGE? lesen - "NOT-HALT aktiv" zeigt den klemmenden Halt.
  if (s_emergency) return;
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
        // s_lastMotionMs wird hier weitergepflegt: der Roboter rollt ja noch
        // aus, und tmcAutoStandbyTick() schaltet die Treiber nach
        // TMC_STANDBY_MS ab. Ohne das wuerde der Treiber mitten im
        // Ausrollen weggeschaltet.
        s_lastMotionMs = millis();
        tgt = 0.0f;
      }
      return;
    }
    // ---- HALTER STOPP (Not-Aus, STOP-Knopf, Timeout, TMC:EN:0) ----
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
    return;
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
    // Stand vor dem Befehl merken: nur ein Uebergang vom Stillstand auf
    // "faehrt" ist ein Start, den man messen will. Muss VOR dem Setzen von
    // tgt geschehen - danach waere die Bedingung nie wieder erfuellt, und im
    // ungerampten Zweig wird zusaetzlich cur auf actual gesetzt.
    const bool fromStandstill = (cur == 0.0f && tgt == 0.0f);
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
    // Messung fuer diesen Start vormerken - aber nur beim echten Anfahren
    // aus dem Stillstand. driveUpdate() ruft setMotorSpeed() sonst in jedem
    // Loop-Durchlauf auf, und jeder Durchlauf waere dann ein "Start".
    // Beide Bedingungen sind noetig: tgt wird sofort gesetzt, cur zieht die
    // Rampe erst hoch - ohne die tgt-Pruefung wuerde solange neu vorgemerkt,
    // wie cur noch 0 ist.
    if (fromStandstill) s_diagArmed = true;
  }
}

void setMotorSpeed(int motorIdx, float speed, bool useRamp) {
  if (!motorReady) return;   // siehe motorReady
  // NUR bei echter Anfahrt wecken. Ein Stillstands-Befehl (speed == 0) darf
  // die Treiber NICHT aufwecken - sonst rekonfiguriert der Roboter nach
  // jedem Auto-Standby seine Treiber erneut, nur weil driveUpdate() alle
  // 50 ms einmal (0,0) meldet. Das war ein Dauerzustand: EN immer an,
  // Dauergeräusch, blockierte Main-Loop.
  if (speed != 0.0f) motorWakeDrivers();
  // Fail-Safe: nachhaken, statt das Ziel stillschweigend zu verwerfen.
  // Der Mutex wird alle 50 us vom Motor-Task genommen. Ein still verworfenes
  // Ziel ist genau der Fehler "ein Motor faehrt erst spaeter los als der
  // andere": driveUpdate() ruft die beiden Motoren nacheinander auf, das
  // Zeitfenster liegt mitten dazwischen, und der zweite Aufruf faellt schon
  // mal aus. Genau wie bei den Stopp-Zweigen: erst kurz warten, dann
  // blockieren. Ein blockierendes Warten ist hier unkritisch - der
  // Motor-Task gibt den Mutex nach spaeestens einem Tick (50 us) wieder frei.
  takeMotorMutex();
  setMotorSpeedLocked(motorIdx, speed, useRamp);
  xSemaphoreGive(motorMutex);
}

void setMotorSpeeds(float speedL, float speedR, bool useRamp) {
  if (!motorReady) return;   // siehe motorReady
  // Siehe setMotorSpeed(): wecken nur, wenn wirklich gefahren wird.
  if (speedL != 0.0f || speedR != 0.0f) motorWakeDrivers();
  // EIN Mutex-Halt fuer beide Seiten - das ist der ganze Punkt dieser
  // Funktion. Zwei einzelne Aufrufe haetten dazwischen den Mutex wieder
  // freigegeben, und genau in diesem Fenster bekommen die Raeder
  // unterschiedliche Startzeitpunkte (siehe Kommentar in Motor.h).
  takeMotorMutex();
  setMotorSpeedLocked(0, speedL, useRamp);
  setMotorSpeedLocked(1, speedR, useRamp);
  xSemaphoreGive(motorMutex);
}

// MSCNT ist ein 10-Bit-Zaehler mit Vorzeichen: CW erhoeht, CCW vermindert.
static int16_t mscntSigned(uint16_t raw) {
  int16_t v = (int16_t)(raw & 0x03FFu);      // 0..1023
  return (v > 511) ? (int16_t)(v - 1024) : v; // -> -512..511
}

bool motorSyncMicrosteps() {
  if (!motorReady || !s_enabled) return false;
  // Nur im Stillstand. Waere einer der Motoren noch unterwegs, wuerde der
  // Ausgleichszwischen Schritt und Mikroschritt liegen und genau die
  // Unsicherheit erzeugen, die beseitigt werden soll.
  if (motorStillRunning()) return false;
  if (fabsf(s_curL) > 0.0f || fabsf(s_curR) > 0.0f) return false;

  const int16_t mL = mscntSigned((uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FFu));
  const int16_t mR = mscntSigned((uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FFu));
  const long aL = labs((long)mL), aR = labs((long)mR);
  const long diff = labs(aL - aR);

  if (diff <= (long)MOTOR_SYNC_TOLERANCE) {
    Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d - bereits im selben Schritt\n",
                  (int)mL, (int)mR);
    return false;
  }
  if (diff > (long)MOTOR_SYNC_MAX_STEPS) {
    Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d, Differenz %ld zu gross "
                  "(Grenze %u) - NICHT ausgeglichen\n",
                  (int)mL, (int)mR, diff, (unsigned)MOTOR_SYNC_MAX_STEPS);
    return false;
  }

  // Dem zurueckgebliebenen Rad die Differenz in Fahrtrichtung nachholen. Nur
  // EIN Rad bewegt sich dabei - maximal %u Mikroschritte, bei 38 mm Rad und
  // 32 Microsteps sind das %.2f mm Weg.
  if (aL < aR) {
    prepPositionMoves(diff * (long)config.leftSign, 0);
  } else {
    prepPositionMoves(0, diff * (long)config.rightSign);
  }
  const uint32_t t0 = millis();
  while (motorStillRunning() && millis() - t0 < MOTOR_SYNC_MAX_MS) {
    bleProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (motorStillRunning()) stopMotors();

  const int16_t aL2 = mscntSigned((uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FFu));
  const int16_t aR2 = mscntSigned((uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FFu));
  Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d -> L=%d R=%d "
                "(%ld Mikroschritte ausgeglichen, Rest %ld)\n",
                (int)mL, (int)mR, (int)aL2, (int)aR2, diff,
                labs(labs((long)aL2) - labs((long)aR2)));
  return true;
}

// Both wheels onto the same microstep index. See Motor.h for the why.
void stopMotors() {
     // Fail-Safe wie beim Stopp-Zweig in setMotorSpeed(): ein Stopp darf nie
     // still scheitern, sonst laeuft der Roboter weiter.
     if (!motorReady) return;   // siehe motorReady
  takeMotorMutex();
  posActiveL = false; remainingL = 0; s_curL = 0.0f; s_tgtL = 0.0f; stepperHalt(stepperL);
  posActiveR = false; remainingR = 0; s_curR = 0.0f; s_tgtR = 0.0f; stepperHalt(stepperR);
  xSemaphoreGive(motorMutex);
}

// ---- Not-Halt: haelt die Motoren an, bis ausdruecklich freigegeben ----
// Aufgerufen vom Tischkanten-Waechter im Sensor-Task (Core 1), mitten in
// jeder denkbaren Betriebsart: freies Fahren, Positionsfahrt, Block-
// programm, Linienfolger, Spiel, Slalom. Es gibt keinen Pfad, auf dem ein
// Befehl am Not-Halt vorbeikommt - setMotorSpeedLocked() und
// prepPositionMoveLocked() pruefen beide auf s_emergency und verwerfen das
// Anfahren.
// 50-us-Schleife, setMotorSpeedLocked() und prepPositionMoveLocked() ebenfalls.

void motorEmergencyStop() {
  s_emergency = true;
  // Ohne den Mutex. Der wird vom 50-us-Puls-Task alle 50 us genommen; ein
  // Wartender wuerde die Motoren womoeglich erst stoppen, wenn sie schon
  // weitergefahren sind. Der Notfall-Stopp darf nicht warten.
  //
  // Der Preis: s_curL/R und die Positionszaehler koennen gerade halb
  // aktualisiert sein. Das ist hier unkritisch - s_emergency blockiert jedes
  // weitere Anfahren, und beim Freigeben wird ueber
  // motorUpdateSpeeds()/setMotorSpeedLocked() sauber neu gesetzt.
  if (!motorReady) return;
  stepperHalt(stepperL);
  stepperHalt(stepperR);
  s_tgtL = 0.0f; s_tgtR = 0.0f;
  s_curL = 0.0f; s_curR = 0.0f;
  posActiveL = false; posActiveR = false;
  remainingL = 0; remainingR = 0;
}

void motorReleaseEmergency() {
  // Erst die Motoren anhalten, DANN den Zustand loeschen. Andersherum
  // koennte ein Befehl zwischen dem Loeschen und dem Stopp durchrutschen.
  stopMotors();
  s_emergency = false;
}

bool motorEmergencyActive() { return s_emergency; }

// Reihenfolge-Vorbereitung: prepPositionMoveLocked() wird von beiden
// oeffentlichen Wegen gebraucht, steht aber weiter unten im Text.
static void prepPositionMoveLocked(int motorIdx, long physicalSteps);

void prepPositionMove(int motorIdx, long physicalSteps) {
  if (!motorReady) return;   // siehe motorReady
  if (physicalSteps != 0) motorWakeDrivers();   // siehe setMotorSpeed()
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    prepPositionMoveLocked(motorIdx, physicalSteps);
    xSemaphoreGive(motorMutex);
  }
}

void prepPositionMoves(long stepsL, long stepsR) {
  if (!motorReady) return;   // siehe motorReady
  if (stepsL != 0 || stepsR != 0) motorWakeDrivers();   // siehe setMotorSpeed()
  // EIN Mutex-Halt fuer beide Seiten - siehe Kommentar in Motor.h. Reihenfolge
  // links zuerst wie bisher, damit sich nichts am Ablauf aendert.
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    prepPositionMoveLocked(0, stepsL);
    prepPositionMoveLocked(1, stepsR);
    xSemaphoreGive(motorMutex);
  }
}

// Voraussetzung: motorMutex ist gehalten.
static void prepPositionMoveLocked(int motorIdx, long physicalSteps) {
  // Not-Halt (Tischkante): keine neue Position setzen. SCHRITT 2, siehe oben.
  if (s_emergency) return;
  {
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
  }
}

bool motorStillRunning() {
  return (posActiveL && remainingL != 0) || (posActiveR && remainingR != 0);
}

// ---- Treiber-Messung: Anstoss, Fortsetzung, Auswertung ----
// Alle drei Funktionen laufen ausschliesslich auf Core 1 (Loop bzw.
// Befehlsannahme). Keine davon wird aus dem 50-us-Puls-Task aufgerufen, und
// keine davon nimmt den Motor-Mutex: sie lesen die Treiber, nicht die Motoren.

// VACTUAL ist 24 Bit vorzeichenbehaftet.
static int32_t tmcSignExtend24(uint32_t v) {
  v &= 0x00FFFFFFu;
  return (v & 0x00800000u) ? (int32_t)(v | 0xFF000000u) : (int32_t)v;
}

// Reihenfolge ist fest: motorDiagTick() ruft am Laufende motorDiagReport(),
// das wiederum motorDiagSendCompact().
static void motorDiagReport();
static void motorDiagSendCompact();

// Lauf anfordern. false, wenn gerade einer laeuft oder die Treiber aus sind.
bool motorDiagRequest() {
  if (!motorReady || !s_enabled) return false;
  if (s_diagStep != 0) return false;
  if (!s_mscntPrimed) motorDiagPrimeMscnt();
  memset(&s_meas, 0, sizeof(s_meas));
  s_diagStep = 1;
  s_diagAuto   = false;
  s_diagWindow = false;
  s_diagFlags  = 0;
  return true;
}

bool motorDiagRunning() { return s_diagStep != 0; }

// Aus dem Loop rufen: Messlauf fortschreiben - genau EIN Register pro Aufruf.
void motorDiagTick() {
  if (!motorReady) return;
  if (!s_enabled) { s_diagStep = 0; s_diagArmed = false; return; }
  if (s_diagStep == 0) {
    if (!s_diagArmed) return;
    s_diagArmed = false;
    // Nur im sicheren Fenster messen. Positionsfahrten werden uebersprungen -
    // dort soll der Roboter seine Strecke ungestoert zuruecklegen, und die
    // Messung wartet auf den naechsten Freifahr-Start.
    if (posActiveL || posActiveR) return;
    const float rate = fabsf(s_curL);
    if (rate < MOT_DIAG_MIN_STEPS_S || rate > MOT_DIAG_MAX_STEPS_S) return;
    if (!motorDiagRequest()) return;
    s_diagAuto   = true;
    s_diagWindow = true;
  } else if (s_diagAuto) {
    // Der Lauf zieht sich ueber 16 Loop-Durchlaeufe, und die Rampe laeuft
    // weiter. Verlaesst der Roboter das Fenster - weil das Tempo steigt ODER
    // weil die Taste losgelassen wurde und er abbremst -, wird abgebrochen:
    // ein Read bei 4321 Schritten/s wuerde dem Puls-Task die Takte wegnehmen,
    // die er zum Erzeugen der Pulse braucht, und ein Bremsen wuerde VACTUAL
    // und TSTEP auf 0 ziehen und damit einen Lesefehler vortaeuschen, den es
    // nicht gibt. Eine von Hand angeforderte Messung laeuft dagegen durch:
    // da hat der Aufrufer das gewollt.
    const float rate = fabsf(s_curL);
    if (rate < MOT_DIAG_MIN_STEPS_S || rate > MOT_DIAG_MAX_STEPS_S ||
        posActiveL || posActiveR) {
      s_diagStep = 0;
      Serial.printf("[MOT-DIAG] abgebrochen: Tempo %.0f Schritte/s (Fenster %.0f..%.0f)\n",
                    (double)rate, (double)MOT_DIAG_MIN_STEPS_S, (double)MOT_DIAG_MAX_STEPS_S);
      return;
    }
  }
  // Ein laufender Messlauf gilt als Bewegung: sonst wuerde der Auto-Standby den
  // Bus mitten in der Messung wegschalten (er schaltet nach TMC_STANDBY_MS
  // ab) und die letzten Reads liefe ins Leere.
  if (s_diagStep != 0) s_lastMotionMs = millis();

  // s_diagStep laeuft 1..16; step ist der echte Index 0..15. Der Index beginnt
  // bei 0, sonst wuerde das Paar case 0/case 1 (IFCNT vorher) nur zur Haelfte
  // ausgefuehrt und ifcntA[1] bliebe 0.
  const uint8_t step = (uint8_t)(s_diagStep - 1);
  TMC2209Diag& d = (step & 1) ? tmcR : tmcL;   // ungerade = rechts
  const uint8_t  i = (uint8_t)(step & 1);
  switch (step) {
    // IFCNT vor und nach der Messung: zaehlt der Bus dazwischen weiter, ist
    // die UART-Kommunikation nachweislich intakt (die frueher geloeschte
    // tmcUartDiag() hat genau daran die Treiberadressen verschoben).
    case  0: case  1: s_meas.ifcntA[i] = (uint8_t)d.read(TMC_REG_IFCNT);     break;
    case  2: case  3: s_meas.sg[i]      = (uint16_t)(d.read(TMC_REG_SG_RESULT) & 0x03FF); break;
    case  4: case  5: s_meas.vactual[i] = tmcSignExtend24(d.read(TMC_REG_VACTUAL)); break;
    case  6: case  7: s_meas.tstep[i]   = d.read(TMC_REG_TSTEP);            break;
    case  8: case  9: s_meas.mscnt[i]   = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x03FF); break;
    case 10: case 11: s_meas.gstat[i]   = (uint8_t)(d.read(TMC_REG_GSTAT) & 0x07); break;
    case 12: case 13: s_meas.drv[i]     = d.read(TMC_REG_DRV_STATUS);        break;
    case 14: case 15: s_meas.ifcntB[i] = (uint8_t)d.read(TMC_REG_IFCNT);     break;
    default: break;
  }
  if (++s_diagStep > 16) { s_diagStep = 0; motorDiagReport(); }

  // ---- Selbsttaetige Momentaufnahme im 2-Sekunden-Takt waehrend der Fahrt ----
  // Damit ein Drift ueber Sekunden im Log steht, ohne dass beim Fahren etwas
  // getippt werden muss. Zwei Reads je 2 s sind ein Bruchteil der Zeit; dass
  // das Lesen den Puls-Task stoert, ist selbst eine Information: veraendert
  // sich das Driftbild genau bei diesen Zeilen, ist die UART-Messung die
  // Ursache - dann faellt sie weg.
  if (s_diagStep == 0 && !posActiveL && !posActiveR && fabsf(s_curL) > 100.0f) {
    const uint32_t now = millis();
    if (now - s_steadyLastMs >= 2000UL) {
      s_steadyLastMs = now;
      const uint32_t t = millis();
      motorDiagSteadySpeed(false);
      s_steadyBusyMs = millis() - t;
    }
  }
}

// Messung mitten in der Fahrt (MOT:SPD? = ausfuehrlich, verbose=true;
// die selbsttaetige Momentaufnahme im 2-Sekunden-Takt = nur eine Zeile).
// Kein Eingriff, nur 4 Reads und eine Auswertung.
bool motorDiagSteadySpeed(bool verbose) {
  if (!motorReady || !s_enabled) return false;
  // Reihenfolge bewusst kurz: erst beide VACTUAL (das ist die eigentliche
  // Information), dann beide MSCNT als Referenz fuer den naechsten Aufruf.
  const int32_t vL = tmcSignExtend24(tmcL.read(TMC_REG_VACTUAL));
  const int32_t vR = tmcSignExtend24(tmcR.read(TMC_REG_VACTUAL));
  const uint16_t mL = (uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FF);
  const uint16_t mR = (uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FF);

  const float stepsPerMM = configStepsPerMM();
  const float mmL = (float)vL / stepsPerMM;
  const float mmR = (float)vR / stepsPerMM;
  // Vergleich ueber den Betrag: die Raeder drehen gegenlaeufig, ihre
  // VACTUAL-Werte haben also unterschiedliches Vorzeichen.
  const float aL = vL < 0 ? -(float)vL : (float)vL;
  const float aR = vR < 0 ? -(float)vR : (float)vR;
  const float mx = aL > aR ? aL : aR;
  const int diffPct = (mx > 0.0f) ? (int)((aL - (aL < aR ? aL : aR)) * 100.0f / mx) : 0;

  if (!verbose) {
    // Die Selbsttaetige Variante: eine Zeile, im Sekundentakt. Genau so ist
    // ein Drift im Log sichtbar, ohne dass beim Fahren irgendetwas getippt
    // werden muss.
    Serial.printf("[DRV] unterwegs: vL=%ld vR=%ld uSchritt/s  Diff %d %%  MSCNT %u/%u  belegt %ld\n",
                  (long)vL, (long)vR, diffPct, mL, mR,
                  (long)s_steadyBusyMs);
    return true;
  }

  Serial.println();
  Serial.println("===== MOMENTAUFNAHME MITTEN IN DER FAHRT =====");
  Serial.printf("VACTUAL L=%ld R=%ld uSchritt/s  (~%.1f / %.1f mm/s)  Diff %d %%\n",
                (long)vL, (long)vR, (double)mmL, (double)mmR, diffPct);
  Serial.printf("MSCNT   L=%u R=%u\n", mL, mR);
  if (vL == 0 && vR == 0) {
    Serial.println("Beide 0: der Roboter steht (oder der Sensor liefert nichts).");
  } else if (diffPct <= 3) {
    Serial.println("BEWERTUNG: beide Raeder drehen gleich schnell. Ein Drift kommt dann");
    Serial.println("           NICHT aus unterschiedlichem Vortschritt - sondern aus");
    Serial.println("           seitlichem Versatz (Achse schief, Boden, Spiel).");
  } else {
    Serial.println("BEWERTUNG: die Raeder drehen UNTERSCHIEDLICH schnell. Ein Rad");
    Serial.println("           kommt nicht voran: Schlupf, Reibung im Getriebe oder");
    Serial.println("           ein Rad schleift. Software-seitig ist die Pulsgabe");
    Serial.println("           gleich (siehe DRV:-Zaehler) - also mechanisch.");
  }
  Serial.println("==============================================");
  Serial.println();

  char m[72];
  snprintf(m, sizeof(m), "MOT:SPD:%ld,%ld,%d,%u,%u",
           (long)vL, (long)vR, diffPct, mL, mR);
sendStatus(m);
  return true;
}

// Auswertung + Meldung. Reine Beobachtung: hier wird NICHTS geregelt,
// nichts nachgestellt und nichts korrigiert.
void motorDiagReport() {
  // MSCNT-Vorreck seit der letzten Meldung aufaddieren, damit auch ein
  // Befund ueber mehrere Anfahrten hinweg sichtbar wird, statt bei jedem
  // Start wieder bei null zu beginnen.
  for (uint8_t i = 0; i < 2; i++) {
    s_mscntAcc[i] += mscntDelta(s_meas.mscnt[i], s_mscntPrev[i]);
    s_mscntPrev[i] = s_meas.mscnt[i];
  }

  uint16_t fl = 0;
  for (uint8_t i = 0; i < 2; i++) {
    if (s_meas.gstat[i] & 0x04) fl |= DIAG_UV_CP;
    if (s_meas.gstat[i] & 0x02) fl |= DIAG_DRV_ERR;
    if (s_meas.drv[i] & (DRVST_OLA | DRVST_OLB)) fl |= DIAG_OPEN_LOAD;
    if (s_meas.drv[i] & (DRVST_S2GA | DRVST_S2GB | DRVST_S2VSA | DRVST_S2VSB)) fl |= DIAG_SHORT;
    if (s_meas.drv[i] & (DRVST_OT | DRVST_OTPW)) fl |= DIAG_OVERTEMP;
  }
  // Beide Treiber muessen bei gleicher Geschwindigkeit im selben Modus laufen.
  // Genau hier faellt eine echte Asymmetrie sofort auf: unterschiedliche
  // Drehmoment-Charakteristik bei sonst gleicher Konfiguration.
  if (((s_meas.drv[0] & DRVST_STEALTH) != 0) != ((s_meas.drv[1] & DRVST_STEALTH) != 0))
    fl |= DIAG_MODE_DIFF;

  const uint32_t tstepSat = 0x000FFFFFu;   // (2^20)-1: Stillstand
  for (uint8_t i = 0; i < 2; i++) {
    if (s_meas.tstep[i] >= tstepSat) fl |= DIAG_TSTEP_SAT;
  }
  // Vergleich nur, wenn beide Werte echt sind: 0 und der Anschlag (2^20-1 im
  // Stillstand) sind keine STEP-Periode.
  if (!(fl & DIAG_TSTEP_SAT) && s_meas.tstep[0] > 0 && s_meas.tstep[1] > 0) {
    const uint32_t lo = s_meas.tstep[0] < s_meas.tstep[1] ? s_meas.tstep[0] : s_meas.tstep[1];
    const uint32_t hi = s_meas.tstep[0] < s_meas.tstep[1] ? s_meas.tstep[1] : s_meas.tstep[0];
    if ((hi - lo) * 100u / lo > DIAG_TSTEP_DIFF_PCT) fl |= DIAG_TSTEP_DIFF;
  }
  // Aktuelle Strombelastung der beiden Seiten (CS_ACTUAL). Ein grosser
  // Unterschied heisst: ein Rad laeuft schwerer als das andere - Klemmen,
  // Schleifen oder unterschiedliche Wicklung.
  const uint8_t csL = (uint8_t)((s_meas.drv[0] >> DRVST_CS_ACTUAL_SHIFT) & DRVST_CS_ACTUAL_MASK);
  const uint8_t csR = (uint8_t)((s_meas.drv[1] >> DRVST_CS_ACTUAL_SHIFT) & DRVST_CS_ACTUAL_MASK);
  if (csL > 0 && csR > 0) {
    const uint8_t mx = csL > csR ? csL : csR;
    const uint8_t mn = csL > csR ? csR : csL;
    if ((uint16_t)(mx - mn) * 100u / mx > DIAG_CS_DIFF_PCT) fl |= DIAG_CURRENT_DIFF;
  }
  const int ds = (int)s_meas.sg[0] - (int)s_meas.sg[1];
  if (ds < 0 ? -ds > DIAG_SG_DIFF_MIN : ds > DIAG_SG_DIFF_MIN) fl |= DIAG_SG_DIFF;
  // VACTUAL-Vergleich: jeder Treiber misst mit seinem eigenen internen Takt,
  // deshalb sind absolute Werte nur auf wenige Prozent genau. Eine Differenz
  // deutlich darueber ist echter Schlupf oder eine mechanische Asymmetrie.
  if (s_meas.vactual[0] != 0 && s_meas.vactual[1] != 0) {
    const int32_t ax = s_meas.vactual[0] < 0 ? -s_meas.vactual[0] : s_meas.vactual[0];
    const int32_t ay = s_meas.vactual[1] < 0 ? -s_meas.vactual[1] : s_meas.vactual[1];
    const int32_t mx = ax > ay ? ax : ay;
    if (mx > 0 && (uint32_t)(ax - (ax > ay ? ay : ax)) * 100u / (uint32_t)mx > DIAG_VACTUAL_DIFF_PCT)
      fl |= DIAG_VACTUAL_DIFF;
  }
  // MSCNT: entscheidend ist der VORRUCK beider Seiten, nicht der Zaehlerstand.
  // Die Raeder sind gegenlaeufig angesteuert, deshalb die Betraege vergleichen.
  const long a0 = s_mscntAcc[0] < 0 ? -s_mscntAcc[0] : s_mscntAcc[0];
  const long a1 = s_mscntAcc[1] < 0 ? -s_mscntAcc[1] : s_mscntAcc[1];
  if (labs(a0 - a1) > DIAG_MSCNT_TOLERANCE) fl |= DIAG_MSCNT_DIFF;

  // Bus-Pruefung: zwischen den beiden IFCNT-Lesungen liegen 12 weitere Reads,
  // der Zaehler muss also um 12 gewachsen sein (plus allem, was die Loop
  // sonst noch macht). uint8-Differenz unter 128 toleriert den Ueberlauf.
  if ((uint8_t)(s_meas.ifcntB[0] - s_meas.ifcntA[0]) > 127 ||
      (uint8_t)(s_meas.ifcntB[1] - s_meas.ifcntA[1]) > 127) fl |= DIAG_READ_ERROR;
  // VACTUAL/TSTEP koennen nur waehrend der Fahrt ungleich null sein - ausserhalb
  // gilt das nicht als Lesefehler (das waere die haeufigste Fehlmeldung).
  if (s_diagWindow && (s_meas.vactual[0] == 0 || s_meas.vactual[1] == 0 ||
                       s_meas.tstep[0] == 0   || s_meas.tstep[1] == 0))
    fl |= DIAG_READ_ERROR;
  s_diagFlags = fl;

  const float spmm = configStepsPerMM();
  Serial.println();
  Serial.println(s_diagAuto ? "===== TREIBER-MESSUNG (nach dem Anfahren) ====="
                            : "===== TREIBER-MESSUNG (manuell angefordert) =====");
  if (!s_diagWindow) {
    Serial.println("Hinweis: gemessen wurde ausserhalb der Fahrt. VACTUAL/TSTEP sind");
    Serial.println("       dann ohne Aussage (Steiger sind im Stillstand nicht messbar);");
    Serial.println("       GSTAT, SG_RESULT, MSCNT und DRV_STATUS gelten trotzdem.");
  }
  // Bewusst OHNE das Vorzeichen-Flag "%+d": der printf der ESP32-ROM-Bibliothek
  // unterstuetzt nicht alle Flags, und eine kaputte Log-Zeile waere schlimmer
  // als ein fehlendes Pluszeichen. Negative Werte bringen ihr Minus von selbst.
  Serial.printf("SG_RESULT L=%u R=%u  (Diff %d, Stall ab <= %u)\n",
                s_meas.sg[0], s_meas.sg[1], ds, TMC_SGTHRS_DEFAULT * 2);
  Serial.println("  Drehmomentreserve: groesser = mehr Reserve. Nur oberhalb TCOOLTHRS");
  Serial.println("  verlaesslich und fuer StealthChop kalibriert. 0 = Stall oder Lesefehler.");
  Serial.printf("VACTUAL   L=%ld R=%ld uSchritt/s  (~%.1f / %.1f mm/s)\n",
                (long)s_meas.vactual[0], (long)s_meas.vactual[1],
                (double)(s_meas.vactual[0] / spmm), (double)(s_meas.vactual[1] / spmm));
  Serial.println("  Von jedem Treiber selbst gemessen. Absolute Werte schwanken je IC");
  Serial.println("  (interner Takt) - aussagekraeftig ist nur der Vergleich.");
  Serial.printf("TSTEP     L=%lu R=%lu  (Anschlag %lu = Stillstand)\n",
                (unsigned long)s_meas.tstep[0], (unsigned long)s_meas.tstep[1],
                (unsigned long)tstepSat);
  Serial.println("  Von beiden Treibern gemessene STEP-Periode. Stark verschieden = einer");
  Serial.println("  bekommt andere Pulse. Anschlag = (fast) keine Pulse gesehen.");
  Serial.printf("MSCNT     L %ld R %ld  (Betragsdifferenz %ld Mikroschritte)\n",
                (long)s_mscntAcc[0], (long)s_mscntAcc[1], labs(a0 - a1));
  Serial.println("  Der Zaehlerstand selbst bedeutet nichts - entscheidend ist der VORRUCK:");
  Serial.println("  beide Seiten sollten gleich weit gelaufen sein (Vorzeichen gegenlaeufig).");
  Serial.printf("GSTAT     L=0x%02X R=0x%02X   (bit1 drv_err, bit2 uv_cp)\n",
                s_meas.gstat[0], s_meas.gstat[1]);
  Serial.printf("DRV_STATUS L=0x%08lX R=0x%08lX\n",
                (unsigned long)s_meas.drv[0], (unsigned long)s_meas.drv[1]);
  Serial.printf("  Modus %s/%s, CS_ACTUAL L=%u R=%u, OLA=%u OLB=%u, OT=%u, Kurzschluss=%u\n",
                (s_meas.drv[0] & DRVST_STEALTH) ? "Stealth" : "Spread",
                (s_meas.drv[1] & DRVST_STEALTH) ? "Stealth" : "Spread",
                csL, csR,
                (s_meas.drv[0] & DRVST_OLA) ? 1u : 0u, (s_meas.drv[0] & DRVST_OLB) ? 1u : 0u,
                (s_meas.drv[0] & (DRVST_OT | DRVST_OTPW)) ? 1u : 0u,
                (s_meas.drv[0] & (DRVST_S2GA | DRVST_S2GB | DRVST_S2VSA | DRVST_S2VSB)) ? 1u : 0u);
  Serial.printf("IFCNT     L %u->%u  R %u->%u  (UART %s)\n",
                s_meas.ifcntA[0], s_meas.ifcntB[0], s_meas.ifcntA[1], s_meas.ifcntB[1],
                (fl & DIAG_READ_ERROR) ? "PRUEFEN" : "zaehlt weiter");
  if (fl == 0) {
    Serial.println("BEFUND    nichts auffaellig - beide Treiber sehen dasselbe Bild.");
  } else {
    Serial.println("BEFUND");
    if (fl & DIAG_UV_CP)       Serial.println("  ! Ladepumpen-Unterspannung (GSTAT bit2). Der Treiber schaltet");
    if (fl & DIAG_UV_CP)       Serial.println("    laut Datenblatt in diesem Zustand ab - Motorstrom ist nicht da.");
    if (fl & DIAG_DRV_ERR)     Serial.println("  ! Treiberfehler (GSTAT bit1) - Bit wird nur durch Schreiben gelesen.");
    if (fl & DIAG_MODE_DIFF)   Serial.println("  ! Ein Treiber in StealthChop, der andere in SpreadCycle: die beiden");
    if (fl & DIAG_MODE_DIFF)   Serial.println("    Seiten haben unterschiedliche Drehmoment-Charakteristik.");
    if (fl & DIAG_OPEN_LOAD)   Serial.println("  ! Offene Last (OLA/OLB): Kabel, Stecker oder Wicklung.");
    if (fl & DIAG_SHORT)       Serial.println("  ! Kurzschluss-Flag gesetzt.");
    if (fl & DIAG_OVERTEMP)    Serial.println("  ! Ueberhitzung gemeldet (OT/OTPW).");
    if (fl & DIAG_CURRENT_DIFF)Serial.println("  ! CS_ACTUAL weit auseinander: ein Rad laeuft deutlich schwerer.");
    if (fl & DIAG_TSTEP_SAT)   Serial.println("  ! TSTEP am Anschlag: eine Seite hat (fast) keine STEP-Pulse bekommen.");
    if (fl & DIAG_TSTEP_DIFF)  Serial.println("  ! STEP-Periode der Treiber weit auseinander: Pulse gehen unterschiedlich an.");
    if (fl & DIAG_MSCNT_DIFF)  Serial.println("  ! MSCNT-Vorreck ungleich: eine Seite hat Mikroschritte verloren -");
    if (fl & DIAG_MSCNT_DIFF)  Serial.println("    das ist der direkte Nachweis fuer einen Schrittverlust.");
    if (fl & DIAG_SG_DIFF)     Serial.println("  ! Drehmomentreserve der Seiten stark verschieden.");
    if (fl & DIAG_VACTUAL_DIFF)Serial.println("  ! Gemessene Drehzahl weit auseinander: Schlupf oder Asymmetrie.");
    if (fl & DIAG_READ_ERROR)  Serial.println("  ! Lesefehler/CRC oder Bus steht - Zahlen dieser Messung nicht verwenden.");
  }
  Serial.println("Hinweis: reine Beobachtung. Es wird nichts nachgeregelt oder ausgeglichen.");
  Serial.println("=======================================================");
  Serial.println();
  motorDiagSendCompact();
}

// Kurzfassung fuer Web-UI/BLE. Reihenfolge der Felder ist Vertrag - die
// Web-UI parst sie anhand der Positionen.
void motorDiagSendCompact() {
  char m[112];
  snprintf(m, sizeof(m),
           "MOT:MEAS:%u,%u,%u,%ld,%ld,%lu,%lu,%ld,%ld,%u,%u",
           s_diagFlags, s_meas.sg[0], s_meas.sg[1],
           (long)s_meas.vactual[0], (long)s_meas.vactual[1],
           (unsigned long)s_meas.tstep[0], (unsigned long)s_meas.tstep[1],
           (long)s_mscntAcc[0], (long)s_mscntAcc[1],
           s_meas.gstat[0], s_meas.gstat[1]);
  sendStatus(m);
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

// Konfiguration beim Einschalten erneut senden.
//
// GEMESSEN: GSTAT bit 0 ("reset") stand dauerhaft auf 1, und die Register
// standen nach jedem Auto-Standby wieder auf dem Auslieferungswert. Das
// ENN-Pin des TMC2209 ist nicht nur ein Enable, sondern setzt den internen
// Zustand zurueck - beim Hochfahren gehen alle Register auf Reset (Datenblatt
// 5.5.1: "All registers have been cleared to reset values").
//
// Die Auto-Standby-Abschaltung nach TMC_STANDBY_MS hat damit JEDES Register
// wieder geloescht, ohne dass irgendetwas einen Fehler gemeldet hat. Deshalb
// wird nach jedem EN-Wechsel neu konfiguriert. Das kostet ~30 ms UART, und
// das passiert nur, wenn der Roboter vorher mindestens 3 s stand - also
// waehrend eines Stillstands, lange bevor die Rampe loslaeuft.
static void tmcEnsureConfigured() {
  if (!motorReady) return;
  if (s_initPhase) return;   // initSteppers() konfiguriert selbst - sonst waeren es vier Durchgaenge
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  motorDiagPrimeMscnt();   // Konfiguration kann MRES angefasst haben
}

void tmcEnable() {
  const bool wasOff = !s_enabled;
  digitalWrite(PIN_EN, LOW);
  s_enabled = true;
  if (wasOff) {
    // Kurz warten, bis der Treiber nach dem EN-Wechsel bereit ist - sonst
    // kann die folgende Konfiguration ins Leere laufen (siehe
    // TMC_EN_SETTLE_MS). Beim Boot gilt dasselbe: initSteppers() ruft
    // tmcEnable() direkt vor seinen configureOne()-Aufrufen auf.
    delay(TMC_EN_SETTLE_MS);
    tmcEnsureConfigured();
  }
}

// Treiber vor dem Mutex-Halt aufwecken - aber NUR wenn wirklich gefahren werden
// soll. Reihenfolge, Timing und Sicherheitsregeln siehe Motor.h.
void motorWakeDrivers() {
  if (!motorReady) return;   // siehe motorReady
  if (!s_enabled) { tmcEnable(); return; }
  // Selbstheilung: Ist der Registersatz NICHT bestaetigt (Boot ohne
  // Motorspannung, EN-Reset durch Auto-Standby, gestoerte UART-Transaktion),
  // wird er beim naechsten Fahrbefehl nachgeholt. Vorher blieb der Roboter
  // in genau dieser Lage stehen, bis jemand TMC:MS oder TMC:INIT sendete.
  // Gedrosselt, damit ein wirklich fehlender Treiber die Fahrbefehle nicht
  // ausbremst (ein Versuch pro Sekunde reicht).
  if (!s_msConfirmed && (uint32_t)(millis() - s_lastCfgRetryMs) >= 1000u) {
    s_lastCfgRetryMs = millis();
    Serial.println("[TMC] Registersatz unvollstaendig - Konfiguration wird beim Fahren nachgeholt");
    tmcEnsureConfigured();
  }
}

// ---- Akku-Waechter (TMC-Proxy, keine Hardware-Aenderung) ----
// Direkte Spannungsmessung gibt es nicht (kein Teiler, kein freier ADC-Pin
// an VM). Stattdessen zaehlt der Waechter GSTAT-uv_cp-Sichtungen (Bit 2 =
// Ladepumpen-Unterspannung): Faellt VM ein, setzt der Treiber das Bit und
// schaltet laut Datenblatt ab. Grob (kein Fuellstand, nur "kritisch"), aber
// besser als stilles Stehenbleiben mit falscher Odometrie.
// Muster wie motorDiagTick: EIN Read pro Aufruf, abwechselnd L/R. Nur wenn
// die Treiber wach sind - im Standby ist nichts pruefbar und nichts noetig.
static uint32_t s_batLastMs = 0;
static uint8_t  s_batSide = 0;
static uint8_t  s_batHits = 0;      // aufeinanderfolgende uv_cp-Sichtungen
static uint32_t s_batReportMs = 0;  // letztes BAT:LOW (zyklischer Re-Send)
static bool     s_batLow = false;   // Latch pro Boot
bool tmcBatteryLow() { return s_batLow; }

void tmcBatteryGuardTick() {
  const uint32_t now = millis();
  // Re-Send im Latch: Seite neu laden oder neu verbinden bekommt die
  // Warnung trotzdem (spaetestens alle 30 s).
  if (s_batLow) {
    if (now - s_batReportMs >= 30000u) {
      s_batReportMs = now;
      sendStatus("BAT:LOW");
    }
    return;
  }
  if (!motorReady || !s_enabled) return;
  if (now - s_batLastMs < 2000u) return;   // alle 2 s genau EIN Read
  s_batLastMs = now;
  TMC2209Diag& d = (s_batSide == 0) ? tmcL : tmcR;
  s_batSide ^= 1;
  const uint8_t g = (uint8_t)(d.read(TMC_REG_GSTAT) & 0x07);
  if (g & 0x04) {
    if (++s_batHits >= 3) {
      s_batLow = true;
      s_batReportMs = now;
      Serial.println("[BAT] Unterspannung (GSTAT uv_cp, 3x bestaetigt) - Akku schwach, bitte laden");
      sendStatus("BAT:LOW");
    }
  } else {
    s_batHits = 0;
  }
}

// Auskommentiert gehalten: der Wetter-koennte-es-besser-wissen-Helfer wird
// nicht gebraucht, weil die Wrapper die Null-Pruefung selbst machen (siehe
// Kommentar dort).

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
  // Wie beim Boot: erst einschalten, dann konfigurieren. Sonst gehen alle
  // Schreibzugriffe ins Leere, ohne dass irgendetwas einen Fehler meldet.
  //
  // Genau ein Konfigurationsdurchgang je Treiber. Vorher standen hier zwei
  // configureOne()-Aufrufe ZUSAETZLICH zu denen aus tmcEnable() ->
  // tmcEnsureConfigured(), also bis zu vier Durchgaenge: rund 40 UART-
  // Transaktionen und 28 Logzeilen fuer einen einzigen Tastendruck.
  tmcEnable();
  const bool ok = tmcConfigureBothDrivers();
  motorDiagPrimeMscnt();   // Konfiguration kann MRES angefasst haben
  Serial.printf("[TMC] Reinit: %s\n", ok ? "OK" : "FEHLER - Register nicht bestaetigt");
}

// tmcSetCurrent() und tmcSetHold() sind ersatzlos entfallen. Der Motorstrom ist
// fest 200 mA (TMC_CURRENT_MA_FIXED), IHOLD ist der halbe Fahrstrom - beides
// wird in configureOne() aus der einen Konstanten abgeleitet. Es gibt keinen
// Befehl, kein NVS-Feld und kein UI-Feld, ueber das man daran kommen koennte.
//
// Grund der Loeschung, nicht nur der Aufraeumwunsch: beide gingen ueber
// TMCStepper und schrieben damit das KOMPLETTE Register IHOLD_IRUN (0x10)
// aus dem Bibliotheks-Schatten. Den Schatten pflegt nur die Bibliothek selbst
// - configureOne() schreibt 0x10 direkt und laesst ihn unberuehrt, wo noch
// die Defaults stehen, darunter irun = 0. TMC:HOLD:20 schrieb deshalb nicht
// "Haltestrom 20", sondern "Haltestrom 20, Fahrstrom 0": rund 12 mA statt
// 210 mA, die Motoren drehten nicht mehr. rms_current() rief zusaetzlich
// vsense() auf, das CHOPCONF an die TMC2130-Adresse 0x6C schreibt statt an
// 0x08 (IHOLD_IRUN.cpp:4 belegt das Schreiben des ganzen Registers).
//
// Ausserdem waren die Regler wirkungslos: die IRUN-Klemme auf 16 hat jede
// Einstellung von 100 bis 200 mA auf dieselben 210 mA gebracht.

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
  // MSCNT-Vorreffer neu nehmen: der Vorreck pro Puls haengt an MRES. Ohne das
  // wuerde der Vergleich gegen die alte Microstep-Zahl laufen und bei jedem
  // Microstep-Wechsel einen Scheinbefund "Schrittverlust" erzeugen.
  motorDiagPrimeMscnt();
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
  // Moduswechsel NUR im Stillstand. GCONF.EN_SPREADCYCLE wird mitten in einer
  // Fahrt gesetzt -> beide Treiber schalten ihre Chopper-Art um, was einen
  // kurzen Drehmoment- und Drehzahl-Einbruch von mehreren Millisekunden
  // bedeutet (gemessen ~10 ms Stillstand). Und motorSyncMicrosteps() waere
  // wirkungslos, weil es bei laufenden Motoren bewusst zurueckkehrt - die
  // Raeder stuenden danach mitten in der Fahrt in verschiedenen Mikroschritt-
  // Phasen. Beides ist vermeidbar: der Aufrufer (Web-UI/Serial) bekommt statt
  //dessen eine klare Ablehnung und kann es im Stillstand wiederholen.
  if (motorStillRunning() || fabsf(s_curL) > 0.0f || fabsf(s_curR) > 0.0f) {
    Serial.println("[TMC] Moduswechsel abgelehnt - Roboter faehrt. Erst anhalten, dann wechseln.");
    sendStatus("TMC:BUSY");
    return;
  }
  s_spread = spread;
  tmcEnable();                 // GCONF aendert den EN-Zustand nicht, aber der
                               // Moduswechsel ist der Moment, an dem beide
                               // Motoren auf derselben Position starten sollen
  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT
                 | 0x01u;   // i_scale_analog, GEMESSEN (siehe configureOne)
  if (s_spread) gconf |= GCONF_EN_SPREADCYCLE;
  tmcWriteChecked(tmcL, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");
  tmcWriteChecked(tmcR, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");
  // Und jetzt beide Raeder auf denselben Mikroschritt bringen. Erst danach
  // ist der neue Modus ueberhaupt sinnvoll: zwei Motoren, deren Sinusphase
  // gegeneinander laeuft, reagieren auf einen Moduswechsel unterschiedlich.
  motorSyncMicrosteps();
}

bool tmcSpreadCycle() { return s_spread; }
uint16_t tmcGetCurrent() { return TMC_CURRENT_MA_FIXED; }
uint8_t tmcGetHold() { return (uint8_t)(irunForMilliAmp(TMC_CURRENT_MA_FIXED) / 2u); }
uint16_t tmcGetMicrosteps() { return s_microsteps; }
bool tmcMicrostepsConfirmed() { return s_msConfirmed; }

// Kompakt-Status fuer BLE/Web-UI (wird nach TMC? und jedem Set-Befehl gesendet)
void tmcSendCompactStatus() {
  const uint8_t cL = tmcL.test_connection(), cR = tmcR.test_connection();
  char m[64];
  snprintf(m, sizeof(m), "TMC:STAT:%u,%u,%u,%u,%u,%u,%u,%u",
    cL == 0 ? 1 : 0, cR == 0 ? 1 : 0,
    tmcL.version(), tmcR.version(),
    TMC_CURRENT_MA_FIXED, s_microsteps, s_spread ? 1 : 0, s_enabled ? 1 : 0);
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
  Serial.printf("Strom fest: IRUN=%u -> %.0f mA RMS (Soll %u mA), IHOLD=%u -> %.0f mA\n",
              irunForMilliAmp(TMC_CURRENT_MA_FIXED),
              (double)milliAmpForIrun(irunForMilliAmp(TMC_CURRENT_MA_FIXED)),
              (unsigned)TMC_CURRENT_MA_FIXED, tmcGetHold(),
              (double)milliAmpForIrun(tmcGetHold()));
  Serial.printf("  Nicht einstellbar - Nennstrom des NEMA8. Obergrenze durch die "
                "560-mOhm-Fuehler waere %.0f mA (IRUN=31); davon fahren wir %.0f mA.\n",
                (double)milliAmpForIrun(31),
                (double)milliAmpForIrun(irunForMilliAmp(TMC_CURRENT_MA_FIXED)));
  Serial.printf("Microsteps: %u %s\n", s_microsteps, s_msConfirmed ? "(OK)" : "(NICHT UEBERNOMMEN!)");
  Serial.printf("Modus: %s\n", s_spread ? "SpreadCycle" : "StealthChop");
  Serial.printf("IFCNT: L=%u R=%u\n", tmcL.IFCNT(), tmcR.IFCNT());
  // GSTAT zuerst: bit1 = Treiberfehler, bit2 = Ladepumpen-Unterspannung.
  // Lesen loescht die Bits nicht - nur Schreiben tut das, und genau das wird
  // hier bewusst unterlassen.
  const uint8_t gL = (uint8_t)(tmcL.read(TMC_REG_GSTAT) & 0x07);
  const uint8_t gR = (uint8_t)(tmcR.read(TMC_REG_GSTAT) & 0x07);
  Serial.printf("GSTAT: L=0x%02X R=0x%02X%s%s\n", gL, gR,
    (gL & 0x02) ? "  <<< drv_err links" : "", (gR & 0x02) ? "  <<< drv_err rechts" : "");
  if ((gL | gR) & 0x04) {
    Serial.println("  ! Ladepumpen-Unterspannung (bit2) - laut Datenblatt schaltet der");
    Serial.println("    Treiber dann ab. VM und alle drei Abblockkondensatoren pruefen.");
  }
  // SG_RESULT steht ausschliesslich im Register 0x41. Die unteren 10 Bit von
  // DRV_STATUS sind KEIN SG_RESULT, sondern Temperatur, offene Last, Kurzschluss
  // und Ueberhitzung - sie wurden hier vorher faelschlich als "SG_RESULT"
  // ausgegeben.
  Serial.printf("SG_RESULT (Register 0x41): L=%u R=%u (Stall ab <= %u)\n",
                tmcL.SG_RESULT(), tmcR.SG_RESULT(), TMC_SGTHRS_DEFAULT * 2);
  // KEIN Readback von TPWMTHRS/TCOOLTHRS/SGTHRS mehr: das Datenblatt
  // kennzeichnet sie als "W" = nur schreibbar, der Readback liefert immer 0 und
  // hat hier eine Fehldiagnose ausgeloest ("Schwellen kommen nicht an").
  // Belegbar sind sie nur ueber IFCNT (zaehlt angenommene Schreibzugriffe).
  Serial.printf("Schwellen gesetzt: SpreadCycle ab %.0f mm/s, StallGuard ab %.0f mm/s "
                "(Konfiguration bestaetigt ueber IFCNT)\n",
                (double)TMC_SPREADCYCLE_AB_MM_S, (double)TMC_STALLGUARD_AB_MM_S);
  Serial.printf("DRV_STATUS: L=0x%08lX R=0x%08lX\n",
                (unsigned long)stL, (unsigned long)stR);
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
    mresCodeFor(s_microsteps), tmcGetCurrent(), tmcGetHold(),
    s_spread ? "SpreadCycle" : "StealthChop", s_enabled ? "EIN" : "AUS");
  // KEIN Readback von TPWMTHRS/TCOOLTHRS/SGTHRS: das Datenblatt kennzeichnet
  // sie als "W" = nur schreibbar, der Readback liefert immer 0 und hat hier
  // eine Fehldiagnose ausgeloest ("Schwellen kommen nicht an"). Belegbar sind
  // sie nur ueber IFCNT, das angenommene Schreibzugriffe zaehlt.
  Serial.printf("Schwellen gesetzt: SpreadCycle ab %.0f mm/s, StallGuard auswertbar ab %.0f mm/s "
                "(bestaetigt ueber IFCNT)\n",
                (double)TMC_SPREADCYCLE_AB_MM_S, (double)TMC_STALLGUARD_AB_MM_S);
  Serial.println("Messwerte selbst: Befehl MOT:DIAG (im Log und als MOT:MEAS: in der Web-UI).");
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

// Register-Dump + Schreibtest. Das ist die Messung, die geklaert hat, warum
// keine Schwellen ankamen: der TMC2209 nimmt Register-Schreibzugriffe nur
// an, wenn er EINGESCHALTET ist (EN low). Bei ausgeschaltetem Treiber sind
// alle Lesewerte die Auslieferungswerte (0) - der Treiber sieht dann
// harmlos aus und ist trotzdem völlig unkonfiguriert.
void tmcRegDump() {
  Serial.println();
  Serial.println("========== TMC REGISTER-DUMP ==========");
  // Adressen und Namen aus dem TMC2209-Datenblatt, NICHT aus TMCStepper. Die
  // Bibliothek vererbt die TMC2130-Namen und -Adressen und nennt 0x07
  // "FCLKCONF" - beim TMC2209 ist das FACTORY_CONF. Ebenso ist 0x05 beim
  // TMC2130 SLAVECONF, beim TMC2209 OTP_READ (nur lesbar, harmlos).
  // 0x04 (OTP_PROG) fehlt bewusst: das programmiert NVM-Zellen.
  struct { const char* n; uint8_t a; } const regs[] = {
    {"GCONF", 0x00}, {"GSTAT", 0x01}, {"IFCNT", 0x02}, {"NODECONF", 0x03},
    {"OTP_READ", 0x05}, {"IOIN", 0x06}, {"FACTORY_CONF", 0x07}, {"CHOPCONF", 0x08},
    {"SHORT_CONF", 0x09}, {"SHORT_STAT", 0x0A}, {"COOLCONF", 0x0B},
    {"IHOLD_IRUN", 0x10}, {"TPOWER", 0x11},
    {"TSTEP", 0x12}, {"TPWMTHRS", 0x13}, {"TCOOLTHRS", 0x14}, {"VACTUAL", 0x22},
    {"SGTHRS", 0x40}, {"SG_RESULT", 0x41}, {"COOLCONF2", 0x42},
    {"MSCNT", 0x6A}, {"DRV_STATUS", 0x6F},
  };
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    const uint32_t v = tmcL.read(regs[i].a);
    // WICHTIG: Das Datenblatt kennzeichnet 0x03, 0x08, 0x11, 0x13, 0x14 und 0x40
    // als "W" = nur schreibbar. Ein Readback liefert dort 0 und beweist NICHTS.
    // Genau daraus ist die Fehldiagnose "Schwellen kommen nicht an" entstanden.
    const bool writeOnly = (regs[i].a == 0x03 || regs[i].a == 0x08 || regs[i].a == 0x11 ||
                            regs[i].a == 0x13 || regs[i].a == 0x14 || regs[i].a == 0x40);
    Serial.printf("  0x%02X %-12s = 0x%08lX%s\n", regs[i].a, regs[i].n,
                  (unsigned long)v, writeOnly ? "   (nur schreibbar - Readback ohne Aussage)" : "");
  }
  // Schreibtest auf GCONF (0x00, R/W) - die einzige Adresse, an der wir ohne
  // Nebenwirkung kippen koennen. Der alte Test schrieb zusaetzlich auf 0x07
  // und nannte das FCLKCONF; das ist beim TMC2209 FACTORY_CONF, und er
  // manipulierte dort ein Feld mit unbekannter Bedeutung.
  Serial.println("--- Schreibtest: GCONF (0x00, R/W) ---");
  {
    TMC2209Diag& d = tmcL;
    const uint32_t g0 = d.read(TMC_REG_GCONF);
    const uint8_t if0 = (uint8_t)d.read(TMC_REG_IFCNT);
    // Genau ein Bit kippen und danach den GEMESSENEN Wert zurueckschreiben.
    // Vorher stand hier die Konstante 0x1C0 - bei aktivem SpreadCycle (0x1C4)
    // hat TMC:REG den Treiber damit still und dauerhaft auf StealthChop
    // zurueckgeschaltet.
    d.write(TMC_REG_GCONF, g0 ^ 0x00000100u);
    const uint32_t g1 = d.read(TMC_REG_GCONF);
    d.write(TMC_REG_GCONF, g0);              // gelesener Wert, nicht eine Konstante
    const uint32_t g2 = d.read(TMC_REG_GCONF);
    const uint8_t if1 = (uint8_t)d.read(TMC_REG_IFCNT);
    Serial.printf("  GCONF   : 0x%08lX -> 0x%08lX -> 0x%08lX  %s\n",
                  (unsigned long)g0, (unsigned long)g1, (unsigned long)g2,
                  (g1 != g0) ? "-> SCHREIBEN WIRKT" : "-> wird ignoriert");
    Serial.printf("  Zustand wiederhergestellt: %s\n",
                  (g2 == g0) ? "ja" : "NEIN - bitte neu konfigurieren (TMC:INIT)");
    // Zwei Schreibzugruege liegen zwischen den beiden IFCNT-Abfragen:
    // das Gekippte und das Zurueckgeschriebene.
    Serial.printf("  IFCNT   : %u -> %u (zaehlt nur Schreibzugruege; 2 Writes = +2 erwartet)\n",
                  if0, if1);
  }
  Serial.println("======================================");
  Serial.println();
}
