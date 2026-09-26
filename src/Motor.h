// EduBot V2 - Motor-Modul (2x TMC2209 via UART + STEP/DIR)
// Stand: Neuaufbau auf EduV1.3-Basis - DRV8834 ersetzt durch TMC2209 (UART).
// BEIDE Treiber am gemeinsamen Single-Wire-Bus:
// links Adresse 0 (MS1=GND, MS2=GND), rechts Adresse 1 (MS1=3V3, MS2=GND).
//
// Getestete Hardware: PCB Edu_V2.2 mit TMC2209-LA direkt bestueckt.
// VIO = 3V3 (Xiao), VM = Motorspannung
// (ohne VM antwortet der Treiber nicht!).
// Verdrahtung PCB Edu_V2.2: ESP-TX --1k (R9)--> PDN-Netz, ESP-RX direkt an PDN (gemeinsam
// fuer BEIDE Treiber, kein Pull-up noetig). J1 (U2/Addr0) = linker Motor,
// J5 (U4/Addr1) = rechter Motor.
//
// Feste Geschwindigkeit - keine Beschleunigung/Rampe irgendwo im Code.
// Die Step-Puls-Erzeugung laeuft in einem eigenen FreeRTOS-Task auf Core 0,
// getaktet ueber einen 50us Hardware-Timer (esp_timer) + Task-Notification.
// Die Main-Loop (Core 1) setzt nur noch Ziel-Geschwindigkeiten/Positionen
// ueber die thread-sichere API unten. Direkte stepperX.run()/runSpeed()-
// Aufrufe sind ausserhalb von Motor.cpp nicht moeglich (Instanzen privat).
// TMC-UART wird nur in init/Befehlen/Diagnose angefasst, nie im Puls-Task.
#pragma once

#include <Arduino.h>

// Stepper-Pins (Xiao ESP32-S3, PCB Edu_V2.2 verdrahtet - NICHT aendern!)
// PCB-Ist: U2 (J1, Addr 0) = links an D2/D8, U4 (J5, Addr 1) = rechts an D0/D1
// STEP/DIR sind auf dem PCB jeweils gedreht gegen das Breadboard-Layout,
// darum hier kompensiert. J1 = links, J5 = rechts.
#define PIN_STEP_L   7   // D8 -> U2-STEP (TMC links)
#define PIN_DIR_L    3   // D2 -> U2-DIR (TMC links)
#define PIN_STEP_R   2   // D1 -> U4-STEP (TMC rechts)
#define PIN_DIR_R    1   // D0 -> U4-DIR (TMC rechts)
#define PIN_EN       4   // EN -> TMC-EN (gemeinsam, LOW aktiv)

// TMC2209-UART (gemeinsamer Single-Wire-Bus fuer BEIDE Treiber)
#define TMC_UART_RX  44  // D7 = RX-Netz (direkt an PDN)
#define TMC_UART_TX  43  // D6 = TX-Netz (ueber 1k an PDN, PCB R9)
#define TMC_BAUD     115200
#define TMC_R_SENSE  0.56f   // PCB Edu_V2.2: 560mOhm Sense (R6/R7/R11/R12)
#define TMC_ADDR_L   0b00    // links U2/J1: MS1=GND, MS2=GND
#define TMC_ADDR_R   0b01    // rechts U4/J5: MS1=3V3, MS2=GND

// Feste Fahrgeschwindigkeit in steps/s.
// Einzige Geschwindigkeits-Konstante im gesamten Projekt.
// 5000 steps/s @16 Microsteps = 1,56 U/s = 94 U/min = ca. 180 mm/s (Rad 37mm).
// Puls-Periode 200us -> 5x schneller als die alten 1000 steps/s (ca. 36 mm/s).
constexpr float FIXED_DRIVE_SPEED  = 5000.0f;

// Motor-Task-Abtastung: 50us-Takt (~20 kHz) >> 5 kHz Puls-Rate.
// Noetig, damit AccelStepper.runSpeed() bei 5000 steps/s (200us Periode)
// jeden Puls mit Reserve trifft (4x Oversampling, Jitter im us-Bereich).
constexpr uint32_t MOTOR_TICK_US = 50;

// Start-Signal nach dem Booten: Drehzahl und Dauer des StartupSpins.
constexpr float STARTUP_SPIN_SPEED = 6000.0f;
constexpr uint32_t STARTUP_SPIN_MS = 500;

// TMC-Defaults (8HS11-0204S NEMA8: 0.2A Nennstrom, 24 Ohm, 8mH)
// Strom = exakt Nennstrom (mehr -> Ueberhitzung, max. 80 Grad Rise im Stillstand).
// Hold 16 = halber Haltestrom (kuehl + akkuschonend, reicht auf ebenem Boden).
// StealthChop = leise (Klassenzimmer); per Web-UI auf SpreadCycle schaltbar.
constexpr uint16_t TMC_CURRENT_MA_DEFAULT = 200;  // 100...800 (RMS)
constexpr uint8_t  TMC_HOLD_DEFAULT       = 16;   // 0...31
constexpr uint16_t TMC_MICROSTEPS_DEFAULT = 16;   // 1,2,4,8,16,32,64,128,256

// Auto-Standby: keine Fahrt seit dieser Zeit + Motoren stehen
// -> Treiber ueber EN abschalten (0 Strom, kuehl). Jede echte Fahrt
// (setMotorSpeed/prepPositionMove) schaltet automatisch wieder ein.
constexpr uint32_t TMC_STANDBY_MS = 3000;

// Serie-Widerstand ESP-TX -> PDN-Netz (PCB Edu_V2.2: R9 = 1k, nur Doku)
constexpr uint32_t R9_SERIES_OHM = 1000;

// Initialisierung (Pins + Stepper-Defaults + Motor-Task auf Core 0 starten)
void initSteppers();

// Start-Signal nach dem Booten: beide Motoren drehen kurz
// (STARTUP_SPIN_SPEED fuer STARTUP_SPIN_MS), danach wieder Stillstand.
void startupSpin();

// Geschwindigkeit setzen (steps/s, Vorzeichen = Richtung).
// Der Betrag wird IMMER auf FIXED_DRIVE_SPEED gelegt - egal was uebergeben wird.
// speed == 0 bedeutet explizit STOPP.
// Thread-sicher: kann von jedem Core aus aufgerufen werden.
void setMotorSpeed(int motorIdx, float speed);

// Beide Motoren sofort stoppen (auch laufende Positions-Fahrten abbrechen).
// Thread-sicher.
void stopMotors();

// Position-Move vorbereiten: 'physicalSteps' Schritte mit FIXED_DRIVE_SPEED
// fahren (vorzeichenkorrigiert: positiv = CW). Keine Beschleunigung - der Motor
// startet sofort mit voller Geschwindigkeit und stoppt hart am Ziel. Das Ende
// wird intern automatisch erkannt (Rest-Zaehler == 0 -> Speed auf 0).
// Thread-sicher.
void prepPositionMove(int motorIdx, long physicalSteps);

// true solange mind. ein Motor noch Positionsschritte offen hat.
// Thread-sicher (liest volatile Zaehler).
bool motorStillRunning();

// ---- TMC2209-Laufzeit-API (UART, BEIDE Treiber: links Addr 0 + rechts Addr 1) ----
// Alle Set-Befehle wirken auf BEIDE Treiber gleichzeitig.
// Treiber ein/ausschalten (EN-Pin). disable stoppt auch die Motoren.
void tmcEnable();
void tmcDisable();
bool tmcIsEnabled();
// In loop() aufrufen: schaltet die Treiber nach TMC_STANDBY_MS Stillstand
// automatisch aus (EN). Naechste Fahrt schaltet wieder ein.
void tmcAutoStandbyTick();
// Konfiguration (Strom/Hold/Microsteps/Modus) erneut an BEIDE Treiber
// senden - z.B. nachträglich, wenn beim Booten noch kein VM anlag.
void tmcReinit();

// Strom / Haltestrom / Microsteps / Modus (fuer Diagnose + Tuning)
bool tmcSetCurrent(uint16_t mA);      // 100...800, false bei ungueltig
bool tmcSetHold(uint8_t ihold);       // 0...31
bool tmcSetMicrosteps(uint16_t ms);   // 1..256 (Liste), verifiziert per Readback
void tmcSetSpreadCycle(bool spread);  // true=SpreadCycle, false=StealthChop
bool tmcSpreadCycle();                // aktueller Modus
uint16_t tmcGetCurrent();
uint8_t tmcGetHold();
uint16_t tmcGetMicrosteps();
bool tmcMicrostepsConfirmed();        // true wenn Treiber den Wert per Readback bestaetigt hat

// Diagnose: UART-Verbindung + DRV_STATUS auf Serial ausgeben.
void tmcPrintStatus();
// Kompakte Statuszeile fuer BLE/Web-UI: "TMC:STAT:<okL>,<okR>,<verL>,<verR>,<mA>,<ms>,<spread>,<en>"
void tmcSendCompactStatus();
// Boot-Diagnose (einmalig in setup()): Verbindung, Version, IFCNT, MRES.
void tmcBootDiag();
// UART-Bus-Diagnose (Echo/Reply/Ruhepegel, Pin-Tausch-Test, Baud-Messung).
void tmcUartDiag();
// PDN-Netz-Diagnose (Pull-up-Vorhandenheit, Level-Divider-Test).
void tmcNetDiag();
