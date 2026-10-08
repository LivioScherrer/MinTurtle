// EduBot V2 - Motor-Modul (2x TMC2209 via UART + STEP/DIR)
// BEIDE Treiber am gemeinsamen Single-Wire-Bus. Die Zuordnung Motor ->
// Platine -> GPIO -> TMC-Adresse steht ausfuehrlich bei den #defines weiter
// unten (ab "Stepper-Pins") und ist dort die EINZIGE Wahrheit. Frueher
// widersprachen die Kommentare an dieser Stelle und bei TMC_ADDR_* einander
// und nannten die Zuordnung umgekehrt; bitte nur noch die #defines
// konsultieren und diese Kopfzeile nicht als Beleg verwenden.
//
// Getestete Hardware: PCB Edu_V2.2 mit TMC2209-LA direkt bestueckt.
// VIO = 3V3 (Xiao), VM = Motorspannung
// (ohne VM antwortet der Treiber nicht!).
// Verdrahtung PCB Edu_V2.2: ESP-TX --1k (R9)--> PDN-Netz, ESP-RX direkt an PDN
// (gemeinsam fuer BEIDE Treiber, kein Pull-up noetig).
//
// Feste Fahrgeschwindigkeit + sanfte Rampe (config.rampMs) fuer Anfahren und
// Abbremsen.
// Die Step-Puls-Erzeugung laeuft in einem eigenen FreeRTOS-Task auf Core 0,
// getaktet ueber einen 50us Hardware-Timer (esp_timer) + Task-Notification.
// Die Main-Loop (Core 1) setzt nur noch Ziel-Geschwindigkeiten/Positionen
// ueber die thread-sichere API unten. Direkte stepperX.run()/runSpeed()-
// Aufrufe sind ausserhalb von Motor.cpp nicht moeglich (Instanzen privat).
// TMC-UART wird nur in init/Befehlen/Diagnose angefasst, nie im Puls-Task.
#pragma once

#include <Arduino.h>
#include "Config.h"   // TMC_CURRENT_MA_MIN/MAX (Stromgrenze 200 mA)

// Stepper-Pins (Xiao ESP32-S3, PCB Edu_V2.2 verdrahtet - NICHT aendern!)
// IST-Verdrahtung dieses Roboters (am Fahrroboter geprueft, 2026-09):
// das Board an U2/J1 (Addr 0, D8/D2) treibt das RECHTE Rad,
// das Board an U4/J5 (Addr 1, D1/D0) treibt das LINKE Rad.
// Vorher stand hier die umgekehrte Annahme - Vorwaerts/Rueckwaerts fiel
// nicht auf (beide Raeder bekommen dasselbe Vorzeichen), aber jede
// Drehung auf der Stelle lief in die falsche Richtung.
// Deshalb ist "links" jetzt U4/J5 und "rechts" U2/J1. Beide Boards werden
// identisch konfiguriert, die Adressen sind reine Diagnose-Labels.
#define PIN_STEP_L   2   // D1 -> U4-STEP (TMC links)
#define PIN_DIR_L    1   // D0 -> U4-DIR  (TMC links)
#define PIN_STEP_R   7   // D8 -> U2-STEP (TMC rechts)
#define PIN_DIR_R    3   // D2 -> U2-DIR  (TMC rechts)
#define PIN_EN       4   // EN -> TMC-EN (gemeinsam, LOW aktiv)

// TMC2209-UART (gemeinsamer Single-Wire-Bus fuer BEIDE Treiber)
#define TMC_UART_RX  44  // D7 = RX-Netz (direkt an PDN)
#define TMC_UART_TX  43  // D6 = TX-Netz (ueber 1k an PDN, PCB R9)
#define TMC_BAUD     115200
#define TMC_R_SENSE  0.56f   // PCB Edu_V2.2: 560mOhm Sense (R6/R7/R11/R12)
#define TMC_ADDR_L   0b01    // links U4/J5: MS1=3V3, MS2=GND
#define TMC_ADDR_R   0b00    // rechts U2/J1: MS1=GND, MS2=GND

// Feste Fahrgeschwindigkeit. WICHTIG: nicht in Schritten fest verdrahtet.
// Die Schrittrate haengt an den
// Microsteps (16 Microsteps = 4x weniger Schritte pro Umdrehung als 64), ein
// fester Schritt/s-Wert wuerde den Roboter beim Umschalten der Microsteps
// 4x schneller oder langsamer machen. Stattdessen wird die gewuenschte
// Geschwindigkeit in mm/s gespeichert (config.driveSpeedMM, einstellbar per
// SPEED:<mm/s> und in der Web-UI) und hier zur Schrittrate umgerechnet.
//
// MAX_DRIVE_STEPS_S (Config.h) ist die Grenze des 50us-Motor-Tasks (20 kHz
// Takt); die Haelfte bleibt als Reserve fuer Jitter. Darueber wuerde der Motor
// einzelne Pulse verschlucken -> gelegentlich zu kurze Strecken.
// Laufzeitwert (Schritte/s) - in initSteppers() und nach jedem Wechsel der
// Microsteps bzw. der Geschwindigkeit neu berechnet (motorUpdateSpeeds()).
extern float FIXED_DRIVE_SPEED;

// Motor-Task-Abtastung: 50us-Takt (~20 kHz) >> 5 kHz Puls-Rate.
// Noetig, damit AccelStepper.runSpeed() bei 5000 steps/s (200us Periode)
// jeden Puls mit Reserve trifft (4x Oversampling, Jitter im us-Bereich).
constexpr uint32_t MOTOR_TICK_US = 50;

// Beschleunigungsrampe: config.rampMs = Zeit, die der Motor von 0 auf
// FIXED_DRIVE_SPEED braucht - und genauso lang wieder herunter. 0 = aus,
// dann laeuft alles sofort mit voller Geschwindigkeit.
// Die Werte selbst stehen in Config.h (RAMP_MS_MIN/MAX/DEFAULT) und werden
// per RAMP:<ms> in der Web-UI gesetzt. Bewusst mild gewaehlt: der Roboter
// reisst nicht mehr los, kurze Strecken (Rasterfeld) erreichen trotzdem fast
// die volle Geschwindigkeit. Einziger Knopf fuer "mehr/weniger Beschleunigung".
//
// Die Rampe gilt an ZWEI Stellen, mit derselben Bedeutung:
//   * Freies Fahren (WASD)  - als Software-Rampe im Motor-Task, weil es dort
//     kein Ziel gibt, vor dem man bremsen koennte. Anfahren UND Abbremsen
//     beim Loslassen der Taste.
//   * Positions-Moves       - als AccelStepper-Rampe (run()/setAcceleration),
//     die zusaetzlich vor dem Ziel abbremst.
// Harter Stopp (useRamp = false) melden ab: der STOP-Knopf, der
// Befehls-Timeout, TMC:EN:0, der Linienfolger und zeitlich begrenzte Fahrten.
// Als Anfahrzeit lesbar in der Web-UI: "Anfahren (ms)".

// TMC-Defaults (8HS11-0204S NEMA8: 0.2A Nennstrom, 24 Ohm, 8mH)
// Strom = exakt Nennstrom (mehr -> Ueberhitzung, max. 80 Grad Rise im Stillstand).
// Hold 16 = halber Haltestrom (kuehl + akkuschonend, reicht flach).
// StealthChop = leise (Klassenzimmer); per Web-UI auf SpreadCycle schaltbar.
//
// Die STROMGRENZEN (TMC_CURRENT_MA_MIN/MAX/DEFAULT) stehen bewusst in
// Config.h neben den anderen Motorkonstanten, weil sie sowohl der Befehl
// (tmcSetCurrent) als auch die NVS-Validierung (Config.cpp) brauchen und
// dort das "wir wirklich 200 mA maximal" durchgesetzt wird.
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

// Schrittrate aus config.driveSpeedMM + Microsteps + Rad berechnen und den
// AccelStepper mitteilen. Muss nach jeder Aenderung an Microsteps, Tempo,
// Rampe oder Geometrie aufgerufen werden - auch wenn sich nur config.rampMs
// geaendert hat: das Flag, das der Puls-Task liest, wird hier gesetzt.
void motorUpdateSpeeds();

// true, wenn Positionsfahrten mit Rampe laufen (config.rampMs > 0), false
// bei "sofort volle Geschwindigkeit". Fuer Befehls-Antworten und Anzeigen.
bool motorRampActive();

// Beide IST-Geschwindigkeiten der Freifahr-Rampe in Schritte/s (mit
// Vorzeichen), unter einem einzigen Mutex-Halt gelesen. Reine Diagnose:
// zeigt, was die Motoren TUN - DRV:spd= zeigt nur, was befohlen wurde.
void motorRampSpeeds(float& outL, float& outR);

// Start-Signal nach dem Booten: beide Motoren drehen kurz auf der Stelle
// (config.spinSpeed Schritte/s fuer config.spinMs), danach wieder Stillstand.
// Beide Werte kommen aus der NVS (Web-UI, SPIN:<s>,<ms>); ist einer davon 0,
// entfaellt die Drehung komplett - ohne Wartezeit. Bereiche stehen als
// SPIN_SPEED_* / SPIN_MS_* in Config.h.
void startupSpin();

// Geschwindigkeit setzen (steps/s, Vorzeichen = Richtung).
// Der Betrag wird IMMER auf FIXED_DRIVE_SPEED gelegt - egal was uebergeben wird.
//
// speed == 0 heisst STOPP, und useRamp entscheidet, WIE:
//   * useRamp == true  -> weiches Ausrollen ueber config.rampMs (Taste
//     losgelassen). Greift nur, wenn die Rampe an ist UND keine
//     Positionsfahrt laeuft - die hat einen festen Zielpunkt und kann nicht
//     weich abgebrochen werden.
//   * useRamp == false -> sofort Stillstand. fuer alles Sicherheitsrelevante:
//     Not-Aus, STOP-Knopf, Befehls-Timeout, TMC:EN:0. Hier darf die Bremse
//     NICHT config.rampMs lang dauern: bei Verbindungsausfall wuerde der
//     Roboter sonst noch sekundenlang rollen.
//
// useRamp == false beim Fahren (speed != 0) heisst: sofort die volle
// Geschwindigkeit, ohne Anfahren. Nur fuer Stellen, die wirklich auf
// SOFORTige Reaktion angewiesen sind - der Linienfolger und zeitlich
// begrenzte Fahrten (gameDrive), bei denen die Rampe die gewaehnte
// Fahrzeit auffressen wuerde.
// Thread-sicher: kann von jedem Core aus aufgerufen werden.
void setMotorSpeed(int motorIdx, float speed, bool useRamp = true);

// Beide Motoren sofort stoppen (auch laufende Positions-Fahrten abbrechen).
// Thread-sicher.
void stopMotors();

// Position-Move vorbereiten: 'physicalSteps' Schritte mit FIXED_DRIVE_SPEED
// fahren (vorzeichenkorrigiert: positiv = CW). Mit sanfter Rampe, sofern
// config.rampMs > 0 (AccelStepper run()/setAcceleration): der Motor
// beschleunigt am Anfang und bremst vor dem Ziel ab, die Schrittzahl bleibt
// exakt. Bei config.rampMs == 0 faehrt er sofort mit voller Geschwindigkeit.
// Das Ende wird intern automatisch erkannt (Rest-Zaehler == 0 -> Speed auf 0).
// Thread-sicher.
void prepPositionMove(int motorIdx, long physicalSteps);

// true solange mind. ein Motor noch Positionsschritte offen hat.
// Thread-sicher (liest volatile Zaehler).
bool motorStillRunning();

// motorSettle() ist ABSICHTLICH nicht mehr vorhanden. Es fuhr vor dem
// Programmstart zwei Vollschritte hin UND exakt zwei zurueck, "damit die
// Raeder auf derselben Microstep-Index stehen" - das ist winkelneutral und
// damit genau dann wirkungslos, wenn es gebraucht wuerde. Sie hat 30...200 ms
// bei jedem Start blockiert und konnte im Timeout-Fall den Programmstart
// komplett verhindern (PROG:NO_SETTLE). Siehe Motor.cpp.

// ---- TMC2209-Laufzeit-API (UART, BEIDE Treiber) ----
// Die Adressen stehen als TMC_ADDR_L / TMC_ADDR_R bei den #defines oben.
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
bool tmcSetCurrent(uint16_t mA);      // 100...200, false bei ungueltig
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
// UART-Bus-Diagnose (Echo/Reply/Ruhepegel, Baud-Messung).
// ACHTUNG: Es gab hier einmal eine Funktion tmcUartDiag(), die per
// Read-Request an die UART-Adressen 0..3 gesendet hat. Die war nie aufgerufen
// und ist entfernt worden: ein Read-Request an die Adresse 0 ist beim
// TMC2209 gleichzeitig ein Auto-Inkrement, wuerde die Adressen der beiden
// Treiber dauerhaft verschieben und damit die UART-Kommunikation zerstoeren.
// Ausserdem band sie Serial2 auf GPIO 43/44 und liess es dort.
// Die harmlose Variante heisst UARTNET -> tmcNetDiag() (nur Rx-Pegel, kein
// UART-Verkehr), UARTTEST ist absichtlich gesperrt.
void tmcNetDiag();
