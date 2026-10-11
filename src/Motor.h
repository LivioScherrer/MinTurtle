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
#include "Config.h"   // MICROSTEPS_MAX, Geometrie, RAMPE

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
// Motorstrom FEST 200 mA - der Nennstrom des NEMA8 laut Datenblatt. Kein
// Regler, kein Befehl, kein NVS-Feld, kein UI-Feld. IHOLD ist der halbe
// Fahrstrom, IHOLDDELAY glaettet das Absenken.
//
// Vorher standen hier MIN/MAX/DEFAULT und TMC_CURRENT_MA_* in Config.h, und
// die Web-UI liess den Strom einstellen. Das war wirkungslos: die
// IRUN-Klemme auf 16 hat jede Einstellung von 100 bis 200 mA auf dieselben
// 210 mA gebracht. Ueber TMCStepper geschrieben wurde der Strom ohnehin nicht,
// sondern nur in die NVS - im Silizium stand die Klemme.
constexpr uint16_t TMC_CURRENT_MA_FIXED   = 200;  // mA RMS Sollstrom
constexpr uint8_t  TMC_IHOLDDELAY          = 4;    // Stufen je Stromabsenkung
constexpr uint16_t TMC_MICROSTEPS_DEFAULT = 16;   // 1,2,4,8,16,32 (Grenze: MICROSTEPS_MAX)
// Anzahl der Register-Schreibzugriffe in configureOne(): GCONF, CHOPCONF,
// IHOLD_IRUN, TPOWER, TPWMTHRS, TCOOLTHRS, SGTHRS, NODECONF. Der Erfolg wird
// ueber IFCNT belegt - zaehlt der Zaehler um mindestens so weit, sind alle
// angekommen. Stand vorher bei 7 statt 8, damit konnten drei Register
// stillschweigend ausfallen.
constexpr uint8_t  TMC_CFG_WRITE_COUNT     = 8;

// ---- Register-Adressen (TMC2209-Datenblatt Rev. 1.09, Abschnitt 5) ----
// ACHTUNG: TMCStepper adressiert den TMC2209 mit der Karte des TMC2130. Das
// fuehrt auf diesem Roboter zu einem stillen Totalausfall der Konfiguration,
// weil beim TMC2130 CHOPCONF auf 0x06 liegt - beim TMC2209 ist 0x06 aber das
// IOIN und damit nur lesbar. GEMESSEN (COM9, 2026-10): nach der
// Konfiguration waren TPWMTHRS, TCOOLTHRS, SGTHRS, TPOWER und CHOPCONF alle 0,
// waehrend GCONF (0x00, in beiden Karten identisch) korrekt ankam. Deshalb
// werden alle Register hier ueber die eigene, per Readback gepruefte
// Schreiberoutine gesetzt - die Adressen kommen aus dem Datenblatt, nicht aus
// der Bibliothek.
constexpr uint8_t TMC_REG_GCONF       = 0x00;   // nur 10 Bit: Modus/Flags
constexpr uint8_t TMC_REG_GSTAT       = 0x01;   // reset / drv_err / uv_cp
constexpr uint8_t TMC_REG_IFCNT       = 0x02;
constexpr uint8_t TMC_REG_NODECONF    = 0x03;   // SENDDELAY
constexpr uint8_t TMC_REG_IHOLD_IRUN   = 0x10;   // IHOLD/IRUN/IHOLDDELAY
constexpr uint8_t TMC_REG_IOIN        = 0x06;   // nur lesbar
constexpr uint8_t TMC_REG_CHOPCONF    = 0x08;   // NICHT 0x06 (das ist IOIN!)
constexpr uint8_t TMC_REG_TPOWER      = 0x11;   // TPOWERDOWN
constexpr uint8_t TMC_REG_TSTEP       = 0x12;   // gemessene STEP-Periode
constexpr uint8_t TMC_REG_TPWMTHRS    = 0x13;
constexpr uint8_t TMC_REG_TCOOLTHRS   = 0x14;
constexpr uint8_t TMC_REG_VACTUAL     = 0x22;   // gemessene Drehzahl
constexpr uint8_t TMC_REG_SGTHRS      = 0x40;
constexpr uint8_t TMC_REG_SG_RESULT   = 0x41;   // Drehmoment-Reserve
constexpr uint8_t TMC_REG_MSCNT       = 0x6A;   // Sequenzer-Mikroschrittzaehler
constexpr uint8_t TMC_REG_DRV_STATUS  = 0x6F;

// GCONF-Bitlagen (Datenblatt 5.5.1).
constexpr uint32_t GCONF_EN_SPREADCYCLE  = 1u << 2;
constexpr uint32_t GCONF_PDN_DISABLE     = 1u << 6;
constexpr uint32_t GCONF_MSTEP_REG_SELECT= 1u << 7;
constexpr uint32_t GCONF_MULTISTEP_FILT  = 1u << 8;
constexpr uint32_t GCONF_MASK            = 0x3FF;   // nur 10 Bit

// CHOPCONF-Bitlagen (Datenblatt 5.5.1). MRES-Codierung: %0000 = 256,
// %0001..%1000 = 128, 64, 32, 16, 8, 4, 2, FULLSTEP - identisch zu
// mresCodeFor() in Motor.cpp.
constexpr uint32_t CHOP_MRES_SHIFT = 24;
constexpr uint32_t CHOP_MRES_MASK  = 0x0F000000u;
constexpr uint32_t CHOP_INTPOL     = 1u << 28;   // auf 256 interpolieren
constexpr uint32_t CHOP_TOFF_SHIFT = 0;
constexpr uint32_t CHOP_TOFF_MASK  = 0x00000007u;

// DRV_STATUS-Bitfelder (Datenblatt 5.5.3). Wichtig fuer die Deutung: die
// unteren 10 Bit sind KEIN SG_RESULT, sondern Temperatur, offene Last und
// Kurzschluss - SG_RESULT steht ausschliesslich im Register 0x41.
constexpr uint32_t DRVST_OTPW      = 1u << 0;
constexpr uint32_t DRVST_OT        = 1u << 1;
constexpr uint32_t DRVST_S2GA      = 1u << 2;
constexpr uint32_t DRVST_S2GB      = 1u << 3;
constexpr uint32_t DRVST_S2VSA     = 1u << 4;
constexpr uint32_t DRVST_S2VSB     = 1u << 5;
constexpr uint32_t DRVST_OLA       = 1u << 6;
constexpr uint32_t DRVST_OLB       = 1u << 7;
constexpr uint32_t DRVST_STEALTH   = 1u << 30;
constexpr uint32_t DRVST_STST      = 1u << 31;
constexpr uint32_t DRVST_CS_ACTUAL_SHIFT = 19;
constexpr uint32_t DRVST_CS_ACTUAL_MASK  = 0xFFu;

// ---- TPWMTHRS: ab hier soll SpreadCycle das Ruder uebernehmen ----
// Vorher stand hier TPWMTHRS(0), und 0 heisst laut Datenblatt "Disabled":
// der Treiber blieb dadurch bei JEDER Geschwindigkeit in StealthChop, auch
// dort, wo StealthChop am wenigsten Drehmoment hat und Mikroschritte eher
// verlorengehen.
//
// Der Registerwert ist die STEP-Periode, bei der umgeschaltet wird, und
// bezieht sich laut Datenblatt immer auf 1/256 Schritt - er ist damit
// unabhaengig von der Microstep-Zahl und EIN Wert reicht fuer alle
// Microstep-Einstellungen:
//     TSTEP = 2^24 * U / (VACTUAL * 256)
//     VACTUAL * U = (Mikroschritte/Umdrehung) * U/s = 200*U*v/CIRC
//     => TSTEP = 2^24 * CIRC / (256 * 200 * v) = 2^24 * CIRC / (51200 * v)
// Gegenprobe mit der eigenen OTP-Tabelle des Herstellers: die Stufen
// 200/300/400/500/800/1200/4000 ergeben damit 180/120/90/72/45/30/9 mm/s -
// die Formel reproduziert die Werksvorgabe, ist also nicht nur hergeleitet.
// Mit dem Rad dieses Roboters (38 mm, DRIVE_TRIM 1.0101 -> 120,59 mm) folgt
//     TPWMTHRS  = 39514 / v[mm/s]     ->  658 bei 60 mm/s
//     TCOOLTHRS = 39514 / v[mm/s]     -> 2634 bei 15 mm/s
constexpr float   TMC_SPREADCYCLE_AB_MM_S = 60.0f;   // ab hier SpreadCycle

// ---- StallGuard scharf machen ----
// SGTHRS: "A stall is signaled with SG_RESULT <= SGTHRS*2". Der Wert ist
// anwendungsabhaengig (Motor, Strom, Last) - das Datenblatt gibt keine
// Vorgabe. Wichtig: NICHTS in der Firmware handelt auf diesem Flag, er wird
// nur gelesen und berichtet. Ein zu niedriger Wert kann deshalb keinen
// Schaden anrichten, und die Ausgabe zeigt beide SG_RESULT-Werte plus die
// Schwelle, damit der echte Wert am Roboter ablesbar ist.
constexpr uint8_t  TMC_SGTHRS_DEFAULT    = 100;    // Stall bei SG_RESULT <= 200
// TCOOLTHRS: unterhalb dieser Geschwindigkeit ist StallGuard nicht
// verlaesslich (das Datenblatt warnt ausdruecklich davor). 15 mm/s liegt
// unter dem Startbereich und ist bewusst konservativ gewaehlt.
constexpr float   TMC_STALLGUARD_AB_MM_S = 15.0f;

// ---- Diagnose-Takt ----
// Ein Register-Read auf dem gemeinsamen Single-Wire-Bus kostet ~2 ms
// (Datagramm + replyDelay = 2 ms, max_retries = 2). Deshalb wird NIE mehr als
// EIN Register pro Aufruf gelesen und die Messung ueber mehrere Loop-Durchlaenge
// verteilt. Ein Read im 50-us-Puls-Task wuerde 40 Ticks verschlucken - also
// genau die Schritte zerstoeren, die man messen will.
//
// Sicherheitsgrenze: die automatische Startmessung liest nur, solange die
// STEP-Periode laenger ist als ein Read. Bei 1500 Schritten/s ist die Periode
// noch 667 us, bei 4321 Schritten/s (150 mm/s, 16 Microsteps) nur noch 231 us -
// dann wuerde die UART-ISR dem Puls-Task genau die Takte wegnehmen, die man
// messen will. Also: frueh nach dem Anfahren messen, bei steigendem Tempo
// abbrechen. Der Lauf umfasst 16 Reads und passt in die ersten ~50 ms einer
// Rampe; reicht die Loop-Rate nicht, wird abgebrochen und das gemeldet.
constexpr float   MOT_DIAG_MIN_STEPS_S  = 20.0f;    // "faehrt wirklich"
constexpr float   MOT_DIAG_MAX_STEPS_S  = 1500.0f;  // ab hier wird abgebrochen

// ---- Schwellen der Messauswertung ----
// Bewusst getrennt von den Messwerten: was als "auffaellig" gilt, ist eine
// Bewertung und keine Messung - sie steht hier, damit sie im Log nachlesbar
// ist, statt in einer if()-Kette zu verschwinden.
constexpr uint16_t DIAG_SG_DIFF_MIN     = 100;   // von 0..1023 Drehmomentreserve
constexpr uint8_t  DIAG_VACTUAL_DIFF_PCT = 8;    // gemessene Drehzahl %
constexpr uint8_t  DIAG_TSTEP_DIFF_PCT  = 5;     // STEP-Periode %
constexpr uint8_t  DIAG_CS_DIFF_PCT     = 15;    // CS_ACTUAL %
constexpr uint16_t DIAG_MSCNT_TOLERANCE = 2;     // Mikroschritte

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
// Aktueller Schrittzaehler-Stand beider Stepper (AccelStepper
// currentPosition()), unter EINEM Mutex-Halt gelesen.
//
// Das ist die Messgroesse, die "Software oder Hardware?" entscheidet: der
// Zaehler zaeht JEDEN erzeugten Puls, unabhaengig davon, ob das Rad rollt.
// Wenn nach einer Fahrt beide Zaehler gleich weit gelaufen sind, aber der
// Roboter trotzdem driftet, kann es nicht die Schritterzeugung sein - dann
// stimmt Winkel, Reibung oder Spiel nicht zum Motor. Bei den gleichen Werten
// ist der Motor schuld, bei unterschiedlichen die Software.
void motorStepPositions(long& outL, long& outR);

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

// BEIDE Motoren unter EINEM Mutex-Halt setzen - fuer jede Stelle, die
// anfahren, bremsen oder steuern will.
//
// Warum das nicht nur Geschmack ist: vorher standen ueberall zwei Aufrufe
// hintereinander (setMotorSpeed(0,..) / setMotorSpeed(1,..)), und jeder
// Aufruf holt den Mutex erst recht. Der Motor-Task nimmt ihn alle 50 us; ein
// verschluckter zweiter Aufruf kostet dann die Differenz zwischen beiden
// Befehlen. Beide Raeder bekommen dadurch unterschiedliche Startzeitpunkte -
// bei 20 ms Versatz sind das bei 150 mm/s und 16 Microsteps rund 3 mm Weg
// und gut 2,5 Grad Gierwinkel, also auf einem Meter deutlich sichtbar.
// Mit einem Aufruf kann das nicht mehr passieren.
//
// Semantik identisch zu zwei einzelnen Aufrufen: useRamp wirkt auf beide.
void setMotorSpeeds(float speedL, float speedR, bool useRamp = true);

// Beide Motoren auf denselben Microstep-Index bringen (Befehl MOT:SYNC,
// wird ausserdem bei jedem Moduswechsel aufgerufen). true = es musste
// ausgeglichen werden.
//
// Der TMC2209 zaehlt die ausgefuehrten Mikroschritte in MSCNT (10 Bit,
// vorzeichenbehaftet: CW hoch, CCW runter). Zwei Motoren, die gleich viele
// Schritte in derselben Fahrtrichtung bekommen, haben deshalb |MSCNT|
// gleich, auch wenn ihre Zaehler wegen der Spiegelung entgegengesetzte
// Vorzeichen tragen. Weichen die BETRAEGE ab, laeuft die Sinusphase der
// beiden Motoren gegeneinander - das Rad mit dem kleineren Zaehler bleibt um
// die Differenz zurueck, und in einer Fahrtrichtung faellt das staerker auf
// als in der anderen.
//
// Bewusst NICHT die alte motorSettle(): die fuhr zwei Vollschritte hin UND
// exakt zwei zurueck. Das ist winkelneutral - am Ende steht jeder Rotor
// genauso versetzt wie vorher, der gewuenschte Effekt trat also nie ein,
// sie hat nur 30..200 ms blockiert und konnte den Programmstart ueber
// PROG:NO_SETTLE verhindern. Hier wird stattdessen genau die Differenz
// ausgeglichen, und zwar in Fahrtrichtung: hoechstens MOTOR_SYNC_MAX_STEPS
// Mikroschritte, also ein Weg von gut einem Millimeter.
//
// Nie blockierend fuer den Programmstart: der Aufruf wartet hoechstens
// MOTOR_SYNC_MAX_MS und laeuft nur im Stillstand.
bool motorSyncMicrosteps();
constexpr uint32_t MOTOR_SYNC_MAX_MS   = 250;    // Hoechstwartedauer
constexpr uint16_t MOTOR_SYNC_MAX_STEPS = 128;   // = 4 Vollschritte
constexpr uint16_t MOTOR_SYNC_TOLERANCE = 1;     // 1 Mikroschritt Toleranz
// Thread-sicher.
void stopMotors();

// NOT-STOPP: haelt die Motoren an, bis motorReleaseEmergency() folgt.
//
// Anders als stopMotors() ist das nicht rueckgaengig: ein nachfolgender
// Befehl kann die Motoren nicht wieder anwerfen. Genau das braucht der
// Tischkanten-Waechter - "an der Kante" darf durch den naechsten
// Fahrbefehl nicht sofort wieder aufgehoben werden.
//
// Blockiert NICHT auf dem Motor-Mutex: der wird mit 0 ms versucht und im
// Zweifel uebergangen. Grund: der Aufrufer ist der Sensor-Task, und der
// 50-us-Puls-Task haelt den Mutex alle 50 us. Ein Wartender wuerde die
// Motoren womoeglich erst stoppen, wenn sie schon weitergefahren sind. Die
// Motion ist ohnehin harmlos, wenn der letzte Puls ausbleibt.
void motorEmergencyStop();
// Nachfuehrlich: Not-Halt aufheben. Ruft motorEmergencyStop() selbst auf,
// damit nach einer Kante zwingend die Motoren erstmal still sind.
void motorReleaseEmergency();
bool motorEmergencyActive();

// Position-Move vorbereiten: 'physicalSteps' Schritte mit FIXED_DRIVE_SPEED
// fahren (vorzeichenkorrigiert: positiv = CW). Mit sanfter Rampe, sofern
// config.rampMs > 0 (AccelStepper run()/setAcceleration): der Motor
// beschleunigt am Anfang und bremst vor dem Ziel ab, die Schrittzahl bleibt
// exakt. Bei config.rampMs == 0 faehrt er sofort mit voller Geschwindigkeit.
// Das Ende wird intern automatisch erkannt (Rest-Zaehler == 0 -> Speed auf 0).
// Thread-sicher.
void prepPositionMove(int motorIdx, long physicalSteps);
// BEIDE Positionsfahrten unter EINEM Mutex-Halt starten - dasselbe Argument
// wie bei setMotorSpeeds(), und bei Drehungen sogar noch schlagkraeftiger:
// zwei Aufrufe mit Mutex dazwischen bedeuten, dass die beiden Raeder den
// Drehbeginn zu verschiedenen Zeitpunkten bekommen. Die Schrittzahlen
// stimmen zwar beide, das Drehmoment ist aber kurzzeitig einseitig - und der
// Roboter steht danach schief. Vorzeichen je Seite getrennt uebergeben.
void prepPositionMoves(long stepsL, long stepsR);

// true solange mind. ein Motor noch Positionsschritte offen hat.
// Thread-sicher (liest volatile Zaehler).
bool motorStillRunning();

// Treiber aufwecken (EN + ggf. UART-Rekonfiguration), VOR einem Mutex-Halt.
// Die vier Anfahr-Wrapper rufen das selbst.
void motorWakeDrivers();

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

// Microsteps / Modus (fuer Diagnose + Tuning).
// tmcSetCurrent() und tmcSetHold() gibt es nicht mehr - der Strom ist fest
// 200 mA (TMC_CURRENT_MA_FIXED) und wird in configureOne() gesetzt.
bool tmcSetMicrosteps(uint16_t ms);   // 1..32 (Liste), verifiziert per IFCNT-Beleg
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
// Alle relevanten Register beider Treiber ausgeben plus Schreibtest auf GCONF
// (das geklappte Bit wird sofort zurueckgeschrieben). Loest die Frage, ob der
// Treiber wirklich konfiguriert ist - aufgeraeumte Register lesen sich wie
// Auslieferungswerte und sind auf den ersten Blick unauffaellig.
void tmcRegDump();
// ---- Treiber-Messung: Schrittverlust und Schlupf BEOBACHTEN ----
// Reine Diagnose (MOT:DIAG/MOT:SPD?). Die Messung selbst regelt nichts nach:
// gelesen wird derselbe MSCNT-Zaehler, den auch motorSyncMicrosteps() nutzt.
// Lesbar sind: SG_RESULT (0x41), GSTAT (0x01), VACTUAL (0x22), TSTEP (0x12),
// MSCNT (0x6A) und DRV_STATUS (0x6F).
//
// Ein Register-Read kostet ~2 ms, deshalb liest motorDiagTick() genau EIN
// Register pro Aufruf und muss einmal pro Loop-Durchlauf aufgerufen werden
// (macht Edu_V2.cpp). Der Lauf ist danach fertig und die Auswertung steht im
// Log sowie als MOT:MEAS:-Zeile fuer die Web-UI zur Verfuegung.
// Messlauf anfordern (Befehl MOT:DIAG). false = Treiber aus oder ein Lauf
// laeuft bereits. Der Lauf selbst zieht sich ueber motorDiagTick() durch.
bool motorDiagRequest();
bool motorDiagRunning();
// Aus dem Loop, einmal pro Durchlauf: Messlauf fortschreiben (ein Register pro
// Aufruf) und am Ende automatisch auswerten + melden.
void motorDiagTick();
// Messung MITTEN in der Fahrt, auf Abruf (Befehl MOT:SPD?).
//
// Das ist die Messung, die bei dauerhaftem Drift weiterhilft: die
// automatische Startmessung ist nach ~50 ms vorbei, ein Drift zeigt sich
// aber ueber Sekunden. Gelesen werden nur VACTUAL (gemessene Drehzahl je
// Treiber) und MSCNT (Sequenzerposition) - 4 Reads, rund 8 ms.
//
// Warum das etwas ueber "Software oder Hardware" entscheidet: die Schritt-
// zaehler in der [DRV]-Zeile sagen, OB beide Raeder gleich viele Pulse
// bekommen. VACTUAL sagt, ob die Raeder daraus gleich viel WEG machen.
// Gleiche Pulse + unterschiedliches VACTUAL = Schlupf, Reibung oder
// schiefe Raeder. Gleiches VACTUAL + Drift = der Roboter wird von aussen
// gelenkt (Boden, Teppichkante, schiefe Achse).
//
// Bewusst NICHT automatisch: 8 ms Lesen mitten in der Fahrt sind eine
// echte Stoerung des 50-us-Puls-Tasks (guter Anlass, sie nur zu wollen
// statt sie zu planen).
// Teilstrom: false = nur eine kompakte Logzeile (selbsttaetiger 2-Sekunden-
// Takt waehrend der Fahrt), true = ausfuehrliche Auswertung plus MOT:SPD:.
bool motorDiagSteadySpeed(bool verbose = true);
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
