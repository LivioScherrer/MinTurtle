// EduBot V2 - Distanzsensor-Implementierung (VL53L0X)
#include "DistanceSensor.h"
#include "Config.h"       // obsDistMM / obsRelMM / obsOffsetMM (NVS-persistent)
#include "ColorSensor.h"  // SDA_PIN / SCL_PIN
#include "BLEManager.h"   // sendStatus + bleProcessEvents
#include "EspNow.h"       // espNowProcessEvents
#include "I2CBus.h"       // geteilter I2C-Bus mit dem Farbsensor
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <new>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static Adafruit_VL53L0X lox;
static bool s_ok = false;
static bool s_auto = true;
static bool s_continuous = false;  // true = Continuous-Modus nutzbar (bevorzugt)
static int16_t s_last = -1;     // letzter korrigierter Messwert
static int16_t s_lastRaw = -1;  // letzter Roh-Messwert (ohne Offset)
static bool s_blocked = false;
static unsigned long s_lastNotify = 0;
static unsigned long s_lastSingle = 0;  // Zeitpunkt der letzten Single-Shot-Messung
// Bus-Scan vom Boot festhalten, damit er auch nach der Initialisierung noch
// abfragbar ist (das Boot-Log ist fluechtig und Serial laeuft oft zu spaetzt).
static char s_bootScan[96] = "";
static unsigned long s_bootMs = 0;
static bool s_bootHad29 = false;
static bool s_bootHad30 = false;
// true = 0x29 gehoert exklusiv dem VL53L0X, Initialisierung dort ist sauber.
// Beim Boot: 0x29 war frei. Wird true, sobald der Sensor erfolgreich auf
// 0x29 zurueckgeschoben wurde (dann ist der TCS34725 dort nicht).
static bool s_ownDefaultAddr = false;
// Diagnose-Zaehler (nur lesend ueber distDiag(), kein Verhaltenseinfluss)
static uint32_t s_statRead = 0;      // erfolgreiche Range-Auswertungen
static uint32_t s_statNotReady = 0;  // isRangeComplete() == false
static uint32_t s_statStatusErr = 0; // Range-Status != Range Valid
static uint32_t s_statRangeErr = 0;  // Wert ausser 0..2000mm
static uint8_t  s_lastStatus = 0xFF; // letzter Range-Status (0xFF = noch keiner)

static bool probeAddr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Den Objektzustand des Sensors komplett neu aufbauen.
//
// WICHTIG: lox ist ein statisches globales Objekt. get_info_from_device()
// liest die NVM-Daten (ReferenceSpadCount/Type + Good-SPAD-Map) GENAU EIN MAL
// pro Instanz, gesichert ueber "ReadDataFromDeviceDone". Wurde der Wert
// einmal im Adresskonflikt mit dem TCS34725 verfaelscht gelesen, bleibt er fuer
// die gesamte Sitzung eingefroren -> StaticInit startet danach immer die
// Ref-SPAD-Kalibrierung und scheitert mit -50.
//
// ACHTUNG: KEINE Zuweisung lox = Adafruit_VL53L0X(); Die Klasse hat keinen
// eigenen Konstruktor, aber ein Default-Member "pMyDevice = &MyDevice".
// Der implizite Zuweisungsoperator kopiert DIESEN Self-Pointer - er zeigte
// danach auf die zerstoerte temporaere Instanz und lox stuertzte beim naechsten
// Zugriff ab (LoadProhibited-Crash, Boot-Loop). Placement-New baut das Objekt
// an derselben Adresse korrekt neu auf.
static void loxResetInstance() {
  lox.~Adafruit_VL53L0X();
  new (&lox) Adafruit_VL53L0X();
}

// Empirisch bestimmen, welcher Mess-Modus brauchbare Werte liefert.
// Continuous ist bevorzugt (non-blocking, kein I2C-Blockieren). Liefert er
// nur Range-Status != 0, weil StaticInit die Phasenkalibrierung nicht
// abschliessen konnte, auf Single-Shot zurueckfallen: readRange() schaltet
// den Sensor intern auf SINGLE_RANGING und initialisiert die Sequenz dabei
// neu - damit kommen gueltige Werte (auf dem Geraet nachgewiesen).
//
// WICHTIG: hier KEIN setMeasurementTimingBudgetMicroSeconds() aufrufen.
// Diese Funktion fuehrt intern perform_phase_calibration() aus und schreibt
// das Ergebnis in das NVM. Mit verdaechtigen Ausgangsdaten wird thereby eine
// kaputte Phasenkalibrierung dauerhaft abgelegt (Range-Status 4, PhaseFail).
// Der Library-Standard (33 ms) wird stattdessen verwendet.
static void distProbeModes() {
  uint8_t contOk = 0, singleOk = 0;

  // --- Continuous ---
  if (i2cBusLock(300)) {
    lox.startRangeContinuous(100);
    for (int i = 0; i < 5; i++) {
      vTaskDelay(pdMS_TO_TICKS(200));
      if (!lox.isRangeComplete()) continue;
      uint16_t mm = lox.readRangeResult();
      if (lox.readRangeStatus() == 0 && mm > 0 && mm < 2000) contOk++;
    }
    lox.stopRangeContinuous();
    i2cBusUnlock();
  }
  Serial.printf("[DIST] Modustest Continuous: %u/5 gueltige Messungen\n", (unsigned)contOk);

  // --- Single-Shot ---
  for (int i = 0; i < 5; i++) {
    vTaskDelay(pdMS_TO_TICKS(60));
    if (!i2cBusLock(DIST_SINGLE_LOCK_MS)) continue;
    uint16_t mm = lox.readRange();
    uint8_t st = lox.readRangeStatus();
    i2cBusUnlock();
    if (st == 0 && mm > 0 && mm < 2000) singleOk++;
  }
  Serial.printf("[DIST] Modustest Single-Shot: %u/5 gueltige Messungen\n", (unsigned)singleOk);

  s_continuous = (contOk >= 2);
  if (s_continuous) {
    if (i2cBusLock(300)) { lox.startRangeContinuous(100); i2cBusUnlock(); }
    Serial.println("[DIST] Mess-Modus: CONTINUOUS (non-blocking)");
  } else {
    s_lastSingle = millis();
    Serial.println("[DIST] Mess-Modus: SINGLE-SHOT (Fallback, blockierend ~alle 200ms)");
    Serial.println("[DIST] Hinweis: Ref-SPAD-Kalibrierung lief nicht (-50) - Messung trotzdem korrekt.");
  }
}

static bool distInitAtTarget(const char* why) {
  // 0x29 muss dem VL53L0X waehrend seiner Initialisierung allein gehoeren.
  // Das ist genau dann der Fall, wenn beim Boot nichts auf 0x29 antwortete
  // (Farbsensor nicht angeschlossen) oder der Sensor dorthin zurueckgeschoben
  // wurde - dann gehoert 0x29 exklusiv ihm (s_ownDefaultAddr).
  //
  // Ist 0x29 doppelt belegt (beide Sensoren am Bus), bleibt nur der Notpfad
  // ueber 0x30: DataInit laeuft dann zwar im Konflikt, der entscheidende
  // NVM-Zugriff aber bereits sauber auf 0x30. Das liefert nur ~10 %
  // brauchbare Messungen - siehe README. Ohne zusaetzliche Hardware
  // (I2C-Multiplexer oder getrennte Busse) laesst sich das nicht umgehen.
  const bool useDefaultAddr = s_ownDefaultAddr;
  const uint8_t initAddr = useDefaultAddr ? 0x29 : DIST_I2C_ADDR;
  Serial.printf("[DIST] Init ueber 0x%02X (%s)\n", initAddr,
                useDefaultAddr ? "0x29 exklusiv - sauber"
                               : "0x29 belegt - NVM-Zugriff auf 0x30");

  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(120);
      Serial.printf("[DIST] Init-Versuch %d (%s)\n", attempt + 1, why);
    }
    // Gesamte Init-Sequenz sperren: der Farbsensor-Task darf den Bus nicht
    // umschalten, waehrend DataInit/StaticInit laufen.
    if (!i2cBusLock(500)) {
      Serial.println("[DIST] I2C-Bus dauerhaft belegt -> Init abgebrochen");
      return false;
    }
    // Frische Instanz: get_info_from_device() liest die NVM-Daten nur EIN MAL
    // pro Objekt (ReadDataFromDeviceDone). Ein eingefrorener Verfaelschungs-
    // wert aus einem vorherigen Boot wuerde sonst wiederkehren.
    // Siehe loxResetInstance() - dort steht auch, warum KEINE Zuweisung
    // lox = Adafruit_VL53L0X() erlaubt ist (Self-Pointer -> Crash).
    loxResetInstance();
    bool begun = lox.begin(initAddr, false, &Wire);
    i2cBusUnlock();

    if (!begun) {
      // StaticInit kann an der Ref-SPAD-Kalibrierung scheitern (-50). DataInit
      // und der Adresswechsel sind dann aber schon erledigt, und der Sensor
      // misst im Single-Shot-Modus korrekt (auf Hardware nachgewiesen).
      Serial.println("[DIST] StaticInit unvollstaendig (Ref-SPAD-Kalibrierung -50)");
      Serial.println("[DIST] -> Mess-Modus wird empirisch bestimmt (Single-Shot-Fallback)");
      distProbeModes();
      s_ok = true;
      return true;
    }

    // Auf die Zieladresse 0x30 umziehen (nur noetig, wenn ueber 0x29 gebootet)
    if (useDefaultAddr) {
      if (!i2cBusLock(300)) return false;
      bool moved = lox.setAddress(DIST_I2C_ADDR);
      i2cBusUnlock();
      delay(50);
      if (!moved) {
        Serial.println("[DIST] Umadressierung auf 0x30 fehlgeschlagen");
        return false;
      }
    }

    // --- Normalfall: StaticInit erfolgreich ---
    // Timing-Budget hier bewusst NICHT setzen: der Aufruf schreibt die
    // Phasenkalibrierung ins NVM. Standard-Budget der Library (33 ms) reicht.
    if (!i2cBusLock(300)) return false;
    lox.stopRangeContinuous();
    delay(20);
    // Rueckgabe pruefen: ein still fehlgeschlagener Start waere sonst ein
    // "s_ok=true ohne Messung"-Zustand.
    bool started = lox.startRangeContinuous(100);
    i2cBusUnlock();
    if (!started) {
      Serial.println("[DIST] startRangeContinuous fehlgeschlagen -> Modustest");
      distProbeModes();
      s_ok = true;
      return true;
    }
    s_ok = true;
    s_continuous = true;
    Serial.println("[DIST] VL53L0X OK (Adresse 0x30, kontinuierlich 100ms)");
    return true;
  }
  return false;
}

// Alle Geraete auf dem I2C-Bus auflisten. Nur zur Diagnose beim Boot:
// zeigt unveraendert, was wirklich haengt (und ob ein Power-Cycle den
// VL53L0X wirklich auf 0x29 zurueckgesetzt hat).
static void distLogBusScan(const char* when) {
  char list[96] = {0};
  size_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (!probeAddr(a)) continue;
    if (n + 4 < sizeof(list)) {
      n += snprintf(list + n, sizeof(list) - n, " %02X", a);
    }
  }
  if (n) strlcpy(s_bootScan, list, sizeof(s_bootScan));
  Serial.printf("[DIST] Bus-Scan %s:%s\n", when, n ? list : " (leer)");
}

// Bus-Zustand VOR der Initialisierung ausgeben (beweist, ob ein Power-Cycle
// den VL53L0X wirklich auf 0x29 zurueckgesetzt hat).
void distBootScan() {
  Serial.println("---- Bus beim Boot (vor Distanz-Init) ----");
  Serial.printf("  Uhr seit Boot : %lu ms\n", (unsigned long)s_bootMs);
  Serial.printf("  0x29 (TCS34725)      : %s\n", s_bootHad29 ? "ACK" : "--");
  Serial.printf("  0x30 (VL53L0X)       : %s\n", s_bootHad30 ? "ACK" : "--");
  Serial.printf("  alle Geraete         : %s\n", s_bootScan[0] ? s_bootScan : "(leer)");
  if (s_bootHad30) {
    Serial.println("  => Der VL53L0X sass BEREITS auf 0x30.");
    Serial.println("     Er war also NICHT stromlos - das NVM wurde nicht zurueckgesetzt.");
  } else {
    Serial.println("  => 0x30 frei: Der VL53L0X sass auf 0x29 (frisch/power-cycled).");
  }
  Serial.println("  Jetzt:");
  Serial.printf("    0x29 : %s\n", probeAddr(0x29) ? "ACK" : "--");
  Serial.printf("    0x30 : %s\n", probeAddr(DIST_I2C_ADDR) ? "ACK" : "--");
  Serial.println("---------------------------------------");
}

void distSetup() {
  i2cBusInit();
  // Wire muss laufen (wird hier gestartet falls noetig)
  Wire.begin(SDA_PIN, SCL_PIN);
  s_bootMs = millis();

  // BEVOR irgendetwas initialisiert oder verschoben wird: Bus protokollieren.
  //
  // ENTSCHEIDEND: Der Adresskonflikt am Start ist der Kern des Problems.
  // Der VL53L0X kann NUR initialisiert werden, wenn er auf 0x29 ALLEINE
  // antwortet - die DataInit- und vor allem die NVM-Lesevorgaenge der
  // Library laufen ueber die Default-Adresse 0x29. Antwortet dort zusaetzlich
  // der TCS34725, kommen verfaelschte Registerwerte zurueck, der
  // Ref-SPAD-Parameter wird ungueltig und StaticInit scheitert mit -50.
  //
  // Ist 0x29 zu BOOT-Zeitpunkt frei (Farbsensor nicht angeschlossen bzw. noch
  // nicht aktiv), ist das der Normalfall fuer einen frischen Sensor.
  s_bootHad29 = probeAddr(0x29);
  s_bootHad30 = probeAddr(DIST_I2C_ADDR);
  distLogBusScan("vor Init");
  Serial.printf("[DIST] 0x29 zum Boot: %s -> %s\n",
                s_bootHad29 ? "belegt" : "frei (nur VL53L0X)",
                s_bootHad29 ? "Initialisierung auf 0x29 riskant"
                            : "Initialisierung auf 0x29 sauber moeglich");

  // Faellt beim Boot NICHTS auf 0x29, gehoert die Adresse exklusiv dem
  // VL53L0X (der Farbsensor ist dann nicht angeschlossen bzw. antwortet nicht).
  s_ownDefaultAddr = !s_bootHad29;

  // Ist der Sensor schon auf 0x30? (vorherige Session, MCU-Reset ohne
  // Power-Cycle - die Adresse bleibt im Sensor erhalten)
  //
  // WICHTIG: Nur zurueckschieben, wenn 0x29 danach nicht dauerhaft blockiert
  // wird. Liegt dort der TCS34725, wuerde der VL53L0X den Farbsensor stoeren.
  // Dann besser den 0x30-Pfad (begin mit Zieladresse) versuchen.
  if (probeAddr(DIST_I2C_ADDR) && s_ownDefaultAddr) {
    // Der Sensor sass noch auf 0x30 (vorherige Sitzung, kein Power-Cycle) und
    // 0x29 ist exklusiv. Dann ist der Standardpfad (Init ueber 0x29) moeglich
    // und der NVM-Zugriff laeuft sauber. Also zurueck auf 0x29 schieben.
    Serial.println("[DIST] 0x30 belegt, 0x29 exklusiv -> Sensor zurueck auf 0x29");
    if (i2cBusLock(500)) {
      loxResetInstance();
      lox.begin(DIST_I2C_ADDR, false, &Wire);
      delay(50);
      bool moved = lox.setAddress(0x29);
      i2cBusUnlock();
      delay(50);
      if (moved) {
        Serial.println("[DIST] Rueckschieben auf 0x29 ok");
      } else {
        Serial.println("[DIST] Rueckschieben fehlgeschlagen -> 0x30-Pfad");
        s_ownDefaultAddr = false;
      }
    }
  } else if (probeAddr(DIST_I2C_ADDR)) {
    Serial.println("[DIST] 0x30 belegt, 0x29 doppelt belegt -> Rueckschieben waere riskant");
    Serial.println("[DIST] -> init mit Zieladresse 0x30 versuchen");
  }

  if (!distInitAtTarget("frisch")) {
    s_ok = false;
    Serial.println("[DIST] VL53L0X nicht initialisierbar.");
    Serial.println("[DIST] -> Kompletten Strom abziehen (USB) und neu booten.");
    Serial.println("[DIST]    Mit DISTDEBUG jeden Init-Schritt nachvollziehen.");
    return;
  }

  // Verifizieren: 0x30 muss antworten UND der Farbsensor bleibt auf 0x29.
  delay(50);
  if (!probeAddr(DIST_I2C_ADDR)) {
    Serial.println("[DIST] 0x30 antwortet NICHT nach Init -> s_ok=false");
    s_ok = false;
    return;
  }
  // Der VL53L0X muss am Ende IMMER auf 0x30 liegen: haette er 0x29, wuerde er
  // dauerhaft mit dem TCS34725 kollidieren und der Farbsensor waere blind.
  if (!probeAddr(0x29)) {
    Serial.println("[DIST] WARNUNG: 0x29 antwortet nicht - Farbsensor fehlt?");
  }
  Serial.println("[DIST] Adressen: VL53L0X=0x30, TCS34725=0x29 (Farbsensor folgt in colorSetup)");
  distLogBusScan("nach Init");
}

// Diagnose: jeden Schritt des Init-Ablaufs einzeln nachvollziehen und den
// Library-Debug-Modus (begin(addr, debug=true)) verwenden. Zeigt, an welcher
// Phase die Initialisierung scheitert (Version/StaticInit/Adresswechsel).
// ACHTUNG: veraendert I2C-Zustand - nur zur Fehlersuche.
void distDebugInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("---- VL53L0X Init-Debug ----");
  Serial.printf("  SDA=GPIO%d SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
  Serial.printf("  0x29: %s   0x30: %s\n",
                probeAddr(0x29) ? "ACK" : "--", probeAddr(DIST_I2C_ADDR) ? "ACK" : "--");

  // Schritt 1: lox.begin(0x30) - der korrekte Aufruf fuer einen Sensor, der
  // auf 0x29 steht (DataInit @ 0x29, StaticInit danach @ 0x30).
  Serial.println("  -- lox.begin(0x30, debug=true) [Bus gesperrt, frische Instanz] --");
  i2cBusLock(500);
    loxResetInstance();
  bool ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
  Serial.printf("  lox.begin(0x30) -> %s\n", ok ? "OK" : "FEHLER");

  if (!ok && probeAddr(DIST_I2C_ADDR)) {
    // Schritt 2: zurueck auf 0x29 schieben (DataInit war fuer den Sensor
    // unerreichbar, weil er schon auf 0x30 sass).
    Serial.println("  -- Rueckschieben auf 0x29, dann erneut begin(0x30) --");
    lox.setAddress(0x29);
    delay(60);
    Serial.printf("  0x29: %s   0x30: %s\n",
                  probeAddr(0x29) ? "ACK" : "--", probeAddr(DIST_I2C_ADDR) ? "ACK" : "--");
    ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
    Serial.printf("  lox.begin(0x30) erneut -> %s\n", ok ? "OK" : "FEHLER");
  }

  if (ok) {    // KEIN Timing-Budget setzen: der Aufruf schreibt die Phasenkalibrierung
    // ins NVM des Sensors und ist die wahrscheinlichste Ursache fuer
    // Range-Status 4 (PhaseFail). Library-Standard (33 ms) verwenden.
    bool started = lox.startRangeContinuous(100);
    Serial.printf("  startRangeContinuous(100) -> %s  (Budget: Library-Standard)\n",
                  started ? "OK" : "FEHLER");
    for (int i = 0; i < 6; i++) {
      vTaskDelay(pdMS_TO_TICKS(300));
      bool ready = lox.isRangeComplete();
      uint16_t mm = ready ? lox.readRangeResult() : 0xffff;
      uint8_t st = lox.readRangeStatus();
      Serial.printf("    isRangeComplete=%d  rangeStatus=%u  mm=%u\n",
                    (int)ready, (unsigned)st, (unsigned)mm);
    }
  } else {
    // StaticInit scheiterte an der Ref-SPAD-Kalibrierung (-50). Die Hardware
    // kann aber messen (Register 0x14 Bit0 = newData). Frage: Laeuft der
    // Continuous-Modus trotzdem? Wenn ja, koennen wir die Messung betreiben
    // und die fehlgeschlagene Kalibrierung tolerieren.
    Serial.println("  -- Continuous-Modus trotz fehlgeschlagener StaticInit testen --");
    // KEIN Timing-Budget: schreibt die Phasenkalibrierung ins NVM.
    bool started = lox.startRangeContinuous(100);
    Serial.printf("  startRangeContinuous(100) -> %s\n", started ? "OK" : "FEHLER");
    int valid = 0;
    for (int i = 0; i < 10; i++) {
      vTaskDelay(pdMS_TO_TICKS(200));
      bool ready = lox.isRangeComplete();
      if (!ready) { Serial.printf("    [%d] not ready\n", i); continue; }
      uint16_t mm = lox.readRangeResult();
      uint8_t st = lox.readRangeStatus();
      if (st == 0 && mm > 0 && mm < 2000) valid++;
      Serial.printf("    [%d] ready=%d status=%u mm=%u\n", i, (int)ready, (unsigned)st, (unsigned)mm);
    }
    Serial.printf("  -> %d/10 gueltige Messungen im Continuous-Modus\n", valid);
    if (valid >= 3) {
      Serial.println("  -> Continuous-Modus BENUTZBAR trotz StaticInit-Fehler!");
      s_ok = true;
    }

    Serial.println("  -- Rohes Statusregister 0x14: Bit0 = neue Messung, Bit3..6 = Status --");
    uint8_t raw = 0;
    Wire.beginTransmission(DIST_I2C_ADDR);
    Wire.write(0x14);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((int)DIST_I2C_ADDR, (size_t)1) == 1) {
      raw = Wire.read();
    }
    Serial.printf("  Register 0x14 (Rohwert) = 0x%02X  -> Bit0(newData)=%d  Status=%d\n",
                  raw, raw & 0x01, (raw >> 3) & 0x07);

    // NVM-Ref-SPAD-Daten direkt auslesen. StaticInit nimmt den Umweg ueber
    // perform_ref_spad_management (die -50 ausloest), wenn
    //   type > 1  ODER  (type==1 UND count>32)  ODER  (type==0 UND count>12)
    // Diese Werte entscheiden also, OB kalibriert werden muss.
    uint8_t spad[4] = {0, 0, 0, 0};
    Wire.beginTransmission(DIST_I2C_ADDR);
    Wire.write((uint8_t)0x80); Wire.write((uint8_t)0x01);
    (void)Wire.endTransmission();

    Wire.beginTransmission(DIST_I2C_ADDR);
    Wire.write((uint8_t)0x94); Wire.write((uint8_t)0x6b);
    (void)Wire.endTransmission();

    Wire.beginTransmission(DIST_I2C_ADDR);
    Wire.write((uint8_t)0x84); Wire.write((uint8_t)0x01);
    (void)Wire.endTransmission();

    Wire.beginTransmission(DIST_I2C_ADDR);
    Wire.write((uint8_t)0x90);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((int)DIST_I2C_ADDR, (size_t)4) == 4) {
      for (int i = 0; i < 4; i++) spad[i] = Wire.read();
    }
    uint32_t tmp = ((uint32_t)spad[0] << 24) | ((uint32_t)spad[1] << 16) |
                   ((uint32_t)spad[2] << 8) | (uint32_t)spad[3];
    uint8_t nvmCount = (uint8_t)((tmp >> 8) & 0x7F);
    uint8_t nvmType  = (uint8_t)((tmp >> 15) & 0x01);
    Serial.printf("  NVM Ref-SPAD: DWord=0x%08X  count=%u  type=%u\n",
                  (unsigned)tmp, (unsigned)nvmCount, (unsigned)nvmType);
    bool needCal = (nvmType > 1) || (nvmType == 1 && nvmCount > 32) ||
                   (nvmType == 0 && nvmCount > 12);
    Serial.printf("  -> %s\n", needCal
                  ? "NVM-Wert ungueltig => StaticInit startet die Ref-SPAD-Kalibrierung (-50)"
                  : "NVM-Wert gueltig => Kalibrierung waere nicht noetig");

    // Entscheidender Test: Ist der Sensor nach dem NVM-Zugriff noch unter
    // 0x30 erreichbar? Die ST-API schreibt in get_info_from_device ueber
    // Register 0xFF (I2C_SLAVE_CONFIG) zwischenzeitlich Adressen 0x01/0x06/0x07.
    // Faellt 0x30 danach aus, war der Read ein NACK und die Library liefert
    // trotzdem 0xFF-Muell zurueck (siehe VL53L0X_read_multi: keine
    // Fehlerpruefung) - dann waere die Ursache ein Adresswechsel, nicht die
    // Hardware.
    Serial.printf("  Nach NVM-Zugriff: 0x29 %s / 0x30 %s (0x01 %s / 0x06 %s / 0x07 %s)\n",
                  probeAddr(0x29) ? "ACK" : "--",
                  probeAddr(DIST_I2C_ADDR) ? "ACK" : "--",
                  probeAddr(0x01) ? "ACK" : "--",
                  probeAddr(0x06) ? "ACK" : "--",
                  probeAddr(0x07) ? "ACK" : "--");
  }
  i2cBusUnlock();
  Serial.println("---------------------------");
}

bool distOk() { return s_ok; }

// Roh-Messung ohne Offset-Korrektur (fuer Kalibrierung + interne Nutzung).
//
// WICHTIG: nicht readRange() verwenden. readRange() laeuft ueber
// PerformSingleRangingMeasurement() -> SetDeviceMode(SINGLE_RANGING) und
// schaltet den Sensor damit bei JEDER Messung aus dem Continuous-Modus
// heraus. Im Dauerbetrieb (startRangeContinuous) fuehrt das zu
// fehlenden/veralteten Werten. readRangeResult() liest nur die
// bereits fertige Messung aus (GetRangingMeasurementData +
// ClearInterruptMask) und laesst den Continuous-Modus unberuehrt.
// Roh-Messung ohne Offset-Korrektur.
//
// ZWEI Modi, weil die Ref-SPAD-Kalibrierung des VL53L0X in dieser
// Konfiguration nicht durchlaeuft (StaticInit -> -50 REF_SPAD_INIT, siehe
// distInitAtTarget). Die Hardware misst trotzdem korrekt:
//
//  * Continuous-Modus (bevorzugt): lox.isRangeComplete() + readRangeResult().
//    Funktioniert nur, wenn StaticInit die Phasenkalibrierung abgeschlossen
//    hat - liefert sonst durchgehend Range-Status 4 (PhaseFail).
//  * Single-Shot-Fallback: lox.readRange() schaltet den Sensor intern auf
//    SINGLE_RANGING und initialisiert die Sequenz dabei neu - damit kommen
//    gueltige Werte (Status 0) zustande.
//
// s_continuous wird beim Start einmal empirisch bestimmt (distProbeModes).
static int16_t readRawMM() {
  if (!s_ok) return -1;

  if (s_continuous) {
    if (!i2cBusLock(30)) return s_lastRaw;
    bool ready = lox.isRangeComplete();
    if (!ready) {
      i2cBusUnlock();
      s_statNotReady++;
      return s_lastRaw;
    }
    uint16_t mm = lox.readRangeResult();
    uint8_t st = lox.readRangeStatus();
    i2cBusUnlock();
    s_lastStatus = st;
    if (st != 0) { s_statStatusErr++; return -1; }
    if (mm == 0 || mm > 2000) { s_statRangeErr++; return -1; }
    s_statRead++;
    s_lastRaw = (int16_t)mm;
    return s_lastRaw;
  }

  // Single-Shot: blockierend, daher mit Mindestabstand und mit Timeout-
  // Reserve. readRange() wartet selbst auf das Messende.
  //
  // RETRIES: der Sensor liefert je nach Geometrie/Umgebungslicht haeufig
  // Range-Status 4 (PhaseFail) oder andere Fehlstatus - das ist bei
  // fehlgeschlagener Ref-SPAD-Kalibrierung normal. Da Status 0 zuverlaessig
  // klare Werte liefert, wiederholen wir die Messung ein paar Mal, statt den
  // Fehler zu melden. Mit Hindernis direkt vor dem Roboter ist die
  // Trefferquote deutlich hoeher als im freien Raum.
  const uint32_t now = millis();
  if (now - s_lastSingle < DIST_SINGLE_PERIOD_MS) return s_lastRaw;
  s_lastSingle = now;
  if (!i2cBusLock(DIST_SINGLE_LOCK_MS * DIST_SINGLE_RETRIES)) return s_lastRaw;

  uint16_t mm = 0;
  uint8_t st = 0xFF;
  for (int attempt = 0; attempt < DIST_SINGLE_RETRIES; attempt++) {
    mm = lox.readRange();
    st = lox.readRangeStatus();
    if (st == 0 && mm > 0 && mm < 2000) break;   // gueltiger Wert -> fertig
    s_statStatusErr++;
  }
  i2cBusUnlock();
  s_lastStatus = st;
  if (st != 0) return -1;
  if (mm == 0 || mm > 2000) { s_statRangeErr++; return -1; }
  s_statRead++;
  s_lastRaw = (int16_t)mm;
  return s_lastRaw;
}

int16_t distMeasureMM() {
  int16_t raw = readRawMM();
  if (raw < 0) return -1;
  int16_t c = raw - config.obsOffsetMM;
  if (c < 0) c = 0;
  s_last = c;
  return s_last;
}

bool distCalibrateOffset(int16_t trueMM) {
  if (!s_ok) return false;
  // Caches verwerfen (alte Korrektur darf nicht einflissen)
  s_last = -1; s_lastRaw = -1;
  long sum = 0;
  int n = 0;
  uint32_t start = millis();
  while (n < 10 && millis() - start < 8000) {
    int16_t r = readRawMM();
    if (r > 0) { sum += r; n++; }
    bleProcessEvents();
    espNowProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  if (n < 5) return false;
  int16_t avg = (int16_t)(sum / n);
  // Plausibilitaet: Mittel muss im linearen Bereich nahe am Ziel liegen
  if (abs(avg - trueMM) > 150) return false;
  int16_t off = avg - trueMM;
  if (off < -200 || off > 200) return false;
  config.obsOffsetMM = off;
  configSaveInt("obsOff", off);
  s_last = -1; s_lastRaw = -1;
  Serial.printf("[DIST] Offset-Kalibrierung: Ziel=%d roh=%d -> offset=%d\n",
                trueMM, avg, off);
  return true;
}

int16_t distOffsetMM() { return config.obsOffsetMM; }

void distClearOffset() {
  config.obsOffsetMM = 0;
  configSaveInt("obsOff", 0);
  s_last = -1; s_lastRaw = -1;
}

int16_t distLastMM() { return s_last; }

bool distBlocked() {
  if (!s_ok) return false;
  int16_t mm = s_last;
  if (mm < 0) { s_blocked = false; return false; }  // kein Wert = frei
  if (s_blocked) {
    if (mm > config.obsRelMM) s_blocked = false;
  } else {
    if (mm < config.obsDistMM) s_blocked = true;
  }
  return s_blocked;
}

void distTick() {
  if (!s_ok || !s_auto) return;
  // Continuous-Modus: nur melden, wenn wirklich eine neue Messung fertig ist.
  // Single-Shot: readRawMM() erledigt die Fertig-Pruefung selbst (es ist
  // blockierend und deshalb intern gedrosselt) - hier nicht zusaetzlich
  // isRangeComplete() fragen, sonst kaeme im Single-Shot-Modus nie eine
  // Messung zustande.
  if (s_continuous) {
    if (!i2cBusLock(20)) return;
    bool ready = lox.isRangeComplete();
    i2cBusUnlock();
    if (!ready) return;
  }
  int16_t mm = distMeasureMM();
  unsigned long now = millis();
  if (now - s_lastNotify >= DIST_NOTIFY_MS) {
    s_lastNotify = now;
    char msg[24];
    if (mm >= 0) snprintf(msg, sizeof(msg), "DIST:%d", mm);
    else snprintf(msg, sizeof(msg), "DIST:--");
    sendStatus(msg);
  }
}

void distSetAuto(bool on) { s_auto = on; }
bool distAuto() { return s_auto; }

// Diagnose: Kompaktstatus als eine Zeile ( fuer BLE/Web-UI ).
// Format: DIST:DIAG:<ok>,<lastMM>,<lastRaw>,<status>,<off>,<read>,<notReady>,<statusErr>,<rangeErr>
// Kein Verhaltenseinfluss - reiner Lesezugriff auf die internen Zaehler.
void distDiag(char* buf, size_t bufLen) {
  snprintf(buf, bufLen, "DIST:DIAG:%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu",
           s_ok ? 1 : 0,
           (int)s_last, (int)s_lastRaw,
           (int)s_lastStatus,
           (int)config.obsOffsetMM,
           (unsigned long)s_statRead,
           (unsigned long)s_statNotReady,
           (unsigned long)s_statStatusErr,
           (unsigned long)s_statRangeErr);
}

// Diagnose auf Serial ( ausfuehrlich, inkl. Range-Status-Text + I2C-Scan ).
void distDiagPrint() {
  static const char* kRangeStatus[7] = {
    "RangeValid", "SigmaFail", "SignalFail", "MinRangeFail",
    "PhaseFail", "HardwareFail", "NoUpdate"
  };
  const char* stName = (s_lastStatus == 0xFF)
                         ? "nie-gemessen"
                         : ((s_lastStatus < 7) ? kRangeStatus[s_lastStatus]
                                               : "Unknown");

  Serial.println("---- VL53L0X Diagnose ----");
  Serial.printf("  s_ok           : %s\n", s_ok ? "ja" : "NEIN");
  Serial.printf("  I2C-Adresse    : 0x%02X (soll 0x%02X)\n", DIST_I2C_ADDR, DIST_I2C_ADDR);
  Serial.printf("  0x29 antwortet : %s (TCS34725 erwartet)\n",
                probeAddr(0x29) ? "ja" : "nein");
  Serial.printf("  0x30 antwortet : %s (VL53L0X erwartet)\n",
                probeAddr(DIST_I2C_ADDR) ? "ja" : "nein");
  Serial.printf("  Modus          : %s\n", s_continuous
                ? "CONTINUOUS_TIMED_RANGING (non-blocking)"
                : "SINGLE-SHOT (Fallback, blockierend, ~alle 200ms)");
  Serial.printf("  letzter Wert   : %d mm (roh %d mm, Offset %d mm)\n",
                (int)s_last, (int)s_lastRaw, (int)config.obsOffsetMM);
  Serial.printf("  Range-Status   : %u (%s)\n", (unsigned)s_lastStatus, stName);
  Serial.printf("  Auto-Meldung   : %s\n", s_auto ? "an" : "aus");
  Serial.printf("  Hindernis      : %s (stopp<%dmm, frei>%dmm)\n",
                s_blocked ? "BLOCKIERT" : "frei",
                (int)config.obsDistMM, (int)config.obsRelMM);
  Serial.println("  --- Zaehler (seit Boot) ---");
  Serial.printf("  gueltig        : %lu\n", (unsigned long)s_statRead);
  Serial.printf("  not ready      : %lu\n", (unsigned long)s_statNotReady);
  Serial.printf("  Status-Fehler  : %lu  (Sensor laeuft, Messung unbrauchbar)\n",
                (unsigned long)s_statStatusErr);
  Serial.printf("  Wert-Fehler    : %lu  (0mm oder >2000mm)\n",
                (unsigned long)s_statRangeErr);

  if (s_ok && s_statRead == 0 && s_statStatusErr == 0 && s_statRangeErr == 0) {
    Serial.println("  HINWEIS: noch keine Auswertung - Intervall abwarten (100ms)");
  }
  if (!s_ok) {
    Serial.println("  HINWEIS: s_ok=false. Boot-Log oben zeigt den Fehlerpunkt.");
    Serial.println("           0x29 UND 0x30 antworten nicht -> Verdrahtung/VCC pruefen.");
    Serial.println("           Beide antworten -> 0x29 doppelt belegt (Konflikt mit TCS34725).");
  }
  if (s_statStatusErr > s_statRead) {
    Serial.println("  HINWEIS: Status-Fehler dominieren -> zu dunkel/zu hell,");
    Serial.println("           starke Reflexion, oder Ziel ausser Reichweite.");
  }
  Serial.println("--------------------------");
}
