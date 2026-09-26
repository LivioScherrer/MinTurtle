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
#include <esp_task_wdt.h>
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

// Adress-Probe mit Bus-Sperre. Diagnose-Ausgaben werden zur Laufzeit
// aufgerufen (DISTDIAG, DISTDEBUG), also waehrend der Farbsensor laeuft -
// ohne Sperre genau das verfaelschte Ueberlappen, das I2CBus verhindern soll.
static bool probeAddrLocked(uint8_t addr) {
  if (!i2cBusLock(30)) return false;
  bool r = probeAddr(addr);
  i2cBusUnlock();
  return r;
}

// Objektzustand verwerfen: lox wird neu aufgebaut, also gehoeren die
// alten Flags nicht mehr dazu. s_ok=false verhindert zusaetzlich, dass
// distTick() Messfunktionen auf einer Instanz aufruft, deren StaticInit
// nicht lief (Skalierungsparameter 0 -> IntegerDivideByZero).
static void distInvalidate() {
  s_ok = false;
  s_continuous = false;
  s_last = -1;
  s_lastRaw = -1;
  s_blocked = false;
  s_lastStatus = 0xFF;
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
  // Skips zaehlen: sonst ist "0/5 gueltige Messungen" nicht von einem
  // toten Sensor zu unterscheiden, und der Modus wird dauerhaft falsch
  // gewaehlt, weil die Gegenprobe nie stattfand.
  int contSkip = 0, singleSkip = 0;

  // --- Continuous ---
  // Nur den Start/Stop des Messbetriebs unter der Sperre halten. Die
  // 5 Wartezeiten laufen AUSSERHALB, sonst stuende der Farbsensor-Task
  // fuer eine volle Sekunde still.
  if (i2cBusLock(300)) {
    lox.startRangeContinuous(100);
    i2cBusUnlock();
  } else {
    contSkip = 5;
  }
  for (int i = 0; i < 5; i++) {
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!i2cBusLock(DIST_SINGLE_LOCK_MS)) { contSkip++; continue; }
    bool ready = lox.isRangeComplete();
    uint16_t mm = ready ? lox.readRangeResult() : 0;
    uint8_t st = lox.readRangeStatus();
    i2cBusUnlock();
    if (ready && st == 0 && mm > 0 && mm < 2000) contOk++;
  }
  if (i2cBusLock(300)) { lox.stopRangeContinuous(); i2cBusUnlock(); }
  Serial.printf("[DIST] Modustest Continuous: %u/5 gueltige Messungen", (unsigned)contOk);
  if (contSkip) Serial.printf("  (%d uebersprungen: I2C-Bus belegt)", contSkip);
  Serial.println();

  // --- Single-Shot ---
  for (int i = 0; i < 5; i++) {
    vTaskDelay(pdMS_TO_TICKS(60));
    if (!i2cBusLock(DIST_SINGLE_LOCK_MS)) { singleSkip++; continue; }
    uint16_t mm = lox.readRange();
    uint8_t st = lox.readRangeStatus();
    i2cBusUnlock();
    if (st == 0 && mm > 0 && mm < 2000) singleOk++;
  }
  Serial.printf("[DIST] Modustest Single-Shot: %u/5 gueltige Messungen", (unsigned)singleOk);
  if (singleSkip) Serial.printf("  (%d uebersprungen: I2C-Bus belegt)", singleSkip);
  Serial.println();

  s_continuous = (contOk >= 2);
  if (s_continuous) {
    if (i2cBusLock(300)) { lox.startRangeContinuous(100); i2cBusUnlock(); }
    Serial.println("[DIST] Mess-Modus: CONTINUOUS (non-blocking)");
  } else {
    s_lastSingle = millis();
    Serial.println("[DIST] Mess-Modus: SINGLE-SHOT (Fallback, blockierend ~alle 200ms)");
    // Hinweis: dieser Pfad wird nur erreicht, wenn begin() ERFOLGREICH war.
    // Ein -50 aus StaticInit schaltet den Sensor in distInitAtTarget ab,
    // es wird hier also nicht gemessen.
  }
}

// Adresswechsel und Initialisierung passieren beide in lox.begin(0x30).
//
// Grund: der Library-Patch (patch_vl53l0x.py) verlegt den Adresswechsel VOR
// DataInit. begin(0x30) zieht den Sensor also bei Bedarf selbst von 0x29 auf
// 0x30 - ueber Adafruits setAddress(), das Register 0x8A korrekt beschreibt -
// und initialisiert danach komplett auf 0x30. Deshalb braucht es hier KEINEN
// eigenen Register-Write mehr.
//
// Wichtig: nie 0xFF schreiben. 0xFF ist nicht die I2C-Adresse, sondern der
// Index-Register zur NVM-Seitenwahl; ein eigener Write dorthin laesst den
// Treiber undefinierte Register lesen.
static bool distInitAtTarget(const char* why) {
  // Der Sensor wurde in distSetup() bereits auf 0x30 geschoben. Durch den
  // Library-Patch (patch_vl53l0x.py) laeuft DataInit jetzt ebenfalls auf
  // 0x30 statt auf dem Default 0x29 - dort ist der Bus sauber, weil der
  // TCS34725 auf 0x29 bleibt. Damit liest DataInit die NVM-Kalibrierdaten
  // unverafaelscht und schreibt sie korrekt zurueck.
  const uint8_t initAddr = DIST_I2C_ADDR;
  Serial.printf("[DIST] Init komplett ueber 0x%02X (NVM-Zugriff ohne Kollision)\n",
                initAddr);

  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(120);
      Serial.printf("[DIST] Init-Versuch %d (%s)\n", attempt + 1, why);
    }
    // Gesamte Init-Sequenz sperren: der Farbsensor-Task darf den Bus nicht
    // umschalten, waehrend DataInit/StaticInit laufen.
    // Ein Lock-Timeout ist ein transienter Zustand (der Farbsensor haelt
    // den Bus gerade) - da lohnt ein weiterer Versuch. Ein Fehlschlag von
    // begin() dagegen ist ein Geraetezustand und wird nicht wiederholt.
    if (!i2cBusLock(500)) {
      Serial.println("[DIST] I2C-Bus belegt (Farbsensor aktiv) -> erneut versuchen");
      continue;
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
      // StaticInit ist an der Ref-SPAD-Kalibrierung gescheitert (-50). Die
      // NVM-Ref-SPAD-Daten sind dann unbrauchbar (z. B. refSpadCount=1070200835,
      // isApertureSpads=247) und die abhaengigen Skalierungsparameter des
      // Treibers bleiben 0. Jede Messfunktion rechnet dann durch Null:
      //   Guru Meditation Error: Core 1 panic'ed (IntegerDivideByZero)
      // Deshalb wird hier bewusst NICHT gemessen und kein Modus getestet.
      // Der Sensor bleibt deaktiviert, bis die Ref-SPAD-Kalibrierung
      // erfolgreich war - das ist die einzige Stelle, die sie schreibt.
      distInvalidate();
      Serial.println("[DIST] StaticInit FEHLGESCHLAGEN (Ref-SPAD-Kalibrierung -50)");
      Serial.println("[DIST] NVM-Ref-SPAD-Daten unbrauchbar -> Messung wuerde durch Null teilen");
      Serial.println("[DIST] -> Distanzsensor deaktiviert (kein Absturz, aber auch keine Werte)");
      Serial.println("[DIST]    Ursache 1: Ref-SPAD-Kalibrierung braucht ein geeignetes Ziel");
      Serial.println("[DIST]    (weisse Flaeche auf ~20 cm, nicht in direkter Sonne).");
      Serial.println("[DIST]    Ursache 2: NVM-Daten beschaedigt (DISTDEBUG zeigt den Rohwert).");
      Serial.println("[DIST]    Abhilfe: Ziel herrichten, dann DISTREF senden.");
      return false;
    }

    // Der Adresswechsel auf 0x30 ist bereits in begin() erfolgt (Library-
    // Patch) - hier nicht noch einmal machen.

    // --- Normalfall: StaticInit erfolgreich ---
    // Timing-Budget hier bewusst NICHT setzen: der Aufruf schreibt die
    // Phasenkalibrierung ins NVM. Standard-Budget der Library (33 ms) reicht.
    if (!i2cBusLock(300)) {
      Serial.println("[DIST] I2C-Bus belegt -> Messstart nicht moeglich");
      distInvalidate();
      return false;
    }
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
  Serial.printf("    0x29 : %s\n", probeAddrLocked(0x29) ? "ACK" : "--");
  Serial.printf("    0x30 : %s\n", probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");
  Serial.println("---------------------------------------");
}

void distSetup() {
  i2cBusInit();
  // Wire muss laufen (wird hier gestartet falls noetig)
  Wire.begin(SDA_PIN, SCL_PIN);
  s_bootMs = millis();

  // BEVOR irgendetwas initialisiert oder verschoben wird: Bus protokollieren.
  //
  // ENTSCHEIDEND: Der Adresskonflikt am Start war der Kern des Problems.
  // Der VL53L0X kann nur initialisiert werden, wenn die NVM-Lesevorgaenge
  // der Library auf einer Adresse laufen, an der er ALLEINE antwortet. Der
  // Original-Treiber las sie noch auf der Default-Adresse 0x29, wo auch der
  // TCS34725 sitzt - dessen Antworten verfaelschten die Registerwerte, der
  // Ref-SPAD-Parameter wurde ungueltig und StaticInit scheitert mit -50.
  //
  // Behoben ueber den Library-Patch: begin(0x30) zieht den Sensor vor
  // DataInit auf 0x30. 0x29 zu BOOT-Zeitpunkt zu belegen ist damit nur noch
  // eine Information (und ein Hinweis, ob der Farbsensor haengt), keine
  // Vorbedingung mehr.
  s_bootHad29 = probeAddr(0x29);
  s_bootHad30 = probeAddr(DIST_I2C_ADDR);
  distLogBusScan("vor Init");
  Serial.printf("[DIST] 0x29 zum Boot: %s -> %s\n",
                s_bootHad29 ? "belegt (TCS34725)" : "frei",
                s_bootHad29 ? "Init laeuft trotzdem ueber 0x30 (Patch)"
                            : "Init ueber 0x30, 0x29 waere auch sauber");

  // Die Belegung von 0x29 ist jetzt reine Information (haengt der
  // Farbsensor?) und keine Vorbedingung mehr: der Patch laesst den
  // kompletten Init auf 0x30 laufen, wo der VL53L0X allein antwortet.

  // =============================================================
  // ADRESSWECHSEL AUF 0x30 - erledigt begin(0x30) selbst
  // =============================================================
  // Der Library-Patch (patch_vl53l0x.py) verlagert den Adresswechsel VOR
  // DataInit. begin(0x30) zieht den Sensor dabei mit einem reinen Write von
  // 0x29 auf 0x30 (bewusst ohne Lesezugriff auf 0x29, siehe patch_vl53l0x.py).
  // Danach laeuft die komplette Initialisierung ebenfalls auf 0x30, wo der
  // I2C-Bus sauber ist.
  //
  // Das loest den urspruenglichen Fehler: DataInit liest die
  // NVM-Kalibrierdaten und schreibt sie zurueck. Auf 0x29 verfaelscht der
  // zweite Sensor diese Reads, der Treiber speichert Muell, und StaticInit
  // scheitert danach mit -50.
  const bool at29 = probeAddr(0x29);
  const bool at30 = probeAddr(DIST_I2C_ADDR);
  Serial.printf("[DIST] Adresslage vor Init: 0x29 %s / 0x30 %s\n",
                at29 ? "ACK" : "--", at30 ? "ACK" : "--");
  if (at30) {
    Serial.println("[DIST] VL53L0X sitzt bereits auf 0x30");
  } else if (at29) {
    Serial.println("[DIST] VL53L0X steht noch auf 0x29 -> begin() zieht ihn auf 0x30 um");
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
  // lox wird unten neu aufgebaut. s_ok/s_continuous gehoeren zum alten
  // Objekt - sie muessen mit, sonst ruft distTick() gleich danach Mess-
  // funktionen auf einer Instanz auf, deren StaticInit nie lief (Skalierungs-
  // parameter 0 -> IntegerDivideByZero).
  distInvalidate();

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("---- VL53L0X Init-Debug ----");
  Serial.printf("  SDA=GPIO%d SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
  Serial.printf("  0x29: %s   0x30: %s\n",
                probeAddrLocked(0x29) ? "ACK" : "--",
                probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");

  // Schritt 1: lox.begin(0x30). Der Library-Patch zieht den Sensor dabei bei
  // Bedarf per Write von 0x29 auf 0x30, ohne dort zu lesen.
  //
  // Der Bus wird nur fuer den begin()-Aufruf gehalten. Die danach folgende
  // Messreihe (6 x 300 ms) laeuft OHNE Sperre: sonst stuende der Farbsensor-
  // Task waehrend der gesamten Diagnose still.
  Serial.println("  -- lox.begin(0x30, debug=true) [Bus gesperrt, frische Instanz] --");
  if (!i2cBusLock(500)) {
    Serial.println("  I2C-Bus dauerhaft belegt -> Debug abgebrochen");
    sendStatus("DIST:DEBUG_DONE");
    return;
  }
  loxResetInstance();
  bool ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
  Serial.printf("  lox.begin(0x30) -> %s\n", ok ? "OK" : "FEHLER");

  // KEIN Rueckschieben auf 0x29 als Wiederholungsversuch: das wuerde den
  // VL53L0X direkt neben den TCS34725 legen, wo alle NVM-Reads verfaelscht
  // werden. Stattdessen: frische Instanz, erneut probieren - der Umzug
  // 0x29 -> 0x30 ist ein reiner Write und wiederholt sich gefahrlos.
  if (!ok) {
    Serial.println("  -- frische Instanz, begin(0x30) erneut --");
    delay(200);
    loxResetInstance();
    ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
    Serial.printf("  lox.begin(0x30) erneut -> %s\n", ok ? "OK" : "FEHLER");
  }
  i2cBusUnlock();   // ab hier ist der Bus wieder frei

  // Endzustand nur noch melden. Ein Reparatur-Write auf 0x30 waere sinnlos:
  // wenn 0x30 nicht antwortet, gibt es dort auch kein Geraet, das man
  // zurueckschieben koennte. Der Aufruf loest den Adresswechsel jetzt
  // eigenstaendig aus.
  delay(60);
  Serial.printf("  Endzustand: 0x29 %s   0x30 %s\n",
                probeAddrLocked(0x29) ? "ACK" : "--",
                probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");

  if (ok) {
    // KEIN Timing-Budget setzen: der Aufruf schreibt die Phasenkalibrierung
    // ins NVM des Sensors und ist die wahrscheinlichste Ursache fuer
    // Range-Status 4 (PhaseFail). Library-Standard (33 ms) verwenden.
    bool started = false;
    if (!i2cBusLock(300)) {
      Serial.println("  I2C-Bus belegt -> Messstart und Messreihe uebersprungen");
    } else {
      started = lox.startRangeContinuous(100);
      i2cBusUnlock();
      Serial.printf("  startRangeContinuous(100) -> %s  (Budget: Library-Standard)\n",
                    started ? "OK" : "FEHLER");
    }
    if (started) {
      // Messreihe bewusst OHNE Bus-Sperre: pro Iteration nur kurz sperren.
      for (int i = 0; i < 6; i++) {
        vTaskDelay(pdMS_TO_TICKS(300));
        if (!i2cBusLock(50)) {
          Serial.printf("    isRangeComplete=?  rangeStatus=?  mm=?  (Bus belegt)\n");
          continue;
        }
        bool ready = lox.isRangeComplete();
        uint16_t mm = ready ? lox.readRangeResult() : 0xffff;
        uint8_t st = lox.readRangeStatus();
        i2cBusUnlock();
        Serial.printf("    isRangeComplete=%d  rangeStatus=%u  mm=%u\n",
                      (int)ready, (unsigned)st, (unsigned)mm);
      }
      // Der Sensor misst tatsaechlich: Zustand setzen, damit distTick()
      // nach einem erfolgreichen DISTDEBUG weiterarbeitet.
      s_ok = true;
      s_continuous = true;
    } else {
      // Messbetrieb laeuft nicht. s_ok=true waere der "bereit ohne Messung"-
      // Zustand: isRangeComplete() bliebe immer false, es kaeme kein
      // DIST:<mm> mehr, und distBlocked() liefe dauerhaft auf "frei".
      Serial.println("  Messbetrieb nicht gestartet -> Sensor bleibt deaktiviert");
      Serial.println("  (alternativ DISTREF fuer die Ref-SPAD-Kalibrierung)");
      distInvalidate();
    }
  } else {
    // StaticInit ist gescheitert. Es wird hier BEWUSST NICHT gemessen:
    // nach fehlgeschlagener Ref-SPAD-Kalibrierung sind die Skalierungs-
    // parameter des Treibers 0, und die Messfunktionen teilen durch sie
    // (IntegerDivideByZero). Stattdessen nur den NVM-Rohwert melden, damit
    // man sieht, ob die Ref-SPAD-Daten ungueltig sind.
    // s_ok bleibt bewusst false (distInvalidate() oben).
    Serial.println("  -- StaticInit fehlgeschlagen: KEINE Messung (Treiber teilt sonst durch 0) --");
    Serial.println("  -- NVM-Rohwert Ref-SPAD stattdessen --");

    uint8_t spad[4] = {0, 0, 0, 0};
    bool spadRead = false;
    if (i2cBusLock(500)) {
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
        spadRead = true;
      }
      i2cBusUnlock();
    } else {
      Serial.println("  I2C-Bus dauerhaft belegt -> NVM-Rohwert nicht lesbar");
    }
    if (spadRead) {
      uint32_t tmp = ((uint32_t)spad[0] << 24) | ((uint32_t)spad[1] << 16) |
                     ((uint32_t)spad[2] << 8) | (uint32_t)spad[3];
      Serial.printf("  NVM Ref-SPAD: DWord=0x%08X  count=%u  isAperture=%u\n",
                    (unsigned)tmp, (unsigned)((tmp >> 8) & 0x7F), (unsigned)((tmp >> 15) & 0x01));
      Serial.println("  -> count > 44 oder isAperture > 1 bedeutet: NVM-Daten unbrauchbar,");
      Serial.println("     Ref-SPAD-Kalibrierung muss neu laufen (DISTREF, geeignetes Ziel noetig).");
    }

    Serial.println("  -- Adresslage zur Kontrolle (nur Adress-Probe) --");
    Serial.printf("  0x29 %s / 0x30 %s\n",
                  probeAddrLocked(0x29) ? "ACK" : "--",
                  probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");
  }
  Serial.println("---------------------------");
  sendStatus("DIST:DEBUG_DONE");
}

int distRefSpadCalibrate() {
  // Reentranzschutz: die Kalibrierung dauert Sekunden und schreibt ins NVM.
  // Ein zweiter DISTREF waehrenddessen wuerde die NVM-Daten dazwischen
  // ueberschreiben.
  static bool s_calibrating = false;
  if (s_calibrating) {
    Serial.println("[DIST] Ref-SPAD-Kalibrierung laeuft bereits -> ignoriert");
    return -96;
  }
  s_calibrating = true;

  // lox wird unten neu aufgebaut -> alten Objektzustand verwerfen. Sonst
  // ruft distTick() Messfunktionen auf einer Instanz auf, deren StaticInit
  // nicht lief (Skalierungsparameter 0 -> IntegerDivideByZero).
  distInvalidate();

  Serial.println("[DIST] === Ref-SPAD-Kalibrierung (DISTREF) ===");
  Serial.println("[DIST] Voraussetzung: weisse, matte Flaeche auf ~20-30 cm,");
  Serial.println("[DIST] nicht in direkter Sonne. Dauert einige Sekunden.");
  Serial.println("[DIST] Der Farbsensor pausiert waehrend der Kalibrierung (geteilter Bus).");

  if (!i2cBusLock(1500)) {
    Serial.println("[DIST] I2C-Bus dauerhaft belegt -> Abbruch");
    s_calibrating = false;
    return -99;
  }

  // Frische Instanz: get_info_from_device() liest die NVM-Daten nur EIN MAL
  // pro Objekt. loxResetInstance() baut das Objekt korrekt neu auf (eine
  // Zuweisung wuerde den Self-Pointer mitkopieren - Crash, siehe dort).
  loxResetInstance();

  uint32_t count = 0;
  uint8_t aperture = 0;
  Serial.println("[DIST] Kalibrierung laeuft (schreibt die NVM-Daten neu)...");
  // eduRefSpadRecalibrate() holt DataInit und StaticInit selbst nach und
  // fuehrt die Ref-SPAD-Kalibrierung danach erneut aus. Das ist der
  // entscheidende Punkt: begin() wuerde bei -50 false liefern und die
  // Kalibrierung damit unerreichbar machen - obwohl genau -50 der Fall ist,
  // den diese Funktion retten soll. -50 entsteht in
  // perform_ref_spad_management, laeuft also NACH StaticInit; die
  // Skalierungsparameter sind in diesem Fall gueltig.
  int st = lox.eduRefSpadRecalibrate(&count, &aperture);
  Serial.printf("[DIST] Ergebnis: Status=%d  refSpadCount=%u  isApertureSpads=%u\n",
                st, (unsigned)count, (unsigned)aperture);
  i2cBusUnlock();

  if (st != 0) {
    Serial.println("[DIST] -> FEHLGESCHLAGEN.");
    if (st == -50) {
      Serial.println("[DIST]    -50 = Ziel unbrauchbar: weisse matte Flaeche auf");
      Serial.println("[DIST]    ~20-30 cm, nicht in direkter Sonne, Sensor still.");
      Serial.println("[DIST]    DISTREF erneut senden, Ziel ist jetzt richtig.");
    } else {
      Serial.println("[DIST]    Kalibrierung konnte nicht durchlaufen (Ziel oder");
      Serial.println("[DIST]    Sensorzustand). DISTDEBUG zeigt den NVM-Rohwert.");
    }
    // Der Sensor bleibt deaktiviert (s_ok=false von distInvalidate oben).
    // Das ist korrekt: nach fehlgeschlagener Ref-SPAD-Kalibrierung sind die
    // Skalierungsparameter nicht vertrauenswuerdig.
    s_calibrating = false;
    return st;
  }
  if (count == 0 || count > 44 || aperture > 1) {
    Serial.println("[DIST] -> Status 0, aber die Werte sind unplausibel");
    Serial.println("[DIST]    (count muss 1..44, isApertureSpads 0 oder 1 sein).");
    s_calibrating = false;
    return -97;
  }

  // Kalibrierung erfolgreich: die NVM-Daten sind jetzt gueltig. Damit der
  // Sensor wieder messbereit wird, muss er nur noch sauber neu initialisiert
  // werden - StaticInit findet die Kalibrierung dann unveraendert vor.
  Serial.println("[DIST] Kalibrierung plausibel -> Sensor wird neu initialisiert");
  loxResetInstance();
  s_calibrating = false;   // ab hier laeuft distInitAtTarget (intern gesperrt)
  if (distInitAtTarget("nach Ref-SPAD-Kalibrierung")) {
    Serial.println("[DIST] DISTREF: Sensor wieder messbereit");
    return 0;
  }
  Serial.println("[DIST] DISTREF: Neuinitialisierung schlug fehl");
  return -98;
}

// --- Asynchroner DISTREF-Weg (siehe distRefSpadRequest) -------------------
// Die Ref-SPAD-Kalibrierung dauert mehrere Sekunden und wuerde den
// Loop-Task laenger als der Loop mit blockiert werden. Sie laeuft deshalb
// in einem eigenen Task; loop() bleibt responsiv und fuettert den WDT.
static volatile int  s_refResult = INT32_MIN;  // INT32_MIN = kein Lauf/kein Ergebnis
static volatile bool s_refBusy = false;

static void refSpadTask(void* arg) {
  (void)arg;
  int st = distRefSpadCalibrate();
  s_refResult = st;   // Ergebnis VOR s_refBusy=false schreiben (Reihenfolge
  s_refBusy = false;  // ist die Sichtbarkeitsgarantie fuer den Leser)
  vTaskDelete(NULL);
}

bool distRefSpadRequest() {
  if (s_refBusy) {
    Serial.println("[DIST] Ref-SPAD-Kalibrierung laeuft bereits");
    return false;
  }
  s_refBusy = true;
  s_refResult = INT32_MIN;
  // Gleiche Prioritaet wie der Loop-Task -> der Loop laeuft zwischendurch
  // weiter (esp_task_wdt_reset() wird nicht mehr verpasst).
  if (xTaskCreatePinnedToCore(refSpadTask, "distRefSpad", 8192, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("[DIST] Kalibrierungs-Task konnte nicht starten");
    s_refBusy = false;
    return false;
  }
  return true;
}

void distRefSpadService() {
  if (s_refBusy || s_refResult == INT32_MIN) return;
  int st = s_refResult;
  s_refResult = INT32_MIN;
  if (st == 0) {
    sendStatus("DISTREF:OK");
  } else {
    char rbuf[32];
    snprintf(rbuf, sizeof(rbuf), "DISTREF:ERR%d", st);
    sendStatus(rbuf);
  }
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
//
// ZWEI Modi. Der Normalfall ist der Continuous-Modus, weil er nicht
// blockiert: lox.isRangeComplete() + readRangeResult(). Nur wenn
// startRangeContinuous() scheitert, ermittelt distProbeModes() den
// Single-Shot-Fallback - lox.readRange() schaltet den Sensor intern auf
// SINGLE_RANGING und initialisiert die Sequenz dabei neu. Beide Modi
// setzen ein erfolgreiches StaticInit voraus; schlug das fehl, ist s_ok
// false und diese Funktion misst gar nicht (IntegerDivideByZero).
//
// RUECKGABE: ein Wert >= 0 ist eine Messung. Bei ausbleibender neuer
// Messung wird bewusst der GESTEIGERTE Wert s_lastRaw geliefert (kein
// "--"): ein gehaltenes Ergebnis ist fuer den Betrieb besser als eine
// Luecke. *fresh (optional) unterscheidet die beiden Faelle - die
// Offset-Kalibrierung braucht das, denn ein gehaltener Wert darf nicht als
// eigene Probe in den Mittelwert eingehen.
static int16_t readRawMM(bool* fresh = nullptr) {
  if (fresh) *fresh = false;
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
    if (fresh) *fresh = true;
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
  if (fresh) *fresh = true;   // hier wurde tatsaechlich gemessen
  return s_lastRaw;
}

int16_t distMeasureMM() {
  int16_t raw = readRawMM();
  if (raw < 0) {
    // Keine gueltige Messung: den Cache loeschen. Sonst bewertet
    // distBlocked() weiter den letzten Wert, waehrend die WebUI beim
    // zugehoerigen "DIST:--" den Badge auf frei schaltet - Firmware und
    // Anzeige wuerden auseinanderlaufen. readRawMM() haelt den Wert fuer
    // den Normalbetrieb bewusst (s_lastRaw bleibt), aber die *korrigierte*
    // Anzeige muss "kein Wert" sagen, sonst gilt die dokumentierte Regel
    // "kein Messwert = frei" nie.
    s_last = -1;
    return -1;
  }
  int16_t c = raw - config.obsOffsetMM;
  if (c < 0) c = 0;
  s_last = c;
  return s_last;
}

bool distCalibrateOffset(int16_t trueMM) {
  if (!s_ok) {
    Serial.println("[DIST] Offset-Kalibrierung: Sensor nicht bereit (s_ok=false)");
    return false;
  }
  // Caches verwerfen (alte Korrektur darf nicht einflussen)
  s_last = -1; s_lastRaw = -1;
  long sum = 0;
  int n = 0;
  // readRawMM() liefert bewusst auch den GESTEIGERTEN Wert s_lastRaw zurueck,
  // wenn keine neue Messung vorliegt (Continuous: isRangeComplete()==false,
  // Single-Shot: innerhalb von DIST_SINGLE_PERIOD_MS). Fuer den Normalbetrieb
  // ist das richtig - ein gehaltenes Messergebnis ist besser als "--". Fuer
  // die Kalibrierung waere es ein Fehler: derselbe Wert wuerde bis zu 10x
  // als unabhaengige Probe gezaehlt und der Mittelwert waere in Wahrheit
  // der allererste Messwert. Deshalb ueber den fresh-Ausgangsparameter nur
  // Proben zaehlen, bei denen wirklich neu gemessen wurde. Wichtig: nicht
  // ueber "Wert unveraendert" filtern - ein stabiles Ziel liefert
  // zu Recht dieselbe Zahl, und das ist eine echte Messung.
  uint32_t start = millis();
  while (n < 10 && millis() - start < 8000) {
    bool fresh = false;
    int16_t r = readRawMM(&fresh);
    if (fresh && r > 0) { sum += r; n++; }
    bleProcessEvents();
    espNowProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  if (n < 5) {
    Serial.printf("[DIST] Offset-Kalibrierung: zu wenige neue Messungen (%d/5)\n", n);
    Serial.println("[DIST]    Sensor liefert keine neuen Werte? Abstand/Ziel pruefen.");
    return false;
  }
  int16_t avg = (int16_t)(sum / n);
  // Plausibilitaet: Mittel muss im linearen Bereich nahe am Ziel liegen
  if (abs(avg - trueMM) > 150) {
    Serial.printf("[DIST] Offset-Kalibrierung: Mittel %d mm liegt zu weit vom Ziel %d mm\n",
                  (int)avg, (int)trueMM);
    return false;
  }
  int16_t off = avg - trueMM;
  if (off < -200 || off > 200) {
    Serial.printf("[DIST] Offset-Kalibrierung: Offset %d mm ausserhalb -200..+200\n", (int)off);
    return false;
  }
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
  // Nur das, was tatsaechlich gemessen wurde. Ein "0x30 (soll 0x30)" waere
  // eine Konstante gegen sich selbst und koennte nie zeigen, dass der
  // Sensor auf der falschen Adresse haengt - genau das waere hier die
  // interessante Abweichung.
  Serial.printf("  0x29 antwortet : %s (TCS34725 erwartet)\n",
                probeAddrLocked(0x29) ? "ja" : "nein");
  Serial.printf("  0x30 antwortet : %s (VL53L0X erwartet, IST die Zieladresse)\n",
                probeAddrLocked(DIST_I2C_ADDR) ? "ja" : "nein");
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
