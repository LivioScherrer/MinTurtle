#include "DistanceSensor.h"
#include "Config.h"
#include "ColorSensor.h"
#include "BLEManager.h"
#include "EspNow.h"
#include "I2CBus.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <new>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static Adafruit_VL53L0X lox;
static volatile bool s_ok = false;
static bool s_auto = true;
static volatile bool s_continuous = false;
static volatile int16_t s_last = -1;
static volatile int16_t s_lastRaw = -1;
static volatile unsigned long s_lastMs = 0;
static bool s_blocked = false;
static unsigned long s_lastNotify = 0;
static unsigned long s_lastSingle = 0;
static char s_bootScan[96] = "";
static unsigned long s_bootMs = 0;
static bool s_bootHad29 = false;
static bool s_bootHad30 = false;
static volatile uint32_t s_statRead = 0;
static volatile uint32_t s_statNotReady = 0;
static volatile uint32_t s_statStatusErr = 0;
static volatile uint32_t s_statRangeErr = 0;
static volatile uint8_t  s_lastStatus = 0xFF;

static bool probeAddr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool probeAddrLocked(uint8_t addr) {
  if (!i2cBusLock(30)) return false;
  bool r = probeAddr(addr);
  i2cBusUnlock();
  return r;
}

static void distInvalidate() {
  s_ok = false;
  s_continuous = false;
  s_last = -1;
  s_lastRaw = -1;
  s_blocked = false;
  s_lastStatus = 0xFF;
}

static void loxResetInstance() {
  lox.~Adafruit_VL53L0X();
  new (&lox) Adafruit_VL53L0X();
}

static void distProbeModes() {
  uint8_t contOk = 0, singleOk = 0;
  int contSkip = 0, singleSkip = 0;

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
  }
}

static bool distInitAtTarget(const char* why) {
  const uint8_t initAddr = DIST_I2C_ADDR;
  Serial.printf("[DIST] Init komplett ueber 0x%02X (NVM-Zugriff ohne Kollision)\n",
                initAddr);

  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(120);
      Serial.printf("[DIST] Init-Versuch %d (%s)\n", attempt + 1, why);
    }
    if (!i2cBusLock(500)) {
      Serial.println("[DIST] I2C-Bus belegt (Farbsensor aktiv) -> erneut versuchen");
      continue;
    }
    loxResetInstance();
    bool begun = lox.begin(initAddr, false, &Wire);
    i2cBusUnlock();

    if (!begun) {
      distInvalidate();
      Serial.println("[DIST] StaticInit FEHLGESCHLAGEN (meist -50, Ref-SPAD-Daten)");
      Serial.println("[DIST] Skalierungsparameter unbrauchbar -> Messung wuerde durch Null teilen");
      Serial.println("[DIST] -> Distanzsensor deaktiviert (kein Absturz, aber auch keine Werte)");
      Serial.println("[DIST]    Ursache 1: Ref-SPAD-Kalibrierung scheitert am Ziel");
      Serial.println("[DIST]    (weisse Flaeche auf ~20 cm, nicht in direkter Sonne)");
      Serial.println("[DIST]    -> Abhilfe: Ziel herrichten, DISTREF senden.");
      Serial.println("[DIST]    Ursache 2: NVM-Ref-SPAD-Daten unbrauchbar. Dann greift");
      Serial.println("[DIST]    die Kalibrierung aus DISTREF nicht (Tuning fehlt) und");
      Serial.println("[DIST]    der Good-Spad-Map-Patch greift nur bei leerer Map.");
      Serial.println("[DIST]    Abhilfe: DISTDEBUG zeigt den NVM-Rohwert; bleibt er");
      Serial.println("[DIST]    unplausibel, Sensor tauschen.");
      return false;
    }


    if (!i2cBusLock(300)) {
      Serial.println("[DIST] I2C-Bus belegt -> Messstart nicht moeglich");
      distInvalidate();
      return false;
    }
    lox.stopRangeContinuous();
    delay(20);
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
  Wire.begin(SDA_PIN, SCL_PIN);
  s_bootMs = millis();

  s_bootHad29 = probeAddr(0x29);
  s_bootHad30 = probeAddr(DIST_I2C_ADDR);
  distLogBusScan("vor Init");
  Serial.printf("[DIST] 0x29 zum Boot: %s -> %s\n",
                s_bootHad29 ? "belegt (TCS34725)" : "frei",
                s_bootHad29 ? "Init laeuft trotzdem ueber 0x30 (Patch)"
                            : "Init ueber 0x30, 0x29 waere auch sauber");


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

  delay(50);
  if (!probeAddr(DIST_I2C_ADDR)) {
    Serial.println("[DIST] 0x30 antwortet NICHT nach Init -> s_ok=false");
    s_ok = false;
    return;
  }
  if (!probeAddr(0x29)) {
    Serial.println("[DIST] WARNUNG: 0x29 antwortet nicht - Farbsensor fehlt?");
  }
  Serial.println("[DIST] Adressen: VL53L0X=0x30, TCS34725=0x29 (Farbsensor folgt in colorSetup)");
  distLogBusScan("nach Init");
}

void distDebugInit() {
  distInvalidate();

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("---- VL53L0X Init-Debug ----");
  Serial.printf("  SDA=GPIO%d SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
  Serial.printf("  0x29: %s   0x30: %s\n",
                probeAddrLocked(0x29) ? "ACK" : "--",
                probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");

  Serial.println("  -- lox.begin(0x30, debug=true) [Bus gesperrt, frische Instanz] --");
  if (!i2cBusLock(500)) {
    Serial.println("  I2C-Bus dauerhaft belegt -> Debug abgebrochen");
    sendStatus("DIST:DEBUG_DONE");
    return;
  }
  loxResetInstance();
  bool ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
  Serial.printf("  lox.begin(0x30) -> %s\n", ok ? "OK" : "FEHLER");

  if (!ok) {
    Serial.println("  -- frische Instanz, begin(0x30) erneut --");
    delay(200);
    loxResetInstance();
    ok = lox.begin(DIST_I2C_ADDR, true, &Wire);
    Serial.printf("  lox.begin(0x30) erneut -> %s\n", ok ? "OK" : "FEHLER");
  }
  i2cBusUnlock();

  delay(60);
  Serial.printf("  Endzustand: 0x29 %s   0x30 %s\n",
                probeAddrLocked(0x29) ? "ACK" : "--",
                probeAddrLocked(DIST_I2C_ADDR) ? "ACK" : "--");

  if (ok) {
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
      for (int i = 0; i < 6; i++) {
        bleProcessEvents();
        espNowProcessEvents();
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
      s_ok = true;
      s_continuous = true;
    } else {
      Serial.println("  Messbetrieb nicht gestartet -> Sensor bleibt deaktiviert");
      Serial.println("  (alternativ DISTREF fuer die Ref-SPAD-Kalibrierung)");
      distInvalidate();
    }
  } else {
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
  static bool s_calibrating = false;
  if (s_calibrating) {
    Serial.println("[DIST] Ref-SPAD-Kalibrierung laeuft bereits -> ignoriert");
    return -96;
  }
  s_calibrating = true;

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

  loxResetInstance();

  uint32_t count = 0;
  uint8_t aperture = 0;
  Serial.println("[DIST] Kalibrierung laeuft (schreibt die NVM-Daten neu)...");
  int st = lox.eduRefSpadRecalibrate(DIST_I2C_ADDR, &count, &aperture);
  Serial.printf("[DIST] Ergebnis: Status=%d  refSpadCount=%u  isApertureSpads=%u\n",
                st, (unsigned)count, (unsigned)aperture);
  i2cBusUnlock();

  if (st != 0) {
    Serial.println("[DIST] -> FEHLGESCHLAGEN.");
    if (st == -50) {
      Serial.println("[DIST]    -50 = Ref-SPAD-Daten unbrauchbar (NVM) oder Ziel");
      Serial.println("[DIST]    ungeeignet. Unterscheiden laesst sich das nur am");
      Serial.println("[DIST]    NVM-Rohwert: DISTDEBUG zeigt ihn.");
      Serial.println("[DIST]    War das Ziel schuld: weisse matte Flaeche auf");
      Serial.println("[DIST]    ~20-30 cm, nicht in direkter Sonne, DISTREF erneut.");
      Serial.println("[DIST]    War der NVM schuld: DISTREF hilft hier nicht weiter,");
      Serial.println("[DIST]    weil die Skalierungsparameter dann fehlen. Ursache");
      Serial.println("[DIST]    ist dann der Library-Patch (Good-Spad-Map) bzw.");
      Serial.println("[DIST]    beschaedigte NVM-Daten -> Sensor neu kaufen.");
    } else {
      Serial.println("[DIST]    Kalibrierung konnte nicht durchlaufen (Ziel oder");
      Serial.println("[DIST]    Sensorzustand). DISTDEBUG zeigt den NVM-Rohwert.");
    }
    s_calibrating = false;
    return st;
  }
  if (count == 0 || count > 44 || aperture > 1) {
    Serial.println("[DIST] -> Status 0, aber die Werte sind unplausibel");
    Serial.println("[DIST]    (count muss 1..44, isApertureSpads 0 oder 1 sein).");
    s_calibrating = false;
    return -97;
  }

  Serial.println("[DIST] Kalibrierung plausibel -> Sensor wird neu initialisiert");
  loxResetInstance();
  s_calibrating = false;
  if (distInitAtTarget("nach Ref-SPAD-Kalibrierung")) {
    Serial.println("[DIST] DISTREF: Sensor wieder messbereit");
    return 0;
  }
  Serial.println("[DIST] DISTREF: Neuinitialisierung schlug fehl");
  return -98;
}

static volatile int  s_refResult = INT32_MIN;
static volatile bool s_refBusy = false;

static void refSpadTask(void* arg) {
  (void)arg;
  int st = distRefSpadCalibrate();
  s_refResult = st;
  s_refBusy = false;
  vTaskDelete(NULL);
}

bool distRefSpadRequest() {
  if (s_refBusy) {
    Serial.println("[DIST] Ref-SPAD-Kalibrierung laeuft bereits");
    return false;
  }
  s_refBusy = true;
  s_refResult = INT32_MIN;
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

  const uint32_t now = millis();
  if (now - s_lastSingle < DIST_SINGLE_PERIOD_MS) return s_lastRaw;
  s_lastSingle = now;
  if (!i2cBusLock(DIST_SINGLE_LOCK_MS * DIST_SINGLE_RETRIES)) return s_lastRaw;

  uint16_t mm = 0;
  uint8_t st = 0xFF;
  for (int attempt = 0; attempt < DIST_SINGLE_RETRIES; attempt++) {
    mm = lox.readRange();
    st = lox.readRangeStatus();
    if (st == 0 && mm > 0 && mm < 2000) break;
    s_statStatusErr++;
  }
  i2cBusUnlock();
  s_lastStatus = st;
  if (st != 0) return -1;
  if (mm == 0 || mm > 2000) { s_statRangeErr++; return -1; }
  s_statRead++;
  s_lastRaw = (int16_t)mm;
  if (fresh) *fresh = true;
  return s_lastRaw;
}

int16_t distMeasureMM() {
  int16_t raw = readRawMM();
  if (raw < 0) {
    s_last = -1;
    return -1;
  }
  int16_t c = (int16_t)lroundf(config.obsScale * (float)raw - (float)config.obsOffsetMM);
  if (c < 0) c = 0;
  s_last = c;
  s_lastMs = millis();
  return s_last;
}

static bool distCollectRawAvg(int16_t* out, uint8_t want, uint8_t minOk, uint32_t timeoutMs) {
  long sum = 0;
  int n = 0;
  uint32_t start = millis();
  while (n < want && millis() - start < timeoutMs) {
    bool fresh = false;
    int16_t r = readRawMM(&fresh);
    if (fresh && r > 0) { sum += r; n++; }
    bleProcessEvents();
    espNowProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  if (n < minOk) {
    Serial.printf("[DIST] zu wenige neue Messungen (%d/%u)\n", n, (unsigned)minOk);
    return false;
  }
  *out = (int16_t)(sum / n);
  return true;
}

bool distCalibrateOffset(int16_t trueMM) {
  if (!s_ok) {
    Serial.println("[DIST] Offset-Kalibrierung: Sensor nicht bereit (s_ok=false)");
    return false;
  }
  s_last = -1; s_lastRaw = -1;
  int16_t avg = 0;
  if (!distCollectRawAvg(&avg, 10, 5, 8000)) {
    Serial.println("[DIST] Offset-Kalibrierung: zu wenige neue Messungen");
    Serial.println("[DIST]    Sensor liefert keine neuen Werte? Abstand/Ziel pruefen.");
    return false;
  }
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
  config.obsScale = 1.0f;
  configSaveField("obsScale", 1.0f);
  config.obsOffsetMM = off;
  configSaveInt("obsOff", off);
  s_last = -1; s_lastRaw = -1;
  Serial.printf("[DIST] Offset-Kalibrierung: Ziel=%d roh=%d -> offset=%d\n",
                trueMM, avg, off);
  return true;
}

bool distMeasureRawAvg(int16_t& outRaw, uint8_t want, uint8_t minOk) {
  if (!s_ok) return false;
  s_lastRaw = -1;
  return distCollectRawAvg(&outRaw, want, minOk, 6000);
}

bool distCalibrate3(int16_t raw50, int16_t raw100, int16_t raw200) {
  const int16_t rr[3] = { raw50, raw100, raw200 };
  const float   tt[3] = { 50.0f, 100.0f, 200.0f };
  if (!(rr[0] > 0 && rr[0] < rr[1] && rr[1] < rr[2])) {
    Serial.printf("[DIST] 3-Punkt: Messwerte nicht monoton (%d/%d/%d)\n",
                  (int)rr[0], (int)rr[1], (int)rr[2]);
    return false;
  }
  for (int i = 0; i < 3; i++) {
    if (fabsf((float)rr[i] - tt[i]) > 150.0f) {
      Serial.printf("[DIST] 3-Punkt: roh %d passt nicht zu %d mm\n", (int)rr[i], (int)tt[i]);
      return false;
    }
  }
  const float rMean = ((float)rr[0] + rr[1] + rr[2]) / 3.0f;
  const float tMean = (tt[0] + tt[1] + tt[2]) / 3.0f;
  float cov = 0.0f, var = 0.0f;
  for (int i = 0; i < 3; i++) {
    const float dr = (float)rr[i] - rMean;
    cov += dr * (tt[i] - tMean);
    var += dr * dr;
  }
  if (var < 4.0f) {
    Serial.println("[DIST] 3-Punkt: Messwerte zu dicht beieinander");
    return false;
  }
  const float a = cov / var;
  const float b = tMean - a * rMean;
  if (a < 0.7f || a > 1.4f) {
    Serial.printf("[DIST] 3-Punkt: Steigung %.3f ausserhalb 0.7..1.4\n", (double)a);
    return false;
  }
  if (b < -400.0f || b > 400.0f) {
    Serial.printf("[DIST] 3-Punkt: Versatz %.1f ausserhalb -400..+400\n", (double)b);
    return false;
  }
  float maxRes = 0.0f;
  for (int i = 0; i < 3; i++) {
    const float res = fabsf(a * (float)rr[i] + b - tt[i]);
    if (res > maxRes) maxRes = res;
  }
  if (maxRes > 30.0f) {
    Serial.printf("[DIST] 3-Punkt: groesste Abweichung %.1f mm > 30 mm\n", (double)maxRes);
    return false;
  }
  config.obsScale    = a;
  config.obsOffsetMM = (int16_t)lroundf(-b);
  configSaveField("obsScale", a);
  configSaveInt("obsOff", config.obsOffsetMM);
  s_last = -1; s_lastRaw = -1;
  Serial.printf("[DIST] 3-Punkt-Kalibrierung: roh=%d/%d/%d -> scale=%.3f offset=%d (maxRes=%.1f)\n",
                (int)rr[0], (int)rr[1], (int)rr[2], (double)a,
                (int)config.obsOffsetMM, (double)maxRes);
  return true;
}

int16_t distOffsetMM() { return config.obsOffsetMM; }

void distClearOffset() {
  config.obsOffsetMM = 0;
  config.obsScale = 1.0f;
  configSaveInt("obsOff", 0);
  configSaveField("obsScale", 1.0f);
  s_last = -1; s_lastRaw = -1;
}

int16_t distLastMM() { return s_last; }

uint32_t distLastAgeMs() {
  if (s_last < 0 || s_lastMs == 0) return 0xFFFFFFFFu;
  return (uint32_t)(millis() - s_lastMs);
}

bool distBlocked() {
  if (!s_ok) return false;
  int16_t mm = s_last;
  if (mm < 0) { s_blocked = false; return false; }

  if (millis() - s_lastMs > DIST_VALUE_MAX_AGE_MS) {
    s_blocked = false;
    return false;
  }

  if (s_blocked) {
    if (mm > config.obsRelMM) s_blocked = false;
  } else {
    if (mm < config.obsDistMM) s_blocked = true;
  }
  return s_blocked;
}

void distTick() {
  if (!s_ok || !s_auto) return;
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
