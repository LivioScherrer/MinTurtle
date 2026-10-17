
import os

Import("env")

LIB_ROOT = os.path.join(env["PROJECT_LIBDEPS_DIR"], env["PIOENV"], "Adafruit_VL53L0X")
SRC_CPP = os.path.join(LIB_ROOT, "src", "Adafruit_VL53L0X.cpp")
SRC_H = os.path.join(LIB_ROOT, "src", "Adafruit_VL53L0X.h")

CPP_ANCHOR = "  pMyDevice->i2c->begin(); // VL53L0X_i2c_init();"

ORIGINAL_ADDR_LINE = "  pMyDevice->I2cDevAddr = VL53L0X_I2C_ADDR; // default"

CPP_PREVIOUS = [
    ("  // [EduV2-Patch] Adressatribut statt fest 0x29: sorgt dafuer, dass\n"
     "  // DataInit/StaticInit auf der vom Aufrufer gewaehlten Adresse laufen.\n"
     "  // Ohne das kollidieren VL53L0X und TCS34725 beide auf 0x29 und\n"
     "  // verfaelschen sich gegenseitig die NVM-Kalibrierdaten (-50).\n"
     "  pMyDevice->I2cDevAddr = i2c_addr;"),

    ("""
  // [EduV2-Patch] Adresswechsel vor DataInit. Siehe patch_vl53l0x.py.
  // Ohne diesen Schritt laeuft DataInit auf 0x29, kollidiert dort mit einem
  // zweiten I2C-Geraet und verfaelscht die NVM-Kalibrierdaten (-50).
  if (i2c_addr != VL53L0X_I2C_ADDR) {
    uint8_t whoAmI = 0;
    if ((VL53L0X_RdByte(pMyDevice, 0xC0, &whoAmI) == VL53L0X_ERROR_NONE) &&
        (whoAmI == 0xEE)) {
      if (!setAddress(i2c_addr)) {
        return false;
      }
    }
  }
  pMyDevice->I2cDevAddr = i2c_addr; // ab hier laeuft alles auf der Zieladresse"""),
]

CPP_NEW = """
  // [EduV2-Patch] Adresswechsel vor DataInit. Siehe patch_vl53l0x.py.
  // Ohne diesen Schritt laeuft DataInit auf 0x29, kollidiert dort mit einem
  // zweiten I2C-Geraet und verfaelscht die NVM-Kalibrierdaten (-50).
  // Die Zieladresse wird nur dann beschrieben, wenn sie noch nicht
  // antwortet: so faellt bei warmem Start grundlos kein Write auf 0x29
  // an, wo der Farbsensor sitzt. Die Probe ist reine Adressphase
  // (beginTransmission/endTransmission), also fuer beide Geraete harmlos.
  if (i2c_addr != VL53L0X_I2C_ADDR) {
    pMyDevice->i2c->beginTransmission(i2c_addr);
    bool atTarget = (pMyDevice->i2c->endTransmission() == 0);
    if (!atTarget) {
      if (!setAddress(i2c_addr)) {
        return false;
      }
    }
  }
  pMyDevice->I2cDevAddr = i2c_addr; // ab hier laeuft alles auf der Zieladresse"""

H_ANCHOR = " private:\n  VL53L0X_Dev_t MyDevice;"

H_PREVIOUS = ["""  // [EduV2-Patch] Ref-SPAD-Kalibrierung von aussen ausloesbar. Siehe
  // patch_vl53l0x.py. begin() startet sie selbst, scheitert aber ohne
  // geeignetes Ziel mit -50 und laesst den Sensor dann unbrauchbar zurueck.
  // Voraussetzung: DataInit ist gelaufen (begin() hat es getan, StaticInit
  // scheiterte erst danach).
  int eduRefSpadCalibration(uint8_t* outCount, uint8_t* outAperture) {
    uint8_t count = 0;
    uint8_t aperture = 0;
    VL53L0X_Error st = VL53L0X_PerformRefSpadManagement(pMyDevice, &count,
                                                        &aperture);
    if (outCount) {
      *outCount = count;
    }
    if (outAperture) {
      *outAperture = aperture;
    }
    return (int)st;
  }

 private:
  VL53L0X_Dev_t MyDevice;""",

"""  // [EduV2-Patch] Ref-SPAD-Kalibrierung von aussen ausloesbar. Siehe
  // patch_vl53l0x.py. begin() startet sie selbst, scheitert aber ohne
  // geeignetes Ziel mit -50 und laesst den Sensor dann unbrauchbar zurueck.
  // Voraussetzung: DataInit ist gelaufen (begin() hat es getan, StaticInit
  // scheiterte erst danach).
  int eduRefSpadCalibration(uint32_t* outCount, uint8_t* outAperture) {
    uint32_t count = 0;
    uint8_t aperture = 0;
    VL53L0X_Error st = VL53L0X_PerformRefSpadManagement(pMyDevice, &count,
                                                        &aperture);
    if (outCount) {
      *outCount = count;
    }
    if (outAperture) {
      *outAperture = aperture;
    }
    return (int)st;
  }

 private:
  VL53L0X_Dev_t MyDevice;""",

"""  // [EduV2-Patch] Ref-SPAD-Kalibrierung erneut ausfuehren, auch wenn
  // begin() genau daran gescheitert ist. Siehe patch_vl53l0x.py.
  //
  // Die Reihenfolge ist der ganze Punkt und darf nicht vereinfacht werden:
  //   DataInit -> StaticInit -> NUR DANN Ref-SPAD-Kalibrierung
  // StaticInit laedt ueber VL53L0X_load_tuning_settings die
  // Skalierungsparameter. Schlaegt StaticInit fehl, sind die 0 und jede
  // Messfunktion teilt durch sie (IntegerDivideByZero) - dann wird hier
  // abgebrochen und es wird nicht gemessen.
  //
  // Warum das den einzigen Rettungsweg ist: -50 (REF_SPAD_INIT) liefert
  // ausschliesslich VL53L0X_perform_ref_spad_management, und das laeuft in
  // begin() NACH StaticInit. Ein begin()-Fehler mit -50 bedeutet daher:
  // Tuning-Daten sind vorhanden, nur die Ref-SPAD-Karte im NVM ist
  // unbrauchbar. Die genau diese Karte neu schreibt - ohne Loeten, ohne
  // Tausch. Ein Aufruf von begin() alone koennte das nicht: es liefert bei
  // -50 false und der Aufrufer wuerde die Kalibrierung nie erreichen.
  int eduRefSpadRecalibrate(uint32_t* outCount, uint8_t* outAperture) {
    VL53L0X_Error st = VL53L0X_DataInit(pMyDevice);
    if (st != VL53L0X_ERROR_NONE) {
      return (int)st;
    }
    st = VL53L0X_StaticInit(pMyDevice);
    if (st != VL53L0X_ERROR_NONE) {
      return (int)st;  // Skalierungsparameter unbrauchbar -> nicht messen
    }
    uint32_t count = 0;
    uint8_t aperture = 0;
    st = VL53L0X_PerformRefSpadManagement(pMyDevice, &count, &aperture);
    if (outCount) {
      *outCount = count;
    }
    if (outAperture) {
      *outAperture = aperture;
    }
    return (int)st;
  }

 private:
  VL53L0X_Dev_t MyDevice;"""]

H_NEW = """  // [EduV2-Patch] Ref-SPAD-Kalibrierung erneut ausfuehren, auch wenn
  // begin() genau daran gescheitert ist. Siehe patch_vl53l0x.py.
  //
  // WICHTIG - eigene Kommunikations-Initialisierung. Diese Methode wird
  // direkt nach einem Neuaufbau des Objekts aufgerufen, also ohne
  // vorheriges begin(). Nach dem Placement-New ist MyDevice
  // wertinitialisiert, pMyDevice->i2c ist NULL und I2cDevAddr ist 0. Ohne
  // den folgenden Block wuerde VL53L0X_DataInit() als erstes ueber
  // i2c->beginTransmission() einen NULL-Zeiger dereferenzieren (Panic, und
  // der I2C-Mutex bliebe haengen). Der Block entspricht begin() Zeile fuer
  // Zeile, inklusive Adresswechsel vor DataInit.
  //
  // Zum Fehlerverhalten: -50 (REF_SPAD_INIT) kann aus zwei Quellen kommen.
  //  (a) VL53L0X_StaticInit ruft VL53L0X_perform_ref_spad_management selbst
  //      auf, wenn die NVM-Ref-SPAD-Daten ungueltig sind. Dann schlaegt
  //      StaticInit fehl und VL53L0X_load_tuning_settings wird NICHT
  //      erreicht - die Skalierungsparameter bleiben 0. Genau dieser Fall
  //      ist es, den diese Methode NICHT reparieren kann; da wird
  //      zurueckgegeben und nicht gemessen.
  //  (b) begin() ruft PerformRefSpadManagement nach erfolgreichem
  //      StaticInit auf. Dann ist der Tuning-Stand gueltig, und genau
  //      diese Kalibrierung ist hier wiederholbar - der Grund fuer diese
  //      Methode: begin() liefert in diesem Fall false und der Aufrufer
  //      wuerde die Kalibrierung nie erreichen.
  int eduRefSpadRecalibrate(uint8_t i2c_addr, uint32_t* outCount,
                            uint8_t* outAperture) {
    pMyDevice->I2cDevAddr = VL53L0X_I2C_ADDR;
    pMyDevice->comms_type = 1;
    pMyDevice->comms_speed_khz = 400;
    pMyDevice->i2c = &Wire;
    pMyDevice->i2c->begin();
    if (i2c_addr != VL53L0X_I2C_ADDR) {
      pMyDevice->i2c->beginTransmission(i2c_addr);
      bool atTarget = (pMyDevice->i2c->endTransmission() == 0);
      if (!atTarget) {
        if (!setAddress(i2c_addr)) {
          return (int)VL53L0X_ERROR_CONTROL_INTERFACE;
        }
      }
    }
    pMyDevice->I2cDevAddr = i2c_addr;

    VL53L0X_Error st = VL53L0X_DataInit(pMyDevice);
    if (st != VL53L0X_ERROR_NONE) {
      return (int)st;
    }
    st = VL53L0X_StaticInit(pMyDevice);
    if (st != VL53L0X_ERROR_NONE) {
      return (int)st;  // Skalierungsparameter unbrauchbar -> nicht messen
    }
    uint32_t count = 0;
    uint8_t aperture = 0;
    st = VL53L0X_PerformRefSpadManagement(pMyDevice, &count, &aperture);
    if (outCount) {
      *outCount = count;
    }
    if (outAperture) {
      *outAperture = aperture;
    }
    return (int)st;
  }

 private:
  VL53L0X_Dev_t MyDevice;"""



CAL_REL = os.path.join(LIB_ROOT, "src", "core", "src", "vl53l0x_api_calibration.cpp")

CAL_ANCHOR = """   * represent spads.
   */
  for (index = 0; index < spadArraySize; index++)
    Dev->Data.SpadData.RefSpadEnables[index] = 0;"""

CAL_NEW = """   * represent spads.
   */
  for (index = 0; index < spadArraySize; index++) {
    Dev->Data.SpadData.RefSpadEnables[index] = 0;
    /* [EduV2-Patch] Good-Spad-Map initialisieren. Der Kommentar oben
       kuemmert das an, der Code tat es nicht - die Map kam ausschliesslich
       aus dem NVM. Ist dieses korrumpiert, bleibt sie leer, get_next_good_spad()
       liefert -1 und die Ref-SPAD-Kalibrierung scheitert dauerhaft mit -50,
       unabhaengig vom Ziel. Leere Bytes gelten als "SPAD gut"; eine belegte
       Map bleibt unveraendert. */
    if (Dev->Data.SpadData.RefGoodSpadMap[index] == 0) {
      Dev->Data.SpadData.RefGoodSpadMap[index] = 0xFF;
    }
  }"""


def fail(msg):
    print("[patch-vl53l0x] FEHLER: " + msg)
    print("[patch-vl53l0x] Abbruch - die Library wurde nicht gepatcht.")
    raise SystemExit(1)


def read(path):
    if not os.path.isfile(path):
        fail("Datei nicht gefunden: " + path)
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def write(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


content = read(SRC_CPP)
if CPP_ANCHOR not in content:
    fail("CPP-Ankerzeile nicht gefunden - hat sich die Library geaendert?")

for idx, block in enumerate(CPP_PREVIOUS):
    if block in content:
        content = content.replace(
            block, ORIGINAL_ADDR_LINE if idx == 0 else "", 1)
        print("[patch-vl53l0x] CPP-Vorversion %d entfernt" % (idx + 1))

if CPP_NEW in content:
    print("[patch-vl53l0x] CPP: Adresswechsel bereits vorhanden")
else:
    content = content.replace(CPP_ANCHOR, CPP_ANCHOR + CPP_NEW, 1)
    write(SRC_CPP, content)
    print("[patch-vl53l0x] CPP: Adresswechsel vor DataInit eingebaut (reiner Write)")

content = read(SRC_H)
for idx, block in enumerate(H_PREVIOUS):
    if block in content:
        content = content.replace(block, H_ANCHOR, 1)
        print("[patch-vl53l0x] H-Vorversion %d entfernt" % (idx + 1))
if H_NEW in content:
    print("[patch-vl53l0x] H: Ref-SPAD-Wrapper bereits vorhanden")
elif H_ANCHOR in content:
    content = content.replace(H_ANCHOR, H_NEW, 1)
    write(SRC_H, content)
    print("[patch-vl53l0x] H: Wrapper eduRefSpadRecalibrate() ergaenzt")
else:
    fail("H-Ankerzeile nicht gefunden - hat sich die Library geaendert?")

content = read(CAL_REL)
if CAL_NEW in content:
    print("[patch-vl53l0x] CAL: Good-Spad-Map-Initialisierung bereits vorhanden")
elif CAL_ANCHOR in content:
    content = content.replace(CAL_ANCHOR, CAL_NEW, 1)
    write(CAL_REL, content)
    print("[patch-vl53l0x] CAL: Good-Spad-Map-Initialisierung ergaenzt")
else:
    fail("CAL-Ankerzeile nicht gefunden - hat sich die Library geaendert?")
