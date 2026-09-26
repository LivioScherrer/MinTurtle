# PlatformIO Pre-Build-Skript: patcht die Adafruit_VL53L0X-Library.
# ======================================================================
# PROBLEM 1 - Adresskonflikt
# Adafruit_VL53L0X::begin(addr) schickt DataInit aus, ohne vorher die Adresse
# zu setzen, die der Aufrufer eigentlich will:
#
#     pMyDevice->I2cDevAddr = VL53L0X_I2C_ADDR;  // fest 0x29 (Werksadresse)
#     VL53L0X_DataInit(&MyDevice);                // laeuft auf 0x29
#
# DataInit LIEST die Kalibrierdaten aus dem NVM. Liegt auf 0x29 noch ein
# zweites Geraet (hier der TCS34725-Farbsensor), verfaelscht dieses die Reads.
# StaticInit scheitert danach mit -50 (REF_SPAD_INIT).
#
# LOESUNG 1
# Den Adresswechsel VOR DataInit ausfuehren. DataInit/StaticInit laufen dann
# komplett auf der Zieladresse, wo der Bus sauber ist.
#
# Drei bewusste Details:
#   * Ganz ohne Lesezugriff auf 0x29. Ein WHO_AM_I-Probe waere nicht
#     verlaesslich: liegen nach einem fehlgeschlagenen Rueckschreiben zwei
#     Geraete auf 0x29, liest man unbrauchbare Bitmuster und der Umzug
#     unterbleibt. Ein Write auf Register 0x8A ist dagegen unkritisch:
#     Ist der VL53L0X schon auf der Zieladresse, trifft der Write den
#     TCS34725. Dessen Register 0x8A gehoert zum Read-only-RGBC-Datenblock
#     (0x80-0x8B); ein Write dorthin wird verworfen und aendert weder den
#     Geraetezustand noch den naechsten Messwert.
#   * setAddress() wird benutzt, nicht ein handgeschriebener Wire-Write.
#     VL53L0X_SetDeviceAddress() schreibt Register 0x8A mit Wert/2
#     (Adafruit uebergibt newAddr*2). Register 0xFF ist NICHT die
#     I2C-Adresse, sondern der Index-Register zur NVM-Seitenwahl - ein
#     eigener Write dorthin setzt den Treiber in einen undefinierten Zustand.
#
# PROBLEM 2 - Ref-SPAD-Kalibrierung nicht aufrufbar
# Ist die NVM-Ref-SPAD-Karte unbrauchbar, ruft begin() selbst die
# Ref-SPAD-Kalibrierung auf - und genau die schreibt die NVM-Daten neu.
# Sie scheitert dann mit -50, begin() gibt false zurueck, und der Aufrufer
# kommt an die Kalibrierung nie heran: der Sensor bleibt unbrauchbar und
# begin() kann das auch nicht wiederholen, weil es bei -50 immer wieder
# false liefert. pMyDevice ist aber private, also von aussen nicht
# erreichbar. Deshalb wird eine schmale oeffentliche Methode ergaenzt, die
# DataInit und StaticInit selbst nachholt und die Kalibrierung danach
# erneut ausfuehrt (DISTREF) - ohne Loeten, ohne Tausch.
#
# Beide Aenderungen sind minimal und idempotent und brechen den Build ab,
# falls sich die Library geaendert hat (nicht stillschweigend weiterbauen).

import os

Import("env")

LIB_ROOT = os.path.join(env["PROJECT_LIBDEPS_DIR"], env["PIOENV"], "Adafruit_VL53L0X")
SRC_CPP = os.path.join(LIB_ROOT, "src", "Adafruit_VL53L0X.cpp")
SRC_H = os.path.join(LIB_ROOT, "src", "Adafruit_VL53L0X.h")

# ---------------------------------------------------------------- Anker CPP
# Anker: unmittelbar nach der Commus-Initialisierung ist pMyDevice->i2c
# gesetzt. Erst ab hier ist setAddress() benutzbar.
CPP_ANCHOR = "  pMyDevice->i2c->begin(); // VL53L0X_i2c_init();"

# Zeile, auf die die ueberholte Variante zurueckgebaut wird.
ORIGINAL_ADDR_LINE = "  pMyDevice->I2cDevAddr = VL53L0X_I2C_ADDR; // default"

# Bekannte Vorversionen des CPP-Blocks. Sie werden restlos entfernt, damit
# genau eine Variante stehen bleibt.
CPP_PREVIOUS = [
    # Variante 1: Adressattribut statt 0x29 (ueberholt).
    ("  // [EduV2-Patch] Adressatribut statt fest 0x29: sorgt dafuer, dass\n"
     "  // DataInit/StaticInit auf der vom Aufrufer gewaehlten Adresse laufen.\n"
     "  // Ohne das kollidieren VL53L0X und TCS34725 beide auf 0x29 und\n"
     "  // verfaelschen sich gegenseitig die NVM-Kalibrierdaten (-50).\n"
     "  pMyDevice->I2cDevAddr = i2c_addr;"),

    # Variante 2: Umzug mit WHO_AM_I-Probe (verworfen, nicht verlaesslich).
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
  // Bewusst ohne Lesezugriff auf 0x29 - der Umzug ist ein reiner Write.
  if (i2c_addr != VL53L0X_I2C_ADDR) {
    if (!setAddress(i2c_addr)) {
      return false;
    }
  }
  pMyDevice->I2cDevAddr = i2c_addr; // ab hier laeuft alles auf der Zieladresse"""

# ---------------------------------------------------------------- Anker H
# Genau vor dem private-Abschnitt, in dem pMyDevice liegt.
H_ANCHOR = " private:\n  VL53L0X_Dev_t MyDevice;"

# Bekannte Vorversionen des Header-Blocks (Signatur-Fehler in der ersten
# Fassung: der Count ist uint32_t*, nicht uint8_t*). Muss entfernt werden,
# sonst stuende die Methode zweimal.
#
# ZWEITE Vorversion: die urspruengliche Fassung rief die Kalibrierung direkt
# auf, ohne DataInit/StaticInit davorzuschalten, und liest damit nicht den
# Zustand zurueck, den begin() hergestellt hat.
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
  VL53L0X_Dev_t MyDevice;"""]

H_NEW = """  // [EduV2-Patch] Ref-SPAD-Kalibrierung erneut ausfuehren, auch wenn
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
  VL53L0X_Dev_t MyDevice;"""



# ---------------------------------------------------------------- Anker CAL
# PROBLEM 3 - fehlende Initialisierung der Good-Spad-Map
# In VL53L0X_perform_ref_spad_management() steht der Kommentar
#   "Currently the good spad map is initialised to 'All good'"
# direkt ueber einer Schleife, die NUR RefSpadEnables nullt. Die
# RefGoodSpadMap wird in dieser Portierung nirgends initialisiert und
# stammt ausschliesslich aus dem NVM (api_core.cpp:380). Ist das NVM
# korrumpiert - Folge eines frueheren Adresskonflikts, der Muell ins NVM
# zurueckschrieb - bleibt die Map leer. Dann liefert get_next_good_spad()
# konstant -1 und count_enabled_spads() 0, und die Kalibrierung scheitert
# dauerhaft mit -50 (REF_SPAD_INIT) - unabhaengig von Ziel, Abstand und
# Licht. Genau das wurde auf der Hardware beobachtet: weisses Papier auf
# 22 cm, trotzdem -50.
#
# LOESUNG 3
# Leere Map auf "alle SPADs gut" setzen, wie es der Vendor-Kommentar
# vorsieht. Konservativ: nur Bytes, die vollstaendig 0 sind, werden
# gefuellt. Eine belegte Map (echte Totalausfaelle aus dem NVM) bleibt
# unangetastet.
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


# ---- CPP: Adresswechsel vor DataInit
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

# ---- H: oeffentliche Wrapper-Methode fuer die Ref-SPAD-Kalibrierung
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

# ---- CAL: fehlende Initialisierung der Good-Spad-Map ergaenzen
content = read(CAL_REL)
if CAL_NEW in content:
    print("[patch-vl53l0x] CAL: Good-Spad-Map-Initialisierung bereits vorhanden")
elif CAL_ANCHOR in content:
    content = content.replace(CAL_ANCHOR, CAL_NEW, 1)
    write(CAL_REL, content)
    print("[patch-vl53l0x] CAL: Good-Spad-Map-Initialisierung ergaenzt")
else:
    fail("CAL-Ankerzeile nicht gefunden - hat sich die Library geaendert?")
