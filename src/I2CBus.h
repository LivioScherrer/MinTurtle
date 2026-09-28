// EduBot V2 - Gemeinsamer I2C-Bus-Lock
// =====================================================
// Auf dem I2C-Bus hängen zwei Geraete, die von ZWEI Tasks/Core bedient
// werden:
//   * TCS34725 (Farbe)  -> colorSensorTask auf Core 0
//   * VL53L0X (Distanz) -> Main-Loop / blockierende Spiel-Moves auf Core 1
// Die Arduino-TwoWire-Implementierung ist NICHT threadsicher: Werden
// Transaktionen verzahnt, kollidieren Adress-Byte, Registerzeiger und
// Datenbytes. Sichtbare Folge war der VL53L0X-Fehler -50 (REF_SPAD_INIT):
// waehrend StaticInit riss der Farbsensor-Recovery-Pfad mit resetI2CBus()
// (pinMode + 9 Taktimpulse + Wire.begin()) den Bus mid-transaction um.
//
// Loesung: ein kurzer Mutex um jede I2C-Sequenz. Im Fehlerfall wird die
// Messung uebersprungen statt zu blockieren.
//
// ZUR UEBERPRUEFUNG DER "kurz"-ANGABE: sie stimmt nicht fuer den Farbsensor.
// Adafruit_TCS34725::getRawData() endet in einem
//   delay((256 - IntegrationTime) * 12/5 + 1)
// Der Wert ist eine Mikrosekunden-Angabe, delay() will aber Millisekunden -
// ein Einheitenfehler im Vendor-Code, der um Faktor 1000 zu lang ist. Bei der
// eingestellten 50-ms-Integration sind das 51 ms gehaltener Mutex von ~201 ms
// Takt, also rund 25 % Busbelegung. Folge: distTick() und readRawMM()
// scheitern regelmaessig am Lock und liefern den gehaltenen Wert -
// unkritisch, aber die DIST:-Meldungen werden dadurch duenn. Die echte
// Loesung waere, die Integration zu verkuerzen oder den Vendor-delay
// zu patchen; bis dahin hier bewusst dokumentiert statt stillschweigend
// falsch.
// Wichtig: i2cBusLock() ist FAIL-CLOSED - ohne angelegte Sperre wird der
// Zugriff verweigert (frueher: erlaubt).
#pragma once

#include <Arduino.h>
#include <semphr.h>

// Mutex anlegen (einmalig beim Boot, vor colorSetup()).
void i2cBusInit();

// Bus fuer 'timeoutMs' sperren. true = gesperrt, false = Timeout.
bool i2cBusLock(uint32_t timeoutMs = 25);

// Sperre freigeben (nur aufrufen, wenn i2cBusLock() true geliefert hat).
void i2cBusUnlock();
