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
// Loesung: ein kurzer Mutex um jede I2C-Sequenz. Sequenzen sind kurz
// (<< 5 ms), der Wait-Bit darf also knapp sein - im Fehlerfall wird die
// Messung uebersprungen statt zu blockieren.
#pragma once

#include <Arduino.h>
#include <semphr.h>

// Mutex anlegen (einmalig beim Boot, vor colorSetup()).
void i2cBusInit();

// Bus fuer 'timeoutMs' sperren. true = gesperrt, false = Timeout.
bool i2cBusLock(uint32_t timeoutMs = 25);

// Sperre freigeben (nur aufrufen, wenn i2cBusLock() true geliefert hat).
void i2cBusUnlock();
