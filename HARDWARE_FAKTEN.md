# Feste Hardware-Fakten EduBot (vom Nutzer bestaetigt)

## 1. Motorstrom: 200 mA ist der Nennstrom
`200 mA RMS` ist der Nennstrom der NEMA8 **laut Datenblatt**. Nicht erhoehen.
Wer den Wert "optimiert", laeuft den Motor ueber seine Spezifikation: er vibriert,
heizt sich hoch und verliert Schritte. In `src/Config.h` ist `tmcCurrent = 200`
mit Kommentar festgeschrieben.

## 2. Motorgeometrie
| Groesse | Wert |
|---|---|
| NEMA8 | **200 Vollschritte** pro Umdrehung |
| Rad-Durchmesser | **35 mm** |
| Radumfang | `pi * 35` = 109,9558 mm |

Umrechnung (in `src/Config.h`, `geoStepsForMM` / `geoStepsForDeg`):

```
Schritte fuer mm     = mm  * 200 * Microsteps / 109,9558
Schritte fuer Grad    = Grad / 360 * 200 * Microsteps
```

Beides wird in **physikalischen Einheiten (mm/Grad)** gespeichert und erst zur
Laufzeit in Schritte umgerechnet. Deshalb ist die Strecke bei **jeder**
Microstep-Einstellung identisch:

| Microsteps | Schritte/Kreis | 100 mm (= 10 cm) | 90 Grad |
|---|---|---|---|
| 16  | 3 200  | 2 910  | 800  |
| 32  | 6 400  | 5 821  | 1 600 |
| 64  | 12 800 | 11 641 | 3 200 |
| 128 | 25 600 | 23 282 | 6 400 |
| 256 | 51 200 | 46 564 | 12 800 |

## 3. Die 3 cm / 90 Grad sind keine Motordaten
`quizForwardMM = 27,5` (ein Rasterfeld = 50 Vollschritte, aus Rad 35 mm) und
`quizTurnDeg = 195` (Werkseinstellung, gemessen: 195 Kommando-Grad = 90 Grad real;
`QUIZ_TURN_DEG = 90` bleibt nur die nominelle Referenz fuer die Block-Skalierung)
sind **Soll-Strecken fuer die Einlernung**
(NVS: `quizMM`/`quizDeg`, per `STEPS:<mm>`/`ANG:<grad>`).
Sie stammen aus der Zeit mit den alten Motoren und haben mit den neuen Motoren
nichts zu tun. Motor-Daten sind ausschliesslich Punkt 2.

## 4. Vibration ist kein Effekt der Mikrostep-Umstellung
Brummen trat bei **alten und neuen** Motoren auf. Im Code ist der wahrscheinlichste
Kandidat: es gibt bewusst **keine Beschleunigungsrampe**
(`src/Motor.cpp`, `src/Drive.h`) - sofort volle Drehzahl.

Ausgeschlossen:
- **Vref-Poti am TMC2209**: irrelevant, weil `I_scale_analog(false)` den Strom
  per UART setzt (`src/Motor.cpp`).
- **Schrittraten-Resonanz**: bei 128 Microsteps sind 10 000 Schritte/s nur
  36,8 mm/s, die Step-Grundfrequenz liegt bei 78 Hz.
