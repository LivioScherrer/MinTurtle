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
| Rad-Durchmesser | **35 mm** (gemessen) |
| Radumfang | `pi * 35` = 109,9558 mm |
| Strecken-Trim | **1,0101** (`DRIVE_TRIM`; Fahrprobe: 10 cm -> 1 mm zu weit) |
| Effektiver Radumfang | `109,9558 * 1,0101` = 111,0663 mm |

Umrechnung (in `src/Config.h`, `geoStepsForMM` / `geoStepsForDeg`):

```
Schritte fuer mm     = mm  * 200 * Microsteps / 111,0663
Schritte fuer Grad    = Grad / 360 * 200 * Microsteps
```

Beides wird in **physikalischen Einheiten (mm/Grad)** gespeichert und erst zur
Laufzeit in Schritte umgerechnet. Deshalb ist die Strecke bei **jeder**
Microstep-Einstellung identisch:

| Microsteps | Schritte/Kreis | 100 mm (= 10 cm) | 90 Grad |
|---|---|---|---|
| 16  | 3 200  | 2 881  | 800  |
| 32  | 6 400  | 5 762  | 1 600 |
| 64  | 12 800 | 11 525 | 3 200 |
| 128 | 25 600 | 23 049 | 6 400 |
| 256 | 51 200 | 46 099 | 12 800 |

## 3. Die 3 cm / 90 Grad sind keine Motordaten
`quizForwardMM = 27,8` (ein Rasterfeld = 50 Vollschritte, aus Rad 35 mm + Trim) und
`quizTurnDeg = 195` (Werkseinstellung, gemessen: 195 Kommando-Grad = 90 Grad real;
`QUIZ_TURN_DEG = 90` bleibt nur die nominelle Referenz fuer die Block-Skalierung)
sind **Soll-Strecken fuer die Einlernung**
(NVS: `quizMM`/`quizDeg`, per `STEPS:<mm>`/`ANG:<grad>`).
Sie stammen aus der Zeit mit den alten Motoren und haben mit den neuen Motoren
nichts zu tun. Motor-Daten sind ausschliesslich Punkt 2.

## 4. Vibration ist kein Effekt der Mikrostep-Umstellung
Brummen trat bei **alten und neuen** Motoren auf. Im Code war der
wahrscheinlichste Kandidat: **keine Beschleunigungsrampe** - sofort volle
Drehzahl. Seit dem Rampen-Umbau haben Positionsfahrten eine sanfte Rampe
(`MOTOR_RAMP_MS = 150`, `src/Motor.h`, AccelStepper `run()`/`setAcceleration()`);
das freie Fahren bleibt direkt. Ob das Brummen damit verschwindet, ist noch
nicht verifiziert.

Ausgeschlossen:
- **Vref-Poti am TMC2209**: irrelevant, weil `I_scale_analog(false)` den Strom
  per UART setzt (`src/Motor.cpp`).
- **Schrittraten-Resonanz**: bei 128 Microsteps sind 10 000 Schritte/s nur
  36,8 mm/s, die Step-Grundfrequenz liegt bei 78 Hz.
