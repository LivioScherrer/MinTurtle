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
| Rad-Durchmesser | **38 mm** (gemessen, 2026-10 neue Raeder montiert) |
| Radumfang | `pi * 38` = 119,3805 mm |
| Strecken-Trim | **1,0101** (`DRIVE_TRIM`; Fahrprobe: 10 cm -> 1 mm zu weit) |
| Effektiver Radumfang | `119,3805 * 1,0101` = 120,5863 mm |
| Weg pro Vollschritt | `120,5863 / 200` = 0,6029 mm (`MM_PER_FULL_STEP`) |

Umrechnung (in `src/Config.h`, `geoStepsForMM` / `geoStepsForDeg`):

```
Schritte fuer mm     = mm  * 200 * Microsteps / 120,5863
Schritte fuer Grad    = Grad / 360 * 200 * Microsteps
```

Beides wird in **physikalischen Einheiten (mm/Grad)** gespeichert und erst zur
Laufzeit in Schritte umgerechnet. Deshalb ist die Strecke bei **jeder**
Microstep-Einstellung identisch:

| Microsteps | Schritte/Kreis | 100 mm (= 10 cm) | 90 Grad |
|---|---|---|---|
| 16  | 3 200  | 2 654  | 800  |
| 32  | 6 400  | 5 307  | 1 600 |
| 64  | 12 800 | 10 615 | 3 200 |
| 128 | 25 600 | 21 230 | 6 400 |
| 256 | 51 200 | 42 459 | 12 800 |

Zum Nachrechnen, am Beispiel 32 Microsteps:

```
Schritte je mm = 200 * 32 / 120,5863   = 53,074
100 mm          = 100 * 53,074         = 5 307,4  -> 5 307 Schritte
90 Grad         = 90 / 360 * 200 * 32  = 1 600    -> 1 600 Schritte
```

Die Microstep-Zahl steckt nur als Faktor drin - deshalb stimmt die Strecke bei
jeder Einstellung, nur feiner oder groeber.

**Obergrenze `MICROSTEPS_MAX = 32`.** Bei mehr Microsteps braucht 240 mm/s
mehr Schritte/s, als der 50-us-Puls-Task erzeugen kann
(`MAX_DRIVE_STEPS_S` = 20 000). Oberhalb von 32 bekommt der Motor nur jeden
fuenften Puls: er dreht dann nicht, sondern pfeift, weil SpreadCycle bei so
sporadischen Pulsen kein Drehmoment aufbaut. `configValidate()` senkt zu grosse
NVS-Werte beim Booten darauf ab. Wer feinere Aufloesung braucht, zahlt sie mit
Tempo.

Dieselbe Schrittraten-Grenze begrenzt auch die **hoechste Fahrtgeschwindigkeit**:

| Microsteps | Schritte/mm | 20 000 Schritte/s ergeben |
|---|---|---|
| 16  | 26,537 | ~753 mm/s |
| 32  | 53,074 | ~377 mm/s |
| 128 | 212,296 | ~94 mm/s |

Wer mehr Tempo will, muss Microsteps senken.

## 3. Die 3 cm / 90 Grad sind keine Motordaten
`quizForwardMM = 30,15` (ein Rasterfeld = 50 Vollschritte, aus Rad 38 mm + Trim) und
`quizTurnDeg = 195` (Werkseinstellung, gemessen: 195 Kommando-Grad = 90 Grad real;
`QUIZ_TURN_DEG = 90` bleibt nur die nominelle Referenz fuer die Block-Skalierung)
sind **Soll-Strecken fuer die Einlernung**
(NVS: `quizMM`/`quizDeg`, per `STEPS:<mm>`/`ANG:<grad>`).
Sie stammen aus der Zeit mit den alten Motoren und haben mit den neuen Motoren
nichts zu tun. Motor-Daten sind ausschliesslich Punkt 2.

## 4. Vibration ist kein Effekt der Mikrostep-Umstellung
Brummen trat bei **alten und neuen** Motoren auf. Im Code war der
wahrscheinlichste Kandidat: **keine Beschleunigungsrampe** - sofort volle
Drehzahl. Heute gibt es `config.rampMs` (Werkwert 150 ms, `src/Motor.h` /
`src/Config.h`); sie gilt für freies Fahren als Software-Rampe im Motor-Task
und für Fahrten nach Strecke als AccelStepper-Rampe. Per `RAMP:<ms>` und in
der Web-UI einstellbar, `0` schaltet sie ganz ab (sofort volle Drehzahl, exakt
das alte Verhalten) - damit lässt sich der Zusammenhang ohne Neubau prüfen.
Das STOPP bleibt auch bei großer Rampe hart. Ob das Brummen damit
verschwindet, ist noch nicht verifiziert.

Ausgeschlossen:
- **Vref-Poti am TMC2209**: irrelevant, weil `I_scale_analog(false)` den Strom
  per UART setzt (`src/Motor.cpp`).
- **Schrittraten-Resonanz**: bei 128 Microsteps sind 10 000 Schritte/s nur
  47,1 mm/s, die Step-Grundfrequenz liegt bei 78 Hz.
