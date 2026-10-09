# EduBot V2

Firmware fuer einen Educational-Roboter auf **Seeed XIAO ESP32S3** mit zwei
NEMA8-Steppern (TMC2209), Farbsensor (TCS34725), Distanzsensor (VL53L0X),
NeoPixel-Ring, BLE-Steuerung und ESP-NOW-Multiplayer.

Zielgruppe der Web-UI ist Grundschule - alle Spiele laufen ueber Bloecke
(Wiederhole, Fahre, Drehe, Warte, Lies Farbe), die per Scratch-Logik
zusammengeklickt werden.

---

## Bauen und Flashen

```bash
pio run                 # kompilieren
pio run -t upload       # auf den Roboter flashen (COM11 in platformio.ini)
pio device monitor      # Serial-Log (115200 baud)
```

Ziel: `seeed_xiao_esp32s3`, Flash 3.3 MB (aktuell ~42 % belegt), RAM
327 KB (aktuell ~21 % belegt).

Zwei Web-Seiten, beide Single File, kein Build-Schritt. Sie sprechen denselben
BLE-Dienst an und werden direkt aus dem Browser geöffnet (Chromium-Browser
nötig, Web Bluetooth).

| Datei | Für wen | Inhalt |
|---|---|---|
| **`Edu_V2.html`** | **Schüler** | Alle Spiele, Freifahren, Code-Bausteine, Lernkarte. **Keine Diagnose**, keine Fehlermeldungen über unbekannte Befehle. |
| **`Edu_V2_dev.html`** | Entwickler | Kleine Konsole: Protokoll jeder gesendeten und empfangenen Zeile, geparste Werte (TMC, Geometrie, Tempo, Distanz, Freifahren-Kette), freier Befehlseingabe. |

Warum das getrennt ist: `handleCommand()` protokolliert jeden Befehl mit
`Serial.printf("[CMD] %s\n", …)`, und die Firmware antwortet auf Unbekanntes
mit `UNKNOWN:<befehl>`. Auf der Schülerseite wird das **nicht** angezeigt —
für ein Kind ist „Unbekannter Befehl: PROG:OLDRUN" eine tote Fehlermeldung, an
der es nichts zu tun hat, während der Roboter völlig normal läuft. Wer sehen
will, was wirklich ankommt, öffnet die Entwicklerseite.

> **Ein `UNKNOWN:` im Protokoll bedeutet fast immer:** der Browser hat eine
> **alte Kopie** der Web-Seite gecacht und sendet Befehle, die die aktuelle
> Firmware nicht mehr kennt. Abhilfe: Seite neu laden (Strg+F5). Die
> Entwicklerseite zeigt dann sofort, welcher Befehl es ist.

Beide Seiten brauchen die gleichen BLE-UUIDs (siehe `BLEManager.cpp` und den
Kopf beider HTML-Dateien).

---

## Hardware / Verdrahtung

### Motoren (TMC2209, STEP/DIR + UART)

Beide Treiber hängen an einem **gemeinsamen Single-Wire-UART-Bus**.
VIO = 3V3, VM = Motorspannung (ohne VM antwortet der Treiber nicht).

> **Diese Tabelle ist die am Fahrroboter geprüfte IST-Verdrahtung (2026-09).**
> Sie stimmt mit `src/Motor.h` überein. Eine frühere Fassung dieses README
> stand hier spiegelverkehrt (U2/J1 als „links") — das war die Verdrahtung,
> bei der jede Drehung auf der Stelle in die falsche Richtung lief. Wer
> `Motor.h` nicht glaubt, sollte `PIN_STEP_*`/`TMC_ADDR_*` dort nachsehen,
> nicht hier.

| Signal | GPIO | Hinweis |
|---|---|---|
| STEP links | 2 (D1) | U4 / J5, TMC-Adresse `0b01` |
| DIR links  | 1 (D0) | |
| STEP rechts | 7 (D8) | U2 / J1, TMC-Adresse `0b00` |
| DIR rechts  | 3 (D2) | STEP/DIR auf dem PCB gedreht, hier kompensiert |
| EN (beide)  | 4    | LOW aktiv |
| UART TX     | 43 (D6) | über 1 kΩ (R9) an das PDN-Netz |
| UART RX     | 44 (D7) | direkt am PDN-Netz |

Adressen: links `0b01` (MS1=3V3, MS2=GND), rechts `0b00` (MS1=GND, MS2=GND).
R_Sense auf dem PCB: 560 mΩ.

> **Achtung, D8 ist auf dem ESP32-S3 GPIO 7** (auf dem klassischen XIAO
> ESP32 war D8 = GPIO 8). `PIN_STEP_R 7 // D8` in `Motor.h` ist also korrekt
> und sieht nur falsch aus — bitte nicht auf 8 „korrigieren“.

#### Drehrichtung (`LEFT_MOTOR_SIGN` / `RIGHT_MOTOR_SIGN`)

Die Drehrichtung steckt in **zwei Konstanten** am Anfang von `src/Config.h`:

```cpp
constexpr int8_t LEFT_MOTOR_SIGN  = +1;   // links  (U4/J5, D0/D1)
constexpr int8_t RIGHT_MOTOR_SIGN = -1;   // rechts (U2/J1, D2/D8)
```

Feste Konstanten, **bewusst keine NVS-Werte und keine Web-UI-Einstellung.**
Ein einmal falsch gespeicherter Wert würde jeden späteren korrigierten
Default bei jedem Build überschreiben – die Quelle soll die einzige
Wahrheit bleiben.

Weiter wichtig:

- Beide Werte werden **an jeder Stelle angewendet** – `setMotorSpeed()`
  multipliziert intern mit dem Vorzeichen, die zehn `prepPositionMove()`-
  Aufrufer ebenfalls. Der Faktor wirkt also **genach einmal**. Wer ihn in
  `Motor.cpp` nochmals umkehrt, dreht beide Motoren wieder um.
- **Die beiden Werte müssen unterschiedlich sein.** Sind sie gleich, drehen
  beide Räder gleichsinnig und der Roboter dreht sich auf der Stelle statt
  zu fahren. Ein `static_assert` in `Config.h` bricht den Build in dem
  Fall ab – lieber kein Build als ein Roboter, der im Kreis fährt.

**Symptome falscher Werte** (Roboter frei auf dem Tisch, `W` gedrückt):

| Beobachtung | Ursache |
|---|---|
| dreht sich auf der Stelle | beide Werte gleich (Build bricht ab) |
| fährt rückwärts | Werte unterschiedlich, aber beide umgekehrt zur nötigen Polarität |
| fährt geradeaus, dreht auf der Stelle in die falsche Richtung | Fahrzeichen stimmt, aber `gameTurnSteps()`-Logik prüfen |

#### Modus-Automatik: `TPWMTHRS` und StallGuard

Zwei Register, die früher nicht gesetzt wurden und jetzt in `configureOne()`
mitlaufen (`src/Motor.h`):

- **`TPWMTHRS` = 601** → Umschaltung auf SpreadCycle ab 60 mm/s. Vorher stand
  hier `TPWMTHRS(0)`, und `0` bedeutet laut Datenblatt *Disabled*: der Treiber
  blieb bei **jeder** Geschwindigkeit in StealthChop, also auch dort, wo
  StealthChop die wenigste Drehmomentreserve hat. Der Wert ist unabhängig von
  der Microstep-Zahl (er bezieht sich immer auf 1/256 Schritt), ein Wert reicht
  für alle Microstep-Einstellungen.
- **`TCOOLTHRS` = 2402** → StallGuard erst ab 15 mm/s auswerten. Unterhalb
  dieser Geschwindigkeit ist er nicht verlässlich.
- **`SGTHRS` = 100** → Stall bei `SG_RESULT ≤ 200`. Der Wert ist
  anwendungsabhängig, das Datenblatt gibt keine Vorgabe. **Nichts in der
  Firmware handelt darauf** — er wird nur gelesen und zusammen mit beiden
  `SG_RESULT`-Werten gemeldet. Ein zu niedriger Wert kann deshalb keinen
  Schaden anrichten, und der echte Wert ist am Roboter ablesbar.

Die Herleitung der Registerwerte steht ausgeschrieben bei
`TMC_SPREADCYCLE_AB_MM_S` in `Motor.h`: `Registerwert = 2^24 · Radumfang /
(51200 · v[mm/s])`, gegengeprüft an der OTP-Tabelle des Herstellers.

### I2C-Sensoren

| Sensor | Adresse | Pins |
|---|---|---|
| TCS34725 (Farbe) | 0x29 | SDA=GPIO5, SCL=GPIO6 |
| VL53L0X (Distanz) | 0x30 (nach Umzug) | gleicher Bus |

> **Reihenfolge zwingend:** `distSetup()` **vor** `colorSetup()`.
> Beide Sensoren haben Default-Adresse 0x29; der VL53L0X wird beim Boot
> auf 0x30 umadressiert, damit sie koexistieren koennen. Die Umadressierung
> ist fluechtig und wird bei jedem Boot erneut durchgefuehrt.
>
> **Wichtig:** Sind beide Sensoren gleichzeitig physisch am Bus, reicht die
> Reihenfolge allein nicht - ein auch noch nicht konfigurierter TCS34725
> antwortet bereits auf 0x29 und verfaelscht damit die NVM-Reads des
> VL53L0X. Details und Loesungswege siehe
> [VL53L0X und der Adresskonflikt](#vl53l0x-und-der-adresskonflikt-wichtigste-falle-im-projekt).

### Weitere Peripherie

- **NeoPixel-Ring** (LED-Strip, 7 LEDs, GPIO 8) - Statusanzeige, siehe unten
- **Motor-Referenz**: NEMA8 8HS11-0204S (0,2 A Nennstrom, 24 Ω, 8 mH)

### LED-Statusanzeige (NeoPixel-Ring)

Der Ring ist die Statusanzeige des Roboters. `LedStrip.cpp` ist eine
Muster-Engine: `ledStripShowXxx()` waehlt nur ein Muster, `ledStripTick()`
zeichnet es zeitbasiert (max. 25 Bilder/s, nicht blockierend). Die
Main-Loop waehlt in `ledPatternUpdate()` **ein** Muster nach Prioritaet,
Alarm (Not-Aus) und Ergebnisblitze uebersteuern die Kette intern fuer eine
kurze Haltezeit.

| Situation | Anzeige |
|---|---|
| Boot | Pixel-Selbsttest (jede LED einmal weiss), danach Sensor-Selbsttest: 2x gruen = beide Sensoren OK; linker Teil magenta blinkend = Farbsensor fehlt, rechter Teil rot blinkend = Distanzsensor fehlt |
| Ruhezustand, App verbunden | blaues Atmen |
| Ruhezustand, keine App | oranges Atmen (wartet auf Verbindung) |
| Slalom gepaart (Gegner verbunden, Start noch offen) | cyanfarbenes Atmen |
| Freifahren | gruenes Lauflicht in Fahrtrichtung; Drehen auf der Stelle: zwei Punkte wandern nach aussen |
| Quiz (Korrigieren) | Fortschrittsbalken: gruen solange alles stimmt, sonst rot; waehrend des Farblesens bleibt der Ring dunkel; Ende: gruener bzw. roter Blitz |
| Memory | Sequenz-Ansicht: violettes Lauflicht; Eingabe: gruener Fortschrittsbalken, der naechste Punkt blinkt |
| Linienfolger | gruener Scanner (Linie erkannt), oranges Blinken (Suche), rotes langsames Blinken (Hindernis) |
| Rennen | gelbes Lauflicht |
| Slalom Bauphase | Zielfarbe pulsiert |
| Slalom Countdown | Startampel: erst 2, dann 4, dann 6 rote LEDs, bei GO die ganze Reihe gruen |
| Slalom entschieden | Sieg = Regenbogen, Niederlage = rotes Atmen |
| Not-Aus (`STOP`) | rotes Doppelblinken, 3 s |

Zwei Regeln sind bewusst so gewaehlt:

1. **Waehrend Farbmessungen bleibt der Ring dunkel** (Slalom-Rennen,
   Quiz-Lesephase, ausgefuehrte Programme). Eigenes LED-Licht koennte sonst
   als Farbe auf dem Boden erkannt werden. Der Not-Aus-Alarm blinkt trotzdem.
2. **Der S-Block** (`Licht`-Baustein) nutzt dieselbe Anzeige: Slots 0-3 zeigen
   die Zielfarbe, Slot 4/5 die Distanz-Bedeutung NAH (dunkelrot) / FREI
   (dunkelgruen), Slot 6 schaltet aus.

### Code-Bausteine (Blockprogrammierung)

Die Web-UI baut das Programm als kurzen String (`PROG:SET:<enc>`). Die
Bausteine sind nach **Aufgabe** gruppiert, nicht nach Technik — genau diese
technische Gruppierung war es, an der man nicht durchblickte.

| Reiter | Bausteine | Kodierung |
|---|---|---|
| Fahren | vorwärts/rückwärts cm, drehe links/rechts ° | `F20` `B15` `L90` `R90` |
| **Fahre bis** | **fahre bis [Farbe]**, **fahre bis Abstand [mm]** | **`P0`**…`P5` **`D150`** |
| Warten | warte s, warte bis [Farbe], warte bis Abstand mm | `W2` `V1` `X150` |
| Wiederholen | wiederhole n mal, wiederhole bis [Farbe] | `U4(...)` `C2(...)` |
| Genau fahren | vorwärts/rückwärts mm | `M45` `N45` |
| Sonstiges | Licht, Stopp, kurze Pause, warte bis frei mm | `S1` `Q` `Z5` `G300` |

Slot 0-3 sind ROT/GELB/GRÜN/BLAU, Slot 4 = Hindernis erkannt, Slot 5 = Weg frei.
Der Farb-Chip im Baustein wird durch Anklicken weitergeschaltet.

**„Fahre bis …" läuft für alle drei Fälle über eine Routine**
(`progDriveUntil()`): eine einzige Rampe, dann durchgehende Fahrt mit einer
Prüfung alle ~10 ms. Vorher gab es zwei getrennte Wege, und der Abstands-Baustein
fuhr in 10-mm-Etappen mit jeweils eigener Rampe — bei 150 ms Anfahrzeit kostet
jede Etappe mehr Zeit im Anfahren als im Fahren.

Drei Dinge, die dabei bewusst so gelöst sind:

1. **Flanken-Logik bei Farben.** Steht der Roboter schon auf der Zielfarbe,
   hält er nicht sofort, sondern erst bei der *nächsten* Farbe. Sonst wäre
   „fahre bis Rot" auf rotem Boden ein Baustein ohne Wirkung.
2. **Zwei Sicherheitsnetze**, unabhängig voneinander: höchstens 1,5 m und
   höchstens 20 s. Ohne sie fährt der Roboter bei blindem Sensor endlos weiter.
3. **Kein zusätzlicher Halt auf der Hindernis-Schwelle** beim Abstands-Baustein.
   Die Schwelle steht an anderer Stelle und würde den Baustein beenden, bevor
   sein eigener Wert erreicht ist — der Roboter hielte an, ohne dass klar wäre,
   warum.

Der alte Weg „F/B mit angehängtem E-Wertblock" (`FE2`, im Parser gleich auf
`OP_DRIVE_UNTIL` umgewandelt) wird weiterhin gelesen und läuft jetzt über
dieselbe Routine. Der E-Wertblock selbst steht nicht mehr in der Palette: für
„bis Farbe" gibt es mit `P` genau einen Weg, und zwei Wege für dieselbe Sache
waren einer zu viel.

### Drift: „Software oder Hardware?" — die Messgröße dafür

Jedes Freifahren-Manöver (`driveUpdate()`) gibt am Ende automatisch eine Zeile
aus:

```
[DRV] Fahrt beendet: Schritte L=16862 R=16861 (Diff 1)
[DRV]   Zaehler praktisch gleich -> Drift ist MECHANISCH
```

Die Zähler sind `AccelStepper::currentPosition()` — sie zählen **jeden
erzeugten Puls**, unabhängig davon, ob das Rad rollt. Damit ist die Frage
entscheidbar statt vermutbar:

| Befund | Bedeutung |
|---|---|
| Zähler gleich (Diff ≤ 2), Roboter driftet sichtbar | **mechanisch**: Rad steht schief, Getriebespiel, Reibung, Spannung im Chassis |
| Zähler unterscheiden sich | **Software**: die Räder werden ungleich angesteuert (zuerst `leftSign`/`rightSign`, dann der Puls-Task) |
| Zähler gleich *und* Strecke stimmt nicht | Zeitbasis: `MOTOR_TICK_US`, `MAX_DRIVE_STEPS_S` |

Gleiche Zahlen stehen auch in `DRV?` (`pos=<L>/<R> d=<Diff>`) und auf der
Diagnose-Seite. Reine Diagnose — am Fahrverhalten ändert sich nichts.

### TMC2209: Registerkarte nicht aus der Bibliothek übernehmen

**Das ist die wichtigste Falle im ganzen Projekt.** TMCStepper adressiert den
TMC2209 mit der Registerkarte des **TMC2130**. Dort liegt `CHOPCONF` auf
**0x06** — beim TMC2209 ist 0x06 aber **IOIN** und damit nur lesbar.

Gemessen auf dem Roboter (2026-10, COM9): `toff`, `mres`, `intpol`, `pwm_autoscale`,
Strom und Hold waren nie angekommen. Stattdessen liefen beide Treiber auf
Auslieferungswerte — mit **IRUN = 31 (voller Strom)**, also rund **396 mA RMS**
bei 560 mΩ Fühlerwiderstand, für einen Motor mit **0,2 A Nennstrom**.

| Register | TMC2209 (UART) | TMC2130 (was TMCStepper nutzt) |
|---|---|---|
| `GCONF` | 0x00 | 0x00 ✔ zufällig gleich |
| `CHOPCONF` | **0x08** | 0x06 ✘ = IOIN, nur lesbar |
| `IHOLD_IRUN` (Strom) | **0x10** | — |
| `TPOWER` | 0x11 | 0x19 ✘ |
| `TSTEP` / `TPWMTHRS` / `TCOOLTHRS` | 0x12 / 0x13 / 0x14 | 0x13 / 0x15 / 0x16 ✘ |
| `VACTUAL` | 0x22 | ✔ |
| `SGTHRS` / `SG_RESULT` | 0x40 / 0x41 | ✔ |

Deshalb schreibt `configureOne()` **alle Register selbst** über
`tmcWriteChecked()` mit den Adressen aus dem Datenblatt.

**Zwei Fallen, die zusammen sehr viel Zeit gekostet haben:**

1. **Nur-schreibbare Register kann man nicht zurücklesen.** `CHOPCONF`,
   `IHOLD_IRUN`, `TPOWER`, `TPWMTHRS`, `TCOOLTHRS`, `SGTHRS` und `NODECONF`
   sind im Datenblatt mit `W` gekennzeichnet. Ein Readback liefert dort
   konstant 0 — daraus entstand die Fehldiagnose „die Schwellen kommen nicht
   an". **Belegbar sind sie nur über `IFCNT`:** „This register becomes
   incremented with each successful UART interface write access." Nach der
   Konfiguration muss dieser Zähler um die Anzahl der Schreibzugriffe
   gewachsen sein. Genau das prüft `configureOne()` jetzt.
2. **`ENN` ist auch ein Reset.** Das Datenblatt (5.5.1): „Indicates that the
   IC has been reset since the last read access to GSTAT. **All registers
   have been cleared to reset values.**" Der Auto-Standby schaltet nach 3 s auf
   EN=HIGH und löscht damit jedes Register — ohne jede Fehlermeldung.
   `tmcEnable()` konfiguriert deshalb neu, sobald der Treiber wieder
   eingeschaltet wird.

**MS1/MS2 tragen doppelte Funktion:** Sie wählen die UART-Adresse *und* — solange
`mstep_reg_select = 0` — die Microstep-Auflösung (Datenblatt 4.3 und 5.5.1).
Diese Platine nutzt sie für die Adresse (links `0b01`, rechts `0b00`); ohne
`mstep_reg_select` wären die Räder auf 128 bzw. 256 Microsteps gelaufen,
während die Firmware für beide 32 rechnet.

**Strom:** `IRUN` (Bit 12..8) und `IHOLD` (Bit 4..0) des Registers 0x10 sind
Strom-*Skalen* 0..31 (0 = 1/32, 31 = 32/32). Die Widerstände und VREF legen die
**Obergrenze** fest (~396 mA bei 560 mΩ und `vsense = 0`), `IRUN` skaliert nur
darunter. Für 200 mA folgt `IRUN = 16` → **210 mA**. Das Datenblatt empfiehlt
ausdrücklich „IRUN is 16 to 31 for best microstep performance" — darunter
wird deshalb gekappt. `IHOLD > IRUN` heißt: im Stillstand fließt **mehr** Strom
als beim Fahren; `tmcPrintStatus()` warnt davor.

### `MOT:SPD?` — Momentaufnahme mitten in der Fahrt

Für einen **dauerhaften** Drift (die Startmessung ist nach ~50 ms vorbei,
ein Drift zeigt sich über Sekunden) liest dieser Befehl auf Abruf
`VACTUAL` und `MSCNT` beider Treiber — 4 Reads, rund 8 ms:

```
VACTUAL L=-4310 R=-4290 uSchritt/s  (~-126.5 / -125.9 mm/s)  Diff 0 %
BEWERTUNG: beide Raeder drehen gleich schnell. Ein Drift kommt dann
           NICHT aus unterschiedlichem Vortschritt - sondern aus
           seitlichem Versatz (Achse schief, Boden, Spiel).
```

Die beiden Messungen zusammen sind die eigentliche Entscheidung:

| `[DRV]`-Zähler | `MOT:SPD?` VACTUAL | Bedeutung |
|---|---|---|
| gleich | gleich | **kein Fortschrittsunterschied** — der Roboter wird von außen gelenkt: Achse schief, Boden, Rad schief |
| gleich | **verschieden** | **mechanisch**: ein Rad kommt nicht voran — Schlupf, Getriebereibung, schleifendes Rad |
| **verschieden** | — | **Software**: die Räder werden ungleich angesteuert |

Bewusst nicht automatisch: 8 ms Lesen mitten in der Fahrt sind eine echte
Störung des 50-µs-Puls-Tasks — ein guter Anlass, sie nur auf Abruf zu machen.

**Richtungsunabhängigkeit als Ausschlusskriterium:** `runSpeed()` rechnet in
AccelStepper mit `fabs()` und hält die Richtung getrennt in `_direction`.
Vorwärts und Rückwärts laufen damit durch denselben Codepfad; eine
Asymmetrie *im* Puls-Task ist ausgeschlossen. Sie kann nur aus
unterschiedlichen Sollwerten je Rad entstehen — und genau das zeigen die
Zähler.

---

## Architektur

### Tasks und Kerne

| Task | Core | Aufgabe |
|---|---|---|
| `loop()` | 1 | Eingaben, Spiel-Logik, Kommando-Handler |
| Motor-Task | 0 | Step-Puls-Erzeugung, 50 µs-Takt (esp_timer + Task-Notification) |
| ColorSensor-Task | 0 | liest TCS34725 alle 150 ms, schreibt unter Mutex |

**Wichtig:** Der Motor-Task erzeugt nur die Pulse. Er liest **keinen**
Spiel-Zustand. Die Main-Loop setzt ausschliesslich Ziel-Geschwindigkeiten
bzw. Ziel-Positionen ueber die threadsichere API in `Motor.h`
(`setMotorSpeed`, `prepPositionMove`, `stopMotors`). `Motor.cpp` sichert
seinen Zustand mit einem eigenen Mutex.

Aus demselben Grund brauchen die Spiel-Flags (`currentGame`, `raceActive`,
`slalom*`) **kein** `volatile`: sie werden nur auf Core 1 gelesen und
geschrieben. ESP-NOW- und BLE-Events werden per Queue *an die Main-Loop*
weitergereicht und dort ausgewertet - die WiFi-/BLE-Callbacks selbst fassen
keinen Spiel-Zustand an.

### Module

| Datei | Verantwortung |
|---|---|
| `Edu_V2.cpp` | Einstieg, Spiel-Engine (blockierende Moves), Kommando-Dispatch, Quiz, Rennen, BLE-Handler, `loop()` |
| `Slalom.cpp/.h` | Farbmarker-Slalom komplett: Zustandsmaschine, ESP-NOW-Sync, Pairing, Retry |
| `Motor.cpp/.h` | 2 × TMC2209: Puls-Task, STEP/DIR, UART, Auto-Standby, Diagnose |
| `ColorSensor.cpp/.h` | TCS34725, Kalibrierung, Farb-Erkennung (`detectColorName`) |
| `DistanceSensor.cpp/.h` | VL53L0X, Adress-Umzug, Hysterese, Offset-Kalibrierung, Diagnose |
| `Drive.cpp/.h` | Frei-Fahren: Tastenstatus → Zielgeschwindigkeiten + Timeout |
| `GameEngine.h` | Spiel-Bewegungs-Primitiven (blockierend, mit Spielabbruch) |
| `ProgGame.cpp/.h` | Programm-Interpreter (Code-Bausteine), `progRunSlalom` |
| `MemoryGame.cpp/.h` | Memory-Sequenz |
| `LineFollower.cpp/.h` | Linienfolger |
| `EspNow.cpp/.h` | ESP-NOW-Peers, Race- und Slalom-Pakete, Event-Queue |
| `BLEManager.cpp/.h` | BLE-Service, Event-Queue, `sendStatus`, Serial-Polling |
| `Config.cpp/.h` | NVS-Konfiguration + Sanity-Validierung |
| `LedStrip.cpp/.h` | NeoPixel-Statusanzeige: Muster-Engine, Boot-Selbsttest, Alarm |

### Bewegung

Die Fahrgeschwindigkeit wird in **mm/s** vorgegeben (`config.driveSpeedMM`,
per `SPEED:` und Web-UI einstellbar, 20..600 mm/s) und zur Schrittrate
`FIXED_DRIVE_SPEED` umgerechnet - aus Rad-Durchmesser (plus Strecken-Trim
`DRIVE_TRIM` aus der Fahrprobe) und der aktuellen
Microstep-Zahl (`configStepsPerS()` in `Config.cpp`, gedeckelt bei
`MAX_DRIVE_STEPS_S` = 20 000 Schritte/s durch den 50-µs-Puls-Task).
`setMotorSpeed()` legt den Betrag immer darauf fest.

Die Geometrie dahinter: Rad **38 mm**, Umfang `pi * 38` = 119,3805 mm, mit Trim
`119,3805 * 1,0101` = 120,5863 mm, also **0,6029 mm pro Vollschritt**
(`MM_PER_FULL_STEP` in `Config.h`). Daraus folgen bei 32 Microsteps
`6400 / 120,5863` = **53,07 Schritte/mm**, also **5 307 Schritte für 100 mm**.

Weil `MAX_DRIVE_STEPS_S` die Obergrenze ist, begrenzt sie je Microstep-Zahl das
tatsächlich fahrbare Tempo:

| Microsteps | Schritte/mm | 20 000 Schritte/s ergeben |
|---|---|---|
| 16 | 26,54 | ~753 mm/s |
| 32 | 53,07 | ~377 mm/s |
| 128 | 212,30 | ~94 mm/s |

**Deshalb ist `MICROSTEPS_MAX = 32`.** Oberhalb davon braucht schon 240 mm/s
mehr Schritte/s, als der 50-µs-Puls-Task schafft; der Motor bekommt nur jeden
fünften Puls, dreht dann nicht, sondern pfeift, weil SpreadCycle bei so
sporadischen Pulsen kein Drehmoment aufbaut. `configValidate()` senkt zu große
NVS-Werte beim Booten darauf ab. Feinere Auflösung kostet Tempo — das ist eine
bewusste Entscheidung, kein Versehen.

Das **Anfahren und Abbremsen** ist eine sanfte Rampe: von 0 auf
`FIXED_DRIVE_SPEED` in `config.rampMs` (Werkwert 150 ms, per `RAMP:` und
Web-UI einstellbar, 0..2000 ms) - und genauso lang wieder herunter.
`0` = aus, dann fährt der Roboter überall sofort mit voller Geschwindigkeit.

Sie gilt an zwei Stellen, mit derselben Bedeutung:

| Wo | Wie | Warum |
|---|---|---|
| Freies Fahren (WASD) | Software-Rampe im Motor-Task: die Ist-Geschwindigkeit wandert pro 50-µs-Tick um `Beschleunigung × dt` auf das Ziel. Anfahren beim Drücken, **Abbremsen beim Loslassen** | AccelStepper kann nur mit einem Ziel rampe(n) - beim Freifahren gibt es keins, es geht nur um „wie schnell gerade" |
| Fahrten nach Strecke (Spiele, Blöcke, Einlern-Tests) | AccelStepper `run()`/`setAcceleration()`, die Schrittzahl bleibt exakt | bremst zusätzlich **vor** dem Ziel ab |

**Hart** bleibt, was im Notfall hart sein muss:

| | Stopp |
|---|---|
| STOP-Knopf / Not-Aus | sofort (`stopMotors()`) |
| Verbindung verloren (Befehls-Timeout) | sofort - eine Bremse über `config.rampMs` ließe den Roboter bei 2000 ms Anfahrzeit noch zwei Sekunden rollen, obwohl niemand mehr da ist |
| `TMC:EN:0` | sofort |
| Linienfolger | sofort - seine Zustandswechsel liegen bei 140 ms, ein Ausrollen liefe in den nächsten hinein |
| `gameStop` / Ende von `gameDrive` | sofort - Spiel-Pausen stehen vor Aktionen, die einen **stehenden** Roboter brauchen (Farbe messen, Drehung ansetzen) |

`setMotorSpeed(idx, 0, /*useRamp=*/true)` ist das weiche Ausrollen und greift
nur, wenn die Rampe an ist **und** keine Positionsfahrt läuft — die hat einen
festen Zielpunkt und eine Schrittzahl, die kann man nicht weich abbrechen.
Not-Aus, Timeout, Linienfolger und Spiele rufen deshalb `useRamp = false`.

Wer die Rampe selbst verstellt, muss den Weg über `motorUpdateSpeeds()`
nehmen - dort - und nur dort - wird das Flag gesetzt, das der Puls-Task liest.
Weil dort `setAcceleration()` intern `computeNewSpeed()` aufruft, und das im
Freifahr-Modus auf einem ins Unendliche wachsenden „Abstand zum Ziel" die
AccelStepper-Rampe starten würde, wird die aktuelle Geschwindigkeit danach
einmal wiederhergestellt.

Warum die Rampe in **Zeit** und nicht in Beschleunigung angegeben wird: nur die
Zeit ergibt unabhängig von Microstep-Zahl und Tempo dasselbe Fahrgefühl. Eine
mm/s²-Angabe müsste bei jedem Wechsel der Microsteps neu umgerechnet werden,
sonst fährt derselbe Wert je nach Motor-Einstellung schneller oder langsamer -
genau der Fehler, den die Geometrie-Umstellung in diesem Projekt vermeiden soll.

Die **Einschalt-Drehung** (kurze Drehung auf der Stelle direkt nach dem Boot,
damit der Roboter „lebt" und die Motoren ihre Ausgangsposition finden) hängt an
`config.spinMs` (Werkwert 500 ms) und `config.spinSpeed` (Werkwert 6000
Schritte/s, ein reiner Firmwarewert). Die Web-UI zeigt nur die Dauer, per
`SPIN:<ms>` einstellbar; `0` schaltet die Drehung **ohne** Wartezeit ab. Über
den Serial Monitor geht beides: `SPIN:<schritte/s>,<ms>`.

Deshalb bleiben **alle Längen unabhängig von der Microstep-Einstellung
richtig**: gespeichert werden mm/Grad, die Schrittzahl wird bei jedem Bedarf
neu berechnet (`configForwardSteps()` / `configTurnSteps()`). Microsteps
ändern dadurch nur die Feinheit, nicht die Strecke.

`Motor.h` hält die Stepper-Instanzen privat - direkte `stepper.run()`-Aufrufe
ausserhalb von `Motor.cpp` sind dadurch unmöglich.

### Kein Einrasten, und warum nicht

Es gab einmal `motorSettle()`: **zwei** Vollschritte vor **und exakt zwei
zurück** vor dem Programmstart, „damit die Räder auf derselben Microstep-Index
stehen". Die Funktion ist gelöscht.

**Warum sie nichts gebracht hat.** Zwei Vollschritte hin *und* zwei zurück sind
**winkelneutral**: am Ende steht jeder Rotor genau wieder auf der Microstep-Index,
auf der er vorher war — nur eben um den Schiefstand gegen das jeweils andere Rad
versetzt. Der gewünschte Effekt (beide gleich) tritt genau dann **nicht** ein,
wenn er gebraucht würde.

**Warum es grundsätzlich nicht geht.** Bei einem **direkt gekoppelten Stepper**
(Motor → Rad ohne Getriebe) hängt die *Fahrweg-Richtung* an der **Rad-/Achs-Montage**
am Chassis, nicht an der Rotorposition. Beide Räder rollen exakt gleich weit →
der Fahrweg ist eine exakt gerade Linie, sie liegt nur um `h` schräg zur
Roboterachse („Krebsgang"). Rotor-Schlupf verschiebt die **Speichen**, nicht die
**Radebene**. „Die Räder stehen anders" ist deshalb eine **Baustuffgröße**, keine
Firmwaresache — man könnte sie nur durch eine Beilenscheibe unter einem Rad
beseitigen, nicht durch Software.

**Was es gekostet hat:** 30…200 ms blockiert bei jedem Programmstart, und im
Timeout-Fall (`PROG:NO_SETTLE`) startete das Programm **gar nicht**.

`PROG:RUN` fährt deshalb direkt los.

---

## Befehlsreferenz

Befehle kommen per BLE (Web-UI) oder ueber den Serial Monitor (gleiche
Syntax, `CR`/`LF` beendet). Befehle sind case-insensitive, Whitespace wird
getrimmt. Jeder Handler liegt in `Edu_V2.cpp` in einer eigenen
`cmd*()`-Funktion - neue Befehle dort ergaenzen, dann in der
Dispatch-Liste am Ende von `handleCommand()` eintragen.

### Steuerung

| Befehl | Wirkung |
|---|---|
| `KEYS:<W><A><S><D>` | Tastenstatus, z. B. `KEYS:1100` |
| `STOP` | Notfall-Stop: Motoren aus, TMC aus, Spiel abbrechen |

### Einlernung (fährt sofort los)

| Befehl | Wirkung |
|---|---|
| `STEPS?` / `STEPS:<5..150>` | Vorwärtsstrecke je Farbe in **mm** (setzt + fährt Test) |
| `ANG?` / `ANG:<10..360>` | Drehung im Spiel in **Grad** (setzt + dreht Test) |
| `GEO?` / `GEO:RESET` | Geometrie melden / Einlernung auf 27,8 mm + 195 Grad zurücksetzen |
| `SPEED?` / `SPEED:<20..600>` | Fahrgeschwindigkeit in mm/s (wird zur Schrittrate umgerechnet) |
| `SAT?` / `SAT:<0..1>` | Mindest-Sättigung für warme Farbtöne (GELB/ROT). Wirkt **nur** auf die warmen Töne: eine blasse, warme Fläche (Parkett, Karton, warmes Licht) fällt sonst als GELB/ROT durch. Blaue und grüne Marker sind nie betroffen. `SAT:0` schaltet die Schranke ab. |

### Distanzsensor

| Befehl | Wirkung |
|---|---|
| `DIST?` | Einmalige Messung, `DIST:<mm>` oder `DIST:--` |
| `DISTAUTO:ON` / `DISTAUTO:OFF` | automatische `DIST:`-Meldungen |
| `DISTDIAG` | **Diagnose:** I2C-Scan, Modus, Range-Status, Fehlerzähler |
| `OBSDIST?` / `OBSDIST:<50..1200>` | Stopp-Schwelle (Hindernis blockiert) |
| `OBSREL?` / `OBSREL:<50..1200>` | Freigabe-Schwelle (Hysterese) |
| `DISTCAL?` / `DISTCAL:CLR` / `DISTCAL:<50..500>` | Offset-Kalibrierung (Datenblatt-Verfahren) |
| `I2C?` | Bus-Scan 0x08..0x77 (gesund: 0x29 **und** 0x30) |

### Farbsensor

| Befehl | Wirkung |
|---|---|
| `CAL:BLACK` / `CAL:WHITE` | Referenz auf aktuelle Rohwerte setzen |
| `CAL:RED` / `CAL:YELLOW` / `CAL:GREEN` / `CAL:BLUE` | Farbreferenz kalibrieren |
| `CAL:?` | alle Kalibrierungen anzeigen (`*` = gesetzt) |

### TMC2209-Treiber

| Befehl | Wirkung |
|---|---|
| `TMC?` | ausführlicher Status auf Serial |
| `TMC:CURRENT:<100..200>` | RMS-Strom in mA (200 = Nennstrom des NEMA8, harte Grenze) |
| `TMC:HOLD:<0..31>` | Haltestrom |
| `TMC:MS:<1..256>` | Microsteps (verifiziert per Readback) |
| `TMC:MODE:STEALTH` / `TMC:MODE:SPREAD` | leise / mehr Drehmoment |
| `TMC:EN:1` / `TMC:EN:0` | Treiber ein/aus |
| `TMC:INIT` | Konfiguration erneut senden (z. B. nachträglich, wenn VM kam) |
| `UARTTEST` / `UARTNET` | UART- bzw. PDN-Netz-Diagnose |
| `MOT:DIAG` | Treiber-Messung (siehe unten) |

#### Treiber-Messung: Schrittverlust und Schlupf **beobachten**

`MOT:DIAG` liest `GSTAT` (0x01), `SG_RESULT` (0x41), `VACTUAL` (0x22),
`TSTEP` (0x12), `MSCNT` (0x6A) und `DRV_STATUS` (0x6F) von **beiden**
Treibern und meldet sie im Log als `MOT:MEAS:` für die Web-UI.

**Es wird nichts nachgeregelt.** Kein Korrekturwert, kein Trim, kein
Ausgleich, kein Halt, keine Wartezeit – das Fahrverhalten ist vorher wie
nachher. Die Messung liefert Zahlen und Befunde, entschieden wird nichts.

| Wert | Wofür |
|---|---|
| `SG_RESULT` | Drehmomentreserve (höher = mehr Reserve). Nur oberhalb `TCOOLTHRS` verlässlich |
| `GSTAT` | Bit 1 `drv_err`, Bit 2 `uv_cp` (Ladepumpen-Unterspannung – der Treiber schaltet dann laut Datenblatt ab). Lesen löscht die Bits **nicht**; nur Schreiben tut das, und das unterbleibt |
| `VACTUAL` | von jedem Treiber selbst gemessene Drehzahl. Erkennt laufenden Schlupf, beweist aber keine einmalig verlorenen Schritte |
| `TSTEP` | von beiden Treibern gemessene STEP-Periode. Weit auseinander = die STEP-Leitungen werden unterschiedlich bedient; Anschlag `0xFFFFF` = Stillstand |
| `MSCNT` | **direkter Schrittverlust-Nachweis**: der *Vorrück* beider Seiten muss gleich sein. Zählerstand allein bedeutet nichts |
| `DRV_STATUS` | Modus (Stealth/Spread muss auf beiden Seiten gleich sein), `CS_ACTUAL` (ungleicher Laststrom), offene Last, Kurzschluss, Temperatur |

Automatisch läuft die Messung einmal nach jedem Anfahren, aber nur im
sicheren Fenster (20…1500 Schritte/s). Grund: ein Register-Read auf dem
gemeinsamen Single-Wire-Bus kostet ~2 ms – bei 4321 Schritten/s wäre die
STEP-Periode nur noch 231 µs, und die UART-ISR würde dem 50-µs-Puls-Task
genau die Takte wegnehmen, die er zum Erzeugen der Pulse braucht. Verlässt
der Roboter das Fenster, wird abgebrochen und das gemeldet. Positionsfahrten
werden nie unterbrochen.

Weil das Register-Layout in `Motor.h` (`TMC_REG_*`) an einer Stelle steht und
die Rohzugriffe über eine kleine Ableitung von `TMC2209Stepper` laufen, nutzt
die Messung den geprüften UART-Code der Bibliothek (CRC, Wiederholungen) – und
nicht eigene Datagramme, die wie die früher gelöschte `tmcUartDiag()` die
Treiberadressen verschieben könnten.

### Anfahren und Einschalten (nicht der Treiber, sondern die Laufzeit)

Gehört bewusst **nicht** ins `TMC:`-Raster: das sind Treiber-Register, hier
wird die Firmware-Laufzeit eingestellt. Die Werte liegen in der NVS und sind
in der Web-UI direkt unter dem Tempo einstellbar.

| Befehl | Wirkung |
|---|---|
| `RAMP?` / `RAMP:<0..2000>` | Anfahr- **und** Abbremszeit in **ms** für das eingestellte Tempo - gilt beim freien Fahren (an *und* ab) und für Fahrten nach Strecke. Antwort `RAMP=<ms>,<0\|1>`; das zweite Feld sagt, ob die Rampe wirklich aktiv ist. `0` = keine Rampe (sofort). Die Not-Aus-Pfade bleiben hart |
| `SPIN?` / `SPIN:<0..3000>` | Dauer der Einschalt-Drehung in ms, `0` = aus (dann wartet der Roboter auch nicht). Die Drehzahl bleibt, wie sie ist |
| `SPIN:<0..20000>,<0..3000>` | Drehzahl (Schritte/s) **und** Dauer zusammen, für den Serial Monitor. Beide nur gemeinsam gültig: ein halb gesetzter Wert wäre eine Fahrt, die nie stattfindet, aber den Boot verzögert |

### Spiele

| Befehl | Wirkung |
|---|---|
| `QUIZ:CHECK:<ROT,GELB,...>` | Farb-Quiz starten (max. 16 Farben, jede max. 4×) |
| `QUIZ:STOP` | Quiz abbrechen |
| `RACE:START` / `RACE:STEP` / `RACE:WIN` / `RACE:LOSE` | Rennen (kurze Form: Ziel 10, 360 ms je Punkt) |
| `RACE:START:<ziel>:<stufe>` | Rennen mit Zielzahl (1..99; das Web-UI schickt die Stufen-Zielzahl) |
| `RACE:STEP:<mm>` | Einen Punkt gutschreiben und **exakt mm** fahren (Rennstrecke) |
| `RACE:STOP` | Rennen abbrechen (Motoren frei, `currentGame` wird gelöscht) |
| `RACE:?` | Ziel/Stufe und Spielstatus melden |
| `RACE:SCAN` / `RACE:PAIR:<MAC>` / `RACE:UNPAIR` | Rennen-Pairing |
| `SLALOM:INVITE:<MAC>` / `SLALOM:INVITE_ACCEPT` / `SLALOM:INVITE_DECLINE` | Slalom-Pairing |
| `SLALOM:START_MATCH` | Spiel starten (nur Host) |
| `SLALOM:PROG:<enc>` | Programm senden + Bereit melden |
| `SLALOM:STOP` / `SLALOM:UNPAIR` | Slalom abbrechen / trennen |
| `LINE:START` / `LINE:STOP` | Linienfolger |
| `MEMORY:START` / `MEMORY:START:<n>` / `MEMORY:INPUT:<n>` / `MEMORY:STOP` | Memory (3..`MEMORY_MAX_LEN`) |
| `PROG:SET:<enc>` / `PROG:RUN` / `PROG:STOP` | Programmierung. `PROG:RUN` fährt direkt los (ein Einrasten gab es einmal, ist aber gelöscht — siehe „Kein Einrasten") |

---

## Wichtige Invarianten

Diese Punkte sind nicht offensichtlich und sollten beim Ändern beachtet
werden:

1. **`distSetup()` vor `colorSetup()`** in `setup()` – beide Sensoren
   starten auf I2C-Adresse 0x29.
2. **Kein `readRange()` im Continuous-Modus.** `Adafruit_VL53L0X::readRange()`
   läuft über `PerformSingleRangingMeasurement()` und schaltet den Sensor
   dabei auf `SINGLE_RANGING` um - das unterbricht den Dauerbetrieb von
   `startRangeContinuous()`. `DistanceSensor.cpp` nutzt deshalb
   `readRangeResult()` und wertet `readRangeStatus()` aus.
3. **Motor-Zugriffe nur über `Motor.h`.** Der Puls-Task läuft auf Core 0;
   alle Stepper-Instanzen sind dort `private`.
4. **Beide Motoren über `setMotorSpeeds()` bzw. `prepPositionMoves()` setzen,
   nicht über zwei einzelne Aufrufe.** Der Puls-Task nimmt den Motor-Mutex
   alle 50 µs; zwischen zwei Aufrufen kann er ihn bekommen, und dann kommt der
   zweite Befehl später an — die Räder fahren kurz einseitig an. Bei 20 ms
   Versatz sind das bei 150 mm/s und 16 Microsteps rund 3 mm Weg und gut
   2,5° Gierwinkel, auf einem Meter deutlich sichtbar. Bei Drehungen
   (`prepPositionMoves`) stimmen die Schrittzahlen zwar beide, aber das
   Drehmoment wirkt kurzzeitig nur einseitig. Die Einzelaufrufe
   `setMotorSpeed()` und `prepPositionMove()` bleiben für echte Einzelfälle.
5. **`currentGame` nicht im Motor-Task lesen.** Es ist single-Core-Logik
   (Core 1), deshalb ohne `volatile`.
6. **Spiel-Moves sind blockierend** und pollen mit `GAME_TICK_MS = 1`.
   Deshalb funktionieren `STOP` und andere BLE-Befehle auch *während* eines
   Spiels: die Warte-Schleifen rufen `bleProcessEvents()` auf und brechen
   bei `currentGame == GAME_NONE` ab.

7. **`handleCommand` ist ein Dispatcher.** Befehlsgruppen sind eigene
   `cmd*()`-Funktionen. Neue Befehle dort ergänzen und in der Dispatch-Liste
   eintragen, statt die Kette in `handleCommand()` zu verlängern.
7. **Nie `setMeasurementTimingBudgetMicroSeconds()` auf dem VL53L0X.** Die
   Funktion führt intern `perform_phase_calibration()` aus und **schreibt das
   Ergebnis ins NVM des Sensors**. Wird sie mit verdächtigen Ausgangsdaten
   aufgerufen, ist die Phasenkalibrierung dauerhaft kaputt und jede Messung
   meldet `Range-Status 4 (PhaseFail)`. Stattdessen den Library-Standard
   (33 ms) verwenden.
8. **Der VL53L0X braucht 0x29 für sich allein, solange er initialisiert
   wird** – siehe den Abschnitt unten.
9. **TMC-Register nur lesen, wenn überhaupt – und nie das Auto-Inkrement-Bit
   setzen.** Ein Read-Request an die *UART*-Adresse 0 ist beim TMC2209
   gleichzeitig ein Auto-Inkrement: die Treiberadressen wandern dauerhaft und
   die UART-Kommunikation ist tot, bis ein Neustart die OTP-Adressen
   zurücksetzt. Deshalb gibt es die gelöschte `tmcUartDiag()` nicht mehr, und
   die Registerzugriffe der Treiber-Messung laufen über die geprüften
   `read()`-Aufrufe einer kleinen Ableitung von `TMC2209Stepper` – CRC-Prüfung
   und Wiederholungen kommen damit aus der Bibliothek. `GSTAT` wird
   ebenfalls nur gelesen: die Fehlerbits werden durch **Schreiben** gelöscht,
   und das unterbleibt absichtlich.
10. **Kein Register-Read aus dem 50-µs-Puls-Task.** Ein Read auf dem
   gemeinsamen Bus kostet ~2 ms und würde 40 Ticks verschlucken. Die Messung
   liest deshalb genau ein Register pro Loop-Durchlauf.

---

## VL53L0X und der Adresskonflikt (wichtigste Falle im Projekt)

Beide Sensoren haben I2C-Default-Adresse **0x29**. Der Adafruit-Treiber
initialisiert den VL53L0X so:

```
begin(TARGET):  DataInit @ 0x29 (im Treiber hart kodiert)
                -> setAddress(TARGET)
                -> GetDeviceInfo @ TARGET
                -> StaticInit @ TARGET      <- liest die NVM-Daten
```

Der entscheidende NVM-Zugriff (`ReferenceSpadCount`, Good-SPAD-Map) läuft in
`StaticInit`, also auf `TARGET`. **Antwortet dort der TCS34725 ebenfalls, sind
die I2C-Reads verfälscht** (zwei Geräte auf einer Adresse). Gemessene Folge
auf dem Gerät:

| Schritt | Ergebnis |
|---|---|
| NVM-Ref-SPAD | `0x7FFFFF77` -> `count=127, type=1` (ungueltig) |
| `StaticInit` | `PerformRefSpadManagement` -> **-50** `REF_SPAD_INIT` |
| danach | jede Messung `Range-Status 4` (PhaseFail) |
| Messwerte | ca. 10 % gueltig, stark streuend |

Zwei Regeln folgen daraus und werden in `DistanceSensor.cpp` eingehalten:

1. **Während der Initialisierung gehoert 0x29 dem VL53L0X allein.**
   `distSetup()` prueft die Lage und waehlt den passenden Pfad:
   - 0x29 frei -> Initialisierung ueber 0x29 (exakt der Weg der frueher
     funktionierenden Fassung, siehe `backup_vor-revert/`)
   - 0x29 belegt -> Initialisierung mit Zieladresse 0x30; DataInit laeuft dann
     zwar im Konflikt, der entscheidende NVM-Zugriff aber sauber auf 0x30
2. **Vor jedem `begin()` eine frische Instanz erzeugen** (`loxResetInstance()`).
   `get_info_from_device()` liest die NVM-Daten nur *einmal* pro Objekt
   (`ReadDataFromDeviceDone`). Ein einmal eingefrorener Fehlwert bliebe sonst
   die ganze Sitzung bestehen. Achtung: **keine** Zuweisung
   `lox = Adafruit_VL53L0X();` - die Klasse hat ein Default-Member
   `pMyDevice = &MyDevice`, dessen Kopie auf die zerstoerte temporaere Instanz
   zeigen wuerde (LoadProhibited-Crash). Deshalb Placement-New.

### Aktueller Stand (auf dem Geraet verifiziert)

| Farbsensor | 0x29 | Ergebnis |
|---|---|---|
| **ausgesteckt** | frei | Init erfolgreich, Continuous-Modus, **46/46 gueltige Messungen**, 191-198 mm bei 20 cm Ziel, `Range-Status 0` |
| angesteckt | doppelt belegt | Konflikt - siehe unten |

### Wenn der Farbsensor wieder dazukommt

Dann ist 0x29 dauerhaft doppelt belegt. Ein **XSHUT-Pin hilft hier nicht** -
er haelt den VL53L0X im Reset, und genau dann kann er nicht initialisiert
werden; sobald er freigegeben wird, startet er wieder auf 0x29 und kollidiert
mit dem TCS34725. Der Farbsensor hat zudem eine feste Adresse, kann also nicht
umgezogen werden.

Ohne Eingriff in die Hardware bleiben nur zwei Wege:

1. **Farbsensor abgezogen lassen** - der Distanzsensor liefert dann
   zuverlaessig Werte (verifiziert: 46/46 gueltig, 191-198 mm bei 20 cm).
   Der Roboter funktioniert dann ohne Farberkennung.
2. **I2C-Multiplexer** (z. B. TCA9548A, wenige Euro) zwischen die Sensoren.
   Damit bekommt jeder Sensor einen eigenen Kanal und die vollstaendige
   Firmware funktioniert wie geplant. Alternative: die Sensoren an getrennte
   I2C-Busse haengen.

Die Firmware erkennt die Lage selbst: ist beim Boot nichts auf 0x29, wird der
VL53L0X ueber 0x29 initialisiert (sauberer Pfad); ist 0x29 belegt, gibt es
automatisch auf den Notpfad ueber 0x30 mit eingebautem Single-Shot-Fallback.
Diagnose: `DISTSCAN` zeigt den Bus-Zustand vom Boot-Zeitpunkt.

### Diagnose-Kommandos

| Befehl | Zweck |
|---|---|
| `DISTDIAG` | Modus, letzter Wert, Range-Status, Fehlerzaehler |
| `DISTSCAN` | Bus-Zustand vom Boot-Zeitpunkt (beweist Power-Cycle-Wirkung) |
| `DISTDEBUG` | jeden Init-Schritt mit Library-Debug nachvollziehen |
| `I2C?` | vollstaendiger Bus-Scan 0x08..0x77 |

---

## Konfiguration (NVS)

Gespeichert unter Namespace `edubot`. Beliebige Werte sind ueber die
Web-UI oder den Serial Monitor setzbar; `configValidate()` in `Config.cpp`
korrigiert unsinnige Werte beim Boot (Bereichsgrenzen, Hysterese-Order,
Schrittweiten) und speichert sie zurueck.

---

## Bekannte offene Punkte

- **VL53L0X und Farbsensor koennen nicht gleichzeitig an einem I2C-Bus
  betrieben werden.** Aktuell ist der Farbsensor dafuer abgezogen; der
  Distanzsensor liefert damit zuverlaessig Werte (46/46 gueltig). Fuer den
  Parallelbetrieb ist ein I2C-Multiplexer (z. B. TCA9548A) noetig - XSHUT
  loest das nicht.
- Keine automatisierten Tests. `configValidate()` und die
  `ProgGame`-Parserergebnisse waeren die naheliegenden Kandidaten.
