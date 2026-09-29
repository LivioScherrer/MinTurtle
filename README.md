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
2. **Der S-Block** (`LED`-Baustein) nutzt dieselbe Anzeige: Slots 0-3 zeigen
   die Zielfarbe, Slot 4/5 die Distanz-Bedeutung NAH (dunkelrot) / FREI
   (dunkelgruen), Slot 6 schaltet aus.

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

Positionsfahrten (Spiele, Programm-Blöcke, Einlern-Tests) haben eine sanfte
**Rampe**: von 0 auf `FIXED_DRIVE_SPEED` in `MOTOR_RAMP_MS` (150 ms,
`src/Motor.h`), auf- und abwärts. Sie kommt von AccelStepper
(`run()`/`setAcceleration()`), die Schrittzahl bleibt exakt. Das freie Fahren
(Tasten) und der Linienfolger bleiben bewusst **ohne Rampe**: dort zählt
sofortige Reaktion - die Suchdrehungen des Linienfolgers schalten alle 140 ms
um und würden von einer Rampe verschluckt.

Deshalb bleiben **alle Längen unabhängig von der Microstep-Einstellung
richtig**: gespeichert werden mm/Grad, die Schrittzahl wird bei jedem Bedarf
neu berechnet (`configForwardSteps()` / `configTurnSteps()`). Microsteps
ändern dadurch nur die Feinheit, nicht die Strecke.

`Motor.h` hält die Stepper-Instanzen privat - direkte `stepper.run()`-Aufrufe
ausserhalb von `Motor.cpp` sind dadurch unmöglich.

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
| `TMC:CURRENT:<100..800>` | RMS-Strom in mA |
| `TMC:HOLD:<0..31>` | Haltestrom |
| `TMC:MS:<1..256>` | Microsteps (verifiziert per Readback) |
| `TMC:MODE:STEALTH` / `TMC:MODE:SPREAD` | leise / mehr Drehmoment |
| `TMC:EN:1` / `TMC:EN:0` | Treiber ein/aus |
| `TMC:INIT` | Konfiguration erneut senden (z. B. nachträglich, wenn VM kam) |
| `UARTTEST` / `UARTNET` | UART- bzw. PDN-Netz-Diagnose |

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
| `PROG:SET:<enc>` / `PROG:RUN` / `PROG:STOP` | Programmierung |

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
4. **`currentGame` nicht im Motor-Task lesen.** Es ist single-Core-Logik
   (Core 1), deshalb ohne `volatile`.
5. **Spiel-Moves sind blockierend** und pollen mit `GAME_TICK_MS = 1`.
   Deshalb funktionieren `STOP` und andere BLE-Befehle auch *während* eines
   Spiels: die Warte-Schleifen rufen `bleProcessEvents()` auf und brechen
   bei `currentGame == GAME_NONE` ab.

6. **`handleCommand` ist ein Dispatcher.** Befehlsgruppen sind eigene
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
