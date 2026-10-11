# Umsetzungs-Spec: Fundament (W0) + Welle 1

Verbindliche Arbeitsgrundlage fuer die Umsetzung von Fundament und Welle 1 der
neuen Spiel-Modi. Uebergeordneter Plan: `GAME_MODES.md`. Stand: 2026-10-10.

> **Status 2026-10-10: umgesetzt und gebaut.** Alle W0-Bausteine und die
> fuenf Welle-1-Modi stehen (Firmware + UI, Build gruen, 43,9 % Flash).
> Zusaetzlich umgesetzt: B4, C1, B5, A9, B6, B3, C2 — siehe Umsetzungsstand
> in `GAME_MODES.md` (inkl. der bewussten Abweichungen: A6-Totbandregler,
> B2-Balken-LED, A5-Schummler-Variante, A9-Straf-Stand). Die Testkarten in
> Abschnitt 4 gelten fuer alle Modi; Geraetetests stehen noch aus.
>
> **Zweiter Durchgang (UI/Symbolik + Verifikation):** Alle Views haben
> kindgerechte Symbolik (Ampel mit 3 Lampen, Winkelscheibe, Zahlenstrahl,
> Abstands-Balken, Budget-Punkte, Formen-Icons, Farb-Swatches,
> Scan-Punkte, Timer/Countdown, Ereignis-Log) und klare Schritt-Ablaeufe
> (B3/B7: „Programm bauen -> GO -> Pruefen" mit Sprung in den Editor).
> B1 sendet jetzt die vorzeichenbehaftete Abweichung (zu weit/zu kurz).
> Automatischer Playwright-Test: 90/90 Checks bestanden, keine JS-Fehler,
> Mobile 375 px ohne Overflow (Screenshots `qa-*.png` im Projektordner).
>
> **Dritter Durchgang (Duell + Button-Fixes):** A5/B1/B2/B6/B3 haben ein
> 1:1-Duell (ESP-NOW-Punkte-Austausch, Modul `Duel.cpp`, Ziel 10 Punkte,
> UI-Panel in jeder der fuenf Views). Kein Roboter-Kontakt, Einzelspieler
> unveraendert. Pfeiltasten funktionieren in allen Fahr-Views,
> Tasten-Highlight trifft das richtige Panel, Winkel-/STOPP-Knoepfe erst
> waehrend des Spiels aktiv. Verifikation: 35/35 Duell+Button-Checks,
> 32/33 Regression (1 Artefakt), Button-Audit ueber alle 12 Views.

Geltende Randbedingungen (Nutzerentscheidung): **keine Kollisions-Spiele**
(A1/A2 gestrichen), A8 nur ohne Roboter-Kontakt.

---

## 0. Umfang

| Welle | Inhalt |
|---|---|
| W0 | `HALT` (weicher Spiel-Stopp), `GameCommon` (ModeBase), `Probe` (Praezisions-Pruefroutine), zwei neue LED-Muster, UI-Grundgeruest, **A5 Einzelmodus als Referenz-Schnitt** |
| W1 | A6 Hand-Faenger, B7 Blinder Pilot, B1 Zahlentreffer, B2 Winkel-Fischer |

Nicht in diesem Paket: Pose-Broadcast (W3), Manual-Freigabe-Nutzung (W2),
A5-Duell (Stretch, Definition am Ende), ein neues ESP-NOW-Paketformat.

---

## 1. Querschnitts-Bausteine (W0)

### 1.1 `HALT` - der weiche Spiel-Stopp

**Warum:** `STOP` ist und bleibt der Not-Aus (`hardStop()`: Treiber aus, alle
Spiele aus, Alarm). Praezisions-Spiele brauchen einen Knopf, der nur die
Motoren anhaelt und das Spiel am Leben laesst.

**Verhalten:**
- `HALT` → `driveClearKeys()`, `stopMotors()`, `speedL = speedR = 0`,
  `currentGame` bleibt, kein `tmcDisable`, kein Alarm.
- Setzt das Halt-Latch (`gameHaltSet()`), Antwort `HALT:OK`.
- Der aktive Modus konsumiert das Latch in seinem Tick (`gameHaltRequested()`
  / `gameHaltClear()`), wertet aus und bleibt im Spiel.
- Ohne aktiven Modus ist `HALT` ein reiner Stopp, das Latch wird sofort
  geloescht.
- `HALT` wird NICHT in `cmdIsGameStop` aufgenommen (es beendet kein Spiel).
- Fuer Modi ohne Halt-Auswertung wirkt es als momentaner Stopp; die UI zeigt
  den HALT-Knopf nur in Modi, die ihn auswerten (A5, B1).

**Wichtig fuer die Messung:** `cmdHalt()` stoppt die Motoren im Moment des
Empfangs. Die BLE-Latenz (Groessenordnung 20-100 ms) addiert sich bei
60 mm/s zu 1-6 mm Messfehler - akzeptiert, die Toleranzen liegen bei 10/30 mm.

### 1.2 `GameCommon.cpp/.h` (ModeBase)

```cpp
// HALT-Latch
void gameHaltSet();            // aus cmdHalt()
bool gameHaltRequested();
void gameHaltClear();

// Manual-Freigabe (W2; jetzt nur der Mechanismus)
void gameSetManualDrive(bool on);
bool gameAllowsManual();

// Kanten-Waechter fuer die Dauer eines Modus aussetzen (Session, KEIN NVS)
void gameSuspendEdgeGuard(bool on);
bool gameEdgeGuardSuspended();

// Standard-Ende: Motoren aus, currentGame = GAME_NONE, Token senden
void gameModeEnd(const char* token);
```

- `driveLockedByGame()` in `Edu_V2.cpp` wird zu
  `currentGame != GAME_NONE && !gameAllowsManual()`.
- `edgeGuardTick()` in `ProgGame.cpp` beginnt mit:
  `if (gameEdgeGuardSuspended()) { s_edgeCount = 0; s_edgeBlocked = false; return; }`.
  `gameSuspendEdgeGuard(true)` ruft zusaetzlich `edgeGuardReset()` +
  `motorReleaseEmergency()`. Trade-off (bewusst): waehrend eines
  suspendierten Modus gibt es keinen Kantenschutz - nur Modi auf dem Boden
  (A9) nutzen das, A1/A2 sind gestrichen.
- `hardStop()` und `onBleDisconnected()` rufen die Stop-Funktionen der neuen
  Modi (Muster wie `memoryStop()`), damit der Not-Aus ueberall greift.

### 1.3 `Probe.cpp/.h` (Praezisions-Pruefroutine)

```cpp
void probeBegin();                 // Referenz auf Null setzen (Moment des Signals)
int32_t probeMM();                 // Strecke seit Begin in mm (vorwaerts positiv)
uint8_t probeScoreMM(int32_t mm);  // 2 = |mm| < 10, 1 = |mm| < 30, sonst 0
```

- Quelle: `motorStepPositions()`, Strecke = Mittelwert beider Radzaehler seit
  `probeBegin()`, umgerechnet mit `configStepsPerMM()`.
- Kein neuer Sensor, keine Blockade. Genutzt von A5 und B1.
- Die Toleranzen (10/30 mm) stehen NUR hier - alle Modi sind damit fair.

### 1.4 Neue LED-Muster (`LedStrip.cpp/.h`)

```cpp
void ledStripShowPulse(uint8_t colorId);          // langsames Atmen in einer Farbe
void ledStripShowDistanceHint(int16_t mm);        // Naehe-Baender (A6, spaeter A3)
void ledStripShowAngle(int16_t curDeg, int16_t targetDeg); // Zeiger + Ziel-Blinken (B2)
```

- `ledPatternUpdate()` bekommt Zweige (nach dem Memory-Block):
  `ampelActive` → Pulse (gruen/rot je Phase), `zahlActive` → Pulse (blau),
  `winkelActive` → Angle, `handActive` → DistanceHint.
- Regeln aus dem README gelten: waehrend Farbmessungen bleibt der Ring dunkel;
  Alarm und Ergebnisblitze uebersteuern weiterhin intern.

### 1.5 UI-Grundgeruest (`Edu_V2.html`)

- 5 neue Views im bestehenden Stil (`<div id="view-X" class="view game-view">`
  mit `.game-header`, `.back-btn`, `.arena`), 5 neue Tiles in der Kategorie
  Spiele.
- 5 zentrale Handler-Zweige in `handleStatusMessage()`:
  `AMPEL:` , `ZAHL:` , `WINKEL:` , `HAND:` , `PILOT:`.
- Konventionen: Status-Tokens max. 64 Zeichen; Schuelerseite zeigt keine
  Diagnose; keine Emoji (`tools/check_no_emoji.py` muss sauber bleiben).
- Jeder Modus hat Start/Stop; A5 und B1 zusaetzlich den grossen
  **`STOPP!`-Knopf = `HALT`** (weich). Der rote `STOP` (Not-Aus) bleibt
  zusaetzlich sichtbar.
- `resetAllLocalGameState()` wird um die neuen Modus-Zustaende erweitert
  (Verbindungsverlust raeumt auf).

### 1.6 Referenz-Schnitt A5

A5 Einzel wird in W0 komplett gebaut (Firmware + LED + UI). Erst danach
entstehen B1, B2, A6, B7 nach demselben Muster.

---

## 2. Welle-1-Modi

### 2.1 A5 - Ampel (Gruenes Licht, rotes Licht)

**Kommandos (UI → Roboter):**

| Befehl | Wirkung |
|---|---|
| `AMPEL:START` | Runde starten (Roboter rollt los) |
| `AMPEL:NEXT` | Naechste Runde (nach Ergebnis) |
| `AMPEL:CHEATPRESS` | "Schummler!" gedrueckt (nur in Schummler-Runde) |
| `AMPEL:STOP` | Modus beenden |
| `HALT` | Bremsen! Runde wird ausgewertet |

**Meldungen (Roboter → UI):** `AMPEL:ARMED`, `AMPEL:RED`,
`AMPEL:RESULT:<mm>,<punkte>`, `AMPEL:MISSED`, `AMPEL:CHEAT`,
`AMPEL:CHEATWIN`, `AMPEL:CHEATLATE`.

**Zustandsmaschine (Tick, nicht blockierend):**
1. `START` → GAME_AMPEL, `AMPEL:ARMED`, rollt mit `AMPEL_ROLL_MM_S = 60`.
2. Nach Zufall 2-6 s → `AMPEL:RED` + `probeBegin()`. Rollt weiter, bis das
   Halt-Latch kommt.
3. Halt-Latch → Motoren stehen bereits; `mm = probeMM()` →
   `AMPEL:RESULT:<mm>,<2|1|0>` (2: <50 mm Stern, 1: <150 mm gut, 0: sonst
   "erwischt"). Latch loeschen, warten auf `NEXT`.
4. Sicherheitsnetz: `probeMM() > 1500` oder 20 s ohne Halt → `AMPEL:MISSED`.
5. Schummler (1 von 6 Runden): Ampel bleibt GRUEN, der Roboter stoppt nach
   2-6 s von selbst und meldet `AMPEL:CHEAT`. Kind muss binnen 3 s
   `AMPEL:CHEATPRESS` senden → `AMPEL:CHEATWIN`, sonst `AMPEL:CHEATLATE`.
   (Bewusste Vereinfachung gegenueber "ignoriert das Rot": eindeutig spielbar
   fuer Kinder, gleicher Lerneffekt "aufmerksam sein".)
6. 2 s nach jedem Ergebnis automatisch bereit fuer die naechste Runde
   (UI zeigt Weiter-Knopf).

**LED:** ARMED = gruenes Atmen (`Pulse`), RED = rotes Atmen, Ergebnis =
`ledStripShowResultFlash(ok)` (gruen ab Punkte 1, rot bei 0).

**A5-Duell (Stretch):** 1:1-Pairing wie Rennen (`RACE:PAIR`), Host wuerfelt
Red-Zeit und sendet sie als GO-Signal; wer zuerst steht (HALT), gewinnt.
Definition erst, wenn Einzel am Geraet steht.

### 2.2 B1 - Zahlentreffer

**Kommandos:**

| Befehl | Wirkung |
|---|---|
| `ZAHL:START:<cm>` | Runde mit Zieldistanz (10..100, Vielfache von 10) |
| `ZAHL:STOP` | Modus beenden |
| `HALT` | Auf der Markierung anhalten (Auswertung) |

**Meldungen:** `ZAHL:GO:<cm>` (bestaetigt), `ZAHL:RESULT:<mmAbw>,<punkte>`
(2: <10 mm, 1: <30 mm, 0: sonst), `ZAHL:OVERRUN`.

**Ablauf:** Nach `START` faehrt der Roboter geradeaus mit
`ZAHL_SPEED_MM_S = 100` (eigener Tick, kein blockierender Move).
Bei Halt-Latch: `gefahren = probeMM()`, `abw = |gefahren - ziel|`,
Ergebnis senden, Latch loeschen, Motoren aus (bereits durch HALT).
Sicherheitsnetz: `gefahren > ziel + 500` → `ZAHL:OVERRUN`, Stop.

**LED:** blaues Atmen waehrend der Fahrt.

### 2.3 B2 - Winkel-Fischer

**Kommandos:**

| Befehl | Wirkung |
|---|---|
| `WINKEL:START:<grad>,<tol>` | Zielwinkel 0..355, Toleranzstufe 45/15/5 |
| `WINKEL:LEFT` / `WINKEL:RIGHT` | Dreht einen Schritt (15 Grad) |
| `WINKEL:CHECK` | Auswertung anfordern |
| `WINKEL:STOP` | Modus beenden |

**Meldungen:** `WINKEL:GO:<grad>`, `WINKEL:POS:<summe>`,
`WINKEL:RESULT:<abw>,<punkte>` (2: <= tol, 1: <= 3*tol, 0: sonst).

**Ablauf:** Der Modus fuehrt die Kommandosumme (in Grad) mit. Eine Drehung =
`gameTurnSteps(configTurnSteps()/6, ...)` (ein "90-Grad-Schritt" ist
`configTurnSteps()`; 15 Grad = /6). Nach `CHECK` Vergleich mit Ziel.
Kein Sensor, kein Probe - die gemessene Drehung ist die kalibrierte
Kommandostrecke (bewusste, ehrliche Vereinfachung).

**LED:** `ledStripShowAngle`: ein LED zeigt die aktuelle Richtung (Ring als
360-Grad-Zeiger), das Ziel blinkt.

### 2.4 A6 - Hand-Faenger

**Kommandos:** `HAND:START`, `HAND:STOP`.

**Meldungen:** `HAND:STATUS:<mm>` (max. 2 Hz, nur bei Aenderung > 20 mm),
`HAND:LOST`, `HAND:OK`.

**Regelkreis (Tick, 10-20 Hz, nicht blockierend):**
- Quelle: letzter gueltiger VL53L0X-Wert (Continuous-Modus; KEIN neuer
  Messbefehl im Tick).
- Ziel `HAND_TARGET_MM = 250`, Totband `HAND_DEADBAND = 40`.
- Fehler > Totband → vor mit `min(150, max(40, |fehler| * 0.8))` mm/s;
  Fehler < -Totband → zurueck mit derselben Formel.
- Kein gueltiger Wert seit 1 s → `stopMotors()`, einmal `HAND:LOST`; kommt
  ein Wert zurueck → weiter (Status `HAND:OK`).
- Beim Stop/Disconnect: Motoren aus, Modus raeumt auf.

**LED:** `ledStripShowDistanceHint` - rot < 150 mm, gelb < 220, gruen < 330,
sonst blaues Atmen.

**Stretch (spaeter):** Blaue "satt"-Zone (3 s still + Regenbogen) und rote
"Falle" - NICHT in W1 (erst nach Geraetetest der Grundfunktion).

### 2.5 B7 - Blinder Pilot

**Kein neuer GameMode.** B7 ist ein Aufsatz auf das bestehende PROG:
`PILOT:START` bewaffnet den Budget-Zaehler (5 Messungen), `DIST?` zaehlt
herunter, `PILOT:CHECK` liest die Farbe unter dem Roboter und meldet.
`GAME_PROG` und der Interpreter bleiben unveraendert.

**Kommandos:**

| Befehl | Wirkung |
|---|---|
| `PILOT:START` | Budget = 5, Zaehlung scharf |
| `PILOT:CHECK` | Farbe unter dem Roboter + Restbudget melden |
| `PILOT:STOP` | Zaehlung aus |

**Meldungen:** `PILOT:ARMED`, `PILOT:LEFT:<n>` (nach jeder Messung),
`PILOT:BUDGET:0` (einmalige Warnung), `PILOT:RESULT:<farbe>,<rest>`.

- Budget 0 ist eine Warnung, kein Abbruch (das Team spielt weiter, verliert
  nur Punkte).
- Hindernis-Verhalten des Programms ("bleibt stehen") kommt aus dem
  bestehenden PROG (am Geraet bestaetigen, offener Punkt).
- UI rechnet: Ergebnis = Zielfarbe getroffen UND Restbudget.

---

## 3. Dateien

**Neu:** `src/GameCommon.cpp/.h`, `src/Probe.cpp/.h`, `src/AmpelGame.cpp/.h`,
`src/ZahlGame.cpp/.h`, `src/WinkelGame.cpp/.h`, `src/HandGame.cpp/.h`,
`src/PilotGame.cpp/.h`.

**Geaendert:** `src/GameEngine.h` (Enum + `GAME_AMPEL, GAME_ZAHL, GAME_WINKEL,
GAME_HAND`), `src/Edu_V2.cpp` (HALT, Dispatch, hardStop/Disconnect,
ledPatternUpdate, loop-Ticks), `src/LedStrip.cpp/.h` (3 Muster),
`src/ProgGame.cpp` (Kanten-Suspend-Zeile), `Edu_V2.html` (Tiles, Views,
Handler, HALT-Knoepfe), `GAME_MODES.md` (Status).

---

## 4. Definition of Done + Geraete-Testkarten

Pflicht fuer alle: `pio run` fehlerfrei; kein `UNKNOWN:` auf der
Entwicklerseite bei normalem Spielverlauf; Flash/RAM-Stand notiert;
STOP (Not-Aus) bricht jeden Modus sauber ab; Disconnect raeumt auf.

| Modus | Testkarte (am Geraet) |
|---|---|
| HALT | Im Freifahren `HALT` senden → Motoren sofort aus, kein Alarm, `HALT:OK`; danach WASD wieder fahrbar |
| A5 | Start: gruenes Atmen + rollt; RED kommt nach 2-6 s; HALT → RESULT mit plausibler mm-Zahl (Handstopp gegenpruefen); 3 Runden mit unterschiedlichen Werten; 1,5-m-Netz greift; Schummler-Runde erscheint und CHEATPRESS wertet aus; STOP mitten in der Runde killt sauber |
| B1 | Ziel 50 cm: HALT bei Markierung → Abweichung < 3 cm; absichtlich zu spaet → OVERRUN/hohe Abweichung; 3 Ziele |
| B2 | Ziel 90 Grad: 6x RIGHT → CHECK ≈ 0; 1x LEFT vorher → Abweichung 15; LED-Zeiger plausibel |
| A6 | Hand auf 25 cm: Roboter haelt Abstand; Hand naeher → weicht; Hand weg → folgt; Hand zu weit/kein Wert 1 s → LOST + Stop; HAND:STOP |
| B7 | Budget 5: 5x `DIST?` → LEFT 0 + BUDGET:0-Warnung; CHECK auf farbigem Feld meldet Farbe |

**Offene Geraete-Checks:** Handreflexion fuer den ToF (Zielbereich 150-400 mm);
ob PROG bei Hindernis stehen bleibt (B7); BLE-Latenz fuer A5/B1 einmal grob
messen (HALT-Knopf, Sichtpruefung).

---

## 5. Entscheidungen (Kurzprotokoll)

1. `HALT` als neuer Befehl statt STOP umzudeuten - STOP bleibt Not-Aus.
2. Probe misst ausschliesslich mm (B2 braucht sie nicht: Kommandosumme).
3. Schummler als "Roboter stoppt selbst und luegt" - eindeutig fuer Kinder.
4. B7 ohne neuen GameMode (Seitenschiene auf PROG) - kein Interpreter-Umbau.
5. Kanten-Suspend als Session-Flag, nie NVS.
6. Kein neues ESP-NOW-Format; A5-Duell erst als Stretch nach Geraetetest.
