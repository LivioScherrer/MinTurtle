# Neue Spiel-Modi: Planung EduBot V2

Planung fuer die neuen Spiel-Modi des EduBot (10 Modi; A1/A2 sowie B2, A6, B7, B4, C1, B5, A9, B6, B3 und C2 sind gestrichen).
Basis sind die in `README.md` und `HARDWARE_FAKTEN.md` beschriebene Firmware
und Hardware. Dieser Plan beschreibt Konzept, Mechanik, Lernwert und Aufwand
je Modus; die detaillierte Umsetzung von Fundament + Welle 1 steht in
`GAME_MODES_W0_W1.md`.

> **Nutzerentscheidung 2026-10-10: keine Kollisions-Spiele.** A1 (Koenig des
> Berges) und A2 (Sumo) sind gestrichen — bei beiden sollen sich die Roboter
> gegenseitig schubsen. A4 bleibt: „Fangen" zaehlt ueber Abstand < 15 cm,
> ohne Kontakt. A8 (Fussball) darf nur ohne Roboter-zu-Roboter-Kontakt
> gebaut werden, sonst entfaellt es.

Stand: 2026-10-10. Der Farbsensor (TCS34725) ist in Betrieb (am Roboter
bestaetigt) — Farb-Spiele sind moeglich, ein I2C-Multiplexer ist nicht mehr
notwendig (der Patch in `patch_vl53l0x.py` loest den Adresskonflikt mit dem
VL53L0X).

---

## 0. Umsetzungsstand (2026-10-10)

**Umgesetzt und gebaut (5 Modi):** A5 Ampel (Einzel + Schummler-Runde),
B1 Zahlentreffer, B8 Geheim-Strecke, B9 Einparken, B10 Zeitfahren.

Fundament (W0) steht: `HALT` (weicher Spiel-Stopp), PrecisionProbe,
GameCommon (Manual-Freigabe, Kanten-Suspend als Session-Flag,
Tempo-Override RAM-only), UI-Grundgeruest (Tiles, Views, Status-Handler,
Fahr-Tasten in den Spiel-Views). Firmware-Stand: 43,9 % Flash, 20,9 % RAM.

**UI/Symbolik + Verifikation (2026-10-10, zweiter Durchgang):** Beide Modi
haben kindgerechte Symbolik statt Textwueste — 3-Lampen-Ampel mit Leucht-Glow
(A5), Zahlenstrahl mit Ziel- und Stopp-Marke (B1, Firmware sendet dafuer die
VORZEICHENBEHAFTETE Abweichung: zu weit/zu kurz). Automatisierter
Browser-Test (Playwright, 90/90 Checks): alle Kommandos, UI-Reaktionen,
Timer und Mobile 375 px verifiziert, keine JS-Fehler. Screenshots:
`qa-a5-ampel-rot.png`, `qa-dashboard-tiles.png`.

**Multiplayer-Duell + Button-Fixes (2026-10-10, dritter Durchgang):**
Die Modi **A5, B1, B8, B9 und B10** haben jetzt ein eingebautes 1:1-Duell
(ESP-NOW, Pairing wie beim Rennen — `RACE:SCAN` / `RACE:PAIR`): beide
Roboter spielen ihren Modus, die Punkte werden per Funk verglichen
(`SLALOM_CMD_DUEL`, Modul `Duel.cpp`). Wer zuerst 10 Punkte hat, gewinnt.
**Kein Kontakt zwischen den Robotern** — nur der Punktestand
wird geteilt. Ohne aktives Duell laeuft alles unveraendert als
Einzelspieler. UI: Duell-Panel in jeder der fünf Views (einklappbar,
versteckt sich waehrend laufender Runden von selbst; Punktestand
„Du x : y Gegner", Sieg-/Niederlage-Banner). Button-Fixes: Pfeiltasten-Steuerung funktioniert
jetzt in ALLEN Fahr-Views, das Tasten-Highlight trifft das richtige Panel,
die STOPP!-Knoepfe sind erst waehrend des Spiels aktiv (Knoepfe ohne Wirkung
verwirrten). Playwright-Verifikation:
35/35 Duell- und Button-Checks, 32/33 Regressions-Checks (1 Testartefakt,
kein Bug), vollstaendiger Button-Audit ueber beide Views, keine
JS-Fehler. Screenshot: `qa-duell-panel.png`.

**Stufen + B8 (2026-10-11, vierter Durchgang):** A5 und B1 haben
Schwierigkeits-Stufen nach dem Blockly-Prinzip (gleiche Regeln, wachsender
Anspruch): A5 verengt je 3 Erfolge das Rot-Fenster (2-6 s bis 2 s fix,
max. Stufe 5), B1 rollt je 3 Erfolge 20 mm/s schneller (100-180 mm/s,
max. Stufe 5). Wertung und Ziel bleiben immer gleich (fair). Neuer Modus
**B8 Geheim-Strecke** (unten): Schaetzen + Kalibrieren als dritte
Faehigkeit neben Reaktion (A5) und Bremsen (B1).

Abweichungen zur Spec (`GAME_MODES_W0_W1.md`), alle bewusst:

- **A5-Schummler:** Der Roboter stoppt von selbst und die UI fragt
  "Schummler!" — eindeutig spielbare Variante.
- **Duell vorgezogen:** Das 1:1-Punkte-Duell (A5/B1, spaeter B8) kam schon in
  Welle 1 als Stretch, weil es nur das vorhandene Race-Pairing nutzt —
  Welle 3 braucht dafuer nichts mehr zu tun.

**Noch offen:** A3 Heiss-Kalt und A4 Fangen (Pose-Welle W3), A7 Staffel,
A8 Fussball, C3 Spiegel-Tanz. Ein synchrones Runden-Rennen (gemeinsames
Rot-Signal beim A5-Duell) ist noch nicht dabei — das jetzige Duell vergleicht
die Punkte unabhaengig, ohne Funk-Timing.

---

## 1. Die Hardware als Spielfeld

Alles, was ein neuer Modus nutzen darf, ist hier aufgefuehrt:

| Bauteil | Faehigkeit im Spiel |
|---|---|
| 2 x NEMA8 (TMC2209) | Differentialantrieb, exakt mm/Grad, 0,6 mm pro Vollschritt, Drehen auf der Stelle, 20..600 mm/s einstellbar |
| Farbsensor TCS34725 | ROT, GELB, GRUEN, BLAU erkennen, Schwarz (Klebeband), Gesamthelligkeit (Kante, Licht) |
| Distanzsensor VL53L0X | 50..1200 mm, Hindernisse vor dem Roboter |
| Tischkanten-Waechter | Schwarz unter dem Roboter (2 Messungen) -> sofortiger Halt |
| LED-Ring (7 Pixel) | Fortschritt, Richtungslauflicht, Ampelfarben, Regenbogen als Jubel |
| ESP-NOW | Max. 4 Roboter im Netz, 94-Byte-Pakete mit 83 Byte Nutzlast |
| Web-UI (BLE) | Steuerknueppel, Bloecke, Aufgabenstellung, Ergebnisanzeige |
| Block-Programmierung | „Fahre bis Farbe/Abstand", „warte bis", Schleifen |

### Zwei Dinge, die der Roboter bewusst NICHT kann

- **Kein Lautsprecher.** Jedes Spiel muss ohne Ton funktionieren: Feedback
  laeuft ueber LED-Ring, Motor (kurze Rueckstoesse als „Kopfschuetteln") und
  Web-UI.
- **Kein RSSI.** Die Plattform (`espressif32@^6.7.0`, ESP-IDF 4.4) liefert im
  ESP-NOW-Empfangspfad keine Signalstaerke mit. Ein Upgrade auf Arduino-Core
  3.x wuerde das aendern, ist aber ein Risiko fuer den gesamten TMC-/BLE-Code.
  **Ersatz: Pose-Broadcast** (Abschnitt 3) — die Roboter teilen sich gegenseitig
  ihre geschaetzte Position mit, daraus werden Abstand und Richtung berechnet.

---

## 2. Zwei Querschnitts-Mechanismen

Fast alle neuen Modi brauchen zwei Dinge, die einmal gebaut und dann geteilt
werden. Beide liegen in eigenen Modulen (nach dem Muster von `MemoryGame.cpp`
oder `LineFollower.cpp`), damit `Edu_V2.cpp` nicht weiter waechst.

### 2.1 Pose-Broadcast (Multiplayer-Infrastruktur 2.0)

Heute kann ESP-NOW nur paarweise Rennstand melden (`RACE:STEP`, Slalom).
Fuer Spiele wie „Heiss-Kalt" oder „Fangen" brauchen die Roboter eine
Vorstellung voneinander. Loesung ohne RSSI:

- Jeder Roboter fuehrt eine eigene Odometrie: `x`, `y` (mm) und `hdg` (Grad),
  aus den bereits vorhandenen Schrittzählern der AccelStepper-Instanzen
  (dieselben Werte, die `[DRV]` und `DRV?` heute melden).
- Beim Modus-Start wird die Odometrie auf (0, 0, 0) gesetzt — der Startpunkt
  ist der Ursprung.
- Laeuft ein Multiplayer-Modus, sendet jeder Roboter seine Pose ca. 4x pro
  Sekunde als Broadcast. Neuer Kommando-Wert in `RacePacket.slalomCmd`
  (z. B. `CMD_POSE = 20`), Nutzlast als Kurztext in `slalomData`, z. B.
  `"x;y;hdg"` = `"312;-87;90"`. 83 Byte reichen problemlos.
- Der Empfaenger berechnet den Abstand: `d = sqrt(dx*dx + dy*dy)` und die
  Richtung zum Sender. Fertig ist „heiss/kalt" und „wo ist der Gegner".
- **Drift ist bekannt und akzeptiert.** Odometrie driftet ueber Minuten um
  einige Zentimeter bis Dezimeter. Fuer „heiss/kalt"-Bänder (3 m / 1,5 m /
  0,8 m) reicht das. Kein Modus darf auf exakte Pose-Positionen bauen — nur
  auf Abstand und Richtung.
- Alternative mit exakten Positionen (UWB, Kamera) waere Hardware-Arbeit und
  ist nicht geplant.

### 2.2 Präzisions-Prüfroutine (PrecisionProbe)

Roboter-Stopps sind auf den Millimeter exakt (0,6 mm/Vollschritt) — dieses
Versprechen wird heute nirgends gegen den Menschen gemessen. Ein kleines
Modul macht daraus ein Spielelement:

- `probeBegin()` nullt die Schrittzaehler beider Raehler an einer definierten
  Linie und merkt sich den Soll-Wert.
- `probeResultMM()` / `probeResultDeg()` liefern Ist-Position, Abweichung in
  mm/Grad und eine Punktzahl ueber Toleranzschwellen (z. B. < 1 cm = perfekt,
  < 3 cm = gut, sonst daneben).
- Ergebnis geht per `sendStatus` an die Web-UI (gleicher Weg wie `RACE:ME:`
  oder `QUIZ:CHECKING`).
- Genutzt von den Modi A5 und B1 — dort ist die gemessene Abweichung der
  Spielstand. Die Toleranz-Schwellen stehen im Modul, damit beide Modi
  dieselbe „Fairness" haben.

### 2.3 Integrationsmuster (fuer alle Modi identisch)

Jeder Modus haelt sich an das Muster, das `MemoryGame` und `LineFollower`
vorgeben — dadurch bleibt `Edu_V2.cpp` ueberschaubar:

1. `src/<Name>Game.cpp/.h` mit `<name>Start()`, `<name>Stop()`, `<name>Tick()`.
2. Neuer Wert im Enum `GameMode` (`GameEngine.h`), damit `currentGame`
   gesetzt ist: Freifahren gesperrt, STOP bricht sauber ab.
3. Bewegungen nur ueber die blockierenden Primitiven aus `GameEngine.h`
   (`gameDriveForward`, `gameTurnLeftSteps`, ...) — der Puls-Task auf Core 0
   bleibt unberuehrt.
4. Befehle `NAME:START/STOP/...` als eigene `cmd*()`-Funktion und Eintrag in
   der Dispatch-Liste in `handleCommand()`.
5. LED-Muster in `LedStrip.cpp` + Prioritaetenkette in `ledPatternUpdate()`.
6. UI: ein Tile im Dashboard und eine View in `Edu_V2.html`.

---

## 3. Die Spiel-Modi

### A-Reihe: Action und Spass

#### A1 - Koenig des Berges (King of the Hill) - GESTRICHEN

Gestrichen am 2026-10-10 (Nutzerentscheidung): Das Spiel funktioniert nur,
wenn die Roboter sich gegenseitig aus dem Ring schubsen — Kollisionen am
Fahrzeug (Motoren, Raeder, Chassis) sind nicht gewuenscht.

#### A2 - Sumo-Ringen - GESTRICHEN

Gestrichen am 2026-10-10 (Nutzerentscheidung): reines Roboter-gegen-Roboter-
Stossen, siehe A1.

#### A3 - Heiss-Kalt-Suche / Robo-Verstecken

- **Spieler:** 1 Geist + 1-3 Sucher
- **Ablauf:** Der Geist-Roboter wird 20 Sekunden vom Kind gesteuert, versteckt
  sich irgendwo im Raum, stellt den LED-Ring ab und sendet nur noch seine
  Pose (Abschnitt 2.1). Die Sucher fahren suchend umher und bekommen die
  Naehe ueber den LED-Ring angezeigt:

  | Abstand (Pose) | LED-Anzeige |
  |---|---|
  | ueber 3 m | langsames blaues Atmen |
  | 1,5-3 m | gruen |
  | 0,8-1,5 m | gelb |
  | unter 0,8 m | rotes Blinken + Jubel |

- **Fund:** Sucher-Pose innerhalb 80 cm vom Geist -> „GEFUNDEN"-Blitz, Geist
  zeigt Regenbogen. Dann Rollentausch (Geist wird Sucher).
- **Lernen:** Funksignale und Naehrung ohne direkten Sensorkontakt; das
  Konzept „unsichtbare Information sichtbar machen" (wie GPS, WLAN).
- **Spassfaktor:** hoch — „Roboter-Verstecken" ist sofort verstaendlich.
- **Aufwand:** mittel (Pose-Infrastruktur + LED-Baender).

#### A4 - Verfolgungsjagd (Fangen)

- **Spieler:** 2 Roboter (1 Jaeger, 1 Fluechtling; ein Kind lenkt)
- **Ablauf:** Der Jaeger-Roboter verfolgt automatisch den Fluechtling anhand
  der Pose-Distanz: weit weg = langsames Suchen, nah = Spurten. Das Kind
  faehrt den Fluechtling per App. Faellt der Abstand unter 15 cm, ist der
  Fluechtling gefangen (reine Abstandsmeldung der Pose, KEIN Kontakt), der
  Jaeger bekommt einen Punkt, dann Rollentausch.
- **Taktik-Element „Einfrieren":** Ein UI-Knopf haelt den Fluechtling 5
  Sekunden lang still. In dieser Zeit wird keine Pose gesendet — der Jaeger
  ist blind gegen ihn (24 Sekunden Abklingzeit, damit es nicht gespammt
  wird).
- **Arena:** Der Kanten-Waechter haelt beide Roboter im Spielfeld.
- **Lernen:** Distanz/Eile-Abwaegung, Regel-Taktik.
- **Spassfaktor:** hoch, besonders zu zweit.
- **Aufwand:** mittel (Jaeger-KI: Geschwindigkeitsstaffelung je Abstand).

#### A5 - Grünes Licht, rotes Licht / Reaktions-Duell

- **Spieler:** 1 Roboter, oder 2 Roboter im Duell
- **Ablauf Einzel:** Der Roboter rollt langsam los (ca. 60 mm/s). Zu
  zufaelligen Zeiten schaltet die UI-Lichtampel auf Rot. Das Kind muss
  SOFORT STOP druecken. Der Roboter misst per Odometrie, wie weit er nach Rot
  noch gerollt ist:
  - unter 5 cm: Stern
  - unter 15 cm: gut
  - darueber: „erwischt!"
- **Duell-Modus:** Beide Roboter bekommen per ESP-NOW dasselbe GO-Signal,
  beide Kinder druecken gleichzeitig, wer zuerst steht, gewinnt die Runde.
- **Erweiterung „Schummler":** Auf Wunsch ignoriert der Roboter selbst
  manchmal das Rot (Zufall, 1 von 6 Runden) und rollt weiter. Wer es zuerst
  bemerkt und den „Schummler!"-Knopf drueckt, bekommt den Punkt.
- **Stufen:** Je 3 erfolgreiche Runden steigt die Stufe (max. 5, `AMPEL:STAGE`);
  das Rot-Fenster schrumpft pro Stufe um 1 s (2-6 s bis 2 s fix). Die Wertung
  (< 5 cm Stern, < 15 cm gut) bleibt immer gleich.
- **Lernen:** Impulskontrolle, Reaktionszeit — paedagogisch wertvoll.
- **Spassfaktor:** sehr hoch, Party-tauglich, einfache Regel.
- **Aufwand:** klein (nutzt die Praezisions-Pruefroutine aus 2.2).

#### A7 - Staffel-Rennen

- **Spieler:** 4 Roboter = 2 Teams a 2
- **Aufbau:** Zwei parallele Bahnen, je Bahn 2 Roboter (Laeufer 1 und 2) und
  eine Farbzone in der Mitte sowie ein Zielfeld am Ende.
- **Ablauf:** Laeufer 1 startet, faehrt bis zur Farbzone (eigenes Team: Gelb
  oder Gruen), erkennt sie mit „Fahre bis Farbe" — und gibt automatisch per
  ESP-NOW das Staffelholz. Laeufer 2 startet selbsttaetig und faehrt zum
  Zielfeld. Gemessen werden Etappenzeit und Uebergabezeit (Zeit zwischen
  Zonen-Erkennung und Start des Laeufers 2).
- **Sieg:** schnellere Gesamtzeit des Teams.
- **Lernen:** Teamwork, Reihenfolgen, „einer haengt am anderen".
- **Spassfaktor:** hoch in groesseren Gruppen.
- **Aufwand:** mittel bis gross — braucht Broadcast-Pakete mit Team-/Etappen-
  Kennung (heute ist ESP-NOW nur 1:1-Paarung; `MAX_PEERS = 4` ist vorhanden,
  die Rollen-Logik muss neu).
- **Hinweis:** Die UI-Koordination („gleiches GO fuer alle 4") ist der
  aufwaendigste Teil, nicht die Roboterseite.

#### A8 - Robo-Fußball

- **Spieler:** 2 Roboter (halb-autonom)
- **Aufbau:** Zwei Tore als farbiges Klebeband (ROT und BLAU) an den
  Stirnseiten der Arena, ein Ball (Tennisball passt zur Robotergroesse) in
  der Mitte.
- **Ablauf:** Das Kind lenkt seinen Roboter per App-Joystick, versucht den
  Ball ins Tor zu schuppsen. Tor-Erkennung, Punktezaehlung, Jubel (Regenbogen)
  und Ball-Reset macht der Roboter selbst: einfahren zur Mitte, kurzer Ton
  der Anzeige (LED-Impuls), Seitenwechsel zur Halbzeit.
- **Sieg:** mehr Tore nach 2 x 3 Minuten, bei Gleichstand Verlaengerung.
- **Lernen:** Koerperbeherrschung und Raumorientierung aus der
  Vogelperspektive.
- **Spassfaktor:** sehr hoch.
- **Aufwand:** **gross** — die Ball-Physik (Ball groesse, Roboterlippe,
  Boden) muss am echten Geraet getunt werden. Groesstes Hardware-Risiko im
  ganzen Plan; erst angehen, wenn A-Reihe sonst steht.
- **Fallback:** Falls der Ball nicht zuverlaessig funktioniert: „Scheiben-
  Fußball" mit runden Pappscheiben statt Ball (gleiche Regeln, weniger
  Rollwiderstand).

### B-Reihe: Lernen

#### B1 - Zahlentreffer (Präzisions-Parcours)

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Aufbau:** Ein Zahlenstrahl aus Klebeband auf dem Boden, 0-100 cm,
  Markierungen alle 10 cm. Startlinie weiss.
- **Ablauf:** Der Roboter startet an der Startlinie, die UI nennt eine Zahl
  (z. B. „70"). Der Roboter faehrt los, das Kind muss exakt auf der Markierung
  anhalten. Der Roboter misst die Abweichung aus seinen Schrittzählern und
  zeigt sie in der UI:
  - unter 1 cm: perfekt
  - unter 3 cm: gut
  - darueber: daneben
- **Duell:** Beide Roboter bekommen dieselbe Zahl, wer naeher dran ist,
  gewinnt die Runde; Zeit ist Tiebreak.
- **Stufen:** Je 3 erfolgreiche Runden steigt die Stufe (max. 5, `ZAHL:STAGE`);
  der Roboter rollt pro Stufe 20 mm/s schneller (100-180 mm/s). Ziel und
  Wertung (< 1 cm perfekt, < 3 cm gut) bleiben immer gleich.
- **Lernen:** Schätzen und Präzision — Abstand zwischen „ungefähr" und
  „exakt" wird am eigenen Ergebnis sichtbar.
- **Aufwand:** klein (nutzt die Praezisions-Pruefroutine aus 2.2).

#### B8 - Geheim-Strecke (Schaetzen + Kalibrieren)

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Ablauf:** Der Roboter wuerfelt eine geheime Strecke (Stufe 1: 20-50 cm).
  Das Kind waehlt eine Schaetzung (10-100 cm), der Roboter faehrt die
  Geheim-Strecke ab und meldet die Abweichung mit Richtung: „x cm zu kurz
  geraten" oder „zu weit geraten". Knackt das Kind die Strecke (Stufe 1:
  <= 2 cm), gibt es eine neue.
- **Stufen:** Je geknackter Strecke waechst der Bereich (20-50, 10-80,
  10-100 cm) und die Toleranz wird enger (perfekt 2/1,5/1 cm, gut 5/4/3 cm),
  max. Stufe 3 (`GEHEIM:STAGE`).
- **Protokoll:** `GEHEIM:START` (neue Strecke), `GEHEIM:START:<cm>` (Ratesrunde,
  10-100 cm in 1-cm-Schritten (Slider), `GEHEIM:STOP`; zurueck kommen `GEHEIM:READY`,
  `GEHEIM:GO`, `GEHEIM:RESULT:<mmAbw>,<0-2>` (+ = weiter als geraten),
  `GEHEIM:STOPPED`, `GEHEIM:BUSY`.
- **Duell:** Beide Roboter wuerfeln eigene Strecken, naeher dran gibt mehr
  Punkte (gleiche Punkteskala wie A5/B1, hoeher gewinnt).
- **Lernen:** Schaetzen, Kalibrieren, Zahlensinn — die dritte Faehigkeit
  neben Reaktion (A5) und Bremsen (B1).
- **Aufwand:** klein (Positionsfahrt + Odometrie gibt es; Schaetz-UI wie B1).

#### B9 - Einparken (Distanz-Bremsung)

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Aufbau:** Freie Flaeche mit einem Hindernis (Kiste, Wand) in Fahrtrichtung.
- **Ablauf:** Der Roboter rollt langsam aufs Hindernis zu (60 mm/s). Das Kind
  bremst mit HALT so nah wie moeglich - ohne anzustossen. Der Distanzsensor
  misst den Restabstand: einziger Modus, der den ToF-Sensor als Spielgeraet
  nutzt. Anstossen stoppt von selbst (`PARK:TOUCH`), ohne Hindernis in
  Reichweite meldet er `PARK:EMPTY`.
- **Stufen:** Je 2 erfolgreiche Runden steigt die Stufe (max. 3,
  `PARK:STAGE`); die Baender werden enger (perfekt 10/8/6 cm, gut 20/15/12 cm).
- **Duell:** Beide parken vor eigenem Hindernis, naeher dran gibt mehr Punkte.
- **Lernen:** Abstands-Gefuehl, Bremsverhalten einschaetzen.
- **Aufwand:** klein (Fahrt + HALT + ToF gibt es).

#### B10 - Zeitfahren (Tempo gegen die Uhr)

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Aufbau:** Eine Zielfarbe (Rot/Gelb/Gruen/Blau) als Klebeband am Boden.
- **Ablauf:** Der Roboter faehrt los bis zur Zielfarbe (2 Treffer in Folge
  zaehlen, Timeout ohne Farbe bricht ab). Die Zeit wird gemessen - Bestzeit
  knacken ist das Spiel. Neue Bestzeit gibt 2 Punkte, Ankommen 1 Punkt.
- **Steigerung:** ueber Bestzeit-Jagd (keine Stufen): Die UI zeigt die Uhr
  live und die Bestzeit an. STOP loescht die Bestzeit (fairer Neustart).
- **Duell:** Beide fahren auf eigene Zielfarben, Punkte fuer Bestzeiten.
- **Lernen:** Zeit-Gefuehl, Optimieren (kuerzere Wege, weniger Schlenker).
- **Aufwand:** klein (Fahren-bis-Farbe gibt es; Uhr laeuft in der UI).

### C-Reihe: Kreativ und Entdecken

#### C3 - Spiegel-Tanz

- **Spieler:** 2 Roboter (2 Kinder)
- **Ablauf:** Roboter A wird von Kind 1 gesteuert und sendet laufend seine
  Bewegung als einfachen Tastenzustand (vor/zurueck/links/rechts, LED-Farbe)
  per ESP-NOW. Roboter B spiegelt: er faehrt genau dieselbe Bewegung,
  **aber links und rechts vertauscht**. Stehen sich die beiden Roboter
  gegenueber, tanzen sie synchron wie ein Spiegelbild.
- **Lern-Setting:** Kind 1 faehrt heimlich eine Form, Kind 2 muss sie vom
  Spiegelbild ablesen und benennen — Geometrie aus zwei Perspektiven.
- **Lernen:** Symmetrie, Koordinaten-Denken, Perspektivwechsel.
- **Aufwand:** klein (1 Byte Tastenzustand pro Paket; Freifahren gibt es).

---

## 4. Bewertungsmatrix

Jedes Kriterium 1-5 Punkte (5 = am besten). „Aufwand" ist hier
**Umgekehrt** gewertet: 5 = sehr leicht umsetzbar. Ø ist der Mittelwert der
vier Spalten; „MP" heisst Multiplayer-fähig.

| Modus | Lernen | Spass | Wiederspiel | Aufwand (5 = leicht) | Ø | MP |
|---|---|---|---|---|---|---|
| A3 Heiss-Kalt | 4 | 4 | 4 | 3 | 3,8 | ja |
| A4 Verfolgungsjagd | 2 | 4 | 4 | 3 | 3,3 | ja |
| A5 Grünes Licht | 3 | 5 | 4 | 5 | 4,3 | ja |
| A7 Staffel-Rennen | 3 | 4 | 3 | 3 | 3,3 | ja |
| A8 Robo-Fußball | 2 | 5 | 4 | 2 | 3,3 | ja |
| B1 Zahlentreffer | 4 | 3 | 3 | 5 | 3,8 | ja |
| B8 Geheim-Strecke | 4 | 4 | 4 | 5 | 4,3 | ja |
| B9 Einparken | 4 | 5 | 4 | 5 | 4,5 | ja |
| B10 Zeitfahren | 3 | 4 | 4 | 4 | 3,8 | ja |
| C3 Spiegel-Tanz | 4 | 5 | 3 | 5 | 4,3 | ja |

**Top nach Punkten:** B9 (4,5) - A5 / C3 / B8 (je 4,3) - A3 / B1 / B10 (je 3,8).

---

## 5. Umsetzungs-Wellen

Die Reihenfolge folgt der Regel: erst Modi ohne neue Infrastruktur, dann die,
die auf den Querschnitts-Mechanismen (Pose, Praezision) aufbauen. So steht
jede Welle fuer sich und laesst sich einzeln ausliefern.

| Welle | Modi | Enthaltene Mechanik | Fokus | Aufwand (Firmware + UI) |
|---|---|---|---|---|
| 0 | **Fundament + A5 Einzel als Referenz-Schnitt** | HALT (weicher Spiel-Stopp), Praezisions-Pruefroutine (2.2), ModeBase, UI-Grundgeruest | Ein Schnitt durch alle Schichten — danach Copy-Paste-Tempo | ca. 2 Tage |
| 1 | **B1 Zahlentreffer, B8 Geheim-Strecke, B9 Einparken, B10 Zeitfahren** (+ Duell A5/B1/B8/B9/B10) | Praezisions-Pruefroutine, HALT, Duell-Punkte, ToF, Farbe-bis | Schnelle Erfolge, kleiner Multiplayer-Aufbau | ca. 4-5 Tage |
| 2 | **C3 Spiegel-Tanz** | 1-Byte-Bewegungspaket (C3) | Erster Multiplayer-Tanz | ca. 3-4 Tage |
| 3 | **A3 Heiss-Kalt, A4 Fangen** | Pose-Broadcast (2.1) | Volle Multiplayer-Infrastruktur wird ausgelastet | ca. 6-8 Tage |
| 4 | **A7, A8** | Team-/Etappen-Kennung (A7) | Staffel-Teamplay, Fussball-Flaggschiff | ca. 6-10 Tage |

**Warum diese Reihenfolge:**
1. Welle 0 baut die Querschnitts-Mechanismen (HALT, Praezision) und das
   UI-Grundgeruest an EINEM Modus (A5) komplett durch — erst danach werden
   die weiteren Modi nach demselben Muster kopiert.
2. Welle 1 laeuft ohne neues Funk-Protokoll (Odometrie-Zaehler, HALT,
   Praezisions-Pruefroutine); das Duell kam als Stretch dazu und nutzt das
   vorhandene Race-Pairing.
3. Welle 2 bringt mit C3 den einfachsten Multiplayer-Fall
   (Tastenzustand-Forwarding).
4. Welle 3 ist die grosse Pose-Welle. Nur hier entsteht das Risiko eines
   neuen Protokolls.
5. Fussball (A8) steht bewusst zuletzt: hoechstes Hardware-Risiko, groesster
   Tuning-Aufwand — und nur ohne Roboter-zu-Roboter-Kontakt (siehe
   Nutzerentscheidung oben).

**Risiken:**
- **A8 Fussball:** Ballgroesse/Lippe/Boden muessen am echten Roboter
  getunt werden. Fallback „Scheiben-Fussball" ist eingeplant. Zusaetzlich:
  kein Roboter-Kontakt — falls sich das ohne Kontakt nicht gut spielen
  laesst, entfaellt der Modus.
- **Pose-Drift:** fuer Baender (heiss/kalt) und Fangen-Richtung reicht die
  Odometrie; kein Modus verlaesst sich auf exakte Pose-Positionen.
- **UI-Wachstum:** `Edu_V2.html` hat bereits rund 10.000 Zeilen. Jeder Modus
  bekommt ein eigenes Modul, die UI nutzt ein gemeinsames Modus-Grundgeruest
  (Startpanel, Statuszeile, Ergebnis-Overlay) statt Einzelviews. Hinweis:
  Die UI laeuft als Datei im Browser und kostet KEINEN Roboter-Flash — das
  Risiko ist reine Pflege, kein Speicher.
- **Speicher (Firmware):** aktuell rund 43 % Flash, rund 21 % RAM belegt.
  Jedes Modul ist klein (< 5 KB); nach jeder Welle den Verbrauch pruefen.

---

## 6. UI-Leitplanken (fuer alle Modi gleich)

1. **Schueler-Seite zeigt keine Diagnose.** Unbekannte Befehle, Rohwerte und
   Fehler bleiben auf der Entwicklerseite (Regel aus dem README).
2. **Ein Grundgeruest pro Modus:** Titel, Kurz-Erklarung (1 Satz),
   Start/Stop-Knopf, Live-Statuszeile, Ergebnis-Overlay mit Punktzahl. Neue
   Modi fuellen das Geruest, statt neue Layouts zu erfinden.
3. **Farben und Formen wie bisher:** Handschrift-Look, keine System-Glyphen
   (`tools/check_no_emoji.py` haelt die Seiten sauber — das gilt auch fuer
   jede neue View).
4. **Der STOP-Knopf bleibt immer sichtbar.** Jeder Modus laesst sich
   unterbrechen; blockierende Moves pollen bereits mit `bleProcessEvents()`.
5. **Kindgerechte Rueckmeldung:** Der Roboter zeigt zuerst am LED-Ring, was
   los ist (Punkt, Fehler, Sieg) — die UI ist die ausfuehrliche Anzeige,
   nicht der einzige Kanal.

---

## 7. Nicht geplant (und warum)

| Idee | Warum nicht |
|---|---|
| Roboter-Kollisionsspiele (schubsen, Sumo-Ringen) | Nutzerentscheidung 2026-10-10: keine Kollisionen am Fahrzeug. A1/A2 gestrichen; A8 nur ohne Kontakt. |
| RSSI-Spiele („heiss/kalt" ueber Funkstaerke) | ESP-IDF 4.4 liefert keine RSSI im Empfangspfad; Core-Upgrade waere ein Risiko fuer TMC/BLE/WiFi. Pose-Broadcast leistet dasselbe. |
| Ton/Musik-Spiele | Kein Lautsprecher am Roboter. |
| Greifer-Spiele (aufsammeln, stapeln) | Kein Servo/Greifer verbaut. |
| Kamera-/Bilderkennung | Keine Kamera; der Farbsensor deckt die Rolle ab. |
| Kippel-/Lage-Spiele | Kein Beschleunigungssensor (IMU) vorhanden. |
| Labyrinth selbst loesen | Braucht Mapping ueber mehrere Minuten und exakte Odometrie — Drift macht es unzuverlaessig. |

---

## 8. Naechste Schritte

1. **Geraetetests** der 5 umgesetzten Modi (A5, B1, B8, B9, B10) (Testkarten in
   `GAME_MODES_W0_W1.md`, Abschnitt 4; Testergebnisse hier eintragen).
2. Danach Entscheid: Welle 2 (C3 Spiegel-Tanz) oder Welle 3 (Pose-Broadcast
   fuer A3/A4 + Duelle).
3. Dieses Dokument bei jedem umgesetzten Modus fortschreiben („Status:
   umgesetzt" je Absatz), damit der Plan die Landkarte des Projekts bleibt.



