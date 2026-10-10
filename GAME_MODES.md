# Neue Spiel-Modi: Planung EduBot V2

Planung fuer 19 neue Spiel-Modi des EduBot. Basis sind die in `README.md` und
`HARDWARE_FAKTEN.md` beschriebene Firmware und Hardware. Dieser Plan beschreibt
Konzept, Mechanik, Lernwert und Aufwand je Modus; er ist keine
Implementierungsanleitung.

Stand: 2026-10-10. Der Farbsensor (TCS34725) ist in Betrieb (am Roboter
bestaetigt) — Farb-Spiele sind moeglich, ein I2C-Multiplexer ist nicht mehr
notwendig (der Patch in `patch_vl53l0x.py` loest den Adresskonflikt mit dem
VL53L0X).

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
- Genutzt von den Modi A5, B1 und B2 — dort ist die gemessene Abweichung der
  Spielstand. Die Toleranz-Schwellen stehen im Modul, damit alle drei Modis
  dieselbe „Fairness" haben.

### 2.3 Integrationsmuster (fuer alle 19 Modi identisch)

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

## 3. Die 19 Spiel-Modi

### A-Reihe: Action und Spass

#### A1 - König des Berges (King of the Hill)

- **Spieler:** 2-4 Roboter (Paarungen nacheinander)
- **Aufbau:** Ein Ring aus schwarzem Klebeband (ca. 80-120 cm Durchmesser) auf
  hellem Boden. Jeder Roboter startet in der Mitte.
- **Ablauf:** Nach dem Start fahren beide Roboter los. Ziel ist es, selbst im
  Ring zu bleiben und den Gegner herauszuplaeren. Beruehrt ein Roboter das
  schwarze Band, faellt er heraus: 3 Sekunden Strafbox ausserhalb, dann
  zurueck in die Mitte. Danach geht die Runde weiter.
- **Sieg:** Best-of-3 Runden, oder wer nach 120 Sekunden noch oben steht.
- **Hardware:** Tischkanten-Waechter (derselbe Helligkeitsabfall wie bei
  `EDGE`), LED-Ring, ESP-NOW fuer den Runden-Austausch.
- **Wichtigster Implementierungspunkt:** Der normale Kanten-Waechter sperrt
  den Roboter bis `EDGE:RESET` — fuer ein Spiel viel zu hart. Der Modus
  bekommt einen **eigenen Kanten-Check im Tick** (gleiche Schwelle, keine
  dauerhafte Sperre): erkennen, Strafbox fahren, weiterspielen.
- **Lernen:** kaum, aber: Regelverstaendnis, Fairness, „wieder aufstehen".
- **Spassfaktor:** hoch — der Klassiker unter den Kinderspielen.
- **Aufwand:** klein.

#### A2 - Sumo-Ringen

- **Spieler:** 2 Roboter
- **Aufbau:** Wie A1, aber der Ring ist kleiner (ca. 60 cm), damit es schnell
  geht.
- **Ablauf:** Roboter drehen sich langsam suchend, bis der Distanzsensor ein
  Hindernis unter 25 cm erkennt -> Vollladung vorwaerts. Wer aus dem Ring
  fliegt, verliert. Zwei Betriebsarten:
  - **AUTO:** beide Roboter kaempfen allein — die Kinder schauen zu und
    jubeln. Ideal fuer Vorfuehrungen.
  - **MANUAL:** Ein Kind faehrt per App, der Gegner-Roboter ist autonom.
- **Gegner-Erkennung:** primaer ueber die Pose aus Abschnitt 2.1 (robust,
    unabhaengig von der Einbaulage des Distanzsensors). Der Distanzsensor
    bestaetigt die Beruehrung, wenn er den anderen Roboter sieht — die
    Einbaulage muss am Geraet geprueft werden (Sichtweisst bis 25 cm).
- **Sieg:** 3 Siege in Folge, oder wer nach 90 Sekunden mehr Runden gewonnen
    hat.
- **Lernen:** kaum — reine Arena-Action.
- **Spassfaktor:** sehr hoch, besonders als Zuschauerspiel.
- **Aufwand:** mittel (KI-Tuning: Angriffstempo, Suchverhalten).

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
  faehrt den Fluechtling per App. Beruehrung (unter 15 cm) gibt dem Jaeger
  einen Punkt, dann Rollentausch.
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
- **Lernen:** Impulskontrolle, Reaktionszeit — paedagogisch wertvoll.
- **Spassfaktor:** sehr hoch, Party-tauglich, einfache Regel.
- **Aufwand:** klein (nutzt die Praezisions-Pruefroutine aus 2.2).

#### A6 - Hand-Faenger (Zauberhand)

- **Spieler:** 1 Roboter, kein App-Zwang
- **Ablauf:** Der Roboter haelt per Distanzsensor konstanten Abstand zu
  deiner Hand: Hand naeher -> er weicht zurueck, Hand weiter -> er folgt.
  Fuehre die Hand, und der Roboter folgt dir durch den Raum — ganz ohne
  Smartphone (nur Start/Stop per UI-Knopf).
- **Spiel-Element „Fuettern":** Blaue Zone am Boden = „satt" (Roboter haelt
  3 Sekunden still, dann Regenbogen + Punkt). Rote Zone = „Falle" (Beruehrung
  unter 5 Sekunden = 5 Sekunden Einfrieren mit rotem LED-Standbild).
- **Lernen:** geschlossener Regelkreis erleben — der Roboter regelt sich
  selbst, ohne dass jemand programmiert.
- **Spassfaktor:** sehr hoch, weil ohne App sofort begreifbar.
- **Aufwand:** klein (Regelkreis auf den Distanzsensor, 2 Farbzonen).

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

#### A9 - Minenfeld

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Aufbau:** Ein Kurs (Klebeband-Pfad oder freie Flaeche) mit 5-8 schwarzen
  „Minen" (runde Aufkleber, 4-6 cm). Startfeld weiss, Zielfeld blau.
- **Ablauf:** 5 Sekunden Zeit, um die Mine zu umfahren (Block maximale
  Fahrstrecke begrenzen):
  1. Kind faehrt den Roboter per App oder programmiert blind mit Bloecken.
  2. Beruehrt der Roboter eine schwarze Mine: rotes Aufblitzen, 5 Sekunden
     Strafzeit, der Roboter wird zurueckgesetzt auf den letzten Checkpoint.
  3. Zielfeld erreicht: Zeit wird gestoppt.
- **Duell:** Zwei Roboter, gleicher Kurs, schnellste minenfreie Fahrt
  gewinnt.
- **Lernen:** Risiko-Einschätzung, Iteration als Teil des Lernens.
- **Spassfaktor:** hoch — Spannung bis zur letzten Mine.
- **Aufwand:** mittel (Mine = Schwarz-Erkennung unter dem Roboter, gibt es
  schon; Checkpoints neu).

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
- **Lernen:** Schätzen und Präzision — Abstand zwischen „ungefähr" und
  „exakt" wird am eigenen Ergebnis sichtbar.
- **Aufwand:** klein (nutzt die Praezisions-Pruefroutine aus 2.2).

#### B2 - Winkel-Fischer

- **Spieler:** 1 Roboter, oder 2 im Duell
- **Ablauf:** Der Roboter bekommt einen Zielwinkel (z. B. 135 Grad), angezeigt
  als pulsierendes Muster am LED-Ring plus Zahl in der UI. Das Kind dreht ihn
  mit zwei Knoepfen (plus 15 / minus 15 Grad) in Stellung. Nach „Pruefen"
  rechnet der Roboter seine absolute Drehung aus der Summe aller
  Dreh-Schritte und meldet die Abweichung.
- **Stufen:** 45 Grad Toleranz (leicht), 15 Grad (mittel), 5 Grad (schwer).
- **Duell:** gleicher Zielwinkel fuer beide, naeher dran gewinnt.
- **Lernen:** Winkel-Gefuehl ohne Geodreieck — die Drehung wird gemessen,
  nicht geschaetzt.
- **Aufwand:** mittel (LED-Puls als Winkelanzeige ist neu; Drehung selbst
  gibt es ueber `gameTurnLeftSteps`).

#### B3 - Schatzsuche (Der blinde Programmierer)

- **Spieler:** 1 Roboter (Selbstversuch), 2 im Duell
- **Aufbau:** Ein „Schatz" (gelbes Plaettchen) liegt irgendwo im Raum. Der
  Roboter startet 2-3 m davon entfernt.
- **Ablauf:** Das Kind baut mit den vorhandenen Programmier-Bloecken einen
  Blind-Pfad („3 Kacheln vor, drehe links, 5 vor") und drueckt Start — ohne
  hinzusehen. Der Roboter faehrt das Programm exakt ab.
- **Treffer:** Endet der Roboter auf dem gelben Plaettchen, gewinnt er
  (Farberkennung am Programmende). Sonst zeigt der LED-Ring die Richtung zum
  Schatz und die Blink-Geschwindigkeit die Entfernung — das Kind korrigiert
  sein Programm und startet erneut.
- **Hilfe:** Ab Versuch 3 wird ein Hinweis freigeschaltet (die UI zeigt die
  Soll-Route des Lehrers).
- **Lernen:** **das paedagogische Flaggschiff des Plans.** Odometrie,
  Drift-Erfahrung („der Roboter macht genau was ich sage, und landet doch
  daneben"), Programm-Korrigieren als Lernschleife.
- **Aufwand:** mittel (blosse Programm-Ausfuehrung gibt es; Farberkennung am
  Programmende plus Richtungs-Hinweis neu).

#### B4 - Formen-Zauberer

- **Spieler:** 1 Roboter
- **Ablauf:** Auf Knopfdruck faehrt der Roboter geometrische Formen: Quadrat,
  Dreieck, Fünfeck, Acht, Spirale. An jeder Ecke blinkt der LED-Ring und die
  UI zeigt den Winkel der Ecke. Das Kind rät der Reihe nach: „wie viele
  Ecken hat die naechste Form?"
- **Ausbau:** Erweiterung „eigene Form" — nimmt das aktuelle Block-Programm
  und faehrt es als Form ab (Bruecke zu C1).
- **Lernen:** Geometrie aus der Vogelperspektive, Winkel wiedererkennen.
- **Aufwand:** klein (Sequenz aus Fahren + Drehen, gibt es als Primitive).

#### B5 - Sonnenblume

- **Spieler:** 1 Roboter
- **Ablauf:** Der Roboter dreht sich in 30-Grad-Schritten und misst jedes Mal
  die gemessene Helligkeit (Gesamthelligkeit des Farbsensors). Danach dreht
  er zur hellsten Richtung und faehrt ein Stueck darauf zu. Am „Licht"
  (Fenster, Lampe, Taschenlampen-Kind) kommt er zur Ruhe und jubelt.
- **Erweiterung „Tag und Nacht":** Zwei Lichtquellen (Lampe hell, Kerze
  dunkel). „Sonne"-Modus: Roboter faehrt zur helleren. „Mondblume"-Modus: zur
  dunkleren.
- **Lernen:**   Optimierung durch Suchen, Sensorik verständlich machen — und
  das echte Vorbild: die Sonnenblume richtet sich nach dem Licht aus
  (Heliotropismus).
- **Aufwand:** klein (Helligkeits-Sampling als Schleife, Lichtquelle reicht).

#### B6 - Futter-Suche

- **Spieler:** 1-2 Roboter
- **Aufbau:** 6-8 Farbkarten liegen offen im Feld: 3 gruene Blaetter (+1),
  2 rote Giftpilze (-1, 3 s Strafzeit), 1 gelber Stern (+5).
- **Ablauf:** 45 Sekunden Sammelzeit. Der Roboter faehrt autonom mit
  „Fahre bis Farbe", erkennt die Karte unter sich und nimmt sie (virtuell).
  Duell: zwei Roboter im selben Feld — Karten sind begrenzt, es entsteht ein
  Wettrennen um die besten Karten.
- **Sieg:** meiste Punkte; Bonus-Stern fuer die schnellste korrekte Fahrt.
- **Lernen:** autonome Entscheidung unter Zeitdruck, Risiko (Gift kostet).
- **Aufwand:** mittel („Fahre bis Farbe" gibt es; Sammel-Logik und
  Strafzeiten neu).

#### B7 - Blinder Pilot

- **Spieler:** 2 Kinderpaare mit je 1 Roboter
- **Aufbau:** Der Roboter steht hinter einem Sichtschutz (Aktendeckel, Karton)
  oder in einem Nebenraum. Ein farbiges Zielfeld liegt irgendwo im Feld.
- **Ablauf:** Der Partner programmiert mit Bloecken „fahre zum Ziel" oder
  steuert per Freifahren. Damit es ein Spiel bleibt, gibt es ein
  **Mess-Budget**: nur 5 Messungen (`DIST?`) darf der Partner waehrend der
  Fahrt anfordern. Jede weitere Messung kostet einen Punkt. Faehrt der
  Roboter auf ein Hindernis, bleibt er stehen — der Partner muss mit seinem
  Wissen weiterplanen.
- **Sieg:** steht der Roboter am Programmende auf dem Zielfeld und hat das
  groesste Mess-Budget uebrig, gewinnt das Team.
- **Lernen:** Kommunikation, Programmieren mit beschraenkter Information —
    „Was muss ich wissen, um entscheiden zu koennen?"
- **Aufwand:** klein (bestehende Bloecke + `DIST?` + Zaehler; kein neues
    Bewegungsverhalten).

### C-Reihe: Kreativ und Entdecken

#### C1 - Licht-Malerei

- **Spieler:** 1-4 Roboter (auch gemeinsam als Klasse)
- **Ablauf:** In einem abgedunkelten Raum faehrt der Roboter mit
  eingeschaltetem LED-Ring ein vorgefertigtes Bild: Herz, Stern, Buchstabe,
  Spirale. Mit der Langzeitbelichtung eines Handys wird das Bild
  „gemalt" — ein Foto fuer die Klasse oder den Elternabend. Kinder koennen
  eigene Pfade mit Bloecken bauen und dann als Licht-Spur abfahren. Bei
  mehreren Robotern fahren verschiedene Farben gleichzeitig.
- **Lernen:** ausgefeilte Praezisions-Wege, Gestaltung mit Technik (STEAM).
- **Aufwand:** klein (Form-Abfahrt gibt es aus B4; LED-Ring sowieso).

#### C2 - Erkundungs-Modus (Kartograf)

- **Spieler:** 1-2 Roboter
- **Ablauf:** Der Roboter erkundet autonom: vorwaerts bis ein Hindernis unter
  30 cm auftaucht, Stopp, kurzes „Nachdenken" am LED-Ring, 90 Grad drehen,
  weiter. Beim zweiten blockierten Versuch in Folge weicht er zurueck aus
  (20 cm zurueck, 180 Grad drehen). Der Kanten-Waechter haelt ihn auf dem
  Tisch. Ereignisse (Stopp, Drehung, Position) gehen an die UI.
- **Schul-Variante:** Die Kinder zeichnen waehrenddessen das Zimmer auf
  Papier — danach Vergleich mit der „Karte" des Roboters (Pose-Punkte).
- **Lernen:** autonome Systeme, Kartenlesen, Zusammenführen von Beobachtung
  und Modell.
- **Aufwand:** mittel (Zustandsmaschine mit Ecken-Ausweichen; Karten-
  Anzeige in der UI optional).

#### C3 - Spiegel-Tanz

- **Spieler:** 2 Roboter (2 Kinder)
- **Ablauf:** Roboter A wird von Kind 1 gesteuert und sendet laufend seine
  Bewegung als einfachen Tastenzustand (vor/zurueck/links/rechts, LED-Farbe)
  per ESP-NOW. Roboter B spiegelt: er faehrt genau dieselbe Bewegung,
  **aber links und rechts vertauscht**. Stehen sich die beiden Roboter
  gegenueber, tanzen sie synchron wie ein Spiegelbild.
- **Lern-Setting:** Kind 1 faehrt heimlich eine Form, Kind 2 muss sie vom
  Spiegelbild ablesen und benennen — Geometrie aus zwei Perspektiven.
- **Erweiterung:** A faehrt mit eingeschaltetem LED-Ring (C1), B spiegelt
  mit Langzeitbelichtung — „Doppel-Lichtmalerei".
- **Lernen:** Symmetrie, Koordinaten-Denken, Perspektivwechsel.
- **Aufwand:** klein (1 Byte Tastenzustand pro Paket; Freifahren gibt es).

---

## 4. Bewertungsmatrix

Jedes Kriterium 1-5 Punkte (5 = am besten). „Aufwand" ist hier
**Umgekehrt** gewertet: 5 = sehr leicht umsetzbar. Ø ist der Mittelwert der
vier Spalten; „MP" heisst Multiplayer-fähig.

| Modus | Lernen | Spass | Wiederspiel | Aufwand (5 = leicht) | Ø | MP |
|---|---|---|---|---|---|---|
| A1 König des Berges | 3 | 5 | 4 | 5 | 4,3 | ja |
| A2 Sumo | 2 | 5 | 3 | 3 | 3,3 | ja |
| A3 Heiss-Kalt | 4 | 4 | 4 | 3 | 3,8 | ja |
| A4 Verfolgungsjagd | 2 | 4 | 4 | 3 | 3,3 | ja |
| A5 Grünes Licht | 3 | 5 | 4 | 5 | 4,3 | ja |
| A6 Hand-Faenger | 4 | 5 | 4 | 5 | 4,5 | nein |
| A7 Staffel-Rennen | 3 | 4 | 3 | 3 | 3,3 | ja |
| A8 Robo-Fußball | 2 | 5 | 4 | 2 | 3,3 | ja |
| A9 Minenfeld | 3 | 4 | 4 | 3 | 3,5 | ja |
| B1 Zahlentreffer | 4 | 3 | 3 | 5 | 3,8 | ja |
| B2 Winkel-Fischer | 4 | 3 | 4 | 4 | 3,8 | ja |
| B3 Schatzsuche | 5 | 4 | 5 | 4 | 4,5 | ja |
| B4 Formen-Zauberer | 4 | 3 | 3 | 4 | 3,5 | nein |
| B5 Sonnenblume | 5 | 3 | 3 | 5 | 4,0 | nein |
| B6 Futter-Suche | 3 | 4 | 4 | 3 | 3,5 | ja |
| B7 Blinder Pilot | 5 | 4 | 4 | 5 | 4,5 | nein |
| C1 Licht-Malerei | 3 | 4 | 3 | 5 | 3,8 | nein |
| C2 Erkundung | 4 | 3 | 3 | 3 | 3,3 | nein |
| C3 Spiegel-Tanz | 4 | 5 | 3 | 5 | 4,3 | ja |

**Top nach Punkten:** A6 / B3 / B7 (je 4,5) - A1 / A5 / C3 (je 4,3) -
B5 (4,0) - A3 / B1 / B2 / C1 (je 3,8).

---

## 5. Umsetzungs-Wellen

Die Reihenfolge folgt der Regel: erst Modi ohne neue Infrastruktur, dann die,
die auf den Querschnitts-Mechanismen (Pose, Praezision) aufbauen. So steht
jede Welle fuer sich und laesst sich einzeln ausliefern.

| Welle | Modi | Enthaltene Mechanik | Fokus | Aufwand (Firmware + UI) |
|---|---|---|---|---|
| 1 | **B7 Blinder Pilot, A6 Hand-Faenger, A5 Grünes Licht** | Praezisions-Pruefroutine (2.2) | Schnelle Erfolge, kein Multiplayer-Risiko, zwei paedagogische Spitzenreiter | ca. 3-4 Tage |
| 2 | **B3 Schatzsuche, A1 König des Berges, C3 Spiegel-Tanz** | Kanten-Check als Spiel-Logik (A1), 1-Byte-Bewegungspaket (C3) | Das paedagogische Flaggschiff; erste Arena; erster Multiplayer-Tanz | ca. 5-6 Tage |
| 3 | **A3 Heiss-Kalt, A4 Fangen, A2 Sumo, B2 Winkel-Fischer, B1 Zahlentreffer** | Pose-Broadcast (2.1) | Volle Multiplayer-Infrastruktur wird ausgelastet | ca. 7-9 Tage |
| 4 | **B5, B4, C1, A9, B6, A7, C2, A8** | alles weitere | Autonomie, Kunst, Fussball-Flaggschiff | ca. 12-16 Tage |

**Warum diese Reihenfolge:**
1. Welle 1 laeuft vollstaendig auf dem, was heute existiert (Distanzsensor,
   Bloecke, Odometrie-Zaehler). Kein ESP-NOW, kein neues Protokoll.
2. Welle 2 bringt mit C3 den einfachsten Multiplayer-Fall (Tastenzustand-
   Forwarding) und mit A1 den Kanten-Check — die Vorarbeit fuer A2 und A4.
3. Welle 3 ist die grosse Pose-Welle. Nur hier entsteht das Risiko eines
   neuen Protokolls; es steht an 3. Stelle, damit die Mechanik aus 2.1 in
   Welle 2 nicht gebraucht wird.
4. Fussball (A8) steht bewusst zuletzt: hoechstes Hardware-Risiko, groesster
   Tuning-Aufwand.

**Risiken:**
- **A8 Fussball:** Ball groesse/Lippe/Boden muessen am echten Roboter
  getunt werden. Fallback „Scheiben-Fussball" ist eingeplant.
- **Pose-Drift:** fuer Bänder (heiss/kalt) und Fangen-Richtung reicht die
  Odometrie; kein Modus verlaesst sich auf exakte Pose-Positionen.
- **UI-Wachstum:** `Edu_V2.html` hat bereits rund 10.000 Zeilen. Jeder Modus
  bekommt ein eigenes Modul, die UI nutzt ein gemeinsames Modus-Grundgeruest
  (Startpanel, Statuszeile, Ergebnis-Overlay) statt 14 Einzelviews.
- **Speicher:** aktuell rund 42 % Flash, rund 21 % RAM belegt. Jedes
  Modul ist klein (< 5 KB), aber 19 Module plus UI-Tiles summieren sich —
  nach Welle 2 und 3 den Verbrauch pruefen.

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
| RSSI-Spiele („heiss/kalt" ueber Funkstaerke) | ESP-IDF 4.4 liefert keine RSSI im Empfangspfad; Core-Upgrade waere ein Risiko fuer TMC/BLE/WiFi. Pose-Broadcast leistet dasselbe. |
| Ton/Musik-Spiele | Kein Lautsprecher am Roboter. |
| Greifer-Spiele (aufsammeln, stapeln) | Kein Servo/Greifer verbaut. |
| Kamera-/Bilderkennung | Keine Kamera; der Farbsensor deckt die Rolle ab. |
| Kippel-/Lage-Spiele | Kein Beschleunigungssensor (IMU) vorhanden. |
| Labyrinth selbst loesen | Braucht Mapping ueber mehrere Minuten und exakte Odometrie — Drift macht es unzuverlaessig. |

---

## 8. Naechste Schritte

1. **Welle 1 umsetzen** (B7, A6, A5 + Praezisions-Pruefroutine): Start frei-
   geben, dann Modul fuer Modul.
2. Nach Welle 1: kurzer Test am Geraet (2 halbe Tage), danach Entscheid
   ueber Welle 2.
3. Dieses Dokument bei jedem umgesetzten Modus fortschreiben („Status:
   umgesetzt" je Absatz), damit der Plan die Landkarte des Projekts bleibt.



