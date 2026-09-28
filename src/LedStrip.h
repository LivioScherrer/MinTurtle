// EduBot V2 - LED-Streifen-Modul (WS2812, 7 LEDs an GPIO 8)
// PCB Edu_V2.2: LED1-2 haengt auf Net D9 = U3-10 = GPIO8. NICHT GPIO9!
// (U3-11/GPIO9 ist auf dem PCB offen.)
//
// Der Streifen ist die Statusanzeige des Roboters:
//   * Edu_V2.cpp waehlt in ledPatternUpdate() EIN Muster nach Prioritaet
//     (Alarm > Slalom > Spielmodus > Freifahren > Ruhezustand).
//   * ledStripTick() zeichnet das Muster zeitbasiert und nicht-blockierend
//     (max. 25 Bilder/s) - einfach in jeder loop()-Runde aufrufen.
//   * Alarm (Not-Aus) und Ergebnisblitz uebersteuern die Musterwahl fuer eine
//     feste Haltezeit, danach greift automatisch wieder die Prioritaetenkette.
//
// Farbdeutung (kurz): blau = App verbunden, orange = wartet auf App,
// gruen = bereit/erkannt/Erfolg, rot = Alarm/Fehler, violett = Memory,
// gelb = Rennen, weiss (Balken) = Fortschritt.
#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_STRIP_PIN   8
#define LED_STRIP_COUNT 7

extern Adafruit_NeoPixel ledStrip;
// true, solange der Not-Aus-Alarm blinkt (wird von ledStripShowAlert() gesetzt).
extern volatile bool ledAlertActive;

// Vordefinierte Farben fuer Statusmuster, z.B. ledStripShowProgress(..., LED_C_GREEN).
enum LedColorId : uint8_t {
  LED_C_RED = 0,
  LED_C_GREEN,
  LED_C_BLUE,
  LED_C_ORANGE,
  LED_C_YELLOW,
  LED_C_VIOLET,
  LED_C_CYAN,
  LED_C_WHITE
};
uint32_t ledColor(uint8_t id);

// Fahrtrichtung fuer das Fahr-Lauflicht (Freifahren).
enum LedDriveDir : uint8_t {
  LED_DIR_IDLE = 0,   // dreht auf der Stelle: zwei Punkte wandern nach aussen
  LED_DIR_FWD,        // Lauflicht in eine Richtung
  LED_DIR_BACK        // Lauflicht in die Gegenrichtung
};

// Linienfolger-Zustand fuer die Anzeige (wird von LineFollower.cpp geliefert).
enum LedLineState : uint8_t {
  LED_LINE_IDLE = 0,  // nicht aktiv: Streifen bleibt aus
  LED_LINE_FOLLOW,    // Linie erkannt: gruener Scanner laeuft
  LED_LINE_SEARCH,    // Linie verloren: oranges Warnblinken
  LED_LINE_OBSTACLE   // Hindernis: rotes, langsames Blinken
};

// Setup: NeoPixel init + Pixel-Selbsttest + Sensor-Selbsttest (~2 s).
// Der Sensor-Selbsttest zeigt fehlende Sensoren direkt am Streifen an
// (Farbsensor = magenta blinkend links, Distanzsensor = rot blinkend rechts).
void ledStripSetup();
// In loop() aufrufen: zeichnet das aktuelle Muster (max. 25 Bilder/s).
void ledStripTick();

// ---- Musterwahl (Alarm hat immer Vorrang) ----
// Zielfarbe (Slalom-Bauphase, S-Block): leuchtet pulsierend in der Farbe.
void ledStripShowTarget(const char* colorName);
// Ruhezustand: blau atmend = App verbunden, orange atmend = wartet auf App.
void ledStripShowStatus(bool bleConnected);
// Slalom-Paarung steht (Gegner verbunden, Spiel noch nicht gestartet): cyan atmend.
void ledStripShowPaired();
// Freifahren: Lauflicht in Fahrtrichtung (dir = LED_DIR_*).
void ledStripShowDrive(uint8_t dir);
// Fortschrittsbalken (done von total) + blinkender naechster Punkt.
void ledStripShowProgress(uint8_t done, uint8_t total, uint8_t colorId);
// Memory: Roboter zeigt gerade die Sequenz (violettes Lauflicht).
void ledStripShowMemoryShow();
// Linienfolger: Muster nach Zustand (LED_LINE_*, siehe LineFollower.h).
void ledStripShowLine(uint8_t state);
// Rennen: gelbes Lauflicht.
void ledStripShowRace();
// Startampel: rot fuellt sich bis GO, dann gruen (msLeft/totalMs aus Slalom-Timer).
void ledStripShowCountdown(uint32_t msLeft, uint32_t totalMs);
// Slalom-Ergebnis: Sieg = Regenbogen, Niederlage = rotes Atmen.
void ledStripShowResult(bool iWon);
// Kurzer gruener (ok) / roter Blitz - z.B. Quiz-Ergebnis pro Schritt.
void ledStripShowResultFlash(bool ok);

// ---- Alarm / Basis ----
// Not-Aus: rotes Doppelblinken fuer 3 s, hoechste Prioritaet.
void ledStripShowAlert();
// Alle LEDs aus; beendet auch Alarm/Blitz (fuer Slalom-Rennen: der Streifen
// darf den Farbsensor am Boden nicht mit der Zielfarbe verwechseln).
void ledStripClear();
