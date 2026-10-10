// EduBot V2 - LED-Streifen-Modul (WS2812, 7 LEDs an GPIO 8, siehe LedStrip.h)
// PCB Edu_V2.2: Net D9 = GPIO8.
//
// Aufbau:
//   * ledStripShow*() waehlt nur ein Muster (Zustand, Farbe, Parameter).
//   * ledStripTick() zeichnet das Muster zeitbasiert - kein delay, damit die
//     Animationsschritte die Main-Loop nicht blockieren.
//   * Alarm (Not-Aus) und Ergebnisblitz setzen eine Haltezeit (s_holdUntil);
//     solange ignoriert setPattern() jede neue Musterwahl.
#include "LedStrip.h"
#include "ColorSensor.h"     // colorSensorOk - Boot-Selbsttest
#include "DistanceSensor.h"  // distOk()      - Boot-Selbsttest

Adafruit_NeoPixel ledStrip(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

volatile bool ledAlertActive = false;

// =====================================================
// MUSTER
// =====================================================
enum LedPattern : uint8_t {
  PAT_OFF = 0,
  PAT_SOLID,       // alle LEDs dauerhaft in einer Farbe
  PAT_BLINK,       // alle LEDs blinken (s_p1: Periode/100ms, min. 1)
  PAT_BREATHE,     // alle LEDs atmen (s_p1: Periode/100ms, min. 1)
  PAT_TARGET,      // Zielfarbe, sanftes Pulsieren
  PAT_ALERT,       // Not-Aus: rotes Doppelblinken (Haltezeit)
  PAT_RESULT,      // kurzer Erfolgs-/Fehlerblitz (Haltezeit)
  PAT_IDLE,        // Ruhe: blau (verbunden) / orange (wartet) atmen
  PAT_PAIRED,      // Slalom gepaart: cyan atmen
  PAT_DRIVE,       // Freifahren: Lauflicht in Fahrtrichtung
  PAT_PROGRESS,    // Fortschrittsbalken mit blinkendem naechstem Punkt
  PAT_MEMORY,      // Memory-Sequenz laeuft (violettes Lauflicht)
  PAT_LINE,        // Linienfolger: Scanner / Warnblinken
  PAT_RACE,        // Rennen: gelbes Lauflicht
  PAT_COUNTDOWN,   // Startampel rot -> gruen (GO)
  PAT_WIN,         // Sieg: Regenbogen
  PAT_LOSE         // Niederlage: rotes Atmen
};

static uint8_t  s_pattern   = PAT_OFF;
static uint32_t s_color     = 0;   // Hauptfarbe (Ziel/Fortschritt/Solid/Blink/Breathe)
static uint8_t  s_p1        = 0;   // Parameter 1 (Richtung/Fortschritt/Phase/Periode)
static uint8_t  s_p2        = 0;   // Parameter 2 (Gesamtzahl)
static uint32_t s_setAt     = 0;   // Musterbeginn (Basis fuer Animationen)
static uint32_t s_frameAt   = 0;   // Zeitpunkt des letzten gezeichneten Bildes
static uint32_t s_holdUntil = 0;   // solange uebersteuern Alarm/Blitz die Musterwahl
static bool     s_offShown  = true;  // PAT_OFF schon gezeichnet? (spart show()s)

static const uint32_t LED_FRAME_MS    = 40;    // ~25 Bilder/s
static const uint32_t ALERT_MS        = 3000;  // Dauer Not-Aus-Alarm
static const uint32_t RESULT_FLASH_MS = 1600;  // Dauer Ergebnisblitz
static const uint8_t  LED_BRIGHTNESS  = 40;    // wie bisher (ca. 16 %)

// Eine Stelle fuer alle Statusfarben - statt verstreuter Color(...)-Literale.
uint32_t ledColor(uint8_t id) {
  switch (id) {
    case LED_C_RED:    return ledStrip.Color(255, 0, 0);
    case LED_C_GREEN:  return ledStrip.Color(0, 255, 0);
    case LED_C_BLUE:   return ledStrip.Color(0, 90, 255);
    case LED_C_ORANGE: return ledStrip.Color(255, 110, 0);
    case LED_C_YELLOW: return ledStrip.Color(255, 170, 0);
    case LED_C_VIOLET: return ledStrip.Color(150, 0, 255);
    case LED_C_CYAN:   return ledStrip.Color(0, 180, 255);
    case LED_C_WHITE:  return ledStrip.Color(255, 255, 255);
  }
  return 0;
}

// Name (Farbsensor / Slalom / S-Block) -> RGB-Farbe.
// Die vier Zielfarben sind ABSICHTLICH die reinen Grundfarben: das Kind
// vergleicht den Streifen mit dem Farbplaettchen.
uint32_t ledColorByName(const char* name) {
  if (!name) return 0;
  if (strcasecmp(name, "ROT") == 0)     return ledStrip.Color(255, 0, 0);
  if (strcasecmp(name, "GELB") == 0)    return ledStrip.Color(255, 150, 0);
  if (strcasecmp(name, "GRUEN") == 0)   return ledStrip.Color(0, 255, 0);
  if (strcasecmp(name, "BLAU") == 0)    return ledStrip.Color(0, 0, 255);
  if (strcasecmp(name, "WEISS") == 0)   return ledStrip.Color(255, 255, 255);
  if (strcasecmp(name, "SCHWARZ") == 0) return ledStrip.Color(0, 0, 0);
  if (strcasecmp(name, "BODEN") == 0)   return ledStrip.Color(40, 40, 40);
  // S-Block Slots 4/5: Distanz-Anzeige (NAH/FREI), siehe ProgGame.h.
  if (strcasecmp(name, "NAH") == 0)     return ledStrip.Color(180, 0, 0);
  if (strcasecmp(name, "FREI") == 0)    return ledStrip.Color(0, 180, 0);
  return ledStrip.Color(80, 0, 80);  // UNBEKANNT -> violett
}

// =====================================================
// ZEICHEN-HELFER (eine Stelle statt Copy-Paste pro Muster)
// =====================================================

// Farbe mit Faktor num/den skalieren (Helligkeit, ohne ledStrip.setBrightness
// anzufassen - das wuerde auch alle anderen Muster treffen).
static uint32_t scaleColor(uint32_t c, uint8_t num, uint8_t den) {
  const uint32_t r = ((c >> 16) & 0xFF) * num / den;
  const uint32_t g = ((c >> 8) & 0xFF) * num / den;
  const uint32_t b = (c & 0xFF) * num / den;
  return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
}

static void fillAll(uint32_t c) {
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) ledStrip.setPixelColor(i, c);
}

// Lauflicht-Punkt mit kurzem Schweif: Helligkeit faellt pro Pixel ab
// (100 % / ~43 % / ~16 % - weicher als die alte 1/(d+1)-Stufung).
static void setDot(uint8_t pos, uint32_t c, uint8_t trailLen) {
  static const uint8_t kTrail[3] = { 255, 110, 40 };
  if (trailLen > 3) trailLen = 3;
  for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
    const uint8_t d = (uint8_t)((pos + LED_STRIP_COUNT - i) % LED_STRIP_COUNT);
    if (d < trailLen) {
      ledStrip.setPixelColor(i, scaleColor(c, kTrail[d], 255));
    }
  }
}

// Dreieckswelle 0 -> 255 -> 0 gleichmaessig ueber 'period' ms (fuer Atmen).
static uint8_t wave(uint32_t now, uint32_t period) {
  if (period == 0) return 255;
  const uint32_t t    = now % period;
  const uint32_t half = period / 2;
  if (half == 0) return 255;
  return (uint8_t)(t < half ? (t * 255 / half) : ((period - t) * 255 / half));
}

// true in der ersten Haelfte jeder Periode - Basis fuer alles Blinkende.
// ABSICHTLICH ab 'now', nicht ab Musterbeginn: sonst springt die Phase bei
// jedem Musterwechsel (z.B. Fortschrittsbalken-Update) sichtbar um.
static bool blinkOn(uint32_t now, uint32_t periodMs) {
  if (periodMs == 0) return true;
  return (now % periodMs) < (periodMs / 2);
}

// Atmungs-Skalierung minPct..maxPct (0..255) ueber 'period' ms.
static uint8_t breatheScale(uint32_t now, uint32_t periodMs, uint8_t minV, uint8_t maxV) {
  return (uint8_t)(minV + (uint16_t)wave(now, periodMs) * (maxV - minV) / 255);
}

// Hin-und-her-Position 0..last ueber 'period' ms (fuer Scanner).
static uint8_t pingPong(uint32_t now, uint32_t periodMs, uint8_t last) {
  if (periodMs == 0 || last == 0) return 0;
  const uint32_t t = now % periodMs;
  const uint32_t half = periodMs / 2;
  return (uint8_t)(t < half ? (t * last / half) : (last - (t - half) * last / half));
}

// =====================================================
// MUSTERWAHL
// =====================================================
static void setPattern(uint8_t p, uint32_t color, uint8_t p1, uint8_t p2) {
  const uint32_t now = millis();
  if (now < s_holdUntil) return;   // Alarm/Ergebnisblitz laeuft noch
  if (s_pattern == p && s_color == color && s_p1 == p1 && s_p2 == p2) return;
  s_pattern = p;
  s_color   = color;
  s_p1      = p1;
  s_p2      = p2;
  s_setAt   = now;
  s_frameAt = 0;                   // naechster Tick zeichnet sofort
  s_offShown = false;
}

// Override (Alarm/Blitz) starten: setzt Muster + Haltezeit an einer Stelle.
static void startOverride(uint8_t p, uint32_t color, uint32_t holdMs) {
  const uint32_t now = millis();
  if (now < s_holdUntil) return;
  s_pattern   = p;
  s_color     = color;
  s_p1        = 0;
  s_p2        = 0;
  s_setAt     = now;
  s_frameAt   = 0;
  s_offShown  = false;
  s_holdUntil = now + holdMs;
}

bool ledIsOverrideActive() {
  return s_holdUntil != 0 && millis() < s_holdUntil;
}

void ledStripShowSolid(LedColorId color) {
  setPattern(PAT_SOLID, ledColor(color), 0, 0);
}

void ledStripShowBlink(LedColorId color, uint16_t periodMs) {
  // Periode in 100-ms-Schritten speichern (reicht bis 25,5 s, spart Parameter).
  uint8_t steps = (uint8_t)(periodMs / 100);
  if (steps == 0) steps = 1;
  setPattern(PAT_BLINK, ledColor(color), steps, 0);
}

void ledStripShowBreathe(LedColorId color, uint16_t periodMs) {
  uint8_t steps = (uint8_t)(periodMs / 100);
  if (steps == 0) steps = 1;
  setPattern(PAT_BREATHE, ledColor(color), steps, 0);
}

void ledStripShowTarget(const char* colorName) {
  setPattern(PAT_TARGET, ledColorByName(colorName), 0, 0);
}

void ledStripShowTarget(LedColorId color) {
  setPattern(PAT_TARGET, ledColor(color), 0, 0);
}

void ledStripShowStatus(bool bleConnected) {
  setPattern(PAT_IDLE, 0, bleConnected ? 1 : 0, 0);
}

void ledStripShowPaired() {
  setPattern(PAT_PAIRED, 0, 0, 0);
}

void ledStripShowDrive(LedDriveDir dir) {
  setPattern(PAT_DRIVE, 0, dir, 0);
}

void ledStripShowProgress(uint8_t done, uint8_t total, LedColorId color) {
  setPattern(PAT_PROGRESS, ledColor(color), done, total);
}

void ledStripShowMemoryShow() {
  setPattern(PAT_MEMORY, 0, 0, 0);
}

void ledStripShowLine(LedLineState state) {
  setPattern(PAT_LINE, 0, state, 0);
}

void ledStripShowRace() {
  setPattern(PAT_RACE, 0, 0, 0);
}

void ledStripShowCountdown(uint32_t msLeft, uint32_t totalMs) {
  // Startampel: erst 2, dann 4, dann 6 rote LEDs, bei GO die ganze Reihe gruen.
  // Ueber Bruchteile der Gesamtzeit -> funktioniert fuer Host (3 s) und
  // Client-Fallback (4,5 s) gleich.
  uint8_t leds;
  if (msLeft == 0)                         leds = 0;   // GO
  else if (totalMs == 0)                   leds = 6;   // kein Total? voll anzeigen
  else if (msLeft * 3 > totalMs * 2)       leds = 2;   // erstes Drittel
  else if (msLeft * 3 > totalMs)           leds = 4;   // zweites Drittel
  else                                     leds = 6;   // letzte Sekunde
  setPattern(PAT_COUNTDOWN, 0, leds, 0);
}

void ledStripShowResult(bool iWon) {
  setPattern(iWon ? PAT_WIN : PAT_LOSE, 0, 0, 0);
}

void ledStripShowResultFlash(bool ok) {
  startOverride(PAT_RESULT, ledColor(ok ? LED_C_GREEN : LED_C_RED), RESULT_FLASH_MS);
}

void ledStripShowAlert() {
  const uint32_t now = millis();
  if (ledAlertActive) return;      // laufenden Alarm nicht neu starten
  ledAlertActive = true;
  s_pattern   = PAT_ALERT;
  s_color     = 0;
  s_p1        = 0;
  s_p2        = 0;
  s_setAt     = now;
  s_frameAt   = 0;
  s_offShown  = false;
  s_holdUntil = now + ALERT_MS;
}

void ledStripClear() {
  // Explizites Ausschalten gewinnt immer - auch gegen Alarm/Blitz.
  // (Slalom-Rennen: der Streifen darf die Zielfarbe nicht an den Boden werfen.)
  ledAlertActive = false;
  s_holdUntil    = 0;
  s_pattern      = PAT_OFF;
  s_color        = 0;
  s_p1           = 0;
  s_p2           = 0;
  s_frameAt      = 0;
  s_offShown     = true;
  ledStrip.clear();
  ledStrip.show();
}

// =====================================================
// RENDERN
// =====================================================
void ledStripTick() {
  const uint32_t now = millis();
  if (now - s_frameAt < LED_FRAME_MS) return;
  s_frameAt = now;

  // Haltezeit (Alarm/Blitz) vorbei? Dann Muster freigeben und dunkel werden;
  // die Prioritaetenkette in der Main-Loop waehlt gleich das naechste Muster.
  if (s_holdUntil != 0 && now >= s_holdUntil) {
    s_holdUntil    = 0;
    ledAlertActive = false;
    if (s_pattern == PAT_ALERT || s_pattern == PAT_RESULT) {
      s_pattern = PAT_OFF;
      s_offShown = false;  // gleich unten einmal dunkel zeichnen
    }
  }

  switch (s_pattern) {
    case PAT_OFF:
      // Nur einmal dunkel zeichnen - nicht 25x/s show() ins Leere rufen.
      if (!s_offShown) {
        ledStrip.clear();
        ledStrip.show();
        s_offShown = true;
      }
      return;  // show() schon erledigt (oder unnoetig)

    case PAT_SOLID:
      fillAll(s_color);
      break;

    case PAT_BLINK:
      fillAll(blinkOn(now, (uint32_t)s_p1 * 100) ? s_color : 0);
      break;

    case PAT_BREATHE: {
      const uint8_t scale = breatheScale(now, (uint32_t)s_p1 * 100, 35, 255);
      fillAll(scaleColor(s_color, scale, 255));
      break;
    }

    case PAT_TARGET: {
      // Sanftes Pulsieren, damit die Zielfarbe "lebendig" wirkt.
      const uint8_t v = wave(now, 1600);
      fillAll(scaleColor(s_color, (uint8_t)(80 + (uint16_t)v * 175 / 255), 255));
      break;
    }

    case PAT_ALERT: {
      // Doppelblitz (Not-Aus): kurz-kurz-lang Pause, klar von allem anderen
      // unterscheidbar.
      const uint32_t t = (now - s_setAt) % 600;
      const bool on = (t < 130) || (t >= 240 && t < 370);
      fillAll(on ? ledColor(LED_C_RED) : 0);
      break;
    }

    case PAT_RESULT: {
      // Erfolg/Fehler: schnelle Blitze in Gruen bzw. Rot.
      fillAll(blinkOn(now, 400) ? s_color : 0);
      break;
    }

    case PAT_IDLE: {
      const bool connected = (s_p1 != 0);
      const uint32_t period = connected ? 2600 : 1300;
      const uint8_t scale = breatheScale(now, period, 35, 255);  // 14 %..100 %
      fillAll(scaleColor(ledColor(connected ? LED_C_BLUE : LED_C_ORANGE), scale, 255));
      break;
    }

    case PAT_PAIRED: {
      // Cyan atmend: Gegner gefunden, der Host kann das Spiel starten.
      const uint8_t scale = breatheScale(now, 1800, 40, 255);
      fillAll(scaleColor(ledColor(LED_C_CYAN), scale, 255));
      break;
    }

    case PAT_DRIVE: {
      ledStrip.clear();
      if (s_p1 == LED_DIR_IDLE) {
        // Dreht auf der Stelle: zwei Punkte wandern von der Mitte nach aussen.
        const uint8_t step = (uint8_t)((now / 120) % 4);
        const uint32_t c = ledColor(LED_C_CYAN);
        ledStrip.setPixelColor(3 + step, c);
        ledStrip.setPixelColor(3 - step, c);
      } else if (s_p1 == LED_DIR_BACK) {
        const uint8_t head = (uint8_t)(LED_STRIP_COUNT - 1 - (now / 90) % LED_STRIP_COUNT);
        setDot(head, ledColor(LED_C_GREEN), 3);
      } else {
        const uint8_t head = (uint8_t)((now / 90) % LED_STRIP_COUNT);
        setDot(head, ledColor(LED_C_GREEN), 3);
      }
      break;
    }

    case PAT_PROGRESS: {
      const uint8_t total  = s_p2;
      const uint8_t done   = s_p1;
      const uint8_t filled = total ? (uint8_t)((uint16_t)done * LED_STRIP_COUNT / total) : 0;
      const bool blink = blinkOn(now, 600);
      ledStrip.clear();
      for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
        if (i < filled) {
          ledStrip.setPixelColor(i, s_color);
        } else if (i == filled && done < total) {
          // Der blinkende Punkt zeigt: hier geht es als Naechstes weiter.
          ledStrip.setPixelColor(i, blink ? s_color : scaleColor(s_color, 60, 255));
        }
      }
      break;
    }

    case PAT_MEMORY: {
      // Violettes Lauflicht: der Roboter zeigt gerade die Sequenz.
      ledStrip.clear();
      setDot((uint8_t)((now / 110) % LED_STRIP_COUNT), ledColor(LED_C_VIOLET), 3);
      break;
    }

    case PAT_LINE: {
      ledStrip.clear();
      if (s_p1 == LED_LINE_FOLLOW) {
        // Linie erkannt: gruener Scanner laeuft hin und her.
        const uint8_t last = LED_STRIP_COUNT - 1;
        const uint8_t pos = pingPong(now, 1400, last);
        ledStrip.setPixelColor(pos, ledColor(LED_C_GREEN));
        if (pos > 0)      ledStrip.setPixelColor(pos - 1, ledStrip.Color(0, 90, 0));
        if (pos < last)   ledStrip.setPixelColor(pos + 1, ledStrip.Color(0, 90, 0));
      } else if (s_p1 == LED_LINE_OBSTACLE) {
        // Hindernis: rotes, langsames Blinken (wartet auf freie Bahn).
        fillAll(blinkOn(now, 1000) ? ledColor(LED_C_RED) : 0);
      } else if (s_p1 == LED_LINE_SEARCH) {
        // Linie verloren: oranges Warnblinken, waehrend die Suche dreht.
        fillAll(blinkOn(now, 500) ? ledColor(LED_C_ORANGE) : 0);
      }
      // LED_LINE_IDLE: bleibt dunkel.
      break;
    }

    case PAT_RACE: {
      // Gelbes Lauflicht (schneller als das Fahr-Lauflicht).
      ledStrip.clear();
      setDot((uint8_t)((now / 80) % LED_STRIP_COUNT), ledColor(LED_C_YELLOW), 3);
      break;
    }

    case PAT_COUNTDOWN: {
      ledStrip.clear();
      if (s_p1 == 0) {
        fillAll(ledColor(LED_C_GREEN));   // GO!
      } else {
        const uint32_t red = ledColor(LED_C_RED);
        for (uint8_t i = 0; i < s_p1 && i < LED_STRIP_COUNT; i++) {
          ledStrip.setPixelColor(i, red);
        }
      }
      break;
    }

    case PAT_WIN: {
      // Regenbogen laeuft ueber den Streifen.
      for (uint8_t i = 0; i < LED_STRIP_COUNT; i++) {
        const uint16_t hue = (uint16_t)(now * 24u + (uint16_t)(i * 9362u));  // 9362 ~ 65536/7
        ledStrip.setPixelColor(i, ledStrip.ColorHSV(hue, 255, 200));
      }
      break;
    }

    case PAT_LOSE: {
      // Rotes, langsames Atmen.
      const uint8_t scale = breatheScale(now, 2200, 25, 155);
      fillAll(scaleColor(ledColor(LED_C_RED), scale, 255));
      break;
    }
  }
  ledStrip.show();
}

// =====================================================
// SETUP: Pixel- + Sensor-Selbsttest
// =====================================================

// 1) Pixel-Selbsttest: jeder Punkt leuchtet einmal weiss auf.
static void ledPixelTest() {
  const uint32_t white = ledColor(LED_C_WHITE);
  for (int i = 0; i < LED_STRIP_COUNT; i++) {
    ledStrip.clear();
    ledStrip.setPixelColor(i, white);
    ledStrip.show();
    delay(70);
  }
  ledStrip.clear();
  ledStrip.show();
}

// 2) Sensor-Selbsttest: fehlender Sensor wird direkt am Streifen gemeldet
// (~2,5 s: linker Teil magenta = Farbsensor fehlt, rechter Teil rot =
// Distanzsensor fehlt, synchron blinkend).
static void ledSensorReport(bool colorOk, bool distOk) {
  Serial.printf("[LED] Selbsttest: Farbsensor=%s, Distanzsensor=%s\n",
                colorOk ? "OK" : "FEHLT", distOk ? "OK" : "FEHLT");
  if (colorOk && distOk) {
    // Alles bereit: zwei gruene Blitze.
    const uint32_t green = ledColor(LED_C_GREEN);
    for (int k = 0; k < 2; k++) {
      fillAll(green);
      ledStrip.show();
      delay(150);
      fillAll(0);
      ledStrip.show();
      delay(120);
    }
    return;
  }
  const uint32_t start = millis();
  while (millis() - start < 2500) {
    const bool on = blinkOn(millis(), 500);
    ledStrip.clear();
    if (on) {
      if (!colorOk) {
        for (int i = 0; i < 3; i++) ledStrip.setPixelColor(i, ledStrip.Color(255, 0, 200));
      }
      if (!distOk) {
        for (int i = 4; i < 7; i++) ledStrip.setPixelColor(i, ledColor(LED_C_RED));
      }
    }
    ledStrip.show();
    delay(50);
  }
}

void ledStripSetup() {
  ledStrip.begin();
  ledStrip.setBrightness(LED_BRIGHTNESS);
  ledStrip.clear();
  ledStrip.show();
  Serial.println("[LED] WS2812 auf GPIO 8 initialisiert (7 LEDs)");

  ledPixelTest();
  ledSensorReport(colorSensorOk, distOk());

  ledStrip.clear();
  ledStrip.show();
  s_pattern = PAT_OFF;
  s_offShown = true;
  Serial.println("[LED] Boot-Pattern fertig");
}
