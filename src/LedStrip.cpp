// EduBot V2 - LED-Streifen-Modul (WS2812, 7 LEDs an GPIO 8, siehe LedStrip.h)
// PCB Edu_V2.2: Net D9 = GPIO8.
// Boot-Pattern, Status-LEDs und Zielfarben-Anzeige
#include "LedStrip.h"

Adafruit_NeoPixel ledStrip(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

volatile bool ledAlertActive = false;
static unsigned long ledAlertStart = 0;
static const unsigned long LED_ALERT_MS = 3000;   // Alert maximal 3s blinken
static const unsigned long LED_ALERT_PERIOD_MS = 250;  // Blinkfrequenz

// Name (wie vom Farbsensor / Slalom) -> RGB-Farbe
static uint32_t colorByName(const char* name) {
  if (!name) return ledStrip.Color(0, 0, 0);
  if (strcasecmp(name, "ROT") == 0)   return ledStrip.Color(255, 0, 0);
  if (strcasecmp(name, "GELB") == 0)  return ledStrip.Color(255, 150, 0);
  if (strcasecmp(name, "GRUEN") == 0) return ledStrip.Color(0, 255, 0);
  if (strcasecmp(name, "BLAU") == 0)  return ledStrip.Color(0, 0, 255);
  if (strcasecmp(name, "WEISS") == 0) return ledStrip.Color(255, 255, 255);
  if (strcasecmp(name, "SCHWARZ") == 0) return ledStrip.Color(0, 0, 0);
  return ledStrip.Color(80, 0, 80);  // UNBEKANNT -> violett
}

void ledStripSetup() {
  ledStrip.begin();
  ledStrip.setBrightness(40);
  ledStrip.show();
  Serial.println("[LED] WS2812 auf GPIO 8 initialisiert (7 LEDs)");

  // Boot-Pattern: Regenbogen-Durchlauf
  const uint8_t bootColors[7][3] = {
    {255, 0, 0}, {255, 150, 0}, {255, 255, 0}, {0, 255, 0},
    {0, 0, 255}, {150, 0, 255}, {255, 255, 255}
  };
  for (int i = 0; i < LED_STRIP_COUNT; i++) {
    ledStrip.setPixelColor(i, bootColors[i][0], bootColors[i][1], bootColors[i][2]);
    ledStrip.show();
    delay(60);
  }
  delay(300);
  ledStripClear();
  Serial.println("[LED] Boot-Pattern fertig");
}

void ledStripTick() {
  if (!ledAlertActive) return;
  if (millis() - ledAlertStart >= LED_ALERT_MS) {
    ledAlertActive = false;
    ledStripClear();
    return;
  }
  bool on = (millis() / LED_ALERT_PERIOD_MS) % 2 == 0;
  for (int i = 0; i < LED_STRIP_COUNT; i++) {
    ledStrip.setPixelColor(i, on ? ledStrip.Color(255, 0, 0) : ledStrip.Color(0, 0, 0));
  }
  ledStrip.show();
}

void ledStripShowTarget(const char* colorName) {
  uint32_t c = colorByName(colorName);
  for (int i = 0; i < LED_STRIP_COUNT; i++) {
    ledStrip.setPixelColor(i, c);
  }
  ledStrip.show();
}

void ledStripShowStatus(bool bleConnected) {
  for (int i = 0; i < LED_STRIP_COUNT; i++) {
    ledStrip.setPixelColor(i, bleConnected ? ledStrip.Color(0, 0, 255)
                                           : ledStrip.Color(0, 255, 0));
  }
  ledStrip.show();
}

void ledStripShowAlert() {
  ledAlertActive = true;
  ledAlertStart = millis();
}

void ledStripClear() {
  ledStrip.clear();
  ledStrip.show();
}
