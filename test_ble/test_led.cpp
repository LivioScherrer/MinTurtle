// =====================================================
// EduBot WS2812 LED TEST - NUR LED-STREIFEN
// =====================================================
// Minimal-Firmware: Nur der WS2812-Streifen (7 LEDs an GPIO 44).
// Kein Farbsensor, keine Motoren, kein BLE.
// =====================================================

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   44
#define LED_COUNT 7

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[LED] Test gestartet - WS2812 an GPIO 44");

  strip.begin();
  strip.setBrightness(50);
  strip.show();
}

void loop() {
  // 1) Alle 7 LEDs nacheinander rot aufleuchten
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
    strip.show();
    Serial.printf("[LED] LED %d ROT\n", i);
    delay(300);
  }
  strip.clear();
  strip.show();
  delay(500);

  // 2) Alle 7 LEDs nacheinander gruen aufleuchten
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
    Serial.printf("[LED] LED %d GRUEN\n", i);
    delay(300);
  }
  strip.clear();
  strip.show();
  delay(500);

  // 3) Alle LEDs blau
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255));
  }
  strip.show();
  Serial.println("[LED] ALLE BLAU");
  delay(1000);

  // 4) Alle LEDs weiss (Test, ob RGB-Weiss richtig drueberkommt)
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }
  strip.show();
  Serial.println("[LED] ALLE WEISS");
  delay(1000);

  strip.clear();
  strip.show();
  Serial.println("[LED] Test-Durchlauf fertig, neustart...");
  delay(1000);
}
