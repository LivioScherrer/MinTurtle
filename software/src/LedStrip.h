#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_STRIP_PIN   8
#define LED_STRIP_COUNT 7

extern Adafruit_NeoPixel ledStrip;
extern volatile bool ledAlertActive;

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
uint32_t ledColorByName(const char* name);

enum LedDriveDir : uint8_t {
  LED_DIR_IDLE = 0,
  LED_DIR_FWD,
  LED_DIR_BACK
};

void ledStripSetup();
void ledStripTick();
bool ledIsOverrideActive();

void ledStripShowSolid(LedColorId color);
void ledStripShowBlink(LedColorId color, uint16_t periodMs = 500);
void ledStripShowBreathe(LedColorId color, uint16_t periodMs = 2000);

void ledStripShowTarget(const char* colorName);
void ledStripShowTarget(LedColorId color);
void ledStripShowStatus(bool bleConnected);
void ledStripShowPaired();
void ledStripShowDrive(LedDriveDir dir);
void ledStripShowProgress(uint8_t done, uint8_t total, LedColorId color);
void ledStripShowMemoryShow();
inline void ledStripShowMemory() { ledStripShowMemoryShow(); }
void ledStripShowRace();
void ledStripShowCountdown(uint32_t msLeft, uint32_t totalMs);
void ledStripShowResult(bool iWon);
void ledStripShowResultFlash(bool ok);

void ledStripShowAlert();
void ledStripClear();
