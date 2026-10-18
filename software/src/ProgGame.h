#pragma once

#include <Arduino.h>

constexpr uint8_t PROG_MAX_STEPS      = 40;
constexpr uint8_t PROG_MAX_DEPTH      = 3;
constexpr uint8_t PROG_UNTIL_MAX_ITER = 60;
constexpr uint8_t PROG_MAX_ENC        = 200;

enum ProgOp : uint8_t {
  OP_FWD = 0,
  OP_BACK,
  OP_LEFT,
  OP_RIGHT,
  OP_WAIT,
  OP_REPEAT,
  OP_UNTIL,
  OP_MM_FWD,
  OP_MM_BACK,
  OP_DRIVE_DIST,
  OP_WAIT_DIST,
  OP_WAIT_FREE,
  OP_DRIVE_UNTIL,
  OP_WAIT_UNTIL,
  OP_LED,
  OP_PAUSE,
  OP_STOP,
  OP_SPEED,
  OP_END
};

constexpr uint8_t PROG_COLOR_ROT   = 0;
constexpr uint8_t PROG_COLOR_GELB  = 1;
constexpr uint8_t PROG_COLOR_GRUEN = 2;
constexpr uint8_t PROG_COLOR_BLAU  = 3;
constexpr uint8_t PROG_SENSOR_IR_HIGH = 4;
constexpr uint8_t PROG_SENSOR_IR_LOW  = 5;
constexpr uint8_t PROG_SLOT_OFF       = 6;

constexpr uint32_t PROG_DRIVE_UNTIL_MAX_MS = 20000UL;
constexpr int32_t  PROG_DRIVE_UNTIL_MAX_MM = 1500;
constexpr uint32_t PROG_DRIVE_UNTIL_HOLD_MS = 60;

struct ProgStep {
  uint8_t op;
  int16_t val;
  int8_t  color;
  uint8_t endIdx;
  uint8_t sibIdx;
};

extern ProgStep progSteps[PROG_MAX_STEPS];
extern uint8_t  progLen;
extern bool     progActive;

bool progSetFromString(const char* s);

void progRun();

void progRunSlalom(const char* targetColor);

void progStop();

void edgeGuardTick(uint16_t rawClear);
void edgeGuardSet(bool on);
bool edgeGuardActive();
bool edgeGuardBlocked();
void edgeGuardReset();
void floorLearn(uint8_t r, uint8_t g, uint8_t b, uint16_t c);
uint16_t edgeGuardMaxNow();
uint16_t edgeGuardReleaseNow();
