#pragma once

#include <Arduino.h>
#include <Preferences.h>

struct ColorRef {
  uint8_t r = 0, g = 0, b = 0;
  bool    calibrated = false;
};

constexpr int8_t LEFT_MOTOR_SIGN  = +1;
constexpr int8_t RIGHT_MOTOR_SIGN = -1;

static_assert(LEFT_MOTOR_SIGN != RIGHT_MOTOR_SIGN,
              "Beide Motoren mit gleichem Vorzeichen: der Roboter wuerde sich "
              "nur im Kreis drehen. Siehe LEFT_MOTOR_SIGN / RIGHT_MOTOR_SIGN.");
static_assert(LEFT_MOTOR_SIGN == 1 || LEFT_MOTOR_SIGN == -1,
              "LEFT_MOTOR_SIGN muss +1 oder -1 sein.");
static_assert(RIGHT_MOTOR_SIGN == 1 || RIGHT_MOTOR_SIGN == -1,
              "RIGHT_MOTOR_SIGN muss +1 oder -1 sein.");

constexpr uint16_t MOTOR_FULL_STEPS_PER_REV = 200;


constexpr float WHEEL_DIAMETER_MM = 38.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265358979f * WHEEL_DIAMETER_MM;
constexpr float DRIVE_TRIM = 1.0101f;
constexpr float EFFECTIVE_CIRCUMFERENCE_MM = WHEEL_CIRCUMFERENCE_MM * DRIVE_TRIM;
constexpr float MM_PER_FULL_STEP = EFFECTIVE_CIRCUMFERENCE_MM / (float)MOTOR_FULL_STEPS_PER_REV;

constexpr int32_t QUIZ_STEP_FULL_STEPS = MOTOR_FULL_STEPS_PER_REV / 4;
constexpr float   QUIZ_STEP_MM  = (float)QUIZ_STEP_FULL_STEPS * MM_PER_FULL_STEP;
constexpr float   QUIZ_TURN_DEG = 90.0f;
constexpr float   TURN_DEG_DEFAULT = 90.0f;
constexpr float FORWARD_MM_MIN = 5.0f,  FORWARD_MM_MAX = 150.0f;
constexpr float TURN_DEG_MIN   = 10.0f,  TURN_DEG_MAX   = 360.0f;
constexpr float SPEED_MM_S_MIN = 20.0f,  SPEED_MM_S_MAX = 600.0f;

constexpr uint16_t RAMP_MS_MIN = 0, RAMP_MS_MAX = 2000, RAMP_MS_DEFAULT = 150;

constexpr uint16_t SPIN_SPEED_MIN = 0, SPIN_SPEED_MAX = 20000, SPIN_SPEED_DEFAULT = 6000;
constexpr uint16_t SPIN_MS_MIN = 0, SPIN_MS_MAX = 3000, SPIN_MS_DEFAULT = 0;

constexpr bool inRange(float x, float lo, float hi) {
  return (x >= lo) && (x <= hi);
}
constexpr float MAX_DRIVE_STEPS_S = 20000.0f;

constexpr uint16_t MICROSTEPS_MAX = 256;

constexpr int32_t geoStepsForFullSteps(int32_t fullSteps, uint16_t microsteps) {
  return fullSteps * (int32_t)microsteps;
}
constexpr int32_t geoStepsForMM(float mm, uint16_t microsteps) {
  return (int32_t)((mm < 0.0f ? -mm : mm)
                   * ((float)MOTOR_FULL_STEPS_PER_REV * (float)microsteps / EFFECTIVE_CIRCUMFERENCE_MM)
                   + 0.5f);
}
constexpr int32_t geoStepsForDeg(float deg, uint16_t microsteps) {
  return (int32_t)((deg < 0.0f ? -deg : deg) / 360.0f
                   * ((float)MOTOR_FULL_STEPS_PER_REV * (float)microsteps)
                   + 0.5f);
}

static_assert(MM_PER_FULL_STEP > 0.602f && MM_PER_FULL_STEP < 0.604f,
              "Ein Vollschritt muss 0,6029 mm ergeben (Rad 38mm / 200 Schritte, inkl. DRIVE_TRIM).");
static_assert(geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 16) == 800
           && geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 128) == 6400,
              "50 Vollschritte muessen 16 Microsteps -> 800 und 128 -> 6400 ergeben.");
static_assert(geoStepsForDeg(QUIZ_TURN_DEG, 16) == 800
           && geoStepsForDeg(QUIZ_TURN_DEG, 128) == 6400,
              "90 Grad muessen 16 Microsteps -> 800 und 128 -> 6400 ergeben.");
static_assert(geoStepsForFullSteps(QUIZ_STEP_FULL_STEPS, 16)
           == geoStepsForDeg(QUIZ_TURN_DEG, 16),
              "Ein Rasterfeld (Vorwaerts) muss genau einer 90-Grad-Drehung entsprechen.");

constexpr uint32_t GEO_DEF_VERSION = 4;
constexpr uint32_t geoSignature() {
  return ((uint32_t)GEO_DEF_VERSION << 28)
       ^ ((uint32_t)MOTOR_FULL_STEPS_PER_REV * 1000000u)
       ^ ((uint32_t)WHEEL_DIAMETER_MM * 1000u);
}

struct Config {
  int8_t leftSign     = LEFT_MOTOR_SIGN;
  int8_t rightSign    = RIGHT_MOTOR_SIGN;

  uint32_t commandTimeoutMs = 500;
  uint16_t tmcMicrosteps = 64;
  bool     tmcSpread = true;

  float quizForwardMM  = QUIZ_STEP_MM;
  float quizTurnDeg    = TURN_DEG_DEFAULT;
  float driveSpeedMM   = 150.0f;
  uint16_t rampMs      = RAMP_MS_DEFAULT;
  uint16_t spinSpeed   = SPIN_SPEED_DEFAULT;
  uint16_t spinMs      = SPIN_MS_DEFAULT;
  uint16_t blackR = 152, blackG = 137, blackB = 98;
  uint16_t whiteR = 4143, whiteG = 4258, whiteB = 3036;
  float minColorSaturation = 0.40f;
  int16_t obsDistMM = 100;
  int16_t obsRelMM  = 130;
  int16_t obsOffsetMM = 0;
  float   obsScale = 1.0f;
  ColorRef calRot;
  ColorRef calGelb;
  ColorRef calGruen;
  ColorRef calBlau;
  bool edgeGuard = true;
  uint8_t floorR = 0, floorG = 0, floorB = 0;
  uint16_t floorC = 0;
  bool floorCalibrated = false;
};

extern Config config;
extern Preferences prefs;

int32_t configForwardSteps();
int32_t configTurnSteps();
float   configStepsPerMM();
float   configStepsPerS();
uint16_t configOptimalMicrosteps(float driveSpeedMM);

void configLoad();
void configValidate();
void configResetGeometry();
void configSaveField(const char* key, float value);
void configSaveInt(const char* key, int32_t value);
void configSaveUShort(const char* key, uint16_t value);
void configSaveUChar(const char* key, uint8_t value);
void configSaveBool(const char* key, bool value);
