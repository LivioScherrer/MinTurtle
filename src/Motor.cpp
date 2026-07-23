// EduBot V1.3 - Motor-Implementierung
#include "Motor.h"
#include "Config.h"

portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t stepFracAddL = 0;
uint32_t stepFracAddR = 0;
uint32_t stepFracAccumL = 0;
uint32_t stepFracAccumR = 0;
volatile uint32_t stepIntervalL = 0;
volatile uint32_t stepIntervalR = 0;

static uint8_t stepPhaseL = 0;
static uint8_t stepPhaseR = 0;

bool IRAM_ATTR stepperTimerISR(void *arg) {
  if (stepPhaseL == 1) { GPIO.out_w1tc = STEP_BIT_L; stepPhaseL = 0; }
  if (stepPhaseR == 1) { GPIO.out_w1tc = STEP_BIT_R; stepPhaseR = 0; }

  if (stepIntervalL > 0) {
    stepFracAccumL += stepFracAddL;
    if (stepFracAccumL >= 0x10000UL) {
      stepFracAccumL -= 0x10000UL;
      GPIO.out_w1ts = STEP_BIT_L;
      stepPhaseL = 1;
    }
  } else {
    stepFracAccumL = 0;
  }
  if (stepIntervalR > 0) {
    stepFracAccumR += stepFracAddR;
    if (stepFracAccumR >= 0x10000UL) {
      stepFracAccumR -= 0x10000UL;
      GPIO.out_w1ts = STEP_BIT_R;
      stepPhaseR = 1;
    }
  } else {
    stepFracAccumR = 0;
  }
  return false;
}

void initStepperTimer() {
  timer_config_t cfg = {};
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.counter_en = TIMER_PAUSE;
  cfg.intr_type = TIMER_INTR_LEVEL;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.divider = 80;

  timer_init(TIMER_GROUP_0, TIMER_0, &cfg);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, STEP_TIMER_US);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, stepperTimerISR, NULL, ESP_INTR_FLAG_IRAM);
  timer_start(TIMER_GROUP_0, TIMER_0);
}

void IRAM_ATTR setMotorSpeed(int motorIdx, float speed) {
  portENTER_CRITICAL(&stepperMux);
  const int dirPin = (motorIdx == 0) ? PIN_DIR_L : PIN_DIR_R;
  const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;

  float actualSpeed = speed * static_cast<float>(sign);

  if (actualSpeed > 0.5f) {
    digitalWrite(dirPin, HIGH);
  } else if (actualSpeed < -0.5f) {
    digitalWrite(dirPin, LOW);
    actualSpeed = -actualSpeed;
  } else {
    if (motorIdx == 0) { stepIntervalL = 0; stepFracAddL = 0; stepFracAccumL = 0; }
    else { stepIntervalR = 0; stepFracAddR = 0; stepFracAccumR = 0; }
    GPIO.out_w1tc = (motorIdx == 0) ? STEP_BIT_L : STEP_BIT_R;
    portEXIT_CRITICAL(&stepperMux);
    return;
  }

  uint32_t speedInt = static_cast<uint32_t>(actualSpeed);
  if (speedInt < 1) speedInt = 1;

  uint32_t interval = STEP_PERIOD_US / speedInt;
  if (interval < MIN_STEP_INTERVAL) interval = MIN_STEP_INTERVAL;
  if (interval > MAX_STEP_INTERVAL) interval = MAX_STEP_INTERVAL;

  uint32_t fracAdd = (STEP_TIMER_US * 0x10000UL) / interval;

  if (motorIdx == 0) {
    stepIntervalL = interval;
    stepFracAddL = fracAdd;
    stepFracAccumL = 0;
  } else {
    stepIntervalR = interval;
    stepFracAddR = fracAdd;
    stepFracAccumR = 0;
  }
  portEXIT_CRITICAL(&stepperMux);
}

void IRAM_ATTR stopMotors() {
  portENTER_CRITICAL(&stepperMux);
  stepIntervalL = 0;
  stepIntervalR = 0;
  stepFracAddL = 0;
  stepFracAddR = 0;
  stepFracAccumL = 0;
  stepFracAccumR = 0;
  GPIO.out_w1tc = STEP_BIT_L | STEP_BIT_R;
  portEXIT_CRITICAL(&stepperMux);
}

float rampSpeed(float current, float target, float dt) {
  float diff = target - current;
  if (diff < 1.0f && diff > -1.0f) {
    return target;
  }
  float maxChange = config.accelRate * dt;
  if (diff > 0) {
    current += maxChange;
    if (current > target) current = target;
  } else {
    current -= maxChange;
    if (current < target) current = target;
  }
  return current;
}
