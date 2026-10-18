#include "Motor.h"
#include "Config.h"
#include "BLEManager.h"
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

namespace {

  AccelStepper stepperL(AccelStepper::DRIVER, PIN_STEP_L, PIN_DIR_L);
  AccelStepper stepperR(AccelStepper::DRIVER, PIN_STEP_R, PIN_DIR_R);

  SemaphoreHandle_t motorMutex = nullptr;

  bool motorReady = false;

  TaskHandle_t motorTaskHandle = nullptr;
  esp_timer_handle_t pulseTimer = nullptr;

  volatile bool posActiveL = false;
  volatile bool posActiveR = false;
  volatile long remainingL = 0;
  volatile long remainingR = 0;

  volatile bool s_rampOn = true;
  volatile float s_accel = 0.0f;

  volatile float s_curL = 0.0f, s_tgtL = 0.0f;
  volatile float s_curR = 0.0f, s_tgtR = 0.0f;

  void stepperHalt(AccelStepper& s) {
    s.setCurrentPosition(s.currentPosition());
  }

  void rampStep(AccelStepper& s, volatile float& cur, float tgt, float maxDelta) {
    if (cur == tgt) return;
    if (!(maxDelta > 0.0f)) {
      cur = tgt; s.setSpeed(tgt); return;
    }
    const float diff = tgt - cur;
    if (diff > maxDelta)       cur += maxDelta;
    else if (diff < -maxDelta) cur -= maxDelta;
    else                       cur = tgt;
    s.setSpeed(cur);
  }

static volatile bool s_emergency = false;

  void motorTaskLoop(void* param) {
    int64_t lastUs = esp_timer_get_time();
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        const bool ramp = s_rampOn;
        const int64_t nowUs = esp_timer_get_time();
        float dt = (float)(nowUs - lastUs) / 1000000.0f;
        lastUs = nowUs;
        if (!(dt > 0.0f))      dt = 0.0f;
        else if (dt > 0.05f)   dt = 0.05f;
        const float maxDelta = s_accel * dt;
        if (posActiveL) {
          const long beforeL = stepperL.currentPosition();
          if (ramp) stepperL.run(); else stepperL.runSpeed();
          if (stepperL.currentPosition() != beforeL) {
            if (--remainingL <= 0) {
              remainingL = 0;
              posActiveL = false;
              stepperL.setSpeed(0.0f);
            }
          }
        } else {
          rampStep(stepperL, s_curL, s_tgtL, maxDelta);
          stepperL.runSpeed();
        }
        if (posActiveR) {
          const long beforeR = stepperR.currentPosition();
          if (ramp) stepperR.run(); else stepperR.runSpeed();
          if (stepperR.currentPosition() != beforeR) {
            if (--remainingR <= 0) {
              remainingR = 0;
              posActiveR = false;
              stepperR.setSpeed(0.0f);
            }
          }
        } else {
          rampStep(stepperR, s_curR, s_tgtR, maxDelta);
          stepperR.runSpeed();
        }
        xSemaphoreGive(motorMutex);
      }
    }
  }

  void pulseTimerCb(void* arg) {
    if (motorTaskHandle != nullptr) {
      xTaskNotifyGive(motorTaskHandle);
    }
  }


  class TMC2209Diag : public TMC2209Stepper {
   public:
    TMC2209Diag(Stream* serial, float rs, uint8_t addr)
      : TMC2209Stepper(serial, rs, addr) {}
    using TMC2208Stepper::read;
    using TMC2208Stepper::write;
  };

  TMC2209Diag tmcL(&Serial1, TMC_R_SENSE, TMC_ADDR_L);
  TMC2209Diag tmcR(&Serial1, TMC_R_SENSE, TMC_ADDR_R);

  uint16_t s_microsteps = TMC_MICROSTEPS_DEFAULT;
uint8_t  s_toff   = 5;
uint8_t  s_tpower = 20;
bool     motorDiagQuiet = false;
bool     s_initPhase = false;
bool s_spread = true;
bool s_enabled = false;
volatile unsigned long s_lastMotionMs = 0;
  bool s_msConfirmed = false;
  uint32_t s_lastCfgRetryMs = 0;
  constexpr uint32_t TMC_EN_SETTLE_MS = 10;

  bool validMicrosteps(uint16_t v) {
    return v == 1 || v == 2 || v == 4 || v == 8 || v == 16 || v == 32 ||
           v == 64 || v == 128 || v == 256;
  }

  uint8_t mresCodeFor(uint16_t ms) {
    switch (ms) {
      case 256: return 0;
      case 128: return 1;
      case 64:  return 2;
      case 32:  return 3;
      case 16:  return 4;
      case 8:   return 5;
      case 4:   return 6;
      case 2:   return 7;
      case 1:   return 8;
      default:  return 4;
    }
  }

  bool applyMicrosteps(TMC2209Diag& d, uint16_t ms) {
    const uint8_t want = mresCodeFor(ms);
    for (uint8_t i = 0; i < 3; i++) {
      uint32_t chop = (((uint32_t)want << CHOP_MRES_SHIFT) & CHOP_MRES_MASK)
        | CHOP_INTPOL
        | ((uint32_t)(s_toff & 7) << CHOP_TOFF_SHIFT)
        | ((uint32_t)(CHOP_HSTRT_DEFAULT & 7) << CHOP_HSTRT_SHIFT)
        | ((uint32_t)(CHOP_HEND_DEFAULT & 15) << CHOP_HEND_SHIFT)
        | ((uint32_t)(CHOP_TBL_DEFAULT & 3) << CHOP_TBL_SHIFT);
      d.write(TMC_REG_CHOPCONF, chop);
      const uint32_t back = d.read(TMC_REG_CHOPCONF);
      if (((back >> CHOP_MRES_SHIFT) & 0x0Fu) == want) return true;
      delay(2);
    }
    return false;
  }

  uint32_t tmcPeriodForMmS(float v_mm_s) {
    if (!(v_mm_s > 1.0f)) return 0xFFFFF;
    const float p = 16777216.0f * EFFECTIVE_CIRCUMFERENCE_MM / (51200.0f * v_mm_s);
    return (p >= 1048575.0f) ? 0xFFFFF : (uint32_t)p;
  }

static void tmcWriteChecked(TMC2209Diag& d, uint8_t reg, uint32_t val,
                            uint32_t mask, const char* name) {
  d.write(reg, val);
  const uint32_t back = d.read(reg);
  if ((back & mask) == (val & mask)) {
    Serial.printf("[TMC] %-9s 0x%02X = 0x%08lX  per Readback bestaetigt\n",
                  name, reg, (unsigned long)(back & mask));
  } else {
    Serial.printf("[TMC] %-9s 0x%02X gesetzt 0x%08lX (nur schreibbar, Readback ohne Aussage)\n",
                  name, reg, (unsigned long)(val & mask));
  }
}

static uint8_t irunForMilliAmp(uint16_t mA) {
  const float vfs = 0.325f;
  float cs = (float)mA / 1000.0f * 32.0f * (TMC_R_SENSE + 0.02f) * 1.41421356f / vfs - 1.0f;
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  return (uint8_t)(cs < 16.0f ? 16.0f : cs);
}

static void configureOne(TMC2209Diag& d, bool& msOkOut);

static float milliAmpForIrun(uint8_t irun) {
  const float vfs = 0.325f;
  return (float)((int)irun + 1) / 32.0f * vfs / (TMC_R_SENSE + 0.02f) / 1.41421356f * 1000.0f;
}

static uint32_t iholdIrunReg(uint8_t irun, uint8_t ihold) {
  uint32_t v = ((uint32_t)irun << 8) & 0x0001F000u;
  v |= ((uint32_t)ihold & 0x1Fu);
  v |= ((uint32_t)TMC_IHOLDDELAY << 16) & 0x000F0000u;
  return v;
}

void configureOne(TMC2209Diag& d, bool& msOkOut) {
  d.begin();
  const uint8_t ifBefore = (uint8_t)d.read(TMC_REG_IFCNT);
  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT
                 | 0x01u;
  if (s_spread) gconf |= GCONF_EN_SPREADCYCLE;
  tmcWriteChecked(d, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");

  const uint32_t mresField = ((uint32_t)mresCodeFor(s_microsteps) << CHOP_MRES_SHIFT) & CHOP_MRES_MASK;
  uint32_t chop = mresField | CHOP_INTPOL
    | ((uint32_t)(s_toff & 7) << CHOP_TOFF_SHIFT)
    | ((uint32_t)(CHOP_HSTRT_DEFAULT & 7) << CHOP_HSTRT_SHIFT)
    | ((uint32_t)(CHOP_HEND_DEFAULT & 15) << CHOP_HEND_SHIFT)
    | ((uint32_t)(CHOP_TBL_DEFAULT & 3) << CHOP_TBL_SHIFT);
  tmcWriteChecked(d, TMC_REG_CHOPCONF, chop,
                  CHOP_MRES_MASK | CHOP_INTPOL | CHOP_TOFF_MASK |
                  CHOP_HSTRT_MASK | CHOP_HEND_MASK | CHOP_TBL_MASK, "CHOPCONF");
  tmcWriteChecked(d, TMC_REG_CHOPCONF, chop,
                  CHOP_MRES_MASK | CHOP_INTPOL | CHOP_TOFF_MASK |
                  CHOP_HSTRT_MASK | CHOP_HEND_MASK | CHOP_TBL_MASK, "CHOPCONF");

  const uint8_t irun  = irunForMilliAmp(TMC_CURRENT_MA_FIXED);
  const uint8_t ihold = (uint8_t)(irun / 2u);
  const uint32_t iholdIrun = iholdIrunReg(irun, ihold);
  tmcWriteChecked(d, TMC_REG_IHOLD_IRUN, iholdIrun, 0x001FFFFFu, "IHOLD/IRUN");
  if (motorDiagQuiet) {
    Serial.printf("[TMC] Strom fest: IRUN=%u -> %.0f mA RMS (Soll %u mA), "
                  "IHOLD=%u -> %.0f mA, moeglich bis %.0f mA\n",
                  irun, (double)milliAmpForIrun(irun), (unsigned)TMC_CURRENT_MA_FIXED,
                  ihold, (double)milliAmpForIrun(ihold),
                  (double)milliAmpForIrun(31));
  }


  tmcWriteChecked(d, TMC_REG_TPOWER, s_tpower, 0xFFu, "TPOWER");

  tmcWriteChecked(d, TMC_REG_TPWMTHRS, 0u, 0x000FFFFFu, "TPWMTHRS");

  tmcWriteChecked(d, TMC_REG_TCOOLTHRS, tmcPeriodForMmS(TMC_STALLGUARD_AB_MM_S),
                  0x000FFFFFu, "TCOOLTHRS");
  tmcWriteChecked(d, TMC_REG_SGTHRS, TMC_SGTHRS_DEFAULT, 0xFFu, "SGTHRS");

  tmcWriteChecked(d, TMC_REG_NODECONF, 0x0Fu, 0x0Fu, "NODECONF");

  tmcWriteChecked(d, TMC_REG_PWMCONF, PWMCONF_VALUE, 0xFFFFFFFFu, "PWMCONF");

  const uint8_t ifAfter = (uint8_t)d.read(TMC_REG_IFCNT);
  const uint8_t sent   = (uint8_t)(ifAfter - ifBefore);
  msOkOut = (sent >= TMC_CFG_WRITE_COUNT);
  if (motorDiagQuiet) {
    Serial.printf("[TMC] Konfiguration: %u Schreibzugriffe angenommen (erwartet %u) - %s\n",
                  sent, (unsigned)TMC_CFG_WRITE_COUNT,
                  msOkOut ? "ALLE REGISTER GESETZT" : "NICHT VOLLSTAENDIG");
  }

}

static bool tmcConfigureBothDrivers() {
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  return s_msConfirmed;
}

  struct TmcMeas {
    uint8_t  ifcntA[2];
    uint8_t  ifcntB[2];
    uint16_t sg[2];
    int32_t  vactual[2];
    uint32_t tstep[2];
    uint16_t mscnt[2];
    uint8_t  gstat[2];
    uint32_t drv[2];
  };

  TmcMeas s_meas;
  uint16_t s_mscntPrev[2] = {0, 0};
  int32_t  s_mscntAcc[2]  = {0, 0};
  uint8_t  s_diagStep = 0;
  bool     s_diagAuto = false;
  bool     s_diagWindow = false;
  uint16_t s_diagFlags = 0;
  bool     s_diagArmed = false;
  bool     s_mscntPrimed = false;
  uint32_t s_steadyLastMs = 0;
  uint32_t s_steadyBusyMs  = 0;

  enum DiagFlag : uint16_t {
    DIAG_UV_CP        = 1u << 0,
    DIAG_DRV_ERR      = 1u << 1,
    DIAG_MODE_DIFF    = 1u << 2,
    DIAG_OPEN_LOAD    = 1u << 3,
    DIAG_SHORT        = 1u << 4,
    DIAG_OVERTEMP     = 1u << 5,
    DIAG_CURRENT_DIFF = 1u << 6,
    DIAG_TSTEP_DIFF   = 1u << 7,
    DIAG_TSTEP_SAT    = 1u << 8,
    DIAG_MSCNT_DIFF   = 1u << 9,
    DIAG_SG_DIFF      = 1u << 10,
    DIAG_VACTUAL_DIFF = 1u << 11,
    DIAG_READ_ERROR   = 1u << 12,
  };

  void motorDiagPrimeMscnt() {
    for (uint8_t i = 0; i < 2; i++) {
      TMC2209Diag& d = (i == 0) ? tmcL : tmcR;
      s_mscntPrev[i] = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x03FF);
    }
    s_mscntAcc[0] = 0;
    s_mscntAcc[1] = 0;
    s_mscntPrimed = true;
  }

  int16_t mscntDelta(uint16_t now, uint16_t prev) {
    int16_t d = (int16_t)((now - prev) & 0x03FF);
    return (d > 511) ? (int16_t)(d - 1024) : d;
  }
}

float FIXED_DRIVE_SPEED = 0.0f;

static float motorAccelStepsS2() {
  if (!(FIXED_DRIVE_SPEED > 0.0f)) return 0.0f;
  const uint16_t ms = config.rampMs;
  if (ms == 0) return 0.0f;
  return FIXED_DRIVE_SPEED * 1000.0f / (float)ms;
}

static void motorUpdateSpeedsLocked() {
  FIXED_DRIVE_SPEED = configStepsPerS();
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  s_rampOn = (config.rampMs > 0);
  s_accel = motorAccelStepsS2();
  stepperL.setAcceleration(s_accel);
  stepperR.setAcceleration(s_accel);
  if (s_curL != 0.0f) stepperL.setSpeed(s_curL);
  if (s_curR != 0.0f) stepperR.setSpeed(s_curR);
  const float wanted = config.driveSpeedMM * configStepsPerMM();
  Serial.printf("[MOTOR] Tempo: %.0f mm/s -> %.0f Schritte/s (%u Schritte/Kreis, %u Microsteps)%s\n",
                (double)config.driveSpeedMM, (double)FIXED_DRIVE_SPEED,
                (unsigned)(MOTOR_FULL_STEPS_PER_REV * s_microsteps), s_microsteps,
                wanted > MAX_DRIVE_STEPS_S ? " - am 50us-Puls-Task begrenzt" : "");
  if (s_rampOn) {
    Serial.printf("[MOTOR] Anfahren: %u ms (0 -> %.0f Schritte/s = %.0f Schritte/s^2)\n",
                  (unsigned)config.rampMs, (double)FIXED_DRIVE_SPEED, (double)s_accel);
  } else {
    Serial.println("[MOTOR] Anfahren: aus (sofort volle Geschwindigkeit)");
  }
}

static void takeMotorMutex();

void motorUpdateSpeeds() {
  if (!motorMutex) { motorUpdateSpeedsLocked(); return; }
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
  motorUpdateSpeedsLocked();
  xSemaphoreGive(motorMutex);
}

bool motorRampActive() { return s_rampOn; }

void motorRampSpeeds(float& outL, float& outR) {
  outL = s_curL;
  outR = s_curR;
  if (!motorMutex) return;
  takeMotorMutex();
  outL = s_curL;
  outR = s_curR;
  xSemaphoreGive(motorMutex);
}

void motorStepPositions(long& outL, long& outR) {
  outL = 0;
  outR = 0;
  if (!motorMutex) return;
  takeMotorMutex();
  outL = stepperL.currentPosition();
  outR = stepperR.currentPosition();
  xSemaphoreGive(motorMutex);
}

void initSteppers() {
  s_microsteps = config.tmcMicrosteps;
  s_spread = config.tmcSpread;
  motorUpdateSpeeds();
  stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
  stepperL.setSpeed(0.0f);
  stepperR.setSpeed(0.0f);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);
  s_enabled = false;
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  digitalWrite(PIN_STEP_L, LOW);
  digitalWrite(PIN_STEP_R, LOW);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);

  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(50);
  s_initPhase = true;
  tmcEnable();
  bool okL = false, okR = false;
  motorDiagQuiet = true;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  motorDiagQuiet = false;
  s_initPhase = false;
  s_msConfirmed = okL && okR;
  motorDiagPrimeMscnt();

  motorMutex = xSemaphoreCreateMutex();
  if (!motorMutex) {
    Serial.println("[MOTOR] FATAL: Mutex konnte nicht angelegt werden");
    return;
  }

  bool taskOk = (xTaskCreatePinnedToCore(
        motorTaskLoop, "MotorTask",
        4096, NULL,
        5,
        &motorTaskHandle,
        0
      ) == pdPASS);
  if (!taskOk) {
    Serial.println("[MOTOR] FATAL: Motor-Task konnte nicht erstellt werden");
    return;
  }

  const esp_timer_create_args_t timerArgs = {
    .callback            = &pulseTimerCb,
    .arg                 = nullptr,
    .dispatch_method     = ESP_TIMER_TASK,
    .name                = "motorPulse",
    .skip_unhandled_events = true
  };
  if (esp_timer_create(&timerArgs, &pulseTimer) != ESP_OK) {
    Serial.println("[MOTOR] FATAL: Puls-Timer konnte nicht erstellt werden - Motor bleibt still");
    return;
  }
  if (esp_timer_start_periodic(pulseTimer, MOTOR_TICK_US) != ESP_OK) {
    Serial.println("[MOTOR] FATAL: Puls-Timer startete nicht - Motor bleibt still");
    return;
  }
  motorReady = true;
}

void startupSpin() {
  if (!motorReady) {
    Serial.println("[MOTOR] Startup-Spin uebersprungen (Motor nicht bereit)");
    return;
  }
  if (config.spinMs == 0 || config.spinSpeed == 0) {
    Serial.println("[MOTOR] Anlaufdrehung aus (Drehzahl oder Dauer = 0)");
    return;
  }
  const long spinL = (long)config.spinSpeed * (long)config.leftSign;
  const long spinR = -(long)config.spinSpeed * (long)config.rightSign;
  Serial.printf("[MOTOR] Anlaufdrehung: %u Schritte/s fuer %u ms\n",
                (unsigned)config.spinSpeed, (unsigned)config.spinMs);
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    stepperL.setMaxSpeed((float)config.spinSpeed);
    stepperR.setMaxSpeed((float)config.spinSpeed);
    stepperL.setSpeed((float)spinL);
    stepperR.setSpeed((float)spinR);
    xSemaphoreGive(motorMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(config.spinMs));
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    stepperHalt(stepperL);
    stepperHalt(stepperR);
    stepperL.setMaxSpeed(FIXED_DRIVE_SPEED);
    stepperR.setMaxSpeed(FIXED_DRIVE_SPEED);
    xSemaphoreGive(motorMutex);
  }
}

static void takeMotorMutex() {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
  }
}

static void setMotorSpeedLocked(int motorIdx, float speed, bool useRamp) {
  if (s_emergency) return;
  if (speed == 0.0f) {
    if (useRamp && s_rampOn && !(motorIdx == 0 ? posActiveL : posActiveR)) {
      volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
      volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
      if (cur != 0.0f || tgt != 0.0f) {
        s_lastMotionMs = millis();
        tgt = 0.0f;
      }
      return;
    }
    if (motorIdx == 0) {
      posActiveL = false; remainingL = 0; s_curL = 0.0f; s_tgtL = 0.0f;
      stepperHalt(stepperL);
    } else {
      posActiveR = false; remainingR = 0; s_curR = 0.0f; s_tgtR = 0.0f;
      stepperHalt(stepperR);
    }
    return;
  }
  {
    if (!s_enabled) return;
    s_lastMotionMs = millis();
    const int sign = (motorIdx == 0) ? config.leftSign : config.rightSign;
    const float actual = (speed > 0.0f ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED) * static_cast<float>(sign);
    AccelStepper& s = (motorIdx == 0) ? stepperL : stepperR;
    volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
    volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
    if (motorIdx == 0) { posActiveL = false; remainingL = 0; }
    else               { posActiveR = false; remainingR = 0; }
    const bool fromStandstill = (cur == 0.0f && tgt == 0.0f);
    stepperHalt(s);
    tgt = actual;
    if (useRamp && s_rampOn) {
      s.setSpeed(cur);
    } else {
      cur = actual;
      s.setSpeed(actual);
    }
    if (fromStandstill) s_diagArmed = true;
  }
}

void setMotorSpeed(int motorIdx, float speed, bool useRamp) {
  if (!motorReady) return;
  if (speed != 0.0f) motorWakeDrivers();
  takeMotorMutex();
  setMotorSpeedLocked(motorIdx, speed, useRamp);
  xSemaphoreGive(motorMutex);
}

void setMotorSpeeds(float speedL, float speedR, bool useRamp) {
  if (!motorReady) return;
  if (speedL != 0.0f || speedR != 0.0f) motorWakeDrivers();
  takeMotorMutex();
  setMotorSpeedLocked(0, speedL, useRamp);
  setMotorSpeedLocked(1, speedR, useRamp);
  xSemaphoreGive(motorMutex);
}

static int16_t mscntSigned(uint16_t raw) {
  int16_t v = (int16_t)(raw & 0x03FFu);
  return (v > 511) ? (int16_t)(v - 1024) : v;
}

bool motorSyncMicrosteps() {
  if (!motorReady || !s_enabled) return false;
  if (motorStillRunning()) return false;
  if (fabsf(s_curL) > 0.0f || fabsf(s_curR) > 0.0f) return false;

  const int16_t mL = mscntSigned((uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FFu));
  const int16_t mR = mscntSigned((uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FFu));
  const long aL = labs((long)mL), aR = labs((long)mR);
  const long diff = labs(aL - aR);

  if (diff <= (long)MOTOR_SYNC_TOLERANCE) {
    Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d - bereits im selben Schritt\n",
                  (int)mL, (int)mR);
    return false;
  }
  if (diff > (long)MOTOR_SYNC_MAX_STEPS) {
    Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d, Differenz %ld zu gross "
                  "(Grenze %u) - NICHT ausgeglichen\n",
                  (int)mL, (int)mR, diff, (unsigned)MOTOR_SYNC_MAX_STEPS);
    return false;
  }

  if (aL < aR) {
    prepPositionMoves(diff * (long)config.leftSign, 0);
  } else {
    prepPositionMoves(0, diff * (long)config.rightSign);
  }
  const uint32_t t0 = millis();
  while (motorStillRunning() && millis() - t0 < MOTOR_SYNC_MAX_MS) {
    bleProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (motorStillRunning()) stopMotors();

  const int16_t aL2 = mscntSigned((uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FFu));
  const int16_t aR2 = mscntSigned((uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FFu));
  Serial.printf("[MOTOR] Microstep-Abgleich: MSCNT L=%d R=%d -> L=%d R=%d "
                "(%ld Mikroschritte ausgeglichen, Rest %ld)\n",
                (int)mL, (int)mR, (int)aL2, (int)aR2, diff,
                labs(labs((long)aL2) - labs((long)aR2)));
  return true;
}

static uint32_t s_alignHoldUntil = 0;

static bool alignReadMscnt(int motorIdx, uint16_t& out) {
  TMC2209Diag& d = (motorIdx == 0) ? tmcL : tmcR;
  for (uint8_t i = 0; i < 3; i++) {
    const uint16_t a = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x3FFu);
    const uint16_t b = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x3FFu);
    if (a == b) { out = a; return true; }
    delay(2);
  }
  return false;
}

static bool alignWaitStill(uint32_t ms) {
  const uint32_t t0 = millis();
  while (motorStillRunning() && millis() - t0 < ms) {
    bleProcessEvents();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (motorStillRunning()) { stopMotors(); return false; }
  return true;
}

static int alignSnapSide(int motorIdx) {
  const int8_t fwdSign = (motorIdx == 0) ? config.leftSign : config.rightSign;
  const uint16_t ms = s_microsteps;
  if (ms <= 1) return 0;
  uint16_t raw0 = 0;
  if (!alignReadMscnt(motorIdx, raw0)) return -1;
  prepPositionMove(motorIdx, 4L * (long)fwdSign);
  if (!alignWaitStill(150)) return -1;
  uint16_t raw1 = 0;
  if (!alignReadMscnt(motorIdx, raw1)) return -1;
  prepPositionMove(motorIdx, -4L * (long)fwdSign);
  if (!alignWaitStill(150)) return -1;
  const uint16_t delta = (uint16_t)((raw1 - raw0) & 0x3FFu);
  if (delta == 0) return -1;
  const int dir = (delta <= 511) ? +1 : -1;
  uint16_t raw2 = 0;
  if (!alignReadMscnt(motorIdx, raw2)) return -1;
  const uint16_t r = (dir > 0) ? (uint16_t)((ms - (raw2 % ms)) % ms)
                               : (uint16_t)(raw2 % ms);
  if (r == 0) return 0;
  prepPositionMove(motorIdx, (long)r * (long)fwdSign);
  if (!alignWaitStill(250)) return -1;
  return (int)r;
}

bool motorAlignRequest() {
  if (!motorReady) return false;
  if (motorStillRunning()) return false;
  if (fabsf(s_curL) > 0.0f || fabsf(s_curR) > 0.0f) return false;
  if (s_emergency) return false;
  motorWakeDrivers();
  if (!s_enabled) return false;
  Serial.println("[MOTOR] Ausrichten: Vollschritt-Snap laeuft");
  const int movedL = alignSnapSide(0);
  const int movedR = alignSnapSide(1);
  if (movedL < 0 || movedR < 0) {
    Serial.println("[MOTOR] Ausrichten: MSCNT nicht lesbar - abgebrochen");
    return false;
  }
  const uint8_t irun = irunForMilliAmp(TMC_CURRENT_MA_FIXED);
  const uint32_t full = iholdIrunReg(irun, irun);
  tmcWriteChecked(tmcL, TMC_REG_IHOLD_IRUN, full, 0x001FFFFFu, "IHOLD/IRUN");
  tmcWriteChecked(tmcR, TMC_REG_IHOLD_IRUN, full, 0x001FFFFFu, "IHOLD/IRUN");
  s_alignHoldUntil = millis() + MOT_ALIGN_HOLD_MS;
  Serial.printf("[MOTOR] Ausrichten: Snap L=%d R=%d Mikroschritte, VOLLSTROM-Halt %lu ms - jetzt gerade hinstellen\n",
                movedL, movedR, (unsigned long)MOT_ALIGN_HOLD_MS);
  return true;
}

void stopMotors() {
     if (!motorReady) return;
  takeMotorMutex();
  posActiveL = false; remainingL = 0; s_curL = 0.0f; s_tgtL = 0.0f; stepperHalt(stepperL);
  posActiveR = false; remainingR = 0; s_curR = 0.0f; s_tgtR = 0.0f; stepperHalt(stepperR);
  xSemaphoreGive(motorMutex);
}


void motorEmergencyStop() {
  s_emergency = true;
  if (!motorReady) return;
  stepperHalt(stepperL);
  stepperHalt(stepperR);
  s_tgtL = 0.0f; s_tgtR = 0.0f;
  s_curL = 0.0f; s_curR = 0.0f;
  posActiveL = false; posActiveR = false;
  remainingL = 0; remainingR = 0;
}

void motorReleaseEmergency() {
  stopMotors();
  s_emergency = false;
}

bool motorEmergencyActive() { return s_emergency; }

static void prepPositionMoveLocked(int motorIdx, long physicalSteps);

void prepPositionMove(int motorIdx, long physicalSteps) {
  if (!motorReady) return;
  if (physicalSteps != 0) motorWakeDrivers();
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    prepPositionMoveLocked(motorIdx, physicalSteps);
    xSemaphoreGive(motorMutex);
  }
}

void prepPositionMoves(long stepsL, long stepsR) {
  if (!motorReady) return;
  if (stepsL != 0 || stepsR != 0) motorWakeDrivers();
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    prepPositionMoveLocked(0, stepsL);
    prepPositionMoveLocked(1, stepsR);
    xSemaphoreGive(motorMutex);
  }
}

static void prepPositionMoveLocked(int motorIdx, long physicalSteps) {
  if (s_emergency) return;
  {
    AccelStepper& s = (motorIdx == 0) ? stepperL : stepperR;
    volatile float& cur = (motorIdx == 0) ? s_curL : s_curR;
    volatile float& tgt = (motorIdx == 0) ? s_tgtL : s_tgtR;
    cur = 0.0f; tgt = 0.0f;
    if (physicalSteps == 0) {
      if (motorIdx == 0) { posActiveL = false; remainingL = 0; stepperHalt(s); }
      else               { posActiveR = false; remainingR = 0; stepperHalt(s); }
    } else {
      if (!s_enabled) return;
      s_lastMotionMs = millis();
      stepperHalt(s);
      s.setMaxSpeed(FIXED_DRIVE_SPEED);
      if (motorIdx == 0) { posActiveL = true;  remainingL = labs(physicalSteps); }
      else               { posActiveR = true;  remainingR = labs(physicalSteps); }
      if (s_rampOn) {
        s.setAcceleration(motorAccelStepsS2());
        s.move(physicalSteps);
      } else {
        s.setSpeed(physicalSteps >= 0 ? FIXED_DRIVE_SPEED : -FIXED_DRIVE_SPEED);
      }
    }
  }
}

bool motorStillRunning() {
  return (posActiveL && remainingL != 0) || (posActiveR && remainingR != 0);
}


static int32_t tmcSignExtend24(uint32_t v) {
  v &= 0x00FFFFFFu;
  return (v & 0x00800000u) ? (int32_t)(v | 0xFF000000u) : (int32_t)v;
}

static void motorDiagReport();
static void motorDiagSendCompact();

bool motorDiagRequest() {
  if (!motorReady || !s_enabled) return false;
  if (s_diagStep != 0) return false;
  if (!s_mscntPrimed) motorDiagPrimeMscnt();
  memset(&s_meas, 0, sizeof(s_meas));
  s_diagStep = 1;
  s_diagAuto   = false;
  s_diagWindow = false;
  s_diagFlags  = 0;
  return true;
}

bool motorDiagRunning() { return s_diagStep != 0; }

void motorDiagTick() {
  if (!motorReady) return;
  if (!s_enabled) { s_diagStep = 0; s_diagArmed = false; return; }
  if (s_diagStep == 0) {
    if (!s_diagArmed) return;
    s_diagArmed = false;
    if (posActiveL || posActiveR) return;
    const float rate = fabsf(s_curL);
    if (rate < MOT_DIAG_MIN_STEPS_S || rate > MOT_DIAG_MAX_STEPS_S) return;
    if (!motorDiagRequest()) return;
    s_diagAuto   = true;
    s_diagWindow = true;
  } else if (s_diagAuto) {
    const float rate = fabsf(s_curL);
    if (rate < MOT_DIAG_MIN_STEPS_S || rate > MOT_DIAG_MAX_STEPS_S ||
        posActiveL || posActiveR) {
      s_diagStep = 0;
      Serial.printf("[MOT-DIAG] abgebrochen: Tempo %.0f Schritte/s (Fenster %.0f..%.0f)\n",
                    (double)rate, (double)MOT_DIAG_MIN_STEPS_S, (double)MOT_DIAG_MAX_STEPS_S);
      return;
    }
  }
  if (s_diagStep != 0) s_lastMotionMs = millis();

  const uint8_t step = (uint8_t)(s_diagStep - 1);
  TMC2209Diag& d = (step & 1) ? tmcR : tmcL;
  const uint8_t  i = (uint8_t)(step & 1);
  switch (step) {
    case  0: case  1: s_meas.ifcntA[i] = (uint8_t)d.read(TMC_REG_IFCNT);     break;
    case  2: case  3: s_meas.sg[i]      = (uint16_t)(d.read(TMC_REG_SG_RESULT) & 0x03FF); break;
    case  4: case  5: s_meas.vactual[i] = tmcSignExtend24(d.read(TMC_REG_VACTUAL)); break;
    case  6: case  7: s_meas.tstep[i]   = d.read(TMC_REG_TSTEP);            break;
    case  8: case  9: s_meas.mscnt[i]   = (uint16_t)(d.read(TMC_REG_MSCNT) & 0x03FF); break;
    case 10: case 11: s_meas.gstat[i]   = (uint8_t)(d.read(TMC_REG_GSTAT) & 0x07); break;
    case 12: case 13: s_meas.drv[i]     = d.read(TMC_REG_DRV_STATUS);        break;
    case 14: case 15: s_meas.ifcntB[i] = (uint8_t)d.read(TMC_REG_IFCNT);     break;
    default: break;
  }
  if (++s_diagStep > 16) { s_diagStep = 0; motorDiagReport(); }

  if (s_diagStep == 0 && !posActiveL && !posActiveR && fabsf(s_curL) > 100.0f) {
    const uint32_t now = millis();
    if (now - s_steadyLastMs >= 2000UL) {
      s_steadyLastMs = now;
      const uint32_t t = millis();
      motorDiagSteadySpeed(false);
      s_steadyBusyMs = millis() - t;
    }
  }
}

bool motorDiagSteadySpeed(bool verbose) {
  if (!motorReady || !s_enabled) return false;
  const int32_t vL = tmcSignExtend24(tmcL.read(TMC_REG_VACTUAL));
  const int32_t vR = tmcSignExtend24(tmcR.read(TMC_REG_VACTUAL));
  const uint16_t mL = (uint16_t)(tmcL.read(TMC_REG_MSCNT) & 0x03FF);
  const uint16_t mR = (uint16_t)(tmcR.read(TMC_REG_MSCNT) & 0x03FF);

  const float stepsPerMM = configStepsPerMM();
  const float mmL = (float)vL / stepsPerMM;
  const float mmR = (float)vR / stepsPerMM;
  const float aL = vL < 0 ? -(float)vL : (float)vL;
  const float aR = vR < 0 ? -(float)vR : (float)vR;
  const float mx = aL > aR ? aL : aR;
  const int diffPct = (mx > 0.0f) ? (int)((aL - (aL < aR ? aL : aR)) * 100.0f / mx) : 0;

  if (!verbose) {
    Serial.printf("[DRV] unterwegs: vL=%ld vR=%ld uSchritt/s  Diff %d %%  MSCNT %u/%u  belegt %ld\n",
                  (long)vL, (long)vR, diffPct, mL, mR,
                  (long)s_steadyBusyMs);
    return true;
  }

  Serial.println();
  Serial.println("===== MOMENTAUFNAHME MITTEN IN DER FAHRT =====");
  Serial.printf("VACTUAL L=%ld R=%ld uSchritt/s  (~%.1f / %.1f mm/s)  Diff %d %%\n",
                (long)vL, (long)vR, (double)mmL, (double)mmR, diffPct);
  Serial.printf("MSCNT   L=%u R=%u\n", mL, mR);
  if (vL == 0 && vR == 0) {
    Serial.println("Beide 0: der Roboter steht (oder der Sensor liefert nichts).");
  } else if (diffPct <= 3) {
    Serial.println("BEWERTUNG: beide Raeder drehen gleich schnell. Ein Drift kommt dann");
    Serial.println("           NICHT aus unterschiedlichem Vortschritt - sondern aus");
    Serial.println("           seitlichem Versatz (Achse schief, Boden, Spiel).");
  } else {
    Serial.println("BEWERTUNG: die Raeder drehen UNTERSCHIEDLICH schnell. Ein Rad");
    Serial.println("           kommt nicht voran: Schlupf, Reibung im Getriebe oder");
    Serial.println("           ein Rad schleift. Software-seitig ist die Pulsgabe");
    Serial.println("           gleich (siehe DRV:-Zaehler) - also mechanisch.");
  }
  Serial.println("==============================================");
  Serial.println();

  char m[72];
  snprintf(m, sizeof(m), "MOT:SPD:%ld,%ld,%d,%u,%u",
           (long)vL, (long)vR, diffPct, mL, mR);
sendStatus(m);
  return true;
}

void motorDiagReport() {
  for (uint8_t i = 0; i < 2; i++) {
    s_mscntAcc[i] += mscntDelta(s_meas.mscnt[i], s_mscntPrev[i]);
    s_mscntPrev[i] = s_meas.mscnt[i];
  }

  uint16_t fl = 0;
  for (uint8_t i = 0; i < 2; i++) {
    if (s_meas.gstat[i] & 0x04) fl |= DIAG_UV_CP;
    if (s_meas.gstat[i] & 0x02) fl |= DIAG_DRV_ERR;
    if (s_meas.drv[i] & (DRVST_OLA | DRVST_OLB)) fl |= DIAG_OPEN_LOAD;
    if (s_meas.drv[i] & (DRVST_S2GA | DRVST_S2GB | DRVST_S2VSA | DRVST_S2VSB)) fl |= DIAG_SHORT;
    if (s_meas.drv[i] & (DRVST_OT | DRVST_OTPW)) fl |= DIAG_OVERTEMP;
  }
  if (((s_meas.drv[0] & DRVST_STEALTH) != 0) != ((s_meas.drv[1] & DRVST_STEALTH) != 0))
    fl |= DIAG_MODE_DIFF;

  const uint32_t tstepSat = 0x000FFFFFu;
  for (uint8_t i = 0; i < 2; i++) {
    if (s_meas.tstep[i] >= tstepSat) fl |= DIAG_TSTEP_SAT;
  }
  if (!(fl & DIAG_TSTEP_SAT) && s_meas.tstep[0] > 0 && s_meas.tstep[1] > 0) {
    const uint32_t lo = s_meas.tstep[0] < s_meas.tstep[1] ? s_meas.tstep[0] : s_meas.tstep[1];
    const uint32_t hi = s_meas.tstep[0] < s_meas.tstep[1] ? s_meas.tstep[1] : s_meas.tstep[0];
    if ((hi - lo) * 100u / lo > DIAG_TSTEP_DIFF_PCT) fl |= DIAG_TSTEP_DIFF;
  }
  const uint8_t csL = (uint8_t)((s_meas.drv[0] >> DRVST_CS_ACTUAL_SHIFT) & DRVST_CS_ACTUAL_MASK);
  const uint8_t csR = (uint8_t)((s_meas.drv[1] >> DRVST_CS_ACTUAL_SHIFT) & DRVST_CS_ACTUAL_MASK);
  if (csL > 0 && csR > 0) {
    const uint8_t mx = csL > csR ? csL : csR;
    const uint8_t mn = csL > csR ? csR : csL;
    if ((uint16_t)(mx - mn) * 100u / mx > DIAG_CS_DIFF_PCT) fl |= DIAG_CURRENT_DIFF;
  }
  const int ds = (int)s_meas.sg[0] - (int)s_meas.sg[1];
  if (ds < 0 ? -ds > DIAG_SG_DIFF_MIN : ds > DIAG_SG_DIFF_MIN) fl |= DIAG_SG_DIFF;
  if (s_meas.vactual[0] != 0 && s_meas.vactual[1] != 0) {
    const int32_t ax = s_meas.vactual[0] < 0 ? -s_meas.vactual[0] : s_meas.vactual[0];
    const int32_t ay = s_meas.vactual[1] < 0 ? -s_meas.vactual[1] : s_meas.vactual[1];
    const int32_t mx = ax > ay ? ax : ay;
    if (mx > 0 && (uint32_t)(ax - (ax > ay ? ay : ax)) * 100u / (uint32_t)mx > DIAG_VACTUAL_DIFF_PCT)
      fl |= DIAG_VACTUAL_DIFF;
  }
  const long a0 = s_mscntAcc[0] < 0 ? -s_mscntAcc[0] : s_mscntAcc[0];
  const long a1 = s_mscntAcc[1] < 0 ? -s_mscntAcc[1] : s_mscntAcc[1];
  if (labs(a0 - a1) > DIAG_MSCNT_TOLERANCE) fl |= DIAG_MSCNT_DIFF;

  if ((uint8_t)(s_meas.ifcntB[0] - s_meas.ifcntA[0]) > 127 ||
      (uint8_t)(s_meas.ifcntB[1] - s_meas.ifcntA[1]) > 127) fl |= DIAG_READ_ERROR;
  if (s_diagWindow && (s_meas.vactual[0] == 0 || s_meas.vactual[1] == 0 ||
                       s_meas.tstep[0] == 0   || s_meas.tstep[1] == 0))
    fl |= DIAG_READ_ERROR;
  s_diagFlags = fl;

  const float spmm = configStepsPerMM();
  Serial.println();
  Serial.println(s_diagAuto ? "===== TREIBER-MESSUNG (nach dem Anfahren) ====="
                            : "===== TREIBER-MESSUNG (manuell angefordert) =====");
  if (!s_diagWindow) {
    Serial.println("Hinweis: gemessen wurde ausserhalb der Fahrt. VACTUAL/TSTEP sind");
    Serial.println("       dann ohne Aussage (Steiger sind im Stillstand nicht messbar);");
    Serial.println("       GSTAT, SG_RESULT, MSCNT und DRV_STATUS gelten trotzdem.");
  }
  Serial.printf("SG_RESULT L=%u R=%u  (Diff %d, Stall ab <= %u)\n",
                s_meas.sg[0], s_meas.sg[1], ds, TMC_SGTHRS_DEFAULT * 2);
  Serial.println("  Drehmomentreserve: groesser = mehr Reserve. Nur oberhalb TCOOLTHRS");
  Serial.println("  verlaesslich und fuer StealthChop kalibriert. 0 = Stall oder Lesefehler.");
  Serial.printf("VACTUAL   L=%ld R=%ld uSchritt/s  (~%.1f / %.1f mm/s)\n",
                (long)s_meas.vactual[0], (long)s_meas.vactual[1],
                (double)(s_meas.vactual[0] / spmm), (double)(s_meas.vactual[1] / spmm));
  Serial.println("  Von jedem Treiber selbst gemessen. Absolute Werte schwanken je IC");
  Serial.println("  (interner Takt) - aussagekraeftig ist nur der Vergleich.");
  Serial.printf("TSTEP     L=%lu R=%lu  (Anschlag %lu = Stillstand)\n",
                (unsigned long)s_meas.tstep[0], (unsigned long)s_meas.tstep[1],
                (unsigned long)tstepSat);
  Serial.println("  Von beiden Treibern gemessene STEP-Periode. Stark verschieden = einer");
  Serial.println("  bekommt andere Pulse. Anschlag = (fast) keine Pulse gesehen.");
  Serial.printf("MSCNT     L %ld R %ld  (Betragsdifferenz %ld Mikroschritte)\n",
                (long)s_mscntAcc[0], (long)s_mscntAcc[1], labs(a0 - a1));
  Serial.println("  Der Zaehlerstand selbst bedeutet nichts - entscheidend ist der VORRUCK:");
  Serial.println("  beide Seiten sollten gleich weit gelaufen sein (Vorzeichen gegenlaeufig).");
  Serial.printf("GSTAT     L=0x%02X R=0x%02X   (bit1 drv_err, bit2 uv_cp)\n",
                s_meas.gstat[0], s_meas.gstat[1]);
  Serial.printf("DRV_STATUS L=0x%08lX R=0x%08lX\n",
                (unsigned long)s_meas.drv[0], (unsigned long)s_meas.drv[1]);
  Serial.printf("  Modus %s/%s, CS_ACTUAL L=%u R=%u, OLA=%u OLB=%u, OT=%u, Kurzschluss=%u\n",
                (s_meas.drv[0] & DRVST_STEALTH) ? "Stealth" : "Spread",
                (s_meas.drv[1] & DRVST_STEALTH) ? "Stealth" : "Spread",
                csL, csR,
                (s_meas.drv[0] & DRVST_OLA) ? 1u : 0u, (s_meas.drv[0] & DRVST_OLB) ? 1u : 0u,
                (s_meas.drv[0] & (DRVST_OT | DRVST_OTPW)) ? 1u : 0u,
                (s_meas.drv[0] & (DRVST_S2GA | DRVST_S2GB | DRVST_S2VSA | DRVST_S2VSB)) ? 1u : 0u);
  Serial.printf("IFCNT     L %u->%u  R %u->%u  (UART %s)\n",
                s_meas.ifcntA[0], s_meas.ifcntB[0], s_meas.ifcntA[1], s_meas.ifcntB[1],
                (fl & DIAG_READ_ERROR) ? "PRUEFEN" : "zaehlt weiter");
  if (fl == 0) {
    Serial.println("BEFUND    nichts auffaellig - beide Treiber sehen dasselbe Bild.");
  } else {
    Serial.println("BEFUND");
    if (fl & DIAG_UV_CP)       Serial.println("  ! Ladepumpen-Unterspannung (GSTAT bit2). Der Treiber schaltet");
    if (fl & DIAG_UV_CP)       Serial.println("    laut Datenblatt in diesem Zustand ab - Motorstrom ist nicht da.");
    if (fl & DIAG_DRV_ERR)     Serial.println("  ! Treiberfehler (GSTAT bit1) - Bit wird nur durch Schreiben gelesen.");
    if (fl & DIAG_MODE_DIFF)   Serial.println("  ! Ein Treiber in StealthChop, der andere in SpreadCycle: die beiden");
    if (fl & DIAG_MODE_DIFF)   Serial.println("    Seiten haben unterschiedliche Drehmoment-Charakteristik.");
    if (fl & DIAG_OPEN_LOAD)   Serial.println("  ! Offene Last (OLA/OLB): Kabel, Stecker oder Wicklung.");
    if (fl & DIAG_SHORT)       Serial.println("  ! Kurzschluss-Flag gesetzt.");
    if (fl & DIAG_OVERTEMP)    Serial.println("  ! Ueberhitzung gemeldet (OT/OTPW).");
    if (fl & DIAG_CURRENT_DIFF)Serial.println("  ! CS_ACTUAL weit auseinander: ein Rad laeuft deutlich schwerer.");
    if (fl & DIAG_TSTEP_SAT)   Serial.println("  ! TSTEP am Anschlag: eine Seite hat (fast) keine STEP-Pulse bekommen.");
    if (fl & DIAG_TSTEP_DIFF)  Serial.println("  ! STEP-Periode der Treiber weit auseinander: Pulse gehen unterschiedlich an.");
    if (fl & DIAG_MSCNT_DIFF)  Serial.println("  ! MSCNT-Vorreck ungleich: eine Seite hat Mikroschritte verloren -");
    if (fl & DIAG_MSCNT_DIFF)  Serial.println("    das ist der direkte Nachweis fuer einen Schrittverlust.");
    if (fl & DIAG_SG_DIFF)     Serial.println("  ! Drehmomentreserve der Seiten stark verschieden.");
    if (fl & DIAG_VACTUAL_DIFF)Serial.println("  ! Gemessene Drehzahl weit auseinander: Schlupf oder Asymmetrie.");
    if (fl & DIAG_READ_ERROR)  Serial.println("  ! Lesefehler/CRC oder Bus steht - Zahlen dieser Messung nicht verwenden.");
  }
  Serial.println("Hinweis: reine Beobachtung. Es wird nichts nachgeregelt oder ausgeglichen.");
  Serial.println("=======================================================");
  Serial.println();
  motorDiagSendCompact();
}

void motorDiagSendCompact() {
  char m[112];
  snprintf(m, sizeof(m),
           "MOT:MEAS:%u,%u,%u,%ld,%ld,%lu,%lu,%ld,%ld,%u,%u",
           s_diagFlags, s_meas.sg[0], s_meas.sg[1],
           (long)s_meas.vactual[0], (long)s_meas.vactual[1],
           (unsigned long)s_meas.tstep[0], (unsigned long)s_meas.tstep[1],
           (long)s_mscntAcc[0], (long)s_mscntAcc[1],
           s_meas.gstat[0], s_meas.gstat[1]);
  sendStatus(m);
}



static void tmcEnsureConfigured() {
  if (!motorReady) return;
  if (s_initPhase) return;
  bool okL = false, okR = false;
  configureOne(tmcL, okL);
  configureOne(tmcR, okR);
  s_msConfirmed = okL && okR;
  motorDiagPrimeMscnt();
}

void tmcEnable() {
  const bool wasOff = !s_enabled;
  s_alignHoldUntil = 0;
  digitalWrite(PIN_EN, LOW);
  s_enabled = true;
  if (wasOff) {
    delay(TMC_EN_SETTLE_MS);
    tmcEnsureConfigured();
  }
}

void motorWakeDrivers() {
  if (!motorReady) return;
  if (!s_enabled) { tmcEnable(); return; }
  if (!s_msConfirmed && (uint32_t)(millis() - s_lastCfgRetryMs) >= 1000u) {
    s_lastCfgRetryMs = millis();
    Serial.println("[TMC] Registersatz unvollstaendig - Konfiguration wird beim Fahren nachgeholt");
    tmcEnsureConfigured();
  }
}

static uint32_t s_batLastMs = 0;
static uint8_t  s_batSide = 0;
static uint8_t  s_batHits = 0;
static uint32_t s_batReportMs = 0;
static bool     s_batLow = false;
bool tmcBatteryLow() { return s_batLow; }

void tmcBatteryGuardTick() {
  const uint32_t now = millis();
  if (s_batLow) {
    if (now - s_batReportMs >= 30000u) {
      s_batReportMs = now;
      sendStatus("BAT:LOW");
    }
    return;
  }
  if (!motorReady || !s_enabled) return;
  if (now - s_batLastMs < 2000u) return;
  s_batLastMs = now;
  TMC2209Diag& d = (s_batSide == 0) ? tmcL : tmcR;
  s_batSide ^= 1;
  const uint8_t g = (uint8_t)(d.read(TMC_REG_GSTAT) & 0x07);
  if (g & 0x04) {
    if (++s_batHits >= 3) {
      s_batLow = true;
      s_batReportMs = now;
      Serial.println("[BAT] Unterspannung (GSTAT uv_cp, 3x bestaetigt) - Akku schwach, bitte laden");
      sendStatus("BAT:LOW");
    }
  } else {
    s_batHits = 0;
  }
}


void tmcDisable() {
  stopMotors();
  digitalWrite(PIN_EN, HIGH);
  s_enabled = false;
}

bool tmcIsEnabled() { return s_enabled; }

void tmcAutoStandbyTick() {
  if (!motorReady) return;
  if (!s_enabled) return;
  if (s_alignHoldUntil != 0) {
    if ((int32_t)(millis() - s_alignHoldUntil) < 0) {
      s_lastMotionMs = millis();
      return;
    }
    const uint8_t irun = irunForMilliAmp(TMC_CURRENT_MA_FIXED);
    const uint32_t half = iholdIrunReg(irun, (uint8_t)(irun / 2u));
    tmcL.write(TMC_REG_IHOLD_IRUN, half);
    tmcR.write(TMC_REG_IHOLD_IRUN, half);
    s_alignHoldUntil = 0;
    Serial.println("[MOTOR] Ausrichten: Halt abgelaufen, Haltestrom wieder halb");
  }
  if (millis() - s_lastMotionMs < TMC_STANDBY_MS) return;
  bool idle = false;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    idle = !posActiveL && !posActiveR &&
           stepperL.speed() == 0.0f && stepperR.speed() == 0.0f;
    xSemaphoreGive(motorMutex);
  }
  if (idle) {
    tmcDisable();
    Serial.println("[TMC] Auto-Standby: Treiber aus (EN)");
  }
}

void tmcReinit() {
  tmcEnable();
  const bool ok = tmcConfigureBothDrivers();
  motorDiagPrimeMscnt();
  Serial.printf("[TMC] Reinit: %s\n", ok ? "OK" : "FEHLER - Register nicht bestaetigt");
}


bool tmcSetMicrosteps(uint16_t ms) {
  if (!validMicrosteps(ms)) return false;
  const uint16_t prevShadow = s_microsteps;
  const uint16_t prevConfig = config.tmcMicrosteps;
  config.tmcMicrosteps = ms;
  s_microsteps = ms;
  motorUpdateSpeeds();
  const bool okL = applyMicrosteps(tmcL, ms);
  const bool okR = applyMicrosteps(tmcR, ms);
  s_msConfirmed = okL && okR;
  motorDiagPrimeMscnt();
  if (!s_msConfirmed) {
    s_microsteps = prevShadow;
    config.tmcMicrosteps = prevConfig;
    motorUpdateSpeeds();
    Serial.printf("[TMC] Microsteps %u nicht bestaetigt (L=%d R=%d) - auf %u zurueckgesetzt\n",
                  ms, okL ? 1 : 0, okR ? 1 : 0, prevShadow);
  }
  return s_msConfirmed;
}

void tmcSetSpreadCycle(bool spread) {
  if (motorStillRunning() || fabsf(s_curL) > 0.0f || fabsf(s_curR) > 0.0f) {
    Serial.println("[TMC] Moduswechsel abgelehnt - Roboter faehrt. Erst anhalten, dann wechseln.");
    sendStatus("TMC:BUSY");
    return;
  }
  s_spread = spread;
  tmcEnable();
  uint32_t gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT | GCONF_MULTISTEP_FILT
                 | 0x01u;
  if (s_spread) gconf |= GCONF_EN_SPREADCYCLE;
  tmcWriteChecked(tmcL, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");
  tmcWriteChecked(tmcR, TMC_REG_GCONF, gconf, GCONF_MASK, "GCONF");
  motorSyncMicrosteps();
}

bool tmcSpreadCycle() { return s_spread; }
uint16_t tmcGetCurrent() { return TMC_CURRENT_MA_FIXED; }
uint8_t tmcGetHold() { return (uint8_t)(irunForMilliAmp(TMC_CURRENT_MA_FIXED) / 2u); }
uint16_t tmcGetMicrosteps() { return s_microsteps; }
bool tmcMicrostepsConfirmed() { return s_msConfirmed; }

void tmcSendCompactStatus() {
  const uint8_t cL = tmcL.test_connection(), cR = tmcR.test_connection();
  char m[64];
  snprintf(m, sizeof(m), "TMC:STAT:%d,%d,%d,%d,%d,%d,%d,%d",
    cL == 0 ? 1 : 0, cR == 0 ? 1 : 0,
    tmcL.version(), tmcR.version(),
    TMC_CURRENT_MA_FIXED, s_microsteps, s_spread ? 1 : 0, s_enabled ? 1 : 0);
  sendStatus(m);
}

void tmcPrintStatus() {
  const uint8_t connL = tmcL.test_connection();
  const uint8_t connR = tmcR.test_connection();
  const uint32_t stL = tmcL.DRV_STATUS();
  const uint32_t stR = tmcR.DRV_STATUS();
  Serial.println();
  Serial.println("========== TMC2209 STATUS (Edu V2) ==========");
  Serial.printf("UART L(addr %u): %s | R(addr %u): %s\n",
    TMC_ADDR_L, connL == 0 ? "OK" : "FEHLER",
    TMC_ADDR_R, connR == 0 ? "OK" : "FEHLER");
  Serial.printf("Version L=0x%X R=0x%X (erwartet 0x21)\n", tmcL.version(), tmcR.version());
  Serial.printf("Treiber-EN (GPIO%d): %s\n", PIN_EN, s_enabled ? "EIN" : "AUS");
  Serial.printf("Strom fest: IRUN=%u -> %.0f mA RMS (Soll %u mA), IHOLD=%u -> %.0f mA\n",
              irunForMilliAmp(TMC_CURRENT_MA_FIXED),
              (double)milliAmpForIrun(irunForMilliAmp(TMC_CURRENT_MA_FIXED)),
              (unsigned)TMC_CURRENT_MA_FIXED, tmcGetHold(),
              (double)milliAmpForIrun(tmcGetHold()));
  Serial.printf("  Nicht einstellbar - Nennstrom des NEMA8. Obergrenze durch die "
                "560-mOhm-Fuehler waere %.0f mA (IRUN=31); davon fahren wir %.0f mA.\n",
                (double)milliAmpForIrun(31),
                (double)milliAmpForIrun(irunForMilliAmp(TMC_CURRENT_MA_FIXED)));
  Serial.printf("Microsteps: %u %s\n", s_microsteps, s_msConfirmed ? "(OK)" : "(NICHT UEBERNOMMEN!)");
  Serial.printf("Modus: %s\n", s_spread ? "SpreadCycle" : "StealthChop");
  Serial.printf("IFCNT: L=%u R=%u\n", tmcL.IFCNT(), tmcR.IFCNT());
  const uint8_t gL = (uint8_t)(tmcL.read(TMC_REG_GSTAT) & 0x07);
  const uint8_t gR = (uint8_t)(tmcR.read(TMC_REG_GSTAT) & 0x07);
  Serial.printf("GSTAT: L=0x%02X R=0x%02X%s%s\n", gL, gR,
    (gL & 0x02) ? "  <<< drv_err links" : "", (gR & 0x02) ? "  <<< drv_err rechts" : "");
  if ((gL | gR) & 0x04) {
    Serial.println("  ! Ladepumpen-Unterspannung (bit2) - laut Datenblatt schaltet der");
    Serial.println("    Treiber dann ab. VM und alle drei Abblockkondensatoren pruefen.");
  }
  Serial.printf("SG_RESULT (Register 0x41): L=%u R=%u (Stall ab <= %u)\n",
                tmcL.SG_RESULT(), tmcR.SG_RESULT(), TMC_SGTHRS_DEFAULT * 2);
  Serial.printf("Modus: immer StealthChop (TPWMTHRS=0, kein Spread-Umschalter)\n");
  Serial.printf("DRV_STATUS: L=0x%08lX R=0x%08lX\n",
                (unsigned long)stL, (unsigned long)stR);
  Serial.println("=============================================");
  Serial.println();
}

void tmcBootDiag() {
  Serial.println("========== TMC BOOT-DIAG ==========");
  const uint8_t connL = tmcL.test_connection();
  const uint8_t connR = tmcR.test_connection();
  Serial.printf("UART L: %s (Code %u) | R: %s (Code %u)\n",
    connL == 0 ? "OK" : "FEHLER", connL, connR == 0 ? "OK" : "FEHLER", connR);
  Serial.printf("Version L=0x%X R=0x%X (erwartet 0x21)\n", tmcL.version(), tmcR.version());
  const uint8_t ifBefore = tmcL.IFCNT();
  { const uint32_t g = tmcL.read(TMC_REG_GCONF); tmcL.write(TMC_REG_GCONF, g); }
  const uint8_t ifAfter = tmcL.IFCNT();
  Serial.printf("IFCNT L: %u -> %u %s\n", ifBefore, ifAfter,
    ifAfter != ifBefore ? "(zaehlt = Austausch OK)" : "(steht = KEIN Austausch!)");
  Serial.printf("MRES L: %u | Strom: %u mA | Hold: %u | Modus: %s | EN: %s\n",
    mresCodeFor(s_microsteps), tmcGetCurrent(), tmcGetHold(),
    s_spread ? "SpreadCycle" : "StealthChop", s_enabled ? "EIN" : "AUS");
  Serial.printf("Modus: immer StealthChop (TPWMTHRS=0, bestaetigt ueber IFCNT)\n");
  Serial.println("Messwerte selbst: Befehl MOT:DIAG (im Log und als MOT:MEAS: in der Web-UI).");
  Serial.println("===================================");
}

void tmcNetDiag() {
  Serial.println("========== NETZ-DIAG ==========");
  Serial1.end();
  pinMode(TMC_UART_TX, INPUT);
  pinMode(TMC_UART_RX, INPUT);
  delayMicroseconds(20);
  const int idle = digitalRead(TMC_UART_RX);
  Serial.printf("[a] Netzpegel ohne ESP-Treiber: %s\n",
    idle ? "HIGH (Pull-up vorhanden)" : "LOW (kein externer Pull-up)");

  pinMode(TMC_UART_TX, OUTPUT_OPEN_DRAIN);
  digitalWrite(TMC_UART_TX, LOW);
  delayMicroseconds(50);
  const int pulled = digitalRead(TMC_UART_RX);
  digitalWrite(TMC_UART_TX, HIGH);
  const int s0 = digitalRead(TMC_UART_RX);
  delayMicroseconds(1);   const int s1 = digitalRead(TMC_UART_RX);
  delayMicroseconds(9);   const int s10 = digitalRead(TMC_UART_RX);
  delayMicroseconds(90);  const int s100 = digitalRead(TMC_UART_RX);
  delay(1);               const int s1000 = digitalRead(TMC_UART_RX);
  Serial.printf("[b] Open-Drain LOW gezogen: Netz=%s\n", pulled ? "HIGH?!" : "LOW");
  Serial.printf("[c] Nach Release: 0us=%d 1us=%d 10us=%d 100us=%d 1ms=%d (1=HIGH)\n",
    s0, s1, s10, s100, s1000);

  pinMode(TMC_UART_TX, OUTPUT);
  digitalWrite(TMC_UART_TX, LOW);
  delayMicroseconds(50);
  const int ll = digitalRead(TMC_UART_RX);
  digitalWrite(TMC_UART_TX, HIGH);
  pinMode(TMC_UART_TX, INPUT);
  Serial.printf("[d] TX-PushPull LOW durch R9(%u Ohm): ESP liest %s\n",
    (unsigned)R9_SERIES_OHM, ll ? "HIGH" : "LOW");

  Serial1.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(2);
  Serial.println("================================");
}

void tmcRegDump() {
  Serial.println();
  Serial.println("========== TMC REGISTER-DUMP ==========");
  struct { const char* n; uint8_t a; } const regs[] = {
    {"GCONF", 0x00}, {"GSTAT", 0x01}, {"IFCNT", 0x02}, {"NODECONF", 0x03},
    {"OTP_READ", 0x05}, {"IOIN", 0x06}, {"FACTORY_CONF", 0x07}, {"SHORT_CONF", 0x08},
    {"SHORT_STAT", 0x09}, {"GLOBAL_SCALER", 0x0B},
    {"IHOLD_IRUN", 0x10}, {"TPOWER", 0x11},
    {"TSTEP", 0x12}, {"TPWMTHRS", 0x13}, {"TCOOLTHRS", 0x14}, {"VACTUAL", 0x22},
    {"SGTHRS", 0x40}, {"SG_RESULT", 0x41}, {"COOLCONF", 0x42},
    {"MSCNT", 0x6A}, {"CHOPCONF", 0x6C}, {"DRV_STATUS", 0x6F}, {"PWMCONF", 0x70},
  };
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    const uint32_t v = tmcL.read(regs[i].a);
    const bool writeOnly = (regs[i].a == 0x10 || regs[i].a == 0x11 ||
                            regs[i].a == 0x13 || regs[i].a == 0x14 || regs[i].a == 0x40);
    Serial.printf("  0x%02X %-12s = 0x%08lX%s\n", regs[i].a, regs[i].n,
                  (unsigned long)v, writeOnly ? "   (nur schreibbar - Readback ohne Aussage)" : "");
  }
  Serial.println("--- Schreibtest: GCONF (0x00, R/W) ---");
  {
    TMC2209Diag& d = tmcL;
    const uint32_t g0 = d.read(TMC_REG_GCONF);
    const uint8_t if0 = (uint8_t)d.read(TMC_REG_IFCNT);
    d.write(TMC_REG_GCONF, g0 ^ 0x00000100u);
    const uint32_t g1 = d.read(TMC_REG_GCONF);
    d.write(TMC_REG_GCONF, g0);
    const uint32_t g2 = d.read(TMC_REG_GCONF);
    const uint8_t if1 = (uint8_t)d.read(TMC_REG_IFCNT);
    Serial.printf("  GCONF   : 0x%08lX -> 0x%08lX -> 0x%08lX  %s\n",
                  (unsigned long)g0, (unsigned long)g1, (unsigned long)g2,
                  (g1 != g0) ? "-> SCHREIBEN WIRKT" : "-> wird ignoriert");
    Serial.printf("  Zustand wiederhergestellt: %s\n",
                  (g2 == g0) ? "ja" : "NEIN - bitte neu konfigurieren (TMC:INIT)");
    Serial.printf("  IFCNT   : %u -> %u (zaehlt nur Schreibzugruege; 2 Writes = +2 erwartet)\n",
                  if0, if1);
  }
  Serial.println("======================================");
  Serial.println();
}
