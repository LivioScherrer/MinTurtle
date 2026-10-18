#pragma once

#include <Arduino.h>
#include "Config.h"

#define PIN_STEP_L   2
#define PIN_DIR_L    1
#define PIN_STEP_R   7
#define PIN_DIR_R    3
#define PIN_EN       4

#define TMC_UART_RX  44
#define TMC_UART_TX  43
#define TMC_BAUD     115200
#define TMC_R_SENSE  0.56f
#define TMC_ADDR_L   0b01
#define TMC_ADDR_R   0b00

extern float FIXED_DRIVE_SPEED;

constexpr uint32_t MOTOR_TICK_US = 50;


constexpr uint16_t TMC_CURRENT_MA_FIXED   = 200;
constexpr uint8_t  TMC_IHOLDDELAY          = 4;
constexpr uint16_t TMC_MICROSTEPS_DEFAULT = 16;
constexpr uint8_t  TMC_CFG_WRITE_COUNT     = 9;

constexpr uint8_t TMC_REG_GCONF       = 0x00;
constexpr uint8_t TMC_REG_GSTAT       = 0x01;
constexpr uint8_t TMC_REG_IFCNT       = 0x02;
constexpr uint8_t TMC_REG_NODECONF    = 0x03;
constexpr uint8_t TMC_REG_IHOLD_IRUN   = 0x10;
constexpr uint8_t TMC_REG_IOIN        = 0x06;
constexpr uint8_t TMC_REG_TPOWER      = 0x11;
constexpr uint8_t TMC_REG_TSTEP       = 0x12;
constexpr uint8_t TMC_REG_TPWMTHRS    = 0x13;
constexpr uint8_t TMC_REG_TCOOLTHRS   = 0x14;
constexpr uint8_t TMC_REG_VACTUAL     = 0x22;
constexpr uint8_t TMC_REG_SGTHRS      = 0x40;
constexpr uint8_t TMC_REG_SG_RESULT   = 0x41;
constexpr uint8_t TMC_REG_COOLCONF    = 0x42;
constexpr uint8_t TMC_REG_MSCNT       = 0x6A;
constexpr uint8_t TMC_REG_DRV_STATUS  = 0x6F;
constexpr uint8_t TMC_REG_CHOPCONF    = 0x6C;
constexpr uint8_t TMC_REG_PWMCONF     = 0x70;

constexpr uint32_t GCONF_EN_SPREADCYCLE  = 1u << 2;
constexpr uint32_t GCONF_PDN_DISABLE     = 1u << 6;
constexpr uint32_t GCONF_MSTEP_REG_SELECT= 1u << 7;
constexpr uint32_t GCONF_MULTISTEP_FILT  = 1u << 8;
constexpr uint32_t GCONF_MASK            = 0x3FF;

constexpr uint32_t CHOP_MRES_SHIFT = 24;
constexpr uint32_t CHOP_MRES_MASK  = 0x0F000000u;
constexpr uint32_t CHOP_INTPOL     = 1u << 28;
constexpr uint32_t CHOP_TOFF_SHIFT = 0;
constexpr uint32_t CHOP_TOFF_MASK  = 0x00000007u;
constexpr uint32_t CHOP_HSTRT_SHIFT = 4;
constexpr uint32_t CHOP_HSTRT_MASK  = 0x00000070u;
constexpr uint32_t CHOP_HEND_SHIFT  = 7;
constexpr uint32_t CHOP_HEND_MASK   = 0x00000780u;
constexpr uint32_t CHOP_TBL_SHIFT   = 15;
constexpr uint32_t CHOP_TBL_MASK    = 0x00018000u;
constexpr uint8_t  CHOP_HSTRT_DEFAULT = 5;
constexpr uint8_t  CHOP_HEND_DEFAULT  = 0;
constexpr uint8_t  CHOP_TBL_DEFAULT   = 1;
constexpr uint32_t PWMCONF_VALUE = (36u)
  | (0u << 8) | (1u << 16)
  | (1u << 18) | (1u << 19)
  | (8u << 24) | (12u << 28);

constexpr uint32_t DRVST_OTPW      = 1u << 0;
constexpr uint32_t DRVST_OT        = 1u << 1;
constexpr uint32_t DRVST_S2GA      = 1u << 2;
constexpr uint32_t DRVST_S2GB      = 1u << 3;
constexpr uint32_t DRVST_S2VSA     = 1u << 4;
constexpr uint32_t DRVST_S2VSB     = 1u << 5;
constexpr uint32_t DRVST_OLA       = 1u << 6;
constexpr uint32_t DRVST_OLB       = 1u << 7;
constexpr uint32_t DRVST_STEALTH   = 1u << 30;
constexpr uint32_t DRVST_STST      = 1u << 31;
constexpr uint32_t DRVST_CS_ACTUAL_SHIFT = 19;
constexpr uint32_t DRVST_CS_ACTUAL_MASK  = 0xFFu;

constexpr float   TMC_SPREADCYCLE_AB_MM_S = 60.0f;

constexpr uint8_t  TMC_SGTHRS_DEFAULT    = 100;
constexpr float   TMC_STALLGUARD_AB_MM_S = 15.0f;

constexpr float   MOT_DIAG_MIN_STEPS_S  = 20.0f;
constexpr float   MOT_DIAG_MAX_STEPS_S  = 1500.0f;

constexpr uint16_t DIAG_SG_DIFF_MIN     = 100;
constexpr uint8_t  DIAG_VACTUAL_DIFF_PCT = 8;
constexpr uint8_t  DIAG_TSTEP_DIFF_PCT  = 5;
constexpr uint8_t  DIAG_CS_DIFF_PCT     = 15;
constexpr uint16_t DIAG_MSCNT_TOLERANCE = 2;

constexpr uint32_t TMC_STANDBY_MS = 3000;

constexpr uint32_t MOT_ALIGN_HOLD_MS = 10000;
bool motorAlignRequest();

constexpr uint32_t R9_SERIES_OHM = 1000;

void initSteppers();

void motorUpdateSpeeds();

bool motorRampActive();

void motorRampSpeeds(float& outL, float& outR);
void motorStepPositions(long& outL, long& outR);

void startupSpin();

void setMotorSpeed(int motorIdx, float speed, bool useRamp = true);

void setMotorSpeeds(float speedL, float speedR, bool useRamp = true);

bool motorSyncMicrosteps();
constexpr uint32_t MOTOR_SYNC_MAX_MS   = 250;
constexpr uint16_t MOTOR_SYNC_MAX_STEPS = 128;
constexpr uint16_t MOTOR_SYNC_TOLERANCE = 1;
void stopMotors();

void motorEmergencyStop();
void motorReleaseEmergency();
bool motorEmergencyActive();

void prepPositionMove(int motorIdx, long physicalSteps);
void prepPositionMoves(long stepsL, long stepsR);

bool motorStillRunning();

void motorWakeDrivers();


void tmcEnable();
void tmcDisable();
bool tmcIsEnabled();
void tmcAutoStandbyTick();
void tmcBatteryGuardTick();
bool tmcBatteryLow();
void tmcReinit();

bool tmcSetMicrosteps(uint16_t ms);
void tmcSetSpreadCycle(bool spread);
bool tmcSpreadCycle();
uint16_t tmcGetCurrent();
uint8_t tmcGetHold();
uint16_t tmcGetMicrosteps();
bool tmcMicrostepsConfirmed();

void tmcPrintStatus();
void tmcSendCompactStatus();
void tmcBootDiag();
void tmcRegDump();
bool motorDiagRequest();
bool motorDiagRunning();
void motorDiagTick();
bool motorDiagSteadySpeed(bool verbose = true);
void tmcNetDiag();
