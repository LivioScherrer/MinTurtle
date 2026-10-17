#include "LineFollower.h"
#include "Motor.h"
#include "ColorSensor.h"
#include "Config.h"
#include "Drive.h"
#include "BLEManager.h"
#include "DistanceSensor.h"

bool lineFollowerActive = false;

namespace {
  enum LineState : uint8_t {
    LS_IDLE = 0,
    LS_FORWARD,
    LS_LOST,
    LS_SEARCH_L,
    LS_SEARCH_R
  };
  LineState lsState = LS_IDLE;

  uint32_t lsStateEnterMs = 0;
  uint32_t lsLastSeenMs = 0;

  const float LINE_FOLLOW_SPEED = 380.0f;
  const float LINE_SEARCH_SPEED = 320.0f;
  const uint32_t LINE_GRACE_MS = 80;
  const uint32_t LINE_SEARCH_TURN_MS = 140;
  const uint8_t  LINE_SEARCH_MAX_TRIES = 4;
  uint8_t lsSearchTries = 0;

  bool lineDetected = false;
  bool    lsStoppedForObstacle = false;
  LineState lsResumeState = LS_FORWARD;

  void changeState(LineState next) {
    lsState = next;
    lsStateEnterMs = millis();
  }

  bool readLineDetected() {
    char nm[16] = "UNBEKANNT";
    getCurrentColorName(nm, sizeof(nm));
    if (strcasecmp(nm, "WEISS") == 0)    return false;
    if (strcasecmp(nm, "UNBEKANNT") == 0) return false;
    if (strcasecmp(nm, "BODEN") == 0)    return false;
    return true;
  }

  void motorsForward(float v) {
    setMotorSpeeds(v, v, false);
  }
  void motorsTurnLeft(float v) {
    setMotorSpeeds(-v, v, false);
  }
  void motorsTurnRight(float v) {
    setMotorSpeeds(v, -v, false);
  }
  void motorsOff() {
    setMotorSpeeds(0, 0, false);
  }
}

LedLineState lineFollowerLedState() {
  if (!lineFollowerActive) return LED_LINE_IDLE;
  switch (lsState) {
    case LS_IDLE:    return LED_LINE_OBSTACLE;
    case LS_FORWARD: return LED_LINE_FOLLOW;
    default:         return LED_LINE_SEARCH;
  }
}

void lineFollowerStart() {
  if (lineFollowerActive) return;
  lineFollowerActive = true;
  lsSearchTries = 0;
  lsStoppedForObstacle = false;
  lsLastSeenMs = millis();
  lineDetected = readLineDetected();
  if (lineDetected) {
    motorsForward(LINE_FOLLOW_SPEED);
    changeState(LS_FORWARD);
  } else {
    motorsOff();
    changeState(LS_LOST);
  }
  Serial.println("[LINE] Folge starten");
}

void lineFollowerStop() {
  if (!lineFollowerActive) return;
  motorsOff();
  uint32_t start = millis();
  while (motorStillRunning() && millis() - start < 800) {
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  lineFollowerActive = false;
  lsState = LS_IDLE;
  lsSearchTries = 0;
  lsStoppedForObstacle = false;
  Serial.println("[LINE] Gestoppt");
}

void lineFollowerTick() {
  if (!lineFollowerActive) return;

  lineDetected = readLineDetected();
  uint32_t now = millis();

  if (distBlocked()) {
    motorsOff();
    if (lsState != LS_IDLE) {
      lsStoppedForObstacle = true;
      lsResumeState = lsState;
    }
    changeState(LS_IDLE);
    return;
  }
  if (lsState == LS_IDLE) {
    if (lsStoppedForObstacle) {
      lsStoppedForObstacle = false;
      changeState(lsResumeState);
    } else {
      changeState(LS_FORWARD);
    }
  }

  if (lineDetected) {
    lsLastSeenMs = now;
    lsSearchTries = 0;
  }

  switch (lsState) {
    case LS_IDLE:
      motorsOff();
      break;

    case LS_FORWARD: {
      if (lineDetected) {
        motorsForward(LINE_FOLLOW_SPEED);
      } else {
        if (now - lsLastSeenMs > LINE_GRACE_MS) {
          motorsOff();
          lsSearchTries = 0;
          changeState(LS_LOST);
        }
      }
      break;
    }

    case LS_LOST: {
      if (now - lsStateEnterMs > 60) {
        lsSearchTries = 0;
        motorsTurnLeft(LINE_SEARCH_SPEED);
        changeState(LS_SEARCH_L);
      }
      break;
    }

    case LS_SEARCH_L: {
      if (lineDetected) {
        motorsForward(LINE_FOLLOW_SPEED);
        changeState(LS_FORWARD);
      } else if (now - lsStateEnterMs > LINE_SEARCH_TURN_MS) {
        lsSearchTries++;
        if (lsSearchTries >= LINE_SEARCH_MAX_TRIES) {
          motorsTurnRight(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_R);
        } else {
          motorsTurnLeft(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_L);
        }
      }
      break;
    }

    case LS_SEARCH_R: {
      if (lineDetected) {
        motorsForward(LINE_FOLLOW_SPEED);
        changeState(LS_FORWARD);
      } else if (now - lsStateEnterMs > LINE_SEARCH_TURN_MS) {
        lsSearchTries++;
        if (lsSearchTries >= LINE_SEARCH_MAX_TRIES * 2) {
          motorsOff();
          lineFollowerActive = false;
          lsState = LS_IDLE;
          lsStoppedForObstacle = false;
          Serial.println("[LINE] Linie verloren - Stopp");
        } else {
          motorsTurnRight(LINE_SEARCH_SPEED);
          changeState(LS_SEARCH_R);
        }
      }
      break;
    }
  }
}
