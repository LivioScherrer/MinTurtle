#include "MemoryGame.h"
#include "GameEngine.h"
#include "Config.h"
#include "BLEManager.h"
#include "Motor.h"
#include "Drive.h"
#include "LedStrip.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr uint32_t MEM_GAP_MS            = 350;
constexpr uint32_t MEM_INPUT_TIMEOUT_MS  = 8000;


uint8_t memorySeq[MEMORY_MAX_LEN];
uint8_t memoryLen = 0;
uint8_t memoryInputIdx = 0;
bool    memoryActive   = false;
bool    memoryAwaitInput = false;
bool    memoryStopReq  = false;
uint32_t memoryInputStart = 0;

void memoryExecuteMove(uint8_t move) {
  switch (move) {
    case MEM_FWD:   gameDriveForwardSteps(configForwardSteps()); break;
    case MEM_BACK:  gameDriveBackwardSteps(configForwardSteps()); break;
    case MEM_LEFT:  gameTurnLeftSteps(configTurnSteps()); break;
    case MEM_RIGHT: gameTurnRightSteps(configTurnSteps()); break;
  }
}

void memoryShowSequence() {
  memoryStopReq = false;
  sendStatusF("MEMORY:SHOW:%u", memoryLen);
  ledStripShowMemoryShow();
  ledStripTick();
  for (uint8_t i = 0; i < memoryLen; i++) {
    if (memoryStopReq) return;
    memoryExecuteMove(memorySeq[i]);
    ledStripTick();
    sendStatusF("MEMORY:SHOW_STEP:%u,%u", i, memorySeq[i]);
    bleProcessEvents();
    vTaskDelay(pdMS_TO_TICKS(MEM_GAP_MS));
    if (memoryStopReq) return;
  }
  memoryAwaitInput = true;
  memoryInputIdx = 0;
  memoryInputStart = millis();
  sendStatus("MEMORY:YOUR_TURN");
}

void memoryStart(uint8_t startLen) {
  if (memoryActive) return;
  if (startLen < 1) startLen = 1;
  if (startLen > MEMORY_MAX_LEN) startLen = MEMORY_MAX_LEN;
  if (startLen > memoryLen && memoryLen > 0) {
    for (uint8_t i = memoryLen; i < startLen; i++) memorySeq[i] = random(0, 4);
    memoryLen = startLen;
  } else {
    memoryLen = startLen;
    for (uint8_t i = 0; i < memoryLen; i++) {
      memorySeq[i] = random(0, 4);
    }
  }
  memoryInputIdx = 0;
  memoryActive = true;
  memoryAwaitInput = false;
  memoryStopReq = false;
  sendStatusF("MEMORY:START:%u", memoryLen);
  Serial.printf("[MEMORY] Start, len=%u\n", memoryLen);
  memoryShowSequence();
}

void memoryStop() {
  if (!memoryActive) return;
  memoryActive = false;
  memoryAwaitInput = false;
  memoryStopReq = true;
  stopMotors();
  speedL = 0; speedR = 0;
  currentGame = GAME_NONE;
  sendStatus("MEMORY:ABORT");
  Serial.println("[MEMORY] Abgebrochen");
}

bool memoryInput(uint8_t move) {
  if (!memoryActive || !memoryAwaitInput) return false;
  if (move > 3) return false;
  uint8_t expected = memorySeq[memoryInputIdx];
  if (move != expected) {
    stopMotors();
    speedL = 0; speedR = 0;
    currentGame = GAME_NONE;
    sendStatusF("MEMORY:WRONG:%u,%u", memoryInputIdx, move);
    memoryActive = false;
    memoryAwaitInput = false;
    ledStripShowResultFlash(false);
    ledStripTick();
    return false;
  }
  memoryAwaitInput = false;
  memoryExecuteMove(move);
  memoryInputIdx++;
  if (memoryInputIdx >= memoryLen) {
    memoryActive = false;
    currentGame = GAME_NONE;
    sendStatusF("MEMORY:WIN:%u", memoryLen);
    ledStripShowResultFlash(true);
    ledStripTick();
    Serial.printf("[MEMORY] Gewonnen mit %u Schritten\n", memoryLen);
    return true;
  }
  memoryAwaitInput = true;
  memoryInputStart = millis();
  sendStatusF("MEMORY:STEP:%u,%u", memoryInputIdx, memoryLen);
  return true;
}

void memoryTick() {
  if (!memoryActive || !memoryAwaitInput) return;
  if (millis() - memoryInputStart > MEM_INPUT_TIMEOUT_MS) {
    stopMotors();
    speedL = 0; speedR = 0;
    memoryActive = false;
    memoryAwaitInput = false;
    currentGame = GAME_NONE;
    sendStatus("MEMORY:TIMEOUT");
    ledStripShowResultFlash(false);
    ledStripTick();
  }
}
