#pragma once

#include <Arduino.h>

constexpr uint8_t MEMORY_MAX_LEN = 16;
constexpr uint8_t MEM_FWD   = 0;
constexpr uint8_t MEM_BACK  = 1;
constexpr uint8_t MEM_LEFT  = 2;
constexpr uint8_t MEM_RIGHT = 3;

extern uint8_t memorySeq[MEMORY_MAX_LEN];
extern uint8_t memoryLen;
extern uint8_t memoryInputIdx;
extern bool    memoryActive;
extern bool    memoryAwaitInput;

void memoryStart(uint8_t startLen = 1);
void memoryStop();
bool memoryInput(uint8_t move);
void memoryTick();
