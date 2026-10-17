#pragma once

#include <Arduino.h>
#include <semphr.h>

void i2cBusInit();

bool i2cBusLock(uint32_t timeoutMs = 25);

void i2cBusUnlock();
