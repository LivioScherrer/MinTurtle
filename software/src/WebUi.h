#pragma once

#include <Arduino.h>

void webUiSetup();

void webUiStart();

void webUiStop();

bool webUiIsUp();

uint32_t webUiUpFor();

void webUiFlush();

void webUiBroadcast(const char* line);

bool webUiClientConnected();

uint8_t webUiClientCount();

String webUiEnsurePassword();

String webUiPassword();

String webUiResetPassword();

bool webUiAutoEnabled();

bool webUiSetAutoEnabled(bool enabled);

String webUiSsid();

void webUiPrintDiag();