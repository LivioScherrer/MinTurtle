#pragma once

#include <Arduino.h>
#include <stdint.h>

enum GameMode : uint8_t {
  GAME_NONE = 0,
  GAME_KORRIGIEREN,
  GAME_RENNEN,
  GAME_MEMORY,
  GAME_LINEFOLLOWER,
  GAME_PROG,
  GAME_SLALOM,
  GAME_AMPEL,
  GAME_ZAHL,
  GAME_GEHEIM,
  GAME_PARK,
  GAME_ZEIT,
};
extern GameMode currentGame;

bool driveLockedByGame();

extern const uint32_t GAME_MOVE_MS;
extern const uint32_t GAME_COLOR_READ_MS;

extern bool gameMoveBusy;


void gameDriveForward(uint32_t ms);
void gameDriveBackward(uint32_t ms);

void gameTurnLeftSteps(int32_t steps);
void gameTurnRightSteps(int32_t steps);

void gameDriveForwardSteps(int32_t steps);
void gameDriveBackwardSteps(int32_t steps);

void gameStop(uint32_t ms);
