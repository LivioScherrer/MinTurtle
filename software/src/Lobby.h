#pragma once

#include <Arduino.h>
#include <EspNow.h>

enum LobbyPhase : uint8_t {
  LOBBY_IDLE     = 0,
  LOBBY_INVITING = 1,
  LOBBY_INVITED  = 2,
  LOBBY_JOINED   = 3
};

extern LobbyPhase lobbyPhase;
extern bool lobbyIsHost;

const char* lobbyPhaseName();

bool lobbyHasPeer();
uint8_t* lobbyPeerMacPtr();
uint8_t* lobbyInviterMacPtr();

void lobbySendPacket(uint8_t cmd, const char* data, bool reliable);
void lobbyOnPacket(const EduPacket* pkt);

void lobbyInviteSend(const uint8_t destMac[6]);
void lobbyTick();

bool lobbyGameBusy();

bool lobbyCmd(const String& cmd);
