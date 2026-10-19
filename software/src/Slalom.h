#pragma once

#include <Arduino.h>
#include <EspNow.h>
#include <ProgGame.h>

enum SlalomPhase : uint8_t {
  SLALOM_IDLE       = 0,
  SLALOM_PAIRED     = 1,
  SLALOM_BUILD      = 2,
  SLALOM_COUNTDOWN  = 3,
  SLALOM_RACING     = 4,
  SLALOM_DONE       = 5
};

extern bool slalomRunning;
extern bool slalomFinished;
extern bool slalomWon;
extern bool slalomIWonLast;
extern bool slalomMyRunDone;
extern bool slalomOppRunDone;
extern bool slalomIsHost;
extern bool slalomSoloTime;
extern bool slalomReady;
extern bool slalomOppReady;
extern char  slalomTargetColor[16];
extern char  slalomMyProgram[PROG_MAX_ENC + 1];
extern SlalomPhase slalomPhase;
extern unsigned long slalomGoTimer;
extern bool slalomGoPending;

void slalomSetPhase(SlalomPhase p);
void slalomResetRuntime();

bool slalomHasPeer();
uint8_t* slalomPeerMacPtr();
void slalomPairWith(const uint8_t mac[6], bool initiator);
void slalomUnpair();

void slalomSendPacket(uint8_t cmd, const char* data, bool reliable = false);
void slalomSendPacketTo(uint8_t cmd, const char* data, const uint8_t destMac[6],
                        bool reliable = false);
void onSlalomPacket(const EduPacket* pkt);

void slalomInviteSend(const uint8_t destMac[6]);
void slalomTick();

void slalomStartGame();
void slalomOnStartReceived(const char* col);
void slalomSetProgram(const char* enc);
void slalomMaybeStart();
void slalomExecute();
void slalomStartSoloTime(const char* color);
void slalomExecuteSoloTime();
void slalomCheckRetry();
void slalomEndGame(bool iWon);
void slalomAbortGame(bool notifyPeer = true);

bool slalomInvitePending();
uint8_t* slalomInviterMacPtr();
void slalomClearInvite();
