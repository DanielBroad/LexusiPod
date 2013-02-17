#ifndef __IPOD_H
#define __IPOD_H

#include "avclandrv.h"
/*
  Copyright (C) 2006 Daniel Broad <daniel@dorada.co.uk>.

   -----------------------------------------------------------------------
	this file is a part of the Lexus iPod Project
 -----------------------------------------------------------------------
 		http://www.lexusipod.com
*/

void RS232_LINGO(u08* pBuf);
void RS232_LINGO_TEST(u08* pBuf);
void iPod_Data(u08 tmp);
void iPod_ProcessResponse();
void iPod(u08 command);
void iPod_Poll();
void iPod_Error();
u08 BCDu08 (u08 inbyte);
void iPod_Init();
void iPod_SetTime(u08 b1, u08 b2, u08 b3, u08 b4);
void iPodSetRandom();
void iPodSetRepeat();

u08 iPodConnected;

volatile cd_modes iPodMode;

u08 iPodShuffle;
u08 iPodRepeat;
u08 iPodPlaylist;
u08 iPodTrack;
u08 iPodMin;
u08 iPodSec;

// receive state machine
typedef enum { DUMMY, WAITING_FOR_FF, WAITING_FOR_55, WAITING_FOR_LENGTH, GETTING_MESSAGE, WAITING_FOR_CHECKSUM, RX_FINISHED, RX_ERROR } ReceiveState;
ReceiveState RXState;

u08	CheckSum;
u08 Length;
u08 BytesToGo;

u08 RxBuffer[15], RxCharEnd;

u08 iPodEvent;

#define evNone 0
#define evPlaylistPosition 1
#define evPlayListOne 2
#define evPlaylistCount 3
#define evShuffle 4
#define evRepeat 5
#define evTimeStatus 6

#define cmdLingo 0
#define cmdPlay 1
#define cmdPause 2
#define cmdStop 3
#define cmdNumPlaylists 4
#define cmdPlaylist 5
#define cmdNextTrack 6
#define cmdPrevTrack 7
#define cmdNextPlaylist 8
#define cmdPrevPlaylist 9
#define cmdFF 10
#define cmdRR 11
#define cmdPlayListPosition 12
#define cmdPlayListOne 13
#define cmdShuffle 14
#define cmdRepeat 15
#define cmdResume 16
#define cmdTimeStatus 17
#define cmdGetShuffle 18
#define cmdGetRepeat 19
#define cmdPollingOn 20

#define startup_Begin 00
#define startup_PlaylistCount 01
#define startup_PlayStatus 02
#define startup_PlaylistPosition 03
#define startup_Shuffle 04
#define startup_Repeat 05

#define waiting_Begin 10
#define waiting_PlaylistCount 11
#define waiting_PlayStatus 12
#define waiting_PlaylistPosition 13
#define waiting_Shuffle 14
#define waiting_Repeat 15

#define startup_Finish 99
u08 iPodStartup;
u08 last;

#endif
