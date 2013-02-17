/*
  Copyright (C) 2006 Daniel Broad <daniel@dorada.co.uk>.

   -----------------------------------------------------------------------
	this file is a part of the Lexus iPod Project
 -----------------------------------------------------------------------
 		http://www.lexusipod.com
*/
#include "com232.h"
#include "ipod.h"
#include "avclandrv.h"
#include "inputoutput.h"
#include "delay.h"
#include "const.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

//lingo commands		    Len
const u08 cmLingo[] 	PROGMEM = {0x03, 0x00, 0x01, 0x04};
const u08 cmPlayPause[] PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x01};
const u08 cmstop[]		PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x02};
const u08 cmPollingOn[] PROGMEM = {0x04, 0x04, 0x00, 0x26, 0x01};
const u08 cmSkipTrackF[]PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x03};
const u08 cmSkipTrackR[]PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x04};
const u08 cmFF[]		PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x05};
const u08 cmRR[]		PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x06};
const u08 cmResume[]		PROGMEM = {0x04, 0x04, 0x00, 0x29, 0x07};
const u08 cmPlaylistPos[]	PROGMEM = {0x03, 0x04, 0x00, 0x1E};
const u08 cmExecSwitch[]	PROGMEM = {0x07, 0x04, 0x00, 0x28,0x00,0x00,0x00,0x00};
const u08 cmPlaylistCount[]	PROGMEM = {0x04, 0x04, 0x00, 0x18, 0x01};
const u08 cmGetShuffle[]	PROGMEM = {0x03, 0x04, 0x00, 0x2C};
const u08 cmGetRepeat[]		PROGMEM = {0x03, 0x04, 0x00, 0x2F};
const u08 cmGetTimeStatus[]		PROGMEM = {0x03, 0x04, 0x00, 0x1C};

u08 cmShuffle[]		= {0x04, 0x04, 0x00, 0x2E, 0x00};
u08 cmRepeat[]		= {0x04, 0x04, 0x00, 0x31, 0x00};
u08 cmPlaylistSwitch[] = {0x08, 0x04, 0x00, 0x17, 0x01, 0x00,0x00,0x00,0x00};

//lingo replies
#define rpNumplaylists 		0x19
#define rpTimeStatusInfo 	0x1D
#define rpPlaylistPosition  0x1F
#define rpSongTime 			0x27
#define rpACK				0x01
#define rpTimeStatus		0x1D
#define rpShuffleStatus 	0x2D
#define rpRepeatMode		0x30
//------------------------------------------------------------------------------
void AddBuffer(u08 tmp)
{
	if (RxCharEnd<15) {
		RxBuffer[RxCharEnd] = tmp;		// Store received character 
	};
    RxCharEnd++;
}
//------------------------------------------------------------------------------
void ClearInBuffer()
{
	u08 i;
	for (i=0;i<15;i++)
		{
		RxBuffer[i] = 0x00;
		};
	RxCharEnd=0;
}
//------------------------------------------------------------------------------
#ifndef MONITOR
void RS232_LINGO(u08* pBuf)
{
	// output a lingo message
	register u08 c;
	u08 i = 1;
	while (i<=pBuf[0])
	{
		c = pBuf[i];
		RS232_SendByte(c);
		i++;
	}
}
#endif
//------------------------------------------------------------------------------
#ifdef MONITOR
void RS232_LINGO_TEST(u08* pBuf)
{
	// test output a lingo message to pc on rs232
	register u08 c;
	u08 i = 1;
	while (i<=pBuf[0])
	{
		c = pBuf[i];
		RS232_PrintHex8(c);
		i++;
	}
	RS232_SendByte('\r');
	RS232_SendByte('\n');
}
#endif
//------------------------------------------------------------------------------
#ifndef MONITOR
void RS232_LINGO_P(u08* pBuf)
{
	// output a lingo message
	register u08 c;
	u08 i = 1;
	u08 length = pgm_read_byte(pBuf);
	while (i<=length)
	{
		c = pgm_read_byte(pBuf+i);
		RS232_SendByte(c);
		i++;
	}
}
#endif
//------------------------------------------------------------------------------
#ifdef MONITOR
void RS232_LINGO_TEST_P(u08* pBuf)
{
	// test output a lingo message to pc on rs232
	register u08 c;
	u08 i = 1;
	u08 length = pgm_read_byte(pBuf);
	while (i<=length)
	{
		c = pgm_read_byte(pBuf+i);
		RS232_PrintHex8(c);
		i++;
	}
	RS232_SendByte('\r');
	RS232_SendByte('\n');
}
#endif
//------------------------------------------------------------------------------
void iPod_Poll()
{

	// check if iPod connected, set lingo mode
	if (IPOD_IS_CONNECTED!=iPodConnected)	
		{
			iPodConnected = IPOD_IS_CONNECTED;
			if (iPodConnected)
				{
				iPodStartup = startup_Begin;
				ClearInBuffer();
				RXState = WAITING_FOR_FF;
				iPodMode = stStop;
				AuxInput = Aux2 ;
				SwitchInputs();
				}
			else
				{
				// switchinputs knows its not connected so will switch to aux2
				SwitchInputs();
				}
		};


	if (IPOD_IS_CONNECTED) // iPod initialisation
		{
			switch(iPodStartup)
				{
				case startup_Finish:
					break;
				case startup_Begin:
					delay_ms(50);
					iPod(cmdLingo);
					iPod(cmdPollingOn);
					iPodStartup = waiting_Begin;
					//iPodStartup=startup_Finish;
					break;
				case startup_PlaylistCount: 
					iPodStartup = waiting_PlaylistCount;
					iPod(cmdNumPlaylists);
					break;
				case startup_PlayStatus:
					iPodStartup = waiting_PlayStatus;
					iPod(cmdTimeStatus);
					break;
				case startup_PlaylistPosition:
					iPodStartup = waiting_PlaylistPosition;
					iPod(cmdPlayListPosition);
					break;
				case startup_Shuffle:
					iPodStartup = waiting_Shuffle;
					iPod(cmdGetShuffle);
					break;
				case startup_Repeat:
					iPodStartup = waiting_Repeat;
					iPod(cmdGetRepeat);
					break;
				}

		};

	// if we need to sync car play/pause state to ipod
	if ((CD_Mode != iPodMode) & (AuxInput == Aux1) & (iPodStartup==startup_Finish))
		{
		switch (CD_Mode)	{
			case stStop:
			case stPlay: 
				if ((iPodMode == stFF) | (iPodMode == stRR))
						iPod(cmdResume);
					else
						iPod(cmdPlay);
				break;
			case stFF:
				iPod(cmdFF);
				break;
			case stRR:
				iPod(cmdRR);
				break;
			default:
				break;
			};
		iPodMode = CD_Mode;
		};

	// other input, iPod should be stop
    if (AuxInput == Aux2)
		if (iPodMode != stStop)
			{
			if (iPodConnected) iPod(cmdPlay);
			iPodMode = stStop;
			iPodEvent = evTimeStatus;
			}

	// do iPod events
	if ((IPOD_IS_CONNECTED) & (iPodStartup==startup_Finish))
		{
		#ifdef MONITOR
		if (iPodEvent != evNone)
			{
			RS232_Print("iPod Event:");
			RS232_PrintHex8(iPodEvent);
			RS232_Print("\r\n");
			};
		#endif
		switch (iPodEvent) {
			case evNone:
				break;
			case evPlaylistPosition:
				iPod(cmdPlayListPosition);
				iPodEvent = evNone;
				break;
			case evPlayListOne:
				iPod(cmdPlayListOne);
				iPodEvent = evPlaylistPosition;
				iPodPlaylist = 1;
				iPodMode = stPlay;
				break;
			case evPlaylistCount:
				iPod(cmdNumPlaylists);
				iPodEvent = evPlaylistPosition;
				break;
			case evTimeStatus:
				iPod(cmdTimeStatus);
				iPodEvent = evPlaylistPosition;
			default:
				break;
		};


		}

	// if RX error reset comms.
	if (RXState == RX_ERROR) {
		ClearInBuffer();
		RXState = WAITING_FOR_FF;
	};

	// check to see if we have an ipod message waiting to be processed
	if (RXState == RX_FINISHED) 
		iPod_ProcessResponse();

}

//------------------------------------------------------------------------------
void iPod_Data(u08 tmp) {


	switch (RXState)
		{
		case WAITING_FOR_FF:
			if( tmp == 0xFF )
				RXState = WAITING_FOR_55;
			break;
			
		case WAITING_FOR_55:
			if( tmp == 0x55 )
				RXState = WAITING_FOR_LENGTH;
			else
				{
				if (tmp == 0xFF)	
					RXState = WAITING_FOR_55;
				else
					RXState = RX_ERROR;
				}
			break;
			
		case WAITING_FOR_LENGTH:
			if (tmp > 15)
				{
					RXState = RX_ERROR;
					break;
				}
			CheckSum = tmp;
			Length   = tmp;
			BytesToGo = tmp;
			RXState = GETTING_MESSAGE;
			break;
			
		case GETTING_MESSAGE:
			CheckSum += tmp;
			AddBuffer(tmp);
			if( !--BytesToGo )
				RXState = WAITING_FOR_CHECKSUM;
			break;
			
		case WAITING_FOR_CHECKSUM:
			CheckSum = ( CheckSum ^ 0xff ) + 1;	
			if (tmp == CheckSum)
				{
		        RXState = RX_FINISHED; 
				}
			else
				{
				RXState = RX_ERROR; 
				}
			break;
			
		case RX_FINISHED:	
			break;

		case RX_ERROR:
			break;
			
		default:
			#ifdef MONITOR
			RS232_Print("Error\n");
			#endif
		    RXState = RX_ERROR;
	}
}
//------------------------------------------------------------------------------
void iPod_ProcessResponse()
{ // 04 00 XX ......

uint32_t temp = 0;

	#ifdef MONITOR
		RS232_PrintHex8(RxBuffer[2]);
		RS232_SendByte('\r');
		RS232_SendByte('\n');
	#endif
	#ifndef MONITOR
	last = RxBuffer[2];
	switch (RxBuffer[2])	{
		case rpACK:
			if ((iPodStartup == waiting_Begin) & (RxBuffer[5] == 0x26)) iPodStartup = startup_PlaylistCount;
			break;
		case rpNumplaylists:
			temp = RxBuffer[5];			
			temp = temp << 8;
			temp = RxBuffer[6];
			temp = temp + 1;
			maxDisc = temp;
			if (maxDisc > 99) maxDisc = 99;
			if (AuxInput == Aux1) SetChangerModeForiPod();
			if (iPodStartup == waiting_PlaylistCount) iPodStartup = startup_PlayStatus;
			break;
		case rpSongTime:
			if (RxBuffer[4] == 0xFF)
				{
				// We are not playing
				iPodEvent = evTimeStatus;
				}
			else
				{
				iPod_SetTime(RxBuffer[4], RxBuffer[5], RxBuffer[6], RxBuffer[7]);
				}
			break;
		case rpPlaylistPosition:
			temp = cd_Track;
			if (RxBuffer[3] == 0xFF)
				{
				// We are not playing?
				iPodEvent = evTimeStatus;
				}
			else
				{		
				temp = RxBuffer[4];
				temp = temp << 8;	
				temp += RxBuffer[5];
				temp = temp << 8;
				temp += RxBuffer[6];
				temp++;
				temp = temp % 100;
				if (AuxInput == Aux1)
					cd_Track = temp;
				}
			if (iPodStartup == waiting_PlaylistPosition) iPodStartup = startup_Shuffle;
			break;

		case rpTimeStatus:
			iPod_SetTime(RxBuffer[7], RxBuffer[8], RxBuffer[9], RxBuffer[10]);
			switch (RxBuffer[11])
				{
					case 0x00:
						iPodEvent = evPlayListOne;
						break;
					case 0x01:
						iPodMode = stPlay;
						break;
					case 0x02:
						iPodMode = stStop;
						break;
				};
			if (iPodStartup == waiting_PlayStatus) iPodStartup = startup_PlaylistPosition;
			break;
		case rpShuffleStatus:
			randomMode = RxBuffer[3];
			iPodShuffle = randomMode;
			if (iPodStartup == waiting_Shuffle) iPodStartup = startup_Repeat;
			break;
		case rpRepeatMode:
			repeatMode = RxBuffer[3];
			iPodRepeat = repeatMode;
			if (iPodStartup == waiting_Repeat) iPodStartup = 99; //done
			break;
		default:
			break;
	}	

	#endif
	// reset ready to go again
	ClearInBuffer();
	RXState = WAITING_FOR_FF;
}
//------------------------------------------------------------------------------
void iPod_SendCommand(u08* command, u08 progmem)
{
	u08 CheckSum = 0;
	u08 length;
	u08 i;
		
	if (progmem)
		length = pgm_read_byte(command);
	else
		length = command[0];

	u08 begin[] = {0x03,0xFF,0x55,0x00};
	u08 checksum[] = {0x01, 0xFF};

	begin[3] = length;
	// calc checksum
	for (i=0;i<=length;i++)
		{	
			if (progmem)
				CheckSum += pgm_read_byte(command+i);
			else
				CheckSum += command[i];
		};
	CheckSum = ( CheckSum ^ 0xff ) + 1;	
	checksum[1] = CheckSum;

	#ifdef MONITOR
		{
		RS232_LINGO_TEST((u08*) begin);
		if (progmem)
			RS232_LINGO_TEST_P(command);	
		else
			RS232_LINGO_TEST(command);	

		RS232_LINGO_TEST((u08*) checksum);
		}
	#else
		{
		RS232_LINGO((u08*) begin);
		if (progmem)
			RS232_LINGO_P(command);	
		else
			RS232_LINGO(command);
		RS232_LINGO((u08*) checksum);
		}
	#endif
}
//------------------------------------------------------------------------------
void iPod(u08 command)
{
	#ifdef MONITOR
		{
		RS232_Print("iPod:");
		RS232_PrintHex8(command);
		RS232_Print("\r\n");
		}
	#endif
	switch (command) {
		case cmdLingo:
			iPod_SendCommand((u08*) cmLingo,1);
			break;
		case cmdPlay:
			iPod_SendCommand((u08*) cmPlayPause,1);
			iPodEvent = evTimeStatus;
		 	break;
		case cmdStop:
		 	break;
		case cmdNumPlaylists:
			iPod_SendCommand((u08*) cmPlaylistCount,1);
		 	break;
		case cmdPlaylist:
			if (cd_Disc < maxDisc)
				{
				cmPlaylistSwitch[8] = cd_Disc-1;
				iPod_SendCommand((u08*) cmPlaylistSwitch,0);
				iPod_SendCommand((u08*) cmExecSwitch,1);
				iPodPlaylist = cd_Disc;
			};
			iPodEvent = evPlaylistCount;
			Event |= EV_STATUS;
		 	break;
		case cmdNextTrack:
			iPodMin = cd_Time_Min = 0;
			iPodSec = cd_Time_Sec = 0;
			cd_Track++;
			Event |= EV_STATUS;
			iPod_SendCommand((u08*) cmSkipTrackF,1);
			iPodEvent = evPlaylistPosition;
		 	break;
		case cmdPrevTrack:
			iPodMin = cd_Time_Min = 0;
			iPodSec = cd_Time_Sec = 0;
			if (cd_Track>1) cd_Track--;
			Event |= EV_STATUS;
			iPod_SendCommand((u08*) cmSkipTrackR,1);
			iPodEvent = evPlaylistPosition;
		 	break;
		case cmdNextPlaylist:
		 	break;
		case cmdPrevPlaylist:
		 	break;
		case cmdFF:
			iPod_SendCommand((u08*) cmFF,1);
			break;
		case cmdRR:
			iPod_SendCommand((u08*) cmRR,1);
			break;
		case cmdPlayListPosition:
			iPod_SendCommand((u08*) cmPlaylistPos,1);
			break;
		case cmdPlayListOne:
			cmPlaylistSwitch[8] = 0x00;
			iPod_SendCommand((u08*) cmPlaylistSwitch,0);
			iPod_SendCommand((u08*) cmExecSwitch,1);
			cd_Disc = 1;
			iPodPlaylist = cd_Disc;
			break;
		case cmdShuffle:
			cmShuffle[4] = randomMode;
			iPod_SendCommand((u08*) cmShuffle,0);
			break;
		case cmdRepeat:
			cmRepeat[4] = repeatMode;
			iPod_SendCommand((u08*) cmRepeat,0);
			break;
		case cmdResume:
			iPod_SendCommand((u08*) cmResume,1);
			break;
		case cmdTimeStatus:
			iPod_SendCommand((u08*) cmGetTimeStatus,1);
			break;
		case cmdGetShuffle:
			iPod_SendCommand((u08*) cmGetShuffle,1);
			break;
		case cmdGetRepeat:
			iPod_SendCommand((u08*) cmGetRepeat,1);
			break;
		case cmdPollingOn:
			iPod_SendCommand((u08*) cmPollingOn,1);	
		default:
			break;
	}
}

u08 BCDu08 (u08 inbyte)
{
u08 units = (inbyte % 10);
u08 tens = ((inbyte) / 10);
    return (tens * 16) + units;
}

void iPod_Error()
{
	RXState = RX_ERROR;
}

void iPod_Init()
{
	RXState = WAITING_FOR_FF;	
	iPodPlaylist = 1;
	iPodTrack = 1;
 	AuxInput=Aux2;
 	SwitchInputs();
	//iPodConnected = 99;
}

void iPod_SetTime(u08 b1, u08 b2, u08 b3, u08 b4)
				{
#ifndef MONITOR
				uint32_t temp = 0;
				if (AuxInput == Aux2) return;
				temp = b1;
				temp = temp << 8;
				temp = b2;
				temp = temp << 8;
				temp += b3;
				temp = temp << 8;
				temp += b4;
				temp = temp / (uint32_t) 1000;
				cd_Time_Min = BCDu08((temp / (uint32_t)60) % (uint32_t)100);
				iPodMin = cd_Time_Min;
				u08 secs = BCDu08(temp % (uint32_t)60);
				if (cd_Time_Sec != secs)
					{
					cd_Time_Sec = secs;
					iPodEvent = evPlaylistPosition;
					iPodSec = cd_Time_Sec;
					Event |= EV_STATUS;
					iPodMode = stPlay;
					};
#endif
	};

void iPodSetRandom()
{				
if (randomMode != iPodShuffle)
		{
		iPod(cmdShuffle);
		iPodShuffle = randomMode;
		}
};

void iPodSetRepeat()
{
	if (repeatMode != iPodRepeat)
		{
		iPod(cmdRepeat);
		iPodRepeat = repeatMode;
		};
}
