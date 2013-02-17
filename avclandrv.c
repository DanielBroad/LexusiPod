/*
  Copyright (C) 2006 Marcin Slonicki <marcin@softservice.com.pl>.


  Some parts modify from original file
  written by:	Vitaliy Koutchaev <vitat@mail.spbnit.ru>
  - part of the IP-Bus emu system.


  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


	This file is part of
	TOYOTA AVC-Lan CD-Changer Emulator
*/

#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#include "timer.h"
#include "const.h"
#include "avclandrv.h"
#include "delay.h"
#include "com232.h"
#include "sniffit.h"
#include "inputoutput.h"
#include "ipod.h"
//------------------------------------------------------------------------------

u08 parity_bit;

u08 scanMode;

u08 retries;

// cd changer commands

//const u08 stat1[] PROGMEM 		= { 0x4,	0x00, 0x00, 0x01, 0x0A };
const u08 stat2[] PROGMEM		= { 0x4,	0x00, 0x00, 0x01, 0x08 }; // diag memory
const u08 stat3[] PROGMEM	    = { 0x4,	0x00, 0x00, 0x01, 0x09 }; // clear diag
const u08 stat4[] PROGMEM		= { 0x3,	0x00, 0x01, 0x0C }; // sys check
const u08 play_req1[]  PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0x80 };

u08 play_req2[]	= { 0x5,	0x00, 0xFF, 0x63, 0x42, 0x01};  //version dependant
u08 play_req3[]	= { 0x5,	0x00, 0xFF, 0x63, 0x42, 0x41}; //version dependant
u08 stop_req[]	= { 0x5,	0x00, 0xFF, 0x63, 0x43, 0x01 }; //version dependant
u08 stop_req2[]	= { 0x5,	0x00, 0xFF, 0x63, 0x43, 0x41 }; //version dependant

const u08 PROGMEM next_track[] 	= { 0x4,	0x00, 0x25, 0x63, 0x94 };
const u08 PROGMEM prev_track[] 	= { 0x4,	0x00, 0x25, 0x63, 0x95 };

const u08 next_cd[]	PROGMEM		= { 0x4,	0x00, 0x25, 0x63, 0x90 };
const u08 prev_cd[]	PROGMEM		= { 0x4,	0x00, 0x25, 0x63, 0x91 };

const u08 CD[] PROGMEM		= { 0x4,	0x00, 0x25, 0x63, 0x92};

const u08 fast_forward[] PROGMEM= { 0x4,	0x00, 0x25, 0x63, 0x98 };
const u08 fast_back[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0x99 };

const u08 scan_on[]	PROGMEM		= { 0x4,	0x00, 0x25, 0x63, 0xA6 };
const u08 scan_off[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xA7 };
const u08 scan_d_on[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xA9 };
const u08 scan_d_off[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xAA };

const u08 repeat_on[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xA0 };
const u08 repeat_off[] PROGMEM 	= { 0x4,	0x00, 0x25, 0x63, 0xA1 };
const u08 repeat_d_on[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xA3 };
const u08 repeat_d_off[] PROGMEM= { 0x4,	0x00, 0x25, 0x63, 0xA4 };

const u08 random_on[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xB0 };
const u08 random_off[] PROGMEM	= { 0x4,	0x00, 0x25, 0x63, 0xB1 };
const u08 random_d_on[]	PROGMEM = { 0x4,	0x00, 0x25, 0x63, 0xB3 };
const u08 random_d_off[]PROGMEM = { 0x4,	0x00, 0x25, 0x63, 0xB4 };

const u08 trackdiscstatus[] PROGMEM=    {0x4,     	0x00, 0x31, 0x63, 0xE2 };
const u08 discstatus[] PROGMEM		= 	{0x4,     	0x00, 0x31, 0x63, 0xE0 };
const u08 broadcastyourstatus[] PROGMEM={0x4,     	0x00, 0x12, 0x63, 0x8E };
const u08 changerstatus2[] PROGMEM=     {0x4,     	0x00, 0x31, 0x63, 0xE4 };

const u08 aslon[] PROGMEM 		= { 0x4,    0x00, 0x25,0x74,0xB0 };
const u08 asloff[] PROGMEM 		= { 0x4,    0x00, 0x25,0x74,0xB0 };

// broadcast
const u08 lan_stat1[] PROGMEM	= { 0x3,	0x00, 0x01, 0x0A };
const u08 lan_reg_11[] PROGMEM	= { 0x3,	0x11, 0x01, 0x00 };
const u08 lan_init_11[] PROGMEM	= { 0x3,	0x11, 0x01, 0x01 };
const u08 lan_reg_12[] PROGMEM	= { 0x3,	0x12, 0x01, 0x00 };
const u08 lan_init_12[] PROGMEM	= { 0x3,	0x12, 0x01, 0x01 };
u08 lan_check[] = { 0x3,	0xFF, 0x01, 0x20 }; //version dependent
u08 lan_check2[] = { 0x4,	0x00, 0xFF, 0x01, 0x20 }; //version dependent
u08 lan_playit[] = { 0x4,	0xFF, 0x01, 0x45, 0x63 };;//version dependant

u08 lan_registration[] = {0x04, 0x00, 0x01, 0xFF, 0x10};

// answers
u08	CMD_REGISTER[]	= {0x1,		0x05,	0x00, 0x01,	0xFF, 0x10, 0x63 }; //version dependant
u08	CMD_PLAY_OK1[]	= {0x1,		0x05,	0x00, 0x63, 0xFF, 0x50, 0x01 }; //version dependant
u08	CMD_PLAY_OK2[]	= {0x1,		0x05,	0x00, 0x63, 0xFF, 0x52, 0x01 }; //version dependant
u08	CMD_STOP1[]		= {0x1,		0x05,	0x00, 0x63, 0xFF, 0x53, 0x01 }; //version dependant

//u08	CMD_EJECTED[]	= {0x1,		0x05,	0x00, 0x63, 0xFF, 0x51, 0x01 }; //version dependant

const u08	CMD_STATUS2[] 	= {0x1,		0x04,	0x00, 0x01, 0x00, 0x18 }; // diag memory clear
const u08	CMD_STATUS3[] 	= {0x1,		0x04,	0x00, 0x01, 0x00, 0x19 }; 
const u08	CMD_STATUS4[] 	= {0x1,		0x05,	0x00, 0x01, 0x00, 0x1C, 0x00 }; //  sys check

u08			CMD_CHECK[]		= {0x1,		0x06, 0x00, 0x01, 0x12, 0x30, 0x05, 0x00 };
const u08   CMD_NOCART[] 	= {0x0,		0x04, 	0x63, 0x31, 0x9F, 0x00};

u08   CMD_DISCS_INCHANGER_BC[] 	= { 0x0, 	0x0A, 0x63, 0x31, 0xF3, 0x00, 0x3F, 0x00, 0x3F, 0x00, 0x3F, 0x05};
u08   CMD_DISCS_INCHANGER_DR[] 	= { 0x2, 	0x0B, 0x00, 0x63, 0x31, 0xF3, 0x00, 0x3F, 0x00, 0x3F, 0x00, 0x3F, 0x05};

//------------------------------------------------------------------------------
void Set_AVC_Version(u08 *cmd,u08 avcversion,int i)
{
 cmd[i] = avcversion;
 return; 
}
void Init_Commands(u08 avcversion)
{
Set_AVC_Version(lan_check, avcversion,1);
Set_AVC_Version(lan_check2, avcversion,2);
Set_AVC_Version(lan_playit, avcversion,1);
Set_AVC_Version(play_req2, avcversion,2);
Set_AVC_Version(play_req3, avcversion,2);
Set_AVC_Version(stop_req, avcversion,2);
Set_AVC_Version(stop_req2, avcversion,2);
Set_AVC_Version(CMD_PLAY_OK1, avcversion,4);
Set_AVC_Version(CMD_PLAY_OK2, avcversion,4);
Set_AVC_Version(CMD_STOP1, avcversion,4);
Set_AVC_Version(lan_registration, avcversion,3);
}
//------------------------------------------------------------------------------
void AVCLan_Init()
{

 // AVCLan TX+/TX-		write line OUTPUT
 sbi(DATAOUT_DDR,  DATAOUT);
 sbi(DATAOUT_PORT, DATAOUT);

 answerReq   = cmNull;
 cd_Disc     = 1;
 cd_Track    = 99;
 CD_Mode	 = stStop;

 masterDevice1 = eeprom_read_byte(EEPROM_MasterDevice1);
 masterDevice2 = eeprom_read_byte(EEPROM_MasterDevice2);
 Init_Commands(eeprom_read_byte(EEPROM_AVCVersion));
 avcversion = eeprom_read_byte(EEPROM_AVCVersion);
}
//------------------------------------------------------------------------------
u08 AVCLan_Read_Byte(u08 length)
{
 u08 byte = 0;
 u08 wT;
 
 while (1) {
   while (INPUT_IS_CLEAR);
   timer0_start();
   while (INPUT_IS_SET);
   wT = inp(TCNT0);
   if (wT<8) { 
      byte++;
	  parity_bit++;
   }
   length--;
   if (!length) return byte;
   byte = byte << 1;
 } 
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_StartBit()
{
 cbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to output and this be = 1 on IP-Bus
 delay1(166);					// 141-192
 sbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to input
 delay1(30);					// 02-57

 if (INPUT_IS_CLEAR) return 1; else return 0; // 0  = collision!!
}
//------------------------------------------------------------------------------
void AVCLan_Send_Bit1()
{
 cbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to output and this be = 1 on IP-Bus
 delay1(20);					// 17-23

 sbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to input
 delay1(16);					// 12-21
}
//------------------------------------------------------------------------------
void AVCLan_Send_Bit0()
{
 cbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to output and this be = 1 on IP-Bus
 delay1(32);					// 28-37
	
 sbi(DATAOUT_PORT, DATAOUT);	// DATA_OUT pin set to input
 delay1(4);						// 00-09
}
//------------------------------------------------------------------------------
u08 AVCLan_Read_ACK()
{
 u08 time = 0;
 cbi(DATAOUT_PORT, DATAOUT);		// DATA_OUT pin set to output and this be = 1 on IP-Bus
 delay1(19);
 sbi(DATAOUT_PORT, DATAOUT);		// DATA_OUT pin set to input
 timer0_source(CK64);
 timer0_start();
 while(1) {
	time = inp(TCNT0);
	if (INPUT_IS_SET && (time > 1)) break;
	if (time > 5) return 1;
 }
	
 while(INPUT_IS_SET);
 return 0;
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_ACK()
{
 timer0_source(CK64);				//update every 1us
 timer0_start();
 while (INPUT_IS_CLEAR)	{
 	if (inp(TCNT0) >= 25) return 0;	// max wait time
 }
 cbi(DATAOUT_PORT, DATAOUT);		// DATA_OUT pin set to output and this be = 1 on IP-Bus
 delay1(32);						//28-37
 sbi(DATAOUT_PORT, DATAOUT);		// DATA_OUT pin set to input
 delay1(4);							//00-09
 return 1;
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_Byte(u08 byte, u08 len)
{
 u08 b;
 if (len==8) {
 	b = byte;
 } else {
    b = byte << (8-len);
 }

 while (1) {
   if ( b & 128 ) {
     AVCLan_Send_Bit1();
	 parity_bit++;
   } else { 
   	 AVCLan_Send_Bit0();
   }
   len--;
   if (!len) { 
     return 1;
   }
   b = b << 1;
 } 

}
//------------------------------------------------------------------------------
u08 AVCLan_Send_ParityBit()
{
 if ( parity_bit & 1 ) {
     AVCLan_Send_Bit1();
	 parity_bit++;
 } else {
   	 AVCLan_Send_Bit0();
 }
 parity_bit=0;
 return 1;
}
//------------------------------------------------------------------------------
u08 CheckCmd(u08 *cmd)
{
 u08 i;
 u08 *c;
 u08 l;

 c = cmd;
 l = *c++;

 for (i=0; i<l; i++) {
 	if (message[i] != *c) return 0;
	c++;
 }
 return 1;
}

u08 CheckCmd_P(u08 *cmd)
{
 u08 i;
 u08 *c;
 u08 l;

 c = cmd;
 l = pgm_read_byte(c++);

 for (i=0; i<l; i++) {
 	if (message[i] != pgm_read_byte(c)) return 0;
	c++;
 }
 return 1;
}

//------------------------------------------------------------------------------
u08 AVCLan_Read_Message()
{

 u16 T = 0;

 u08 i;
 u08 for_me = 0;

 STOPEvent;					// disable timer1 interrupt

 timer0_source(CK64);

 // check start bit
 timer0_start();
 while (INPUT_IS_SET) { 
 	T=inp(TCNT0);
	if (T>254) {
		STARTEvent;
		return 0;
	}
 }
 if (T<20) {//(T<20) {
 	STARTEvent;
	return 0;
 }
 
 broadcast = AVCLan_Read_Byte(1);

 parity_bit = 0;
 master1 = AVCLan_Read_Byte(4);
 master2 = AVCLan_Read_Byte(8);
 if ((parity_bit&1)!=AVCLan_Read_Byte(1)) {
	STARTEvent;
	return 0;
 }

 parity_bit = 0;
 slave1 = AVCLan_Read_Byte(4);
 slave2 = AVCLan_Read_Byte(8);
 if ((parity_bit&1)!=AVCLan_Read_Byte(1)) {
	STARTEvent;
	return 0;
 }
 // is this command for me ?
 if ((slave1==MY_ID_1)&&(slave2==MY_ID_2)) {
 	for_me=1;
 }

 if ((registered) & (!for_me) & (broadcast))
 	{
 	STARTEvent;
	return 0;
  }

 if (for_me) AVCLan_Send_ACK();
 		else AVCLan_Read_Byte(1);


 parity_bit = 0;
 AVCLan_Read_Byte(4);	// control - always 0xF
 if ((parity_bit&1)!=AVCLan_Read_Byte(1)) {
	STARTEvent;
	return 0;
 }
 if (for_me) AVCLan_Send_ACK();
 		else AVCLan_Read_Byte(1);

 parity_bit = 0;
 message_len = AVCLan_Read_Byte(8);
 if ((parity_bit&1)!=AVCLan_Read_Byte(1)) {
	STARTEvent;
	return 0;
 }
 if (for_me) AVCLan_Send_ACK();
 		else AVCLan_Read_Byte(1);

 if (message_len > MAXMSGLEN) {
	STARTEvent;
	return 0;
 }

 for (i=0; i<message_len; i++) {
	parity_bit = 0;
 	message[i] = AVCLan_Read_Byte(8);
	if ((parity_bit&1)!=AVCLan_Read_Byte(1)) {
		STARTEvent;
		return 0;
 	}
	if (for_me) {
		AVCLan_Send_ACK();
 	} else {
		AVCLan_Read_Byte(1);
	}
 }


 STARTEvent;

 #ifdef MONITOR
 ShowInMessage();
 #endif

 if (for_me) {

 	if (CheckCmd_P((u08*)CD))		{ 
			cd_Time_Min = 0;
			cd_Time_Sec = 0;
			cd_Track=1;
			cd_Disc=message[4];
			if ((AUX1ISIPOD) & (AuxInput == Aux1))
				iPod(cmdPlaylist);
			Event |= EV_STATUS;
			return 1; 
			}

	if (CheckCmd_P((u08*)next_track))		{ 
			if ((AUX1ISIPOD) & (AuxInput == Aux1))
				iPod(cmdNextTrack);
			if (AuxInput == Aux2)
				{
				cd_Track++;
				ShowDiagnostics();
				}
			return 1;
	}
	
	if (CheckCmd_P((u08*)prev_track))		{ 
			if ((AUX1ISIPOD) & (AuxInput == Aux1))
				iPod(cmdPrevTrack);
			return 1; 
	}

	if (CheckCmd_P((u08*)next_cd))		{ 
			cd_Time_Min = 0;
			cd_Time_Sec = 0; 
			cd_Track =1;
			if (cd_Disc < maxDisc)
				cd_Disc++;
			if ((AUX1ISIPOD) & (AuxInput == Aux1))
				iPod(cmdPlaylist);
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)prev_cd))		{ 
			cd_Time_Min = 0;
			cd_Time_Sec = 0; 
			cd_Track =1;
			if (cd_Disc>1) 
				{
				cd_Disc--;
				};
			if ((AUX1ISIPOD) & (AuxInput == Aux1))
				iPod(cmdPlaylist);
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)fast_forward))	{ 
			CD_Mode= stFF; 
			return 1; 
	}

	if (CheckCmd_P((u08*)fast_back))		{ 
			CD_Mode = stRR; 
			return 1; 
	}

	if (CheckCmd_P((u08*)repeat_on))		{ 
			repeatMode = 1; 
			iPodSetRepeat();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)repeat_off))		{ 
			repeatMode = 0; 
			iPodSetRepeat();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)repeat_d_on))	{ 
			repeatMode = 2; 
			iPodSetRepeat();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)repeat_d_off))	{ 
			repeatMode = 0; 
			iPodSetRepeat();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)random_on))		{ 
			randomMode = 1; 
			iPodSetRandom();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)random_off))		{ 
			randomMode = 0; 
			iPodSetRandom();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)random_d_on))	{ 
			randomMode = 2; 
			iPodSetRandom();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)random_d_off))	{ 
			randomMode = 0; 
			iPodSetRandom();
			Event |= EV_STATUS;
			return 1; 
	}

	if (CheckCmd_P((u08*)scan_on))		{ 
			//scanMode = 1;	  
			SwitchInputs();
			return 1; 
	}
	
	if (CheckCmd_P((u08*)scan_off))		{ 
			scanMode = 0;   
			return 1; 
	}

	if (CheckCmd_P((u08*)scan_d_on))		{ 
			//scanMode = 2;	  
			return 1; 
	}

	if (CheckCmd_P((u08*)scan_d_off))		{ 
			scanMode = 0;   
			return 1; 
	}

//	if (CheckCmd_P((u08*)stat1)) { answerReq = cmStatus1; return 1; }
	if (CheckCmd_P((u08*)stat2)) { answerReq = cmStatus2; return 1; }
	if (CheckCmd_P((u08*)stat3)) { answerReq = cmStatus3; return 1; }
	if (CheckCmd_P((u08*)stat4)) { answerReq = cmStatus4; return 1; }

	if (CheckCmd_P((u08*)play_req1)) { 
			if ((!IPOD_IS_CONNECTED) & (!AUX2ENABLED))
				answerReq = cmNoCart;
			else
				answerReq = cmPlayReq1;
				//if ((iPodMode != stFF) & (iPodMode != stRR)) SwitchInputs();
			return 1; }

	if (CheckCmd((u08*)play_req2)) { 
			answerReq = cmPlayReq2; 
			return 1; }
	if (CheckCmd((u08*)play_req3)) { 
			answerReq = cmPlayReq3; 
			return 1; }
	if (CheckCmd((u08*)stop_req))  { 
			answerReq = cmStopReq;  
			return 1; }
	if (CheckCmd((u08*)stop_req2))  { 
			answerReq = cmStopReq2;  
			return 1; }
	if (CheckCmd_P((u08*)discstatus))	{ 
			answerReq = cmDiscStatus;	
			return 1; }

	if (CheckCmd_P((u08*)trackdiscstatus))	{ 
			answerReq = cmTrackDiscStatus;	
			return 1; }

	if (CheckCmd_P((u08*)broadcastyourstatus)) { //broadcast your info
			answerReq = cmInit;	
			return 1; }

	if (CheckCmd_P((u08*)changerstatus2))	{ //direct response back
			answerReq = cmCartStatus;	
			return 1; }

	if (CheckCmd((u08*)lan_check2))	{ 
			answerReq = cmCheck;
			CMD_CHECK[6]=message[4];
			CMD_CHECK[4]=message[1];
			return 1; 
	}

	// unknown command received, log it
	u08 i;
	eeprom_write_byte(EEPROM_UnknownCommand,message_len);
	for (i=0;i<message_len;i++)
		{
		eeprom_write_byte(EEPROM_UnknownCommand+1+i,message[i]);
		}
	eeprom_write_byte(EEPROM_UnknownCommand+1+i,0x00);

	// try a changer status message?
	//answerReq = cmInit;

    } else { // broadcast check

	if (CheckCmd((u08*)lan_playit))	{ 
			answerReq = cmPlayIt;	
			return 1; }
	if (CheckCmd((u08*)lan_check))	{ 
			answerReq = cmCheck;
			CMD_CHECK[6]=message[3];
			CMD_CHECK[4]=message[0];
			return 1; 
	}

	if (CheckCmd_P((u08*)lan_reg_11) | CheckCmd_P((u08*)lan_reg_12))	{ 
			//register with everyone!!
			masterDevice1 = master1;
			masterDevice2 = master2;
			avcversion = message[0];
			CMD_REGISTER[4]=avcversion; //correct AVCLAN version
			if (registered == 0) answerReq = cmRegister;
			return 1; }
	if (CheckCmd_P((u08*)lan_init_11) | CheckCmd_P((u08*)lan_init_12))	{ 
			masterDevice1 = master1;
			masterDevice2 = master2;
			eeprom_write_byte(EEPROM_MasterDevice1,master1);
			eeprom_write_byte(EEPROM_MasterDevice2,master2);
			eeprom_write_byte(EEPROM_AVCVersion,message[0]);
			Init_Commands(message[0]);
			answerReq = cmInit;		
			return 1; }
	if (CheckCmd_P((u08*)lan_stat1))	{ 
			answerReq = cmStatus1;	
			return 1; }

	if (CheckCmd((u08*)lan_registration) & (registered == 0)) {
			answerReq = cmRegister;
			return 1;
	};
			
 }

 return 1;
}
//------------------------------------------------------------------------------
u08 AVCLan_SendData(u08 m1, u08 m2)
{
 u08 i;
 


 // wait for free line
 u08 T=0;
 u08 line_busy = 1;

 timer0_source(CK64);
 do {
 	timer0_start();
 	while (INPUT_IS_CLEAR) {
		T = inp(TCNT0);
		if (T >= 50) break;
 	}
 	if (T > 49) line_busy=0;
 } while (line_busy);

 STOPEvent;

 AVCLan_Send_StartBit();
 AVCLan_Send_Byte(0x1,  1);		// regular communication

 parity_bit = 0;
 AVCLan_Send_Byte(MY_ID_1, 4);	// CD Changer ID as master
 AVCLan_Send_Byte(MY_ID_2, 8);
 AVCLan_Send_ParityBit();

 AVCLan_Send_Byte(m1, 4);	// HeadUnit ID as slave
 AVCLan_Send_Byte(m2, 8);
 AVCLan_Send_ParityBit();
 if (AVCLan_Read_ACK()) {
	 STARTEvent;
	 return 1;
 }

 AVCLan_Send_Byte(0xF, 4);		// 0xf - control -> COMMAND WRITE
 AVCLan_Send_ParityBit();
 if (AVCLan_Read_ACK()) {
	 STARTEvent;
	 return 2;
 }

 AVCLan_Send_Byte(data_len,  8);// data lenght
 AVCLan_Send_ParityBit();
 if (AVCLan_Read_ACK()) {
	 STARTEvent;
	 return 3;
 }

 for (i=0;i<data_len;i++) {
	AVCLan_Send_Byte(data[i], 8);// data byte
 	AVCLan_Send_ParityBit();
 	if (AVCLan_Read_ACK()) {
		 STARTEvent;
		 return 4;
 	}
 }

 STARTEvent;
 #ifdef MONITOR
 ShowOutMessage(1);
 #endif
 return 0;
}
//------------------------------------------------------------------------------
u08 AVCLan_SendDataBroadcast()
{
 u08 i;
 

;

 // wait for free line
 u08 T=0;
 u08 line_busy = 1;

 timer0_source(CK64);
 
 do {
 	timer0_start();
 	while (INPUT_IS_CLEAR) {
		T = inp(TCNT0);
		if (T >= 50) break;
 	}
 	if (T > 49) line_busy=0;
 } while (line_busy);

 STOPEvent;

 if (!AVCLan_Send_StartBit()) {
 	STARTEvent;
	return 5;
	};
 AVCLan_Send_Byte(0x0,  1);		// broadcast

 parity_bit = 0;
 AVCLan_Send_Byte(MY_ID_1, 4);	// CD Changer ID as master
 AVCLan_Send_Byte(MY_ID_2, 8);
 AVCLan_Send_ParityBit();

 AVCLan_Send_Byte(0x1, 4);		// all audio devices
 AVCLan_Send_Byte(0xFF, 8);
 AVCLan_Send_ParityBit();
 AVCLan_Send_Bit1();

 AVCLan_Send_Byte(0xF, 4);		// 0xf - control -> COMMAND WRITE
 AVCLan_Send_ParityBit();
 AVCLan_Send_Bit1();
 
 AVCLan_Send_Byte(data_len,  8);	// data lenght
 AVCLan_Send_ParityBit();
 AVCLan_Send_Bit1();

 for (i=0;i<data_len;i++) {
	AVCLan_Send_Byte(data[i], 8); // data byte
 	AVCLan_Send_ParityBit();
	AVCLan_Send_Bit1();
 }

 STARTEvent;
 #ifdef MONITOR
 ShowOutMessage(0);
 #endif
 return 0;
}
//------------------------------------------------------------------------------
u08 AVCLan_SendAnswerFrame(u08 *cmd)
{
 u08 i;
 u08 *c;
 u08 b;

 c = cmd;
 
 b = *c++;
 data_control = 0xF;
 data_len	 = *c++;
 for (i=0; i<data_len; i++) {
 	data[i]= *c++;
 }
 switch (b) {
 	case 0x00: //broadcast
		return AVCLan_SendDataBroadcast();
	case 0x01: // to lan master
		return AVCLan_SendData(masterDevice1, masterDevice2);
	case 0x02: // to requester
		return AVCLan_SendData(master1, master2);
    case 0x03:
			// to audio HU
		if (avcversion==0x11)
			{
			return AVCLan_SendData(0x01, 0x90);
			}
		else
			{
			return AVCLan_SendData(0x01, 0x60);
			}
		
	default:
		return 0;
};
 	
}
//------------------------------------------------------------------------------
u08 AVCLan_SendDiscStatus(u08 audioHU)
{
 u08 r;
 u08 c2[] = { 0x2, 0x06,      0x00, 0x63, 0x31, 0xF0, 0x06, 0x00};
 if (audioHU)
 	c2[0] = 0x3;
 c2[6] = BCDu08(cd_Disc); 
 c2[7] = BCDu08(cd_Track); 
 r = AVCLan_SendAnswerFrame((u08*)c2); //e0
 return r;
}

u08 AVCLan_SendDiscTrackStatus(u08 audioHU)
{
 u08 r = 0;
 u08 c4[] = { 0x1, 0x0C,	    0x00, 0x63, 0x31, 0xF2, 0x05, 0x28, 0x06, 0x01, 0x00, 0x20, 0x00, 0x80};

 if (audioHU)
 	c4[0] = 0x3;

 if (CD_Mode == stPlay)
 	{
		c4[7] = 0x10;
	}
	else
	{
		c4[7] = 0x28;
	};

 c4[8] =BCDu08(cd_Disc);
 c4[9] =BCDu08(cd_Track);
 c4[10]=cd_Time_Min;
 c4[11]=cd_Time_Sec;

 c4[12] = 0;

 switch (repeatMode) {
	case 1:	c4[12] |= 0x10; break;
	case 2:	c4[12] |= 0x08; break;
 }
 switch (randomMode) {
 	case 1:	c4[12] |= 0x04; break;
	case 2:	c4[12] |= 0x02; break;
 }
 switch (scanMode) {
 	case 1:	c4[12] |= 0x40; break;
	case 2:	c4[12] |= 0x20; break;
 }
 r = AVCLan_SendAnswerFrame((u08*)c4);  //e2

 return r;
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_Changer_Status()
{
 // broadcast my status
 CMD_DISCS_INCHANGER_BC[6] = Changer_Mode;
 CMD_DISCS_INCHANGER_BC[8] = Changer_Mode;

 return AVCLan_SendAnswerFrame((u08*)CMD_DISCS_INCHANGER_BC);
  
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_Changer_Status_Direct()
{
 // direct back to the requesting client
 CMD_DISCS_INCHANGER_DR[7] = Changer_Mode;
 CMD_DISCS_INCHANGER_DR[9] = Changer_Mode;

 return AVCLan_SendAnswerFrame((u08*)CMD_DISCS_INCHANGER_DR);
  
}
//------------------------------------------------------------------------------
u08 AVCLan_Send_Status(u08 Status)
{
//                                                        disc  track t_min t_sec
 u08 STATUS[] = {0x0, 0x0B, 0x63, 0x31, 0xF1, 0x05, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x80 };

 STATUS[7] = BCDu08(cd_Disc);
 STATUS[8] = BCDu08(cd_Track);
 STATUS[9] = cd_Time_Min;
 STATUS[10] = cd_Time_Sec;

 STATUS[11] = 0;

 switch (repeatMode) {
	case 1:	STATUS[11] |= 0x10; break;
	case 2:	STATUS[11] |= 0x08; break;
 }
 switch (randomMode) {
 	case 1:	STATUS[11] |= 0x04; break;
	case 2:	STATUS[11] |= 0x02; break;
 }
 switch (scanMode) {
 	case 1:	STATUS[11] |= 0x40; break;
	case 2:	STATUS[11] |= 0x20; break;
 }


 STATUS[6] = Status;

 if ((CD_Mode == stStop) & (Status != 0x00))
 	{
	STATUS[5] = 0x00;
	}

 AVCLan_SendAnswerFrame((u08*)STATUS);
 return 0;
}
//------------------------------------------------------------------------------
void AVCLan_Send_F7_Status(u08 Status)
{
 u08			CMD_F7[]  = {0x0,		0x05,	0x63, 0x31, 0xF7, 0x05, 0x63};

 CMD_F7[5] = Status;

 AVCLan_SendAnswerFrame((u08*)CMD_F7);
}
//------------------------------------------------------------------------------
u08 AVCLan_SendAnswer()
{
 u08 r = 0 ;
 
 switch (answerReq) {
// 	case cmStatus1:		r = AVCLan_SendAnswerFrame((u08*)CMD_STATUS1); 
//						break;
 	case cmStatus2:		r = AVCLan_SendAnswerFrame((u08*)CMD_STATUS2); 
						break;
 	case cmStatus3:		r = AVCLan_SendAnswerFrame((u08*)CMD_STATUS3); 
						break;
 	case cmStatus4:		r = AVCLan_SendAnswerFrame((u08*)CMD_STATUS4); 
						break;
 	case cmRegister:	CD_Mode = stStop;
						r = AVCLan_SendAnswerFrame((u08*)CMD_REGISTER);
						registered = 1;
						break;
 	case cmInit:		r = AVCLan_Send_Changer_Status();
						break;
 	case cmCartStatus:	r = AVCLan_Send_Changer_Status_Direct();
						if (!r) answerReq_InASec = cmTrackDiscStatus;
						break;
 	case cmCheck:		r = AVCLan_SendAnswerFrame((u08*)CMD_CHECK); 
						break;
 	case cmPlayReq1:	if (CD_Mode == stStop) 
							{
							r = AVCLan_SendAnswerFrame((u08*)CMD_PLAY_OK1);
							};
						if ((CD_Mode == stFF)|(CD_Mode == stRR))
							CD_Mode = stPlay;
						break;

 	case cmPlayReq2:
 	case cmPlayReq3:
	case cmPlayReq4:	playMode = 1;
						r = AVCLan_SendAnswerFrame((u08*)CMD_PLAY_OK2); 
						CD_Mode = stPlay;
						Event |= EV_STATUS;
						break;

	case cmPlayIt:		if ((iPodEvent == evNone) & (iPodMode == CD_Mode)) iPodEvent = evTimeStatus;
						break;
	case cmStopReq:		
	case cmStopReq2:	CD_Mode = stStop;
						playMode = 0;
						r = AVCLan_SendAnswerFrame((u08*)CMD_STOP1); 
						AVCLan_Send_Status(0x30);
						break;
    case cmDiscStatus:  r = AVCLan_SendDiscStatus(0);
						break;
    case cmTrackDiscStatus:    
						r = AVCLan_SendDiscTrackStatus(0);
						if (r == 0) {
							answerReq_InASec = cmDiscStatus;
						};
						break;
	case cmNoCart:		r = AVCLan_SendAnswerFrame((u08*)CMD_NOCART);
						break;
 }

if (r == 0) {
 	answerReq = cmNull;
	retries=0;
	}
else
	{
	#ifdef MONITOR
		RS232_Print("Retry ");
		RS232_PrintHex8(r);
		RS232_Print(" ");
		RS232_PrintHex8(answerReq);
		RS232_Print("\n");
	#endif
	// wait a short time then retry, its a downer that we won't receive anything during this but we wouldn't
	// if we were saturating the bus anyways like we used to
	retries++;
	// don't retry indefinately if somethings going on
	if (retries > 5)
		{
		answerReq= cmNull;
		retries=0;
	};
};
 return r;
}
//------------------------------------------------------------------------------
//u08 HexInc(u08 data)
//{
// if ((data & 0x9)==0x9) 
// 	return (data + 7);
 
// return (data+1);
//}
//------------------------------------------------------------------------------
//u08 HexDec(u08 data)
//{
// if ((data & 0xF)==0) 
// 	return (data - 7);
 
// return (data-1);
//}
//------------------------------------------------------------------------------
