/*
  Copyright (C) 2006 Marcin Slonicki <marcin@softservice.com.pl>.

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
	iPod interface for TOYOTA AVC-Lan CD-Changer Emulator
	v1.0
	(C) 2006 SLONIU
				http://www.softservice.com.pl/corolla/avc
				marcin@softservice.com.pl
 ---------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include "delay.h"
#include "const.h"
#include "avclandrv.h"
#include "inputoutput.h"
#include "com232.h"
#include "ipod.h"
#include "timer.h"

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void PIN_Setup();

//------------------------------------------------------------------------------

void Wait1msForAVCLan()
{
 timer0_source(CK64);
 u08 T;

 // check start bit
 timer0_start();
 while (!INPUT_IS_SET) { 
 	T=inp(TCNT0);
	if (T>100) {
		return;
	}
 }

};
//------------------------------------------------------------------------------
int main()
{

 PIN_Setup();

 AVCLan_Init();

 iPod_Init();


#ifdef MONITOR
	delay_ms(100);
	 RS232_Print("********************** RESET *********************8\n");
 #endif

 while (1) {

	//Wait1msForAVCLan();
	if ((INPUT_IS_SET>0) & (ACC_IS_ON>0)) {	 // if message from some device on AVCLan begin
  		AVCLan_Read_Message();
	} else {
		// check command from HU
		if ((answerReq != 0) & (RXState == WAITING_FOR_FF)) AVCLan_SendAnswer();
	
    }

	// HandleEvent
	if (((Event & EV_STATUS)>0) & (RXState == WAITING_FOR_FF)) {
	  	if (CD_Mode != stStop) {
			if ((CD_Mode == stFF) | (CD_Mode == stRR)) {
				if (!AVCLan_Send_Status(0x18)) Event &= ~EV_STATUS;
				}
			else
				{
			    switch (playMode) {
					case 0: if (!AVCLan_Send_Status(0x03)) Event &= ~EV_STATUS; break;
					case 1:	if (!AVCLan_Send_Status(0x10)) Event &= ~EV_STATUS; break;
				};
			};
		};
    }

	if (Event & EV_DISPLAY)	{
		if (!AVCLan_Send_Changer_Status()) Event &= ~EV_DISPLAY;;
	};

	if (ACC_IS_ON) 	{
		// audio output
		if (CD_Mode == stStop)
			CHANNELS_OFF();

		if (CD_Mode == stPlay)
			{
			if (AuxInput==Aux2)
				CHANNEL_TWO();
			if (AuxInput==Aux1)
				CHANNEL_ONE();
			}
		}	else 	{ 
			CHANNELS_OFF();
			CD_Mode = stStop;
			playMode = 0;
		};

	// iPod
	if (AUX1ISIPOD)
		iPod_Poll();
	
	if (!(ACC_IS_ON))
		registered = 0;
	// zzzzz
	//if (sleepcounter > 5)
	//	{ 
	//	CHANNELS_OFF();
	//	sleep_mode();
	//	sleepcounter = 0;
	//	}
 } // end while(1)
}
//------------------------------------------------------------------------------
void PIN_Setup()
{
 outb(GIMSK, 0);			// disable external interupts
 outb(MCUCR, 0);
 sbi(TIMSK, TOIE1); // Enable timer1 interrupt

  // Timer 1
 outb(TCCR1A, 0);
 outb(TCCR1B, (_BV(CS12)) | (_BV(CS10)));	// Prescaler /1024
 outb(TCNT1H, TI1_H);						// Load counter value hi
 outb(TCNT1L, TI1_L);						// Load counter value lo

 IO_Init();
 RS232_Init();
 Event = EV_NOTHING;
 sei();

 //MCUCR     = (0<<ISC11)|(1<<ISC10)|(0<<ISC01)|(1<<ISC00);
}

//------------------------------------------------------------------------------
SIGNAL(SIG_OVERFLOW1)					// Timer1 overflow every 1Sec
{
	outb(TCNT1H, TI1_H);					// reload timer1
	outb(TCNT1L, TI1_L);

	if ((CD_Mode!=stStop)) {
			if (!((AuxInput == Aux1) & (AUX1ISIPOD))) // ipod not providing updates
				Event |= EV_STATUS;
	};
	if ((answerReq_InASec != 0) & (answerReq == 0))
		{
		answerReq = answerReq_InASec;
		answerReq_InASec = 0;
		}

	if ((iPodStartup == waiting_Begin) | (iPodStartup == waiting_PlaylistCount)) 
		{
		iPodStartup = startup_Begin;
		}
}

SIGNAL(SIG_COMPARATOR)
{
	//do nothing we're now awake!
};

SIGNAL(INT0_vect)
{
	// great we're awake
}; 

SIGNAL(INT1_vect)
{
	// great we're awake
};

void SetChangerModeForiPod()
{
		Changer_Mode = stDisc1;
		if (maxDisc > 0)
			Changer_Mode |= stDisc2;
		if (maxDisc > 1)
			Changer_Mode |= stDisc3;
		if (maxDisc > 2)
			Changer_Mode |= stDisc4;
		if (maxDisc > 3)
			Changer_Mode |= stDisc5;
		if (maxDisc > 4)
			Changer_Mode |= stDisc6;
		Event |= EV_DISPLAY;
};
//------------------------------------------------------------------------------
void SwitchInputs()
{
#ifdef MONITOR
	if (AUX1ISIPOD)
		RS232_Print("Aux 1 is iPod\n");
	if (IPOD_IS_CONNECTED)
		RS232_Print("iPod is connected\n");
#endif
	if (!AUX2ENABLED) //only one input
		AuxInput = Aux1;
	else
		if (AuxInput ==Aux1)
			AuxInput = Aux2;
		else
			AuxInput = Aux1;

	if ((AUX1ISIPOD) & (!IPOD_IS_CONNECTED))
		{
		AuxInput = Aux2;
		}

	if (AuxInput == Aux1)
		{
		//iPod
			if (AUX1ISIPOD)
				{
				cd_Disc = iPodPlaylist;
				cd_Time_Min = iPodMin;
				cd_Time_Sec = iPodSec;
				cd_Track = iPodTrack;
				}
			else
				{
				cd_Disc = 1;
				maxDisc = 1;
				cd_Track = 1;
				cd_Time_Min = BCDu08(versionMajor);
				cd_Time_Sec = BCDu08(versionMinor);
				}

		SetChangerModeForiPod();
		
		
		}
		else
		{
		//AUX
		cd_Disc = 2;
		cd_Track = 1;
		cd_Time_Min = BCDu08(versionMajor);
		cd_Time_Sec = BCDu08(versionMinor);
		//cd_Time_Sec = BCDu08(iPodStartup);
		//cd_Time_Min = BCDu08(last);
		Changer_Mode = stDisc2;
		Event |= EV_DISPLAY;
		}
}

void ShowDiagnostics()
{
	u08 diag = eeprom_read_byte((uint8_t*)eeprombase+(cd_Track-2));
	cd_Time_Min = BCDu08(diag / 100);
	cd_Time_Sec = BCDu08(diag % 100);
}
