#include "inputoutput.h"

#include <avr/io.h>

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void IO_Init()
{
 // set two ports as outputs
 CHANNEL_DDR |= CHANNELONE;
 CHANNEL_DDR |= CHANNELTWO;
 CHANNEL_PORT = 0;

 // one input
 ACC_DDR &= ACC;
 ACC_DDR &= IPOD;
 ACC_PORT |= IPOD; //pull up

 // two inputs 
 SETTINGS_DDR &= SETTING1;
 SETTINGS_DDR &= SETTING2;
 SETTINGS_PORT |= SETTING1; //pull up
 SETTINGS_PORT |= SETTING2; //pull up
}

//------------------------------------------------------------------------------
void CHANNEL_ONE()
{
 CHANNEL_PORT = CHANNELONE;
}

void CHANNEL_TWO()
{ 
 CHANNEL_PORT = CHANNELTWO;
}
//------------------------------------------------------------------------------
void CHANNELS_OFF()
{
 CHANNEL_PORT = 0;
}
//------------------------------------------------------------------------------
 
