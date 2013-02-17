#ifndef __IO_H
#define __IO_H
//------------------------------------------------------------------------------

#define CHANNEL_DDR		DDRC
#define CHANNEL_PORT	PORTC
//#define	LED_PIN		PINC
#define CHANNELTWO		0x1
#define CHANNELONE		0x2

void IO_Init();
void CHANNEL_ONE();
void CHANNEL_TWO();
void CHANNELS_OFF();

#define ACC_DDR		DDRD
#define ACC_PORT	PORTD
#define ACC_PIN		PIND

#define ACC			0x4
#define IPOD		0x8

#define ACC_IS_ON ((inb(ACC_PIN)&ACC)>0) 

#define IPOD_IS_CONNECTED ((!(inb(ACC_PIN)&IPOD))>0)

#define SETTINGS_DDR	DDRB
#define SETTINGS_PORT	PORTB
#define SETTINGS_PIN	PINB

#define SETTING1	0x1
#define SETTING2	0x2

#define AUX1ISIPOD ((inb(SETTINGS_PIN)&SETTING1)>0)
#define AUX2ENABLED ((inb(SETTINGS_PIN)&SETTING2)>0)

//------------------------------------------------------------------------------
#endif
