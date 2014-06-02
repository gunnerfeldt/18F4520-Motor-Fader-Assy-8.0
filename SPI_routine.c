
#include <usart.h>	   /* Serial UART stuff */
#include <spi.h>	/* Master Synchonous Serial I2C */	

#include "SPI_routine.h"

#define AUTO_LED  LATBbits.LATB0
#define TOUCH_LED  LATBbits.LATB1
#define WRITE_LED  LATBbits.LATB2
#define MUTE_LED  LATBbits.LATB3

union indata {
	unsigned char bytes[2];
	struct  {
		unsigned vca_lo:8;
		unsigned vca_hi:2;
		unsigned status:2;
		unsigned :2;
		unsigned motor_on:1;
		unsigned mute:1;
		};
	struct  {
		unsigned CHANNEL_ID:7;
		unsigned :1;
		unsigned :4;
		unsigned ID_CHANGE:1;
		unsigned BLINK_EVENT:1;
		unsigned :2;
		};
	unsigned int word;
};
union outdata{
	unsigned char bytes[2];
	struct  {
		unsigned vca_lo:8;
		unsigned vca_hi:2;
		unsigned status:2;
		unsigned touch_press:1;
		unsigned touch_release:1;
		unsigned mute_press:1;
		unsigned mute_release:1;
		};
	unsigned int word;
};

#pragma udata access volatile_access
unsigned char index=0;
unsigned char localTouchSense=0;

#pragma ram near udata
ram near union indata inbuffer;
ram near union outdata outbuffer;


//---------------------------------------------------------------------
// Read From SPI buffer
//---------------------------------------------------------------------
char Read_SPI(void)
{	
	char DataIn;
	return DataIn;
}

//---------------------------------------------------------------------
// Write To SPI Buffer
//---------------------------------------------------------------------

void Write_SPI(unsigned int data)
{
	;
}

unsigned char getData(unsigned char indx)
{
	return inbuffer.bytes[indx];
}

void setData1(unsigned char data)
{
	outbuffer.bytes[0]=data;
}
void setData2(unsigned char data)
{
	outbuffer.bytes[1]=data;
}