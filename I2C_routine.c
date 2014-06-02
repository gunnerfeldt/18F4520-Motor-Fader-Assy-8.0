
#include <usart.h>	   /* Serial UART stuff */
#include <i2c.h>	/* Master Synchonous Serial I2C */	
#include "I2C_routine.h"


// definitions, can be stripped
#define BUFSIZE 1
#define STATE_1 0b00001001 
#define STATE_2 0b00101001
#define STATE_3 0b00001100
#define STATE_4 0b00101100
#define STATE_5 0b00101000

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
// SSP_Handler
//---------------------------------------------------------------------
// The I2C code below checks for 5 states:
//---------------------------------------------------------------------
// State 1: I2C write operation, last byte was an address byte.
// SSPSTAT bits: S = 1, D_A = 0, R_W = 0, BF = 1
//
// State 2: I2C write operation, last byte was a data byte.
// SSPSTAT bits: S = 1, D_A = 1, R_W = 0, BF = 1
//
// State 3: I2C read operation, last byte was an address byte.
// SSPSTAT bits: S = 1, D_A = 0, R_W = 1 
//
// State 4: I2C read operation, last byte was a data byte.
// SSPSTAT bits: S = 1, D_A = 1, R_W = 1, BF = 0
//
// State 5: Slave I2C logic reset by NACK from master.
// SSPSTAT bits: S = 1, D_A = 1, BF = 0, CKP = 1 
//----------------------------------------------------------------------		

char HandleI2C(void)
	{	
		unsigned char address;
		unsigned char I2CSTAT;
		unsigned char I2Caddress=0;
															// copy status register		
		I2CSTAT = SSPSTAT & 0b00101101;						// mask out the used bits
															// decide the state
			if (I2CSTAT == STATE_1){						// address match, write			
				address = SSPBUF;							// dummy read to clear BF
				index = 0;									// clear buffer index	
				while(SSPCON1bits.SSPOV)
				{
					SSPCON1bits.SSPOV=0;
					address = SSPBUF;
				}
				SSPCON1bits.CKP = 1; // release clock
				return 1;}
			if (I2CSTAT == STATE_2){						// master write byte
				inbuffer.bytes[index] = SSPBUF;				// read data and clear BF
				while(SSPCON1bits.SSPOV)
				{
					SSPCON1bits.SSPOV=0;
					inbuffer.bytes[index] = SSPBUF;
				}
				SSPCON1bits.CKP = 1; // release clock
				index++;	
				if(index>1) 							// if buffer length exceeded, clear index
				{
					return (0x08+index);
					index=0;
				}
				else return 2;									// increment index
				}
				
			if ((I2CSTAT & 0b00101100)== STATE_3){			// address match, read (ignore BF bit)
				index = 0;									// clear buffer index
				address = SSPBUF;							// dummy read to clear BF
				while(SSPCON1bits.SSPOV)
				{
					SSPCON1bits.SSPOV=0;
					address = SSPBUF;
				}
				myWriteI2C(outbuffer.bytes[index]);			// a repeating function for writing
				index++;									// increment index
				return 3;}
			if (I2CSTAT == STATE_4){						// master read, buffer is empty. no need to read it.
				if(SSPCON1bits.CKP == 0)					// make sure the clock line hasn't been released
				{
					if(index>BUFSIZE) index = 0;			// if buffer length exceeded, clear index
					myWriteI2C(outbuffer.bytes[index]);		// a repeating function for writing
					index++;								// increment index
					return 4;
				}	
				SSPCON1bits.CKP = 1; // release clock
			}
//			if ((I2CSTAT) == STATE_5){			// NACK received when sending data to the master			
//				SSPCON1bits.CKP = 1; // release clock
//			return 5;}										// Mask RW bit in SSPSTAT
			while(SSPCON1bits.SSPOV)
			{
				SSPCON1bits.SSPOV=0;
				address = SSPBUF;
			}
			if(	I2CSTAT&0x01)
			{
				WRITE_LED=0;
				CloseI2C();
				TRISCbits.TRISC3=0;
				TRISCbits.TRISC4=0;
				LATCbits.LATC3=1;
				LATCbits.LATC4=1;
				TRISCbits.TRISC3=1;
				TRISCbits.TRISC4=1;
				I2Caddress =(xID-1);
				I2Caddress <<= 1;
				OpenI2C (SLAVE_7, SLEW_ON);
				SSPADD = 0xE0+I2Caddress; 	// slave address
				SSPCON2bits.SEN = 1;     		// hmmm clock streching ????
				PIE1bits.SSPIE = 1; // SSP interrupt enable
			}
			SSPCON1bits.CKP = 1; // release clock
		return 5;										    // if no state, return error code
		}

//---------------------------------------------------------------------
// myWriteI2C
//---------------------------------------------------------------------

void myWriteI2C(unsigned int data)
{
	do
	{
		SSPCON1bits.WCOL = 0;								// clear collision bit
		SSPBUF = data;									// load hardware buffer
	}
	while(SSPCON1bits.WCOL);							// if there was a collision, try again
	SSPCON1bits.CKP = 1;								// release the clock stretch
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