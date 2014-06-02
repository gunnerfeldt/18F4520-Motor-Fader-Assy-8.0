#include "LEDs_routine.h"
#include "I2C_routine.h"
#include <p18f4520.h>   /* for TRIS and PORT declarations */

#define AUTO_LED  		LATBbits.LATB0
#define TOUCH_LED  		LATBbits.LATB1
#define WRITE_LED  		LATBbits.LATB2
#define MUTE_LED  		LATBbits.LATB3

#define DISPLAY_DATA	LATDbits.LATD0
#define DISPLAY_CLOCK	LATDbits.LATD1
#define DISPLAY_STROBE	LATDbits.LATD2 = 1;LATDbits.LATD2 = 0
#define DISPLAY_ON 		LATDbits.LATD3



#pragma udata

const unsigned int SEGMENTA[] = {
0x2A2C,
0x2020,
0x280D,
0x2829,
0x2221,
0x0A29,
0x0A2D,
0x2820,
0x2A2D,
0x2A29,
0x4000,
0x1000,
0x0001};

const unsigned int SEGMENTB[] = {
0x94D0,
0x0480,
0x90C2,
0x14C2,
0x0492,
0x1452,
0x9452,
0x04C0,
0x94D2,
0x14D2,
0x0100,
0x2000,
0x0002
};

const unsigned int LETTERA[] = {
0x0000,				// ""
0x2A25,				// A
0x222C,				// U
0x2820,				// T
0x2A2C,				// O
0x2A24,				// M
0x2A25,				// A
0x2A24,				// N

};
const unsigned int LETTERB[] = {
0x0000,				// ""
0x0000,				// A
0x0000,				// U
0x0040,				// T
0x0000,				// O
0x04C0,				// M
0x0000,				// A
0x0000,				// N
};

void Update_LED_Display_Bits(unsigned int SHIFT_DIGITS)
{
	unsigned int n;
	for(n=0;n<16;n++)
	{
		DISPLAY_DATA = SHIFT_DIGITS & 1;
		SHIFT_DIGITS >>= 1;
		DISPLAY_CLOCK = 1;
		DISPLAY_CLOCK  = 0;
	}
	DISPLAY_STROBE;
	DISPLAY_ON=AttentionBlink;
}


void Update_LED_Display(unsigned char Digit)
{
	unsigned int SHIFT_DIGITS = 0;

	if(Digit<10)
	{
		SHIFT_DIGITS = SEGMENTB[Digit];
	}
	else
	{
		SHIFT_DIGITS = SEGMENTA[Digit/10] + SEGMENTB[Digit%10];
	}
	Update_LED_Display_Bits(SHIFT_DIGITS);
}

void SlideShow(unsigned char Digit)
{
	unsigned int SHIFT_DIGITS = 0;

	SHIFT_DIGITS = LETTERA[Digit] + LETTERB[Digit];
	Update_LED_Display_Bits(SHIFT_DIGITS);
}

void Handle_LEDs(void)
	{
	MUTE_LED = inbuffer.mute;
		switch (inbuffer.status)
			{
			case 0x00:
			AUTO_LED = 1;
			TOUCH_LED = 1;
			WRITE_LED = 1 & !(BlinkLED & LocalTouch) ;
			break;
			case 0x01:
			AUTO_LED = 0;
			TOUCH_LED = 1;
			WRITE_LED = 1 & !(BlinkLED & LocalTouch) ;
			break;
			case 0x02:
			AUTO_LED = 0;
			TOUCH_LED = 0;
			WRITE_LED = 1 & !(BlinkLED & LocalTouch) ;
			break;
			case 0x03:
			AUTO_LED = 0;
			TOUCH_LED = 1;
			WRITE_LED = 1 & (BlinkLED & LocalTouch) ;
			break;
			}
	}

/*
int Calculate_dB_Display(void){
	signed double DIFF;															// variable for exact difference
	int DIFF_DISPLAY;													// variable for truncated difference
	DIFF = FADER_USB;
	DIFF = (DIFF - FADER_IN)*0.05; 							// difference in dB

	if(DIFF==0){															// spot on
		return SEGMENTB[0] & (BlinkLED * 0xFFFF);							// result: blinking zero
		}

	if(DIFF>0){															// if positive difference
		if(DIFF>=10){													// over 10dB +
			return big_plus;												// result: big plus
			}
		else{
			if(DIFF>=1){													// between 1 and 9 dB
				DIFF_DISPLAY = DIFF;									// truncate
				return SEGMENTB[DIFF_DISPLAY];						// result: difference, no sign
				}
			else{															//Â between 0.1 and 0.9 dB
				DIFF_DISPLAY = DIFF * 10;								// truncate and multiply
				return (0x4000 + SEGMENTB[DIFF_DISPLAY]);	// result: difference, with DP
				}		
			}
		}

	else	{																// if negative difference
		if(DIFF<=-10){													// under 10dB -
			return big_minus;												// result: big minus
			}
		else{
			if(DIFF<=-1){												// between -1 and -9 dB
				DIFF_DISPLAY = fabs(DIFF);								// truncate absolute
				return 0x0001 + SEGMENTB[DIFF_DISPLAY];		// result: difference, signed
				}
			else{															//Â between -0.1 and -0.9 dB
				DIFF_DISPLAY = fabs(DIFF * 10);							// truncate and multiply absolute
				return (0x4000 + SEGMENTB[DIFF_DISPLAY]
				+ 0x0001);										// result: difference, signed with DP
				}		
			}
		}
	}
*/