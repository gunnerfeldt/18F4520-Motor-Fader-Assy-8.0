///////////////////////////////////////////////////////////////
// PICJOESI2CSLAVE
//
// An I2C slave test program
// in 'C'
//
// (C) 2008 J.W.Brown
//
///////////////////////////////////////////////////////////////
#include <p18f4520.h>   /* for TRIS and PORT declarations */
#include <delays.h>	   /* Delays declarations */
#include <usart.h>	   /* Serial UART stuff */
#include <i2c.h>	/* Master Synchonous Serial I2C */	
#include <timers.h>
#include <adc.h>
#include <math.h>
#include <pwm.h>
#include "typedefs.h"
#include "PWM_routine.h"
#include "I2C_routine.h"
#include "SWITCHES_routine.h"


/* Set configuration bits:
 *  - disable watchdog timer
 *  - disable low voltage programming
 *
 */
 
//#pragma config WDT=OFF
//#pragma config LVP=OFF


#define BUFSIZE 2

#define motor_enable LATCbits.LATC0


#define ON	0
#define OFF 1

#define DEB0  LATEbits.LATE0
#define DEB1  LATEbits.LATE1

#define AUTO_SW  !PORTCbits.RC5
#define TOUCH_SW  !PORTCbits.RC6
#define WRITE_SW  !PORTCbits.RC7
#define MUTE_SW  !PORTAbits.RA4
#define AUTO_LED  LATBbits.LATB0
#define TOUCH_LED  LATBbits.LATB1
#define WRITE_LED  LATBbits.LATB2
#define MUTE_LED  LATBbits.LATB3
//
#define TOUCH_ADC		ADC_CH7
#define TOUCH_X_ADC		ADC_CH1
#define TOUCH_X_OUTPUT	TRISAbits.TRISA1 = 0
#define TOUCH_X_HIGH	LATAbits.LATA1 = 1

#define TOUCH_OUTPUT	TRISEbits.TRISE2 = 0
#define TOUCH_INPUT		TRISEbits.TRISE2 = 1
#define TOUCH_HI		LATEbits.LATE2 = 1
#define TOUCH_LOW		LATEbits.LATE2 = 0

#define LED_DRIVER_DATA		LATDbits.LATD0 = 0
#define LED_DRIVER_CLOCK	LATDbits.LATD1 = 0
#define LED_DRIVER_STROBE	LATDbits.LATD2 = 0
#define LED_DRIVER_ON		LATDbits.LATD3 = 0

#define big_plus  		0x0840		// hard code graphic for a plus sign
#define big_minus  		0x1008		// hard code graphic for a minus sign

#define ERROR_MASK_0	0b01010101
#define ERROR_MASK_1	0b10101010

#define bit_set(var,bitno) ((var) |= 1 << (bitno))
#define bit_clr(var,bitno) ((var) &= ~(1 << (bitno)))
#define testbit_on(data,bitno) ((data>>bitno)&0x01)
#define testbitmask(data,bitmask) ((data&bitmask)==bitmask)

#define bits_on(var,mask) var |= mask
#define bits_off(var,mask) var &= ~0 ^ mask

#define hibyte(x)       (unsigned char)(x>>8)
#define lobyte(x)        (unsigned char)(x & 0xFF)

#define INT_OFF		INTCONbits.GIE = 0
#define INT_ON		INTCONbits.GIE = 1

#define TOUCH_SENSE 	210
#define TOUCH_RELEASE	90

#define SAMPLERATE  	4
#define BLINKRATE  		800
#define TOUCHRATE  		2


// allocate some resources in ACCESS RAM for speedy access in interrupt routines
// all locations in this section are volatile
#pragma udata access volatile_access


//volatile near unsigned char       I2CSTAT;
//struct {
//    unsigned BF:1;
//    unsigned UA:1;
//    unsigned R_W:1;
//    unsigned S:1;
//    unsigned P:1;
//    unsigned D_A:1;
//    unsigned CKE:1;
//    unsigned SMP:1;
//}I2CSTATbits;

volatile unsigned int SampleCnt = 0;
volatile unsigned int TouchCnt = 0;
volatile unsigned int BlinkCnt = 0;

#pragma udata





void Init(void);
//----------------------------------------------------------------------------
// High priority interrupt routine (legacy interrupt)

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh () 
{
    unsigned char I2Cstate=0;                                   // State for I2C sequence. 0-5
	unsigned int FaderPosition;

    // ************************ I 2 C  I n t e r r u p t ******************************************
	if(PIR1bits.SSPIF)					                        // if new I2C event occured
	{	
		TOUCH_LED=!TOUCH_LED;
		
//	SSPSTATbits.BF=0;
		DEB1=!DEB1;
//		I2Cstate=HandleI2C();                                   // Service I2C line. Should be fast
		if(I2Cstate==5)
		{
	//		Read_Switches();
	//		Handle_LEDs();
		}										// done with the trasfer
	    PIR1bits.SSPIF = 0;				                        // reset I2C interrupt	
	}

    // ************************ T M R 2  I n t e r r u p t ****************************************
	if (PIR1bits.TMR2IF)	                                    // Timer base
	{	

		AUTO_LED=!AUTO_LED;
		DEB0=!DEB0;		
		SampleCnt++;                                            // ADC Timer
//		TouchCnt++;                                             // Touch Timer
//		BlinkCnt++;                                            	// Blink Timer
		
        // ************************ B l i n k  O v e r f l o w ****************************************
/*		if(BlinkCnt>BLINKRATE)                                                  // Rate=800
		{
			if(AttentionFlag)                                                   // Routine for flash delay
			{           
				AttentionCnt++;
				if(AttentionCnt>ATTENTION_DELAY)                                // Delay=50
				{ 
					AttentionFlag = 0;
					AttentionCnt=0;	
				}
			}

//			BlinkLED = !BlinkLED;                                               // Toggle Blink Value
			BlinkCnt = 0;
		}
*/				
        // ************************ F a d e r  O v e r f l o w ***************************************
        // Test the speed. Once every 10uS makes about 350 between MTC points
		if(SampleCnt>SAMPLERATE)                                                // ADC timer. Move ?
		{
//			AUTO_LED=!AUTO_LED;
/*			Start_ADC(); 
			FaderPosition=Read_ADC();                                       	// Sample Fader CV

			outbuffer.vca_lo=lobyte(FaderPosition);								// Make CV data read to ship!
			outbuffer.vca_hi=hibyte(FaderPosition);
			
			if(LinCntrCnt=<LinThresh)                                           // Linear filter based on time between I2C pulses
			{
			    LinFaderCv=(((LinThresh-LinCntrCnt)*PreviousFaderCvSet)+(LinCntrCnt*FaderCvSet))/LinThresh;     
			}
			Calculate_Motor_PWM(LinFaderCv);                                    // make out the PWM values
			Handle_Fader();                                                     // Update PWM 
			INT_ON;                                                             // Turn on  ISR. !!!!???!!!
			
			if(LOOP_CNTR<29)LOOP_CNTR++;                                        // determine loops. Change to timer
			

//			USB_FILTER_REG = USB_FILTER_REG - (USB_FILTER_REG >> READ_SHIFT_USB) + FaderPosition;          // Filter for CV to Host
//			FADER_USB = USB_FILTER_REG >> (READ_SHIFT_USB);                                                 // Filter
                                                                                // Reset ADC timer
			scan_switches();                                                                                // Read switches
			if (FADER_TOUCH)                                                                                // Update Led display
				{
				Update_LED_Display_Bits(Calculate_dB_Display());                                            // If Touch sense is true
				}
			else
				{                                                                                           // This shouldn't be done all the time
				Update_LED_Display(DISPLAY);
				}
			Handle_LEDs(indata.data[1]); 
*/	                                                                   // Update status and mute LEDs
			SampleCnt = 0;  
			}		
		PIR1bits.TMR2IF = 0;                                    // Reset Timer
	}
}


#pragma code

void main(void)
{	
	unsigned int cnt=0;
 	Init();
	while(1)
	{	
		for(cnt=0;cnt<1;cnt++)
		{;}
	}
}


	




	


void Init(void)
	{
	OSCTUNEbits.TUN0 = 1;
	OSCTUNEbits.TUN1 = 1;
	OSCTUNEbits.TUN2 = 1;
	OSCTUNEbits.TUN3 = 1;
	OSCTUNEbits.TUN4 = 1;
	OSCCONbits.IRCF0 = 1;
	OSCCONbits.IRCF1 = 1;
	OSCCONbits.IRCF2 = 1;
	OSCTUNEbits.PLLEN = 1;
	OSCTUNEbits.INTSRC = 0;


	TRISA = 0xFF;
	TRISB = 0xF0;	
	TRISC = 0xF8;
	TRISD = 0x00;
	TRISE = 0xFC;

//	TRISEbits.PSPMODE = 0;

	TRISCbits.TRISC3 =1;
	TRISCbits.TRISC4 =1;

//	SSPCON2bits.SEN = 1;     		// hmmm clock streching ????

//	SET FADER ADDRESS !!!!
//
//	fader_ID = ID-1;

//	fader_address = fader_ID*2;
//	DISPLAY = ID;
//	SSPADD = 0xE0 + fader_address; // slave address


	// enable interrupts - legacy mode
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;



/*	
	// OpenPWM1(0 to 255)  PWM period value
	OpenPWM1(0xFF);									// Turn PWM on
	SetDCPWM1(0); 

	OpenPWM2(0xFF);									// Turn PWM on
	SetDCPWM2(0); 


	OpenADC(	ADC_FOSC_32     	&
				ADC_RIGHT_JUST		&
				ADC_2_TAD,
				ADC_CH12			&
				ADC_INT_OFF			&
				ADC_VREFPLUS_EXT	&
				ADC_VREFMINUS_EXT
				, 15 );
	//		
	ADCON1 = 0x0C;
*/

 	OpenTimer2(T2_PS_1_1 & TIMER_INT_ON & T2_POST_1_4);	// 31.248 kHz

//	OpenI2C (SLAVE_7, SLEW_ON);
//	SSPADD = 0xE0; // slave address
//	PIE1bits.SSPIE = 1; // SSP interrupt enable

//	SSPSTATbits.BF=0;
//	SSPCON1bits.CKP = 1;
//	do 
//	{							
//		SSPCON1bits.WCOL = 0;								// clear collision bit
//	}
//	while(SSPCON1bits.WCOL);


	}

