/* Compile options:  -ml (Large code model) */

//#include <stdio.h>
#include <p18f4520.h>
#include <timers.h>
#include "typedefs.h"
#include <i2c.h>	/* Master Synchonous Serial I2C */	
#include <adc.h>
#include "I2C_routine.h"
#include "PWM_routine.h"
#include "SWITCHES_routine.h"
#include "LEDs_routine.h"


// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************

#define FADER_ID	3

// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************
// *************************** F  A  D  E  R    I  D  ************************************

#define ATTENTION_RELEASE		4				// (seconds) Release time for attention BLINK
#define INIT_SPEED		20				// 

#pragma config WDT = OFF
	
/*
 *  IO_card.h
 *  Test
 *
 *  Created by Pelle Gunnerfeldt on 2011-04-12.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

// allocate some resources in ACCESS RAM for speedy access in interrupt routines
// all locations in this section are volatile

/*
#define DEB0 LATBbits.LATB0
#define DEB1 LATBbits.LATB1
#define DEB2 LATBbits.LATB2
*/

#define AUTO_LED  		LATBbits.LATB0
#define TOUCH_LED  		LATBbits.LATB1
#define WRITE_LED  		LATBbits.LATB2
#define MUTE_LED  		LATBbits.LATB3


#define hibyte(x)       (unsigned char)(x>>8)
#define lobyte(x)        (unsigned char)(x & 0xFF)



#pragma udata access volatile_access
ram near unsigned int PreviousFaderGoal;
ram near unsigned int FaderGoal;
ram near unsigned char NewDataIn;
ram near unsigned char WorkFader;
unsigned int Steps=30;
ram near unsigned int LoopCnt;
ram near unsigned int InitCntDelay;
ram near unsigned int FaderGoalSteps;
ram near unsigned int FaderPosition;  
ram near unsigned char I2Cstate;
ram near unsigned char ShopSlots;


#pragma udata

unsigned int Cnt=0;
unsigned int BlinkCnt = 0;
unsigned int AttCnt = 0;
unsigned char SSLchannel=FADER_ID;
unsigned char xID=FADER_ID;

unsigned char AttentionBlink=0;
unsigned char BlinkLED=0;
unsigned char FaderReady=0;


// *********************************** Prototypes ****************************************
void Init(void);
void HighISRCode();
void LowISRCode();
void Init_Display (void);
unsigned int Init_Fader (void);

// ********************************* Interrupt vectors ***********************************
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
  _asm GOTO HighISRCode _endasm
}

#pragma code low_vector=0x18
void interrupt_at_low_vector (void)
{
  _asm GOTO LowISRCode _endasm
}

// *********************************** Interrupts ****************************************

#pragma interrupt HighISRCode
void HighISRCode()
{
//	if(PIR1bits.TMR2IF)					                        // 
//	{
//		PIR1bits.TMR2IF=0;
//
//	}

// ************************ I 2 C  I n t e r r u p t ******************************************

	if(PIR1bits.SSPIF)					                        // if new I2C event occured
	{	
		I2Cstate=HandleI2C();                                   // Service I2C line. Should be fast
	    PIR1bits.SSPIF = 0;				                        // reset I2C interrupt
		if(I2Cstate==0x0A)NewDataIn=1;					
	}


}

#pragma interruptlow LowISRCode
void LowISRCode()
{	

// ************************ P W M  I n t e r r u p t ******************************************

	if (INTCONbits.TMR0IF)									// Timer base
	{	
		
				Start_ADC(); 															// 0.03ms
				FaderPosition=Read_ADC();                            					// Sample Fader CV			
				FaderGoalSteps=(((Steps-LoopCnt)*PreviousFaderGoal)+(LoopCnt*FaderGoal))/Steps;		// 0.14ms
				Calculate_Motor_PWM(FaderGoalSteps,FaderPosition);			
				Handle_Fader();										// 0.04ms
				outbuffer.word=(outbuffer.word&0xfc00)+FaderPosition;					// Make CV data read to ship!
				LoopCnt++;					
				BlinkCnt++;					
				Start_Scan_Touch();	           											                
				Scan_Touch();               					// Sample Fader Touch

			if(FaderReady && (LoopCnt>15))							// done with the trasfer
			{
				LoopCnt=0;
				PreviousFaderGoal=FaderGoal;
				FaderGoal=inbuffer.word&0x3ff;
				Steps=16;
			}
			if(!FaderReady && (LoopCnt>16))							// done with the trasfer
			{
				PreviousFaderGoal=FaderGoal;
				FaderGoal=Init_Fader();
				Steps=LoopCnt;
				LoopCnt=0;
			}
		INTCONbits.TMR0IF = 0;
	}

	
}	//This return will be a "retfie", since this is in a #pragma interruptlow section 


#pragma code

// *********************************** MAIN ****************************************
void main (void)
{ 
	Init();
	WRITE_LED=0;
	while(1)
	{

		if((BlinkCnt>50) && !NewDataIn)
		{
			BlinkLED=!BlinkLED;
			BlinkCnt=0;
			if(AttCnt>0)AttCnt--;
			AttentionBlink=AttCnt;
		}	



		if(NewDataIn)
		{
			NewDataIn=0;
			if(FaderReady)					
			{
				if(!(inbuffer.bytes[1]&0x30))
				{
//					PreviousFaderGoal=FaderGoal;
//					FaderGoal=inbuffer.word&0x3ff;
						Handle_LEDs();
//					SSLchannel=LoopCnt;
					Update_LED_Display(SSLchannel);
					Read_Switches();
					Steps=LoopCnt;
//					LoopCnt=0;
				}
				else
				{
					if(inbuffer.ID_CHANGE)SSLchannel=inbuffer.CHANNEL_ID;
					if(inbuffer.BLINK_EVENT)AttCnt=ATTENTION_RELEASE*10;
				}		
			}
		}
	}
}
// *********************************** Init ****************************************
void Init(void)
{
	unsigned char I2Caddress=0;

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
	
	TRISEbits.PSPMODE = 0;

	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
	RCONbits.IPEN = 1;            //enable priority levels


	OpenTimer0( TIMER_INT_ON &
	T0_8BIT &
	T0_SOURCE_INT &
	T0_PS_1_64 );

	INTCON2bits.TMR0IP=0;			// TMR0 loprio interrupts

	OpenADC(	ADC_FOSC_32     	&
				ADC_RIGHT_JUST		&
				ADC_2_TAD,
				ADC_CH12			&
				ADC_INT_OFF			&
				ADC_VREFPLUS_EXT	&
				ADC_VREFMINUS_EXT
				, 15 );		
	ADCON1 = 0x0C;

	// OpenPWM1(0 to 255)  PWM period value
	OpenTimer2(T2_PS_1_1 & TIMER_INT_OFF & T2_POST_1_1);	// 31.248 kHz
	IPR1bits.TMR2IP=1;			// TMR2 high prio interrupts
//	PR2=2;

	OpenPWM1(0xFF);									// Turn PWM on
	SetDCPWM1(0); 

	OpenPWM2(0xFF);									// Turn PWM on
	SetDCPWM2(0);  

	InitCntDelay=0;
}



unsigned int Init_Fader (void)
{
	static unsigned	char TestPhase=0;
	static int InitCnt=0;
	static int SeqCnt=0;
	static int SeqDelay=0;
	static unsigned int FaderGoal=0;
	unsigned char I2Caddress=0;

	outbuffer.status=0;
	outbuffer.mute_press=0;
	outbuffer.mute_release=0;
	outbuffer.touch_press=0;
	outbuffer.touch_release=0;

	SeqDelay++;
	if(SeqDelay>(65-(FADER_ID*8)))
	{
		SeqCnt++;
		if((SeqCnt>>3)<0x08)SlideShow(SeqCnt>>3);
		else SlideShow(0);
	}	
	else SlideShow(0);
	

	if(TestPhase==0)
	{
		AUTO_LED=1;
		TOUCH_LED=1;
		WRITE_LED=1;
		MUTE_LED=1;
		
		FaderGoal=0;
		InitCnt++;
		if(InitCnt>10)
		{		
			TestPhase=1;
			InitCntDelay=0;
		}
	}
	if(TestPhase==1)
	{
		InitCntDelay++;
		if(InitCntDelay>(FADER_ID<<3))
		{
			if(FaderGoal<(1023-INIT_SPEED))FaderGoal+=INIT_SPEED;
			else 
				{
				FaderGoal=1023;
				InitCnt++;
				if(InitCnt>20)
				{
					Start_ADC(); 
					TopLimit=Read_ADC(); 
					if(TopLimit<985)TopLimit=1022;
					InitCnt=0;
					TestPhase=2;
				}		
			}
		}
	}
	if(TestPhase==2)
	{
		if(FaderGoal>INIT_SPEED)FaderGoal-=INIT_SPEED;
		else 
			{
			FaderGoal=0;
			InitCnt++;
			if(InitCnt>20)
			{		
				Start_ADC(); 
				BotLimit=Read_ADC(); 
				if(BotLimit>35)BotLimit=1;
				InitCnt=0;
				SetScale();	
				FaderReady=1;
				TestPhase=0;

				I2Caddress =(FADER_ID-1);
				I2Caddress <<= 1;
				OpenI2C (SLAVE_7, SLEW_ON);
				SSPADD = 0xE0+I2Caddress; 	// slave address
				SSPCON2bits.SEN = 1;     		// hmmm clock streching ????
				PIE1bits.SSPIE = 1; // SSP interrupt enable

			}			
		}
	}
	return FaderGoal;
}









