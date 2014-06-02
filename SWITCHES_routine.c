#include "typedefs.h"
#include "I2C_routine.h"
#include "SWITCHES_routine.h"
#include <p18f4520.h>   /* for TRIS and PORT declarations */
#include <adc.h>

#define WRITE_LED  LATBbits.LATB2

#define AUTO_SW  !PORTDbits.RD5
#define TOUCH_SW  !PORTCbits.RC6
#define WRITE_SW  !PORTCbits.RC7
#define MUTE_SW  !PORTAbits.RA4

#define TOUCH_ADC		ADC_CH7
#define TOUCH_X_ADC		ADC_CH1
#define TOUCH_X_OUTPUT	TRISAbits.TRISA1 = 0
#define TOUCH_X_HIGH	LATAbits.LATA1 = 1

#define TOUCH_OUTPUT	TRISEbits.TRISE2 = 0
#define TOUCH_INPUT		TRISEbits.TRISE2 = 1
#define TOUCH_HI		LATEbits.LATE2 = 1
#define TOUCH_LOW		LATEbits.LATE2 = 0

#define TOUCH_THRESHOLD 	250
#define TOUCH_RELEASE	90


#pragma udata access volatile_access
unsigned char LocalTouch=0;

#pragma udata
unsigned char StoredSwitchBits=0;
unsigned int touch_rel;
unsigned int touch_press;


#pragma code
//---------------------------------------------------------------------
// Read Switches and detect lo to hi
//---------------------------------------------------------------------
void Read_Switches(void)
{
	unsigned char SwitchXOR=0;
	Switches PresSwitches;
	Switches RelSwitches;
	Switches SwitchBits;

	SwitchBits.byte=0;
	PresSwitches.byte=0;
	RelSwitches.byte=0;

	SwitchBits.autoSW=AUTO_SW;
	SwitchBits.touchSW=TOUCH_SW;
	SwitchBits.writeSW=WRITE_SW;
	SwitchBits.muteSW=MUTE_SW;
	SwitchBits.touchSense=LocalTouch;

	outbuffer.status=0;
	outbuffer.touch_press=0;
	outbuffer.touch_release=0;
	if(SwitchBits.byte!=StoredSwitchBits) {                            // If previous 8 bits are NOT equal with the current 8 bits:
	    SwitchXOR=SwitchBits.byte^StoredSwitchBits;                    // Create a mask with all changed bits
	    PresSwitches.byte=SwitchBits.byte&SwitchXOR;                        // Mask out all Pressed switches
	    RelSwitches.byte=StoredSwitchBits&SwitchXOR;                    // Mask out all Released Switches

	if(PresSwitches.autoSW)outbuffer.status=0x01;
	if(PresSwitches.touchSW)outbuffer.status=0x02;
	if(PresSwitches.writeSW)outbuffer.status=0x03;
	if(PresSwitches.touchSense)outbuffer.touch_press=1;
	if(RelSwitches.touchSense)outbuffer.touch_release=1;
	SettleCnt=0;
	}
	StoredSwitchBits=SwitchBits.byte;                                // Copy Switch pins til next read
}

//---------------------------------------------------------------------
// scan_Touch
//---------------------------------------------------------------------
void Start_Scan_Touch(void)                                  
	{
	TOUCH_X_OUTPUT;
	TOUCH_X_HIGH;
	SetChanADC(TOUCH_X_ADC);
	TOUCH_OUTPUT;
	TOUCH_LOW;
	TOUCH_INPUT;

	SetChanADC(TOUCH_ADC);
	ConvertADC();
	}
void Scan_Touch(void)                                  
	{
	unsigned int fader_touch_sens;
	while( BusyADC() );
	fader_touch_sens = ADRESH;
	fader_touch_sens = fader_touch_sens << 8;
	fader_touch_sens = fader_touch_sens + ADRESL;


	if (fader_touch_sens<TOUCH_THRESHOLD) 
		{	
		if (touch_press > 0) 
			{
			LocalTouch = 1;
			touch_rel = 0;
			}
		else 
			{
			touch_press +=1;
			BlinkLED = 1;
			BlinkCnt = 0;
			}
		}
	else
		{
		if (touch_rel > 16) touch_press = 0;
		if (touch_rel > TOUCH_RELEASE) 
			{
			LocalTouch = 0;
			}
		else 
			{touch_rel +=1;}			
		}

	}	