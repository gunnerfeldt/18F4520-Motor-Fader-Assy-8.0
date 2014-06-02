
#include <p18f4520.h>   /* for TRIS and PORT declarations */
#include <adc.h>
#include <math.h>
#include <pwm.h>
#include "PWM_routine.h"
#include "I2C_routine.h"

#define COMP_ADC		ADC_CH0
#define TEST_ADC		ADC_CH1

#define MOTOR_ON 		LATCbits.LATC0

#define AUTO_LED  		LATBbits.LATB0
#define TOUCH_LED  		LATBbits.LATB1
#define WRITE_LED  		LATBbits.LATB2
#define MUTE_LED  		LATBbits.LATB3


#pragma udata
const unsigned int maxPWM =1023;

const signed long pK = 1000;	//1000
const signed long iK = 1;	//3	
const signed long dK = 4000;	//4000
const long gain = 75;	//1
const long iKmult= 1;  // 1
const int TOL = 1;
const int SettleDelay = 1000;				//ms. after new incomming CV or touch. Turn off motor.
const int iError_max = 500; //500			

unsigned int SettleCnt=0;
unsigned int SettleThrs=500;		// Threshold for SettleDelay. SettleDelay * fps in Hz / 1000
signed int lastpError;
signed int lastiError;

unsigned int TopLimit=1023;
unsigned int BotLimit=0;
unsigned long ScaleRatio=100; 

#pragma ram udata
ram signed long fspeed; 
/************************************************************************************************

    Calculate PWM values
    Input: Goal to chase. 10 bit value

*************************************************************************************************/


#pragma code
void Calculate_Motor_PWM (int FaderGoal, int FaderPosition)
	{
	unsigned int PWM1;
	unsigned int PWM2;
	signed int pError;	
	int Scaled_Max;	
	static signed int D_REG;
	static signed int D_SPEED;
	static signed long lstP;
	static int LastGoal=0;
	signed int iError;
	signed int dError;

//	FaderGoal=(FaderGoal+LastGoal)/2;

	pError = FaderGoal-FaderPosition;
	iError += (pError*iKmult);                                              // Integral error ??
	Scaled_Max = iError_max; // *10;                                        // ??
	if (iError>Scaled_Max) iError =Scaled_Max;                              // ??
	if (iError<-Scaled_Max) iError =-Scaled_Max;

//	LastGoal=FaderGoal;

	dError = (pError-lstP);	                                                // Derivate error
	lstP = pError;		                                                    // Previous error
	fspeed = (((pK*pError)+(iK*iError)+(dK*dError))/(101-gain));            // Algorithm for PWM


	}


/************************************************************************************************

    Output PWM values

*************************************************************************************************/

void Handle_Fader(void)
{
	unsigned int PWM1;
	unsigned int PWM2;
	
	if(SettleCnt<SettleThrs || !FaderReady)
	{
		if ((!LocalTouch && inbuffer.status!=3) || !FaderReady)                // If host motors off or touch. Check touch variable !!!
		{
			if(fspeed<-1)                                   // Hmmm. -1. is fspeed without decimals
			{                                           // and should it not be 0 ?????
				PWM1=fabs(fspeed);
				if(PWM1>maxPWM) PWM1 = maxPWM;
				PWM2=0;
				MOTOR_ON = 1;                           // This turns on motor driver
				SetDCPWM1(PWM1);
				SetDCPWM2(PWM2);
			}
			else if(fspeed>1)
			{
				PWM2=fabs(fspeed);
				if(PWM2>maxPWM) PWM2 = maxPWM;
				PWM1=0;
				MOTOR_ON = 1;
				SetDCPWM1(PWM1);
				SetDCPWM2(PWM2);
			}
			else
			{
				SetDCPWM2(0);
				SetDCPWM1(0);
				MOTOR_ON = 0;
			}
		}
		else
		{	
			SetDCPWM2(0);
			SetDCPWM1(0);
			MOTOR_ON = 0;
		}
	}
	else
	{	
		SettleCnt=SettleThrs;
		SetDCPWM2(0);
		SetDCPWM1(0);
		MOTOR_ON = 0;
	}
}



/************************************************************************************************

    Sample Fader

*************************************************************************************************/
void Start_ADC (void)
{
	SetChanADC(COMP_ADC);
	ConvertADC();  
//	AUTO_LED=!AUTO_LED;    
}

unsigned int Read_ADC(void)
{
	unsigned int FADER_POS;
	signed long ScaledPos;
	while( BusyADC() );	           
	FADER_POS = ADRESH;
	FADER_POS = FADER_POS << 8;
	FADER_POS = FADER_POS + ADRESL;
	if(!FaderReady)
	{
		return FADER_POS;
	}
	else
	{
		ScaledPos = FADER_POS;
		ScaledPos -=BotLimit;
		if(ScaledPos<0)ScaledPos=0;
		ScaledPos=ScaledPos*ScaleRatio;
		ScaledPos=ScaledPos/100;
//		if(ScaledPos<0)ScaledPos=0;
		if(ScaledPos>1023)ScaledPos=1023;
		return ScaledPos;
	}

}

void SetScale(void)
{
	unsigned long n;
	n=TopLimit-BotLimit; 		// Whole scale
	ScaleRatio = (102300)/n;
}