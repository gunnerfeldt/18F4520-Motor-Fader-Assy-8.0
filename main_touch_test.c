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


/* Set configuration bits:
 *  - set HSPLL oscillator (10Mhz xtal, 40Mhz Osc
 *  - disable watchdog timer
 *  - disable low voltage programming
 *
 */
 
//#pragma config OSC= HSPLL
//#pragma config WDT=OFF
//#pragma config LVP=OFF

#define BUFSIZE 2
// #define SW  !PORTCbits.RC6
// allocate some resources in ACCESS RAM for speedy access in interrupt routines
// all locations in this section are volatile
#pragma udata access volatile_access

ram near unsigned char sspcon1copy;
ram near unsigned char address;
ram near unsigned char data;
ram near unsigned char rxbuf[3];
ram near unsigned char txbuf[3];
ram near unsigned char rxcount, txcount;
ram near unsigned int PWM;
ram near unsigned char check;
int fader_goal = 0;
int fader_int_goal = 0;
double fader_dbl_goal = 0;
int fader_compare = 0;
unsigned char touch;
unsigned int fader_pos = 0;
unsigned char fader_status = 0;
unsigned char fader_address = 0;
unsigned char local_fader_status = 0;
unsigned char indata[2];

#pragma udata

char PWMflag =0;
char I2Cflag =0;

const unsigned char MANUAL = 0x00;
const unsigned char AUTO = 0x04;
const unsigned char TOUCH = 0x08;
const unsigned char WRITE = 0x0C;
const unsigned char MUTE = 0x80;
const unsigned char MOTOR_ENABLE = 0x10;
const unsigned char ERROR_MASK = 0x60;
const unsigned char ERROR_CHECK = 0x40;
const unsigned char FADERSTATUS = 0x0C;
const int touch_sensivity = 261;			//261
float lastpError;
float lastiError;
float pError;
float iError;
float dError;
float pK;
float iK;
float dK;
float gain;
float time_interval;
float derTime;
float intTime;
int tol;


unsigned char I2C_started = 0;
int motorstart;
unsigned int speed;
float fspeed;
int iError_max;

unsigned int simCnt = 0;
int sim = 100;


unsigned int fader_touch_sens = 0;
unsigned char touch_knob = 0;
unsigned int touch_rel;
unsigned int touch_press;
unsigned char new_data = 0;
unsigned char soft_motor_on = 0;
unsigned char BlinkLED = 0;
unsigned int BlinkCnt = 0;
//unsigned int FakeCnt = 0;
unsigned char PWMCnt = 0;
unsigned double acc_fader_pos;
unsigned char DIG_TOGGLE;
unsigned char DIGIT[1];
unsigned char SEGMENTA[] = {0x00,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
unsigned char SEGMENTB[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
unsigned char CHANNEL_NO = 1;
unsigned int displayint;

#define motor_enable LATCbits.LATC0

#define AUTO_LED  LATBbits.LATB0
#define TOUCH_LED  LATBbits.LATB1
#define WRITE_LED  LATBbits.LATB2
#define MUTE_LED  LATBbits.LATB3

#define SEG_A  LATDbits.LATD0
#define SEG_B  LATDbits.LATD1
#define SEG_C  LATDbits.LATD2
#define SEG_D  LATDbits.LATD3
#define SEG_E  LATDbits.LATD4
#define SEG_F  LATDbits.LATD5
#define SEG_G  LATDbits.LATD6
#define SEG_DP  LATDbits.LATD7
#define DIG_0  LATEbits.LATE0
#define DIG_1  LATEbits.LATE1

#define ON	0
#define OFF 1

#define AUTO_SW  !PORTCbits.RC5
#define TOUCH_SW  !PORTCbits.RC6
#define WRITE_SW  !PORTCbits.RC7
#define MUTE_SW  !PORTAbits.RA4

#define COMP_ADC		ADC_CH0
#define TOUCH_ADC		ADC_CH7
#define TOUCH_X_ADC		ADC_CH1
#define TOUCH_X_OUTPUT	TRISAbits.TRISA1 = 0
#define TOUCH_X_HIGH	LATAbits.LATA1 = 1

#define TOUCH_OUTPUT	TRISEbits.TRISE2 = 0
#define TOUCH_INPUT		TRISEbits.TRISE2 = 1
#define TOUCH_HI		LATEbits.LATE2 = 1
#define TOUCH_LOW		LATEbits.LATE2 = 0

unsigned char fader_adc;

#define bit_set(var,bitno) ((var) |= 1 << (bitno))
#define bit_clr(var,bitno) ((var) &= ~(1 << (bitno)))
#define testbit_on(data,bitno) ((data>>bitno)&0x01)
#define testbitmask(data,bitmask) ((data&bitmask)==bitmask)

#define bits_on(var,mask) var |= mask
#define bits_off(var,mask) var &= ~0 ^ mask

#define hibyte(x)       (unsigned char)(x>>8)
#define lobyte(x)        (unsigned char)(x & 0xFF)

#define fader_addr (~(PORTA & 0b11100000)>>5)

void InterruptHandlerHigh (void); // prototype for int handler
void InterruptHandlerLow (void); // prototype for int handler
int Handle_I2C(void);	
void Load_Ch_Digits(unsigned int);
void Handle_LEDs(void);		
void Handle_SWs(void);		
void Read_Pos(void);		
void Update_LED_Display(void);
unsigned char Scan_Touch(void);		
void Calculate_PWM(void);		
void Run_Motor(void);		
void Init(void);				


//----------------------------------------------------------------------------
// High priority interrupt vector (legacy interrupt)
#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}
/*
// Low priority interrupt vector
#pragma code InterruptVectorLow = 0x18
void
InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow //jump to interrupt routine
  _endasm
}
*/

//----------------------------------------------------------------------------
// High priority interrupt routine (legacy interrupt)

#pragma code
#pragma interrupt InterruptHandlerHigh

void
InterruptHandlerHigh () 
{
//					AUTO_LED =!AUTO_LED;
//	INTCONbits.TMR0IE = 0;
	PIE1bits.TMR2IE = 0;

	if (PIR1bits.SSPIF == 1)									// its an I2C interrupt
	{	
		PIR1bits.SSPIF = 0; // clear interrupt flag
		sspcon1copy = SSPCON1;
		sspcon1copy &= 0b00000110;
//		if (SSPSTATbits.BF) AUTO_LED = !AUTO_LED;
		if (sspcon1copy == 0b00000110)							// its a master
			{
			if (SSPSTATbits.R_W == 0)							// its a WRITE from master
				{	
				if (SSPSTATbits.D_A == 0)						// its an ADDRESS - STATE 1:
					{				
					address = SSPBUF; 							// address not used in this test
					SSPCON1bits.CKP = 1; // release clock
					rxcount = 0;
					txcount = 0;
					}

				else											// its DATA - STATE 2:
					{
					if (SSPSTATbits.BF)					// there's DATA in the SSPBUF
						{	
						data = SSPBUF;	
						SSPCON1bits.CKP = 1; // release clock
						if (rxcount < 3)
							{	
							rxbuf[rxcount]=data;
							if (rxcount==2) check =rxbuf[2];	
							if (SSPCON1bits.SSPOV)
								{
								SSPCON1bits.SSPOV = 0;
								}
							else
								{
								rxcount++;
									new_data = (rxcount==3);
								}
							}
						}				
					}
				}
			else												// its a READ by master
				{

				if (SSPSTATbits.D_A == 0)						// its an ADDRESS - STATE 3:
					{				
					address = SSPBUF; 							// address not used in this test
					SSPCON1bits.CKP = 1; // release clock
					rxcount = 0;
					txcount = 0;
					}
				else											// its DATA - STATE 2:
					{		
					data = SSPBUF; // dummy read
					if (txcount < 3) 								// is there a byte to transmit?
						{
						
						SSPBUF = txbuf[txcount]; 				// send byte
						SSPCON1bits.CKP = 1; // release clock
				

						if (SSPCON1bits.WCOL)
							{
							SSPCON1bits.WCOL = 0;
							}
						else
							{
							txcount++;						
							}		
						}

					
					}
						SSPCON1bits.WCOL = 0;
						SSPCON1bits.SSPOV = 0;
				}	
			}
		
	}

		if (PIR1bits.TMR2IF)	
			{				
			if(PWMCnt>3)
				{
				PWMflag = 1;
				PWMCnt = 0;
				}		
		
			if(BlinkCnt>254)
				{
				BlinkLED = !BlinkLED;
				BlinkCnt = 0;

				}


			BlinkCnt++;
			PWMCnt++;
			PIR1bits.TMR2IF = 0;
			}






		PIE1bits.TMR2IE = 1;
//		INTCONbits.TMR0IE = 1;
//		SSPCON1bits.CKP = 1; // release clock line
}
/*
#pragma interrupt InterruptHandlerLow

void InterruptHandlerLow(void)
{

}
*/
#pragma code 

void main(void)
{
	Init();
	while (1) // loop forever
	{
		if (new_data)
			{			
			INTCONbits.GIE = 0;
			if(Handle_I2C()==1){
				Handle_SWs();
				Handle_LEDs();				
				new_data =0;
//				fader_dbl_goal = (fader_int_goal*0.50) + (fader_goal*0.50);
//				fader_int_goal = fader_dbl_goal;
//				fader_int_goal = fader_goal;		// kan det vara det här som strulat?
//				fader_int_goal = 512;		//	verkar oxå funka
				}
			INTCONbits.GIE = 1;
			}
//		}


		if (PWMflag ==1)
			{
	
				Load_Ch_Digits((fader_address/2)+1);

				Update_LED_Display();

				INTCONbits.GIE = 0;
				touch_knob = Scan_Touch();
//				fader_int_goal = 512;		//funkar oxå
				Calculate_PWM();	
				Run_Motor();	
				INTCONbits.GIE = 1;
				PWMflag=0;
			}

	}
}

int Handle_I2C(void)
	{
	int success = 0;
	if ((rxbuf[2] & 0x60)==0x40)			// check bit 5,6 for errors
		{
		if (rxbuf[0]==(rxbuf[1] | rxbuf[2]))			// 
			{
	//		AUTO_LED = !AUTO_LED;
	
			indata[0] = rxbuf[1];
			indata[1] = rxbuf[2];
		
			fader_goal = (indata[1] & 0x03);
			fader_goal = fader_goal << 8;
			fader_goal =  fader_goal + indata[0];
			fader_int_goal = fader_goal;
	
			soft_motor_on = testbitmask(indata[1],MOTOR_ENABLE);
			success = 1;
			}
		}	

		txbuf[0] = lobyte(fader_pos);
		txbuf[1] = hibyte(fader_pos)+local_fader_status+0x40;
		txbuf[2] = (txbuf[0]&0xF0)+(txbuf[1]&0x0F);	
//		txbuf[0] = 0;
//		txbuf[1] = 2;
//		txbuf[2] = 2;	
	return success;
	}

unsigned char Scan_Touch(void)
	{		
	TOUCH_X_OUTPUT;
	TOUCH_X_HIGH;
	SetChanADC(TOUCH_X_ADC);
	TOUCH_OUTPUT;
	TOUCH_LOW;
	TOUCH_INPUT;

//	TOUCH_OUTPUT;
//	TOUCH_HI;
//	TOUCH_LOW;
//	TOUCH_INPUT;

	SetChanADC(TOUCH_ADC);
	ConvertADC();
	while( BusyADC() );
	fader_touch_sens = ADRESH;
	fader_touch_sens = fader_touch_sens << 8;
	fader_touch_sens = fader_touch_sens + ADRESL;


	if (fader_touch_sens<200) 
		{	
		if (touch_press > 0) 
			{touch = 1;
			touch_rel = 0;}
		else 
			{
			touch_press +=1;
			BlinkLED = 0;
			BlinkCnt = 0;
			}
		}
	else
		{
//		touch_press -=1;
		if (touch_rel > 16) touch_press = 0;
		if (touch_rel > 48) 
			{touch = 0;
//			touch_press = 0;
			}
		else 
			{touch_rel +=1;}			
		}
	return touch;
	}	



//#define SEG_A  LATDbits.LATD0
//#define SEG_B  LATDbits.LATD1
//#define SEG_C  LATDbits.LATD2
//#define SEG_D  LATDbits.LATD3
//#define SEG_E  LATDbits.LATD4
//#define SEG_F  LATDbits.LATD5
//#define SEG_G  LATDbits.LATD6
//#define SEG_DP  LATDbits.LATD7
//#define DIG_0  LATEbits.LATE0
//#define DIG_1  LATEbits.LATE1

void Load_Ch_Digits (unsigned int displayint)
	{
	DIGIT[1] = ~SEGMENTA[displayint/10];
	DIGIT[0] = ~SEGMENTB[displayint%10];
	}

void Update_LED_Display(void)
	{
	unsigned char toggle = DIG_0;	// store last digit enabled
	DIG_0 = 1;
	DIG_1 = 1;
	PORTD = DIGIT[toggle];
	DIG_0 = !toggle;
	DIG_1 = toggle;
	}

void Read_Pos(void)
	{	
//	SetChanADC(ADC_CH0);
//	ConvertADC();
//	while( BusyADC() );
//	
//	fader_pos = ADRESH;
//	fader_pos = fader_pos << 8;
//	fader_pos = fader_pos + ADRESL;
	
	fader_pos = fader_compare;

//	fader_pos++;
//	if(fader_pos==1024) fader_pos=0;
	
	}	


void Calculate_PWM(void)
	{	
	SetChanADC(COMP_ADC);
	ConvertADC();
	while( BusyADC() );
	

	fader_compare = ADRESH;
	fader_compare = fader_compare << 8;
	fader_compare = fader_compare + ADRESL;
	acc_fader_pos = (acc_fader_pos*0.5) + (fader_compare*0.5);
	fader_compare = acc_fader_pos;
	fader_pos = fader_compare;
	
//			fader_int_goal = 512;	// OOOOBBBSSS !!! ta bort. men det funkar

			pError = fader_int_goal-fader_compare;

			if (fabs(pError)<tol) iError =0; 

			if (iError>iError_max) iError =iError_max;
			if (iError<-iError_max) iError =-iError_max;

			iError = iError + (lastpError*intTime);
			if (fabs(lastpError)>fabs(pError)) iError *= (1-intTime);

			{dError = (lastpError - pError)*derTime/time_interval;}



			fspeed = (((pK*pError)+(iK*iError)+(dK*dError))*gain);
			
			lastpError = pError;

				
			speed = fabs(fspeed);

	
			if(speed>1023) speed = 1023;
			PWM=speed;
	}



void Handle_SWs(void)
	{
	local_fader_status = 0;	
	local_fader_status += (AUTO_SW*AUTO);
	local_fader_status += (TOUCH_SW*TOUCH);
	local_fader_status += (WRITE_SW*WRITE);
	local_fader_status += (touch_knob*0x10);
	}	
	
void Handle_LEDs(void)
	{
	unsigned char nibble;
	nibble = indata[1]&0x0C;
	MUTE_LED = !testbitmask(indata[1],MUTE);
	switch (nibble)
		{
		case 0x00:
		AUTO_LED = 1;
		TOUCH_LED = 1;
		WRITE_LED = 1;
		break;
		case 0x04:
		AUTO_LED = 0;
		TOUCH_LED = 1;
		WRITE_LED = 1;
		break;
		case 0x08:
		AUTO_LED = 0;
		TOUCH_LED = 0;
		WRITE_LED = 1;
		break;
		case 0x0C:
		AUTO_LED = 0;
		TOUCH_LED = 1;
		WRITE_LED = 0;
		break;
		}
		if (touch_knob)
			{WRITE_LED = BlinkLED;}

	}		
void Run_Motor(void)
	{
	unsigned int dutycycle ;

	if (touch)
		{
		SetDCPWM2(0);
		SetDCPWM1(0);
		motor_enable = 0;
		return;
		}
		
	
//	PWM *= PWM;PWM/=1023;
	dutycycle = PWM;

	if (fabs(fader_compare-fader_int_goal)>tol)
		{
		if (dutycycle<motorstart) dutycycle = 0;
//			{
			if(fspeed>0)
				{
				SetDCPWM2(dutycycle);
				SetDCPWM1(0);
//				SetDCPWM2(1023-dutycycle);
				motor_enable = soft_motor_on;
//				motor_enable = 1;

				}
			else
				{
				SetDCPWM1(dutycycle);
				SetDCPWM2(0);
//				SetDCPWM1(1023-dutycycle);
				motor_enable = soft_motor_on;
//				motor_enable = 1;
				}

		}	
	else
		{

		SetDCPWM2(0);
		SetDCPWM1(0);
		motor_enable = 0;
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
	// initialise buffer ptrs and counts to 0
	rxcount = txcount = 0;

	TRISA = 0xFF;
	TRISB = 0xF0;	
	TRISC = 0xF8;
	TRISD = 0x00;
	TRISE = 0xFC;

	TRISEbits.PSPMODE = 0;

	TRISCbits.TRISC3 =1;
	TRISCbits.TRISC4 =1;
	// setup I2C module
//	SSPCON1 = 0x36 + 0x00; 	//  bit 5=1: enable I2C
							//  bit 4=1: release clock
							//  bits3:0: SSPM3:SSPM0; I2C Slave mode with 7bit address, no interrupts on start or stop bits

	OpenI2C (SLAVE_7, SLEW_ON);
//	SSPADD = 24; // for 400Khz // ( (Fosc/Bitrate)/4 ) - 1
	SSPCON2bits.SEN = 1;

//	fader_address = 0 * 2;	 //	Bit 0 is for READ. So, address is 00, 02, 04, etc



	// enable interrupts - legacy mode
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
	PIE1bits.SSPIE = 1; // SSP interrupt enable
// 	INTCONbits.TMR0IE=1; //enable TMR0 overflow interrupt enable bit
 	PIE1bits.TMR2IE=1; //enable TMR0 overflow interrupt enable bit

//  	OpenTimer0 (TIMER_INT_ON & T0_SOURCE_INT & T0_8BIT & T0_PS_1_256);

	OpenADC(	ADC_FOSC_32     &
				ADC_RIGHT_JUST	&
				ADC_2_TAD,
				ADC_CH12		&
				ADC_INT_OFF		&
				ADC_VREFPLUS_EXT
				, 15 );
	//			ADC_VREFPLUS_EXT
	ADCON1 = 0x0C;
//	T2CON.TMR2ON = 1;
	
//	AUTO_LED = 0;


// for L6202 chip


	fader_adc = COMP_ADC; 
	tol = 0;
	gain = 0.500;				// 1
	pK = 25;				
	iK = 25.000; // 10
	dK = 0.15;	// 0.20
	intTime = 0.5;			// 0.19
	iError_max = 2023;
	derTime = -2;			// 1
	time_interval = 0.0015;	// 1
	motorstart =0;


// for TCA0372 chip

/*
	fader_adc = COMP_ADC; 
	tol = 1;
	gain = 0.700;				// 1
	pK = 25;				
	iK = 15; // 10
	dK = 0.2;	// 0.20
	intTime = 0.5;			// 0.19
	iError_max = 2023;
	derTime = -2;			// 1
	time_interval = 0.0015;	// 1
	motorstart =100;
*/

	// T2_PS_1_1, T2_PS_1_4, T2_PS_1_16 - Prescaler
	// T2_POST_1_1 to 1_16 - Postscaler, not for PWM. Only for interrupt
//	OpenTimer2(T2_PS_1_16 & TIMER_INT_ON & T2_POST_1_8);	// 1.952 kHz
	OpenTimer2(T2_PS_1_1 & TIMER_INT_ON & T2_POST_1_16);	// 31.248 kHz

	
	// OpenPWM1(0 to 255)  PWM period value
	OpenPWM1(255);									// Turn PWM on
	SetDCPWM1(0); 

	OpenPWM2(255);									// Turn PWM on
	SetDCPWM2(0);  


	fader_address = fader_addr * 2;	 //	Bit 0 is for READ. So, address is 00, 02, 04, etc
	SSPADD = 0xE0 + fader_address; // slave address
	}

