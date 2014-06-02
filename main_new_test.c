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
 *  - disable watchdog timer
 *  - disable low voltage programming
 *
 */
 
//#pragma config WDT=OFF
//#pragma config LVP=OFF


// definitions, can be stripped
#define BUFSIZE 3
#define STATE_1 0b00001001 
#define STATE_2 0b00101001
#define STATE_3 0b00001100
#define STATE_4 0b00101100
#define STATE_5 0b00101000

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

#define fader_addr (~(PORTA & 0b11100000)>>5)

#define INT_OFF		INTCONbits.GIE = 0
#define INT_ON		INTCONbits.GIE = 1

#define TOUCH_SENSE 	210
#define TOUCH_RELEASE	150


// allocate some resources in ACCESS RAM for speedy access in interrupt routines
// all locations in this section are volatile
#pragma udata access volatile_access

volatile int FADER_POSITION = 0;
volatile int FADER_READ = 0;
volatile int FADER_GOAL = 0;
volatile unsigned int FADER_TOUCH = 0;
volatile unsigned char HARD_MOTOR_ON = 0;

volatile near unsigned char       I2CSTAT;
struct {
    unsigned BF:1;
    unsigned UA:1;
    unsigned R_W:1;
    unsigned S:1;
    unsigned P:1;
    unsigned D_A:1;
    unsigned CKE:1;
    unsigned SMP:1;
}I2CSTATbits;

volatile near union INdataUnion {
unsigned char data[BUFSIZE];
struct  {
		unsigned vca_lo:8;
		unsigned vca_hi:2;
		unsigned status:2;
		unsigned motor_on:1;
		unsigned error:2;
		unsigned mute:1;
		};
}indata;

volatile near union OUTdataUnion {
unsigned char data[BUFSIZE];
struct  {
		unsigned vca_lo:8;
		unsigned vca_hi:2;
		unsigned status:2;
		unsigned touch_sense:1;
		unsigned error:2;
		unsigned mute:1;
		};
}outdata;

// ram near unsigned int PWM;					// holds the PWM value
volatile unsigned char PWMflag = 0;
volatile unsigned char I2Cflag = 0;
volatile float fspeed;
volatile unsigned char PWMCnt = 0;
volatile unsigned char SampleCnt = 0;
volatile unsigned char TouchCnt = 0;

// fader in/out variables. Strip down please
volatile int touch = 0;
unsigned char fader_status = 0;
volatile unsigned char fader_address = 0;
unsigned char local_fader_status = 0;
unsigned char index;			// index counter for the I2C i/out buffer arrays
unsigned char address;		// where to store the dummy read. Might be used for general call

// funtion definitions
int HandleI2C(void);
void myWriteI2C(void);	
void scan_switches(void);
void InterruptHandlerHigh (void); // prototype for int handler
void InterruptHandlerLow (void);  // prototype for int handler
void Load_Ch_Digits(unsigned int);
void Handle_LEDs(void);		
void Handle_SWs(void);		
void Read_Pos(void);		
void Update_LED_Display(void);
unsigned char Scan_Touch(void);
void Read_Fader(void);		
void CalcPWM1(int);		
void CalcPWM2(int);		
void Handle_Fader(void);		
void Init(void);
unsigned char ErrorCode(unsigned char, unsigned char);


#pragma udata


// Misc variables. Please redeem!!!
const unsigned char MANUAL = 0x00;
const unsigned char AUTO = 0x04;
const unsigned char TOUCH = 0x08;
const unsigned char WRITE = 0x0C;
const unsigned char MUTE = 0x80;
const unsigned char MOTOR_ENABLE = 0x10;
const unsigned char ERROR_MASK = 0x60;
const unsigned char ERROR_CHECK = 0x40;
const unsigned char FADERSTATUS = 0x0C;


signed float lastpError;
signed float lastiError;
signed float pError;
signed float iError;
signed float dError;


unsigned int GOAL_CNT;
const unsigned int SET_TIME = 5000;
const unsigned char TouchScanRate = 2;
const unsigned char SampleRate = 0;
const unsigned char PWMupdateRate = 1;
const unsigned int maxPWM =1023;

const float pK = 25;	//30
const float iK = 3.0;	//2.5	
const float dK = 0.08;	//0.9
const float gain = 0.4;
const float intTime= 5;  // 0 - 1
const float time_interval = 0.0005;
const float derTime = -4;
const unsigned int IDrate = 0;
const int tol = 0;
const int itol = 0;
const int motorstart = 300;
const int iError_max = 300;

const int SPEED_SHIFT = 2;
const int FADER_SHIFT_PID = 2;
const int READ_SHIFT = 4;
const int GOAL_SHIFT = 5;
const int DER_SHIFT = 1;

unsigned int IDcnt = 0;
unsigned int touch_rel;
unsigned int touch_press;
unsigned char new_data = 0;
unsigned char soft_motor_on = 0;
unsigned char BlinkLED = 0;
unsigned int BlinkCnt = 0;
unsigned char DIG_TOGGLE;
unsigned char DIGIT[1];
unsigned char SEGMENTA[] = {0x00,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x55};
unsigned char SEGMENTB[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x55};
unsigned char CHANNEL_NO = 1;
unsigned int displayint;
unsigned char fader_adc;
unsigned char trouble = 0;
unsigned char ScanFlag = 0;



//----------------------------------------------------------------------------
// High priority interrupt routine (legacy interrupt)

#pragma code
#pragma interrupt InterruptHandlerHigh

void
InterruptHandlerHigh () 
{
//	INTCONbits.TMR0IE = 0;
//	PIE1bits.TMR2IE = 0;
	if(PIR1bits.SSPIF)					// if new I2C event occured
		{
		if (HandleI2C()==5)				// when I2C transfer i done	
			{	
			I2Cflag = 1;				// flag for process	
			}
		PIR1bits.SSPIF = 0;				// reset	
		}


	if (PIR1bits.TMR2IF)	
		{			
		PWMCnt++;
		TouchCnt++;
		SampleCnt++;
//		ScanFlag = 1;	
		if(BlinkCnt>200)
			{
			BlinkLED = !BlinkLED;
			BlinkCnt = 0;
			}
		BlinkCnt++;
		PIR1bits.TMR2IF = 0;
		}
//		INTCONbits.TMR0IE = 1;
//		PIE1bits.TMR2IE = 1;
}


#pragma code

void main(void)
{	
	unsigned int temp_data;
	unsigned int FADER_SET;
	unsigned int GOAL;
	static long FADER_FILTER_REG;
	static long GOAL_FILTER_REG;
	Init();


	while (1) // loop forever
	{	
//		AUTO_LED = BlinkLED;
	if (I2Cflag)
		{
//		INTCONbits.TMR0IE = 0;	
		INT_OFF;
		Load_Ch_Digits((fader_address/2)+1);
		I2Cflag = 0;
		if (indata.data[2]==ErrorCode(indata.data[0],indata.data[1]))
			{
			FADER_SET = indata.data[1] & 3;
			FADER_SET = (FADER_SET<<8)+indata.data[0];
			Handle_LEDs();
			}
		outdata.data[0] = lobyte(FADER_READ);
		outdata.data[1] = hibyte(FADER_READ);
		outdata.touch_sense = FADER_TOUCH;
		scan_switches();
		outdata.data[2] = ErrorCode(outdata.data[0],outdata.data[1]);
		
		INT_ON;	
//		INTCONbits.TMR0IE = 1;
		}	

//	if (ScanFlag)
//		{


		if(SampleCnt>SampleRate)
			{
			INT_OFF;
			Read_Fader();


				GOAL_FILTER_REG = GOAL_FILTER_REG - (GOAL_FILTER_REG >> GOAL_SHIFT) + FADER_SET;
				GOAL = GOAL_FILTER_REG >> GOAL_SHIFT;
				FADER_FILTER_REG = FADER_FILTER_REG - (FADER_FILTER_REG >> READ_SHIFT) + FADER_POSITION;
				FADER_READ = FADER_FILTER_REG >> READ_SHIFT;


			SampleCnt = 0;
			INT_ON;
			}

		if(TouchCnt>TouchScanRate)
			{
			INT_OFF;
			FADER_TOUCH = Scan_Touch();
//			WRITE_LED = !touch;
			TouchCnt = 0;
			INT_ON;
			}


		if(PWMCnt>PWMupdateRate)
			{
			INT_OFF;
			PWMCnt=0;
			fspeed = fspeed*(1-(motorstart/1023));

			Update_LED_Display();

			CalcPWM1(GOAL);
			Handle_Fader();
//			ScanFlag=0;
			INT_ON;
			}
//		}	
	}
}


void CalcPWM1 (int GOAL)
	{
	static float speed;
	unsigned float SPEED_LP;
	static unsigned int STATIC_GOAL;
	static float D_REG;
	
	FADER_GOAL = GOAL;
	pError = FADER_GOAL-FADER_POSITION;

	if (GOAL==STATIC_GOAL && touch==0)
		{GOAL_CNT++;}
	else GOAL_CNT = 0;
	
	STATIC_GOAL = GOAL;

	if (GOAL_CNT<SET_TIME)
		{
		if (fabs(pError)>=itol) iError = iError + (pError*intTime);

		{dError = (lastpError - pError)*derTime/time_interval;}

		dError = ((D_REG*1.5) + (dError*0.5)) / 2;
		D_REG = dError;

		lastpError = pError;

		if (fabs(pError)<=tol) {iError *= 0.5;}
		if (iError==0){dError=0;lastpError = 0;}

		if (iError>iError_max) iError =iError_max;
		if (iError<-iError_max) iError =-iError_max;

		SPEED_LP = speed*(SPEED_SHIFT-1);
		speed = (SPEED_LP+(((pK*pError)+(iK*iError)+(dK*dError))*gain))/SPEED_SHIFT;
		if (fabs(pError)<=tol) {speed=0;}
		fspeed = speed;
		}
	else fspeed = 0;
	}

void CalcPWM2 (int GOAL)
	{
	int Scaled_Max;	
	FADER_GOAL = GOAL;
	pError = FADER_GOAL-FADER_POSITION;	

	IDcnt++;
	if (IDcnt>IDrate)
		{
		IDcnt=0;
		if (fabs(lastpError) <= fabs(pError))	{iError += pError;}
		else {iError*= 0.5;}
		Scaled_Max = iError_max*iK;
		if (iError>Scaled_Max) iError =Scaled_Max;
		if (iError<-Scaled_Max) iError =-Scaled_Max;
		dError = (pError - lastpError)/ intTime;
		if (fabs(pError)<=tol) iError = 0;
	   
		}	
	lastpError = pError;

	fspeed = (((pK*pError)+(iK*iError)+(dK*dError))*gain);	
	}

void Handle_Fader(void)
	{
	unsigned int PWM1;
	unsigned int PWM2;

	HARD_MOTOR_ON &= indata.motor_on;
	if (HARD_MOTOR_ON == 0 || touch)
		{
		SetDCPWM2(0);
		SetDCPWM1(0);
		motor_enable = 0;
//		AUTO_LED=1;
//		TOUCH_LED=1;
//		WRITE_LED=1;
		iError = 0;
		dError = 0;
		lastpError = 0;
		return;
		}




	if(fspeed<0)
		{
		PWM1=motorstart+fabs(fspeed);
		if(PWM1>maxPWM) PWM1 = maxPWM;
		PWM2=1023-PWM1;
//		WRITE_LED=1;
//		TOUCH_LED=0;
		motor_enable = 1;
		SetDCPWM1(PWM1);
		SetDCPWM2(PWM2);
		}
	else if(fspeed>0)
		{
		PWM2=motorstart+fabs(fspeed);
		if(PWM2>maxPWM) PWM2 = maxPWM;
		PWM1=1023-PWM2;
//		TOUCH_LED=1;
//		WRITE_LED=0;
		motor_enable = 1;
		SetDCPWM1(PWM1);
		SetDCPWM2(PWM2);
		}
	else
		{
//		AUTO_LED=0;
//		TOUCH_LED=1;
//		WRITE_LED=1;
		SetDCPWM2(0);
		SetDCPWM1(0);
		motor_enable = 0;
		return;
		}
	}
unsigned char ErrorCode(unsigned char dataA, unsigned char dataB)
	{
	unsigned char checkA;
	unsigned char checkB;
	unsigned char check;
	checkA = dataA & ERROR_MASK_0;
	checkB = dataB & ERROR_MASK_1;
	check = checkA + checkB;
	return ~check;
//	return dataB;
	}

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

int HandleI2C(void)
	{	
															// copy status register		
		I2CSTAT = SSPSTAT & 0b00101101;						// mask out the used bits
															// decide the state
			if (I2CSTAT == STATE_1){						// address match, write
				index = 0;									// clear buffer index
				address = SSPBUF;							// dummy read to cear BF
				SSPCON1bits.CKP = 1; // release clock			
				return 1;}
			if (I2CSTAT == STATE_2){						// master write byte
				indata.data[index] = SSPBUF;				// read data and clear BF
				SSPCON1bits.CKP = 1; // release clock				
				index++;									// increment index
				if(index>BUFSIZE) index = 0;				// if buffer length exceeded, clear index
				return 2;}
			if ((I2CSTAT & 0b00101100) == STATE_3){			// address match, read (ignore BF bit)
				index = 0;									// clear buffer index
				myWriteI2C();								// a repeating function for writing
				index++;									// increment index
				return 3;}
			if (I2CSTAT == STATE_4){						// master read, buffer is empty. no need to read it.
				if(SSPCON1bits.CKP == 0){					// make sure the clock line hasn't been released
					if(index>BUFSIZE) index = 0;			// if buffer length exceeded, clear index
					myWriteI2C();							// a repeating function for writing
					index++;								// increment index
					return 4;}}
			if ((I2CSTAT & 0b00101000) == STATE_5){			// NACK received when sending data to the master		
			return 5;}										// Mask RW bit in SSPSTAT
		return -1;										// if no state, return error code
		}

//---------------------------------------------------------------------
// myWriteI2C
//---------------------------------------------------------------------

void myWriteI2C(void)
	{
//	while(SSPSTATbits.BF)									// wait until bus is free
//		{													
//		;
//		}
	do 
		{							
		SSPCON1bits.WCOL = 0;								// clear collision bit
		SSPBUF = outdata.data[index];						// load hardware buffer
		}while(SSPCON1bits.WCOL);							// if there was a collision, try again
		SSPCON1bits.CKP = 1;								// release the clock stretch
	}


//---------------------------------------------------------------------
// scan_switches
//---------------------------------------------------------------------
void scan_switches()
	{

	outdata.status = 0;
	if (AUTO_SW) outdata.status = 0x01;
	if (TOUCH_SW) outdata.status = 0x02;
	if (WRITE_SW) outdata.status = 0x03;
	outdata.mute = MUTE_SW;
	
//	if (AUTO_SW) {trouble = 1 ; fader_address=80;}
//	if (WRITE_SW) trouble = 0;

	}




// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

void Read_Fader(void)
	{
	static long FADER_FILTER_REG;
	unsigned int AverageRead = 0;
	unsigned int FADER_POS = 0;
	unsigned int cnt;
	SetChanADC(COMP_ADC);
	for (cnt=0;cnt<4;cnt++)
		{
		ConvertADC();
		while( BusyADC() );	
		FADER_POS = ADRESH;
		FADER_POS = FADER_POS << 8;
		FADER_POS = FADER_POS + ADRESL;
		AverageRead += FADER_POS;
		}

	FADER_POS = AverageRead / 4;
	FADER_FILTER_REG = FADER_FILTER_REG - (FADER_FILTER_REG >> FADER_SHIFT_PID) + FADER_POS;
	FADER_POSITION = FADER_FILTER_REG >> FADER_SHIFT_PID;
	}





unsigned char Scan_Touch(void)
	{
	unsigned int fader_touch_sens;
	TOUCH_X_OUTPUT;
	TOUCH_X_HIGH;
	SetChanADC(TOUCH_X_ADC);
	TOUCH_OUTPUT;
	TOUCH_LOW;
	TOUCH_INPUT;

	SetChanADC(TOUCH_ADC);
	ConvertADC();
	while( BusyADC() );
	fader_touch_sens = ADRESH;
	fader_touch_sens = fader_touch_sens << 8;
	fader_touch_sens = fader_touch_sens + ADRESL;


	if (fader_touch_sens<TOUCH_SENSE) 
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
		if (touch_rel > 16) touch_press = 0;
		if (touch_rel > TOUCH_RELEASE) 
			{touch = 0;
			}
		else 
			{touch_rel +=1;}			
		}
	return touch;
	}	


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
	

void Handle_SWs(void)
	{
	local_fader_status = 0;	
	local_fader_status += (AUTO_SW*AUTO);
	local_fader_status += (TOUCH_SW*TOUCH);
	local_fader_status += (WRITE_SW*WRITE);
//	local_fader_status += (touch_knob*0x10);
	}	
	
void Handle_LEDs(void)
	{
	static char NIBBLE_BUF;
	unsigned char nibble;
	nibble = indata.data[1]&0x0C;
	if (nibble != NIBBLE_BUF) GOAL_CNT = 0;
	NIBBLE_BUF = nibble;
	MUTE_LED = !testbitmask(indata.data[1],MUTE);
	switch (nibble)
		{
		case 0x00:
		AUTO_LED = 1;
		TOUCH_LED = 1;
		WRITE_LED = 1;
		HARD_MOTOR_ON = 0;
		break;
		case 0x04:
		AUTO_LED = 0;
		TOUCH_LED = 1;
		WRITE_LED = 1;
		HARD_MOTOR_ON = 1;
		break;
		case 0x08:
		AUTO_LED = 0;
		TOUCH_LED = 0;
		WRITE_LED = 1;
		HARD_MOTOR_ON = 1;
		break;
		case 0x0C:
		AUTO_LED = 0;
		TOUCH_LED = 1;
		WRITE_LED = 0;
		HARD_MOTOR_ON = 0;
		break;
		}
		if (FADER_TOUCH)
			{WRITE_LED = BlinkLED;}

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

	TRISEbits.PSPMODE = 0;

	TRISCbits.TRISC3 =1;
	TRISCbits.TRISC4 =1;

	OpenI2C (SLAVE_7, SLEW_ON);
	SSPCON2bits.SEN = 1;

//	fader_address = 0 * 2;	 //	Bit 0 is for READ. So, address is 00, 02, 04, etc



	// enable interrupts - legacy mode
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
	PIE1bits.SSPIE = 1; // SSP interrupt enable
// 	INTCONbits.TMR0IE=1; //enable TMR0 overflow interrupt enable bit
// 	PIE1bits.TMR2IE=1; //enable TMR0 overflow interrupt enable bit


	OpenADC(	ADC_FOSC_32     	&
				ADC_RIGHT_JUST		&
				ADC_2_TAD,
				ADC_CH12			&
				ADC_INT_OFF			&
				ADC_VREFPLUS_EXT	&
				ADC_VREFMINUS_EXT
				, 15 );
	//			ADC_VREFPLUS_EXT

	ADCON1 = 0x0C;
	fader_adc = COMP_ADC; 

//  	OpenTimer0(TIMER_INT_ON & T0_SOURCE_INT & T0_8BIT & T0_PS_1_32);

//	OpenTimer2(T2_PS_1_1 & TIMER_INT_OFF & T2_POST_1_1);	// 31.248 kHz
	OpenTimer2(T2_PS_1_1 & TIMER_INT_ON & T2_POST_1_4);	// 31.248 kHz

	
	// OpenPWM1(0 to 255)  PWM period value
	OpenPWM1(0xFF);									// Turn PWM on
	SetDCPWM1(0); 

	OpenPWM2(0xFF);									// Turn PWM on
	SetDCPWM2(0);  


	fader_address = fader_addr * 2;	 //	Bit 0 is for READ. So, address is 00, 02, 04, etc
	SSPADD = 0xE0 + fader_address; // slave address

	}

