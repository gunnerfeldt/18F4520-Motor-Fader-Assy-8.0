void Start_ADC(void);
unsigned int Read_ADC(void);
void myWriteI2C(unsigned int);
void Calculate_Motor_PWM (int, int);
void Handle_Fader(void);
void SetScale(void);
extern unsigned char LocalTouch;
extern unsigned int TopLimit;
extern unsigned int BotLimit;
extern unsigned char FaderReady;
extern unsigned int SettleCnt;
extern unsigned int SettleThrs;

