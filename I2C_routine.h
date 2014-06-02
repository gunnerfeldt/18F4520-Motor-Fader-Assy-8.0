char HandleI2C(void);
void myWriteI2C(unsigned int data);
unsigned char getQframe(void);
unsigned char getData(unsigned char);
void setData1(unsigned char);
void setData2(unsigned char);

extern union indata {
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
extern union outdata{
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

extern ram near union indata inbuffer;
extern ram near union outdata outbuffer;

extern unsigned char BlinkLED;
extern unsigned char LocalTouch;

extern unsigned char xID;



