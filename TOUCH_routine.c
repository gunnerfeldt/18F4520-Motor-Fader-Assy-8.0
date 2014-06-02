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