## Porting to other MCUs
Intention is that minimal things have to be done to port:
1) 	a) 3 phase (complimentary if required)PWM out required. Timer1 typically used, Timer8 can be used but main loop currently would need manually changing.
	b) set up CUBE with minimum 4x ADC readings (3x phase current and voltage compulsory, phase voltage sensors optional but strongly recommended and input for throttle if required)
	c) The ADC readings must be triggered at top center of the PWM, which can either be triggered through CCR4 or the update event.
	d) The readings can either be made by the Injected or regular conversion manager. Recommended that the injected is used for JDR1 = Iu, JDR2 = Iv, JDR3 = Iw, JDR4 = Vbus.
	e) The ADC can be set up to include an out of range watchdog which can make a rapid overcurrent protection. This is hardware specific, set AWD interrupt and thresholds in CUBE, and populate the ADC interrupt handler with handleError(ERROR_ADC_OUT_OF_RANGE_IA)
	f) The mapping between the ADC readings and the FOC inputs are made in the getRawADC() function, which is implemented per project. Replicate appropriately, using example from F405 project.
2) populate the timer1 update interrupt with 	MESC_PWM_IRQ_handler() and __HAL_TIM_CLEAR_IT(&htim1,TIM_IT_UPDATE);
3) populate timer 3 or 4 interrupt with MESC_Slow_IRQ_handler(&htimx) and 	__HAL_TIM_CLEAR_IT(&htimx,TIM_IT_UPDATE);
4) Run MESCInit() in the main
5) Create your own hardware conf file which includes the methods and settings and defaults for your hardware. 
6) Map your throttle and thermistor values in the getRawADC() function in your MESC_hw_setup.c file (replicate this from a similar chip is easiest)
7) Either hard code your motor parameters in your hardware header file, or implement the autodetection every time, or implement a variant of the flash reader/writer (this is the hard bit for new MCUs)
8) Resolve the includes - your project must point to ../MESC_Common/Inc in Properties ->C/C++ General->Paths and symbols
9) Populate the fault handlers with handleError(ERROR_HARDFAULT) and other appropriate fault names
