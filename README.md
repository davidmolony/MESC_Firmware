# MESC_Firmware
Custom FOC, speed control firmware for use with the MESC_FOC_ESC hardware project
Also now ported to work on typical F405 based hardware (e.g. VESC, FSESC, custom boards...)

## Foreword:
This project is new as of 28/06/2020, and is the work of David Molony, experienced mechanical and electrical engineer, and software learner.   
Contributions from others welcome, but 
1) The project is not intended to be an all inclusive do everything like VESC. This project is intended to be minimal FOC that "just works" and is trivial to understand and build applications from
2) The project will remain in its entirety BSD 3 clause, MIT or other equivalent entirely permissively licenced.
Thanks to contributors, especially:
c0d3b453 for large amounts of helpwith C++ and teaching, 
Salavat for initial STM32 setup and teaching,
Elwin (offline) for testing, motor control idea bouncing and assistance with current controllers,
Jens (Netzpfuscher) for tidying up and contributions to SinLUT,

## Porting to other MCUs
Intention is that minimal things have to be done to port:
1) set up CUBE with minimum 4x ADC readings (current and voltage compulsory, input and phase sensors optional) and 3 phase (complimentary if required)PWM out
2) populate the timer1 update interrupt with 	MESC_PWM_IRQ_handler() and __HAL_TIM_CLEAR_IT(&htim1,TIM_IT_UPDATE);
3) populate timer 3 or 4 interrupt with MESC_Slow_IRQ_handler(&htimx) and 	__HAL_TIM_CLEAR_IT(&htimx, TIM_IT_UPDATE);
4) Run MESCInit() in the main
5) Start the timers and UART in main
6) either hard code your motor parameters, or implement the autodetection every time, or implement a variant of the flash reader/writer (this is the hard bit for new MCUs)

## Licence
This project will initially (and perhaps perpetually) contain a lot of firmware licenced under the STM Cube licence, BSD 3 clause https://opensource.org/licenses/BSD-3-Clause .   
The rest of the custom code is intended to be contained primarily in the MESC files, also BSD 3 Clause.
It is requested that if you borrow, port, refactor parts into your own code... etc this firmware, you may let the project owner know, But you are not compelled to. 
It would also be nice if concise and focused improvements were contributed back, but again, you are not compelled to.

You must retain credit for the code origin in your source, even on small segments borrowed or reimplemented. This is compelled.
If this code is borrowed in part or whole for use and published in GPLV2 V3... projects, you must grant a perpetual licence for your projects GPL code to be used permissively in this project reciprocally.
Otherwise, for commercial and permissively licenced projects, the code can be used as BSD-3-Clause. The intention of this project is to be useful to whoever wants to use, learn, build, sell... 

## Hardware:
Any STM32F303CB or F405 based 3 phase system using Timer1 for PWM High and Low and 3 shunt ADC measurement with internal or external op amps.
Specifically intended for the MESC_FOC_ESC hardware, but also running on F405 targets.  
Can probably be easily ported to other MCUs (STM32F3, F4, L4... with timer1 and a floating point unit but external opamp hardware will be needed, and checks that ADCs sample within required time).

## General methodology
Main fast loop runs on timer1 update top. There is a "hyperloop" as well, which runs on update bottom, to enable signal injection and faster PWM update
3 phase Sin wave calculated every PWM period, and FOC loop run.
Sensorless observer runs in the fastloop timer top update, and is based on the integration and clamping of fluxes in alpha beta frame. 
HFI runs in the hyperloop, timer bottom update, and is dynamically switched on and off in the slowloop.
Slowloop runs in the RCPWM input timer update interrupt. The timer should be set up to overflow and generate an interupt at 20Hz (50us) or generate an interrupt in reset mode when PWMin rising edge detected
Comparators can be set up for overcurrent events on the F303 based hardware.
ADC analog watchdog can be used on either hardware as a high priority interrupt to shutdown in case of overcurrent.

### PWM
Timer1 set up to generate complimentary centre aligned PWM with dead time, frequency 72MHz/1024 (10bit voltage)/(prescaler) = 17.6kHz (prescaler 2) or and ~35.2kHz (prescaler 1). 
Future updates will be able to change frequency, but only 35.2kHz supported presently. It may be possible to subdivide the FOC loop and the SVPWM to enable much higher PWM frequency.
Owing to the number of clock cycles per PWM period for the MCU to complete math (e.g. 72M/35.2K= 2048 clock cycles) math occurring in the interupt must be fast, and best not to support frequency above 35.2kHz.
Firmware will be primarily tested at 35.2kHz PWM (70.4kHz zero vector frequency), the harder case for fitting in the control loop. Anticipated that 17kHz might be preferable for larger motors with lower eRPMs due to better switching efficiency

Space vector modulation variant "mid point clamp" primarily used; bottom clamp implementation exists, but less effective with sensorless at low speeds and broadly incompatible with HFI.
Centring at 50% duty cycle has a few advantages - it allows recirculation through the high side FETs as well as the low side, which evens out the load on them, and cancels most offsets. 
Disadvantages - reduced sampling time for currents , more switching events, all phases floating... might be worse for EMC emmisions?
To enable easy avoidance of counter underflow on the PWM timer and higher modulation indices, bottom clamp implementation can be used for FOC control. Bit noisier, less FET sharing, but safer initially. 
SVPWM mid point clamp clips the Vd and Vq at fixed limits such that the inverse transform is limited to what will fit and is commonly seen in outrunner motors.

There is no hard limit on ERPM, but eventually it trips. Has worked to over 180kerpm. This is uselessly high in practice
Target 30000mechrpm at 6PP gives a feasible 180000erpm, with 12PWMperiods/sinwave @35kHz. ~3kHz electrical rotation frequency.
Most motors cannot cope with this speed due to stator eddie and hysteresis losses. Typical 0.2mm laminations become very inefficient at around 1kHz.

### ADC
ADC conversions are triggered by timer1 overflow on TRGO. ADCs 1,2,3 are used to get fully synchronous current readings on F303 and F405. Vbus read by ADC1 immediately after current reading.   
The main loop runs on the timer1 update interrupt, in which the new PWM values are calculated from an 8 bit  sin table, or calculator developed by Jens and FOC/sensorless observer  
ADC interrupt is now reserved for overcurrent protection events with the analog watchdog.

### Hall
Hall sensors are supported, but support will be gradually deprecated since sensorless works much better. Hall startup should become an option.
Timer3 or 4 available but not used. Can be set up in XOR activation reset mode, so gives a duration between each hall sensor change of state, which can be directly converted to a speed. For a minimum speed of 10eRPM, =(10/60)Hz=1 XOR changes/second, 1 seconds/XOR change, requires 16 bit TIM4 to clock at 65536/1=65.536kHz. This implies a max speed of 65536(Hz)/6(hall states)x60(RPM/Hz)=655360RPM electrical. With typical BLDC motors being 6PP, this enables 100000RPM mechanical measureable, but the PWM frequency is not high enough to support this!
Timer hall input is not currently used, instead the fastloop just samples and counts PWM cycles since the last change. This reduces accuracy, but improves portability.

### PWM Input
Timer3 or 4 (not used for halls) set up in reset mode, prescaler 72 (1us resolution), 65535 period, with trigger/reset mapped to TI1FP1, and channel one capturing on rising edge, direct mode, channel 2 capturing on falling edge indirect mode - remapped to TI2FP2. This gives two CC register values, CC1 timing the period of the pulses, and CC2 timing the on time.  
Interrupt: update interrupt used. We expect a value very close to 20000 from an RC sender. Check CC1 is 20000+/-~10000. If outside this bounds, set input capture flag low.

### Over Current Comparators
Comparators set up to trigger Tim1 break2 state in the event of overcurrent event, which should turn off all outputs to high impedance on F303 hardware. 
0.5mOhm shunts (2x1mohm) or 1mOhm (2x2mOhm original hardware) at 100amps gives 50mV, with a pullup to 100mV. Vrefint is 1.23V, so 1/4Vref used for comparator-ve - 310mV. This triggers the comparator at 400A nominal (a LOT of current, but the intended FETs are rated for that for 100us, which is ~2 PWM periods... If alternate FETs used, should check this, or just hope for the best, or modify the shunt resistors to have higher value.  
Timer1 should have a BRK filter set to avoid switching noise  
Soft current control e.g. constantly running at 100+amps must be taken care of by the fast control loop.
On F405 targets, the expectation is that there is a fault input on the PB12 pin, timer1 is setup to capture and stop PWM generation on this input.

## Coms
Primarily, initially, serial used. Potentially USB CDC later, and I2C

### Serial 
Serial USART3 may eventually be used in DMA recieve mode to eliminate interrupts and MCU blocking behaviour.  
Serial DMA to interrupt on overflow of buffer - Hopefully never reaches overflow of buffer.  
If overflow occurs, just throw away the whole buffer, reset DMA to load first position and await further data
Currently, Serial interrupts on a per byte basis and can be used to trigger basic functionality.

## Watchdog timer
Not currently implemented. ToDo...
Watchdog timer should be kicked by the fast control loop to ensure response to overvoltage and current (if not on ADC watchdog) events possible.
Period of ~1ms
On overflow, generate a break state on the motor and reset MCU - control loop no longer running, motor could be stopped, freewheeling, generally making a mess of currents and generating high voltages.

## Fast Control loop
Fast control loop must:  
Check for over limit events if not handled in hardware
Retrieve current values from ADC conversion  
Retrieve voltage values  
  IF logging: write currents and voltage to logging buffer, increment logging pointer  (not yet implemented)
  IF logging pointer >logging samples, set logging complete (not yet implemented)
Check state (Idle, running, tracking, fault/BRK)  
Manipulate currents and voltages to Vab (FOC)  
Calculate current position and speed (get Hallsensor values, sensorless observer, ToDo Encoder)  
Run FOC loop
Run SVPWM (include inverse clark/park)
Update inverter PWM values based on phase and voltage  

## Slow control loop
Execute every 20ms on PWM input, 50ms on overflow (UART, ADC in...
State machine processing (ToDo)

Update parameters (e.g. volts to PWM, gains...)
Run Field Weakening
Run power and current limitations

Speed Ramps etc... ToDo


## Coms loop

ToDo, currently only simple single character arguments. C0d3b453 working on an implementation...
Runs on UART RX interrupt
DMA used to transmit strings.

## Speeds, angles, input params...
### Motor params demanded will be:  
PP - Pole pairs
Finds kV as mWb x Fpwm on detection
Rph - Phase resistance (=1/2 phase:phase resistance) detected by measurement protocol
Ld - Phase inductance in Ld detected by measurement protocol as Lx = Vdinjected/(di/dt)
Lq - As Ld but found by Vq injection
Max motor current
Possible future...
OLRampCurrent - Open loop ramp current 
OLmechRPM - open loop mechanical RPM (will ramp up to this speed in open loop, and stay there until sync'd/detected)

### Inverter params will be:
FOC will take arguments of Id_req and Iq_req as floats
FOC algorithm will convert these into Vd Vq values, to be inverted into 3 phase via park and clark.

Angles and steps will be determined as a uint16_t. The whole firmware will work on the basis of eHz for speed and eAngle in uint16_t 

### Controller params will be
ToDo, calculate OL voltage ramps etc. 
ToDo, add parameters for speed steps

