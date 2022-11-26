# MESC_Firmware
Custom FOC, speed control firmware for use with the MESC_FOC_ESC hardware project
Also now ported to work on typical F405 based hardware (e.g. VESC, FSESC, custom boards...)
Also ported to work on L431RC and F401CC (Blackpill)
Other STM32 ports are easy to do, and perhaps other MCU vendors.

## Foreword:
This project is new as of 28/06/2020, and is the work of David Molony, experienced mechanical and electrical engineer, and software learner.   
Contributions from others welcome, but 
1) The project is not intended to be an all inclusive do everything like VESC. This project is intended to be minimal FOC that "just works" and is trivial to understand, port and build applications from
2) The project will remain in its entirety BSD 3 clause, MIT or other equivalent entirely permissively licenced.
3) As the project matures, while the code style remains nothing special, there are increasing numbers of originally developed and both effective and easy to implement techniques that are offered permissively. 
Borrowing sections of code for other projects is allowed, but without exception, if you borrow the code, even if you rename variables or break it up/relocate into various subroutines, you MUST credit the origin, and maintain the permissive BSD licencing inline if necessary. Failure to do so means you grant a perpetual permissive licence to your project and lose your rights to using this code permissively. 
This statement is in response to a pernicious yet ubiquitous habit of open source projects ripping out permissive licencing or taking public domain code and re-issuing under copyleft terms. The origin of this ire was searching for python source and finding the entire cpython project online (since removed) with all trace of the BSD/PSF licence removed and replaced with GPL.

Thanks to contributors, especially:
c0d3b453 for large amounts of helpwith C and teaching, 
Salavat for initial STM32 setup and teaching,
Elwin (offline) for testing, motor control idea bouncing and assistance with current controllers,
Jens (Netzpfuscher) for tidying up and contributions to SinLUT,

## Noteable features
Simple and robust sensorless observer 
HFI using d-q coupled current
Dead time compensation and characterisation
100% modulation techniques
Field weakening and MTPA
Fast fault shutdown
Parameter (Rs, Ld, Lq, flux linkage)detection
Operation up to ~100kHz PWM (200kHz V0V7 frequency) with F405 MCU, operation to about 40kHz with F401 MCU and about 35kHz with F303. Stable to <2.5kHz PWM frequency, though this is definitely not advised for most applications.
Easy porting to any STM32 with a floating point unit and timer1
Probably easily portable to any other MCU with FPU, 3 phase timer and a 1MHz+ ADC

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
5) Start the slow loop timer and UART in main (Timer1 started by MESCInit())
6) Create your own hardware conf file which includes the methods and settings and defaults for your hardware. 
7) Either hard code your motor parameters in your hardware header file, or implement the autodetection every time, or implement a variant of the flash reader/writer (this is the hard bit for new MCUs)
8) Resolve the includes - your project must point to ../MESC_Common/Inc in Properties ->C/C++ General->Paths and symbols
9) Populate the fault handlers with handleError(ERROR_HARDFAULT) and other appropriate fault names
## Licence
This project will initially (and perhaps perpetually) contain a lot of firmware licenced under the STM Cube licence, BSD 3 clause https://opensource.org/licenses/BSD-3-Clause .   
The rest of the custom code is intended to be contained primarily in the MESC files, also BSD 3 Clause.
It is requested that if you borrow, port, refactor parts into your own code... etc this firmware, you may let the project owner know, But you are not compelled to. 
It would also be nice if concise and focused improvements were contributed back, but again, you are not compelled to.

You must retain credit for the code origin in your source, even on small segments borrowed or reimplemented. This is compelled.
If this code is borrowed in part or whole for use and published in GPLV2 V3... projects, you must grant a perpetual licence for your projects GPL code to be used permissively in this project reciprocally.
Otherwise, for commercial and permissively licenced projects, the code can be used as BSD-3-Clause. The intention of this project is to be useful to whoever wants to use, learn, build, sell... 

## Hardware:
Any STM32 based 3 phase system using Timer1 for PWM High and Low and 3 phase current plus bus voltage measurement. Canonical hardware is F303, but better results can be made with external amplification so the F303 is no longer preferred.
Preferable to have phase voltage sensors for restart while spinning and tracking without modulation (gives reduced drag and better efficiency)
Specifically intended for the MESC_FOC_ESC hardware, but also running on F405, F401, L431 and F411 targets.  

## General methodology
Main fast loop runs on timer1 update top. There is a "hyperloop" as well, which runs on update bottom, to enable signal injection and faster PWM update
3 phase Sin wave calculated every PWM period, and FOC loop run.
Sensorless observer runs in the fastloop timer top update, and is based on the integration and clamping of fluxes in alpha beta frame. 
HFI runs in the hyperloop, timer bottom update, and is dynamically switched on and off in the slowloop.
Slowloop runs in the RCPWM input timer update interrupt. The timer should be set up to overflow and generate an interupt at 20Hz (50us) or generate an interrupt in reset mode when PWMin rising edge detected
Comparators can be set up for overcurrent events on the F303 based hardware, or an external BRK or Trip input can be put on PB12 to tri-state the bridge.
ADC analog watchdog can be used as a high priority interrupt to shutdown in case of overcurrent or ADC out of range.

### PWM
Timer1 set up to generate complimentary centre aligned PWM with dead time, frequency configurable.
Owing to the number of clock cycles per PWM period for the MCU to complete math (e.g. 72M/35.2K/2= 1024 clock cycles) math occurring in the interrupt must be fast.
Firmware will be targetting a <1000 clock cycles per Fastloop and <<1000 cycles per hyperloop to enable high frequency operation. Therefore, many functions and precalculations are pushed into the slowloop.

Space vector modulation variant "mid point clamp" primarily used; bottom clamp implementation exists, but less effective with sensorless at low speeds and broadly incompatible with HFI.
Centring at 50% duty cycle has a few advantages - it allows recirculation through the high side FETs as well as the low side, which evens out the load on them, and cancels most offsets. 
Disadvantages - reduced sampling time for currents , more switching events, all phases floating... might be worse for EMC emmisions?
For higher modulation indices, bottom clamp implementation can be used for FOC control. Bit noisier, less FET sharing, but allows for longer current sampling periods, and therefore higher modulation. 
SVPWM mid point clamp clips the Vd and Vq at fixed limits such that the inverse transform is limited to what will fit and is commonly seen in outrunner motors.


There is no hard limit on ERPM, but eventually it trips. Has worked to over 180kerpm. This is uselessly high in practice
Target 30000mechrpm at 6PP gives a feasible 180000erpm, with 12PWMperiods/sinwave @35kHz. ~3kHz electrical rotation frequency.
Most motors cannot cope with this speed due to stator eddie and hysteresis losses. Typical 0.2mm laminations become very inefficient at around 1kHz.

### ADC
ADC conversions are triggered by timer1 overflow on TRGO. ADCs 1,2,3 are used to get fully synchronous current readings on F303 and F405. Vbus read by ADC1 immediately after current reading.   
The main loop runs on the timer1 update interrupt, in which the new PWM values are calculated from an 8 bit  sin table, or calculator developed by Jens and FOC/sensorless observer  
ADC interrupt is now reserved for overcurrent protection events with the analog watchdog.

### Hall
Hall sensors are supported, but only in forward mode (swap a pair of motor phase wires if it runs backwards) but full support will be gradually deprecated since sensorless works much better. Hall startup optioncoming.
Timer3 or 4 available but not used. Can be set up in XOR activation reset mode, so gives a duration between each hall sensor change of state, which can be directly converted to a speed. For a minimum speed of 10eRPM, =(10/60)Hz=1 XOR changes/second, 1 seconds/XOR change, requires 16 bit TIM4 to clock at 65536/1=65.536kHz. This implies a max speed of 65536(Hz)/6(hall states)x60(RPM/Hz)=655360RPM electrical. With typical BLDC motors being 6PP, this enables 100000RPM mechanical measureable, but the PWM frequency is not high enough to support this!
Timer XOR hall input is not currently used, instead the fastloop just samples and counts PWM cycles since the last change. This reduces accuracy, but improves portability.

### Encoder
TLE5012B crudely supported in SSC (SPI) mode. 
ABI not currently supported. 

### PWM Input
Timer3 or 4 (not used for halls) set up in reset mode, prescaler 72 (1us resolution), 65535 period, with trigger/reset mapped to TI1FP1, and channel one capturing on rising edge, direct mode, channel 2 capturing on falling edge indirect mode - remapped to TI2FP2. This gives two CC register values, CC1 timing the period of the pulses, and CC2 timing the on time.  
Interrupt: update interrupt used. We expect a value very close to 20000 from an RC sender. Check CC1 is 20000+/-~10000. If outside this bounds, set input capture flag low.

### Over Current Comparators
Comparators set up to trigger Tim1 break2 state in the event of overcurrent event, which should turn off all outputs to high impedance on F303 hardware. 
0.5mOhm shunts (2x1mohm) or 1mOhm (2x2mOhm original hardware) at 100amps gives 50mV, with a pullup to 100mV. Vrefint is 1.23V, so 1/4Vref used for comparator-ve - 310mV. This triggers the comparator at 400A nominal (a LOT of current, but the intended FETs are rated for that for 100us, which is ~2 PWM periods... If alternate FETs used, should check this, or just hope for the best, or modify the shunt resistors to have higher value.  
Timer1 should have a BRK filter set to avoid switching noise  
On F405 targets, the expectation is that there is a fault input on the PB12 pin, timer1 is setup to capture and stop PWM generation on this input.
Hardware without overcurrent comparators is OK, the overcurrent and over voltage is also taken care of in the fastloop, or by the analog watchdog if set up.

## Coms
Primarily, initially, serial used. 
USB CDC (serial) implemented for F303, F401 and F405 targets

### Serial 
TBC.

## Watchdog timer
Not currently implemented. Lack of it has not presented any issue so far. ToDo...
Watchdog timer should be kicked by the fast control loop after the VIcheck is completed to ensure response to overvoltage and current (if not on ADC watchdog) events possible.
Period of ~1ms 
On overflow, generate a break state on the motor and reset MCU - control loop no longer running, motor could be stopped, freewheeling, generally making a mess of currents and generating high voltages.

## Fast Control loop
Fast control loop must:  
Check for over limit events if not handled in hardware
Retrieve current values from ADC conversion  
Retrieve voltage values  
  IF logging: write currents and voltage to logging buffer, increment logging pointer  (not yet implemented)
  IF logging pointer >logging samples, set logging complete (not yet implemented)
Check state (Idle, running, tracking, fault/BRK...)  
Manipulate currents and voltages to Vab (FOC)  
Calculate current position and speed (get Hallsensor values, sensorless observer, ToDo Encoder)  
Calculate Field weakening
Run FOC loop
Run SVPWM (include inverse clark/park)
Update inverter PWM values based on phase and voltage  

## Slow control loop
Execute every 20ms on PWM input, 50ms on overflow (UART, ADC in...)
Without RCPWM input, or with RCPWM processed elsewhere, the slowloop can be run faster (recommended <<1kHz) on any timer with priority second to the fastloop timer and ADC interrupt (if used)
State machine processing (ToDo)

Update parameters (e.g. volts to PWM, gains...)
Run MTPA
Run power and current limitations

Speed Ramps etc... ToDo


## Coms loop

ToDo, currently only simple single character arguments. C0d3b453 working on an implementation...
Runs on UART RX interrupt
DMA used to transmit strings.

## Speeds, angles, input params...
### Motor params demanded will be:  
PP - Pole pairs
Finds kV as mWb on detection
Rph - Phase resistance (=1/2 phase:phase resistance) detected by measurement protocol
Ld - Phase inductance in Ld detected by measurement protocol as Lx = Vdinjected/(di/dt)
Lq - As Ld but found by Vq injection
Max motor current
Max power
Switching frequency
Possible future...
OLRampCurrent - Open loop ramp current 
OLmechRPM - open loop mechanical RPM (will ramp up to this speed in open loop, and stay there until sync'd/detected)

### Inverter params will be:


### Controller params will be


