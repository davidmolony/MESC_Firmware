## Hardware:
Any STM32 based 3 phase system using Timer1 for PWM High and Low and 3 phase current plus bus voltage measurement. Canonical hardware is F303, but better results can be made with external amplification so the F303 is no longer preferred.
Preferable to have phase voltage sensors for restart while spinning and tracking without modulation (gives reduced drag and better efficiency)
Specifically intended for the MESC_FOC_ESC and MP2 hardware, but also running on F405, F401, L431 and F411 targets.  

## Forenote:
After the explanation in the PWM section, it will be assumed that timer1 refers to any timer used for the PWM generation, which is propogated as mtr[n]->mtimer after being introduced as a handle in the PWM ISR. 
Multi motor is a new concept originating December 2022.
PWM frequency and V0V7 frequency are distinct, and there is latent confusion since some codebases and manufacturers like to cite ambiguously "switching frequency" since it gives the impression the system is capable of double the frequency it really is.
When choosing a PWM frequency when coming from another motor control system, be aware the convention used. VESC, ASI some BLDC controllers and probably others cite switching frequency. MESC uses PWM frequency so you may need to choose a value half what you used before, e.g. VESC 30kHz is equivalent to MESC 15kHz.


### Center Aligned PWM
//ToDo, add pictures and exlpanation of PWM center aligned generation for 3 phase currents. Anyone already reading this... There are quite a few resources that can explain this on the web already.

### PWM
Timer1 (or Timer 8 if using dual motor) set up to generate complimentary centre aligned PWM with dead time, with frequency configurable using #define PWM_FREQUENCY 25000 in your hardware apecific header.

These timers are assigned an alias specific to the motor instance such that mtr[0]->mtimer = htim1 and mtr[1]->mtimer = htim8. It is also possible to use htim20 on large STM devices.

The center aligned PWM counts (mtimer->Instance->CNT) up from 0 to ARR (Auto Reload), where the direction is changed and it starts to count down. When the count is below the CCR (Compare Register) the timer outputs HIGH to the high side MOS and LOW to the low side MOS, and the phase is connected to VBat. When it is above the CCR, the two outputs swap, and the phase is connected to ground.
This means that a higher CCR gives a larger pulse of bus voltage to the motor.
 
Space vector modulation variant "mid point clamp" primarily used; bottom clamp implementation exists, and is used for overmodulation, but results in less good FET currnt sharing.
Centring at 50% duty cycle has a few advantages - it allows recirculation through the high side FETs as well as the low side, which evens out the load on them, and cancels most offsets. 
The SVM is implemented as the true inverse clarke transform (2 phase to 3 phase) followed by selection of the highest and lowest voltage phases to generate a mid point. This mid point is then fixed to mtimer->ARR/2, effectively allowing the unipolar DC link to generate AC voltages.

For higher modulation indices (typically >95%), bottom clamp implementation can be used for FOC control. It is a bit noisier, less FET sharing, but allows for longer current sampling periods, and therefore higher modulation. 
SVPWM mid point clamp assumes the circle limiter clips the Vd and Vq such that the inverse transform is limited to what will fit within the bus voltage.

There is no hard limit on ERPM (= 60 x eHz), but eventually it trips. Has worked to over 300kerpm on F405 targets with a GaN power stage. This is uselessly high in practice.
Most motors cannot cope with this speed due to stator eddie and hysteresis losses. Typical 0.2mm laminations become very inefficient at around 1kHz.
The limit of proper commutation is roughly 20PWM periods per eHz, so at 20kHz PWM, it is possible to run to 1000eHz with good performance. This can be increased slightly by enabling interpolation (#define INTERPOLATE_V7_ANGLE)

#### PWM ISR (Interrupt Service Routine, aka IRQ) generation
The mtimer generates the interrupts for the FOC, with interrupts enabled at over and underflow of the timer (when CNT = 0 and ARR). The configured ISR this enters through is TIM1_UP_TIM10 at vector address 0x0000 00A4 on STM32F405. ST remap this to the "stm32f4xx_it.c" where it enters the function: 
void TIM1_UP_TIM10_IRQHandler(void)
Within this ST calllback, MESC calls 
MESC_PWM_IRQ_handler(&mtr[n]) where n would typically be 0 for the timer1 callback, 1 for timer8 callback and 2 for timer20 if used. This function is implemented in MESC_Common in MESCfoc.c

From here on, the code is largely generic to MESC (exception being the getRawADC(_motor) function which has to return the specific ADC readings mapping the ADC to the MESC RAW data.

Owing to the number of clock cycles per PWM period for the MCU to complete math (e.g. for STM32F303 72MHzcore/25KHzpwm/2 = 1440 clock cycles) math occurring in the interrupt must be fast.
Firmware will be targetting <1000 clock cycles per Fastloop and <<1000 cycles per hyperloop to enable high frequency operation. Therefore, many functions and precalculations are pushed into the slowloop.

The MESC_PWM_IRQ_handler calls fastLoop(_motor) when the timer is counting down, and hyperloop(_motor) when the timer is counting up (note that _motor represents mtr[0], mtr[1], mtr[2]... depending on what handle was passed in the PWM ISR function!)
fastLoop contains the FOC control, the feedback between current measurement and voltage output and the sensorless observer.
hyperLoop contains the HFI injection function, angle interpolation and when used, synchronous encoder readings. 
Both contain a call to the SVPWM function writePWM(_motor) so that the PWM can be updated 2x per PWM period.

### ADC
ADC conversions are triggered (preferably, where available) by mtimer CCR4 on TRGO. ADCs 1,2,3 are used to get fully synchronous current readings on F303 and F405, but on other targets, or with multiple motors, they sometimes must be sequential. This does not in practice result in a noticeable loss of performance. Vbus is read immediately after current reading as the next most critical parameter.

Within the fastLoop, which occurs at mtimer top shortly after the ADC conversion has been initiated, a call is made to ADCConversion(_motor) which calls getRawADC(_motor) which is a function specific to the hardware target, and must be implemented in the user's MESChw_setup.c file if different to existing targets. Once the raw values are attained, the fastLoop procedes with no hardware specific call.

ADC interrupt is now reserved for overcurrent protection events with the analog watchdog and it is recommended to set the threshold to a range the current sensors can reliably reach; typically 10-50 counts for opamp and perhaps up to 400 counts for inductive sensors. These thresholds can be set in CUBEMX for new hardware or can be written directly to the hadc1.Instance->HTR and LTR.

### Interrupt priority
Interrupt priority is critical and must be taken into account when setting up the system. The order should go:
-1 Reset, NMI
0 Physical fault handlers (busfault etc...) and the hardfault handler. These should be populated with a generateBreakAll() function to immediately disable all motors if they are reached.
1 (optional) Timer1/8 BRK interrupt - This will already have turned off the PWM through hardware link.
1 (optional) ADC watchdog interrupt (should only ever be reached in an out of range event, and should be populated with a handleError(&mtr[n], ERROR_ADC_OUT_OF_RANGE_IA) or generateBreakAll().
2 Timer1/8 Update - This calls the fast and hyperloops and is the main control loop.
3 Slowloop timer
...
5-onwards USB, UART, DMAs, SPI, systick... in any order of your choosing.


### Hall
Hall sensors only commutation is supported, but only in forward mode (swap a pair of motor phase wires if it runs backwards) but full support will be gradually deprecated since sensorless works much better. 
Hall startup option relies on a different mechanism and work in either direction by preloading the observer with flux linkages derived during Tracking.
Timer XOR hall input is not currently used, instead the fastloop just samples and counts PWM cycles since the last hall state change, which is fed into a filter. This reduces accuracy, but improves reliability, noise rejection and portability.

### Encoder
TLE5012B crudely supported in SSC (SPI) mode. The SPI reads are called synchronously from the hyperLoop and therefore PWM frequency is limited to about 20kHz to allow time for the data transfer.
ABI not currently supported. 

### PWM Input
Timer2,3 or 4 can be set up in reset mode, with prescaler sunch that one count = 1us (e.g. for F303 at 72MHz we would use a prescaler of 71. We use a 65535 period, with trigger/reset mapped to TI1FP1, and channel one capturing on rising edge, direct mode, channel 2 capturing on falling edge indirect mode - remapped to TI2FP2. This gives two CC register values, CC1 timing the period of the pulses, and CC2 timing the on time.  
Interrupt: update interrupt used. We expect a value very close to 20000us from an RC sender (50Hz). Check CC1 is 20000+/-~10000. If outside this bounds, set input capture flag low. CC2 is the pulse time.

### Over Current Comparators

#### On F303 based hardware 
Comparators set up to trigger Tim1 break2 state in the event of overcurrent event, which should turn off all outputs to high impedance on F303 hardware. 
0.5mOhm shunts (2x1mohm) or 1mOhm (2x2mOhm original hardware) at 100amps gives 50mV, with a pullup to 100mV. Vrefint is 1.23V, so 1/4Vref used for comparator-ve - 310mV. This triggers the comparator at 400A nominal (a LOT of current, but the intended FETs are rated for that for 100us, which is ~2 PWM periods... If alternate FETs used, should check this, or just hope for the best, or modify the shunt resistors to have higher value.  
Timer1 should have a BRK filter set to avoid switching noise  

#### On most other hardware 
On F4, L4 and other targets, the expectation is that there is a fault input on the PB12 pin for timer1,which is setup to capture and stop PWM generation on this input. Timer8 is mapped to PA6, annoyingly removing an ADC channel.
Hardware without overcurrent comparators is OK, the overcurrent and over voltage is also taken care of in the fastloop, or by the analog watchdog if set up.

## Coms
Primarily, initially, serial used. 
USB CDC (serial) implemented for F303, F401 and F405 targets

### Serial 
TBC.

## Watchdog timer
Not currently implemented. Lack of it has not presented any issue so far. ToDo...
If implementing specifically for your hardware,set it running in the hardwareInit() function. The watchdog timer should be kicked by the fast control loop after the VIcheck is completed to ensure a response to overvoltage and current events is possible.
Period of ~1ms 
On overflow, generate a break state on the motor and reset MCU - control loop no longer running, motor could be stopped, freewheeling, generally making a mess of currents and generating high voltages.

## General workflow

### Fast Control loop
Fast control loop must:  
Retrieve current and Vbus values from ADC 
Carry out the conversion to floating point SI units
Check for over limit events if not handled in hardware
Retrieve phase voltage values if not modulating
Process the state machine (Idle, running, tracking, error... and sensor mode)  
Manipulate currents and voltages to Vab (Clarke transform) and Vdq(Park transform)(FOC)  
Calculate current position and speed (get Hallsensor values, sensorless observer)  
Run FOC PI controller
Run Field weakening and circle limiter
Run SVPWM (include inverse clark/park)
Update inverter PWM values based on phase and voltage  

### Hyper Control loop
Hyper control loop has more optional things, which can include:
Process HFI
Read encoder
Interpolate the angle
IF logging: write currents and voltage to logging buffer, increment/wrap logging buffer index 


### Slow control loop
Execute every 10ms. Originally this ran on the RCPWM input, but not it runs independently. 
The slowloop can be run at any speed from about 20Hz to 1kHz on any timer with interrupt priority lower than the fastloop timer and ADC interrupt (if used)
State machine processing (ToDo)

Update parameters (e.g. volts to PWM, gains...)
Run MTPA
Run power and current limitations

Speed Ramps etc... ToDo


### Coms loop

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