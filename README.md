# MESC_Firmware
Custom FOC, BLDC, speed control firmware for use with the MESC_FOC_ESC hardware project

## Foreword:
This project is new as of 28/06/2020, and is the work of David Molony, experienced mechanical engineer, moderately experienced electrical engineer, and software newbie. If/when this firmware becomes useable enough to be useful to the world, this foreword will be edited to acknowledge.  
For now, the hardware is useable using STMicro's Motor Control Workbench and CUBEMX/CUBEIDE/Truestudio installations, for which one day I shall place a binary in the hardware folder...

## Code style
This section formalises naming rules for variables, constants, function names, etc. It also All contributors are required to adhere to the code style rules. Certain functionality of IDE can be enabled to help with automating the checks.

### Code structure 
Eclipse IDE can be setup to achieve this code structure automatically. Open dialog
Window --> Preferences. In the preferences window make following changes.  
1. C/C++ --> Code Style --> Formatter. Select K&R[built-in] profile.
2. C/C++ --> Editor --> Save Actions. Tick Format source code and Format edited lines. Tick Ensure newline at the end of the file.

Once it is all setup the code format will be enforced for all edited lines upon save. If you want to reformat the code press Ctrl-Shift-F.  
Below is the example of K&R formatted code.

	/*
	 * A sample source file for the code formatter preview
	 */
	#include <math.h>
	class Point {
	public:
		Point(double x, double y) :
				x(x), y(y) {
		}
		double distance(const Point &other) const;
	
		double x;
		double y;
	};

	double Point::distance(const Point &other) const {
		double dx = x - other.x;
		double dy = y - other.y;
		return sqrt(dx * dx + dy * dy);
	}

### Code analysis.
It is highly advisable to enable code style and structure monitoring provided by the IDE. In Preferences window to go C/C++ --> Code analysis. Enable all.  
Some rules might seem to be overly restrictive and those can be disabled. For example, Coding style --> Line comments will complain about "//" single line style comments. Given extensive use of those by David it's probably not worth enforcing that rule.

### Precompiler defined values
These are constant values given a name through pre-compiler directive

	#define PI 3.14

### Variables

### Function names

### Constant values


## Useful Eclipse IDE extensions
To modify visual impact of coding work and reduce eye strain it is advisable to use darker colour schemes. We find that Darkest Dark Eclipse extension works very well and has good colour choices. In order to install it go to Help --> Eclipse Marketplace... In the search bar type "Darkest Dark". It should be first in the search results. Install it. After restart of IDE it'll present a set of choices. Leave default ones. They can later be modified in the Preferences window by going to DevStyle --> Color Themes.

## Licence
This project will initially (and perhaps perpetually) contain a lot of firmware licenced under the STM Cube licence, BSD 3 clause https://opensource.org/licenses/BSD-3-Clause .   
The rest of the custom code is intended to be contained primarily in the MESC files, and will have a licence assigned at a later date, probably also BSD 3 Clause.

## Hardware:
Any STM32F303CB based 3 phase system using Timer1 for PWM High and Low and 3 shunt ADC measurement with internal op amps 1,2,3 and internal comparators 1,2,4
Specifically intended for the MESC_FOC_ESC hardware.  
Can probably be easily ported to other MCUs (STM32F3, F4, L4 maybe even F0, L0... but external opamp hardware will be needed, and checks that ADCs sample within required time).

## General methodology

Threads run for comms, ramp generation on FreeRTOS  
Main fast loop runs on ADCconversion complete interrupt, which occurs <1us after timer underflow.    
3 phase Sin wave calculated every PWM period, and correction for phase, FOC, ZCdetect, hall state...  
Comparators set up for overcurrent events, 

### PWM
Timer1 set up to generate complimentary centre aligned PWM with dead time, frequency 72MHz/1024 (10bit voltage)/(prescaler) = 17.6kHz (prescaler 2) or and ~35.2kHz (prescaler 1).  
Owing to the number of clock cycles per PWM period for the MCU to complete math (e.g. 72M/35.2K= 2048 clock cycles) math occurring in the interupt must be fast, and best not to support frequency above 35.2kHz.
Firmware will be primarily tested at 35.2kHz, the harder case for fitting in the control loop. Anticipated that 17kHz might be preferable for larger motors with lower eRPMs due to better switching efficiency

PWM will be centred at 512, 50% duty cycle, for 0V for sinusoidal operation. For BLDC mode, if ever made, one phase will obviously be constantly grounded at any time.  
Centring at 50% duty cycle has a few  
Advantages - it allows recirculation through the high side FETs as well as the low side, which evens out the load on them, and removes the math to zero the signals. Disadvantages - reduced sampling time for currents , more switching events, all phases floating... might be worse for EMC emmisions?

Target 60000mechrpm for a 6PP motor (say really aggressive, small Electronic jet engine RC model) => 1000Hz mechanical => 6000Hz electrical(360000erpm)=> ~3 PWM periods/sin wave at 17.5kHz,  ~6PWM periods at 35kHz. Dubious whether this is feasible or sensible.  
Target 30000mechrpm gives a more feasible 180000erpm, with 12PWMperiods/sinwave @35kHz.

### ADC
ADC conversions are triggered by timer1 underflow on TRGO. ADCs 1,2,3 are used to get fully synchronous current readings. Vbus read by ADC1 immediately after current reading.   The main loop runs on the ADC conversion complete interrupt, in which the new PWM values are calculated from an 8 bit  Sin table, and FOC/sensorless observer/sensorless BLDC... will be implemented  

### Hall
Timer4 set up in XOR activation reset mode, so gives a duration between each hall sensor change of state, which can be directly converted to a speed. For a minimum speed of 10eRPM, =(10/60)Hz=1 XOR changes/second, 1 seconds/XOR change, requires 16 bit TIM4 to clock at 65536/1=65.536kHz. This implies a max speed of 65536(Hz)/6(hall states)x60(RPM/Hz)=655360RPM electrical. With typical BLDC motors being 6PP, this enables 100000RPM mechanical measureable, but the PWM frequency is not high enough to support this!

### PWM Input
Timer3 set up in reset mode, prescaler 72 (1us resolution), 65535 period, with trigger/reset mapped to TI1FP1, and channel one capturing on rising edge, direct mode, channel 2 capturing on falling edge indirect mode - remapped to TI2FP2. This gives two CC register values, CC1 timing the period of the pulses, and CC2 timing the on time.  
Interrupt: Only interested in CC2 for the value, ut CC1, period, can be used to determine whether the RC is still transmitting - expect a value very close to 20000 from an RC sender. Check CC1 is 20000+/-~5000, and check the interrupt was not triggered by timer overflow - if timer overflowed, then period between pulses much more than 20000, RC sender probably broken/unplugged. Set action flag.


### Over Current Comparators
Comparators set up to trigger Tim1 break2 state in the event of overcurrent event, which should turn off all outputs to high impedance  
0.5mOhm shunts (2x1mohm) at 100amps gives 50mV. Vrefint is 1.23V, so 1/4Vref used for comparator-ve - 310mV. This triggers the comparator at 600A nominal (a LOT of current, but the intended FETs are rated for that for 100us, which is ~2 PWM periods... https://www.st.com/resource/en/datasheet/sth310n10f7-2.pdf). If alternate FETs used, should check this, or just hope for the best, or modify the shunt resistors to have higher value.  
Timer1 should have a BRK filter set to avoid switching noise  
Soft current control e.g. constantly running at 100+amps must be taken care of by the fast control loop.

## Coms
Primarily, initially, serial used. Potentially USB CDC later, and I2C
### Serial 
Serial USART3 to be used in DMA recieve mode to eliminate interrupts and MCU blocking behaviour.  
Serial DMA to interrupt on overflow of buffer - Hopefully never reaches overflow of buffer.  
  If overflow occurs, just throw away the whole buffer, reset DMA to load first position and await further data

## Watchdog timer
Watchdog timer should be kicked by the fast control loop.
Period of ~1ms
On overflow, generate a break state on the motor and reset MCU - control loop no longer running, motor could be stopped, freewheeling, generally making a mess of currents and generating high voltages.

## Fast Control loop
Fast control loop must:  
Retrieve current values from ADC conversion  
Retrieve voltage values  
  IF logging: write currents and voltage to logging buffer , increment logging pointer
  IF logging pointer >logging samples, set logging complete
Check state (Idle, revup, running,  fault/BRK,  
Manipulate currents and voltages to single phase (FOC?)  
Calculate current position and speed (get Hallsensor values and tim4 count, or for sensorless... observer? Voltage-current phase calculation?  
Based on current position and speed,  
  IF speed control: Change voltage request  to catch-up/slow down the motor  
    IF max current reached, revert to torque control and update correct next position to avoid losing synch  
  IF current control: Update next position to correct next position and voltage to moderate current to requested value if torque control  
Update inverter PWM values based on phase and voltage  

## Slow control loop
Execute every ~100ms
State machine processing  
Ramps  

## Coms loop
Every (~1second?) fire the Comms loop
Start at current character of coms loop in DMA buffer
Parse for command
  IF end recieved (e.g.\r) execute command
  Else, wait another coms loop period, hopefully \r command will be there next time
If DMA buffer counter<=coms loop counter, reset DMA start position and coms loop pointer - Coms loop is up to date with DMA
NOTE: Currently I have very litle idea what I am doing regarding parsing serial, comms management, hopefully this will not be a monumental mess.

## Speeds, angles, input params...
### Motor params demanded will be:  
PP - Pole pairs
mechKV - Approx mechkV
Rph - Approx phase resistance (=1/2 phase:phase resistance)
Lph - Approx inductance
MaxMotorCurrent - Max motor current
MaxMechRPM - Maximum motor RPM before explosion, user setpoint etc
OLRampCurrent - Open loop ramp current 
OLmechRPM - open loop mechanical RPM (will ramp up to this speed in open loop, and stay there until sync'd

### Inverter params will be:
The whole firmware will work on the basis of electrical Hz - inputs should be immediately converted into eHz and eAngle  
  eHz=mechRPMx60xPP  
  Angles and steps will be determined as a uint16_t. The whole firmware will work on the basis of eHz for speed and eAngle  

eHzR - requested electrical speed  
eHzP - Present electrical speed(back calculated from eStepP)  
eStepR - requested (derived) electrical step = 65536xeHzR/fPWM  
eStepP - eStepR +/- the correction values to keep sync. Generate a present electrical step angle per PWM cycle eStep = eHzx65536/fPWM - each PWM period, the inverter PWM will shift by this much.  
eAngleP - present electrical angle, The electrical angle will integrate each PWM period by eStepPInv and be handed to the inverter. When the uint16_t overflows, it will automatically loop. Inverter will take this uint16_t angle, right shift it 8 times to generate an 8 bit angle and look up the relevant sin value  

VDemandInv - uint16_t Inverter voltage request. Inverter will take this value, multiple it by the 8 bit sintable value (centred on 128) and right shift by (8+(16-10)) to scale it to fit the PWM duty.  
### Controller params will be
ToDo, calculate OL voltage ramps etc. 
ToDo, add parameters for speed steps
