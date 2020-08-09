/*
File full of notes, code ideas...
Not really for compiling...
*/




//Motor states are:
typedef enum
{
IDLE=0; 
	//All PWM should be off state, nothing happening. Motor may be spinning freely
DETECTING=1;
	//PWM not generating output, but still running to trigger the ADC/check for hall sensors. 
	//Returned values from ADC used to detect if the motor is spinning, how fast, what speed...
ALIGN=2;
	//Hold one phase at current
MEASURING=3;
	//Measuring resistance and inducance of phase
OPENLOOPSTARTUP=4;
	//Starting up in sensorless mode
OPENLOOPTRANSITION=5;
	//Checking motor is running synchronously and phaselocking
HALLNEARSTATIONARY=6;
	/*Hall sensors detected but the hall timer is overflowing because motor is too slow. Commutation based on number of steps advanced/lagging
		Positive throttle implies step will always give positive torque - efield 60 or 120 degrees ahead of hall sensors
		Negative throttle implies braking - efield always aligned 1 step behind direction of spin
			Direction of spin not needed to be known - just 
				if((efieldstep-hallstep)>1){efieldstep-1;}//need to account for overflow of steps1-6
				if((efieldstep-hallstep)<-1){efieldstep+1;}//need to account for overflow of steps1-6
		Implement PI loop+feed forward based on PP, kV, Resistance
	*/
HALLRUN=7;
	/*Hall sensors are changing state fast enough for the timer to detect them. From this, a continuous sinusoidal FOC algorithm can be running
		Always align the current 90degrees to the field(hall sensor)
		Magnitude of current proportional to throttle demand (+/- can either be +/- current, or invert 90degree angle, depending on inverter algorithm
	*/
SENSORLESSRUN=8;
	/*
	*/
ERROR=10;
	/*Enter this state when the overcurrent or overvoltage trips, or illegal hall state or sensorless observer fault occurs
	All PWM signals should be disabled, the timer may be in fault mode with all outputs disabled, or it may be required to implement the bit writes to turn off the outputs
	
	*/
RECOVERING=11;
		/*
		After a fault state, might want to implement a routine to restart the system on the fly - detect if motor is running, detect speed, phase, re-enable PWM
		*/

} MotorState_e;		







typedef enum
{
	OPENLOOP=0;
	HALLSENSORS=1;
	SENSORLESS=2;
} OpMode_e;






//Global variables required for operation
typedef struct
{
uint16_t IMid[3]={0,0,0};	//The ADC offset for each channel due to resistor mismatch at zero current - Set at startup when all switches off.
	
uint16_t elecAngle;			//Current electrical angle 
int sector;					//Current electrical sector - 6 sectors, as a consequence of Hall and 3 phase sinwave numbered 0-5
uint16_t anglePerSector=65536/6; //6 sectors per eRevolution
uint16_t rotorAngle;		//Rotor angle, either fetched from hall sensors as (sector*anglePerSector+tim3Count)/6 or from observer
uint16_t angleStep=0; 		//at startup, step angle is zero, zero speed. This is the angle by which the inverter increments each PWM cycle under open loop

uint16_t PWM[3]; //3 phase vector for the PWM generation, do math on these before writing them to the timer registers
} basicVars_st;

//FoC variables
typedef struct
{
int Iab[2]; 				//signed vector containing the clark transformed current
int fieldAngle=0;			//Signed number containing the angle between the electrical field and rotor, implemented as rotorAngle-elecAngle
int Idq[2];					//signed vector containing the park trasnformed current
} FoCVars_st;



//Motor variables
typedef struct
{
	//Should these be floats? Float ops are fast on CortexM4

uint16_t Rphase=0;			//unsigned int containing phase resistance in mOhms, populated by DETECTING if not already known;
uint16_t Lphase=0;			//unsigned int containing phase inductance in uH, range from veryvery low inductance high kV strong magnet BLDC motors to low kV weak magnet ones;
uint16_t RawCurrLim=4096;  	//Current limit that will trigger a software generated break from ADC. Actual current equal to (RawCurrLim-IMid)*3.3/4096/Gain/Rshunt 	//example (4096-2048)*3.3/(4096*16*0.001)= 103A
uint16_t RawVoltLim=2300;	//Voltage limit that will trigger a software generated break from ADC. Actual voltage equal to RawVoltLim*3.3*Divider/4096			// example 2303*3.3/4096*(R1k5+R47k/R1K5)=60V
} MotorVars_st;

Hardware Variables
typedef struct
{
	//Should these be floats? Float ops are fast on CortexM4 (single cycle) might be best to typecast one time at ADC conversion and once more at PWM generation
uint16_t Rshunt=1000		//microohms Rshunt - will never use a shunt over 65mOhms 
uint16_t RVBT=47000;		//unsigned int containing Vbus top divider;
uint16_t RVBB=1500;			//unsigned int containing Vbus top divider;
uint16_t RIphPU=4700;		//unsigned int containing phase current pullup
uint16_t RIphSR=150;		//unsigned int containing phase current series resistance
uint16_t OpGain=16;			//OpAmp gain, if external, or internal PGA
uint16_t Igain=Rshunt*OpGain*RIphPU/(RIphSR+RIphPU);	//Resistor gain network*opamp gain - total gain bfore the current hits the ADC, might want this inverted to avoid using division?
} HWVars_st;

//256 element array with a Sinwave table for performing loopups
uint8_t Sin[]={128,131,134,137,141,144,147,150,153,156,159,162,165,168,171,174,177,180,183,186,188,191,194,196,199,202,204,207,209,212,214,216,219,221,223,225,227,229,231,233,234,236,238,239,241,242,244,245,246,247,249,250,250,251,252,253,254,254,255,255,255,256,256,256,256,256,256,256,255,255,255,254,254,253,252,251,250,250,249,247,246,245,244,242,241,239,238,236,234,233,231,229,227,225,223,221,219,216,214,212,209,207,204,202,199,196,194,191,188,186,183,180,177,174,171,168,165,162,159,156,153,150,147,144,141,137,134,131,128,125,122,119,115,112,109,106,103,100,97,94,91,88,85,82,79,76,73,70,68,65,62,60,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,22,20,18,17,15,14,12,11,10,9,7,6,6,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,6,6,7,9,10,11,12,14,15,17,18,20,22,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,60,62,65,68,70,73,76,79,82,85,88,91,94,97,100,103,106,109,112,115,119,122,125};


//Variables filling up by ADC DMAs

typedef struct
{
uint16_t RawADC1[3]; //ADC1 returns Ucurrent, DClink voltage and U phase voltage 
uint16_t RawADC2[3]; //ADC2 returns Vcurrent, V and W phase voltages
uint16_t RawADC3[3]; //ADC3 returns Wcurrent,

uint16_t ADCoffset[3]; //During detect phase, need to sense the zero current offset
uint16_t RCPWMin[2]; //Time between pulses and length of pulses - hand to DMA

} Measurement_Vars_st;		

int GetHallState(){
	int hallState=0;
	hallState=((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))|((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))<<1)|((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))<<2));
	switch(hallState)
		{
			case 0:
				return 7; //7 is the no hall sensor detected state (all low)
				break;
			case 7:
				return 6; //6 is the no hall sensor detected state (all high)
				break;
			case :
				return 0;
				break;
			case :
				return 1;
				break;
			case :
				return 2;
				break;
			case :
				return 3;
				break;
			case :
				return 4;
				break;
			case :
				return 5;
				break;
			default:
				return 8;
				break;
		}
}




void fastLoop(){//Call this directly from the ADC callback IRQ
V_I_Check(); //Run the current and voltage checks
switch(MotorState)
	{
		 case SENSORLESSRUN:
			//Call the observer
			//Call the current and phase controller
			//Write the PWM values
			break;
		 case HALLRUN:
			//Get the current position from HallTimer
			//Call the current and phase controller
			//Write the PWM values
			break;
		 case HALLNEARSTATIONARY:
			//Call GetHallState
			//Call the BLDC discrete controller - Override the normal current controller, this is 6 step DC only
			//Write the PWM values
			break;
		 case OPENLOOPSTARTUP:
			//Same as open loop
			//Write the PWM values
			break;
		 case OPENLOOPTRANSITION:
			//Run open loop
			//Run observer
			//RunFOC
			//Weighted average of the outputs N PWM cycles
			//Write the PWM values
			break;
		 case OPENLOOP: //Is this necessary? Just run openloopstartup with no end point?
			//Increment the angle
			//Call the BLDC current controller
			//Write the PWM values
			break;
		 case IDLE:
			  // Do basically nothing
			  //ToDo Set PWM to no output state
		  break;
		 
		 case DETECTING:
			int test=8;
			test=GetHallState();
			if(test==6)|(test==7)){
				//no hall sensors detected
				OpMode=SENSORLESS;
			}
			else if(test==8){
				MotorState=ERROR;
			}
			//ToDo add resporting
			else {
				//hall sensors detected
				OpMode=HALLSENSORS;			
			}
		 break;
		 case MEASURING:
			if(Rphase==0){//Every PWM cycle we enter this function until the resistance measurement has converged at a good value. Once the measurement is complete, Rphase is set, and this is no longer called
				measureResistance();
			break;
			}
			else if(Lphase==0){
				//As per resistance measurement, this will be called until an inductance measurement is converged.
				//Inductance measurement might require a serious reset of the ADC, or calling this functionmany times per PWM period by resetting the OCR4 register to trigger the ADC successively
				measureInductance();
			break;
			}
		 case ERROR:
			GenerateBreak();	//Generate a break state
			//Now panic and freak out
			break;
	}
}

void V_I_Check(&RawADC1,&RawADC2, &RawADC3){ //Is this the correct use of &pointers? Just need it to look in the buffers filled by the DMA
	//Check currents, voltages are within panic limits
	if((RawADC1[0]>RawCurrLim)|(RawADC2[0]>RawCurrLim)|(RawADC3[0]>RawCurrLim)|(RawADC1[1]>RawVoltLim)){
		GenerateBreak();
		MotorState=ERROR;
	}
}

void GenerateBreak(){
	//Here we set all the PWMoutputs to LOW, without triggering the timerBRK, which should only be set by the hardware comparators, in the case of a shoot-through orother catastrophic event
	//This function means that the timer can be left running, ADCs sampling etc which enables a recovery, or single PWM period break in which the backEMF can be measured directly
	//This function needs implementing and testing before any high current or voltage is applied, otherwise... DeadFETs
}

typedef enum
{
     DIR_CLOCKWISE = 0;
     DIR_COUNTERCLOCK = 1;
} SpinDirection_t;