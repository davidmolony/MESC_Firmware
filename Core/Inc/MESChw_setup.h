#include "stm32f3xx_hal.h"

#define HW_SETUP_RSHUNT (1000)
//:
//#define HW_SETUP_IGAIN ((HW_SETUP_RSHUNT*...)/(...))
// _OR
typedef float hardware_vars_t; //Let's have all the hardware and everything in float for now, until we start running out of clock cycles?

typedef struct
{
	hardware_vars_t Rshunt; //Shunt resistance, ohms
	hardware_vars_t RVBT;	//Vbus top divider - Also for switch divider
	hardware_vars_t RVBB;	//Vbus bottom divider - Also for switch divider
	hardware_vars_t VBGain; //=RVBB/(RVBB+RVBT); 		//Resistor divider network gain (fractional)
	hardware_vars_t RIphPU;	//phase current pullup
	hardware_vars_t RIphSR;	//phase current series resistance
	hardware_vars_t OpGain;	//OpAmp gain, if external, or internal PGA
	hardware_vars_t Igain;	//=Rshunt*OpGain*RIphPU/(RIphSR+RIphPU);	//Resistor gain network*opamp gain - total gain before the current hits the ADC, might want this inverted to avoid using division?
} hw_setup_s;


typedef struct
{
	hardware_vars_t Rphase;		//unsigned int containing phase resistance in mOhms, populated by DETECTING if not already known;
	uint8_t uncertainty;		//uncertainty should start at 255 an as the measure resistance is called each PWM cycle, be deprecated by accumulating the measurements until it reaches 0, at which point the resistance is accepted
	hardware_vars_t Lphase;		//unsigned int containing phase inductance in uH, range from very very low inductance high kV strong magnet BLDC motors to low kV weak magnet ones;
    uint16_t      RawCurrLim;	//Current limit that will trigger a software generated break from ADC. Actual current equal to (RawCurrLim-IMid)*3.3/4096/Gain/Rshunt 	//example (4096-2048)*3.3/(4096*16*0.001)= 103A
    uint16_t      RawVoltLim;	////Voltage limit that will trigger a software generated break from ADC. Actual voltage equal to RawVoltLim*3.3*Divider/4096			// example 2303*3.3/4096*(R1k5+R47k/R1K5)=60V
} motor_s;

motor_s motor;

//void motor_init( struct motor_s *motor);	//Rob created prototype init, unused for now



hw_setup_s g_hw_setup;

// _OR_
// void hw_setup_init( hw_setp_s * hw_setup );
//

/* Function prototypes -----------------------------------------------*/

