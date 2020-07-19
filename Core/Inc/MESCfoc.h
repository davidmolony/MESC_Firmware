
#include "stm32f3xx_hal.h"


#define FOC_SECTORS_PER_REVOLUTION 	(6)
#define FOC_CONV_CHANNELS           (3)
#define FOC_TRANSFORMED_CHANNELS    (2)
#define FOC_NUM_ADC					(3)

typedef uint16_t foc_angle_t;
typedef int32_t  foc_field_angle_t;
typedef float current_amps_t;
typedef float voltage_t;

typedef struct
{
    foc_angle_t ElecAngle;		//Current electrical angle
    uint16_t    Sector;			//Current electrical sector - 6 sectors, as a consequence of Hall and 3 phase sinwave numbered 0-5 [0,FOC_SECTORS_PER_REVOLUTION)
    foc_angle_t AnglePerSector; //6 sectors per eRevolution ((USHRT_MAX + 1) / FOC_SECTORS_PER_REVOLUTION)
    foc_angle_t RotorAngle;		//Rotor angle, either fetched from hall sensors as (sector*anglePerSector+tim3Count)/6 or from observer
    foc_angle_t AngleStep;		//At startup, step angle is zero, zero speed. This is the angle by which the inverter increments each PWM cycle under open loop

    uint16_t PWM[FOC_CONV_CHANNELS]; 	//3 phase vector for the PWM generation, do math on these before writing them to the timer registers

    current_amps_t  Iab[FOC_TRANSFORMED_CHANNELS];	//Float vector containing the Clark transformed current in amps
    current_amps_t 	Idq[FOC_TRANSFORMED_CHANNELS];	//Float vector containing the Park transformed current in amps
    voltage_t		Vbus;							//Float vector containing the bus voltage in Volts
	voltage_t		Vswitch[FOC_CONV_CHANNELS];		//Float vector containing the switch node voltages (phase voltages)
    foc_field_angle_t FieldAngle;					//Signed number containing the angle between the electrical field and rotor, implemented as rotorAngle-elecAngle

} MESCfoc_s;

void foc_init( MESCfoc_s * foc );


typedef struct
{
    uint16_t RawADC[FOC_NUM_ADC][FOC_CONV_CHANNELS]; 	//ADC1 returns Ucurrent, DClink voltage and U phase voltage
    													//ADC2 returns Vcurrent, V and W phase voltages
														//ADC3 returns Wcurrent,
    uint16_t ADCOffset[FOC_NUM_ADC];	//During detect phase, need to sense the zero current offset

} foc_measurement_t;

foc_measurement_t measurement_buffers;

typedef struct
{
    uint16_t Delta;
    uint16_t Length;
}        MESC_RCPWMin_t;

/* Function prototypes -----------------------------------------------*/

void fastLoop();
void V_I_Check();
void GenerateBreak(); //Software break that does not stop the PWM, sum of phU,V,W_Break();
int GetHallState();
void measureResistance();
void measureInductance();
void phU_Break(); 	//Turn all phase U FETs off, Tristate the ouput - For BLDC mode mainly, but also used for measuring
void phU_Enable();	//Basically un-break phase U, opposite of above...
void phV_Break();
void phV_Enable();
void phW_Break();
void phW_Enable();


