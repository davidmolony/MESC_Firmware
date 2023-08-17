# davidmolony.github.io/MESC_Firmware/

Code base that blew up a MP2 with 150v FETs.

Notes:
** MP2 worked fine at 30v running a flipsky
** Connected MP2 to QS205, with 100v psu, brought up throttle slowly - board stopped
** Failure blew the pill, the DC-DC 12v
** Performing continuity check from https://github.com/badgineer/MP2-ESC/blob/main/docs/PCB_ASSEMBLY_TESTING.md
** Top and bottom FETs on phase V tested anonymously

MESC_MOTOR_DEFAULTS.h
```
#elif defined(QS205)
#d#define MAX_MOTOR_PHASE_CURRENT 350.0f
#define DEFAULT_MOTOR_POWER 5000.0f
#define DEFAULT_FLUX_LINKAGE 0.0514f //motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.0000475f  //Henries
#define DEFAULT_MOTOR_Lq 0.0000615f  //Henries
#define DEFAULT_MOTOR_R 0.01445f     //Ohms
#define DEFAULT_MOTOR_PP 16          //Pole Pairs
```
MP2_V0_1.h
```
#define QS205
#define PWM_FREQUENCY 20000
#define CUSTOM_DEADTIME 800 //ns, MAX 1500ns! implementation in MESCInit().

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 40.0f /
#define ABS_MAX_BUS_VOLTAGE 105.0f
#define ABS_MIN_BUS_VOLTAGE 80.0f
```

A safer thing to do is just run switch:
```
$ git clone https://github.com/davidmolony/MESC_Firmware.git
$ cd MESC_Firmware
$ git switch OW_running_on_bike
$ cp README.md ../.
$ git checkout FW_ADC_sampling
$ git merge -X ours OW_running_on_bike
$ git rev-parse --abbrev-ref HEAD
$ git switch OW_running_on_bike
$ cp ../README.md .
$ git add . 
$ git rev-parse --abbrev-ref HEAD

$ git switch OW_running_on_bike
```
...when possible. 

NOTE: to set hall, HFI or PWM encoder, in Jen's term there's a variable in MESC_RTOS/MESC/MESCinterface.c:	
```
TERM_addVar(mtr[0].SLStartupSensor	, 0			, 30		, "SL_sensor"	, "0=OL, 1=Hall, 2=PWMENC, 3=HFI"		, VAR_ACCESS_RW	, NULL		, &TERM_varList);
```

General notes for standing up a new F405 pill / MP2 board
* add 100nF to throttle input
* clip the NC pin and fill that header hole with epoxy to prevent placing pill in wrong orientation
* using IPP075N15N3G, David is concerned about 5nF input capacitance, consider changing the gate resistor to 10 to 15 ohm
* on the topic of making the throttle safer: "So we swap in 1k-1.2k and set it up so that anything above 3.2V is it if range, error."

MESC_Firmware/MESC_F405RG/Core/Inc/MP2_V0_1.h
note: ABS_MAX_PHASE_CURRENT overrules everything. MAX_IQ_REQUEST can be overwritten by jens
So you can step up amps using term
```
#define ABS_MAX_PHASE_CURRENT 400.0f 
#define MAX_IQ_REQUEST 200.0f 
#define ABS_MAX_BUS_VOLTAGE 80.0f

#define QS165 
#define MIN_IQ_REQUEST 0

#define KILLSWITCH_GPIO GPIOB
#define KILLSWITCH_PIN GPIO_PIN_3
```
MESC_Firmware/MESC_F405RG/Core/Inc/MESC_F405.h
```
#include "MP2_V0_1.h"
```

MESC_F405RG/MESC_F405RG.ioc
* add UART1
* PB3 to GPIO_input

MESC_F405RG/Core/Inc/MESC_F405.h:
```
define HW_UART huart1
```

MESC_Common/Inc/MESCfoc.h
```
 int16_t ADC_in_ext1;
 int16_t ADC_in_ext2;
```

MESC_Common/Src/MESCfoc.c
```
        // _motor->offset.Iu = ADC_OFFSET_DEFAULT;
	// _motor->offset.Iv = ADC_OFFSET_DEFAULT;
	// _motor->offset.Iw = ADC_OFFSET_DEFAULT;
	
	_motor->offset.Iu = 1978;
	_motor->offset.Iv = 1940;
	_motor->offset.Iw = 1949;
```

Temperature measurement is broken -- comment out this line
there are TWO occurences in MESCfoc.c
```
 // ThrottleTemperature(_motor);
```

MESC_RTOS/Tasks/MESCinterface.c:

```
	TermVariableDescriptor * desc;
        desc = TERM_addVar(mtr[0].Conv.Vbus                             , 0.0f          , HUGE_VAL  , "vbus"            , "Read input voltage"                                  , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].FOC.eHz                               , -HUGE_VAL , HUGE_VAL  , "ehz"                 , "Motor electrical hz"                                 , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.d    , -HUGE_VAL , HUGE_VAL  , "idq_d"                       , "Phase Idq_d smoothed"                                        , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].FOC.Idq_smoothed.q    , -HUGE_VAL , HUGE_VAL  , "idq_q"                       , "Phase Idq_q smoothed"                                        , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].Raw.ADC_in_ext1               , 0 , 4096                      , "adc_ext1"            , "Raw ADC throttle"                                    , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].Conv.MOSu_T                   , 0 , 4096                      , "MOS_temp"            , "MOSFET temp, kelvin"                                 , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(mtr[0].Conv.Motor_T                  , 0 , 4096                      , "MOT_temp"            , "Motor temp, kelvin"                                  , VAR_ACCESS_TR  , NULL         , &TERM_varList);
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);

        desc = TERM_addVar(MESC_errors                   , -HUGE_VAL , HUGE_VAL  , "error" , "System errors"       , VAR_ACCESS_TR  , NULL, &TERM_varList);
	
        TERM_setFlag(desc, FLAG_TELEMETRY_ON);
```

This is NOT in kill switch mode:
make sure input_opts is 9 to avoid reading the kill switch
