# davidmolony.github.io/MESC_Firmware/

Git commands
THESE ARE WRONG. They break the branch
```
$ git branch -m More_measurement OW_running_on_bike
$ git rev-parse --abbrev-ref HEAD
$ git commit -am "incorporated latest from More_measurement"
$ git push -f origin OW_running_on_bike

```
MESC_Firmware/MESC_F405RG/Core/Inc/MP2_V0_1.h
```
#define MAX_IQ_REQUEST 200.0f
#define ABS_MAX_BUS_VOLTAGE 80.0f

#define QS165 // I think this no longer matters
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
