# davidmolony.github.io/MESC_Firmware/

Code base that blew up a MP2 with 150v FETs.

Notes:

* MP2 worked fine at 30v running a flipsky
* Connected MP2 to QS205, with 100v psu, brought up throttle slowly
* Throttle hit near adc1_min, board stopped
* No sound. No parts smoked. 
* Failure blew the pill, the DC-DC 12v
* Performing continuity check from https://github.com/badgineer/MP2-ESC/blob/main/docs/PCB_ASSEMBLY_TESTING.md
* Top and bottom FETs on phase V tested anonymously

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

#define ABS_MAX_PHASE_CURRENT 40.0f
#define ABS_MAX_BUS_VOLTAGE 105.0f
#define ABS_MIN_BUS_VOLTAGE 80.0f
```

