# davidmolony.github.io/MESC_Firmware/

- This branch will hopefully run on my 300A bike. 
- https://github.com/owhite/MP2-DFN version 0.3 PCB
- Motor: QS165v2

## Settings

**Motors settings**
- set par_ld 0.000042
- set par_lq 0.000065
- set par_r 0.006
- set par_pp 7

**Other values**
- set input_opt 1
- set node_id 11
- set can_adc 3
- set ol_step 20

remember that adc1_min must be below initial adc1
- set adc1_min 1200

MESC_F405.h:
uncomment ```#include "MP2_V0_1.h"```

MP2_V0_1.h
```
#define ABS_MAX_PHASE_CURRENT 400.0f 
#define ABS_MAX_BUS_VOLTAGE 105.0f
#define ABS_MIN_BUS_VOLTAGE 24.0f
```

## CAN
- SPEEDO_TEST is a platformio project used to test CAN
- CAN appeared to work out of the box

## Label on the pill 

`MESC_Firmware/patch_V2_JUNE7_2026.bin

[Download patch_V2_JUNE7_2026.bin](./patch_V2_JUNE7_2026.bin)

