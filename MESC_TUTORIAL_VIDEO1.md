# MESC Firmware and documentation
## see this [MESC tutorial to flash firmware on to the MP2-ESC](https://www.youtube.com/watch?v=ffT_TOowyAI)

Downloading MESC:
+ The latest build is https://github.com/davidmolony/MESC_Firmware
+ The branch for this video is FLASH_TUTORIAL_2023
+ There are lots of git tools for pulling branches
+ Or you can do it on the command line with:
```
$ git clone --branch FLASH_TUTORIAL_2023 https://github.com/davidmolony/MESC_Firmware.git
```

Advice for loading MESC into STM32CubeIDE:
+ Loading STM32CubeIDE can be really annoying
+ The easiest way is create a whole new workspace
+ Open STM32CubeIDE, when it asks if you want to open a workspace, hit browse
+ Create a new workspace folder, hit open
+ When the IDE starts, use the menu
+ Window->Show view->Project explorer
+ Hopefully that shows window that says "Import projects"
+ Then find "Existing Projects into Workspace"
+ Link that to the top level folder for MESC project you downloaded

If all this gets loaded, these are the three files you have to edit. Note that sometimes these are in a slightly different directory structure in STM32CubeIDE. The files:
+ MESC_Common/Inc/MESC_MOTOR_DEFAULTS.h
+ MESC_F405RG/Core/Inc/MP2_V0_1.h
+ MESC_F405RG/Core/Inc/MESC_F405.h

## Editing MESC_MOTOR_DEFAULTS.h: 
These are variables that need to be created for your motor, these are the variables I'm going to use for a TP128:
```
#elif defined(TP128)
#define MAX_MOTOR_PHASE_CURRENT 350.0f //350A seems like a reasonable upper limit for these
#define DEFAULT_MOTOR_POWER 12000.0f   //Go on, change this to 15000
#define DEFAULT_FLUX_LINKAGE 0.0167f   //Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000032f     //Henries
#define DEFAULT_MOTOR_Lq 0.000046f     //Henries
#define DEFAULT_MOTOR_R 0.0080f        //Ohms
#define DEFAULT_MOTOR_PP 5
```

A document to find motor parameters can be found [here](https://github.com/badgineer/MP2-ESC/blob/main/docs/MOTOR_PARAM.md). A video showing parameter generation is [here](https://www.youtube.com/watch?v=9YggapDcg0M). 

## MESC_F405RG/Core/Inc/MESC_F405.h
Open MESC_F405.h and look at this code block:
```
#include "MP2_V0_1.h"
//#include "CL700_V0_3.h"
//#include "INDI-600.h"
//#include "MX_FOC_IMS.h"
//#include "MX_FOC_GaN.h"
//#include "GIGAVESC.h"
```
Notice that MP2_V0_1.h is uncommented, and out all the other potential header files are comment out. 

THIS WAS NOT MENTIONED IN THE VIDEO. In the same file, change

```
#define DEFAULT_INPUT 0b0001 
```
in order for the UART to be set to your throttle input

## MESC_F405RG/Core/Inc/MP2_V0_1.h

Remember we made a configuration for your motor? One of the first defines in this file requires you to describe your block in MESC_MOTOR_DEFAULTS.h:
```
//Pick a motor for default
#define YOUR_MOTOR_NAME // has to be consistent with MESC_MOTOR_DEFAULTS.h
```

In this case we'll use: 
```
//Pick a motor for default
#define TP128 
```

Next move to this code block in the same file:
```
#define ABS_MAX_PHASE_CURRENT 400.0f 
#define ABS_MAX_BUS_VOLTAGE 80.0f
#define ABS_MIN_BUS_VOLTAGE 45.0f

#define MAX_IQ_REQUEST 200.0f 

//Phase and Vbus voltage sensors
#define R_VBUS_BOTTOM 3300.0f 
#define R_VBUS_TOP 150000.0f

```
Notes:
- ABS_MAX_PHASE_CURRENT: The MP2 will not exceed this phase current
- ABS_MAX_BUS_VOLTAGE: if exceeded by VBat, MP2 throws an error 
- ABS_MIN_BUS_VOLTAGE: if VBat drops below, MP2 throws an error 
- MAX_IQ_REQUEST 200.0f: set to half of ABS_MAX_PHASE_CURRENT
- R_VBUS_BOTTOM and TOP refer to the voltage divider that measures VBat
  - the values shown are 150k, and 3.3k for resistors on the board
- Obviously there are many other values in these files -- **you do not need to change any of them**

Now select the hammer icon which will launch a compile. Alternatively in the Project Explorer you can right mouse click on the MESC_F405RG folder and go to "Build project". 

Once your compile works try connecting the STLINK V2 as showning in the video, and see if you can flash the firmware on to your pill. 
