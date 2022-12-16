/*
 * mxlemming_FOC_GaN.h
 *
 *  Created on: Dec 16, 2022
 *      Author: HPEnvy
 */

#ifndef INC_MXLEMMING_FOC_GAN_H_
#define INC_MXLEMMING_FOC_GAN_H_

#define MCMASTER_70KV_8080

#define SHUNT_POLARITY -1.0f

#define ABS_MAX_PHASE_CURRENT 45.0f
#define ABS_MAX_BUS_VOLTAGE 45.0f
#define ABS_MIN_BUS_VOLTAGE 38.0f
#define R_SHUNT 0.00333f

#define R_VBUS_BOTTOM 3300.0f //Phase and Vbus voltage sensors
#define R_VBUS_TOP 100000.0f

#define OPGAIN 10.0f


#define MAX_ID_REQUEST 2.0f
#define MAX_IQ_REQUEST 30.0f

#define SEVEN_SECTOR		//Normal SVPWM implemented as midpoint clamp. If not defined, you will get 5 sector, bottom clamp
#define DEADTIME_COMP		//This injects extra PWM duty onto the timer which effectively removes the dead time.


//#define USE_FIELD_WEAKENINGV2


//#define USE_LR_OBSERVER

/////////////////////Related to ANGLE ESTIMATION////////////////////////////////////////
#define INTERPOLATE_V7_ANGLE

//#define USE_HFI

//#define USE_ENCODER //Only supports TLE5012B in SSC mode using onewire SPI on SPI3 F405...
#define POLE_PAIRS 10
#define ENCODER_E_OFFSET 25000
#define POLE_ANGLE (65536/POLE_PAIRS)

//#define USE_SALIENT_OBSERVER //If not defined, it assumes that Ld and Lq are equal, which is fine usually.




#endif /* INC_MXLEMMING_FOC_GAN_H_ */
