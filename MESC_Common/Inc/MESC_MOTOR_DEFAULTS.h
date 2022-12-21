/*
 * MESC_MOTOR_DEFAULTS.h
 *
 *  Created on: Dec 16, 2022
 *      Author: David Molony
 */

#ifndef INC_MESC_MOTOR_DEFAULTS_H_
#define INC_MESC_MOTOR_DEFAULTS_H_

#include "stm32fxxx_hal.h"

//#define MCMASTER_70KV_8080
//#define ANT_120_70_62KV
//#define CA120
//#define ALIEN_50KV_8080
//#define JENS_SERVO
//#define SEM_HDM82A8_30S
//#define QS165
//#define PROPDRIVE2830_1000KV
//#define BR3536_1200KV
//#define QS138_90H
//#define //... Define your motor name here...//


//////Motor parameters

#if defined(MCMASTER_70KV_8080) //Reliable params
#define MAX_MOTOR_PHASE_CURRENT 60.0f
#define DEFAULT_MOTOR_POWER 500.0f
#define DEFAULT_FLUX_LINKAGE 0.01180f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000080f //Henries
#define DEFAULT_MOTOR_Lq 0.000130f//Henries
#define DEFAULT_MOTOR_R 0.0530f //Ohms

#elif defined(ANT_120_70_62KV) //Reliable params, but need checking
#define MAX_MOTOR_PHASE_CURRENT 200.0f
#define DEFAULT_MOTOR_POWER 250.0f
#define DEFAULT_FLUX_LINKAGE 0.0068f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000020f //Henries
#define DEFAULT_MOTOR_Lq 0.0000250f//Henries
#define DEFAULT_MOTOR_R 0.011f //Ohms

#elif defined(CA120) //Reliable params
#define MAX_MOTOR_PHASE_CURRENT 300.0f
#define DEFAULT_MOTOR_POWER 600.0f
#define DEFAULT_FLUX_LINKAGE 0.00380f//wB
#define DEFAULT_MOTOR_Ld 0.0000060f //Henries
#define DEFAULT_MOTOR_Lq 0.0000120f//Henries
#define DEFAULT_MOTOR_R 0.0060f //Ohms

#elif defined(ALIEN_50KV_8080) //GUESSED, UPDATE
#define MAX_MOTOR_PHASE_CURRENT 50.0f
#define DEFAULT_MOTOR_POWER 250.0f
#define DEFAULT_FLUX_LINKAGE 0.016f//wB
#define DEFAULT_MOTOR_Ld 0.000110f //Henries
#define DEFAULT_MOTOR_Lq 0.000170f//Henries
#define DEFAULT_MOTOR_R 0.070f //Ohms

#elif defined(JENS_SERVO)

#elif defined(SEM_HDM82A8_30S)

#elif defined(QS165)
#define MAX_MOTOR_PHASE_CURRENT 350.0f //350A seems like a reasonable upper limit for these
#define DEFAULT_MOTOR_POWER 5000.0f //Go on, change this to 15000
#define DEFAULT_FLUX_LINKAGE 0.0128f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000087f //Henries
#define DEFAULT_MOTOR_Lq 0.000099f//Henries
#define DEFAULT_MOTOR_R 0.0080f //Ohms
//This assumes a 6.5mohm QS165 and 1.5mohm of MOS and cable. With such low resistance, it becomes important

#elif defined(QS138)
#define MAX_MOTOR_PHASE_CURRENT 400.0f //There seems to be no limit for these
#define DEFAULT_MOTOR_POWER 4000.0f //Go on, change this to 20000...
#define DEFAULT_FLUX_LINKAGE 0.01180f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000080f //Henries
#define DEFAULT_MOTOR_Lq 0.000130f//Henries
#define DEFAULT_MOTOR_R 0.0530f //Ohms

#elif defined(PROPDRIVE2830_1000KV) //Can't remember, guessing.
#define MAX_MOTOR_PHASE_CURRENT 20.0f //
#define DEFAULT_MOTOR_POWER 400.0f //Go on, change this to 20000...
#define DEFAULT_FLUX_LINKAGE 0.000092f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000020f //Henries
#define DEFAULT_MOTOR_Lq 0.000028f//Henries
#define DEFAULT_MOTOR_R 0.030f //Ohms

#elif defined(BR3536_1200KV)

//#elif defined() //... Define your motor parameters here...//

#else
#define MAX_MOTOR_PHASE_CURRENT 2.0f //2A seems like a reasonable default for any motor
#define DEFAULT_MOTOR_POWER 50.0f //
#define DEFAULT_FLUX_LINKAGE 0.03f//Could be absolutely anything.
#define DEFAULT_MOTOR_Ld 0.000005f //Henries Could be anything, but setting it low means the current controller P term will be more stable/slower
#define DEFAULT_MOTOR_Lq 0.000005f//Henries
#define DEFAULT_MOTOR_R 0.0080f //Ohms Could be anything, but setting it lower means slower I term on the PID
#endif

#endif /* INC_MESC_MOTOR_DEFAULTS_H_ */
