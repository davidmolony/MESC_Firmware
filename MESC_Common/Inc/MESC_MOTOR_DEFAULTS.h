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
//#define QS165V2
//#define PROPDRIVE2830_1000KV
//#define BR3536_1200KV
//#define QS138_90H
//#define //... Define your motor name here...//


//////Motor parameters

#if defined(MCMASTER_70KV_8080) //Reliable params
#define MAX_MOTOR_PHASE_CURRENT 60.0f
#define DEFAULT_MOTOR_POWER 500.0f
#define DEFAULT_FLUX_LINKAGE 0.01180f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000085f //Henries
#define DEFAULT_MOTOR_Lq 0.000150f//Henries
#define DEFAULT_MOTOR_R 0.0530f //Ohms
#define DEFAULT_MOTOR_PP 7 //Pole Pairs

#elif defined(ANT_120_70_62KV) //Reliable params, but need checking
#define MAX_MOTOR_PHASE_CURRENT 200.0f
#define DEFAULT_MOTOR_POWER 250.0f
#define DEFAULT_FLUX_LINKAGE 0.0068f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000020f //Henries
#define DEFAULT_MOTOR_Lq 0.0000250f//Henries
#define DEFAULT_MOTOR_R 0.011f //Ohms
#define DEFAULT_MOTOR_PP 14 //Pole Pairs


#elif defined(CA120) //Reliable params
#define MAX_MOTOR_PHASE_CURRENT 300.0f
#define DEFAULT_MOTOR_POWER 600.0f
#define DEFAULT_FLUX_LINKAGE 0.00380f//wB
#define DEFAULT_MOTOR_Ld 0.0000060f //Henries
#define DEFAULT_MOTOR_Lq 0.0000120f//Henries
#define DEFAULT_MOTOR_R 0.0060f //Ohms
#define DEFAULT_MOTOR_PP 10 //Pole Pairs


#elif defined(ALIEN_50KV_8080) //GUESSED, UPDATE
#define MAX_MOTOR_PHASE_CURRENT 50.0f
#define DEFAULT_MOTOR_POWER 250.0f
#define DEFAULT_FLUX_LINKAGE 0.016f//wB
#define DEFAULT_MOTOR_Ld 0.000110f //Henries
#define DEFAULT_MOTOR_Lq 0.000170f//Henries
#define DEFAULT_MOTOR_R 0.070f //Ohms
#define DEFAULT_MOTOR_PP 7 //Pole Pairs


#elif defined(JENS_SERVO)

#elif defined(SEM_HDM82A8_30S)

#elif defined(QS165)
#define MAX_MOTOR_PHASE_CURRENT 350.0f //350A seems like a reasonable upper limit for these
#define DEFAULT_MOTOR_POWER 5000.0f //Go on, change this to 15000
#define DEFAULT_FLUX_LINKAGE 0.0128f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000087f //Henries
#define DEFAULT_MOTOR_Lq 0.000099f//Henries
#define DEFAULT_MOTOR_R 0.0080f //Ohms
#define DEFAULT_MOTOR_PP 7 //Pole Pairs
//This assumes a 6.5mohm QS165 and 1.5mohm of MOS and cable. With such low resistance, it becomes important

#elif defined(QS165V2)
#define MAX_MOTOR_PHASE_CURRENT 450.0f //450A seems like a reasonable upper limit for these V2
#define DEFAULT_MOTOR_POWER 12000.0f //Go on, change this to 15000
#define DEFAULT_FLUX_LINKAGE 0.019f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000042f //Henries
#define DEFAULT_MOTOR_Lq 0.000065f//Henries
#define DEFAULT_MOTOR_R 0.0060f //Ohms
#define DEFAULT_MOTOR_PP 5 //Pole Pairs
//This assumes a 5mohm QS165V2 and 1.5mohm of MOS and cable. With such low resistance, it becomes important



#elif defined(QS138_90H)
#define MAX_MOTOR_PHASE_CURRENT 700.0f //There seems to be no limit for these
#define DEFAULT_MOTOR_POWER 15000.0f //Go on, change this to 20000...
#define DEFAULT_FLUX_LINKAGE 0.0195f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000030f //Henries
#define DEFAULT_MOTOR_Lq 0.000042f//Henries
#define DEFAULT_MOTOR_R 0.0030f //Ohms
#define DEFAULT_MOTOR_PP 5 //Pole Pairs

#elif defined(VOILAMART1500W)
#define MAX_MOTOR_PHASE_CURRENT 100.0f //
#define DEFAULT_MOTOR_POWER 3000.0f //...
#define DEFAULT_FLUX_LINKAGE 0.02650f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.0001290f //Henries
#define DEFAULT_MOTOR_Lq 0.000280f//Henries
#define DEFAULT_MOTOR_R 0.0510f //Ohms
#define DEFAULT_MOTOR_PP 17 //Pole Pairs - No idea, guess from remembering 38 magnets

#elif defined(PROPDRIVE2830_1000KV) //Can't remember, guessing.
#define MAX_MOTOR_PHASE_CURRENT 20.0f //
#define DEFAULT_MOTOR_POWER 400.0f //Go on, change this to 20000...
#define DEFAULT_FLUX_LINKAGE 0.000098f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.000020f //Henries
#define DEFAULT_MOTOR_Lq 0.000028f//Henries
#define DEFAULT_MOTOR_R 0.062f //Ohms
#define DEFAULT_MOTOR_PP 6 //Pole Pairs

#elif defined(PKP246D23A2) //Can't remember, guessing.
#define MAX_MOTOR_PHASE_CURRENT 2.3f //
#define DEFAULT_MOTOR_POWER 40.0f
#define DEFAULT_FLUX_LINKAGE 0.004f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.00135f //Henries
#define DEFAULT_MOTOR_Lq 0.00135f//Henries
#define DEFAULT_MOTOR_R 0.20f //Ohms
#define DEFAULT_MOTOR_PP 50 //Pole Pairs

#elif defined(G30)
#define MAX_MOTOR_PHASE_CURRENT 50.0f //
#define DEFAULT_MOTOR_POWER 250.0f //...
#define DEFAULT_FLUX_LINKAGE 0.0180f//Set this to the motor linkage in wB
#define DEFAULT_MOTOR_Ld 0.0000850f //Henries
#define DEFAULT_MOTOR_Lq 0.000150f//Henries
#define DEFAULT_MOTOR_R 0.0210f //Ohms
#define DEFAULT_MOTOR_PP 17 //Pole Pairs - No idea, guess

#elif defined(BR3536_1200KV)

//#elif defined() //... Define your motor parameters here...//

#else
#define MAX_MOTOR_PHASE_CURRENT 2.0f //2A seems like a reasonable default for any motor
#define DEFAULT_MOTOR_POWER 50.0f //
#define DEFAULT_FLUX_LINKAGE 0.03f//Could be absolutely anything.
#define DEFAULT_MOTOR_Ld 0.000005f //Henries Could be anything, but setting it low means the current controller P term will be more stable/slower
#define DEFAULT_MOTOR_Lq 0.000005f//Henries
#define DEFAULT_MOTOR_R 0.0080f //Ohms Could be anything, but setting it lower means slower I term on the PID
#define DEFAULT_MOTOR_PP 7 //Pole Pairs
#define MIN_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE
#define MAX_FLUX_LINKAGE DEFAULT_FLUX_LINKAGE
#define FLUX_LINKAGE_GAIN (10.0f * sqrtf(DEFAULT_FLUX_LINKAGE))
#define NON_LINEAR_CENTERING_GAIN 5000.0f

#endif

#endif /* INC_MESC_MOTOR_DEFAULTS_H_ */
