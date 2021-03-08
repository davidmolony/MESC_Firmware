/*
 **
 ******************************************************************************
 * @file           : MESCfoc.h
 * @brief          : FOC running code and ADC buffers
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************

 * MESCfoc.h
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

#include "stm32f3xx_hal.h"

#define FOC_SECTORS_PER_REVOLUTION (6)
#define FOC_CONV_CHANNELS          (3)
#define FOC_TRANSFORMED_CHANNELS   (2)
#define FOC_NUM_ADC                (3)
#define FOC_PERIODS                (1)

// fixme: I think this type of stuff is causing confusion later on especially in
// the code that strives for maintainability. What does an alias give you?
typedef uint16_t foc_angle_t;
typedef float current_amps_t;
typedef float voltage_t;

// fixme: why is this in a struct?
typedef struct
{
    int initing;  // Flag to say we are initialising
    uint16_t FOCError;
    foc_angle_t RotorAngle;  // Rotor angle, either fetched from hall sensors or from observer
    foc_angle_t AngleStep;   // At startup, step angle is zero, zero speed. This is the angle by which the inverter increments
                             // each PWM cycle under open loop

    foc_angle_t HallAngle;  // Angle generated in the hall sensor estimator

    float sincosangle[2];  // This variable carries the current sin and cosine of the angle being used for Park and Clark transforms, so
                           // they only need computing once per pwm cycle

    float Iab[FOC_TRANSFORMED_CHANNELS + 1];  // Float vector containing the Clark transformed current in amps
    float Idq[FOC_TRANSFORMED_CHANNELS];      // Float vector containing the Park transformed current in amps

    float Vdq[FOC_TRANSFORMED_CHANNELS];
    float Vab[FOC_TRANSFORMED_CHANNELS + 1];
    float inverterVoltage[FOC_TRANSFORMED_CHANNELS + 1];
    float smoothed_idq[2];
    float Idq_req[2];

    uint16_t hall_table[6][4];  // Lookup table, populated by the getHallTable() function and used in estimating the rotor position from
                                // hall sensors in HallAngleEstimator()
} MESCfoc_s;

MESCfoc_s foc_vars;

// fixme: why is this in a struct? what advantages does this give?
typedef struct
{
    int32_t RawADC[FOC_NUM_ADC][FOC_CONV_CHANNELS];  // ADC1 returns Ucurrent, DClink
                                                     // voltage and U phase voltage
                                                     //  ADC2 returns Vcurrent, V and Wphase
                                                     //  voltages
                                                     // ADC3 returns Wcurrent
    // We can use ints rather than uints, since this later helps the conversion of values to float, and the sign bit remains untouched (0)
    int32_t ADCOffset[FOC_NUM_ADC];                      // During detect phase, need to sense the
                                                         // zero current offset
    float ConvertedADC[FOC_NUM_ADC][FOC_CONV_CHANNELS];  // We will fill this with currents
                                                         // in A and voltages in Volts
    uint32_t adc1, adc2, adc3, adc4;

} foc_measurement_t;

foc_measurement_t measurement_buffers;  // fixme: floating function prototype


/* Function prototypes -----------------------------------------------*/

// fixme: inconsistent naming convention for functions. Choose one and stick to
// it across entire codebase. Choice needs to be documented. I recommend
// variable: lower_snake_case = 1;
// function: lowerCamelCase();
// constant: UPPER_CASE;
void MESCInit();
void fastLoop();
void VICheck();
void ADCConversion();  // Roll this into the V_I_Check? less branching, can
                       // probably reduce no.ops and needs doing every cycle
                       // anyway...
// convert currents from uint_16 ADC readings into float A and uint_16 voltages
// into float volts Since the observer needs the Clark transformed current, do
// the Clark and Park transform now
void hallAngleEstimator();  // Going to attempt to make a similar hall angle estimator that rolls the hall state into the main function,
                            // and calls a vector table to find the angle from hall offsets.

void OLGenerateAngle();  // For open loop FOC startup, just use this to generate an angle and velocity ramp, then keep the phase currents at
                         // the requested value without really thinking about things like synchronising, phase etc...

void observerTick();  // Call every time to allow the observer, whatever it is,
                      // to update itself and find motor position

void MESCFOC();  // Field and quadrature current control (PI?)
                 // Inverse Clark and Park transforms

void writePWM();  // Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  // subtract lowest phase to always clamp one phase at 0V or
                  // SVPWM
                  // write CCR registers

void generateBreak();  // Software break that does not stop the PWM timer but
                       // disables the outputs, sum of phU,V,W_Break();
int isMotorRunning();  // return motor state if state is one of the running
                       // states, if it's an idle, error or break state, disable
                       // all outputs and measure the phase voltages - if all
                       // the same, then it's stationary.
void measureResistance();
void measureInductance();
void getHallTable();
void phU_Break();   // Turn all phase U FETs off, Tristate the ouput - For BLDC
                    // mode mainly, but also used for measuring
void phU_Enable();  // Basically un-break phase U, opposite of above...
void phV_Break();
void phV_Enable();
void phW_Break();
void phW_Enable();
