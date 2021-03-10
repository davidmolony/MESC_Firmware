/*
 **
 ******************************************************************************
 * @file           : MESCfoc.c
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

 * MESCfoc.c
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/
#include "MESCfoc.h"

#include "MESCBLDC.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"
#include "sin_cos.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern OPAMP_HandleTypeDef hopamp1, hopamp2, hopamp3;
extern ADC_HandleTypeDef hadc1, hadc2, hadc3;
extern COMP_HandleTypeDef hcomp1, hcomp2, hcomp4, hcomp7;

float one_on_sqrt6 = 0.408248;
float one_on_sqrt3 = 0.577350;
float one_on_sqrt2 = 0.707107;
float sqrt_two_on_3 = 0.816497;
float sqrt3_2 = 1.22474;
float sqrt2 = 1.41421;
float sqrt1_2 = 0.707107;
int adc_conv_end;
uint8_t b_write_flash = 0;
uint8_t b_read_flash = 0;

void MESCInit()
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(3000);  // Give the everything else time to start up (e.g. throttle, controller, PWM source...)

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /// Dummy halltable, for avoiding the startup hall detection
    //    foc_vars.hall_table[0][0] = 3;
    //    foc_vars.hall_table[0][1] = 16000;
    //    foc_vars.hall_table[1][0] = 1;
    //    foc_vars.hall_table[1][1] = foc_vars.hall_table[0][1] + 10922;
    //    foc_vars.hall_table[2][0] = 5;
    //    foc_vars.hall_table[2][1] = foc_vars.hall_table[1][1] + 10922;
    //    foc_vars.hall_table[3][0] = 4;
    //    foc_vars.hall_table[3][1] = foc_vars.hall_table[2][1] + 10922;
    //    foc_vars.hall_table[4][0] = 6;
    //    foc_vars.hall_table[4][1] = foc_vars.hall_table[3][1] + 10922;
    //    foc_vars.hall_table[5][0] = 2;
    //    foc_vars.hall_table[5][1] = foc_vars.hall_table[4][1] + 10922;
    //
    //    for (int i = 0; i < 6; i++)  // Generate the mid point values of the hall table for hall locked loop
    //    {
    //        if (foc_vars.hall_table[i][1] < foc_vars.hall_table[(i + 1) % 6][1])
    //        {
    //            foc_vars.hall_table[i][2] =
    //                (uint16_t)(((uint32_t)foc_vars.hall_table[i][1] + (uint32_t)foc_vars.hall_table[(i + 1) % 6][1]) / 2);
    //        }
    //        else
    //        {  // take care of the loop around behaviour of the uint16_t
    //            foc_vars.hall_table[i][2] =
    //                (uint16_t)(((uint32_t)foc_vars.hall_table[i][1] + (uint32_t)foc_vars.hall_table[(i + 1) % 6][1] + 65535) / 2);
    //        }
    //    }

    motor_init();  // Initialise the motor parameters, either with real values, or zeros if we are to determine the motor params at startup
    hw_init();     // Populate the resistances, gains etc of the PCB - edit within this function if compiling for other PCBs
    // motor.Rphase = 0.1; //Hack to make it skip over currently not used motor parameter detection
    foc_vars.initing = 1;  // Tell it we ARE initing...
                           // BLDCInit();	//Not currently using this, since FOC has taken over as primary method of interest
    // Although we are using an exponential filter over thousands of samples to find this offset, accuracy still improved by starting near
    // to the final value.
    measurement_buffers.ADCOffset[0] = 1900;
    measurement_buffers.ADCOffset[1] = 1900;
    measurement_buffers.ADCOffset[2] = 1900;

    // Start the PWM channels, reset the counter to zero each time to avoid triggering the ADC, which in turn triggers the ISR routine and
    // wrecks the startup
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1022);

    // Initialise the comparators - 3 overcurrent and 1 overvoltage,
    HAL_COMP_Start(&hcomp1);
    HAL_COMP_Start(&hcomp2);
    HAL_COMP_Start(&hcomp4);
    HAL_COMP_Start(&hcomp7);  // OVP comparator, may be unwanted if operating above the divider threshold, the ADC conversion can also be
                              // used to trigger a protection event

    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&measurement_buffers.RawADC[0][0], 3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&measurement_buffers.RawADC[1][0], 3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&measurement_buffers.RawADC[2][0], 1);

    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_EOS);  // We are using the ADC_DMA, so the HAL initialiser doesn't actually enable the ADC conversion
                                              // complete interrupt. This does.

    htim1.Instance->BDTR |= TIM_BDTR_MOE;  // initialising the comparators triggers the break state, so turn it back on
    // At this point we just let the whole thing run off into interrupt land, and the fastLoop() starts to be triggered by the ADC
    // conversion complete interrupt
}

void fastLoop()
{                     // Call this directly from the ADC callback IRQ
    ADCConversion();  // First thing we ever want to do is convert the ADC values to real, useable numbers.

    switch (MotorState)
    {
        case MOTOR_STATE_SENSORLESS_RUN:
            // Call the observer
            // Call the current and phase controller
            // Write the PWM values
            break;

        case MOTOR_STATE_HALL_RUN:
            // transform
            if (MotorControlType == MOTOR_CONTROL_TYPE_BLDC)
            {  // BLDC is hopefully just a temporary "Get it spinning" kind of thing, to be deprecated in
               // favour of FOC
                BLDCCurrentController();
                BLDCCommuteHall();
            }
            if (MotorControlType == MOTOR_CONTROL_TYPE_FOC)
            {
                hallAngleEstimator();
                // foc_vars.Idq_req[0] = 2;
                // foc_vars.Idq_req[1] = 2;
                MESCFOC();
            }
            // Get the current position from HallTimer
            // Call the current and phase controller
            // Write the PWM values
            break;

        case MOTOR_STATE_HALL_NEAR_STATIONARY:

            // Call GetHallState
            // Call the BLDC discrete controller - Override the normal current
            // controller, this is 6 step DC only Write the PWM values
            break;

        case MOTOR_STATE_OPEN_LOOP_STARTUP:
            // Same as open loop
            OLGenerateAngle();
            MESCFOC();
            // Write the PWM values
            break;

        case MOTOR_STATE_OPEN_LOOP_TRANSITION:
            // Run open loop
            // Run observer
            // RunFOC
            // Weighted average of the outputs N PWM cycles
            // Write the PWM values
            break;

        case MOTOR_STATE_IDLE:
            // Do basically nothing
            // ToDo Set PWM to no output state
            break;

        case MOTOR_STATE_DETECTING:;

            int test = GetHallState();

            if ((test == 6) || (test == 7))
            {
                // no hall sensors detected
                MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
            }
            else if (test == 8)
            {
                MotorState = MOTOR_STATE_ERROR;
                MotorError = MOTOR_ERROR_HALL;

            }
            // ToDo add reporting
            else
            {
                // hall sensors detected
                MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
                if (1)
                {
                    getHallTable();
                }
                else
                {
                    MotorState = MOTOR_STATE_HALL_RUN;
                }
                MESCFOC();
            }
            break;

        case MOTOR_STATE_MEASURING:
            if (b_read_flash)
            {
                MotorState = MOTOR_STATE_HALL_RUN;
                b_read_flash = 0;
                break;
            }
            if (motor.Rphase == 0)
            {  // Every PWM cycle we enter this function until
               // the resistance measurement has converged at a
               // good value. Once the measurement is complete,
               // Rphase is set, and this is no longer called
                if (foc_vars.initing == 0)
                {
                    measureResistance();
                }
                break;
            }
            else if (motor.Lphase == 0)  // This is currently rolled into measureResistance() since it seemed pointless to re-write
                                         // basically the same function...
            {
                // As per resistance measurement, this will be called until an
                // inductance measurement is converged.
                // measureInductance();
                break;
            }
            break;

        case MOTOR_STATE_ERROR:
            generateBreak();  // Generate a break state (software disabling all PWM phases, hardware OVCP reserved for fatal situations
                              // requiring reset)
                              // Now panic and freak out
            break;

        case MOTOR_STATE_ALIGN:
            // Turn on at a given voltage at electricalangle0;
            break;
        case MOTOR_STATE_RECOVERING:

            // No clue so far. Read the phase voltages and determine position
            // and attempt to restart? Should already be in break state, and
            // should stay there...
            break;
    }
}

#define MAX_ERROR_COUNT 3

// TODO: refactor this function. Is this function called by DMA interrupt?
void VICheck()
{  // Check currents, voltages are within panic limits
    static int errorCount = 0;

    if ((measurement_buffers.RawADC[0][0] > g_hw_setup.RawCurrLim) || (measurement_buffers.RawADC[1][0] > g_hw_setup.RawCurrLim) ||
        (measurement_buffers.RawADC[2][0] > g_hw_setup.RawCurrLim) || (measurement_buffers.RawADC[0][1] > g_hw_setup.RawVoltLim))
    {
    	foc_vars.Idq_req[0]=foc_vars.Idq_req[0]*0.9;
    	foc_vars.Idq_req[1]=foc_vars.Idq_req[1]*0.9;

        errorCount++;
        if (errorCount >= MAX_ERROR_COUNT)
        {
            generateBreak();
            measurement_buffers.adc1 = measurement_buffers.RawADC[0][0];
            measurement_buffers.adc2 = measurement_buffers.RawADC[1][0];
            measurement_buffers.adc3 = measurement_buffers.RawADC[2][0];
            measurement_buffers.adc4 = measurement_buffers.RawADC[0][1];

            MotorState = MOTOR_STATE_ERROR;
            MotorError = MOTOR_ERROR_OVER_LIMIT;

        }
    }
    else
    {
        errorCount = 0;
    }
}

void ADCConversion()
{
    VICheck();

    // Here we take the raw ADC values, offset, cast to (float) and use the
    // hardware gain values to create volt and amp variables

    if (foc_vars.initing)
    {
        measurement_buffers.ADCOffset[0] = (255 * measurement_buffers.ADCOffset[0] + measurement_buffers.RawADC[0][0]) / 256;
        measurement_buffers.ADCOffset[1] = (255 * measurement_buffers.ADCOffset[1] + measurement_buffers.RawADC[1][0]) / 256;
        measurement_buffers.ADCOffset[2] = (255 * measurement_buffers.ADCOffset[2] + measurement_buffers.RawADC[2][0]) / 256;
        static int initcycles = 0;
        initcycles = initcycles + 1;
        if (initcycles > 1000)
        {
            htim1.Instance->BDTR |= TIM_BDTR_MOE;

            foc_vars.initing = 0;
        }
    }
    else
    {
        measurement_buffers.ConvertedADC[0][0] =
            (float)(measurement_buffers.RawADC[0][0] - measurement_buffers.ADCOffset[0]) * g_hw_setup.Igain;  // Currents
        measurement_buffers.ConvertedADC[1][0] =
            (float)(measurement_buffers.RawADC[1][0] - measurement_buffers.ADCOffset[1]) * g_hw_setup.Igain;
        measurement_buffers.ConvertedADC[2][0] =
            (float)(measurement_buffers.RawADC[2][0] - measurement_buffers.ADCOffset[2]) * g_hw_setup.Igain;
        measurement_buffers.ConvertedADC[0][1] = (float)measurement_buffers.RawADC[0][1] * g_hw_setup.VBGain;  // Vbus
        measurement_buffers.ConvertedADC[0][2] = (float)measurement_buffers.RawADC[0][2] * g_hw_setup.VBGain;  // Usw
        measurement_buffers.ConvertedADC[1][1] = (float)measurement_buffers.RawADC[1][1] * g_hw_setup.VBGain;  // Vsw
        measurement_buffers.ConvertedADC[1][2] = (float)measurement_buffers.RawADC[1][2] * g_hw_setup.VBGain;  // Wsw

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Here we do the FOC transforms - Clark and Park, using the previous sin values, since they were the correct ones at the time of
        // sampling
        // clang-format off

        // Clark - Power invariant version - 3 phases; tested, woring, but won't cope with high duty cycles without phase current sensors
        /*foc_vars.Iab[0] = 	(2.0f * measurement_buffers.ConvertedADC[0][0] -
        					measurement_buffers.ConvertedADC[1][0] -
							measurement_buffers.ConvertedADC[2][0]) * one_on_sqrt6;

        foc_vars.Iab[1] = 	(measurement_buffers.ConvertedADC[1][0] -
        					measurement_buffers.ConvertedADC[2][0]) * one_on_sqrt2;

        foc_vars.Iab[2] = 	(measurement_buffers.ConvertedADC[0][0] +
        					measurement_buffers.ConvertedADC[1][0] +
							measurement_buffers.ConvertedADC[2][0]) * 0.333f;
        */
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Version of Clark transform that avoids low duty cycle ADC measurements - use only 2 phases
        if(htim1.Instance->CCR2>900){
        	//Clark using phase U and W
        	foc_vars.Iab[0] = sqrt3_2*measurement_buffers.ConvertedADC[0][0];
        	foc_vars.Iab[1] = -sqrt1_2*measurement_buffers.ConvertedADC[0][0]-sqrt2*measurement_buffers.ConvertedADC[2][0];
        }
        else if(htim1.Instance->CCR3>900){
        //Clark using phase U and V
        	foc_vars.Iab[0] = sqrt3_2*measurement_buffers.ConvertedADC[0][0];
        	foc_vars.Iab[1] = sqrt2*measurement_buffers.ConvertedADC[1][0]-sqrt1_2*measurement_buffers.ConvertedADC[0][0];
        }
        else{
        	//Clark using phase V and W (hardware V1 has best ADC readings on channels V and W - U is plagued by the DCDC converter)
        	foc_vars.Iab[0] = -sqrt3_2*measurement_buffers.ConvertedADC[1][0]-sqrt3_2*measurement_buffers.ConvertedADC[2][0];
        	foc_vars.Iab[1] = sqrt1_2*measurement_buffers.ConvertedADC[1][0]-sqrt1_2*measurement_buffers.ConvertedADC[2][0];
        }

        // Park
        foc_vars.Idq[0] = foc_vars.sincosangle[1] * foc_vars.Iab[0] + foc_vars.sincosangle[0] * foc_vars.Iab[1];
        foc_vars.Idq[1] = foc_vars.sincosangle[1] * foc_vars.Iab[1] - foc_vars.sincosangle[0] * foc_vars.Iab[0];
        // clang-format on

        adc_conv_end = htim1.Instance->CNT;
    }
}
/////////////////////////////////////////////////////////////////////////////
////////Hall Sensor Implementation///////////////////////////////////////////
static float dir = 1;
static uint16_t testvar;
static int current_hall_state;
static uint16_t current_hall_angle;
static int last_hall_state;
static uint16_t last_hall_angle;
static float ticks_since_last_hall_change = 0;
static float last_hall_period = 65536;
static float one_on_last_hall_period = 1;
float angular_velocity = 0;

void hallAngleEstimator_new()
{
    // 0. Check for hall sensor state errors
    // 1. calculate bulk angle
    // 2. add current velocity correction
    static uint16_t base_angle = 0;
    current_hall_state = ((GPIOB->IDR >> 6) & 0x7);

    if (current_hall_state != last_hall_state)
    {
        if (current_hall_state == 0)
        {
            MotorState = MOTOR_STATE_ERROR;
        }
        else if (current_hall_state == 7)
        {
            MotorState = MOTOR_STATE_ERROR;
        }

        // todo: I want this section to have a PLL like property, adding only a proportion of the error, not rigidly locking it.
        foc_vars.HallAngle = foc_vars.hall_table[current_hall_state - 1][2];
        if (angular_velocity > 0)
        {
            base_angle = foc_vars.HallAngle - (foc_vars.hall_table[current_hall_state - 1][3] >> 1);
        }
        else
        {
            base_angle = foc_vars.HallAngle + (foc_vars.hall_table[current_hall_state - 1][3] >> 1);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        last_hall_state = current_hall_state;
    }
    /* Add velocity correction */
    uint16_t angle_correction = foc_vars.hall_table[last_hall_state - 1][3];
    angle_correction *= angular_velocity * htim4.Instance->CNT;
    foc_vars.HallAngle = base_angle + angular_velocity * htim4.Instance->CNT;  // angle_correction;
}
static int hall_error = 0;
void hallAngleEstimator()
{  // Implementation using the mid point of the hall sensor angles, which should be much more reliable to generate that the edges

    current_hall_state = ((GPIOB->IDR >> 6) & 0x7);

    if (current_hall_state != last_hall_state)
    {
        if (current_hall_state == 0)
        {
            MotorState = MOTOR_STATE_ERROR;
            MotorError = MOTOR_ERROR_HALL;

        }
        else if (current_hall_state == 7)
        {
            MotorState = MOTOR_STATE_ERROR;
            MotorError = MOTOR_ERROR_HALL;
        }
        //////////Implement the Hall table here, but the vector can be dynamically created/filled by another function/////////////
        current_hall_angle = foc_vars.hall_table[current_hall_state - 1][2];

        // Calculate Hall error
        hall_error = foc_vars.HallAngle - current_hall_angle;

        // Todo I want this section to have a PLL like property, adding only a proportion of the error, not rigidly locking it.
        // foc_vars.HallAngle = foc_vars.hall_table[current_hall_state - 1][2];
        uint16_t a;
        if ((a = current_hall_angle - last_hall_angle) < 32000)
        {
            hall_error = hall_error + 5460;
            dir = 1.0f;
            // foc_vars.HallAngle = foc_vars.HallAngle - 5460;
        }
        else
        {
            hall_error = hall_error - 5460;
            dir = -1.0f;
            // foc_vars.HallAngle = foc_vars.HallAngle + 5460;
        }
        if (hall_error > 32000)
        {
            hall_error = hall_error - 65536;
        }
        if (hall_error < -32000)
        {
            hall_error = hall_error + 65536;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        last_hall_state = current_hall_state;
        last_hall_angle = current_hall_angle;
        last_hall_period = (4 * last_hall_period + ticks_since_last_hall_change) * 0.2;
        one_on_last_hall_period = 1 / last_hall_period;  // / ticks_since_last_hall_change;
        ticks_since_last_hall_change = 0;
    }

    ticks_since_last_hall_change = ticks_since_last_hall_change + 1;
    if (ticks_since_last_hall_change <= 1.2 * last_hall_period)
    {
        if (dir > 0)
        {  // Apply a gain to the error as well as the feed forward from the last hall period. Gain between 0.05 and 1.8 seems stable, but
           // 0.05 slow to respond to changing speeds, and approaching 2 it is clear that the overshoot on the error correction is about be
           // unstable... NOISE A gain of ~0.2 seems to be a good compromise for dealing with poorly placed hall sensors while retaining
           // strong phase locking.
            foc_vars.HallAngle = foc_vars.HallAngle + (uint16_t)(one_on_last_hall_period * (10922 - 0.2 * hall_error));
        }
        else if (dir < 0)
        {
            foc_vars.HallAngle = foc_vars.HallAngle - (uint16_t)(one_on_last_hall_period * (10922 + 0.2 * hall_error));
        }
    }
    if (ticks_since_last_hall_change > 500.0f)
    {
    	last_hall_period = 500.0f;//(ticks_since_last_hall_change);
    	one_on_last_hall_period = 1.0f / last_hall_period;  // / ticks_since_last_hall_change;
        foc_vars.HallAngle = current_hall_angle;
    }
    //        (uint16_t)((16383 * ((uint32_t)foc_vars.HallAngle) + (uint32_t)temp_current_hall_angle) >> 14) +
    //        (uint16_t)(one_on_last_hall_period * 10922);
    //(uint16_t)(10922.0f * (current_hall_state + ticks_since_last_hall_change / last_hall_period));
}
void OLGenerateAngle()
{
    // ToDo
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FOC PID algorithms//////////////////////////////////////////////////////////////////////////////////////////
void MESCFOC()
{  // Here we are going to do a PID loop to control the dq currents, converting Idq into Vdq

    // Calculate the errors
    static float Idq_err[2];
    Idq_err[0] = foc_vars.Idq[0] - foc_vars.Idq_req[0];
    Idq_err[1] = foc_vars.Idq[1] - foc_vars.Idq_req[1];

    // Integral error
    static float Idq_int_err[2];
    Idq_int_err[0] = Idq_int_err[0] + 0.05f * Idq_err[0];
    Idq_int_err[1] = Idq_int_err[1] + 0.05f * Idq_err[1];

    static int i = 0;
    if (i == 0)
    {  // set or release the PID controller; may want to do this for cycle skipping, which may help for high inductance motors
        // Bounding
        // clang-format off
        static int integral_d_limit = 150;
        static int integral_q_limit = 600;

        if (Idq_int_err[0] > integral_d_limit){Idq_int_err[0] = integral_d_limit;}
        if (Idq_int_err[0] < -integral_d_limit){Idq_int_err[0] = -integral_d_limit;}
        if (Idq_int_err[1] > integral_q_limit){Idq_int_err[1] = integral_q_limit;}
        if (Idq_int_err[1] < -integral_q_limit){Idq_int_err[1] = -integral_q_limit;}
        // clang-format on
        // Apply the PID, and smooth the output for noise - sudden changes in VDVQ will create instability in the presence of inductance.
        foc_vars.Vdq[0] = (3 * foc_vars.Vdq[0] + Idq_err[0] + Idq_int_err[0]) * 0.25;  // trial pgain of 10
        foc_vars.Vdq[1] = (3 * foc_vars.Vdq[1] + Idq_err[1] + Idq_int_err[1]) * 0.25;
        i = FOC_PERIODS;
        //Field weakening? - The below works pretty nicely, but needs turning into an implementation where it is switchable by the user. Can result in problems e.g. tripping PSUs...
//        if((foc_vars.Vdq[1]>300)){
//        	foc_vars.Idq_req[0]=(foc_vars.Vdq[1]-300)*-0.1; //30A max field weakening current
//        }
//        else if((foc_vars.Vdq[1]<-300)){
//        	foc_vars.Idq_req[0]=(foc_vars.Vdq[1]+300)*0.1; //30A max field weakening current
//        }
//        else{
//        	foc_vars.Idq_req[0]=0; //30A max field weakening current
//
//        }
    }
    i = i - 1;

    // Now we update the sin and cos values, since when we do the inverse transforms, we would like to use the most up to date
    // versions(or even the next predicted version...)
    foc_vars.sincosangle[0] = sinwave[foc_vars.HallAngle >> 8];
    foc_vars.sincosangle[1] = sinwave[(foc_vars.HallAngle >> 8) + 64];
    // Inverse Park transform
    foc_vars.Vab[0] = foc_vars.sincosangle[1] * foc_vars.Vdq[0] - foc_vars.sincosangle[0] * foc_vars.Vdq[1];
    foc_vars.Vab[1] = foc_vars.sincosangle[0] * foc_vars.Vdq[0] + foc_vars.sincosangle[1] * foc_vars.Vdq[1];
    foc_vars.Vab[2] = 0;
    // Inverse Clark transform - power invariant
    foc_vars.inverterVoltage[0] = 0;
    foc_vars.inverterVoltage[1] = -foc_vars.Vab[0] * one_on_sqrt6;
    foc_vars.inverterVoltage[2] = foc_vars.inverterVoltage[1] - one_on_sqrt2 * foc_vars.Vab[1];
    foc_vars.inverterVoltage[1] = foc_vars.inverterVoltage[1] + one_on_sqrt2 * foc_vars.Vab[1];
    foc_vars.inverterVoltage[0] = sqrt_two_on_3 * foc_vars.Vab[0];

    writePWM();
}

void writePWM()
{
    ///////////////////////////////////////////////
	if((foc_vars.Vdq[1]>300)|(foc_vars.Vdq[1]<-300)){
		// Bottom Clamp implementation. Implemented initially because this avoids all nasty behaviour in the event of underflowing the timer
	    // CCRs; if negative values fed to timers, they will saturate positive, which will result in a near instant overcurrent event.

	float minimumValue = 0;

    if (foc_vars.inverterVoltage[0] < foc_vars.inverterVoltage[1])
    {
        if (foc_vars.inverterVoltage[0] < foc_vars.inverterVoltage[2])
        {
            minimumValue = foc_vars.inverterVoltage[0];
        }
        else
        {
            minimumValue = foc_vars.inverterVoltage[2];
        }
    }
    else if (foc_vars.inverterVoltage[1] < foc_vars.inverterVoltage[2])
    {
        minimumValue = foc_vars.inverterVoltage[1];
    }
    else
    {
        minimumValue = foc_vars.inverterVoltage[2];
    }

    foc_vars.inverterVoltage[0] -= minimumValue;
    foc_vars.inverterVoltage[1] -= minimumValue;
    foc_vars.inverterVoltage[2] -= minimumValue;

    ////////////////////////////////////////////////////////
    // SVPM implementation
    // ToDo HAHA not ready for that yet, makes my brain hurt.

    ////////////////////////////////////////////////////////
    // Actually write the value to the timer registers
    htim1.Instance->CCR1 = (uint16_t)(foc_vars.inverterVoltage[0]);
    htim1.Instance->CCR2 = (uint16_t)(foc_vars.inverterVoltage[1]);
    htim1.Instance->CCR3 = (uint16_t)(foc_vars.inverterVoltage[2]);
	}

    ///////////////////////////////////////////////

	else{
		// Sinusoidal implementation
		            htim1.Instance->CCR1 = (uint16_t)(512.0f + foc_vars.inverterVoltage[0]);
		            htim1.Instance->CCR2 = (uint16_t)(512.0f + foc_vars.inverterVoltage[1]);
		            htim1.Instance->CCR3 = (uint16_t)(512.0f + foc_vars.inverterVoltage[2]);
	}
}

// Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
// which should only be set by the hardware comparators, in the case of a
// shoot-through or other catastrophic event This function means that the
// timer can be left running, ADCs sampling etc which enables a recovery, or
// single PWM period break in which the backEMF can be measured directly
// This function needs implementing and testing before any high current or
// voltage is applied, otherwise... DeadFETs
void generateBreak()
{
    phU_Break();
    phV_Break();
    phW_Break();
}

void measureResistance()
{
    /*In this function, we are going to use an openloop  controller to
     * create a current, probably ~4A, through a pair of motor windings, keeping
     * the third tri-stated. We then generate a pair of V and I values, from the
     * bus voltage and duty cycle, and the current reading. We repeat this at
     * higher current, say ~12A, and then apply R=dV/dI from the two values to
     * generate a resistance. Don't use a single point, since this is subject to
     * anomalies from switching dead times, ADC sampling position...etc. Use of
     * the derivative eliminates all steady state error sources ToDo Repeat for
     * all phases? Or just assume they are all close enough that it doesn't
     * matter? Could be useful for disconnection detection...
     */
    static float currAcc1 = 0;
    static float currAcc2 = 0;
    static float currAcc3 = 0;
    static float currAcc4 = 0;

    static uint16_t PWMcycles = 0;

    if (0)  // isMotorRunning() //ToDo, implement this
    {
        // do nothing
    }
    else
    {
        // turn off phW, we are just going to measure RUV
        static uint16_t testPWM1 = 0;  // Start it at zero point, adaptive current thingy can ramp it
        static uint16_t testPWM2 = 0;  //
        static uint16_t testPWM3 = 0;  // Use this for the inductance measurement, calculate it later
        phW_Break();
        phU_Enable();
        phV_Enable();
        ///////////////////////////////////////////////////////RESISTANCE/////////////////////////////////////////////////////////////////////

        if (PWMcycles < 5000)  // Resistance lower measurement point
        {
            if (measurement_buffers.ConvertedADC[1][0] < 3.0f)
            {  // Here we set the PWM duty automatically for this conversion to ensure a current between 3A and 10A
                testPWM1 = testPWM1 + 1;
            }
            if (measurement_buffers.ConvertedADC[1][0] > 10.0f)
            {
                testPWM1 = testPWM1 - 1;
            }

            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = testPWM1;
            // Accumulate the currents with an exponential smoother. This
            // averaging should remove some noise and increase
            // effective resolution
            currAcc1 = (99 * currAcc1 + measurement_buffers.ConvertedADC[1][0]) * 0.01;
        }

        else if (PWMcycles < 10000)  // Resistance higher measurement point
        {
            if (measurement_buffers.ConvertedADC[1][0] < 10.0f)
            {  // Here we set the PWM to get a current between 10A and 20A
                testPWM2 = testPWM2 + 1;
            }
            if (measurement_buffers.ConvertedADC[1][0] > 20.0f)
            {
                testPWM2 = testPWM2 - 1;
            }

            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = testPWM2;
            // Accumulate the currents with an exponential smoother
            currAcc2 = (99 * currAcc2 + measurement_buffers.ConvertedADC[1][0]) * 0.01;
        }
        ///////////////////////////////////////////////////////INDUCTANCE/////////////////////////////////////////////////////////////////////
        else if (PWMcycles == 10000)
        {
            // calculate the resistance from two accumulated currents and two
            // voltages
            testPWM3 = testPWM2;  // We assign the value we determined was OK for the resistance measurement as the
                                  // value to use for inductance measurement, since we need an absolutely  stable steady state
        }

        else if (PWMcycles < 65000)  // Inductance measurement points are rolled into one loop, we will skip pulses on the PWM to generate a
                                     // higher ripple. ToDo Untested with higher inductance motors (only 1.5uH, 6uH and ~60uH motors tested
                                     // as of 20201224) may have to skip multiple pulses
        {
            static int a = 0;  // A variable local to here to track whether the PWM was high or low last time
            if (a == 1)
            {
                htim1.Instance->CCR1 = testPWM3;  // Write the high PWM, the next cycle will be a higher current
                currAcc4 = (999 * currAcc4 + measurement_buffers.ConvertedADC[1][0]) * 0.001;

                a = 0;
            }
            else if (a == 0)
            {
                htim1.Instance->CCR1 = 0;  // Write the PWM low, the next PWM pulse is skipped, and the current allowed to decay
                currAcc3 = (999 * currAcc3 + measurement_buffers.ConvertedADC[1][0]) * 0.001;

                a = 1;
            }
        }

        // This was a prototype where the sampling point was moved within the PWM cycle.
        //(Un?)fortunately, the change in current was quite small, and so the inductance measurement subject to noise.

        /*else if (PWMcycles < 15000)
        {                                 // Measure the inductance first point
            htim1.Instance->CCR4 = 1022;  // Move the ADC trigger point - It does not like being moved to 1023 for some reason...
            htim1.Instance->CCR1 = testPWM3;
            currAcc3 = (999 * currAcc3 + measurement_buffers.ConvertedADC[1][0]) * 0.001;
        }

        else if (PWMcycles < 20000)
        {  // Measure the inductance second point
            if (htim1.Instance->CCR4 > 222)
            {
                htim1.Instance->CCR4 = htim1.Instance->CCR4 - 1;
            }
            // Move the ADC trigger point gradually down to 500 counts from where it was
            // This method works OK, but the change in current is tiny, even for a low inductance motor (~0.5A).
            // Might be better to implement this as skipping cycles and changing CCR1 on a cycle by cycle basis.
            htim1.Instance->CCR1 = testPWM3;
            currAcc4 = (999 * currAcc4 + measurement_buffers.ConvertedADC[1][0]) * 0.001;
        }*/

        else if (PWMcycles == 65000)  // This really does not need to be 65000 cycles, yet I don't want to change it :(
        {                             // Do the calcs
            // First let's just turn everything off. Nobody likes motors sitting
            // there getting hot while debugging.
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR3 = 0;

            phU_Break();
            phV_Break();
            phW_Break();

            motor.Rphase =
                (((float)(testPWM2 - testPWM1)) / (2.0f * 1024.0f) * measurement_buffers.ConvertedADC[0][1]) / (currAcc2 - currAcc1);
            motor.Lphase = ((currAcc3 + currAcc4) * motor.Rphase * (2048.0f / 72000000.0f) / ((currAcc4 - currAcc3) * 2));
            // L=iRdt/di, where R in this case is 2*motor.Rphase
            MotorState = MOTOR_STATE_DETECTING;  // MOTOR_STATE_HALL_RUN;
            phU_Enable();
            phV_Enable();
            phW_Enable();
        }
    }
    PWMcycles = PWMcycles + 1;
}

void getHallTable()
{
    static int firstturn = 1;
    static int hallstate;
    hallstate = ((GPIOB->IDR >> 6) & 0x7);
    static int lasthallstate;
    static int offset = 1000;
    static uint16_t pwm_count = 0;
    static int anglestep = 1;  // This defines how fast the motor spins
    static uint32_t hallangles[7][2];
    static int rollover;

    if (firstturn)
    {
        lasthallstate = hallstate;
        firstturn = 0;
    }

    ////// Align the rotor////////////////////
    static uint16_t a = 65535;
    if (a)  // Align time
    {
        foc_vars.Idq_req[0] = 10;
        foc_vars.Idq_req[1] = 0;

        foc_vars.HallAngle = 32768;
        a = a - 1;
    }
    else
    {
        foc_vars.Idq_req[0] = 10;
        foc_vars.Idq_req[1] = 0;
        static int dir = 1;
        if (pwm_count < 65534)
        {
            if (foc_vars.HallAngle < (anglestep))
            {
                rollover = hallstate;
            }
            if ((foc_vars.HallAngle < (30000)) && (foc_vars.HallAngle > (30000 - anglestep)))
            {
                rollover = 0;
            }
            lasthallstate = hallstate;
            if (rollover == hallstate)
            {
                hallangles[hallstate][0] = hallangles[hallstate][0] + (uint32_t)65535;  // Accumulate the angles through the sweep
            }

            foc_vars.HallAngle = foc_vars.HallAngle + anglestep;                       // Increment the angle
            hallangles[hallstate][0] = hallangles[hallstate][0] + foc_vars.HallAngle;  // Accumulate the angles through the sweep
            hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for this hall state
            pwm_count = pwm_count + 1;
        }
        else if (pwm_count < 65535)
        {
            if (dir == 1)
            {
                dir = 0;
                rollover = 0;
            }
            if ((foc_vars.HallAngle < (12000)) && (hallstate != last_hall_state))
            {
                rollover = hallstate;
            }
            if ((foc_vars.HallAngle < (65535)) && (foc_vars.HallAngle > (65535 - anglestep)))
            {
                rollover = 0;
            }
            lasthallstate = hallstate;
            if (rollover == hallstate)
            {
                hallangles[hallstate][0] = hallangles[hallstate][0] + (uint32_t)65535;  // Accumulate the angles through the sweep
            }

            foc_vars.HallAngle = foc_vars.HallAngle - anglestep;                       // Increment the angle
            hallangles[hallstate][0] = hallangles[hallstate][0] + foc_vars.HallAngle;  // Accumulate the angles through the sweep
            hallangles[hallstate][1]++;  // Accumulate the number of PWM pulses for this hall state
            pwm_count = pwm_count + 1;
        }
    }
    if (pwm_count == 65535)
    {
        generateBreak();  // Debugging
        for (int i = 1; i < 7; i++)
        {
            hallangles[i][0] = hallangles[i][0] / hallangles[i][1];
            if (hallangles[i][0] > 65535)
            {
                hallangles[i][0] = hallangles[i][0] - 65535;
            }
        }
        for (int i = 0; i < 6; i++)
        {
            foc_vars.hall_table[i][0] = i;
            foc_vars.hall_table[i][2] = hallangles[i + 1][0];
            foc_vars.hall_table[i][3] = hallangles[i + 1][1];
        }
        b_write_flash = 1;
        MotorState = MOTOR_STATE_HALL_RUN;
        foc_vars.Idq_req[0] = 0;
        foc_vars.Idq_req[1] = 0;
        phU_Enable();
        phV_Enable();
        phW_Enable();
    }
}

void measureInductance()  // UNUSED, THIS HAS BEEN ROLLED INTO THE MEASURE RESISTANCE... no point in 2 functions really...
{
    /*
     * In this function, we are going to run at a fixed duty cycle (perhaps as
     * determined by Measure Resistance?), pushing ~5A through the motor coils
     * (~100ADCcounts). We will then wait until steady state achieved... 1000
     * PWM cycles? before modulating CCR4, which triggers the ADC to capture
     * currents at at least 2 time points within the PWM cycle With this change
     * in current, and knowing R from previous measurement, we can calculate L
     * using L=Vdt/dI=IRdt/dI ToDo Actually do this... ToDo Determination of the
     * direct and quadrature inductances for MTPA in future?
     */
}

/*fixme: this variable is not scope limited, so it is not temporary. It needs to
 * get a better name and be placed in a .h file. */
uint32_t tmpccmrx;
// Temporary buffer which is used to turn on/off phase PWMs
// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
// ToDo TEST THOROUGHLY The register manipulations for the break functions were
// used previously on an STM32F042K6 for my first BLDC drive, on TIM1, which
// should be identical, but definitely needs checking

void phU_Break()
{
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
}
// Basically un-break phase U, opposite of above...
void phU_Enable()
{
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
}

void phV_Break()
{
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
}

void phV_Enable()
{
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_PWM1 << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
}

void phW_Break()
{
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
}

void phW_Enable()
{
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
}
