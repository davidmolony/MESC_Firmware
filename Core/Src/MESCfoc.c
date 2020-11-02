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

extern TIM_HandleTypeDef htim1;
extern OPAMP_HandleTypeDef hopamp1, hopamp2, hopamp3;
extern ADC_HandleTypeDef hadc1, hadc2, hadc3;
extern COMP_HandleTypeDef hcomp1, hcomp2, hcomp4, hcomp7;

// clang-format off
float sinwave[321] = {0,0.0245412285,0.0490676743,0.0735645636,0.0980171403,0.1224106752,0.1467304745,0.1709618888,0.195090322,0.2191012402,0.2429801799,0.2667127575,0.2902846773,0.3136817404,0.3368898534,0.3598950365,0.3826834324,0.405241314,0.4275550934,0.4496113297,0.4713967368,0.4928981922,0.5141027442,0.5349976199,0.555570233,0.5758081914,0.5956993045,0.6152315906,0.6343932842,0.653172843,0.6715589548,0.6895405447,0.7071067812,0.724247083,0.7409511254,0.7572088465,0.7730104534,0.7883464276,0.8032075315,0.8175848132,0.8314696123,0.8448535652,0.85772861,0.8700869911,0.8819212643,0.8932243012,0.9039892931,0.9142097557,0.9238795325,0.9329927988,0.9415440652,0.9495281806,0.9569403357,0.9637760658,0.9700312532,0.97570213,0.9807852804,0.9852776424,0.98917651,0.9924795346,0.9951847267,0.9972904567,0.9987954562,0.9996988187,1,0.9996988187,0.9987954562,0.9972904567,0.9951847267,0.9924795346,0.98917651,0.9852776424,0.9807852804,0.97570213,0.9700312532,0.9637760658,0.9569403357,0.9495281806,0.9415440652,0.9329927988,0.9238795325,0.9142097557,0.9039892931,0.8932243012,0.8819212643,0.8700869911,0.85772861,0.8448535652,0.8314696123,0.8175848132,0.8032075315,0.7883464276,0.7730104534,0.7572088465,0.7409511254,0.724247083,0.7071067812,0.6895405447,0.6715589548,0.653172843,0.6343932842,0.6152315906,0.5956993045,0.5758081914,0.555570233,0.5349976199,0.5141027442,0.4928981922,0.4713967368,0.4496113297,0.4275550934,0.405241314,0.3826834324,0.3598950365,0.3368898534,0.3136817404,0.2902846773,0.2667127575,0.2429801799,0.2191012402,0.195090322,0.1709618888,0.1467304745,0.1224106752,0.0980171403,0.0735645636,0.0490676743,0.0245412285,1.22464679914735E-016,-0.0245412285,-0.0490676743,-0.0735645636,-0.0980171403,-0.1224106752,-0.1467304745,-0.1709618888,-0.195090322,-0.2191012402,-0.2429801799,-0.2667127575,-0.2902846773,-0.3136817404,-0.3368898534,-0.3598950365,-0.3826834324,-0.405241314,-0.4275550934,-0.4496113297,-0.4713967368,-0.4928981922,-0.5141027442,-0.5349976199,-0.555570233,-0.5758081914,-0.5956993045,-0.6152315906,-0.6343932842,-0.653172843,-0.6715589548,-0.6895405447,-0.7071067812,-0.724247083,-0.7409511254,-0.7572088465,-0.7730104534,-0.7883464276,-0.8032075315,-0.8175848132,-0.8314696123,-0.8448535652,-0.85772861,-0.8700869911,-0.8819212643,-0.8932243012,-0.9039892931,-0.9142097557,-0.9238795325,-0.9329927988,-0.9415440652,-0.9495281806,-0.9569403357,-0.9637760658,-0.9700312532,-0.97570213,-0.9807852804,-0.9852776424,-0.98917651,-0.9924795346,-0.9951847267,-0.9972904567,-0.9987954562,-0.9996988187,-1,-0.9996988187,-0.9987954562,-0.9972904567,-0.9951847267,-0.9924795346,-0.98917651,-0.9852776424,-0.9807852804,-0.97570213,-0.9700312532,-0.9637760658,-0.9569403357,-0.9495281806,-0.9415440652,-0.9329927988,-0.9238795325,-0.9142097557,-0.9039892931,-0.8932243012,-0.8819212643,-0.8700869911,-0.85772861,-0.8448535652,-0.8314696123,-0.8175848132,-0.8032075315,-0.7883464276,-0.7730104534,-0.7572088465,-0.7409511254,-0.724247083,-0.7071067812,-0.6895405447,-0.6715589548,-0.653172843,-0.6343932842,-0.6152315906,-0.5956993045,-0.5758081914,-0.555570233,-0.5349976199,-0.5141027442,-0.4928981922,-0.4713967368,-0.4496113297,-0.4275550934,-0.405241314,-0.3826834324,-0.3598950365,-0.3368898534,-0.3136817404,-0.2902846773,-0.2667127575,-0.2429801799,-0.2191012402,-0.195090322,-0.1709618888,-0.1467304745,-0.1224106752,-0.0980171403,-0.0735645636,-0.0490676743,-0.0245412285,-2.44929359829471E-016,0.0245412285,0.0490676743,0.0735645636,0.0980171403,0.1224106752,0.1467304745,0.1709618888,0.195090322,0.2191012402,0.2429801799,0.2667127575,0.2902846773,0.3136817404,0.3368898534,0.3598950365,0.3826834324,0.405241314,0.4275550934,0.4496113297,0.4713967368,0.4928981922,0.5141027442,0.5349976199,0.555570233,0.5758081914,0.5956993045,0.6152315906,0.6343932842,0.653172843,0.6715589548,0.6895405447,0.7071067812,0.724247083,0.7409511254,0.7572088465,0.7730104534,0.7883464276,0.8032075315,0.8175848132,0.8314696123,0.8448535652,0.85772861,0.8700869911,0.8819212643,0.8932243012,0.9039892931,0.9142097557,0.9238795325,0.9329927988,0.9415440652,0.9495281806,0.9569403357,0.9637760658,0.9700312532,0.97570213,0.9807852804,0.9852776424,0.98917651,0.9924795346,0.9951847267,0.9972904567,0.9987954562,0.9996988187,1};
//Vector 256 long of sin wave, stretched by 65 to allow computation of cosine as sin(angle+64) without needing wrapping
// clang-format on

float one_on_sqrt6 = 0.408248;
float one_on_sqrt3 = 0.577350;
float one_on_sqrt2 = 0.707107;
float sqrt_two_on_3 = 0.816497;
int adc_conv_end;

void MESCInit()
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(3000);  // Give the everything else time to start up (e.g. throttle, controller, PWM source...)

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
    /// Dummy halltable
    foc_vars.hall_table[0][0] = 1;
    foc_vars.hall_table[0][1] = 0;
    foc_vars.hall_table[1][0] = 3;
    foc_vars.hall_table[1][1] = 10922;
    foc_vars.hall_table[2][0] = 2;
    foc_vars.hall_table[2][1] = 21844;
    foc_vars.hall_table[3][0] = 6;
    foc_vars.hall_table[3][1] = 32766;
    foc_vars.hall_table[4][0] = 4;
    foc_vars.hall_table[4][1] = 43688;
    foc_vars.hall_table[5][0] = 5;
    foc_vars.hall_table[5][1] = 54610;

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
    //__HAL_TIM_MOE_ENABLE(&htim1);  // initialising the comparators triggers the break state

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
            {  // BLDC is hopefully just a
               // temporary "Get it spinning" kind
               // of thing, to be deprecated in
               // favour of FOC
                BLDCCurrentController();
                BLDCCommuteHall();
            }
            if (MotorControlType == MOTOR_CONTROL_TYPE_FOC)
            {
                // HallAngleEstimator();
                HallAngleEstimator_v2();
                //foc_vars.Idq_req[0] = 2;
                //foc_vars.Idq_req[1] = 2;

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

            // openLoopPIFF();
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
            }
            // ToDo add reporting
            else
            {
                // hall sensors detected
                MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
                getHallTable();
                MESCFOC();
            }
            break;

        case MOTOR_STATE_MEASURING:
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
            GenerateBreak();  // Generate a break state (software disabling all PWM phases, hardware OVCP reserved for fatal situations
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

// TODO: refactor this function. Is this function called by DMA interrupt?
void V_I_Check()
{  // Check currents, voltages are within panic limits
    if ((measurement_buffers.RawADC[0][0] > g_hw_setup.RawCurrLim) || (measurement_buffers.RawADC[1][0] > g_hw_setup.RawCurrLim) ||
        (measurement_buffers.RawADC[2][0] > g_hw_setup.RawCurrLim) || (measurement_buffers.RawADC[0][1] > g_hw_setup.RawVoltLim))
    {
        GenerateBreak();
        MotorState = MOTOR_STATE_ERROR;
    }
}

void ADCConversion()
{
    V_I_Check();

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

        // Here we do the FOC transforms - Clark and Park, using the previous sin values, since they were the correct ones at the time of
        // sampling
        foc_vars.Iab[0] = (2.0f * measurement_buffers.ConvertedADC[0][0] - measurement_buffers.ConvertedADC[1][0] -
                           measurement_buffers.ConvertedADC[2][0]) *
                          one_on_sqrt6;
        foc_vars.Iab[1] = (measurement_buffers.ConvertedADC[1][0] - measurement_buffers.ConvertedADC[2][0]) * one_on_sqrt2;
        foc_vars.Iab[2] =
            (measurement_buffers.ConvertedADC[0][0] + measurement_buffers.ConvertedADC[1][0] + measurement_buffers.ConvertedADC[2][0]) *
            0.333f;

        foc_vars.Idq[0] = foc_vars.sincosangle[1] * foc_vars.Iab[0] + foc_vars.sincosangle[0] * foc_vars.Iab[1];
        foc_vars.Idq[1] = foc_vars.sincosangle[1] * foc_vars.Iab[1] - foc_vars.sincosangle[0] * foc_vars.Iab[0];

        // Now we update the sin and cos values, since when we do the inverse transforms, we would like to use the most up to date
        // versions(or even the next predicted version...)
        foc_vars.sincosangle[0] = sinwave[foc_vars.HallAngle >> 8];
        foc_vars.sincosangle[1] = sinwave[(foc_vars.HallAngle >> 8) + 64];
        adc_conv_end = htim1.Instance->CNT;
    }
}

void HallAngleEstimator()
{  // ToDo This does not work when going backwards!it steps to the next 60 degrees and then counts forwards
    static int current_hall_state;
    static int last_hall_state;
    static float ticks_since_last_hall_change = 0;
    static float last_hall_period = 65536;

    current_hall_state = GetHallState();

    if (current_hall_state != last_hall_state)
    {
        last_hall_state = current_hall_state;
        last_hall_period = ticks_since_last_hall_change;
        ticks_since_last_hall_change = 0;
    }
    else
    {
        ticks_since_last_hall_change = ticks_since_last_hall_change + 1;
    }
    // map the current hall state and current ticks into the hall state to an angle where uint16_t represents one revolution
    if (last_hall_period > ticks_since_last_hall_change)
    {
        foc_vars.HallAngle = (uint16_t)(10922.0f * (current_hall_state + ticks_since_last_hall_change / last_hall_period));
    }
}

void HallAngleEstimator_v2()
{
    static uint16_t hallstate;
    static uint16_t last_hall_state;
    static float dir = 1;
    static float ticks_since_last_hall_change = 0;
    static float one_on_last_hall_period = 1;
    static float last_hall_period = 65536;

    hallstate = ((GPIOB->IDR >> 6) & 0x7);  // This is board specific, depending where the hall sensors are wired

    if (hallstate != last_hall_state)
    {
        if (hallstate == 0)
        {
            MotorState = MOTOR_STATE_ERROR;
        }
        else if (hallstate == 7)
        {
            MotorState = MOTOR_STATE_ERROR;
        }
        //////////Implement the Hall table here, but the vector can be dynamically created/filled by another function/////////////
        else if (hallstate == foc_vars.hall_table[0][0])
        {
            if (last_hall_state == foc_vars.hall_table[5][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[0][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[1][1];
                dir = -1;
            }
        }
        /////////////////////////////
        else if (hallstate == foc_vars.hall_table[1][0])
        {
            if (last_hall_state == foc_vars.hall_table[0][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[1][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[2][1];
                dir = -1;
            }
        }
        //////////////////////////////
        else if (hallstate == foc_vars.hall_table[2][0])
        {
            if (last_hall_state == foc_vars.hall_table[1][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[2][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[3][1];
                dir = -1;
            }
        }
        //////////////////////////////
        else if (hallstate == foc_vars.hall_table[3][0])
        {
            if (last_hall_state == foc_vars.hall_table[2][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[3][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[4][1];
                dir = -1;
            }
        }
        //////////////////////////////
        else if (hallstate == foc_vars.hall_table[4][0])
        {
            if (last_hall_state == foc_vars.hall_table[3][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[4][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[5][1];
                dir = -1;
            }
        }
        ///////////////////////////
        else if (hallstate == foc_vars.hall_table[5][0])
        {
            if (last_hall_state == foc_vars.hall_table[4][0])
            {  // Forwards
                foc_vars.HallAngle = foc_vars.hall_table[5][1];
                dir = 1;
            }
            else
            {  // Backwards
                foc_vars.HallAngle = foc_vars.hall_table[0][1];
                dir = -1;
            }
        }
        ///////////////End of table implementation, now do the reseting of counters and once per hall change stuff////////////
        last_hall_state = hallstate;
        last_hall_period = ticks_since_last_hall_change;
        ticks_since_last_hall_change = 0;
        one_on_last_hall_period = 1 / last_hall_period;
    }

    else
    {
        ticks_since_last_hall_change = ticks_since_last_hall_change + 1;
    }

    if (last_hall_period >
        ticks_since_last_hall_change)  // Does this really need doing? Stops sin wave jumping back if the period is
                                       // getting longer, but does it really matter... it'll jump forward if accelerating regardless
    {                                  // Addition to angle stuff here
        if (dir == 1)
        {
            foc_vars.HallAngle = foc_vars.HallAngle + (uint16_t)(10922.0f * (one_on_last_hall_period));
        }
        if (dir == -1)
        {
            foc_vars.HallAngle = foc_vars.HallAngle - (uint16_t)(10922.0f * (one_on_last_hall_period));
        }
    }
}

void MESCFOC()
{
    // Here we are going to do a PID loop to control the dq currents, converting Idq into Vdq
    // We will then inverse Park and Clark the Vdq which will then be written into the PWM registers by the writePWM() function.
    if (0)
    {
        float duty = 10 * BLDCVars.ReqCurrent;  /// Massive hack... essentially turns the PWM input from a requested current into a duty
                                                /// cycle...
        foc_vars.Vdq[0] = 0.0 * duty;
        foc_vars.Vdq[1] = 0.0 * duty;
    }
    // Get rid of this hack later, once the inverse Park and Clark work.
    if (1)
    {
        // Generate Idq
        // static float Idq_req[2];
        // foc_vars.Idq_req[0] = BLDCVars.ReqCurrent * -1.0f;  // Map this to experimentally found optimum
        // foc_vars.Idq_req[1] = BLDCVars.ReqCurrent * -0.0f;  //

        // First, we want to get a smoother version of the current, less susceptible to jitter and noise, use exponential filter. This
        // unfortunately creates lag.
        foc_vars.smoothed_idq[0] = (1.0f * foc_vars.smoothed_idq[0] + foc_vars.Idq[0]) * 0.5f;
        foc_vars.smoothed_idq[1] = (1.0f * foc_vars.smoothed_idq[1] + foc_vars.Idq[1]) * 0.5f;

        // Calculate the errors
        static float Idq_err[2];

        Idq_err[0] = foc_vars.smoothed_idq[0] - foc_vars.Idq_req[0];
        Idq_err[1] = foc_vars.smoothed_idq[1] - foc_vars.Idq_req[1];

        // Integral error
        static float Idq_int_err[2];
        Idq_int_err[0] = Idq_int_err[0] + 0.28f * Idq_err[0];
        Idq_int_err[1] = Idq_int_err[1] + 0.28f * Idq_err[1];
        // Bounding
        static int integral_limit = 200;
        if (Idq_int_err[0] > integral_limit)
        {
            Idq_int_err[0] = integral_limit;
        }
        if (Idq_int_err[0] < -integral_limit)
        {
            Idq_int_err[0] = -integral_limit;
        }
        if (Idq_int_err[1] > integral_limit)
        {
            Idq_int_err[1] = integral_limit;
        }
        if (Idq_int_err[1] < -integral_limit)
        {
            Idq_int_err[1] = -integral_limit;
        }
        static int i=0;
        if (i==0)
        {  // set or release the PID controller
            // Apply the PID
            foc_vars.Vdq[0] = 10 * Idq_err[0] + Idq_int_err[0];  // trial pgain of 10
            foc_vars.Vdq[1] = 10 * Idq_err[0] + Idq_int_err[1];
            i=1;
        }
        i=i-1;
    }
    // Inverse Park transform
    foc_vars.Vab[0] = foc_vars.sincosangle[1] * foc_vars.Vdq[0] - foc_vars.sincosangle[0] * foc_vars.Vdq[1];
    foc_vars.Vab[1] = foc_vars.sincosangle[0] * foc_vars.Vdq[0] + foc_vars.sincosangle[1] * foc_vars.Vdq[1];
    foc_vars.Vab[2] = 0;
    // Inverse Clark transform
    foc_vars.inverterVoltage[0] = 0;
    foc_vars.inverterVoltage[1] = -foc_vars.Vab[0] * one_on_sqrt6;
    foc_vars.inverterVoltage[2] = foc_vars.inverterVoltage[1] - one_on_sqrt2 * foc_vars.Vab[1];
    foc_vars.inverterVoltage[1] = foc_vars.inverterVoltage[1] + one_on_sqrt2 * foc_vars.Vab[1];
    foc_vars.inverterVoltage[0] = sqrt_two_on_3 * foc_vars.Vab[0];

    writePWM();
}

void writePWM()
{
    htim1.Instance->CCR1 = (uint16_t)(512 + foc_vars.inverterVoltage[0]);
    htim1.Instance->CCR2 = (uint16_t)(512 + foc_vars.inverterVoltage[1]);
    htim1.Instance->CCR3 = (uint16_t)(512 + foc_vars.inverterVoltage[2]);
}

void GenerateBreak()
{
    // Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
    // which should only be set by the hardware comparators, in the case of a
    // shoot-through or other catastrophic event This function means that the
    // timer can be left running, ADCs sampling etc which enables a recovery, or
    // single PWM period break in which the backEMF can be measured directly
    // This function needs implementing and testing before any high current or
    // voltage is applied, otherwise... DeadFETs
    phU_Break();
    phV_Break();
    phW_Break();
}

int GetHallState()
{
    switch (((GPIOB->IDR >> 6) & 0x7))
    {
        case 0:
            return 7;  // 7 is the no hall sensor detected state (all low)
            break;
        case 7:
            return 6;  // 6 is the no hall sensor detected state (all high)
            break;
            // Implement the hall table order here, depending how the hall
            // sensors are configured
        case 1:
            return 0;
            break;
        case 3:
            return 1;
            break;
        case 2:
            return 2;
            break;
        case 6:
            return 3;
            break;
        case 4:
            return 4;
            break;
        case 5:
            return 5;
            break;
        default:
            return 8;
            break;
    }
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

    if (0)  // isMotorRunning()
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
                                     // higher ripple. ToDo Untested with higher inductance motors (only 1.5uH and 6uHmotor tested as of
                                     // 20201030) may have to skip multiple pulses
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
            htim1.Instance->CCR2 = 0;

            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = 0;

            phU_Break();
            phV_Break();
            phW_Break();

            motor.Rphase =
                (((float)(testPWM2 - testPWM1)) / (2.0f * 1024.0f) * measurement_buffers.ConvertedADC[0][1]) / (currAcc2 - currAcc1);
            motor.Lphase = ((currAcc3 + currAcc4) * motor.Rphase * (2048.0f / 72000000.0f) / ((currAcc4 - currAcc3) * 2));
            // L=iRdt/di, where R in this case is 2*motor.Rphase
            __NOP();
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
    static int offset=5000;
    static int count=0;
    static int anglestep=20;
    if (firstturn)
    {
        lasthallstate = hallstate;
        firstturn = 0;
    }

    // Align the rotor
    static uint16_t a = 65535;
    if (a)  // Align time
    {
        foc_vars.Idq_req[0] = 15;
        foc_vars.Idq_req[1] = 0;

        foc_vars.HallAngle = 0;
        a = a - 1;
    }
    // Slowly spin the rotor
    else
    {
            foc_vars.HallAngle = foc_vars.HallAngle + anglestep;
    }
    if (hallstate != lasthallstate)
    {
        lasthallstate = hallstate;
        if (foc_vars.HallAngle < offset)
        {
            foc_vars.hall_table[5][0] = hallstate;
            foc_vars.hall_table[5][1] = foc_vars.HallAngle;//(uint16_t)((31*((uint32_t)foc_vars.hall_table[5][1]+(uint32_t)foc_vars.HallAngle))>>5);
        }
        else if (foc_vars.HallAngle < (offset+10922))
        {
            foc_vars.hall_table[0][0] = hallstate;
            foc_vars.hall_table[0][1] = foc_vars.HallAngle;
        }
        else if (foc_vars.HallAngle < (offset+21844))
        {
            foc_vars.hall_table[1][0] = hallstate;
            foc_vars.hall_table[1][1] = foc_vars.HallAngle;
        }
        else if (foc_vars.HallAngle < (offset+32766))
        {
            foc_vars.hall_table[2][0] = hallstate;
            foc_vars.hall_table[2][1] = foc_vars.HallAngle;
        }
        else if (foc_vars.HallAngle < (offset+43688))
        {
            foc_vars.hall_table[3][0] = hallstate;
            foc_vars.hall_table[3][1] = foc_vars.HallAngle;
        }
        else if (foc_vars.HallAngle < (offset+54610))
        {
            foc_vars.hall_table[4][0] = hallstate;
            foc_vars.hall_table[4][1] = foc_vars.HallAngle;
        }
        else if (foc_vars.HallAngle < (offset+65535))
        {
            foc_vars.hall_table[5][0] = hallstate;
            foc_vars.hall_table[5][1] =  foc_vars.HallAngle;//(uint16_t)((31*((uint32_t)foc_vars.hall_table[5][1]+(uint32_t)foc_vars.HallAngle))>>5);
        }
        if(count>1000){MotorState=MOTOR_STATE_HALL_RUN;
        foc_vars.Idq_req[0]=0;
        foc_vars.Idq_req[1]=-0.2;

        }
        count=count+1;
        if(anglestep<200){anglestep++;}
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
