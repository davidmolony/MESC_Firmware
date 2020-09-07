/*
 **
 ******************************************************************************
 * @file           : MESCBLDC.c
 * @brief          : BLDC running code
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

 * MESCBLDC.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

#include "MESCBLDC.h"

#include "MESCfoc.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"

extern TIM_HandleTypeDef htim1;

void BLDCInit() {
    BLDCVars.ReqCurrent = 0;  // Start the motor at 0 current
    BLDCVars.BLDCduty = 0;
    BLDCVars.CurrentChannel = 0;
    BLDCVars.currentCurrent = 0;
    BLDCVars.pGain =
        1023 * motor.Rphase /
        8;  // wtf should I set the gain as by default... V/Amp error...Perhaps
            // base it on Rphase and the bus voltage (nominally 48V)? But we
            // don;t know the exact bus voltage yet...
    BLDCVars.iGain =
        BLDCVars.pGain;  // Initially, let's just make the iGain the
                         // same as the pGain, so after 1 second
                         // their contributions will be equal.
    BLDCVars.BLDCEstate = GetHallState();
    BLDCState = BLDC_FORWARDS;
}

void BLDCCommuteHall() {
    int CurrentHallState =
        GetHallState();  // Borrow the hall state detection from the FOC system
    static int LastHallState = 7;

    if (BLDCState == BLDC_FORWARDS) {
        BLDCVars.BLDCEstate = (CurrentHallState + 2) % 6;
        writeBLDC();  // Write the PWM values for the next state to generate
                      // forward torque
        if (!(BLDCVars.BLDCEstate == (CurrentHallState + 1))) {
            // ToDo Fix if the writeBLDC command is put in here, the PWM duty
            // gets stuck at 0.
        }
    } else if (BLDCState == BLDC_BACKWARDS) {
        BLDCVars.BLDCEstate = (CurrentHallState + 4) % 6;
        writeBLDC();  // Write the PWM values for the previous state to generate
                      // reverse torque
                      // FIXME: what is this supposed to accomplish?
        // commented out since this code does nothing and is likely removed by
        // the compiler. 	if(!(CurrentHallState==CurrentHallState)){
        //	}
    } else if (BLDCState == BLDC_BRAKE) {
        int hallStateChange = CurrentHallState - LastHallState;
        // ToDo Logic to always be on synch or hanging 1 step in front or
        // behind... ToDo this does not cope with the roll-over, making for a
        // very jerky brake
        // TODO: the expression inside if() statement is very hard to read.
        // Create separate variable.
        if (((hallStateChange) % 6) > 1) {
            BLDCVars.BLDCEstate = (CurrentHallState + 5) % 6;
        } else if (((CurrentHallState - LastHallState) % 6) < -1) {
            BLDCVars.BLDCEstate = (CurrentHallState + 1) % 6;
            LastHallState = CurrentHallState;
        }
        writeBLDC();
    } else {
        // Disable the drivers, freewheel
        // fixme: misleading function name. If this is freewheel, then it should
        // be named as such.
        phU_Break();
        phV_Break();
        phW_Break();
    }
}

void BLDCCurrentController() {
    // Implement a simple PI controller
    static float CurrentError = 0;
    static float CurrentIntegralError = 0;
    static int Duty = 0;

    BLDCVars.currentCurrent =
        measurement_buffers.ConvertedADC[BLDCVars.CurrentChannel][0];

    CurrentError = (BLDCVars.ReqCurrent - BLDCVars.currentCurrent);
    // measurement_buffers.ConvertedADC[BLDCVars.CurrentChannel][0]);

    CurrentIntegralError =
        CurrentIntegralError +
        CurrentError * 0.000027;  // 37kHz PWM, so the integral portion should
                                  // be multiplied by 1/37k before accumulating
    if (CurrentIntegralError > 10) CurrentIntegralError = 10;  // Magic numbers
    if (CurrentIntegralError < -10)
        CurrentIntegralError = -10;  // Magic numbers

    Duty = (int)(CurrentError * BLDCVars.pGain +
                 CurrentIntegralError * BLDCVars.iGain);

    if (Duty > 1023) {
        Duty = 1023;
    } else if (Duty < 0) {
        Duty = 0;
    }
    BLDCVars.BLDCduty = Duty;
}

void writeBLDC() {
    switch (BLDCVars.BLDCEstate) {
        case 0:
            // disable phase first
            phW_Break();
            // WritePWM values
            htim1.Instance->CCR1 = BLDCVars.BLDCduty;
            htim1.Instance->CCR2 = 0;
            phU_Enable();
            phV_Enable();
            BLDCVars.CurrentChannel =
                1;  // Write the field into which the lowside current will flow,
                    // to be retrieved from the FOC_measurement_vars
            break;

        case 1:
            phV_Break();
            htim1.Instance->CCR1 = BLDCVars.BLDCduty;
            htim1.Instance->CCR3 = 0;
            phU_Enable();
            phW_Enable();
            BLDCVars.CurrentChannel = 2;
            break;

        case 2:
            phU_Break();
            htim1.Instance->CCR2 = BLDCVars.BLDCduty;
            htim1.Instance->CCR3 = 0;
            phV_Enable();
            phW_Enable();
            BLDCVars.CurrentChannel = 2;
            break;

        case 3:
            phW_Break();
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR2 = BLDCVars.BLDCduty;
            phU_Enable();
            phV_Enable();
            BLDCVars.CurrentChannel = 0;
            break;

        case 4:
            phV_Break();
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = BLDCVars.BLDCduty;
            phU_Enable();
            phW_Enable();
            BLDCVars.CurrentChannel = 0;
            break;

        case 5:
            phU_Break();
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR3 = BLDCVars.BLDCduty;
            phV_Enable();
            phW_Enable();
            BLDCVars.CurrentChannel = 1;
            break;
        default:
            break;
    }
}
