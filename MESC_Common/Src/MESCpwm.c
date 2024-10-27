/*
 **
 ******************************************************************************
 * @file           : MESCpwm.c
 * @brief          : Functions for driving the PWM
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
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************
 */

#include "MESCpwm.h"
#include "MESChfi.h"
#include "MESCsin_lut.h"



static const float sqrt3_on_2 = 0.866025f;

//Debug
#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

// This should be the function needed to be added into the PWM interrupt
// for MESC to run. Ensure that it is followed by the clear timer update
// interrupt
void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor) {
#ifdef FASTLED
	FASTLED->BSRR = FASTLEDIO;
#endif
	uint32_t cycles = CPU_CYCLES;
	if (_motor->mtimer->Instance->CR1&0x16) {//Polling the DIR (direction) bit on the motor counter DIR = 1 = downcounting
		MESCpwm_Write(_motor);
	}
	if (!(_motor->mtimer->Instance->CR1&0x16)) {//Polling the DIR (direction) bit on the motor counter DIR = 0 = upcounting
		  MESChfi_Run(_motor);
		  MESCpwm_Write(_motor);
	}
	_motor->FOC.cycles_pwmloop = CPU_CYCLES - cycles;

#ifdef FASTLED
	FASTLED->BSRR = FASTLEDIO<<16U;
#endif
}

void MESCpwm_Write(MESC_motor_typedef *_motor) {
	float mid_value = 0;
	float top_value;
	float bottom_value;

	float Vd, Vq;

	Vd = _motor->FOC.Vdq.d + _motor->HFI.Vd_injectionV;
	Vq = _motor->FOC.Vdq.q + _motor->HFI.Vq_injectionV;

    // Now we update the sin and cos values, since when we do the inverse
    // transforms, we would like to use the most up to date versions(or even the
    // next predicted version...)
#ifdef INTERPOLATE_V7_ANGLE
if((fabsf(_motor->FOC.eHz)>0.005f*_motor->FOC.pwm_frequency)&&(_motor->HFI.inject==0)){
	//Only run it when there is likely to be good speed measurement stability and
	//actual utility in doing it. At low speed, there is minimal benefit, and
	//unstable speed estimation could make it worse.
	//Presently, this causes issues with openloop iteration, and effectively doubles the speed. TBC
	_motor->FOC.FOCAngle = _motor->FOC.FOCAngle + 0.5f*_motor->FOC.PLL_int;
}
#endif
	sin_cos_fast(_motor->FOC.FOCAngle, &_motor->FOC.sincosangle.sin, &_motor->FOC.sincosangle.cos);

    // Inverse Park transform
    _motor->FOC.Vab.a = _motor->FOC.sincosangle.cos * Vd -
                      _motor->FOC.sincosangle.sin * Vq;
    _motor->FOC.Vab.b = _motor->FOC.sincosangle.sin * Vd +
                      _motor->FOC.sincosangle.cos * Vq;
#ifdef STEPPER_MOTOR//Skip inverse Clark

    _motor->mtimer->Instance->CCR1 = (uint16_t)(1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.a) + _motor->FOC.PWMmid);
    _motor->mtimer->Instance->CCR2 = (uint16_t)(-1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.a) + _motor->FOC.PWMmid);
    _motor->mtimer->Instance->CCR3 = (uint16_t)(1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.b) + _motor->FOC.PWMmid);
    _motor->mtimer->Instance->CCR4 = (uint16_t)(-1.0f * _motor->FOC.Vab_to_PWM * (_motor->FOC.Vab.b) + _motor->FOC.PWMmid);


#else
	// Inverse Clark transform - power variant
	_motor->FOC.inverterVoltage[0] = _motor->FOC.Vab.a;
	_motor->FOC.inverterVoltage[1] = -0.5f*_motor->FOC.inverterVoltage[0];
	_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[1] - sqrt3_on_2 * _motor->FOC.Vab.b;
	_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1] + sqrt3_on_2 * _motor->FOC.Vab.b;

    ////////////////////////////////////////////////////////
    // SVPM implementation
    // Try to do this as a "midpoint clamp" where rather than finding the
    // lowest, we find the highest and lowest and subtract the middle
    top_value = _motor->FOC.inverterVoltage[0];
    bottom_value = top_value;
    _motor->HighPhase = U;

    if (_motor->FOC.inverterVoltage[1] > top_value) {
      top_value = _motor->FOC.inverterVoltage[1];
      _motor->HighPhase = V;
    }
    if (_motor->FOC.inverterVoltage[2] > top_value) {
      top_value = _motor->FOC.inverterVoltage[2];
      _motor->HighPhase = W;
    }
    if (_motor->FOC.inverterVoltage[1] < bottom_value) {
      bottom_value = _motor->FOC.inverterVoltage[1];
    }
    if (_motor->FOC.inverterVoltage[2] < bottom_value) {
      bottom_value = _motor->FOC.inverterVoltage[2];
    }
    if(_motor->FOC.Voltage < _motor->FOC.V_3Q_mag_max){
        _motor->HighPhase = N; //Trigger the full clark transform
    }

    switch(_motor->options.pwm_type){
    case PWM_SVPWM:
    	   mid_value = _motor->FOC.PWMmid -
    	                0.5f * _motor->FOC.Vab_to_PWM * (top_value + bottom_value);

    	    ////////////////////////////////////////////////////////
    	    // Actually write the value to the timer registers
    	    _motor->mtimer->Instance->CCR1 =
    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0] + mid_value);
    	    _motor->mtimer->Instance->CCR2 =
    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1] + mid_value);
    	    _motor->mtimer->Instance->CCR3 =
    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2] + mid_value);

    	    //Dead time compensation
    	#ifdef DEADTIME_COMP
    	    // LICENCE NOTE:
    	    	  // This function deviates slightly from the BSD 3 clause licence.
    	    	  // The work here is entirely original to the MESC FOC project, and not based
    	    	  // on any appnotes, or borrowed from another project. This work is free to
    	    	  // use, as granted in BSD 3 clause, with the exception that this note must
    	    	  // be included in where this code is implemented/modified to use your
    	    	  // variable names, structures containing variables or other minor
    	    	  // rearrangements in place of the original names I have chosen, and credit
    	    	  // to David Molony as the original author must be noted.
    	    //The problem with dead time, is that it is essentially a voltage tie through the body diodes to VBus or ground, depending on the current direction.
    	    //If we know the direction of current, and the effective dead time length we can remove this error, by writing the corrected voltage.
    	    //This is observed to improve sinusoidalness of currents, but has a slight audible buzz
    	    //When the current is approximately zero, it is hard to resolve the direction, and therefore the compensation is ineffective.
    	    //However, no torque is generated when the current and voltage are close to zero, so no adverse performance except the buzz.
    	    if(_motor->Conv.Iu < -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1-_motor->FOC.deadtime_comp;}
    	    if(_motor->Conv.Iv < -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2-_motor->FOC.deadtime_comp;}
    	    if(_motor->Conv.Iw < -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3-_motor->FOC.deadtime_comp;}
    	    if(_motor->Conv.Iu > -0.030f){_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1+_motor->FOC.deadtime_comp;}
    	    if(_motor->Conv.Iv > -0.030f){_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2+_motor->FOC.deadtime_comp;}
    	    if(_motor->Conv.Iw > -0.030f){_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3+_motor->FOC.deadtime_comp;}

    	#endif
    	break;
    case PWM_SIN:

    	//Fallthrough FOR NOW
    case PWM_BOTTOM_CLAMP:

    	//Fallthrough FOR NOW
    case PWM_SIN_BOTTOM:
    	//Threshold for turning on sinusoidal modulation
    	    if(_motor->FOC.Voltage < _motor->FOC.V_3Q_mag_max){//Sinusoidal
    	    	   mid_value = _motor->FOC.PWMmid -
    	    	                0.5f * _motor->FOC.Vab_to_PWM * (top_value + bottom_value);

    	    	    ////////////////////////////////////////////////////////
    	    	    // Actually write the value to the timer registers
    	    	    _motor->mtimer->Instance->CCR1 =
    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0] + mid_value);
    	    	    _motor->mtimer->Instance->CCR2 =
    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1] + mid_value);
    	    	    _motor->mtimer->Instance->CCR3 =
    	    	    		(uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2] + mid_value);

//    			_motor->FOC.inverterVoltage[0] = _motor->FOC.inverterVoltage[0]+ mid_value;//0.5*_motor->FOC.Vmag_max;
//    			_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1]+ mid_value;//0.5*_motor->FOC.Vmag_max;
//    			_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[2]+ mid_value;//0.5*_motor->FOC.Vmag_max;
    	    }else{//Bottom Clamp
    			_motor->FOC.inverterVoltage[0] = _motor->FOC.inverterVoltage[0]-bottom_value;
    			_motor->FOC.inverterVoltage[1] = _motor->FOC.inverterVoltage[1]-bottom_value;
    			_motor->FOC.inverterVoltage[2] = _motor->FOC.inverterVoltage[2]-bottom_value;

    			//Write the timer registers
				_motor->mtimer->Instance->CCR1 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[0]);
				_motor->mtimer->Instance->CCR2 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[1]);
				_motor->mtimer->Instance->CCR3 = (uint16_t)(_motor->FOC.Vab_to_PWM * _motor->FOC.inverterVoltage[2]);
    	    }
    	#ifdef OVERMOD_DT_COMP_THRESHOLD
    	    //Concept here is that if we are close to the VBus max, we just do not turn the FET off.
    	    //Set CCRx to ARR, record how much was added, then next cycle, remove it from the count.
    	    //If the duty is still above the threshold, the CCR will still be set to ARR, until the duty request is sufficiently low...
    	static int carryU, carryV, carryW;

//    		_motor->mtimer->Instance->CCR1 = 	_motor->mtimer->Instance->CCR1 - carryU;
//    		_motor->mtimer->Instance->CCR2 = 	_motor->mtimer->Instance->CCR2 - carryV;
//    		_motor->mtimer->Instance->CCR3 = 	_motor->mtimer->Instance->CCR3 - carryW;
//    		carryU = 0;
//    		carryV = 0;
//    		carryW = 0;

//    		if(_motor->mtimer->Instance->CCR1>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
//    			carryU = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR1; //Save the amount we have overmodulated by
//    			_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->ARR;
//    		}
//    		if(_motor->mtimer->Instance->CCR2>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
//    			carryV = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR2; //Save the amount we have overmodulated by
//    			_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->ARR;
//    		}
//    		if(_motor->mtimer->Instance->CCR3>(_motor->mtimer->Instance->ARR-OVERMOD_DT_COMP_THRESHOLD)){
//    			carryW = _motor->mtimer->Instance->ARR-_motor->mtimer->Instance->CCR3; //Save the amount we have overmodulated by
//    			_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
//    		}
    	#endif
    	break;
    }//end of pwm type switch

#endif //End of #ifdef STEPPER_MOTOR
  }

// Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
 // which should only be set by the hardware comparators, in the case of a
 // shoot-through or other catastrophic event This function means that the
 // timer can be left running, ADCs sampling etc which enables a recovery, or
 // single PWM period break in which the backEMF can be measured directly
 // This function needs implementing and testing before any high current or
 // voltage is applied, otherwise... DeadFETs
 void MESCpwm_generateBreak(MESC_motor_typedef *_motor) {
#ifdef INV_ENABLE_M1
	  INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO<<16U; //Write the inverter enable pin low
#endif
#ifdef INV_ENABLE_M2
	  INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO<<16U; //Write the inverter enable pin low
#endif
   MESCpwm_phU_Break(_motor);
   MESCpwm_phV_Break(_motor );
   MESCpwm_phW_Break(_motor );
 }
 void MESCpwm_generateEnable(MESC_motor_typedef *_motor) {
#ifdef INV_ENABLE_M1
	  INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO;//Write the inverter enable pin high
#endif
#ifdef INV_ENABLE_M2
	  INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO;//Write the inverter enable pin high
#endif
   MESCpwm_phU_Enable(_motor);
   MESCpwm_phV_Enable(_motor);
   MESCpwm_phW_Enable(_motor);
 }

 void MESCpwm_generateBreakAll() {
#ifdef INV_ENABLE_M1
	  INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO<<16U; //Write the inverter enable pin low
#endif
#ifdef INV_ENABLE_M2
	  INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO<<16U; //Write the inverter enable pin low
#endif
   for(int i=0;i<NUM_MOTORS;i++){
   	MESCpwm_generateBreak(&mtr[i]);
   }
 }

 uint32_t tmpccmrx;  // Temporary buffer which is used to turn on/off phase PWMs

 // Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
 // mainly, but also used for measuring, software fault detection and recovery
 void MESCpwm_phU_Break(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR1;
   tmpccmrx &= ~TIM_CCMR1_OC1M;
   tmpccmrx &= ~TIM_CCMR1_CC1S;
   tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC1E;   // disable
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
 }
 // Basically un-break phase U, opposite of above...
 void MESCpwm_phU_Enable(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR1;
   tmpccmrx &= ~TIM_CCMR1_OC1M;
   tmpccmrx &= ~TIM_CCMR1_CC1S;
   tmpccmrx |= TIM_OCMODE_PWM1;
   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC1E;   // enable
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC1NE;  // enable
 }

 void MESCpwm_phV_Break(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR1;
   tmpccmrx &= ~TIM_CCMR1_OC2M;
   tmpccmrx &= ~TIM_CCMR1_CC2S;
   tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC2E;   // disable
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
 }

 void MESCpwm_phV_Enable(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR1;
   tmpccmrx &= ~TIM_CCMR1_OC2M;
   tmpccmrx &= ~TIM_CCMR1_CC2S;
   tmpccmrx |= TIM_OCMODE_PWM1 << 8;
   _motor->mtimer->Instance->CCMR1 = tmpccmrx;
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC2E;   // enable
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC2NE;  // enable
 }

 void MESCpwm_phW_Break(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR2;
   tmpccmrx &= ~TIM_CCMR2_OC3M;
   tmpccmrx &= ~TIM_CCMR2_CC3S;
   tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
   _motor->mtimer->Instance->CCMR2 = tmpccmrx;
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC3E;   // disable
   _motor->mtimer->Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
 }

 void MESCpwm_phW_Enable(MESC_motor_typedef *_motor) {
   tmpccmrx = _motor->mtimer->Instance->CCMR2;
   tmpccmrx &= ~TIM_CCMR2_OC3M;
   tmpccmrx &= ~TIM_CCMR2_CC3S;
   tmpccmrx |= TIM_OCMODE_PWM1;
   _motor->mtimer->Instance->CCMR2 = tmpccmrx;
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC3E;   // enable
   _motor->mtimer->Instance->CCER |= TIM_CCER_CC3NE;  // enable
 }

