/*
 **
 ******************************************************************************
 * @file           : MESCpwm.h
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

#ifndef INC_MESCPWM_H_
#define INC_MESCPWM_H_

#include "MESCfoc.h"

void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor); 	//Put this into the PWM interrupt,
void MESCpwm_Write(MESC_motor_typedef *_motor);  		// Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  	  	  	  	  	  	  	  	  		// subtract lowest phase to always clamp one phase at 0V or
                  	  	  	  	  	  	  	  	    	// SVPWM
                  	  	  	  	  	  	  	  	  	  	// write CCR registers

void MESCpwm_generateBreak(MESC_motor_typedef *_motor); // Software break that does not stop the PWM timer but
                       	   	   	   	   	   	   	   	    // disables the outputs, sum of phU,V,W_Break();
void MESCpwm_generateEnable(MESC_motor_typedef *_motor);// Opposite of generateBreak
void MESCpwm_generateBreakAll();						//Disables all drives

void MESCpwm_phU_Break(MESC_motor_typedef *_motor);   	// Turn all phase U FETs off, Tristate the ouput - For BLDC
                    									// mode mainly, but also used for measuring
void MESCpwm_phU_Enable(MESC_motor_typedef *_motor);  	// Basically un-break phase U, opposite of above...
void MESCpwm_phV_Break(MESC_motor_typedef *_motor);
void MESCpwm_phV_Enable(MESC_motor_typedef *_motor);
void MESCpwm_phW_Break(MESC_motor_typedef *_motor);
void MESCpwm_phW_Enable(MESC_motor_typedef *_motor);

#endif /* INC_MESCPWM_H_ */
