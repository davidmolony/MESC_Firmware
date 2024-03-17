/*
 **
 ******************************************************************************
 * @file           : MESCmotor_state.c
 * @brief          : Code for motor state machine
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

 * MESCmotor_state.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/
#include "MESCmotor_state.h"

motor_state_e MotorState;
motor_sensor_mode_e MotorSensorMode;
motor_error_type_e MotorError;
motor_direction_e MotorDirection;
motor_control_type_e MotorControlType;
test_mode_e TestMode;

