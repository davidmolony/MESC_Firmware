/*
 **
 ******************************************************************************
 * @file           : MESCmotor_state.h
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

 * MESCmotor_state.h
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

#ifndef MESC_MOTOR_STATE_H
#define MESC_MOTOR_STATE_H

#include "stm32fxxx_hal.h"

typedef enum {
  // fixme: why do you assign enum values explicitly? Do they have meaning?
  MOTOR_STATE_IDLE = 0,
  // All PWM should be off state, nothing happening. Motor may be spinning
  // freely
  MOTOR_STATE_DETECTING = 1,
  // PWM not generating output, but still running to trigger the ADC/check for
  // hall sensors. Returned values from ADC used to detect if the motor is
  // spinning, how fast, what speed...
  MOTOR_STATE_ALIGN = 2,
  // Hold one phase at current
  MOTOR_STATE_MEASURING = 3,
  // Measuring resistance and inducance of phase
  MOTOR_STATE_OPEN_LOOP_STARTUP = 4,
  // Starting up in sensorless mode
  MOTOR_STATE_OPEN_LOOP_TRANSITION = 5,
  // Checking motor is running synchronously and phaselocking
  MOTOR_STATE_TRACKING = 6,
  // Monitor the phase voltages while the PWM is disabled
  // Perform Clark and park
  // Run sensorless/hall observer to keep the angle
  // Load the PID integral values with the current Vd and Vq

  MOTOR_STATE_RUN = 7,
  /*Hall sensors are changing state fast enough for the timer to detect them.
   From this, a continuous sinusoidal FOC algorithm can be running Always
   align the current 90degrees to the field(hall sensor) Magnitude of current
   proportional to throttle demand (+/- can either be +/- current, or invert
   90degree angle, depending on inverter algorithm
   */
  MOTOR_STATE_GET_KV = 8,
  /*
   */
  MOTOR_STATE_TEST = 9,
  MOTOR_STATE_ERROR = 10,
  /*Enter this state when the overcurrent or overvoltage trips, or illegal
   hall state or sensorless observer fault occurs All PWM signals should be
   disabled, the timer may be in fault mode with all outputs disabled, or it
   may be required to implement the bit writes to turn off the outputs

   */
  MOTOR_STATE_RECOVERING = 11,
  /*
   After a fault state, might want to implement a routine to restart the
   system on the fly - detect if motor is running, detect speed, phase,
   re-enable PWM
   */

} motor_state_e;

extern motor_state_e MotorState;

typedef enum {
  MOTOR_SENSOR_MODE_SENSORLESS,
  MOTOR_SENSOR_MODE_HALL,
  MOTOR_SENSOR_MODE_OPENLOOP,
} motor_sensor_mode_e;

extern motor_sensor_mode_e MotorSensorMode;

typedef enum {
  MOTOR_ERROR_NONE,
  MOTOR_ERROR_HALL0,
  MOTOR_ERROR_HALL7,
  MOTOR_ERROR_OVER_LIMIT,
  MOTOR_ERROR_OVER_LIMIT_CURR1,
  MOTOR_ERROR_OVER_LIMIT_CURR2,
  MOTOR_ERROR_OVER_LIMIT_CURR3,
  MOTOR_ERROR_OVER_LIMIT_VBUS,
  MOTOR_ERROR_OVER_LIMIT_TEMP,
  MOTOR_ERROR_OTHER,
} motor_error_type_e;

extern motor_error_type_e MotorError;

typedef enum {
  MOTOR_DIRECTION_CLOCKWISE,
  MOTOR_DIRECTION_COUNTERCLOCKWISE
} motor_direction_e;

extern motor_direction_e MotorDirection;

typedef enum {
  MOTOR_CONTROL_TYPE_BLDC,
  MOTOR_CONTROL_TYPE_FOC
} motor_control_type_e;

extern motor_control_type_e MotorControlType;

/* Function prototypes -----------------------------------------------*/

void MESC_Init();

#endif
