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
#define FOC_CONV_CHANNELS (3)
#define FOC_TRANSFORMED_CHANNELS (2)
#define FOC_NUM_ADC (3)

// fixme: I think this type of stuff is causing confusion later on especially in
// the code that strives for maintainability. What does an alias give you?
typedef uint16_t foc_angle_t;
typedef int32_t foc_field_angle_t;
typedef float current_amps_t;
typedef float voltage_t;

// fixme: why is this in a struct?
typedef struct {
  foc_angle_t ElecAngle;  // Current electrical angle
  uint16_t Sector;  // Current electrical sector - 6 sectors, as a consequence
                    // of Hall and 3 phase sinwave numbered 0-5
                    // [0,FOC_SECTORS_PER_REVOLUTION)
  foc_angle_t AnglePerSector;  // 6 sectors per eRevolution ((USHRT_MAX + 1) /
                               // FOC_SECTORS_PER_REVOLUTION)
  foc_angle_t
      RotorAngle;  // Rotor angle, either fetched from hall sensors as
                   // (sector*anglePerSector+tim3Count)/6 or from observer
  foc_angle_t AngleStep;  // At startup, step angle is zero, zero speed. This is
                          // the angle by which the inverter increments each PWM
                          // cycle under open loop

  uint16_t PWM[FOC_CONV_CHANNELS];  // 3 phase vector for the PWM generation, do
                                    // math on these before writing them to the
                                    // timer registers

  current_amps_t
      Iab[FOC_TRANSFORMED_CHANNELS];  // Float vector containing the Clark
                                      // transformed current in amps
  current_amps_t
      Idq[FOC_TRANSFORMED_CHANNELS];  // Float vector containing the Park
                                      // transformed current in amps
  voltage_t Vbus;  // Float vector containing the bus voltage in Volts
  voltage_t Vswitch[FOC_CONV_CHANNELS];  // Float vector containing the switch
                                         // node voltages (phase voltages)
  foc_field_angle_t
      FieldAngle;  // Signed number containing the angle between the electrical
                   // field and rotor, implemented as rotorAngle-elecAngle

} MESCfoc_s;

void foc_init(MESCfoc_s *foc);  // fixme: floating function prototype

// fixme: why is this in a struct? what advantages does this give?
typedef struct {
  uint32_t
      RawADC[FOC_NUM_ADC]
            [FOC_CONV_CHANNELS];  // ADC1 returns Ucurrent, DClink voltage and U
                                  // phase voltage ADC2 returns Vcurrent, V and
                                  // W phase voltages ADC3 returns Wcurrent,
  uint16_t ADCOffset[FOC_NUM_ADC];  // During detect phase, need to sense the
                                    // zero current offset
  float ConvertedADC[FOC_NUM_ADC]
                    [FOC_CONV_CHANNELS];  // We will fill this with currents in
                                          // A and voltages in Volts

} foc_measurement_t;

foc_measurement_t measurement_buffers;  // fixme: floating function prototype

typedef struct {
  uint16_t Delta;
  uint16_t Length;
} MESC_RCPWMin_t;

/* Function prototypes -----------------------------------------------*/

// fixme: inconsistent naming convention for functions. Choose one and stick to
// it across entire codebase. Choice needs to be documented. I recommend
// variable: lower_snake_case = 1;
// function: lowerCamelCase();
// constant: UPPER_CASE;

void fastLoop();
void V_I_Check();
void ADCConversion();  // Roll this into the V_I_Check? less branching, can
                       // probably reduce no.ops and needs doing every cycle
                       // anyway...
// convert currents from uint_16 ADC readings into float A and uint_16 voltages
// into float volts Since the observer needs the Clark transformed current, do
// the Clark transform now
void observerTick();  // Call every time to allow the observer, whatever it is,
                      // to update itself and find motor position
void MESCFOC();       // Park transform and current control (PI?)

void openLoopPIFF();  // Just keep the phase currents at the requested value
                      // without really thinking about things like
                      // synchronising, phase etc...

void writePWM();  // Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  // subtract lowest phase to always clamp one phase at 0V,
                  // write CCR registers

void GenerateBreak();  // Software break that does not stop the PWM timer but
                       // disables the outputs, sum of phU,V,W_Break();
int isMotorRunning();  // return motor state if state is one of the running
                       // states, if it's an idle, error or break state, disable
                       // all outputs and measure the phase voltages - if all
                       // the same, then it's stationary.
int GetHallState();    // Self explanatory...
void measureResistance();
void measureInductance();
void phU_Break();   // Turn all phase U FETs off, Tristate the ouput - For BLDC
                    // mode mainly, but also used for measuring
void phU_Enable();  // Basically un-break phase U, opposite of above...
void phV_Break();
void phV_Enable();
void phW_Break();
void phW_Enable();
