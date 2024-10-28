/*
 **
 ******************************************************************************
 * @file           : MESCApp.h
 * @brief          : Application specific code, such as for managing vehicle inputs for brake, stands...
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 David Molony.
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
 * MESCfoc.h
 *
 *  Created on: Oct 2024
 *      Author: David Molony
 */

#ifndef INC_MESCAPP_H_
#define INC_MESCAPP_H_

#include "MESCfoc.h"

enum VEHICLE_STATE
{
	VEHICLE_IDLE = 0,
	VEHICLE_PARKED = 1,
	VEHICLE_DRIVE = 2,
	VEHICLE_x = 3,
	VEHICLE_y = 4,
	VEHICLE_ERROR = 5
};
enum VEHICLE_DIRECTION
{
	VEHICLE_FORWARD = 0,
	VEHICLE_REVERSE = 1,
};

void No_app(MESC_motor_typedef *_motor);
void Vehicle_app(MESC_motor_typedef *_motor);

#endif /* INC_MESCAPP_H_ */
