/*
 * MESClrobs.h
 *
 *  Created on: Sep 24, 2024
 *      Author: jkerrinnes
 */

#ifndef INC_MESCLROBS_H_
#define INC_MESCLROBS_H_

#include "MESCfoc.h"

void MESClrobs_Run(MESC_motor_typedef *_motor);
void MESClrobs_Collect(MESC_motor_typedef *_motor);
void MESClrobs_Init(MESC_motor_typedef *_motor);

#endif /* INC_MESCLROBS_H_ */
