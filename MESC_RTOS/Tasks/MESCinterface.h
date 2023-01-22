/*
 * init.h
 *
 *  Created on: Jul 9, 2022
 *      Author: jensk
 */

#ifndef INC_MESC_INTERFACE_H_
#define INC_MESC_INTERFACE_H_

#include "task_cli.h"

void MESCinterface_init(void);

uint8_t CMD_detectHFI(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_getkv(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_detectHFI(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);


#endif /* INC_MESC_INTERFACE_H_ */
