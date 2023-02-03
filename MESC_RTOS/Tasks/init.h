/*
 * init.h
 *
 *  Created on: Jul 9, 2022
 *      Author: jensk
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

#include "task_cli.h"
#include "TTerm/Core/include/TTerm.h"

void init_system(void);
void init_system_running(void);

extern port_str main_uart;
extern port_str main_usb;

extern TERMINAL_HANDLE null_handle;

#endif /* INC_INIT_H_ */
