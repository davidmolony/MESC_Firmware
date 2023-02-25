/*
 * tcp_serv.h
 *
 *  Created on: 24.07.2022
 *      Author: jensk
 */

#ifndef INC_TCP_SERV_H_
#define INC_TCP_SERV_H_

#include "stdint.h"
#include "TTerm/Core/include/TTerm.h"

void tcp_serv_init();
uint8_t CMD_ifconfig(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
int tcp_serv_bind(uint16_t port);

#endif /* INC_TCP_SERV_H_ */
