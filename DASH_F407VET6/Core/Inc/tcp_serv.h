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
#include "lwip.h"
#include "socket.h"
#include "Tasks/task_cli.h"

#define SA struct sockaddr

void tcp_serv_init();
int tcp_serv_bind(uint16_t port);
uint8_t CMD_ifconfig(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
int tcp_serv_bind(uint16_t port);

void ext_printnet(port_str * port, const char* format, ...);

#endif /* INC_TCP_SERV_H_ */
