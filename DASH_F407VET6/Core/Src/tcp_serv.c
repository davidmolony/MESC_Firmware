#include "tcp_serv.h"
#include "lwip.h"
#include "socket.h"
#include "dns.h"
#include "stdarg.h"
#include "FreeRTOS.h"

#include "Tasks/task_cli.h"

static void task_server(void * argument);
static void task_client(void * argument);

typedef struct{
	int socket;
}tcp_serv_str;


#define DEBUG_TCP 0

void tcp_serv_init(){

	tcp_serv_str * server = pvPortMalloc(sizeof(tcp_serv_str));
	if(server==NULL) return;

	server->socket = tcp_serv_bind(7);
	if(server->socket < 0) return;

	xTaskCreate(task_server, "tskServ", 256, server, osPriorityNormal, NULL);
}

void ext_printnet(port_str * port, const char* format, ...) {
	va_list arg;
	va_start (arg, format);

	if(format != NULL){
		int len;
		char send_buffer[128];
		len = vsnprintf(send_buffer, 128, format, arg);
		if(len > sizeof(send_buffer)){
			len = sizeof(send_buffer);
		}

		if(len > 0) {
			write(*(int*)port->hw, send_buffer, len);
		}
	}else{
		char *s = va_arg(arg, char*);
		int len = va_arg(arg, int);
		write(*(int*)port->hw, s, len);
	}

	va_end (arg);

}



void task_server(void * argument){
	tcp_serv_str * server = argument;
	while(1){
		struct sockaddr_in cli;
		int connfd;

		socklen_t len = sizeof(cli);
		connfd = accept(server->socket, (SA*)&cli, &len);

		if (connfd < 0) {
#if DEBUG_TCP
			printf("server accept failed...\n");
#endif
		}else{
#if DEBUG_TCP
			printf("server accept the client...\n");
#endif
			// Function for chatting between client and server
			int* dat = pvPortMalloc(sizeof(int));
			*dat = connfd;
			xTaskCreate(task_client, "tskClient", 1024, dat, osPriorityNormal, NULL);
		}
	}
}


// Driver function
int tcp_serv_bind(uint16_t port){
    struct sockaddr_in servaddr;
    int sockfd;

    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
#if DEBUG_TCP
        printf("socket creation failed...\n");
#endif
        return -1;
    }
    else{
#if DEBUG_TCP
        printf("Socket successfully created..\n");
#endif
    }
    bzero(&servaddr, sizeof(servaddr));

    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port);
    // Binding newly created socket to given IP and verification
    if (( bind(sockfd, (SA*)&servaddr, sizeof(servaddr)) ) != 0)
    {
#if DEBUG_TCP
        printf("socket bind failed...\n");
#endif
        return -1;
    } else {
#if DEBUG_TCP
        printf("Socket successfully binded..\n");
#endif
    }

    // Now server is ready to listen and verification
    if ((listen(sockfd, 5)) != 0) {
#if DEBUG_TCP
        printf("Listen failed...\n");
#endif
        return -1;
    } else {
#if DEBUG_TCP
        printf("Server listening..\n");
#endif
    	return sockfd;
    }
}


#define MAX 128

void task_client(void * argument){
	int connfd = *((int*)argument);
    char buff[MAX];
    int n;
    // infinite loop for chat
#if DEBUG_TCP
    printf("Start Task...\r\n");
#endif
    port_str * port = pvPortMalloc(sizeof(port_str));
    port->hw = argument;

    TERMINAL_HANDLE * handle = TERM_createNewHandle(ext_printnet, port, pdTRUE, &TERM_defaultList,NULL,"net");



	port->tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(port->tx_semaphore);

	port->term_block = xSemaphoreCreateBinary();
	xSemaphoreGive(port->term_block);



    for (;;) {

        n = recv(connfd, buff, sizeof(buff), MSG_DONTWAIT);
        // read the message from client and copy it in buffer
        // print buffer which contains the client contents
        if(n > 0){
        	xSemaphoreTake(port->term_block, portMAX_DELAY);
        	TERM_processBuffer((uint8_t*)&buff,n,handle);
        	xSemaphoreGive(port->term_block);
        }
        if(n == 0){
        	break;
        }


        vTaskDelay(5);
    }

    vSemaphoreDelete(port->tx_semaphore);
    vSemaphoreDelete(port->term_block);
    vPortFree(port);

    TERM_destroyHandle(handle);
#if DEBUG_TCP
    printf("Delete Task...\r\n");
#endif
    close(connfd);
    vPortFree(argument);
    vTaskDelete(NULL);

}


void help_ifconfig(TERMINAL_HANDLE * handle){
    ttprintf("\teth0 [ip]\r\n");
    ttprintf("\tnetmask [netmask]\r\n");
    ttprintf("\tgw [netmask]\r\n");
    ttprintf("\tdhcp\r\n");
}

uint8_t CMD_ifconfig(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    uint8_t currArg = 0;
    ip4_addr_t ip = *netif_ip4_addr(netif_default);
    ip4_addr_t netmask = *netif_ip4_netmask(netif_default);
    ip4_addr_t gw = *netif_ip4_gw(netif_default);
    for(;currArg<argCount; currArg++){
        if(strcmp(args[currArg], "-?") == 0){
            help_ifconfig(handle);
            return TERM_CMD_EXIT_SUCCESS;
        }
        if(strcmp(args[currArg], "eth0") == 0){
            if(++currArg<argCount){
                if(ipaddr_aton(args[currArg], &ip)){
                	dhcp_stop(netif_default);
                	netif_set_addr(netif_default, &ip, &netmask, &gw);
                }else{
                    help_ifconfig(handle);
                    return TERM_CMD_EXIT_SUCCESS;
                }
            }else{
               help_ifconfig(handle);
               return TERM_CMD_EXIT_SUCCESS;
            }
        }
        if(strcmp(args[currArg], "netmask") == 0){
            if(++currArg<argCount){
            	if(ipaddr_aton(args[currArg], &netmask)){
            		dhcp_stop(netif_default);
            		netif_set_addr(netif_default, &ip, &netmask, &gw);
                }else{
                    help_ifconfig(handle);
                    return TERM_CMD_EXIT_SUCCESS;
                }
            }else{
                help_ifconfig(handle);
                return TERM_CMD_EXIT_SUCCESS;
            }
        }
        if(strcmp(args[currArg], "gw") == 0){
            if(++currArg<argCount){
            	if(ipaddr_aton(args[currArg], &gw)){
            		dhcp_stop(netif_default);
            		netif_set_addr(netif_default, &ip, &netmask, &gw);
                }else{
                    help_ifconfig(handle);
                    return TERM_CMD_EXIT_SUCCESS;
                }
            }else{
                help_ifconfig(handle);
                return TERM_CMD_EXIT_SUCCESS;
            }
        }
        if(strcmp(args[currArg], "dhcp") == 0){
        	dhcp_start(netif_default);
        	ttprintf("Activating DHCP...\r\n");
        	vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    char buff[16];
    ttprintf("eth0\r\x1b[%dCLink type: Ethernet  HWaddr %02x:%02x:%02x:%02x:%02x:%02x\r\n", 7, netif_default->hwaddr[0], netif_default->hwaddr[1], netif_default->hwaddr[2], netif_default->hwaddr[3], netif_default->hwaddr[4], netif_default->hwaddr[5]);
    ipaddr_ntoa_r(&ip, buff, sizeof(buff));
    ttprintf("\x1b[%dCinet addr: %s ", 7, buff);
    ipaddr_ntoa_r(&netmask, buff, sizeof(buff));
    ttprintf("Mask: %s\r\n", buff);
    ipaddr_ntoa_r(&gw, buff, sizeof(buff));
    ttprintf("\x1b[%dCgateway: %s ", 7, buff);
    //FreeRTOS_inet_ntoa(ulDNSServerAddress, buff);
    //ttprintf("dns: %s\r\n\n", buff);

    return TERM_CMD_EXIT_SUCCESS;
}
