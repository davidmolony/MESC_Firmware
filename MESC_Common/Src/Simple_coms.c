/*
 * Simple_coms.c
 *
 *  Created on: Dec 14, 2022
 *      Author: HPEnvy
 */


#include "stm32fxxx_hal.h"
#include "Simple_coms.h"
#include "MESCfoc.h"

#ifdef MESC_UART_USB
#include "usbd_cdc_if.h"
#endif

#include <stdio.h>

void SimpleComsInit(UART_HandleTypeDef *uart, COMS_data_t *coms_instance){
	coms_instance->UART_handle = uart;
	coms_instance->time = 0;
	coms_instance->period = 100;
}

extern DMA_HandleTypeDef hdma_usart3_tx;

void SimpleComsProcess(COMS_data_t *coms_instance){
	if(!print_samples_now){
	if((HAL_GetTick() - coms_instance->time) > coms_instance->period){
		coms_instance->time = HAL_GetTick();
		coms_instance->len = sprintf(coms_instance->data,"Vbus: %.2f, eHz: %.2f, Id: %.2f, Iq: %.2f, P: %.2f, \r\n",
		//coms_instance->len = sprintf(coms_instance->data,"%.2f,%.2f,%.2f,%.2f, %.2f \r\n",
			(double)mtr[0].Conv.Vbus,
			(double)mtr[0].FOC.eHz,
			(double)mtr[0].FOC.Idq_smoothed.d,
			(double)mtr[0].FOC.Idq_smoothed.q,
			(double)(mtr[0].FOC.currentPower.q+mtr[0].FOC.currentPower.d));
#ifdef MESC_UART_USB
		CDC_Transmit_FS((uint8_t*)coms_instance->data, coms_instance->len);
#else
	HAL_UART_Transmit_DMA(coms_instance->UART_handle, coms_instance->data, coms_instance->len);
#endif
	}
	}else{
		printSamples(coms_instance->UART_handle, &hdma_usart3_tx);
	}

}
