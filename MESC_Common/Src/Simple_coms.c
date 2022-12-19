/*
 * Simple_coms.c
 *
 *  Created on: Dec 14, 2022
 *      Author: HPEnvy
 */


#include "stm32fxxx_hal.h"
#include "Simple_coms.h"
#include "MESCfoc.h"

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
			measurement_buffers.ConvertedADC[0][1],
			motor1.FOC.eHz,
			motor1.FOC.Idq_smoothed.d,
			motor1.FOC.Idq_smoothed.q,
			(motor1.FOC.currentPower.q+motor1.FOC.currentPower.d));
#ifdef MESC_UART_USB
		CDC_Transmit_FS(coms_instance->data, coms_instance->len);
#else
	HAL_UART_Transmit_DMA(coms_instance->UART_handle, coms_instance->data, coms_instance->len);
#endif
	}
	}else{
		printSamples(coms_instance->UART_handle, &hdma_usart3_tx);
	}

}
