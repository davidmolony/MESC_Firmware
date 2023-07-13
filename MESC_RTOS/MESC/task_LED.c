/*
 * task_LED.c
 *
 *  Created on: May 3, 2023
 *      Author: HPEnvy
 */

#include "task_LED.h"

#include "stm32fxxx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

extern uint32_t MESC_errors;

static void task_led( void * pvParameters )
{
	GPIO_TypeDef * led = (GPIO_TypeDef *)pvParameters;

	for( ;; )
	{
		if (!MESC_errors)
		{
			taskYIELD();
			continue;
		}
#if 0
		led->BSRR = SLOWLEDIO;
		vTaskDelay(2000);
		led->BSRR = SLOWLEDIO<<16U;
		vTaskDelay(2000);
#endif
		TickType_t xLastWakeTime = xTaskGetTickCount ();

		for (uint32_t i = 1; (i <= MESC_errors); )
		{
			uint32_t delay = (MESC_errors & i) ? 500 : 100;

			led->BSRR = SLOWLEDIO;
//			vTaskDelay(delay);
			vTaskDelayUntil( &xLastWakeTime, delay );
			led->BSRR = SLOWLEDIO<<16U;

			i <<= 1;

			delay = (i <= MESC_errors) ? 500 : 3000;
//			vTaskDelay(delay);
			vTaskDelayUntil( &xLastWakeTime, delay );
		}
	}
}

void task_led_init( void )
{
	TaskHandle_t xHandle = NULL;
	xTaskCreate( task_led, "task_led", 128, SLOWLED, tskIDLE_PRIORITY, &xHandle );
	configASSERT( xHandle );
}
