#ifndef OVERLAY_TYPES_H_
#define OVERLAY_TYPES_H_

#include "task.h"
#include <stdint.h>

typedef struct{
	TaskHandle_t task_handle;
	uint8_t output_type;
	uint16_t delay;
} overlay_handle;

#endif /* OVERLAY_TYPES_H_ */
