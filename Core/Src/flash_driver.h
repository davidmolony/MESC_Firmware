/*
 * flash_driver.h
 *
 *  Created on: Dec 28, 2020
 *      Author: salavat.magazov
 */

#ifndef SRC_FLASH_DRIVER_H_
#define SRC_FLASH_DRIVER_H_

#include "stdint.h"

/**
 * Write data into flash.
 * Note: this process will take considerable amount of time during which processor will be unresponsive. Ensure that nothing critical is
 * happening.
 * @param data Pointer to data to write. All data must be in uint32_t format.
 * @param count Number of uint32_t elements to write.
 * @return Number of uint32_t elements written. If zero, then write has failed.
 */
uint32_t writeFlash(uint32_t const *const data, uint32_t const count);

/**
 * Read data from flash.
 * @param data Pointer to storage area where read data will be stored. User must ensure it is large enough to store all data.
 * @param count Number of uint32_t elements to read.
 * @return Number of non-empty elements read. If zero, then flash is empty.
 */
uint32_t readFlash(uint32_t *const data, uint32_t const count);

#endif /* SRC_FLASH_DRIVER_H_ */
