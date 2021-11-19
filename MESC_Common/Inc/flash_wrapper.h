/*
 * flash_wrapper.h
 *
 *  Created on: Dec 28, 2020
 *      Author: salavat.magazov
 */

#ifndef SRC_FLASH_WRAPPER_H_
#define SRC_FLASH_WRAPPER_H_

#include <stdint.h>

typedef enum
{
    EMPTY = 0,
    VALID,
    UNKNOWN
} flash_status_t;

/**
 * Get status of the flash memory.
 * @return EMPTY if flash has been untouched after erase. VALID if the exact expected number of values are written. UNKNOWN if some values
 * are written, but not exact match to expected size.
 */
flash_status_t getStatus();

/**
 * Get size of stored data.
 * @return Zero if empty, otherwise number of elements stored.
 */
uint32_t getSize();

/**
 * Read all data from flash.
 * @return Zero if failed, otherwise number of elements read.
 */
uint32_t readData();

/**
 * Write all data into flash.
 * Note: this process will take considerable amount of time during which processor will be unresponsive. Ensure that nothing critical is
 * happening.
 * @return Zero if failed, otherwise number of elements written.
 */
uint32_t writeData();

/**
 * Erase entire page.
 */
void eraseData();

#endif /* SRC_FLASH_WRAPPER_H_ */
