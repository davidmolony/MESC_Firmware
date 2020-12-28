/*
 * flash_wrapper.h
 *
 *  Created on: Dec 28, 2020
 *      Author: salavat.magazov
 */

#ifndef SRC_FLASH_WRAPPER_H_
#define SRC_FLASH_WRAPPER_H_

#include "flash_driver.h"

/**
 * Check if flash is empty.
 * @return Zero if empty, otherwise number of elements stored.
 */
uint32_t isEmpty();

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
 * Erase flash.
 * Note: this process will take considerable amount of time during which processor will be unresponsive. Ensure that nothing critical is
 * happening.
 */
void eraseData();

#endif /* SRC_FLASH_WRAPPER_H_ */
