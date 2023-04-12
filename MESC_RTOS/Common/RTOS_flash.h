/*
 **
 ******************************************************************************
 * @file           : RTOS_flash.h
 * @brief          : Flash IO functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Jens Kerrinnes.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 ******************************************************************************
 *In addition to the usual 3 BSD clauses, it is explicitly noted that you
 *do NOT have the right to take sections of this code for other projects
 *without attribution and credit to the source. Specifically, if you copy into
 *copyleft licenced code without attribution and retention of the permissive BSD
 *3 clause licence, you grant a perpetual licence to do the same regarding turning sections of your code
 *permissive, and lose any rights to use of this code previously granted or assumed.
 *
 *This code is intended to remain permissively licensed wherever it goes,
 *maintaining the freedom to distribute compiled binaries WITHOUT a requirement to supply source.
 *
 *This is to ensure this code can at any point be used commercially, on products that may require
 *such restriction to meet regulatory requirements, or to avoid damage to hardware, or to ensure
 *warranties can reasonably be honoured.
 ******************************************************************************/

#ifndef RTOS_FLASH_H_
#define RTOS_FLASH_H_

#include <stdint.h>

uint32_t RTOS_flash_clear(void * address, uint32_t len);
uint32_t RTOS_flash_start_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_end_write(void * address, void * data, uint32_t len);
uint32_t RTOS_flash_sector_address( uint32_t const index );
uint32_t RTOS_flash_sector_index(uint32_t const address);
uint32_t RTOS_flash_base_address(void);
uint32_t RTOS_flash_base_size(void);


#endif /* RTOS_FLASH_H_ */
