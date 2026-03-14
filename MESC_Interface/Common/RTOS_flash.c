/*
 **
 ******************************************************************************
 * @file           : RTOS_flash.c
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

#include "RTOS_flash.h"
#include "main.h"
#include "MESChw_setup.h"

#ifdef STM32F4
static uint32_t const flash_sector_map[] = {
    // 4 x  16k
    FLASH_BASE + (0 * (16 << 10)),
    FLASH_BASE + (1 * (16 << 10)),
    FLASH_BASE + (2 * (16 << 10)),
    FLASH_BASE + (3 * (16 << 10)),
    // 1 x  64k
    FLASH_BASE + (4 * (16 << 10)),
    // 7 x 128k
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (0 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (1 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (2 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (3 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (4 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (5 * (128 << 10)),
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (6 * (128 << 10)),
    // END
    FLASH_BASE + (4 * (16 << 10)) + (64 << 10) + (7 * (128 << 10)),
};



uint32_t RTOS_flash_sector_address(uint32_t const index){
    return flash_sector_map[index];
}

uint32_t RTOS_flash_sector_index(uint32_t const address){
    for (uint32_t i = 0; i < ((sizeof(flash_sector_map) / sizeof(*flash_sector_map)) - 1); i++)
    {
        if ((flash_sector_map[i] <= address) && (address < flash_sector_map[i + 1]))
        {
            return i;
        }
    }

    // error
    return UINT32_MAX;
}

uint32_t RTOS_flash_start_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Unlock();
	FLASH_WaitForLastOperation(500);
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	return written;
}

uint32_t RTOS_flash_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	return written;
}

uint32_t RTOS_flash_end_write(void * address, void * data, uint32_t len){
	uint8_t * buffer = data;
	uint32_t written=0;
	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address++;
		len--;
	}
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Lock();
	FLASH_WaitForLastOperation(500);
	return written;
}

#endif

#ifdef STM32L4

uint32_t RTOS_flash_sector_address(uint32_t const index){
	return FLASH_BASE + (index * 2048);  //All Pages are 2k in size
}

uint32_t RTOS_flash_sector_index(uint32_t const address){
	uint32_t ret = address - FLASH_BASE;
	ret = ret / 2048;

	if(ret > 255){
		return UINT32_MAX; //Error
	}else{
		return ret;
	}
}

uint32_t RTOS_flash_start_write(void * address, void * data, uint32_t len){
	uint64_t * buffer = data;
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Unlock();
	FLASH_WaitForLastOperation(500);
	uint32_t written=0;

	if(len<8) return 0;
	if(len%8 != 0) return 0; //Sorry only 8 byte alligned writes possible

	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address+=8;
		len-=8;
	}
	return written;
}

uint32_t RTOS_flash_write(void * address, void * data, uint32_t len){
	uint64_t * buffer = data;
	uint32_t written=0;

	if(len<8) return 0;
	if(len%8 != 0) return 0; //Sorry only 8 byte alligned writes possible

	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)address,*buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address+=8;
		len-=8;
	}
	return written;
}

uint32_t RTOS_flash_end_write(void * address, void * data, uint32_t len){
	uint64_t * buffer = data;
	uint32_t written=0;

	if(len<8) return 0;
	if(len%8 != 0) return 0; //Sorry only 8 byte alligned writes possible

	while(len){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)address, *buffer)==HAL_OK){
			written++;
		}
		buffer++;
		address+=8;
		len-=8;
	}
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Lock();
	FLASH_WaitForLastOperation(500);
	return written;
}

#endif

uint32_t RTOS_flash_base_address( void )
{
/*
The base address is FLASH_BASE = 0x08000000 but this is shared with program
memory and so a suitable offset should be used
*/
    return RTOS_flash_sector_address( FLASH_STORAGE_PAGE );
}

uint32_t RTOS_flash_base_size( void )
{
    return RTOS_flash_sector_address( FLASH_STORAGE_PAGE + 1) - RTOS_flash_sector_address( FLASH_STORAGE_PAGE );
}

uint32_t RTOS_flash_erase( uint32_t const address, uint32_t const length )
{
    // Disallow zero length (could ignore)
    if (length == 0)
    {
        return 0;
    }

    uint32_t const saddr = address;
    uint32_t const eaddr = saddr + length - 1;

    uint32_t const ssector = RTOS_flash_sector_index( saddr );
    uint32_t const esector = RTOS_flash_sector_index( eaddr );

    // Limit erasure to a single sector
    if (ssector != esector)
    {
        return 0;
    }

    FLASH_EraseInitTypeDef sector_erase;
#ifdef STM32F4
    sector_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    sector_erase.Banks     = FLASH_BANK_1; // (ignored)
    sector_erase.Sector    = ssector;
    sector_erase.NbSectors = (esector - ssector + 1);
#endif
#ifdef STM32L4
    sector_erase.TypeErase = FLASH_TYPEERASE_PAGES;
    sector_erase.Banks = FLASH_BANK_1; //Only one bank
    sector_erase.Page    = ssector;
    sector_erase.NbPages = (esector - ssector + 1);
#endif
    uint32_t bad_sector = 0;

    HAL_StatusTypeDef const sts = HAL_FLASHEx_Erase( &sector_erase, &bad_sector );

    return sts;

}


uint32_t RTOS_flash_clear(void * address, uint32_t len){
	FLASH_WaitForLastOperation(500);
	HAL_FLASH_Unlock();
	FLASH_WaitForLastOperation(500);
	RTOS_flash_erase((uint32_t)address, len);
	HAL_FLASH_Lock();
	FLASH_WaitForLastOperation(500);
	return len;
}



