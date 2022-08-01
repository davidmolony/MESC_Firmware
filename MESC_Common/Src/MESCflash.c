/*
 * MESCflash.c
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 */

#include "MESCflash.h"

#include "stm32fxxx_hal.h"

static ProfileStatus readFlash( void        * const buffer, uint32_t const address, uint32_t const length )
{
    uint32_t const * src = (uint32_t const *)(getFlashBaseAddress() + address);
    uint32_t       * dst = (uint32_t       *)buffer;

    for ( uint32_t i = 0, j = 0; i < length; i = i + 4, j = j + 1 )
    {
         dst[j] = src[j];
    }

    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus writeBegin( void )
{
	HAL_StatusTypeDef const sts = HAL_FLASH_Unlock();

	if (sts != HAL_OK)
	{
		return PROFILE_STATUS_ERROR_STORAGE_WRITE;
	}

	uint32_t      const addr = getFlashBaseAddress();
    ProfileStatus const ret  = eraseFlash( addr, PROFILE_MAX_SIZE );

    return ret;
}

static ProfileStatus writeFlash( void const * const buffer, uint32_t const address, uint32_t const length )
{
    uint32_t         addr  = getFlashBaseAddress() + address;
    uint32_t const * src   = (uint32_t const *)buffer;
    ProfileStatus    ret   = PROFILE_STATUS_SUCCESS;

	for (
		uint32_t i = 0, j = 0;
		i < length;
#if defined FLASH_TYPEPROGRAM_WORD
		i = i + 4, j = j + 1
#elif defined FLASH_TYPEPROGRAM_DOUBLEWORD
		i = i + 8, j = j + 2
#else
#error Unhandled FLASH_TYPEPROGRAM_
#endif
		)
	{
		HAL_StatusTypeDef sts = HAL_FLASH_Unlock();

		if (sts != HAL_OK)
		{
			return PROFILE_STATUS_ERROR_STORAGE_WRITE;
		}
#if defined FLASH_TYPEPROGRAM_WORD
		sts = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (addr + i), src[j] );
#elif defined FLASH_TYPEPROGRAM_DOUBLEWORD
		sts = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, (addr + i), src[j] );
#else
#error Unhandled FLASH_TYPEPROGRAM_
#endif
		switch (sts)
		{
			case HAL_OK:
				break;
			case HAL_ERROR:
				ret = PROFILE_STATUS_ERROR_STORAGE_WRITE;
				break;
			default:
				ret = PROFILE_STATUS_UNKNOWN;
				break;
		}

		if (sts != HAL_OK)
		{
			break;
		}
	}

    return ret;
}

static ProfileStatus writeEnd( void )
{
	HAL_StatusTypeDef const sts = HAL_FLASH_Lock();

	if (sts != HAL_OK)
	{
		return PROFILE_STATUS_ERROR_STORAGE_WRITE;
	}

	return PROFILE_STATUS_SUCCESS;
}

void flash_register_profile_io( void )
{
    profile_configure_storage_io( readFlash, writeFlash, writeBegin, writeEnd );
}
