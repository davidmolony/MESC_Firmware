/*
 * MESCflash.c
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 */

#include "MESCflash.h"

#include "stm32fxxx_hal.h"

/*static*/ ProfileStatus readFlash( void        * const buffer, uint32_t const address, uint32_t const length ) // TODO static
{
    uint32_t const * src = (uint32_t const *)(getFlashBaseAddress() + address);
    uint32_t       * dst = (uint32_t       *)buffer;

    for ( uint32_t i = 0; i < length; i = i + 4 )
    {
         dst[i] = src[i];
    }

    return PROFILE_STATUS_SUCCESS;
}

/*static*/ ProfileStatus writeFlash( void const * const buffer, uint32_t const address, uint32_t const length ) // TODO static
{
    uint32_t         addr  = getFlashBaseAddress() + address;
    uint32_t const * src   = (uint32_t const *)buffer;
    uint32_t const * dst   = (uint32_t const *)address;

    HAL_FLASH_Unlock();

    ProfileStatus ret = eraseFlash( address, length );

    if (ret == PROFILE_STATUS_SUCCESS)
    {
        for ( uint32_t i = 0; i < length; i = i + 4 )
        {
            if (dst[i] == src[i])
            {
                continue;
            }

            HAL_StatusTypeDef const sts = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (addr + i), src[i] );

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
    }

    HAL_FLASH_Lock();

    return ret;
}

void flash_register_profile_io( void )
{
    profile_configure_storage_io( readFlash, writeFlash );
}
