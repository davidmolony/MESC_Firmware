/*
 * MESCflash.c
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 */

#include "MESCflash.h"

#include "stm32f3xx_hal.h"

static uint32_t flash_address = 0x08000000; // NVM

static uint32_t flash_page_address( uint32_t const page )
{
    return (flash_address + (page * FLASH_PAGE_SIZE));
}

static uint32_t flash_page( uint32_t const address )
{
    return (address / FLASH_PAGE_SIZE);
}

static uint32_t flash_page_length( uint32_t const address, uint32_t const length )
{
    uint32_t const f = flash_page( address );
    uint32_t const l = flash_page( address + length + (FLASH_PAGE_SIZE - 1) ) + 1;

    return (l - f);
}

static ProfileStatus flash_erase_pages( uint32_t const page, uint32_t const count )
{
    uint32_t const address = flash_page_address( page );

    FLASH_EraseInitTypeDef fe;

    fe.TypeErase   = FLASH_TYPEERASE_PAGES;
    fe.PageAddress = address;
    fe.NbPages     = count;

    uint32_t res = 0;

    HAL_StatusTypeDef const sts = HAL_FLASHEx_Erase( &fe, &res );

    switch (sts)
    {
        case HAL_OK:
            return PROFILE_STATUS_SUCCESS;
        case HAL_ERROR:
            return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        default:
            break;
    }

    return PROFILE_STATUS_UNKNOWN;
}

static ProfileStatus flash_read( void        * const buffer, uint32_t const address, uint32_t const length )
{
    uint32_t const * src = (uint32_t const *)(flash_page_address( 0 ) + address);
    uint32_t       * dst = (uint32_t       *)buffer;

    for ( uint32_t i = 0; i < length; i = i + 4 )
    {
         dst[i] = src[i];
    }

    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus flash_write( void const * const buffer, uint32_t const address, uint32_t const length )
{
    uint32_t       page    = flash_page( address );
    uint32_t       addr    = flash_page_address( 0 ) + address;
    uint32_t const pages   = flash_page_length( address, length );
    uint32_t const * src   = (uint32_t const *)buffer;
    uint32_t const * dst   = (uint32_t const *)address;

    HAL_FLASH_Unlock();

    ProfileStatus ret = flash_erase_pages( page, pages );

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
    profile_configure_storage_io( flash_read, flash_write );
}
