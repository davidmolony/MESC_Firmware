/*
 * flash_driver.c
 *
 *  Created on: Dec 28, 2020
 *      Author: salavat.magazov
 */

#include "flash_driver.h"
#include "stm32f3xx_hal.h"

uint32_t *const p_flash = (uint32_t *)(0x0801F800);

uint32_t writeFlash(uint32_t const *const p_data, uint32_t const count)
{
    uint32_t number_written = 0;
    uint32_t const *p_data_runner = p_data;
    HAL_FLASH_Unlock();
    /* if intended destination is not empty... */
    if (*p_flash != EMPTY_SLOT)
    {
        /* ...erase entire page before proceeding. */
        FLASH_EraseInitTypeDef page_erase;
        page_erase.TypeErase = FLASH_TYPEERASE_PAGES;
        page_erase.PageAddress = (uint32_t)p_flash;
        page_erase.NbPages = 1;
        uint32_t result = 0;
        HAL_FLASHEx_Erase(&page_erase, &result);
        if (result != EMPTY_SLOT)
        {
            return (number_written);
        }
    }
    /* write all p_data in 32-bit words into flash. */
    uint32_t const *p_flash_runner = p_flash;
    for (int i = 0; i < count; i++, p_flash_runner++, p_data_runner++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)p_flash_runner, *p_data_runner) == HAL_OK)
        {
            number_written++;
        }
        else
        {
            break;
        }
    }
    HAL_FLASH_Lock();
    return (number_written);
}

uint32_t readFlash(uint32_t *const p_data, uint32_t const count)
{
    uint32_t number_read = 0;
    uint32_t *p_data_runner = p_data;
    uint32_t *p_flash_runner = p_flash;
    for (int i = 0; i < count; i++, p_data_runner++, p_data_runner++)
    {
        *p_data_runner = *p_flash_runner;
        if (*p_flash_runner != EMPTY_SLOT)
        {
            number_read++;
        }
    }
    return (number_read);
}

uint32_t *const getFlashAddress()
{
    return (p_flash);
}
