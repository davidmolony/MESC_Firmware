/*
 * flash_wrapper.c
 *
 *  Created on: Dec 28, 2020
 *      Author: salavat.magazov
 */

/* declare all variables to be stored here. */

#include "flash_wrapper.h"
#include "MESCfoc.h"
#include "MESChw_setup.h"

// extern MESCfoc_s foc_vars;
// extern motor_s motor;

/* actual data to write
 * foc_vars.hall_table[6][4]
 * motor.Lphase
 * motor.Rphase
 */

#define STORAGE_SIZE 26
uint32_t data_array[STORAGE_SIZE];

uint32_t getSize()
{
    uint32_t storage_slots = 0;
    uint32_t *p_flash_runner = getFlashAddress();
    while (*p_flash_runner++ != EMPTY_SLOT)
    {
        storage_slots++;
    }
    return (storage_slots);
}

/*
 * When editing readData and writeData make absolutely certain that you read and write in the same sequence.
 */
uint32_t readData()
{
    uint32_t storage_slots = readFlash(data_array, STORAGE_SIZE);
    uint32_t *p_data = data_array;
    if (storage_slots)
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j; j < 4; j++)
            {
                foc_vars.hall_table[i][j] = *p_data;
                p_data++;
            }
        }
    }
    motor.Rphase = (hardware_vars_t)(*p_data);
    p_data++;
    motor.Lphase = (hardware_vars_t)(*p_data);
    //    p_data++;	//this last increment is not necessary, but if more variables are added, then uncomment it and follow the same
    //    pattern.
    return (storage_slots);
}

/*
 * When editing readData and writeData make absolutely certain that you read and write in the same sequence.
 */
uint32_t writeData()
{
    uint32_t storage_slots = 0;
    uint32_t *p_data = data_array;
    for (int i = 0; i < 6; i++)
    {
        for (int j; j < 4; j++)
        {
            *p_data = foc_vars.hall_table[i][j];
            p_data++;
        }
    }
    *p_data = (uint32_t)motor.Rphase;
    p_data++;
    *p_data = (uint32_t)motor.Lphase;
    //    p_data++;	//this last increment is not necessary, but if more variables are added, then uncomment it and follow the same
    //    pattern.
    storage_slots = writeFlash(data_array, STORAGE_SIZE);
    return (storage_slots);
}

void eraseData() {}
