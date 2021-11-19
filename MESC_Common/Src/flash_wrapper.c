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

#include "MESCflash.h"

extern ProfileStatus readFlash( void        * const buffer, uint32_t const address, uint32_t const length ); // DEPRECATED - REMOVE & STATIC
extern ProfileStatus writeFlash( void const * const buffer, uint32_t const address, uint32_t const length ); // DEPRECATED - REMOVE & STATIC

#define EMPTY_SLOT UINT32_MAX

/* actual data to write
 * foc_vars.hall_table[6][4] - 24 values
 * motor.Lphase - 1 value
 * motor.Rphase - 1 value
 */

#define STORAGE_SIZE 26
uint32_t data_array[STORAGE_SIZE];

flash_status_t getStatus() {
  switch (getSize()) {
    case 0:
      return (EMPTY);
      break;
    case STORAGE_SIZE:
      return (VALID);
      break;
    default:
      return (UNKNOWN);
  }
}

uint32_t getSize() {
  uint32_t storage_slots = 0;
  uint32_t *p_flash_runner = (uint32_t *)getFlashBaseAddress();
  while (*p_flash_runner++ != EMPTY_SLOT) {
    storage_slots++;
  }
  return (storage_slots);
}

/*
 * When editing readData and writeData make absolutely certain that you read and
 * write in the same sequence.
 */
uint32_t readData() {
  readFlash( data_array, 0, (STORAGE_SIZE * sizeof(uint32_t)) );
  uint32_t *p_data = data_array;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      foc_vars.hall_table[i][j] = *p_data;
      p_data++;
    }
  }
  /* weird casting is to ensure that it's the pointers that get cast, not actual
   * values. */
  motor.Rphase = *(hardware_vars_t *)(p_data);
  p_data++;
  motor.Lphase = *(hardware_vars_t *)(p_data);
  //    p_data++;	//this last increment is not necessary, but if more
  //    variables are added, then uncomment it and follow the same pattern.
  return STORAGE_SIZE;
}

/*
 * When editing readData and writeData make absolutely certain that you read and
 * write in the same sequence.
 */
uint32_t writeData() {
  uint32_t *p_data = data_array;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      *p_data = foc_vars.hall_table[i][j];
      p_data++;
    }
  }
  /* weird casting is to ensure that it's the pointers that get cast, not actual
   * values. */
  *p_data = *(uint32_t *)(&motor.Rphase);
  p_data++;
  *p_data = *(uint32_t *)(&motor.Lphase);
  //    p_data++;	//this last increment is not necessary, but if more
  //    variables are added, then uncomment it and follow the same pattern.
  writeFlash( data_array, 0, (STORAGE_SIZE * sizeof(uint32_t)) );
  return STORAGE_SIZE;
}

void eraseData() {
	eraseFlash( 0, (STORAGE_SIZE * sizeof(uint32_t)) );
}
