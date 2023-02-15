/*
 * MESCflash.h
 *
 *  Created on: 16 May 2021
 *      Author: cod3b453
 */

#ifndef INC_MESCFLASH_H_
#define INC_MESCFLASH_H_

#include "MESCprofile.h"

/*
TODO: Define in MESChw_setup.c
BEGIN
*/
uint32_t getFlashBaseAddress( void );
uint32_t getFlashBaseSize( void );
ProfileStatus eraseFlash( uint32_t const address, uint32_t const length );
/*
END
*/

void flash_register_profile_io( void );

#endif /* INC_MESCFLASH_H_ */
