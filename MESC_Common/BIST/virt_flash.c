/*
* Copyright 2021-2022 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "virt_flash.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void       * read_buffer  = NULL;
static void const * write_buffer = NULL;

static bool use_mem_not_fs     = true;
static bool read_zero_on_error = false;

static uint8_t * mem = NULL;

void virt_flash_configure( bool const use_mem_not_fs_, bool const read_zero_on_error_ )
{
    use_mem_not_fs     = use_mem_not_fs_;
    read_zero_on_error = read_zero_on_error_;
}

static uint32_t corrupt_offset = 0;
static uint32_t corrupt_length = 0;

void virt_flash_apply_corruption( void )
{
    if (corrupt_length > 0)
    {
        uint32_t buffer = 0;

        assert( corrupt_length <= sizeof(buffer) );

        memcpy( &buffer, &((uint8_t *)read_buffer)[corrupt_offset], corrupt_length );

        buffer ^= UINT32_MAX;

        memcpy( &((uint8_t *)read_buffer)[corrupt_offset], &buffer, corrupt_length );
    }
}

void virt_flash_corrupt( char const * name, uint32_t const offset, uint32_t const length )
{
    if (length == 0)
    {
        fprintf( stdout, "INFO: Corruption disabled\n" );
    }
    else
    {
        fprintf( stdout, "INFO: Corrupting %s at offset %" PRIu32 " size %" PRIu32 "\n", name, offset, length );
    }

    corrupt_offset = offset;
    corrupt_length = length;
}

ProfileStatus virt_flash_read( void * data, uint32_t const address, uint32_t const length )
{
    read_buffer = data;

    FILE * f = NULL;

    if (use_mem_not_fs == false)
    {
        f = fopen( "FLASH", "rb" );
    }

    if (f != NULL)
    {
        int const ret = fseek( f, address, SEEK_SET );

        if (ret != 0)
        {
            return PROFILE_STATUS_ERROR_DATA_OFFSET;
        }

        size_t const len = fread( data, 1, length, f );

        fclose( f );

        if (len != length)
        {
            return PROFILE_STATUS_ERROR_STORAGE_READ;
        }

        virt_flash_apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }

    if (use_mem_not_fs)
    {
        memcpy( data, &mem[address], length );

        virt_flash_apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }

    if (read_zero_on_error == false)
    {
        return PROFILE_STATUS_ERROR_STORAGE_READ;
    }
    else
    {
        memset( data, 0, length );

        virt_flash_apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }
}

ProfileStatus virt_flash_write( void const * data, uint32_t const address, uint32_t const length )
{
    write_buffer = data;

    FILE * f = NULL;

    if (use_mem_not_fs == false)
    {
        f = fopen( "FLASH", "r+" );
    }

    if (f != NULL)
    {
        int const ret = fseek( f, address, SEEK_SET );

        if (ret != 0)
        {
            return PROFILE_STATUS_ERROR_DATA_OFFSET;
        }

        size_t const len = fwrite( data, 1, length, f );

        fclose( f );

        if (len != length)
        {
            return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        }

        return PROFILE_STATUS_COMMIT_SUCCESS;
    }

    if (use_mem_not_fs)
    {
        memcpy( &mem[address], data, length );

        return PROFILE_STATUS_COMMIT_SUCCESS;
    }

    return PROFILE_STATUS_ERROR_STORAGE_WRITE;
}

ProfileStatus virt_flash_begin( void )
{
    return PROFILE_STATUS_SUCCESS;
}

ProfileStatus virt_flash_end( void )
{
    return PROFILE_STATUS_SUCCESS;
}

void virt_flash_reset( void )
{
    if (use_mem_not_fs == false)
    {
        remove( "FLASH" );
    }
}

void virt_flash_init( void )
{
    mem = calloc( PROFILE_MAX_SIZE, 1 );
}

void virt_flash_free( void )
{
    if (mem)
    {
        free( mem );
        mem = NULL;
    }
}
