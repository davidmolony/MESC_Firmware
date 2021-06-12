/*
* Copyright 2021 cod3b453
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

#include "MESCprofile.h"

#include "MESCbat.h"
#include "MESCspeed.h"
#include "MESCtemp.h"
#include "MESCui.h"

#include <assert.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ID_ENTRY(id) { id, #id }
#define ID_INDEX(id,n) { id(0x##n), #id "_" #n }

static char const * getProfileStatusName( ProfileStatus const ps )
{
    static struct
    {
        ProfileStatus val;
        char const *  str;
    } const status_id[] =
    {
        ID_ENTRY( PROFILE_STATUS_UNKNOWN                ),

        ID_ENTRY( PROFILE_STATUS_SUCCESS                ),

        ID_ENTRY( PROFILE_STATUS_INIT_SUCCESS_DEFAULT   ),
        ID_ENTRY( PROFILE_STATUS_INIT_SUCCESS_LOADED    ),
        ID_ENTRY( PROFILE_STATUS_INIT_FALLBACK_DEFAULT  ),

        ID_ENTRY( PROFILE_STATUS_ERROR_STORAGE_READ     ),
        ID_ENTRY( PROFILE_STATUS_ERROR_STORAGE_WRITE    ),

        ID_ENTRY( PROFILE_STATUS_ERROR_ENTRY_ALLOC      ),
        ID_ENTRY( PROFILE_STATUS_ERROR_ENTRY_READONLY   ),

        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 00        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 01        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 02        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 03        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 04        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 05        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 06        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 07        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 08        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 09        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0A        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0B        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0C        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0D        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0E        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 0F        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 10        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 11        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 12        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 13        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 14        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 15        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 16        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 17        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 18        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 19        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1A        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1B        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1C        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1D        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1E        ),
        ID_INDEX( PROFILE_STATUS_ERROR_ENTRY, 1F        ),

        ID_ENTRY( PROFILE_STATUS_ERROR_HEADER_SIGNATURE ),
        ID_ENTRY( PROFILE_STATUS_ERROR_HEADER_SIZE      ),
        ID_ENTRY( PROFILE_STATUS_ERROR_HEADER_VERSION   ),
        ID_ENTRY( PROFILE_STATUS_ERROR_HEADER_CHECKSUM  ),
        ID_ENTRY( PROFILE_STATUS_ERROR_HEADER_ZEROS     ),

        ID_ENTRY( PROFILE_STATUS_ERROR_IMAGE_LENGTH     ),
        ID_ENTRY( PROFILE_STATUS_ERROR_IMAGE_CHECKSUM   ),

        ID_ENTRY( PROFILE_STATUS_ERROR_ENTRY_SIGNATURE  ),
        ID_ENTRY( PROFILE_STATUS_ERROR_ENTRY_SIZE       ),
        ID_ENTRY( PROFILE_STATUS_ERROR_ENTRY_ZEROS      ),

        ID_ENTRY( PROFILE_STATUS_ERROR_NAME_LENGTH      ),
        ID_ENTRY( PROFILE_STATUS_ERROR_NAME             ),

        ID_ENTRY( PROFILE_STATUS_ERROR_DATA_SIGNATURE   ),
        ID_ENTRY( PROFILE_STATUS_ERROR_DATA_OFFSET      ),
        ID_ENTRY( PROFILE_STATUS_ERROR_DATA_LENGTH      ),

        ID_ENTRY( PROFILE_STATUS_COMMIT_SUCCESS         ),
        ID_ENTRY( PROFILE_STATUS_COMMIT_SUCCESS_NOOP    ),

        ID_ENTRY( PROFILE_STATUS_SUCCESS_ENTRY_ALLOC    ),
        ID_ENTRY( PROFILE_STATUS_SUCCESS_ENTRY_FREE     ),
        ID_ENTRY( PROFILE_STATUS_SUCCESS_ENTRY_NOOP     ),
        ID_ENTRY( PROFILE_STATUS_SUCCESS_ENTRY_UPDATE   ),
    };

    assert( status_id[ps].val == ps );

    return status_id[ps].str;
}

struct DemoEntry
{
    uint32_t x;
    uint32_t y;
    uint32_t z;
};

typedef struct DemoEntry DemoEntry;

static DemoEntry demo_entry;
static uint32_t demo_entry_size = sizeof(demo_entry);

static void * read_buffer = NULL;
static void const * write_buffer = NULL;

static uint8_t use_mem_not_fs = 1;
static uint8_t read_zero_on_error = 0;

static uint8_t * mem = NULL;

static uint32_t corrupt_offset = 0;
static uint32_t corrupt_length = 0;

static void apply_corruption()
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

#define revoke_corruption apply_corruption

static ProfileStatus read( void * data, uint32_t const address, uint32_t const length )
{
    read_buffer = data;

    FILE * f = NULL;

    if (use_mem_not_fs == 0)
    {
        f = fopen( "FLASH", "rb" );
    }

    if (f != NULL)
    {
        fseek( f, address, SEEK_SET );

        size_t const len = fread( data, 1, length, f );

        fclose( f );

        if (len != length)
        {
            return PROFILE_STATUS_ERROR_STORAGE_READ;
        }

        apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }

    if (use_mem_not_fs != 0)
    {
        memcpy( data, mem, length );

        apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }

    if (read_zero_on_error == 0)
    {
        return PROFILE_STATUS_ERROR_STORAGE_READ;
    }
    else
    {
        memset( data, 0, length );

        apply_corruption();

        return PROFILE_STATUS_SUCCESS;
    }
}

static ProfileStatus write( void const * data, uint32_t const address, uint32_t const length )
{
    write_buffer = data;

    FILE * f = NULL;

    if (use_mem_not_fs == 0)
    {
        f = fopen( "FLASH", "r+" );
    }

    if (f != NULL)
    {
        fseek( f, address, SEEK_SET );

        size_t const len = fwrite( data, 1, length, f );

        fclose( f );

        if (len != length)
        {
            return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        }

        return PROFILE_STATUS_COMMIT_SUCCESS;
    }

    if (use_mem_not_fs != 0)
    {
        memcpy( mem, data, length );

        return PROFILE_STATUS_COMMIT_SUCCESS;
    }

    return PROFILE_STATUS_ERROR_STORAGE_WRITE;
}

static void reset( void )
{
    if (use_mem_not_fs == 0)
    {
        remove( "FLASH" );
    }
}

static void corrupt( char const * name, uint32_t const offset, uint32_t const length )
{
    if (length == 0)
    {
        fprintf( stdout, "INFO: Corruption disabled\n" );
    }
    else
    {
        fprintf( stdout, "INFO: Corrupting %s offset %" PRIu32 " size %" PRIu32 "\n", name, offset, length );
    }

    corrupt_offset = offset;
    corrupt_length = length;
}

#define CORRUPTION_ENTRY(T,m) { offsetof(T,m), sizeof(((T *)NULL)->m), #T "." #m }

static struct
{
    uint32_t offset;
    uint32_t length;
    char const * name;
}   const corruptions[] =
{
    // ProfileHeader
    CORRUPTION_ENTRY( ProfileHeader, signature ),

    CORRUPTION_ENTRY( ProfileHeader, _zero_signature ),
    CORRUPTION_ENTRY( ProfileHeader, version_major ),
    CORRUPTION_ENTRY( ProfileHeader, version_minor ),
    CORRUPTION_ENTRY( ProfileHeader, size ),

    CORRUPTION_ENTRY( ProfileHeader, checksum ),

    // Skip ProfileHeader entry_map

    CORRUPTION_ENTRY( ProfileHeader, image_length ),
    CORRUPTION_ENTRY( ProfileHeader, image_checksum ),

    CORRUPTION_ENTRY( ProfileHeader, fingerprint.day ),
    CORRUPTION_ENTRY( ProfileHeader, fingerprint._zero ),

    CORRUPTION_ENTRY( ProfileHeader, fingerprint.githash[2] ),

    // End
    { 0, 0, NULL },
};

void bist_profile( void )
{
    ProfileStatus ret;
    ProfileStatus s;
    ProfileStatus h;
    ProfileStatus e;
    ProfileStatus o;

    fprintf( stdout, "Starting Profile BIST\n" );

    mem = calloc( 4096/*TODO*/, 1 );

    demo_entry.x = 1;
    demo_entry.y = 3;
    demo_entry.z = 2;

    read_zero_on_error = 1;
    use_mem_not_fs = 1;

    profile_configure_storage_io( read, write );

    fprintf( stdout, "INFO: Performing initial load & store\n" );

    ret = profile_init();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Load\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o) );

    ret = profile_commit();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Store (no update)\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o)  );

    profile_put_entry( "demo_entry", 0x12345678, &demo_entry, &demo_entry_size );

    ret = profile_commit();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Store (update)\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o) );

    fprintf( stdout, "INFO: Performing corruption sweep\n" );

    for ( uint32_t i = 0; i < (sizeof(corruptions) / sizeof(*corruptions)); ++i )
    {
        corrupt( corruptions[i].name, corruptions[i].offset, corruptions[i].length );

        ret = profile_init();

        profile_get_last( &s, &h, &e, & o );

        switch (ret)
        {
            case PROFILE_STATUS_INIT_SUCCESS_DEFAULT:
                fprintf( stdout, "INFO: PROFILE_STATUS_INIT_SUCCESS_DEFAULT\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
            case PROFILE_STATUS_INIT_SUCCESS_LOADED:
                fprintf( stdout, "INFO: PROFILE_STATUS_INIT_SUCCESS_LOADED\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
            case PROFILE_STATUS_INIT_FALLBACK_DEFAULT:
                fprintf( stdout, "INFO: PROFILE_STATUS_INIT_FALLBACK_DEFAULT\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
            default:
                fprintf( stdout, "ERROR:\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
        }

        revoke_corruption();
    }

    // TODO

    fprintf( stdout, "INFO: Scanning entries\n" );

    void const * buffer = NULL;
    uint32_t length = 0;

    if (profile_get_entry( "MESC_BAT", BAT_PROFILE_SIGNATURE, &buffer, &length ) == PROFILE_STATUS_SUCCESS)
    {
        BATProfile const * bp = (BATProfile *)buffer;
        (void)bp;
        fprintf( stdout, "INFO: Found Battery entry\n" );
    }
    else
    {
        static BATProfile bp;
        static uint32_t bp_size = sizeof(bp);
        (void)bp;
        fprintf( stdout, "INFO: Adding Battery entry\n" );

        ret = profile_put_entry( "MESC_BAT", BAT_PROFILE_SIGNATURE, &bp, &bp_size );
    }

    if (profile_get_entry( "MESC_SPEED", SPEED_PROFILE_SIGNATURE, &buffer, &length ) == PROFILE_STATUS_SUCCESS)
    {
        SPEEDProfile const * sp = (SPEEDProfile *)buffer;
        (void)sp;
        fprintf( stdout, "INFO: Found Speed entry\n" );
    }
    else
    {
        static SPEEDProfile sp;
        static uint32_t sp_size = sizeof(sp);
        (void)sp;
        fprintf( stdout, "INFO: Adding Speed entry\n" );

        ret = profile_put_entry( "MESC_SPEED", SPEED_PROFILE_SIGNATURE, &sp, &sp_size );
    }

    if (profile_get_entry( "MESC_TEMP", TEMP_PROFILE_SIGNATURE, &buffer, &length ) == PROFILE_STATUS_SUCCESS)
    {
        TEMPProfile const * tp = (TEMPProfile *)buffer;
        (void)tp;
        fprintf( stdout, "INFO: Found Temperature entry\n" );
    }
    else
    {
        static TEMPProfile tp;
        static uint32_t tp_size = sizeof(tp);
        (void)tp;
        fprintf( stdout, "INFO: Adding Temperature entry\n" );

        ret = profile_put_entry( "MESC_TEMP", TEMP_PROFILE_SIGNATURE, &tp, &tp_size );
    }

    if (profile_get_entry( "MESC_UI", UI_PROFILE_SIGNATURE, &buffer, &length ) == PROFILE_STATUS_SUCCESS)
    {
        UIProfile const * up = (UIProfile *)buffer;
        (void)up;
        fprintf( stdout, "INFO: Found UI entry\n" );
    }
    else
    {
        static UIProfile up;
        static uint32_t up_size = sizeof(up);
        (void)up;
        fprintf( stdout, "INFO: Adding UI entry\n" );

        ret = profile_put_entry( "MESC_UI", UI_PROFILE_SIGNATURE, &up, &up_size );
    }

    reset();

    free( mem );

    fprintf( stdout, "Finished Profile BIST\n" );
}
