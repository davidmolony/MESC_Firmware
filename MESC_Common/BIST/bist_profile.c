/*
* Copyright 2021-2023 cod3b453
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
#include "MESCmotor.h"
#include "MESCspeed.h"
#include "MESCui.h"

#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "virt_flash.h"

extern char   const gen_profile[];
extern size_t const gen_profile_size;

static uint8_t parseHexNybble( char const c )
{
    if  (
            ('0' <= c)
        &&  (c <= '9')
        )
    {
        return (uint8_t)(c - '0');
    }

    if  (
            ('A' <= c)
        &&  (c <= 'F')
        )
    {
        return (uint8_t)(c - 'A' + 10);
    }

    return 0;
}

static ProfileStatus rom_flash_read(  void       * data, uint32_t const address, uint32_t const length )
{
    if  (
            ( address           <  gen_profile_size)
        &&  ((address + length) <= gen_profile_size)
        )
    {
        for (uint32_t i = 0, offset = address; i < length; i++, offset++ )
        {
            ((uint8_t *)data)[i] = (parseHexNybble(gen_profile[(2 * offset) + 0]) << BITS_PER_NYBBLE)
                                 |  parseHexNybble(gen_profile[(2 * offset) + 1]);
        }
        return PROFILE_STATUS_SUCCESS;
    }

    return PROFILE_STATUS_ERROR_STORAGE_READ;
}

static ProfileStatus rom_flash_write( void const * data, uint32_t const address, uint32_t const length )
{
    return PROFILE_STATUS_ERROR_STORAGE_WRITE;
    (void)data;
    (void)address;
    (void)length;
}

#define virt_flash_revoke_corruption virt_flash_apply_corruption


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
        ID_ENTRY( PROFILE_STATUS_ERROR_STORAGE_BEGIN    ),
        ID_ENTRY( PROFILE_STATUS_ERROR_STORAGE_END      ),

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

        ID_ENTRY( PROFILE_STATUS_FAILURE_SCAN           ),
    };
    static_assert( (sizeof(status_id) / sizeof(*status_id)) == (PROFILE_STATUS_FAILURE_SCAN + 1), "Malformed ProfileStatus list" );
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

#define CORRUPTION_ENTRY(T,m,s,h,e,o) { (uint32_t)offsetof(T,m), (uint32_t)sizeof(((T *)NULL)->m), #T "." #m, s,h,e,o }

static struct
{
    uint32_t        offset;
    uint32_t        length;
    char const *    name;
    struct
    {
    ProfileStatus   storage;
    ProfileStatus   header;
    ProfileStatus   entry;
    ProfileStatus   other;
    }   expres;
}   const corruptions[] =
{
    // ProfileHeader
    CORRUPTION_ENTRY( ProfileHeader, signature             , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_SIGNATURE,   // The header signature should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    CORRUPTION_ENTRY( ProfileHeader, _zero_signature       , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_ZEROS,       // The header zeros should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    CORRUPTION_ENTRY( ProfileHeader, version_major         , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_VERSION,     // The header version should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile
    CORRUPTION_ENTRY( ProfileHeader, version_minor         , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_VERSION,     // The header version should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile
    CORRUPTION_ENTRY( ProfileHeader, size                  , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_SIZE,        // The header size should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    CORRUPTION_ENTRY( ProfileHeader, checksum              , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_CHECKSUM,    // The header checksum should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    // Skip ProfileHeader entry_map

    CORRUPTION_ENTRY( ProfileHeader, image_length          , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_CHECKSUM,    // The header checksum should fail (there is no failure condition for image length)
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile
    CORRUPTION_ENTRY( ProfileHeader, image_checksum        , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_CHECKSUM,    // The header checksum should fail (there is no failure condition for image checksum)
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    CORRUPTION_ENTRY( ProfileHeader, fingerprint.day       , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_CHECKSUM,    // The header checksum should fail (there is no failure condition for fingerprint)
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile
    CORRUPTION_ENTRY( ProfileHeader, fingerprint._zero     , { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_ZEROS   ,    // The header zeros should fail
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile
    CORRUPTION_ENTRY( ProfileHeader, fingerprint.githash[2], { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_ERROR_HEADER_CHECKSUM,    // The header checksum should fail (there is no failure condition for fingerprint)
                                                               PROFILE_STATUS_UNKNOWN,                  // There should be no entry errors (this is the header)
                                                               PROFILE_STATUS_INIT_SUCCESS_DEFAULT } ), // The failure should revert to the default profile

    // End
    { 0, 0, NULL,                                            { PROFILE_STATUS_SUCCESS,                  // There should be no storage errors
                                                               PROFILE_STATUS_SUCCESS,                  // The header should be valid
                                                               PROFILE_STATUS_SUCCESS,                  // The entries should be valid
                                                               PROFILE_STATUS_UNKNOWN              } }, // No other information required
};

static void profile_check_last(
    ProfileStatus const storage,
    ProfileStatus const header,
    ProfileStatus const entry,
    ProfileStatus const other
) {
    ProfileStatus s;
    ProfileStatus h;
    ProfileStatus e;
    ProfileStatus o;

    profile_get_last( &s, &h, &e, & o );

    if  (
            ( s == storage )
        &&  ( h == header  )
        &&  ( e == entry   )
        &&  ( o == other   )
        )
    {
        fprintf( stdout, "PASS\n" );
    }
    else
    {
        fprintf( stdout, "FAIL\n" );
    }
}

void bist_profile( void )
{
    ProfileStatus ret;
    ProfileStatus s;
    ProfileStatus h;
    ProfileStatus e;
    ProfileStatus o;

    fprintf( stdout, "Starting Profile BIST\n" );

    virt_flash_init();

    demo_entry.x = 1;
    demo_entry.y = 3;
    demo_entry.z = 2;

    virt_flash_configure( true, true );

    profile_configure_storage_io( virt_flash_read, virt_flash_write, virt_flash_begin, virt_flash_end );

    fprintf( stdout, "INFO: Performing initial load & store\n" );

    ret = profile_init();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Load %d %s\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        ret, getProfileStatusName(ret),
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o) );

    profile_check_last(
        PROFILE_STATUS_SUCCESS,
        PROFILE_STATUS_ERROR_HEADER_SIGNATURE,
        PROFILE_STATUS_UNKNOWN,
        PROFILE_STATUS_INIT_SUCCESS_DEFAULT );

    ret = profile_commit();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Store (no update) %d %s\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        ret, getProfileStatusName(ret),
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o)  );

    profile_check_last(
        PROFILE_STATUS_COMMIT_SUCCESS_NOOP,
        PROFILE_STATUS_UNKNOWN,
        PROFILE_STATUS_UNKNOWN,
        PROFILE_STATUS_UNKNOWN );

    profile_put_entry( "demo_entry", 0x12345678, &demo_entry, &demo_entry_size );

    ret = profile_commit();
    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Store (update) %d %s\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        ret, getProfileStatusName(ret),
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o) );

    profile_check_last(
        PROFILE_STATUS_COMMIT_SUCCESS,
        PROFILE_STATUS_UNKNOWN,
        PROFILE_STATUS_UNKNOWN,
        PROFILE_STATUS_UNKNOWN );

    fprintf( stdout, "INFO: Performing corruption sweep\n" );

    for ( uint32_t i = 0; i < (sizeof(corruptions) / sizeof(*corruptions)); ++i )
    {
        virt_flash_corrupt( corruptions[i].name, corruptions[i].offset, corruptions[i].length );

        ret = profile_init();

        profile_get_last( &s, &h, &e, & o );

        switch (ret)
        {
            case PROFILE_STATUS_INIT_SUCCESS_DEFAULT:
            case PROFILE_STATUS_INIT_SUCCESS_LOADED:
            case PROFILE_STATUS_INIT_FALLBACK_DEFAULT:
                fprintf( stdout, "INFO: %d %s\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    ret, getProfileStatusName(ret),
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
            default:
                fprintf( stdout, "ERROR: %d %s\n"
                                 "    S:%d %s\n"
                                 "    H:%d %s\n"
                                 "    E:%d %s\n"
                                 "    O:%d %s\n",
                    ret, getProfileStatusName(ret),
                    s, getProfileStatusName(s),
                    h, getProfileStatusName(h),
                    e, getProfileStatusName(e),
                    o, getProfileStatusName(o) );
                break;
        }

        profile_check_last(
            corruptions[i].expres.storage,
            corruptions[i].expres.header,
            corruptions[i].expres.entry,
            corruptions[i].expres.other );

        virt_flash_revoke_corruption();
    }

    fprintf( stdout, "INFO: Scanning entries\n" );

    void * buffer = NULL;
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

        profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

        profile_check_last(
            PROFILE_STATUS_UNKNOWN,                 // No storage operation should be performed
            PROFILE_STATUS_UNKNOWN,                 // No header operation should be performed
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC,     // A new entry should be allocated
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC );   // A new entry should be allocated
    }

    if (profile_get_entry( "MESC_MOTOR", MOTOR_PROFILE_SIGNATURE, &buffer, &length ) == PROFILE_STATUS_SUCCESS)
    {
        MOTORProfile const * sp = (MOTORProfile *)buffer;
        (void)sp;
        fprintf( stdout, "INFO: Found Motor entry\n" );
    }
    else
    {
        static MOTORProfile sp;
        static uint32_t sp_size = sizeof(sp);
        (void)sp;
        fprintf( stdout, "INFO: Adding Motor entry\n" );

        ret = profile_put_entry( "MESC_MOTOR", MOTOR_PROFILE_SIGNATURE, &sp, &sp_size );

        profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

        profile_check_last(
            PROFILE_STATUS_UNKNOWN,                 // No storage operation should be performed
            PROFILE_STATUS_UNKNOWN,                 // No header operation should be performed
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC,     // A new entry should be allocated
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC );   // A new entry should be allocated
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

        profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

        profile_check_last(
            PROFILE_STATUS_UNKNOWN,                 // No storage operation should be performed
            PROFILE_STATUS_UNKNOWN,                 // No header operation should be performed
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC,     // A new entry should be allocated
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC );   // A new entry should be allocated
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

        profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

        profile_check_last(
            PROFILE_STATUS_UNKNOWN,                 // No storage operation should be performed
            PROFILE_STATUS_UNKNOWN,                 // No header operation should be performed
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC,     // A new entry should be allocated
            PROFILE_STATUS_SUCCESS_ENTRY_ALLOC );   // A new entry should be allocated
    }

    fprintf( stdout, "INFO: Commit image\n" );

    ret = profile_commit();

    profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

    profile_check_last(
        PROFILE_STATUS_COMMIT_SUCCESS,  // Storage should be successful
        PROFILE_STATUS_UNKNOWN,         // No header operation should be performed
        PROFILE_STATUS_UNKNOWN,         // No entry operation should be performed
        PROFILE_STATUS_UNKNOWN );       // No other operation should be performed

    fprintf( stdout, "INFO: Read-back image\n" );

    ret = profile_init();

    profile_get_last( &s, &h, &e, & o );
        fprintf( stdout, "INFO: %d %s\n"
                         "    S:%d %s\n"
                         "    H:%d %s\n"
                         "    E:%d %s\n"
                         "    O:%d %s\n",
            ret, getProfileStatusName(ret),
            s, getProfileStatusName(s),
            h, getProfileStatusName(h),
            e, getProfileStatusName(e),
            o, getProfileStatusName(o) );

    profile_check_last(
        PROFILE_STATUS_SUCCESS,     // Storage should be successful
        PROFILE_STATUS_SUCCESS,     // Header should be valid
        PROFILE_STATUS_SUCCESS,     // Entries should be valid
        PROFILE_STATUS_UNKNOWN );   // No other operation should be performed

    virt_flash_reset();

    virt_flash_free();

    fprintf( stdout, "INFO: Profile image\n" );

    profile_configure_storage_io( rom_flash_read, rom_flash_write, virt_flash_begin, virt_flash_end );

    ret = profile_init();

    profile_get_last( &s, &h, &e, & o );
    fprintf( stdout, "INFO: Init\n"
                     "    S:%d %s\n"
                     "    H:%d %s\n"
                     "    E:%d %s\n"
                     "    O:%d %s\n",
        s, getProfileStatusName(s),
        h, getProfileStatusName(h),
        e, getProfileStatusName(e),
        o, getProfileStatusName(o) );

    profile_check_last(
        PROFILE_STATUS_SUCCESS,     // Storage should be successful
        PROFILE_STATUS_SUCCESS,     // Header should be valid
        PROFILE_STATUS_SUCCESS,     // Entries should be valid
        PROFILE_STATUS_UNKNOWN );   // No other operation should be performed

    if (ret == PROFILE_STATUS_INIT_SUCCESS_LOADED)
    {
        fprintf( stdout, "INFO: Scanning image\n" );

        uint32_t index = 0;
        ProfileEntry const * entry = NULL;

        do
        {
            ret = profile_read_entry( &index, &entry );

            profile_get_last( &s, &h, &e, & o );
            fprintf( stdout, "INFO: %d %s\n"
                             "    S:%d %s\n"
                             "    H:%d %s\n"
                             "    E:%d %s\n"
                             "    O:%d %s\n",
                ret, getProfileStatusName(ret),
                s, getProfileStatusName(s),
                h, getProfileStatusName(h),
                e, getProfileStatusName(e),
                o, getProfileStatusName(o) );

            profile_check_last(
                PROFILE_STATUS_UNKNOWN,     // No storage operation should be performed
                PROFILE_STATUS_UNKNOWN,     // No header operation should be performed
                PROFILE_STATUS_UNKNOWN,     // No entry operation should be performed
                PROFILE_STATUS_UNKNOWN );   // No other operation should be performed

            if  (
                    (ret == PROFILE_STATUS_SUCCESS)
                &&  (entry != NULL)
                )
            {
                fprintf( stdout, "ENTRY[%" PRIu32 "]\n"
                                 "              Name %s\n"
                                 "    Data Signature %" PRIX32 "\n",
                                 index,
                                 entry->name,
                                 entry->data_signature );
            }

            ++index;
        }
        while (ret == PROFILE_STATUS_SUCCESS);

        fprintf( stdout, "INFO: Scanning finished\n" );

        profile_check_last(
            PROFILE_STATUS_UNKNOWN,     // No storage operation should be performed
            PROFILE_STATUS_UNKNOWN,     // No header operation should be performed
            PROFILE_STATUS_UNKNOWN,     // No entry operation should be performed
            PROFILE_STATUS_UNKNOWN );   // No other operation should be performed
    }
    else
    {
        fprintf( stdout, "INFO: No entries found\n" );
    }

    fprintf( stdout, "Finished Profile BIST\n" );
}
