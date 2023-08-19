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

#include "MESCcli.h"
#include "MESCfnv.h"

#include "bit_op.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

static MESCFingerprint const fingerprint = MESC_FINGERPRINT;

static struct
{
    ProfileHeader   header;
    ProfileEntry    entry[PROFILE_HEADER_ENTRIES];
}   profile_stub;

static struct
{
    void           * buffer;
    uint32_t const * length;
}   profile_entry[PROFILE_HEADER_ENTRIES];

static bool profile_modified = false;

static ProfileStatus profile_status_storage = PROFILE_STATUS_UNKNOWN;
static ProfileStatus profile_status_header  = PROFILE_STATUS_UNKNOWN;
static ProfileStatus profile_status_entry   = PROFILE_STATUS_UNKNOWN;
static ProfileStatus profile_status_other   = PROFILE_STATUS_UNKNOWN;

void profile_get_last(
    ProfileStatus * const storage,
    ProfileStatus * const header,
    ProfileStatus * const entry,
    ProfileStatus * const other )
{
    if (storage != NULL)
    {
        *storage = profile_status_storage;
    }

    if (header != NULL)
    {
        *header = profile_status_header;
    }

    if (entry != NULL)
    {
        *entry = profile_status_entry;
    }

    if (other != NULL)
    {
        *other = profile_status_other;
    }
}

static void profile_init_default( void )
{
    memset( &profile_stub, 0, sizeof(profile_stub) );

    profile_stub.header.signature      = PROFILE_SIGNATURE;

    profile_stub.header.size           = PROFILE_HEADER_SIZE;
    profile_stub.header.version_major  = PROFILE_VERSION_MAJOR;
    profile_stub.header.version_minor  = PROFILE_VERSION_MINOR;

    profile_stub.header.checksum       = PROFILE_SIGNATURE;

    profile_stub.header.image_checksum = fnv1a_init();

    profile_stub.header.checksum       = fnv1a_data( &profile_stub.header, PROFILE_HEADER_SIZE );

    uint32_t blk = 0;
    uint32_t fld = 0;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        profile_stub.header.entry_map[blk] |= (PROFILE_ENTRY_FREE << fld);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }
    }

    memcpy( &profile_stub.header.fingerprint, &fingerprint, sizeof(fingerprint) );

    profile_status_other = PROFILE_STATUS_INIT_SUCCESS_DEFAULT;
}

static ProfileStatus profile_header_validate( ProfileHeader * const header )
{
    if (header->signature != PROFILE_SIGNATURE)
    {
        return PROFILE_STATUS_ERROR_HEADER_SIGNATURE;
    }

    if (header->size != PROFILE_HEADER_SIZE)
    {
        return PROFILE_STATUS_ERROR_HEADER_SIZE;
    }

    if  (
            (header->version_major != PROFILE_VERSION_MAJOR)
        ||  (header->version_minor != PROFILE_VERSION_MINOR)
        )
    {
        return PROFILE_STATUS_ERROR_HEADER_VERSION;
    }

    if  (
            (header->_zero_signature != 0)
        ||  (header->fingerprint._zero != 0)
        )
    {
        return PROFILE_STATUS_ERROR_HEADER_ZEROS;
    }

    uint32_t hash = fnv1a_init();
    uint32_t const offset = offsetof(ProfileHeader,checksum);
    uint32_t const sig    = PROFILE_SIGNATURE;
    uint32_t const end    = offsetof(ProfileHeader,entry_map);

    hash = fnv1a_process_data( hash, header, offset );
    hash = fnv1a_process_data( hash, &sig, sizeof(sig) );
    hash = fnv1a_process_data( hash, &header->entry_map[0], (PROFILE_HEADER_SIZE - end) );

    if (hash != header->checksum)
    {
        return PROFILE_STATUS_ERROR_HEADER_CHECKSUM;
    }

    uint32_t const image_length = header->image_length;

    if (image_length < (PROFILE_HEADER_ENTRIES * sizeof(ProfileEntry)))
    {
        return PROFILE_STATUS_ERROR_IMAGE_LENGTH;
    }
/*
NOTE

Defer image checksum check to caller, as entries and payload may not have been
loaded yet.
*/
    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus profile_entry_validate( ProfileEntry * const entry )
{
    if (entry->signature != PROFILE_ENTRY_SIGNATURE)
    {
        return PROFILE_STATUS_ERROR_ENTRY_SIGNATURE;
    }

    if (entry->size != PROFILE_ENTRY_SIZE)
    {
        return PROFILE_STATUS_ERROR_ENTRY_SIZE;
    }

    if (entry->name_length > PROFILE_ENTRY_MAX_NAME_LENGTH)
    {
        return PROFILE_STATUS_ERROR_NAME_LENGTH;
    }

    if (entry->data_signature == 0)
    {
        return PROFILE_STATUS_ERROR_DATA_SIGNATURE;
    }

    if (entry->data_length == 0)
    {
        return PROFILE_STATUS_ERROR_DATA_LENGTH;
    }

    if  (
            ((entry->data_offset & 3) != 0) // align 4
        ||  (entry->data_offset < PROFILE_ENTRY_MIN_OFFSET) // Must follow header
        )
    {
        return PROFILE_STATUS_ERROR_DATA_OFFSET;
    }

    if  (
            (entry->_zero_signature != 0)
        ||  (entry->_zero_name != 0)
        )
    {
        return PROFILE_STATUS_ERROR_ENTRY_ZEROS;
    }

    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus profile_read_noop( void * data, uint32_t const address, uint32_t const length )
{
    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    return PROFILE_STATUS_ERROR_STORAGE_READ;
    (void)data;
    (void)address;
    (void)length;
}

static ProfileStatus profile_write_noop( void const * data, uint32_t const address, uint32_t const length )
{
    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    return PROFILE_STATUS_ERROR_STORAGE_WRITE;
    (void)data;
    (void)address;
    (void)length;
}

static ProfileStatus (* profile_storage_read)(  void       * data, uint32_t const address, uint32_t const length ) = profile_read_noop;
static ProfileStatus (* profile_storage_write_begin)( void );
static ProfileStatus (* profile_storage_write)( void const * data, uint32_t const address, uint32_t const length ) = profile_write_noop;
static ProfileStatus (* profile_storage_write_end)( void );

void profile_configure_storage_io(
    ProfileStatus (* const read )( void       * buffer, uint32_t const address, uint32_t const length ),
    ProfileStatus (* const write)( void const * buffer, uint32_t const address, uint32_t const length ),
	ProfileStatus (* const begin)( void ),
	ProfileStatus (* const end  )( void ) )
{
    if (read != NULL)
    {
        profile_storage_read = read;
    }

    if (write != NULL)
    {
    	if (begin != NULL)
    	{
    		profile_storage_write_begin = begin;
    	}
        profile_storage_write = write;
        if (end != NULL)
		{
        	profile_storage_write_end = end;
		}
    }

    cli_configure_storage_io( write );
}

#ifdef USE_TTERM
uint8_t profile_cli_info(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

    ProfileStatus const res = profile_header_validate( &profile_stub.header );

    if (res != PROFILE_STATUS_SUCCESS )
    {
    	ttprintf( "Invalid/Missing MESC Profile - %d" "\r" "\n", res );
    }
    else
    {
        ttprintf(
            "%s v%" PRIu8 "." PRIu8 "\n"
            "%s (",
            (char const *)&profile_stub.header.signature,
            profile_stub.header.version_major, profile_stub.header.version_minor,
            (char const *)&profile_stub.header.fingerprint );

        for ( uint32_t i = MESC_GITHASH_WORDS; i > 0; )
        {
            i--;

            ttprintf(
                "%08" PRIX32,
                profile_stub.header.fingerprint.githash[i] );
        }

        ttprintf(
            "\n"
            "Size %" PRIu32 "\r" "\n",
            profile_stub.header.image_length );

        uint32_t blk = 0;
        uint32_t fld = 0;

        for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
        {
            ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

            fld = fld + PROFILE_ENTRY_BITS;

            if (fld == BITS_PER_BYTE)
            {
                fld = 0;
                blk++;
            }

            ttprintf( "Entry %" PRIu32 " ", i );

            switch (map)
            {
                case PROFILE_ENTRY_DATA:
                	ttprintf( "DATA" "\r" "\n" );
                    continue;
                case PROFILE_ENTRY_LOCK:
                	ttprintf( "LOCK" "\r" "\n" );
                    break;
                case PROFILE_ENTRY_FREE:
                	ttprintf( "FREE" "\r" "\n" );
                    continue;
                case PROFILE_ENTRY_USED:
                	ttprintf( "USED" "\r" "\n" );
                    break;
                default:
                	ttprintf( "?" "\r" "\n" );
                    continue;
            }

            // NOTE - This point means there is a valid entry to decode

            ttprintf("    %s (%08" PRIX32") Offset: %08" PRIX32 " Len: %d\r\n", profile_stub.entry[i].name, profile_stub.entry[i].signature, profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length);
//            ttprintf("    %s (%s %08" PRIX32 ")" "\r" "\n" "    %08" PRIX32 "-%08" PRIX32 "\r" "\n",
//                profile_stub.entry[i].signature,
//                profile_stub.entry[i].name, profile_stub.entry[i].data_signature,
//                profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length );
        }
    }
    return TERM_CMD_EXIT_SUCCESS;
}
#endif

static uint8_t profile_buffer[PROFILE_MAX_SIZE];

ProfileStatus profile_init( void )
{
    uint32_t address = 0;

    profile_status_storage = profile_storage_read( &profile_stub.header, address, sizeof(profile_stub.header) );
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    address = address + sizeof(profile_stub.header);


    if (profile_status_storage != PROFILE_STATUS_SUCCESS)
    {
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    profile_status_header = profile_header_validate( &profile_stub.header );

    if (profile_status_header != PROFILE_STATUS_SUCCESS)
    {
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    uint32_t blk = 0;
    uint32_t fld = 0;
    uint32_t image_checksum = fnv1a_init();

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        if ((map & PROFILE_ENTRY_R) == PROFILE_ENTRY_R)
        {
            profile_status_storage = profile_storage_read( &profile_stub.entry[i], address, sizeof(profile_stub.entry[i]) );

            if (profile_status_storage != PROFILE_STATUS_SUCCESS)
            {
                profile_status_other = PROFILE_STATUS_ERROR_ENTRY(i);
                profile_init_default();
                return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
            }

            profile_status_entry = profile_entry_validate( &profile_stub.entry[i] );

            if (profile_status_entry != PROFILE_STATUS_SUCCESS)
            {
                profile_init_default();
                return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
            }

            image_checksum = fnv1a_process_data( image_checksum, &profile_stub.entry[i], sizeof(profile_stub.entry[i]) );

            profile_entry[i].buffer = &profile_buffer[profile_stub.entry[i].data_offset];
            profile_entry[i].length = &profile_stub.entry[i].data_length;
        }
        else
        {
            image_checksum = fnv1a_process_zero( image_checksum, sizeof(profile_stub.entry[i]) );

            profile_entry[i].buffer = NULL;
        }

        address = address + sizeof(ProfileEntry);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }
    }

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        if (profile_entry[i].buffer != NULL && profile_entry[i].length != NULL)
        {
            profile_status_storage = profile_storage_read( profile_entry[i].buffer, profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length );

            if (profile_status_storage != PROFILE_STATUS_SUCCESS)
            {
                profile_status_other = PROFILE_STATUS_ERROR_ENTRY(i);
                profile_init_default();
                return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
            }

            image_checksum = fnv1a_process_data( image_checksum, profile_entry[i].buffer, *(profile_entry[i].length) );

        }
    }

    if (image_checksum != profile_stub.header.image_checksum)
    {
        profile_status_header = PROFILE_STATUS_ERROR_IMAGE_CHECKSUM;
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    return PROFILE_STATUS_INIT_SUCCESS_LOADED;
}

ProfileStatus profile_alloc_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t const * const length )
{
    uint32_t blk = 0;
    uint32_t fld = 0;

    uint32_t const name_length = (uint32_t)strlen( name );

    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    if (name_length > PROFILE_ENTRY_MAX_NAME_LENGTH)
    {
        return PROFILE_STATUS_ERROR_NAME_LENGTH;
    }

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        if (map == PROFILE_ENTRY_FREE)
        {
            profile_stub.header.entry_map[blk] &= ~(PROFILE_ENTRY_MASK << fld);
            profile_stub.header.entry_map[blk] |=  (PROFILE_ENTRY_USED << fld);

            memset( &profile_stub.entry[i], 0, PROFILE_ENTRY_SIZE );

            profile_stub.entry[i].signature = PROFILE_ENTRY_SIGNATURE;

            profile_stub.entry[i].size = PROFILE_ENTRY_SIZE;
            profile_stub.entry[i].name_length = (uint8_t)name_length;
            strcpy( profile_stub.entry[i].name, name );

            profile_stub.entry[i].data_signature = signature;

            profile_stub.entry[i].data_length = 0; // Deferred (profile_entry[i].length)
            profile_stub.entry[i].data_offset = 0; // Deferred (alloc)

            profile_entry[i].buffer = buffer;
            profile_entry[i].length = length;

            return PROFILE_STATUS_SUCCESS_ENTRY_ALLOC;
        }

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }
    }

    return PROFILE_STATUS_ERROR_ENTRY_ALLOC;
}

ProfileStatus profile_get_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t * const length)
{
    ProfileStatus ret = PROFILE_STATUS_ERROR_NAME;
    uint32_t blk = 0;
    uint32_t fld = 0;

    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if ((map & PROFILE_ENTRY_R) != PROFILE_ENTRY_R)
        {
            continue;
        }

        if (strncmp( profile_stub.entry[i].name, name, PROFILE_ENTRY_MAX_NAME_LENGTH ) == 0)
        {
            ret = PROFILE_STATUS_ERROR_ENTRY_SIGNATURE;

            if (profile_stub.entry[i].data_signature == signature)
            {
                ret = PROFILE_STATUS_SUCCESS;

                if  (
                        (buffer == NULL)
                    &&  (length == NULL)
                    )
                {
                    if (map == PROFILE_ENTRY_USED)
                    {
                        profile_stub.header.entry_map[blk] &= ~(PROFILE_ENTRY_MASK << fld);
                        profile_stub.header.entry_map[blk] |=  (PROFILE_ENTRY_FREE << fld);

                        profile_entry[i].buffer = NULL;
                        profile_entry[i].length = NULL;

                        profile_modified = true;

                        ret = PROFILE_STATUS_SUCCESS_ENTRY_FREE;
                    }
                    else
                    {
                        ret = PROFILE_STATUS_ERROR_ENTRY_READONLY;
                    }
                }
                else if ( (profile_entry[i].buffer != NULL) &&  (profile_entry[i].length != NULL)){

                    if (*length != profile_stub.entry[i].data_length)
                    {
                        return PROFILE_STATUS_ERROR_DATA_LENGTH;
                    }

                    ret = profile_storage_read( buffer, profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length );

                    if (ret == PROFILE_STATUS_SUCCESS)
                    {
                        profile_entry[i].buffer = buffer;
                        profile_entry[i].length = length;
                    }
                }

                break;
            }
        }
    }

    return ret;
}

static ProfileStatus profile_check_entry(
    char const * name, uint32_t const signature )
{
    ProfileStatus ret = PROFILE_STATUS_ERROR_NAME;
    uint32_t blk = 0;
    uint32_t fld = 0;

    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if ((map & PROFILE_ENTRY_R) != PROFILE_ENTRY_R)
        {
            continue;
        }

        if (strncmp( profile_stub.entry[i].name, name, PROFILE_ENTRY_MAX_NAME_LENGTH ) == 0)
        {
            ret = PROFILE_STATUS_ERROR_ENTRY_SIGNATURE;

            if (profile_stub.entry[i].data_signature == signature)
            {
                ret = PROFILE_STATUS_SUCCESS;
            }

        }
    }

    return ret;
}

ProfileStatus profile_read_entry(
    uint32_t * const index, ProfileEntry const ** const entry )
{
    ProfileStatus ret = PROFILE_STATUS_FAILURE_SCAN;
    uint32_t blk = (*index >> PROFILE_ENTRY_BITS);
    uint32_t fld = (*index &  PROFILE_ENTRY_MASK) * PROFILE_ENTRY_BITS;

    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    for ( uint32_t i = *index; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if ((map & PROFILE_ENTRY_R) != PROFILE_ENTRY_R)
        {
            continue;
        }

        if (profile_stub.entry[i].signature != PROFILE_ENTRY_SIGNATURE)
        {
            continue;
        }

        if (profile_stub.entry[i].size != PROFILE_ENTRY_SIZE)
        {
            continue;
        }

        if (profile_stub.entry[i].name_length > PROFILE_ENTRY_MAX_NAME_LENGTH)
        {
            continue;
        }

        ret = PROFILE_STATUS_SUCCESS;
        *index = i;
        *entry = &profile_stub.entry[i];
        return ret;
    }

    *index = PROFILE_HEADER_ENTRIES;
    *entry = NULL;
    return ret;
}

ProfileStatus profile_scan_entry(
    uint32_t * const index, uint32_t const signature,
    void * const buffer, uint32_t * const length,
    char const ** const name )
{
    ProfileStatus ret = PROFILE_STATUS_FAILURE_SCAN;
    uint32_t blk = (*index >> PROFILE_ENTRY_BITS);
    uint32_t fld = (*index &  PROFILE_ENTRY_MASK) * PROFILE_ENTRY_BITS;

    for ( uint32_t i = *index; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if ((map & PROFILE_ENTRY_R) != PROFILE_ENTRY_R)
        {
            continue;
        }

        if (profile_stub.entry[i].signature != PROFILE_ENTRY_SIGNATURE)
        {
            continue;
        }

        if (profile_stub.entry[i].size != PROFILE_ENTRY_SIZE)
        {
            continue;
        }

        if (profile_stub.entry[i].name_length < PROFILE_ENTRY_MAX_NAME_LENGTH)
        {
            continue;
        }

        if  (profile_stub.entry[i].data_signature == signature)
        {
            ret = PROFILE_STATUS_SUCCESS;
            *index = i;
            *name = profile_stub.entry[i].name;

            if (
                    (profile_entry[i].buffer == NULL)
                &&  (profile_entry[i].length == NULL)
                )
            {
                if (*length != profile_stub.entry[i].data_length)
                {
                    return PROFILE_STATUS_ERROR_DATA_LENGTH;
                }

                ret = profile_storage_read( buffer, profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length );

                if (ret == PROFILE_STATUS_SUCCESS)
                {
                    profile_entry[i].buffer = buffer;
                    profile_entry[i].length = length;
                }
            }

            break;
        }
    }

    return ret;
}

ProfileStatus profile_put_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t * const length )
{
    profile_status_storage = PROFILE_STATUS_UNKNOWN;
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = profile_check_entry( name, signature );
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    if (profile_status_entry != PROFILE_STATUS_SUCCESS)
    {
        profile_status_entry = profile_alloc_entry( name, signature, buffer, length );
    }

    switch (profile_status_entry)
    {
        case PROFILE_STATUS_SUCCESS_ENTRY_ALLOC:
            profile_status_other = PROFILE_STATUS_SUCCESS_ENTRY_ALLOC;
            break;
        case PROFILE_STATUS_SUCCESS:
            profile_status_other = PROFILE_STATUS_SUCCESS_ENTRY_UPDATE;
            break;
        default:
            // Many possible failures; abort
            return PROFILE_STATUS_UNKNOWN;
    }

    profile_modified = true;

    return PROFILE_STATUS_SUCCESS;
}

ProfileStatus profile_del_entry(
    char const * name, uint32_t const signature )
{
    return profile_get_entry( name, signature, NULL, NULL);
}

ProfileStatus profile_commit( void )
{
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    if (profile_modified == false)
    {
        profile_status_storage = PROFILE_STATUS_COMMIT_SUCCESS_NOOP;
        return PROFILE_STATUS_SUCCESS;
    }

    uint32_t address = sizeof(profile_stub.header);

    uint32_t blk = 0;
    uint32_t fld = 0;
    uint32_t offset = sizeof(ProfileHeader) + (sizeof(ProfileEntry) * PROFILE_HEADER_ENTRIES);
    uint32_t checksum = fnv1a_init();

    ProfileStatus sts = profile_storage_write_begin();

    if (sts != PROFILE_STATUS_SUCCESS)
    {
    	profile_storage_write_end();
    	return PROFILE_STATUS_ERROR_STORAGE_BEGIN;
    }

    // Write entries
    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if (map != PROFILE_ENTRY_FREE)
        {
            profile_stub.entry[i].data_offset = offset;
            profile_stub.entry[i].data_length = *(profile_entry[i].length); // Must be aligned 4

            profile_status_storage = profile_storage_write( &profile_stub.entry[i], address, sizeof(profile_stub.entry[i]) );
            checksum = fnv1a_process_data( checksum, &profile_stub.entry[i], sizeof(profile_stub.entry[i]) );

            offset = offset + profile_stub.entry[i].data_length;

            switch (profile_status_storage)
            {
                case PROFILE_STATUS_SUCCESS:
                case PROFILE_STATUS_COMMIT_SUCCESS:
                case PROFILE_STATUS_COMMIT_SUCCESS_NOOP:
                    break;
                default:
                	profile_storage_write_end();
                    return PROFILE_STATUS_ERROR_STORAGE_WRITE;
            }
        }
        else
        {
            checksum = fnv1a_process_zero( checksum, sizeof(profile_stub.entry[i]) );
        }

        address = address + sizeof(profile_stub.entry[i]);
    }

    profile_stub.header.image_length = offset - sizeof(profile_stub.header);

    address = sizeof(profile_stub); // Should already be the case

    blk = 0;
    fld = 0;

    // Write Entry data
    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_stub.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if ((map & PROFILE_ENTRY_R) != PROFILE_ENTRY_R)
        {
            continue;
        }

        profile_status_storage = profile_storage_write( profile_entry[i].buffer, profile_stub.entry[i].data_offset, profile_stub.entry[i].data_length );
        checksum = fnv1a_process_data( checksum, profile_entry[i].buffer, profile_stub.entry[i].data_length );

        switch (profile_status_storage)
        {
            case PROFILE_STATUS_SUCCESS:
            case PROFILE_STATUS_COMMIT_SUCCESS:
            case PROFILE_STATUS_COMMIT_SUCCESS_NOOP:
                break;
            default:
            	profile_storage_write_end();
                return PROFILE_STATUS_ERROR_STORAGE_WRITE;
        }

        address = address + profile_stub.entry[i].data_length;
    }

    address  = 0;

    // Patch header
    profile_stub.header.checksum       = PROFILE_SIGNATURE;
    profile_stub.header.image_checksum = checksum;

    checksum = fnv1a_init();
    checksum = fnv1a_process_data( checksum, &profile_stub.header, sizeof(profile_stub.header) );
    profile_stub.header.checksum = checksum;

    // Write header
    profile_status_storage = profile_storage_write( &profile_stub.header, address, sizeof(profile_stub.header) );

    sts = profile_storage_write_end();

    if (sts != PROFILE_STATUS_SUCCESS)
    {
    	return PROFILE_STATUS_ERROR_STORAGE_END;
    }

    switch (profile_status_storage)
    {
        case PROFILE_STATUS_SUCCESS:
        case PROFILE_STATUS_COMMIT_SUCCESS:
        case PROFILE_STATUS_COMMIT_SUCCESS_NOOP:
            break;
        default:
        	profile_storage_write_end();
            return PROFILE_STATUS_ERROR_STORAGE_WRITE;
    }

    profile_modified = false;

    return PROFILE_STATUS_SUCCESS;
}

bool profile_get_modified( void )
{
	return profile_modified;
}
