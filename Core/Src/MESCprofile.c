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

#include "MESCfnv.h"

#include "bit_op.h"

#include <stddef.h>
#include <string.h>

static union
{
    struct
    {
    ProfileHeader   header;
    ProfileEntry    entry[PROFILE_HEADER_ENTRIES];
    uint8_t         data;
    }               layout;
    uint8_t         raw;
}   profile_image;

static uint8_t profile_modified = 0;

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
    memset( &profile_image, 0, sizeof(profile_image) );

    profile_image.layout.header.signature      = PROFILE_SIGNATURE;

    profile_image.layout.header.size           = PROFILE_HEADER_SIZE;
    profile_image.layout.header.version_major  = PROFILE_VERSION_MAJOR;
    profile_image.layout.header.version_minor  = PROFILE_VERSION_MINOR;

    profile_image.layout.header.checksum       = PROFILE_SIGNATURE;

    profile_image.layout.header.image_checksum = fnv1a_init();

    profile_image.layout.header.checksum       = fnv1a_data( &profile_image.layout.header, PROFILE_HEADER_SIZE );

    uint32_t blk = 0;
    uint32_t fld = 0;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        profile_image.layout.header.entry_map[blk] |= (PROFILE_ENTRY_FREE << fld);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }
    }

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
        ||  (header->_reserved != 0)
        )
    {
        return PROFILE_STATUS_ERROR_HEADER_ZEROS;
    }

    uint32_t hash = fnv1a_init();
    uint32_t const offset = offsetof(ProfileHeader,checksum);
    uint32_t const sig = PROFILE_SIGNATURE;

    hash = fnv1a_process_data( hash, header, offset );
    hash = fnv1a_process_data( hash, &sig, sizeof(sig) );
    hash = fnv1a_process_data( hash, &header->entry_map[0], (PROFILE_HEADER_SIZE - offset) );

    if (hash != header->checksum)
    {
        return PROFILE_STATUS_ERROR_HEADER_CHECKSUM;
    }

    uint32_t const image_length = header->image_length;

    if (image_length > (4096/*TODO*/ - PROFILE_HEADER_SIZE))
    {
        return PROFILE_STATUS_ERROR_IMAGE_LENGTH;
    }

    void const * image = &((uint8_t const *)header)[PROFILE_HEADER_SIZE];
    uint32_t const image_hash = fnv1a_data( image, image_length );

    if (image_hash != header->image_checksum)
    {
        return PROFILE_STATUS_ERROR_IMAGE_CHECKSUM;
    }

    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus profile_entry_validate( ProfileHeader * const header, ProfileEntry * entry )
{
    uint32_t blk = 0;
    uint32_t fld = 0;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((header->entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

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

        if (entry[i].signature != PROFILE_ENTRY_SIGNATURE)
        {
            return PROFILE_STATUS_ERROR_ENTRY_SIGNATURE;
        }

        if (entry[i].size != PROFILE_ENTRY_SIZE)
        {
            return PROFILE_STATUS_ERROR_ENTRY_SIZE;
        }

        if (entry[i].name_length > PROFILE_ENTRY_MAX_NAME_LENGTH)
        {
            return PROFILE_STATUS_ERROR_NAME_LENGTH;
        }

        if (entry[i].data_signature == 0)
        {
            return PROFILE_STATUS_ERROR_DATA_SIGNATURE;
        }

        if  (
                (entry[i].data_length == 0)
            ||  ((entry[i].data_offset & 3) != 0) // align 4
            ||  (entry[i].data_offset < PROFILE_HEADER_SIZE)
            ||  ((entry[i].data_offset + entry[i].data_length) > 4096/*TODO*/)
            )
        {
            return PROFILE_STATUS_ERROR_DATA_LENGTH;
        }

        if  (
                (entry[i]._zero_signature != 0)
            ||  (entry[i]._zero_name != 0)
            )
        {
            return PROFILE_STATUS_ERROR_ENTRY_ZEROS;
        }
    }

    return PROFILE_STATUS_SUCCESS;
}

static ProfileStatus profile_read_noop( void * data, uint32_t const length )
{
    return PROFILE_STATUS_UNKNOWN;
    (void)data;
    (void)length;
}

static ProfileStatus profile_write_noop( void const * data, uint32_t const length )
{
    return PROFILE_STATUS_UNKNOWN;
    (void)data;
    (void)length;
}

static ProfileStatus (* profile_storage_read)(  void       *, uint32_t const ) = profile_read_noop;
static ProfileStatus (* profile_storage_write)( void const *, uint32_t const ) = profile_write_noop;

void profile_configure_storage_io(
    ProfileStatus (* const read )( void       * buffer, uint32_t const length ),
    ProfileStatus (* const write)( void const * buffer, uint32_t const length ) )
{
    if (read != NULL)
    {
        profile_storage_read = read;
    }

    if (write != NULL)
    {
        profile_storage_write = write;
    }
}

ProfileStatus profile_init( void )
{
    profile_status_storage = profile_storage_read( &profile_image, sizeof(profile_image) );
    profile_status_header  = PROFILE_STATUS_UNKNOWN;
    profile_status_entry   = PROFILE_STATUS_UNKNOWN;
    profile_status_other   = PROFILE_STATUS_UNKNOWN;

    if (profile_status_storage != PROFILE_STATUS_SUCCESS)
    {
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    if (profile_image.layout.header.signature != PROFILE_SIGNATURE)
    {
        if (profile_image.layout.header.signature == 0)
        {
            profile_init_default();
            return PROFILE_STATUS_INIT_SUCCESS_DEFAULT;
        }
        else
        {
            profile_init_default();
            return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
        }
    }

    profile_status_header = profile_header_validate( &profile_image.layout.header );

    if (profile_status_header != PROFILE_STATUS_SUCCESS)
    {
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    profile_status_entry = profile_entry_validate( &profile_image.layout.header, &profile_image.layout.entry[0] );

    if (profile_status_entry != PROFILE_STATUS_SUCCESS)
    {
        profile_init_default();
        return PROFILE_STATUS_INIT_FALLBACK_DEFAULT;
    }

    return PROFILE_STATUS_INIT_SUCCESS_LOADED;
}

ProfileStatus profile_alloc_entry(
    char const * name, uint32_t const signature,
    void const ** const buffer, uint32_t ** const length )
{
    uint32_t blk = 0;
    uint32_t fld = 0;

    uint32_t const name_length = (uint32_t)strlen( name );

    if (name_length > PROFILE_ENTRY_MAX_NAME_LENGTH)
    {
        return PROFILE_STATUS_ERROR_NAME_LENGTH;
    }

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_image.layout.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

        fld = fld + PROFILE_ENTRY_BITS;

        if (fld == BITS_PER_BYTE)
        {
            fld = 0;
            blk++;
        }

        if (map != PROFILE_ENTRY_FREE)
        {
            continue;
        }

        if (fld == 0)
        {
            fld = BITS_PER_BYTE;
            blk--;
        }

        fld = fld - PROFILE_ENTRY_BITS;

        profile_image.layout.header.entry_map[blk] &= (PROFILE_ENTRY_MASK << fld);
        profile_image.layout.header.entry_map[blk] |= (PROFILE_ENTRY_USED << fld);

        memset( &profile_image.layout.entry[i], 0, PROFILE_ENTRY_SIZE );

        profile_image.layout.entry[i].signature = PROFILE_ENTRY_SIGNATURE;

        profile_image.layout.entry[i].size = PROFILE_ENTRY_SIZE;
        profile_image.layout.entry[i].name_length = (uint8_t)name_length;
        strcpy( profile_image.layout.entry[i].name, name );

        profile_image.layout.entry[i].data_signature = signature;

        profile_image.layout.entry[i].data_length = 32; // TODO alloc
        profile_image.layout.entry[i].data_offset = PROFILE_HEADER_SIZE; // TODO alloc

        // TODO allocate memeory region
        *buffer = &profile_image.layout.entry[i].data_offset;
        *length = &profile_image.layout.entry[i].data_length;

        return PROFILE_STATUS_SUCCESS_ENTRY_ALLOC;
    }

    return PROFILE_STATUS_ERROR_ENTRY_ALLOC;
}

ProfileStatus profile_get_entry(
    char const * name, uint32_t const signature,
    void const ** const buffer, uint32_t * const length )
{
    ProfileStatus ret = PROFILE_STATUS_ERROR_NAME;
    uint32_t blk = 0;
    uint32_t fld = 0;

    for ( uint32_t i = 0; i < PROFILE_HEADER_ENTRIES; ++i )
    {
        ProfileEntryMap const map = (ProfileEntryMap)((profile_image.layout.header.entry_map[blk] >> fld) & PROFILE_ENTRY_MASK);

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

        if (strncmp( profile_image.layout.entry[i].name, name, PROFILE_ENTRY_MAX_NAME_LENGTH ) == 0)
        {
            ret = PROFILE_STATUS_ERROR_ENTRY_SIGNATURE;

            if (profile_image.layout.entry[i].signature == signature)
            {
                uint32_t const offset = profile_image.layout.entry[i].data_offset;
                *buffer = &(&profile_image.raw)[offset];
                *length = profile_image.layout.entry[i].data_length;

                return PROFILE_STATUS_SUCCESS;
            }
        }
    }

    return ret;
}

ProfileStatus profile_put_entry(
    char const * name, uint32_t const signature,
    void const * const buffer, uint32_t const length )
{
    void const * entry_buffer = NULL;
    uint32_t     entry_length_ = 0;
    uint32_t *   entry_length = &entry_length_;

    profile_status_entry = profile_get_entry( name, signature, &entry_buffer, &entry_length_ );

    if (profile_status_entry != PROFILE_STATUS_SUCCESS)
    {
        profile_status_entry = profile_alloc_entry( name, signature, &entry_buffer, &entry_length );
    }

    switch (profile_status_entry)
    {
        case PROFILE_STATUS_SUCCESS_ENTRY_ALLOC:
            // The entry is new; populate
            // TODO
            (void)buffer;
            (void)length;
            profile_status_other = PROFILE_STATUS_SUCCESS_ENTRY_ALLOC;
            break;
        case PROFILE_STATUS_SUCCESS:
            // The entry already exists; update
            // TODO
            (void)buffer;
            (void)length;
            profile_status_other = PROFILE_STATUS_SUCCESS_ENTRY_UPDATE;
            break;
        default:
            // Many possible failures; abort
            return PROFILE_STATUS_UNKNOWN;
    }

    profile_modified = 1;

    return PROFILE_STATUS_SUCCESS;
}

ProfileStatus profile_del_entry(
    char const * name, uint32_t const signature )
{
    void const * entry_buffer = NULL;
    uint32_t     entry_length = 0;

    profile_status_entry = profile_get_entry( name, signature, &entry_buffer, &entry_length );

    if (profile_status_entry == PROFILE_STATUS_SUCCESS)
    {
        // TODO remove entry

        profile_modified = 1;

        return PROFILE_STATUS_SUCCESS_ENTRY_FREE;
    }

    return PROFILE_STATUS_SUCCESS_ENTRY_NOOP;
}

ProfileStatus profile_commit( void )
{
    if (profile_modified == 0)
    {
        profile_status_storage = PROFILE_STATUS_COMMIT_SUCCESS_NOOP;
        return PROFILE_STATUS_SUCCESS;
    }

    // TODO determine size

    profile_status_storage = profile_storage_write( &profile_image, sizeof(profile_image) );
    profile_modified = 0;

    return PROFILE_STATUS_SUCCESS;
}
