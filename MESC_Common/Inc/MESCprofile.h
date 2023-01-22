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

#ifndef MESC_PROFILE_H
#define MESC_PROFILE_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bit_op.h"
#include "string_op.h"

#include "fingerprint.h"

#define PROFILE_SIGNATURE     MAKE_UINT32_STRING('M','E','S','C')

#define PROFILE_VERSION_MAJOR UINT8_C(1)
#define PROFILE_VERSION_MINOR UINT8_C(0)

#define PROFILE_MAX_ENTRIES (8)

struct ProfileHeader
{
    uint32_t    signature;          // PROFILE_SIGNATURE

    uint8_t     _zero_signature;    // Must be zero
    uint8_t     version_major;      // Major version
    uint8_t     version_minor;      // Minor version
    uint8_t     size;               // Size of this header in bytes (PROFILE_HEADER_SIZE)

    uint32_t    checksum;           // Checksum of header (excluding this field; treat as PROFILE_SIGNATURE)

    uint8_t     entry_map[PROFILE_MAX_ENTRIES]; // ProfileEntry map (PROFILE_ENTRY_BITS bits ProfileEntryMap per entry)

    uint32_t    image_length;       // Length of image (after this header) in bytes
    uint32_t    image_checksum;     // Checksum of image

    MESCFingerprint fingerprint;    // Timestamp and git hash
};

typedef struct ProfileHeader ProfileHeader;

#define PROFILE_HEADER_SIZE (sizeof(ProfileHeader))

static_assert( ((PROFILE_HEADER_SIZE % 16) == 0), "PROFILE_HEADER_SIZE must be 16-byte aligned" );

enum ProfileEntryMap
{
    // Access flags
    PROFILE_ENTRY_R = 0x1,
    PROFILE_ENTRY_W = 0x2,

    PROFILE_ENTRY_RW = (PROFILE_ENTRY_R | PROFILE_ENTRY_W),
    // Aliases
    PROFILE_ENTRY_DATA = 0x0,               // Used by data (not available)
    PROFILE_ENTRY_LOCK = PROFILE_ENTRY_R,   // Profile entry present (read-only)
    PROFILE_ENTRY_FREE = PROFILE_ENTRY_W,   // No profile entry present (available)
    PROFILE_ENTRY_USED = PROFILE_ENTRY_RW,  // Profile entry present (read-write)
};

typedef enum ProfileEntryMap ProfileEntryMap;

#define PROFILE_ENTRY_BITS (2)
#define PROFILE_ENTRY_MASK BIT_MASK_32( PROFILE_ENTRY_BITS )
#define PROFILE_ENTRY_SUBINDEX(e) ((e) * PROFILE_ENTRY_BITS)

static_assert( ((BITS_PER_BYTE % PROFILE_ENTRY_BITS) == 0), "PROFILE_ENTRY_BITS must fit into a byte" );

#define PROFILE_HEADER_ENTRIES ((BITS_PER_BYTE * sizeof(((ProfileHeader *)NULL)->entry_map)) / PROFILE_ENTRY_BITS)

#define PROFILE_ENTRY_SIGNATURE MAKE_UINT32_STRING('M','P','E','H')

#define PROFILE_ENTRY_MAX_NAME_LENGTH UINT32_C(12)

struct ProfileEntry
{
    uint32_t signature;         // PROFILE_ENTRY_SIGNATURE

    uint8_t _zero_signature;    // Must be zero
    uint8_t size;               // Size of this header (PROFILE_ENTRY_SIZE)
    uint8_t name_length;        // Length of name string
    char    name[PROFILE_ENTRY_MAX_NAME_LENGTH];
    uint8_t _zero_name;         // Must be zero

    uint32_t data_signature;    // Entry-specific signature

    uint32_t data_length;       // Size of data in bytes
    uint32_t data_offset;       // Offset from start of ProfileHeader (aligned to 16 by convention; must be aligned to 4)
};

typedef struct ProfileEntry ProfileEntry;

#define PROFILE_ENTRY_SIZE (sizeof(ProfileEntry))

static_assert( ((PROFILE_ENTRY_SIZE % 16) == 0), "PROFILE_ENTRY_SIZE must be 16-byte aligned" );

#define PROFILE_ENTRY_MIN_OFFSET PROFILE_HEADER_SIZE

enum ProfileStatus
{
    PROFILE_STATUS_UNKNOWN,                 // Result unknown

    PROFILE_STATUS_SUCCESS,

    PROFILE_STATUS_INIT_SUCCESS_DEFAULT,    // Default profile loaded
    PROFILE_STATUS_INIT_SUCCESS_LOADED,     // Saved profile loaded
    PROFILE_STATUS_INIT_FALLBACK_DEFAULT,   // Saved profile invalid; default loaded

    PROFILE_STATUS_ERROR_STORAGE_READ,      // Error reading from the storage
    PROFILE_STATUS_ERROR_STORAGE_WRITE,     // Error writing to the storage
	PROFILE_STATUS_ERROR_STORAGE_BEGIN,     // Error preparing write
	PROFILE_STATUS_ERROR_STORAGE_END,       // Error finalising write

    PROFILE_STATUS_ERROR_ENTRY_ALLOC,       // No free entries available
    PROFILE_STATUS_ERROR_ENTRY_READONLY,    // Cannot modify read-only entry

    PROFILE_STATUS_ERROR_ENTRY_0,           // Additional error indicator
    PROFILE_STATUS_ERROR_ENTRY_N = (PROFILE_STATUS_ERROR_ENTRY_0 + PROFILE_HEADER_ENTRIES - 1),

    PROFILE_STATUS_ERROR_HEADER_SIGNATURE,  // Invalid header signature
    PROFILE_STATUS_ERROR_HEADER_SIZE,       // Invalid header size
    PROFILE_STATUS_ERROR_HEADER_VERSION,    // Invalid/unsupported version
    PROFILE_STATUS_ERROR_HEADER_CHECKSUM,   // Header checksum failure
    PROFILE_STATUS_ERROR_HEADER_ZEROS,      // Header zero region(s) not zero

    PROFILE_STATUS_ERROR_IMAGE_LENGTH,      // Invalid image length
    PROFILE_STATUS_ERROR_IMAGE_CHECKSUM,    // Invalid image checksum

    PROFILE_STATUS_ERROR_ENTRY_SIGNATURE,   // Invalid entry signature
    PROFILE_STATUS_ERROR_ENTRY_SIZE,        // Invalid entry size
    PROFILE_STATUS_ERROR_ENTRY_ZEROS,       // Entry zero region(s) not zero

    PROFILE_STATUS_ERROR_NAME_LENGTH,       // Invalid name length
    PROFILE_STATUS_ERROR_NAME,              // Invalid name

    PROFILE_STATUS_ERROR_DATA_SIGNATURE,    // Invalid data signature
    PROFILE_STATUS_ERROR_DATA_OFFSET,       // Invalid data offset
    PROFILE_STATUS_ERROR_DATA_LENGTH,       // Invalid data length

    PROFILE_STATUS_COMMIT_SUCCESS,          // Profile successfully written to storage
    PROFILE_STATUS_COMMIT_SUCCESS_NOOP,     // Profile writing skipped (no actual changes)

    PROFILE_STATUS_SUCCESS_ENTRY_ALLOC,     // Entry was allocated
    PROFILE_STATUS_SUCCESS_ENTRY_FREE,      // Entry was removed
    PROFILE_STATUS_SUCCESS_ENTRY_NOOP,      // Entry was not present
    PROFILE_STATUS_SUCCESS_ENTRY_UPDATE,    // Entry was updated

    PROFILE_STATUS_FAILURE_SCAN,            // End of scan
};

typedef enum ProfileStatus ProfileStatus;

#define PROFILE_STATUS_ERROR_ENTRY(e) ((ProfileStatus)(PROFILE_STATUS_ERROR_ENTRY_0 + (e)))

#define PROFILE_MIN_SIZE (PROFILE_HEADER_SIZE + (PROFILE_HEADER_ENTRIES * PROFILE_ENTRY_SIZE))
#define PROFILE_MAX_SIZE UINT32_C(4096)

static_assert( ((PROFILE_MAX_SIZE % 2048/*FLASH_BLOCK_SIZE*/) == 0), "PROFILE_MAX_SIZE must be flash block aligned" );

void profile_configure_storage_io(
    ProfileStatus (* const read)(  void       * buffer, uint32_t const address, uint32_t const length ),
    ProfileStatus (* const write)( void const * buffer, uint32_t const address, uint32_t const length ),
	ProfileStatus (* const begin)( void ),
	ProfileStatus (* const end  )( void ) );

ProfileStatus profile_init( void );

void profile_get_last(
    ProfileStatus * const storage,
    ProfileStatus * const header,
    ProfileStatus * const entry,
    ProfileStatus * const other );

ProfileStatus profile_alloc_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t const * const length );

ProfileStatus profile_get_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t * const length );

ProfileStatus profile_read_entry(
    uint32_t * const index, ProfileEntry const ** const entry );

ProfileStatus profile_scan_entry(
    uint32_t * const index, uint32_t const signature,
    void * const buffer, uint32_t * const length,
    char const ** const name );

ProfileStatus profile_put_entry(
    char const * name, uint32_t const signature,
    void * const buffer, uint32_t * const length );

ProfileStatus profile_del_entry(
    char const * name, uint32_t const signature );

ProfileStatus profile_commit( void );

bool profile_get_modified( void );

#ifdef USE_TTERM
#include "TTerm/Core/include/TTerm.h"
uint8_t profile_cli_info(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
#endif

#define PROFILE_DEFAULT NULL // Generic symbol to indicate loading of default profile

#endif
