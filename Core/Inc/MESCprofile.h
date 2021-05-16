
#ifndef MESC_PROFILE_H
#define MESC_PROFILE_H

#include <stdint.h>

#define MAKE_UINT32_STRING(a,b,c,d) \
       (((uint32_t)(a) <<  0) \
    |   ((uint32_t)(b) <<  8) \
    |   ((uint32_t)(c) << 16) \
    |   ((uint32_t)(d) << 24))

#define PROFILE_SIGNATURE     MAKE_UINT32_STRING('M','E','S','C')

#define PROFILE_VERSION_MAJOR UINT8_C(1)
#define PROFILE_VERSION_MINOR UINT8_C(0)

struct ProfileHeader
{
    uint32_t signature;         // PROFILE_SIGNATURE

    uint8_t  _zero_signature;   // Must be zero
    uint8_t  size;              // Size of this header in bytes (PROFILE_HEADER_SIZE)
    uint8_t  version_major;     // Major version
    uint8_t  version_minor;     // Minor version

    uint32_t checksum;          // Checksum of header (excluding this field; treat as PROFILE_SIGNATURE)

    uint8_t  entry_map[8];      // ProfileEntry map (2 bits ProfileEntryMap per entry)

    uint32_t _reserved;

    uint32_t image_length;      // Length of image (after this header) in bytes
    uint32_t image_checksum;    // Checksum of image
};

typedef struct ProfileHeader ProfileHeader;

#define PROFILE_HEADER_SIZE (sizeof(ProfileHeader))

#define PROFILE_HEADER_ENTRIES ((8/*BITS_PER_BYTE*/ * sizeof(((ProfileHeader *)NULL)->entry_map)) / 2/*ENTRY_MAP_BITS*/)

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

enum ProfileStatus
{
    PROFILE_STATUS_UNKNOWN,                 // Result unknown

    PROFILE_STATUS_SUCCESS,

    PROFILE_STATUS_INIT_SUCCESS_DEFAULT,    // Default profile loaded
    PROFILE_STATUS_INIT_SUCCESS_LOADED,     // Saved profile loaded
    PROFILE_STATUS_INIT_FALLBACK_DEFAULT,   // Saved profile invalid; default loaded

    PROFILE_STATUS_ERROR_STORAGE_READ,      // Error reading from the storage
    PROFILE_STATUS_ERROR_STORAGE_WRITE,     // Error writing to the storage

    PROFILE_STATUS_ERROR_ENTRY_ALLOC,       // No free entries available
    PROFILE_STATUS_ERROR_ENTRY_READONLY,    // Cannot modify read-only entry

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
};

typedef enum ProfileStatus ProfileStatus;

void profile_configure_storage(
    ProfileStatus (* const read)( void * buffer, uint32_t const length ),
    ProfileStatus (* const write)( void const * buffer, uint32_t const length ) );

ProfileStatus profile_init( void );

void profile_get_last(
    ProfileStatus * const storage,
    ProfileStatus * const header,
    ProfileStatus * const entry,
    ProfileStatus * const other );

ProfileStatus profile_alloc_entry(
    char const * name, uint32_t const signature,
    void const ** const buffer, uint32_t ** const length );

ProfileStatus profile_get_entry(
    char const * name, uint32_t const signature,
    void const ** const buffer, uint32_t * const length );

ProfileStatus profile_put_entry(
    char const * name, uint32_t const signature,
    void const * const buffer, uint32_t const length );

ProfileStatus profile_del_entry(
    char const * name, uint32_t const signature );

ProfileStatus profile_commit( void );

#endif
