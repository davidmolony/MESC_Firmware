
#ifndef MESC_PROFILE_H
#define MESC_PROFILE_H

#include <stdint.h>

#define PROFILE_SIGNATURE     UINT32_C(0x4353454D);   // 'MESC'

#define PROFILE_VERSION_MAJOR UINT8_C(1)
#define PROFILE_VERSION_MINOR UINT8_C(0)

struct ProfileHeader
{
    uint32_t signature;         // PROFILE_SIGNATURE

    uint8_t  zero;              // Must be zero
    uint8_t  header_size;       // Size of this header in bytes (PROFILE_HEADER_SIZE)
    uint8_t  version_major;     // Major version
    uint8_t  version_minor;     // Minor version

    uint32_t header_checksum;   // Checksum of header (excluding this field)

    uint32_t reserved;

    uint32_t data_length;       // Length of data (after this header) in bytes
    uint32_t data_checksum;     // Checksum of data

    uint32_t reserved2[2];      // User-defined values
};

typedef struct ProfileHeader ProfileHeader;

#define FLASH_HEADER_SIZE (sizeof(ProfileHeader))

enum ProfileStatus
{
    PROFILE_STATUS_SUCCESS,

    PROFILE_STATUS_ERROR_READ,

    PROFILE_STATUS_ERROR_SIGNATURE,

    PROFILE_STATUS_ERROR_CORRUPT,
    PROFILE_STATUS_ERROR_HEADER_SIZE,
    PROFILE_STATUS_ERROR_VERSION,

    PROFILE_STATUS_ERROR_HEADER_CHECKSUM,

    PROFILE_STATUS_ERROR_DATA_LENGTH,
    PROFILE_STATUS_ERROR_DATA_CHECKSUM,

    PROFILE_STATUS_ERROR_WRITE,
};

typedef enum ProfileStatus ProfileStatus;

ProfileStatus profile_create( void );

ProfileStatus profile_read( void );

ProfileStatus profile_header_validate( void );

uint32_t profile_get_length( void );

uint32_t profile_get_size( void );

ProfileStatus profile_get_offset( uint32_t const offset, void const ** buffer, uint32_t const * length );

ProfileStatus profile_put_offset( uint32_t const offset, void const *  buffer, uint32_t const   length );

ProfileStatus profile_commit( void );

#endif
