
#include "MESCprofile.h"

#include <stdio.h>

static void bist_profile_error_signature( void )
{
}

static void bist_profile_error_length( void )
{
}

static void bist_profile_error_size( void )
{
}

static void bist_profile_error_checksum( void )
{
}

void bist_profile( void )
{
    fprintf( stdout, "Starting Profile BIST\n" );

    bist_profile_error_signature();
    bist_profile_error_length();
    bist_profile_error_size();
    bist_profile_error_checksum();

    fprintf( stdout, "Finished Profile BIST\n" );
}
