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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "MESCprofile.h"

#include "MESCbat.h"
#include "MESCfnv.h"
#include "MESCspeed.h"
#include "MESCtemp.h"
#include "MESCui.h"

#include "virt_flash.h"

extern FILE * popen( char const *, char const * );
extern int pclose( FILE * );

extern void bist_bat( void );
extern void bist_cli( void );
extern void i_cli( void );
extern void bist_profile( void );
extern void bist_temp( void );

static void flash_register_profile_io( void )
{
    virt_flash_init();
    virt_flash_configure( true, true );

    profile_configure_storage_io( virt_flash_read, virt_flash_write, virt_flash_begin, virt_flash_end );
}

static int bist_main( void )
{
/*
NOTE

This code should mirror the implementation of main.c
*/
    fprintf( stdout, "Attach flash IO to profile\n" );
    flash_register_profile_io();

    fprintf( stdout, "Load stored profile\n" );
    profile_init();

    fprintf( stdout, "Initialise components\n" );
    bat_init( PROFILE_DEFAULT );
    speed_init( PROFILE_DEFAULT );
    fprintf( stdout, "Initialise user Interface\n" );
    ui_init( PROFILE_DEFAULT );

    virt_flash_free();

    return EXIT_SUCCESS;
}

// Loaded image (any any)
static uint8_t       * virt_flash_load        = NULL;
static ProfileHeader * virt_flash_load_header = NULL;
static ProfileEntry  * virt_flash_load_entry[PROFILE_HEADER_ENTRIES] = { NULL };

// Local image (possibly with modifications)
static uint8_t       * virt_flash_this        = NULL;
static ProfileHeader * virt_flash_this_header;
static ProfileEntry  * virt_flash_this_entry[PROFILE_HEADER_ENTRIES] = { NULL };

// Last saved image (if any)
static uint8_t       * virt_flash_last        = NULL;
static ProfileHeader * virt_flash_last_header = NULL;
static ProfileEntry  * virt_flash_last_entry[PROFILE_HEADER_ENTRIES] = { NULL };

static void cmd_flash_new( void )
{
    fprintf( stdout, "Creating new profile...\n" );

    virt_flash_this_header->signature       = PROFILE_SIGNATURE;

    virt_flash_this_header->_zero_signature = 0;
    virt_flash_this_header->version_major   = PROFILE_VERSION_MAJOR;
    virt_flash_this_header->version_minor   = PROFILE_VERSION_MINOR;
    virt_flash_this_header->size            = PROFILE_HEADER_SIZE;

    virt_flash_this_header->checksum        = PROFILE_SIGNATURE;

    uint8_t const v = (PROFILE_ENTRY_FREE << PROFILE_ENTRY_SUBINDEX(3))
                    | (PROFILE_ENTRY_FREE << PROFILE_ENTRY_SUBINDEX(2))
                    | (PROFILE_ENTRY_FREE << PROFILE_ENTRY_SUBINDEX(1))
                    | (PROFILE_ENTRY_FREE << PROFILE_ENTRY_SUBINDEX(0));

    memset( virt_flash_this_header->entry_map, v, PROFILE_MAX_ENTRIES );

    virt_flash_this_header->image_length   = 0;
    virt_flash_this_header->image_checksum = fnv1a_init();

    fprintf( stdout, "Loading fingerprint...\n" );
#if defined _MSC_VER
    FILE * f = fopen( "../Gen/fingerprint.stdout", "r" );
    char * p = fgets( (char*)&virt_flash_this_header->fingerprint,
                       sizeof(virt_flash_this_header->fingerprint), f );
#else
    FILE * pstdout = popen( "/bin/bash ../Gen/fingerprint.sh 2> /dev/null", "r" );
    assert( pstdout );

    char * p = fgets( (char *)&virt_flash_this_header->fingerprint,
                        sizeof(virt_flash_this_header->fingerprint), pstdout );
#endif
    assert( p );

    fprintf( stdout, "%s\n", p );
#if defined _MSC_VER
    fclose( f );
#else
    pclose( pstdout );
#endif
}

static void cmd_flash_reload( void )
{
    if (virt_flash_load)
    {
        fprintf( stdout, "Reloading profile...\n" );

        virt_flash_this = malloc( PROFILE_MAX_SIZE );
        assert( virt_flash_this );
        memcpy( virt_flash_this, virt_flash_load, PROFILE_MAX_SIZE );

        virt_flash_this_header = (ProfileHeader *)virt_flash_this;
        ProfileEntry * s = (ProfileEntry *)(&virt_flash_this[sizeof(ProfileHeader)]);

        for ( uint32_t e = 0; e < PROFILE_HEADER_ENTRIES; ++e )
        {
            virt_flash_this_entry[e] = &s[e];
        }
    }
    else
    {
        fprintf( stdout, "(no profile loaded)\n" );
    }
}

static void cmd_flash_save( void )
{
    virt_flash_last        = virt_flash_this;
    virt_flash_last_header = virt_flash_this_header;

    for ( uint32_t e = 0; e < PROFILE_HEADER_ENTRIES; ++e )
    {
        virt_flash_last_entry[e] = virt_flash_this_entry[e];
    }

    virt_flash_this = calloc( PROFILE_MAX_SIZE, 1 );
    assert( virt_flash_this );

    virt_flash_this_header = (ProfileHeader *)virt_flash_this;
    ProfileEntry * s = (ProfileEntry *)(&virt_flash_this[sizeof(ProfileHeader)]);

    for ( uint32_t e = 0; e < PROFILE_HEADER_ENTRIES; ++e )
    {
        virt_flash_this_entry[e] = &s[e];
    }
}

static void cmd_flash_write( FILE * f )
{
    (void)f;
}

enum CMDOption
{
    CMD_OPTION_INVALID,

    CMD_OPTION_HEADER,
    CMD_OPTION_ENTRY,
    CMD_OPTION_LIST,
    CMD_OPTION_NEW,
    CMD_OPTION_RELOAD,
    CMD_OPTION_WRITE,
    CMD_OPTION_QUIT,
};

typedef enum CMDOption CMDOption;

static CMDOption virt_flash_prompt( void )
{
    CMDOption cmdopt = CMD_OPTION_INVALID;

    do
    {
        fprintf( stdout, "Select option:\n"
                         "    [h] Header\n"
                         "    [e] Entry\n"
                         "    [l] List\n"
                         "    [n] New\n"
                         "    [r] Reload (discard changes)\n"
                         "    [w] Write changes\n"
                         "    [q] Quit\n"
                         "> " );

        int c = getc( stdin );

        if (c == EOF)
        {
            return CMD_OPTION_QUIT;
        }

        switch (c)
        {
            case 'h': cmdopt = CMD_OPTION_HEADER; break;
            case 'e': cmdopt = CMD_OPTION_ENTRY ; break;
            case 'l': cmdopt = CMD_OPTION_LIST  ; break;
            case 'n': cmdopt = CMD_OPTION_NEW   ; break;
            case 'r': cmdopt = CMD_OPTION_RELOAD; break;
            case 'w': cmdopt = CMD_OPTION_WRITE ; break;
            case 'q': cmdopt = CMD_OPTION_QUIT  ; break;
            default:
                break;
        }
    }
    while (cmdopt == CMD_OPTION_INVALID);

    return cmdopt;
}

static void virt_flash_header()
{
    // TODO header operations
}

static void virt_flash_entry()
{
    // TODO entry operations
}

static int virt_flash( int argc, char * argv[] )
{
    int const argl = argc - 1;
    int ret = EXIT_SUCCESS;

    char * arg_fi = NULL;
    char * arg_fo = NULL;

    for ( int a = 1; a < argl; ++a )
    {
        if (strcmp( argv[a], "--fi" ) == 0)
        {
            ++a;

            if (argv[a][0] != '-')
            {
                arg_fi = argv[a];
            }
        }

        if (strcmp( argv[a], "--fo" ) == 0)
        {
            ++a;

            if (argv[a][0] != '-')
            {
                arg_fo = argv[a];
            }
        }
    }

    FILE * fi = NULL;
    FILE * fo = NULL;

    if (arg_fi)
    {
        fi = fopen( arg_fi, "r" );
        assert( fi );

        (void)virt_flash_load;
        (void)virt_flash_load_header;
        (void)virt_flash_load_entry;
    }

    if (arg_fo)
    {
        fi = fopen( arg_fo, "r" );
        assert( fo );
    }

    if (virt_flash_load)
    {
        virt_flash_this = malloc( PROFILE_MAX_SIZE );
        assert( virt_flash_this );
        
        memcpy( virt_flash_this, virt_flash_load, PROFILE_MAX_SIZE );
    }
    else
    {
        virt_flash_this = calloc( PROFILE_MAX_SIZE, 1 );
        assert( virt_flash_this );
    }

    virt_flash_this_header = (ProfileHeader *)virt_flash_this;
    ProfileEntry * s = (ProfileEntry *)(&virt_flash_this[sizeof(ProfileHeader)]);

    for ( uint32_t e = 0; e < PROFILE_HEADER_ENTRIES; ++e )
    {
        virt_flash_this_entry[e] = &s[e];
    }

    fprintf( stdout, "Virtual FLASH editor\n" );

    bool running = true;

    while (running)
    {
        CMDOption const cmdopt = virt_flash_prompt();

        switch (cmdopt)
        {
            case CMD_OPTION_HEADER:
                virt_flash_header();
                break;
            case CMD_OPTION_ENTRY:
                virt_flash_entry();
                break;
            case CMD_OPTION_LIST:
                // TODO list
                break;
            case CMD_OPTION_NEW:
                // TODO check modified - prompt save
                cmd_flash_new();
                break;
            case CMD_OPTION_RELOAD:
                cmd_flash_reload();
                break;
            case CMD_OPTION_WRITE:
                cmd_flash_save();
                cmd_flash_write( fo );
                break;
            case CMD_OPTION_QUIT:
                // TODO check modified - prompt save
                running = 0;
                break;
            case CMD_OPTION_INVALID:
            default:
                break;
        }
    }

    if (fi)
    {
        fclose( fi );
    }

    if (fo)
    {
        fclose( fo );
    }

    return ret;
}

int main( int argc, char * argv[] )
{
    bool const en   = (argc > 1) ? false : true;
    bool en_bat     = en;
    bool en_cli     = en;
    bool en_profile = en;
    bool en_temp    = en;

    for ( int a = 1; a < argc; ++a )
    {

        if (strcmp( argv[a], "--cli:i" ) == 0)
        {
            i_cli();
            return EXIT_SUCCESS;
        }

        if (strcmp( argv[a], "--main" ) == 0)
        {
            return bist_main();
        }

        if (strcmp( argv[a], "--virt_flash" ) == 0)
        {
            return virt_flash( argc, argv );
        }

        if (strcmp( argv[a], "+bat" ) == 0)
        {
            en_bat = true;
        }

        if (strcmp( argv[a], "+cli" ) == 0)
        {
            en_cli = true;
        }

        if (strcmp( argv[a], "+profile" ) == 0)
        {
            en_profile = true;
        }

        if (strcmp( argv[a], "+temp" ) == 0)
        {
            en_temp = true;
        }
    }

    if (en_bat)
    {
        bist_bat();
    }

    if (en_cli)
    {
        bist_cli();
    }

    if (en_profile)
    {
        bist_profile();
    }

    if (en_temp)
    {
        bist_temp();
    }

    return EXIT_SUCCESS;
(void)argc;
(void)argv;
}
