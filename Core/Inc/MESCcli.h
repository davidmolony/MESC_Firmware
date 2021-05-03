
#ifndef MESC_CLI_H
#define MESC_CLI_H

#include <stdint.h>

/*
C VVVV vvvv
^
R Read
W Write

  ^..^
  variable name

       ^..^
       value
*/

void cli_process_int( char const c );

void cli_process_uint( char const c );

void cli_process_float( char const c );

void cli_process_enum( char const c );

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    void (* fn)( char const c ) );

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    void (* fn)( char const c ) );

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    void (* fn)( char const c ) );

void cli_process( char const c );

#endif
