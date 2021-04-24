
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

void cli_process_bool( void const * entry, char const c );

void cli_register_variable_ro( char const * name, void const * address, uint32_t const size, void (* fn)( void const * entry, char const c ), void * info );

void cli_register_variable_rw( char const * name, void       * address, uint32_t const size, void (* fn)( void const * entry, char const c ), void * info );

void cli_register_variable_wo( char const * name, void       * address, uint32_t const size, void (* fn)( void const * entry, char const c ), void * info );

void cli_process( char const c );

#endif
