
#ifndef MESC_CLI_H
#define MESC_CLI_H

#include <stdint.h>

/*
Command             Description
R <NAME>            Read variable <NAME>
W <NAME> <VALUE>    Write variable <NAME> with <VALUE>
X <NAME>            Execute function <NAME>
I <NAME> <VALUE>    Increment variable <NAME> by <VALUE>
D <NAME> <VALUE>    Decrement variable <NAME> by <VALUE>
*/

enum CLIVariableType
{
    CLI_VARIABLE_INT,
    CLI_VARIABLE_UINT,
    CLI_VARIABLE_FLOAT,
};

typedef enum CLIVariableType CLIVariableType;

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type );

void cli_register_function(
    char const * name,
    void (* const fn)( void ) );

void cli_process( char const c );

#endif
