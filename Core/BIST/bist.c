
#include <stdlib.h>

extern void bist_bat( void );
extern void bist_cli( void );
extern void bist_profile( void );
extern void bist_speed( void );
extern void bist_temp( void );

int main( int argc, char * argv[] )
{
    bist_bat();
    bist_cli();
    bist_profile();
    bist_speed();
    bist_temp();

    return EXIT_SUCCESS;
(void)argc;
(void)argv;
}
