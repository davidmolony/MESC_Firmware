
#ifndef MESC_UTIL_NTC_H
#define MESC_UTIL_NTC_H

#include <stddef.h>

struct NTCNode
{
    float   T;
    float   R;
};

typedef struct NTCNode NTCNode;

extern NTCNode const * const ntc_T_R;
extern size_t  const         ntc_T_R_count;

#endif
