
OUTFIL=MESCsin_lut.c

cat << 'HEAD' > ${OUTFIL}

#include "MESCsin_lut.h"

uint8_t const g_sin_lut[SINLUT_ENTRIES] =
{//      x0  x1  x2  x3  x4  x5  x6  x7  x8  x9  xA  xB  xC  xD  xE  xF
HEAD

ENVCC=${CC:-gcc}
echo "INFO: Using ${ENVCC}"
${ENVCC} -Wall -Wextra -pedantic -std=c11 -g -ggdb3 -O0 -I../Inc util_sinlut.c -lm -o util_sinlut

./util_sinlut >> ${OUTFIL}

cat << 'FOOT' >> ${OUTFIL}
};
FOOT
