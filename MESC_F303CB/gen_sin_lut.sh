
cat << 'HEAD' > sin_lut.c

#include "sin_lut.h"

const uint8_t g_sin_lut[SINLUT_ENTRIES] =
{//      x0  x1  x2  x3  x4  x5  x6  x7  x8  x9  xA  xB  xC  xD  xE  xF
HEAD

gcc gen_sin_lut.c -lm
./a.out >> sin_lut.c

cat << 'FOOT' >> sin_lut.c
};
FOOT
