
#ifndef MESC_BIT_OP_H
#define MESC_BIT_OP_H

#define BITS_PER_BYTE   UINT32_C(8)
#define BITS_PER_NYBBLE UINT32_C(4)

#define BIT_MASK_32(bits)  (((UINT32_C(1) - ((bits) / 32)) << ((bits) & 31)) - UINT32_C(1))

#endif
