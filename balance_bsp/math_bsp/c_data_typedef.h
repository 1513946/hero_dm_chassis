#ifndef C_DATA_TYPEDEF_H
#define C_DATA_TYPEDEF_H

/* Use the standard integer types instead of redefining them:
 * the old F4 project typedef'd int8_t/uint8_t/... itself, which conflicts
 * with <stdint.h> that is pulled in by CMSIS/HAL on the H7 project. */
#include <stdint.h>

typedef unsigned char bool_t;
typedef float         fp32;
typedef double        fp64;

#endif
