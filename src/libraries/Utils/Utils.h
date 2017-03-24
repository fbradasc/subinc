#pragma once

#ifndef constraint
#define constraint(v,min,max) (((v)<(min))?(min):(((v)>(max))?(max):(v)))
#endif

#ifndef linearmap
#define linearmap(x, imin, imax, omin, omax) (((x) - (imin)) * ((omax) - (omin)) / ((imax) - (imin)) + (omin))
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

typedef int16_t          pulse_width_t;
typedef uint64_t         timestamp_t;

#define PULSE_WIDTH_BITS 15
#define PULSE_WIDTH_MIN  0x0000
#define PULSE_WIDTH_ERR  ( ( 1 << ( PULSE_WIDTH_BITS + 1 ) ) - 1 )
#define PULSE_WIDTH_MAX  ( ( 1 << ( PULSE_WIDTH_BITS + 0 ) ) - 1 )
