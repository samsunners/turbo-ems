#ifndef CRANKMATHS_H
#define CRANKMATHS_H

#include "globals.h"
#include "decoders.h"

typedef uint16_t au_t;

inline uint32_t au_to_time(au_t angle)//TODO: Force inline
{
    return  ( ((uint32_t)angle * (uint32_t)10986) / ( (uint32_t)currentStatus.RPM*(uint32_t)6 ) );
}

inline au_t time_to_au(uint32_t t)//TODO: Force inline
{
    return (unsigned int)(((uint32_t)currentStatus.RPM * (uint32_t)6 * (uint32_t)t) / (uint32_t)10986);
}

#define AU_TO_DEGREES(x) ((x)/91)

#define DEGREES_TO_AU(x) ((x)*91)

#endif
