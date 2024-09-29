#ifndef OBUTILS_H
#define OBUTILS_H
#include <stdint.h>
#include <cmath>

#ifndef CELSIUS_TO_KELVIN_OFFSET
#define CELSIUS_TO_KELVIN_OFFSET    (273.16)
#endif


class ObUtils
{
public:
    ObUtils();


public:

//    static stream_tag_t parseStreamTag(void* data);

    static inline float FixPoint2Temperature(uint16_t temp)
    {
       return (float)temp / 10.f - CELSIUS_TO_KELVIN_OFFSET;
    }

    static inline uint16_t Temperature2FixPoint(float temp)
    {
        return (uint16_t) ((int) std::round((temp + CELSIUS_TO_KELVIN_OFFSET) * 10));   // 以0.1k为单位 （273.16)
    }

    static int16_t int12_to_int16(uint16_t raw12)
    {
       return ((raw12 & 0x800) ? (raw12 | 0xF000) : raw12);
    }


};

#endif // OBUTILS_H
