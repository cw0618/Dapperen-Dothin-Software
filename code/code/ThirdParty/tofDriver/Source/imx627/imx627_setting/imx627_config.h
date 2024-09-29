#ifndef IMX627_CONFIG_H_
#define IMX627_CONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

struct reglist {
    uint16_t reg;
    uint8_t val;
};

#ifdef __cplusplus
}
#endif

#endif