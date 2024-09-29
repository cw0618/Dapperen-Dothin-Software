#ifndef S5K33D_CONFIG_H_
#define S5K33D_CONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

struct reglist {
    uint16_t reg;
    uint16_t val;
};


/* first byte is platform/project, second byte is version
 * 0x01: CX3
 * 0x02: Dothin
 * 0x03: Phone Customer
*/
#define S5K33D_CX3                   0x0101

#define CURRENT_USED_SETTING_CX3        S5K33D_CX3

#ifdef __cplusplus
}
#endif

#endif