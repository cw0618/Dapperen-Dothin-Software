#ifndef RK1608_H
#define RK1608_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RK1608_BOARD_ID                 0x00001608
#define RK1608_PLECO_BOARD_ID           0x11231608
#define RK1608_S5K33DXX_BOARD_ID        0x313D1608

#define RK1608_REF_BUFFER_ADDR          0x6A000000

extern int rk1608_init();
extern int rk1608_pleco_init();
extern int rk1608_S5K33dxx_init();



#ifdef __cplusplus
}
#endif
#endif // RK1608_H
