#ifndef __RK1608_S5K33DXX_H__
#define __RK1608_S5K33DXX_H__
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RK1608_S5K33D_FREQ_MODE        (1)//rk1608 output with freq setting 0:single freq non-shffule (raw phase=>depth) 1: dual freq non-shffule (2raw phase=>depth)

#define RK1608_S5K33D_SENSOR_ID         0x313D
#define S5K33D_I2C_ADDR				    0x20

extern int set_rk1608_s5k33dxx_raw_phase_out();
extern int set_rk1608_s5k33dxx_depth_out();
extern void rk1608_s5k33dxx_set_phase_src_info();
extern void rk1608_s5k33dxx_set_depth_src_info();
extern int rk1608_s5k33dxx_get_firmware(uint8_t fw_type, const uint8_t **get_fireware_data);
extern int rk1608_s5k33dxx_func_load();

#ifdef __cplusplus
}
#endif
#endif // __RK1608_S5K33DXX_H__
