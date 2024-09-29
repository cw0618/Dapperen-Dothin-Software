#ifndef __RK1608_PLECO__
#define __RK1608_PLECO__
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RK1608_PLECO_SENSOR_ID      (0x1123)
#define	RK1608_PLECO_FREQ_MODE      (1)

	extern int rk1608_pleco_func_load();
	extern void rk1608_pleco_set_phase_src_info();
	extern void rk1608_pleco_set_depth_src_info();
	extern int rk1608_pleco_get_firmware(uint8_t fw_type, const uint8_t **get_fireware_data);
	extern int set_rk1608_pleco_raw_phase_out();
	extern int set_rk1608_pleco_depth_out();
	extern int rk1608_pleco_gpio_expander_init();

#ifdef __cplusplus
}
#endif


#endif
