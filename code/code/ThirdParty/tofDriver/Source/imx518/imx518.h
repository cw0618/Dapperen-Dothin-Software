#ifndef IMX518_H
#define IMX518_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define imx518_sensor_id   0x0518

int imx518_init();
int imx518_hardware_trigger();
int imx518_software_trigger();
int imx518_video_streaming(bool enable);
int imx518_get_fps(uint8_t *fps);
int imx518_set_fps(uint8_t fps);

int imx518_get_sensor_id(uint16_t *id);
int imx518_get_rx_temp(float *temperature);
int imx518_get_tx_temp(float *temperature);
int imx518_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int imx518_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int imx518_illum_power_control(bool enable);


int imx518_get_integration_time(uint16_t *integrationTime);
int imx518_set_integration_time(uint16_t integrationTime);
int imx518_get_modulation_frequency(uint16_t *modFreq);
int imx518_set_modulation_frequency(uint16_t modFreq);
int imx518_get_data_output_mode(uint8_t *mode);
int imx518_set_data_output_mode(uint8_t mode);
int imx518_get_img_mirror_flip(uint8_t *mode);
int imx518_set_img_mirror_flip(uint8_t mode);
int imx518_test_pattern(uint8_t mode);
int imx518_func_init();





#ifdef __cplusplus
}
#endif


#endif // MLX75027_H
