#ifndef pleco_H
#define pleco_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif


#define pleco_sensor_id      0x0123



int pleco_sensor_init();
int pleco_hardware_trigger();
int pleco_software_trigger();
int pleco_video_streaming(bool enable);
int pleco_get_fps(uint8_t *fps);
int pleco_set_fps(uint8_t fps);

int pleco_get_sensor_id(uint16_t *id);
int pleco_get_rx_temp(float *temperature);
int pleco_get_tx_temp(float *temperature);
int pleco_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int pleco_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int pleco_illum_power_control(bool enable);

int pleco_get_integration_time(uint16_t *integrationTime);
int pleco_set_integration_time(uint16_t integrationTime);
int pleco_get_modulation_frequency(uint16_t *modFreq);
int pleco_set_modulation_frequency(uint16_t modFreq);
int pleco_get_illum_duty_cycle(uint16_t *duty);
int pleco_set_illum_duty_cycle(uint16_t duty);
int pleco_get_data_output_mode(uint8_t *mode);
int pleco_set_data_output_mode(uint8_t mode);
int pleco_get_img_mirror_flip(uint8_t *mode);
int pleco_set_img_mirror_flip(uint8_t mode);
int pleco_test_pattern(uint8_t mode);
int pleco_func_init();


#ifdef __cplusplus
}
#endif


#endif // MLX75027_H
