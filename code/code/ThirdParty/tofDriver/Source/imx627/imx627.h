#ifndef IMX627_H
#define IMX627_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define imx627_sensor_id   0x0627

#define IMVCK 2400 //MHZ


#define IMX627_REG_LIST_6272        1 // normal used setting
#define IMX627_REG_LIST_6130        0 // QC and AA used setting
#define IMX627_REG_LIST_SELECT      1



int imx627_init();
int imx627_hardware_trigger();
int imx627_software_trigger();
int imx627_video_streaming(bool enable);
int imx627_get_fps(uint8_t *fps);
int imx627_set_fps(uint8_t fps);

int imx627_get_sensor_id(uint16_t *id);
int imx627_get_rx_temp(float *temperature);
int imx627_get_tx_temp(float *temperature);
int imx627_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int imx627_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int imx627_illum_power_control(bool enable);


int imx627_get_integration_time(uint16_t *integrationTime);
int imx627_set_integration_time(uint16_t integrationTime);
int imx627_get_modulation_frequency(uint16_t *modFreq);
int imx627_set_modulation_frequency(uint16_t modFreq);
int imx627_get_data_output_mode(uint8_t *mode);
int imx627_set_data_output_mode(uint8_t mode);
int imx627_get_img_mirror_flip(uint8_t *mode);
int imx627_set_img_mirror_flip(uint8_t mode);
int imx627_test_pattern(uint8_t mode);
int imx627_func_init();





#ifdef __cplusplus
}
#endif


#endif // MLX75027_H
