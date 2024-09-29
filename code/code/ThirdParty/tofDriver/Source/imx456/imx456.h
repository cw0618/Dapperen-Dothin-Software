#ifndef IMX456_H
#define IMX456_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define imx456_sensor_id     0x0456


#define DEBUG_IMX456_IN_QT          0   // should set to zero when use in SDK

#if DEBUG_IMX456_IN_QT
extern void* iic_handle;
typedef int (*post_iic_write)(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size, void* handle);
extern post_iic_write iic_write_callback;
void set_iic_write_callback(post_iic_write cb, void* handle);

typedef int (*post_iic_read)(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size, void* handle);
extern post_iic_read iic_read_callback;
void set_iic_read_callback(post_iic_read cb, void* handle);

typedef int (*post_gpio)(int pin, bool level, void* handle);
extern post_gpio gpio_callback;
void set_gpio_callback(post_gpio cb, void* handle);


/**/
int sensor_write_reg(uint16_t reg, uint8_t value);
int sensor_read_reg(uint16_t reg, uint8_t *value);

int eeprom_write_byte(uint16_t reg, uint8_t value);
int eeprom_read_byte(uint16_t reg, uint8_t *value);


#define IMAGE_WIDTH          640
#define IMAGE_HEIGHT         480
#define IMAGE_RESOLUTION     (IMAGE_WIDTH*IMAGE_HEIGHT)
#define PIXEL_BITS           12

#define DUTY_CYCLE_LIST_NUM  31

#else

#if 0
/* 定义SENSOR需要的电源类型 */
///定义SENSOR需要的电源类型。
typedef enum {
    /* A通道，或只有一个通道时 */
    POWER_AVDD = 0,			///<AVDD
    POWER_DOVDD = 1,		///<DOVDD
    POWER_DVDD = 2,			///<DVDD
    POWER_AFVCC = 3,		///<AFVCC
    POWER_VPP = 4,			///<VPP

    /* B通道,(B通道电源定义，只有UH920使用) */
    POWER_AVDD_B = 5,		///<B通道AVDD
    POWER_DOVDD_B = 6,		///<B通道DOVDD
    POWER_DVDD_B = 7,		///<B通道DVDD
    POWER_AFVCC_B = 8,		///<B通道AFVCC
    POWER_VPP_B = 9,		///<B通道VPP

    /* 新增加的电源通道定义 */
    POWER_OISVDD = 10,
    POWER_AVDD2 = 11,
    POWER_AUX1 = 12,
    POWER_AUX2 = 13,
    POWER_VPP2 = 14
}SENSOR_POWER;
#endif

#endif



int imx456_init();
int imx456_hardware_trigger();
int imx456_software_trigger();
int imx456_video_streaming(bool enable);
int imx456_get_fps(uint8_t *fps);
int imx456_set_fps(uint8_t fps);

int imx456_get_sensor_id(uint16_t *id);
int imx456_get_sensor_temperature(float *temp);
int imx456_get_rx_temp(float *temperature);
int imx456_get_tx_temp(float *temperature);
int imx456_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int imx456_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int imx456_illum_power_control(bool enable);
int imx456_get_vcsel_pd(uint32_t *value);
int imx456_illum_power_test(uint8_t mode);
int imx456_get_integration_time(uint16_t *integrationTime);
int imx456_set_integration_time(uint16_t integrationTime);
int imx456_get_modulation_frequency(uint16_t *modFreq);
int imx456_set_modulation_frequency(uint16_t modFreq);
int imx456_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list);
int imx456_get_illum_duty_cycle(uint16_t *duty);
int imx456_set_illum_duty_cycle(uint16_t duty);
int imx456_get_data_output_mode(uint8_t *mode);
int imx456_set_data_output_mode(uint8_t mode);
int imx456_get_img_mirror_flip(uint8_t *mode);
int imx456_set_img_mirror_flip(uint8_t mode);
int imx456_get_pixel_binning(uint8_t *mode);
int imx456_set_pixel_binning(uint8_t mode);
int imx456_test_pattern(uint8_t mode);
int imx456_func_init();

/*
int imx456_set_user_ID(uint8_t value);
int imx456_set_trigger_mode(uint8_t mode);
int imx456_set_stream_mode();


int imx456_sensor_initialize();
int imx456_shadow_register(bool enable);
int imx456_get_pixel_error_count_low(uint8_t phase, uint32_t *value);
int imx456_get_pixel_error_count_high(uint8_t phase, uint32_t *value);
int imx456_pixel_binning(uint8_t mode);
int imx456_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
int imx456_set_frame_startup_time(uint16_t startupTime);
int imx456_set_phase_count(uint8_t phaseCount);
int imx456_set_phase_pretime(uint16_t preTime);
int imx456_set_phase_idle_time(uint8_t value);

int imx456_pixel_statistics(bool enable, uint8_t mode);
int imx456_set_pixel_lower_limit(uint16_t value);
int imx456_set_pixel_upper_limit(uint16_t value);
int imx456_get_illum_duty_cycle_range(uint8_t mod_freq, uint8_t *min, uint8_t *max);
int imx456_illum_duty_cycle_adjust(uint8_t mode, uint8_t value);

int imx456_metadata_output(uint8_t mode);
*/


#ifdef __cplusplus
}
#endif


#endif // IMX456_H
