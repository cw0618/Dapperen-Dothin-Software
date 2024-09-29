#ifndef IMX516_H
#define IMX516_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define imx516_sensor_id   0x0516

#define DEBUG_IMX516_IN_QT          0   // should set to zero when use in SDK

#if DEBUG_IMX516_IN_QT
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


enum iic_device {sensor, rx_temp, tx_temp, dac5574, eeprom};
/**/
int sensor_write_reg(uint32_t reg, uint32_t value);
int sensor_read_reg(uint32_t reg, uint32_t *value);

int eeprom_write_byte(uint32_t reg, uint32_t value);
int eeprom_read_byte(uint32_t reg, uint32_t *value);


#define IMAGE_WIDTH          640
#define IMAGE_HEIGHT         3864
#define IMAGE_RESOLUTION     (IMAGE_WIDTH*IMAGE_HEIGHT)
#define PIXEL_BITS           12
#define DUTY_CYCLE_LIST_NUM  31

#else
#if 0	
/* ����SENSOR��Ҫ�ĵ�Դ���� */
///����SENSOR��Ҫ�ĵ�Դ���͡�
typedef enum {
	/* Aͨ������ֻ��һ��ͨ��ʱ */
	POWER_AVDD = 0,			///<AVDD
	POWER_DOVDD = 1,		///<DOVDD
	POWER_DVDD = 2,			///<DVDD
	POWER_AFVCC = 3,		///<AFVCC
	POWER_VPP = 4,			///<VPP

	/* Bͨ��,(Bͨ����Դ���壬ֻ��UH920ʹ��) */
	POWER_AVDD_B = 5,		///<Bͨ��AVDD
	POWER_DOVDD_B = 6,		///<Bͨ��DOVDD
	POWER_DVDD_B = 7,		///<Bͨ��DVDD
	POWER_AFVCC_B = 8,		///<Bͨ��AFVCC
	POWER_VPP_B = 9,		///<Bͨ��VPP

	/* �����ӵĵ�Դͨ������ */
	POWER_OISVDD = 10,
	POWER_AVDD2 = 11,
	POWER_AUX1 = 12,
	POWER_AUX2 = 13,
	POWER_VPP2 = 14
}SENSOR_POWER;
#endif
#endif


int imx516_init();
int imx516_hardware_trigger();
int imx516_software_trigger();
int imx516_video_streaming(bool enable);
int imx516_get_fps(uint8_t *fps);
int imx516_set_fps(uint8_t fps);

int imx516_get_sensor_id(uint16_t *id);
int imx516_get_sensor_temperature(float *temp);
int imx516_get_rx_temp(float *temperature);
int imx516_get_tx_temp(float *temperature);
int imx516_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int imx516_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int imx516_illum_power_control(bool enable);


int imx516_get_integration_time(uint16_t *integrationTime);
int imx516_set_integration_time(uint16_t integrationTime);
int imx516_get_modulation_frequency(uint16_t *modFreq);
int imx516_set_modulation_frequency(uint16_t modFreq);
int imx516_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list);
int imx516_get_illum_duty_cycle(uint8_t *duty);
int imx516_set_illum_duty_cycle(uint8_t duty);
int imx516_get_data_output_mode(uint8_t *mode);
int imx516_set_data_output_mode(uint8_t mode);
int imx516_get_img_mirror_flip(uint8_t *mode);
int imx516_set_img_mirror_flip(uint8_t mode);
int imx516_test_pattern(uint8_t mode);
int imx516_func_init();

/*
int imx516_set_user_ID(uint8_t value);
int imx516_set_trigger_mode(uint8_t mode);
int imx516_set_stream_mode();


int imx516_sensor_initialize();
int imx516_shadow_register(bool enable);
int imx516_get_pixel_error_count_low(uint8_t phase, uint32_t *value);
int imx516_get_pixel_error_count_high(uint8_t phase, uint32_t *value);
int imx516_pixel_binning(uint8_t mode);
int imx516_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
int imx516_set_frame_startup_time(uint16_t startupTime);
int imx516_set_phase_count(uint8_t phaseCount);
int imx516_set_phase_pretime(uint16_t preTime);
int imx516_set_phase_idle_time(uint8_t value);
int imx516_set_phase_led_enable_pulse(uint8_t phaseLedEn);
int imx516_pixel_statistics(bool enable, uint8_t mode);
int imx516_set_pixel_lower_limit(uint16_t value);
int imx516_set_pixel_upper_limit(uint16_t value);
int imx516_get_illum_duty_cycle_range(uint8_t mod_freq, uint8_t *min, uint8_t *max);
int imx516_illum_duty_cycle_adjust(uint8_t mode, uint8_t value);
int imx516_illum_signal(uint8_t mode);
int imx516_metadata_output(uint8_t mode);
*/


#ifdef __cplusplus
}
#endif


#endif // IMX516_H
