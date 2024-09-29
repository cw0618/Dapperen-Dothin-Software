#ifndef MLX75027_H
#define MLX75027_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define mlx75027_sensor_id 0x5027

#define DEBUG_MLX75027_IN_QT          0   // should set to zero when use in SDK

#if DEBUG_MLX75027_IN_QT
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
int temp_write_reg(uint8_t addr, uint8_t reg, uint16_t value);
int temp_read_reg(uint8_t addr, uint8_t reg, uint16_t *value);
int dac5574_write_reg(uint8_t reg, uint16_t value);
int dac5574_read_reg(uint8_t reg, uint16_t *value);
int eeprom_write_byte(uint32_t reg, uint32_t value);
int eeprom_read_byte(uint32_t reg, uint32_t *value);
#endif

#define IMAGE_WIDTH          640
#define IMAGE_HEIGHT         480
#define IMAGE_RESOLUTION     (IMAGE_WIDTH*IMAGE_HEIGHT)
#define PIXEL_BITS           12

#define DUTY_CYCLE_LIST_NUM  31					

// GPIO control
#define TRIGGER_PIN          2 // PO1
#define LD_ENABLE_PIN        1 // PO2
#define LD_ERROR_PIN         3

// IIC slave device
#define SENSOR_ADDR          (0x57 << 1)
#define DAC5574_ADDR         (0x4C << 1) // 0x98
#define TEMP_RX_ADDR         (0x48 << 1)
#define TEMP_TX_ADDR         (0x4A << 1)
#define EEPROM_ADDR          (0x50 << 1)
#define CXA4016_IIC_ADDR     (0x22 << 1) // 0x44, visit quad cxa4016 via stm32 i2c





int mlx75027_init();
int mlx75027_hardware_trigger();
int mlx75027_software_trigger();
int mlx75027_video_streaming(bool enable);
int mlx75027_get_fps(uint8_t *fps);
int mlx75027_set_fps(uint8_t fps);													

int mlx75027_get_sensor_id(uint16_t *id);
int mlx75027_get_sensor_temperature(float *temp);
int mlx75027_get_rx_temp(float *temperature);
int mlx75027_get_tx_temp(float *temperature);
int mlx75027_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int mlx75027_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int mlx75027_illum_power_control(bool enable);


int mlx75027_get_integration_time(uint16_t *integrationTime);
int mlx75027_set_integration_time(uint16_t integrationTime);
int mlx75027_get_modulation_frequency(uint16_t *modFreq);
int mlx75027_set_modulation_frequency(uint16_t modFreq);
int mlx75027_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list);																				 
int mlx75027_get_illum_duty_cycle(uint16_t *duty);
int mlx75027_set_illum_duty_cycle(uint16_t duty);
int mlx75027_get_data_output_mode(uint8_t *mode);
int mlx75027_set_data_output_mode(uint8_t mode);
int mlx75027_get_img_mirror_flip(uint8_t *mode);
int mlx75027_set_img_mirror_flip(uint8_t mode);
int mlx75027_get_pixel_binning(uint8_t *mode);
int mlx75027_set_pixel_binning(uint8_t mode);
int mlx75027_test_pattern(uint8_t mode);
int mlx75027_func_init();

/*
int mlx75027_set_user_ID(uint8_t value);
int mlx75027_set_trigger_mode(uint8_t mode);
int mlx75027_set_stream_mode();


int mlx75027_sensor_initialize();
int mlx75027_shadow_register(bool enable);
int mlx75027_get_pixel_error_count_low(uint8_t phase, uint32_t *value);
int mlx75027_get_pixel_error_count_high(uint8_t phase, uint32_t *value);
int mlx75027_pixel_binning(uint8_t mode);
int mlx75027_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
int mlx75027_set_frame_startup_time(uint16_t startupTime);
int mlx75027_set_phase_count(uint8_t phaseCount);
int mlx75027_set_phase_pretime(uint16_t preTime);
int mlx75027_set_phase_idle_time(uint8_t value);

int mlx75027_pixel_statistics(bool enable, uint8_t mode);
int mlx75027_set_pixel_lower_limit(uint16_t value);
int mlx75027_set_pixel_upper_limit(uint16_t value);
int mlx75027_get_illum_duty_cycle_range(uint8_t mod_freq, uint8_t *min, uint8_t *max);
int mlx75027_illum_duty_cycle_adjust(uint8_t mode, uint8_t value);
int mlx75027_illum_signal(uint8_t mode);
int mlx75027_metadata_output(uint8_t mode);
*/


#ifdef __cplusplus
}
#endif


#endif // MLX75027_H
