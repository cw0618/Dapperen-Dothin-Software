#ifndef GAEA_H
#define GAEA_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define gaea_sensor_id 0xe000

#define DEBUG_GAEA_IN_QT          0   // should set to zero when use in SDK

#if DEBUG_GAEA_IN_QT
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

#define IMAGE_WIDTH          1200
#define IMAGE_HEIGHT         1922
#define IMAGE_RESOLUTION     (IMAGE_WIDTH*IMAGE_HEIGHT)
#define PIXEL_BITS           10

#define DUTY_CYCLE_LIST_NUM  31

// GPIO control
#define TRIGGER_PIN          2 // PO1
#define LD_ENABLE_PIN        1 // PO2
#define LD_ERROR_PIN         3

// IIC slave device
#define SENSOR_ADDR          (0xCC) //(0xCC << 1)
#define LM3644_ADDR          (0xC6) //(0xC6 << 1)
#define TCA9548_ADDR         (0xE0) //(0xE0 << 1

#define DEBUG_FLAG           (0xDE) //(0xE0 << 1)

#define DEBUG_TYPE_SBG_SELFTEST     (0x01)
#define DEBUG_TYPE_LOAD_DEFCFG      (0x02)

#define GAEA_WORKMODE_NORMAL        (0x00)
#define GAEA_WORKMODE_SL0           (0x01)
#define GAEA_WORKMODE_SL1           (0x02)
#define GAEA_WORKMODE_SL2           (0x03)
#define GAEA_WORKMODE_SL3           (0x04)
#define GAEA_WORKMODE_SL4           (0x05)

int gaea_init();
int gaea_hardware_trigger();
int gaea_software_trigger();
int gaea_video_streaming(bool enable);
int gaea_get_fps(uint8_t *fps);
int gaea_set_fps(uint8_t fps);

int gaea_get_sensor_id(uint16_t *id);
int gaea_get_sensor_temperature(float *temp);
int gaea_get_rx_temp(float *temperature);
int gaea_get_tx_temp(float *temperature);
int gaea_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int gaea_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int gaea_illum_power_control(bool enable);

int gaea_get_integration_time(uint16_t *integrationTime);
int gaea_set_integration_time(uint16_t integrationTime);
int gaea_get_modulation_frequency(uint16_t *modFreq);
int gaea_set_modulation_frequency(uint16_t modFreq);
int gaea_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list);
int gaea_get_illum_duty_cycle(uint16_t *duty);
int gaea_set_illum_duty_cycle(uint16_t duty);
int gaea_get_data_output_mode(uint8_t *mode);
int gaea_set_data_output_mode(uint8_t mode);

int gaea_set_gain(uint32_t gain);
int gaea_get_gain(uint32_t *gain);

int gaea_set_window_originy(uint32_t originy);
int gaea_set_window_originx(uint32_t originx);
int gaea_set_window_height(uint32_t height);
int gaea_get_window_height(uint32_t *height);
int gaea_set_window_width(uint32_t width);
int gaea_get_window_width(uint32_t *width);

int gaea_set_odd_dgain(uint32_t gain);
int gaea_set_even_dgain(uint32_t gain);

int gaea_set_sub_samp(uint8_t value);
int gaea_get_sub_samp(uint8_t *value);

int gaea_get_img_mirror_flip(uint8_t *mode);
int gaea_set_img_mirror_flip(uint8_t mode);
int gaea_get_pixel_binning(uint8_t *mode);
int gaea_set_pixel_binning(uint8_t mode);
int gaea_test_pattern(uint8_t mode);
int gaea_func_init();

#ifdef __cplusplus
}
#endif


#endif // GAEA_H
