#ifndef MLX75027_MAX96717_MAX96716A_H
#define MLX75027_MAX96717_MAX96716A_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define mlx75027_max96717_max96716a_sensor_id 0x5027

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
#define MAX96716A_ADDR       (0x98)     // << 1
#define SENSOR_ADDR          (0x57 << 1)
#define MAX96717_ADDR        (0x80)
#define TMP112_ADDR          (0x90)
#define MCU_ADDR             (0xA2)
/* MCU Opcode definitions */
#define OP_GET_DEVICE_ID                            0x51
#define OP_GET_APP_VERSION                          0x52
#define OP_SET_FREQ1                                0x53
#define OP_SET_FREQ2                                0x54
#define OP_SET_FPS                                  0x55
#define OP_GET_FPS                                  0x56
#define OP_SET_DELAY                                0x57
#define OP_SET_DUTY1                                0x58
#define OP_SET_DUTY2                                0x59
#define OP_SET_START                                0x60
#define OP_SET_STOP                                 0x61

#define TMP112_TEMP_REG      0x00
#define TMP112_CONF_REG      0x01
#define TMP112_TLOW_REG      0x02
#define TMP112_THIGH_REG     0x03

#define TMP112_CONF_SD       0x0001
#define TMP112_CONF_TM       0x0010
#define TMP112_CONF_POL      0x0020
#define TMP112_CONF_F0       0x0080
#define TMP112_CONF_F1       0x0100
#define TMP112_CONF_R0       0x0200
#define TMP112_CONF_R1       0x0400
#define TMP112_CONF_OS       0x0800
#define TMP112_CONF_EM       0x1000
#define TMP112_CONF_AL       0x2000
#define TMP112_CONF_CR0      0x4000
#define TMP112_CONF_CR1      0x8000

//#define TMP112_CONFIG        (TMP112_CONF_CR1 | TMP112_CONF_CR0 | TMP112_CONF_TM)
#define TMP112_CONFIG        (TMP112_CONF_TM)

#define DAC5574_ADDR         (0x4C << 1) // 0x98
#define TEMP_RX_ADDR         (0x48 << 1)
#define TEMP_TX_ADDR         (0x48 << 1)
#define EEPROM_ADDR          (0x50 << 1)
#define CXA4016_IIC_ADDR     (0x22 << 1) // 0x44, visit quad cxa4016 via stm32 i2c

#define LED1_RES_ADDR        (0x2E << 1)
#define LED2_RES_ADDR        (0x3E << 1)
#define LED_RES_COMMAND      (0x00)

//GT24P128E EEPROM, page size 64 Byte, total size 32K Byte
#define GT24P128E_EEPROM_I2C_ADDR       0xA0
#define GT24P128E_EEPROM_PAGE_SIZE      64

int mlx75027_max96717_max96716a_init();
int mlx75027_max96717_max96716a_hardware_trigger();
int mlx75027_max96717_max96716a_software_trigger();
int mlx75027_max96717_max96716a_video_streaming(bool enable);
int mlx75027_max96717_max96716a_get_fps(uint8_t *fps);
int mlx75027_max96717_max96716a_set_fps(uint8_t fps);													

int mlx75027_max96717_max96716a_get_sensor_id(uint16_t *id);
int mlx75027_max96717_max96716a_get_sensor_temperature(float *temp);
int mlx75027_max96717_max96716a_get_rx_temp(float *temperature);
int mlx75027_max96717_max96716a_get_tx_temp(float *temperature);
int mlx75027_max96717_max96716a_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
int mlx75027_max96717_max96716a_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
int mlx75027_max96717_max96716a_illum_power_control(bool enable);


int mlx75027_max96717_max96716a_get_integration_time(uint16_t *integrationTime);
int mlx75027_max96717_max96716a_set_integration_time(uint16_t integrationTime);
int mlx75027_max96717_max96716a_get_modulation_frequency(uint16_t *modFreq);
int mlx75027_max96717_max96716a_set_modulation_frequency(uint16_t modFreq);
int mlx75027_max96717_max96716a_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list);																				 
int mlx75027_max96717_max96716a_get_illum_duty_cycle(uint16_t *duty);
int mlx75027_max96717_max96716a_set_illum_duty_cycle(uint16_t duty);
int mlx75027_max96717_max96716a_get_data_output_mode(uint8_t *mode);
int mlx75027_max96717_max96716a_set_data_output_mode(uint8_t mode);
int mlx75027_max96717_max96716a_get_img_mirror_flip(uint8_t *mode);
int mlx75027_max96717_max96716a_set_img_mirror_flip(uint8_t mode);
int mlx75027_max96717_max96716a_get_pixel_binning(uint8_t *mode);
int mlx75027_max96717_max96716a_set_pixel_binning(uint8_t mode);
int mlx75027_max96717_max96716a_test_pattern(uint8_t mode);
int mlx75027_max96717_max96716a_func_init();



#ifdef __cplusplus
}
#endif


#endif // MLX75027_MAX96717_MAX96716A_H
