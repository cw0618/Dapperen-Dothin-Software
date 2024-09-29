#ifndef S5K33DXX_H
#define S5K33DXX_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define s5k33dxx_sensor_id        0x303D


#define IMAGE_WIDTH          640
#define IMAGE_HEIGHT         480
#define IMAGE_RESOLUTION     (IMAGE_WIDTH*IMAGE_HEIGHT)

	// GPIO control
#define TRIGGER_PIN          2  // PO1
#define LD_ENABLE_PIN        1  // PO2
#define LD_ERROR_PIN         3

	// IIC slave device
#define S5K33D_I2C_ADDR_L    0x20  //S5K33D_TAISHAN
#define S5K33D_I2C_ADDR_H    0x5A  //Rx2003 Module,MeiZu project S5K33D_T200515, F201201 project

	// F201201 TX AA MAX7783 IIC addres
#define ST_MAX7783_ADDR      (0x66 << 1)

	// Taishan FM24C128D EEPROM, page size 64 Byte, total size 16K Byte
#define FM24C128D_EEPROM_PAGE_SIZE       64
#define FM24C128D_CSP_CONFIG_REG         0x06CA
#define FM24C128D_CC_WREN_REG            0x3F35
#define FM24C128D_DATA_MEMORY            0xA0

	// Huangshan(MEIZU) GT24C512B EEPROM, page size 128 Byte, total size 64K Byte
#define GT24C512B_EEPROM_I2C_ADDR        0xA0
#define GT24C512B_EEPROM_PAGE_SIZE       128

	// Rx2003(MEIZU) GT24P256B EEPROM, page size 64 Byte, total size 32K Byte
#define GT24P256B_EEPROM_I2C_ADDR        0xA0
#define GT24P256B_EEPROM_PAGE_SIZE       64

	typedef enum {
		UNUSED_INDEX,
		F201201_TX_AA_INDEX,
		F201201_TX_QC_INDEX,
	} AA_QC_INDEX;

	int s5k33dxx_sensor_init();
	int s5k33dxx_hardware_trigger();
	int s5k33dxx_software_trigger();
	int s5k33dxx_video_streaming(bool enable);
	int s5k33dxx_get_fps(uint8_t *fps);
	int s5k33dxx_set_fps(uint8_t fps);
	int s5k33dxx_AE(bool enable);
	int s5k33dxx_get_sensor_id(uint16_t *id);
	int s5k33dxx_get_rx_temp(float *temperature);
	int s5k33dxx_get_tx_temp(float *temperature);
	int s5k33dxx_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
	int s5k33dxx_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
	int s5k33dxx_get_vcsel_pd(uint32_t *value);
	int s5k33dxx_get_vcsel_error(uint16_t *value);

	int vcsel_driver_read_reg_16(uint16_t reg, uint16_t *value);
	int vcsel_driver_write_reg_16(uint16_t reg, uint16_t value);

	int s5k33dxx_get_integration_time(uint16_t *integrationTime);
	int s5k33dxx_set_integration_time(uint16_t integrationTime);
	int s5k33dxx_get_modulation_frequency(uint16_t *modFreq);
	int s5k33dxx_set_modulation_frequency(uint16_t modFreq);
	int s5k33dxx_get_illum_duty_cycle(uint16_t *duty);
	int s5k33dxx_set_illum_duty_cycle(uint16_t duty);
	int s5k33dxx_get_img_mirror_flip(uint8_t *mode);
	int s5k33dxx_set_img_mirror_flip(uint8_t mode);
	int s5k33dxx_test_pattern(uint8_t mode);
	int s5k33dxx_func_init();
	int fm24c128d_eeprom_write(uint16_t offset, uint8_t *buf, uint16_t size);
	int fm24c128d_eeprom_read(uint16_t offset, uint8_t *buf, uint16_t size);
	int gt24c512b_eeprom_block_write(uint16_t offset, uint8_t *buf, uint16_t size);
	int gt24c512b_eeprom_block_read(uint16_t offset, uint8_t *buf, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif  // s5k33dxx_H
