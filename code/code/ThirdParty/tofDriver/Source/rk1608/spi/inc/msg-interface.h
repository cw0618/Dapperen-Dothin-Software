#ifndef MSG_INTERFACE_H
#define MSG_INTERFACE_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int(*post_iic_write)(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size);
extern post_iic_write iic_write_callback;
void set_iic_write_callback(post_iic_write cb);

typedef int(*post_iic_read)(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size);
extern post_iic_read iic_read_callback;
void set_iic_read_callback(post_iic_read cb);

typedef int(*post_gpio)(int pin, bool level);
extern post_gpio gpio_callback;
void set_gpio_callback(post_gpio cb);

int i2c_reg_read_ex(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size);
int i2c_reg_write_ex(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size);
int gpio_control(int pin, bool level);


typedef int(*post_spi_block_write)(uint32_t addr, uint8_t *data, uint32_t data_size);
extern post_spi_block_write spi_block_write_callback;
void set_spi_block_write_callback(post_spi_block_write cb);

typedef int(*post_spi_block_read)(uint32_t addr, uint8_t *data, uint32_t data_size);
extern post_spi_block_read spi_block_read_callback;
void set_spi_block_read_callback(post_spi_block_read cb);

typedef int(*post_spi_block_rw)(uint8_t *tx, uint8_t *rx, uint32_t tx_len, uint32_t rx_len);
extern post_spi_block_rw spi_block_rw_callback;
void set_spi_block_rw_callback(post_spi_block_rw cb);

int spi_block_read(uint32_t addr, uint8_t *data, uint32_t data_size);
int spi_block_write(uint32_t addr, uint8_t *data, uint32_t data_size);
int spi_block_rw(uint8_t *tx, uint8_t *rx, uint32_t tx_len, uint32_t rx_len);


int set_duxin_access_flash();
int set_rk1608_access_flash();
int set_duxin_access_rk1608();
int rk1608_reset();
int sensor_power_on();


#ifdef __cplusplus
}
#endif


#endif
