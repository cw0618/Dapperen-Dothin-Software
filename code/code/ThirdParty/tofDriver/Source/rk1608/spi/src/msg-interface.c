#include "spi\inc\msg-interface.h"
#include "windows.h"


void* iic_handle = NULL;
void* gpio_handle = NULL;
post_iic_write iic_write_callback = NULL;
post_iic_read iic_read_callback = NULL;
post_gpio gpio_callback = NULL;

void set_iic_write_callback(post_iic_write cb)
{
	iic_write_callback = cb;
}
void set_iic_read_callback(post_iic_read cb)
{
	iic_read_callback = cb;
}

void set_gpio_callback(post_gpio cb)
{
	gpio_callback = cb;
}

int i2c_reg_read_ex(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size)
{
	if (iic_read_callback == NULL) {
		//qDebug() << "null iic handle";
		return -1;
	}
	return iic_read_callback(addr, reg, reg_size, data, data_size);

}

int i2c_reg_write_ex(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size)
{
	if (iic_write_callback == NULL) {
		printf("null iic handle");
		return -1;
	}
	return iic_write_callback(addr, reg, reg_size, data, data_size);

}

int gpio_control(int pin, bool level)
{
	if (gpio_callback == NULL) {
		//qDebug() << "null iic handle";
		return -1;
	}
	return gpio_callback(pin, level);
}

post_spi_block_write spi_block_write_callback = NULL;
post_spi_block_read spi_block_read_callback = NULL;
post_spi_block_rw spi_block_rw_callback = NULL;

void set_spi_block_read_callback(post_spi_block_read cb)
{
	spi_block_read_callback = cb;
}

void set_spi_block_write_callback(post_spi_block_write cb)
{
	spi_block_write_callback = cb;
}

void set_spi_block_rw_callback(post_spi_block_rw cb)
{
	spi_block_rw_callback = cb;
}

int spi_block_read(uint32_t addr, uint8_t *data, uint32_t data_size)
{
	if (spi_block_read_callback == NULL) {
		printf("null spi handle");
		return -1;
	}
	return spi_block_read_callback(addr, data, data_size);
}

int spi_block_write(uint32_t addr, uint8_t *data, uint32_t data_size)
{
	if (spi_block_write_callback == NULL) {
		//qDebug() << "null iic handle";
		return -1;
	}
	return spi_block_write_callback(addr, data, data_size);
}

int spi_block_rw(uint8_t *tx, uint8_t *rx, uint32_t tx_len, uint32_t rx_len)
{
	if (spi_block_rw_callback == NULL) {
		//qDebug() << "null iic handle";
		return -1;
	}
	return spi_block_rw_callback(tx, rx, tx_len, rx_len);
}


int set_duxin_access_flash()
{
	int ret;
	ret = gpio_control(1, 1);
	ret = gpio_control(2, 0); 
	// 或许需要断开跳线帽或者0欧电阻, 两个 master 是否能同时接一个flash ?
	return ret;
}

// rk1608 default is in spi master mode
int set_rk1608_access_flash()
{
	int ret;
	ret = gpio_control(1, 0);
	ret = gpio_control(2, 1);
	
	return ret;
}

// rk1608 should enter spi slave mode, this is done in Dothinkey.cpp
int set_duxin_access_rk1608()
{
	int ret;
	ret = gpio_control(1, 1);
	ret = gpio_control(2, 1);

	return ret;
}

#define DUXIN_GPIO_EXPANDER1_ADDR			(0x20 << 1) // TCA6408A
#define DUXIN_GPIO_EXPANDER2_ADDR			(0x21 << 1)
int rk1608_reset()
{
	int ret = 0;

	ret = i2c_reg_write_ex(DUXIN_GPIO_EXPANDER2_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
	ret = i2c_reg_write_ex(DUXIN_GPIO_EXPANDER2_ADDR, 0x01, 1, 0x00, 1); // set all gpio to low
	printf("rk1608_reset ret = %d", ret);
	Sleep(10);
	ret = i2c_reg_write_ex(DUXIN_GPIO_EXPANDER2_ADDR, 0x01, 1, 0x01, 1); // set P0 to high

	return ret;
}

enum gpio_expander1 {

	RK1608_SLEEP = 0x80,
	RK1608_IRQ = 0x40,
	RK1608_WAKEUP = 0x20,
	RK1608_TRIG = 0x10,
	IR_VDDA_2V8_EN = 0x08,
	IR_VDDIO_1V8_EN = 0x04,
	LD_3V6_EN = 0x02,
	V3P3_LD_EN = 0x01
};

int sensor_power_on()
{
	int ret = 0;
	uint8_t set_io = V3P3_LD_EN | LD_3V6_EN | IR_VDDIO_1V8_EN | IR_VDDA_2V8_EN;
	ret = i2c_reg_write_ex(DUXIN_GPIO_EXPANDER1_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
	ret = i2c_reg_write_ex(DUXIN_GPIO_EXPANDER1_ADDR, 0x01, 1, set_io, 1); // set selected gpio to high

	return ret;
}
