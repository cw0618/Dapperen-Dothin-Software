#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "delay_board.h"
#include "temperature_table/temp_conversion_table.h"
#include <tof_sensors.h>
#include <obc_tee_funcs.h>

#ifdef __linux__
#include <unistd.h>
#endif

#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
//#define ALOGE(...)
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread

#define dothin_device_id             tops_t.ap_ops.device_id
#define dothin_set_gpio_level        tops_t.ap_ops.SetGpioPinLevel

#define DX_P01_MCU_RST          2  // PO1, DX_P01_MCU_RST

#define DELAY_BOARD_I2C_ADDR			0x68  // STM32 IIC Slave
#define I2C_M_RD     1
#define I2C_M_WT     0

#define POWER_ON                1
#define POWER_OFF               0

#if 0
typedef struct {
	uint8_t version[4];  //Version: v[0].v[1].v[2].v[3]
	uint8_t power_state;  //0:power off  1:power on
	uint8_t delay_select;  //0:close delay  1:open delay
	uint32_t delay_time;  //delay time range:(0~67452ps)
	double chip1_temp;  //U305 temperature  range:(3~100℃)
	double chip2_temp;  //U306 temperature
	double chip3_temp;  //U307 temperature
	double chip4_temp;  //U309 temperature
	double chip5_temp;  //U310 temperature
	double chip6_temp;  //U311 temperature
}delay_board_t;
#endif


static int gpio_control(int pin, bool level)
{
	dothin_set_gpio_level(pin, level, dothin_device_id);

	return 0;
}

typedef struct i2c_msg {
    uint8_t slave_addr;
    uint8_t rw_mode;
    uint16_t reg;
    uint8_t  reg_size;
    uint32_t* data;
    uint16_t data_size;
}i2c_msg_t;

static int i2c_reg_read(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size)
{
    int ret = 0;
    i2c_msg_t msg;

    msg.slave_addr = addr;
    msg.rw_mode = I2C_M_RD;
    msg.reg = reg;
    msg.reg_size = reg_size;
    msg.data = data;
    msg.data_size = data_size;

    ret = orbbec_i2c_writeread(&msg, 1);

    return ret;
}

static int i2c_reg_write(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size)
{
    int ret = 0;
    i2c_msg_t msg;

    msg.slave_addr = addr;
    msg.rw_mode = I2C_M_WT;
    msg.reg = reg;
    msg.reg_size = reg_size;
    msg.data = &data;
    msg.data_size = data_size;

    ret = orbbec_i2c_writeread(&msg, 1);

    return ret;
}

//0:power off  1:power on
int delay_board_set_power_on_off(uint8_t on_off)
{
	int ret = 0;

	if (on_off == POWER_ON) {
		ret += i2c_reg_write((0x20 << 1), 0x03, 1, 0x00, 1); // set all gpio as output
		ret += i2c_reg_write((0x20 << 1), 0x01, 1, 0xFF, 1); // set all gpio to high

		ret += i2c_reg_write((0x21 << 1), 0x03, 1, 0x00, 1); // set all gpio as output
		ret += i2c_reg_write((0x21 << 1), 0x01, 1, 0xFF, 1); // set all gpio to high

		gpio_control(DX_P01_MCU_RST, false);  //mcu cancel reset

		ALOGE("set delay_board power on!");
	} else if (on_off == POWER_OFF) {
		ret += i2c_reg_write((0x20 << 1), 0x03, 1, 0x00, 1); // set all gpio as output
		ret += i2c_reg_write((0x20 << 1), 0x01, 1, 0x00, 1); // set all gpio to low

		ret += i2c_reg_write((0x21 << 1), 0x03, 1, 0x00, 1); // set all gpio as output
		ret += i2c_reg_write((0x21 << 1), 0x01, 1, 0x00, 1); // set all gpio to low

		ALOGE("set delay_board power off!");
	}

	return ret;
}

//0:power off  1:power on
int delay_board_get_power_on_off(uint8_t *value_p)
{
	int ret = 0;
	uint8_t io_v1 = 0, io_v2 = 0;

	ret += i2c_reg_read((0x20 << 1), 0x01, 1, &io_v1, 1); // get expand gpio ic1 state
	ret += i2c_reg_read((0x21 << 1), 0x01, 1, &io_v2, 1); // get expand gpio ic2 state

	if (io_v1 == 0xff && io_v2 == 0xff) {
		*value_p = POWER_ON;
	} else {
		*value_p = POWER_OFF;
	}

	ALOGE("get delay_board power state: %d\r\n", *value_p);

	return ret;
}

//Version: v[0].v[1].v[2].v[3]
int delay_board_get_firmware_version(uint8_t *value_p)
{
	int ret = 0;
	uint8_t buff[4] = { 0 };

	ret = i2c_reg_read(DELAY_BOARD_I2C_ADDR, 0xfc, 1, buff, 4);

	value_p[0] = buff[0];
	value_p[1] = buff[1];
	value_p[2] = buff[2];
	value_p[3] = buff[3];

	ALOGE("get delay_board firmware version: V%d.%d.%d.%d \r\n", value_p[0], value_p[1], value_p[2], value_p[3]);

	return ret;
}

//0:close delay  1:open delay
int delay_board_set_delay_select(uint8_t state)
{
	int ret = 0;

	uint16_t value = state * 256;

	ret = i2c_reg_write(DELAY_BOARD_I2C_ADDR, 0x00, 1, value, 2);

	ALOGE("set delay_board delay select: %d\r\n", value);

	return ret;
}

//0:close delay  1:open delay
int delay_board_get_delay_selcet(uint8_t *state_p)
{
	int ret = 0;

	ret = i2c_reg_read(DELAY_BOARD_I2C_ADDR, 0x00, 1, state_p, 1);

	ALOGE("get delay_board delay select: %d\r\n", *state_p);

	return ret;
}

//final delay time = num_11ps * 11ps;  e.g.when num_11ps = 10, final delay 110ps;  range:(0~67452ps)
int delay_board_set_delay_time(uint32_t ps)
{
	int ret = 0;
	uint16_t num_11ps = ps / 11;

	ret = i2c_reg_write(DELAY_BOARD_I2C_ADDR, 0x02, 1, num_11ps, 2);

	ALOGE("set delay_board delay time: %d\r\n", num_11ps);

	return ret;
}

//final delay time = num_11ps * 11ps;  e.g.when num_11ps = 10, final delay 110ps;  range:(0~67452ps)
int delay_board_get_delay_time(uint32_t *ps_p)
{
	int ret = 0;
	uint16_t num_11ps = 0;

	ret = i2c_reg_read(DELAY_BOARD_I2C_ADDR, 0x02, 1, &num_11ps, 2);

	*ps_p = 11 * num_11ps;

	ALOGE("get delay_board delay time: %d\r\n", *ps_p);

	return ret;
}

/* get U305、U306、U307、U309、U310、U311 temperature  range:(3~100℃)
*  adc_v_p: Pointing to an array of six elements for saveing adc original temperature value 
*  centi_p: Pointing to an array of six elements for saveing celsius value
*/
int delay_board_get_all_temperature(uint16_t *adc_v_p, double *centi_p)
{
	int ret = 0;

	uint8_t buff[14] = { 0 };
	uint8_t retry_count = 0;

	ret += i2c_reg_read(DELAY_BOARD_I2C_ADDR, 0x5f, 1, buff, 14);

	for (int i = 0; i < 6; i++) {

		adc_v_p[i] = buff[2 * i] * 256 + buff[2 * i + 1];

		if (adc_v_p[i] > 4068)
		{
			adc_v_p[i] = 4068;
		}

		if (adc_v_p[i] < 2295)
		{
			adc_v_p[i] = 2295;
		}

		if (i == 6) {  //stm32 temp, now not used

			centi_p[6] = ((adc_v_p[6] - 943.0)*0.0025 + 25.0);
		} else {  //NTC temp

			centi_p[i] = temp_conversion_table[4068 - adc_v_p[i]];
		}
	}

	while (centi_p[0] == 95.0 && retry_count < 3) {

		usleep(100000);  //delay 100ms

		ret += i2c_reg_read(DELAY_BOARD_I2C_ADDR, 0x5f, 1, buff, 14);

		for (int i = 0; i < 6; i++) {

			adc_v_p[i] = buff[2 * i] * 256 + buff[2 * i + 1];

			if (adc_v_p[i] > 4068)
			{
				adc_v_p[i] = 4068;
			}

			if (adc_v_p[i] < 2295)
			{
				adc_v_p[i] = 2295;
			}

			if (i == 6) {  //stm32 temp, now not used

				centi_p[6] = ((adc_v_p[6] - 943.0)*0.0025 + 25.0);
			}
			else {  //NTC temp
				centi_p[i] = temp_conversion_table[4068 - adc_v_p[i]];
			}
		}

		retry_count++;
	}

	if (centi_p[0] == 95.0) {
		ret = -100;  //i2c error
	}

	ALOGE("get delay_board temperature: %d,%d,%d,%d,%d,%d \r\n", adc_v_p[0], adc_v_p[1], adc_v_p[2], adc_v_p[3], adc_v_p[4], adc_v_p[5]);
	ALOGE("get delay_board temperature: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f \r\n", centi_p[0], centi_p[1], centi_p[2], centi_p[3], centi_p[4], centi_p[5]);

	return ret;
}

static int nb6l295_i2c_write_reg(uint8_t reg, uint16_t value)
{
    int ret = i2c_reg_write(DELAY_BOARD_I2C_ADDR, reg, 1, value, 2);
    return ret;
}

static int nb6l295_i2c_read_reg(uint8_t reg, uint16_t *value)
{
    int ret = i2c_reg_read(DELAY_BOARD_I2C_ADDR, reg, 1, value, 2);
    return ret;
}

int nb6l295_write(uint16_t value) // delay unit: ps
{
	int ret;

	ret = nb6l295_i2c_write_reg(0x02, value);
	if (ret < 0)
		return 0;

	ret = nb6l295_i2c_write_reg(0x01, 0x0201); // start transfer

	return ret;
}

int nb6l295_read(uint16_t *value)
{
	int ret;

	ret = nb6l295_i2c_read_reg(0x02, value);
	if (ret < 0)
		return 0;

	return ret;
}

int nb6l295_get_id(uint16_t *id)
{
	int ret;

	uint16_t value = 0;
	ret = nb6l295_i2c_read_reg(0x03, &value);
	if (ret < 0)
		*id = 0xFFFF;
	
	*id = value;
	
	return ret;
}