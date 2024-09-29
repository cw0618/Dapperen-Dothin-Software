#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stmpe801.h"
#include "tof_sensors.h"

#include "mx6xhal\obc_tee_funcs.h"

#ifdef __linux__
#include <unistd.h>
#endif

//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
//#define ALOGE(...)
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread

#define stmpe801_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel
#define stmpe801_set_gpio_dir        tops_t.ap_ops.SetGpioPinDir
#define stmpe801_device_id           tops_t.ap_ops.device_id

#define GPIO_RESET_PIN               3
#define GPIO_LEVEL_HIGH              1
#define GPIO_LEVEL_LOW               0

#define I2C_M_RD     1
#define I2C_M_WT     0

struct regList {
    uint16_t reg;
    uint16_t val;
};

typedef struct i2c_msg {
    uint8_t slave_addr;
    uint8_t rw_mode;
    uint16_t reg;
    uint8_t  reg_size;
    uint32_t* data;
    uint16_t data_size;
}i2c_msg_t;

static int isConfig = 0;

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

static int stmpe801_write_reg(uint8_t reg, uint8_t value)
{
    int ret = i2c_reg_write(STMPE801_I2C_ADDR, reg, 1, value, 1);

    return ret;
}

static int stmpe801_read_reg(uint8_t reg, uint8_t *value)
{
    int ret = i2c_reg_read(STMPE801_I2C_ADDR, reg, 1, value, 1);
    
    return ret;
}

int stmpe801_read_id(uint16_t* id)
{
    int ret;
    uint8_t id_h, id_l;
	
    ret = stmpe801_read_reg(0x01, &id_h);
    if (ret < 0) {
        return ret;
    }

    ret = stmpe801_read_reg(0x00, &id_l);
    if (ret < 0) {
        return ret;
    }

    id = (id_h << 8) | id_l;

    return ret;
}

int stmpe801_init()
{
    int ret;
    uint16_t id;

    stmpe801_set_gpio_level(GPIO_RESET_PIN, GPIO_LEVEL_HIGH, stmpe801_device_id);

    usleep(5);

    ret = stmpe801_read_id(&id);
    if (ret < 0)
        return ret;

    ret = stmpe801_write_reg(0x12, 0xff);//config gpio as output
    if (ret < 0) {
        return ret;
    }

    ret = stmpe801_write_reg(0x11, 0xff); //init gpio status
    if (ret < 0) {
        return ret;
    }

    return ret;
}

int stmpe801_set_io_state(uint8_t gpio, uint8_t level)
{
    uint16_t mask = 1;
    uint16_t value = 0;

    int ret = 0;

    if (gpio > 7)
        return -1;

    ret = stmpe801_read_reg(0x10, &value);
    if (ret < 0)
        return ret;

    mask <<= gpio;

    if (level) {
        value |= mask;
    }
    else {
        value &= (~mask);
    }

    ret = stmpe801_write_reg(0x11, value);

    return ret;
}

int stmpe801_get_io_state(uint8_t gpio, uint8_t* level)
{
    int ret = 0;
    uint16_t mask = 1;
    uint16_t value = 0;

    if (gpio > 7)
        return -1;

    ret = stmpe801_read_reg(0x10, &value);
    if (ret < 0)
        return ret;
	
	*level = (value >> gpio) & mask;

	return 0;
}