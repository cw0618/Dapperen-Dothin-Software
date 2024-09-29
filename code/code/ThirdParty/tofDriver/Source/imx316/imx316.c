#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imx316.h"
#include "hw_obstatus.h"
#include "hw_property.h"
#include "project_config.h"
#include "stdint.h"

#define GPIO_EXPANDER_RST_PIN_OB    3 // PO3, OB TB6309 board 
#define CXA4046_XCLR                2 // PO1


// IIC slave device
#define GPIO_EXPANDER_IIC_ADDR	    (0x20 << 1) // TCA6408A
#define IMX316_I2C_ADDR             0x34
#define EEPROM_I2C_ADDR             0xA0
#define MAX77831_I2C_ADDR           (0x66 << 1)

// vcsel driver IC type
#define DRIVER_IC_CXA4046              4046

#define DUAL_FREQ      1
#define SINGLE_FREQ    0

static uint16_t HMAX = 0x05A6;
static float CLK120MHz = 120.0f; // result turned out to be us

static uint16_t driver_ic_type = DRIVER_IC_CXA4046;

struct regList {
	uint16_t reg;
	uint8_t val;
};


#include <tof_sensors.h>
#include <obc_tee_funcs.h>

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

#define dothin_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage
#define dothin_pmu_set_onoff       tops_t.ap_ops.PmuSetOnOff
#define dothin_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define dothin_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_sensor_enable       tops_t.ap_ops.SensorEnable

#define dothin_device_id             tops_t.ap_ops.device_id
#define dothin_set_gpio_level        tops_t.ap_ops.SetGpioPinLevel

#define I2C_M_RD     1
#define I2C_M_WT     0


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

static int gpio_control(int pin, bool level)
{
    dothin_set_gpio_level(pin, level, dothin_device_id);
    return 0;
}

static int imx316_get_sensor_info(struct sensor_info_t *info) {

    info->embedded_data_size = 480 * 1.5 * 2;
    info->vcsel_num = 1;
    info->vcsel_driver_id = driver_ic_type;
    info->sensor_id = imx316_sensor_id;
    return 0;
}

static int imx316_dothin_config()
{
    int ret = 0;

    DDEBUG("dothin_device_id=%d\n", dothin_device_id);

    ret = dothin_enable_softpin(true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_enable_softpin ret=%d\n", ret);
    }
    ret = dothin_enable_gpio(true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_enable_gpio ret=%d\n", ret);
    }
    ret = dothin_set_sensor_clock(true, 24 * 10, dothin_device_id); // 24Mhz mclk
    if (ret < 0) {
        DDEBUG("dothin_set_sensor_clock ret=%d\n", ret);
    }
    ret = dothin_set_softpin_pullup(1, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_set_softpin_pullup ret=%d\n", ret);
    }
    usleep(1000 * 10);
    ret = dothin_set_sensor_i2c_rate(1, dothin_device_id); // 400Khz
    if (ret < 0) {
        DDEBUG("dothin_set_sensor_i2c_rate ret=%d\n", ret);
    }
    ret = dothin_sensor_enable(1, true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 20);
    ret = dothin_sensor_enable(2, true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 50);

    SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
    int           power_value[] = { 2800,       1800,      1200,        3300,       1200 };
    ret = dothin_pmu_set_voltage(sensor_power, power_value, 5, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d", ret);
    }
    usleep(1000 * 50);
    DDEBUG("end");

    return ret;
}

static int get_expander_tx_gpio(uint8_t *A_status_enable, uint8_t *B_status_enable)
{
	int rtn = 0;
	uint32_t gpio_status = 0;
    rtn = i2c_reg_read(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, &gpio_status, 1);

	*A_status_enable = ((gpio_status & 0x40) >> 6);
	*B_status_enable = ((gpio_status & 0x80) >> 7);

	return rtn;
}
//IO1 -> A SPOT, IO2 -> B SPOT
static int set_expander_tx_gpio(uint8_t A_status_enable, uint8_t B_status_enable)
{
	int rtn = 0;
	uint8_t gpio_status = 0;
    if (A_status_enable && B_status_enable)
        return 0;
    if (B_status_enable) {
        rtn = vcsel_driver_write_reg(0x0D, 0x19); // current 0.39A
        gpio_status = 0x3F | (B_status_enable << 7); // set IO2 to high
        rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, gpio_status, 1);    
    }
    if (A_status_enable) {      
        gpio_status = 0x3F | (A_status_enable << 6); // set IO1 to high
        rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, gpio_status, 1);
        rtn = vcsel_driver_write_reg(0x0D, 0xDF); // current 3.5A
    }
	return rtn;
}

static int gpio_expander_init() // for power control, Orbbec F201103 Board
{
    int rtn = 0;

    rtn = gpio_control(GPIO_EXPANDER_RST_PIN_OB, 1); // pull gpio expander rst pin to high	
    rtn = gpio_control(CXA4046_XCLR, 1); // pull cxa4046 rst pin to high
    rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
    rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, 0x3F, 1); // set selected gpio to high
    rtn = max77831_write_reg(0x13, 0x55); // 6.25V
    rtn = set_expander_tx_gpio(1, 0); // default use SPOT_A
    return rtn;
}

static int eeprom_write_reg(uint16_t reg, uint16_t value)
{
	int ret = i2c_reg_write(EEPROM_I2C_ADDR, reg, 2, value, 2);
	return ret;
}

static int eeprom_read_reg(uint16_t reg, uint16_t *value)
{
	int ret = i2c_reg_read(EEPROM_I2C_ADDR, reg, 2, value, 2);
	return ret;
}

static int max77831_write_reg(uint8_t reg, uint8_t value)
{
    int ret = i2c_reg_write(MAX77831_I2C_ADDR, reg, 1, value, 1);
    return ret;
}

static int max77831_read_reg(uint8_t reg, uint8_t *value)
{
    int ret = i2c_reg_read(MAX77831_I2C_ADDR, reg, 1, value, 1);
    return ret;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
    int ret = i2c_reg_write(IMX316_I2C_ADDR, reg, 2, value, 1);
	return ret;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
    int ret = i2c_reg_read(IMX316_I2C_ADDR, reg, 2, value, 1);
	return ret;
}

int imx316_illum_power_test_init()
{
	int rtn = 0;
	rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC
	rtn = sensor_write_reg(0x215C, 0x01); // group A output 1 phase
	rtn = sensor_write_reg(0x215D, 0x01); // group B output 1 phase
	rtn = sensor_write_reg(0x2144, 0xFF); // phase 1-4, no pulse modulation(vcsel constant light when integration)
    //rtn = imx316_set_stream_mode();
    //rtn = imx316_set_fps(20);


	return rtn;
}

int imx316_get_vcsel_pd(uint32_t *value) // driver ic pd value
{
	int rtn = 0;
	/*
	uint8_t valueH, valueL;
	rtn = sensor_read_reg(0x1544, &valueH); // PD_H2[9:8], addr 0x23
	rtn = sensor_read_reg(0x1549, &valueL); // PD_H2[7:0], addr 0x24
	uint16_t pd_h2 = valueL + (valueH & 0x03) * 256;

	rtn = sensor_read_reg(0x1543, &valueH); // PD_BG[9:8], addr 0x1E
	rtn = sensor_read_reg(0x1545, &valueL); // PD_BG[7:0], addr 0x1F
	uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
	//printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
	*value = (pd_h2 << 16) + pd_bg;
*/


	uint8_t valueH, valueL;
	rtn = vcsel_driver_read_reg(0x23, &valueH); // PD_TARGET_DATA[9:8]
	rtn = vcsel_driver_read_reg(0x26, &valueL); // PD_TARGET_DATA[7:0]
	uint16_t pd_h2 = valueL + ((valueH >> 4) & 0x03) * 256;

	rtn = vcsel_driver_read_reg(0x1E, &valueH); // PD_BG[9:8]
	rtn = vcsel_driver_read_reg(0x1F, &valueL); // PD_BG[7:0]
	uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
	printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
	*value = (pd_h2 << 16) + pd_bg;

	return rtn;
}

int imx316_get_rx_temp(float *temperature)
{
	uint32_t value;
	int rtn = sensor_read_reg(0x1403, &value);

	*temperature = (float)(value & 0xff) - 40;
	//("sensor temp %f", temperature);
	//qDebug() << "value temp " << value << *temp;

	return rtn;
}

int imx316_get_tx_temp(float *temperature)
{
	uint32_t valueH = 0;
	uint32_t valueL = 0;
	uint32_t value = 0;
	int rtn = sensor_read_reg(0x153d, &valueH);
	rtn |= sensor_read_reg(0x153e, &valueL);
	value = ((valueH & 0x03) << 8) + valueL;
	*temperature = 25 + (value - 297) / 5.4f;
	//printf("valueH %d, valueL %d\n", valueH, valueL);
	return rtn;
}

// [REG_M_12_12_10_1_5_33], default 15MHz
struct regList imx316_reglist_singlefreq[] = {
    { 0x1006, 0x18 },
    { 0x1007, 0x00 },
    { 0x1000, 0x00 },
    { 0x210F, 0x00 },
    { 0x4053, 0x00 },
    { 0x474C, 0x00 },
    { 0x474D, 0x00 },
    { 0x474E, 0x1F },
    { 0x474F, 0xFF },
    { 0x492D, 0x78 },
    { 0x4937, 0x3C },
    { 0x493D, 0x76 },
    { 0x4947, 0x3F },
    { 0x4963, 0x78 },
    { 0x496D, 0x3C },
    { 0x4B7F, 0x41 },
    { 0x4B83, 0x44 },
    { 0x4C39, 0x07 },
    { 0x4C4C, 0x02 },
    { 0x4D13, 0x01 },
    { 0x4E11, 0x01 },
    { 0x4F08, 0x72 },
    { 0x5825, 0x2F },
    { 0x5827, 0x1E },
    { 0x2140, 0x00 },
    { 0x4F03, 0x40 },
    { 0x5857, 0xDB },
    { 0x5886, 0x3B },
    { 0x5887, 0x28 },
    { 0x588A, 0x05 },
    { 0x588B, 0x14 },
    { 0x4765, 0xD4 },
    { 0x4767, 0xD6 },
    { 0x4755, 0xDA },
    { 0x4756, 0x0B },
    { 0x4757, 0x41 },
    { 0x4BBD, 0xDA },
    { 0x4BBE, 0x0B },
    { 0x4BBF, 0x41 },
    { 0x4B7C, 0x0A },
    { 0x4B7D, 0xDA },
    { 0x4B7E, 0x0B },
    { 0x4B80, 0x0A },
    { 0x4B81, 0xD7 },
    { 0x4B82, 0x0B },
    { 0x4B8C, 0x0A },
    { 0x4B8D, 0xDA },
    { 0x4B8E, 0x0A },
    { 0x4B8F, 0xDC },
    { 0x4D03, 0x04 },
    { 0x4D0A, 0x04 },
    { 0x4D0F, 0x04 },
    { 0x3C2B, 0x1B },
    { 0x4C02, 0x2F },
    { 0x4F09, 0x18 },
    { 0x4058, 0xFF },
    { 0x4059, 0xFF },
    { 0x405A, 0xFF },
    { 0x405B, 0xFF },
    { 0x21B8, 0x31 },
    { 0x21B9, 0x09 },
    { 0x21BA, 0x09 },
    { 0x1040, 0x00 },
    { 0x1041, 0x96 },
    { 0x1042, 0x03 },
    { 0x1048, 0x00 },
    { 0x1049, 0x96 },
    { 0x104A, 0x03 },
    { 0x100C, 0x09 },
    { 0x100D, 0x60 },
    { 0x1060, 0x00 },
    { 0x1071, 0x06 },
    { 0x020E, 0x00 },
    { 0x020F, 0x96 },
    { 0x1010, 0x01 },
    { 0x0800, 0x02 },
    { 0x0801, 0xFE },
    { 0x4015, 0x00 },
    { 0x4016, 0x39 },
    { 0x407B, 0x15 },
    { 0x4083, 0x3A },
    { 0x46FA, 0x0E },
    { 0x46FB, 0x77 },
    { 0x4716, 0x0E },
    { 0x4717, 0x77 },
    { 0x2148, 0x05 },
    { 0x2149, 0x05 },
    { 0x214A, 0x05 },
    { 0x214B, 0x05 },
    { 0x400E, 0x03 },
    { 0x400F, 0xFF },
    { 0x5865, 0x00 },
    { 0x5866, 0x2C },
    { 0x5867, 0xE2 },
    { 0x082C, 0x22 },
    { 0x082D, 0x22 },
    { 0x0830, 0x33 },
    { 0x0831, 0x33 },
    { 0x0834, 0xFF },
    { 0x1433, 0x01 },
    { 0x149b, 0x01 },
    { 0x1434, 0x00 },
    { 0x1435, 0x00 },
    { 0x1436, 0x00 },
    { 0x1437, 0x00 },
    { 0x1438, 0x00 },
    { 0x1439, 0x00 },
    { 0x143C, 0x06 },
    { 0x143D, 0x40 },
    { 0x143E, 0x06 },
    { 0x143F, 0x40 },
    { 0x1440, 0x06 },
    { 0x1441, 0x40 },
    { 0x0808, 0x00 },
    { 0x0809, 0x01 },
    { 0x080a, 0x00 },
    { 0x080b, 0x5A },
    { 0x1485, 0x00 },
    { 0x0812, 0x00 },
    { 0x2110, 0x00 },
    { 0x2111, 0x00 },
    { 0x2112, 0xEA },
    { 0x2113, 0x60 },
    { 0x2168, 0x04 },
    { 0x213C, 0x00 },
    { 0x216C, 0x28 },
    { 0x210A, 0x14 },
    { 0x210B, 0x63 },
    { 0x2100, 0x08 },
    { 0x2154, 0x00 },
    { 0x2155, 0x00 },
    { 0x2018, 0x01 },
    { 0x2178, 0x06 },
    { 0x2179, 0x1A },
    { 0x217A, 0x2E },
    { 0x217B, 0x42 },
    { 0x4131, 0x01 },
    { 0x21a8, 0x1A },
    { 0x0148, 0x01 },
    { 0x2020, 0x01 },
    { 0x3071, 0x00 },
    { 0x2F05, 0x01 },
    { 0x2F07, 0x9D },

     // user added
    { 0x1433,0x00 },// disable pixel error flag
    { 0x2C0C,0x01 },// EBD length  480*1.5 Byte
    { 0x2C0D,0xE0 },
};

// [REG_M_12_12_17_1_5_33], default 15MHz&75MHz
struct regList imx316_reglist_dualfreq[] = {
    {0x1006, 0x18},
    {0x1007, 0x00},
    {0x1000, 0x00},
    {0x210F, 0x00},
    {0x4053, 0x00},
    {0x474C, 0x00},
    {0x474D, 0x00},
    {0x474E, 0x1F},
    {0x474F, 0xFF},
    {0x492D, 0x78},
    {0x4937, 0x3C},
    {0x493D, 0x76},
    {0x4947, 0x3F},
    {0x4963, 0x78},
    {0x496D, 0x3C},
    {0x4B7F, 0x41},
    {0x4B83, 0x44},
    {0x4C39, 0x07},
    {0x4C4C, 0x02},
    {0x4D13, 0x01},
    {0x4E11, 0x01},
    {0x4F08, 0x72},
    {0x5825, 0x2F},
    {0x5827, 0x1E},
    {0x2140, 0x00},
    {0x4F03, 0x40},
    {0x5857, 0xDB},
    {0x5886, 0x3B},
    {0x5887, 0x28},
    {0x588A, 0x05},
    {0x588B, 0x14},
    {0x4765, 0xD4},
    {0x4767, 0xD6},
    {0x4755, 0xDA},
    {0x4756, 0x0B},
    {0x4757, 0x41},
    {0x4BBD, 0xDA},
    {0x4BBE, 0x0B},
    {0x4BBF, 0x41},
    {0x4B7C, 0x0A},
    {0x4B7D, 0xDA},
    {0x4B7E, 0x0B},
    {0x4B80, 0x0A},
    {0x4B81, 0xD7},
    {0x4B82, 0x0B},
    {0x4B8C, 0x0A},
    {0x4B8D, 0xDA},
    {0x4B8E, 0x0A},
    {0x4B8F, 0xDC},
    {0x4D03, 0x04},
    {0x4D0A, 0x04},
    {0x4D0F, 0x04},
    {0x3C2B, 0x1B},
    {0x4C02, 0x2F},
    {0x4F09, 0x18},
    {0x4058, 0xFF},
    {0x4059, 0xFF},
    {0x405A, 0xFF},
    {0x405B, 0xFF},
    {0x21B8, 0x31},
    {0x21B9, 0x09},
    {0x21BA, 0x09},

    {0x1040, 0x00},
    {0x1041, 0x96},
    {0x1042, 0x03},
    {0x1048, 0x00},
    {0x1049, 0x96},
    {0x104A, 0x03},
    {0x100C, 0x09},
    {0x100D, 0x60},
    {0x1060, 0x00},
    {0x1071, 0x06},
    {0x020E, 0x00},
    {0x020F, 0x96},
    {0x1010, 0x01},
    {0x0800, 0x02},
    {0x0801, 0xFE},
    {0x4015, 0x00},
    {0x4016, 0x39},
    {0x407B, 0x15},
    {0x4083, 0x3A},
    {0x46FA, 0x0E},
    {0x46FB, 0x77},
    {0x4716, 0x0E},
    {0x4717, 0x77},
    {0x2148, 0x05},
    {0x2149, 0x05},
    {0x214A, 0x05},
    {0x214B, 0x05},
    {0x400E, 0x03},
    {0x400F, 0xFF},
    {0x5865, 0x00},
    {0x5866, 0x2C},
    {0x5867, 0xE2},
    {0x082C, 0x22},
    {0x082D, 0x22},
    {0x0830, 0x33},
    {0x0831, 0x33},
    {0x0834, 0xFF},
    {0x1433, 0x01},
    {0x149b, 0x01},
    {0x1434, 0x00},
    {0x1435, 0x00},
    {0x1436, 0x00},
    {0x1437, 0x00},
    {0x1438, 0x00},
    {0x1439, 0x00},
    {0x143C, 0x06},
    {0x143D, 0x40},
    {0x143E, 0x06},
    {0x143F, 0x40},
    {0x1440, 0x06},
    {0x1441, 0x40},
    {0x0808, 0x00},
    {0x0809, 0x01},
    {0x080a, 0x00},
    {0x080b, 0x5A},
    {0x1485, 0x00},
    {0x0812, 0x00},
    {0x2110, 0x00},
    {0x2111, 0x00},
    {0x2112, 0xEA},
    {0x2113, 0x60},
    {0x2114, 0x00},
    {0x2115, 0x00},
    {0x2116, 0xEA},
    {0x2117, 0x60},
    {0x2168, 0x04},
    {0x213C, 0x00},
    {0x216C, 0x28},
    {0x216D, 0x08},
    {0x210A, 0x03},
    {0x210B, 0xCC},
    {0x2100, 0x48},
    {0x2154, 0x0C},
    {0x2155, 0xCA},
    {0x2018, 0x01},
    {0x2178, 0x06},
    {0x2179, 0x1A},
    {0x217A, 0x2E},
    {0x217B, 0x42},
    {0x2180, 0x06},
    {0x2181, 0x0A},
    {0x2182, 0x0E},
    {0x2183, 0x12},
    {0x4131, 0x01},
    {0x21a8, 0x1A},
    {0x21a9, 0x05},
    {0x0148, 0x01},
    {0x2020, 0x01},
    {0x3071, 0x00},
    {0x2F05, 0x01},
    {0x2F07, 0x9D},



	// user added
	{ 0x1433,0x00 },// disable pixel error flag
	{ 0x2C0C,0x01 },// EBD length  480*1.5 Byte
	{ 0x2C0D,0xE0 },
	//{ 0x0450,0x47 },// spi master config
	//{ 0x1001,0x01 },

};

static struct regList cxa4046_reglist[] = {
    // CXA4046 reset
    {0x0450, 0x47},
    {0x0403, 0x20},//Select of Slave Device for Transmit channel 1: device2 (SS2)
    {0x0405, 0x01},//Number of transaction for Transmit channel 1:transaction x 2
    {0x0500, 0x02},//Tx buffer 00
    {0x0501, 0x3C},
    {0x0502, 0x01},
    {0x0503, 0x02},
    {0x0504, 0x32},
    {0x0505, 0x01},
    {0x0401, 0x01},
    {0x0400, 0x01},
    {0x0401, 0x00},

    // CXA4046 register init
    {0x0403, 0x20},//start by register (SEND1_STTRIG);Select of Slave Device for Transmit channel 1: device2
    {0x0405, 0x00},// transaction x 1
    {0x0407, 0x00},//Start pointer of TX buffer for Transmit channel 1
    {0x0500, 0x20},//Tx buffer 00
    {0x0501, 0x00},
    {0x0502, 0x00},// 0x00
    {0x0503, 0x01},// 0x01
    {0x0504, 0x19},// 0x02
    {0x0505, 0x00},// 0x03
    {0x0506, 0x40},// 0x04
    {0x0507, 0x25},// 0x05
    {0x0508, 0x1E},// 0x06
    {0x0509, 0x80},// 0x07
    {0x050A, 0x42},// 0x08
    {0x050B, 0x88},// 0x09
    {0x050C, 0x00},// 0x0A
    {0x050D, 0x00},// 0x0B
    {0x050E, 0x5F},// 0x0C
    {0x050F, 0xDF},// 0x0D  ISW_FIX   LD_B: 0x19 -> 0.39A, LD_A: 0xDF -> 3.5A
    {0x0510, 0x00},// 0x0E
    {0x0511, 0x00},// 0x0F
    {0x0512, 0x00},// 0x10
    {0x0513, 0x00},// 0x11
    {0x0514, 0x00},// 0x12
    {0x0515, 0x00},// 0x13
    {0x0516, 0x00},// 0x14
    {0x0517, 0x00},// 0x15
    {0x0518, 0x00},// 0x16
    {0x0519, 0x00},// 0x17
    {0x051A, 0x00},// 0x18
    {0x051B, 0xC0},// 0x19
    {0x051C, 0xD0},// 0x1A
    {0x051D, 0x78},// 0x1B
    {0x051E, 0x21},// 0x1C
    {0x051F, 0x3F},// 0x1D
    {0x0520, 0x07},// 0x1E
    {0x0401, 0x01},
    {0x0400, 0x01},
    {0x0401, 0x00},

    // auto read/write CXA4046 register
    {0x0423, 0xA0},//THERMO_ start by internal sync signal (LDD_THRM_TIM_R),device1 (SS1)
    {0x0425, 0x02},//number of transaction Thermometer channel:3
    {0x0426, 0x00},//Start pointer of RX buffer for Thermometer channel
    {0x0427, 0x0E},//Start pointer of TX buffer for Thermometer channel
    {0x042a, 0x01},//Number of Tx byte in Tx/Rx CH: 1st byte is Tx, after Rx
    {0x0443, 0x03},//Start point of Temp Data
    {0x050e, 0x04},
    {0x050f, 0xAC},
    {0x0510, 0x02},
    {0x0511, 0xA2},
    {0x0512, 0x02},
    {0x0513, 0xA4},
    {0x0421, 0x01},
};

int cxa4046_initialize()
{
    int rtn = 0;
    for (int i = 0; i < sizeof(cxa4046_reglist) / sizeof(struct regList); i++) {

        rtn = sensor_write_reg(cxa4046_reglist[i].reg, cxa4046_reglist[i].val);
    }
    return rtn;
}

int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
	int rtn = 0;

	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0403, 0x20); // send1 channel, start by register trig
	rtn = sensor_write_reg(0x0405, 0x00); // send1 transcationx1
	rtn = sensor_write_reg(0x0407, 0x00); // send1 start pointer of Tx buffer
	rtn = sensor_write_reg(0x0500, 0x02); // number of byte to be write
	rtn = sensor_write_reg(0x0501, reg);  // address to be write to laser (write address = laser addr)
	rtn = sensor_write_reg(0x0502, value);// value to be write to laser
	rtn = sensor_write_reg(0x0401, 0x01); // enable
	rtn = sensor_write_reg(0x0400, 0x01); // trig
	rtn = sensor_write_reg(0x0401, 0x00); // disable

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable

	return rtn;
}

int vcsel_driver_read_reg(uint8_t reg, uint8_t *value) // do not use, Rx buffer is taken by thermal channel
{
	int rtn = 0;
	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
	rtn = sensor_write_reg(0x0436, 0x00); // start pointer of Rx buffer
	rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
	rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
	rtn = sensor_write_reg(0x0500, 0x02); // number of byte to be write and read
	rtn = sensor_write_reg(0x0501, reg + 0x80); // address to be read from laser (read address = laser addr + 0x80)
	rtn = sensor_write_reg(0x0431, 0x01); // enable
	rtn = sensor_write_reg(0x0430, 0x01); // trig
	rtn = sensor_write_reg(0x0431, 0x00); // disable

	rtn = sensor_read_reg(0x0580, value); // first byte in Rx buffer

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable
	return rtn;
}

int imx316_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
   
	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0403, 0x20);
	rtn = sensor_write_reg(0x0405, 0x00);
	rtn = sensor_write_reg(0x0407, 0x00);
	rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write
	rtn = sensor_write_reg(0x0501, 0x0C);
	rtn = sensor_write_reg(0x0502, value_B); // IBIAS
	rtn = sensor_write_reg(0x0503, value_A); // ISW
	rtn = sensor_write_reg(0x0401, 0x01);
	rtn = sensor_write_reg(0x0400, 0x01);
	rtn = sensor_write_reg(0x0401, 0x00);

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable

	return rtn;
}

int imx316_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
	*vcsel_num = 1;

	rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
	rtn = sensor_write_reg(0x0436, 0x00); // start pointer of Rx buffer
	rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
	rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
	rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write and read
	rtn = sensor_write_reg(0x0501, 0x0C + 0x80); // address to be read from laser (read address = laser addr 0x07 + 0x80)
	rtn = sensor_write_reg(0x0431, 0x01); // enable
	rtn = sensor_write_reg(0x0430, 0x01); // trig
	rtn = sensor_write_reg(0x0431, 0x00); // disable

	rtn = sensor_read_reg(0x0580, value_B); // first byte in Rx buffer
	rtn = sensor_read_reg(0x0581, value_A); // second byte in Rx buffer

	return rtn;
}

int imx316_illum_power_test(uint8_t mode)
{
	int rtn = 0;
	if (mode) { // illum power test mode, AA and QC used mode
		//rtn = vcsel_driver_write_reg(0x00, 0x00); // disable APC
		//rtn = sensor_write_reg(0x2144, 0xFF); // phase 1 is high fix, phase 2/3/4 is low fix during integration
	}
	else {
		//rtn = vcsel_driver_write_reg(0x00, 0x01); // enable APC
		//rtn = sensor_write_reg(0x2144, 0x00); // normal modulation 
	}

	return rtn;
}

int imx316_get_sensor_id(uint16_t *id)
{
    int rtn = 0;
    uint8_t value = 0;
    static int isConfig = 0;

    if (!isConfig) {
        isConfig = 1;
        rtn = gpio_expander_init();// power enable
        imx316_dothin_config();
}
    rtn = sensor_read_reg(0x0000, &value);

    if (value == 0x8C || value == 0x9b)
        *id = imx316_sensor_id;

    //ALOGE("imx316_sensor_id: %x, *id:%x", value, *id);
    return rtn;
}

int imx316_sensor_init()
{
	int rtn = 0;
#if 0	
    for (int i = 0; i < sizeof(imx316_reglist_dualfreq) / sizeof(struct regList); i++) {

        rtn = sensor_write_reg(imx316_reglist_dualfreq[i].reg, imx316_reglist_dualfreq[i].val);
	}
#else
    for (int i = 0; i < sizeof(imx316_reglist_singlefreq) / sizeof(struct regList); i++) {

        rtn = sensor_write_reg(imx316_reglist_singlefreq[i].reg, imx316_reglist_singlefreq[i].val);
    }
#endif
    rtn = cxa4046_initialize();

	return rtn;
}

int imx316_shadow_register(bool enable)
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(0x0102, 0x01);
	else
		rtn = sensor_write_reg(0x0102, 0x00);
	return rtn;
}

int imx316_set_trigger_mode(uint8_t mode) // should stop stream first
{
	int rtn = 0;
	if (mode == 0) { // hardware trigger mode
		rtn = sensor_write_reg(0x2020, 0x00);
		rtn |= sensor_write_reg(0x2100, 0x00);
		rtn |= sensor_write_reg(0x2F05, 0x07);
		rtn |= sensor_write_reg(0x2F06, 0x00);
		rtn |= sensor_write_reg(0x2F07, 0x00);
		rtn |= sensor_write_reg(0x3071, 0x03);
	}
	else { // software trigger mode
		rtn = sensor_write_reg(0x2020, 0x01);
		rtn |= sensor_write_reg(0x2100, 0x01);
		rtn |= sensor_write_reg(0x2F05, 0x01);
		rtn |= sensor_write_reg(0x2F06, 0x09);
		rtn |= sensor_write_reg(0x2F07, 0x7A);
		rtn |= sensor_write_reg(0x3071, 0x00);
	}
	return rtn;
}

int imx316_set_stream_mode() // should stop stream first
{
	int rtn = 0;
	rtn = sensor_write_reg(0x2020, 0x01);
	rtn |= sensor_write_reg(0x2100, 0x08);
	rtn |= sensor_write_reg(0x2F05, 0x01);
	rtn |= sensor_write_reg(0x2F06, 0x09);
	rtn |= sensor_write_reg(0x2F07, 0x7A);
	rtn |= sensor_write_reg(0x3071, 0x00);
	return rtn;
}

int imx316_get_modulation_frequency(uint16_t *modFreq)
{
	int rtn = 0;
	/*
	const static struct sensor_freq_duty_t freq_duty_list[15] = {
		{ 0, 0, 0, 0, 4 }, // not exist
		{ 120, 80, 37.5, 33.3 ,4 },
		{ 120, 20, 37.5, 33.3 ,4 },
		{ 120, 15, 37.5, 37.5 ,4 },
		{ 120, 10, 37.5, 33.3 ,4 },
		{ 100, 60, 33.3, 35.0 ,4 },
		{ 100, 20, 37.5, 35.0 ,4 },
		{ 100, 15, 33.3, 35.0 ,4 },
		{ 100, 10, 37.5, 35.0 ,4 },
		{ 80, 20, 37.5, 37.5 ,4 },
		{ 80, 15, 33.3, 37.5 ,4 },
		{ 80, 10, 33.3, 33.3 ,4 },
		{ 60, 20, 37.5, 33.3 ,4 },
		{ 60, 15, 37.5, 37.5 ,4 },
		{ 60, 10, 37.5, 33.3 ,4 }
	};
*/
	uint8_t freq_0, freq_1;
	uint8_t imxck = 0;
	uint8_t predivsel = 0;
	uint8_t fmodsel = 0;
	rtn = sensor_read_reg(0x1049, &imxck);

	rtn = sensor_read_reg(0x214C, &predivsel);
	rtn = sensor_read_reg(0x217C, &fmodsel);
	freq_0 = imxck * 8 / (pow(2, predivsel)*fmodsel*2);

	rtn = sensor_read_reg(0x214D, &predivsel);
	rtn = sensor_read_reg(0x217D, &fmodsel);
	freq_1 = imxck * 8 / (pow(2, predivsel)*fmodsel * 2);

	*modFreq = (freq_0 << 8) + freq_1;

	return rtn;
}

int imx316_set_modulation_frequency(uint16_t modFreq)
{
	int rtn = 0;

	static uint16_t index[] = { 120, 100, 80, 60, 20, 15, 10 };
	uint16_t mod_freq = index[modFreq];
	printf("set mod freq to %d MHz\r\n", mod_freq);

	switch (mod_freq) {
	case 120:
		rtn = sensor_write_reg(0x1049, 0x78);
		rtn = sensor_write_reg(0x214C, 0x00);
		rtn = sensor_write_reg(0x217C, 0x04);
		break;	
	case 100:
		rtn = sensor_write_reg(0x1049, 0x64);
		rtn = sensor_write_reg(0x214C, 0x00);
		rtn = sensor_write_reg(0x217C, 0x04);
		break;
	case 80:
		rtn = sensor_write_reg(0x1049, 0x50);
		rtn = sensor_write_reg(0x214C, 0x00);
		rtn = sensor_write_reg(0x217C, 0x04);
		break;
	case 60:
		rtn = sensor_write_reg(0x1049, 0x78);
		rtn = sensor_write_reg(0x214C, 0x01);
		rtn = sensor_write_reg(0x217C, 0x04);
		break;
	case 20:
		rtn = sensor_write_reg(0x1049, 0x78);
		rtn = sensor_write_reg(0x214C, 0x01);
		rtn = sensor_write_reg(0x217C, 0x0C);
		break;
	case 15:
		rtn = sensor_write_reg(0x1049, 0x78);
		rtn = sensor_write_reg(0x214C, 0x02);
		rtn = sensor_write_reg(0x217C, 0x08);
		break;
	case 10:
		rtn = sensor_write_reg(0x1049, 0x78);
		rtn = sensor_write_reg(0x214C, 0x02);
		rtn = sensor_write_reg(0x217C, 0x0C);
		break;
	default:
		break;
	}

	return rtn;
}

int imx316_get_fps(uint8_t *fps)
{
	uint8_t byte0, byte1, byte2, byte3;
	int rtn = sensor_read_reg(0x2108, &byte3);
	rtn = sensor_read_reg(0x2109, &byte2);
	rtn = sensor_read_reg(0x210A, &byte1);
	rtn = sensor_read_reg(0x210B, &byte0);

	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0 + 2;
	uint32_t frameTime = ((value * BPMAX_DIV_120) + BPMAX_DIV_120) + BPMAX_DIV_120;
	*fps = (uint8_t)(1000000 / frameTime);
	//printf("byte 3 2 1 0:  %x %x %x %x", byte3, byte2, byte1, byte0);
	//printf("frameTime %d, fps %d", frameTime, *fps);

	return rtn;
}

int imx316_set_fps(uint8_t fps)
{
	int rtn = 0;

	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t frameTime = 1000000 / fps; // us
	uint32_t value = ((frameTime - BPMAX_DIV_120) - BPMAX_DIV_120) / BPMAX_DIV_120;
	rtn = sensor_write_reg(0x2108, (value >> 24) & 0xFF);
	rtn = sensor_write_reg(0x2109, (value >> 16) & 0xFF);
	rtn = sensor_write_reg(0x210A, (value >> 8) & 0xFF);
	rtn = sensor_write_reg(0x210B, (value & 0xFF));

	return rtn;
}

int imx316_get_integration_time(uint16_t *integrationTime) // checked
{
	uint8_t byte0, byte1, byte2, byte3;
	int rtn = sensor_read_reg(0x2110, &byte3);
	rtn |= sensor_read_reg(0x2111, &byte2);
	rtn |= sensor_read_reg(0x2112, &byte1);
	rtn |= sensor_read_reg(0x2113, &byte0);

	uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;

	*integrationTime = (uint16_t)(value / CLK120MHz); // may slightly different from what we set because of floor function

	//printf("imx316_get_integration_time %d   ret=%d\n",*integrationTime,rtn);

	return rtn;
}

int imx316_set_integration_time(uint16_t integrationTime) // checked
{
	if (integrationTime > 1000)
		return -1;

	unsigned int value = (unsigned int)integrationTime*CLK120MHz;

    int rtn = imx316_shadow_register(true);
	for (int group = 0; group < 3; group++) {
		// set integration time
		rtn |= sensor_write_reg(0x2110 + group * 4, (value >> 24) & 0xFF);
		rtn |= sensor_write_reg(0x2111 + group * 4, (value >> 16) & 0xFF);
		rtn |= sensor_write_reg(0x2112 + group * 4, (value >> 8) & 0xFF);
		rtn |= sensor_write_reg(0x2113 + group * 4, (value & 0xFF));
	}

    rtn |= imx316_shadow_register(false);
	//printf("imx316_set_integration_time %d,  ret=%d\n", integrationTime,rtn);
	return rtn;
}

int imx316_get_img_mirror_flip(uint8_t *mode)
{
	int rtn = 0;
	uint32_t h = 0, v = 0;
	rtn |= sensor_read_reg(0x0811, &h);
	rtn |= sensor_read_reg(0x0810, &v);
	if (h == 0) {
		if (v == 0)
			*mode = 0;
		else
			*mode = 2;
	}
	else {
		if (v == 0)
			*mode = 1;
		else
			*mode = 3;
	}
	return rtn;
}

int imx316_set_img_mirror_flip(uint8_t mode)
{
	int rtn = 0;
	if (mode == 0) {
		rtn = sensor_write_reg(0x0811, 0x00); // default
		rtn = sensor_write_reg(0x0810, 0x00); // default
	}
	else if (mode == 1) {
		rtn = sensor_write_reg(0x0811, 0x01);
		rtn = sensor_write_reg(0x0810, 0x00);
	}
	else if (mode == 2) {
		rtn = sensor_write_reg(0x0811, 0x00);
		rtn = sensor_write_reg(0x0810, 0x01);
	}
	else if (mode == 3) {
		rtn = sensor_write_reg(0x0811, 0x01);
		rtn = sensor_write_reg(0x0810, 0x01);
	}
	return rtn;
}

#define FREQ_DUTY_LIST_NUM    1
int imx316_get_frequency_and_duty(int *index, int *index_max, struct sensor_freq_duty_t *freq_duty)
{
    const static struct sensor_freq_duty_t freq_duty_list[FREQ_DUTY_LIST_NUM] = {
        { 15, 0, 33.3, 0 ,4 }
    };
    *index_max = FREQ_DUTY_LIST_NUM;
    *freq_duty = freq_duty_list[0];

    return 0;
}

int imx316_set_frequency_and_duty(int index)
{
    return -HW_ERR_INVALID;
}

int imx316_get_frequency_mode(uint8_t *mode)
{
    uint8_t value;
    int rtn = sensor_read_reg(0x2100, &value);
    //ALOGE("imx316_get_frequency_mode: %d", value);
    if (value == 0x08)
        *mode = SINGLE_FREQ; // single freq
    else if (value == 0x48)
        *mode = DUAL_FREQ; // dual freq
    else
        return -1;

    return 0;
}

int imx316_set_frequency_mode(uint8_t mode)
{/*
 int rtn;
 if (mode == SINGLE_FREQ) {
 sensor_write_reg(0x2100, 0x08);
 imx518_set_fps(15);
 }
 else if (mode == DUAL_FREQ) {
 sensor_write_reg(0x2100, 0x48);
 imx518_set_fps(30);
 }
 */
    return -HW_ERR_INVALID;
}

int imx316_get_resolution(uint64_t *resolution)
{
    *resolution = RESOLUTION_480_180;
    return 0;
}

int imx316_get_pixel_bit(uint8_t *pixel_bit)
{
    *pixel_bit = 12;
    return 0;
}

int imx316_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
    *mipi_pack_bit = 12;
    return 0;
}

int imx316_video_streaming(bool enable)
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(0x1001, 0x01);
	else
		rtn = sensor_write_reg(0x1001, 0x00);

	return rtn;
}

int imx316_init()
{
	int rtn = 0;
    rtn = imx316_sensor_init();
    rtn = set_expander_tx_gpio(1, 0); // default use SPOT_A
	return rtn;
}

int imx316_func_init()
{
    memset(&tof_sensor, 0, sizeof(tof_sensor));
    tof_sensor.init = imx316_init;
    tof_sensor.get_sensor_id = imx316_get_sensor_id;
    tof_sensor.video_streaming = imx316_video_streaming;
    tof_sensor.get_fps = imx316_get_fps;
    tof_sensor.set_fps = imx316_set_fps;
    tof_sensor.get_rx_temp = imx316_get_rx_temp;
    tof_sensor.get_tx_temp = imx316_get_tx_temp;
    tof_sensor.set_illum_power = imx316_set_illum_power;
    tof_sensor.get_illum_power = imx316_get_illum_power;
    tof_sensor.illum_power_test = imx316_illum_power_test;
    tof_sensor.get_integration_time = imx316_get_integration_time;
    tof_sensor.set_integration_time = imx316_set_integration_time;
    tof_sensor.get_img_mirror_flip = imx316_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = imx316_set_img_mirror_flip;
    tof_sensor.get_sensor_info = imx316_get_sensor_info;
    tof_sensor.set_tx_a_b_power = set_expander_tx_gpio;
    tof_sensor.get_tx_a_b_power = get_expander_tx_gpio;
    tof_sensor.get_frequency_and_duty = imx316_get_frequency_and_duty;
    tof_sensor.set_frequency_and_duty = imx316_set_frequency_and_duty;
    tof_sensor.get_frequency_mode = imx316_get_frequency_mode;
    tof_sensor.set_frequency_mode = imx316_set_frequency_mode;
    tof_sensor.sensor_write_reg_8 = sensor_write_reg;
    tof_sensor.sensor_read_reg_8 = sensor_read_reg;
    tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;
    tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;
    tof_sensor.get_tof_sensor_resolution = imx316_get_resolution;
    tof_sensor.get_tof_sensor_pixel_bit = imx316_get_pixel_bit;
    tof_sensor.get_mipi_pack_bit = imx316_get_mipi_pack_bit;
    return 0;
}
