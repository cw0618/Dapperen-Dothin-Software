#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imx627.h"
#include "hw_obstatus.h"
#include "hw_property.h"
#include "tofinfo.h"
#include "imx627_setting/imx627_dual_freq.h"
#include "imx627_setting/imx627_single_freq.h"
#include "imx627_setting/imx627_single_freq_hard_reset.h"

#if  0
#define DDEBUG(fmt, ...)   	printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define DDEBUG(fmt, ...)   	
#endif

#define GPIO_EXPANDER_RST_PIN_QT    1 // PO2, QT big board  
#define GPIO_EXPANDER_RST_PIN_OB    3 // PO3, OB polaris board 
#define GPIO_EXPANDER_ADDR			(0x20 << 1) // TCA6408A

#define B_AREA_CTRL_PIN_QT   4 // PO4, QT big board
#define B_AREA_CTRL_PIN_OB   2 // PO1, OB polaris board and QT small board

// GPIO control
#define TRIGGER_PIN          2 // PO1
#define LD_ENABLE_PIN        1 // PO2
#define LD_ERROR_PIN         3

// IIC slave device
#define IMX627_I2C_ADDR      0x20
#define EEPROM_I2C_ADDR      0xA0

// vcsel driver IC type
#define DRIVER_IC_CXA4046              4046 // Polaris Tx
#define DRIVER_IC_CXA4026              4026 // Polaris Tx
#define DRIVER_IC_PHX3D_3021_AA        5016 // Taishan DVT2 Tx
#define DRIVER_IC_PHX3D_3021_CB        5017 // Taishan DVT3 Tx
#define DRIVER_IC_DW9912               9912 // DongWoon

#define DUAL_FREQ      1
#define SINGLE_FREQ    0

#define DUAL_FREQ_NO_BINNING_RESOLUTION         RESOLUTION_2560_7680
#define DUAL_FREQ_2X2_BINNING_RESOLUTION        0
#define DUAL_FREQ_4X4_BINNING_RESOLUTION        0

#define SINGLE_FREQ_NO_BINNING_RESOLUTION       RESOLUTION_2560_3840
#define SINGLE_FREQ_2X2_BINNING_RESOLUTION      0
#define SINGLE_FREQ_4X4_BINNING_RESOLUTION      0

#define PIXEL_BIT       10
#define MIPI_PACK_BIT   10

static uint8_t sin_dual_freq = DUAL_FREQ;
static uint16_t HMAX = 0x05A6;
static float CLK120MHz = 120.0f; // result turned out to be us

static uint16_t driver_ic_type = 0;

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

static int gpio_expander_init();
int imx627_set_frequency_mode(uint8_t mode);
static int sensor_read_reg(uint16_t reg, uint8_t *value);
static int vcsel_driver_write_reg(uint8_t reg, uint8_t value);

/*
typedef struct i2c_msg {
uint8_t slave_addr;
uint8_t rw_mode;
uint16_t reg;
uint8_t  reg_size;
uint32_t* data;
uint16_t data_size;
}i2c_msg_t;
*/

static struct regList cxa4046_reglist_temp_embed_transaction[] = {
    { 0x5823, 0xA0 },
    { 0x5825, 0x11 },
    { 0x5826, 0x00 },
    { 0x5827, 0x00 },
    { 0x582a, 0x01 },
    { 0x5900, 0x02 },
    { 0x5901, 0xAB },//addr0
    { 0x5902, 0x02 },
    { 0x5903, 0xA2 },//addr1
    { 0x5904, 0x04 },
    { 0x5905, 0xA4 },//addr2
    { 0x5906, 0x04 },
    { 0x5907, 0xD4 },//addr3
    { 0x5908, 0x02 },
    { 0x5909, 0xDA },//addr4
    { 0x590A, 0x04 },
    { 0x590B, 0xD7 },//addr5
    { 0x590C, 0x03 },
    { 0x590D, 0xDB },
    { 0x590E, 0x03 },
    { 0x590F, 0xAD },
    { 0x5910, 0x03 },
    { 0x5911, 0xB0 },
    { 0x5912, 0x02 },
    { 0x5913, 0x87 },
    { 0x5914, 0x03 },
    { 0x5915, 0x8F },
    { 0x5916, 0x03 },
    { 0x5917, 0x93 },
    { 0x5918, 0x04 },
    { 0x5919, 0x8C },
    { 0x591a, 0x05 },
    { 0x591b, 0x97 },
    { 0x591c, 0x02 },
    { 0x591d, 0x9E },
    { 0x591e, 0x02 },
    { 0x591f, 0x89 },
    { 0x5920, 0x05 },
    { 0x5921, 0xA7 },
    { 0x5922, 0x02 },
    { 0x5923, 0xAC },
    { 0x5821, 0x01 },
};

static struct regList cxa4046_reglist_write[] = {
    { 0x5821, 0x00 },//disable thermo channel

    { 0x5833, 0x20 },//GENERAL Select of start trigger
    { 0x5835, 0x00 },//Transaction x1
    { 0x5837, 0x00 },//Tx No0
    { 0x583a, 0x02 },
    { 0x5900, 0x02 },//read and write number
    { 0x5901, 0x0D },//ISW_FIX addr
    { 0x5902, 0x00 },//ISW_FIX value

    { 0x5831, 0x01 },
    { 0x5830, 0x01 },
    { 0x5831, 0x00 },
};

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

	ret = orbbec_i2c_writeread((uint8_t *)&msg, 1);

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

	ret = orbbec_i2c_writeread((uint8_t *)&msg, 1);

	return ret;
}

static int gpio_control(int pin, bool level)
{
	dothin_set_gpio_level(pin, level, dothin_device_id);
	return 0; // should wrap method by SDK
}

static int imx627_get_sensor_info(struct sensor_info_t *info) {

	info->embedded_data_size = 1280 * 2 * 10 / 8 * 8;
	info->vcsel_num = 1;
	info->vcsel_driver_id = driver_ic_type;
	info->sensor_id = imx627_sensor_id;

	return 0;
}

int imx627_dothin_config()
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
	ret = dothin_sensor_enable(3, true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

	ret = dothin_sensor_enable(3, true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

	SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
	int           power_value[] = { 2800,       1800,      1200,        3300,       1200 };
	ret = dothin_pmu_set_voltage((int *)sensor_power, power_value, 5, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d", ret);
	}
	usleep(1000 * 50);
	DDEBUG("end");

	return ret;
}

int imx627_get_sensor_id(uint16_t *id) // checked
{
	int rtn = 0;
	uint8_t value = 0;
	static int isConfig = 0;
	rtn = gpio_expander_init();// power enable

	if (!isConfig) {
		isConfig = 1;
		imx627_dothin_config();
	}
	rtn = sensor_read_reg(0x0000, &value);

	if (value == 0x03) //0x93 -> IMX516 sensor
		*id = imx627_sensor_id;

	ALOGE("imx627_sensor_id:%x,*id:%x", value, *id);
	return rtn;
}

static int gpio_expander_init() // for power control
{
	int rtn = 0;
	rtn = gpio_control(GPIO_EXPANDER_RST_PIN_OB, 1); // pull gpio expander rst pin to high
	rtn = gpio_control(GPIO_EXPANDER_RST_PIN_QT, 1); // pull gpio expander rst pin to high
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0xFF, 1); // set selected gpio to high
	return rtn;
}

static uint8_t current_freq_duty_index;
static uint8_t B_status;
static int get_expander_tx_gpio(uint8_t *A_status_enable, uint8_t *B_status_enable)
{
    /*
	int rtn = 0;
	uint32_t gpio_status = 0;
#if 0 // Polaris A 方案一
	*B_status_enable = B_status; // TODO: cannot read from GPIO right now
	*A_status_enable = 1; // A is always on, no deed to control

#else // Polaris A 方案三
	if (B_status) { // TODO: cannot read from GPIO right now
		*B_status_enable = 1;
		*A_status_enable = 0;
	}
	else {
		*B_status_enable = 0;
		*A_status_enable = 1;
	}
#endif
	return rtn;
    */
    return 0;
}

static int set_expander_tx_gpio(uint8_t A_status_enable, uint8_t B_status_enable)
{
    /*
	int rtn = 0;
	uint8_t gpio_status = 0;
#if 0 // Polaris A 方案一
	if (!A_status_enable) // area A cannot be switched off
		return -1;
	if (!B_status_enable) { // for safety. should decrease current before switched to A area 
		rtn = imx627_set_illum_power(1, 0x38, 0x00); // 1.55A -> 0x56,  1.0A -> 0x38
		rtn = vcsel_driver_write_reg(0x15, 0x9F); // 鏃犵數鎰?0x89锛屾湁鐢垫劅 0x9F
		B_status = 0;
	}
	rtn = gpio_control(B_AREA_CTRL_PIN_QT, B_status_enable);
	rtn = gpio_control(B_AREA_CTRL_PIN_OB, B_status_enable);
	if (B_status_enable){  // for safety. should increase current after switched to A+B area 
		rtn = imx627_set_illum_power(1, 0xC2, 0x00); // 3.5A
		rtn = vcsel_driver_write_reg(0x15, 0x52);
		B_status = 1;
	}
#else // Polaris A 方案三
	if (A_status_enable && B_status_enable) // area A and B cannot be turned on at the same time
		return -1;
	if (A_status_enable) { // for safety. should decrease current before switched to A area 
		rtn = imx627_set_illum_power(1, 0x56, 0x00); // 1.55A -> 0x56,  1.0A -> 0x38
		rtn = gpio_control(B_AREA_CTRL_PIN_OB, 0);
		rtn = gpio_control(B_AREA_CTRL_PIN_QT, 0);
		if (current_freq_duty_index == 6 || current_freq_duty_index == 7)//100M & 15M ,35.7% duty
			rtn = vcsel_driver_write_reg(0x15, 0x58);
		else if (current_freq_duty_index == 15)//60M & 15M ,35.7% duty
			rtn = vcsel_driver_write_reg(0x15, 0x56);
		else
			rtn = vcsel_driver_write_reg(0x15, 0x89);
		B_status = 0;
	}

	if (B_status_enable) {  // for safety. should increase current after switched to A+B area 
		rtn = gpio_control(B_AREA_CTRL_PIN_OB, 1);
		rtn = gpio_control(B_AREA_CTRL_PIN_QT, 1);
		rtn = imx627_set_illum_power(1, 0x92, 0x00); // 2.625A -> 0x92

		if (current_freq_duty_index == 6 || current_freq_duty_index == 7)//100M & 15M ,35.7% duty
			rtn = vcsel_driver_write_reg(0x15, 0x59);
		else if (current_freq_duty_index == 15)//60M & 15M ,35.7% duty
			rtn = vcsel_driver_write_reg(0x15, 0x52);
		else
			rtn = vcsel_driver_write_reg(0x15, 0x15);
		B_status = 1;
	}
#endif
	return rtn;
    */
    return 0;
}

static int eeprom_write_reg(uint16_t reg, uint16_t value)
{
	int ret = i2c_reg_write(EEPROM_I2C_ADDR, reg, 2, value, 2);
	return ret;
}

static int eeprom_read_reg(uint16_t reg, uint16_t *value)
{
	int ret = i2c_reg_read(EEPROM_I2C_ADDR, reg, 2, (uint32_t *)value, 2);
	return ret;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
	int ret = i2c_reg_write(IMX627_I2C_ADDR, reg, 2, value, 1);

	return ret;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
	int ret = i2c_reg_read(IMX627_I2C_ADDR, reg, 2, (uint32_t *)value, 1);
	return ret;
}

static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
    int rtn = 0;
    cxa4046_reglist_write[7].val = value;
    cxa4046_reglist_write[6].val = reg;
    for (int i = 0; i < sizeof(cxa4046_reglist_write) / sizeof(struct regList); i++) {
        rtn |= sensor_write_reg(cxa4046_reglist_write[i].reg, cxa4046_reglist_write[i].val);
    }
    //恢复thermo通道传输
    for (int i = 0; i < sizeof(cxa4046_reglist_temp_embed_transaction) / sizeof(struct regList); i++) {
        rtn |= sensor_write_reg(cxa4046_reglist_temp_embed_transaction[i].reg, cxa4046_reglist_temp_embed_transaction[i].val);
    }

    return rtn;
}

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
    int rtn = 0;
    rtn |= sensor_write_reg(0x5821, 0x00); //关闭thermo通道传输
    rtn |= sensor_write_reg(0x583a, 0x01); // 1st byte is Tx, after Rx
    rtn |= sensor_write_reg(0x5833, 0x20); // general channel, start by register trig
    rtn |= sensor_write_reg(0x5835, 0x00); //Transaction x1
    rtn |= sensor_write_reg(0x5836, 0x00);// start pointer of Rx buffer
    rtn |= sensor_write_reg(0x5837, 0x00); // start pointer of Tx buffer

    rtn |= sensor_write_reg(0x5900, 0x02); // number of byte to be write and read
    rtn |= sensor_write_reg(0x5901, reg | 0x80); // address to be read from laser (read address = laser addr 0x07 + 0x80)

    rtn |= sensor_write_reg(0x5831, 0x01); // enable
    rtn |= sensor_write_reg(0x5830, 0x01); // trig
    rtn |= sensor_write_reg(0x5831, 0x00); // disable

    rtn |= sensor_read_reg(0x5980, value); // first byte in Rx buffer

    //恢复thermo通道传输
    for (int i = 0; i < sizeof(cxa4046_reglist_temp_embed_transaction) / sizeof(struct regList); i++) {
        rtn |= sensor_write_reg(cxa4046_reglist_temp_embed_transaction[i].reg, cxa4046_reglist_temp_embed_transaction[i].val);
    }

    return rtn;
}

int imx627_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;

	if (driver_ic_type == DRIVER_IC_CXA4046)
	{
		struct TX_WAVE_S {
			uint8_t value1;  // AS_W 0x09
			uint8_t value2;  // AS_I 0x1E
			uint8_t illum;
		} tx_wave[] = {
			{ 0x80, 0x0f, 96 },
			{ 0x82, 0x0f, 127 },
			{ 0x88, 0x0f, 159 },
			{ 0x8d, 0x0f, 191 },
			{ 0x8a, 0x05, 223 },
		};

		for (int i = 0; i < sizeof(tx_wave) / sizeof(struct TX_WAVE_S); i++)
		{
			ALOGE("write tx_wave debug : [ %d ] ", i);

			if (value_A == tx_wave[i].illum)
			{
				rtn |= vcsel_driver_write_reg(0x09, tx_wave[i].value1);
				rtn |= vcsel_driver_write_reg(0x1E, tx_wave[i].value2);

				ALOGE("write tx_wave_reg, 0x09: 0x%x  0x1E: 0x%x ", tx_wave[i].value1, tx_wave[i].value2);
			}
		}
	}

    rtn |= sensor_write_reg(0x5821, 0x00); //关闭thermo通道传输

    rtn |= sensor_write_reg(0x583a, 0x03); // 1 2 3 st byte is Tx, after Rx
    rtn |= sensor_write_reg(0x5833, 0x20); // general channel, start by register trig
    rtn |= sensor_write_reg(0x5835, 0x00); //Transaction x1
    rtn |= sensor_write_reg(0x5837, 0x00); // start pointer of Tx buffer

    rtn |= sensor_write_reg(0x5900, 0x03); // number of byte to be write and read
    rtn |= sensor_write_reg(0x5901, 0x0C); // address to be read from laser (read address = laser addr 0x07 + 0x80)
    rtn |= sensor_write_reg(0x5902, value_B);// IBIAS
    rtn |= sensor_write_reg(0x5903, value_A);// ISW

    rtn |= sensor_write_reg(0x5831, 0x01); // enable
    rtn |= sensor_write_reg(0x5830, 0x01); // trig
    rtn |= sensor_write_reg(0x5831, 0x00); // disable

	//rtn = vcsel_driver_write_reg(0x0B, value_A); // ISW_APC_H2
    //恢复thermo通道传输
    for (int i = 0; i < sizeof(cxa4046_reglist_temp_embed_transaction) / sizeof(struct regList); i++) {
        rtn |= sensor_write_reg(cxa4046_reglist_temp_embed_transaction[i].reg, cxa4046_reglist_temp_embed_transaction[i].val);
    }

	ALOGE("imx627_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B); valueA: %d  valueB: %d ", value_A, value_B);

	return rtn;
}

int imx627_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
    int rtn = 0;

    rtn |= sensor_write_reg(0x5821, 0x00); //关闭thermo通道传输

    rtn |= sensor_write_reg(0x583a, 0x01); // 1st byte is Tx, after Rx
    rtn |= sensor_write_reg(0x5833, 0x20); // general channel, start by register trig
    rtn |= sensor_write_reg(0x5835, 0x00); //Transaction x1
    rtn |= sensor_write_reg(0x5836, 0x00);// start pointer of Rx buffer
    rtn |= sensor_write_reg(0x5837, 0x00); // start pointer of Tx buffer

    rtn |= sensor_write_reg(0x5900, 0x03); // number of byte to be write and read
    rtn |= sensor_write_reg(0x5901, 0x0C | 0x80); // address to be read from laser (read address = laser addr 0x07 + 0x80)

    rtn |= sensor_write_reg(0x5831, 0x01); // enable
    rtn |= sensor_write_reg(0x5830, 0x01); // trig
    rtn |= sensor_write_reg(0x5831, 0x00); // disable

    rtn |= sensor_read_reg(0x5980, value_B); // first byte in Rx buffer
    rtn |= sensor_read_reg(0x5981, value_A); // first byte in Rx buffer

    //恢复thermo通道传输
    for (int i = 0; i < sizeof(cxa4046_reglist_temp_embed_transaction) / sizeof(struct regList); i++) {
        rtn |= sensor_write_reg(cxa4046_reglist_temp_embed_transaction[i].reg, cxa4046_reglist_temp_embed_transaction[i].val);
    }

	ALOGE("imx627_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B); *value_A: %d  *value_B: %d ", *value_A, *value_B);

    return rtn;
}

int imx627_illum_power_test(uint8_t mode)
{
    /*
	int rtn = 0;
	if (mode) { // illum power test mode, AA and QC used mode
        // AA  PuTuoShan  鎵撳紑锛孧eizu/Polaris QC鍏抽棴
		//rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC

		rtn = sensor_write_reg(0x2144, 0xAB);// 0xAB: first phase on only, 0xFF four phase
		rtn = imx627_set_illum_power(1, 195, 0);
		rtn = imx627_set_frequency_mode(SINGLE_FREQ);
		rtn = imx627_set_fps(10); // 10fps equals to 20fps in single freq
		rtn = imx627_set_integration_time(500);
	}
	else {
		//rtn = vcsel_driver_write_reg(0x00, 0x0C); // enable APC
		rtn = sensor_write_reg(0x2144, 0x00); // normal modulation 
		rtn = imx627_set_frequency_mode(DUAL_FREQ);
	}

	return rtn;
    */
    return -1;
}

int imx627_get_vcsel_pd(uint32_t *value) // driver ic pd value
{
    /*
	int rtn = 0;
	if (driver_ic_type == 4026) {
		uint8_t valueH, valueL;
		rtn = sensor_read_reg(0x1544, &valueH); // PD_H2[9:8], addr 0x23
		rtn = sensor_read_reg(0x154B, &valueL); // PD_H2[7:0], addr 0x24
		uint16_t pd_target_data = valueL + ((valueH >> 4) & 0x03) * 256;

		rtn = sensor_read_reg(0x1543, &valueH); // PD_BG[9:8], addr 0x1E
		rtn = sensor_read_reg(0x1545, &valueL); // PD_BG[7:0], addr 0x1F
		uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
		//printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
		*value = (pd_target_data << 16) + pd_bg;
	}
	else if (driver_ic_type == 5017) {
		uint8_t valueH, valueL;
		rtn = sensor_read_reg(0x154E, &valueH); // PD_H2[9:8], addr 0x1B
		rtn = sensor_read_reg(0x1553, &valueL); // PD_H2[7:0], addr 0x20
		uint16_t pd_h2 = valueL + (valueH & 0x03) * 256;

		rtn = sensor_read_reg(0x154D, &valueH); // PD_BG[9:8], addr 0x1A
		rtn = sensor_read_reg(0x154F, &valueL); // PD_BG[7:0], addr 0x1C
		uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
		//printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
		*value = (pd_h2 << 16) + pd_bg;
	}
	else if (driver_ic_type == 9912) {
		uint8_t valueH, valueL;
		rtn = sensor_read_reg(0x154E, &valueH); // CHECK_APC_IMOD[9:8], addr 0x1B
		rtn = sensor_read_reg(0x1555, &valueL); // CHECK_APC_IMOD[7:0], addr 0x22
		uint16_t check_apc_imod = valueL + ((valueH >> 4) & 0x03) * 256;

		rtn = sensor_read_reg(0x154D, &valueH); // PD_BG[9:8], addr 0x1A
		rtn = sensor_read_reg(0x154F, &valueL); // PD_BG[7:0], addr 0x1C
		uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
		//printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
		*value = (check_apc_imod << 16) + pd_bg;
	}

	return rtn;
    */
    return -1;
}

int imx627_get_vcsel_error(uint16_t *value)
{
    /*
	int rtn = 0;
	if (driver_ic_type == 4026) {
		uint8_t valueH, valueL;
		rtn = sensor_read_reg(0x154C, &valueH); // addr 0x27
		rtn = sensor_read_reg(0x154D, &valueL); // addr 0x28
		*value = (valueH << 8) + valueL;
	}
	else if (driver_ic_type == 5017 || driver_ic_type == 9912) {
		uint8_t valueH, valueL;
		rtn = sensor_read_reg(0x1556, &valueH); // addr 0x23
		rtn = sensor_read_reg(0x1557, &valueL); // addr 0x24
		*value = (valueH << 8) + valueL;
	}
	return rtn;
    */
    return -1;
}

int imx627_get_rx_temp(float *temperature)
{
	uint32_t value;
	int rtn = sensor_read_reg(0x1805, (uint8_t *)&value);
	*temperature = (float)(value & 0xff) - 40;

	ALOGE("imx627_get_rx_temp(float *temperature); *temperature: %.2f ", *temperature);

	return rtn;
}

int imx627_get_tx_temp(float *temperature)
{
	int rtn = 0;
	
	uint8_t valueH = 0;
	uint8_t valueL = 0;

	rtn |= vcsel_driver_read_reg(0x22, &valueH);
	rtn |= vcsel_driver_read_reg(0x24, &valueL);

	uint16_t value = ((valueH & 0x03) << 8) + valueL;
	*temperature = 25 + (value - 296) / 4.44f;
	//printf("valueH %d, valueL %d\n", valueH, valueL);

	ALOGE("imx627_get_tx_temp(float *temperature); *temperature: %.2f ", *temperature);

	return rtn;
}

int imx627_hardware_trigger()
{/*
 //mdevices.gpio_control(TRIGGER_PIN, 1);
 mdevices.gpio_control(TRIGGER_PIN, 0); // active low
 mdevices.sleepms(0.001); // 1us, not accurate
 //mdevices.usleep(100);
 mdevices.gpio_control(TRIGGER_PIN, 1);
 */
	return 0;
}

int imx627_software_trigger()
{
	//int rtn = sensor_write_reg(0x2100, 0x01);
	return -1;
}

int imx627_illum_power_control(bool enable)
{
	//return gpio_control(LD_ENABLE_PIN, enable); // active high
	return 0;
}

static int vcsel_driver_detect()
{
    driver_ic_type = DRIVER_IC_CXA4046;
}

int imx627_sensor_init()
{
	int rtn = 0;
#if 1
	for (int i = 0; i < sizeof(imx627_reglist_single_freq) / sizeof(struct reglist); i++) {
		rtn |= sensor_write_reg(imx627_reglist_single_freq[i].reg, imx627_reglist_single_freq[i].val);
	}
#else
	for (int i = 0; i < sizeof(imx627_reglist_single_freq_hardreset) / sizeof(struct reglist); i++) {
		rtn |= sensor_write_reg(imx627_reglist_single_freq_hardreset[i].reg, imx627_reglist_single_freq_hardreset[i].val);
	}
#endif
	vcsel_driver_detect();

	ALOGE("imx627_sensor_single_freq_init();  rtn: %d ", rtn);

	return rtn;
}

int imx627_sensor_dual_freq_init()
{
    int rtn = 0;

    for (int i = 0; i < sizeof(imx627_reglist_dual_freq) / sizeof(struct reglist); i++) {
        rtn |= sensor_write_reg(imx627_reglist_dual_freq[i].reg, imx627_reglist_dual_freq[i].val);
    }

	vcsel_driver_detect();

	ALOGE("imx627_sensor_dual_freq_init();  rtn: %d ", rtn);

    return rtn;
}

int imx627_shadow_register(bool enable) // checked
{
    return -1;
}

int imx627_get_data_output_mode(uint8_t *mode) // checked
{
	*mode = 4;

	return 0;
}

// be careful that the image width of A&B mode will change to 1280 pixel
// should stop streaming before change data output mode
int imx627_set_data_output_mode(uint8_t mode)
{
	return 0;
}

int imx627_get_frequency_and_duty(int *index, int *index_max, struct sensor_freq_duty_t *freq_duty)
{
	int rtn = 0;

	const static struct sensor_freq_duty_t freq_duty_list[] = {
		{ 20,   0, 33.3,    0, 4 },
		{ 100,  0, 33.3,    0, 4 },
		{ 20,  54, 33.3, 33.3, 4 },
		{ 20, 100, 33.3, 33.3, 4 },  // 54.54M
	};

	*index_max = sizeof(freq_duty_list) / sizeof(struct sensor_freq_duty_t);

	if (*index < 0)
	{
		*index = current_freq_duty_index;
	}
	else if (*index >= *index_max)
	{
		return  -2002;
	}

	*freq_duty = freq_duty_list[*index];

	ALOGE("imx627_get_frequency_and_duty(); *index_max: %d  ", *index_max);

    return rtn;
}

int imx627_set_frequency_and_duty(int index)
{
	int rtn = 0;

	if (index < 2)  // 0,1 single frequency
	{
		imx627_set_frequency_mode(0);
	}
	else  // dual frequency
	{
		imx627_set_frequency_mode(1);
	}

	current_freq_duty_index = index;
	struct regList freq_duty[] = {
		// 20M, 33%duty
		{ 0x0845, 0x01 },
		{ 0x0851, 0x00 },
		{ 0x085C, 0x03 },
		{ 0x1001, 0x78 }, //
		{ 0x1003, 0x78 }, //
		{ 0x1019, 0x3A }, //
		{ 0x101b, 0x3C }, //
		{ 0x1009, 0x28 },
		{ 0x100b, 0x3C },

		// 100M, 33%duty
		{ 0x0845, 0x01 },
		{ 0x0851, 0x00 },
		{ 0x085C, 0x01 },
		{ 0x1001, 0x18 }, //
		{ 0x1003, 0x78 }, //
		{ 0x1019, 0x0A }, //
		{ 0x101b, 0x3C }, //
		{ 0x1009, 0x08 },
		{ 0x100b, 0x3C },

		// 20M, 33%duty  54.54M, 33%duty
		{ 0x0845, 0x00 },
		{ 0x0851, 0x01 },
		{ 0x085C, 0x07 },
		{ 0x1001, 0x78 }, //
		{ 0x1003, 0x2C }, //
		{ 0x1019, 0x3A }, //
		{ 0x101b, 0x14 }, //
		{ 0x1009, 0x28 },
		{ 0x100b, 0x0E },

		// 20M, 33%duty  100M, 33%duty
		{ 0x0845, 0x00 },
		{ 0x0851, 0x01 },
		{ 0x085C, 0x06 },
		{ 0x1001, 0x78 }, //
		{ 0x1003, 0x18 }, //
		{ 0x1019, 0x3A }, //
		{ 0x101b, 0x0A }, //
		{ 0x1009, 0x28 },
		{ 0x100b, 0x08 },
	};

	int table_size = 9;
	for (int i = 0; i < table_size; i++)
	{
		rtn = sensor_write_reg(freq_duty[i + table_size * index].reg, freq_duty[i + table_size * index].val);
	}

	ALOGE("imx627_set_frequency_and_duty(); index: %d  rtn: %d ", index, rtn);

	return rtn;
}

static uint16_t temp_mod_freq = 0x6464;
int imx627_get_modulation_frequency(uint16_t *modFreq)
{
    int rtn = 0;

	*modFreq = temp_mod_freq;

	ALOGE("imx627_get_modulation_frequency(uint16_t *modFreq); *modFreq: %x ", *modFreq);
#if 0
    uint8_t freq_a, freq_b;
    uint8_t value0 = 0, value1 = 0;

    rtn |= sensor_read_reg(0x1000, &value1);
    rtn |= sensor_read_reg(0x1001, &value0);//FMOD_PERIOD_A[7:0]
    if (((value1 & 0x3F << 8) | value0) != 0)
    {
        freq_a = (uint32_t)IMVCK / ((value1 << 8) | value0);
        //printf("freq_a = %d\r\n", freq_a);
    }
    else
        return -1;

    rtn |= sensor_read_reg(0x1002, &value1);
    rtn |= sensor_read_reg(0x1003, &value0);//FMOD_PERIOD_B[7:0]
    if (((value1 & 0x3F << 8) | value0) != 0)
        freq_b = IMVCK / ((value1 << 8) | value0);
    else
        return -1;

    *modFreq = freq_a;
#endif
    return rtn;
}

int imx627_set_modulation_frequency(uint16_t modFreq)
{
	int rtn = 0;

	temp_mod_freq = modFreq;

	ALOGE("imx627_set_modulation_frequency(uint16_t modFreq); modFreq: %x ", modFreq);
#if 0
    static uint16_t index[] = { 120, 100, 80, 60, 20, 15, 10 };//mod_freq = 20 24 30 40 120 160 240
    uint16_t mod_freq = index[modFreq];
    uint16_t value = IMVCK / mod_freq, value1;
    uint8_t value_read = 0;

    printf("set mod freq to %d MHz\r\n", mod_freq);
    sensor_read_reg(0x1000, &value_read);
    rtn |= sensor_write_reg(0x1000, (value >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x1001, value & 0xff);//FMOD_PERIOD_A[7:0]

    sensor_read_reg(0x1002, &value_read);
    rtn |= sensor_write_reg(0x1002, (value >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x1003, value & 0xff);//FMOD_PERIOD_B[7:0]

    value1 = value * 0.333 + 0.5;
    sensor_read_reg(0x1008, &value_read);
    rtn |= sensor_write_reg(0x1008, (value1 >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x1009, value1 & 0xff);//LSR1_WID_A[7:0]
    sensor_read_reg(0x100a, &value_read);
    rtn |= sensor_write_reg(0x100a, (value1 >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x100b, value1 & 0xff);//LSR1_WID_B[7:0]

    value1 = value * 0.5;
    //printf("imx518_set_modulation_frequency value = %#x \r\n", value);
    sensor_read_reg(0x1018, &value_read);
    rtn |= sensor_write_reg(0x1018, (value1 >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x1019, value1 & 0xff);//GD_WID_A[7:0]
    sensor_read_reg(0x101a, &value_read);
    rtn |= sensor_write_reg(0x101a, (value1 >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x101b, value1 & 0xff);//GD_WID_B[7:0]
#endif
	return rtn;
}

int imx627_get_fps(uint8_t *fps)
{
    uint8_t byte0, byte1, byte2, byte3;
    int rtn = 0;
    rtn |= sensor_read_reg(0x0800, &byte3);
    rtn |= sensor_read_reg(0x0801, &byte2);
    rtn |= sensor_read_reg(0x0802, &byte1);
    rtn |= sensor_read_reg(0x0803, &byte0);
    static int BPMAX_DIV_120 = 5;
    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
    uint32_t frameTime = value * BPMAX_DIV_120;
    //姝ゅ闇€瑕佸啀鍒ゆ柇甯т腑鍖呭惈鐨勭浉浣嶆暟鎹绠楀嚭鐨勫抚闀垮害澶у皬鏄惁澶т簬璁剧疆鐨勫抚闀垮害
    *fps = (uint8_t)(1000000 / frameTime);
    return rtn;
}

int imx627_set_fps(uint8_t fps)
{
    int rtn = 0;
    static int BPMAX_DIV_120 = 5;//base pluse
    uint32_t frameTime = 1000000 / fps; // us
    uint32_t value = frameTime / BPMAX_DIV_120;
    rtn |= sensor_write_reg(0x0800, (value >> 24) & 0xFF);
    rtn |= sensor_write_reg(0x0801, (value >> 16) & 0xFF);
    rtn |= sensor_write_reg(0x0802, (value >> 8) & 0xFF);
    rtn |= sensor_write_reg(0x0803, (value & 0xFF));
	return rtn;
}

int imx627_group_param_hold(bool enable) // checked
{
    int rtn = 0;
    if (enable)
        rtn = sensor_write_reg(0x0111, 0x01);
    else
        rtn = sensor_write_reg(0x0111, 0x00);
    return rtn;
}

int imx627_get_integration_time(uint16_t *integrationTime) // checked
{
    uint8_t byte0, byte1, byte2, byte3;
    float clk200mhz = 200.0f;
    int rtn = sensor_read_reg(0x0820, &byte3);
    rtn |= sensor_read_reg(0x0821, &byte2);
    rtn |= sensor_read_reg(0x0822, &byte1);
    rtn |= sensor_read_reg(0x0823, &byte0);

    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;

    *integrationTime = (uint16_t)(value / clk200mhz); // may slightly different from what we set because of floor function

    return rtn;
}

int imx627_set_integration_time(uint16_t integrationTime) // checked
{
	int rtn = 0;

    if (integrationTime > 1000)
        return -1;
    float clk200mhz = 200.0f;
    uint32_t value = integrationTime * clk200mhz;

	rtn = imx627_group_param_hold(true);
    for (int group = 0; group < 2; group++) {
        // set integration time
        rtn |= sensor_write_reg(0x0820 + group * 4, (value >> 24) & 0xFF);
        rtn |= sensor_write_reg(0x0821 + group * 4, (value >> 16) & 0xFF);
        rtn |= sensor_write_reg(0x0822 + group * 4, (value >> 8) & 0xFF);
        rtn |= sensor_write_reg(0x0823 + group * 4, (value & 0xFF));
    }

    rtn |= imx627_group_param_hold(false);

    //printf("imx516_set_integration_time %d,  ret=%d\n", integrationTime,rtn);
    return rtn;
}

int imx627_set_phase_led_enable_pulse(uint8_t phaseLedEn)
{
    return -1;
}

int imx627_pixel_binning(uint8_t mode)
{
    return -1;
}

int imx627_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    return -1;
}

int imx627_get_img_mirror_flip(uint8_t *mode)
{
    return -1;
}

int imx627_set_img_mirror_flip(uint8_t mode)
{
    return -1;
}

int imx627_test_pattern(uint8_t mode)
{
    return -1;
}

int imx627_illum_duty_cycle_adjust(uint8_t mode, uint8_t value)
{
	int rtn = 0;
#if 0
	if (mode == 0) { // no duty cycle correction (= disabled) default.
		rtn |= sensor_write_reg(0x4E9E, 0x00);
	}
	else if (mode == 1) { // time delay on the falling edge (= increased duty cycle)
		rtn |= sensor_write_reg(0x4E9E, 0x01);
		rtn |= sensor_write_reg(0x21B9, value);
	}
	else if (mode == 2) { // time delay on the rising edge (= decreased duty cycle)
		rtn |= sensor_write_reg(0x4E9E, 0x02);
		rtn |= sensor_write_reg(0x21B9, value);
	}
#endif
	return rtn;
}

// different modulation frequency have different adjustable duty cycle range
int imx627_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    static uint8_t mod_freq_list[] = { 120, 20, };
    //static float duty_list[239] = { 0 };
    int i = 0;
    //float cycle = 2400.0f / mod_freq;
    uint8_t cycle = 2400 / mod_freq;

    for (i = 0; i < sizeof(mod_freq_list); i++) {
        if (mod_freq_list[i] == mod_freq)
            break;
    }
    if (i == sizeof(mod_freq_list))
        return -HW_ERR_INVALID;
#if 0
    for (int i = 0; i < 239; i++)
	{
        duty_cycle_list[i] = 0;
    }

    for (int i = 0; i < cycle - 1; i++) {
        duty_cycle_list[i] = (float)(i + 1) / (float)(cycle) * 100;
    }
#else
	float list[31] = {
		27, 28.4, 29.86, 31.32, 32.78, 34.24, 35.7, 37.16, 38.62, 40.08, 41.54, 43, 44.4, 45.8, 47.2, 48.7,  // 15
		50, 51.4, 52.7, 54.2, 55.66, 57.12, 58.58, 60.04, 61.5, 62.96, 64.42, 65.88, 67.34, 68.8, 70.5,  // 30
	};

	memcpy(duty_cycle_list, list, sizeof(list));
#endif

	ALOGE("imx627_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list); ");

    return 0;
}

static uint16_t temp_duty = 0;
int imx627_get_illum_duty_cycle(uint16_t *duty)
{
	int rtn = 0;

	*duty = temp_duty;

	ALOGE("imx627_get_illum_duty_cycle(uint16_t *duty); *duty: %d ", *duty);
#if 0
    uint8_t value_read_h = 0, value_read_l = 0;

    rtn |= sensor_read_reg(0x1008, &value_read_h);
    rtn |= sensor_read_reg(0x1009, &value_read_l);//GD_WID_A[7:0]
    //*value_a = (value_read_h & 0x3F) << 8 | value_read_l;
    *duty = (uint16_t)value_read_l << 8 - 1;

    rtn |= sensor_read_reg(0x100a, &value_read_h);
    rtn |= sensor_read_reg(0x100b, &value_read_l);//GD_WID_B[7:0]
    //*value_b = (value_read_h & 0x3F) << 8 | value_read_l;
    *duty |= (value_read_l - 1);
#endif
    return rtn;
}

int imx627_set_illum_duty_cycle(uint16_t duty)
{
	int rtn = 0;

	temp_duty = duty;

	ALOGE("imx627_set_illum_duty_cycle(uint16_t duty); duty: %d ", duty);
#if 0
    uint8_t value_read = 0;
    uint8_t duty_a = (duty >> 8) & 0xFF + 1;
    uint8_t duty_b = duty & 0xFF + 1;

    rtn = sensor_read_reg(0x1008, &value_read);
    rtn |= sensor_write_reg(0x1008, (duty_a >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x1009, duty_a & 0xff);//LSR1_WID_A[7:0]
    rtn |= sensor_read_reg(0x100a, &value_read);
    rtn |= sensor_write_reg(0x100a, (duty_b >> 8) & 0x3F | (value_read & 0xc0));
    rtn |= sensor_write_reg(0x100b, duty_b & 0xff);//LSR1_WID_B[7:0]
#endif
    return rtn;
}

static uint8_t freq_mode = 0;  // 0: single freq  1: dual freq
int imx627_set_frequency_mode(uint8_t mode)
{
	int rtn;

	freq_mode = mode;

	ALOGE("imx627_set_frequency_mode(uint8_t mode); mode : %d ", mode);
#if 0
	if (mode == SINGLE_FREQ)
	{
		rtn = sensor_write_reg(0x0845, 0x01);
		rtn = sensor_write_reg(0x0851, 0x00);
		rtn = sensor_write_reg(0x085c, 0x01);

		rtn = sensor_write_reg(0x1001, 0x18);
		rtn = sensor_write_reg(0x1003, 0x78);
		rtn = sensor_write_reg(0x1019, 0x0A);
		rtn = sensor_write_reg(0x101b, 0x3c);
		rtn = sensor_write_reg(0x1009, 0x08);
		rtn = sensor_write_reg(0x100b, 0x3c);
        //rtn = imx627_sensor_init();
		sin_dual_freq = SINGLE_FREQ;
		ALOGE("imx627_set_frequency_mode(uint8_t mode); set single_freq: write_reg(0x0851, 0x00)");
		//imx627_set_fps(15);
	}
	else if (mode == DUAL_FREQ)
	{
		rtn = sensor_write_reg(0x0845, 0x00);
		rtn = sensor_write_reg(0x0851, 0x01);
		rtn = sensor_write_reg(0x085c, 0x06);

		rtn = sensor_write_reg(0x1001, 0x78);
		rtn = sensor_write_reg(0x1003, 0x18);
		rtn = sensor_write_reg(0x1019, 0x3A);
		rtn = sensor_write_reg(0x101b, 0x0A);
		rtn = sensor_write_reg(0x1009, 0x28);
		rtn = sensor_write_reg(0x100b, 0x08);
        //rtn = imx627_sensor_dual_freq_init();
		sin_dual_freq = DUAL_FREQ;
		ALOGE("imx627_set_frequency_mode(uint8_t mode); set dual_freq: write_reg(0x0851, 0x01)");
		//imx627_set_fps(30);
	}
#endif
	return rtn;
}

int imx627_get_frequency_mode(uint8_t *mode)
{
	uint8_t value;

	//int rtn = sensor_read_reg(0x0851, &value);
	value = freq_mode;

	if (value == 0x00)
		*mode = SINGLE_FREQ; // single freq
	else if (value == 0x01)
		*mode = DUAL_FREQ; // dual freq
	else
		return -1;

	ALOGE("imx627_get_frequency_mode(uint8_t *mode); *mode: %d ", *mode);

	return 0;
}

static int imx627_AE(bool enable)
{
    int ret = 0;

    return ret;
}

static int get_binning_mode(uint8_t *mode_param)
{
	int rtn = 0;
	uint8_t reg_value1 = 0, reg_value2 = 0;

	rtn = sensor_read_reg(0x040C, &reg_value1);
	rtn = sensor_read_reg(0x6bcf, &reg_value2);

	if (reg_value2 == 0x2D)
	{
		if (reg_value1 == 0)
		{
			*mode_param = 0;  // no binning
		}
	}
	else if (reg_value2 == 0x52)
	{
		if (reg_value1 == 1)
		{
			*mode_param = 1;  // 2*2 binning
		}
		else if (reg_value1 == 2)
		{
			*mode_param = 2;  // 4*4 binning
		}
	}
	else
	{
		ALOGE("[REG ERROR] : reg_value2 %x ", reg_value2);
	}

	ALOGE("get_binning_mode(uint8_t *mode_param); *mode_param : %d ", *mode_param);

	return rtn;
}

static int set_binning_mode(uint8_t mode_param)
{
    int rtn = 0;

	if (mode_param == 0)  // no binning
	{
		rtn = sensor_write_reg(0x040C, 0x00);
		rtn = sensor_write_reg(0x6bcf, 0x2D);

		rtn = sensor_write_reg(0x0164, 0x0A);
		rtn = sensor_write_reg(0x0165, 0x00);
		rtn = sensor_write_reg(0x0166, 0x0A);
		rtn = sensor_write_reg(0x0167, 0x00);
	}
	else if (mode_param == 1)  // 2*2 binning
	{
		rtn = sensor_write_reg(0x040C, 0x01);
		//rtn = sensor_write_reg(0x040D, 0x01);
		rtn = sensor_write_reg(0x6bcf, 0x52);

		rtn = sensor_write_reg(0x0164, 0x05);
		rtn = sensor_write_reg(0x0165, 0x00);
		rtn = sensor_write_reg(0x0166, 0x05);
		rtn = sensor_write_reg(0x0167, 0x00);
	}
	else if (mode_param == 2)  // 4*4 binning
	{
		rtn = sensor_write_reg(0x040C, 0x02);
		rtn = sensor_write_reg(0x6bcf, 0x52);

		rtn = sensor_write_reg(0x0164, 0x02);
		rtn = sensor_write_reg(0x0165, 0x80);
		rtn = sensor_write_reg(0x0166, 0x02);
		rtn = sensor_write_reg(0x0167, 0x80);
	}
	else
	{
		ALOGE("[PARAM ERROR] : mode param %d ", mode_param);
	}

	ALOGE("set_binning_mode(uint8_t mode_param); mode_param : %d ", mode_param);

    return rtn;
}

int imx627_get_tof_sensor_resolution(uint64_t *resolution)
{
	if (sin_dual_freq == DUAL_FREQ)
	{
		*resolution = DUAL_FREQ_4X4_BINNING_RESOLUTION << 16 | DUAL_FREQ_2X2_BINNING_RESOLUTION << 8 | DUAL_FREQ_NO_BINNING_RESOLUTION;
	}
	else  // single freq
	{
		*resolution = SINGLE_FREQ_4X4_BINNING_RESOLUTION << 16 | SINGLE_FREQ_2X2_BINNING_RESOLUTION << 8 | SINGLE_FREQ_NO_BINNING_RESOLUTION;
	}

	ALOGE("imx627_get_tof_sensor_resolution(uint64_t *resolution); resolution: %d , single_dual freq: %d ", *resolution, sin_dual_freq);

    return 0;
}

int imx627_get_tof_sensor_pixel_bit(uint8_t *pixel_bit)
{
    *pixel_bit = PIXEL_BIT;
    return 0;
}

int imx627_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
    *mipi_pack_bit = MIPI_PACK_BIT;
    return 0;
}

int imx627_video_streaming(bool enable)
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(0x0101, 0x01);
	else
		rtn = sensor_write_reg(0x0101, 0x00);

	return rtn;
}

int imx627_init()
{
	int rtn = 0;

	rtn = imx627_sensor_init();

	//rtn = imx627_sensor_dual_freq_init();

#if (IMX627_REG_LIST_SELECT == IMX627_REG_LIST_6130)
	rtn = imx627_illum_power_test(1);
#endif
	//rtn = set_expander_tx_gpio(0, 1);

	return rtn;
}

int imx627_func_init()
{
	tof_sensor.init = imx627_init;
	tof_sensor.get_sensor_id = imx627_get_sensor_id;//
	tof_sensor.hardware_trigger = imx627_hardware_trigger;
	tof_sensor.software_trigger = imx627_software_trigger;
	tof_sensor.video_streaming = imx627_video_streaming;//
	tof_sensor.get_fps = imx627_get_fps;//
	tof_sensor.set_fps = imx627_set_fps;//
	tof_sensor.get_rx_temp = imx627_get_rx_temp;//
	tof_sensor.get_tx_temp = imx627_get_tx_temp;//
	tof_sensor.set_illum_power = imx627_set_illum_power;//
	tof_sensor.get_illum_power = imx627_get_illum_power;//
	tof_sensor.illum_power_control = imx627_illum_power_control;
	tof_sensor.illum_power_test = imx627_illum_power_test;
	tof_sensor.get_integration_time = imx627_get_integration_time;//
	tof_sensor.set_integration_time = imx627_set_integration_time;//
	tof_sensor.get_modulation_frequency = imx627_get_modulation_frequency;//
	tof_sensor.set_modulation_frequency = imx627_set_modulation_frequency;//
	tof_sensor.get_data_output_mode = imx627_get_data_output_mode;
	tof_sensor.set_data_output_mode = imx627_set_data_output_mode;
	tof_sensor.get_img_mirror_flip = imx627_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = imx627_set_img_mirror_flip;
	tof_sensor.test_pattern = imx627_test_pattern;
	tof_sensor.get_sensor_info = imx627_get_sensor_info;
	tof_sensor.set_tx_a_b_power = set_expander_tx_gpio;
	tof_sensor.get_tx_a_b_power = get_expander_tx_gpio;
	tof_sensor.get_frequency_and_duty = imx627_get_frequency_and_duty;
	tof_sensor.set_frequency_and_duty = imx627_set_frequency_and_duty;
	tof_sensor.get_frequency_mode = imx627_get_frequency_mode;
	tof_sensor.set_frequency_mode = imx627_set_frequency_mode;
	tof_sensor.get_vcsel_pd = imx627_get_vcsel_pd;
	tof_sensor.get_vcsel_error = imx627_get_vcsel_error;
	tof_sensor.sensor_write_reg_8 = sensor_write_reg;
	tof_sensor.sensor_read_reg_8 = sensor_read_reg;
	tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;//
	tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;//
	tof_sensor.AE = imx627_AE;
	tof_sensor.set_binning_mode = set_binning_mode;
	tof_sensor.get_binning_mode = get_binning_mode;
	tof_sensor.get_illum_duty_cycle_list = imx627_get_illum_duty_cycle_list;

    tof_sensor.get_tof_sensor_resolution = imx627_get_tof_sensor_resolution;
    tof_sensor.get_tof_sensor_pixel_bit = imx627_get_tof_sensor_pixel_bit;
    tof_sensor.get_mipi_pack_bit = imx627_get_mipi_pack_bit;

    //tof_sensor.get_illum_duty_cycle_list = imx627_get_illum_duty_cycle_list;
    tof_sensor.get_illum_duty_cycle = imx627_get_illum_duty_cycle;
    tof_sensor.set_illum_duty_cycle = imx627_set_illum_duty_cycle;

	return 0; 
}
