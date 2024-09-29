#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imx518.h"
#include "hw_obstatus.h"
#include "project_config.h"

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
#define IMX518_I2C_ADDR      0x20
#define EEPROM_I2C_ADDR      0xA0

// vcsel driver IC type
#define DRIVER_IC_CXA4026              4026 // Polaris Tx
#define DRIVER_IC_PHX3D_3021_AA        5016 // Taishan DVT2 Tx
#define DRIVER_IC_PHX3D_3021_CB        5017 // Taishan DVT3 Tx
#define DRIVER_IC_DW9912               9912 // DongWoon

#define DUAL_FREQ      1
#define SINGLE_FREQ    0
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

static int gpio_control(int pin, bool level)
{
	dothin_set_gpio_level(pin, level, dothin_device_id);
	return 0; // should wrap method by SDK
}

static int imx518_get_sensor_info(struct sensor_info_t *info) {

	info->embedded_data_size = 1280 * 1.5 * 3;
	info->vcsel_num = 1;
	info->vcsel_driver_id = driver_ic_type;
	info->sensor_id = imx518_sensor_id;
	return 0;
}

int imx518_dothin_config()
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
	ret = dothin_pmu_set_voltage(sensor_power, power_value, 5, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d", ret);
	}
	usleep(1000 * 50);
	DDEBUG("end");

	return ret;
}

int imx518_get_sensor_id(uint16_t *id) // checked
{
	int rtn = 0;
	uint8_t value = 0;
	static int isConfig = 0;
	rtn = gpio_expander_init();// power enable

	if (!isConfig) {
		isConfig = 1;
		imx518_dothin_config();
	}
	rtn = sensor_read_reg(0x0000, &value);

	if (value == 0x94 || value == 0x99 || value == 0x93) //0x93 -> IMX516 sensor
		*id = imx518_sensor_id;

	//ALOGE("imx518_sensor_id:%x,*id:%x", value, *id);
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
}

static int set_expander_tx_gpio(uint8_t A_status_enable, uint8_t B_status_enable)
{
	int rtn = 0;
	uint8_t gpio_status = 0;
#if 0 // Polaris A 方案一
	if (!A_status_enable) // area A cannot be switched off
		return -1;
	if (!B_status_enable) { // for safety. should decrease current before switched to A area 
		rtn = imx518_set_illum_power(1, 0x38, 0x00); // 1.55A -> 0x56,  1.0A -> 0x38
		rtn = vcsel_driver_write_reg(0x15, 0x9F); // 无电感 0x89，有电感 0x9F
		B_status = 0;
	}
	rtn = gpio_control(B_AREA_CTRL_PIN_QT, B_status_enable);
	rtn = gpio_control(B_AREA_CTRL_PIN_OB, B_status_enable);
	if (B_status_enable){  // for safety. should increase current after switched to A+B area 
		rtn = imx518_set_illum_power(1, 0xC2, 0x00); // 3.5A
		rtn = vcsel_driver_write_reg(0x15, 0x52);
		B_status = 1;
	}
#else // Polaris A 方案三
	if (A_status_enable && B_status_enable) // area A and B cannot be turned on at the same time
		return -1;
	if (A_status_enable) { // for safety. should decrease current before switched to A area 
		rtn = imx518_set_illum_power(1, 0x56, 0x00); // 1.55A -> 0x56,  1.0A -> 0x38  4.5A -> 0xFA
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
		rtn = imx518_set_illum_power(1, 0x92, 0x00); // 2.625A -> 0x92 4.5A -> 0xFA

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
}

static int eeprom_write_reg(uint16_t reg, uint16_t value)
{
	int ret = i2c_reg_write(EEPROM_I2C_ADDR, reg, 2, &value, 2);
	return ret;
}

static int eeprom_read_reg(uint16_t reg, uint16_t *value)
{
	int ret = i2c_reg_read(EEPROM_I2C_ADDR, reg, 2, value, 2);
	return ret;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
	int ret = i2c_reg_write(IMX518_I2C_ADDR, reg, 2, value, 1);
	return ret;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
	int ret = i2c_reg_read(IMX518_I2C_ADDR, reg, 2, value, 1);
	return ret;
}


static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
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

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
	int rtn = 0;
	// Rx buffer is taken by thermal channel, should disable it first
	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
	rtn = sensor_write_reg(0x0436, 0x1F); // start pointer of Rx buffer
	rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
	rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
	rtn = sensor_write_reg(0x0500, 0x02); // number of byte to be write and read
	rtn = sensor_write_reg(0x0501, reg + 0x80); // address to be read from laser (read address = laser addr + 0x80)
	rtn = sensor_write_reg(0x0431, 0x01); // enable
	rtn = sensor_write_reg(0x0430, 0x01); // trig
	rtn = sensor_write_reg(0x0431, 0x00); // disable

	rtn = sensor_read_reg(0x059F, value); // first byte in Rx buffer

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable

	return rtn;
}

int imx518_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0403, 0x20);
	rtn = sensor_write_reg(0x0405, 0x00);
	rtn = sensor_write_reg(0x0407, 0x00);
	rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write
	rtn = sensor_write_reg(0x0501, 0x07);
	rtn = sensor_write_reg(0x0502, value_B); // IBIAS
	rtn = sensor_write_reg(0x0503, value_A); // ISW
	rtn = sensor_write_reg(0x0401, 0x01);
	rtn = sensor_write_reg(0x0400, 0x01);
	rtn = sensor_write_reg(0x0401, 0x00);

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable

	rtn = vcsel_driver_write_reg(0x06, value_A); // ISW_APC_H2

	return rtn;
}

int imx518_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
	*vcsel_num = 1;
	rtn = sensor_write_reg(0x0411, 0x00); // send2 disable
	rtn = sensor_write_reg(0x0421, 0x00); // thermo disable

	rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
	rtn = sensor_write_reg(0x0436, 0x1E); // start pointer of Rx buffer
	rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
	rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
	rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write and read
	rtn = sensor_write_reg(0x0501, 0x07 + 0x80); // address to be read from laser (read address = laser addr 0x07 + 0x80)
	rtn = sensor_write_reg(0x0431, 0x01); // enable
	rtn = sensor_write_reg(0x0430, 0x01); // trig
	rtn = sensor_write_reg(0x0431, 0x00); // disable

	rtn = sensor_read_reg(0x059E, value_B); // first byte in Rx buffer
	rtn = sensor_read_reg(0x059F, value_A); // second byte in Rx buffer

	rtn = sensor_write_reg(0x0411, 0x01); // send2 enable
	rtn = sensor_write_reg(0x0421, 0x01); // thermo enable
	return rtn;
}

int imx518_illum_power_test(uint8_t mode)
{
	int rtn = 0;

	if (mode) { // illum power test mode, AA and QC used mode
        // AA  PuTuoShan  打开，Meizu/Polaris QC关闭
		//rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC

		rtn = sensor_write_reg(0x2144, 0xAB);// 0xAB: first phase on only, 0xFF four phase
		rtn = imx518_set_illum_power(1, 195, 0);
		rtn = imx518_set_frequency_mode(SINGLE_FREQ);
		rtn = imx518_set_fps(10); // 10fps equals to 20fps in single freq
		rtn = imx518_set_integration_time(500);
	}
	else {
		//rtn = vcsel_driver_write_reg(0x00, 0x0C); // enable APC
		rtn = sensor_write_reg(0x2144, 0x00); // normal modulation 
		rtn = imx518_set_frequency_mode(DUAL_FREQ);
	}

	return rtn;
}

int imx518_get_vcsel_pd(uint32_t *value) // driver ic pd value
{
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
}

int imx518_get_vcsel_error(uint16_t *value)
{
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
}

int imx518_get_rx_temp(float *temperature)
{
	uint32_t value;
	int rtn = sensor_read_reg(0x1403, &value);
	*temperature = (float)(value & 0xff) - 40;

	return rtn;
}

int imx518_get_tx_temp(float *temperature)
{
	int rtn = 0;
	
	uint8_t valueH = 0;
	uint8_t valueL = 0;
	if (driver_ic_type == DRIVER_IC_CXA4026) {
		rtn = sensor_read_reg(0x153d, &valueH);
		rtn = sensor_read_reg(0x153e, &valueL);
	}
	else if (driver_ic_type == DRIVER_IC_PHX3D_3021_CB || driver_ic_type == DRIVER_IC_DW9912) {
		rtn = sensor_read_reg(0x1547, &valueH);
		rtn = sensor_read_reg(0x1548, &valueL);
	}
	else {
		return -HW_ERR_INVALID;
	}
	uint16_t value = ((valueH & 0x03) << 8) + valueL;
	*temperature = 25 + (value - 297) / 5.4f;
	//printf("valueH %d, valueL %d\n", valueH, valueL);

	return rtn;
}

int imx518_hardware_trigger()
{/*
 //mdevices.gpio_control(TRIGGER_PIN, 1);
 mdevices.gpio_control(TRIGGER_PIN, 0); // active low
 mdevices.sleepms(0.001); // 1us, not accurate
 //mdevices.usleep(100);
 mdevices.gpio_control(TRIGGER_PIN, 1);
 */
	return 0;
}

int imx518_software_trigger()
{
	//int rtn = sensor_write_reg(0x2100, 0x01);
	return -1;
}

int imx518_illum_power_control(bool enable)
{
	//return gpio_control(LD_ENABLE_PIN, enable); // active high
	return 0;
}

struct regList imx518_reglist_6130[] = {
	{ 0x1006,0x18 },
	{ 0x1007,0x00 },
	{ 0x1000,0x00 },
	{ 0x2268,0x40 },
	{ 0x2269,0x01 },
	{ 0x226A,0x42 },
	{ 0x226B,0x11 },
	{ 0x400E,0x03 },
	{ 0x400F,0x84 },
	{ 0x402B,0x02 },
	{ 0x405C,0x01 },
	{ 0x4130,0x08 },
	{ 0x4131,0x02 },
	{ 0x4132,0x02 },
	{ 0x4133,0x02 },
	{ 0x4134,0x00 },
	{ 0x4135,0x02 },
	{ 0x4136,0x00 },
	{ 0x4137,0x02 },
	{ 0x4138,0x00 },
	{ 0x4139,0x02 },
	{ 0x413A,0x00 },
	{ 0x413B,0x02 },
	{ 0x413C,0x00 },
	{ 0x413D,0x02 },
	{ 0x443C,0x00 },
	{ 0x443D,0x79 },
	{ 0x443E,0x00 },
	{ 0x443F,0x8A },
	{ 0x4596,0x0B },
	{ 0x4597,0x45 },
	{ 0x45AE,0x0B },
	{ 0x45AF,0x45 },
	{ 0x45B6,0x0B },
	{ 0x45B7,0xBD },
	{ 0x45BA,0x0B },
	{ 0x45BB,0x45 },
	{ 0x45C6,0x0B },
	{ 0x45C7,0x45 },
	{ 0x45CE,0x0B },
	{ 0x45CF,0xBD },
	{ 0x47B4,0x00 },
	{ 0x47B5,0x56 },
	{ 0x47BE,0x00 },
	{ 0x47BF,0x3C },
	{ 0x47C4,0x00 },
	{ 0x47C5,0x00 },
	{ 0x47C6,0x03 },
	{ 0x47C7,0xFF },
	{ 0x47D0,0x00 },
	{ 0x47D1,0x54 },
	{ 0x47DA,0x00 },
	{ 0x47DB,0x3F },
	{ 0x47DC,0x00 },
	{ 0x47DD,0x00 },
	{ 0x47DE,0x03 },
	{ 0x47DF,0xFF },
	{ 0x493C,0x07 },
	{ 0x494E,0x02 },
	{ 0x4998,0x01 },
	{ 0x4999,0x01 },
	{ 0x4A01,0x01 },
	{ 0x4A05,0x01 },
	{ 0x4C00,0x81 },
	{ 0x4C01,0xCC },
	{ 0x4C07,0x1F },
	{ 0x4C08,0x21 },
	{ 0x4C10,0x07 },
	{ 0x5852,0x0A },
	{ 0x5853,0xD5 },
	{ 0x5854,0x0A },
	{ 0x5855,0xDB },
	{ 0x5856,0x0B },
	{ 0x5857,0x43 },
	{ 0x586D,0x00 },
	{ 0x586E,0x2E },
	{ 0x586F,0xEB },
	{ 0x5881,0x00 },
	{ 0x5882,0x3B },
	{ 0x5883,0x28 },
	{ 0x5885,0x00 },
	{ 0x5886,0x05 },
	{ 0x5887,0x14 },
	{ 0x5889,0x00 },
	{ 0x588A,0x03 },
	{ 0x588B,0xAC },
	{ 0x45F8,0x1F },
	{ 0x45F9,0xFF },
	{ 0x104B,0x02 },

	{ 0x1040,0x00 },
	{ 0x1041,0x78 },
	{ 0x1042,0x02 },
	{ 0x1048,0x00 },
	{ 0x1049,0x78 },
	{ 0x104a,0x03 },

	{ 0x100c,0x0B },
	{ 0x100d,0x40 },
	{ 0x100e,0x00 },
	{ 0x100f,0x00 },
	{ 0x1016,0x02 },
	{ 0x1017,0x00 },
	{ 0x1060,0x00 },
	{ 0x1070,0x06 },
	{ 0x1071,0x06 },
	{ 0x020e,0x00 },
	{ 0x020f,0x96 },
	{ 0x1010,0x01 },

	{ 0x0800,0x02 },
	{ 0x0801,0xDC },
	{ 0x2108,0x02 },
	{ 0x2109,0x58 },
	{ 0x4015,0x00 },
	{ 0x4016,0x36 },
	{ 0x4078,0x00 },
	{ 0x4079,0x00 },
	{ 0x407a,0x00 },
	{ 0x407b,0x1A },
	{ 0x4080,0x00 },
	{ 0x4081,0x00 },
	{ 0x4082,0x00 },
	{ 0x4083,0x36 },

	{ 0x082C,0x22 },
	{ 0x082D,0x22 },
	{ 0x082E,0x22 },
	{ 0x082F,0x22 },
	{ 0x0830,0x22 },
	{ 0x0831,0x22 },
	{ 0x0832,0x22 },
	{ 0x0833,0x22 },
	{ 0x0834,0x22 },
	{ 0x0835,0x22 },
	{ 0x0836,0x22 },
	{ 0x0837,0x22 },
	{ 0x0838,0x33 },
	{ 0x0839,0x33 },
	{ 0x083a,0x33 },
	{ 0x083b,0x33 },
	{ 0x083c,0x33 },
	{ 0x083d,0x33 },
	{ 0x083e,0x33 },
	{ 0x083f,0x33 },
	{ 0x0840,0x33 },
	{ 0x0841,0x33 },
	{ 0x0842,0x33 },
	{ 0x0843,0x33 },
	{ 0x0844,0xFF },
	{ 0x0848,0xFF },
	{ 0x084c,0xFF },
	{ 0x213c,0x00 },
	{ 0x213d,0x00 },
	{ 0x2140,0x00 },
	{ 0x2141,0x00 },
	{ 0x2144,0x00 },
	{ 0x2145,0x00 },

	{ 0x1433,0x01 },
	{ 0x149b,0x00 },
	{ 0x1434,0x00 },
	{ 0x1435,0x00 },
	{ 0x1436,0x00 },
	{ 0x1437,0x00 },
	{ 0x1438,0x00 },
	{ 0x1439,0x00 },
	{ 0x143c,0x06 },
	{ 0x143d,0x40 },
	{ 0x143e,0x06 },
	{ 0x143f,0x40 },
	{ 0x1440,0x07 },
	{ 0x1441,0xFF },

	{ 0x2c08,0x00 },
	{ 0x2c09,0x80 },
	{ 0x3c18,0x03 },
	{ 0x2c0c,0x01 },
	{ 0x0804,0x00 },
	{ 0x0805,0x04 },
	{ 0x0806,0x02 },
	{ 0x0807,0x80 },
	{ 0x0808,0x00 },
	{ 0x0809,0x04 },
	{ 0x080a,0x01 },
	{ 0x080b,0xE3 },
	{ 0x080c,0x00 },
	{ 0x080d,0x00 },
	{ 0x080e,0x00 },

	{ 0x0810,0x00 },
	{ 0x0811,0x00 },

	{ 0x2247,0x0C },
	{ 0x2248,0x00 },
	{ 0x2249,0x00 },
	{ 0x224a,0x00 },
	{ 0x2254,0x01 },
	{ 0x2255,0x00 },
	{ 0x2256,0x00 },

	{ 0x2124,0x00 },
	{ 0x2125,0x00 },
	{ 0x2126,0xEA },
	{ 0x2127,0x60 },
	{ 0x2128,0x00 },
	{ 0x2129,0x00 },
	{ 0x212a,0x09 },
	{ 0x212b,0x60 },
	{ 0x212c,0x00 },
	{ 0x212d,0x00 },
	{ 0x212e,0x09 },
	{ 0x212f,0x60 },

	{ 0x2118,0x00 },
	{ 0x2119,0x00 },
	{ 0x211a,0x00 },
	{ 0x211b,0x00 },
	{ 0x211c,0x00 },
	{ 0x211d,0x00 },
	{ 0x211e,0x00 },
	{ 0x211f,0x00 },
	{ 0x2120,0x00 },
	{ 0x2121,0x00 },
	{ 0x2122,0x00 },
	{ 0x2123,0x00 },

	{ 0x215c,0x04 },
	{ 0x215d,0x04 },
	{ 0x215e,0x04 },
	{ 0x214c,0x00 },
	{ 0x214d,0x01 },
	{ 0x214e,0x00 },
	{ 0x217c,0x04 },
	{ 0x217d,0x0E },
	{ 0x217e,0x04 },

	{ 0x210c,0x00 },
	{ 0x210d,0x00 },
	{ 0x210e,0x1A },
	{ 0x210f,0x0B },
	{ 0x2110,0x00 },
	{ 0x2111,0x00 },
	{ 0x2112,0x00 },
	{ 0x2113,0x00 },
	{ 0x2114,0x00 },
	{ 0x2115,0x00 },
	{ 0x2116,0x00 },
	{ 0x2117,0x00 },
	{ 0x2100,0x08 },
	{ 0x0828,0x30 },
	{ 0x2164,0x00 },
	{ 0x2165,0x00 },
	{ 0x2168,0x00 },
	{ 0x2169,0x00 },
	{ 0x216c,0x00 },
	{ 0x216d,0x00 },

	{ 0x2184,0x0D },
	{ 0x2188,0x06 },
	{ 0x2189,0x08 },
	{ 0x218a,0x0A },
	{ 0x218b,0x0C },
	{ 0x2190,0x06 },
	{ 0x2191,0x0D },
	{ 0x2192,0x14 },
	{ 0x2193,0x1B },
	{ 0x2198,0x06 },
	{ 0x2199,0x08 },
	{ 0x219a,0x0A },
	{ 0x219b,0x0C },

	{ 0x2244,0x00 },
	{ 0x2245,0x00 },
	{ 0x2246,0x00 },
	{ 0x21b8,0x02 },
	{ 0x21b9,0x07 },
	{ 0x21ba,0x02 },

	{ 0x0145,0x00 },
	{ 0x0148,0x01 },
	{ 0x2020,0x01 },
	{ 0x3071,0x00 },
	{ 0x2f05,0x01 },
	{ 0x2f06,0x00 },
	{ 0x2f07,0x07 },
	{ 0x2001,0x01 },

	// user added
	{ 0x1433,0x00 },// disable pixel error flag
	{ 0x2C08,0x05 },// EBD length  1280*1.5 Byte
	{ 0x2C09,0x00 },

	{ 0x2144,0x00 },// 0x00: laser normal modulation, 0xFF直流
	{ 0x0450,0x47 },// spi master config
};

struct regList imx518_reglist_6272[] = {
	{ 0x1006,0x18 },
	{ 0x1007,0x00 },

	{ 0x1000,0x00 },
	{ 0x2268,0x40 },
	{ 0x2269,0x01 },
	{ 0x226A,0x42 },
	{ 0x226B,0x11 },
	{ 0x400E,0x03 },
	{ 0x400F,0x84 },
	{ 0x402B,0x02 },
	{ 0x405C,0x01 },
	{ 0x4130,0x08 },
	{ 0x4131,0x02 },
	{ 0x4132,0x02 },
	{ 0x4133,0x02 },
	{ 0x4134,0x00 },
	{ 0x4135,0x02 },
	{ 0x4136,0x00 },
	{ 0x4137,0x02 },
	{ 0x4138,0x00 },
	{ 0x4139,0x02 },
	{ 0x413A,0x00 },
	{ 0x413B,0x02 },
	{ 0x413C,0x00 },
	{ 0x413D,0x02 },
	{ 0x443C,0x00 },
	{ 0x443D,0x79 },
	{ 0x443E,0x00 },
	{ 0x443F,0x8A },
	{ 0x4596,0x0B },
	{ 0x4597,0x45 },
	{ 0x45AE,0x0B },
	{ 0x45AF,0x45 },
	{ 0x45B6,0x0B },
	{ 0x45B7,0xBD },
	{ 0x45BA,0x0B },
	{ 0x45BB,0x45 },
	{ 0x45C6,0x0B },
	{ 0x45C7,0x45 },
	{ 0x45CE,0x0B },
	{ 0x45CF,0xBD },
	{ 0x47B4,0x00 },
	{ 0x47B5,0x56 },
	{ 0x47BE,0x00 },
	{ 0x47BF,0x3C },
	{ 0x47C4,0x00 },
	{ 0x47C5,0x00 },
	{ 0x47C6,0x03 },
	{ 0x47C7,0xFF },
	{ 0x47D0,0x00 },
	{ 0x47D1,0x54 },
	{ 0x47DA,0x00 },
	{ 0x47DB,0x3F },
	{ 0x47DC,0x00 },
	{ 0x47DD,0x00 },
	{ 0x47DE,0x03 },
	{ 0x47DF,0xFF },
	{ 0x493C,0x07 },
	{ 0x494E,0x02 },
	{ 0x4998,0x01 },
	{ 0x4999,0x01 },
	{ 0x4A01,0x01 },
	{ 0x4A05,0x01 },
	{ 0x4C07,0x1F },
	{ 0x5852,0x0A },
	{ 0x5853,0xD5 },
	{ 0x5854,0x0A },
	{ 0x5855,0xDB },
	{ 0x5856,0x0B },
	{ 0x5857,0x43 },
	{ 0x586D,0x00 },
	{ 0x586E,0x2E },
	{ 0x586F,0xEB },
	{ 0x5881,0x00 },
	{ 0x5882,0x3B },
	{ 0x5883,0x28 },
	{ 0x5885,0x00 },
	{ 0x5886,0x05 },
	{ 0x5887,0x14 },
	{ 0x5889,0x00 },
	{ 0x588A,0x03 },
	{ 0x588B,0xAC },
	{ 0x45F8,0x1F },
	{ 0x45F9,0xFF },

	{ 0x1040,0x00 },
	{ 0x1041,0x78 },
	{ 0x1042,0x02 },
	{ 0x1048,0x00 },
	{ 0x1049,0x78 },
	{ 0x104a,0x03 },
	{ 0x104b,0x00 },

	{ 0x100c,0x0B },
	{ 0x100d,0x40 },
	{ 0x100e,0x00 },
	{ 0x100f,0x00 },
	{ 0x1016,0x02 },
	{ 0x1017,0x00 },
	{ 0x1060,0x00 },
	{ 0x1070,0x06 },
	{ 0x1071,0x06 },
	{ 0x020e,0x00 },
	{ 0x020f,0x96 },
	{ 0x1010,0x01 },

	{ 0x0800,0x02 },
	{ 0x0801,0xDC },
	{ 0x2108,0x02 },
	{ 0x2109,0x58 },
	{ 0x4015,0x00 },
	{ 0x4016,0x36 },
	{ 0x4078,0x00 },
	{ 0x4079,0x00 },
	{ 0x407a,0x00 },
	{ 0x407b,0x1A },
	{ 0x4080,0x00 },
	{ 0x4081,0x00 },
	{ 0x4082,0x00 },
	{ 0x4083,0x36 },

	{ 0x082C,0x22 },
	{ 0x082D,0x22 },
	{ 0x082E,0x22 },
	{ 0x082F,0x22 },
	{ 0x0830,0x22 },
	{ 0x0831,0x22 },
	{ 0x0832,0x22 },
	{ 0x0833,0x22 },
	{ 0x0834,0x22 },
	{ 0x0835,0x22 },
	{ 0x0836,0x22 },
	{ 0x0837,0x22 },
	{ 0x0838,0x33 },
	{ 0x0839,0x33 },
	{ 0x083a,0x33 },
	{ 0x083b,0x33 },
	{ 0x083c,0x33 },
	{ 0x083d,0x33 },
	{ 0x083e,0x33 },
	{ 0x083f,0x33 },
	{ 0x0840,0x33 },
	{ 0x0841,0x33 },
	{ 0x0842,0x33 },
	{ 0x0843,0x33 },
	{ 0x0844,0xFF },
	{ 0x0848,0xFF },
	{ 0x084c,0xFF },
	{ 0x213c,0x00 },
	{ 0x213d,0x00 },
	{ 0x2140,0x00 },
	{ 0x2141,0x00 },
	{ 0x2144,0x00 },
	{ 0x2145,0x00 },

	{ 0x1433,0x01 },
	{ 0x149b,0x00 },
	{ 0x1434,0x00 },
	{ 0x1435,0x00 },
	{ 0x1436,0x00 },
	{ 0x1437,0x00 },
	{ 0x1438,0x00 },
	{ 0x1439,0x00 },
	{ 0x143c,0x06 },
	{ 0x143d,0x40 },
	{ 0x143e,0x06 },
	{ 0x143f,0x40 },
	{ 0x1440,0x07 },
	{ 0x1441,0xFF },

	{ 0x2c08,0x00 },
	{ 0x2c09,0x80 },
	{ 0x3c18,0x03 },
	{ 0x2c0c,0x01 },
	{ 0x0804,0x00 },
	{ 0x0805,0x04 },
	{ 0x0806,0x02 },
	{ 0x0807,0x80 },
	{ 0x0808,0x00 },
	{ 0x0809,0x04 },
	{ 0x080a,0x01 },
	{ 0x080b,0xE3 },
	{ 0x080c,0x00 },
	{ 0x080d,0x00 },
	{ 0x080e,0x00 },

	{ 0x0810,0x00 },
	{ 0x0811,0x00 },

	{ 0x2247,0x0C },
	{ 0x2248,0x00 },
	{ 0x2249,0x00 },
	{ 0x224a,0x00 },
	{ 0x2254,0x05 },
	{ 0x2255,0x02 },
	{ 0x2256,0x00 },

	{ 0x2124,0x00 },
	{ 0x2125,0x00 },
	{ 0x2126,0x75 },
	{ 0x2127,0x30 },
	{ 0x2128,0x00 },
	{ 0x2129,0x00 },
	{ 0x212a,0x75 },
	{ 0x212b,0x30 },
	{ 0x212c,0x00 },
	{ 0x212d,0x00 },
	{ 0x212e,0x09 },
	{ 0x212f,0x60 },

	{ 0x2118,0x00 },
	{ 0x2119,0x00 },
	{ 0x211a,0x00 },
	{ 0x211b,0x00 },
	{ 0x211c,0x00 },
	{ 0x211d,0x00 },
	{ 0x211e,0x00 },
	{ 0x211f,0x00 },
	{ 0x2120,0x00 },
	{ 0x2121,0x00 },
	{ 0x2122,0x00 },
	{ 0x2123,0x00 },

	{ 0x215c,0x04 },
	{ 0x215d,0x04 },
	{ 0x215e,0x04 },
	{ 0x214c,0x02 },
	{ 0x214d,0x00 },
	{ 0x214e,0x00 },
	{ 0x217c,0x08 },
	{ 0x217d,0x04 },
	{ 0x217e,0x04 },

	{ 0x210c,0x00 },
	{ 0x210d,0x00 },
	{ 0x210e,0x0C },
	{ 0x210f,0xF4 },
	{ 0x2110,0x00 },
	{ 0x2111,0x00 },
	{ 0x2112,0x0D },
	{ 0x2113,0x13 },
	{ 0x2114,0x00 },
	{ 0x2115,0x00 },
	{ 0x2116,0x00 },
	{ 0x2117,0x00 },
	{ 0x2100,0x48 },
	{ 0x0828,0x72 },
	{ 0x2164,0x00 },
	{ 0x2165,0x00 },
	{ 0x2168,0x00 },
	{ 0x2169,0x00 },
	{ 0x216c,0x00 },
	{ 0x216d,0x00 },

	{ 0x2184,0x0D },
	{ 0x2188,0x06 },
	{ 0x2189,0x0A },
	{ 0x218a,0x0E },
	{ 0x218b,0x12 },
	{ 0x2190,0x06 },
	{ 0x2191,0x08 },
	{ 0x2192,0x0A },
	{ 0x2193,0x0C },
	{ 0x2198,0x06 },
	{ 0x2199,0x08 },
	{ 0x219a,0x0A },
	{ 0x219b,0x0C },

	{ 0x2244,0x01 },
	{ 0x2245,0x00 },
	{ 0x2246,0x00 },
	{ 0x21b8,0x06 },
	{ 0x21b9,0x07 },
	{ 0x21ba,0x02 },

	{ 0x0145,0x00 },
	{ 0x0148,0x01 },
	{ 0x2020,0x01 },
	{ 0x3071,0x00 },
	{ 0x2f05,0x01 },
	{ 0x2f06,0x00 },
	{ 0x2f07,0x00 },
	{ 0x2001,0x01 },


	// user added
	{ 0x1433,0x00 },// disable pixel error flag
	{ 0x2C08,0x05 },// EBD length  1280*1.5 Byte
	{ 0x2C09,0x00 },
	{ 0x0450,0x47 },// spi master config


	// 100M_60M
	{ 0x1049, 0x96 }, // IMXCK = 150
	{ 0x214C, 0x00 }, // GroupA predivsel
	{ 0x217C, 0x06 }, // GroupA fmodsel
	{ 0x214D, 0x00 }, // GroupB predivsel
	{ 0x217D, 0x0A }, // GroupB fmodsel

	{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
	{ 0x2189, 0x09 }, // phase increment = GroupA fmodsel / 2 = 3
	{ 0x218A, 0x0C },
	{ 0x218B, 0x0F },
	{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
	{ 0x2191, 0x0B }, // phase increment = GroupB fmodsel / 2 = 5
	{ 0x2192, 0x10 },
	{ 0x2193, 0x15 },

	{ 0x2244, 0x00 }, // GropuA LSRDTYSEL
	{ 0x21B8, 0x04 }, // GroupA CTRL_LSRADD, duty = 50%
	{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
	{ 0x21B9, 0x09 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 9 / (10 * 2) = 45%

	{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
	{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
	{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step

	//{ 0x1001,0x01 },

};

static struct regList cxa4026_reglist_acc[] = { // ACC mode
	// Laser Driver(CXA4026) reset
	{ 0x0403,0x20 },
	{ 0x0405,0x01 },
	{ 0x0500,0x02 },
	{ 0x0501,0x2D }, // reset
	{ 0x0502,0x01 },

	// Sony Laser Driver (CXA4026)  initial
	{ 0x0503,0x19 },
	{ 0x0504,0x00 },
	{ 0x0505,0x0C }, // 0x00
	{ 0x0506,0x00 },
	{ 0x0507,0x00 },
	{ 0x0508,0x00 },
	{ 0x0509,0x00 },
	{ 0x050A,0x00 },
	{ 0x050B,0x00 },
	{ 0x050C,0x00 },
	{ 0x050D,0x52 }, // 0x08        0x52 -> 1.55A 0xFA -> 4.5A
	{ 0x050E,0x00 },
	{ 0x050F,0x04 }, // 0x0A
	{ 0x0510,0x00 },
	{ 0x0511,0x28 }, // 0x0C
	{ 0x0512,0x00 },
	{ 0x0513,0x84 }, // 0x0E PD resistance
	{ 0x0514,0xC0 },
	{ 0x0515,0xFF }, // 0x10
	{ 0x0516,0x80 },
	{ 0x0517,0x00 },
	{ 0x0518,0x00 },
#if (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6130)
	{ 0x0519,0x20 }, // 0x14  R_ITO = 100kΩ
#else
	{ 0x0519,0x00 }, // 0x14  R_ITO disabled
#endif
	{ 0x051a,0x93 }, // 0x15
	{ 0x051b,0x08 },
	{ 0x051c,0x01 },
	{ 0x0401,0x01 },
	{ 0x0400,0x01 },
	{ 0x0401,0x00 },

	
	// Sony Laser Driver (CXA4026) Temperature and Embedded Data
	{ 0x0413,0xA0 }, // send2 channel start by internal sync signal
	{ 0x0417,0x1d }, // send2 channel start pointer of TX buffer

	{ 0x0423,0xA0 }, // thermal channel start by internal sync signal
	{ 0x0425,0x07 }, // thermal channel transaction x 8 
	{ 0x0426,0x00 }, // thermal channel start pointer of RX buffer
	{ 0x0427,0x20 }, // thermal channel start pointer of TX buffer
	{ 0x042a,0x01 }, // thermal channel 1st byte is tx, after rx

	{ 0x051d,0x02 },
	{ 0x051e,0x17 },
	{ 0x051f,0x21 }, // send2 channel start ADC

	{ 0x0520,0x02 }, // thermal channel 1st transaction, 1 tx byte, 1 rx byte
	{ 0x0521,0xA9 }, // driver IC addr 0x29 (0xA9 - 0x80)
	{ 0x0522,0x08 }, // thermal channel 2nd transaction, 1 tx byte, 7 rx byte
	{ 0x0523,0x98 }, // driver IC addr 0x18 (0x98 - 0x80)
	{ 0x0524,0x02 }, // thermal channel 3rd transaction, 1 tx byte, 1 rx byte
	{ 0x0525,0xA3 }, // driver IC addr 0x23 (0xA3 - 0x80)
	{ 0x0526,0x05 }, // ..... and so on
	{ 0x0527,0x9F },
	{ 0x0528,0x06 },
	{ 0x0529,0xA4 },
	{ 0x052a,0x04 },
	{ 0x052b,0x80 },
	{ 0x052c,0x08 },
	{ 0x052d,0x85 },
	{ 0x052e,0x02 },
	{ 0x052f,0x8F },
	{ 0x0411,0x01 },
	{ 0x0421,0x01 },
	/**/
};


static struct regList cxa4026_reglist_apc[] = { // APC mode
	// Laser Driver(CXA4026) reset
	{ 0x0403,0x20 },
	{ 0x0405,0x01 },
	{ 0x0450,0x47 },
	{ 0x0500,0x02 },
	{ 0x0501,0x2D }, // reset
	{ 0x0502,0x01 },

	// Sony Laser Driver (CXA4026)  initial
	{ 0x0503,0x19 },
	{ 0x0504,0x00 },
	{ 0x0505,0x08 }, // 0x00
	{ 0x0506,0x00 },
	{ 0x0507,0x00 },
	{ 0x0508,0x00 },
	{ 0x0509,0x00 },
	{ 0x050A,0xA2 },
	{ 0x050B,0xC2 },
	{ 0x050C,0x00 },
	{ 0x050D,0xC2 }, // 0x08
	{ 0x050E,0x00 },
	{ 0x050F,0x05 }, // 0x0A
	{ 0x0510,0x81 },
	{ 0x0511,0x28 }, // 0x0C
	{ 0x0512,0xCD },
	{ 0x0513,0x88 },
	{ 0x0514,0x3D },
	{ 0x0515,0x20 },
	{ 0x0516,0xFF },
	{ 0x0517,0x1F },
	{ 0x0518,0x49 },
	{ 0x0519,0x00 },
	{ 0x051a,0x93 },
	{ 0x051b,0x01 },
	{ 0x051c,0x01 },
	{ 0x0401,0x01 },
	{ 0x0400,0x01 },
	{ 0x0401,0x00 },


	// Sony Laser Driver (CXA4026) Temperature and Embedded Data
	{ 0x0413,0xA0 }, // send2 channel start by internal sync signal
	{ 0x0417,0x1d }, // send2 channel start pointer of TX buffer

	{ 0x0423,0xA0 }, // thermal channel start by internal sync signal
	{ 0x0425,0x07 }, // thermal channel transaction x 8 
	{ 0x0426,0x00 }, // thermal channel start pointer of RX buffer
	{ 0x0427,0x20 }, // thermal channel start pointer of TX buffer
	{ 0x042a,0x01 }, // thermal channel 1st byte is tx, after rx

	{ 0x051d,0x02 },
	{ 0x051e,0x17 },
	{ 0x051f,0x21 }, // send2 channel start ADC

	{ 0x0520,0x02 }, // thermal channel 1st transaction, 1 tx byte, 1 rx byte
	{ 0x0521,0xA9 }, // driver IC addr 0x29 (0xA9 - 0x80)
	{ 0x0522,0x08 }, // thermal channel 2nd transaction, 1 tx byte, 7 rx byte
	{ 0x0523,0x98 }, // driver IC addr 0x18 (0x98 - 0x80)
	{ 0x0524,0x02 }, // thermal channel 3rd transaction, 1 tx byte, 1 rx byte
	{ 0x0525,0xA3 }, // driver IC addr 0x23 (0xA3 - 0x80)
	{ 0x0526,0x05 }, // ..... and so on
	{ 0x0527,0x9F },
	{ 0x0528,0x06 },
	{ 0x0529,0xA4 },
	{ 0x052a,0x04 },
	{ 0x052b,0x80 },
	{ 0x052c,0x08 },
	{ 0x052d,0x85 },
	{ 0x052e,0x02 },
	{ 0x052f,0x8F },
	{ 0x0411,0x01 },
	{ 0x0421,0x01 },

};

// PHX3D 3025 almost the same with PHX3D 3021 except XCLR mask control function
// (PHX3D 3025, register 0x2E bit 0 should set to 1, while PHX3D 3021 don't care this)
static struct regList phx3d_3021_cb_reglist[] = {
	//Laser Driver(PHX3D) reset
#if 1  //0515 project 2020/12 month trial-produce
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 }, // 0x25
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//PhotonIC Laser Driver (PHX3D)  initial
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x03 },
	{ 0x0503,  0x15 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // 0x00
	{ 0x0506,  0x20 }, // 0x01
	{ 0x0507,  0x30 }, // 0x02
	{ 0x0508,  0x10 }, // 0x03
	{ 0x0509,  0x20 }, // 0x04
	{ 0x050A,  0x69 }, // 0x05
	{ 0x050B,  0xC3 }, // 0x06 ISW_APCH2, PD_H2 value is decided by this current, keep the same with ISW_FIX
	{ 0x050C,  0x00 }, // 0x07 IBIAS_FIX
	{ 0x050D,  0xC3 }, // 0x08 ISW_FIX
	{ 0x050E,  0x00 }, // 0x09
	{ 0x050F,  0x0D }, // 0x0A
	{ 0x0510,  0xFF }, // 0x0B
	{ 0x0511,  0xD9 }, // 0x0C
	{ 0x0512,  0xC0 }, // 0x0D IPD_OFFSET[2:0]  off
	{ 0x0513,  0x78 }, // 0x0E ISW_FIX threshold
	{ 0x0514,  0x04 }, // 0x0F
	{ 0x0515,  0xDF }, // 0x10
	{ 0x0516,  0x00 }, // 0x11
	{ 0x0517,  0x00 }, // 0x12
	{ 0x0518,  0x01 }, // 0x13
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x19 },
	{ 0x0519,  0x02 },
	{ 0x051A,  0x26 },
	{ 0x051B,  0x10 }, // 0x26  internal PD resistance
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x22 },
	{ 0x0522,  0x09 },
	{ 0x0523,  0x2B },
	{ 0x0524,  0xFF }, // 0x2B
	{ 0x0525,  0x00 }, // 0x2C
	{ 0x0526,  0x03 }, // 0x2D
	{ 0x0527,  0x03 }, // 0x2E
	{ 0x0528,  0x00 }, // 0x2F
	{ 0x0529,  0x00 }, // 0x30   duty increase
	{ 0x052A,  0x00 }, // 0x31   duty decrease
	{ 0x052B,  0xFF }, // 0x32   falling edge speed control
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },


	// PHX3D Laser Driver Temperature and Embedded Data
	{ 0x0413,0xA0 }, // send2 channel start by internal sync signal
	{ 0x0417,0x23 }, // send2 channel start pointer of TX buffer

	{ 0x0423,0xA0 }, // thermal channel start by internal sync signal
	{ 0x0425,0x01 }, // thermal channel transaction x 2
	{ 0x0426,0x00 }, // thermal channel start pointer of RX buffer
	{ 0x0427,0x26 }, // thermal channel start pointer of TX buffer
	{ 0x042a,0x01 }, // thermal channel 1st byte is tx, after rx

	{ 0x0523,0x02 },
	{ 0x0524,0x13 },
	{ 0x0525,0x21 }, // send2 channel start ADC

					 // read driver IC addr[0x00 - 0x0A] correspond to sensor rx buffer[0x0580 - 0x058A]
	{ 0x0526,0x0C }, // thermal channel 1st transaction, 1 tx byte, 11 rx byte
	{ 0x0527,0x80 }, // driver IC addr 0x00 (0x80 - 0x80)
					 // read driver IC addr[0x14 - 0x24] correspond to sensor rx buffer[0x058B - 0x059B]
	{ 0x0528,0x13 }, // thermal channel 2nd transaction, 1 tx byte, 18 rx byte
	{ 0x0529,0x94 }, // driver IC addr 0x14 (0x94 - 0x80)


	{ 0x0411,0x01 },
	{ 0x0421,0x01 },

#else
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 }, // 0x25
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//PhotonIC Laser Driver (PHX3D)  initial
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x03 },
	{ 0x0503,  0x15 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // 0x00
	{ 0x0506,  0x20 }, // 0x01
	{ 0x0507,  0x30 }, // 0x02
	{ 0x0508,  0x10 }, // 0x03
	{ 0x0509,  0x20 }, // 0x04
	{ 0x050A,  0x69 }, // 0x05
	{ 0x050B,  0xC2 }, // 0x06 ISW_APCH2, PD_H2 value is decided by this current, keep the same with ISW_FIX
	{ 0x050C,  0x00 }, // 0x07 IBIAS_FIX
	{ 0x050D,  0xC2 }, // 0x08 ISW_FIX
	{ 0x050E,  0x24 }, // 0x09
	{ 0x050F,  0x0D }, // 0x0A
	{ 0x0510,  0xFF }, // 0x0B
	{ 0x0511,  0xD9 }, // 0x0C
	{ 0x0512,  0xC2 }, // 0x0D IPD_OFFSET[2:0]  40uA
	{ 0x0513,  0x78 }, // 0x0E ISW_FIX threshold
	{ 0x0514,  0x04 }, // 0x0F
	{ 0x0515,  0xDF }, // 0x10
	{ 0x0516,  0x00 }, // 0x11
	{ 0x0517,  0x00 }, // 0x12
	{ 0x0518,  0x01 }, // 0x13
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x19 },
	{ 0x0519,  0x02 },
	{ 0x051A,  0x26 },
	{ 0x051B,  0x19 }, // 0x26  internal PD resistance
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x22 },
	{ 0x0522,  0x09 },
	{ 0x0523,  0x2B },
	{ 0x0524,  0xF1 }, // 0x2B
	{ 0x0525,  0xF8 }, // 0x2C
	{ 0x0526,  0x03 }, // 0x2D
	{ 0x0527,  0x03 }, // 0x2E
	{ 0x0528,  0x00 }, // 0x2F
	{ 0x0529,  0x00 }, // 0x30   duty increase
	{ 0x052A,  0x00 }, // 0x31   duty decrease
	{ 0x052B,  0xFF }, // 0x32   falling edge speed control
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },


	// PHX3D Laser Driver Temperature and Embedded Data
	{ 0x0413,0xA0 }, // send2 channel start by internal sync signal
	{ 0x0417,0x23 }, // send2 channel start pointer of TX buffer

	{ 0x0423,0xA0 }, // thermal channel start by internal sync signal
	{ 0x0425,0x01 }, // thermal channel transaction x 2
	{ 0x0426,0x00 }, // thermal channel start pointer of RX buffer
	{ 0x0427,0x26 }, // thermal channel start pointer of TX buffer
	{ 0x042a,0x01 }, // thermal channel 1st byte is tx, after rx

	{ 0x0523,0x02 },
	{ 0x0524,0x13 },
	{ 0x0525,0x21 }, // send2 channel start ADC

					 // read driver IC addr[0x00 - 0x0A] correspond to sensor rx buffer[0x0580 - 0x058A]
	{ 0x0526,0x0C }, // thermal channel 1st transaction, 1 tx byte, 11 rx byte
	{ 0x0527,0x80 }, // driver IC addr 0x00 (0x80 - 0x80)
					 // read driver IC addr[0x14 - 0x24] correspond to sensor rx buffer[0x058B - 0x059B]
	{ 0x0528,0x13 }, // thermal channel 2nd transaction, 1 tx byte, 18 rx byte
	{ 0x0529,0x94 }, // driver IC addr 0x14 (0x94 - 0x80)


	{ 0x0411,0x01 },
	{ 0x0421,0x01 },
#endif
};

static struct regList dw9912_reglist[] = {

    // Laser Driver reset
    { 0x0403,  0x20 },
    { 0x0405,  0x03 },
    { 0x0450,  0x47 },
    { 0x0500,  0x02 },
    { 0x0501,  0x25 },
    { 0x0502,  0x01 }, // 0x25
    { 0x0503,  0x0A },
    { 0x0504,  0x00 },
    { 0x0505,  0x0C }, // 0x00
    { 0x0506,  0x23 }, // 0x01
    { 0x0507,  0x46 }, // 0x02
    { 0x0508,  0x66 }, // 0x03
    { 0x0509,  0x86 }, // 0x04
    { 0x050A,  0xA6 }, // 0x05
    { 0x050B,  0xc6 }, // 0x06
    { 0x050C,  0x00 }, // 0x07
    { 0x050D,  0xFF }, // 0x08
    { 0x050E,  0x07 },
    { 0x050F,  0x0D },
    { 0x0510,  0xC0 }, // 0x0D
    { 0x0511,  0x07 }, // 0x0E
    { 0x0512,  0x83 }, // 0x0F
    { 0x0513,  0x0f }, // 0x10
    { 0x0514,  0x01 }, // 0x11
    { 0x0515,  0x07 }, // 0x12
    { 0x0516,  0x02 },
    { 0x0517,  0x3B },
    { 0x0518,  0x07 }, // 0x3B
    { 0x0401,  0x01 },
    { 0x0400,  0x01 },
    { 0x0401,  0x00 },


    // Laser Driver Temperature and Embedded Data
    { 0x0413,0xA0 }, // send2 channel start by internal sync signal
    { 0x0417,0x23 }, // send2 channel start pointer of TX buffer
    { 0x0423,0xA0 }, // thermal channel start by internal sync signal
    { 0x0425,0x01 }, // thermal channel transaction x 2
    { 0x0426,0x00 }, // thermal channel start pointer of RX buffer
    { 0x0427,0x26 }, // thermal channel start pointer of TX buffer
    { 0x042a,0x01 }, // thermal channel 1st byte is tx, after rx

    { 0x0523,0x02 },
    { 0x0524,0x13 },
    { 0x0525,0x21 }, // send2 channel start ADC

					 // read driver IC addr[0x00 - 0x0A] correspond to sensor rx buffer[0x0580 - 0x058A]
    { 0x0526,0x0C }, // thermal channel 1st transaction, 1 tx byte, 11 rx byte
    { 0x0527,0x80 }, // driver IC addr 0x00 (0x80 - 0x80)
					 // read driver IC addr[0x14 - 0x24] correspond to sensor rx buffer[0x058B - 0x059B]
    { 0x0528,0x13 }, // thermal channel 2nd transaction, 1 tx byte, 18 rx byte
    { 0x0529,0x94 }, // driver IC addr 0x14 (0x94 - 0x80)


    { 0x0411,0x01 },
    { 0x0421,0x01 },
};

static int cxa4026_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(cxa4026_reglist_acc) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(cxa4026_reglist_acc[i].reg, cxa4026_reglist_acc[i].val);
	}
	return rtn;
}

static int phx3d_3021_cb_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);
	}
	return rtn;
}

static int dw9912_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(dw9912_reglist) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(dw9912_reglist[i].reg, dw9912_reglist[i].val);
	}
	return rtn;
}

static int vcsel_driver_detect()
{
	int rtn = 0;
	uint8_t value = 0;
	rtn = vcsel_driver_write_reg(0x2D, 0x01); // CXA4026 reset
	rtn = vcsel_driver_write_reg(0x25, 0x01); // PHX3D_3021_CB reset
	rtn = vcsel_driver_read_reg(0x0F, &value);

	if (value == 0x06) {
		driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
		rtn = phx3d_3021_cb_initialize();
	}
	else if (value == 0x04) {
		driver_ic_type = DRIVER_IC_CXA4026;
		rtn = cxa4026_initialize();
	}
	else if (value == 0xA7) {
		driver_ic_type = DRIVER_IC_DW9912;
		rtn = dw9912_initialize();
	}
	else {
		driver_ic_type = 0;

	}
	//printf("driver_ic_type: %d. value: %d.\r\n", driver_ic_type, value);
	return rtn;
}

int imx518_sensor_init()
{
	int rtn = 0;
#if (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6130)
	for (int i = 0; i < sizeof(imx518_reglist_6130) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(imx518_reglist_6130[i].reg, imx518_reglist_6130[i].val);
	}
#elif (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6272)
	for (int i = 0; i < sizeof(imx518_reglist_6272) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(imx518_reglist_6272[i].reg, imx518_reglist_6272[i].val);
	}
#endif

	rtn = vcsel_driver_detect();

	return rtn;
}

int imx518_shadow_register(bool enable) // checked
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(0x0102, 0x01);
	else
		rtn = sensor_write_reg(0x0102, 0x00);
	return rtn;
}

int imx518_set_user_ID(uint8_t value)
{
	return sensor_write_reg(0x0824, value);
}

int imx518_set_hmax(uint16_t hmax)
{
	int rtn = 0;
	// set HMAX register
	rtn = sensor_write_reg(0x0800, (hmax >> 8) & 0xFF);
	rtn |= sensor_write_reg(0x0801, (hmax & 0xFF));
	/**/
	// set PLLSSETUP register
	uint8_t pllssetup = (uint8_t)ceil(503 * 120.0f / hmax + 8.0f);
	rtn |= sensor_write_reg(0x4010, pllssetup);

	// set PIXRST register
	uint16_t pixrst = (uint16_t)ceil(50 * 120.0f / hmax);
	rtn |= sensor_write_reg(0x4015, (pixrst >> 8) & 0xFF);
	rtn |= sensor_write_reg(0x4016, (pixrst & 0xFF));

	// set RANDNM0 register
	uint32_t randnm0 = (uint32_t)ceil(hmax*pixrst - 1070 - 2098);
	rtn |= sensor_write_reg(0x5265, (randnm0 >> 16) & 0xFF);
	rtn |= sensor_write_reg(0x5266, (randnm0 >> 8) & 0xFF);
	rtn |= sensor_write_reg(0x5267, (randnm0 & 0xFF));

	return rtn;
}

int imx518_set_trigger_mode(uint8_t mode) // should stop stream first
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

int imx518_set_stream_mode() // should stop stream first
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

int imx518_get_data_output_mode(uint8_t *mode) // checked
{/*
	uint32_t value = 0;
	int rtn = sensor_read_reg(0x082C, &value);
	//printf("imx518_get_data_output_mode %d \r\n", value);
	*mode = value & 0x0f;
	return rtn;*/

	*mode = 4;

	return 0;
}

// be careful that the image width of A&B mode will change to 1280 pixel
// should stop streaming before change data output mode
int imx518_set_data_output_mode(uint8_t mode)
{/*
	// mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
	if (mode > 4)
		return -1;
	int rtn = 0;
	uint8_t set_mode = (mode << 4) + mode;
	for (int i = 0; i < 8; i++) { // set only group A and group B here
		rtn = sensor_write_reg(0x082C + i, set_mode);
	}
	return rtn;*/
	return 0;
}
#if (IMX518_REG_LIST_SELECT == IMX518_REG_LIST_6272) 
#define FREQ_DUTY_LIST_NUM    19
#else
#define FREQ_DUTY_LIST_NUM    7
#endif
int imx518_get_frequency_and_duty(int *index, int *index_max, struct sensor_freq_duty_t *freq_duty)
{
    //printf("index = %d\n",*index);
	int rtn = 0;
#if (IMX518_REG_LIST_SELECT == IMX518_REG_LIST_6272) 
	const static struct sensor_freq_duty_t freq_duty_list[FREQ_DUTY_LIST_NUM] = {

		{ 80, 120, 33.3, 37.5, 4 },
		{ 20, 120, 33.3, 37.5, 4 },
		{ 20, 120, 30.3, 32.0, 4 },
		{ 15, 120, 37.5, 37.5, 4 },
		{ 10, 120, 33.3, 37.5, 4 },
		{ 60, 100, 35.0, 33.3, 4 },
		{ 20, 100, 35.0, 37.5, 4 },
		{ 15, 100, 35.0, 33.3, 4 },// index = 7
		{ 15, 100, 37.0, 35.0, 4 },
		{ 15, 100, 40.0, 41.0, 4 },
		{ 15, 100, 50.0, 50.0, 4 },// index = 10
		{ 10, 100, 35.0, 37.5, 4 },
		{ 20,  80, 37.5, 37.5, 4 },
		{ 15,  80, 37.5, 33.3, 4 },
		{ 10,  80, 33.3, 33.3, 4 },
		{ 20,  60, 33.3, 37.5, 4 },
		{ 15,  60, 33.3, 34.8, 4 },// index = 16
		{ 10,  60, 34.5, 34.8, 4 },
		{ 100,100, 70.2, 70.2, 4 }
	};

    *index_max = FREQ_DUTY_LIST_NUM;
    if (*index < 0) {
		*index = current_freq_duty_index;
#if 0
        uint8_t freq_1, freq_2;
        uint8_t imxck = 0;
        uint8_t predivsel = 0;
        uint8_t fmodsel = 0;
        rtn = sensor_read_reg(0x1049, &imxck);

        rtn = sensor_read_reg(0x214C, &predivsel);
        rtn = sensor_read_reg(0x217C, &fmodsel);
        freq_1 = imxck * 8 / (pow(2, predivsel)*fmodsel * 2);

        rtn = sensor_read_reg(0x214D, &predivsel);
        rtn = sensor_read_reg(0x217D, &fmodsel);
        freq_2 = imxck * 8 / (pow(2, predivsel)*fmodsel * 2);

        for (int i = 0; i < FREQ_DUTY_LIST_NUM; i++) {
            if (freq_1 == freq_duty_list[i].mod_freq1 && freq_2 == freq_duty_list[i].mod_freq2) {
                *index = i;
                break;
            }
			if (i == (FREQ_DUTY_LIST_NUM - 1)) { // not exist
				*index = -10;
				return  -2002;
			}
        }
#endif
    }
    else if (*index >= *index_max) {
        return  -2002;
    }

    //printf("index %d, freq_1 %d,freq_2=%d", *index,freq_1, freq_2);
    *freq_duty = freq_duty_list[*index];
#else
	const static struct sensor_freq_duty_t freq_duty_list[FREQ_DUTY_LIST_NUM] = {
		{ 120, 0, 37.5, 0, 4 },
		{ 100, 0, 37.5, 0 ,4 },
		{  80, 0, 37.5, 0 ,4 },
		{  60, 0, 37.5, 0 ,4 },
		{  20, 0, 33.3, 0 ,4 },
		{  15, 0, 37.5, 0 ,4 },
		{  10, 0, 33.3, 0 ,4 }
	};
	*index_max = FREQ_DUTY_LIST_NUM;
	if (*index < 0) {
		uint8_t freq_1;
		uint8_t imxck = 0;
		uint8_t predivsel = 0;
		uint8_t fmodsel = 0;
		rtn = sensor_read_reg(0x1049, &imxck);

		rtn = sensor_read_reg(0x214C, &predivsel);
		rtn = sensor_read_reg(0x217C, &fmodsel);
		freq_1 = imxck * 8 / (pow(2, predivsel)*fmodsel * 2);

		for (int i = 0; i < FREQ_DUTY_LIST_NUM; i++) {
			if (freq_1 == freq_duty_list[i].mod_freq1) {
				*index = i;
				break;
			}
			if (i == (FREQ_DUTY_LIST_NUM - 1)) { // not exist
				*index = -10;
				return  -2002;
			}
		}
	}
	else if (*index >= *index_max) {
		return  -2002;
	}
	//printf("index %d, freq_1 %d", *index,freq_1);
	*freq_duty = freq_duty_list[*index];
#endif
	return rtn;
}

int imx518_set_frequency_and_duty(int index)
{
#if (IMX518_REG_LIST_SELECT == IMX518_REG_LIST_6272) 
	if (index > FREQ_DUTY_LIST_NUM)
		return -2002;
	current_freq_duty_index = index;
	struct regList freq_duty[] = {

		// 80_120M
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x00 }, // GroupA predivsel
		{ 0x217C, 0x06 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x09 }, // phase increment = GroupA fmodsel / 2 = 3
		{ 0x218A, 0x0C },
		{ 0x218B, 0x0F },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x00 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x04 }, // GroupA CTRL_LSRADD, duty = 50%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x01 }, // GroupA duty adjustment, 260ps/step, duty = 50% - (1.5 + 0.26)/12.5ns) = 36%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/8.3ns) = 32%


		// 20M_120M
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x06 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x09 }, // phase increment = GroupA fmodsel / 2 = 3
		{ 0x218A, 0x0C },
		{ 0x218B, 0x0F },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x05 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 5 / (6 * 2) = 41.6%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x03 }, // GroupA duty adjustment, 260ps/step, duty = 41.6% - (1.5 + 0.26*3)/50ns = 36.5%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/8.3ns) = 32%


		// 20M_120M
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x01 }, // GroupA predivsel
		{ 0x217C, 0x0C }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0C }, // phase increment = GroupA fmodsel / 2 = 6
		{ 0x218A, 0x12 },
		{ 0x218B, 0x18 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (12 * 2) = 33.3%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 33.3% - 1.5/50ns = 30.3%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/8.3ns) = 32%


		// 15M_120M
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x08 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0A }, // phase increment = GroupA fmodsel / 2 = 4
		{ 0x218A, 0x0E },
		{ 0x218B, 0x12 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x06 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 6 / (8 * 2) = 37.5%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 37.5% - (1.5ns/66.6ns) = 35.3%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/8.3ns) = 32%


		// 10M_120M
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0C }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0C }, // phase increment = GroupA fmodsel / 2 = 6
		{ 0x218A, 0x12 },
		{ 0x218B, 0x18 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (12 * 2) = 33.3%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 33.3% - (1.5ns/100ns) = 32%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/8.3ns) = 32%


		// 60M_100M
		{ 0x1049, 0x96 }, // IMXCK = 150
		{ 0x214C, 0x00 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x09 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 9 / (10 * 2) = 45%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 45% - (1.5ns/16.6ns) = 36%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/10ns) = 35%


		// 20M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x64 }, // IMXCK = 100
		{ 0x214C, 0x01 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (10 * 2) = 40%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x02 }, // GroupA duty adjustment, 260ps/step, duty = 40% - (1.5 + 0.26*2)/50ns = 36%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/10ns) = 35%


		// 15M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x96 }, // IMXCK = 150
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x07 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 7 / (10 * 2) = 35%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (6 * 2) = 33.3%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 15M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x96 }, // IMXCK = 150
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL, duty = 50%
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (10 * 2) = 40%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (6 * 2) = disabled here

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 40% - 1500ps/66.6ns = 37%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - 1500ps/10ns = 35%


		// 15M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x96 }, // IMXCK = 150
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (10 * 2) = 40%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x05 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 5 / (6 * 2) = 41%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 15M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x96 }, // IMXCK = 150
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x00 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = 50%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x05 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 10M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x64 }, // IMXCK = 100
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0A }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0B }, // phase increment = GroupA fmodsel / 2 = 5
		{ 0x218A, 0x10 },
		{ 0x218B, 0x15 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x07 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 7 / (10 * 2) = 35%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step, duty = 35% - (1.5ns/100ns) = 33.5%
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns/10ns) = 35%


		// 20M_80M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x50 }, // IMXCK = 80
		{ 0x214C, 0x01 }, // GroupA predivsel
		{ 0x217C, 0x08 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0A }, // phase increment = GroupA fmodsel / 2 = 4
		{ 0x218A, 0x0E },
		{ 0x218B, 0x12 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x06 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 6 / (8 * 2) = 37.5%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x03 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 3 / (4 * 2) = 37.5%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 15M_80M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x08 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0A }, // phase increment = GroupA fmodsel / 2 = 4
		{ 0x218A, 0x0E },
		{ 0x218B, 0x12 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x06 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 6 / (8 * 2) = 37.5%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (6 * 2) = 33.3%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 10M_80M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0C }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x06 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0C }, // phase increment = GroupA fmodsel / 2 = 6
		{ 0x218A, 0x12 },
		{ 0x218B, 0x18 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x09 }, // phase increment = GroupB fmodsel / 2 = 3
		{ 0x2192, 0x0C },
		{ 0x2193, 0x0F },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (12 * 2) = 33.3%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (6 * 2) = 33.3%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 20M_60M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x01 }, // GroupA predivsel
		{ 0x217C, 0x0C }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x08 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0C }, // phase increment = GroupA fmodsel / 2 = 6
		{ 0x218A, 0x12 },
		{ 0x218B, 0x18 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x0A }, // phase increment = GroupB fmodsel / 2 = 4
		{ 0x2192, 0x0E },
		{ 0x2193, 0x12 },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x08 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 8 / (12 * 2) = 33.3%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x06 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 6 / (8 * 2) = 37.5%

		{ 0x2247, 0x00 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x00 }, // GroupA duty adjustment, 260ps/step
		{ 0x2255, 0x00 }, // GroupB duty adjustment, 260ps/step


		// 15M_60M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x08 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x08 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0A }, // phase increment = GroupA fmodsel / 2 = 4
		{ 0x218A, 0x0E },
		{ 0x218B, 0x12 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x0A }, // phase increment = GroupB fmodsel / 2 = 4
		{ 0x2192, 0x0E },
		{ 0x2193, 0x12 },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x06 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 6 / (8 * 2) = 37.5%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x06 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x05 }, // GroupA duty adjustment, 260ps/step, duty = 37.5% - (1.5ns+5*0.26)/66.6ns = 33.3%
		{ 0x2255, 0x04 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns+4*0.26)/16.6ns = 34.8%


		// 10M_60M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x78 }, // IMXCK = 120
		{ 0x214C, 0x02 }, // GroupA predivsel
		{ 0x217C, 0x0C }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x08 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x0C }, // phase increment = GroupA fmodsel / 2 = 6
		{ 0x218A, 0x12 },
		{ 0x218B, 0x18 },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x0A }, // phase increment = GroupB fmodsel / 2 = 4
		{ 0x2192, 0x0E },
		{ 0x2193, 0x12 },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x09 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 9 / (12 * 2) = 37.5%
		{ 0x2245, 0x00 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x06 }, // GroupB CTRL_LSRADD, duty = 50%

		{ 0x2247, 0x0C }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x06 }, // GroupA duty adjustment, 260ps/step, duty = 37.5% - (1.5ns+6*0.26)/100ns = 34.5%
		{ 0x2255, 0x04 }, // GroupB duty adjustment, 260ps/step, duty = 50% - (1.5ns+4*0.26)/16.6ns = 34.8%


		// 100M_100M
		// freq = IMXCK * 8 / ((2^predivsel) * fmodsel * 2)
		{ 0x1049, 0x64 }, // IMXCK = 100
		{ 0x214C, 0x00 }, // GroupA predivsel
		{ 0x217C, 0x04 }, // GroupA fmodsel
		{ 0x214D, 0x00 }, // GroupB predivsel
		{ 0x217D, 0x04 }, // GroupB fmodsel

		{ 0x2188, 0x06 }, // GroupA CTRLPH_LSR
		{ 0x2189, 0x08 }, // phase increment = GroupA fmodsel / 2 = 2
		{ 0x218A, 0x0A },
		{ 0x218B, 0x0C },
		{ 0x2190, 0x06 }, // GroupB CTRLPH_LSR
		{ 0x2191, 0x08 }, // phase increment = GroupB fmodsel / 2 = 2
		{ 0x2192, 0x0A },
		{ 0x2193, 0x0C },

		{ 0x2244, 0x01 }, // GropuA LSRDTYSEL
		{ 0x21B8, 0x04 }, // GroupA CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (4 * 2) = 50%
		{ 0x2245, 0x01 }, // GropuB LSRDTYSEL
		{ 0x21B9, 0x04 }, // GroupB CTRL_LSRADD, duty = CTRL_LSRADD / (fmodsel * 2) = 4 / (4 * 2) = 50%

		{ 0x2247, 0x14 }, // duty adjustment 0x00->OFF, 0x0C->compress, 0x14->expand
		{ 0x2254, 0x02 }, // GroupA duty adjustment, 260ps/step, duty = 50% + (1.5ns+2*0.26)/10ns = 70.2%
		{ 0x2255, 0x02 }, // GroupB duty adjustment, 260ps/step, duty = 50% + (1.5ns+2*0.26)/10ns = 70.2%


	};

	int rtn = 0;
	rtn = imx518_shadow_register(true);
	int table_size = sizeof(freq_duty) / (FREQ_DUTY_LIST_NUM * sizeof(struct regList));
	for (int i = 0; i < table_size; i++) {

		rtn = sensor_write_reg(freq_duty[i + table_size * index].reg, freq_duty[i + table_size * index].val);
	}
	// overwrite duty to 50%
	//rtn = sensor_write_reg(0x2244, 0x00);
	//rtn = sensor_write_reg(0x2245, 0x00);
	rtn = imx518_shadow_register(false);
#else
	int rtn = 0;
	if (index >= FREQ_DUTY_LIST_NUM)
		return -2002;
	current_freq_duty_index = index;
	static uint16_t modFreq[] = { 120, 100, 80, 60, 20, 15, 10 };
	uint16_t mod_freq = modFreq[index];
	//printf("set mod freq to %d MHz\r\n", mod_freq);

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
#endif

	return rtn;
}

int imx518_get_modulation_frequency(uint16_t *modFreq)
{
	int rtn = 0;
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

int imx518_set_modulation_frequency(uint16_t modFreq)
{
	int rtn = 0;
	
	return rtn;
}

int imx518_get_fps(uint8_t *fps)
{
	uint8_t byte0, byte1, byte2, byte3;
	int rtn = sensor_read_reg(0x210C, &byte3);
	rtn = sensor_read_reg(0x210D, &byte2);
	rtn = sensor_read_reg(0x210E, &byte1);
	rtn = sensor_read_reg(0x210F, &byte0);
#if (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6130)
	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0 + 2;
	uint32_t frameTime = ((value * BPMAX_DIV_120) + BPMAX_DIV_120) + BPMAX_DIV_120;
	*fps = (uint8_t)(1000000 / frameTime);
	printf("byte 3 2 1 0:  %x %x %x %x", byte3, byte2, byte1, byte0);
	printf("frameTime %d, fps %d", frameTime, *fps);
#elif (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6272)
	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0 + 2;
	uint32_t frameTime = ((value * BPMAX_DIV_120) + BPMAX_DIV_120) * 2 + BPMAX_DIV_120;
	*fps = (uint8_t)(1000000 / frameTime);
#endif
	return rtn;
}

int imx518_set_fps(uint8_t fps)
{
	int rtn = 0;
#if (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6130)
	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t frameTime = 1000000 / fps; // us
	uint32_t value = ((frameTime - BPMAX_DIV_120) - BPMAX_DIV_120) / BPMAX_DIV_120;
	rtn = sensor_write_reg(0x210C, (value >> 24) & 0xFF);
	rtn = sensor_write_reg(0x210D, (value >> 16) & 0xFF);
	rtn = sensor_write_reg(0x210E, (value >> 8) & 0xFF);
	rtn = sensor_write_reg(0x210F, (value & 0xFF));
#elif (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6272)
	static int BPMAX_DIV_120 = 600 / 120;
	uint32_t frameTime = 1000000 / fps; // us
	uint32_t value = (((frameTime - BPMAX_DIV_120) / 2) - BPMAX_DIV_120) / BPMAX_DIV_120;
	rtn = sensor_write_reg(0x210C, (value >> 24) & 0xFF);
	rtn = sensor_write_reg(0x210D, (value >> 16) & 0xFF);
	rtn = sensor_write_reg(0x210E, (value >> 8) & 0xFF);
	rtn = sensor_write_reg(0x210F, (value & 0xFF));
#endif

	return rtn;
}

int imx518_get_integration_time(uint16_t *integrationTime) // checked
{
	uint8_t byte0, byte1, byte2, byte3;
	int rtn = sensor_read_reg(0x2124, &byte3);
	rtn = sensor_read_reg(0x2125, &byte2);
	rtn = sensor_read_reg(0x2126, &byte1);
	rtn = sensor_read_reg(0x2127, &byte0);

	uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;

	*integrationTime = (uint16_t)(value / CLK120MHz); // may slightly different from what we set because of floor function

	//printf("imx516_get_integration_time %d   ret=%d\n",*integrationTime,rtn);

	return rtn;
}

int imx518_set_integration_time(uint16_t integrationTime) // checked
{
	if (integrationTime > 1000)
		return -1;

	unsigned int value = integrationTime*CLK120MHz;

	int rtn = imx518_shadow_register(true);
	for (int group = 0; group < 3; group++) {
		// set integration time
		rtn = sensor_write_reg(0x2124 + group * 4, (value >> 24) & 0xFF);
		rtn = sensor_write_reg(0x2125 + group * 4, (value >> 16) & 0xFF);
		rtn = sensor_write_reg(0x2126 + group * 4, (value >> 8) & 0xFF);
		rtn = sensor_write_reg(0x2127 + group * 4, (value & 0xFF));
	}

	rtn |= imx518_shadow_register(false);
	//printf("imx516_set_integration_time %d,  ret=%d\n", integrationTime,rtn);
	return rtn;
}

int imx518_set_phase_led_enable_pulse(uint8_t phaseLedEn)
{
	return sensor_write_reg(0x21C4, phaseLedEn); // default is 0x00, disable 8 phase .
}

/*
mode = 0: no binning (= VGA resolution, 640x480 pixels)  // default mode
mode = 1: 2x2 binning (= QVGA resolution, 320x240 pixels)
mode = 2: 4x4 binning (= QQVGA resolution, 160x120 pixels)
mode = 3: 8x8 binning (= QQQVGA resolution, 80x60 pixels)
*/
int imx518_pixel_binning(uint8_t mode)
{
	return sensor_write_reg(0x14A5, mode);
}

int imx518_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t value[4];
	value[0] = x1;
	value[1] = x2 - x1 + 1;
	value[2] = (y1 - 1) / 2;
	value[3] = y2 / 2 + 1;

	int rtn = 0;
	for (int i = 0; i < 4; i++) {

		rtn |= sensor_write_reg(0x0804 + i * 2, (value[i] >> 8) & 0xFF);
		rtn |= sensor_write_reg(0x0805 + i * 2, (value[i] & 0xFF));
	}

	return rtn;
}

int imx518_get_img_mirror_flip(uint8_t *mode)
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

int imx518_set_img_mirror_flip(uint8_t mode)
{
	int rtn = 0;
	if (mode == 0) {
		rtn |= sensor_write_reg(0x0811, 0x00); // default   H
		rtn |= sensor_write_reg(0x0810, 0x00); // default   V
	}
	else if (mode == 1) {
		rtn |= sensor_write_reg(0x0811, 0x01);
		rtn |= sensor_write_reg(0x0810, 0x00);
	}
	else if (mode == 2) {
		rtn |= sensor_write_reg(0x0811, 0x00);
		rtn |= sensor_write_reg(0x0810, 0x01);
	}
	else if (mode == 3) {
		rtn |= sensor_write_reg(0x0811, 0x01);
		rtn |= sensor_write_reg(0x0810, 0x01);
	}
	return rtn;
}

int imx518_test_pattern(uint8_t mode)
{
	int rtn = 0;
	if (mode) {
		rtn |= sensor_write_reg(0x1405, 0x00);
		rtn |= sensor_write_reg(0x1406, 0x04);
		rtn |= sensor_write_reg(0x1407, 0x01);
	}
	else {
		rtn |= sensor_write_reg(0x1407, 0x00); // default
	}

	return rtn;
}

int imx518_illum_duty_cycle_adjust(uint8_t mode, uint8_t value)
{
	int rtn = 0;
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

	return rtn;
}

// different modulation frequency have different adjustable duty cycle range
int imx518_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
	float duty = 0;
	float cycle = 1000.0f / mod_freq; // period ns
	for (int step = -15; step <= 15; step++) {
		duty = (0.5f + (step*0.5f - 2.0f) / cycle) * 100; // 0.5ns/step, 2.0ns is a const missing caused by IC-HG vcsel driver
		if (duty < 0) {
			duty = 0;
		}
		else if ((duty > 100)) {
			duty = 100;
		}
		duty_cycle_list[step + 15] = duty;
	}

	return 0;
}

int imx518_set_frequency_mode(uint8_t mode)
{
	int rtn;
	if(mode == SINGLE_FREQ){
		sensor_write_reg(0x2100, 0x08);
		imx518_set_fps(15);
	}else if (mode == DUAL_FREQ) {
		sensor_write_reg(0x2100, 0x48);
		imx518_set_fps(30);
	}

	return 0;
}

int imx518_get_frequency_mode(uint8_t *mode)
{
	uint8_t value;
	int rtn = sensor_read_reg(0x2100, &value);
	//ALOGE("imx518_get_frequency_mode: %d", value);
	if (value == 0x08)
		*mode = SINGLE_FREQ; // single freq
	else if (value == 0x48)
		*mode = DUAL_FREQ; // dual freq
	else
		return -1;

	return 0;
}

int imx518_video_streaming(bool enable)
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(0x1001, 0x01);
	else
		rtn = sensor_write_reg(0x1001, 0x00);

	return rtn;
}

int imx518_init()
{
	int rtn = 0;

	rtn = imx518_sensor_init();

#if (IMX518_PROJECT_SELECT == IMX518_REG_LIST_6130)
	rtn = imx518_illum_power_test(1);
#endif
	rtn = set_expander_tx_gpio(0, 1);

	return rtn;
}

int imx518_func_init()
{
#if !DEBUG_IMX518_IN_QT
	tof_sensor.init = imx518_init;
	tof_sensor.get_sensor_id = imx518_get_sensor_id;
	tof_sensor.hardware_trigger = imx518_hardware_trigger;
	tof_sensor.software_trigger = imx518_software_trigger;
	tof_sensor.video_streaming = imx518_video_streaming;
	tof_sensor.get_fps = imx518_get_fps;
	tof_sensor.set_fps = imx518_set_fps;
	tof_sensor.get_rx_temp = imx518_get_rx_temp;
	tof_sensor.get_tx_temp = imx518_get_tx_temp;
	tof_sensor.set_illum_power = imx518_set_illum_power;
	tof_sensor.get_illum_power = imx518_get_illum_power;
	tof_sensor.illum_power_control = imx518_illum_power_control;
	tof_sensor.illum_power_test = imx518_illum_power_test;
	tof_sensor.get_integration_time = imx518_get_integration_time;
	tof_sensor.set_integration_time = imx518_set_integration_time;
	tof_sensor.get_modulation_frequency = imx518_get_modulation_frequency;
	tof_sensor.set_modulation_frequency = imx518_set_modulation_frequency;
	tof_sensor.get_data_output_mode = imx518_get_data_output_mode;
	tof_sensor.set_data_output_mode = imx518_set_data_output_mode;
	tof_sensor.get_img_mirror_flip = imx518_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = imx518_set_img_mirror_flip;
	tof_sensor.test_pattern = imx518_test_pattern;
	tof_sensor.get_sensor_info = imx518_get_sensor_info;
	tof_sensor.set_tx_a_b_power = set_expander_tx_gpio;
	tof_sensor.get_tx_a_b_power = get_expander_tx_gpio;
	tof_sensor.get_frequency_and_duty = imx518_get_frequency_and_duty;
	tof_sensor.set_frequency_and_duty = imx518_set_frequency_and_duty;
	tof_sensor.get_frequency_mode = imx518_get_frequency_mode;
	tof_sensor.set_frequency_mode = imx518_set_frequency_mode;
	tof_sensor.get_vcsel_pd = imx518_get_vcsel_pd;
	tof_sensor.get_vcsel_error = imx518_get_vcsel_error;
	tof_sensor.sensor_write_reg_8 = sensor_write_reg;
	tof_sensor.sensor_read_reg_8 = sensor_read_reg;
	tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;
	tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;
#endif
	return 0;
}
