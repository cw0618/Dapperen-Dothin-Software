#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <hw_obstatus.h>
#include "dapperen100hy.h"

static uint8_t vcsel_number = 2;
static uint16_t vcsel_driver_type = 3644;

struct regList {
	uint16_t reg;
	uint8_t val;
};

#if !DEBUG_DAPPEREN100HY_IN_QT
#include <tof_sensors.h>
#include <obc_tee_funcs.h>
//#include "hw_modules.h"

#ifdef __linux__
#include <unistd.h>
#endif

#define DDEBUG(fmt, ...)   \
    printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)


//#define ALOGE(...)
#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread

#define dothin_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define dothin_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_sensor_enable       tops_t.ap_ops.SensorEnable
#define dothin_device_id           tops_t.ap_ops.device_id

#define I2C_M_RD     1
#define I2C_M_WT     0

#define DAPPEREN100HY_PCLK           24000000 // pclk 24MHz
#define DAPPEREN100HY_PCLK_MHZ       24 // pclk 24MHz

uint8_t dapperen100hy_tca9548_val = 0;

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
	return 0; // should wrap method by SDK
}

#else
//#include <QDebug.h>
void* iic_handle = NULL;
post_iic_write iic_write_callback = NULL;
post_iic_read iic_read_callback = NULL;
post_gpio gpio_callback = NULL;
void set_iic_write_callback(post_iic_write cb, void* handle)
{
	iic_write_callback = cb;
	iic_handle = handle;
}
void set_iic_read_callback(post_iic_read cb, void* handle)
{
	iic_read_callback = cb;
	iic_handle = handle;
}
void set_gpio_callback(post_gpio cb, void* handle)
{
	gpio_callback = cb;
	iic_handle = handle;
}

static int i2c_reg_read(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size)
{
	if (iic_handle == NULL){
		//qDebug() << "null iic handle";
		return -1;
	}
	return iic_read_callback(addr, reg, reg_size, data, data_size, iic_handle);

}

static int i2c_reg_write(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size)
{
	if (iic_handle == NULL){
		//qDebug() << "null iic handle";
		return -1;
	}
	return iic_write_callback(addr, reg, reg_size, data, data_size, iic_handle);

}

static int gpio_control(int pin, bool level)
{
	if (iic_handle == NULL){
		//qDebug() << "null iic handle";
		return -1;
	}
	return gpio_callback(pin, level, iic_handle);
}
#endif

int dapperen100hy_dothin_config()
{
#if !DEBUG_DAPPEREN100HY_IN_QT
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
	ret = dothin_set_sensor_clock(true, 8 * 10, dothin_device_id); // 8Mhz mclk
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

	return ret;
#endif
}

int dapperen100hy_tca9548_write_reg(uint8_t reg, uint8_t value)
{
	int rtn = 0;
	uint8_t v1 = 0;

	if (dapperen100hy_tca9548_val != value)
	{
		rtn |= i2c_reg_write(TCA9548_ADDR, reg, 0, value, 1);
		rtn |= i2c_reg_read(TCA9548_ADDR, reg, 0, &v1, 1);

		if (rtn || (value - v1))
			DDEBUG("tca9548 write reg error, ret=%d\n", rtn);
		else
			dapperen100hy_tca9548_val = value;
	}

	return rtn;
}

int dapperen100hy_tca9548_read_reg(uint8_t reg, uint8_t *value)
{
	int rtn = 0;

	rtn = i2c_reg_read(TCA9548_ADDR, reg, 0, value, 1);

	if (!rtn)
		dapperen100hy_tca9548_val = *value;
	else
		DDEBUG("tca9548 read reg error, ret=%d\n", rtn);

	return rtn;
}

int dapperen100hy_write_reg(uint16_t reg, uint8_t value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x01);
	return i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
}

int dapperen100hy_read_reg(uint16_t reg, uint8_t *value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x01);
	return i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
}

int dapperen100hy_lm3644_write_reg(uint8_t reg, uint8_t value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x06);
	//return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_lm3644_read_reg(uint8_t reg, uint8_t *value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x06);
	//return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_lm3644_ldm_write_reg(uint8_t reg, uint8_t value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x04);
	//return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_lm3644_ldm_read_reg(uint8_t reg, uint8_t *value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x04);
	//return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_lm3644_led_write_reg(uint8_t reg, uint8_t value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x02);
	//return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_lm3644_led_read_reg(uint8_t reg, uint8_t *value)
{
	//dapperen100hy_tca9548_write_reg(0, 0x02);
	//return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int dapperen100hy_set_debug(uint8_t type, uint8_t value)
{
	int rtn = 0;
	uint8_t val = 0, en1 = 0, en2 = 0;

	switch (type)
	{
	case DEBUG_TYPE_SBG_SELFTEST:
		// check sl4 mode
		rtn |= dapperen100hy_get_data_output_mode(&val);
		if (val == NORMAL_FREE_RUN)
		{
			rtn |= dapperen100hy_lm3644_ldm_read_reg(0x01, &en1);
			rtn |= dapperen100hy_lm3644_led_read_reg(0x01, &en2);

			if (value == 1)
			{
				// set sl4 self test mode, background ldm
				// ldm torch mode
				rtn |= dapperen100hy_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x08);
				rtn |= dapperen100hy_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x24);

				// gpio map, set seq config, c0: strobe1->pad9, strobe1->pad8, c1: strobe0->pad9, strobe0->pad8,
				rtn |= dapperen100hy_write_reg(0x0c01, 0x9f);
				rtn |= dapperen100hy_write_reg(0x0c1e, 0x1c);
				rtn |= dapperen100hy_write_reg(0x0c20, 0x1c);

				DDEBUG("dapperen100hy debug set sbg test mode (backgroud ldm) \r\n");
			}
			else if (value == 2)
			{
				// set sl4 self test mode, background led
				// led torch mode
				rtn |= dapperen100hy_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x24);
				rtn |= dapperen100hy_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x08);

				// gpio map, set seq config, c0: strobe1->pad9, strobe1->pad8, c1: strobe0->pad9, strobe0->pad8,
				rtn |= dapperen100hy_write_reg(0x0c01, 0x9f);
				rtn |= dapperen100hy_write_reg(0x0c1e, 0x1c);
				rtn |= dapperen100hy_write_reg(0x0c20, 0x1c);

				DDEBUG("dapperen100hy debug set sbg test mode (backgroud led) \r\n");
			}
			else
			{
				// clear sl4 self test mode
				// ldm/led ir mode
				rtn |= dapperen100hy_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x24);
				rtn |= dapperen100hy_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x24);
				//rtn |= dapperen100hy_lm3644_write_reg(0x01, 0x25);

				// gpio map, set user config
				rtn |= dapperen100hy_write_reg(0x0c01, 0x1f);
				rtn |= dapperen100hy_write_reg(0x0c04, 0x1c); // strobe0->pad9, strobe1->pad8

				DDEBUG("dapperen100hy debug clear sbg test mode \r\n");
			}
		}

		break;

	case DEBUG_TYPE_LOAD_DEFCFG:
		dapperen100hy_sensor_load_defcfg();
		break;

	default:
		break;
	}

	return rtn;
}

int dapperen100hy_demo_write_reg(uint16_t reg, uint8_t value)
{
	int rtn = 0;
	uint8_t reg_h = reg >> 8;
	uint8_t reg_l = reg & 0xff;

	/*if (reg_h == LM3644_ADDR)
	{
	rtn = dapperen100hy_lm3644_write_reg(reg_l, value);
	DDEBUG("lm3644 write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
	}
	else if (reg_h == LM3644_ADDR + 0x02)
	{
	rtn = dapperen100hy_lm3644_led_write_reg(reg_l, value);
	DDEBUG("lm3644 led write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
	}
	else if (reg_h == LM3644_ADDR + 0x04)
	{
	rtn = dapperen100hy_lm3644_ldm_write_reg(reg_l, value);
	DDEBUG("lm3644 ldm write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
	}
	else if (reg_h == TCA9548_ADDR)
	{
	rtn = dapperen100hy_tca9548_write_reg(reg_l, value);
	DDEBUG("tca9548 write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
	}
	else if (reg_h == DEBUG_FLAG)
	{
	rtn = dapperen100hy_set_debug(reg, value);
	DDEBUG("dapperen100hy set debug, type 0x%x, val 0x%x \r\n", reg_l, value);
	}
	else
	{
	rtn = dapperen100hy_write_reg(reg, value);
	DDEBUG("dapperen100hy write reg, reg 0x%x, val 0x%x \r\n", reg, value);
	}*/
	rtn = dapperen100hy_write_reg(reg, value);
	DDEBUG("dapperen100hy write reg, reg 0x%x, val 0x%x \r\n", reg, value);
	return rtn;
}

int dapperen100hy_demo_read_reg(uint16_t reg, uint8_t *value)
{
	int rtn = 0;
	uint8_t reg_h = reg >> 8;
	uint8_t reg_l = reg & 0xff;

	//if (reg_h == LM3644_ADDR)
	//{
	//	rtn = dapperen100hy_lm3644_read_reg(reg, value);
	//	DDEBUG("lm3644 read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
	//}
	//else if (reg_h == LM3644_ADDR + 0x02)
	//{
	//	rtn = dapperen100hy_lm3644_led_read_reg(reg, value);
	//	DDEBUG("lm3644 led read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
	//}
	//else if (reg_h == LM3644_ADDR + 0x04)
	//{
	//	rtn = dapperen100hy_lm3644_ldm_read_reg(reg, value);
	//	DDEBUG("lm3644 ldm read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
	//}
	//else if (reg_h == TCA9548_ADDR)
	//{
	//	rtn = dapperen100hy_tca9548_read_reg(reg, value);
	//	DDEBUG("tca9548 read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
	//}
	//else
	//{
	//	rtn = dapperen100hy_read_reg(reg, value);
	//	DDEBUG("dapperen100hy read reg, reg 0x%x, val 0x%x \r\n", reg, *value);
	//}
	rtn = dapperen100hy_read_reg(reg, value);
	DDEBUG("dapperen100hy read reg, reg 0x%x, val 0x%x \r\n", reg, *value);
	return rtn;
}

int dapperen100hy_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;

	// vcsel_num bit 0: ldm enable
	if (vcsel_num & 0x1)
		dapperen100hy_lm3644_ldm_write_reg(0x01, 0x25);
	else
		dapperen100hy_lm3644_ldm_write_reg(0x01, 0x24);

	// vcsel_num bit 1: led enable
	if (vcsel_num & 0x2)
		dapperen100hy_lm3644_led_write_reg(0x01, 0x25);
	else
		dapperen100hy_lm3644_led_write_reg(0x01, 0x24);


	// value A: LDM current
	if (value_A)
	{
		dapperen100hy_lm3644_ldm_write_reg(0x03, value_A & 0x7f);
	}

	// value B: LED current
	if (value_B)
	{
		dapperen100hy_lm3644_led_write_reg(0x03, value_B & 0x7f);
	}

	return rtn;
}

int dapperen100hy_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
	uint8_t val = 0;

	*vcsel_num = 0;

	rtn |= dapperen100hy_lm3644_ldm_read_reg(0x01, &val);
	if (val & 0x01)
		*vcsel_num |= 0x01;

	rtn |= dapperen100hy_lm3644_led_read_reg(0x01, &val);
	if (val & 0x01)
		*vcsel_num |= 0x02;

	rtn |= dapperen100hy_lm3644_ldm_read_reg(0x03, value_A);

	rtn |= dapperen100hy_lm3644_led_read_reg(0x03, value_B);

	return rtn;
}

struct regList lm3644_reglist[] = {
	{ 0x07, 0x89 }, // reset
	{ 0x07, 0x09 }, // clear reset, set pass mode
	{ 0x08, 0x10 }, // set torch ramp time(1ms) and flash timeout(40ms)
	{ 0x01, 0x25 }, // enable strobe and LED1/LED2, set IR mode
	{ 0x03, 0x64 }, // led1 flash current
	{ 0x04, 0x00 }, // led2 flash current
	{ 0x05, 0x7f }, // led1 torch current
	{ 0x06, 0x00 }, // led2 torch current
};

struct regList dapperen100hy_reglist[] = {
	{ 0x1022, 0x00 }, // PLL1 PLL2 always on
	//{ 0x0C06, 0xFF }, // 数字内部各种时钟不gateing
	//{ 0x0c12, 0x20 }, // sensor stop
	//{ 0x0c08, 0x01 }, // CHIP软复位
	//{ 0x0c07, 0x01 }, // MIPI软复位
	//{ 0x0c07, 0x00 },
	//{ 0x0c08, 0x00 },
};

struct regList dapperen100hy_defcfg_reglist[] = {
	{ 0x1019, 0x00 }, // PLL1 PLL2 always on
	//{ 0x0c06, 0xff }, //数字内部各种时钟不gateing
	//{ 0x0c21, 0x01 }, // DPHY BG_RT的来源选择 1为从寄存器读取数据   0为从OTP读取数据  
	//{ 0x143e, 0x3f }, // 模拟信号配置来源 3F来自寄存器 0来自OTP
	//{ 0x143d, 0xff }, // 模拟信号配置来源 FF来自寄存器 0来自OTP
	//{ 0x0c23, 0x7e },
	//{ 0x0c22, 0x00 }, // DB offset = -100 -> 7F9C
	//{ 0x0c11, 0x02 }, // 2为无限帧， 0 为有限帧
	//{ 0x306b, 0x00 },
	//{ 0x306a, 0x05 }, // 有限帧率下的帧数，无限帧下配置无效

	//{ 0x0c07, 0x01 },
	//{ 0x0c07, 0x01 },
	//{ 0x0c07, 0x00 }, // MIPI软复位
	//{ 0x0c07, 0x00 }, // MIPI软复位
	//{ 0x1f19, 0xfb }, // 打开mipi，不包括ZQ
	//{ 0x1f19, 0xfb }, // 打开mipi，不包括ZQ

	//{ 0x204f, 0x80 }, // RAW 配置  0 为RAW12， 1 为RAW10. [7] enable embedded data
	//{ 0x2052, 0x2c }, // EMB Data 类型 raw10 0x2a, raw12 0x2c
	//{ 0x0c00, 0x01 }, // 驱动光源, pad7/8/9 output enable
	//{ 0x0c04, 0x1b }, // 驱动光源, strobe0->pad9, strobe0->pad8
	////{ 0x0c04, 0x23 }, // 驱动光源, strobe1->pad9, strobe0->pad8
	////{ 0x0c04, 0x1c }, // 驱动光源, strobe0->pad9, strobe1->pad8
	//{ 0x0009, 0x7d }, // 4lane 的配置 // 配置PLL2 输出时钟为750MHZ。

	//{ 0x000a, 0x03 }, // 配置PLL2 输出时钟为750MHZ。
	//{ 0x000a, 0x03 }, // 配置PLL2 输出时钟为750MHZ。
	//{ 0x1f01, 0x0c },
	//{ 0x1f01, 0x0c },

	//{ 0x2000, 0x80 }, //开窗与时序设置 //ISP使能打开
	//{ 0x2015, 0x80 }, //ROI开窗功能打开
	//{ 0x2016, 0x08 },
	//{ 0x2017, 0x00 }, //纵向起始列=8
	//{ 0x2018, 0xb0 },
	//{ 0x2019, 0x04 }, //纵向列数=1200
	//{ 0x3031, 0x80 },
	//{ 0x3032, 0x07 }, //读出行数=1920
	//{ 0x3033, 0x80 },
	//{ 0x3034, 0x07 }, //读出行数=1920
	//{ 0x3035, 0x80 },
	//{ 0x3036, 0x07 }, //读出行数=1920
	//{ 0x3049, 0x0b },
	//{ 0x304a, 0x00 }, //读出起始行=11
	//{ 0x304b, 0x0b },
	//{ 0x304c, 0x00 }, //读出起始行=11
	//{ 0x304d, 0x0b },
	//{ 0x304e, 0x00 }, //读出起始行=11

	//{ 0x300a, 0x64 }, // tline c0
	//{ 0x300b, 0x00 },
	//{ 0x300c, 0x64 }, // tline c1
	//{ 0x300d, 0x00 },
	//{ 0x300e, 0x64 }, // tline c2
	//{ 0x300f, 0x00 },

	//{ 0x3010, 0xf4 },
	//{ 0x3011, 0x01 }, //t_slot_c0=500*16ns, 8us
	//{ 0x3012, 0xf4 },
	//{ 0x3013, 0x01 }, //t_slot_c1=500*16ns, 8us
	//{ 0x3014, 0xf4 },
	//{ 0x3015, 0x01 }, //t_slot_c2=500*16ns, 8us
	//{ 0x3016, 0x00 },
	//{ 0x3017, 0x02 }, //exp_c0=512*t_slot, 4ms
	//{ 0x3018, 0x00 },
	//{ 0x3019, 0x02 }, //exp_c1=512*t_slot, 4ms
	//{ 0x301a, 0x00 },
	//{ 0x301b, 0x02 }, //exp_c2=512*t_slot, 4ms
	//{ 0x3022, 0x80 },
	//{ 0x3023, 0x00 }, //exp_rs_c0=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)
	//{ 0x3024, 0x80 },
	//{ 0x3025, 0x00 }, //exp_rs_c1=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)
	//{ 0x3026, 0x80 },
	//{ 0x3027, 0x00 }, //exp_rs_c2=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)

	//{ 0x201a, 0x00 }, // dig gain, [3:1] c2:c0, [0] noise
	//{ 0x201d, 0x20 }, // c0 odd gain, 1/16 * val, 1/6 ~ 16
	//{ 0x201e, 0x20 }, // c0 even gain, 1/16 * val, 1/6 ~ 16
	//{ 0x201f, 0x20 }, // c1 odd gain, 1/16 * val, 1/6 ~ 16
	//{ 0x2020, 0x20 }, // c1 even gain, 1/16 * val, 1/6 ~ 16
	//{ 0x2021, 0x20 }, // c2 odd gain, 1/16 * val, 1/6 ~ 16
	//{ 0x2022, 0x20 }, // c2 even gain, 1/16 * val, 1/6 ~ 16

	//{ 0x3089, 0x08 }, // analog gain c0, 1 + 0.25 * val, 1x~4.75x
	//{ 0x308a, 0x08 }, // analog gain c1, 1 + 0.25 * val, 1x~4.75x
	//{ 0x308b, 0x08 }, // analog gain c2, 1 + 0.25 * val, 1x~4.75x

	//{ 0x1407, 0x29 }, //调节VREF电压值

	//{ 0x0005, 0x05 }, // lost
	//{ 0x0005, 0x05 },

	//{ 0x0006, 0x05 }, //调节CP输入频率
	//{ 0x31dc, 0x05 }, //调节FOT1时间倍数
	//{ 0x3075, 0x02 },
	//{ 0x309d, 0x00 },
	//{ 0x309c, 0x6d }, //pix_tim_val
	//{ 0x309f, 0x00 },
	//{ 0x309e, 0x6d }, //pix_tim_val
	//{ 0x30a1, 0x00 },
	//{ 0x30a0, 0x6d }, //pix_tim_val
	//{ 0x3100, 0x03 }, //调节PIX时间倍数
	//{ 0x1408, 0x29 }, //调节V_CLIP_RST
	//{ 0x315e, 0x81 }, //adc timing delay 200
	//{ 0x315f, 0x2c },
	//{ 0x3160, 0x00 },
	//{ 0x3161, 0x01 },
	//{ 0x3162, 0x21 },
	//{ 0x3163, 0x00 },
	//{ 0x3164, 0x01 },
	//{ 0x3165, 0x21 },
	//{ 0x3166, 0x00 },
	//{ 0x3167, 0x02 },
	//{ 0x3168, 0x40 },
	//{ 0x3169, 0x00 },
	//{ 0x316a, 0x28 },
	//{ 0x316b, 0x20 },
	//{ 0x316c, 0x00 },
	//{ 0x316d, 0x10 },
	//{ 0x316e, 0x68 },
	//{ 0x316f, 0x01 },
	//{ 0x3170, 0x80 },
	//{ 0x3171, 0x20 },
	//{ 0x3172, 0x00 },
	//{ 0x3173, 0x2a },
	//{ 0x3174, 0x28 },
	//{ 0x3175, 0x00 },
	//{ 0x3176, 0x80 },
	//{ 0x3177, 0x80 },
	//{ 0x3178, 0x00 },
	//{ 0x3179, 0x00 },
	//{ 0x317a, 0x21 },
	//{ 0x317b, 0x00 },
	//{ 0x317c, 0x00 },
	//{ 0x317d, 0x21 },
	//{ 0x317e, 0x00 },
	//{ 0x317f, 0x00 },
	//{ 0x3180, 0x24 },
	//{ 0x3181, 0x00 },
	//{ 0x3182, 0x04 },
	//{ 0x3183, 0x22 },
	//{ 0x3184, 0x00 },
	//{ 0x3185, 0x00 },
	//{ 0x3186, 0x24 },
	//{ 0x3187, 0x00 },
	//{ 0x3188, 0x48 },
	//{ 0x3189, 0x20 },
	//{ 0x318a, 0x00 },
	//{ 0x318b, 0x10 },
	//{ 0x318c, 0xa8 },
	//{ 0x318d, 0x01 },
	//{ 0x318e, 0x00 },
	//{ 0x318f, 0x00 },
	//{ 0x3190, 0x01 },
	//{ 0x3191, 0x80 },
	//{ 0x3192, 0x20 },
	//{ 0x3193, 0x00 },
	//{ 0x3194, 0x4c },
	//{ 0x3195, 0x28 },
	//{ 0x3196, 0x00 },
	//{ 0x3197, 0x80 },
	//{ 0x3198, 0x20 },
	//{ 0x3199, 0x00 },
	//{ 0x319a, 0x00 },
	//{ 0x319b, 0x30 },
	//{ 0x319c, 0x00 },
	//{ 0x319d, 0x00 },
	//{ 0x319e, 0x70 },
	//{ 0x319f, 0x00 },
	//{ 0x31a0, 0x00 },
	//{ 0x31a1, 0x02 },
	//{ 0x31a2, 0x00 },
	//{ 0x31a3, 0x00 },
	//{ 0x31a4, 0x00 },
	//{ 0x31a5, 0x00 },
	//{ 0x0c12, 0x20 }, // sensor stop
	//{ 0x0c12, 0x01 }, // sensor start
};

int dapperen100hy_sensor_initialize()
{
	int rtn = 0;

	for (int i = 0; i < sizeof(dapperen100hy_reglist) / sizeof(struct regList); i++){

		rtn |= dapperen100hy_write_reg(dapperen100hy_reglist[i].reg, dapperen100hy_reglist[i].val);
	}

	DDEBUG("dapperen100hy_sensor_initialize, return 0x%x \r\n", rtn);

	return rtn;
}

int dapperen100hy_sensor_load_defcfg()
{
	int rtn = 0;

	for (int i = 0; i < sizeof(dapperen100hy_defcfg_reglist) / sizeof(struct regList); i++) {

		rtn |= dapperen100hy_write_reg(dapperen100hy_defcfg_reglist[i].reg, dapperen100hy_defcfg_reglist[i].val);
	}

	DDEBUG("dapperen100hy_sensor_load_defcfg, return 0x%x \r\n", rtn);

	return rtn;
}

int dapperen100hy_get_sensor_id(uint16_t *id) // id should be 0x96
{
	int rtn = 0;
	uint8_t v1 = 0;
	uint8_t v2 = 0;

	static int isConfig = 0;
	if (!isConfig) {
		isConfig = 1;
		dapperen100hy_dothin_config();
		DDEBUG("config dothin \r\n");
	}

	//rtn |= dapperen100hy_read_reg(0x0c30, &v1);
	//rtn |= dapperen100hy_read_reg(0x0c31, &v2);
	*id = 0xe100;
	//if (!rtn)
	//	//*id = ((uint16_t)v1 << 8) | v2;
	//	*id = 0xe100;
	//else
	//	*id = 0xffff;

	DDEBUG("dapperen100hy chip id: 0x%x \r\n", *id);

	//rtn |= dapperen100hy_lm3644_read_reg(0x00, &v1);
	//rtn |= dapperen100hy_lm3644_read_reg(0x0c, &v2);

	//DDEBUG("aw3644 chip id: 0x%x, devce id: 0x%x \r\n", v1, v2);

	return rtn;
}

/*软触发*/
int dapperen100hy_video_streaming(bool enable)
{
	int rtn = 0;
#if 1
	if (enable)
		rtn |= dapperen100hy_write_reg(0x1022, 0x01);
	else{
		rtn |= dapperen100hy_write_reg(0x1019, 0x01);
		rtn |= dapperen100hy_write_reg(0x1019, 0x00);
	}

#endif
	return rtn;
}

/*进入deepsleep模式*/
int dapperen100hy_deepsleep_mode(bool enable)
{
	int rtn = 0;
	uint8_t val = 0;
	rtn |= dapperen100hy_read_reg(0x101b, &val);

#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x101b, 0x04 | val);
		DDEBUG("dapperen100hy deepsleep mode enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x101b, 0x00 | val);
		DDEBUG("dapperen100hy deepsleep mode disable");
	}
#endif
	return rtn;
}

/*进入hdr模式*/
int dapperen100hy_hdr_algorithm(bool enable)
{
	int rtn = 0;

	uint8_t val1 = 0, val2 = 0;

	rtn |= dapperen100hy_read_reg(0x3001, &val1);
	rtn |= dapperen100hy_read_reg(0x4015, &val2);

#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
		rtn |= dapperen100hy_write_reg(0x3001, 0x10^val1);//context0 HDR使能
		rtn |= dapperen100hy_write_reg(0x4000, 0x02);
		rtn |= dapperen100hy_write_reg(0x4015, val2 + 1);//行开窗多一行
		DDEBUG("dapperen100hy hdr_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x3001, 0x10^val1);
		rtn |= dapperen100hy_write_reg(0x4000, 0x00);
		rtn |= dapperen100hy_write_reg(0x4015, val2 - 1);
		DDEBUG("dapperen100hy hdr_algorithm disable");
	}
#endif

	return rtn;
}

/*hist算法使能*/
int dapperen100hy_hist_algorithm(bool enable)
{
	int rtn = 0;

	uint8_t val1 = 0,val2 = 0;

	rtn |= dapperen100hy_read_reg(0x3002, &val1);
	rtn |= dapperen100hy_read_reg(0x301d, &val2);
#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
		rtn |= dapperen100hy_write_reg(0x3002, 0x30^val1);//context0直方图算法使能
		rtn |= dapperen100hy_write_reg(0x301d, 0x10^val2);//emb数据使能，301d寄存器第4位写1
		rtn |= dapperen100hy_write_reg(0x301e, 0x2b);//ebd从MIPI输出数据格式，可以伪装成raw8/10, 则设置0x2a/2b
		DDEBUG("dapperen100hy hist_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x3002, 0x10^val1);
		rtn |= dapperen100hy_write_reg(0x301d, 0x10^val2);
		DDEBUG("dapperen100hy hist_algorithm disable");
	}
#endif
	return rtn;
}

/*median算法使能，需要多读两行*/
int dapperen100hy_median_algorithm(bool enable)
{
	int rtn = 0;
	uint8_t val1 = 0,val2 = 0;

	rtn |= dapperen100hy_read_reg(0x3001, &val1);
	rtn |= dapperen100hy_read_reg(0x4015, &val2);
#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
		rtn |= dapperen100hy_write_reg(0x3001, 0x01^val1);//context0 median算法使能
		rtn |= dapperen100hy_write_reg(0x4015, val2 + 2);//开窗多2行
		DDEBUG("dapperen100hy median_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x3001, 0x01^val1);
		rtn |= dapperen100hy_write_reg(0x4015, val2 - 2);
		DDEBUG("dapperen100hy median_algorithm disable");
	}
#endif
	return rtn;
}

/*ebc+thrw算法（列校正）使能*/
int dapperen100hy_ebc_algorithm(bool enable)
{
	int rtn = 0;
#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x3000, 0x15);//ISP使能,context0 ebc/thrw算法使能
		rtn |= dapperen100hy_write_reg(0x4013, 0x46);//context0 eb像素行读取次数
		rtn |= dapperen100hy_write_reg(0x3003, 0x06);//需要丢弃的eb像素行数
		rtn |= dapperen100hy_write_reg(0x3005, 0xB4);//EDPC的阈值
		rtn |= dapperen100hy_write_reg(0x3007, 0xB4);//列校正offset
		DDEBUG("dapperen100hy ebc_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);
		rtn |= dapperen100hy_write_reg(0x4013, 0x00);//context0 eb像素行读取次数
		rtn |= dapperen100hy_write_reg(0x3003, 0x00);//需要丢弃的eb像素行数
		DDEBUG("dapperen100hy ebc_algorithm disable");
	}
#endif
	return rtn;
}

/*lsc算法使能*/
int dapperen100hy_lsc_algorithm(bool enable)
{
	int rtn = 0;
#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
		rtn |= dapperen100hy_write_reg(0x3001, 0x0c);//context0&context1 lsc算法使能
		DDEBUG("dapperen100hy lsc_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x3000, 0x01);
		DDEBUG("dapperen100hy lsc_algorithm disable");
	}
#endif
	return rtn;
}

/*行校正算法使能*/
int dapperen100hy_correction_algorithm(bool enable)
{
	int rtn = 0;
#if 1
	if (enable){
		rtn |= dapperen100hy_write_reg(0x2000, 0x06);//行校正使能
		rtn |= dapperen100hy_write_reg(0x2005, 0x32);//行校正offset
		DDEBUG("dapperen100hy lsc_algorithm enable");
	}
	else{
		rtn |= dapperen100hy_write_reg(0x2000, 0x00);
		DDEBUG("dapperen100hy lsc_algorithm disable");
	}
#endif
	return rtn;
}

/*获取当前模式*/
int dapperen100hy_get_data_output_mode(uint8_t *mode)
{
	int rtn = 0;
	uint8_t val1 = 0, val2 = 0;

	rtn |= dapperen100hy_read_reg(0x1021, &val1);
	rtn |= dapperen100hy_read_reg(0x101b, &val2);

	if (val1 == 0x01){
		*mode = 0x00;
	}
	else if ((val1 == 0x00)&(val2 & 0x01 == 0x00)){
		*mode = 0x01;
	}
	else if ((val1 == 0x00)&(val2 & 0x01 == 0x01)){
		*mode = 0x02;
	}
	else if (val1 == 0x12){
		*mode = 0x03;
	}
	else if ((val1 == 0x02)&(val2 == 0x00)){
		*mode = 0x04;
	}
	else if ((val1 == 0x02)&(val2 & 0x02 == 0x02)){
		*mode = 0x05;
	}
	else if ((val1 == 0x06)&(val2 & 0x02 == 0x02)){
		*mode = 0x06;
	}
	else if ((val1 == 0x0f)&(val2 & 0x02 == 0x02)){
		*mode = 0x07;
	}
	DDEBUG("dapperen100hy get output mode: 0x%x \r\n", *mode);

	return rtn;
}

/*normal & motion模式设置*/
int dapperen100hy_set_data_output_mode(uint8_t mode)
{
	int rtn = 0;
	uint8_t val = 0, en = 0;

	rtn |= dapperen100hy_video_streaming(0);//关流
	rtn |= dapperen100hy_read_reg(0x101b, &val);

	val = val & 0x04;

	switch (mode)
	{
	case NORMAL_FREE_RUN:
		dapperen100hy_write_reg(0x1021, 0x01);
		break;
	case NORMAL_BURST_AUTO_OFF:
		dapperen100hy_write_reg(0x101b, 0x00 | val); //auto off
		dapperen100hy_write_reg(0x1021, 0x00);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		break;
	case NORMAL_BURST_AUTO_ON:
		dapperen100hy_write_reg(0x101b, 0x01 | val); //auto on
		dapperen100hy_write_reg(0x1021, 0x00);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		dapperen100hy_write_reg(0x101c, 0x0b); //间隔时间12ms
		dapperen100hy_write_reg(0x101d, 0x01);
		break;
	case MOTION_FREE_RUN:
		dapperen100hy_write_reg(0x1021, 0x12);
		dapperen100hy_write_reg(0x3026, 0x04);
		dapperen100hy_write_reg(0x1014, 0x01);//gpio9_func_sel切换到motion
		break;
	case MOTION_BURST_AUTO_OFF:
		dapperen100hy_write_reg(0x3026, 0x06);
		dapperen100hy_write_reg(0x101b, 0x00 | val);
		dapperen100hy_write_reg(0x1021, 0x02);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		break;
	case MOTION_BURST_AUTO_ON_STOP:
		dapperen100hy_write_reg(0x3026, 0x06);
		dapperen100hy_write_reg(0x101b, 0x02 | val);
		dapperen100hy_write_reg(0x1021, 0x02);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		dapperen100hy_write_reg(0x1029, 0x0b); //间隔时间
		dapperen100hy_write_reg(0x102a, 0x00);
		break;
	case MOTION_BURST_AUTO_ON_KEEP:
		dapperen100hy_write_reg(0x3026, 0x06);
		dapperen100hy_write_reg(0x101b, 0x02 | val);
		dapperen100hy_write_reg(0x1021, 0x06);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		dapperen100hy_write_reg(0x1029, 0x0b); //间隔时间
		dapperen100hy_write_reg(0x102a, 0x00);
		break;
	case MOTION_BURST_AUTO_ON_NORMAL:
		dapperen100hy_write_reg(0x3026, 0x06);
		dapperen100hy_write_reg(0x101b, 0x02 | val);
		dapperen100hy_write_reg(0x1021, 0x0f);
		dapperen100hy_write_reg(0x4023, 0x05); //单次burst出的帧数
		dapperen100hy_write_reg(0x4024, 0x00);
		dapperen100hy_write_reg(0x1029, 0x0b); //间隔时间
		dapperen100hy_write_reg(0x102a, 0x00);
		break;
	}

	DDEBUG("dapperen100hy set output mode: 0x%x \r\n", mode);

	return rtn;
}

/*设置行开窗起始位置*/
int dapperen100hy_set_window_originy(uint32_t originy)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = originy & 0xff;
	val_h = (originy >> 8) & 0x03;

	rtn |= dapperen100hy_write_reg(0x4019, val_l);
	rtn |= dapperen100hy_write_reg(0x401a, val_h);//context0
	rtn |= dapperen100hy_write_reg(0x401b, val_l);
	rtn |= dapperen100hy_write_reg(0x401c, val_h);//context1

	DDEBUG("dapperen100hy set window originy: 0x%x \r\n", originy);

	return rtn;
}

/*设置列开窗起始位置*/
int dapperen100hy_set_window_originx(uint32_t originx)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = originx & 0xff;
	val_h = (originx >> 8) & 0x03;
	uint8_t val = 0;
	rtn |= dapperen100hy_read_reg(0x3001, &val);

	rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
	rtn |= dapperen100hy_write_reg(0x3001, 0xc0^val);//context0/context1开窗使能打开
	rtn |= dapperen100hy_write_reg(0x3015, val_l);
	rtn |= dapperen100hy_write_reg(0x3016, val_h);

	DDEBUG("dapperen100hy set window originx: 0x%x \r\n", originx);

	return rtn;
}

/*设置行开窗高度*/
int dapperen100hy_set_window_height(uint32_t height)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = height & 0xff;
	val_h = (height >> 8) & 0x03;

	rtn |= dapperen100hy_write_reg(0x4015, val_l);
	rtn |= dapperen100hy_write_reg(0x4016, val_h);//context0
	rtn |= dapperen100hy_write_reg(0x4017, val_l);
	rtn |= dapperen100hy_write_reg(0x4018, val_h);//context1

	DDEBUG("dapperen100hy set window height: 0x%x \r\n", height);

	return rtn;
}

/*获取行开窗高度*/
int dapperen100hy_get_window_height(uint32_t *height)
{
	int rtn = 0;
	uint32_t val_h = 0;
	uint32_t val_l = 0;

	rtn |= dapperen100hy_read_reg(0x4015, &val_l);
	rtn |= dapperen100hy_read_reg(0x4016, &val_h);
	*height = (val_h << 0x08) | val_l;

	DDEBUG("dapperen100hy get window height: 0x%x \r\n", *height);

	return rtn;
}

/*设置列开窗宽度*/
int dapperen100hy_set_window_width(uint32_t width)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = width & 0xff;
	val_h = (width >> 8) & 0x0f;
	uint8_t val = 0;
	rtn |= dapperen100hy_read_reg(0x3001, &val);

	rtn |= dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
	rtn |= dapperen100hy_write_reg(0x3001, 0xc0^val);//context0/context1开窗使能打开
	rtn |= dapperen100hy_write_reg(0x3017, val_l);
	rtn |= dapperen100hy_write_reg(0x3018, val_h);

	DDEBUG("dapperen100hy set window width: 0x%x \r\n", width);

	return rtn;
}

/*获取列开窗宽度*/
int dapperen100hy_get_window_width(uint32_t *width)
{
	int rtn = 0;
	uint32_t val_h = 0;
	uint32_t val_l = 0;

	rtn |= dapperen100hy_read_reg(0x3017, &val_l);
	rtn |= dapperen100hy_read_reg(0x3018, &val_h);
	*width = (val_h << 8) | val_l;

	DDEBUG("dapperen100hy get window width: 0x%x \r\n", *width);

	return rtn;
}

int dapperen100hy_get_fps(uint8_t *fps)
{
	return -HW_ERR_NO_SUPPORT;
}

int dapperen100hy_set_fps(uint8_t fps)
{
	return -HW_ERR_NO_SUPPORT;
}

int dapperen100hy_get_integration_time(uint16_t *integrationTime)
{
	int rtn = 0;
	uint8_t v1 = 0, v2 = 0;
	uint32_t tslot = 0, exp = 0;

	// get tslot
	rtn = dapperen100hy_read_reg(0x3010, &v1);
	rtn = dapperen100hy_read_reg(0x3011, &v2);
	tslot = ((uint32_t)v2 << 8) | v1;

	// get exp
	rtn = dapperen100hy_read_reg(0x3016, &v1);
	rtn = dapperen100hy_read_reg(0x3017, &v2);
	exp = ((uint32_t)v2 << 8) | v1;

	*integrationTime = (uint16_t)((float)exp * tslot / DAPPEREN100HY_PCLK_MHZ);

	DDEBUG("dapperen100hy_get_exp %d \r\n", *integrationTime);

	return rtn;
}

int dapperen100hy_set_integration_time(uint16_t integrationTime)
{
	int rtn = 0;
	uint8_t v1 = 0, v2 = 0;
	uint32_t tslot = 0, exp = 0;

	// get tslot
	rtn = dapperen100hy_read_reg(0x3010, &v1);
	rtn = dapperen100hy_read_reg(0x3011, &v2);
	tslot = ((uint32_t)v2 << 8) | v1;


	exp = (uint32_t)(integrationTime * DAPPEREN100HY_PCLK_MHZ / (float)tslot);

	v1 = exp & 0xff;
	v2 = (exp >> 8) & 0xff;

	// set gs exp
	rtn = dapperen100hy_write_reg(0x3016, v1);
	rtn = dapperen100hy_write_reg(0x3017, v2);
	rtn = dapperen100hy_write_reg(0x3018, v1);
	rtn = dapperen100hy_write_reg(0x3019, v2);
	rtn = dapperen100hy_write_reg(0x301a, v1);
	rtn = dapperen100hy_write_reg(0x301b, v2);
	// set rs exp
	exp /= 4;
	v1 = exp & 0xff;
	v2 = (exp >> 8) & 0xff;
	rtn = dapperen100hy_write_reg(0x3022, v1);
	rtn = dapperen100hy_write_reg(0x3023, v2);
	rtn = dapperen100hy_write_reg(0x3024, v1);
	rtn = dapperen100hy_write_reg(0x3025, v2);
	rtn = dapperen100hy_write_reg(0x3026, v1);
	rtn = dapperen100hy_write_reg(0x3027, v2);

	DDEBUG("dapperen100hy_set_exp %d, return 0x%x \r\n", integrationTime, rtn);

	return rtn;
}

/*读取曝光时间*/
int dapperen100hy_get_exp(uint32_t *exposure)
{
	int rtn = 0;
	uint8_t v1 = 0, v2 = 0;
	uint32_t tslot = 0, exp = 0;

	// get tslot
	rtn = dapperen100hy_read_reg(0x4007, &v1);
	rtn = dapperen100hy_read_reg(0x4008, &v2);
	tslot = ((uint32_t)v2 << 8) | v1;

	// get exp
	rtn = dapperen100hy_read_reg(0x400b, &v1);
	rtn = dapperen100hy_read_reg(0x400c, &v2);
	exp = ((uint32_t)v2 << 8) | v1;

	*exposure = (uint32_t)((float)exp * tslot / DAPPEREN100HY_PCLK_MHZ);

	DDEBUG("dapperen100hy_get_exp %d \r\n", *exposure);

	return rtn;
}

/*设置曝光时间*/
int dapperen100hy_set_exp(uint32_t exposure)
{
	int rtn = 0;
	uint8_t v1 = 0, v2 = 0;
	uint32_t tslot = 0, exp = 0;

	// get tslot
	rtn = dapperen100hy_read_reg(0x4007, &v1);
	rtn = dapperen100hy_read_reg(0x4008, &v2);
	tslot = ((uint32_t)v2 << 8) | v1;


	exp = (uint32_t)(exposure * DAPPEREN100HY_PCLK_MHZ / (float)tslot);

	v1 = exp & 0xff;
	v2 = (exp >> 8) & 0xff;

	// set gs exp
	rtn = dapperen100hy_write_reg(0x400b, v1);
	rtn = dapperen100hy_write_reg(0x400c, v2);

	DDEBUG("dapperen100hy_set_exp %d, return 0x%x \r\n", exposure, rtn);

	return rtn;
}

int dapperen100hy_get_gain(uint32_t *gain)
{
	return  -HW_ERR_NO_SUPPORT;
}

/*设置模拟增益*/
int dapperen100hy_set_gain(uint32_t gain)
{
	int rtn = 0;
	//switch (gain)
	//{
	//case 0:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x3F);//context0,1x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x3F);//context1
	//	break;
	//case 1:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x2A);//context0,1.5x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x2A);//context1
	//	break;
	//case 2:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x1F);//context0,2x增益   
	//	rtn = dapperen100hy_write_reg(0x403e, 0x1F);//context1
	//	break;
	//case 3:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x19);//context0,2.5x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x19);//context1
	//	break;
	//case 4:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x14);//context0,3x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x14);//context1
	//	break;
	//case 5:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x11);//context0,3.5x增益   
	//	rtn = dapperen100hy_write_reg(0x403e, 0x11);//context1
	//	break;
	//case 6:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x0F);//context0,4x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x0F);//context1
	//	break;
	//case 7:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x07);//context0,8x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x07);//context1
	//	break;
	//case 8:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x03);//context0,16x增益   
	//	rtn = dapperen100hy_write_reg(0x403e, 0x03);//context1
	//	break;
	//case 9:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x01);//context0,32x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x01);//context1
	//	break;
	//case 10:
	//	rtn = dapperen100hy_write_reg(0x403d, 0x00);//context0,64x增益
	//	rtn = dapperen100hy_write_reg(0x403e, 0x00);//context1
	//	break;
	//}
	uint8_t val1 = 0;
	val1 = gain < 1 ? 1 : (gain > 64 ? 64 : gain);
	float val2 = 0;
	val2 = 64 / val1 - 1;
	if (val2 - (int)val2 >= 0.5){
		val2++;
	}

	rtn = dapperen100hy_write_reg(0x403d, (int)val2 & 0x3f);//context0
	rtn = dapperen100hy_write_reg(0x403e, (int)val2 & 0x3f);//context1

	DDEBUG("dapperen100hy_set_gain %d, return 0x%x \r\n", gain, rtn);

	return rtn;
}

/*设置数字增益*/
int dapperen100hy_set_odd_dgain(uint32_t gain)
{
	int rtn = 0;

	rtn = dapperen100hy_write_reg(0x3000, 0x01);
	rtn = dapperen100hy_write_reg(0x3002, 0x01);//context0数字增益使能
	rtn = dapperen100hy_write_reg(0x3019, gain);//context0数字增益0~255

	DDEBUG("dapperen100hy_set_odd_dgain %d, return 0x%x \r\n", gain, rtn);

	return rtn;
}

int dapperen100hy_set_even_dgain(uint32_t gain)
{
	return  -HW_ERR_NO_SUPPORT;
}

/*设置列抽样*/
int dapperen100hy_set_sub_samp(uint8_t value)
{
	int rtn = 0;

	rtn = dapperen100hy_write_reg(0x3000, 0x01);//ISP使能
	if (value == 0){
		rtn = dapperen100hy_write_reg(0x3002, 0x00);//列抽样关闭
	}
	else{
		rtn = dapperen100hy_write_reg(0x3002, 0xc0);//列抽样开启
	}
	rtn = dapperen100hy_write_reg(0x301c, value);//列抽样间隔

	DDEBUG("dapperen100hy_set_sub_samp %d, return 0x%x \r\n", value, rtn);

	return rtn;
}

/*获取列抽样间隔*/
int dapperen100hy_get_sub_samp(uint8_t *value)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn = dapperen100hy_read_reg(0x301c, &val);

	*value = val;

	DDEBUG("dapperen100hy_get_sub_samp %d \r\n", *value);

	return rtn;
}

/*设置行抽样*/
int dapperen100hy_set_sub_sampv(uint8_t value)
{
	int rtn = 0;

	rtn = dapperen100hy_write_reg(0x4038, value);//行抽样间隔

	DDEBUG("dapperen100hy_set_sub_sampv %d, return 0x%x \r\n", value, rtn);

	return rtn;
}

/*获取行抽样间隔*/
int dapperen100hy_get_sub_sampv(uint8_t *value)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn = dapperen100hy_read_reg(0x4038, &val);

	*value = val;

	DDEBUG("dapperen100hy_get_sub_sampv %d \r\n", *value);

	return rtn;
}

int dapperen100hy_get_ldm_en(uint32_t *en)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn |= dapperen100hy_lm3644_ldm_read_reg(0x01, &val);
	if (val & 0x01)
		*en = 1;

	DDEBUG("dapperen100hy_get_ldm_en %d \r\n", *en);

	return rtn;
}

int dapperen100hy_set_ldm_en(uint32_t en)
{
	int rtn = 0;

	if (en)
		rtn |= dapperen100hy_lm3644_ldm_write_reg(0x01, 0x25);
	else
		rtn |= dapperen100hy_lm3644_ldm_write_reg(0x01, 0x24);

	DDEBUG("dapperen100hy_set_ldm_en %d \r\n", en);

	return rtn;
}

int dapperen100hy_get_led_en(uint32_t *en)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn |= dapperen100hy_lm3644_led_read_reg(0x01, &val);
	if (val & 0x01)
		*en = 1;

	DDEBUG("dapperen100hy_get_led_en %d \r\n", *en);

	return rtn;
}

int dapperen100hy_set_led_en(uint32_t en)
{
	int rtn = 0;

	if (en)
		rtn |= dapperen100hy_lm3644_led_write_reg(0x01, 0x25);
	else
		rtn |= dapperen100hy_lm3644_led_write_reg(0x01, 0x24);

	DDEBUG("dapperen100hy_set_led_en %d \r\n", en);

	return rtn;
}
int dapperen100hy_get_ldm_current(uint32_t *current)
{
	int rtn = 0;
	uint8_t val = 0;


	dapperen100hy_lm3644_ldm_read_reg(0x03, &val);

	*current = (val & 0x7f) * 12;

	DDEBUG("dapperen100hy_get_ldm_current %d \r\n", *current);

	return rtn;
}

int dapperen100hy_set_ldm_current(uint32_t current)
{
	int rtn = 0;
	uint8_t val = 0;

	val = (current > 1524 ? 1524 : current) / 12;

	rtn |= dapperen100hy_lm3644_ldm_write_reg(0x03, val);

	DDEBUG("dapperen100hy_set_ldm_current %d, return 0x%x \r\n", current, rtn);

	return rtn;
}

int dapperen100hy_get_led_current(uint32_t *current)
{
	int rtn = 0;
	uint8_t val = 0;


	dapperen100hy_lm3644_led_read_reg(0x03, &val);

	*current = (val & 0x7f) * 12;

	DDEBUG("dapperen100hy_get_led_current %d \r\n", *current);

	return rtn;
}

int dapperen100hy_set_led_current(uint32_t current)
{
	int rtn = 0;
	uint8_t val = 0;

	val = (current > 1524 ? 1524 : current) / 12;

	rtn |= dapperen100hy_lm3644_led_write_reg(0x03, val);

	DDEBUG("dapperen100hy_set_led_current %d, return 0x%x \r\n", current, rtn);

	return rtn;
}

/*设置binning模式*/
int dapperen100hy_set_pixel_binning(uint8_t mode)
{
	int rtn = 0;
	uint8_t val1 = 0, val2 = 0;

	rtn = dapperen100hy_read_reg(0x3000, &val1);//ISP使能
	rtn = dapperen100hy_read_reg(0x3002, &val2);//ISP使能

	rtn = dapperen100hy_write_reg(0x3000, 0x01^val1);//ISP使能
	rtn = dapperen100hy_write_reg(0x3002, 0x0c);//context0、context1 bining使能

	if (mode == 0){
		rtn = dapperen100hy_write_reg(0x3002, 0x00);
		rtn = dapperen100hy_write_reg(0x301b, 0x00);//1*1binning
	}
	else if (mode == 1){
		rtn = dapperen100hy_write_reg(0x301b, 0x01);//2*2binning
	}
	else if (mode == 2){
		rtn = dapperen100hy_write_reg(0x301b, 0x02);//3*3binning
	}
	else{
		rtn = dapperen100hy_write_reg(0x301b, 0x00);//1*1binning
	}

	DDEBUG("dapperen100hy set bining mode:%d, return 0x%x \r\n", mode, rtn);

	return rtn;
}

/*获取binning模式*/
int dapperen100hy_get_pixel_binning(uint8_t *mode)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn |= dapperen100hy_read_reg(0x301b, &val);
	*mode = val;

	DDEBUG("dapperen100hy get bining mode: 0x%x \r\n", *mode);

	return rtn;
}

int dapperen100hy_get_img_mirror_flip(uint8_t *mode)
{
	return -HW_ERR_NO_SUPPORT;
}

int dapperen100hy_set_img_mirror_flip(uint8_t mode)
{
	int rtn = 0;
	uint8_t val = 0;

	switch (mode)
	{
	case 0:
		rtn = dapperen100hy_write_reg(0x2000, 0x00);//水平翻转关闭
		rtn = dapperen100hy_write_reg(0x401d, 0x00);//context0垂直翻转关闭
		break;
	case 1:
		rtn = dapperen100hy_write_reg(0x401d, 0x01);//context0垂直翻转开
		rtn = dapperen100hy_write_reg(0x401e, 0x01);//context1垂直翻转开
		break;
	case 2:
		rtn = dapperen100hy_write_reg(0x2000, 0xc0);//水平翻转开
		break;
	}

	DDEBUG("dapperen100hy set flip mode:%d, return 0x%x \r\n", mode, rtn);

	return rtn;
}

int dapperen100hy_get_sensor_temperature(float *temp)
{
	return  -HW_ERR_NO_SUPPORT;
}

int dapperen100hy_test_pattern(uint8_t mode)
{
	int rtn = 0;

	return rtn;
}

int dapperen100hy_get_sensor_info(struct sensor_info_t *info) {
	info->embedded_data_size = IMAGE_WIDTH * 2;
	info->vcsel_num = vcsel_number;
	info->vcsel_driver_id = vcsel_driver_type;
	info->sensor_id = dapperen100hy_sensor_id;
	return 0;
}

int dapperen100hy_ldm_led_initialize()
{
	int rtn = 0;

	for (int i = 0; i < sizeof(lm3644_reglist) / sizeof(struct regList); i++) {

		rtn |= dapperen100hy_lm3644_write_reg(lm3644_reglist[i].reg & 0xff, lm3644_reglist[i].val);
	}

	DDEBUG("dapperen100hy_ldm_initialize, return 0x%x \r\n", rtn);

	return rtn;
}

int dapperen100hy_led_initialize()
{
	int rtn = 0;

	return rtn;
}

int dapperen100hy_init()
{
	int rtn = 0;

	// set LDM and LED driver
	//rtn |= dapperen100hy_ldm_led_initialize();
	//rtn |= dapperen100hy_set_ldm_en(1);
	//rtn |= dapperen100hy_set_illum_power(3, 100, 100);


	// set dapperen100hy sensor
	rtn |= dapperen100hy_sensor_initialize();
	//rtn |= dapperen100hy_set_data_output_mode(0);

	return rtn;
}

int dapperen100hy_set_chip_reset(uint32_t reset_en)
{
	int rtn = 0;

	if (reset_en)
		rtn = dapperen100hy_init();

	return rtn;
}

int dapperen100hy_func_init()
{
#if !DEBUG_DAPPEREN100HY_IN_QT
	tof_sensor.init = dapperen100hy_init;
	tof_sensor.get_sensor_id = dapperen100hy_get_sensor_id;
	tof_sensor.video_streaming = dapperen100hy_video_streaming;
	tof_sensor.deepsleep_mode = dapperen100hy_deepsleep_mode;
	tof_sensor.hdr_algorithm = dapperen100hy_hdr_algorithm;
	tof_sensor.hist_algorithm = dapperen100hy_hist_algorithm;
	tof_sensor.median_algorithm = dapperen100hy_median_algorithm;
	tof_sensor.ebc_algorithm = dapperen100hy_ebc_algorithm;
	tof_sensor.lsc_algorithm = dapperen100hy_lsc_algorithm;
	tof_sensor.correction_algorithm = dapperen100hy_correction_algorithm;
	tof_sensor.get_fps = dapperen100hy_get_fps;
	tof_sensor.set_fps = dapperen100hy_set_fps;
	tof_sensor.get_sensor_temperature = dapperen100hy_get_sensor_temperature;
	tof_sensor.set_illum_power = dapperen100hy_set_illum_power;
	tof_sensor.get_illum_power = dapperen100hy_get_illum_power;
	tof_sensor.get_integration_time = dapperen100hy_get_integration_time;
	tof_sensor.set_integration_time = dapperen100hy_set_integration_time;
	tof_sensor.get_data_output_mode = dapperen100hy_get_data_output_mode;
	tof_sensor.set_data_output_mode = dapperen100hy_set_data_output_mode;
	tof_sensor.set_window_originy = dapperen100hy_set_window_originy;
	tof_sensor.set_window_originx = dapperen100hy_set_window_originx;
	tof_sensor.set_window_height = dapperen100hy_set_window_height;
	tof_sensor.get_window_height = dapperen100hy_get_window_height;
	tof_sensor.set_window_width = dapperen100hy_set_window_width;
	tof_sensor.get_window_width = dapperen100hy_get_window_width;
	tof_sensor.get_img_mirror_flip = dapperen100hy_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = dapperen100hy_set_img_mirror_flip;
	tof_sensor.get_pixel_binning = dapperen100hy_get_pixel_binning;
	tof_sensor.set_pixel_binning = dapperen100hy_set_pixel_binning;
	tof_sensor.test_pattern = dapperen100hy_test_pattern;
	tof_sensor.get_sensor_info = dapperen100hy_get_sensor_info;
	tof_sensor.sensor_write_reg_8 = dapperen100hy_demo_write_reg;
	tof_sensor.sensor_read_reg_8 = dapperen100hy_demo_read_reg;
	tof_sensor.get_exp = dapperen100hy_get_exp;
	tof_sensor.set_exp = dapperen100hy_set_exp;
	tof_sensor.get_gain = dapperen100hy_get_gain;
	tof_sensor.set_gain = dapperen100hy_set_gain;
	tof_sensor.set_odd_dgain = dapperen100hy_set_odd_dgain;
	tof_sensor.set_even_dgain = dapperen100hy_set_even_dgain;
	tof_sensor.set_sub_samp = dapperen100hy_set_sub_samp;
	tof_sensor.get_sub_samp = dapperen100hy_get_sub_samp;
	tof_sensor.set_sub_sampv = dapperen100hy_set_sub_sampv;
	tof_sensor.get_sub_sampv = dapperen100hy_get_sub_sampv;
	tof_sensor.get_ldm_en = dapperen100hy_get_ldm_en;
	tof_sensor.set_ldm_en = dapperen100hy_set_ldm_en;
	tof_sensor.get_led_en = dapperen100hy_get_led_en;
	tof_sensor.set_led_en = dapperen100hy_set_led_en;
	tof_sensor.get_ldm_current = dapperen100hy_get_ldm_current;
	tof_sensor.set_ldm_current = dapperen100hy_set_ldm_current;
	tof_sensor.get_led_current = dapperen100hy_get_led_current;
	tof_sensor.set_led_current = dapperen100hy_set_led_current;
	tof_sensor.set_chip_reset = dapperen100hy_set_chip_reset;
#endif
	return 0;
}
