#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "spi\inc\msg-queue.h"
#include "hw_obstatus.h"
#include "rk1608_pleco.h"
#include "debug2log.h"
#include "hw_property.h"
#include "tofinfo.h"

#ifdef __linux__
#include <unistd.h>
#endif

#include <tof_sensors.h>
#include <obc_tee_funcs.h>

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread       tops_t.ops_writeread
//#define orbbec_spi_writeread       tops_t.ap_ops.SensorSpiRWEx
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

#define PLECO_I2C_ADDR              0x20 // TOF sensor
#define TCA6408A_I2C_ADDR           0x40 // GPIO expander

// vcsel driver IC type
#define DRIVER_IC_CXA4016              4016
#define DRIVER_IC_PHX3D_3021_AA        5016
#define DRIVER_IC_PHX3D_3021_CB        5017
#define DRIVER_IC_DW9912               9912 // DongWoon
#define DAC5574_I2C_ADDR               0x98 // DAC
static uint16_t driver_ic_type = 0;

#define DUAL_FREQ      1
#define SINGLE_FREQ    0
#define AF_FREQ        2

#define BOOT_FROM_FLASH   0
#define BOOT_FROM_DDR     1

#define DUAL_FREQ_NO_BINNING_RESOLUTION         RESOLUTION_1920_2880
#define DUAL_FREQ_2X2_BINNING_RESOLUTION        0
#define DUAL_FREQ_4X4_BINNING_RESOLUTION        0

#define SINGLE_FREQ_NO_BINNING_RESOLUTION       RESOLUTION_1920_1440
#define SINGLE_FREQ_2X2_BINNING_RESOLUTION      0
#define SINGLE_FREQ_4X4_BINNING_RESOLUTION      0

#define PIXEL_BIT      10
#define MIPI_PACK_BIT   8

static ObcSensorInfo get_src_info;

struct regList {
	uint16_t reg;
	uint8_t val;
};

struct regList rk1608_pleco_reglist[] = {
	{ 16, 0x01 }, // group hold pull high

	{ 580 ,179 },// 179 -> DB test
	{ 38 , 228 },// 484
	{ 39 , 1 },
	{ 740 , 160 }, // 1440
	{ 741 , 5 },
	{ 784 , 224 }, // 480
	{ 785 , 1 },
	{ 704 , 239 },
	{ 702 , 226 },// 482
	{ 703 , 1 },

	// EBD length 2880 byte
	{ 735 , 64 },
	{ 736 , 11 },
	{ 705 , 0x68 },
	{ 706 , 12 },
	{ 67 , 0x44 },
	{ 68 , 0x44 },

	{ 74 ,100 },// ADC error
	{ 75 ,17 },

	{ 35 ,10 },// PGA gain

	{ 593 ,169 },// analog test ramp_st,default 9
				 //{ 594 ,0xE1 },// analog test ramp_st,

	{ 585 ,43 },// vramp_st adjust value  // 0.94V
	{ 586 ,41 },// vramp_st adjust value

	{ 578 ,0xB2 }, // DB offset value
	{ 579 ,0x6B }, // DB offset value
	{ 224 ,0 },// test bias continue

	{ 580 ,177 },// DB test pattern, default+2

	{ 702 ,170 },// 1450, rd_line_max, 3 subframe
	{ 703 , 5 },

	//{ 18 ,118 },// horizontal & vertical flip
	{ 18 ,86 },// vertical flip

			   // DMFD
	{ 784 ,224 },// 480
	{ 785 ,1 },
	{ 702 ,86 },// 2902, rd_line_max, 6 subframe
	{ 703 , 11 },
	{ 704 , 239 },
	{ 740 , 160 },//1440
	{ 741 , 5 },
	{ 31, 170 },// frame number 10922 in DMFD
	{ 32, 42 },

	// 2lane 1.4Gbps
	{ 62 ,180 },// 200 -> 1.6Gbps, 174 -> 1.44Gbps
				//{ 635, 178 },
				//{ 634, 37},

	{ 66 ,0x2B },// data type RAW 10
	{ 576, 0xAC },// ADC 10bit
#if 1
	{ 735 , 0 },// RAW10, EBD length 1280 byte
	{ 736 , 5 },
	{ 705 , 43 },// EBD pixel num = EBD length/8 -1 = 159
	{ 706 , 12 },
#else
	{ 735 , 96 },// RAW10, EBD length 2400 byte
	{ 736 , 9 },
	{ 705 , 43 },// EBD pixel num = EBD length/8 -1 = 299
	{ 706 , 12 },
#endif

	{ 632 ,13 },//VSG_M bg value adjust  0V
	{ 633 ,0 }, // VTG_M bg value adjust
	{ 634 ,160 },//VDRN_M bg value adjust

	{ 602 ,19 },//VDRN LOW 0V   147 0V    179  -0.2V   211 -0.4V    243  -0.6V
	{ 603 ,166 },//VDRN LOW

				 //{583 ,130},// PGA current
	{ 609 ,18 },// SG LO1   0V   best

				//{611 ,213},//SG L2
	{ 609 ,26 },//SG L1
	{ 608, 244 },// dark current utilize
	{ 569, 1 },// disable antijamm

	{ 742, 184 },// af_vld_line
	{ 743, 2 },

	// power consumption utilize
	{ 0x154, 0x01 },
	{ 0x257, 0x63 },
	{ 0x258, 0x50 },
	{ 0x263, 0x5A },
	{ 0x264, 0x0C },

	{ 16, 0x00 }, // group hold pull low
};

// PHX3D 3025 almost the same with PHX3D 3021 except XCLR mask control function
// (PHX3D 3025, register 0x2E bit 0 should set to 1, while PHX3D 3021 don't care this)
static struct regList phx3d_3021_cb_reglist[] = {

	// PHX3D Laser Driver reset
	{ 356,  0x04 },
	{ 375,  0x02 },
	{ 376,  0x25 },
	{ 377,  0x01 },
	{ 354,  0x03 },
	{ 354,  0x00 },

	// PHX3D Laser Driver initial
	{ 356,  0x07 },
	{ 378,  0x15 },
	{ 379,  0x00 },
	{ 380,  0x0C }, // 0x00
	{ 381,  0x20 }, // 0x01
	{ 382,  0x30 }, // 0x02
	{ 383,  0x10 }, // 0x03
	{ 384,  0x20 }, // 0x04
	{ 385,  0x69 }, // 0x05
	{ 386,  0xC3 }, // 0x06
	{ 387,  0x00 }, // 0x07 IBIAS_FIX
	{ 388,  0xD3 }, // 0x08 ISW_FIX
	{ 389,  0x24 }, // 0x09
	{ 390,  0x0D }, // 0x0A
	{ 391,  0xFF }, // 0x0B
	{ 392,  0xD9 }, // 0x0C
	{ 393,  0xFF }, // 0x0D
	{ 394,  0x78 }, // 0x0E ISW_FIX threshold
	{ 395,  0x01 }, // 0x0F
	{ 396,  0xDF }, // 0x10
	{ 397,  0x00 }, // 0x11
	{ 398,  0x00 }, // 0x12
	{ 399,  0x01 }, // 0x13
	{ 354,  0x03 }, // send1 channel register trig
	{ 354,  0x00 },

	{ 356,  0x1D },
	{ 400,  0x02 },
	{ 401,  0x26 },
	{ 402,  0x19 }, // 0x26  internal PD resistance
					//{ 403,  0x31 }, // 0x27
					//{ 404,  0xFF }, // 0x28
	{ 354,  0x03 },
	{ 354,  0x00 },

	{ 356,  0x22 },
	{ 405,  0x09 },
	{ 406,  0x2B },
	{ 407,  0xF1 }, // 0x2B
	{ 408,  0xF8 }, // 0x2C
	{ 409,  0x03 }, // 0x2D
	{ 410,  0x13 }, // 0x2E
	{ 411,  0x00 }, // 0x2F
	{ 412,  0x00 }, // 0x30   duty increase
	{ 413,  0x00 }, // 0x31   duty decrease
	{ 414,  0xFF }, // 0x32   falling edge speed control
	{ 354,  0x03 },
	{ 354,  0x00 },


	// Laser Driver ADC auto start
	{ 357,  0x05 }, // send2 channel start by internal sync signal
	{ 359,  0x20 }, // send2 start pointer of Tx buffer
	{ 403,  0x02 }, // send2 total write byte
	{ 404,  0x13 }, // laser address 0x13
	{ 405,  0x23 }, // value write to laser address 0x13
	{ 365,  0x01 }, // internal sync enable for spi master write, start of phase
	{ 366,  0x01 }, // internal sync timing for spi master write

					// Laser Driver temperature auto read
	{ 344,  0x05 }, // thermo channel start by internal sync signal
	{ 346,  0x23 }, // thermo start pointer of Tx buffer
	{ 347,  0x10 }, // thermo start pointer of Rx buffer
	{ 406,  0x03 }, // number of byte to be write and read
	{ 407,  0x94 }, // temperature(TEMP_AD_DATA[9:8][7:0]) address to be read from laser (read address = laser addr 0x14 + 0x80)
	{ 360,  0x01 }, // internal sync enable for spi master read, start of phase
	{ 361,  0xFF }, // internal sync timing for spi master read
	{ 362,  0x20 }, // internal sync timing for spi master read

};
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

	ret = orbbec_i2c_writeread((uint8_t*) &msg, 1);

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

	ret = orbbec_i2c_writeread((uint8_t*) &msg, 1);

	return ret;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
	int rtn = i2c_reg_write(PLECO_I2C_ADDR, reg, 2, value, 1);
	return rtn;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
	int rtn = i2c_reg_read(PLECO_I2C_ADDR, reg, 2, (uint32_t*) value, 1);
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

static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
	int rtn = 0;
#if 0
	//rtn = sensor_write_reg(354, 0x00); // send1 channel, start by register trig
	rtn = sensor_write_reg(356, 0x00); // send1 start pointer of Tx buffer
	rtn = sensor_write_reg(371, 0x02); // number of byte to be write
	rtn = sensor_write_reg(372, reg);  // address to be write to laser (write address = laser addr)
	rtn = sensor_write_reg(373, value);// value to be write to laser
									   //rtn = sensor_write_reg(354, 0x01); // enable
	rtn = sensor_write_reg(354, 0x03); // trig
	rtn = sensor_write_reg(354, 0x00); // disable
#else
	rtn = sensor_write_reg(356, 0x10); // send1 start pointer of Tx buffer
	rtn = sensor_write_reg(387, 0x02); // number of byte to be write
	rtn = sensor_write_reg(388, reg);  // address to be write to laser (write address = laser addr)
	rtn = sensor_write_reg(389, value);// value to be write to laser

	rtn = sensor_write_reg(354, 0x03); // trig
	rtn = sensor_write_reg(354, 0x00); // disable
#endif
	return rtn;
}

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
	int rtn = 0;
	rtn = sensor_write_reg(349, 0x00); // general channel, start by register trig
	rtn = sensor_write_reg(351, 0x00); // start pointer of Tx buffer
	rtn = sensor_write_reg(352, 0x00); // start pointer of Rx buffer
									   //rtn = sensor_write_reg(353, 0x01); // 1st byte is Tx, after Rx
	rtn = sensor_write_reg(371, 0x02); // number of byte to be write and read
	rtn = sensor_write_reg(372, reg + 0x80); // address to be read from laser (read address = laser addr + 0x80)
											 //rtn = sensor_write_reg(349, 0x01); // enable
	rtn = sensor_write_reg(349, 0x03); // trig
	rtn = sensor_write_reg(349, 0x00); // disable

	rtn = sensor_read_reg(419, value); // first byte in Rx buffer

	return rtn;
}

static int vcsel_driver_detect()
{
	int rtn = 0;
	uint32_t value = 0;
	rtn = vcsel_driver_write_reg(0x25, 0x01); // reset
	rtn = vcsel_driver_read_reg(0x0F, (uint8_t*) &value);

	if (value == 0x06) {
		uint8_t phx3d_value = 0;
		rtn = vcsel_driver_read_reg(0x2F, &phx3d_value);
		if (phx3d_value == 0x01) {
			driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
			get_src_info.vcsel_driver_id = DRIVER_IC_PHX3D_3021_CB;
			phx3d_3021_cb_initialize();
		}
	}
	else {
		driver_ic_type = 0;
		get_src_info.vcsel_driver_id = 0;
	}

	return rtn;
}

static int gpio_control(int pin, bool level)
{
	dothin_set_gpio_level(pin, level, dothin_device_id);
	return 0;
}

static int pleco_get_sensor_info(struct sensor_info_t *info) {

	info->embedded_data_size = 1920 * 1.5 * 3;
	info->vcsel_num = 1;
	info->vcsel_driver_id = driver_ic_type;
	info->sensor_id = RK1608_PLECO_SENSOR_ID;
	return 0;
}

int rk1680_pleco_dothin_config()
{
	int ret = 0;

	ALOGD("dothin_device_id=%d\n", dothin_device_id);

	ret = dothin_enable_softpin(true, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_enable_softpin ret=%d\n", ret);
	}
	ret = dothin_enable_gpio(true, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_enable_gpio ret=%d\n", ret);
	}
	ret = dothin_set_sensor_clock(true, 24 * 10, dothin_device_id); // 24Mhz mclk
	if (ret < 0) {
		ALOGE("dothin_set_sensor_clock ret=%d\n", ret);
	}
	ret = dothin_set_softpin_pullup(1, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_set_softpin_pullup ret=%d\n", ret);
	}
	usleep(1000 * 10);
	ret = dothin_set_sensor_i2c_rate(1, dothin_device_id); // 400Khz
	if (ret < 0) {
		ALOGE("dothin_set_sensor_i2c_rate ret=%d\n", ret);
	}
	ret = dothin_sensor_enable(1, true, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 20);
	ret = dothin_sensor_enable(3, true, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

	ret = dothin_sensor_enable(3, true, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

	SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
	int           power_value[] = { 2800,       1800,      1200,        3300,       1200 };
	ret = dothin_pmu_set_voltage((int *)sensor_power, power_value, 5, dothin_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d", ret);
	}
	usleep(1000 * 50);
	ALOGD("end");

	return ret;
}

static int set_vmghi_volt(uint8_t value)
{
	uint16_t VMG_HI = (value << 8);
	int rtn = i2c_reg_write(DAC5574_I2C_ADDR, 0x10, 1, VMG_HI, 2);
	//ALOGE("driver set_vmghi_volt:%d\r\n", value);
	return rtn;
}

int rk1608_pleco_gpio_expander_init() // for power control
{
	int rtn = 0;
	uint8_t set_io = 0;
	set_io = 0x01 | 0x04 | 0x08 | 0x10 | 0x40 | 0x80;
	rtn = i2c_reg_write(0x42, 0x03, 1, 0x00, 1); // set all gpio as output
	rtn |= i2c_reg_write(0x42, 0x01, 1, set_io, 1); 

	set_io = 0x01 | 0x02 | 0x04 | 0x08;
	rtn |= i2c_reg_write(0x40, 0x03, 1, 0x00, 1);
	rtn |= i2c_reg_write(0x40, 0x01, 1, set_io, 1); // set all gpio as output
	rtn = i2c_reg_write(TCA6408A_I2C_ADDR, 0x01, 1, 0xEF, 1);  // pleco新板子
	//rtn |= set_vmghi_volt(20); // 60 ->1.2V, 40 ->1.4V, 20 ->1.6V, 0 ->1.8V
							   //rtn = set_vrampst_volt(162); // 162 -> 0.95V
	return rtn;
}

static int init_config()
{
	int rtn = 0;
	rtn = rk1608_pleco_gpio_expander_init();
	rtn |= rk1680_pleco_dothin_config();

	return rtn;
}

static int rk1608_pleco_get_sensor_id(uint16_t *id)
{
	int rtn = 0;
	uint8_t value = 0;
	static int isConfig = 0;

	if (!isConfig) {
		rtn = init_config();
		if (rtn < 0) {
			ALOGE("rk1608_pleco init_config fail, ret=%d", rtn);
			*id = 0xFFFF;
			return rtn;
		}
		isConfig = 1;
	}
	rtn = sensor_read_reg(7, &value); // TODO: use true sensor id
	if (value == 90) {
		*id = RK1608_PLECO_SENSOR_ID;
	}
	else {
		*id = 0xFFFF;
		ALOGE("rk1608_pleco read id fail, value=%d, ret=%d", value, *id, rtn);
	}

	return rtn;
}


static int rk1608_pleco_get_sensor_info(struct sensor_info_t *info)
{
	memcpy(info, &get_src_info, sizeof(get_src_info));

	return 0;
}

/**
* @brief  rk1608_s5k33dxx_sensor_init
* @return int
*/
int rk1608_pleco_sensor_init()
{
	int rtn = 0;

	for (int i = 0; i < sizeof(rk1608_pleco_reglist) / sizeof(struct regList); i++) {
		rtn = sensor_write_reg(rk1608_pleco_reglist[i].reg, rk1608_pleco_reglist[i].val);
		if (rtn < 0)
			return rtn;
	}

	get_src_info.embedded_data_size = 1280 * 2;//现有rk1608板上33d兼容平台端配置，统一采用EBD packed
	get_src_info.sensor_id = RK1608_PLECO_SENSOR_ID;
	/*
	if (get_src_info.vcsel_driver_id == 0)
		get_src_info.vcsel_driver_id = DRIVER_IC_PHX3D_3021_CB;
	*/
	get_src_info.vcsel_num = 1;
	get_src_info.project_id = 0;

	rtn = vcsel_driver_detect();
	return rtn;
}

 int rk1608_pleoc_set_streaming_type(int mode)
 {
	 ALOGD("==========>");
	 return -1;
 }

 int rk1608_pleco_download_ref(uint32_t rk1608_store_addr, uint32_t *ref_data_addr, uint32_t ref_data_len)
 {
	 ALOGD("==========>");
	 return -1;
 }

 int rk1608_pleco_get_ref_buffer_addr(uint32_t *rk1608_store_addr)
 {
	 ALOGD("==========>");
	 return -1;
 }

 int rk1608_pleco_hardware_trigger()
 {
	 ALOGD("==========>");
	 return -1;
 }

int rk1608_pleco_software_trigger()
{
	ALOGD("==========>");
	return -1;
}

int rfk1608_pleco_group_hold(bool enable)
{
	int rtn = 0;
	if (enable)
		rtn = sensor_write_reg(16, 0x01);
	else
		rtn = sensor_write_reg(16, 0x00);

	return rtn;
}

int rk1608_pleco_video_streaming(bool enable)
{
    //ALOGD("rk1608_pleco_video_streaming==========> enable=%d", enable);
	int rtn = 0;
#if 1
	if (enable) {/**/
		rtn = rfk1608_pleco_group_hold(true);
		rtn = sensor_write_reg(31, (10922 % 256)); // frame number should set to 10922 in DMFD (21845 in SMFD) streaming mode
		rtn = sensor_write_reg(32, (10922 / 256));
		rtn = rfk1608_pleco_group_hold(false);
		rtn = sensor_write_reg(1, 0x01); // trigger
		rtn = sensor_write_reg(1, 0x00);
	}
	else {/**/
		rtn = rfk1608_pleco_group_hold(true);
		//rtn = sensor_write_reg(2, 1);//  deep sleep enable
		rtn = sensor_write_reg(31, 0x00); // frame number should set to 0 to stop streaming
		rtn = sensor_write_reg(32, 0x00);
		rtn = rfk1608_pleco_group_hold(false);
		rtn = sensor_write_reg(17, 252); // reset sequencer
		rtn = sensor_write_reg(17, 253);
	}

#else
	if (enable) {
		rtn = sensor_write_reg(1, 0x01); // trigger
		rtn = sensor_write_reg(1, 0x00);
	}
	else {
		rtn = sensor_write_reg(17, 252); // reset sequencer
		rtn = sensor_write_reg(17, 253);
	}
#endif

	return rtn;
}

int rk1608_pleco_get_fps(uint8_t *fps)
{
	int rtn = 0;
	uint8_t pixel_clock = 10; // uint: ns, 100MHz
	uint8_t valueL, valueH, valueHH;
	rtn = sensor_read_reg(26, &valueL);
	rtn = sensor_read_reg(27, &valueH);
	uint16_t line_length = (valueH << 8) + valueL;
	rtn = sensor_read_reg(28, &valueL);
	rtn = sensor_read_reg(29, &valueH);
	rtn = sensor_read_reg(30, &valueHH);
	uint32_t subframe_length = (valueHH << 16) + (valueH << 8) + valueL;
	uint8_t freq_mode, subframe_num;
	pleco_get_frequency_mode(&freq_mode);
	if (DUAL_FREQ == freq_mode || AF_FREQ == freq_mode)
		subframe_num = 6;
	else if (SINGLE_FREQ == freq_mode)
		subframe_num = 3;
	unsigned int frame_length = subframe_length * subframe_num; // 6 subframe in dual freq mode
	*fps = 1000000000 / (frame_length*line_length*pixel_clock);

	return 0;
}

int rk1608_pleco_set_fps(uint8_t fps)
{
	int rtn = 0;
	uint8_t freq_mode, subframe_num;
	rtn = pleco_get_frequency_mode(&freq_mode);
	if (DUAL_FREQ == freq_mode || AF_FREQ == freq_mode) {
		subframe_num = 6;
		if (fps == 0 || fps > 30)
			return -HW_ERR_INVALID;
	}
	else if (SINGLE_FREQ == freq_mode) {
		subframe_num = 3;
		if (fps == 0 || fps > 60)
			return -HW_ERR_INVALID;
	}

	// frame_time = line_length*frame_length*pixel_clock
	uint32_t frame_time = 1000000000 / fps; // uint: ns
	uint8_t pixel_clock = 10; // uint: ns, 100MHz
	uint8_t valueL, valueH;
	rtn = sensor_read_reg(26, &valueL);
	rtn = sensor_read_reg(27, &valueH);
	uint16_t line_length = (valueH << 8) + valueL;
	uint32_t frame_length = frame_time / (line_length * pixel_clock);
	uint32_t subframe_length = frame_length / subframe_num; // 6 subframe in dual freq mode
															//printf("subframe_length: %d", subframe_length);
	if (subframe_length < 2100)
		return -HW_ERR_INVALID;
	unsigned char valueHH = subframe_length / 65535;
	uint16_t valueHL = subframe_length % 65535;
	valueL = valueHL % 256;
	valueH = valueHL / 256;
	rtn = pleco_group_hold(true);
	rtn = sensor_write_reg(28, valueL);
	rtn = sensor_write_reg(29, valueH);
	rtn = sensor_write_reg(30, valueHH);
	rtn = pleco_group_hold(false);
	return 0;
}

int rk1608_pleco_get_rx_temp(float *temperature)
{
	int rtn = 0;
	uint8_t valueH, valueL;
	rtn = sensor_read_reg(451, &valueL);
	rtn = sensor_read_reg(452, &valueH);

	uint16_t value = (valueH << 8) | valueL;
	*temperature = value*0.16f - 272;
	//printf("IIC sensor_temp %d %d %d \r\n", valueL, valueH, *temperature);
	return rtn;
}

int rk1608_pleco_get_tx_temp(float *temperature)
{
	int rtn = 0;
	uint8_t valueH, valueL;
	rtn = sensor_read_reg(419 + 0x10, &valueH); // 0x10's byte in Rx buffer
	rtn = sensor_read_reg(420 + 0x10, &valueL);

	uint32_t value = ((valueH & 0x03) << 8) + valueL;
	*temperature = 25 + ((value - 296)) / 5.4f;
	//printf("value %d,  valueH %d, valueL %d    tx=%f  \n", value, valueH, valueL, *temperature);
	return rtn;
}

int rk1608_pleco_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
	rtn = vcsel_driver_write_reg(0x07, value_B);// IBIAS
	rtn = vcsel_driver_write_reg(0x08, value_A);// ISW

	return rtn;
}

int rk1608_pleco_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
	*vcsel_num = 1;
	rtn = vcsel_driver_read_reg(0x07, value_B);
	rtn = vcsel_driver_read_reg(0x08, value_A);

	return rtn;
}

int rk1608_pleco_illum_power_control(bool enable)
{
	ALOGD("==========>");
	return -1;
}

int rk1608_pleco_get_integration_time(uint16_t *integrationTime)
{
	//ALOGD("==========>");
	int rtn = 0;
	uint8_t valueL, valueH;
	rtn = sensor_read_reg(526, &valueL); // F0
	rtn = sensor_read_reg(527, &valueH);
	*integrationTime = (valueH << 8) + valueL;

	return 0;
}

int rk1608_pleco_set_integration_time(uint16_t integrationTime)
{
	int rtn = 0;
	if (integrationTime > 4095)
		return -HW_ERR_INVALID;
	unsigned char valueL, valueH;
	valueL = (uint8_t)(integrationTime % 256);
	valueH = (uint8_t)(integrationTime / 256);
	rtn = pleco_group_hold(true);
	rtn = sensor_write_reg(526, valueL); // F0
	rtn = sensor_write_reg(527, valueH);

	rtn = sensor_write_reg(528, valueL); // F1
	rtn = sensor_write_reg(529, valueH);
	rtn = pleco_group_hold(false);
	return 0;
}

int rk1608_pleco_get_modulation_frequency(uint16_t *modFreq)
{
	int rtn = 0;
	uint8_t freq_0, freq_1;
	rtn = sensor_read_reg(567, &freq_0);
	rtn = sensor_read_reg(568, &freq_1);
	*modFreq = ((freq_0 + 3) << 8) + (freq_1 + 3);

	return rtn;
}

int rk1608_pleco_set_modulation_frequency(uint16_t modFreq)
{
	int rtn = 0;
	uint8_t freq_0, freq_1;
	freq_0 = (modFreq >> 8) & 0xFF;
	freq_1 = modFreq & 0xFF;
	//printf("pleco_set_modulation_frequency: %d,  %d\r\n", freq_0, freq_1);
	static uint16_t npulse[] = { 348, 264, 216, 180, 159, 141, 126 ,114 };// should be divided by 3
	if (freq_0 >= 3 && freq_0 <= 10 && freq_0 >= 3 && freq_1 <= 10) {
		uint8_t valueH, valueL;
		rtn = pleco_group_hold(true);

		//uint16_t f0_npulse = (1000 / freq_0) + 15;// turn out to be 1us
		uint16_t f0_npulse = npulse[freq_0 - 3];
		valueL = f0_npulse & 0xFF;
		valueH = (f0_npulse >> 8) & 0xFF;
		rtn = sensor_write_reg(537, valueL);
		rtn = sensor_write_reg(538, valueH);

		//uint16_t f1_npulse = (1000 / freq_1) + 15;// turn out to be 1us
		uint16_t f1_npulse = npulse[freq_1 - 3];
		valueL = f1_npulse & 0xFF;
		valueH = (f1_npulse >> 8) & 0xFF;
		rtn = sensor_write_reg(541, valueL);
		rtn = sensor_write_reg(542, valueH);

		rtn = sensor_write_reg(567, (freq_0 - 3));
		rtn = sensor_write_reg(568, (freq_1 - 3));
		rtn = pleco_group_hold(false);
		//printf("f0_npulse: %d, f1_npulse: %d\r\n", f0_npulse, f1_npulse);
	}
	else {
		return -HW_ERR_INVALID;
	}
	return rtn;
}

int rk1608_pleco_get_illum_duty_cycle(uint16_t *duty)
{
	int rtn = 0;
	uint8_t f0_duty = 0;
	uint8_t f1_duty = 0;
	uint8_t value = 0;
	rtn = sensor_read_reg(547, &value);
	if (rtn < 0) {
		return rtn;
	}
	if (value == 0) // no change
		f0_duty = 7;
	else if ((value >> 3) > 0) {
		f0_duty = 7 + (value >> 3); // increased duty
	}
	else if ((value & 0x07) > 0) {
		f0_duty = 7 - (value & 0x07); // decreased duty
	}
	value = 0;
	rtn = sensor_read_reg(551, &value);
	if (value == 0) // no change
		f1_duty = 7;
	else if ((value >> 3) > 0) {
		f1_duty = 7 + (value >> 3); // increased duty
	}
	else if ((value & 0x07) > 0) {
		f1_duty = 7 - (value & 0x07); // decreased duty
	}
	*duty = (f0_duty << 8) | f1_duty;

	return rtn;
}

int rk1608_pleco_set_illum_duty_cycle(uint16_t duty)
{
	uint8_t f0_duty = (duty >> 8) & 0xFF;
	uint8_t f1_duty = duty & 0xFF;
	if (f0_duty > 14 || f1_duty > 14)
		return -HW_ERR_INVALID;

	int rtn = 0;
	uint8_t value = 0;
	rtn = rfk1608_pleco_group_hold(true);
	if (f0_duty == 7) { // no change
		rtn = sensor_write_reg(547, 0); // freq_0
	}
	else if (f0_duty > 7) { // increase duty
		value = f0_duty - 7;
		rtn = sensor_write_reg(547, (value << 3));
	}
	else if (f0_duty < 7) { // decrease duty
		value = 7 - f0_duty;
		rtn = sensor_write_reg(547, value);
	}
	value = 0;
	if (f1_duty == 7) { // no change
		rtn = sensor_write_reg(551, 0); // freq_1
	}
	else if (f1_duty > 7) { // increase duty
		value = f1_duty - 7;
		rtn = sensor_write_reg(551, (value << 3));
	}
	else if (f1_duty < 7) { // decrease duty
		value = 7 - f1_duty;
		rtn = sensor_write_reg(551, value);
	}
	rtn = rfk1608_pleco_group_hold(false);
	return rtn;
}

int rk1608_pleco_get_data_output_mode(uint8_t *mode)
{
	int rtn = 0;
	uint8_t value = 0;
	rtn = sensor_read_reg(21, &value);
	*mode = (value >> 6);

	return rtn;
}

int rk1608_pleco_set_data_output_mode(uint8_t mode)
{
	int rtn = 0;
	if (mode > 3)
		return -HW_ERR_INVALID;
	unsigned char value = 0;
	rtn = sensor_read_reg(21, &value);
	value = (value & 0x3F) | (mode << 6);
	rtn = sensor_write_reg(21, value);
	ALOGE("pleco_set_data_output_mode: %d\r\n", mode);
	return rtn;
}

int rk1608_pleco_get_img_mirror_flip(uint8_t *mode)
{
	int rtn;
	uint8_t value;
	rtn = sensor_read_reg(18, &value);
	uint8_t mode_mask[] = { 2, 3, 0, 1 };
	value = (value >> 5) & 0x03;
	*mode = mode_mask[value];

	return rtn;
}

int rk1608_pleco_set_img_mirror_flip(uint8_t mode)
{
	int rtn;
	uint8_t value;
	rtn = sensor_read_reg(18, &value);
	rtn = pleco_group_hold(true);
	switch (mode)
	{
	case IMAGE_NORMAL: // V = 1, H = 0
		value &= ~(3 << 5);
		value |= (2 << 5);
		rtn = sensor_write_reg(18, value);
		break;
	case IMAGE_H_MIRROR: // V = 1, H = 1
		value &= ~(3 << 5);
		value |= (3 << 5);
		rtn = sensor_write_reg(18, value);
		break;
	case IMAGE_V_MIRROR: // V = 0, H = 0
		value &= ~(3 << 5);
		rtn = sensor_write_reg(18, value);
		break;
	case IMAGE_HV_MIRROR: // V = 0, H = 1
		value &= ~(3 << 5);
		value |= (1 << 5);
		rtn = sensor_write_reg(18, value);
		break;
	default:
		rtn = -HW_ERR_INVALID;
		break;
	}
	rtn = pleco_group_hold(false);
	//ALOGE("pleco_set_img_mirror_flip: %d, mode: %d \r\n", value, mode);
	return rtn;
}

int rk1608_pleco_test_pattern(uint8_t mode)
{
	ALOGD("==========>");
	return -1;
}

int rk1608_pleco_AE(bool enable)
{
	ALOGD("==========>");
	return -1;
}

int rk1608_pleco_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
	//ALOGD("==========>");
	float duty = 0;
	for (int step = -7; step <= 7; step++) {
		duty = step*0.2; // 0.2ns per step

		duty_cycle_list[step + 7] = duty;
	}

	return 0;
}

int rk1608_pleco_get_pixel_binning(uint8_t *mode)
{
	ALOGD("==========>");
	return -1;
}

int rk1608_pleco_set_pixel_binning(uint8_t mode)
{
	ALOGD("==========>");
    return -1;
}

int rk1608_pleco_get_antijam(uint8_t *status)
{
	ALOGD("==========>");
	int rtn = 0;
	uint8_t value;
	rtn = sensor_read_reg(569, &value);
	*status = value & 0x02;

	return 0;
}

int rk1608_pleco_set_antijam(uint8_t status)
{
	ALOGD("==========>");
	int rtn = 0;
	uint8_t value;
	rtn = rfk1608_pleco_group_hold(true);
	rtn = sensor_read_reg(569, &value);
	if (status)
		value |= (1 << 1);
	else
		value &= ~(1 << 1);
	rtn = sensor_write_reg(569, value);
	rtn = rfk1608_pleco_group_hold(false);
	return 0;
}

int rk1608_pleco_set_frequency_mode(uint8_t mode)
{
	if (mode > 2)
		return -HW_ERR_INVALID;
	int rtn;
	ALOGE("pleco_set_frequency_mode: %d\r\n", mode);
	rtn = rfk1608_pleco_group_hold(true);
	if (mode == SINGLE_FREQ) { // SMFD
		rtn = sensor_write_reg(21, 64);
		rtn = sensor_write_reg(31, (21845 % 256)); // frame number should set to 10922 in DMFD (21845 in SMFD) streaming mode
		rtn = sensor_write_reg(32, (21845 / 256));
		// 1450, rd_line_max, 3 subframe
		rtn = sensor_write_reg(702, 170);
		rtn = sensor_write_reg(703, 5);
		rtn = sensor_write_reg(700, 0);  // AF disable
		pleco_set_fps(30);
	}
	else if (mode == DUAL_FREQ) { // DMFD
		rtn = sensor_write_reg(21, 0);
		rtn = sensor_write_reg(31, (10922 % 256)); // frame number should set to 10922 in DMFD (21845 in SMFD) streaming mode
		rtn = sensor_write_reg(32, (10922 / 256));
		// 2902, rd_line_max, 6 subframe
		rtn = sensor_write_reg(702, 86);
		rtn = sensor_write_reg(703, 11);
		rtn = sensor_write_reg(700, 0);  // AF disable
		pleco_set_fps(30);
	}
	else if (mode == AF_FREQ) { // AF
		rtn = sensor_write_reg(21, 192);
		rtn = sensor_write_reg(31, 170);
		rtn = sensor_write_reg(32, 42);
		rtn = sensor_write_reg(702, 86);
		rtn = sensor_write_reg(703, 11);
		rtn = sensor_write_reg(700, 1);  // AF enable
		rtn = sensor_write_reg(701, 1);
		pleco_set_fps(10);
	}
	rtn = rfk1608_pleco_group_hold(false);

	return rtn;
}

int rk1608_pleco_get_frequency_mode(uint8_t *mode)
{
	uint8_t value;
	int rtn = sensor_read_reg(21, &value);
	if (value == 64) // SMFD
		*mode = SINGLE_FREQ;
	else if (value == 0) // DMFD
		*mode = DUAL_FREQ;
	else if (value == 192) // AF
		*mode = AF_FREQ;

	return 0;
}

static int get_vmghi_volt(uint8_t *value)
{
	uint16_t VMG_HI;
	int rtn = i2c_reg_read(DAC5574_I2C_ADDR, 0x10, 1, (uint32_t *)&VMG_HI, 2);
	*value = (VMG_HI >> 8) & 0xFF;
	//ALOGE("driver get_vmghi_volt:%d\r\n", *value);
	return rtn;
}

int rk1608_pleco_get_af_depth(uint16_t *value)
{
	//ALOGD("==========>");
	int rtn = 0;
	rtn = sensor_write_reg(776, 0);
	rtn = sensor_write_reg(776, 1);

	uint8_t value_H = 0, value_L = 0;
	rtn = sensor_read_reg(844, &value_L);
	rtn = sensor_read_reg(845, &value_H);
	*value = value_L + (value_H * 256);
	*value &= 0x1fff;  // low 13 bit valid
	return rtn;
}

void rk1608_pleco_set_phase_src_info()
{
	get_src_info.embedded_data_size = 1280 * 2;
}

void rk1608_pleco_set_depth_src_info()
{
	//get_src_info.embedded_data_size = 640 * 2 * 2;
	get_src_info.embedded_data_size = 1280 * 2;
}

int rk1608_pleco_get_firmware(uint8_t fw_type, const uint8_t **get_fireware_data)
{
	FILE *fp = NULL;
	int file_len = 0;
	char *file_name = NULL;
	const uint8_t * fireware_data = NULL;

	if (fw_type == BOOT_FROM_DDR)
		file_name = ".\\config\\firmware\\donghai\\preisp_pleco.rkl";
	else if (fw_type == BOOT_FROM_FLASH)
		file_name = ".\\config\\firmware\\donghai\\preisp_spi_boot.bin";
	else
		return -HW_ERR_NO_SUPPORT;

	//判断文件是否打开失败
	if ((fp = fopen(file_name, "rb")) == NULL) {
		ALOGE("%s:open firmware file fail", __FUNCTION__);
		return -HW_ERR_INVALID;
	}

	fseek(fp, 0, SEEK_END); //定位到文件末 
	if ((file_len = ftell(fp)) < 1)//文件长度
	{
		ALOGE("%s:firmware file length error", __FUNCTION__);
		fclose(fp);
		return -HW_ERR_INVALID;
	}
	fseek(fp, 0, SEEK_SET); //定位到文件头
	fireware_data = (uint8_t*)malloc(sizeof(char)*(file_len + 1));
	if (NULL == fireware_data)
	{
		ALOGE("%s:get firmware memory fail", __FUNCTION__);
		fclose(fp);
		return -HW_ERR_NO_MEM;
	}
	memset((void*)fireware_data, 0, file_len + 1);
	fread((void*)fireware_data, file_len + 1, 1, fp);
	*get_fireware_data = fireware_data;

	fclose(fp);
	return HW_NO_ERR;
}

int cmd_to_set_ae_param_test()
{
    AEParam ae_param;

    ae_param.fov = 0.9f;
    ae_param.t_min = 500;
    ae_param.t_max = 20;
    ae_param.t_step_ratio_min = 0.1f;
    ae_param.t_step_ratio_max = 0.3f;
    ae_param.over_exposure_value = 1000;
    ae_param.over_dark_value = 600;
    ae_param.ratio_thresh = 0.005f;
    ae_param.ir_thresh = 500.0f;

    cmd_msg_set_ae_param(0, &ae_param);
}


/*
*@brief:  发送数据到rk1608（dsp）,配置相应参数和算法,让raw phase输出.
@param[in] m：发送的数据结构体
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int set_rk1608_pleco_raw_phase_out()
{
	int8_t cam_id = 0;
	int8_t in_mipi = 0;
	int8_t out_mipi = 0;

	uint16_t in_embed_width = 1280; // 1280 Byte
	uint16_t in_embed_height = 12;
	uint16_t in_pixel_width = 1920;
	uint16_t in_pixel_height = 2880;
	uint16_t out_width = 1280;
	uint16_t out_height = 5882;
	uint16_t out_htotal = 2000;
	uint16_t out_vtotal = 6500;

	uint32_t mipi_clock = 1500000000;
	char* sensor_name = "sc132gs";
	uint8_t i2c_slave_addr = 0x20;
	uint8_t i2c_bus = 0;
	uint8_t master_slave_mode = MASTER;

	uint8_t stream_in_type = DUAL_FREQ_SHUFFLE;
	uint8_t sync_freq = 30;
	uint8_t stream_out_type = CAM_PHASE_AND_DEPTH_STREAM;
	//uint8_t skip_frame = 10;
	uint8_t skip_frame = 0;
	uint32_t init_enable_param = 1;
	int ret = -1;

	ret = cmd_msg_init_sensor(cam_id, in_mipi, out_mipi, sensor_name, i2c_slave_addr, i2c_bus);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_init_sensor fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	usleep(20000);
	ret = cmd_msg_set_pleco_input_size(cam_id, in_embed_width, in_embed_height, in_pixel_width, in_pixel_height);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_input_size fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = cmd_msg_set_output_size(cam_id, out_width, out_height, out_htotal, out_vtotal, mipi_clock);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_output_size fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = cmd_msg_set_stream_in_mode(cam_id, master_slave_mode, stream_in_type, sync_freq, skip_frame);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_in_mode fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = cmd_msg_set_stream_out_type(cam_id, stream_out_type);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_out_type fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

#if 1
	ret = cmd_msg_set_param_init(cam_id, init_enable_param);
	if (ret < 0)
	{
		return -1;
	}

    //cmd_to_set_ae_param_test();
#endif

	ret = cmd_msg_set_stream_in_on(cam_id);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_in_on fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	usleep(30000);
	ret = cmd_msg_set_stream_out_on(cam_id);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_out_on fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	return HW_NO_ERR;
}

int set_rk1608_pleco_depth_out()
{
	return set_rk1608_pleco_raw_phase_out();
}

int rk1608_pleco_get_tof_sensor_resolution(uint64_t *resolution)
{
    int ret;
    uint8_t mode = 0;
    ret = rk1608_pleco_get_frequency_mode(&mode);
    *resolution = 0;
    if (ret == 0)
    {
        if (mode == DUAL_FREQ)
        {
            *resolution = DUAL_FREQ_4X4_BINNING_RESOLUTION << 16 | DUAL_FREQ_2X2_BINNING_RESOLUTION << 8 | DUAL_FREQ_NO_BINNING_RESOLUTION;
        }
        else
        {
            if (mode == SINGLE_FREQ)
            {
                *resolution = SINGLE_FREQ_4X4_BINNING_RESOLUTION << 16 | SINGLE_FREQ_2X2_BINNING_RESOLUTION << 8 | SINGLE_FREQ_NO_BINNING_RESOLUTION;
            }
        }
    }
    return ret;
}

int rk1608_pleco_get_tof_sensor_pixel_bit(uint8_t *pixel_bit)
{
    *pixel_bit = PIXEL_BIT;
    return 0;
}

int rk1608_pleco_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
    *mipi_pack_bit = MIPI_PACK_BIT;
    return 0;
}


/*
*@brief:  初始化rk1608的板上pleoc 函数接口,对一些接口进行覆盖，和sensor处理解耦合
*@return   0          成功
*/
int rk1608_pleco_func_load()
{
    tof_sensor.init = rk1608_pleco_sensor_init;
    tof_sensor.get_sensor_id = rk1608_pleco_get_sensor_id;
    //tof_sensor.hardware_trigger = rk1608_pleco_hardware_trigger;
    //tof_sensor.software_trigger = rk1608_pleco_software_trigger;
    tof_sensor.video_streaming = rk1608_pleco_video_streaming;
    tof_sensor.get_fps = rk1608_pleco_get_fps;
    tof_sensor.set_fps = rk1608_pleco_set_fps;
    tof_sensor.get_rx_temp = rk1608_pleco_get_rx_temp;
    tof_sensor.get_tx_temp = rk1608_pleco_get_tx_temp;
    tof_sensor.set_illum_power = rk1608_pleco_set_illum_power;
    tof_sensor.get_illum_power = rk1608_pleco_get_illum_power;
    //tof_sensor.illum_power_control = rk1608_pleco_illum_power_control;
    tof_sensor.get_integration_time = rk1608_pleco_get_integration_time;
    tof_sensor.set_integration_time = rk1608_pleco_set_integration_time;
    tof_sensor.get_modulation_frequency = rk1608_pleco_get_modulation_frequency;
    tof_sensor.set_modulation_frequency = rk1608_pleco_set_modulation_frequency;
    tof_sensor.get_illum_duty_cycle = rk1608_pleco_get_illum_duty_cycle;
    tof_sensor.set_illum_duty_cycle = rk1608_pleco_set_illum_duty_cycle;
    tof_sensor.get_data_output_mode = rk1608_pleco_get_data_output_mode;
    tof_sensor.set_data_output_mode = rk1608_pleco_set_data_output_mode;
    tof_sensor.get_img_mirror_flip = rk1608_pleco_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = rk1608_pleco_set_img_mirror_flip;
    //tof_sensor.get_pixel_binning = rk1608_pleco_get_pixel_binning;
    //tof_sensor.set_pixel_binning = rk1608_pleco_set_pixel_binning;
    //tof_sensor.test_pattern = rk1608_pleco_test_pattern;
    tof_sensor.get_sensor_info = rk1608_pleco_get_sensor_info;
    tof_sensor.get_illum_duty_cycle_list = rk1608_pleco_get_illum_duty_cycle_list;
    //tof_sensor.get_antijam = rk1608_pleco_get_antijam;
    //tof_sensor.set_antijam = rk1608_pleco_set_antijam;
    tof_sensor.sensor_write_reg_8 = sensor_write_reg;
    tof_sensor.sensor_read_reg_8 = sensor_read_reg;
    tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;
    tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;
    tof_sensor.get_frequency_mode = rk1608_pleco_get_frequency_mode;
    tof_sensor.set_frequency_mode = rk1608_pleco_set_frequency_mode;
    tof_sensor.get_vmghi_voltage = get_vmghi_volt;
    tof_sensor.set_vmghi_voltage = set_vmghi_volt;
    //tof_sensor.get_af_depth = rk1608_pleco_get_af_depth;

    tof_sensor.get_tof_sensor_resolution = rk1608_pleco_get_tof_sensor_resolution;
    tof_sensor.get_tof_sensor_pixel_bit = rk1608_pleco_get_tof_sensor_pixel_bit;
    tof_sensor.get_mipi_pack_bit = rk1608_pleco_get_mipi_pack_bit;

    return 0;
}