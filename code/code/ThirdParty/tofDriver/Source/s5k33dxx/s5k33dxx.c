#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "s5k33dxx.h"
#include "s5k33d_setting\s5k33d_config.h"
#include "stmpe801.h"
#include <tof_sensors.h>
#include "rk1608.h"
#include "debug2log.h"
#include "hw_property.h"

#include <obc_tee_funcs.h>
#include "hw_obstatus.h"
#include "project_config.h"
#include "s5k33d_setting/s5k33d_cx3.h"
#include "s5k33d_setting/s5k33d_taishan.h"
#include "s5k33d_setting/s5k33d_huangshan.h"
#include "s5k33d_setting/s5k33d_t200515.h"

#ifdef __linux__
#include <unistd.h>
#endif

//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
//#define ALOGE(...)
//#define TEE_LOG_LEVEL_ERROR        8
//#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

//#define X210116

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread

#define s5k33dxx_device_id           tops_t.ap_ops.device_id

#define I2C_M_RD     1
#define I2C_M_WT     0

uint16_t s5k33d_project_id = 0;
AA_QC_INDEX s5k33d_aa_qc_index = UNUSED_INDEX;
uint8_t start_streaming_called = 0;
static uint8_t start_stream_count = 0;  //when value = 0,be allowed update driver ic setting. others value be not allowed.
static uint16_t duty_cycle = 15;
uint16_t driver_ic_type = 0;
uint16_t set_fps_called = 0;
// vcsel driver IC type
#define DRIVER_IC_CXA4026              4026 // Polaris B Tx
#define DRIVER_IC_CXA4046              4046
#define DRIVER_IC_CXA4016              4016 // Taishan DVT1 Tx
#define DRIVER_IC_PHX3D_3021_AA        5016 // Taishan DVT2 Tx
#define DRIVER_IC_PHX3D_3021_CB        5017 // Taishan DVT3 Tx
#define OBC_DRIVER_IC_PHX3D_3018       3018 // F201201
#define DRIVER_IC_DW9912               9912 // DongWoon

#define DUAL_FREQ_NO_BINNING_RESOLUTION         RESOLUTION_1280_3840
#define DUAL_FREQ_2X2_BINNING_RESOLUTION        RESOLUTION_640_1920
#define DUAL_FREQ_4X4_BINNING_RESOLUTION        RESOLUTION_320_960

#define SINGLE_FREQ_NO_BINNING_RESOLUTION       0
#define SINGLE_FREQ_2X2_BINNING_RESOLUTION      0
#define SINGLE_FREQ_4X4_BINNING_RESOLUTION      0

#define DUAL_FREQ      1
#define SINGLE_FREQ    0
#define AF_FREQ        2

#define PIXEL_BIT       10
#define MIPI_PACK_BIT   10

struct regList {
	uint16_t reg;
	uint16_t val;
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
static int isConfig = 0;

#if (USE_WHICH_CONVERTER == kConverterIsCx3)
char *test_name = "use_c2x3";

#define s5k33dxx_mipi_cfg            tops_t.ap_ops.SetMipiConfiguration
#define s5k33dxx_set_gpio_dir        tops_t.ap_ops.SetGpioPinDir
#define s5k33dxx_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel

/* warning :T200515_CX3_TOP_V1.0 board in fact is different,in which
1.SENSOR_VDDIO_1V8_GPIO is control by Gpio 21
2.CXA4016_VDDIO_1V8_GPIO needn't control
3.Gpio 18 haven't used
*/
#define SENSOR_RESET_GPIO          0xFF
#define SENSOR_VDDIO_1V8_GPIO      18
#define SENSOR_VDDA_2V8_GPIO       19
#define CXA4016_VCC_3V3_GPIO       20
#define CXA4016_VDDIO_1V8_GPIO     21
#define CXA4016_3V6_GPIO           22



static MipiConfiguration s5k33d_mipi_cfg = {
	.data_format = 0x26,
	.num_datalanes = 2,
	.pll_prd = 2,
	.pll_fbd = 119,
	.pll_frs = 1,
	.csi_rx_clk_div = 1,
	.par_clk_div = 1,
	.mclk_ctl = 0,
	.mclk_ref_div = 2,
	.hresolution = 1280,
	.fifo_delay = 50,
	.pll_clock = 384,
	.mclk = 24,
};

#elif (USE_WHICH_CONVERTER == kConverterIsDuxin)
char *test_name = "use_duxin";
uint8_t flag = 0;

#define B_AREA_CTRL_PIN_OB          2 // PO1, Polaris B board
#define GPIO_EXPANDER_RST_PIN       3 // PO3
#define GPIO_EXPANDER_ADDR          (0x20 << 1) // TCA6408A

#define s5k33dxx_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage
#define s5k33dxx_pmu_set_onoff       tops_t.ap_ops.PmuSetOnOff
#define s5k33dxx_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define s5k33dxx_enable_gpio         tops_t.ap_ops.EnableGpio
#define s5k33dxx_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define s5k33dxx_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define s5k33dxx_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define s5k33dxx_sensor_enable       tops_t.ap_ops.SensorEnable

#define dothin_device_id             tops_t.ap_ops.device_id
#define dothin_set_gpio_level        tops_t.ap_ops.SetGpioPinLevel
#endif

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

static int sensor_write_reg_8(uint16_t reg, uint8_t value)
{
	int ret;

	if (s5k33d_project_id == S5K33D_T200515 || s5k33d_project_id == S5K33D_F201201)
	{
		ret = i2c_reg_write(S5K33D_I2C_ADDR_H, reg, 2, value, 1);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN || s5k33d_project_id == S5K33D_TAISHAN || s5k33d_project_id == S5K33D_X210116)
	{
		ret = i2c_reg_write(S5K33D_I2C_ADDR_L, reg, 2, value, 1);
	}
	else
	{
		ret = HW_ERR_NO_SUPPORT;

		ALOGE("%s I2C addrees not matching", __FUNCTION__);
	}

	return ret;
}

static int sensor_read_reg_8(uint16_t reg, uint8_t *value)
{
	int ret;

	if (s5k33d_project_id == S5K33D_T200515 || s5k33d_project_id == S5K33D_F201201)
	{
		ret = i2c_reg_read(S5K33D_I2C_ADDR_H, reg, 2, value, 1);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN || s5k33d_project_id == S5K33D_TAISHAN || s5k33d_project_id == S5K33D_X210116)
	{
		ret = i2c_reg_read(S5K33D_I2C_ADDR_L, reg, 2, value, 1);
	}
	else
	{
		ret = HW_ERR_NO_SUPPORT;

		ALOGE("%s I2C addrees not matching", __FUNCTION__);
	}

	return ret;
}

static int sensor_write_reg_16(uint16_t reg, uint16_t value)
{
	int ret;
	if (reg == 0xffff)
	{
		usleep(value);
		return 0;
	}

	if (s5k33d_project_id == S5K33D_T200515 || s5k33d_project_id == S5K33D_F201201)
	{
		ret = i2c_reg_write(S5K33D_I2C_ADDR_H, reg, 2, value, 2);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN || s5k33d_project_id == S5K33D_TAISHAN || s5k33d_project_id == S5K33D_X210116)
	{
		ret = i2c_reg_write(S5K33D_I2C_ADDR_L, reg, 2, value, 2);
	}
	else
	{
		ret = HW_ERR_NO_SUPPORT;

		ALOGE("%s I2C addrees not matching", __FUNCTION__);
	}

	return ret;
}

static int sensor_read_reg_16(uint16_t reg, uint16_t *value)
{
	int ret;

	if (s5k33d_project_id == S5K33D_T200515 || s5k33d_project_id == S5K33D_F201201)
	{
		ret = i2c_reg_read(S5K33D_I2C_ADDR_H, reg, 2, value, 2);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN || s5k33d_project_id == S5K33D_TAISHAN || s5k33d_project_id == S5K33D_X210116)
	{
		ret = i2c_reg_read(S5K33D_I2C_ADDR_L, reg, 2, value, 2);
	}
	else
	{
		ret = HW_ERR_NO_SUPPORT;

		ALOGE("%s I2C addrees not matching", __FUNCTION__);
	}

	return ret;
}

static int cxa4016_reg_write(uint16_t reg, uint8_t data)
{
	int ret;

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, reg);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, data);

	return ret;
}

static int cxa4016_reg_read(uint16_t reg, uint8_t *data)
{
	int ret;

	ret = sensor_write_reg_16(0x602C, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602E, reg);
	if (ret < 0)
		return ret;
	ret = sensor_read_reg_8(0x6F12, data);

	return ret;
}

static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
	int ret;

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFE);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, reg);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B06);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, value);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFD);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x01);

	if (start_streaming_called == 0) {
		ret = s5k33dxx_video_streaming(true);  //start stream

		usleep(100000);  //need wait next frame,the setting be written into the driver ic.

		ret = s5k33dxx_video_streaming(false);  //stop stream
	}
	else {
		usleep(100000);  //need wait next frame,the setting be written into the driver ic.
	}

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFD);
	if (ret < 0)
		return ret;
		ret = sensor_write_reg_8(0x6F12, 0x00);

		return ret;
}

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
	int ret;

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B10);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, reg);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B0F);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x01);

	if (start_streaming_called == 0) {
		ret = s5k33dxx_video_streaming(true);  //start stream

		usleep(100000);  //need wait next frame,the setting be written into the driver ic.

		ret = s5k33dxx_video_streaming(false);  //stop stream
	}
	else {
		usleep(100000);  //need wait next frame,the setting be written into the driver ic.
	}

	ret = sensor_write_reg_16(0x602C, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602E, 0x4B60);
	if (ret < 0)
		return ret;
	ret = sensor_read_reg_8(0x6F12, value);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B0F);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x00);

	return ret;
}

static int vcsel_driver_write_reg_16(uint16_t reg, uint16_t value)
{
	int ret;

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFE);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x6F12, reg);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B06);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x6F12, value);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFD);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x02);

	if (start_streaming_called == 0) {
		ret = s5k33dxx_video_streaming(true);  //start stream

		usleep(100000);  //need wait next frame,the setting be written into the driver ic.

		ret = s5k33dxx_video_streaming(false);  //stop stream
	}
	else {
		usleep(100000);  //need wait next frame,the setting be written into the driver ic.
	}

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4AFD);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x00);

	return ret;
}

static int vcsel_driver_read_reg_16(uint16_t reg, uint16_t *value)
{
	int ret;

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B10);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x6F12, reg);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B0F);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x02);

	if (start_streaming_called == 0) {
		ret = s5k33dxx_video_streaming(true);  //start stream

		usleep(100000);  //need wait next frame,the setting be written into the driver ic.

		ret = s5k33dxx_video_streaming(false);  //stop stream
	}
	else {
		usleep(100000);  //need wait next frame,the setting be written into the driver ic.
	}

	ret = sensor_write_reg_16(0x602C, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602E, 0x4B60);
	if (ret < 0)
		return ret;
	ret = sensor_read_reg_16(0x6F12, value);

	ret = sensor_write_reg_16(0x6028, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602A, 0x4B0F);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_8(0x6F12, 0x00);

	return ret;
}

double fixed2float(uint32_t fp, uint32_t wl, uint32_t frac, bool is_signed)
{
	uint64_t m_fp = fp;
	double val;

	int s = (m_fp&((uint64_t)1 << (wl - 1))) >> (wl - 1);

	if (s == 1)  //signed
	{
		uint64_t v1 = (~(m_fp - 1)&~(((uint64_t)0xFFFFFFFFFFFFFFFF) << (wl - 1)));
		val = -((double)v1) / (double)powf((double)2.0f, (double)(frac));
	}
	else  //unsigned
	{
		val = ((double)m_fp) / (double)powf((double)2.0f, (double)(frac));
	}

	return val;
}

#if (USE_WHICH_CONVERTER == kConverterIsCx3)
int s5k33d_power_up()
{
	s5k33dxx_mipi_cfg(&s5k33d_mipi_cfg, sizeof(s5k33d_mipi_cfg), s5k33dxx_device_id);

	s5k33dxx_set_gpio_dir(SENSOR_VDDIO_1V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_dir(SENSOR_VDDA_2V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_dir(CXA4016_VCC_3V3_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_dir(CXA4016_VDDIO_1V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_dir(CXA4016_3V6_GPIO, 0, s5k33dxx_device_id);

	s5k33dxx_set_gpio_level(SENSOR_VDDIO_1V8_GPIO, 1, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(SENSOR_VDDA_2V8_GPIO, 1, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_VCC_3V3_GPIO, 1, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_VDDIO_1V8_GPIO, 1, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_3V6_GPIO, 1, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(SENSOR_RESET_GPIO, 0, s5k33dxx_device_id);
	usleep(10);
	s5k33dxx_set_gpio_level(SENSOR_RESET_GPIO, 1, s5k33dxx_device_id);

	return 0;
}

int s5k33d_power_down()
{
	s5k33dxx_set_gpio_level(SENSOR_RESET_GPIO, 0, s5k33dxx_device_id);

	s5k33dxx_set_gpio_level(SENSOR_VDDIO_1V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(SENSOR_VDDA_2V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_VCC_3V3_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_VDDIO_1V8_GPIO, 0, s5k33dxx_device_id);
	s5k33dxx_set_gpio_level(CXA4016_3V6_GPIO, 0, s5k33dxx_device_id);

	return 0;
}

/*******************************************************************************
* Function Name: get_binning_mode
********************************************************************************
*
* Summary:
* get the sensor binning mode for SDK.
*
* Parameters:
* @mode_param 0: no scaling
*             1: 2x2 binning
*             2: 4x4 binning
*         other:error
*
* Return:
* @load_value 0:success, other is fail.
*
*******************************************************************************/
int get_binning_mode(uint8_t *mode_param)
{
	int ret = 0;
	uint8_t binning_mode = 0;
	uint8_t binning_type = 0;
	uint16_t y_odd_inc = 0;

	ret = sensor_read_reg_8(0x0900, &binning_mode);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_8(0x0901, &binning_type);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_16(0x0386, &y_odd_inc);
	if (ret < 0)
		return ret;

	if (1 == binning_mode) {  //Enabled analog binning
		if (binning_type == 0x12 && y_odd_inc == 0x0003) {
			*mode_param = 1;  //2x2 binning
		}
		else if (binning_type == 0x14 && y_odd_inc == 0x0007) {
			*mode_param = 2;
		}
		else {
			*mode_param = 0xff;
			return -HW_ERR_INVALID;
		}
	}
	else {
		*mode_param = 0x0;
	}

	return ret;
}
#endif

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
int s5k33dxx_dothin_config()
{
	int ret = 0;

	ret = s5k33dxx_enable_softpin(true, s5k33dxx_device_id);
	if (ret < 0) {
		ALOGE("dothin_enable_softpin ret=%d\n", ret);
	}
	ret = s5k33dxx_enable_gpio(true, s5k33dxx_device_id);
	if (ret < 0) {
		ALOGE("dothin_enable_gpio ret=%d\n", ret);
	}
	ret = s5k33dxx_set_sensor_clock(true, 24 * 10, s5k33dxx_device_id);  // 24Mhz mclk
	if (ret < 0) {
		ALOGE("dothin_set_sensor_clock ret=%d\n", ret);
	}
	ret = s5k33dxx_set_softpin_pullup(1, s5k33dxx_device_id);
	if (ret < 0) {
		ALOGE("dothin_set_softpin_pullup ret=%d\n", ret);
	}
	usleep(1000 * 10);
	ret = s5k33dxx_set_sensor_i2c_rate(1, s5k33dxx_device_id);  // 400Khz
	if (ret < 0) {
		ALOGE("dothin_set_sensor_i2c_rate ret=%d\n", ret);
	}
	ret = s5k33dxx_sensor_enable(1, true, s5k33dxx_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 20);
	ret = s5k33dxx_sensor_enable(3, true, s5k33dxx_device_id);
	if (ret < 0) {
		ALOGE("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

	return ret;
}

static int gpio_control(int pin, bool level)
{
	dothin_set_gpio_level(pin, level, dothin_device_id);
	return 0;  // should wrap method by SDK
}

static int max77831_write_reg(uint8_t reg, uint8_t value)
{
    int ret = i2c_reg_write(0x66<<1, reg, 1, value, 1);
    return ret;
}

//IO1 -> A SPOT, IO2 -> B SPOT
static int set_expander_tx_gpio_x210116(uint8_t A_status_enable, uint8_t B_status_enable)
{
    int rtn = 0;
    uint8_t gpio_status = 0;
    if (A_status_enable && B_status_enable)
        return 0;
    if (B_status_enable) {
        rtn = vcsel_driver_write_reg(0x0D, 0x19); // current 0.39A
        gpio_status = 0x3F | (B_status_enable << 7); // set IO2 to high
        rtn = i2c_reg_write(0x20<<1, 0x01, 1, gpio_status, 1);
    }
    if (A_status_enable) {
        gpio_status = 0x3F | (A_status_enable << 6); // set IO1 to high
        rtn = i2c_reg_write(0x20 << 1, 0x01, 1, gpio_status, 1);
        rtn = vcsel_driver_write_reg(0x0D, 0xDF); // current 3.5A
    }
    return rtn;
}

static int gpio_expander_init() // for power control
{
	int rtn = 0;
	rtn = gpio_control(GPIO_EXPANDER_RST_PIN, 1);  // pull gpio expander rst pin to high
#ifdef X210116
    rtn = gpio_control(2, 1); // pull cxa4046 rst pin to high
#endif
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x03, 1, 0x00, 1);  // set all gpio as output
#ifdef X210116
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0x3F, 1);  // set all gpio to high
    rtn = max77831_write_reg(0x13, 0x55); // 6.25V
    //rtn = set_expander_tx_gpio_x210116(1, 0); // default use SPOT_A
#else
    rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0xFF, 1);
#endif
	return rtn;
}

// Huangshan V1.0 board support only
static int tx_illum_power_control(bool enable)  // Tx LDVCC power on/off control
{
	int rtn = 0;
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		if (enable) {
			rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0xFF, 1);  // set all gpio to high
		}
		else {
			rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0xF7, 1);  // set all gpio to high except P1 (LD_VCC power)
		}
	}
	else {
		rtn = -HW_ERR_INVALID;
	}
	return rtn;
}
static uint8_t B_status;
static int get_expander_tx_gpio(uint8_t *A_status_enable, uint8_t *B_status_enable)
{
	int rtn = 0;
	uint32_t gpio_status = 0;
	if (driver_ic_type == DRIVER_IC_CXA4026) {  // Polaris B
		*B_status_enable = B_status;  // TODO: cannot read from GPIO right now
		*A_status_enable = 1;  // A is always on, no deed to control
	}
	else {
#if 0  // Taishan
		rtn = gpio_control(GPIO_EXPANDER_RST_PIN, 1);  // pull gpio expander rst pin to high
		rtn = i2c_reg_read(GPIO_EXPANDER_ADDR, 0x01, 1, &gpio_status, 1);

		*B_status_enable = ((gpio_status & 0x02) >> 1);
		*A_status_enable = gpio_status & 0x01;
#else  // Polaris C
		if (B_status) {  // TODO: cannot read from GPIO right now
			*B_status_enable = 1;
			*A_status_enable = 0;
		}
		else {
			*B_status_enable = 0;
			*A_status_enable = 1;
		}
#endif
	}
	return rtn;
}

static int set_expander_tx_gpio(uint8_t A_status_enable, uint8_t B_status_enable)
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	} else if (s5k33d_project_id == S5K33D_F201201) {
		return 0;
    }
    else if (s5k33d_project_id == S5K33D_X210116) {
        set_expander_tx_gpio_x210116(A_status_enable, B_status_enable);
        if (A_status_enable)
            B_status = 0;
        if (B_status_enable)
            B_status = 1;
        return 0;
    }

	int rtn = 0;
	uint8_t gpio_status = 0;
	if (driver_ic_type == DRIVER_IC_CXA4026) {  // Polaris B
		if (!A_status_enable)  // area A cannot be switched off
			return -1;
		if (!B_status_enable) {  // for safety. should decrease current before switched to A area
			rtn = s5k33dxx_set_illum_power(1, 0x3D, 0x00);  // 1.1A -> 0x3D
			//rtn = vcsel_driver_write_reg(0x15, 0x89);  // TODO: for light wave utilization
			B_status = 0;
		}
		rtn = gpio_control(B_AREA_CTRL_PIN_OB, B_status_enable);
		if (B_status_enable) {  // for safety. should increase current after switched to A+B area
			rtn = s5k33dxx_set_illum_power(1, 0xC2, 0x00);  // 3.5A -> 0xC2
			//rtn = vcsel_driver_write_reg(0x15, 0x52);
			B_status = 1;
		}
	}
	else {
#if 0  // Taishan
		rtn = gpio_control(GPIO_EXPANDER_RST_PIN, 1);  // pull gpio expander rst pin to high
		rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x03, 1, 0x00, 1);  // set all gpio as output
		gpio_status = 0xFC | (B_status_enable << 1) | A_status_enable;
		rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, gpio_status, 1);  // set all gpio to high
#else  // Polaris C
		if (A_status_enable && B_status_enable)  // area A and B cannot be turned on at the same time
			return -1;
		if (A_status_enable) {  // for safety. should decrease current before switched to A area
			rtn = vcsel_driver_write_reg(0x08, 0x56);  // 1.55A -> 0x56,  1.0A -> 0x38
			rtn = gpio_control(B_AREA_CTRL_PIN_OB, 0);
			rtn = vcsel_driver_write_reg(0x0F, 0x89);
			rtn = vcsel_driver_write_reg(0x10, 0xC0);
			B_status = 0;
		}

		if (B_status_enable) {  // for safety. should increase current after switched to A+B area
			rtn = gpio_control(B_AREA_CTRL_PIN_OB, 1);
			rtn = vcsel_driver_write_reg(0x08, 0x92);  // 2.625A -> 0x92
			rtn = vcsel_driver_write_reg(0x0F, 0x89);
			rtn = vcsel_driver_write_reg(0x10, 0xC0);
			B_status = 1;
		}
#endif
	}
	return rtn;
}

static int get_shuffle_mode(bool *enable)  //0x01 enable shuffle mode, 0x00 disable shuffle
{
	int ret = 0;
	uint8_t shuffle_mode = 0;  //4:shuffle*dual_freq 2:non_shuffle*dual_freq
	uint8_t toogle_mode = 0;  //03:toogle every 1 frame 00:always low(disable tap-shuffle)

	ret = sensor_write_reg_16(0x602C, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602E, 0x2054);
	if (ret < 0)
		return ret;
	ret = sensor_read_reg_8(0x6F12, &shuffle_mode);
	if (ret < 0)
		return ret;

	ret = sensor_write_reg_16(0x602C, 0x2000);
	if (ret < 0)
		return ret;
	ret = sensor_write_reg_16(0x602E, 0x126f);
	if (ret < 0)
		return ret;
	ret = sensor_read_reg_8(0x6F12, &toogle_mode);
	if (ret < 0)
		return ret;

	if (toogle_mode == 0x03 && shuffle_mode == 0x04) {
		*enable = 1;
	}
	else if (toogle_mode == 0x00 && shuffle_mode == 0x02) {
		*enable = 0;
	}
	else {
		return -HW_ERR_INVALID;
	}

	return ret;
}

static int set_shuffle_mode(bool enable)  //0x01 enable shuffle mode, 0x00 disable shuffle
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
	int ret = 0;
	uint8_t gpio_status = 0;

	ret = sensor_write_reg_8(0x0100, 0x00);  // should stop streaming first
	if (ret < 0)
		return ret;

	if (enable) {
		ret = sensor_write_reg_16(0x6028, 0x2000);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x126F);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x03);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x602A, 0x2054);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x04);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x602A, 0x1197);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x04);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x1199);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x1B);
		if (ret < 0)
			return ret;
	}
	else {
		ret = sensor_write_reg_16(0x6028, 0x2000);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x126F);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x00);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x602A, 0x2054);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x02);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x602A, 0x1197);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x02);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x1199);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_8(0x6F12, 0x10);
		if (ret < 0)
			return ret;
	}


	if (start_streaming_called)
		ret = sensor_write_reg_8(0x0100, 0x01);

	return ret;
}
/*******************************************************************************
* Function Name: get_binning_mode
********************************************************************************
*
* Summary:
* get the sensor binning mode for SDK.
*
* Parameters:
* @mode_param 0: no scaling
*             1: 2x2 binning
*             2: 4x4 binning
*         other:error
*
* Return:
* @load_value 0:success, other is fail.
*
*******************************************************************************/
int get_binning_mode(uint8_t *mode_param)
{
	int ret = 0;
	uint8_t binning_mode = 0;
	uint8_t binning_type = 0;
	uint16_t y_odd_inc   = 0;

	ret = sensor_read_reg_8(0x0900, &binning_mode);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_8(0x0901, &binning_type);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_16(0x0386, &y_odd_inc);
	if (ret < 0)
		return ret;

	if (1 == binning_mode) {  //Enabled analog binning
		if (binning_type == 0x12 && y_odd_inc == 0x0003) {
			*mode_param = 1;  //2x2 binning
		}
		else if (binning_type == 0x14 && y_odd_inc == 0x0007) {
			*mode_param = 2;
		}
		else {
			*mode_param = 0xff;
			return -HW_ERR_INVALID;
		}
	}
	else {
		*mode_param = 0x0;
	}

	return ret;
}
/*******************************************************************************
* Function Name: set_binning_mode
********************************************************************************
*
* Summary:
* set the sensor binning mode for SDK.
*
* Parameters:
* @mode_param 0: no scaling
*             1: 2x2 binning
*             2: 4x4 binning
*         other:error
*
* Return:
* @load_value 0:success, other is fail.
*
*******************************************************************************/
static int set_binning_mode(uint8_t mode_param)
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
	int ret = 0;

	if (mode_param > 2) {
		return -HW_ERR_INVALID;
	}

	ret = sensor_write_reg_8(0x0100, 0x00);  // should stop streaming first
	if (ret < 0)
		return ret;

	if (2 == mode_param) {
		ret = sensor_write_reg_16(0xF4E0, 0x0101);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034C, 0x0140);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034E, 0x00F0);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0386, 0x0007);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0900, 0x0114);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0404, 0x4000);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x6028, 0x2000);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x2038);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0030);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0024);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0110);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x00CC);
		if (ret < 0)
			return ret;
	}
	else if (1 == mode_param) {
		ret = sensor_write_reg_16(0xF4E0, 0x0101);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034C, 0x0280);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034E, 0x01E0);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0386, 0x0003);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0900, 0x0112);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0404, 0x2000);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x6028, 0x2000);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x2038);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0060);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0048);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0220);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0198);
		if (ret < 0)
			return ret;
	}
	else {
		ret = sensor_write_reg_16(0xF4E0, 0x0100);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034C, 0x0500);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x034E, 0x03C0);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0386, 0x0001);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0900, 0x0011);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0404, 0x1000);
		if (ret < 0)
			return ret;

		ret = sensor_write_reg_16(0x6028, 0x2000);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x602A, 0x2038);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x00C0);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0090);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0440);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x6F12, 0x0330);
		if (ret < 0)
			return ret;
	}

	if (start_streaming_called)
		ret = sensor_write_reg_8(0x0100, 0x01);

    ALOGE("set_binning_mode %d, ret = %d", mode_param,ret);

	return ret;
}

/*******************************************************************************
* Function Name: get_burst_mode
********************************************************************************
*
* Summary:
* get the sensor burst mode for SDK.
*
* Parameters:
* @mode_param 0: mode1(normal)
*             1: mode2
*             2: mode3
*         other: error
*
* Return:
* @load_value 0:success, other is fail.
*
*******************************************************************************/
static int get_burst_mode(uint8_t *mode_param)
{
	int      ret                           = 0;
	uint8_t  n_frames_as_one_slow          = 0;
	uint16_t total_slow_frame_timing_lines = 0;
	uint16_t frame_length_lines            = 0;

	ret = sensor_read_reg_8(0x070D, &n_frames_as_one_slow);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_16(0x070E, &total_slow_frame_timing_lines);
	if (ret < 0)
		return ret;

	ret = sensor_read_reg_16(0x0340, &frame_length_lines);
	if (ret < 0)
		return ret;

	if (4 == n_frames_as_one_slow) {
		if (frame_length_lines == (total_slow_frame_timing_lines / 4)) {
			*mode_param = 0;
		}
		else if (frame_length_lines < (total_slow_frame_timing_lines / 4)) {
			*mode_param = 2;
		}
		else {
			*mode_param = 0xff;
			return -HW_ERR_INVALID;
		}
	}
	else if (2 == n_frames_as_one_slow) {
		if (frame_length_lines == (total_slow_frame_timing_lines / 2)) {
			*mode_param = 0;
		}
		else if (frame_length_lines < (total_slow_frame_timing_lines / 2)) {
			*mode_param = 1;
		}
		else {
			*mode_param = 0xff;
			return -HW_ERR_INVALID;
		}
	}
	else if (1 == n_frames_as_one_slow) {
		*mode_param = 0;
	}
	else {
		*mode_param = 0xff;
		return -HW_ERR_INVALID;
	}

	return ret;
}
/*******************************************************************************
* Function Name: set_burst_mode
********************************************************************************
*
* Summary:
* set the sensor burst mode for SDK.
*
* Parameters:
* @mode_param 0: mode1(normal)
*             1: mode2
*             2: mode3
*         other: error
*
* Return:
* @load_value 0:success, other is fail.
*
*******************************************************************************/
static int set_burst_mode(uint8_t mode_param)
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
	int ret = 0;
	uint8_t  n_frames_as_one_slow = 0;
	uint16_t total_slow_frame_timing_lines = 0;

	if (mode_param > 2) {
		return HW_ERR_INVALID;
	}

	ret = sensor_write_reg_8(0x0100, 0x00);  // should stop streaming first
	if (ret < 0)
		return ret;

	if (2 == mode_param) {
		ret = sensor_write_reg_8(0x070D, 0x04);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0340, 0x0456);
		if (ret < 0)
			return ret;
		if (0 != set_fps_called) {
			ret = sensor_write_reg_16(0x070E, (0x188A * 30 / set_fps_called));
			if (ret < 0)
				return ret;
		}
		else {
			ret = sensor_write_reg_16(0x070E, 0x188A);
			if (ret < 0)
				return ret;
		}
	}
	else if (1 == mode_param) {
		ret = sensor_write_reg_8(0x070D, 0x02);
		if (ret < 0)
			return ret;
		ret = sensor_write_reg_16(0x0340, 0x0456);
		if (ret < 0)
			return ret;
		if (0 != set_fps_called) {
			ret = sensor_write_reg_16(0x070E, (0x188A * 30 / set_fps_called));
			if (ret < 0)
				return ret;
		}
		else {
			ret = sensor_write_reg_16(0x070E, 0x188A);
			if (ret < 0)
				return ret;
		}
	}
	else {
		ret = sensor_read_reg_8(0x070D, &n_frames_as_one_slow);
		if (ret < 0)
			return ret;
		ret = sensor_read_reg_16(0x070E, &total_slow_frame_timing_lines);
		if (ret < 0)
			return ret;

		if (4 == n_frames_as_one_slow) {
			ret = sensor_write_reg_16(0x0340, (total_slow_frame_timing_lines / 4));
			if (ret < 0)
				return ret;
		}
		else if (2 == n_frames_as_one_slow) {
			ret = sensor_write_reg_16(0x0340, (total_slow_frame_timing_lines / 2));
			if (ret < 0)
				return ret;
		}
	}

	if (start_streaming_called)
		ret = sensor_write_reg_8(0x0100, 0x01);

	return ret;
}

int s5k33d_get_frequency_mode(uint8_t *mode)
{
	uint8_t value;

	int rtn = 0;

	*mode = DUAL_FREQ;  // DMFD

	return rtn;
}

int s5k33d_set_frequency_mode(uint8_t mode)
{
	int ret;

	switch (mode) {
	case DUAL_FREQ: {
		ret = HW_NO_ERR;
		break;
	}
	case SINGLE_FREQ: {
		ret = HW_ERR_NO_SUPPORT;
		break;
	}
	case AF_FREQ: {
		ret = HW_ERR_NO_SUPPORT;
		break;
	}
	default: {
		ret = HW_ERR_INVALID;
		break;
	}
	}

	return ret;
}

#endif

static struct regList cxa4016_reglist[] = {
	// for CXA4016 driver IC setting
	{ 0x602A, 0x1F90 },// address switch
	{ 0x6F12, 0x1325 },// 1> driver IC, total 0x13, addr 0x25
	{ 0x6F12, 0x0001 },// 2>
	{ 0x6F12, 0x0203 },// 3>
	{ 0x6F12, 0x0405 },// 4>
	{ 0x6F12, 0x0607 },// 5>
	{ 0x6F12, 0x0809 },// 6>
	{ 0x6F12, 0x0A0B },// 7>
	{ 0x6F12, 0x0C0D },// 8>
	{ 0x6F12, 0x0E0F },// 9>
	{ 0x6F12, 0x1013 },// 10>
	{ 0x602A, 0x1FC0 },// address switch
	{ 0x6F12, 0x0001 },// 1> driver IC, value 0x01
	{ 0x6F12, 0x0C00 },// 2>
	{ 0x6F12, 0x0000 },// 3>
	{ 0x6F12, 0x0000 },// 4>
	{ 0x6F12, 0x0000 },// 5>
	{ 0x6F12, 0x4300 },// 6>     0x43->1A    0xFF->3.8A
	{ 0x6F12, 0x0400 },// 7>
	{ 0x6F12, 0x28C3 },// 8>     0x28C3->difuser error detect off,  0x280C->origin
	{ 0x6F12, 0x77A0 },// 9>
	{ 0x6F12, 0x0003 },// 10>
};


struct regList cxa4026_reglist[] = {
	// for PHX3D driver IC setting
	{ 0x602A,0x1F90 },// address switch
	{ 0x6F12,0x192D },// 1> driver IC, total 0x19, addr 0x2D
	{ 0x6F12,0x0001 },// 2>
	{ 0x6F12,0x0203 },// 3>
	{ 0x6F12,0x0405 },// 4>
	{ 0x6F12,0x0607 },// 5>
	{ 0x6F12,0x0809 },// 6>
	{ 0x6F12,0x0A0B },// 7>
	{ 0x6F12,0x0C0D },// 8>
	{ 0x6F12,0x0E0F },// 9>
	{ 0x6F12,0x1011 },// 10>
	{ 0x6F12,0x1213 },// 11>
	{ 0x6F12,0x1415 },// 12>
	{ 0x6F12,0x1617 },// 13>
	{ 0x602A,0x1FC0 },// address switch
	{ 0x6F12,0x0001 },// 1> driver IC, value 0x01
	{ 0x6F12,0x1C00 },// 2>
	{ 0x6F12,0x0000 },// 3>
	{ 0x6F12,0x0000 },// 4>
	{ 0x6F12,0x0000 },// 5>
	{ 0x6F12,0xC200 },// 6>     0x43->1A    0xD3->3.8A
	{ 0x6F12,0x0400 },// 7>
	{ 0x6F12,0x2800 },// 8>
	{ 0x6F12,0x06C0 },// 9>
	{ 0x6F12,0xFF80 },// 10>
	{ 0x6F12,0x0000 },// 11>
	{ 0x6F12,0x0093 },// 12>
	{ 0x6F12,0x0F21 },// 13>
};

static struct regList cxa4046_reglist[] = {
    // for PHX3D driver IC setting
    { 0x602A,0x1F90 },// address switch
    { 0x6F12,0x213C },// 1> driver IC, total 0x19, addr 0x2D
    { 0x6F12,0x0001 },// 2>
    { 0x6F12,0x0203 },// 3>
    { 0x6F12,0x0405 },// 4>
    { 0x6F12,0x0607 },// 5>
    { 0x6F12,0x0809 },// 6>
    { 0x6F12,0x0A0B },// 7>
    { 0x6F12,0x0C0D },// 8>
    { 0x6F12,0x0E0F },// 9>
    { 0x6F12,0x1011 },// 10>
    { 0x6F12,0x1213 },// 11>
    { 0x6F12,0x1415 },// 12>
    { 0x6F12,0x1617 },// 13>
    { 0x6F12,0x1819 },// 14>
    { 0x6F12,0x1A1B },// 15>
    { 0x6F12,0x1C1D },// 16>
    { 0x6F12,0x1E1F },// 17>

    { 0x602A,0x1FC0 },// address switch
    { 0x6F12,0x0001 },// 1> driver IC, value 0x01
    { 0x6F12,0x0001 },// 2>
    { 0x6F12,0x1900 },// 3>
    { 0x6F12,0x4025 },// 4>
    { 0x6F12,0x1E81 },// 5>
    { 0x6F12,0x428F },// 6>     0x43->1A    0xD3->3.8A
    { 0x6F12,0x0000 },// 7>
    { 0x6F12,0x00df },// 8>
    { 0x6F12,0x0000 },// 9>
    { 0x6F12,0x0000 },// 10>
    { 0x6F12,0x0000 },// 9>
    { 0x6F12,0x0000 },// 10>
    { 0x6F12,0x0000 },// 9>
    { 0x6F12,0x00c0 },// 10>
    { 0x6F12,0xD078 },// 11>
    { 0x6F12,0x213f },// 12>
    { 0x6F12,0x0000 },// 13>
};

static struct regList phx3d_3021_aa_reglist[] = {
	// for PHX3D driver IC setting
	{ 0x602A,0x1F90 },// address switch
	{ 0x6F12,0x1725 },// 1> driver IC, total 0x17, addr 0x25
	{ 0x6F12,0x0001 },// 2>
	{ 0x6F12,0x0203 },// 3>
	{ 0x6F12,0x0405 },// 4>
	{ 0x6F12,0x0607 },// 5>
	{ 0x6F12,0x0809 },// 6>
	{ 0x6F12,0x0A0B },// 7>
	{ 0x6F12,0x0C0D },// 8>
	{ 0x6F12,0x0E0F },// 9>
	{ 0x6F12,0x1011 },// 10>
	{ 0x6F12,0x2627 },// 11>
	{ 0x6F12,0x282D },// 12>
	{ 0x602A,0x1FC0 },// address switch
	{ 0x6F12,0x0001 },// 1> driver IC, value 0x01
	{ 0x6F12,0x0CA0 },// 2>
	{ 0x6F12,0x800A },// 3>
	{ 0x6F12,0x2849 },// 4>
	{ 0x6F12,0x8700 },// 5>
	{ 0x6F12,0x7A00 },// 6>     0x43->1A    0xD3->3.8A
	{ 0x6F12,0x0135 },// 7>
	{ 0x6F12,0x28C0 },// 8>
	{ 0x6F12,0x7804 },// 9>
	{ 0x6F12,0xD006 },// 10>
	{ 0x6F12,0x1931 },// 11>
	{ 0x6F12,0xFFFF },// 12>
};

// PHX3D 3025 almost the same with PHX3D 3021 except XCLR mask control function
// (PHX3D 3025, register 0x2E bit 0 should set to 1, while PHX3D 3021 don't care this)
static struct regList phx3d_3021_cb_reglist[] = {
	{ 0x602A,0x1F90 },// address switch
	{ 0x6F12,0x1925 },// 1> driver IC, total 0x19, addr 0x25
	{ 0x6F12,0x0001 },// 2>
	{ 0x6F12,0x0203 },// 3>
	{ 0x6F12,0x0405 },// 4>
	{ 0x6F12,0x0607 },// 5>
	{ 0x6F12,0x0809 },// 6>
	{ 0x6F12,0x0A0B },// 7>
	{ 0x6F12,0x0C0D },// 8>
	{ 0x6F12,0x0E0F },// 9>
	{ 0x6F12,0x1032 },// 10>
	{ 0x6F12,0x262B },// 11>
	{ 0x6F12,0x2C2D },// 12>
	{ 0x6F12,0x2E2F },// 13>
	{ 0x602A,0x1FC0 },// address switch
	{ 0x6F12,0x0001 },// 1> driver IC, value 0x01
	{ 0x6F12,0x1C20 },// 2>
	{ 0x6F12,0x3010 },// 3>
	{ 0x6F12,0x2069 },// 4>
	{ 0x6F12,0xC300 },// 5>
	{ 0x6F12,0xAC24 },// 6>  0x43->1A  0xD3->3.8A  0xAC->3.1A
	{ 0x6F12,0x0DFF },// 7>
	{ 0x6F12,0xD900 },// 8>
	{ 0x6F12,0x0F04 },// 9>
	{ 0x6F12,0xCFFF },// 10>
	{ 0x6F12,0x19F1 },// 11>
	{ 0x6F12,0xF803 },// 12>
	{ 0x6F12,0x1300 },// 13>
};

static struct regList phx3d_3018_cb_reglist[] = {
	{ 0x602A, 0x1F90 },// address switch
	{ 0x6F12, 0x2F2D },// 1> driver IC, total 0x2F, addr 0x2D
	{ 0x6F12, 0x2B2D },// 2>
	{ 0x6F12, 0x0001 },// 3>
	{ 0x6F12, 0x0203 },// 4>
	{ 0x6F12, 0x0405 },// 5>
	{ 0x6F12, 0x0607 },// 6>
	{ 0x6F12, 0x0809 },// 7>
	{ 0x6F12, 0x0A0B },// 8>
	{ 0x6F12, 0x0C10 },// 9>
	{ 0x6F12, 0x1516 },// 10>
	{ 0x6F12, 0x0E33 },// 11>
	{ 0x6F12, 0x3435 },// 12>
	{ 0x6F12, 0x4041 },// 13>
	{ 0x6F12, 0x4243 },// 14>
	{ 0x6F12, 0x4445 },// 15>
	{ 0x6F12, 0x4647 },// 16>
	{ 0x6F12, 0x4849 },// 17>
	{ 0x6F12, 0x4A4B },// 18>
	{ 0x6F12, 0x5055 },// 19>
	{ 0x6F12, 0x5663 },// 20>
	{ 0x6F12, 0x6465 },// 21>
	{ 0x6F12, 0x0F2C },// 22>
	{ 0x6F12, 0x2E2F },// 23>
	{ 0x6F12, 0x0D32 },// 24>
	{ 0x602A, 0x1FC0 },// address switch
	{ 0x6F12, 0x0001 },// 1> driver IC, value 0x01
	{ 0x6F12, 0x0002 },// 2>
	{ 0x6F12, 0x1C20 },// 3>
	{ 0x6F12, 0x3010 },// 4>
	{ 0x6F12, 0x2069 },// 5>
	{ 0x6F12, 0xC300 },// 6>
	{ 0x6F12, 0xC224 },// 7>   3.1A
	{ 0x6F12, 0x0DFF },// 8>
	{ 0x6F12, 0xD9FF },// 9>
	{ 0x6F12, 0x0004 },// 10>
	{ 0x6F12, 0x07FF },// 11>
	{ 0x6F12, 0x0003 },// 12>
	{ 0x6F12, 0x1C20 },// 13>
	{ 0x6F12, 0x3010 },// 14>
	{ 0x6F12, 0x2069 },// 15>
	{ 0x6F12, 0xC300 },// 16>
	{ 0x6F12, 0xC224 },// 17>   3.1A
	{ 0x6F12, 0x0DFF },// 18>
	{ 0x6F12, 0xFF00 },// 19>
	{ 0x6F12, 0x04F1 },// 20>
	{ 0x6F12, 0xF803 },// 21>
	{ 0x6F12, 0x04F8 },// 22>
	{ 0x6F12, 0x1200 },// 23>
	{ 0x6F12, 0x00FF },// 24>

	{ 0x602A, 0x2024 },
	{ 0x6F12, 0x0018 },
	{ 0x6F12, 0xFFFF },
	{ 0x6F12, 0x0000 },
	{ 0x6F12, 0x0000 },
	{ 0x602A, 0x4AA8 },
	{ 0x6F12, 0x0009 },
	{ 0x602A, 0x19D0 },
	{ 0x6F12, 0x143C },
	{ 0x602A, 0x12DE },
	{ 0x6F12, 0x0010 },
	{ 0x602A, 0x1130 },
	{ 0x6F12, 0x0001 },
	{ 0x602A, 0x2050 },
	{ 0x6F12, 0x0200 },
	{ 0x602A, 0x4ACC },
	{ 0x6F12, 0x0008 },
	{ 0x602A, 0x4AAA },
	{ 0x6F12, 0x0017 },
	{ 0x6F12, 0x0030 },
	{ 0x6F12, 0x0020 },
	{ 0x6F12, 0x0018 },
	{ 0x6F12, 0x0080 },
	{ 0x6F12, 0x0000 },
	{ 0x6F12, 0x0001 },
	{ 0x6F12, 0x0001 },
	{ 0x6F12, 0x0000 },
};

struct regList dw9912_reglist[] = {
	// for DW9912 driver IC setting
	{ 0x602A, 0x1F90 },// address switch
	{ 0x6F12, 0x1125 },// 1> driver IC, total 0x11, addr 0x25
	{ 0x6F12, 0x0001 },// 2>
	{ 0x6F12, 0x0203 },// 3>
	{ 0x6F12, 0x0405 },// 4>
	{ 0x6F12, 0x0607 },// 5>
	{ 0x6F12, 0x080D },// 6>
	{ 0x6F12, 0x0E0F },// 7>
	{ 0x6F12, 0x1012 },// 8>
	{ 0x6F12, 0x3B3B },// 9>
	{ 0x602A, 0x1FC0 },// address switch
	{ 0x6F12, 0x0001 },// 1> driver IC,address 0x25, value 0x01
	{ 0x6F12, 0x0C23 },// 2>
	{ 0x6F12, 0x4666 },// 3>
	{ 0x6F12, 0x86A6 },// 4>
	{ 0x6F12, 0xc600 },// 5>
	{ 0x6F12, 0xFFc0 },// 6>
	{ 0x6F12, 0x0783 },// 7>
	{ 0x6F12, 0x0f01 },// 8>
	{ 0x6F12, 0x0707 },// 9>
};

static int sensor_register_init()
{
	int ret = 0;
#if (USE_WHICH_CONVERTER == kConverterIsCx3)
#if (CURRENT_USED_SETTING_CX3 == S5K33D_CX3)
	for (int i = 0; i < sizeof(s5k33d_reglist_cx3) / sizeof(struct regList); i++) {
		ret = sensor_write_reg_16(s5k33d_reglist_cx3[i].reg, s5k33d_reglist_cx3[i].val);
	}
#endif
#endif

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
    ALOGE("debug====>s5k33d_project_id=%d", s5k33d_project_id);
	switch (s5k33d_project_id) {
	case S5K33D_HUANGSHAN:
		for (int i = 0; i < sizeof(s5k33d_reglist_huangshan) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(s5k33d_reglist_huangshan[i].reg, s5k33d_reglist_huangshan[i].val);
		}
		break;

    case S5K33D_X210116:
	case S5K33D_F201201:
	case S5K33D_TAISHAN:
		for (int i = 0; i < sizeof(s5k33d_reglist_taishan) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(s5k33d_reglist_taishan[i].reg, s5k33d_reglist_taishan[i].val);
		}
		break;

	case S5K33D_T200515:
		for (int i = 0; i < sizeof(s5k33d_reglist_t200515) / sizeof(struct regList); i++) {
			ret |= sensor_write_reg_16(s5k33d_reglist_t200515[i].reg, s5k33d_reglist_t200515[i].val);
		}
		break;

	default:
		ret = HW_ERR_NO_SUPPORT;
	}
#endif
	return ret;
}

static int cxa4046_init()
{
    int ret = 0;
    ret |= vcsel_driver_write_reg(0x3C, 0x01);
    ret |= vcsel_driver_write_reg(0x00, 0x00);
    ret |= vcsel_driver_write_reg(0x01, 0x01);
    ret |= vcsel_driver_write_reg(0x02, 0x19);
    ret |= vcsel_driver_write_reg(0x03, 0x00);
    ret |= vcsel_driver_write_reg(0x04, 0x40);
    ret |= vcsel_driver_write_reg(0x05, 0x25);
    ret |= vcsel_driver_write_reg(0x06, 0x1E);
    ret |= vcsel_driver_write_reg(0x07, 0x80);
    ret |= vcsel_driver_write_reg(0x08, 0x42);
    ret |= vcsel_driver_write_reg(0x09, 0x88);
    ret |= vcsel_driver_write_reg(0x0a, 0x00);
    ret |= vcsel_driver_write_reg(0x0b, 0x00);
    ret |= vcsel_driver_write_reg(0x0c, 0x5F);
    ret |= vcsel_driver_write_reg(0x0d, 0xDF);
    ret |= vcsel_driver_write_reg(0x0e, 0x00);
    ret |= vcsel_driver_write_reg(0x0f, 0x00);
    ret |= vcsel_driver_write_reg(0x10, 0x00);
    ret |= vcsel_driver_write_reg(0x11, 0x00);
    ret |= vcsel_driver_write_reg(0x12, 0x00);
    ret |= vcsel_driver_write_reg(0x13, 0x00);
    ret |= vcsel_driver_write_reg(0x14, 0x00);
    ret |= vcsel_driver_write_reg(0x15, 0x00);
    ret |= vcsel_driver_write_reg(0x16, 0x00);
    ret |= vcsel_driver_write_reg(0x17, 0x00);
    ret |= vcsel_driver_write_reg(0x18, 0x00);
    ret |= vcsel_driver_write_reg(0x19, 0xC0);
    ret |= vcsel_driver_write_reg(0x1a, 0xD0);
    ret |= vcsel_driver_write_reg(0x1b, 0x78);
    ret |= vcsel_driver_write_reg(0x1c, 0x21);
    ret |= vcsel_driver_write_reg(0x1d, 0x3F);
    ret |= vcsel_driver_write_reg(0x1e, 0x04);
    return ret;
}

static int driver_ic_detect()
{
	int ret = 0;
#if (USE_WHICH_CONVERTER == kConverterIsCx3)
	ret = sensor_write_reg_8(0x0100, 0x01);  // start streaming

	uint8_t value = 0;

	ret = vcsel_driver_read_reg(0x0F, &value);  // correspond to driver IC address 0x0F
	if (value == 0x06) {
		uint8_t phx3d_value = 0;

		ret = vcsel_driver_read_reg(0x2F, &phx3d_value);  // correspond to driver IC address 0x2F

		if (phx3d_value == 0x00) {
			driver_ic_type = DRIVER_IC_PHX3D_3021_AA;
		}
		else if (phx3d_value == 0x01) {
			driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
		}
	}
	else if (value == 0x46) {
		driver_ic_type = DRIVER_IC_CXA4016;
	}
	else if (value == 0x04) {
		driver_ic_type = DRIVER_IC_CXA4026;
	}
	else if (value == 0xA7) {
		driver_ic_type = DRIVER_IC_DW9912;
	}
	else if (value == 0x70) {
		driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
	}
	else if (value == 0xA0) {
		driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
	}
	else
	{
		driver_ic_type = 0;
		ALOGE("no driver ic detected");
	}

	ALOGE("driver ic type %d ", driver_ic_type);

	ret = sensor_write_reg_8(0x0100, 0x00); // stop streaming

	if (driver_ic_type == DRIVER_IC_CXA4016) {
		for (int i = 0; i < sizeof(cxa4016_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(cxa4016_reglist[i].reg, cxa4016_reglist[i].val);

		}
	}
	else if (driver_ic_type == DRIVER_IC_CXA4026) {
		for (int i = 0; i < sizeof(cxa4026_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(cxa4026_reglist[i].reg, cxa4026_reglist[i].val);

		}
	}
	else if (driver_ic_type == DRIVER_IC_PHX3D_3021_AA) {
		for (int i = 0; i < sizeof(phx3d_3021_aa_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(phx3d_3021_aa_reglist[i].reg, phx3d_3021_aa_reglist[i].val);

		}
	}
	else if (driver_ic_type == DRIVER_IC_PHX3D_3021_CB || driver_ic_type == 0) {
		for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);
		}
	}
	else if (driver_ic_type == DRIVER_IC_DW9912) {
		for (int i = 0; i < sizeof(dw9912_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(dw9912_reglist[i].reg, dw9912_reglist[i].val);

		}
	}
#endif
    if (s5k33d_project_id == S5K33D_X210116) {
        driver_ic_type = DRIVER_IC_CXA4046;
        ret = sensor_write_reg_8(0x0100, 0x00);  // stop streaming
        ret = cxa4046_init();//
        /*
        for (int i = 0; i < sizeof(cxa4046_reglist) / sizeof(struct regList); i++) {
            ret = sensor_write_reg_16(cxa4046_reglist[i].reg, cxa4046_reglist[i].val);
        }
        */
        return ret;
    }

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		driver_ic_type = DRIVER_IC_PHX3D_3021_AA;
	}
	else if (s5k33d_project_id == S5K33D_T200515) {
		driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
		return 0;
	}
	else if (s5k33d_project_id == S5K33D_F201201) {
		driver_ic_type = OBC_DRIVER_IC_PHX3D_3018;

		for (int i = 0; i < sizeof(phx3d_3018_cb_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(phx3d_3018_cb_reglist[i].reg, phx3d_3018_cb_reglist[i].val);
		}
		ALOGE("driver ic type: %d", driver_ic_type);
		//临时修改
		uint16_t value_m = 6800;//3J 6.8V
		s5k33d_aa_qc_index = F201201_TX_QC_INDEX;
		s5k33dxx_set_ld_vcc(value_m);
		ALOGE("s5k33dxx_set_ld_vcc: %d mV", value_m);
		//end
	}
	else {
		ret = sensor_write_reg_8(0x0100, 0x01);  // start streaming

		uint8_t value = 0;

		ret = vcsel_driver_read_reg(0x0F, &value);  // correspond to driver IC address 0x0F

		if (value == 0x06) {
			uint8_t phx3d_value = 0;

			ret = vcsel_driver_read_reg(0x2F, &phx3d_value);  // correspond to driver IC address 0x2F

			ALOGE("vcsel driver 0x2F val: %d", phx3d_value);

			if (phx3d_value == 0x00) {
				driver_ic_type = DRIVER_IC_PHX3D_3021_AA;
			}
			else if (phx3d_value == 0x01) {
				driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
			}
		}
		else if (value == 0x46) {
			driver_ic_type = DRIVER_IC_CXA4016;
		}
		else if (value == 0x04) {
			driver_ic_type = DRIVER_IC_CXA4026;
		}
		else if (value == 0xA7) {
			driver_ic_type = DRIVER_IC_DW9912;
		}
		else {
			driver_ic_type = 0;
			ALOGE("no driver ic detected");
		}

		ALOGE("driver ic type %d ", driver_ic_type);

		ret = sensor_write_reg_8(0x0100, 0x00);  // stop streaming

		if (driver_ic_type == DRIVER_IC_CXA4016) {
			for (int i = 0; i < sizeof(cxa4016_reglist) / sizeof(struct regList); i++) {
				ret = sensor_write_reg_16(cxa4016_reglist[i].reg, cxa4016_reglist[i].val);

			}
		}
		else if (driver_ic_type == DRIVER_IC_CXA4026) {
			for (int i = 0; i < sizeof(cxa4026_reglist) / sizeof(struct regList); i++) {
				ret = sensor_write_reg_16(cxa4026_reglist[i].reg, cxa4026_reglist[i].val);

			}
		}
		else if (driver_ic_type == DRIVER_IC_PHX3D_3021_AA) {
			for (int i = 0; i < sizeof(phx3d_3021_aa_reglist) / sizeof(struct regList); i++) {
				ret = sensor_write_reg_16(phx3d_3021_aa_reglist[i].reg, phx3d_3021_aa_reglist[i].val);

			}
		}
		else if (driver_ic_type == DRIVER_IC_PHX3D_3021_CB || driver_ic_type == 0) {
			for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {
				ret = sensor_write_reg_16(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);
			}
		}
		else if (driver_ic_type == DRIVER_IC_DW9912) {
			for (int i = 0; i < sizeof(dw9912_reglist) / sizeof(struct regList); i++) {
				ret = sensor_write_reg_16(dw9912_reglist[i].reg, dw9912_reglist[i].val);

			}
		}
	}

#endif
	return ret;
}

int s5k33dxx_driver_ic_detect(uint16_t * ic_type)
{
	if (NULL == ic_type) {
		return  -HW_ERR_NULL;
	}

	int ret = driver_ic_detect();
	if (ret < 0) {
		return  ret;
	}

	if (0 == driver_ic_type) {
		return -HW_ERR_BAD_VALUE;
	}
	*ic_type = driver_ic_type;

	return 0;
}

static int set_project_default_param(void)
{
	int ret = 0;

	if (s5k33d_project_id == S5K33D_TAISHAN || s5k33d_project_id == S5K33D_X210116)
	{
		ret = s5k33dxx_set_illum_duty_cycle(16);  // 50% duty cycle

		ret += s5k33dxx_set_integration_time(600);

		ret += s5k33dxx_set_modulation_frequency(0x640A);  // 100M + 10M
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN)
	{
		ret = s5k33dxx_set_illum_duty_cycle(16);  // 50% duty cycle

		ret += s5k33dxx_set_integration_time(600);

		ret += s5k33dxx_set_modulation_frequency(0x6450);  // 100M + 80M
	}
	else if (s5k33d_project_id == S5K33D_T200515)
	{

	}
	else
	{
		return HW_ERR_NO_SUPPORT;
	}

	return ret;
}

/**
* @brief  s5k33dxx_sensor_init
* @return int
*/
int s5k33dxx_sensor_init()
{
	int ret = 0;

	start_stream_count = 0;  //sensor init setting,need allowed update driver ic setting.

	ret = sensor_register_init();

	ret = driver_ic_detect();

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	ret = set_project_default_param();

#ifndef X210116
	ret = set_expander_tx_gpio(0, 1);
#endif

#endif

	return ret;
}

int s5k33dxx_get_project_id(uint16_t *id)
{
#ifdef X210116
    *id = S5K33D_X210116;
#else
	uint8_t value[2];

	if (i2c_reg_read(S5K33D_I2C_ADDR_H, 0x0000, 2, value, 2) == 0)  // T200515  F201201
	{
		if (i2c_reg_read(ST_MAX7783_ADDR, 0x13, 1, value, 1) == 0)
		{
			*id = S5K33D_F201201;
		}
		else
		{
			*id = S5K33D_T200515;
		}
	}
	else if (i2c_reg_read(S5K33D_I2C_ADDR_L, 0x0000, 2, value, 2) == 0)  // Taishan  Huangshan
	{
		if (fm24c128_eeprom_write_protect(0) == 0)
		{
			*id = S5K33D_TAISHAN;
		}
		else if (gt24c512b_eeprom_block_read(0x0000, value, 2) == 0)
		{
			*id = S5K33D_HUANGSHAN;
		}
		else
		{
			return -HW_ERR_NULL;
		}
	}
#endif
	return 0;
}

/**
* @brief  s5k33dxx_get_sensor_id 闁兼儳鍢茶ぐ鍣慹nsor閻犱焦鍎抽ˇ鐞丏
* @param  [out] uint16_t *id
* @return int
*/
int s5k33dxx_get_sensor_id(uint16_t *id)
{
	int ret = 0;
	uint16_t value = 0;

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)

	ret = gpio_expander_init();  //tca6408a power enable
	if (ret < 0) {
		ret = stmpe801_init();  // power enable
		if (ret < 0) {
			return ret;
		}
	}

	if (!isConfig) {
		isConfig = 1;
		s5k33dxx_dothin_config();
	}
#endif

	// First get project id. Different project,the sensor function may be different operation.
	if (s5k33d_project_id == 0) {
		s5k33dxx_get_project_id(&s5k33d_project_id);
		ALOGD("s5k33dxx_get_project_id: 0x%x", s5k33d_project_id);
	}

	ret = sensor_read_reg_16(0x0000, &value);
	if (ret < 0)
	{
		*id = 0xFFFF;
		return ret;
	}
	*id = value;

	return ret;
}

int s5k33dxx_get_chip_id(uint8_t *id)  // id is available only after streaming
{
	int ret;

	usleep(2000);  // sleep is needed

	ret = sensor_write_reg_16(0x0A02, 0x0000);  // select page 0
	ret = sensor_write_reg_16(0x0A00, 0x0100);  // NVM read enable
	// After this sequence, data is ready in bytes 0x0A04-0x0A43
	// 0A24-0A29 is chip id

	int i = 0;
	for (i = 0; i < 6; i++) {
		ret = sensor_read_reg_8(0x0A24 + i, &id[i]);
		ALOGE("chip id[%d]: %x", i, id[i]);
	}
	ret = sensor_write_reg_16(0x0A00, 0x0000);  // NVM read disable

	return ret;
}

/**
* @brief  s5k33dxx_group_hold setup grouped parameter hold
* @return int
*/
int s5k33dxx_group_hold(bool enable)
{
	int ret = 0;
	if (enable)
		ret = sensor_write_reg_8(0x0104, 0x01);
	else
		ret = sensor_write_reg_8(0x0104, 0x00);

	return ret;
}

int s5k33dxx_hardware_trigger()
{
	return 0;
}

int s5k33dxx_software_trigger()
{
	return 0;
}

/**
* @brief  s5k33dxx_video_streaming
* @param  [in] bool enable
* @return int
*/
int s5k33dxx_video_streaming(bool enable)
{
	int ret;

	if (enable) {
		if (s5k33d_project_id == S5K33D_HUANGSHAN) {
			ret = sensor_register_init();
		}

		if (start_stream_count == 0) {
			start_stream_count = 1;
		}
		else if (start_stream_count == 1) {  //The last time have update driver ic setting,now close init driver ic setting.
			ret = sensor_write_reg_16(0x602A, 0x1F90);
			ret = sensor_write_reg_16(0x6F12, 0x0000);

			start_stream_count = 2;
		}
		else {
			start_stream_count = 3;
		}

		ret = sensor_write_reg_8(0x0100, 0x01);
		start_streaming_called = 1;

		s5k33dxx_set_illum_duty_cycle(duty_cycle);

	}
	else {
		ret = sensor_write_reg_8(0x0100, 0x00);
		start_streaming_called = 0;
	}

	return ret;
}

int s5k33dxx_AE(bool enable)
{
	printf("--->s5k33dxx_AE  %d", enable);
	int ret = 0;

	ret = sensor_write_reg_8(0x0100, 0x00);
	if (enable) {
		ret = sensor_write_reg_16(0x602A, 0x2052);
		ret = sensor_write_reg_16(0x6F12, 0x0001);
	}
	else {
		ret = sensor_write_reg_16(0x602A, 0x2052);
		ret = sensor_write_reg_16(0x6F12, 0x0000);
	}

	if (start_streaming_called)
		ret = sensor_write_reg_8(0x0100, 0x01);

	return ret;
}

/**
* @brief  s5k33dxx_set_fps: set current fps  @param  [in] uint8_t fps
* @return int
*/
int s5k33dxx_set_fps(uint8_t fps)
{
	int ret;
#if 0
	uint16_t vt_pll_multi, vt_pre_pll_clk_div, vt_sys_clk_div, vt_pix_clk_div;
	uint16_t vt_pix_clk_freq, frame_length_lines, line_length_pck;

	ret = sensor_read_reg_16(0x0306, &vt_pll_multi);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0304, &vt_pre_pll_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0302, &vt_sys_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0300, &vt_pix_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0342, &line_length_pck);
	if (ret < 0) {
		return ret;
	}

	vt_pix_clk_freq = 24 * (vt_pll_multi * 4) / (vt_pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div);

	frame_length_lines = (uint16_t)((1000000 * vt_pix_clk_freq) / (line_length_pck * fps));

	ret = sensor_write_reg_16(0x0340, frame_length_lines);
#else
	set_fps_called = fps;
	if (fps == 60)
	{
		if (s5k33d_aa_qc_index == F201201_TX_AA_INDEX)
		{
			ret = sensor_write_reg_16(0x070E, 0x0C45);
		}
		else
		{
			return HW_ERR_NO_SUPPORT;
		}
	}
	else if (fps == 30)
	{
		ret = sensor_write_reg_16(0x070E, 0x188A);
	}
	else if (fps == 15)
	{
		ret = sensor_write_reg_16(0x070E, 0x3114);
	}
	else
	{
		ret = -2002;
	}

	ALOGE("set fps: %d", fps);
#endif
	return ret;
}

/**
* @brief  s5k33dxx_get_fps: get current fps
* @param  [out] uint8_t* fps
* @return int
*/
int s5k33dxx_get_fps(uint8_t* fps)
{
	int ret;
#if 0
	uint16_t vt_pll_multi, vt_pre_pll_clk_div, vt_sys_clk_div, vt_pix_clk_div;
	uint16_t vt_pix_clk_freq, frame_length_lines, line_length_pck;

	ret = sensor_read_reg_16(0x0306, &vt_pll_multi);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0304, &vt_pre_pll_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0302, &vt_sys_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0300, &vt_pix_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0340, &frame_length_lines);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0342, &line_length_pck);
	if (ret < 0) {
		return ret;
	}

	vt_pix_clk_freq = 24 * (vt_pll_multi * 4) / (vt_pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div);

	*fps = (uint8_t)((1000000 * vt_pix_clk_freq) / (frame_length_lines * line_length_pck));

#else
	uint16_t frame_timing_lines = 0;

	ret = sensor_read_reg_16(0x070E, &frame_timing_lines);

	if (frame_timing_lines == 0x0C45)
		*fps = 60;
	else if (frame_timing_lines == 0x188A)
		*fps = 30;
	else if (frame_timing_lines == 0x3114)
		*fps = 15;
	else
		*fps = 0;

	ALOGE("get fps: %d", *fps);
#endif
	return ret;
}

int s5k33dxx_get_rx_temp(float *temperature)
{
	int ret;
	uint16_t value;

	ret = sensor_read_reg_16(0x000A, &value);
	if (ret < 0)
		return ret;

	*temperature = (float)fixed2float(value, 16, 8, 1);

	return 0;
}

int s5k33dxx_get_tx_temp(float *temperature)
{
	int ret;
	uint16_t value;

	ret = sensor_read_reg_16(0x00F4, &value);
	if (ret < 0)
		return ret;

	*temperature = 25 + ((value - 296)) / 5.4f;

	return ret;
}

int s5k33dxx_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int ret = 0;

	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
    else {
        if (s5k33d_project_id == S5K33D_X210116) {
            ret = vcsel_driver_write_reg(0x0D, value_A);//3.5A
            ret = vcsel_driver_write_reg(0x09, 0x04); // current AS_W
            ret = vcsel_driver_write_reg(0x1E, 0x00); // current AS_I
            return ret;        
        }
    }

	//ALOGE("set illum power: %d = %d", value_A, value_B);
	ret = vcsel_driver_write_reg(0x08, value_A);

	// this is PHX3D_3021_AA config for DVT2
	if (driver_ic_type == DRIVER_IC_PHX3D_3021_AA) {
		static uint8_t value = 0xD0; // keep high 4 bit value

		if (value_A > 110 && value_A <= 130) { // different current need adjust light wave

			ret = vcsel_driver_write_reg(0x0F, 0x30);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x01));  // 0x10  TrAS
		}
		else if (value_A > 130 && value_A <= 147) {

			ret = vcsel_driver_write_reg(0x0F, 0x21);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x03));  // 0x10  TrAS
		}
		else if (value_A > 147 && value_A <= 170) {

			ret = vcsel_driver_write_reg(0x0F, 0x31);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x05));  // 0x10  TrAS
		}
		else if (value_A > 170 && value_A <= 185) {

			ret = vcsel_driver_write_reg(0x0F, 0x03);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x03));  // 0x10  TrAS
		}
		else if (value_A > 185 && value_A <= 197) {

			ret = vcsel_driver_write_reg(0x0F, 0x03);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x0B));  // 0x10  TrAS
		}
		else if (value_A > 197 && value_A <= 211) {

			ret = vcsel_driver_write_reg(0x0F, 0x04);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x0B));  // 0x10  TrAS
		}
	}
	// this is PHX3D_3021_CB config for DVT3
	else if (driver_ic_type == DRIVER_IC_PHX3D_3021_CB) {
		static uint8_t value = 0xC0; // keep high 4 bit value

		if (value_A > 110 && value_A <= 130) { // different current need adjust light wave

			ret = vcsel_driver_write_reg(0x0F, 0x70);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x00));  // 0x10  TrAS
		}
		else if (value_A > 130 && value_A <= 147) {

			ret = vcsel_driver_write_reg(0x0F, 0x90);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x00));  // 0x10  TrAS
		}
		else if (value_A > 147 && value_A <= 170) {

			ret = vcsel_driver_write_reg(0x0F, 0x62);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x04));  // 0x10  TrAS
		}
		else if (value_A > 170 && value_A <= 185) {

			ret = vcsel_driver_write_reg(0x0F, 0x54);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x01));  // 0x10  TrAS
		}
		else if (value_A > 185 && value_A <= 197) {
#ifdef PUTUOSHAN
			ret = vcsel_driver_write_reg(0x0F, 0x06);
			ret = vcsel_driver_write_reg(0x0F, ((value & 0xF0) | 0x07));
#else
			if (s5k33d_project_id == S5K33D_T200515) {

				ret = vcsel_driver_write_reg(0x0F, 0xA3);  // 0x0F  AS_W, AS_I
				ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x05));  // 0x10  TrAS
			}
			else {

				ret = vcsel_driver_write_reg(0x0F, 0x04);  // 0x0F  AS_W, AS_I
				ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x0C));  // 0x10  TrAS
			}
#endif
		}
		else if (value_A > 197 && value_A <= 211) {

			ret = vcsel_driver_write_reg(0x0F, 0x04);  // 0x0F  AS_W, AS_I
			ret = vcsel_driver_write_reg(0x10, ((value & 0xF0) | 0x0F));  // 0x10  TrAS
		}
	}
	else if (driver_ic_type == DRIVER_IC_DW9912) {

		ret = vcsel_driver_write_reg(0x0F, 0x65);  // 0x0F  AS_W, AS_I
		ret = vcsel_driver_write_reg(0x10, 0x02);  // 0x10  TrAS
	}

	return ret;
}

int s5k33dxx_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
    int ret = 0;

	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
    else {
        if (s5k33d_project_id == S5K33D_X210116) {
            ret = vcsel_driver_read_reg(0x0D, value_A);
            return ret;
        }
    }
	*vcsel_num = 1;
	ret = vcsel_driver_read_reg(0x08, value_A);

	if (ret < 0)
		return ret;
	*value_B = 0;

	return ret;
}

/**
* @brief  s5k33dxx_set_integration_time
* if integration time need to greater than 800us, register 0x0340 should increase
* @param  [in] uint16_t integrationTime (us)
* @return int
*/
int s5k33dxx_set_integration_time(uint16_t integrationTime)
{
	int ret;

	uint16_t vt_pll_multi, vt_pre_pll_clk_div, vt_sys_clk_div, vt_pix_clk_div;
	uint16_t vt_pix_clk_freq, coarse_integration_time, line_length_pck;

	ret = sensor_read_reg_16(0x0306, &vt_pll_multi);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0304, &vt_pre_pll_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0302, &vt_sys_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0300, &vt_pix_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0342, &line_length_pck);
	if (ret < 0) {
		return ret;
	}

	vt_pix_clk_freq = 24 * (vt_pll_multi * 4) / (vt_pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div);

	coarse_integration_time = (vt_pix_clk_freq * integrationTime) / line_length_pck;

	int pll_s = 0;
	ret = sensor_read_reg_16(0x030c, &pll_s);
	if (1 == pll_s) {
		coarse_integration_time = coarse_integration_time / 2;
	}
#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	if (integrationTime > 800 && coarse_integration_time > 0x93) {// mode3 max integration time is limited by VTS, 0x0490 => max_inte_time 800us,(0x0456 => max_inte_time 520us)
            uint8_t get_mode_param = 0;
            get_burst_mode(&get_mode_param);
            if (get_mode_param == 2) {
                ret = sensor_write_reg_16(0x0340, coarse_integration_time - 0x93 + 0x490);
            }
        }
#endif
	ret = sensor_write_reg_16(0x0202, coarse_integration_time);

	return ret;
}

/**
* @brief  s5k33dxx_get_integration_time: get current integration time
* @param  [out] uint16_t *integrationTime (us)
* @return int
*/
int s5k33dxx_get_integration_time(uint16_t *integrationTime)
{
	int ret;

	uint16_t vt_pll_multi, vt_pre_pll_clk_div, vt_sys_clk_div, vt_pix_clk_div;
	uint16_t vt_pix_clk_freq, coarse_integration_time, line_length_pck;

	ret = sensor_read_reg_16(0x0306, &vt_pll_multi);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0304, &vt_pre_pll_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0302, &vt_sys_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0300, &vt_pix_clk_div);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0342, &line_length_pck);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x0202, &coarse_integration_time);
	if (ret < 0) {
		return ret;
	}

	vt_pix_clk_freq = 24 * (vt_pll_multi * 4) / (vt_pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div);

	uint16_t integration_time = 0;
	integration_time = (coarse_integration_time * line_length_pck) / vt_pix_clk_freq;
	int pll_s = 0;
	ret = sensor_read_reg_16(0x030c, &pll_s);
	if (1 == pll_s) {
		integration_time = integration_time * 2;
	}
	*integrationTime = integration_time;

	return ret;
}

int s5k33dxx_get_vcsel_pd(uint32_t *value)
{
	int rtn = HW_ERR_NO_SUPPORT;
	uint16_t pd_h2, pd_bg;

	if (s5k33d_project_id == S5K33D_T200515) {
		if (driver_ic_type == 5017) {
			rtn = vcsel_driver_read_reg_16(0x1B20, &pd_h2);
			rtn = vcsel_driver_read_reg_16(0x1A1C, &pd_bg);

			pd_h2 &= 0x3ff;  // bit[9:0] is valid
			pd_bg &= 0x3ff;  // bit[9:0] is valid
			*value = (pd_h2 << 16) + pd_bg;  // bit[25:16] -> pd_h2
		}
	}

	return rtn;
}

int s5k33dxx_get_vcsel_error(uint16_t *value)
{
	int rtn = 0;

	if (driver_ic_type == 5017) {
		rtn = vcsel_driver_read_reg_16(0x2324, value);  // read reg 0x23,0x24
	}

	return rtn;
}

/* F201201 TX AA:2J & 3J ld vcc different,by control max7783 provide different voltages
*/
int s5k33dxx_set_ld_vcc(uint16_t value_mv)
{
	int rtn = 0;
	uint16_t value = 0;

	if ((s5k33d_aa_qc_index == F201201_TX_AA_INDEX) || (s5k33d_aa_qc_index == F201201_TX_QC_INDEX))
	{
		if (value_mv <= 4440)  // value_mv <= 4.44V
		{
			value = 0x3C;
		}
		else if (value_mv > 4440 && value_mv <= 9100)  // 4.44v < value_mv <= 9.1v
		{
			value = ((float)value_mv - 4440.0) / 72.8 + 61.0;
		}
		else  // value_mv > 9.1v
		{
			value = 0x7D;
		}

		if (value < 0x3C)
		{
			value = 0x3C;
		}
		else if (value > 0x7D)
		{
			value = 0x7D;
		}
	}
	else
	{
		ALOGE("s5k33dxx_set_ld_vcc: HW_ERR_NO_SUPPORT ");
		return HW_ERR_NO_SUPPORT;
	}

	rtn = i2c_reg_write(ST_MAX7783_ADDR, 0x13, 1, value, 1);

	ALOGE("s5k33dxx_set_ld_vcc: %d", value_mv);

	return rtn;
}

/* F201201 TX AA:2J & 3J ld vcc different,by control max7783 provide different voltages
*/
int s5k33dxx_get_ld_vcc(uint16_t *value_mv_p)
{
	int rtn = 0;
	uint8_t reg_value = 0;

	if ((s5k33d_aa_qc_index == F201201_TX_AA_INDEX) || (s5k33d_aa_qc_index == F201201_TX_QC_INDEX))
	{
		rtn = i2c_reg_read(ST_MAX7783_ADDR, 0x13, 1, &reg_value, 1);

		if (reg_value <= 0x3C)  // value_mv <= 4.44V
		{
			*value_mv_p = 4440;
		}
		else if (reg_value > 0x3D && reg_value <= 0x7D)  // 4.44v < value_mv <= 9.1v
		{
			*value_mv_p = (reg_value - 0x3D) * 72.8 + 4440;
		}
		else  // value_mv > 9.1v
		{
			*value_mv_p = 9100;
		}
	}
	else
	{
		ALOGE("s5k33dxx_get_ld_vcc: HW_ERR_NO_SUPPORT ");
		return HW_ERR_NO_SUPPORT;
	}

	ALOGE("s5k33dxx_get_ld_vcc: %d", *value_mv_p);

	return rtn;
}

int s5k33dxx_set_aa_qc_test(uint16_t index)
{
	int ret;

    ALOGE("s5k33dxx_set_aa_qc_test: in");

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	if (index == F201201_TX_AA_INDEX)  //TX AA
	{
		s5k33d_aa_qc_index = F201201_TX_AA_INDEX;

		ret = set_binning_mode(1);  // 2*2 binning

		ret += s5k33dxx_set_illum_duty_cycle(30);  // 70% duty cycle

		ret += s5k33dxx_set_modulation_frequency(0x6464);  // 100M + 100M

		ret += s5k33dxx_set_integration_time(800);

		ret += s5k33dxx_set_fps(60);  // Raw Phase fps: 240fps, infact 160fps

		ret += vcsel_driver_write_reg(0x23, 0);  // clear error

        ALOGE("s5k33dxx_set_aa_qc_test: TX_AA");
	}
	else if (index == F201201_TX_QC_INDEX)  //TX QC
	{
		s5k33d_aa_qc_index = F201201_TX_QC_INDEX;

		ret = s5k33dxx_set_modulation_frequency(0x0A0A);  // 10M + 10M

		ret += s5k33dxx_set_illum_duty_cycle(16);  // 50% duty cycle

		ret += s5k33dxx_set_integration_time(500);

		ret += sensor_write_reg_16(0x0340, 0x24D0);  // 1% frame duty cycle

        ALOGE("s5k33dxx_set_aa_qc_test: TX_QC");
	}
	else
	{
        ALOGE("s5k33dxx_set_aa_qc_test: HW_ERR_NO_SUPPORT");
		return HW_ERR_NO_SUPPORT;
	}
#endif

	return ret;
}

int s5k33dxx_get_aa_qc_test(uint16_t *index_p)
{
	int ret;

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	*index_p = s5k33d_aa_qc_index;
#endif

	return ret;
}

int s5k33dxx_get_modulation_frequency(uint16_t *modFreq)
{
	int ret;
	uint16_t freq_0, freq_1;
	uint16_t P, M, S, D;

	ret = sensor_read_reg_16(0x02F0, &P);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02F2, &M);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02F4, &S);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02F6, &D);
	if (ret < 0) {
		return ret;
	}
	freq_0 = 24 * M / (4 * P*D*pow(2, S));
	if (ret < 0) {
		return ret;
	}
	//printf("freq_0:%d, P:%x, M:%x, S:%x, D:%x \r\n", freq_0, P, M, S, D);

	ret = sensor_read_reg_16(0x02F8, &P);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02FA, &M);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02FC, &S);
	if (ret < 0) {
		return ret;
	}
	ret = sensor_read_reg_16(0x02FE, &D);
	if (ret < 0) {
		return ret;
	}
	freq_1 = 24 * M / (4 * P*D*pow(2, S));
	if (ret < 0) {
		return ret;
	}

	*modFreq = ((freq_0 << 8) | freq_1);

	return ret;
}

int s5k33dxx_set_modulation_frequency(uint16_t modFreq)
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
	int ret = 0;
	uint16_t freq_0, freq_1;

	freq_0 = (modFreq >> 8) & 0xFF;
	freq_1 = modFreq & 0xFF;


	ret = sensor_write_reg_8(0x0100, 0x00);  // should stop streaming first

	switch (freq_0) {  // PLL M can not set too large, so we change PLL S and D to get target frequency
	case 100:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0064);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0002);
		break;
	case 95:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x005F);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0002);
		break;
	case 80:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0050);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0002);
		break;
	case 70:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0046);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0002);
		break;
	case 60:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0078);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0004);
		break;
	case 20:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0050);
		ret = sensor_write_reg_16(0x02F4, 0x0000);
		ret = sensor_write_reg_16(0x02F6, 0x0008);
		break;
	case 15:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0078);
		ret = sensor_write_reg_16(0x02F4, 0x0002);
		ret = sensor_write_reg_16(0x02F6, 0x0004);
		break;
	case 10:
		ret = sensor_write_reg_16(0x02F0, 0x0003);
		ret = sensor_write_reg_16(0x02F2, 0x0050);
		ret = sensor_write_reg_16(0x02F4, 0x0001);
		ret = sensor_write_reg_16(0x02F6, 0x0008);
		break;
	default:
		break;
	}

	switch (freq_1) {
	case 100:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0064);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0002);
		break;
	case 95:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x005F);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0002);
		break;
	case 80:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0050);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0002);
		break;
	case 70:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0046);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0002);
		break;
	case 60:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0078);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0004);
		break;
	case 20:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0050);
		ret = sensor_write_reg_16(0x02FC, 0x0000);
		ret = sensor_write_reg_16(0x02FE, 0x0008);
		break;
	case 15:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0078);
		ret = sensor_write_reg_16(0x02FC, 0x0002);
		ret = sensor_write_reg_16(0x02FE, 0x0004);
		break;
	case 10:
		ret = sensor_write_reg_16(0x02F8, 0x0003);
		ret = sensor_write_reg_16(0x02FA, 0x0050);
		ret = sensor_write_reg_16(0x02FC, 0x0001);
		ret = sensor_write_reg_16(0x02FE, 0x0008);
		break;
	default:
		break;
	}

	if (start_streaming_called)
		ret = sensor_write_reg_8(0x0100, 0x01);

	return ret;
}

int s5k33dxx_get_illum_duty_cycle(uint16_t *duty)
{
	int ret;
	uint16_t inc_value = 0, dec_value = 0;

	if (start_streaming_called == 0)  //When have not start stream.
	{
		*duty = duty_cycle;
	}
	else
	{
		ret = sensor_read_reg_16(0xF4AC, &dec_value);
		if (ret < 0)
			return ret;

		ret = sensor_read_reg_16(0xF49C, &inc_value);
		if (ret < 0)
			return ret;

		if ((inc_value == 0) && (dec_value != 0))
		{
			*duty = (15 - dec_value);
		}
		else if ((inc_value != 0) && (dec_value == 0))
		{
			*duty = (15 + inc_value);
		}
		else if ((inc_value == 0) && (dec_value == 0))
		{
			*duty = 15;
		}
	}

	return ret;
}

int s5k33dxx_set_illum_duty_cycle(uint16_t duty)
{
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		return -HW_ERR_INVALID;
	}
	int ret = 0;
	uint16_t value = 0;
	uint16_t inc_value, dec_value;

	if (start_streaming_called == 0)  //When have not start stream.
	{
		duty_cycle = duty;
	}
	else
	{
		if (duty == 15)
		{
			ret = sensor_write_reg_16(0xF49C, 0x0000);
			if (ret < 0)
				return ret;
			ret = sensor_write_reg_16(0xF4AC, 0x0000);
			if (ret < 0)
				return ret;
		}
		else if (duty > 15)
		{
			value = (uint16_t)(duty - 15);
			ret = sensor_write_reg_16(0xF4AC, 0x0000);
			if (ret < 0)
				return ret;
			ret = sensor_write_reg_16(0xF49C, value);
			if (ret < 0)
				return ret;
		}
		else if (duty < 15)
		{
			value = (uint16_t)(15 - duty);
			ret = sensor_write_reg_16(0xF49C, 0x0000);
			if (ret < 0)
				return ret;
			ret = sensor_write_reg_16(0xF4AC, value);
			if (ret < 0)
				return ret;
		}
	}

	return ret;
}

/**
* @brief  s5k33dxx_set_img_mirror_flip 閻犱礁澧介悿鍞杄nsor闂傗偓濠婂啫鍓?
* @param  [in] uint8_t mode
* @return int
*/
int s5k33dxx_set_img_mirror_flip(uint8_t mode)
{
	int ret;

	switch (mode)
	{
	case IMAGE_NORMAL:
		ret = sensor_write_reg_8(0x0101, 0);
		break;
	case IMAGE_H_MIRROR:
		ret = sensor_write_reg_8(0x0101, 1);
		break;
	case IMAGE_V_MIRROR:
		ret = sensor_write_reg_8(0x0101, 2);
		break;
	case IMAGE_HV_MIRROR:
		ret = sensor_write_reg_8(0x0101, 3);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

/**
* @brief  s5k33dxx_get_img_mirror_flip 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呴梻鈧鍐ㄥ壖闁绘鍩栭埀?
* @param  [out] uint8_t* mode
* @return int
*/
int s5k33dxx_get_img_mirror_flip(uint8_t* mode)
{
	int ret;

	ret = sensor_read_reg_8(0x0101, mode);

	return ret;
}

/**
* @brief  s5k33dxx_test_pattern test pattern閻犱礁澧介悿?* @param  [in] uint8_t mode
* @return int
*/
int s5k33dxx_test_pattern(uint8_t mode)
{
	int ret;

	switch (mode)
	{
	case IMAGE_NO_PATTERN:
		ret = sensor_write_reg_16(0x0600, 0x0000);
		break;
	case IMAGE_SOLID_PATTERN:
		ret = sensor_write_reg_16(0x0600, 0x0301);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_16(0x0602, 0x0000);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_16(0x0604, 0x0000);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_16(0x0606, 0x0000);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_16(0x0608, 0x0000);
		break;
	case IMAGE_GRADIENT_PATTERN:
		ret = sensor_write_reg_16(0x0600, 0x0302);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_8(0x0615, 0x03);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_8(0x0616, 0x00);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_8(0x0617, 0x00);
		if (ret < 0) {
			break;
		}
		ret = sensor_write_reg_8(0x0618, 0x00);
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

int s5k33d_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
	float list[31] = {
		27, 28.4, 29.86, 31.32, 32.78, 34.24, 35.7, 37.16, 38.62, 40.08, 41.54, 43, 44.4, 45.8, 47.2, 48.7,  // 15
		50, 51.4, 52.7, 54.2, 55.66, 57.12, 58.58, 60.04, 61.5, 62.96, 64.42, 65.88, 67.34, 68.8, 70.5,  // 30
	};

	memcpy(duty_cycle_list, list, sizeof(list));
	return 0;
}

int s5k33dxx_get_sensor_info(struct sensor_info_t *info)
{
	uint8_t binning_mod = 0;
	int ret = get_binning_mode(&binning_mod);
	switch (binning_mod) {
	case 0: info->embedded_data_size = 1280 * 2; break;
	case 1: info->embedded_data_size = 640 * 2;; break;
	case 2: info->embedded_data_size = 320 * 2;; break;
	default:
		info->embedded_data_size = 1280 * 2;
	}

	info->vcsel_num = 1;
	info->vcsel_driver_id = driver_ic_type;
	info->sensor_id = s5k33dxx_sensor_id;
	info->project_id = s5k33d_project_id;
	return 0;
}

static int eeprom_read(uint8_t addr, uint16_t reg, uint16_t reg_size, uint32_t* data, uint16_t data_size)
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

static int eeprom_write(uint8_t addr, uint16_t reg, uint16_t reg_size, uint32_t* data, uint16_t data_size)
{
	int ret = 0;
	i2c_msg_t msg;

	msg.slave_addr = addr;
	msg.rw_mode = I2C_M_WT;
	msg.reg = reg;
	msg.reg_size = reg_size;
	msg.data = data;
	msg.data_size = data_size;

	ret = orbbec_i2c_writeread(&msg, 1);

	usleep(6000);

	return ret;
}

int fm24c128_eeprom_write_protect(uint8_t en)
{
	int ret = 0;
	uint8_t data = 0x00;
	uint8_t p_write = 0x0D;

	ret = eeprom_write(0xB0, FM24C128D_CC_WREN_REG, 2, &data, 1);
	if (ret < 0) {
		return ret;
	}

	p_write |= (en << 1);

	ret = eeprom_write(0xB0, FM24C128D_CSP_CONFIG_REG, 2, &p_write, 1);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

int fm24c128d_eeprom_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

	uint8_t cnt = 0;
	uint8_t blocksize = FM24C128D_EEPROM_PAGE_SIZE;
	uint32_t addr = offset;
	uint8_t *buf_ptr_r = buf;
	uint8_t prog_time = size / blocksize;
	uint8_t lastsize = size % blocksize;
	uint32_t addr_offset = addr % blocksize;
	uint8_t  read_out_data[FM24C128D_EEPROM_PAGE_SIZE] = { 0 };

	ret = fm24c128_eeprom_write_protect(0);
	if (ret < 0) {
		return ret;
	}

	if (0 == addr_offset)  // write at page head
	{
		for (cnt = 0; cnt < prog_time; cnt++)
		{
			ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
			addr += blocksize;
			buf_ptr_r += blocksize;
		}

		if (lastsize != 0)
		{
			ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
		}
	}
	else  // write not at page head
	{
		ret = fm24c128d_eeprom_read((addr - addr_offset), read_out_data, addr_offset);  // read the head data of a page
		if ((blocksize - addr_offset) > size)  // write in a page
		{
			ret = eeprom_write(FM24C128D_DATA_MEMORY, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, size);
		}
		else  // write over a page
		{   // write not at head of page
			ret = eeprom_write(FM24C128D_DATA_MEMORY, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, (blocksize - addr_offset));
			addr += (blocksize - addr_offset);
			buf_ptr_r += (blocksize - addr_offset);
			// write at head of page
			prog_time = (size - blocksize + addr_offset) / blocksize;
			lastsize = (size - blocksize + addr_offset) % blocksize;
			for (cnt = 0; cnt < prog_time; cnt++)
			{
				ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
				addr += blocksize;
				buf_ptr_r += blocksize;
			}

			if (lastsize != 0)
			{
				ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
			}
		}
	}

	ret = fm24c128_eeprom_write_protect(1);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

int fm24c128d_eeprom_read(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

	uint8_t cnt;
	uint8_t blocksize = FM24C128D_EEPROM_PAGE_SIZE;
	uint8_t prog_time = size / blocksize;
	uint8_t lastsize = size % blocksize;
	uint8_t *buf_ptr_r = buf;
	uint32_t addr = offset;

	for (cnt = 0; cnt< prog_time; cnt++)
	{
		ret = eeprom_read(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
		addr += blocksize;
		buf_ptr_r += blocksize;
	}

	if (lastsize != 0)
	{
		ret = eeprom_read(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
	}

	return ret;
}

int gt24c512b_eeprom_block_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

#if 0
	ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, offset, 2, buf, size);

	Sleep(5);
#else
	uint16_t blocksize = GT24C512B_EEPROM_PAGE_SIZE;
#if 0
	if ((offset % blocksize) != 0)  // start addr should at the beginning of the block
		return -2002;
#endif
	uint16_t cnt = 0;
	uint32_t addr = offset;
	uint8_t  *buf_ptr_r = buf;
	uint16_t prog_time = size / blocksize;
	uint16_t lastsize = size % blocksize;
	uint32_t addr_offset = addr % blocksize;
	uint8_t  read_out_data[GT24C512B_EEPROM_PAGE_SIZE] = { 0 };
	if (0 == addr_offset)// write at page head
	{
		for (cnt = 0; cnt < prog_time; cnt++)
		{
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
			addr += blocksize;
			buf_ptr_r += blocksize;
		}

		if (lastsize != 0)
		{
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
		}
	}
	else  // write not at page head
	{
		ret = gt24c512b_eeprom_block_read((addr - addr_offset), read_out_data, addr_offset);  // read the head data of a page
		if ((blocksize - addr_offset) > size) {  // write in a page
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, size);
		}
		else  // write over a page
		{   // write not at head of page
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, (blocksize - addr_offset));
			addr += (blocksize - addr_offset);
			buf_ptr_r += (blocksize - addr_offset);
			// write at head of page
			prog_time = (size - blocksize + addr_offset) / blocksize;
			lastsize = (size - blocksize + addr_offset) % blocksize;
			for (cnt = 0; cnt < prog_time; cnt++)
			{
				ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
				addr += blocksize;
				buf_ptr_r += blocksize;
			}

			if (lastsize != 0)
			{
				ret = eeprom_write(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
			}
		}
	}
#endif
	return ret;
}

int gt24c512b_eeprom_block_read(uint16_t offset, uint8_t *buf, uint32_t size)
{
	int ret = 0;
#if 0
	ret = eeprom_read(GT24C512B_EEPROM_I2C_ADDR, offset, 2, buf, size);
	Sleep(5);
#else
	uint16_t cnt;
	uint16_t blocksize = GT24C512B_EEPROM_PAGE_SIZE;
	uint16_t prog_time = size / blocksize;
	uint16_t lastsize = size % blocksize;
	uint8_t  *buf_ptr_r = buf;
	uint32_t addr = offset;

	for (cnt = 0; cnt < prog_time; cnt++)
	{
		ret = eeprom_read(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
		addr += blocksize;
		buf_ptr_r += blocksize;
	}

	if (lastsize != 0)
	{
		ret = eeprom_read(GT24C512B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
	}
#endif
	return ret;
}

int gt24p256b_eeprom_block_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

#if 0
	ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, offset, 2, buf, size);

	Sleep(5);
#else
	uint16_t blocksize = GT24P256B_EEPROM_PAGE_SIZE;
#if 0
	if ((offset % blocksize) != 0)  // start addr should at the beginning of the block
		return -2002;
#endif
	uint16_t cnt = 0;
	uint32_t addr = offset;
	uint8_t  *buf_ptr_r = buf;
	uint16_t prog_time = size / blocksize;
	uint16_t lastsize = size % blocksize;
	uint32_t addr_offset = addr % blocksize;
	uint8_t  read_out_data[GT24P256B_EEPROM_PAGE_SIZE] = { 0 };

	if (0 == addr_offset)  // write at page head
	{
		for (cnt = 0; cnt < prog_time; cnt++)
		{
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
			addr += blocksize;
			buf_ptr_r += blocksize;
		}

		if (lastsize != 0)
		{
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
		}
	}
	else  // write not at page head
	{
		ret = gt24p256b_eeprom_block_read((addr - addr_offset), read_out_data, addr_offset);  // read the head data of a page
		if ((blocksize - addr_offset) > size) {  // write in a page
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, size);
		}
		else  // write over a page
		{   // write not at head of page
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
			ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, (blocksize - addr_offset));
			addr += (blocksize - addr_offset);
			buf_ptr_r += (blocksize - addr_offset);
			// write at head of page
			prog_time = (size - blocksize + addr_offset) / blocksize;
			lastsize = (size - blocksize + addr_offset) % blocksize;
			for (cnt = 0; cnt < prog_time; cnt++)
			{
				ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
				addr += blocksize;
				buf_ptr_r += blocksize;
			}

			if (lastsize != 0)
			{
				ret = eeprom_write(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
			}
		}
	}
#endif
	return ret;
}
int gt24p256b_eeprom_block_read(uint16_t offset, uint8_t *buf, uint32_t size)
{
	int ret = 0;
#if 0
	ret = eeprom_read(GT24P256B_EEPROM_I2C_ADDR, offset, 2, buf, size);
	Sleep(5);
#else
	uint16_t cnt;
	uint16_t blocksize = GT24P256B_EEPROM_PAGE_SIZE;
	uint16_t prog_time = size / blocksize;
	uint16_t lastsize = size % blocksize;
	uint8_t * buf_ptr_r = buf;
	uint32_t addr = offset;

	for (cnt = 0; cnt < prog_time; cnt++)
	{
		ret = eeprom_read(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
		addr += blocksize;
		buf_ptr_r += blocksize;
	}

	if (lastsize != 0)
	{
		ret = eeprom_read(GT24P256B_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
	}
#endif
	return ret;
}

static int eeprom_block_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;
	if (s5k33d_project_id == S5K33D_TAISHAN) {
		ret = fm24c128d_eeprom_write(offset, buf, size);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		ret = gt24c512b_eeprom_block_write(offset, buf, size);
	}
	else if (s5k33d_project_id == S5K33D_T200515) {
		ret = gt24p256b_eeprom_block_write(offset, buf, size);
	}
	else
		ret = -HW_ERR_NULL;
	return ret;
}

static int eeprom_block_read(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;
	if (s5k33d_project_id == S5K33D_TAISHAN) {
		ret = fm24c128d_eeprom_read(offset, buf, size);
	}
	else if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		ret = gt24c512b_eeprom_block_read(offset, buf, size);
	}
	else if (s5k33d_project_id == S5K33D_T200515) {
		ret = gt24p256b_eeprom_block_read(offset, buf, size);
	}
	else
		ret = -HW_ERR_NULL;
	return ret;
}

int s5k33dxx_get_tof_sensor_resolution(uint64_t *resolution)
{
	int ret;
	uint8_t mode = 0;
	*resolution = 0;
	if (s5k33d_project_id == S5K33D_HUANGSHAN) {
		*resolution = DUAL_FREQ_NO_BINNING_RESOLUTION;
	}
	else {
		*resolution = DUAL_FREQ_4X4_BINNING_RESOLUTION << 16 | DUAL_FREQ_2X2_BINNING_RESOLUTION << 8 | DUAL_FREQ_NO_BINNING_RESOLUTION;
	}

	return ret;
}

int s5k33dxx_get_tof_sensor_pixel_bit(uint8_t *pixel_bit)
{
	*pixel_bit = PIXEL_BIT;
	return 0;
}

int s5k33dxx_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
	*mipi_pack_bit = MIPI_PACK_BIT;
	return 0;
}

int s5k33dxx_func_init()
{
	memset(&tof_sensor, 0, sizeof(tof_sensor));
#if (USE_WHICH_CONVERTER == kConverterIsCx3)
	s5k33d_power_up();
	tof_sensor.init = s5k33dxx_sensor_init;
	tof_sensor.get_sensor_id = s5k33dxx_get_sensor_id;
	tof_sensor.video_streaming = s5k33dxx_video_streaming;
	tof_sensor.get_rx_temp = s5k33dxx_get_rx_temp;
	tof_sensor.get_tx_temp = s5k33dxx_get_tx_temp;
	tof_sensor.set_illum_power = s5k33dxx_set_illum_power;
	tof_sensor.get_illum_power = s5k33dxx_get_illum_power;
	tof_sensor.get_integration_time = s5k33dxx_get_integration_time;
	tof_sensor.set_integration_time = s5k33dxx_set_integration_time;
	tof_sensor.get_modulation_frequency = s5k33dxx_get_modulation_frequency;
	tof_sensor.set_modulation_frequency = s5k33dxx_set_modulation_frequency;
	tof_sensor.get_illum_duty_cycle = s5k33dxx_get_illum_duty_cycle;
	tof_sensor.set_illum_duty_cycle = s5k33dxx_set_illum_duty_cycle;
	tof_sensor.get_img_mirror_flip = s5k33dxx_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = s5k33dxx_set_img_mirror_flip;
	tof_sensor.test_pattern = s5k33dxx_test_pattern;
	tof_sensor.get_sensor_info = s5k33dxx_get_sensor_info;
	tof_sensor.get_illum_duty_cycle_list = s5k33d_get_illum_duty_cycle_list;
	tof_sensor.AE = s5k33dxx_AE;
	tof_sensor.driver_ic_detect = s5k33dxx_driver_ic_detect;
	tof_sensor.eeprom_read = fm24c128d_eeprom_read;
	tof_sensor.eeprom_write = fm24c128d_eeprom_write;
#endif

#if (USE_WHICH_CONVERTER == kConverterIsDuxin)
	tof_sensor.init = s5k33dxx_sensor_init;
	tof_sensor.get_chip_id = s5k33dxx_get_chip_id;
	tof_sensor.get_sensor_id = s5k33dxx_get_sensor_id;
	tof_sensor.get_sensor_info = s5k33dxx_get_sensor_info;
	tof_sensor.video_streaming = s5k33dxx_video_streaming;
	tof_sensor.get_rx_temp = s5k33dxx_get_rx_temp;
	tof_sensor.get_tx_temp = s5k33dxx_get_tx_temp;
	tof_sensor.set_integration_time = s5k33dxx_set_integration_time;
	tof_sensor.get_integration_time = s5k33dxx_get_integration_time;
	tof_sensor.get_vcsel_pd = s5k33dxx_get_vcsel_pd;
	tof_sensor.get_vcsel_error = s5k33dxx_get_vcsel_error;
	tof_sensor.set_ld_vcc = s5k33dxx_set_ld_vcc;
	tof_sensor.get_ld_vcc = s5k33dxx_get_ld_vcc;
	tof_sensor.set_aa_qc_test = s5k33dxx_set_aa_qc_test;
	tof_sensor.get_aa_qc_test = s5k33dxx_get_aa_qc_test;
    
	tof_sensor.sensor_write_reg_16 = sensor_write_reg_16;
	tof_sensor.sensor_read_reg_16 = sensor_read_reg_16;
	tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;
	tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;

	tof_sensor.hardware_trigger = s5k33dxx_hardware_trigger;
	tof_sensor.software_trigger = s5k33dxx_software_trigger;
	tof_sensor.get_fps = s5k33dxx_get_fps;
	tof_sensor.set_fps = s5k33dxx_set_fps;
	tof_sensor.set_illum_power = s5k33dxx_set_illum_power;
	tof_sensor.get_illum_power = s5k33dxx_get_illum_power;
    
	tof_sensor.get_modulation_frequency = s5k33dxx_get_modulation_frequency;
	tof_sensor.set_modulation_frequency = s5k33dxx_set_modulation_frequency;
	tof_sensor.get_illum_duty_cycle = s5k33dxx_get_illum_duty_cycle;
	tof_sensor.set_illum_duty_cycle = s5k33dxx_set_illum_duty_cycle;
	tof_sensor.get_img_mirror_flip = s5k33dxx_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = s5k33dxx_set_img_mirror_flip;
	tof_sensor.test_pattern = s5k33dxx_test_pattern;
	tof_sensor.get_illum_duty_cycle_list = s5k33d_get_illum_duty_cycle_list;
	tof_sensor.AE = s5k33dxx_AE;
	tof_sensor.driver_ic_detect = s5k33dxx_driver_ic_detect;
	tof_sensor.set_tx_a_b_power = set_expander_tx_gpio;
	tof_sensor.get_tx_a_b_power = get_expander_tx_gpio;
	tof_sensor.set_shuffle_mode = set_shuffle_mode;
	tof_sensor.get_shuffle_mode = get_shuffle_mode;
	tof_sensor.set_binning_mode = set_binning_mode;
	tof_sensor.get_binning_mode = get_binning_mode;
	tof_sensor.set_burst_mode = set_burst_mode;
	tof_sensor.get_burst_mode = get_burst_mode;
	tof_sensor.get_frequency_mode = s5k33d_get_frequency_mode;
	tof_sensor.set_frequency_mode = s5k33d_set_frequency_mode;

	tof_sensor.illum_power_control = tx_illum_power_control;
	tof_sensor.eeprom_read = eeprom_block_read;
	tof_sensor.eeprom_write = eeprom_block_write;
	tof_sensor.get_tof_sensor_resolution = s5k33dxx_get_tof_sensor_resolution;
	tof_sensor.get_tof_sensor_pixel_bit = s5k33dxx_get_tof_sensor_pixel_bit;
	tof_sensor.get_mipi_pack_bit = s5k33dxx_get_mipi_pack_bit;

#endif

	return 0;
}
