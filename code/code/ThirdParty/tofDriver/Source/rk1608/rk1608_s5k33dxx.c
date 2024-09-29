#include <stdio.h>
#include <rk1608_s5k33dxx.h>
#include <math.h>

#include "spi\inc\spi2apb.h"
#include "spi\inc\isp-fw.h"
#include "spi\inc\msg-queue.h"
#include "spi\inc\msg-interface.h"

#include <tof_sensors.h>
#include <hw_modules.h>
#include <hw_obstatus.h>
#include "debug2log.h"
#include "hw_property.h"

#ifdef __linux__
#include <unistd.h>
#endif

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

struct reglist {
	uint16_t reg;
	uint16_t val;
};
#include "rk1608_s5k33d.c"
#include "rk1608_s5k33d_dual_freq.c"

#define malloc					   tops_t.qsee_malloc
#define dothin_device_id           tops_t.ap_ops.device_id
#define dothin_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel
#define dothin_i2c_writeread       tops_t.ops_writeread
#define dothin_spi_writeread       tops_t.ap_ops.SensorSpiRWEx
#define usleep					   tops_t.tee_usleep
#define orbbec_i2c_writeread       tops_t.ops_writeread
#define dothin_set_soft_pin        tops_t.ap_ops.SetSoftPin
#define dothin_enable_soft_pin     tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_soft_pin_pullup tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_set_sensor_enable   tops_t.ap_ops.SensorEnable
#define dothin_master_spi_config   tops_t.ap_ops.MasterSpiConfig
#define dothin_write_sensor_reg    tops_t.ap_ops.WriteSensorReg

#define BOOT_FROM_FLASH			   0
#define BOOT_FROM_DDR			   1
#define I2C_M_RD				   1
#define I2C_M_WT				   0
#define I2C_400K				   1
#define I2C_100K				   0

#ifndef FALSE
#define FALSE                      0
#endif

#ifndef TRUE
#define TRUE                       1
#endif

#define DT_ERROR_OK				   1	///<操作成功
#define DT_ERROR_FAILED			   0	///<操作失败
#define STATUS_OK				   0    /*!<成功 */
#define STATUS_ERROR			   3001 // !<度信操作失败
#define IO_PULLUP				   1
#define IO_NOPULL				   0

// vcsel driver IC type
#define DRIVER_IC_CXA4026          4026 // Polaris B Tx
#define DRIVER_IC_CXA4016          4016 // Taishan DVT1 Tx
#define DRIVER_IC_PHX3D_3021_AA    5016 // Taishan DVT2 Tx
#define DRIVER_IC_PHX3D_3021_CB    5017 // Taishan DVT3 Tx
#define DRIVER_IC_DW9912           9912 // DongWoon

#define DUAL_FREQ_NO_BINNING_RESOLUTION         RESOLUTION_1280_3840
#define DUAL_FREQ_2X2_BINNING_RESOLUTION        0
#define DUAL_FREQ_4X4_BINNING_RESOLUTION        0

#define SINGLE_FREQ_NO_BINNING_RESOLUTION       0
#define SINGLE_FREQ_2X2_BINNING_RESOLUTION      0
#define SINGLE_FREQ_4X4_BINNING_RESOLUTION      0

#define PIXEL_BIT       10
#define MIPI_PACK_BIT   8

///I2C模式定义。
enum I2CMODE
{
	I2CMODE_NORMAL = 0,		///< 8 bit addr,8 bit value 
	I2CMODE_SAMSUNG,		///< 8 bit addr,8 bit value,Stopen
	I2CMODE_MICRON,			///< 8 bit addr,16 bit value
	I2CMODE_STMICRO,		///< 16 bit addr,8 bit value, (eeprom also)
	I2CMODE_MICRON2,		///< 16 bit addr,16 bit value
};
/* SPI配置结构体 */
typedef struct MasterSpiConfig_s
{
	double  fMhz;               ///< 配置SPI的时钟
	unsigned char  byWordLen;          ///< Word length in bits. 0： 8bit ；1：16bit（暂时无效废弃），默认为0
	unsigned char  byCtrl;             ///< 支持的位控制码：MASTER_CTRL_DATA_SHIFT/MASTER_CTRL_CPOL/ MASTER_CTRL_CPHA/MASTER_CTRL_DELAY
	unsigned char  Rsv[64];            ///< 保留 */
}MasterSpiConfig_t;

struct regList {
	uint16_t reg;
	uint16_t val;
};

#define CHECK_DOTHIN_STATUS(code)    \
do {    \
      if (DT_ERROR_OK != code)          \
	  {                                 \
	   code = (code == DT_ERROR_FAILED) ? -STATUS_ERROR : code;  \
	   /*ALOGE("dothin access error code: %d\n",   code);*/      \
	  return code;                                               \
	  }                                 \
	  code = STATUS_OK;                \
}while (0)

static struct sensor_info_t get_src_info;
static uint8_t rk1608_s5k33d_start_streaming = 0;

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

	ret = dothin_i2c_writeread((uint8_t*) &msg, 1);

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

	ret = dothin_i2c_writeread((uint8_t*) &msg, 1);

	return ret;
}

static int sensor_write_reg_8(uint16_t reg, uint8_t value)
{
    return i2c_reg_write(S5K33D_I2C_ADDR, reg, 2, value, 1);
}

static int sensor_read_reg_8(uint16_t reg, uint8_t *value)
{
    return i2c_reg_read(S5K33D_I2C_ADDR, reg, 2, (uint32_t*) value, 1);
}

static int sensor_write_reg_16(uint16_t reg, uint16_t value)
{
    if (reg == 0xffff)
    {
        usleep(value);
        return 0;
    }
	int ret = i2c_reg_write(S5K33D_I2C_ADDR, reg, 2, value, 2);

	return ret;
}

static int sensor_read_reg_16(uint16_t reg, uint16_t *value)
{
	int ret = i2c_reg_read(S5K33D_I2C_ADDR, reg, 2, (uint32_t*) value, 2);

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

void rk1608_s5k33dxx_set_phase_src_info()
{
	get_src_info.embedded_data_size = 1280 * 2 * 10 / 8;
}

void rk1608_s5k33dxx_set_depth_src_info()
{
	get_src_info.embedded_data_size = 640 * 2 * 2;
}

/*
*@brief:  获取需要下载rk1608的固件，目录为执行同目录下的config文件夹中
@param[in] fw_type：选择是下载到外部flash还是内部DDR
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int rk1608_s5k33dxx_get_firmware(uint8_t fw_type, const uint8_t **get_fireware_data)
{
	FILE *fp = NULL;
	int file_len = 0;
	char *file_name = NULL;
	const uint8_t *fireware_data = NULL;

	if (fw_type == BOOT_FROM_DDR)
		file_name = ".\\config\\firmware\\kunlunshan\\preisp_33d.rkl";
	else if (fw_type == BOOT_FROM_FLASH)
		file_name = ".\\config\\firmware\\kunlunshan\\preisp_spi_boot.bin";
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
	fireware_data = (uint8_t*) malloc(sizeof(char)*(file_len + 1));
	if (NULL == fireware_data)
	{
		ALOGE("%s:get firmware memory fail", __FUNCTION__);
		fclose(fp);
		return -HW_ERR_NO_MEM;
	}
	memset((void*) fireware_data, 0, file_len + 1);
	fread((void*) fireware_data, file_len + 1, 1, fp);
	*get_fireware_data = fireware_data;
	
	fclose(fp);
	return HW_NO_ERR;
}

/*
*@brief:  发送数据到rk1608（dsp）,配置相应参数和算法,让depth输出.
@param[in] m：发送的数据结构体
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数

@IR mode (CAM_IR_STREAM)
         640(RAW16)=1280(RAW8)
           -------------
EMB  2    |             |
           -------------
          |             |
IR    480 |             |
           -------------

@Depth mode (CAM_DEPTH_STREAM)
         640(RAW16)=1280(RAW8)
           -------------
 EMB  2   |             |
           -------------
          |             |
Depth 480 |             |
           -------------

@Depth + phase mode (CAM_PHASE_AND_DEPTH_STREAM)
@single	640(RAW16)=1280(RAW8)
           -------------
EMB  2    |             |
	       -------------
	      |             |
depth 480 |             |
	       -------------
	      |             |
raw 1200  |             |
	      |             |
           -------------

@dual    640(RAW16)=1280(RAW8)
           -------------
EMB_L  2  |             |
	       -------------
	      |             |
rawL 1200 |             |
	      |             |
           -------------
           -------------
EMB_H  2  |             |
           -------------
	      |             |
rawH 1200 |             |
	      |             |
           -------------
          |             |
depth 480 |             |
           -------------

@Depth + IR mode (CAM_IR_AND_DEPTH_STREAM)
         640(RAW16)=1280(RAW8)
           -------------
EMB  2    |             |
           -------------
          |             |
depth 480 |             |
           -------------
          |             |
IR   480  |             |
           -------------
*/
int set_rk1608_s5k33dxx_depth_out()
{
	int8_t cam_id = 0;
	int8_t in_mipi = 0;
	int8_t out_mipi = 0;
	uint16_t in_width = 1280;
	uint16_t in_height = 962;
	uint16_t out_width = 1280;//640 * uint16 / RAW8
#if (RK1608_S5K33D_FREQ_MODE == 0)
	uint16_t out_height = 1682;// 480 + 2;//1280*2*Raw10/8=3200Byte=640*5 line 5line/2=>2
    uint16_t vtotal = 0x0A41;
    uint8_t stream_in_type = SINGLE_FREQ_DISABLE_SHUFFLE;
    uint8_t sync_freq = 25;
#elif (RK1608_S5K33D_FREQ_MODE == 1)
    uint16_t out_height = 2884;//2882;// 480 + 2;//1280*2*Raw10/8=3200Byte=640*5 line 5line/2=>2
    uint16_t vtotal = 0x0C41;
    uint8_t stream_in_type = DUAL_FREQ_NONSHUFFLE;//DUAL_FREQ_DISABLE_SHUFFLE;
    uint8_t sync_freq = 6;
#endif
	uint16_t htotal = 0x0A18;
	uint32_t mipi_clock = 1500000000;
	char* sensor_name = "sc132gs";
	uint8_t i2c_slave_addr = 0x20;
	uint8_t i2c_bus = 0;
    uint8_t master_slave_mode = SLAVE;
    uint8_t stream_out_type = CAM_PHASE_AND_DEPTH_STREAM;
    uint8_t skip_frame = 15;
    uint32_t init_enable_param = 1;
	int ret = -1;

	ret = cmd_msg_init_sensor(cam_id, in_mipi, out_mipi, sensor_name, i2c_slave_addr, i2c_bus);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_init_sensor fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	usleep(20000);
	ret = cmd_msg_set_input_size(cam_id, in_width, in_height);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_input_size fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = cmd_msg_set_output_size(cam_id, out_width, out_height, htotal, vtotal, mipi_clock);
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

    ret = cmd_msg_set_param_init(cam_id, init_enable_param);
    if (ret < 0)
    {
        ALOGE("%s cmd_msg_set_param_init fail", __FUNCTION__);
        return -HW_ERR_NO_SUPPORT;
    }

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

/*
*@brief:  发送数据到rk1608（dsp）,配置相应参数和算法,让raw phase输出.
@param[in] m：发送的数据结构体
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int set_rk1608_s5k33dxx_raw_phase_out()
{
	int8_t cam_id = 0;
	int8_t in_mipi = 0;
	int8_t out_mipi = 0;
	uint16_t in_width = 1280;
	uint16_t in_height = 962;
	uint16_t out_width = 1280;
	uint16_t out_height = 1682;
	uint16_t htotal = 0x0A18;
	uint16_t vtotal = 0x0E82;
	uint32_t mipi_clock = 1500000000;
	char* sensor_name = "sc132gs";
	uint8_t i2c_slave_addr = 0x20;
	uint8_t i2c_bus = 0;
    uint8_t master_slave_mode = SLAVE;
#if (RK1608_S5K33D_FREQ_MODE == 0)
    uint8_t stream_in_type = SINGLE_FREQ_DISABLE_SHUFFLE;
#elif (RK1608_S5K33D_FREQ_MODE == 1)
    uint8_t stream_in_type = DUAL_FREQ_NONSHUFFLE;// DUAL_FREQ_DISABLE_SHUFFLE;
#endif
    uint8_t sync_freq = 30;
    uint8_t stream_out_type = CAM_PHASE_STREAM;
    uint8_t skip_frame = 10;
	int ret = -1;

	ret = cmd_msg_init_sensor(cam_id, in_mipi, out_mipi, sensor_name, i2c_slave_addr, i2c_bus);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_init_sensor fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	usleep(20000);
	ret = cmd_msg_set_input_size(cam_id, in_width, in_height);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_input_size fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = cmd_msg_set_output_size(cam_id, out_width, out_height, htotal, vtotal, mipi_clock);
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

static struct regList phx3d_3021_cb_reglist[] = {
	// for PHX3D driver IC setting
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
	{ 0x6F12,0xD324 },// 6>     0x43->1A    0xD3->3.8A
	{ 0x6F12,0x0DFF },// 7>
	{ 0x6F12,0xD900 },// 8>
	{ 0x6F12,0x0F04 },// 9>
	{ 0x6F12,0xCFFF },// 10>
	{ 0x6F12,0x19F1 },// 11>
	{ 0x6F12,0xF803 },// 12>
	{ 0x6F12,0x1300 },// 13>
};

static int driver_ic_detect()
{
	int ret = 0;

	if (get_src_info.vcsel_driver_id == DRIVER_IC_PHX3D_3021_CB || get_src_info.vcsel_driver_id == 0) {
		for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {
			ret = sensor_write_reg_16(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);

		}
	}

	return ret;
}

/**
* @brief  rk1608_s5k33dxx_sensor_init
* @return int
*/
int rk1608_s5k33dxx_sensor_init()
{
	int ret = 0;
#if (RK1608_S5K33D_FREQ_MODE == 0)
	for (int i = 0; i < sizeof(rk1608_s5k33d_reglist) / sizeof(struct reglist); i++) {
		ret = sensor_write_reg_16(rk1608_s5k33d_reglist[i].reg, rk1608_s5k33d_reglist[i].val);
	}
#elif (RK1608_S5K33D_FREQ_MODE == 1)
    for (int i = 0; i < sizeof(rk1608_s5k33d_dual_freq_reglist) / sizeof(struct reglist); i++) {
        ret = sensor_write_reg_16(rk1608_s5k33d_dual_freq_reglist[i].reg, rk1608_s5k33d_dual_freq_reglist[i].val);
    }
#endif

    get_src_info.embedded_data_size = 1280 * 2 * 10 / 8;//现有rk1608板上33d兼容平台端配置，统一采用EBD packed
    get_src_info.sensor_id = RK1608_S5K33D_SENSOR_ID;
    if (get_src_info.vcsel_driver_id == 0)
        get_src_info.vcsel_driver_id = DRIVER_IC_PHX3D_3021_CB;

    get_src_info.vcsel_num = 1;
    get_src_info.sensor_id = RK1608_S5K33D_SENSOR_ID;
    get_src_info.project_id = 0;

	ret |= driver_ic_detect();

	return ret;
}

/** 2s call this
* @brief  rk1608_s5k33dxx_hardware_trigger
* @return int
*/
static int rk1608_s5k33dxx_hardware_trigger()
{
	return 0;
}

/**
* @brief  rk1608_s5k33dxx_software_trigger
* @return int
*/
static int rk1608_s5k33dxx_software_trigger()
{
	ALOGD("%s success", __FUNCTION__);
	return 0;
}

/**
* @brief  rk1608_s5k33dxx_get_sensor_id
* @param  [out] uint16_t *id
* @return int
*/
static int rk1608_s5k33dxx_get_sensor_id(uint16_t *id)
{
	int ret = 0;
	uint16_t value = 0;

	ret = sensor_read_reg_16(0x0000, &value);
	if (ret < 0)
	{
		*id = 0xFFFF;
		return ret;
	}

	*id = (value | 0x100);//更改ID(0x313D),用于核度信+33D板载情况进行区分。

	return ret;
}

/*
*@brief:  初始化rk1608的板上33d获取sensor信息接口,先读取原接口数据后修改赋值
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_s5k33dxx_get_sensor_info(struct sensor_info_t *info)
{
    memcpy(info, &get_src_info, sizeof(get_src_info));

    return 0;
}

/**
* @brief  rk1608_s5k33dxx_download_ref
* @param  [in] rk1608_store_addr:the addr of rk1608 store;ref_data_addr:the buffer addr of ref;ref_data_len:the buffer len of ref
* @return int
*/
static int rk1608_s5k33dxx_download_ref(uint32_t rk1608_store_addr, uint32_t *ref_data_addr, uint32_t ref_data_len)
{
	return spi2apb_safe_write(rk1608_store_addr, (int32_t *) ref_data_addr, ref_data_len);
}

/*
*@brief:  重写get_frequency_mode() function。
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_s5k33dxx_get_frequency_mode(uint8_t *mode)
{
	ALOGE("frequency_mode enter");
#if (RK1608_S5K33D_FREQ_MODE == 0)
		*mode = 0; // single freq
#elif (RK1608_S5K33D_FREQ_MODE == 1)
		*mode = 1; // dual freq
#endif
	return 0;
}

int rk1608_s5k33dxx_get_modulation_frequency(uint16_t *modFreq)
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

/**
* @brief  s5k33dxx_video_streaming
* @param  [in] bool enable
* @return int
*/
int rk1608_s5k33dxx_video_streaming(bool enable)
{
    int ret;

    if (enable) {
        ret = sensor_write_reg_8(0x0100, 0x01);
        rk1608_s5k33d_start_streaming = 1;
    }
    else {
        ret = sensor_write_reg_8(0x0100, 0x00);
        rk1608_s5k33d_start_streaming = 0;
    }

    return ret;
}
/*
double fixed2float(uint32_t fp, uint32_t wl, uint32_t frac, bool is_signed)
{
	uint64_t m_fp = fp;
	double val;

	int s = (m_fp&((uint64_t)1 << (wl - 1))) >> (wl - 1);

	if (s == 1)//signed
	{
		uint64_t v1 = (~(m_fp - 1)&~(((uint64_t)0xFFFFFFFFFFFFFFFF) << (wl - 1)));
		val = -((double)v1) / (double)powf((double)2.0f, (double)(frac));
	}
	else//unsigned
	{
		val = ((double)m_fp) / (double)powf((double)2.0f, (double)(frac));
	}

	return val;
}*/

int rk1608_s5k33dxx_get_rx_temp(float *temperature)
{
    //TODO rx
    int ret;
    uint16_t value;

    ret = sensor_read_reg_16(0x000A, &value);
    if (ret < 0)
        return ret;

    //ALOGE("value =%d\n", value);
    *temperature = (float)fixed2float(value, 16, 8, 1);

    return 0;
}

int rk1608_s5k33dxx_get_tx_temp(float *temperature)
{
    int ret;
    uint16_t value;

    ret = sensor_read_reg_16(0x00F4, &value);
    if (ret < 0)
        return ret;
    //ALOGE("value =%d", value);

    *temperature = 25 + ((value - 296)) / 5.4f;

    //ALOGE("temperature =%f", *temperature);
    return ret;
}

/**
* @brief  s5k33dxx_set_integration_time
* if integration time need to greater than 800us, register 0x0340 should increase
* @param  [in] uint16_t integrationTime (us)
* @return int
*/
int rk1608_s5k33dxx_set_integration_time(uint16_t integrationTime)
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
    ret = sensor_read_reg_16(0x030c, (uint16_t *)&pll_s);
    if (1 == pll_s) {
        coarse_integration_time = coarse_integration_time / 2;
    }

    ret = sensor_write_reg_16(0x0202, coarse_integration_time);

    return ret;
}

/**
* @brief  s5k33dxx_get_integration_time 鑾峰彇褰撳墠绉垎鏃堕棿
* @param  [out] uint16_t *integrationTime (us)
* @return int
*/
int rk1608_s5k33dxx_get_integration_time(uint16_t *integrationTime)
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
    ret = sensor_read_reg_16(0x030c, (uint16_t *)&pll_s);
    if (1 == pll_s) {
        integration_time = integration_time * 2;
    }
    *integrationTime = integration_time;

    return ret;
}

int rk1608_s5k33dxx_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
    int ret = 0;
#if (RK1608_S5K33D_FREQ_MODE == 0)
    uint16_t current_addr = 0x4D4A;
    uint16_t AS_addr = 0x4D4B;
    uint16_t TrAS_addr = 0x4D4C;
#elif (RK1608_S5K33D_FREQ_MODE == 1)
    uint16_t current_addr = 0x5085;
    uint16_t AS_addr = 0x5086;
    uint16_t TrAS_addr = 0x5087;
#endif

    //ALOGE("set illum power: %d = %d", value_A, value_B);
    ret = cxa4016_reg_write(current_addr, value_A);

    static uint8_t value = 0xC0; // keep high 4 bit value

    if (value_A > 110 && value_A <= 130) { // different current need adjust light wave

        cxa4016_reg_write(AS_addr, 0x70); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x00)); // 0x10  TrAS
    }
    else if (value_A > 130 && value_A <= 147) {

        cxa4016_reg_write(AS_addr, 0x90); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x00)); // 0x10  TrAS
    }
    else if (value_A > 147 && value_A <= 170) {

        cxa4016_reg_write(AS_addr, 0x62); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x04)); // 0x10  TrAS
    }
    else if (value_A > 170 && value_A <= 185) {

        cxa4016_reg_write(AS_addr, 0x54); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x01)); // 0x10  TrAS
    }
    else if (value_A > 185 && value_A <= 197) {
        cxa4016_reg_write(AS_addr, 0x04); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x0C)); // 0x10  TrAS
    }
    else if (value_A > 197 && value_A <= 211) {

        cxa4016_reg_write(AS_addr, 0x04); // 0x0F  AS_W, AS_I
        cxa4016_reg_write(TrAS_addr, ((value & 0xF0) | 0x0F)); // 0x10  TrAS
    }

    return ret;
}

int rk1608_s5k33dxx_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
    int ret = 0;
    *vcsel_num = 1;

#if (RK1608_S5K33D_FREQ_MODE == 0)
    ret = cxa4016_reg_read(0x4D54, value_A);   //0x2891
#elif (RK1608_S5K33D_FREQ_MODE == 1)
    ret = cxa4016_reg_read(0x50FC, value_A);   //0x2891
#endif
    if (ret < 0)
        return ret;
    *value_B = 0;
    return ret;
}

int rk1608_s5k33dxx_set_modulation_frequency(uint16_t modFreq)
{
    int ret = 0;
    uint16_t freq_0, freq_1;

    freq_0 = (modFreq >> 8) & 0xFF;
    freq_1 = modFreq & 0xFF;

    ret = sensor_write_reg_8(0x0100, 0x00); // should stop streaming first

    switch (freq_0) { // PLL M can not set too large, so we change PLL S and D to get target frequency
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

    if (rk1608_s5k33d_start_streaming)
        ret = sensor_write_reg_8(0x0100, 0x01);

    return ret;
}

int rk1608_s5k33dxx_get_illum_duty_cycle(uint16_t *duty)
{
    int ret;
    uint16_t inc_value, dec_value;

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

    return ret;
}

int rk1608_s5k33dxx_set_illum_duty_cycle(uint16_t duty)
{
    int ret = 0;
    uint16_t value = 0;
    uint16_t inc_value, dec_value;

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

    return ret;
}

int rk1608_s5k33dxx_AE(bool enable)
{
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
    if (rk1608_s5k33d_start_streaming)
        ret = sensor_write_reg_8(0x0100, 0x01);

    return ret;
}

int rk1608_s5k33d_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    float list[31] = {
        27, 28.4, 29.86, 31.32, 32.78, 34.24, 35.7, 37.16, 38.62, 40.08, 41.54, 43, 44.4, 45.8, 47.2, 48.7,
        50, 51.4, 52.7, 54.2, 55.66, 57.12, 58.58, 60.04, 61.5, 62.96, 64.42, 65.88, 67.34, 68.8, 70.5,
    };

    memcpy(duty_cycle_list, list, sizeof(list));
    return 0;
}


int rk1608_s5k33dxx_get_tof_sensor_resolution(uint64_t *resolution)
{
    int ret;
    uint8_t mode = 0;
    *resolution = 0;
    *resolution = DUAL_FREQ_4X4_BINNING_RESOLUTION << 16 | DUAL_FREQ_2X2_BINNING_RESOLUTION << 8 | DUAL_FREQ_NO_BINNING_RESOLUTION;

    return ret;
}

int rk1608_s5k33dxx_get_tof_sensor_pixel_bit(uint8_t *pixel_bit)
{
    *pixel_bit = PIXEL_BIT;
    return 0;
}

int rk1608_s5k33dxx_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
    *mipi_pack_bit = MIPI_PACK_BIT;
    return 0;
}

/*
*@brief:  初始化rk1608的板上33d 函数接口,对一些接口进行覆盖，和sensor处理解耦合
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/

int rk1608_s5k33dxx_func_load()
{
	tof_sensor.init = rk1608_s5k33dxx_sensor_init;
	tof_sensor.software_trigger = rk1608_s5k33dxx_software_trigger;
	tof_sensor.hardware_trigger = rk1608_s5k33dxx_hardware_trigger;
	tof_sensor.get_sensor_id = rk1608_s5k33dxx_get_sensor_id;
    tof_sensor.get_sensor_info = rk1608_s5k33dxx_get_sensor_info;
	tof_sensor.get_frequency_mode = rk1608_s5k33dxx_get_frequency_mode;
    tof_sensor.get_modulation_frequency = rk1608_s5k33dxx_get_modulation_frequency;
    tof_sensor.set_modulation_frequency = rk1608_s5k33dxx_set_modulation_frequency;
    tof_sensor.get_illum_duty_cycle = rk1608_s5k33dxx_get_illum_duty_cycle;
    tof_sensor.set_illum_duty_cycle = rk1608_s5k33dxx_set_illum_duty_cycle;
    tof_sensor.video_streaming = rk1608_s5k33dxx_video_streaming;
    tof_sensor.get_rx_temp = rk1608_s5k33dxx_get_rx_temp;
    tof_sensor.get_tx_temp = rk1608_s5k33dxx_get_tx_temp;
    tof_sensor.set_integration_time = rk1608_s5k33dxx_set_integration_time;
    tof_sensor.get_integration_time = rk1608_s5k33dxx_get_integration_time;
    tof_sensor.set_illum_power = rk1608_s5k33dxx_set_illum_power;
    tof_sensor.get_illum_power = rk1608_s5k33dxx_get_illum_power;
    tof_sensor.AE = rk1608_s5k33dxx_AE;
    tof_sensor.get_illum_duty_cycle_list = rk1608_s5k33d_get_illum_duty_cycle_list;

	//rk1608
	//tof_sensor.set_streaming_type = rk1608_set_streaming_type;
	//tof_sensor.load_ref_buffer = rk1608_s5k33dxx_download_ref;
	//tof_sensor.get_ref_buffer_addr = rk1608_get_ref_buffer_addr;
    tof_sensor.get_tof_sensor_resolution = rk1608_s5k33dxx_get_tof_sensor_resolution;
    tof_sensor.get_tof_sensor_pixel_bit = rk1608_s5k33dxx_get_tof_sensor_pixel_bit;
    tof_sensor.get_mipi_pack_bit = rk1608_s5k33dxx_get_mipi_pack_bit;

	return 0;
}
