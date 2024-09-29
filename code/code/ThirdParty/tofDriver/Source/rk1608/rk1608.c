#include <stdio.h>
#include "rk1608.h"

#include "spi\inc\spi2apb.h"
#include "spi\inc\isp-fw.h"
#include "spi\inc\msg-queue.h"
#include "spi\inc\msg-interface.h"

#include <tof_sensors.h>
#include <hw_modules.h>
#include <hw_obstatus.h>

#include "rk1608_pleco.h"
#include <rk1608_s5k33dxx.h>
#include "debug2log.h"

#ifdef __linux__
#include <unistd.h>
#endif

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

static int rk1608_s5k33dxx_func_init();
static int rk1608_pleco_func_init();

//static int(*before_video_streaming)(bool enable);

/*
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [TOF_DLL] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define TEE_LOG_LEVEL_DEBUG        2
#define ALOGD(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[DEBUG] [TOF_DLL] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
*/

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

sensor_list_t rk1608_sensor_list[] = {
	{ RK1608_S5K33D_SENSOR_ID, rk1608_s5k33dxx_func_init},
	{ RK1608_PLECO_SENSOR_ID, rk1608_pleco_func_init }
};

typedef struct {
	int(*set_phase_out)();
	int(*set_depth_out)();
	void(*set_phase_src_info)();
	void(*set_depth_src_info)();
	int (*get_firmware)(uint8_t);
} rk1608_sensor_func_t;

rk1608_sensor_func_t rk1608_sensor_func = { 0 };

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

const uint8_t *fireware_data = NULL;//固件数据指针，用于指向固件数据
static struct sensor_info_t get_src_info;
uint8_t rk1608_s5k33d_start_streaming = 0;

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

	ret = dothin_i2c_writeread(&msg, 1);

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

	ret = dothin_i2c_writeread(&msg, 1);

	return ret;
}

static int sensor_write_reg_8(uint16_t reg, uint8_t value)
{
    return i2c_reg_write(S5K33D_I2C_ADDR, reg, 2, value, 1);
}

static int sensor_read_reg_8(uint16_t reg, uint8_t *value)
{
    return i2c_reg_read(S5K33D_I2C_ADDR, reg, 2, value, 1);
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
	int ret = i2c_reg_read(S5K33D_I2C_ADDR, reg, 2, value, 2);

	return ret;
}

static int gpio_control_interface(int pin, bool level)
{
	int rtn = dothin_set_gpio_level(pin, level, dothin_device_id);
	if (rtn < 0) {
		ALOGE("DothinekyGpioControl\n");
	}

	return rtn;
}

static int spi_block_read_interface(uint32_t addr, uint8_t *data, uint32_t data_size)
{
	int rtn = dothin_spi_writeread(true, true, true, NULL, data, data_size, data_size, dothin_device_id);
	if (rtn != 1) {
		ALOGE("DothinekyRSPI\n");
		return -1;
	}

	return 0;
}

static int spi_block_write_interface(uint32_t addr, uint8_t *data, uint32_t data_size)
{
	int rtn = dothin_spi_writeread(true, true, true, data, NULL, data_size, 0, dothin_device_id);
	if (rtn != 1) {
		ALOGE("DothinekyWSPI\n");
		return -1;
	}

	return 0;
}

static int spi_block_write_read_interface(uint8_t *tx, uint8_t *rx, uint32_t tx_len, uint32_t rx_len)
{
	int rtn = dothin_spi_writeread(true, true, true, tx, rx, tx_len, rx_len, dothin_device_id);
	if (rtn != 1) {
		ALOGE("DothinekyRWSPI\n");
		return -1;
	}

	return 0;
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

/*
*@brief:  配置i2c或者spi等通信接口
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
static int set_fun_handle(void)
{
	set_gpio_callback(gpio_control_interface);
	set_iic_write_callback(i2c_reg_write);
	set_iic_read_callback(i2c_reg_read);
	set_spi_block_read_callback(spi_block_read_interface);
	set_spi_block_write_callback(spi_block_write_interface);
	set_spi_block_rw_callback(spi_block_write_read_interface);

	return HW_NO_ERR;
}

/*
*@brief:  发送指令使rk1608初始化内部硬件
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
void rkpreisp_hw_init()
{
	int ret = spi2apb_safe_w32(0x12008098, 0xff004000);
	if (ret != 0) {
		ALOGE("%s fail", __FUNCTION__);
	}
}

/*
*@brief:  控制rk1608从内部DDRAM启动
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int boot_from_ddr()  // first download should call this
{
	int ret = 0;

	set_fun_handle();
	ret = set_duxin_access_rk1608();
	if (ret < 0) {
		ALOGE("%s set_duxin_access_rk1608 fail", __FUNCTION__);
		return -HW_ERR_RW;
	}
	spi2apb_switch_to_msb();
	rkpreisp_hw_init();
	//ret = get_firmware(BOOT_FROM_DDR);
	ret = rk1608_sensor_func.get_firmware(BOOT_FROM_DDR, &fireware_data);

	if (ret < 0) {
		ALOGE("%s DDR get_firmware", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = spi_download_fw(fireware_data);
	if (ret != 0) {
		ALOGE("%s boot_from_ddr error", __FUNCTION__);
		return -HW_ERR_RW;
	}
	ALOGD("%s success", __FUNCTION__);
	return ret;
}
/*
*@brief: 控制rk1608从外部flash启动
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int boot_from_flash()
{
	int ret;

	set_fun_handle();
	//	ret = set_duxin_access_flash();
	//	ret = get_firmware(BOOT_FROM_FLASH);
	//	qDebug() << "start download firmware, size: " << firmware.size();
	//	ret = flash_download_fw((uint8_t *)firmware.data(), firmware.size());
	//	ret = set_rk1608_access_flash();
	//	ret = rk1608_reset();

	return ret;
}

/*
*@brief:  控制rk1608上电
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int rkpreisp_power_on()
{
	int ret = 0;

	// TODO: power control etc..

	/* download fw and start run */
	//ret = rkpreisp_download_fw();

	//if (!ret)
	//    rkpreisp_set_log_level(pdata, pdata->log_level);

	return ret;
}

/*
*@brief:  发送数据到rk1608（dsp）,配置rk1608停止出流
@param[in] m：发送的数据结构体
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
static int set_rk1608_stream_off()
{
	int8_t cam_id = 0;
	int ret = -1;

	ret = cmd_msg_set_stream_in_off(cam_id);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_in_off fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	usleep(20000);
	ret = cmd_msg_set_stream_out_off(cam_id);
	if (ret < 0)
	{
		ALOGE("%s cmd_msg_set_stream_out_off fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}
	usleep(2000);

	return ret;
}

/*
*@brief:  配置引脚使rk1608 spi进入从模式和对应的spi、i2c通信引脚
@return   HW_NO_ERR          成功
-HW_ERR_NULL				 传入空指针
-HW_ERR_INVALID				 无效参数
*/
int dothin_pin_config()
{
	int ret = 0;
	uint8_t  pinDef[40] = { 0 };

	pinDef[0] = 20;
	pinDef[1] = 0;
	pinDef[2] = 2;
	pinDef[3] = 1;
	pinDef[4] = 3;
	pinDef[5] = 4;
	pinDef[6] = 5;
	pinDef[7] = 6;
	pinDef[8] = 7;
	pinDef[9] = 8;
	pinDef[10] = 9;
	pinDef[11] = 20;
	pinDef[12] = 10;
	pinDef[13] = 11;
	pinDef[14] = 12;
	pinDef[15] = 20;
	pinDef[16] = 17;// PIN_GPIO1; // DX_SPI_CS
	pinDef[17] = 13;
	pinDef[18] = 21;// PIN_GPIO2; // DX_TO_FLASH
	pinDef[19] = 14;
	pinDef[20] = 19;
	pinDef[21] = 18;
	pinDef[22] = 28;// PIN_SPI_SCK; // PO2
	pinDef[23] = 22;// PIN_GPIO3;   // PO1
	pinDef[24] = 31;// PIN_SPI_SDO; // PO3
	pinDef[25] = 30;// PIN_SPI_SDI; // PO4

	//配置柔性接口
	ret = dothin_set_soft_pin(pinDef, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "SetSoftPin: ");

	//使能柔性接口
	ret = dothin_enable_soft_pin(TRUE, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "EnableSoftPin: ");

	ret = dothin_enable_gpio(TRUE, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "EnableGpio: ");

	//开启IO上拉电阻
	ret = dothin_set_soft_pin_pullup(IO_PULLUP, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "SetSoftPinPullUp: ");

	/* RK1608 SPI enter slave mode operation */
	// Dothin GPIO Expander 2
	//WriteSensorReg WriteSensorI2c
	ret = dothin_write_sensor_reg(0x42, 0x03, 0x00, I2CMODE_NORMAL, dothin_device_id);// set all gpio as output
	ret = dothin_write_sensor_reg(0x42, 0x01, 0x00, I2CMODE_NORMAL, dothin_device_id);//set GPIO_P0(RK1608 Reset) to low
	usleep(2000);
	ret = dothin_set_gpio_level(2, 1, dothin_device_id); // pull DX_SPI_CS pin to high
	usleep(2000);
	ret = dothin_set_gpio_level(1, 1, dothin_device_id); // pull DX_TO_FLASH pin to high
	usleep(2000);
	ret = dothin_set_gpio_level(3, 0, dothin_device_id); // pull PO3(SPI_CS) pin to low
	usleep(10000); // T0
	ret = dothin_write_sensor_reg(0x42, 0x01, 0x01, I2CMODE_NORMAL, dothin_device_id);// set GPIO_P0(RK1608 Reset) to high	 
	usleep(10000);// T1
	ret = dothin_write_sensor_reg(0x42, 0x01, 0x00, I2CMODE_NORMAL, dothin_device_id);// set GPIO_P0(RK1608 Reset) to low
	usleep(10000);// T2
	ret = dothin_write_sensor_reg(0x42, 0x01, 0x01, I2CMODE_NORMAL, dothin_device_id);// set GPIO_P0(RK1608 Reset) to high      
	usleep(10000);// T3
	ret = dothin_set_gpio_level(3, 1, dothin_device_id); // pull PO1(SPI_CS) pin to high
	usleep(2000);

	// Dothin GPIO Expander 1
	//WriteSensorReg WriteSensorI2c
	ret = dothin_write_sensor_reg(0x40, 0x03, 0x00, I2CMODE_NORMAL, dothin_device_id);// set all gpio as output
	ret = dothin_write_sensor_reg(0x40, 0x01, 0x00, I2CMODE_NORMAL, dothin_device_id);// set TCA6408_P6(IR_VSYNC_IN) to low, TCA6408_P0-4(tof power)to low
	usleep(2000);
	ret = dothin_write_sensor_reg(0x40, 0x01, 0x0f, I2CMODE_NORMAL, dothin_device_id);// set TCA6408_P6(IR_VSYNC_IN) to low, TCA6408_P0-4(tof power)to high
	usleep(2000);

	//关闭IO上拉电阻
	ret = dothin_set_soft_pin_pullup(IO_NOPULL, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "SetSoftPinPullUp: ");


	pinDef[16] = 35; // GPIO5   DX_SPI_CS
	pinDef[18] = 36; // GPIO6   DX_TO_FLASH
	pinDef[22] = 28;//PIN_SPI_SCK; // PO2
	pinDef[23] = 29;//PIN_SPI_CS;  // PO1
	pinDef[24] = 30;//PIN_SPI_SDI; // PO3
	pinDef[25] = 31;//PIN_SPI_SDO; // PO4
	//配置柔性接口
	ret = dothin_set_soft_pin(pinDef, dothin_device_id);

	//使能柔性接口
	ret = dothin_enable_soft_pin(TRUE, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "EnableSoftPin: ");

	ret = dothin_enable_gpio(TRUE, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "EnableGpio: ");

	//开启IO上拉电阻
	ret = dothin_set_soft_pin_pullup(IO_PULLUP, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "SetSoftPinPullUp: ");

	usleep(10000);

	MasterSpiConfig_t SPIConfig;
	SPIConfig.fMhz = 12; // max support 20MHz
	SPIConfig.byWordLen = 0;
	SPIConfig.byCtrl = 0; // CS = 0, CPOL = 0, CPHA = 0, spi protocal config
	ret = dothin_master_spi_config(0, &SPIConfig, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret, "MasterSpiConfig: ");
	usleep(10000);

	//i2C init....
	//设置SENSOR I2C的速率为400Kbps,允许从设备为Streching mode（从设备端可以拉低scl和sda电平来表示busy）
	ret = dothin_set_sensor_i2c_rate(I2C_400K, dothin_device_id);  //I2C_400K
	CHECK_DOTHIN_STATUS(ret);
	//check sensor is on line or not ,if on line,init sensor to work....
	ret = dothin_set_sensor_enable(1, 1, dothin_device_id); //reset
	CHECK_DOTHIN_STATUS(ret);

	usleep(20000);
	ret = dothin_set_sensor_enable(3, 1, dothin_device_id);
	CHECK_DOTHIN_STATUS(ret);
	usleep(50000);

	return HW_NO_ERR;
}

/**
* @brief  rk1608_set_streaming_type
* @param  [in] bool enable
* @return int
*/
static int rk1608_set_streaming_type(int mode)
{
	int ret = -1, i;
    //ALOGD("%s rk1608_set_streaming_type:%d", __FUNCTION__, mode);
	if (mode == STREAM_TOF_PHASE) {
		rk1608_sensor_func.set_phase_src_info();
		ret = rk1608_sensor_func.set_phase_out();
	}
	else if (mode == STREAM_DEPTH) {
		rk1608_sensor_func.set_depth_src_info();
		ret = rk1608_sensor_func.set_depth_out();
	}
	else if (mode == STREAM_DISABLED) {
        ret = set_rk1608_stream_off();
	}
	else
	{
		ALOGE("%s rk1608 stream control fail:%d", __FUNCTION__, mode);
	}

	return ret;
}

/**
* @brief  rk1608_s5k33dxx_download_ref
* @param  [in] rk1608_store_addr:the addr of rk1608 store;ref_data_addr:the buffer addr of ref;ref_data_len:the buffer len of ref
* @return int
*/
static int rk1608_download_ref(uint32_t rk1608_store_addr, uint32_t *ref_data_addr, uint32_t ref_data_len)
{
	int ret;
	ret = spi2apb_safe_write(rk1608_store_addr, ref_data_addr, ref_data_len);
	return ret;
}

/**
* @brief  rk1608_s5k33dxx_download_ref
* @param  [in] rk1608_store_addr:the offset addr of rk1608 store ref buffer
* @return int
*/
static int rk1608_get_ref_buffer_addr(uint32_t *rk1608_store_addr)
{
	*rk1608_store_addr = RK1608_REF_BUFFER_ADDR;
	return 0;
}

/*
*@brief:  初始化rk1608的板上sensor,与rk1608相关的接口进行覆盖，和sensor处理解耦合
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_func_load()
{
	tof_sensor.set_streaming_type = rk1608_set_streaming_type;
	tof_sensor.load_ref_buffer = rk1608_download_ref;
	tof_sensor.get_ref_buffer_addr = rk1608_get_ref_buffer_addr;

	return 0;
}

/*
*@brief:  初始化rk1608的板上sensor,对pleoc sensor和rk1608相关的接口进行覆盖，和sensor处理解耦合
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_pleco_func_init()
{
	memset(&tof_sensor, 0, sizeof(tof_sensor));
	memset(&rk1608_sensor_func, 0, sizeof(rk1608_sensor_func));

	rk1608_pleco_func_load();
	rk1608_func_load();

	rk1608_sensor_func.get_firmware = rk1608_pleco_get_firmware;
	rk1608_sensor_func.set_depth_out = set_rk1608_pleco_depth_out;
	rk1608_sensor_func.set_depth_src_info = rk1608_pleco_set_depth_src_info;
	rk1608_sensor_func.set_phase_out = set_rk1608_pleco_raw_phase_out;
	rk1608_sensor_func.set_phase_src_info = rk1608_pleco_set_phase_src_info;

	return 0;
}

/*
*@brief:  初始化rk1608的板上sensor,对s5k33dxx sensor和rk1608相关的接口进行覆盖，和sensor处理解耦合
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_s5k33dxx_func_init()
{
	memset(&tof_sensor, 0, sizeof(tof_sensor));
	memset(&rk1608_sensor_func, 0, sizeof(rk1608_sensor_func));

	rk1608_s5k33dxx_func_load();
	rk1608_func_load();

	rk1608_sensor_func.get_firmware = rk1608_s5k33dxx_get_firmware;
	rk1608_sensor_func.set_depth_out = set_rk1608_s5k33dxx_depth_out;
	rk1608_sensor_func.set_depth_src_info = rk1608_s5k33dxx_set_depth_src_info;
	rk1608_sensor_func.set_phase_out = set_rk1608_s5k33dxx_raw_phase_out;
	rk1608_sensor_func.set_phase_src_info = rk1608_s5k33dxx_set_phase_src_info;

	return 0;
}

/*
*@brief:  初始化rk1608的板上sensor,对一些接口进行覆盖，和sensor处理解耦合
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
static int rk1608_board_sensor_init()
{
	uint8_t i;
	uint16_t sensor_id = 0;
	int16_t ret;

	for (i = 0; i < (sizeof(rk1608_sensor_list) / sizeof(sensor_list_t)); i++)
	{
		if (rk1608_sensor_list[i].sensor_init == NULL)
			break;
		rk1608_sensor_list[i].sensor_init();
		tof_sensor.get_sensor_id(&sensor_id);
		if (sensor_id == rk1608_sensor_list[i].SensorId)
		{
			ret = tof_sensor.init();
			break;
		}
	}
	if (i < sizeof(rk1608_sensor_list) / sizeof(sensor_list_t)) {
		return  ret;
	}
	return  -HW_ERR_NO_MATCH;
}

/*
*@brief:  加载rk1608板子初始化函数
@return   HW_NO_ERR          成功
-HW_ERR_NULL       传入空指针
-HW_ERR_NO_MATCH   ID匹配失败
*/
int rk1608_init()
{
	int ret = 0;

	ret = dothin_pin_config();  // 配置引脚使rk1608 spi进入从模式和对应的spi、i2c通信引脚
	if (ret < 0) {
		ALOGE("%s dothin_pin_config fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = rk1608_board_sensor_init();//初始化rk1608的板上sensor，度信平台上度信通过I2C直接控制sensor，这里进行一些接口的覆盖如init()、hardware_trigger()等
	if (ret < 0) {
		ALOGE("%s rk1608_board_sensor_init fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = boot_from_ddr();  // first download should call this
	if (ret < 0) {
		ALOGE("%s boot_from_ddr fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	//ALOGD("%s success", __FUNCTION__);
	return ret;
}

int rk1608_pleco_init()
{
	int ret = 0;
	uint16_t sensor_id = 0;

	ret = dothin_pin_config();  // 配置引脚使rk1608 spi进入从模式和对应的spi、i2c通信引脚
	if (ret < 0) {
		ALOGE("%s dothin_pin_config fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	rk1608_pleco_func_init();//rk1608_pleco 函数加载
	ret = tof_sensor.get_sensor_id(&sensor_id);//pleco sensor供电配置和度信配置
	ret |= tof_sensor.init();
	if (ret < 0) {
		ALOGE("%s rk1608_board_sensor_init fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = boot_from_ddr();  // first download should call this
	if (ret < 0) {
		ALOGE("%s boot_from_ddr fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	//ALOGD("%s success", __FUNCTION__);
	return ret;
}

int rk1608_S5K33dxx_init()
{
	int ret = 0;
	uint16_t sensor_id = 0;

	ret = dothin_pin_config();  // 配置引脚使rk1608 spi进入从模式和对应的spi、i2c通信引脚
	if (ret < 0) {
		ALOGE("%s dothin_pin_config fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	rk1608_s5k33dxx_func_init();
	//ret = tof_sensor.get_sensor_id(&sensor_id);
	ret |= tof_sensor.init();
	if (ret < 0) {
		ALOGE("%s rk1608_board_sensor_init fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	ret = boot_from_ddr();  // first download should call this
	if (ret < 0) {
		ALOGE("%s boot_from_ddr fail", __FUNCTION__);
		return -HW_ERR_NO_SUPPORT;
	}

	//ALOGD("%s success", __FUNCTION__);
	return ret;
}
