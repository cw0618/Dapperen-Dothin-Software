#ifndef XN_MX6X_PROPETIES
#define XN_MX6X_PROPETIES

#include <XnOS.h>
#include <stdint.h>
#include "OBCProperties.h"


typedef void* LIB_HANDLE;
typedef void(*FuncAddr)(void *);


/**
* Name of the hal_module_info as a string
*/
#define HAL_MODULE_SYM_AS_STR  "MX6X_HMI"
#define HAL_PROTOCOL_SYM_AS_STR  "PROTOCOL_HMI"



/**
*@brief 定义流类型
*/
typedef enum obc_stream_type 
{
	OBC_STREAM_DISABLED = 0,  //!< 流已经关闭
	OBC_STREAM_IR_FLOOD,      //!< 泛光灯流
	OBC_STREAM_IR_LASER,      //!< 散斑流
	OBC_STREAM_IR_EXT,       //!<
	OBC_STREAM_DEPTH,         //!< 深度流
	OBC_STREAM_BYPASS_ORI,    //!< 原始流
	OBC_STREAM_BYPASS_REF,    //!< 参考图流
	OBC_STREAM_BYPASS_IR,     //!< BYPASS_IR流
	OBC_STREAM_TEST_PATTERN,  //!< 测试流
	OBC_STREAM_ENGINE_MONO,   //!<NO USE
	OBC_STREAM_ENGINE_STEREO,
	OBC_STREAM_DEPTH_MONO,	//!< MONO流
	OBC_STREAM_COLOR,		//!< COLOR流
	OBC_STREAM_DISPARITY,		//!< 视差流
    OBC_STREAM_TOF_PHASE = 20,   //!< 相位图
	OBC_STREAM_INVALID = -1,
} obc_stream_type_t;


/**
*@brief  video 模式定义
*模式定义，包含分辨率及流类型
* @see obc_stream_type_t
**/
typedef struct
{
	XnInt width;
	XnInt height;
	obc_stream_type_t type;
}obc_videomode;

typedef enum hw_stream_type {
	STREAM_DISABLED = 0,  //
	STREAM_IR_FLOOD,      // IR enable flood
	STREAM_IR_LASER,      // IR enable laser
	STREAM_IR_EXT,        //
	STREAM_DEPTH,         
	STREAM_BYPASS_ORI,
	STREAM_BYPASS_REF,
	STREAM_BYPASS_IR,
	STREAM_TEST_PATTERN,
	STREAM_COLOR,
	STREAM_TOF_PHASE = 20,
	STREAM_INVALID = -1,
} hw_stream_type_t;


typedef struct block_buf{
	void* data;
	XnUInt32 len;
	XnUInt32 blocksize;
}block_buf_t;


typedef struct command_data{
	void* data;
	XnUInt32 len;
}command_data_t;


typedef enum mx6x_command{
	MODULE_NAME,
	MODULE_VERSION,
	FW_VERSION,
	LOAD_FW,
	LOAD_REF,

	EEPROM_RW,
	REG_RW,
	STREAM_MODE,
	DATA_FMT,

	FPS,
	AE_ENABLE,
	IR_EXP,
	IR_GAIN,
	PULSE_WIDTH,

	FLOOD_LED,
	LASER,
	IR_SN,
	LDMP_SN,
	LASER_CURRENT,

	FLOOD_CURRENT,
	WATCH_DOG,
	SLEEP_WAKE,
	PACK_FMT,
	DISP_BITS,

	DISP_SUB_BITS,
	DEPTH_ROTATION_TYPE,
	DEPTH_MIRROR_TYPE,
	ENGINE_ID,
	CAMERA_PARAMS,
	IR_TEMP,
	LDMP_TEMP,
	REF_SUMMARY_INFO,
	STREAMS_CAPACITY,
	EXT_PARAMS,
	LDMP_DOUBLE,
	CHIP_RESET,
	ENGINE_NAME,
	NTC_SVC,
	NTC_COEFF,
	DEP_RECTIFY,
	AE_AVG,
	RAW_COMMAND_INT,
	SENSOR_RES,    //tmp  for  debug
	SENSOR_VOL,
	ENGINE_IS_VALID = 44,//!!!CONFIRM IF NEED TO MOVE

	SUPPORT_VIDEO_MODES = 200,
	SENSOR_TEMP = 201,
	RX_TEMP = 202,
	TX_TEMP = 203,
	ILLUM_POWER = 204,
	ILLUM_POWER_CTL = 205,
	INTEGRATION_TIME = 206,
	MODULATION_FREQUENCY = 207,
	DATA_OUTPUT_MODE = 208,
	DUTY_CYCLE = 209,
	MIRROR_FLIP = 210,
	TEST_PATTERN = 211,
	SOFTWARE_TRIGGER = 212,
	HARDWARE_TRIGGER = 213,

	SENSOR_ID = 214,
	SENSOR_INFO = 215,      // EXPAND_DATASIZE
	DUTY_CYCLE_LIST = 216,
	VCSEL_PD = 217,   //get_vcsel_pd
	ILLUM_POWER_TEST = 219, //imx456_illum_power_test
	DEFAULT_PARAMS = 220, //默认参数
	OPS_PTR = 221,      //  obc_ops_t指针
	AE = 222,          // AE  开关
	DRIVER_IC_DETECT,
	REG16_RW,           //sensor 寄存器16bit读写
	DELAY_CIRCUIT_TIME = 225, //延时电路时间
	DELAY_CIRCUIT_ID,       //延时电路ID
	TX_A_B_POWER,       //驱动模式 设置/获取    get_tx_a_b_power
	SENSOR_FREQUENCY_DUTY_CONFIG,     // frequency duty 等的配置  设置目前仅518在用
	SHUFFLE,          // 0x01 enable shuffle（默认）；0x00 disable shuffle
	CHIP_ID,          // sensor 唯一的id 信息 长度6Bytes  需要开流后调用
	BINNING_MODE,     //binning mode  uint8_t   0: no scaling  1: 2x2 binning   2: 4x4 binning  see @ObBinningMode
	BURST_MODE,       // 设置不同模式  可控制帧间隔
	FREQUENCY_MODE,       //sensor 当前处于的频率模式 单频/双频，unsigned 8bit
	ITO_EM,            //ITO通断，Tx工作时读取ITO状态值
	DRIVER_IC_REG8_RW,  //driver ic 寄存器读写   地址为16bit，数据为:8bit
	VMGHI_VOLTAGE,      //供电电压
	AF_DEPTH,           //频率模式为AF时，获取13bit的深度值

	COMMAND_END,    //ENGINE_IS_VALID
}hw_command_t;


typedef struct OBC_Command_t{
	XnChar propertyName[64];
	hw_command_t command;
}OBC_Command_t;


typedef enum {
	I2C_PROTOCOL,
	SPI_PROTOCOL,
}protocol_t;


typedef enum {
	IR_SENSOR = 1,
	DEPTH_SENSOR,
	COLOR_SENSOR,
}capablibity_type_t;


typedef struct support_sensor {
	protocol_t PROTOCOL;
	XnChar sensorname[20];
	capablibity_type_t type;
}support_protocol_type_t;


typedef enum hw_ext_params_t {
	GET_SUPPORT_RESOLUTION,
	DEP_PREFILTER,
	DEP_PREFILTER_LEVEL,
	DEP_SMFILTER,
	DEP_THRFILTER,
	DEP_UNIDIV,
	DEP_PTFILTER,
	DEP_ENGINE,
	GPM,
	GPM_STATUS,
	GPM_IS_ENABLE
}hw_ext_params;


typedef struct OBC_Ext_Command_t
{
	XnChar propertyName[64];
	hw_command_t command;
	hw_ext_params_t ext_command;
}OBC_Ext_Command_t;


typedef struct hw_ext_msg
{
    XnInt subcmd;
	void* msg;
	void* p_data;
}hw_ext_msg_t;


typedef struct
{
    XnInt addr;
    XnInt len;
    XnUInt8 *data;
}EepromData;


typedef struct 
{
    XnInt addr;
    XnInt len;
    XnUInt32 *data;
}RefBufferData;


typedef struct 
{
    XnUInt32 addr;
    XnUInt32 value;
}RegMap32;


typedef struct 
{
    XnUInt16 addr;
    XnUInt16 value;
} RegMap16;






#endif //XN_MX6X_PROPETIES

