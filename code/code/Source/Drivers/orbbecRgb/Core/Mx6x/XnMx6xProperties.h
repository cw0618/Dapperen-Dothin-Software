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
*@brief ����������
*/
typedef enum obc_stream_type 
{
	OBC_STREAM_DISABLED = 0,  //!< ���Ѿ��ر�
	OBC_STREAM_IR_FLOOD,      //!< �������
	OBC_STREAM_IR_LASER,      //!< ɢ����
	OBC_STREAM_IR_EXT,       //!<
	OBC_STREAM_DEPTH,         //!< �����
	OBC_STREAM_BYPASS_ORI,    //!< ԭʼ��
	OBC_STREAM_BYPASS_REF,    //!< �ο�ͼ��
	OBC_STREAM_BYPASS_IR,     //!< BYPASS_IR��
	OBC_STREAM_TEST_PATTERN,  //!< ������
	OBC_STREAM_ENGINE_MONO,   //!<NO USE
	OBC_STREAM_ENGINE_STEREO,
	OBC_STREAM_DEPTH_MONO,	//!< MONO��
	OBC_STREAM_COLOR,		//!< COLOR��
	OBC_STREAM_DISPARITY,		//!< �Ӳ���
    OBC_STREAM_TOF_PHASE = 20,   //!< ��λͼ
	OBC_STREAM_INVALID = -1,
} obc_stream_type_t;


/**
*@brief  video ģʽ����
*ģʽ���壬�����ֱ��ʼ�������
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
	DEFAULT_PARAMS = 220, //Ĭ�ϲ���
	OPS_PTR = 221,      //  obc_ops_tָ��
	AE = 222,          // AE  ����
	DRIVER_IC_DETECT,
	REG16_RW,           //sensor �Ĵ���16bit��д
	DELAY_CIRCUIT_TIME = 225, //��ʱ��·ʱ��
	DELAY_CIRCUIT_ID,       //��ʱ��·ID
	TX_A_B_POWER,       //����ģʽ ����/��ȡ    get_tx_a_b_power
	SENSOR_FREQUENCY_DUTY_CONFIG,     // frequency duty �ȵ�����  ����Ŀǰ��518����
	SHUFFLE,          // 0x01 enable shuffle��Ĭ�ϣ���0x00 disable shuffle
	CHIP_ID,          // sensor Ψһ��id ��Ϣ ����6Bytes  ��Ҫ���������
	BINNING_MODE,     //binning mode  uint8_t   0: no scaling  1: 2x2 binning   2: 4x4 binning  see @ObBinningMode
	BURST_MODE,       // ���ò�ͬģʽ  �ɿ���֡���
	FREQUENCY_MODE,       //sensor ��ǰ���ڵ�Ƶ��ģʽ ��Ƶ/˫Ƶ��unsigned 8bit
	ITO_EM,            //ITOͨ�ϣ�Tx����ʱ��ȡITO״ֵ̬
	DRIVER_IC_REG8_RW,  //driver ic �Ĵ�����д   ��ַΪ16bit������Ϊ:8bit
	VMGHI_VOLTAGE,      //�����ѹ
	AF_DEPTH,           //Ƶ��ģʽΪAFʱ����ȡ13bit�����ֵ

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

