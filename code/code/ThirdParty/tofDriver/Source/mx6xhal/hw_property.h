#ifndef __HW_PROPERTY_
#define __HW_PROPERTY_
#include"ob_resolution.h"
typedef enum hw_stream_type {
	STREAM_DISABLED = 0,  //
	STREAM_IR_FLOOD,      // IR enable flood
	STREAM_IR_LASER,      // IR enable laser
	STREAM_IR_EXT,        // 
	STREAM_DEPTH,         //
	STREAM_BYPASS_ORI,
	STREAM_BYPASS_REF,
	STREAM_BYPASS_IR,
	STREAM_TEST_PATTERN = 8,
	STREAM_COLOR=12,
	STREAM_TOF_PHASE = 20,
    STREAM_TOF_AMPLITUDE = 21,
	STREAM_INVALID = -1,  //
} hw_stream_type_t;


typedef enum
{
	ROTATION_ZERO,
	ROTATION_CLOCKWISE_90,
	ROTATION_180,
	ROTATION_ANTICLOCK_90
} rotation_type;


typedef enum
{
	MIRROR_NON,
	MIRROR_VERTICAL,
	MIRROR_HORIZONTAL,
	MIRROR_VERT_HORIZ
} mirror_type;

typedef struct stream_mode{
   int width;
   int height;
   hw_stream_type_t type;
}stream_mode_t;

typedef struct eeprom_data{
   int addr;
   int len;
   uint8_t* data;
}eeprom_data_t;

typedef struct ref_buffer_data {
	int addr;
	int len;
	uint32_t* data;
}ref_buffer_data_t;

typedef struct reg_map{
   uint32_t addr;
   uint32_t value;
}reg_map_t;

typedef struct reg16_map {
    uint16_t addr;
    uint16_t value;
} reg16_map_t;


typedef struct block_buf{
	void* data;
	uint32_t len;
	uint32_t blocksize;
}block_buf_t;

typedef enum ref_rotation_e
{
	HW_REF_ROTATION_0 = 0,
	HW_REF_ROTATION_CLOCKWISE_90 = 1,
	HW_REF_ROTATION_ANTI_CLOCKWISE_90 = 2,
	HW_REF_ROTATION_180 = 3,
} OB_REF_ROTATION;

typedef struct
{
	float rx, ry, rz;
	float tx, ty, tz;
} ob_external_info;

typedef struct
{
	uint8_t disto_model;
	struct
	{
		float k1;
		float k2;
		float t1;
		float t2;
		float k3;
		///
	} disto_data;
} ob_distortion_info;

typedef struct ref_summary_t {
	float baseline;
	float ref_dist;
	float pixel_size;

	OB_REF_ROTATION ir_rotate;
	uint8_t ir_mirror;
	ob_distortion_info disto_info;
	ob_external_info external_info;
	float ir_coeff;
	float ldmp_coeff;
}ob_ref_summary;



typedef struct command_data{
   void* data;
   uint32_t len;
}command_data_t;

typedef enum {
	I2C_PROTOCOL,
	SPI_PROTOCOL,
}protocol_t;

typedef enum {
	IR_SENSOR = 1,
	DEPTH_SENSOR = 2,
	COLOR_SENSOR = 3,
    PHASE_SENSOR = 4,
}capablibity_type_t;

typedef struct support_sensor {
	protocol_t PROTOCOL;
	char sensorname[20];
	capablibity_type_t type;
}support_protocol_type_t;

typedef struct i2c_msg {
	uint8_t slave_addr;
	uint8_t rw_mode;
	uint16_t reg;
	uint8_t  reg_size;
	uint32_t* data;
	uint16_t data_size;
}i2c_msg_t;


/*
*此结构体的枚举成员不再增加，统一用tofinfo.h中的定义
*/
typedef enum mx6x_command {
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

    AE_ENABLE = 10,
    IR_EXP,
    IR_GAIN,
    PULSE_WIDTH,
    FLOOD_LED,
    LASER,
    IR_SN,
    LDMP_SN,
    LASER_CURRENT,
    FLOOD_CURRENT,

    WATCH_DOG = 20,
    SLEEP_WAKE,
    PACK_FMT,
    DISP_BITS,
    DISP_SUB_BITS,
    DEPTH_ROTATION_TYPE,
    DEPTH_MIRROR_TYPE,
    ENGINE_ID,
    CAMERA_PARAMS,
    IR_TEMP,

    LDMP_TEMP = 30,
    REF_SUMMARY_INFO,
    STREAMS_CAPACITY,
    EXT_PARAMS,
    LDMP_DOUBLE,
    CHIP_RESET,
    ENGINE_NAME,
    NTC_SVC,
    NTC_COEFF,
    DEP_RECTIFY,

    AE_AVG = 40,
    RAW_COMMAND_INT,
    SENSOR_RES,    //tmp  for  debug
    SENSOR_VOL,

    MASTER_SLAVE_MODE = 199,
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
    VCSEL_PD = 217,           //get_vcsel_pd
    ILLUM_POWER_TEST = 219,   

    DEFAULT_PARAMS = 220,   //默认参数
    OPS_PTR = 221,          //obc_ops_t指针
    AE = 222,               //AE开关
    DRIVER_IC_DETECT,
    REG16_RW,               //sensor 寄存器16bit读写
    DELAY_CIRCUIT_TIME = 225, //延时电路时间
    DELAY_CIRCUIT_ID,       //延时电路ID
    TX_A_B_POWER = 227,     //驱动模式 设置/获取    get_tx_a_b_power
    SENSOR_FREQUENCY_DUTY_CONFIG,     // frequency duty 等的配置  设置目前仅518在用
    SHUFFLE,                          // 0x01 enable shuffle（默认）；0x00 disable shuffle

    CHIP_ID = 230,               // sensor 唯一的id 信息 长度6Bytes  需要开流后调用
    BINNING_MODE = 231,          //binning mode   0: no scaling  1: 2x2 binning   2: 4x4 binning  see @binning_mode_t
    BURST_MODE = 232,            // 设置不同模式  可控制帧间隔
    FREQUENCY_MODE = 233,        // sensor 当前处于的频率模式 单频/双频，unsigned 8bit
    ITO_EM = 234,                //ITO通断，Tx工作时读取ITO状态值
    DRIVER_IC_REG8_RW = 235,     //driver ic 寄存器读写   地址为16bit，数据为:8bit
    VMGHI_VOLTAGE = 236,         //供电电压
    AF_DEPTH = 237,              //频率模式为AF时，获取13bit的深度值
    TOF_CONFIG = 238,            //tof滤波参数配置
    LASER_DRIVER_VOLTAGE = 239,  //激光驱动电压, unsigned 16bit 440~9000mv

	WINDOW_ORIGINY = 240,
	WINDOW_ORIGINX = 241,
	WINDOW_HEIGHT = 242,
	WINDOW_WIDTH = 243,
	ODD_DGAIN = 244,
	EVEN_DGAIN = 245,
	SUB_SAMP = 246,
	DEEPSLEEP_MODE = 247,
	HDR_ALGORITHM = 248,
	HIST_ALGORITHM = 249,
	MEDIAN_ALGORITHM = 250,
	EBC_ALGORITHM = 251,
	LSC_ALGORITHM = 252,
	SUB_SAMP_V = 253,
	CORRECTION_ALGORITHM = 254,

    DELAY_BOARD_POWER = 300,        //延时电路板电源开关 unsigned 8bit
    DELAY_BOARD_FW_VERSION = 301,   //延时电路固件版本 unsigned 8bit
    DELAY_BOARD_DELAY_SWITCH = 302, //延时电路延时开关 unsigned 8bit
    DELAY_BOARD_DELAY_TIME = 303,   //延时电路延时时间 unsigned 32bit
    DELAY_BOARD_TEMPERATURE = 304,  //延时电路温度 @see DelayBoardTemperature

	COMMAND_END,
}hw_command_t;


typedef enum 
{ 
    SENSOR_SINGLE_FREQ, 
    SENSOR_DUAL_FREQ,
    SENSOR_AF_FREQ
} SensorFrequencyMode;

typedef enum hw_ext_params_t {
	GET_SUPPORT_RESOLUTION,
	FOCAL_LENGTH,//focal length    焦距长度
	HW_EXT_CMD_COUNT  //子命令格式
}hw_ext_params;


typedef struct hw_ext_msg {
	int subcmd; //@hw_ext_params
	void* msg;  //@hw_stream_type_t
	void* p_data; //uint64_t
}hw_ext_msg_t;

typedef struct ntc_param {
	float ir;
	float ldmp;
}ntc_param_t;

typedef struct ob_suport_res_t {
	//ob_resolution_map* modes;
	uint64_t resolution_mask;
	int size;
}ob_suport_res;

typedef enum //TODO change enum member name, avoid confliction of ob_resolution.h
{
    RESOLUTION_1920_2880 = 1,//pleco dual freq
    RESOLUTION_960_1440 = 2,
    RESOLUTION_1920_1440 = 3,//pleco single freq
    RESOLUTION_960_720 = 4,
    RESOLUTION_1280_3840 = 5,//s5k33d dual freq
    RESOLUTION_640_1920 = 6,
    RESOLUTION_320_960 = 7,
    RESOLUTION_2560_3840 = 8,
    RESOLUTION_2560_7680 = 9,
    RESOLUTION_480_180 = 10,
    OB_TOF_RESOLUTION_640_480 = 200,//depth, amplitude, ir, etc, vga format
    OB_TOF_RESOLUTION_320_240 = 201,
    OB_TOF_RESOLUTION_160_120 = 202,
}ObTofResolution;





#endif