#ifndef MX6X_PROPETIES_H
#define MX6X_PROPETIES_H


/**
* Additional commands for Dothin OBC sensor devices
*
* @Author xiuluo wuzhu
* @Time 2019/08/23
*/


enum 
{
	OBC_MODULE_NAME,
	OBC_MODULE_VERSION,      //参数数据类型 uint32_t*，版本号
	OBC_FW_VERSION,          //参数数据类型 uint32_t*，固件版本号
	OBC_LOAD_FW,             //参数数据类型 const char*,下载固件
	OBC_LOAD_REF,            //参数数据类型 const char*,下载固件

	OBC_EEPROM_RW,           //参数数据类型 eeprom_data_t *, EEPROM读写透传接口
	OBC_REG_RW,              //参数数据类型 obc_reg_map_t *, SPI寄存器读写透传接口
	OBC_STREAM_MODE,         //参数数据类型 stream_mode_t*，切换流类型
	OBC_DATA_FMT,            //参数数据类型 int*, 数据流打包格式：0-openni,1-original

	OBC_FPS,                 //参数数据类型 int*,帧率
	OBC_AE_ENABLE,           //参数数据类型 uint8_t*,AE使能
	OBC_IR_EXP,              //参数数据类型 int*,曝光时间
	OBC_IR_GAIN,             //参数数据类型 int*,增益
	OBC_PULSE_WIDTH,         //参数数据类型 int*,脉宽

	OBC_FLOOD_LED,           //参数数据类型 uint8_t*,泛光灯状态
	OBC_LASER,               //参数数据类型 uint8_t*,激光灯状态
	OBC_IR_SN,               //参数数据类型 uint8_t*,IR_SN
	OBC_LDMP_SN,             //参数数据类型 uint8_t*,LDMP_SN
	OBC_LASER_CURRENT,       //参数数据类型 int*,激光灯电流

	OBC_FLOOD_CURRENT,       //参数数据类型 int*,泛光灯电流
	OBC_WATCH_DOG,           //NO SUPPORT
	OBC_SLEEP_WAKE,          //NO SUPPORT
	OBC_PACK_FMT,            //参数数据类型 int*,数据流打包格式
	OBC_DISP_BITS,           //参数数据类型 int*,视差输出位数

	OBC_DISP_SUB_BITS,       //参数数据类型 int*,视差亚像素输出位数
	OBC_DEPTH_ROTATION_TYPE, //参数数据类型 int*,指定流的旋转角度
	OBC_DEPTH_MIRROR_TYPE,   //参数数据类型 int*,指定流的MIRROR状态
	OBC_ENGINE_ID,           //参数数据类型 char *,深度引擎 device id
	OBC_CAMERA_PARAMS,       //参数数据类型 obc_Property_CameraParams_t*,相机参数
	
	OBC_IR_TEMP,             //参数数据类型 float *,ir温度 读
	OBC_LDMP_TEMP,           //参数数据类型 float *,ldmp 温度 读
	OBC_REF_SUMMARY_INFO,    //NO SUPPORT
	OBC_STREAMS_CAPACITY,    //NO SUPPORT
	OBC_EXT_PARAMS,          //参数数据类型 hw_ext_msg_t*,扩展字段
	
	OBC_LDMP_DOUBLE,         //参数数据类型 uint16_t*,双区控制，高8位：a区，低8位：b区
	OBC_CHIP_RESET,          //参数数据类型 no care*, sensor复位 设置
	OBC_ENGINE_NAME,         //参数数据类型 char *,深度引擎名称 读
	OBC_NTC_SVC,             //参数数据类型 obc_ntc_param_t *,温控接口设置ir，ldmp温度 读写
	OBC_NTC_COEFF,           //参数数据类型 obc_ntc_param_t *,温控参数系数 读写
	
	OBC_DEP_RECTIFY,         //参数数据类型 int* 校正状态 设置
	OBC_AE_AVG,              //参数数据类型 uint32_t* AE流明统计值
	OBC_RAW_COMMAND_INT,
	OBC_SENSOR_RES,          //tmp  for  debug
	OBC_SENSOR_VOL,
	OBC_ENGINE_IS_VALID,
	OBC_SOFT_FILTER,         //softfilter

    OBC_SUPPORT_VIDEO_MODES = 200,
    OBC_SENSOR_TEMP = 201,
    OBC_RX_TEMP = 202,
    OBC_TX_TEMP = 203,
    OBC_ILLUM_POWER = 204,
    OBC_ILLUM_POWER_CTL = 205,
    OBC_INTEGRATION_TIME = 206,
    OBC_MODULATION_FREQUENCY = 207,
    OBC_DATA_OUTPUT_MODE = 208,
    OBC_DUTY_CYCLE = 209,
    OBC_MIRROR_FLIP = 210,
    OBC_TEST_PATTERN = 211,
    OBC_SOFTWARE_TRIGGER = 212,
    OBC_HARDWARE_TRIGGER = 213,

    OBC_SENSOR_ID = 214,        //unsigned 32bit
    OBC_SENSOR_INFO = 215,      // EXPAND_DATASIZE
    OBC_DUTY_CYCLE_LIST = 216,
    OBC_VCSEL_PD = 217,   //get_vcsel_pd
    OBC_ILLUM_POWER_TEST = 219, //imx456_illum_power_test
    OBC_DEFAULT_PARAMS = 220, //默认参数
    OBC_OPS_PTR = 221,      //  obc_ops_t指针
    OBC_AE = 222,          // AE  开关
    OBC_DRIVER_IC_DETECT,
    OBC_REG16_RW,           //sensor 寄存器16bit读写
    OBC_DELAY_CIRCUIT_TIME = 225, //延时电路时间
    OBC_DELAY_CIRCUIT_ID,       //延时电路ID
    OBC_TX_A_B_POWER,       //驱动模式 设置/获取    get_tx_a_b_power
    OBC_SENSOR_FREQUENCY_DUTY_CONFIG,     // frequency duty 等的配置  设置目前仅518在用
    OBC_SHUFFLE,          // 0x01 enable shuffle（默认）；0x00 disable shuffle
    OBC_CHIP_ID=230,          // sensor 唯一的id 信息 长度6Bytes  需要开流后调用
    OBC_BINNING_MODE=231,     //binning mode   0: no scaling  1: 2x2 binning   2: 4x4 binning  see @ObBinningMode
    OBC_BURST_MODE=232,       // 设置不同模式  可控制帧间隔
    OBC_FREQUENCY_MODE=233,       //sensor 当前处于的频率模式 单频/双频，unsigned 8bit
    OBC_ITO_EM=234,            //ITO通断，Tx工作时读取ITO状态值
    OBC_DRIVER_IC_REG8_RW=235,  //driver ic 寄存器读写   地址为16bit，数据为:8bit
    OBC_VMGHI_VOLTAGE=236,      //供电电压
    OBC_AF_DEPTH=237,           //频率模式为AF时，获取13bit的深度值
	TOF_CONFIG = 238,            //tof滤波参数配置
	LASER_DRIVER_VOLTAGE = 239,  //激光驱动电压, unsigned 16bit 440~9000mv
	OBC_WINDOW_ORIGINY = 240,
	OBC_WINDOW_ORIGINX = 241,
	OBC_WINDOW_HEIGHT = 242,
	OBC_WINDOW_WIDTH = 243,
	OBC_ODD_DGAIN = 244,
	OBC_EVEN_DGAIN = 245,
	OBC_SUB_SAMP = 246,
	OBC_DEEPSLEEP_MODE = 247,
	OBC_HDR_ALGORITHM = 248,
	OBC_HIST_ALGORITHM = 249,
	OBC_MEDIAN_ALGORITHM = 250,
	OBC_EBC_ALGORITHM = 251,
	OBC_LSC_ALGORITHM = 252,
	OBC_SUB_SAMP_V = 253,
	OBC_CORRECTION_ALGORITHM = 254,

	OBC_DELAY_POWER = 300,        //延时电路板电源开关 unsigned 8bit
	OBC_DELAY_FW_VERSION = 301,   //延时电路固件版本 unsigned 8bit
	OBC_DELAY_DELAY_SWITCH = 302, //延时电路延时开关 unsigned 8bit
	OBC_DELAY_DELAY_TIME = 303,   //延时电路延时时间 unsigned 32bit
	OBC_DELAY_TEMPERATURE = 304,  //延时电路温度 @see DelayBoardTemperature
	OBC_GROUP_FRAME_MODE = 305,
	OBC_PHASE_COUNT=310,           //一个帧多少个相位组成
	OBC_DEVICE_RESET = 312,         //度信复位
    OBC_COMMAND_END,    //ENGINE_IS_VALID
};





enum {
	OBC_EXT_DEP_PREFILTER = 256,   //参数数据类型 obc_prefilter_ctr_t*
	OBC_EXT_DEP_PREFILTER_LEVEL,   //参数数据类型 uint8_t*
	OBC_EXT_DEP_SMFILTER,          //参数数据类型 obc_filter_sm_t*
	OBC_EXT_DEP_THRFILTER,         //参数数据类型 obc_filter_thre_t*
	OBC_EXT_DEP_UNIDIV,            //参数数据类型 obc_filter_Uni_t*
	OBC_EXT_DEP_PTFILTER,          //参数数据类型 obc_filter_pt_t*,
	OBC_EXT_DEP_ENGINE,            //参数数据类型 obc_engine_setting_t*
	OBC_EXT_GPM,                   //参数数据类型 obc_gpm_t*, GPM设置
	OBC_EXT_GPM_STATUS,            //参数数据类型 obc_gpm_t*, GPM状态，支持获取
	OBC_EXT_GPM_IS_ENABLE,         //参数数据类型 int*, GPM使能设置
};






typedef struct eeprom_data{
	int addr;
	int len;
	uint8_t* data;
}eeprom_data_t;

//SPI read/write
typedef struct obc_reg_map{
	uint32_t addr;
	uint32_t value;
}obc_reg_map_t;

typedef struct OBCSerialNumber
{
	uint32_t size;
	uint8_t SN[32];

}OBCSerialNumber;


typedef struct OBCameraIntrinsic_S
{
	float fx;
	float fy;
	float cx;
	float cy;
} OBCameraIntrinsic;


typedef struct OBTransform_S
{
	float rot[9];    //[r00,r01,r02;r10,r11,r12;r20,r21,r22]
	float trans[3];  //[t1,t2,t3]
} OBTransform;


typedef struct OBBrownDistortion_S
{
	float k1;
	float k2;
	float k3;
	float p1;
	float p2;
} OBBrownDistortion;


typedef struct OBC_CameraParams
{
	OBCameraIntrinsic l_intr; //
	OBCameraIntrinsic r_intr; //[fx,fy,cx,cy]
	OBTransform     transform;
	OBBrownDistortion l_k;
	OBBrownDistortion r_k;
	//int is_mirror;
}OBC_CameraParams;


typedef struct LdmpDouble_T
{
	uint8_t a_en;
	uint8_t b_en;
}LdmpDouble_T;


typedef struct obc_ntc {
	float ir;
	float ldmp;
}obc_ntc_param_t;



/*Ext T*/

/**
*@brief 前置滤波
*
*/
typedef struct prefilter_ctr
{
	uint8_t ftzero; //28
	uint8_t wsz_sqr; //27:20
	uint8_t winsize; //19:16
	uint16_t w1sign; //14:0
	uint32_t pValue[15];
}obc_prefilter_ctr_t;


typedef struct  obc_prefilter_t
{
	obc_prefilter_ctr_t obc_Prefilter;
	uint8_t level;

}obc_prefilter_t;


/**
*@brief 平滑滤波参数
*/
typedef struct filter_sm
{
	uint8_t en;
	uint8_t win;
}obc_filter_sm_t;


/**
*@brief 阈值滤波参数
*/
typedef struct filter_threshold
{
	uint8_t en;
	uint16_t thresold;
}obc_filter_thre_t;


/**
*@brief 后置滤波参数
*/
typedef struct filter_pt
{
	uint8_t en;
	uint16_t maxdiff;
	uint16_t ncos;
	uint16_t radius;
}obc_filter_pt_t;


typedef struct filter_Uni
{
	uint8_t en;
	uint8_t div;
}obc_filter_Uni_t;


typedef struct engine_setting
{
	uint16_t min_disparity;
	uint16_t right_win_sum;
	uint16_t win_radius;
	uint16_t texture0;
	uint16_t texture1;
}obc_engine_setting_t;


/**
*@brief GPM 相关属性定义
*/
typedef struct gpm
{
	uint16_t gp_en; //F3 31
	uint16_t global_point_x[4];
	uint16_t global_point_y[4];
	uint16_t gp_texture_t0; //10:0
	uint16_t gp_texture_t1;//22:12
	uint16_t gp_cont_zeros_t; //F4 23:16
	uint16_t gp_active_t;//14:10
	uint16_t gp_ncost_t; //9:0
	uint16_t gp_ref_dy_neg_t; //F8 22:12
	uint16_t gp_ref_dy_pos_t;//10:0

	int8_t gp_dy_status[34];//23:16
	uint16_t gp_ncost_status[34];//0:9
	int16_t gp_ref_dy; //27:16
	uint16_t gp_num;//12:8
	uint16_t gp_continuous_zeros;//7:0
}obc_gpm_t;


typedef struct
{
    uint16_t a_state;   //A区状态
    uint16_t b_state;   //B区状态
} ObTxABPower;


//从固件获取到的tof参数信息
typedef struct
{
    uint32_t frequency;
    uint32_t duty_cycle;
    int32_t integrantion_time;
}ObTofOriginalParam;


//定义驱动IC id
typedef enum
{
    DRIVER_IC_CXA4026 = 4026,     //Polaris B Tx
    DRIVER_IC_CXA4016 = 4016,     //Taishan DVT1 Tx
    DRIVER_IC_PHX3D3021AA = 5016, //Taishan DVT2 Tx
    DRIVER_IC_PHX3D3021CB = 5017, //Taishan DVT3 Tx
    DRIVER_IC_DW9912 = 9912,      //DongWoon
}ObDriverICTypeId;


typedef enum
{
    TOF_OUTPUT_MODE_A_SUB_B = 0,      //!< 单位，A-B
    TOF_OUTPUT_MODE_A_ADD_B = 1,  //!< 单位，A+B
    TOF_OUTPUT_MODE_A = 2,        //!< 单位，A
    TOF_OUTPUT_MODE_B = 3,        //!< 单位，B
    TOF_OUTPUT_MODE_A_AND_B = 4,  //!< 单位，A&B
    TOF_OUTPUT_MODE_S5K33D_4Tap_DualFreq = 10,  //!< 单位，4-tap, dual-freq., shuffle mode
    TOF_OUTPUT_MODE_3TAP = 20,  //!< 3-tap, pleco, shuffle mode
}ObOutputMode;


typedef enum
{
    TOF_BINNING_MODE_1X1,
    TOF_BINNING_MODE_2X2,
    TOF_BINNING_MODE_4X4,
}ObBinningMode;


//tof频率模式
typedef enum
{
    TOF_SINGLE_FREQUENCY_MODE,
    TOF_DUAL_FREQUENCY_MODE,
    TOF_AF_FREQUENCY_MODE,
}ObFrequencyMode;


//tof sensor id  
typedef enum
{
    TOF_SENSOR_ID_S5K33D = 0x303d,
    TOF_SENSOR_ID_MLX75027 = 0x5027,
    TOF_SENSOR_ID_IMX516 = 0x0516,
    TOF_SENSOR_ID_IMX456 = 0x0456,
    TOF_SENSOR_ID_IMX518 = 0x0518,
    TOF_SENSOR_ID_IMX278 = 0x0278,
    TOF_SENSOR_ID_PLECO = 0x0123,
    TOF_SENSOR_ID_RK1608_S5K33D = 0x313d,
    TOF_SENSOR_ID_RK1608_PLECO = 0x1123,
	TOF_SENSOR_ID_GAEA = 0xe100,
} ObSensorId;

#define OB_DUTY_CYCLE_LIST_NUM  31
typedef struct ObDutyCycleStep
{
    uint32_t frequency;
    float  duty_cycle_steps[OB_DUTY_CYCLE_LIST_NUM];
}ObDutyCycleStep;

typedef struct ObSensorInfo
{
    uint32_t embedded_data_size;
    uint8_t  vcsel_num;
    uint16_t vcsel_driver_id;   //@obc_vcsel_driver_ic_type
    uint16_t sensor_id;
    uint16_t project_id;
} ObSensorInfo;


typedef struct ObSensorCommonInfo
{
    ObSensorInfo sensor_info;
    uint8_t binning_mode;
    uint8_t frequency_mode;
} ObSensorCommonInfo;


typedef struct ObTofFrameInfo
{
    // 包括扩展信息和相位有效数据
    char* buffer;
    uint32_t buffer_size;
    // 相位数据的起始地址,buffer的偏移地址
    char* data_offset;
    int width;
    int height;
    uint32_t frequency;     // 调制频率
    float duty_cycle;       // 占空比
    uint32_t integration_time;  // 积分时间

    uint32_t sensor_type;
    uint32_t out_mode;      // 数据模式 @see ObcTofOutMode
    uint32_t mipi_pack;     // 数据打包格式
    uint32_t phase_map;     // 相位
    float sensor_temp;      // sensor 温度
    float driver_ic_temp;   // 驱动ic tx 温度
    float rx_temp;          // rx温度
    float time_delay_circuit_temp;  // 延时电路温度
    float reserved;         // 保留
    uint8_t frame_index;    // 帧序号
    uint8_t group_index;    // 组序号，没有组概念时为-1
    uint32_t frame_type;
    uint32_t driver_ic_pd;  // 驱动IC获取的PD读数
    uint32_t driver_ic_error_flag;  // 驱动IC获取的error状态
    uint32_t binning_mode;  // binning mode

    void CloneParam(struct ObTofFrameInfo *other)
    {
        this->width = other->width;
        this->height = other->height;
        this->driver_ic_temp = other->driver_ic_temp;
        this->duty_cycle = other->duty_cycle;
        this->frame_index = other->frame_index;
        this->frame_type = other->frame_type;
        this->frequency = other->frequency;
        this->group_index = other->group_index;
        this->integration_time = other->integration_time;
        this->mipi_pack = other->mipi_pack;
        this->out_mode = other->out_mode;
        this->phase_map = other->phase_map;
        this->rx_temp = other->rx_temp;
        this->sensor_temp = other->sensor_temp;
        this->sensor_type = other->sensor_type;
        this->time_delay_circuit_temp = other->time_delay_circuit_temp;
        this->driver_ic_pd = other->driver_ic_pd;
        this->driver_ic_error_flag = other->driver_ic_error_flag;
        this->binning_mode = other->binning_mode;
    }
}ObTofFrameInfo;


typedef struct ObFrameGroup
{
    // 需分配frame_count个指向ObTofFrameInfo的指针
    ObTofFrameInfo** frames;
    // 帧数内存容量
    uint32_t frame_count;
    // 有效的帧数
    uint32_t real_count;
}ObFrameGroup;
typedef struct obc_reg_16_map {
	uint16_t addr;
	uint16_t value;
} obc_reg_16_map_t;

typedef struct request_frame_group_type {
	uint32_t frequency;         // 调制频率
	uint32_t duty_cycle;        // 占空比
	uint32_t integration_time;  // 积分时间
} ObcFreqMode;

typedef enum
{
	OBC_SINGLE_FREQUENCY,
	OBC_DUAL_FREQUENCY,
	OBC_AF_FREQUENCY,
} ObcFrequencyMode;

typedef enum ObcTofOutMode_ {
	OB_TOF_OUT_MODE_A_B = 0,      //!< 单位，A-B
	OB_TOF_OUT_MODE_A_ADD_B = 1,  //!< 单位，A+B
	OB_TOF_OUT_MODE_A = 2,        //!< 单位，A
	OB_TOF_OUT_MODE_B = 3,        //!< 单位，B
	OB_TOF_OUT_MODE_A_AND_B = 4,  //!< 单位，A&B

	OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ = 10,  //!< 单位，4-tap, dual-freq., shuffle mode
	OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ_NO_SHUFFLE_4FRAME = 11,  //!< 单位，4-tap, dual-freq., no-shuffle mode  4帧
	OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ_NO_SHUFFLE = 12,  //!< 单位，4-tap, dual-freq., no-shuffle mode  2帧
	OB_TOF_OUT_MODE_S5K_4TAP_SINGLE_FREQ_SHUFFLE = 14,  //!< 单位，4-tap, dual-freq., shuffle mode   2帧
	OB_TOF_OUT_MODE_S5K_4TAP_SINGLE_FREQ_NO_SHUFFLE = 15,  //!< 单位，4-tap, dual-freq., no-shuffle mode 1帧

	OB_TOF_OUT_MODE_3TAP = 20,  //!< 3 tap, pleco, single_freq: 3 frames, dual_freq: 6 frames
} ObcTofOutMode;

typedef struct obc_request_frame_mode {
	ObcFreqMode requestfreq[2];
	ObcFrequencyMode freqmode;
	int width;
	int height;
	ObcTofOutMode out_mode;  // 帧输出模式
} ObcTofFrameMode;

#endif //MX6X_PROPETIES_H


