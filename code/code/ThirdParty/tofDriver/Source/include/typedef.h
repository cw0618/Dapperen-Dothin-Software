/*****************************************************************************
 *  Orbbec TOF SDK
 *  Copyright (C) 2019 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec TOF SDK.
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description:
 *
 ****************************************************************************/
#ifndef INCLUDE_TYPEDEF_H_
#define INCLUDE_TYPEDEF_H_
#include <stdint.h>
#include <string>
#include "tofinfo.h"



typedef struct obc_tof_frameinfo_
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
    int binning_mode;  // binning mode
    int freq_mode;     // frequency mode

    void CloneParam(struct obc_tof_frameinfo_ *other)
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
        this->freq_mode = other->freq_mode;
    }
}ObcTofFrameInfo;
using  obc_tof_frameinfo = ObcTofFrameInfo;


typedef struct obc_tof_frames_ {
    // 需分配frame_count个指向obc_tof_frameinfo的指针
    obc_tof_frameinfo** frames;
    // 表示传入帧的数量
    uint32_t frame_count;
    // 实际采集的帧数
    uint32_t real_cout;
}ObcFrameGroup;

using   ObcAmplitudeFrameGroup = ObcFrameGroup;
using   ObcMiddleFrameGroup = ObcFrameGroup;

/**
 *@brief 传输使用的数据位数
 */
typedef enum 
{
  PIXEL_FORMAT_RAW10 = 10,
  PIXEL_FORMAT_RAW11 = 11,
  PIXEL_FORMAT_RAW12 = 12,
  PIXEL_FORMAT_RAW14 = 14,
  PIXEL_FORMAT_RAW22 = 22,
} pixel_format;

/**
 *@brief 定义流类型
 */
typedef enum obc_stream_type {
  OBC_STREAM_DISABLED = 0,  //!< 流已经关闭
  OBC_STREAM_IR_FLOOD,      //!< 泛光灯流
  OBC_STREAM_IR_LASER,      //!< 散斑流
  OBC_STREAM_IR_EXT,        //!<
  OBC_STREAM_DEPTH,         //!< 深度流
  OBC_STREAM_BYPASS_ORI,    //!< 原始流
  OBC_STREAM_BYPASS_REF,    //!< 参考图流
  OBC_STREAM_BYPASS_IR,     //!< BYPASS_IR流
  OBC_STREAM_TEST_PATTERN,  //!< 测试流
  OBC_STREAM_ENGINE_MONO,   //!< NO USE
  OBC_STREAM_ENGINE_STEREO,
  OBC_STREAM_DEPTH_MONO,  //!< MONO流
  OBC_STREAM_COLOR,       //!< COLOR流
  OBC_STREAM_DISPARITY,   //!< 视差流

  OBC_STREAM_TOF_PHASE = 20,  //!< TOF 相位图
  OBC_STREAM_TOF_AMP,         //!< TOF 幅值图
  //OBC_STREAM_TOF_DEPTH,       //!< TOF 深度图

  OBC_STREAM_INVALID = -1,  //

} obc_stream_type_t;

typedef enum depth_shape_t {
  OBC_ORI_DISPARITY,
  OBC_SFT_DISPARITY,
  OBC_MIDSFT_DISPARITY,
  OBC_SFT_DEPTH
} depth_shape;


// vcsel driver IC type
#define DRIVER_IC_CXA4016              4016
#define DRIVER_IC_PHX3D                5016




typedef void (*capture_callback)(void *data, int width, int height,
                                 void *handle);

typedef struct obframecapture_t {
  void *handle;
  capture_callback callback;
} obframecapture;


/**  which 对于度信的可选择值
FIRMWARE_BIN2 = 0,
FIRMWARE_BIN1 = 1,
FIRMWARE_BIN3 = 2,
FIRMWARE_BIN4 = 3,
FIRMWARE_BIN5 = 4,
FIRMWARE_BIN6 = 5,
FIRMWARE_BIN7 = 6,
FIRMWARE_BIN8 = 7
*/

typedef struct _obconverter_fw_version_t {
    int which;
    unsigned long version[4];
} obconverter_fw_version;


/**
 * sensor binning mode
 */
typedef enum binning_mode_ {
    kBinningModeNoScaling = 0,
    kBinningMode2x2 = 1,
    kBinningMode4x4 = 2
}ObcSensorBinningMode;

/**
 *@brief  属性命令定义
 * libmx6x_2.0 通过属性系统实现对摄像头各种内部参数的读取和设置
 * 下面每一项中标注了指令的功能，以及每一项的数据类型，为指针类型
 */
typedef enum obc_command {
  OBC_MODULE_NAME = 0, /*!<参数数据类型 char*, sensor的具体芯片型号 如6300对应"2BC56300"  "5027"*/
  OBC_MODULE_VERSION, /*!<参数数据类型 uint32_t*， 版本号 */
  OBC_FW_VERSION,     /*!<参数数据类型 uint32_t*， 固件版本号 */
  OBC_LOAD_FW,  /*!<参数数据类型 const char*, 下载固件 */
  OBC_LOAD_REF, /*!<参数数据类型 const char*, 下载标定文件 */

  OBC_EEPROM_RW, /*!<参数数据类型  ObcEEPROMData *, EEPROM 读写透传接口 */
OBC_REG_RW, /*!<参数数据类型  obc_reg_map_t *, SPI 寄存器 读写透传接口 */
//OBC_REG_I2C_RW, /*!< 暂不支持*/

OBC_STREAM_MODE, /*!< 参数数据类型 stream_mode_t*，切换流类型*/
OBC_DATA_FMT, /*!<参数数据类型 int*,  数据流打包格式：0 - openni, 1 - original*/
OBC_FPS,      /*!<参数数据类型 uint32_t*, 帧率 */

OBC_AE_ENABLE,     /*!<参数数据类型 uint8_t*, AE使能 */
OBC_IR_EXP,        /*!<参数数据类型 int*, 曝光时间 */
OBC_IR_GAIN,       /*!<参数数据类型 int*, 增益 */
OBC_PULSE_WIDTH,   /*!<参数数据类型 int*, 脉宽 */
OBC_FLOOD_LED,     /*!<参数数据类型 uint8_t*, 泛光灯状态 */
OBC_LASER,         /*!<参数数据类型 uint8_t*, 激光灯状态 */
OBC_IR_SN,         /*!<参数数据类型 uint8_t*, IR_SN */
OBC_LDMP_SN,       /*!<参数数据类型 uint8_t*, LDMP_SN */
OBC_LASER_CURRENT, /*!<参数数据类型 int*, 激光灯电流 */
OBC_FLOOD_CURRENT, /*!<参数数据类型 int*, 泛光灯电流 */
OBC_WATCH_DOG,     /*!<NO SUPPORT*/
OBC_SLEEP_WAKE,    /*!<NO SUPPORT*/
OBC_PACK_FMT,      /*!<参数数据类型 int*, 数据流打包格式 */
OBC_DISP_BITS,     /*!<参数数据类型 int*,  视差输出位数*/
OBC_DISP_SUB_BITS, /*!<参数数据类型 int*, 视差亚像素输出位数 */
OBC_DEPTH_ROTATION_TYPE, /*!<参数数据类型 int*, 指定流的旋转角度  */
OBC_DEPTH_MIRROR_TYPE, /*!<参数数据类型 int*, 指定流的MIRROR状态  */
OBC_ENGINE_ID, /*!<参数数据类型 char *,深度引擎 device id 如"6300" "tof_sensors" */
OBC_CAMERA_PARAMS, /*!<参数数据类型  obc_Property_CameraParams_t*, 相机参数 */
OBC_IR_TEMP,          /*!<参数数据类型 float *,ir温度  */
OBC_LDMP_TEMP,        /*!<参数数据类型 float *,ldmp 温度 */
OBC_REF_SUMMARY_INFO, /*!<NO SUPPORT */
OBC_STREAMS_CAPACITY, /*!<NO SUPPORT */
OBC_EXT_PARAMS,       /*!<参数数据类型 hw_ext_msg_t*,扩展字段  */
OBC_LDMP_DOUBLE, /*!<参数数据类型 uint16_t*,双区控制，高8位：a区，低8位：b区 */
OBC_CHIP_RESET,  /*!<参数数据类型 no care*, sensor 复位  */
OBC_ENGINE_NAME, /*!<参数数据类型 char *,深度引擎名称  */
OBC_NTC_SVC,      /*!< 参数数据类型 obc_ntc_param_t * , 温控接口 设置ir，ldmp温度 */
OBC_NTC_COEFF,  /*!< 参数数据类型 obc_ntc_param_t * , 温控参数系数 */
OBC_DEP_RECTIFY,  /*!< 参数数据类型 int*  校正状态 */
OBC_AE_AVG,       /*!< 参数数据类型 uint32_t*  AE流明统计值 */
OBC_COMMAND_MAX_VALUE = 50, /*!< */

// define tof proprity
// TODO
OBC_TOF_MODULE_NAME = OBC_MODULE_NAME, /*!< 参数数据类型 char*， CHIP_NAME */
OBC_TOF_MODULE_VERSION = OBC_MODULE_VERSION, /*!<参数数据类型 uint32_t*， 版本号 */
OBC_TOF_LOAD_REF = OBC_LOAD_REF, /*!<参数数据类型 const char*, 加载标定图 */
OBC_TOF_STREAMS_CAPACITY = OBC_STREAMS_CAPACITY, /*!<参数数据类型 int*, 获取支持的流有那些 */
OBC_TOF_ENGINE_ID = OBC_ENGINE_ID,
OBC_TOF_STREAM_MODE = OBC_STREAM_MODE, /*!< 参数数据类型 stream_mode_t*，切换流类型*/
OBC_TOF_FPS = OBC_FPS, /*!<参数数据类型 uint32_t*, 帧率 */

OBC_TOF_VIDEO_MODES = 200, /*!<参数数据类型 obc_videomode*, 获取支持的video_mode ,size中包含的mode个数 */
OBC_TOF_SENSOR_TEMP = 201, /*!<参数数据类型 float*, sensor temp */
OBC_TOF_RX_TEMP = 202,     /*!<参数数据类型 float*, rx temp */
OBC_TOF_TX_TEMP = 203,     /*!<参数数据类型 float*, tx temp */
OBC_TOF_ILLUM_POWER = 204, /*!<参数数据类型 OBIllumPower*, 电流值, 0-255, 对应0~4.6安培 */
OBC_TOF_ILLUM_POWER_CTL = 205, /*!<参数数据类型 uint32_t*, 电流控制使能 */

OBC_TOF_INTEGRATION_TIME = 206, /*!<参数数据类型 uint32_t*, 积分时间 0-2000 */
OBC_TOF_MODULATION_FREQUENCY = 207, /*!<参数数据类型 uint32_t*, 调制频率 4-100MHZ*/
OBC_TOF_DATA_OUTPUT_MODE = 208, /*!<参数数据类型 uint32_t*, 数据输出模式(A-B,A+B,A,B,A&B) */
OBC_TOF_DUTY_CYCLE = 209,  /*!<参数数据类型 uint32_t*, 占空比*/
OBC_TOF_MIRROR_FLIP = 210, /*!<参数数据类型 uint32_t*, mirror flip*/
OBC_TOF_TEST_PATTERN = 211, /*!<参数数据类型 uint32_t*,  设置test_pattern,1:使能，0：关闭 */
OBC_TOF_SOFTWARE_TRIGGER = 212, /*!<不需要参数,  设置时开启一次 */
OBC_TOF_HARDWARE_TRIGGER = 213, /*!<不需要参数,  设置时开启一次 */
OBC_TOF_DUTY_CYCLE_STEP_LIST = 216, /*!<参数数据类型 OBDutyCycleSteps*, 查询占空比列表 */

OBC_TOF_MIPI_PACK_TYPE = OBC_DISP_BITS,       /*!< mipi 's transmission type :12,10*/
OBC_TOF_SENSOR_ID = 214, /*!<参数数据类型 uint32_t*, */

OBC_TOF_SENSOR_INFO = 215,      /*!<参数数据类型 ObcSensorInfo*, sensor info */
OBC_TOF_VCSEL_PD = 217,        /*!<参数数据类型 uint32_t*, vcsel pd 高16bit为PD_H2(pd_target_data)，低16bit为PD_BG */

OBC_TOF_ILLUM_POWER_TEST = 219,   /*!<参数数据类型 uint32_t*, 光功率测试模式 1:打开,0:关闭，会设置预设的频率模式、帧率及积分时间 */
OBC_TOF_DEFAULT_PARAMS = 220, //默认参数
OBC_OPS_PTR,  /*!<参数数据类型 void**, 操作函数参数 */
OBC_AE = 222,          /*!<参数数据类型 uint32_t*, AE 1:打开,0:关闭 */
OBC_DRIVER_IC_DETECT, /*!<参数数据类型 uint32_t*, driver ic id 实时检测，有效id大于0，可用来实时检测 tx的连接状态 */
OBC_REG16_RW, /*!<参数数据类型  obc_reg16_map_t *, sensor寄存器 读写透传接口 */
OBC_DELAY_CIRCUIT_TIME,  /*!<参数数据类型 uint32_t*, 延时电路时间 */
OBC_DELAY_CIRCUIT_ID,    /*!<参数数据类型 uint32_t*, 延时电路ID   实际16bit数据 */
OBC_TX_A_B_POWER = 227,    /*!<参数数据类型 ObcTxABPower*, Tx双驱动状态数据 */
OBC_SENSOR_FREQUENCY_DUTY_CONFIG,   /*!<参数数据类型 ObcSensorFreqDutyConfig  freq duty的配置  设置目前仅518在用 */
OBC_SHUFFLE,              /*!<参数数据类型 bool   0x01 enable shuffle（默认）；0x00 disable shuffle  全是N帧 */
OBC_SENSOR_CHIP_ID,       /*!<参数数据类型 char*  sensor 唯一的id 信息 长度6Bytes  需要开流后调用 */
OBC_BINNING_MODE = 231,         /*!<参数数据类型 uint8_t *,  binning mode   0: no scaling  1: 2x2 binning   2: 4x4 binning see @ObcSensorBinningMode */
OBC_BURST_MODE,           /*!<参数数据类型 uint8_t *, 设置不同模式,可控制帧间隔  0: mode1(normal) 1: mode2    2: mode3   other: error  */
OBC_FREQUENCY_MODE = 233,        /*!<参数数据类型 uint8_t *, sensor 当前处于的频率模式 单频/双频 */
OBC_ITO_EM = 234,             /*!<参数数据类型 uint16_t *, ITO通断，Tx工作时读取ITO状态值 */
OBC_DRIVER_IC_REG8_RW = 235,  /*!<参数数据类型 obc_reg16_map_t *, driver ic寄存器 读写透传接口， 地址为16bit，数据为:8bit ,addr 0x08,0x0f,0x10 */
OBC_VMGHI_VOLTAGE = 236,      /*!<参数数据类型 uint8_t *, 设置和获取供电电压，默认数值为20，对应1.6V，数值为60，对应1.2V，数值为0，对应1.8V */
OBC_AF_DEPTH = 237,           /*!<参数数据类型 uint16_t *, 频率模式为AF时设置和获取13bit的深度值 */
OBC_LASER_DRIVER_VOLTAGE = 239,  /*!<参数数据类型 uint16_t *, 激光驱动电压 440~9000 mv */
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

  OBC_DELAY_BOARD_POWER = 300,        /*!<参数数据类型 uint8_t  延时电路板电源开关 */
  OBC_DELAY_BOARD_FW_VERSION = 301,   /*!<参数数据类型 uint8_t  延时电路固件版本 */
  OBC_DELAY_BOARD_DELAY_SWITCH = 302, /*!<参数数据类型 uint8_t  延时电路延时开关 */
  OBC_DELAY_BOARD_DELAY_TIME = 303,   /*!<参数数据类型 uint32_t  延时电路延时时间 */
  OBC_DELAY_BOARD_TEMPERATURE = 304,  /*!<参数数据类型 ObcDelayBoardTemperature 延时电路温度 */

  OBC_TOF_SHOW_CONVERTER_STATUS_DIALOG = 501,   /*!<参数数据类型 HWND hwnd*, 打开转换器内部状态对话框 */
  OBC_TOF_CONVERTER_FW_VERSION = 502,   /*!<参数数据类型 obconverter_fw_version*, 转换器固件版本 */
  OBC_TOF_CONVERTER_NAME = 503,       /*!<参数数据类型 char*, 转换器名字： dothin, cx3 */
  OBC_TOF_SDK_VERSION = 504,          /*!<参数数据类型 uint32_t*, 本SDK的版本 */

  OBC_TOF_PROJECT_TEST = 1000,        /*!<参数数据类型 uint32_t*, 0:DEFAULT, 1:F201201_TX_AA, 2:F201201_TX_QC */

  OBC_TOF_COMMAND_MAX_VALUE /*!< */

} obc_command_t;

/**
 *@brief GPM 相关属性定义
 *
 */
typedef struct gpm {
  uint16_t gp_en;  // F3 31
  uint16_t global_point_x[4];
  uint16_t global_point_y[4];
  uint16_t gp_texture_t0;    // 10:0
  uint16_t gp_texture_t1;    // 22:12
  uint16_t gp_cont_zeros_t;  // F4 23:16
  uint16_t gp_active_t;      // 14:10
  uint16_t gp_ncost_t;       // 9:0
  uint16_t gp_ref_dy_neg_t;  // F8 22:12
  uint16_t gp_ref_dy_pos_t;  // 10:0
                             // lihonghua add
  int8_t gp_dy_status[34];       // 23:16
  uint16_t gp_ncost_status[34];  // 0:9
  int16_t gp_ref_dy;             // 27:16
  uint16_t gp_num;               // 12:8
  uint16_t gp_continuous_zeros;  // 7:0
} obc_gpm_t;

typedef struct engine_setting {
  uint16_t min_disparity;
  uint16_t right_win_sum;
  uint16_t win_radius;
  uint16_t texture0;
  uint16_t texture1;
} obc_engine_setting_t;

typedef struct filter_Uni {
  uint8_t en;
  uint8_t div;
} obc_filter_Uni_t;

/**
 *@brief 平滑滤波参数
 */
typedef struct filter_sm {
  uint8_t en;
  uint8_t win;
} obc_filter_sm_t;

/**
 *@brief 阈值滤波参数
 */
typedef struct filter_thre {
  uint8_t en;
  uint16_t thresold;
} obc_filter_thre_t;

/**
 *@brief 后置滤波参数
 */
typedef struct filter_pt {
  uint8_t en;
  uint16_t maxdiff;
  uint16_t ncos;
  uint16_t radius;
} obc_filter_pt_t;

/**
 *@brief 前置滤波
 *
 */
typedef struct prefilter_ctr {
  uint8_t ftzero;   // 28
  uint8_t wsz_sqr;  // 27:20
  uint8_t winsize;  // 19:16
  uint16_t w1sign;  // 14:0
  uint32_t pValue[15];
} obc_prefilter_ctr_t;

/**
 *@brief 寄存器
 *地址与对应的value
 */

typedef struct obc_reg_map {
  uint32_t addr;
  uint32_t value;
} obc_reg_map_t;

typedef struct obc_reg_16_map {
    uint16_t addr;
    uint16_t value;
} obc_reg_16_map_t;

/**
 *@brief 寄存器
 *地址与对应的value
 */

typedef struct obc_ntc {
  float ir;
  float ldmp;
} obc_ntc_param_t;

/**
 *@brief 扩展子命令结构体
 *  具体使用，可参考property示例
 */
typedef struct ob_hw_ext_msg {
  int subcmd;    //!<   子命令命令字
  void *msg;     //!<  消息字段
  void *p_data;  //!<  数据字段
} obc_hw_ext_msg_t;

/**
 *@brief  扩展子命令
 *在属性函数中使用结构体hw_ext_msg_t 包装
 */
typedef enum obc_command_ext {
  OBC_GET_SUPPORT_RESOLUTION, /*!<depressed */
  OBC_DEP_PREFILTER,          /*!<参数数据类型 obc_prefilter_ctr_t*, */
  OBC_DEP_PREFILTER_LEVEL,    /*!<参数数据类型 uint8_t*, */
  OBC_DEP_SMFILTER,           /*!<参数数据类型 obc_filter_sm_t*, */
  OBC_DEP_THRFILTER,          /*!<参数数据类型 obc_filter_thre_t*, */
  OBC_DEP_UNIDIV,             /*!<参数数据类型 obc_filter_Uni_t*, */
  OBC_DEP_PTFILTER,           /*!<参数数据类型 obc_filter_pt_t*, */
  OBC_DEP_ENGINE,             /*!<参数数据类型 obc_engine_setting_t*, */
  OBC_GPM,                    /*!<参数数据类型 obc_gpm_t*, GPM设置*/
  OBC_GPM_STATUS, /*!<参数数据类型 obc_gpm_t*, GPM状态，支持获取*/
  OBC_GPM_IS_ENABLE, /*!<参数数据类型 int*, GPM使能设置*/
  OBC_GET_SUPPORT_COUNT,
  OBC_ROTATE_DIRECTION = OBC_COMMAND_MAX_VALUE,  // RotateDirection
  OBC_SCALE_FACTOR,                              // ScaleFactor
  OBC_PREFILTER_PARAM,                           // PrefilterParam
  OBC_PREFILTER_BY_PASS,                         // PrefilterBypass
  OBC_DISPARITY_XRANGE,                          // DisparityXRange
  OBC_DISPARITY_YRANGE,                          // DisparityYRange
  OBC_MATCH_WINRADIUS,                           // MatchWinRadius
  OBC_MATCH_CORSE,                               // MatchCorse
  OBC_TEXTURE_THRESHOLD,                         // TextureThreshold
  OBC_THRESHOLD_FILTER,                          // ThresholdFilter
  OBC_SUB_PIXEL_COEFFS,                          // SubpixelCoeffs
  OBC_GPM_COORDINATES,                           // GPMCoordinates
  OBC_GPM_THRESHOLD,                             // GPMThreshold
  OBC_POST_FILTER_STATUS,                        // PostFilterStatus
  OBC_POST_FILTER_PARAM,                         // PostFilterParam
  OBC_SMOOTH_FILTER_STATUS,                      // SmoothFilterStatus
  OBC_SMOOTH_FILTER_PARAM,                       // SmoothFilterParam
  OBC_TEMP_COMP_COEFF,                           // TempCompCoeff
  OBC_CAMERA_EXTRINSIC,                          // CameraExtrinsic
  OBC_MONO_CAMERA_INSTRINSICS,                   // MonoCameraIntrinsics
  OBC_MONO_CAMERA_DISTO_LUT,                     // MonoCameraDistoLUT
  OBC_MONO_REFERENCE_IMAGE_U16,                  // MonoReferenceImageU16
  OBC_MONO_REFERENCE_IMAGE_U8,                   // MonoReferenceImageU8
  OBC_MONO_PREFILTER_REF_IMAGE8,                 // MonoPrefilterRefImage8
  OBC_STEREO_CAMERA_INTRINSICS,                  // StereoCameraIntrinsics
  OBC_STEREO_CAMERA_DISTO_LUT,                   // StereoCameraDistoLUT
  OBC_STEREO_REF_CAMERA,                         // StereoRefCamera
  // OBC_CAMERA_PARAMS,		//obc_Property_CameraParams_t
  OBC_COMMAND_EXT_MAX_VALUE

} obc_command_ext_t;

/**
 *@brief  video 模式定义
 *模式定义，包含分辨率及流类型
 * @see obc_stream_type_t
 **/
typedef struct {
  int width;
  int height;
  obc_stream_type_t type;
} obc_videomode;


typedef struct eeprom_data_ {
    int addr;       //eeprom 偏移地址
    int len;        // 数据长度
    uint8_t* data;  // 数据缓存区
}ObcEEPROMData;



/**
 *@brief  版本结构体定义
 */
typedef struct {
  /** Major version number, incremented for major API restructuring. */
  int major;
  /** Minor version number, incremented when significant new features added. */
  int minor;
  /** Maintenance build number, incremented for new releases that primarily
   * provide minor bug fixes. */
  int patchlevel;
  /** Build number. Incremented for each new API build. Generally not shown on
   * the installer and download site. */
  int build;

} version;

/**
 *@brief  SDK版本定义
 */
typedef struct {
  version firmware;
  version obdepth;
  version hardware;
  version logger;
  version device;
  uint32_t checkCode;
} sdk_ver;

/**
 *@brief  相机内参
 */
typedef struct OBCameraIntrinsic_S {
  float fx;
  float fy;
  float cx;
  float cy;
} OBCameraIntrinsic;

/**
 *@brief  相机外参
 */
typedef struct OBTransform_S {
  float rot[9];    // [r00,r01,r02;r10,r11,r12;r20,r21,r22]
  float trans[3];  // [t1,t2,t3]
} OBTransform;

/**
 *@brief  相机畸变参数
 */
typedef struct OBBrownDistortion_S {
  float k1;  // 径向畸变参数
  float k2;
  float k3;
  float p1;  // 切向畸变参数
  float p2;
} OBBrownDistortion;

/**
 *@brief  相机参数
 */
typedef struct OBCameraParams {
  OBCameraIntrinsic l_intr;  
  OBCameraIntrinsic r_intr;  // [fx,fy,cx,cy]
  OBTransform transform;
  OBBrownDistortion l_k;
  OBBrownDistortion r_k;
  // int is_mirror;
} obc_Property_CameraParams_t;

typedef enum PreciseUnit {
  OB_PRECISE_UNIT_0_1MM = 1,  //!< 单位，0.1mm
  OB_PRECISE_UNIT_1MM = 10    //!< 单位，1mm
} obc_precise_unit_t;

//定义ToF输出格式
typedef struct TOFVIDEOMODE : obc_videomode {
  int mipi_pack;
  ObcTofOutMode out_mode;
  uint32_t frequency[2];
} ObcTOFVideoMode;


// 0°  180°   90°   270°
#define OB_NUM_OF_FRAMES_IN_FRAMEGROUP 4
#define OB_PHASE_0_FRAME_INDEX 0
#define OB_PHASE_180_FRAME_INDEX 1
#define OB_PHASE_90_FRAME_INDEX 2
#define OB_PHASE_270_FRAME_INDEX 3

typedef struct request_frame_group_type {
  uint32_t frequency;         // 调制频率
  uint32_t duty_cycle;        // 占空比
  uint32_t integration_time;  // 积分时间
} ObcFreqMode;



/**
 *定义请求帧模式，num代码帧组的个数
 */

typedef struct obc_request_frame_mode {
  ObcFreqMode requestfreq[2];
  ObcFrequencyMode freqmode;
  obc_stream_type_t type;
  int width;
  int height;
  ObcTofOutMode out_mode;  // 帧输出模式

} ObcTofFrameMode;

class ObcFrameGroupRequestCmd {
 public:
  ObcFreqMode frameGroup_param_[2];
  ObcFrequencyMode freqmode_;
  obc_videomode video_mode_;

  ObcTofOutMode out_mode_;

  // 帧输出模式
  int freqs_;      // 频率的个数
  int frame_num_;  // 每种频率的帧数
  int real_width_;
  int real_height_;

  float duty_cycle_percent_;

  ObcFrameGroupRequestCmd();

  virtual ~ObcFrameGroupRequestCmd() {}

  void SetOutMode(ObcTofOutMode mode);
  ObcTofOutMode GetOutMode();

  void GetRealResolution(int &w, int &h);
  void clone(ObcFrameGroupRequestCmd &cmd);
  void clone(ObcTofFrameMode &cmd);

  virtual bool Equal(ObcTofFrameMode &cmd);
  virtual bool isSupportRequest(ObcFrameGroupRequestCmd &cmd) const;

  virtual bool isSupportRequest(ObcTofFrameMode &cmd) const;
};


class ObcSamSungFrameGroupRequestCmd : public ObcFrameGroupRequestCmd {
 public:
  ObcSamSungFrameGroupRequestCmd();
  ~ObcSamSungFrameGroupRequestCmd() {}
  virtual bool isSupportRequest( ObcFrameGroupRequestCmd &cmd) const;
  virtual bool isSupportRequest( ObcTofFrameMode &cmd)const ;
};

class ObcMLXFrameGroupRequestCmd : public ObcFrameGroupRequestCmd {
 
    ObcMLXFrameGroupRequestCmd();
public:

    ObcMLXFrameGroupRequestCmd(uint8_t vcsel_num);
    ~ObcMLXFrameGroupRequestCmd() {}
    virtual bool isSupportRequest(ObcFrameGroupRequestCmd &cmd) const;
    virtual bool isSupportRequest(ObcTofFrameMode &cmd) const;
};

class ObcIMX456FrameGroupRequestCmd : public ObcFrameGroupRequestCmd {

    ObcIMX456FrameGroupRequestCmd();
public:
    ObcIMX456FrameGroupRequestCmd(uint8_t vcsel_num);
    ~ObcIMX456FrameGroupRequestCmd() {}
    virtual bool isSupportRequest(ObcFrameGroupRequestCmd &cmd) const;
    virtual bool isSupportRequest(ObcTofFrameMode &cmd) const;
};

class ObcIMXFrameGroupRequestCmd : public ObcFrameGroupRequestCmd {
public:
    ObcIMXFrameGroupRequestCmd();
    ~ObcIMXFrameGroupRequestCmd() {}
    virtual bool isSupportRequest(ObcFrameGroupRequestCmd &cmd) const;
    virtual bool isSupportRequest(ObcTofFrameMode &cmd) const;
};

class ObcPLECOFrameGroupRequestCmd : public ObcFrameGroupRequestCmd
{
public:
    ObcPLECOFrameGroupRequestCmd();
    ~ObcPLECOFrameGroupRequestCmd() {};
    virtual bool isSupportRequest(ObcFrameGroupRequestCmd &cmd) const;
    virtual bool isSupportRequest(ObcTofFrameMode &cmd) const;
};


#if 1
typedef enum _ob_level {
  OB_LOG_TRACE = 1,
  OB_LOG_DEBUG = 2,
  OB_LOG_INFO = 3,
  OB_LOG_WARN = 4,
  OB_LOG_ERROR = 5
} ObLevel;

typedef enum _ob_log_out_location {
  OB_LOG_LOCATION_NULL = 0,
  OB_LOG_LOCATION_STDOUT = 1,
  OB_LOG_LOCATION_FILE = 2
} ObLogOutLocation;


typedef struct {
  int log_level;  // S_TRACE = 1,S_DEBUG = 2,S_INFO = 3, S_WARN = 4, S_ERROR = 5
  int log_location;  // 0: /dev/null  1: stdout 2: file
  std::string log_dir;
} ObcLoggerConfig;

// 环境配置结构体，内置默认参数
typedef struct device_env2_config_ {
  std::string driver_path;
  std::string driver_name_list;

  ObcLoggerConfig logger_config;
} ObcEnvConfig;
#endif

typedef struct s2dconfig_ {
  uint32_t nconstshift;
  // 多项式参数
  uint32_t nparamcoeff;
  uint32_t nshiftscale;
  uint32_t npixelsizefactor;

  // 最小深度限制
  uint32_t ndepth_mincutoff;
  // 最大深度限制
  uint32_t ndepth_maxcutoff;

  // 分辨率的宽，高, 分配临时交换buffer时需要
  uint32_t nresw;
  uint32_t nresh;
  /** ir sensor focal length 红外传感器焦距 */
  float ir_focallength;
  /*ir sensor cmos pixel size  红外像素尺寸*/
  float ir_pixelsize;

  uint32_t npixelsizeunit;

  uint32_t offset;

  /*
   * depth output fmt: 1, depth; 2 shift
   */
  int outfmt;
  /*
   * depth output  rotation angle
   */
  int nrotation;

  int32_t specklesize;
  int32_t maxdiff;
  uint32_t rotateDirection;
  /*
   * fx
   */
  float fx;
  /** The zero plane pixel size 标定平面的一个量化数值 */
  float baseline;
  /** The zero plane distance in depth units.标定距离 */
  float z0;
  /*
   * input data packed fmt: MIPI, 1; unpack, 0
   */
  uint8_t mipi_packed;
  /*
   * undistort switch: enable, 1, disable, 0
   */
  uint8_t enable_undistort;
  /*
   * output disparity whether of origin fmt:  origin, 1, openni, 0
   */
  uint8_t norigindisp;
  /*
   * disparity data bit wide,  raw14, 14; raw12, 12; raw10 10
   */
  uint8_t ndispbits;
  /*
   * disparity data bit wide,  raw14, 14; raw12, 12; raw10 10
   */
  uint8_t nsubdispbits;

  // mx6x芯片以多少位输出的数据: raw10, raw12, raw14， raw22
  uint8_t npixels;
  uint8_t enablefilter;
  uint8_t enablemedianfilter;

  /* 传输时的数据类型:@see mipi_transfer_datatype  10: raw10   12 :raw 12
   * 100:12bit as raw10*/
  uint8_t ntransfer_datatype;

  // #ifdef __USE_UNDISTORT_FROM_TIGER_
  /** The distance between the emitter and the Depth Cmos */
  float fEmitterDCmosDistance;
  /** The maximum possible shift value from this device. */
  uint32_t nDeviceMaxShiftValue;
  /** The maximum possible depth from this device (as opposed to a cut-off).*/
  uint32_t nDeviceMaxDepthValue;
  float fVerticalFOV;
  float fHorizontalFOV;
  // #endif

  // 后置滤波类型  arm  dsp  @see obc_softfilter_type_t
  int npost_filter_type;
  // 精度单位 10: 1mm, 1:0.1mm
  int nPreciseUnit;
} ob_s2dconfig_t;


#endif   // INCLUDE_TYPEDEF_H_