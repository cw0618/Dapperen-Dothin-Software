#ifndef ORBBEC_CHUNK_TYPE_H_
#define ORBBEC_CHUNK_TYPE_H_

#include <stdint.h>

typedef unsigned char uchar;

#define ORBBEC_STRING_MAX_LEN  20


#ifdef __cplusplus
extern "C"
{
#endif

#define OB_PNG_MAGIC_LEN (5)

typedef struct
{
    uint8_t length;
    uchar name[ORBBEC_STRING_MAX_LEN + 1];
} ORBBEC_STRING;

typedef enum
{
    CHUNK_DEVICE,
    CHUNK_IR_SENSOR,
    CHUNK_PROJECTOR,
    CHUNK_EXTERNAL,
    CHUNK_DRIVER,
    CHUNK_ENV,
    CHUNK_DATA,
    CHUNK_DISTORTION,
    CHUNK_OBPNGINFO,
	CHUNK_TOF_DEVICE,
	CHUNK_TOF_DRIVER,
	CHUNK_TOF_ENV,
	CHUNK_TOF_DATA,
	CHUNK_TOF_IRSENSOR,
} CHUNK_TYPE;

typedef enum
{
    MODEL_MONO_STRUCTURE_LIGHT,
    MODEL_STEREO_ACTIVE,
    MODEL_STEREO_PASSIVE,
	MODEL_TOF
} MODEL_TYPE;

typedef enum
{
    ROTATION_ZERO,
    ROTATION_CLOCKWISE_90,
    ROTATION_180,
    ROTATION_ANTICLOCK_90
} ROTATION_TYPE;

typedef enum
{
    MIRROR_NON,
    MIRROR_VERTICAL,
    MIRROR_HORIZONTAL,
    MIRROR_VERT_HORIZ
} MIRROR_TYPE;

typedef enum
{
    DATA_GRAY_SCALE_BITMAP,
    DATA_DISPARITY_MAP,
    DATA_DEPTH_MAP
} DATA_TYPE;

typedef enum
{
    DISTO_MODEL_PINHOLE,
    DISTO_MODEL_BROWN_K2T2,
    DISTO_MODEL_BROWN_K3T2,
    DISTO_MODEL_BROWN_K4T2,
    DISTO_MODEL_FISHEYE,
    DISTO_MODEL_SPHERICAL
} DISTO_MODEL_TYPE;

typedef enum
{
	PACK_FMT_OPENNI,
	PACK_FMT_ORIGINAL
} DATA_PACK_FMT;

typedef enum
{
	A_MINUS_B,
	A_PLUS_B,
	A,
	B,
	A_AND_B
} TOF_OUTPUT_MODE;

typedef enum
{
	PHASE_MAP,
	DEPTH_MAP,
	AMPLITUDE_MAP,
	IR_DATA_MAP,
} TOF_DATA_TYPE;

typedef struct{
    uint8_t obpng_magic[OB_PNG_MAGIC_LEN];
    ORBBEC_STRING tool_name;
    uint32_t obver;
}OB_PNG_INFO;

typedef struct
{
    /// @brief  设备名称.
    ORBBEC_STRING device;
    /// @brief  IR模组名.
    ORBBEC_STRING ir_model;
    /// @brief  IR Sensor名称.
    ORBBEC_STRING ir_sensor;
    /// @brief  LDMP模组名.
    ORBBEC_STRING ldmp_model;
    /// @brief  设备序列号.
    ORBBEC_STRING dev_serial;
    /// @brief  IR模组序列号.
    ORBBEC_STRING ir_serial;
    /// @brief  LDMP模组序列号.
    ORBBEC_STRING ldmp_serial;
    /// @brief  芯片名称.
    ORBBEC_STRING depth_engine;
    MODEL_TYPE model_type;
} DEVICE_INFO;

typedef struct
{
    uint16_t width;
    uint16_t height;
    ROTATION_TYPE rotation;
    MIRROR_TYPE mirror;
    /// @brief  Size of the pixel, mm为单位.
    float pixel_size;
    /// @brief  fx, fy, cx, cy 以像素为单位.
    float fx;
    float fy;
    float cx;
    float cy;
} IR_SENSOR_INFO;

typedef struct
{
    /// @brief  DOE设计编号.
    ORBBEC_STRING doe_model;
    /// @brief  每块的激光点数.
    uint16_t lp_per_block;
    /// @brief  激光块数.
    uint16_t num_block;
    /// @brief  发光波长.
    uint16_t wave_length;
} PROJECTOR_INFO;

typedef struct
{
    /// @brief  mm为单位.
    float baseline;
    float rx, ry, rz;
    float tx, ty, tz;
} EXTERNAL_INFO;

typedef struct
{
    /// @brief  固件版本号.
    ORBBEC_STRING firmware_ver;
    /// @brief  曝光增益.倍数.
    uint16_t ir_gain;
    /// @brief  曝光时间,微秒为单位.
    uint32_t ir_exposure;
    /// @brief  μA为单位.
    uint32_t laser_curr;
    /// @brief  微秒为单位.
    uint32_t laser_time;
} DRIVER_INFO;

typedef struct
{
    /// @brief  温度以0.01K为单位.
    uint16_t ir_temp;
    uint16_t ldmp_temp;
    /// @brief  可见光照度，单位lux.
    uint32_t illuminance;
} ENV_INFO;

typedef struct
{
    DATA_TYPE data_type;
    /// @brief  深度量化单位，以um为单位.
    float depth_uint;
    /// @brief  视差位数.
    uchar disp_bits;
    /// @brief  视差亚像素位数.
    uchar disp_sub_bits;
    DATA_PACK_FMT pack_fmt;
} DATA_INFO;

typedef struct
{
    DISTO_MODEL_TYPE disto_model;
    struct
    {
        float k1;
        float k2;
        float t1;
        float t2;
        float k3;
    ///< 仅使用与disto_model匹配的成员.
    } disto_data;
} DISTORTION_INFO;

typedef struct{
   uint32_t width;
   uint32_t height;
   uint8_t  bit_depth;
   uint8_t  color_type;
   uint8_t  compression;
   uint8_t  filter;
   uint8_t  interlace;
}IHDR;

typedef struct {
	/// @brief  设备名称.
	ORBBEC_STRING device;
	/// @brief  IR模组名.
	ORBBEC_STRING ir_model;
	/// @brief  lens名.
	ORBBEC_STRING lens;
	/// @brief  IR Sensor名称.
	ORBBEC_STRING ir_sensor;
	/// @brief  vcsel名称.
	ORBBEC_STRING vcsel;
	/// @brief  vcsel数量.
	uint8_t vcsel_num;
	/// @brief  vcsel波长.
	uint32_t vcsel_waveLength;
} TOF_DEVICE_INFO;

typedef struct {
	/// @brief  驱动版本.
	ORBBEC_STRING driver_version;
	/// @brief  拍摄时频率  MHZ
	uint32_t frequency;
	/// @brief  拍摄时相位
	uint32_t phase;
	///占空比 0-100
	uint8_t duty_cycle;
	/// @brief  积分事件  us
	uint32_t integration_time;
	/// @brief  激光峰值光功率  mW
	float vcsel_Pmax;
	/// @brief  0:A-B	1:A+B  2:A 3:B	4:A&B  10-- 4TAP_DUAL_FREQ
	uint8_t output_mode;
	/// 激光数量
	uint8_t vcsel_count;
    ///@brief 4016:cxa4016    5016:PHX3D
    uint16_t vcsel_driver_ic;
    ///双驱的模式： 0：关闭 1：a驱，2：b驱，3：a+b
    uint8_t tx_a_b_power;
    //激光电流，mA
    uint16_t illum_power;
} TOF_DRIVER_INFO;

typedef struct {
	/// @brief  ir温度以0.01K为单位.
	uint16_t ir_temp;
	/// @brief  激光温度以0.01K为单位.
	uint16_t vcsel_temp;
	/// @brief  拍摄延时电路温度以0.01K为单位.
	uint16_t dll_temp;
	/// @brief  照度.
	uint32_t illuminance;
	/// @brief  基线距离 mm.
	float baseline;
	/// @brief  拍摄距离 mm.
	float distance;
} TOF_ENV_INFO;

typedef struct {
	/// @brief  0-PHASE_MAP 1-DEPTH_MAP 2-AMPLITUDE_MAP
	uint8_t data_type;
	/// @brief  水平分辨率
	uint16_t width;
	/// @brief  垂直分辨率
	uint16_t height;
	/// @brief  数据比特数  8/16/32
	uint8_t data_bits;
	/// @brief  驱动版本.
	ORBBEC_STRING depth_version;
	/// @brief  深度量化单位 um.
	float depth_unit;
} TOF_DATA_INFO;

typedef struct {
	/// @brief  内参 fx, fy, cx, cy 以像素为单位.
	float fx;
	float fy;
	float cx;
	float cy;
} TOF_IRSENSOR_INFO;

#ifdef __cplusplus
}
#endif
#endif // !ORBBEC_CHUNK_TYPE_H_
