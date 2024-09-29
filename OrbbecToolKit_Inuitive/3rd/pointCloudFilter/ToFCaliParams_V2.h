/*****************************************************************************
*
*****************************************************************************/
#ifndef __CALIB_PARAM_H__
#define __CALIB_PARAM_H__

#pragma pack(push,1)
#include <cstdint>
////////////////////////////////////////////////////////////////////////////
///@brief 常规头文件信息
enum TOF_CALIPARAMS_TYPE_E
{
	TOF_CALIPARAMS_LUT = 0, //查找表
	TOF_CALIPARAMS_COE = 1, //系数拟合
	TOF_CALIPARAMS_BLOCK = 2, //分块拟合
};
typedef uint8_t TofCaliparamsTypeEnum;

enum TOF_UNDISTO_FLAG_TYPE_E
{
	TOF_UNDISTO_OFF = 0, //关闭
	TOF_UNDISTO_ON = 1, //开启
};
typedef uint8_t CamUndistoFlagEnum;

enum TOF_FREQUENCY_TYPE_E
{
	TOF_SINGLE_FREQUENCY_SHUFFLE = 0, //单频SHUFFLE模式
	TOF_DUAL_FREQUENCY_SHUFFLE = 1, //双频SHUFFLE模式
	TOF_SINGLE_FREQUENCY_NONSHUFFLE = 2, //单频NONSHUFFLE
	TOF_DUAL_FREQUENCY_NONSHUFFLE = 3, //双频NONSHUUFFLE模式
};
typedef uint8_t TofFreqTypeEnum;

typedef struct _ObTofHeader
{
	TofFreqTypeEnum Type;
	char identity[8];
	TofCaliparamsTypeEnum caliparams_type;
} ObTofHeader;

////////////////////////////////////////////////////////////////////////////
///@brief 器件信息
#define SERIAL_NO_MAX_LEN   (32)
typedef struct _ObToFDeviceInfo
{
	char dev_sn[SERIAL_NO_MAX_LEN];// 模组序列号
	char ir_sn[SERIAL_NO_MAX_LEN];// ir 序列号
	char vcsel_sn[SERIAL_NO_MAX_LEN];// 激光序列号
} ObToFDeviceInfo;

enum CAM_DISTO_MODEL_E
{
	CAM_DISTO_MODEL_PINHOLE = 0,
	CAM_DISTO_MODEL_BROWN_K2T2 = 1,
	CAM_DISTO_MODEL_BROWN_K3T2 = 2,
	CAM_DISTO_MODEL_BROWN_K4T2 = 3,
	CAM_DISTO_MODEL_FISHEYE = 4,
	CAM_DISTO_MODEL_SPHERE = 5,
	CAM_DISTO_MODEL_RECT_POLY = 6,
	CAM_DISTO_MODEL_GRID_RADIAL_BASE = 7,
};
typedef uint8_t CamDistoModelEnum;

////////////////////////////////////////////////////////////////////////////
///@brief IR的内参信息
typedef struct ObToFIRIntrinsic_
{
	uint16_t width;          // 原始宽
	uint16_t height;         // 原始高
	float pixel_size;     // 像素大小(um)
	float fx;
	float fy;
	float cx;
	float cy;
	CamDistoModelEnum disto_model;    // 畸变模型

} ObToFIRIntrinsic;

/**
* 这个结构体不能直接写二进制到数据块，要分情况写
*/
typedef struct _ObDistoCoeffs
{
	CamUndistoFlagEnum undisto_flag;
	CamDistoModelEnum disto_model;    // 畸变模型
	float k1;
	float k2;
	float t1;
	float t2;
	float k3;
	float k4;
	float k5;
	float k6;
	float k7;
	float k8;
} ObDistoCoeffs;

////////////////////////////////////////////////////////////////////////////
///@brief ToF Sensor 的参数信息
typedef struct _ObToFSensorParams
{
	uint16_t freq_mod;// 调制频率
	uint16_t integral_time;//积分时间
}ObToFSensorParams;

////////////////////////////////////////////////////////////////////////////
///@brief ToF 温度补偿信息
typedef struct _ObToFTempCompensation
{
	uint16_t vcsel_coeff_size;// vcsel温漂系数数量
	uint16_t sensor_coeff_size;// sensor温漂系数数量
	float *vcsel_temp_comp_coeff;  //vcsel 温度补偿系，从低阶(1阶)到高阶排序,内存由内部释放
	float *sensor_temp_comp_coeff;  //ir 温度补偿系数，从低阶（1阶)到高阶排序,内存由内部释放
	float temp_dist_offset;  //温度引起的零漂
}ObToFTempCompensation;

typedef struct _ObToFDistanceCalib
{
	uint8_t noshufl_num;
	float *module_offset;  // 模组offset
	float *global_offset;  //整机offset

						   //查找表
	uint32_t wiggling_lut_size;// wiggling lut 的尺寸
	uint32_t fppn_lut_size;// fppn lut 尺寸
	int16_t *wiggling_lut; // wiggling 查找表内容, 内存由内部释放
	int16_t *fppn_lut;//fppn 查找表内容, 内存由内部释放

					  //系数拟合
	uint32_t wiggling_coe_size;// wiggling lut 的尺寸
	uint32_t fppn_coe_size;// fppn lut 尺寸
	float *wiggling_coe; // wiggling 查找表内容, 内存由内部释放
	float *fppn_coe;//fppn 查找表内容, 内存由内部释放
}ObToFDistanceCalib;

#define TOF_FREQ_NUM (2)
typedef struct _ObToFParam
{
	ObTofHeader hdr; // Tof param 头文件
	ObToFDeviceInfo dev_info; // 器件信息
	ObToFIRIntrinsic ir_intrin; // ir的内参信息
	ObDistoCoeffs distort; //畸变的类型
	ObToFSensorParams sensor_param[TOF_FREQ_NUM]; //sensor的参数
	ObToFTempCompensation temp_comp[TOF_FREQ_NUM]; //ToF的温度补偿
	ObToFDistanceCalib dist_calib[TOF_FREQ_NUM];// ToF的距离补偿
}ObToFParam;
#pragma pack(pop)
#endif