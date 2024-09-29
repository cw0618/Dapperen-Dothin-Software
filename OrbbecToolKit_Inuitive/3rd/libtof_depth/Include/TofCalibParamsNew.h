/*****************************************************************************
*
*****************************************************************************/
#ifndef __CALIB_PARAMS_H__
#define __CALIB_PARAMS_H__

#include <cstdint>
#include "TofDefines.h"

#pragma pack(push,1)
////////////////////////////////////////////////////////////////////////////
//@brief 常规头文件信息
typedef struct __HeaderInfo__
{
	uint8_t freq_config;		//	0 单频SHUFFLE模式; 1 双频SHUFFLE模式; 2 单频NONSHUFFLE; 3 双频NONSHUUFFLE模式
	char identity[8];
	uint8_t caliparams_type;	//	COE/LUT
} HeaderInfo;

////////////////////////////////////////////////////////////////////////////
//@brief 器件信息
typedef struct __DeviceInfo__
{
	char dev_sn[SERIAL_NO_MAX_LEN];		// 模组序列号
	char ir_sn[SERIAL_NO_MAX_LEN];		// ir 序列号
	char vcsel_sn[SERIAL_NO_MAX_LEN];	// 激光序列号
} DeviceInfo;

////////////////////////////////////////////////////////////////////////////
//@brief IR的内参信息
typedef struct __Intrinsic__
{
	float pixel_size;	// 像素大小(um)
	uint16_t rows;    	// 原始宽
	uint16_t cols;   	// 原始高

	float fx;			//	等效焦距
	float fy;
	float cx;			//	主点坐标
	float cy;

	float k1;			//	径向畸变
	float k2;
	float k3;
	float k4;
	float k5;
	float k6;

	float p1;			//	切向畸变
	float p2;
	
	uint8_t disto_model;    // 畸变模型
} Intrinsic;

////////////////////////////////////////////////////////////////////////////
//@brief IR的内参信息
typedef struct __Extrinsic__
{
	float rx;
	float ry;
	float rz;

	float tx;
	float ty;
	float tz;

	uint8_t rotation_order;    // 先 y 再 x 后 z
} Extrinsic;

////////////////////////////////////////////////////////////////////////////
//@brief ToF Sensor 的参数信息
typedef struct __SensorConfig__
{
	uint16_t freq_mod;		// 调制频率
	uint16_t integral_time;	// 积分时间
} SensorConfig;

////////////////////////////////////////////////////////////////////////////
//@brief ToF 温度补偿信息
typedef struct __TemperatureCompensation__
{
	float temp_dist_offset;  		// 温度引起的零漂

	uint16_t vcsel_size;		// vcsel温漂系数数量
	uint16_t sensor_size;		// sensor温漂系数数量

	float* vcsel_coeff_ptr; 	// vcsel 温度补偿系，从低阶(1阶)到高阶排序,内存由内部释放
	float* sensor_coeff_ptr;	// ir 温度补偿系数，从低阶（1阶)到高阶排序,内存由内部释放
} TemperatureCompensation;

////////////////////////////////////////////////////////////////////////////
typedef struct __PhaseCompensation__
{
	uint8_t noshufl_num;
	
	float module_offset;  // 模组offset
	float global_offset;  // 整机offset

	uint32_t wiggling_size;	// wiggling系数个数
	uint32_t fppn_size;		// fppn lut 尺寸
	uint8_t* wiggling_ptr; 		// wiggling 系数内容, 内存由内部释放
	uint8_t* fppn_ptr;			// fppn 查找表内容, 内存由内部释放
} PhaseCompensation;

////////////////////////////////////////////////////////////////////////////
typedef struct __TofCalibParams__
{
	HeaderInfo					hdr; 							// Tof param 头文件
	DeviceInfo					dev_info; 						// 器件信息
	Intrinsic					ir_intrinsic; 					// ir的内参信息
	Intrinsic					rgb_intrinsic;
	Extrinsic					ir2rgb;
	SensorConfig				sensor_param[TOF_FREQ_NUM]; 	// sensor的参数
	TemperatureCompensation		temp_comp[TOF_FREQ_NUM]; 		// ToF的温度补偿
	PhaseCompensation			dist_calib[TOF_FREQ_NUM];		// ToF的距离补偿
} ObTofCalibParams;

#pragma pack(pop)
#endif