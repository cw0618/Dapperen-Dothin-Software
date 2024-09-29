/*****************************************************************************
*  Orbbec ToFDepth 1.0
*  Copyright (C) 2019 by ORBBEC Technology., Inc.
*
*  This file is part of Orbbec ToFDepth.
*
*  This file belongs to ORBBEC Technology., Inc.
*  It is considered a trade secret, and is not to be divulged or used by
*  parties who have NOT received written authorization from the owner.
*
*  Description:
*
****************************************************************************/
#ifndef __OB_TOFPROC_H__
#define __OB_TOFPROC_H__

#include <stdint.h>

#if defined (_WIN32) || defined(_WIN64)
     # define TOF_API __declspec(dllexport)
 #else
     #define TOF_API 
 #endif

#ifdef __cplusplus
extern "C"
{
#endif

	//! HANDLE type
	typedef void*	HANDLE;

	/**
	* @brief	device info
	*/
	typedef struct {
		float vcsel_temp;		//! vcsel 温度
		float sensor_temp;		//! sensor 温度
		float integral_time;	//! 积分时间
		int   noshuffle_index;  //! noshuffle模式下指示当前rawphase是第几张（0/1/2）
	} ToFDevInfo;

	/**
	* @brief	raw 图信息
	*/
	typedef struct {
		int width;				//! rawphase width
		int height;				//! rawphase height
		int scale;		        //! 相对于深度图宽的缩放系数，rawphse_width = depth_width * scale 
		int frame_num;			//! 计算深度的 rawphase 帧数
        bool is_HDR;            //! 是否为 HDR 模式
		bool is_shuffle;
		int  freq_num;
	} ToFRawInfo;

	/**
	* @brief	ExposureTime 计算信息
	*/
	typedef struct {
		float fov;                  //! 计算范围
		int t_min;                  //! 积分时间的最小值
		int t_max;                  //! 积分时间的最大值
		float t_step_ratio_min;     //! 积分时间的调整比例（最低比例）
		float t_step_ratio_max;     //! 积分时间的调整比例（最高比例）
		int over_exposure_value;    //! 判断过曝的标准，rawphase中大于该值的像素数占比若大于ratio_thresh，认为是过曝
		int over_dark_value;        //! 判断过暗的标准，rawphase中大于该值的像素值占比若小于ratio_thresh，认为是过暗
		float ratio_thresh;         //! 过曝和过暗的比例阈值，配合over_exposure_value和over_dark_value值使用
		float ir_thresh;            //! IR亮度均值阈值，若IR图fov范围内的亮度均值小于该阈值，则认为IR图过暗，需增加积分时间
	} ToFExposureCfg;

    /**
	* @brief Error Type
	*/
    typedef enum TOF_DEPTH_ERR
    {
        TOF_DEPTH_CALC_SUCCESS				= 0,
        FILTER_CONFIG_LOAD_ERROR			= -1,
        LOAD_CALIB_BIN_ERROR				= -2,
        RAWPHASE_BUFFER_SIZE_ERROR			= -3,
        BUFFER_IS_NULL						= -4,
        FREQUENCY_NUM_ERROR					= -5,
        TOF_DEPTH_LOAD_PNG_ERROR			= -6,
        TOF_DEPTH_DATA_TYPE_ERROR			= -7,
        TOF_OUT_CONFIG_ERROR				= -8,
        TOF_DEPTH_RESOLUTION_ERROR			= -9,
        QVGA_RESOLUTION_ERROR				= -10,
        SPOT_FLAG_ERROR						= -11,
        SPOT_STATISTIC_FILTER_WINSIZE_ERROR = -12,

        TOF_DEPTH_INVALID_CALIB_PARAM		= 4095,
        //TOF_CALIB_IO_FILE_OPEN_FAILED		= 4096,
        //TOF_CALIB_PARAMS_CHECKSUM_WRONG	= 4097,

        TOF_DEPTH_WITHOUT_FILTER_CONFIG		= 4100,
        TOF_DEPTH_LOAD_DATA_ERROR			= 4101,
        TOF_DEPTH_DATA_NUM_NOT_MATCHING		= 4102,
        TOF_DEPTH_INVALID_RESULT			= 4103,
        TOF_DEPTH_LOAD_HIGH_FREQ_FIRST		= 4104,
        TOF_DEPTH_LOAD_TILT_FILE_ERROR		= 4105,
        TOF_DEPTH_FREQNUM_SHUFL_ERROR		= 4000,
    } ToFDepthErr;


	 enum TxModeType
	{
		FLOOD_TX = 0, //面光源
		LINE_TOP_TX = 1, //线光源，打在物理世界上部空间的，举例五线
		LINE_BOTTOM_TX = 2,//线光源，打在物理世界下部空间(地面)的，举例三线
	} ;

    /**
	* @brief	获取版本信息
	*/
	TOF_API const char* ToFDepthVersion();


	/**
	* @brief		加载滤波参数
	* @param[in]	file_path - 滤波配置文件路径
	* @return		成功 - 参数句柄，失败 - NULL
	*/
	TOF_API HANDLE LoadToFFilterParam(const char *file_path);

	/**
	* @brief		加载标定参数
	* @param[in]	file_path - 标定文件路径
	* @param[in]	hConfig - 滤波配置参数，用于校验标定参数的有效性
	* @return		成功 - 参数句柄，失败 - NULL
	*/
	TOF_API HANDLE LoadToFCaliParam(const char *file_path, const HANDLE hConfig);

	/**
	* @brief		校验eeprom中深度标定参数
	* @param[in]	buffer - eeprom中读取标定文件数据 buffer
	* @param[in]	buffer_size - 数据长度
	* @param[out]	模组设备序列号
	* @return		成功 - 0，失败 - -1,数据内容错误
	*/
	TOF_API int CheckDepthCalibEEPROM(void* buffer, int buffer_size, char* dev_sn);

	/**
	* @brief			释放句柄
	* @param[in,out]	handle - 句柄
	* @return			0
	*/
	TOF_API int ReleaseHandle(HANDLE handle);

	/**
	* @brief		重置 rawphase 历史记录
	* @param[in]	hCalib - 标定参数
	* @return		0
	*/
	TOF_API int ResetPreRawphase(const HANDLE hCalib);

	/**
	* @brief		获取 raw 图信息
	* @param[in]	hConfig - 配置参数
	* @return		raw 图信息
	*/
	TOF_API ToFRawInfo GetToFRawInfo(const HANDLE hConfig);

	/**
	* @brief		获取 ToF 内存信息
	* @param[in]	hCali - 标定参数
	* @param[out]	intrin[5] - intrinsic matrix (fx,fy,cx,cy,s)
	* @param[out]	distort_coeff[8]   兼容目前的各种畸变模型，最多8个畸变参数- distort_coeff matrix (k1, k2, t1, t2,k3)
	* @return		0 - 成功，其他 - 失败
	*/
	TOF_API int GetToFIntrinsic(const HANDLE hCali, float intrin[5], float distort_coeff[8]);

	/** 
	* @brief		面光源输出、线光源上下曝光分别输出深度计算算法接口，计算深度图、IR 图、幅值图
	* @param[in]	rawphase - rawphase 数据
	* @param[in]	dInfo - 设备信息
	* @param[in]	hConfig - 配置参数
	* @param[in]	hCali - 标定参数
	* @param[out]	depth - 深度图（由用户分配空间）
	* @param[out]	intensity - IR 图（由用户分配空间）
	* @param[out]	amplitude - 幅值图（由用户分配空间）
	* @param[out]	pointcloud - 输出点云，数据按 X Y Z X Y Z ... 排列
	* @return		0 - 成功，其他 - 失败
	*/
	TOF_API int DepthCalc(const uint16_t *rawphase, const ToFDevInfo *dInfo, const HANDLE hConfig, const HANDLE hCali,
		float *depth, float *intensity, float *amplitude, float *pointcloud);



	/** 
	* @brief		线光源上下曝光融合计算深度计算算法接口，计算深度图
	* @param[in]	rawphase - rawphase 数据，顺序是线光源上曝光rawphae,然后线光源下曝光rawphase
	* @param[in]	dInfo_top - 线光源上曝光设备信息
	* @param[in]	hConfig_top- 线光源上曝光配置参数
	* @param[in]	hCali_top - 线光源上曝光标定参数
	* @param[in]	dInfo_down - 线光源下曝光设备信息
	* @param[in]	hConfig_down- 线光源下曝光配置参数
	* @param[in]	hCali_down - 线光源下曝光标定参数
	* @param[out]	depth - 深度图（由用户分配空间）
	* @param[out]	pointcloud - 输出点云，数据按 X Y Z X Y Z ... 排列
	* @return		0 - 成功，其他 - 失败
	*/
	TOF_API int DepthCalc_LineFusion(const uint16_t *rawphase, const ToFDevInfo *dInfo_top, const HANDLE hConfig_top, const HANDLE hCali_top, const ToFDevInfo *dInfo_down, const HANDLE hConfig_down, const HANDLE hCali_down,
		float *depth, float *pointcloud);

	/**
	* @brief		保存输出结果
	* @param[in]	dir - 保存文件目录（算法会在此目录下生成 OUT_FILES 目录）
	* @param[in]	fname - 保存文件名（算法会根据保存数据类型添加相应的后缀）
	* @param[in]	hConfig - 配置参数
	* @param[in]	hCalib - 标定参数
	* @param[in]	count - 每个文件夹下排序第几帧
	* @param[in]	depth - 深度图
	* @param[in]	intensity - IR 图
	* @param[in]	amplitude - 幅值图
	* @param[in]	pointcloud - 点云数据
	* @return		无
	*/
	TOF_API void StoreOutput(const char* dir, const char* fname, const HANDLE hConfig, const HANDLE hCali,int count,
		const float *depth, const float *intensity, const float *amplitude, const float *pointcloud);

    /**
    * @brief	    输入当前积分时间及对应的rawphase,根据rawphase的亮度情况自适应调整积分时间
    * @param[in]    input_t - 当前积分时间
    * @param[in]    rawphase - 当前积分时间下采集的rawphase
    * @param[in]    raw_info - rawphase 信息
    * @param[in]    exposure_cfg - Exposure 配置信息
    * @return		积分时间
    */
    TOF_API int ExposureTimeCalc(int input_t, const uint16_t *rawphase, ToFRawInfo rawInfo, ToFExposureCfg exposureCfg);

	/**
	* @brief	    计算积分时间
	* @param[in]    input_t - 当前积分时间
	* @param[in]    rawphase - 当前一组 rawphase 数据
	* @param[in]    hConfig - 配置参数
	* @param[in]	hCalib - 标定参数
	* @return		积分时间
	*/
	TOF_API int ExposureTimeCalcWithConfig(int input_t, const uint16_t *rawphase, const HANDLE hConfig, const HANDLE hCali, uint8_t tx_mode_type);

#ifdef __cplusplus
}
#endif

#endif //__OB_TOFPROC_H__