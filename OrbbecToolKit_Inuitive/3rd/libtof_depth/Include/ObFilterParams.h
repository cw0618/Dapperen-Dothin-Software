/*****************************************************************************
*  Orbbec ToFDepth 2.0
*  Copyright (C) 2019 by ORBBEC Technology., Inc.
*
*  This file is part of Orbbec ToFDepth.
*
*  This file belongs to ORBBEC Technology., Inc.
*  It is considered a trade secret, and is not to be divulged or used by
* parties who have NOT received written authorization from the owner.
*
*  Description:
*
****************************************************************************/

#ifndef __OB_FILTER_PARAMS_H__
#define __OB_FILTER_PARAMS_H__

#include <stdint.h>
#include <string.h>

#define PHASE_SCATTER_FILTER        (1 << 0)       
#define DEPTH_SCATTER_FILTER        (1 << 1)


// 插值方式
#define OB_INTER_NEAREST       0        // nearest neighbor interpolation 
#define OB_INTER_LINEAR        1        // bilinear interpolation 

#pragma pack(push, 1)
///////////////////////////////////////////////////////////////////////////////////////////////////
//	scattering filter
typedef struct _SCATTER_FILTER_PARAM
{
    float	filter_ratio;// = 0.7;				//	filter ratio
    int     foreground_thresh;// = 90;     // foreground pixels ir thresh
    int     psf_cr;// = 12;
    float   psf_scale;
    int     filter_type;// = DEPTH_SCATTER_FILTER;
    int		bypass;// = 0;						//	skip or not
}ScatterFilterParam;

//	spatial filter
typedef struct _RawphaseFilterParam
{
	int		win_size;					//	window size
	int		speed_up;
	
	int		bypass;						//	skip or not
}RawphaseFilterParam;

//	temporal filter
typedef struct _TEMPORAL_FILTER_PARAM
{
	float	maxdiff;					//	diffstep
	float	weight;						//	current frame weight
	int		median_filter_framenum;		//	frame number for temporal median filter
	int		median_filter_bypass;		//	skip or not median filter
	
	int		bypass;						//	skip or not temporal filter
}TemporalFilterParam;

//	noise filter
typedef struct _NOISE_FILTER_PARAM
{
    int		win_size;					//	window size
    
    int		bypass;						//	skip or not
}NoiseFilterParam;

//	bilateral filter
typedef struct _FASTGUIDED_FILTER_PARAM
{
    int		win_size;					//	filter distance
    float	eps;						//	color sigma
    float	scale;						//	space sigma
    
    int		bypass;						//	skip or not
}FastGuidedFilterParam;

//	confidence filter
typedef struct _CONFIDENCE_FILTER_PARAM
{
    float	confidence_indoor_amp_thres;
    float	confidence_outdoor_amp_thres;
    float	sbr_thres;
    float	intensity_thres;
    float	usm_thres;
    float	reflectivity_thres;
    int		usm_win_size;
	int		usm_depth_thres;
	int		usm_with_depth; 
    int		usm_bypass;
    
	int		bypass;
}ConfidenceFilterParam;

//	post filter
typedef struct _POST_FILTER_PARAM
{
    float	outdoor_th;					//	outdoor th
    float	maxSpeckleSize;				//	max peckle Size
    float	maxdiff;					//	max difference
    float	outdoor_long_dis;			//	outdoor long distance threshold
    int		win_size;					//	window size
    
    int		bypass;						//	skip or not
}PostFilterParam;

//	depth configuration
typedef struct _DEPTH_CONFIGURATION_PARAM
{
    float	depth_uint;
    int		max_depth;
    int		min_depth;
}DepthConfigParam;

//	3d point cloud filter configuration
typedef struct _POINT_CLOUD_FILTER_PARAM
{
    float	statistical_max_slope;
    float	statistical_sigma;

    float	cluster_ratio;
    
    int		filter_type;				// 1 - statistical; 8 - cluster filter;
    int		flypoint_compens;

    int		bypass;
}	PointCloudFilterParam;

typedef struct _SpotFilterParam
{
	int		rawphase_mean_filter_win_size;
	int		pointcloud_statistic_filter_winsize;
	float	pointcloud_statistic_distance_thresh;
	int		pointcloud_statistic_sigma;
	int		spot_radius;
	int		spot_nms_win_size;
	float	mean_scale;	
	int		rawphase_mean_filter_bypass;
	int		pointcloud_statistic_filter_bypass;
}SpotFilterParam;

typedef struct _MotionDeblurParam
{
	int		neighbor_similar_num_search;
	float	ir_similar_ratio_neighbor;
	int		neighbor_max_win_size;
	int		dist_edge_winsize;
	float	dist_edge_ratio_thresh;
	float	ir_diff_ratio_dual_freq;
	float	ir_diff_ratio_single_freq;
	float	ir_similar_ratio_single_freq;
	
	int		bypass;
}MotionDeblurParam;

typedef struct
{
    float   err_ratio;// = 0.03;       // dist error ratio
    int     err_min;// = 20;           // dual freq dist min error
    int     err_max;// = 60;           // dual freq dist max error
    int     type;
    int     bypass;// = 0;             // skip or not
} ErrorFilterParam;

///////////////////////////////////////////////////////////////////////////////////////////////////
typedef union _OB_TOF_FILTER_CONFIG
{
	struct
	{
		ScatterFilterParam		scatter_filter_cfg;
		RawphaseFilterParam		rawphase_filter_cfg;
		TemporalFilterParam		temporal_filter_cfg;
		NoiseFilterParam		noise_filter_cfg;
		FastGuidedFilterParam	fastguided_filter_cfg;
		ConfidenceFilterParam	confidence_filter_cfg;
		PointCloudFilterParam	point_cloud_filter_cfg;
		MotionDeblurParam		motion_deblur_filter_cfg;
		PostFilterParam			post_filter_cfg;
		SpotFilterParam			spot_filter_param;
        ErrorFilterParam        error_filter_cfg;
		DepthConfigParam		depth_cfg;
	} filter_params;

	char filter_params_ptr[512];
}ObToFFilterConfig;

///////////////////////////////////////////////////////////////////////////////////////////////////
enum TOF_DEVICE_TYPE { TOF_33D, TOF_SONY, TOF_PLECO, TOF_MLX};


/**
* @brief	ExposureTime 计算信息
*/
typedef struct {
    int remain_cols = 300;              // the cols num of interest in ir
    int remain_rows = 200;              // the rows num of interest in ir
    int t_min = 20;                     // the minimum exposure time
    int t_max = 1000;                   // the maximum exposure time
    int over_exposure_value = 1020;     // a pixel is considered to be over-exposed if greater than over_exposure_value,1015 for 10-bit rawphase and 2030 for 11-bit rawphase
    float ratio_thresh = 0.005;         // if the ratio of over-exposed pixels is more than ratio_thresh,you should reduce the exposure time
    int adc_offset = 26;                // a fixed offset of ADC,it is independent of integration time
} ExposureCfg;


/**
* @brief    裁剪ROI区域配置
*/
typedef struct {
    int flag = 0;                       // 1 - enable crop, 0 - disable crop
    int roi_x = 0;
    int roi_y = 0;
    int roi_width = 0;
    int roi_height = 0;
} CropCfg;


typedef struct _DepthModeConfig
{
	int height;
	int width;
	//add ground_correction flag and parameter
	float ground_height;
	int ground_correction_flag;
	int rawphase_bit;
	bool qvga;
	int qvga_height;
	int qvga_width;
	bool spot_flag;
	bool with_shuffle;
	int noshuffle_index = 0;
	int freq_num;
	float integral_time = 0;
	float vcsel_temp = 0;
	float sensor_temp = 0;
	int pleco_freq_unit;
	bool is_HDR = true;
    int inter_flag = OB_INTER_NEAREST;
	TOF_DEVICE_TYPE device_type;
    CropCfg         crop_cfg;
    ExposureCfg     exposure_cfg;
}DepthModeConfig;

typedef struct _OutputConfig
{
	uint8_t ir_out;
	uint8_t amplitude_out;
	uint8_t pointcloud_out;
	uint8_t depth_out;
}OutputConfig;
#pragma pack(pop)


#ifdef WIN32
	#define TOFFILTER_API __declspec(dllexport)
	#ifndef WINAPI
		#define WINAPI _stdcall
	#endif
#else
	#define TOFFILTER_API __attribute((visibility("default")))
	#define WINAPI
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	TOFFILTER_API const char* WINAPI GetConfigureLoaderVersion();

	TOFFILTER_API int WINAPI GetFilterConfigParamVersion(const char * cfg_fname, char *version);

	TOFFILTER_API void WINAPI ExportFilterParams(const char* filename, ObToFFilterConfig& filter_parmas, DepthModeConfig& depth_mode);

	TOFFILTER_API int WINAPI LoadToFilterConfig(char *cfg_fname, ObToFFilterConfig& filter_params, DepthModeConfig &depth_config, OutputConfig &output_config);

    /**
    * @brief        配置文件 ini 转 bin 文件
    * @param[in]    ini_file - ini 文件路径
    * @param[in]    bin_file - bin 文件路径（默认为空，在 ini 文件路径下生成同名的 bin 文件）
    * @return       0 - 成功， 其他 - 失败
    */
    TOFFILTER_API int WINAPI FilterConfigIni2Bin(const char* ini_file, const char* bin_file = NULL);

#ifdef __cplusplus
}
#endif



#endif //__OB_TOF_CONFIG_H__