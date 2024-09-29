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

#ifndef __OB_TOF_CONFIG_H__
#define __OB_TOF_CONFIG_H__
#define USEOMP
//#define SHOWTIME
typedef enum _TOF_FILTER_TYPE
{
	SCATTER_FILTER_TYPE = 1,
	RAWPHASE_FILTER_TYPE = 2,
	TEMPORAL_FILTER_TYPE = 4,
	NOISE_FILTER_TYPE = 8,
	FASTGUIDED__FILTER_TYPE = 16,
	POINTCLOUD_FILTER_TYPE = 32,
	CONFIDENCE_FILTER_TYPE = 64,
	POST_FILTER_TYPE = 128
}TofFilterType;

/***
* @brief median filter
*/
typedef struct _MEDIAN_FILTER_PARAM
{
    void operator = (const _MEDIAN_FILTER_PARAM& other)
    {
        this->win_size = other.win_size;
        this->bypass = other.bypass;
    }
    // window size
    int win_size;
    // skip or not
    int bypass;
}MedianFilterParam;

/***
* scattering filter
*/
typedef struct _SCATTER_FILTER_PARAM
{

	void operator = (const _SCATTER_FILTER_PARAM& other)
	{
		this->filter_ratio = other.filter_ratio;
		this->bypass = other.bypass;
	}

	//filter ratio
	float filter_ratio;
	// skip or not
	int bypass;

}ScatterFilterParam;

/***
* spatial filter
*/
typedef struct _RawphaseFilterParam
{

	void operator = (const _RawphaseFilterParam& other)
	{
		this->win_size = other.win_size;
		this->speed_up = other.speed_up;
		this->bypass = other.bypass;
	}

	//window size
	int  win_size;
	int speed_up;
	// skip or not
	int bypass;

}RawphaseFilterParam;

typedef struct _SpotFilterParam
{

	void operator = (const _SpotFilterParam& other)
	{
		//this->win_size = other.win_size;
		this->rawphase_mean_filter_win_size = other.rawphase_mean_filter_win_size;
		this->pointcloud_statistic_filter_winsize = other.pointcloud_statistic_filter_winsize;
		this->spot_radius = other.spot_radius;
		this->spot_nms_win_size = other.spot_nms_win_size;
		this->mean_scale = other.mean_scale;
		this->pointcloud_statistic_distance_thresh = other.pointcloud_statistic_distance_thresh;
		this->pointcloud_statistic_sigma = other.pointcloud_statistic_sigma;
		this->rawphase_mean_filter_bypass = other.rawphase_mean_filter_bypass;
		this->pointcloud_statistic_filter_bypass = other.pointcloud_statistic_filter_bypass;
		//this->bypass = other.bypass;
	}
	int rawphase_mean_filter_win_size;
	int pointcloud_statistic_filter_winsize;
	float pointcloud_statistic_distance_thresh;  //点云统计滤波的邻域点数
	int pointcloud_statistic_sigma;  //点云统计滤波中的标准偏差系数
	int spot_radius;
	int spot_nms_win_size;
	float mean_scale;
	int rawphase_mean_filter_bypass;
	int pointcloud_statistic_filter_bypass;
}SpotFilterParam;

/***
* temporal filter
*/
typedef struct _TEMPORAL_FILTER_PARAM
{

	void operator = (const _TEMPORAL_FILTER_PARAM& other)
	{
		this->maxdiff = other.maxdiff;
		this->weight = other.weight;
		this->median_filter_framenum = other.median_filter_framenum;
		this->median_filter_bypass = other.median_filter_bypass;
		this->bypass = other.bypass;
	}

	//diffstep
	float  maxdiff;
	//current frame weight
	float weight;
	//frame number for temporal median filter
	int median_filter_framenum;
	//skip or not median filter
	int median_filter_bypass;
	//skip or not temporal filter
	int bypass;

}TemporalFilterParam;

/***
* @brief noise filter
*/
typedef struct _NOISE_FILTER_PARAM
{
    void operator = (const _NOISE_FILTER_PARAM& other)
    {
        this->win_size = other.win_size;
        this->bypass = other.bypass;
    }
    // window size
    int win_size;
    // skip or not
    int bypass;
}NoiseFilterParam;

/***
* @brief bilateral filter
*/
typedef struct _FASTGUIDED_FILTER_PARAM
{
    void operator = (const _FASTGUIDED_FILTER_PARAM& other)
    {
        this->win_size = other.win_size;
        this->eps = other.eps;
        this->scale = other.scale;
        this->bypass = other.bypass;
    }
    // filter distance
    int win_size;
    // color sigma
    float eps;
    // space sigma
    float scale;
    // skip or not
    int bypass;
}FastGuidedFilterParam;

/***
* @brief gaussian filter
*/
typedef struct _GAUSSIAN_FILTER_PARAM
{
    void operator = (const _GAUSSIAN_FILTER_PARAM& other)
    {
        //GAUSSIAN_FILTER_PARAM third;
        this->win_x = other.win_x;
        this->win_y = other.win_y;
        this->sigma_x = other.sigma_x;
        this->sigma_y = other.sigma_y;
        this->bypass = other.bypass;
    }
    // filter size
    int win_x;
    int win_y;
    // x sigma
    float sigma_x;
    // y sigma
    float sigma_y;
    // skip or not
    int bypass;
}GaussianFilterParam;

/***
* flypoint filter
*/
typedef struct _FLYPOINT_FILTER_PARAM
{

    void operator = (const _FLYPOINT_FILTER_PARAM& other)
    {
        this->thres = other.thres;
        this->noise_coeff = other.noise_coeff;
        this->fill_hole = other.fill_hole;
        this->bypass = other.bypass;
    }
    //thres for fly point filtering
    float thres;
    //noise coeff;
    float noise_coeff;
    //fill the hole after fly point filtering
    int fill_hole;
    //	skip or not
    int bypass;
}FlyPointFilterParam;

/***
* confidence filter
*/
typedef struct _CONFIDENCE_FILTER_PARAM
{

    void operator = (const _CONFIDENCE_FILTER_PARAM& other)
    {
        this->confidence_indoor_amp_thres = other.confidence_indoor_amp_thres;
        this->confidence_outdoor_amp_thres = other.confidence_outdoor_amp_thres;
        this->sbr_thres = other.sbr_thres;
        this->intensity_thres = other.intensity_thres;
        this->usm_thres = other.usm_thres;
        this->reflectivity_thres = other.reflectivity_thres;
        this->usm_win_size = other.usm_win_size;
		this->usm_depth_thres = other.usm_depth_thres;
		this->usm_with_depth = other.usm_with_depth;
        this->usm_bypass = other.usm_bypass;
        this->bypass = other.bypass;
    }
    float confidence_indoor_amp_thres;
    float confidence_outdoor_amp_thres;
    float sbr_thres;
    float intensity_thres;
    float usm_thres;
    float reflectivity_thres;
    int usm_win_size;
	int usm_depth_thres;
	int usm_with_depth; 
    int usm_bypass;
    int bypass;
}ConfidenceFilterParam;


/***
* @brief post filter
*/
typedef struct _POST_FILTER_PARAM
{
    void operator = (const _POST_FILTER_PARAM& other)
    {
        this->outdoor_th = other.outdoor_th;
        this->win_size = other.win_size;
        this->maxSpeckleSize = other.maxSpeckleSize;
        this->maxdiff = other.maxdiff;
        this->outdoor_long_dis = other.outdoor_long_dis;
        this->bypass = other.bypass;
    }
    //outdoor th
    float outdoor_th;

    //max peckle Size
    float maxSpeckleSize;

    //max difference
    float maxdiff;

    //outdoor long distance threshold
    float outdoor_long_dis;

    // window size
    int win_size;
    // skip or not
    int bypass;
}PostFilterParam;


/***
* depth configuration
*/
typedef struct _DEPTH_CONFIGURATION_PARAM
{

    void operator = (const _DEPTH_CONFIGURATION_PARAM& other)
    {
        this->depth_uint = other.depth_uint;
        this->max_depth = other.max_depth;
        this->min_depth = other.min_depth;
    }
    float depth_uint;
    int max_depth;
    int min_depth;
}DepthConfigParam;

/**
*	3d point cloud filter configuration
*/
typedef struct _POINT_CLOUD_FILTER_PARAM
{
    void operator = (const _POINT_CLOUD_FILTER_PARAM& other)
    {
        this->statistical_max_slope = other.statistical_max_slope;
        this->statistical_sigma = other.statistical_sigma;

        this->cluster_ratio = other.cluster_ratio;

        this->filter_type = other.filter_type;

        this->flypoint_compens = other.flypoint_compens;

        this->bypass = other.bypass;
    }

    float statistical_max_slope;
    float statistical_sigma;

    float cluster_ratio;
    /// 1 - statistical; 8 - cluster filter;
    int filter_type;
    int flypoint_compens;

    int bypass;

}	PointCloudFilterParam;


typedef struct _OB_TOF_FILTER_CONFIG
{
	ScatterFilterParam scatter_filter_cfg;
    RawphaseFilterParam rawphase_filter_cfg;
	
    TemporalFilterParam temporal_filter_cfg;
    NoiseFilterParam noise_filter_cfg;
    MedianFilterParam median_filter_cfg;
    FastGuidedFilterParam fastguided_filter_cfg;
    GaussianFilterParam gaussian_filter_cfg;
    FlyPointFilterParam flypoint_filter_cfg;
    PostFilterParam post_filter_cfg;
	SpotFilterParam spot_filter_param;
    ConfidenceFilterParam confidence_filter_cfg;
    DepthConfigParam depth_cfg;

    PointCloudFilterParam point_cloud_filter_cfg;
}ObToFFilterConfig;



#endif //__OB_TOF_CONFIG_H__
