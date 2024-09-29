#ifndef __OB_TOF_POINTCLOUD_FILTER_H__
#define __OB_TOF_POINTCLOUD_FILTER_H__
#include "ToFCaliParams_V2.h"
#include<stdint.h>
#ifdef WIN32
#ifdef _TOF_POINTCLOUD_FILTER_EXPORT
# define TOFFILTER_API __declspec(dllexport)
#else
# define TOFFILTER_API __declspec(dllimport)
#endif
#ifndef WINAPI
#define WINAPI _stdcall
#endif
#elif defined linux
#define TOFFILTER_API __attribute ((visibility("default")))
#define WINAPI
#elif defined __ANDROID__
#define TOFFILTER_API __attribute ((visibility("default")))
#define WINAPI 
#endif
TOFFILTER_API int WINAPI PointCloudFilterImpl(uint16_t *depth_ptr_, ObToFParam g_tof_cali_params, float statistical_max_slope, float statistical_sigma);
#endif