#ifndef __OB_TOF_FILTER_CONFIG_H__
#define __OB_TOF_FILTER_CONFIG_H__
//#include "tof_config.h"
#include<sys/stat.h>
#include <sys/types.h>
//#include <dirent.h>
//#include<unistd.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifdef WIN32
#ifdef _TOF_LOAD_FILTER_CONFIG_EXPORT
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

enum TOF_DEVICE_TYPE
{
	TOF_33D,
	TOF_SONY,
	TOF_PLECO,
};
typedef struct DEPTH_MODE_CONFIG_
{
	int height;
	int width;
	int rawphase_bit;
	bool qvga;
	int qvga_height;
	int qvga_width;
	bool spot_flag;
	bool with_shuffle;
	int noshuffle_index=0;
	int freq_num;
	float integral_time=0;
	float vcsel_temp=0;
	float sensor_temp=0;
	int pleco_freq_unit;
	TOF_DEVICE_TYPE device_type;
}Depth_Mode_Config;

typedef struct OUTPUT_CONFIG_
{
	bool ir_out;
	bool amplitude_out;
	bool pointcloud_out;
	bool depth_out;
}Output_Config;
TOFFILTER_API int WINAPI LoadToFConfig(char * cfg_fname,char *filter_param, Depth_Mode_Config &depth_config,Output_Config &output_config);
#endif