/*****************************************************************************
*  Orbbec ToFDepth 1.0
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
#ifndef __OB_TOFPROC_H__
#define __OB_TOFPROC_H__
//#include "tof_depth_and_output_config.h"
#include "load_tof_filter_config.h"
#include <stdint.h>

#ifdef WIN32
    #ifdef TOFDEPTH_EXPORT
        # define TOFDEPTH_API __declspec(dllexport)
    #else
        # define TOFDEPTH_API __declspec(dllimport)
    #endif
    #ifndef WINAPI
        #define WINAPI _stdcall
    #endif
#elif defined linux
    #define TOFDEPTH_API __attribute ((visibility("default")))
    #define WINAPI
#elif defined __ANDROID__
    #define TOFDEPTH_API __attribute ((visibility("default")))
    #define WINAPI 
#endif

#ifdef __cplusplus
extern "C" {
#endif
	/***
	* @brief Error Type
	*/
	typedef enum TOF_DEPTH_ERR
	{
		TOF_DEPTH_CALC_SUCCESS = 0,
		FILTER_CONFIG_LOAD_ERROR = -1,
		LOAD_CALIB_BIN_ERROR = -2,
		RAWPHASE_BUFFER_SIZE_ERROR = -3,
		BUFFER_IS_NULL = -4,
		FREQUENCY_NUM_ERROR = -5,
		TOF_DEPTH_LOAD_PNG_ERROR = -6,
		TOF_DEPTH_DATA_TYPE_ERROR = -7,
		TOF_OUT_CONFIG_ERROR = -8,
		TOF_DEPTH_RESOLUTION_ERROR = -9,
		QVGA_RESOLUTION_ERROR = -10,
		SPOT_FLAG_ERROR=-11,
		SPOT_STATISTIC_FILTER_WINSIZE_ERROR = -12,

		TOF_DEPTH_INVALID_CALIB_PARAM = 4095,
		//TOF_CALIB_IO_FILE_OPEN_FAILED = 4096,
		//TOF_CALIB_PARAMS_CHECKSUM_WRONG = 4097,

		TOF_DEPTH_WITHOUT_FILTER_CONFIG = 4100,
		TOF_DEPTH_LOAD_DATA_ERROR = 4101,
		TOF_DEPTH_DATA_NUM_NOT_MATCHING = 4102,
		TOF_DEPTH_INVALID_RESULT = 4103,
		TOF_DEPTH_LOAD_HIGH_FREQ_FIRST = 4104,
		TOF_DEPTH_LOAD_TILT_FILE_ERROR = 4105,
		TOF_DEPTH_FREQNUM_SHUFL_ERROR = 4000,
	}ToFDepthErr;

    /***
    * @brief get library version
    * @return
    */
    TOFDEPTH_API const char * WINAPI
        ToFDepthVersion();

    /***
    * @brief load ToF configuration
    * @param cfg_fname configuration file name
    * @param depth_config the depth mode config
    * @param output_config output config
    * @return
    */
    TOFDEPTH_API int WINAPI
		LoadToFFilterParam(char * cfg_fname, char *filter_params,Depth_Mode_Config &depth_config, Output_Config &output_config);

    /***
    * @brief load ToF params
    * @param param_fname parameter file name
    * @return
    */
    TOFDEPTH_API int WINAPI
        LoadToFCaliParam(char * param_fname, signed char* &caliparam_buffer,Depth_Mode_Config depth_mode);

    /***
    * @brief get ToF intrinsic parameters
    * @param intrin[5] intrinsic matrix (fx,fy,cx,cy,s)
    * @param distort_coeff[5] distort_coeff matrix (k1, k2, t1, t2,k3)
    * @return
    */
	TOFDEPTH_API int WINAPI
		GetToFIntrinsic(signed char* caliparam_buffer, float intrin[5], float distort_coeff[5]);


	TOFDEPTH_API  int WINAPI 
		ResetPreRawphase(TOF_DEVICE_TYPE type);


    /***
    * @brief calculate depth
    * @param rawphase rawphase buffer
    * @param rawphase_size rawphase size
    * @param temp_buffer temporary buffer
    * @param depth depth buffer
    * @param intensity intensity buffer
    * @param amplitude amplitude buffer

    * @param depth_mode_config the depth mode information
    * @return
    */
    TOFDEPTH_API int WINAPI
        DepthCalc(uint16_t *rawphase, int rawphase_size, signed char* caliparam_buffer, char *filter_param_buffer,float*temp_buffer, float *depth,
            float *intensity, float *amplitude, Depth_Mode_Config depth_mode_config, Output_Config output_config);
	
	TOFDEPTH_API int WINAPI
		ReleaseCaliParamsBuffer(signed char* &caliparam_buffer);


#ifdef __cplusplus
}
#endif


#endif //__OB_TOFPROC_H__

