/****************************************************************************
*
****************************************************************************/
#ifndef __TOF_CALIB_PARAMS_IO_V2_H__
#define __TOF_CALIB_PARAMS_IO_V2_H__

#include <cstdlib>
#include <string>

#include "TofCalibParams.h"

#include "TofCalibParamsNew.h"

#ifdef WIN32
# define CALIBIN_API __declspec(dllexport)
#ifndef WINAPI
#define WINAPI _stdcall
#endif
#else
#define CALIBIN_API __attribute ((visibility("default")))
#define WINAPI
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	// @brief Error Code for Tof Calibration params IO
	namespace TofCalib
	{
		enum ErrorCode
		{
			TOF_CALIB_IO_SUCCESS = 0,
			TOF_CALIB_IO_CHECK_WRONG = -1,
			TOF_CALIB_IO_FILE_OPEN_FAILED = 4096,
			TOF_CALIB_PARAMS_CHECKSUM_WRONG = 4097,
			TOF_CALIB_PARAMS_FILE_WRONG = 4098,		//	file wrong
			TOF_CALIB_PARAMS_FILE_SIZE_ERROR = 4099, //	size wrong
		};

		enum TofChecksumType
		{
			CRC_32_CHECKSUM = 0,
			REMAINDER_255_CHECKSUM = 1,
		};

		CALIBIN_API const char* WINAPI GetVersion();

		CALIBIN_API ErrorCode WINAPI ObCheckTofParams(void *ob_tof_param, int &buffer_size, char *dev_sn);

		CALIBIN_API ErrorCode WINAPI LoadObTofParams(const char *filename, StarTofCalibParams* all_cali_data, uint8_t checksum_type);

		CALIBIN_API int WINAPI ExportTofCalibParams(std::string filename, ObTofCalibParams calib_data);

		CALIBIN_API int WINAPI LoadTofCalibParams(std::string filename, ObTofCalibParams& calib_data);

        CALIBIN_API int WINAPI InitTofCalibParams(const char* file_data, int file_size, ObTofCalibParams& calib_data);

		CALIBIN_API int WINAPI ReleaseTofCalibParams(ObTofCalibParams& calib_data);
	}
#ifdef __cplusplus
}
#endif

#endif //__OB_VTofBase_H__
