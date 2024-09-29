/****************************************************************************
*
****************************************************************************/
#ifndef __TOF_CALIB_PARAMS_IO_V2_H__
#define __TOF_CALIB_PARAMS_IO_V2_H__

#include <cstdlib>
#include <string>

/***
* @brief Error Code for Tof Calibration params IO
*/
#ifdef WIN32
#  ifdef OB_TOFPARAMSPARSER_V2_EXPORT
#    define TOFPARAMSPARSER_V2_API __declspec(dllexport)
#  else
#    define TOFPARAMSPARSER_V2_API __declspec(dllimport)
#  endif

#ifndef WIN_PARSER_V2_API
#  define WIN_PARSER_V2_API  _stdcall
#endif

#else
#define TOFPARAMSPARSER_V2_API
#define WIN_API_V2
#endif


	enum TofCalibIOErrorCode_V2
	{
		TOF_CALIB_IO_SUCCESS_V2 = 0,
		TOF_CALIB_IO_FILE_OPEN_FAILED_V2 = 4096,
		TOF_CALIB_PARAMS_CHECKSUM_WRONG_V2 = 4097,
		TOF_CALIB_PARAMS_FILE_WRONG_V2 = 4098,			//	file wrong 
		TOF_CALIB_PARAMS_FILE_SIZE_ERROR_V2 = 4099,	//	size wrong
	};

	enum TofChecksumType_V2
	{
		CRC_32_CHECKSUM_V2 = 0,
		REMAINDER_255_CHECKSUM_V2 = 1,
	};

	TOFPARAMSPARSER_V2_API const char*  ObTofCalibGetVersion();

	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2 
		ObUniformV2CalcBufferSize(std::string param_fname, uint8_t checksum_type, int &buffer_size);

	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  ObUniformV2ReleaseTofParams(void* ob_tof_param);
	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  ObUniformV2CalcCharSize(void* ob_tof_param, int &char_size);
	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  ObUniformV2FillingZerosToStruct(void*  ob_tof_param);
	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  ObUniformV2StructToBuffer(std::string param_fname, signed char* data_buffer);
	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  BufferToStruct(signed char* data_buffer, void* ob_tof_param);


	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  LoadTofCalibParams(std::string param_fname, void* tof_calib_params, uint8_t checksum_type);
	TOFPARAMSPARSER_V2_API TofCalibIOErrorCode_V2  ObKunLunShanReleaseTofParams(void* ob_tof_param);
#endif //__OB_VTofBase_H__
