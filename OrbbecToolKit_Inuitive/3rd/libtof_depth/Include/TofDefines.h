/*****************************************************************************
*
*****************************************************************************/
#ifndef __TOF_DEFINES_H__
#define __TOF_DEFINES_H__

////////////////////////////////////////////////////////////////////////////
///@brief 常规头文件信息
enum TOF_CALIPARAMS_TYPE_E
{
	TOF_CALIPARAMS_LUT = 0,		//查找表
	TOF_CALIPARAMS_COE = 1,		//系数拟合
	TOF_CALIPARAMS_BLOCK = 2,	//分块拟合
};

enum TOF_UNDISTO_FLAG_TYPE_E
{
	TOF_UNDISTO_OFF = 0,		//关闭
	TOF_UNDISTO_ON = 1,			//开启
};

enum TOF_FREQUENCY_TYPE_E
{
	TOF_SINGLE_FREQUENCY_SHUFFLE = 0,		//单频SHUFFLE模式
	TOF_DUAL_FREQUENCY_SHUFFLE = 1,			//双频SHUFFLE模式
	TOF_SINGLE_FREQUENCY_NONSHUFFLE = 2,	//单频NONSHUFFLE
	TOF_DUAL_FREQUENCY_NONSHUFFLE = 3,		//双频NONSHUUFFLE模式
};

////////////////////////////////////////////////////////////////////////////
///@brief 器件信息
#define SERIAL_NO_MAX_LEN   (32)

#define TOF_FREQ_NUM (2)

enum {FREQ_LOW = 0, FREQ_HIGH = 1,};

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

#define FPPN_X_BLOCK_NUM (40)
#define FPPN_Y_BLOCK_NUM (8)
#define FITTING_PARAMS_NUM (3)
#define FPPN_BLOCKS_COE_SIZE (40*8*3)
#define BLOCK_OVERLAP_PIXELS (0)
#define LIGHT_SPEED 299792458
#define WIGGLING_STEPVAL 0.5
#define WIGGLING_THRESHOLD 60
#define FREQ_NUM 2
#define WIGGLING_LUT_ON 0
#define TEMPERDRIFT_ON 1

#define CHUNK_TYPE_LEN (4)

#define WIGGLING_COEFF_NUM (25)

#endif