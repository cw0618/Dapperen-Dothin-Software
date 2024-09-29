#ifndef __DEPTH_TYPE_H_
#define __DEPTH_TYPE_H_
#include <stdint.h>

#ifdef _MSC_VER

#include <windows.h>
   #ifndef PACK
   #define PACK( __Declaration__ ) __Declaration__
#endif

#ifndef OBC_API_EXPORT
   #define OBC_API_EXPORT __declspec(dllexport) 
#endif

#else
  #define PACK( __Declaration__ ) __Declaration__ __attribute__ ((packed))

#ifndef OBC_API_EXPORT
   #define OBC_API_EXPORT  
#endif

#endif

#define MIN(x, y) ((x > y) ? (y):(x))
#define MAX(x, y) ((x > y) ? (x):(y))

#ifdef __cplusplus
extern "C"{
#endif

#if 0

	typedef enum DataFmt{
		OB_OUT_DEPTH = 1,
		OB_OUT_SHIFT,
	};
#endif

	//定义MONO  输出格式，OB_FORMAT_DEPTH:深度，OB_FOMAT_DISPARITY: 视差
	typedef enum  {
		OB_FORMAT_DEPTH = 1,
		OB_FOMAT_DISPARITY,
	}DataFmt;


#define  DIR_PATH_LEN_MAX  256

	typedef enum obdepth_stream_type {
		OBDEPTH_STREAM_IR_FLOOD = 1,      // IR enable flood
		OBDEPTH_STREAM_IR_LASER,      // IR enable laser
		OBDEPTH_STREAM_IR_EXT,        // 
		OBDEPTH_STREAM_DEPTH,         //
		OBDEPTH_STREAM_INVALID = -1,  //
	} obdepth_stream_type_t;

	typedef enum mipi_transfer_datatype {
		TRANSFER_DATA_TYPE_10BIT_AS_RAW10 = 10,   //!< raw10 数据 mipi 按照 raw10传输
		TRANSFER_DATA_TYPE_12BIT_AS_RAW10 = 100,  //!< raw12 数据 mipi 按照 raw10传输
		TRANSFER_DATA_TYPE_12BIT_AS_RAW12 = 12    //!< raw12 数据 mipi 按照 raw12 传输
	}obc_transfer_type_t;

typedef struct s2dconfig
{
	uint32_t nconstshift;
	//多项式参数
	uint32_t nparamcoeff;
	uint32_t nshiftscale;
	uint32_t npixelsizefactor;

    //最小深度限制  单位  0.1mm  不受 nPreciseUnit影响
	uint32_t ndepth_mincutoff;
    //最大深度限制 单位  0.1mm  不受 nPreciseUnit影响
    uint32_t ndepth_maxcutoff;

	//分辨率的宽，高, 分配临时交换buffer时需要
	uint32_t nresw;
	uint32_t nresh;
    /** ir sensor focal length 红外传感器焦距 */
    float ir_focallength;
    /*ir sensor cmos pixel size  红外像素尺寸*/
    float ir_pixelsize;

	/*
	*
	*/
	uint32_t npixelsizeunit;

	/*
	*
	*/
	uint32_t offset;
	
	/*
	* depth output fmt: 1, depth; 2 shift
	*/
	int outfmt;
	/*
	* depth output  rotation angle
	*/
	int nrotation;

	int32_t specklesize;
	int32_t maxdiff;
	uint32_t rotateDirection;
	/*
	* fx
	*/
	float fx;
	/** The zero plane pixel size 标定平面的一个量化数值 */
	float baseline;
	/** The zero plane distance in depth units.标定距离 */
	float z0;
	/*
	* input data packed fmt: MIPI, 1; unpack, 0
	*/
	uint8_t mipi_packed;
	/*
	* undistort switch: enable, 1, disable, 0
	*/
	uint8_t enable_undistort;
	/*
	* output disparity whether of origin fmt:  origin, 1, openni, 0
	*/
	uint8_t norigindisp;
	/*
	* disparity data bit wide,  raw14, 14; raw12, 12; raw10 10
	*/
	uint8_t ndispbits;
	/*
	* disparity data bit wide,  raw14, 14; raw12, 12; raw10 10
	*/
	uint8_t nsubdispbits;
	/*
	* mx6x芯片以多少位输出的数据: raw10, raw12, raw14， raw22
	*/
	uint8_t npixels;
	uint8_t  enablefilter;
	uint8_t  enablemedianfilter;

	/*传输时的数据类型:@see mipi_transfer_datatype  10: raw10   12 :raw 12  100:12bit as raw10*/
	uint8_t ntransfer_datatype;

//#ifdef __USE_UNDISTORT_FROM_TIGER_
	/** The distance between the emitter and the Depth Cmos */
	float fEmitterDCmosDistance;
	/** The maximum possible shift value from this device. */
	uint32_t nDeviceMaxShiftValue;
	/** The maximum possible depth from this device (as opposed to a cut-off). */
	uint32_t nDeviceMaxDepthValue;
	float fVerticalFOV;
	float fHorizontalFOV;
//#endif

	//后置滤波类型  arm  dsp  @see obc_softfilter_type_t
	int npost_filter_type;
	//精度单位 10: 1mm, 1:0.1mm
	int nPreciseUnit;

} s2dconfig_t;

 

typedef struct sensor_config {
	s2dconfig_t config;
	char sensor_name[64];
	int id;
}sensor_config_t;


#ifdef __USE_UNDISTORT_FROM_TIGER_
typedef struct WorldConversionCache {
    float xzFactor;
    float yzFactor;
    float coeffX;
    float coeffY;
    int resolutionX;
    int resolutionY;
    int halfResX;
    int halfResY;
    float zFactor;
} WorldConversionCache;
#endif



typedef struct disp_cost{
   uint16_t* disparity;
   uint8_t* nconfidence_level;
}disp_cost_t;

/** Holds an orbbec version number, which consists of four separate numbers in the format: @c major.minor.maintenance.build. For example: 2.0.0.20. */
typedef struct
{
    /** Major version number, incremented for major API restructuring. */
    int major;
    /** Minor version number, incremented when significant new features added. */
    int minor;
    /** Maintenance build number, incremented for new releases that primarily provide minor bug fixes. */
    int maintenance;
    /** Build number. Incremented for each new API build. Generally not shown on the installer and download site. */
    int build;
} orbVersion;

typedef struct Point3D {
    float u;
    float v;
    float z;
} Point3D;

//typedef struct RotateMatrix {
//	double r00; double r01; double r02;
//	double r10; double r11; double r12;
//	double r20; double r21; double r22;
//} RotateMatrix;
//
//typedef struct Intrinsic {
//	double fx; double i2; double cx;
//	double i10; double fy; double cy;
//	double i20; double i21; double i22;
//} Intrinsic;
//
//typedef struct Translate {
//	double t0;
//	double t1;
//	double t2;
//} Translate;
//
//typedef struct Distortion {
//	double k0;
//	double k1;
//	double k2;
//    double k3;
//    double k4;
//}Distortion;

typedef struct RGB888{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}RGB888;

typedef struct RGBA{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
}RGBA;

typedef struct RAW10Pack {
	uint8_t P1;
	uint8_t P2;
	uint8_t P3;
	uint8_t P4;
	uint8_t P5;
} RAW10Pack;

PACK(
struct raw14_pack{
	uint32_t p1_h : 8;
	uint32_t p2_h : 8;
	uint32_t p3_h : 8;
	uint32_t p4_h : 8;
	uint32_t p1_l : 6;
	uint32_t p2_l : 6;
	uint32_t p3_l : 6;
	uint32_t p4_l : 6;
});

PACK(
struct raw12_pack{
	uint32_t p1_h : 8;
	uint32_t p2_h : 8;
	uint32_t p1_l : 4;
	uint32_t p2_l : 4;
});

typedef struct raw14_pack raw14_t;
typedef struct raw12_pack raw12_t;

enum obc_status{
	OBC_INVALIED_PARAMS = 2000,
	OBC_NULL_PTR,
	

};

#ifdef __cplusplus
}
#endif

#endif

