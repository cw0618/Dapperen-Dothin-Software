#ifndef SOFTFILTER_H
#define SOFTFILTER_H

#include <stdint.h>

#ifdef WIN32
#  ifdef POSTF_EXPORT
#    define POSTF_API __declspec(dllexport)
#  else
#    define POSTF_API __declspec(dllimport)
#  endif
#else
#  ifdef POSTF_EXPORT
#  	 define POSTF_API __attribute__ ((visibility("default")))
#  else
#    define POSTF_API
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char uchar;
typedef unsigned short ushort;

typedef struct OBIR2DISP_VER
{
  uint8_t major;
  uint8_t minor;
  uint8_t maintenance;
  uint8_t build;
} OBIR2DISP_VER;

// int MaxDiff = 16;	//小数位4bit，maxDiff = 2^4
// int MaxSpeckleSize_1280x960 = 1920;
// int MaxSpeckleSize_640x480 = 480;
// int MaxSpeckleSize_320x240 = 120;
// int MaxSpeckleSize_160x120 = 30;

// 版本信息
POSTF_API OBIR2DISP_VER postflt_version();

// 根据图像分辨率选择处理版本(1280x960和640x480使用Opt版本，其他分辨率使用未优化版本)
POSTF_API void softfilter(
				unsigned char *buf,    	//存储中间结果，buf大小初始化为图像大小的9倍，即width*height*9字节
				unsigned short* img, 	//源图像，同时也是滤波结果图像
				int width,				//输入图像宽度
				int height, 			//输入图像高度
				int maxDiff, 			//像素差异值
				int maxSpeckleSize, 	//连通域大小的门限
				int newVal);			//用于将需要滤除的点置为newVal，newVal一般设为0

// 3x3中值滤波
POSTF_API void medianSmooth3x3(const ushort *Src,ushort *Dst,int w,int h);

#ifdef __cplusplus
}
#endif

#endif
