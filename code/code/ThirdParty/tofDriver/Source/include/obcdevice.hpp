//
// Copyright 2020 ORBBEC Technology., Inc.
//
// This file belongs to ORBBEC Technology., Inc.
// It is considered a trade secret, and is not to be divulged or used by
// parties who have NOT received written authorization from the owner.
// see https:
//

/***********************************************************************************************
***           C O N F I D E N T I A L  ---  O R B B E C   Technology                        ***
***********************************************************************************************
*
*  @file      :    obcdevice.hpp
*  @brief     :    本文件主要定义了obcdevice 设备类，用于访问orbbec 度信设备
*  @date      :                                                                               *
*  @update    :    2020.06.16                                                                 *                                      *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *                                                               *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#ifndef __OBC_DEVICE_INC__
#define __OBC_DEVICE_INC__
#include <iostream>
#include <stdint.h>
#include <vector>
#include "obstatus.h"
#include "typedef.h"

#ifdef _WIN32
#define OBC_API_EXPORT __declspec(dllexport)
#else
#define OBC_API_EXPORT __attribute__ ((visibility("default")))
#endif


namespace orbbec{
    
    class  ObcDeviceHelper;

    /**
    * 默认的obbrec 设备类
    * 提供了访问orbbec 设备的常用API
    */
    class OBC_API_EXPORT ObcDevice{

public: 

    virtual ~ObcDevice(){}; 

    /**
    * @brief  Get SDK Version
    * 
    * @param  sdk_ver * version [out]   version info
    *
    * @return int  0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  GetSdkVer(sdk_ver* version) = 0;

    /**
    * @brief       通过配置文件初始化参数，打开设备之后必须先初始化，然后加载固件/参考图/打开流
    *
    * @param   ini_cfg [in]     配置文件完整路径名称.
    * @return  接口调用结果: 失败 < 0. 成功 0.
    */
    //@Deprecated
    virtual int  Init(const char* inifile) = 0;
    /**
    * @brief  更改设备状态
    *
    * @param  const int enable [in] 设备状态  0:关闭，1：打开
    *
    * @return int  0：成功；非0：  失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  EnableDevice(const int id) = 0;
    /**
    * @brief 加载固件
    *
    * @param  const char * filename  固件文件
    *
    * @return int  0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  LoadFirmware(const char* filename) = 0;
    /**
    * @brief  加载参考文件
    *
    * @param  const char * filename 参考文件
    *
    * @return int  0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  LoadReference(const char*  filename) = 0;
    /**
    * @brief  打开流
    *
    * @param  obc_stream_type_t type 流类型
    * @param  int w	分辨率 宽
    * @param  int h	分辨率 高
    *
    * @return int 0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  OpenStream(obc_stream_type_t type, int w, int h) = 0;
    /**
    * @brief 关闭指定流
    *
    * @param   type  流类型
    *
    * @return int 0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  StopStream(obc_stream_type_t type) = 0;
    /**
    * @brief 等待获取一帧数据，没有数据时会阻塞
    * 
    * @param  [in]  type    流类型
    * @param  [out]  buffer    返回数据帧buffer
    * @param  [in]  width   分辨率 宽
    * @param  [in]  height  分辨率 高
    *
    * @return int 0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  WaitforAnyFrame(obc_stream_type_t type, char* buffer, int width, int height) = 0;
	
    virtual int   WaitforFrameGroup(ObcFrameGroup *tof_frames)=0;


    virtual int   SetTofFrameMode(ObcTofFrameMode & cmd) = 0;
    virtual int   GetTofFrameMode(ObcTofFrameMode & cmd) = 0;

    // @Deprecated
    // virtual int GetRequestFrameGroupCmd(ObcFrameGroupRequestCmd & cmd)=0;
	
    virtual void SetLoggerConfig(ObcLoggerConfig &config) = 0;
    virtual void GetLoggerConfig(ObcLoggerConfig &config) = 0;

    virtual void SetEnvConfig(std::string driver_path, std::string driver_name_list) = 0;
    virtual void GetEnvConfig(std::string driver_path, std::string driver_name_list) = 0;

    /**
    * @brief  设置帧回调
    * 
    * @param  depth_shape shape
    * @param  obframecapture cb
    *
    * @return int  0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  SetFrameCallBack(depth_shape shape, obframecapture cb)= 0;


    /**
    * @brief  获取给定流类型 支持的streamMode列表
    *
    * @param  obc_stream_type_t type  [in] 给定的流类型
    * @param  vector<obc_videomode> & modes    [out] 获取的StreamMode  列表
    *
    * @return int  0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  GetSupportStreamMode(obc_stream_type_t type, std::vector<obc_videomode>& modes) = 0;
	
    /**
    * @brief 是否支持当前流
    *
    * @param  obc_stream_type_t type 流类型
    */
    virtual int  IsStreamSupported(obc_stream_type_t type) = 0;
	
    /**
    * @brief 是否支持该属性
    * 
    * @param int id [in] 属性id
    */
    virtual int  IsPropertySupported(int id) = 0;
	
    /**
    * @brief 读取给定属性
    * 
    * @param  int id        [in]    属性id
    * @param  void * data   [out]   属性存储空间
    * @param  int * size    [out]   输入属性空间大小，返回实际属性值大小
    *
    * @return int 
    *       0  ：成功;
    *       非0：失败; 错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  GetProperty(int id, void* data, int* size) = 0; 
	
    /**
    * @brief 设置属性
    * 
    * @param   id       [in]    属性id
    * @param  data      [in]    属性存储空间
    * @param  size      [in]    属性空间大小
    *
    * @return int		0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    virtual int  SetProperty(int id, void* data, int size) = 0; 

  protected:
    ObcDevice(){};
}; 

	OBC_API_EXPORT int GetLoggerConfig(ObcLoggerConfig & config);
	OBC_API_EXPORT int SetLoggerConfig(ObcLoggerConfig & config);

	OBC_API_EXPORT int SetDriverEnvConfig(std::string list, std::string driver_dir);
	OBC_API_EXPORT int GetDriverEnvConfig(std::string &list, std::string& driver_dir);


    /// RAW图像4种输出格式定义。 ObcRAWFORMAT_RGB
    enum ObcRawFormatRGB
    {
        OBC_FORMAT_RGGB = 0, ///<RGGB输出格式
        OBC_FORMAT_GRBG,     ///<GRBG输出格式
        OBC_FORMAT_GBRG,     ///<GBRG输出格式
        OBC_FORMAT_BGGR,     ///<BGGR输出格式
    };

    /* 将RGBRaw图转化为RGB888
    *
    *   详细说明5
    *   详细说明5...
    *
    *   @param uint8_t *p_raw       原始的raw图，不需要mipi解包。
    *   @param uint8_t * p_rgb888   输出的rgb888图，一定要给够内存空间，不小于w*h*3.
    *   @param int width,           图像分辨率
    *   @param int height,
    *   @param int raw_format = 0    默认的原始图像格式  see@ObcRawFormatRGB
    *   @return         返回值的说明...
    */
    OBC_API_EXPORT int ObdeviceRawToRGB(uint8_t *p_raw, uint8_t * p_rgb888, int width, int height, int raw_format = OBC_FORMAT_BGGR);
   /**
    * @brief  静态接入点 获取当前设备列表
    *
    * @param  char * devlist[]		[out]	设备列表
    * @param  int * number			[in]	输入枚举设备最大数，输出枚举到的设备数
    *
    * @return OBC_API_EXPORT int    0：成功；非0：失败; 错误码定义 参加 obstatus.h  ErrorCode
    */
     OBC_API_EXPORT int EnumerateDevice(char* devlist[], int* number);

    /**
    * @brief  静态接入点 旋转数据接口
    *
    * @param  uint16_t * src    [in]    输入需要旋转的数据
    * @param  uint16_t * dst    [out]   输出旋转后的数据，仅支持 -90, 90度的旋转
    *
    * @return OBC_API_EXPORT int    0：成功；非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    OBC_API_EXPORT int Rotation16bit(const uint16_t* src, uint16_t* dst, int width, int height, int angle);
    /**
    * @brief 获取设备
    * 
    * @param  const char * devname  [in]    设备名
    * @param  int id                [in]    设备id
    *
    * @return OBC_API_EXPORT ObcDevice* 设备指针
    */
     OBC_API_EXPORT ObcDevice* GetDevice(const char* devname, int id); 
    /**
    * @brief 获取设备
    *
    * @param  const char * devname  [in]    设备名
    * @param  int id                [in]    设备id
    * @param  ObcDevice**ppDevice   [out]   用于获取设备指针
    * @return OBC_API_EXPORT    ObcDevice*	设备指针
    */
    OBC_API_EXPORT int GetDeviceByName(const char* kName, int id, ObcDevice**ppDevice);
    /**
    * @brief 获取设备版本号
    * 
    * @param  uint32_t * version  [out]    获取到的设备信息  格式 OBCDEVICE_VERSION_MAJOR << 24 | OBCDEVICE_VERSION_MINOR << 16 | OBCDEVICE_VERSION_REVISION << 8 | OBCDEVICE_VERSION_BUILD
    *
    * @return OBC_API_EXPORT int  
    *       0：  成功；
    *       非0：失败;错误码定义 参加 obstatus.h  ErrorCode
    */
    OBC_API_EXPORT int GetVersion(uint32_t* version); 
    }; 
#endif