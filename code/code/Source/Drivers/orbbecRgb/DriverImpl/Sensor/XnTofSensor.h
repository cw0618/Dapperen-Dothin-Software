#ifndef __XN_TOF_SENSOR_H_
#define __XN_TOF_SENSOR_H_

#include <vector>
#include <Driver/OniDriverAPI.h>
#include "XnOS.h"
#include "XnLog.h"
#include "XnStatusCodes.h"
#include "XnDDK.h"
#include "OpenNI.h"

#include "XnMx6xProperties.h"
#include "OniCProperties.h"
#include "XnMx6xModules.h"
#include "obstatus.h"

#include "tof_depth_calc.h"
#include "load_tof_filter_config.h"

#define  USE_FAKE_TOF_DEPTH_LIB 1

#define XN_MASK_TOF_SENSOR "TofSensor"

const int kBitPerByte = 8;
const int kBytePerPhasePixel = 2;
const int kBytePerDepthPixel = 2;
const int kBytePerAmplitudePixel = 2;
const int kBytePerInfraredPixel = 2;
const int kMaxFrequencyNum = 2;

const int kVGADepthWidth = 640;
const int kVGADepthHeight = 480;
const int kOutputParamBufSize = sizeof(float) * 100; //深度计算，用于存储配置文件内容的buf大小,算法建议此size



class XnTofSensor
{
public:
    XnTofSensor(mx6x_module_t *mx6xModule);

    virtual ~XnTofSensor();

    virtual XnStatus Init();

    virtual XnStatus GetDutyCyclePercentFormat(XnUInt32 freq, XnUInt32 ducy_cle, XnInt sensor_id, float *pduty_cycle_percent_format);

    virtual XnUInt32 GetDothinMipiPackBit(){ return m_dothinMipiPackBit; }

    virtual XnStatus SetModulationFrequency(XnUInt32 freq);

    virtual XnStatus SetIntegrationTime(XnUInt32 integration_time);

    virtual XnStatus SetDutyCycle(XnUInt32 integration_time);

    virtual XnStatus SetTriggerSignal();

    virtual XnStatus SetBinningFactor(XnUInt32 binningFactor);

    virtual XnStatus ChangeFreqMode(
		&mode);

    virtual XnStatus UpdateFrameInfo(OniFrame *frame);

    virtual XnStatus UpdateMode();

    virtual XnStatus ParseExtendedData(ObTofFrameInfo* framegroup, XnInt index);

	//获取帧数据，算出IR数据
	virtual XnStatus GetIRFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //处理从度信接口获取的buffer，填充到帧组
    virtual XnStatus GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup);

    //获取帧数据，包括metaData和多帧相位图
    virtual XnStatus GetPhaseFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //获取深度图
    virtual XnStatus GetDepthFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //获取幅值图
    virtual XnStatus GetAmplitudeFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //获取红外图
    virtual XnStatus GetInfraredFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //获取从度信读取到的buffer大小，包括帧与扩展信息
    virtual XnStatus GetGrabBufferSize(XnUInt32 *grabWidth, XnUInt32 *grabHeight, XnUInt32 *grabBufferSize);

    //获取输出buffer的宽、高、大小，包括帧数据与扩展信息
    virtual XnStatus GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize);

    //获取输出图像的宽、高、大小，不含扩展信息
    virtual XnStatus GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize);

	//计算IR
	virtual XnStatus CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame,int width,int height);
    XnStatus GetBinningFactor(XnUInt32 &binningFactor);

    //加载参数配置文件
	XnStatus LoadFilterConfig(XnChar* filePath);

    //加载标定文件
	XnStatus LoadCaliParam(XnChar* filePath);

    XnStatus SetOutputConfig(Output_Config outputConfig);
	//获取内参及畸变参数
	XnStatus getCameraParams(OBCameraParams *cmameraParams);

protected:
    //根据sensor模式更新图像宽高等基本参数
    virtual XnStatus CalcSensorProperty();

    //更新深度计算相关的参数
    virtual XnStatus UpdateCalcConfig();

    XnStatus MallocFrameGroupBuffer(ObFrameGroup *pFrameGroup);

    XnStatus FreeFrameGroupBuffer(ObFrameGroup *pFrameGroup);

    XnStatus UnpackRaw10Data(XnUInt8 *in, XnUInt16 *out, XnInt width, XnInt height);

    XnStatus UnpackRaw12Data(XnUInt8 *in, XnUInt16 *out, XnInt width, XnInt height);

    XnStatus SortMetaData(const ObTofFrameInfo &frameInfo, void *pMetaData);

    XnStatus SetProperty(XnInt id, void*data, XnInt size);

    XnStatus GetProperty(XnInt id, void*data, XnInt *size);

    XnStatus UpdateBinningFactor();

    XnStatus UpdateFrequecyMode();

    XnStatus UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup);

protected:
    XnInt m_sensorId;
    mx6x_module_t* m_pModule;
    XnUInt32 m_binningFactor;
    XnUInt32 m_freqMode;

    //每一个相位帧的分辨率
    XnUInt32 m_phaseWidth;
    XnUInt32 m_phaseHeight;
    XnUInt32 m_phaseCount;
    XnUInt m_bitPerPhasePixel;

    //利用度信API获取数据的大小
    XnUInt32 m_grabWidth;
    XnUInt32 m_grabHeight;
    XnUInt32 m_dothinMipiPackBit;

    //相位帧组
    ObFrameGroup *m_pCompressedFrameGroup;
    ObFrameGroup *m_pFrameGroup;

    //深度计算相关
    XnUInt32 m_depthWidth;
    XnUInt32 m_depthHeight;
    const XnChar *m_configFilePathAndName;
    const XnChar *m_calibFilePathAndName;
    Depth_Mode_Config m_modeConfig;
    Output_Config m_outputConfig;
    //XnChar *m_configParamBuf;
    //XnInt8 *m_calibParamBuf;
    //XnFloat *m_depth;
    //XnFloat *m_amplitude;
    //XnFloat *m_intensity;
    //XnFloat *m_tmpBuf;
    XnUInt16 *m_phaseBuf;
    XnInt m_phasePixelNum;
    std::vector<XnInt> m_phaseFrameIndex;//深度计算需要拼合多帧相位图，此为拼合顺序

};


#endif//__XN_TOF_SENSOR_H_

