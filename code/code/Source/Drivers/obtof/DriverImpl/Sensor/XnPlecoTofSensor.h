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
*  @project   :
*  @file      :
*  @brief     :    支持Pleco tof sensor
*  @author    :    
*  @version   :    0.0.0.1
*  @date      :    2021.02.20                                                                *                                                              *                                      *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *                                                              *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#ifndef __XN_PLECO_TOF_SENSOR_H_
#define __XN_PLECO_TOF_SENSOR_H_

#include "XnTofSensor.h"

#define XN_MASK_PLECO_TOF_SENSOR "PlecoTofSensor"

class XnPlecoTofSensor : public XnTofSensor
{
public:
    XnPlecoTofSensor(mx6x_module_t *mx6xModule);

    virtual ~XnPlecoTofSensor();

    virtual XnStatus Init();

    virtual XnStatus GetDutyCyclePercentFormat(XnUInt32 freq, XnUInt32 ducy_cle, XnInt sensor_id, XnFloat *pduty_cycle_percent_format);

    virtual XnStatus SetIntegrationTime(XnUInt32 integration_time);

    virtual XnStatus SetDutyCycle(XnUInt32 integration_time);

    virtual XnStatus SetTriggerSignal();

    virtual XnStatus SetBinningFactor(XnUInt32 binningFactor);

    virtual XnStatus UpdateMode();

    //从每帧的buffer中，解析扩展信息
    virtual XnStatus ParseExtendedData(ObTofFrameInfo* framegroup, XnInt index);
    
    //处理从度信接口获取的buffer，填充到帧组
    virtual XnStatus GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup);

	//获取帧数据，算出IR数据
	virtual XnStatus GetIRFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

	//获取深度图
	virtual XnStatus GetDepthFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);

    //获取帧数据，包括metaData和多帧相位图
	virtual XnStatus GetAIFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, OniAIFrame* pAiFrame);
    
	virtual XnStatus GetPhaseFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);
	
	virtual XnStatus CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height);

	XnStatus setFrameResolution(int width, int height) override;
protected:
    //根据sensor模式更新图像宽高等基本参数
    virtual XnStatus CalcSensorProperty();

private:
    XnStatus GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list);

    XnStatus UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup);

	XnStatus GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height);
public:
	const int kGrabExtendedDataLine = 2;
	const int kOriginalPhaseWidth = 1920;
	const int kOriginalPhaseHeight = 480;
	const int kIRWidth = 640;
	const int kIRHeight = 480;
	const int kSingleFreqPhaseInputNum = 3;
	const int kSingleFreqPhaseOutputNum = 3;
	const int kDualFreqPhaseInputNum = 6;
	const int kDualFreqPhaseOutputNum = 6;
	const int kPixelBit = 10;
	const int kMipiPackBit = 10;
};


#endif//__XN_PLECO_TOF_SENSOR_H_

