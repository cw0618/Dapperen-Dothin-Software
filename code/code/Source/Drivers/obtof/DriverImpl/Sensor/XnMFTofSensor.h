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
*  @brief     :    支持MF2202 tof sensor
*  @author    :   
*  @version   :    0.0.0.1
*  @date      :    2022.06.15                                                                *                                                              *                                      *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *                                                              *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#ifndef __XN_MF_TOF_SENSOR_H_
#define __XN_MF_TOF_SENSOR_H_

#include "XnTofSensor.h"

#define XN_MASK_MF_TOF_SENSOR "MFTofSensor"

class XnMFTofSensor : public XnTofSensor
{
public:
	XnMFTofSensor(mx6x_module_t *mx6xModule);

	virtual ~XnMFTofSensor();

	virtual XnStatus Init();

	virtual XnStatus GetDutyCyclePercentFormat(XnUInt32 freq, XnUInt32 ducy_cle, XnInt sensor_id, XnFloat *pduty_cycle_percent_format);

	virtual XnStatus SetIntegrationTime(XnUInt32 integration_time);

	virtual XnStatus SetDutyCycle(XnUInt32 integration_time);

	virtual XnStatus SetTriggerSignal();

	virtual XnStatus SetBinningFactor(XnUInt32 binningFactor);

	virtual XnStatus UpdateMode();

	virtual XnStatus updateFrameInfo();

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
	
	virtual XnStatus GetPhaseFrame(const XnChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnInt16 *frameData, XnUInt32 *frameDataSize);
	
	virtual XnStatus CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height);

	XnStatus setFrameResolution(int width, int height) override;
protected:
	//根据sensor模式更新图像宽高等基本参数
	virtual XnStatus CalcSensorProperty();

private:
	XnStatus GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list);

	XnStatus UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup);

	XnStatus UnpackCompressedIRFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup, int phaseIndex);

	XnStatus GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height);
	//把多个帧组成一个帧组
	XnStatus organizeFrame(XnChar *frameDataAddr, XnUInt32 outputImageSize);
	//单频的组帧
	XnStatus singleFreqorganize(XnChar *frameDataAddr, XnUInt32 outputImageSize);
	//双频的组帧
	XnStatus dualFreqorganize(XnChar *frameDataAddr, XnUInt32 outputImageSize);
	int mGroupIndex = 0;
public:
	const int kGrabExtendedDataLine = 2;
	const int kIRExtendedDataLine = 1;
	const int kOriginalPhaseWidth = 1280;
	const int kOriginalPhaseHeight = 480;
	const int kIRWidth = 640;
	const int kIRHeight = 480;
	const int kFreqPhaseInputNum = 1;
	int kFreqPhaseOutputNum = 4;
	const int kPixelBit = 12;
	const int kMipiPackBit = 12;
};


#endif//__XN_MF_TOF_SENSOR_H_


