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
*  @author    :    TF
*  @version   :    0.0.0.1
*  @date      :    2022.11.18                                                                *                                                              *                                      *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *                                                              *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#ifndef __XN_MINI_PLECO_TOF_SENSOR_H_
#define __XN_MINI_PLECO_TOF_SENSOR_H_

#include "XnTofSensor.h"

#define XN_MASK_TOF_SENSOR "MiniPlecoTofSensor"

const int kMiniGrabExtendedDataLine = 2;
const int kMiniOriginalPhaseWidth = 720;
const int kMiniOriginalPhaseHeight = 180;
const int kMiniSingleNonShuffleFreqPhaseNum = 1;
const int kMiniDualNonShuffleFreqPhaseNum = 2;
const int kMiniSingleShuffleFreqPhaseNum = 3;
const int kMiniDualShuffleFreqPhaseNum = 6;
const int kMiniDualFreqPhaseNum = 6;
const int kMiniPixelBit = 12;
const int kMiniMipiPackBit = 12;



class XnMiniPlecoTofSensor : public XnTofSensor
{
public:
	XnMiniPlecoTofSensor(mx6x_module_t *mx6xModule);

	virtual ~XnMiniPlecoTofSensor();

	virtual XnStatus Init();

	virtual XnStatus GetDutyCyclePercentFormat(XnUInt32 freq, XnUInt32 ducy_cle, XnInt sensor_id, XnFloat *pduty_cycle_percent_format);

	virtual XnStatus SetIntegrationTime(XnUInt32 integration_time);

	virtual XnStatus SetDutyCycle(XnUInt32 integration_time);

	virtual XnStatus SetTriggerSignal();

	virtual XnStatus SetBinningFactor(XnUInt32 binningFactor);

	virtual XnStatus UpdateMode();
	virtual XnStatus UpdateCalcConfig();

	//从每帧的buffer中，解析扩展信息
	virtual XnStatus ParseExtendedData(ObTofFrameInfo* framegroup, XnInt index, XnBool ifHdrMode, XnInt groupCount);

	//处理从度信接口获取的buffer，填充到帧组
	virtual XnStatus GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup);

	//获取帧数据，包括metaData和多帧相位图
	virtual XnStatus GetAIFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, OniAIFrame* pAiFrame);
	virtual XnStatus GetPhaseFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize);
	virtual XnStatus CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height);
	virtual XnStatus CalcAmplitudeFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_Amplitude_frame, int width, int height);
protected:
	//根据sensor模式更新图像宽高等基本参数
	virtual XnStatus CalcSensorProperty();

private:
	XnStatus GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list);

	XnStatus UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup);

	XnStatus GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height);
};


#endif//__XN_MINI_PLECO_TOF_SENSOR_H_

