//
// Copyright 2020 ORBBEC Technology., Inc.
//
// This file belongs to ORBBEC Technology., Inc.
// It is considered a trade secret, and is not to be divulged or used by
// parties who have NOT received written authorization from the owner.
// see https:
//

/**********************************************************************************************
***           C O N F I D E N T I A L  ---  O R B B E C   Technology                        ***
***********************************************************************************************
*
*  @project   :
*  @file      :
*  @brief     :    支持mini Pleco tof sensor 深度图、幅值图、强度图的计算
*  @author    :    TF
*  @version   :
*  @date      :    2022.11.18                                                                *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#ifndef __XN_MINI_PLECO_TOF_SENSOR_CALC_H_
#define __XN_MINI_PLECO_TOF_SENSOR_CALC_H_

#include "XnMiniPlecoTofSensor.h"

#define XN_MASK_TOF_SENSOR_CALC "MiniPlecoTofSensorCalc"



class XnMiniPlecoTofSensorCalc : public XnMiniPlecoTofSensor
{
public:
	XnMiniPlecoTofSensorCalc(mx6x_module_t *mx6xModule);

	virtual ~XnMiniPlecoTofSensorCalc();

	virtual XnStatus UpdateFrameInfo(OniFrame *frame);

	virtual XnStatus UpdateMode();

	//获取输出buffer的宽、高、大小，包括帧数据与扩展信息
	virtual XnStatus GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize);

	//获取输出图像的宽、高、大小，不含扩展信息
	virtual XnStatus GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize);
	int getWidth();
	int getHeight();
protected:
	//根据sensor模式更新图像宽高等基本参数
	virtual XnStatus CalcSensorProperty();

	//更新深度计算相关的参数
	virtual XnStatus UpdateCalcConfig();

};


#endif//__XN_MINI_PLECO_TOF_SENSOR_CALC_H_

