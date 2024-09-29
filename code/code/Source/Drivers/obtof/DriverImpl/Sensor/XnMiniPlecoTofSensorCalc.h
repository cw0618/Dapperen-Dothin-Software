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
*  @brief     :    ֧��mini Pleco tof sensor ���ͼ����ֵͼ��ǿ��ͼ�ļ���
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

	//��ȡ���buffer�Ŀ��ߡ���С������֡��������չ��Ϣ
	virtual XnStatus GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize);

	//��ȡ���ͼ��Ŀ��ߡ���С��������չ��Ϣ
	virtual XnStatus GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize);
	int getWidth();
	int getHeight();
protected:
	//����sensorģʽ����ͼ���ߵȻ�������
	virtual XnStatus CalcSensorProperty();

	//������ȼ�����صĲ���
	virtual XnStatus UpdateCalcConfig();

};


#endif//__XN_MINI_PLECO_TOF_SENSOR_CALC_H_

