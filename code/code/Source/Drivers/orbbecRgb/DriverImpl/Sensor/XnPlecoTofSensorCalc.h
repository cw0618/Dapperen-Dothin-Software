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
*  @brief     :    ֧��Pleco tof sensor ���ͼ����ֵͼ��ǿ��ͼ�ļ���
*  @author    :    ZhuoWeifeng
*  @version   :    
*  @date      :    2021.05.1 0                                                                *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#ifndef __XN_PLECO_TOF_SENSOR_CALC_H_
#define __XN_PLECO_TOF_SENSOR_CALC_H_

#include "XnPlecoTofSensor.h"

#define XN_MASK_PLECO_TOF_SENSOR_CALC "PlecoTofSensorCalc"



class XnPlecoTofSensorCalc : public XnPlecoTofSensor
{
public:
    XnPlecoTofSensorCalc(mx6x_module_t *mx6xModule);

    virtual ~XnPlecoTofSensorCalc();

    virtual XnStatus UpdateFrameInfo(OniFrame &frame);

	virtual XnStatus UpdateMode();
    
    //��ȡ���buffer�Ŀ��ߡ���С������֡��������չ��Ϣ
    virtual XnStatus GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize);

    //��ȡ���ͼ��Ŀ��ߡ���С��������չ��Ϣ
    virtual XnStatus GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize);

protected:
    //����sensorģʽ����ͼ���ߵȻ�������
    virtual XnStatus CalcSensorProperty();

    //������ȼ�����صĲ���
    virtual XnStatus UpdateCalcConfig();

};


#endif//__XN_PLECO_TOF_SENSOR_CALC_H_

