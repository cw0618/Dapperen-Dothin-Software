/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef _XN_ONI_AI_STREAM_H_
#define _XN_ONI_AI_STREAM_H_

#include "XnOniMapStream.h"
#include "../Sensor/XnSensor.h"


class XnOniAIStream : public XnOniMapStream
{
public:
    XnOniAIStream(XnSensor* pSensor, XnOniDevice* pDevice);

    virtual OniBool isPropertySupported(int propertyId);
    virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize);
};

#endif /// _XN_ONI_AI_STREAM_H_
