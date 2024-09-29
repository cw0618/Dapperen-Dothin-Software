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
#ifndef _XN_PHASE_PROCESSOR_H_
#define _XN_PHASE_PROCESSOR_H_
#include "XnFrameStreamProcessor.h"
#include "XnSensorPhaseStream.h"


class XnPhaseProcessor : public XnFrameStreamProcessor
{
public:
    XnPhaseProcessor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    virtual ~XnPhaseProcessor();

    XnStatus Init();

protected:
    virtual XnUInt32 CalculateExpectedSize();
    virtual void OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS);
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);

    inline XnSensorPhaseStream* GetStream()
    {
        return (XnSensorPhaseStream*)XnFrameStreamProcessor::GetStream();
    }
};

#endif /// _XN_PHASE_PROCESSOR_H_
