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
#ifndef _XN_PHASE_UNPACKED16_PROCESSOR_H_
#define _XN_PHASE_UNPACKED16_PROCESSOR_H_

#include "XnPhaseProcessor.h"


class XnPhaseUnpacked16Processor : public XnPhaseProcessor
{
public:
    XnPhaseUnpacked16Processor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    virtual ~XnPhaseUnpacked16Processor();

protected:
    /// Overridden Functions.
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
    virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);
};

#endif /// _XN_PHASE_UNPACKED16_PROCESSOR_H_
