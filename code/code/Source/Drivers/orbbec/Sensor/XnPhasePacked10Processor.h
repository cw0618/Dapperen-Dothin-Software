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
#ifndef _XN_PHASE_PACKED10_PROCESSOR_H_
#define _XN_PHASE_PACKED10_PROCESSOR_H_
#include "XnPhaseProcessor.h"


class XnPhasePacked10Processor : public XnPhaseProcessor
{
public:
    XnPhasePacked10Processor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    virtual ~XnPhasePacked10Processor();

    XnStatus Init();

protected:
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
    virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);

private:
    XnStatus UnpackMipiRaw10(XnUInt16* pDst, const XnUInt8* pSrc);
    XnStatus UnpackRaw10To16(XnUInt16* pDst, const XnUInt8* pSrc);
    XnStatus UnpackRaw10To16Ex(XnUInt16* pDst, const XnUInt8* pSrc);

private:
    XnUInt32 m_stride;
    XnUInt32 m_paddedStride;

    XnUInt32 m_packetSize;
    XnUInt32 m_align_bytes;

    XnBuffer m_recvBuf;
};

#endif /// _XN_PHASE_PACKED10_PROCESSOR_H_
