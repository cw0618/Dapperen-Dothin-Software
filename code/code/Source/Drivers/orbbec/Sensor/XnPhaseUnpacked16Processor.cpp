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
#include <XnProfiling.h>
#include "XnPhaseUnpacked16Processor.h"


#define XN_MASK_PHASE_UNPACKED_16BIT_PROCESSOR "XnPhaseUnpacked16Processor"

XnPhaseUnpacked16Processor::XnPhaseUnpacked16Processor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnPhaseProcessor(pStream, pHelper, pBufferManager)
{
}

XnPhaseUnpacked16Processor::~XnPhaseUnpacked16Processor()
{
}

void XnPhaseUnpacked16Processor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION(XN_MASK_PHASE_UNPACKED_16BIT_PROCESSOR)

    /// When phase is uncompressed, we can just copy it directly to write buffer.
    XnBuffer* pWriteBuffer = GetWriteBuffer();

    /// Check if there is enough room to receive packet data.
    if (CheckWriteBufferForOverflow(nDataSize))
    {
        /// Sometimes, when packets are lost, we get uneven number of bytes.
        /// So we need to complete one byte, in order to keep UINT16 alignment.
        if (0 != nDataSize % 2)
        {
            nDataSize--;
            pData++;
        }

        /// Copy values. Make sure we do not get corrupted depths.
        XnUInt16* pSrc = (XnUInt16*)pData;
        OniDepthPixel* pDst = (OniDepthPixel*)pWriteBuffer->GetUnsafeWritePointer();
        while (pSrc < (XnUInt16*)(pData + nDataSize))
        {
            *pDst++ = *pSrc++;
        }
        pWriteBuffer->UnsafeUpdateSize(nDataSize);

        /// All is good.
        return;
    }

    /// Not expected...
    xnLogWarning(XN_MASK_PHASE_UNPACKED_16BIT_PROCESSOR, "Bad overflow phase: size = %d, maxSize = %d", pWriteBuffer->GetSize(), pWriteBuffer->GetMaxSize());
    FrameIsCorrupted();
    pWriteBuffer->Reset();

    XN_PROFILING_END_SECTION
}

void XnPhaseUnpacked16Processor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XnPhaseProcessor::OnEndOfFrame(pHeader);
}
