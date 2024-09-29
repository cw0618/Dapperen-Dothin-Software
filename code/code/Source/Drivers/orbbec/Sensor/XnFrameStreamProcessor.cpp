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
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "XnFrameStreamProcessor.h"
#include "XnSensor.h"
#include <XnProfiling.h>

#define XN_MASK_FRAME_STREAM_PROCESSOR "XnFrameStreamProcessor"

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
XnFrameStreamProcessor::XnFrameStreamProcessor(XnFrameStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager, XnUInt16 nTypeSOF, XnUInt16 nTypeEOF)
    : XnStreamProcessor(pStream, pHelper)
    , m_nTypeSOF(nTypeSOF)
    , m_nTypeEOF(nTypeEOF)
    , m_pTripleBuffer(pBufferManager)
    , m_InDump(NULL)
    , m_InternalDump(NULL)
    , m_bFrameCorrupted(FALSE)
    , m_bAllowDoubleSOF(FALSE)
    , m_nLastSOFPacketID(0)
    , m_nFirstPacketTimestamp(0)
{
    sprintf(m_csInDumpMask, "%sIn", pStream->GetType());
    sprintf(m_csInternalDumpMask, "Internal%s", pStream->GetType());
    m_InDump = xnDumpFileOpen(m_csInDumpMask, "%s_0.raw", m_csInDumpMask);
    m_InternalDump = xnDumpFileOpen(m_csInternalDumpMask, "%s_0.raw", m_csInternalDumpMask);
}

XnFrameStreamProcessor::~XnFrameStreamProcessor()
{
    xnDumpFileClose(m_InDump);
    xnDumpFileClose(m_InternalDump);
}

void XnFrameStreamProcessor::ProcessPacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION("XnFrameStreamProcessor::ProcessPacketChunk");

    // if first data from SOF packet
    if (pHeader->nType == m_nTypeSOF && nDataOffset == 0)
    {
        if (!m_bAllowDoubleSOF || pHeader->nPacketID != (m_nLastSOFPacketID + 1))
        {
            m_nLastSOFPacketID = pHeader->nPacketID;
            OnStartOfFrame(pHeader);
        }
    }

    if (!m_bFrameCorrupted)
    {
        xnDumpFileWriteBuffer(m_InDump, pData, nDataSize);
        ProcessFramePacketChunk(pHeader, pData, nDataOffset, nDataSize);
    }

    // if last data from EOF packet
    if (pHeader->nType == m_nTypeEOF && (nDataOffset + nDataSize) == pHeader->nBufSize)
    {
        OnEndOfFrame(pHeader);
    }

    // ADD by ZW
    if (m_bIgnoreEOF == TRUE)
    {
        OnEndOfFrame(pHeader);
    }

    XN_PROFILING_END_SECTION
}

void XnFrameStreamProcessor::OnPacketLost()
{
    FrameIsCorrupted();
}

void XnFrameStreamProcessor::OnStartOfFrame(const XnSensorProtocolResponseHeader* /*pHeader*/)
{
    m_bFrameCorrupted = FALSE;
    m_pTripleBuffer->GetWriteBuffer()->Reset();
    if (m_pDevicePrivateData->pSensor->ShouldUseHostTimestamps())
    {
        m_nFirstPacketTimestamp = GetHostTimestamp();
    }
}

void XnFrameStreamProcessor::ProcessMetadata(OniMetadata* pMeta, const XnUInt8* pData)
{
    Metadata* pInner = (Metadata*)pData;
    if (!pMeta || !pInner)
    {
        xnLogError(XN_MASK_FRAME_STREAM_PROCESSOR, "Bad input parameters...");
        return;
    }

    pMeta->width = pInner->width;
    pMeta->height = pInner->height;

    pMeta->frameIndex = pInner->frameIndex;
    pMeta->groupIndex = pInner->groupIndex;

    pMeta->temperTX = pInner->temperTX;
    pMeta->temperRX = pInner->temperRX;
    pMeta->temperDelay = pInner->temperDelay;

    pMeta->frequency[0] = pInner->frequency[0];
    pMeta->frequency[1] = pInner->frequency[1];
    pMeta->dutyCycle[0] = pInner->dutyCycle[0];
    pMeta->dutyCycle[1] = pInner->dutyCycle[1];
    pMeta->integration[0] = pInner->intergration[0];
    pMeta->integration[1] = pInner->intergration[1];

    pMeta->id = (OniSensorID)pInner->id;
    pMeta->mode = (OniTOFSensorMode)pInner->mode;
    pMeta->type = (OniFrameType)pInner->frameType;
    pMeta->shuffle = pInner->shuffle ? true : false;
}

void XnFrameStreamProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    m_bIgnoreEOF = FALSE;
    // write dump
    XnBuffer* pCurWriteBuffer = m_pTripleBuffer->GetWriteBuffer();
    xnDumpFileWriteBuffer(m_InternalDump, pCurWriteBuffer->GetData(), pCurWriteBuffer->GetSize());
    xnDumpFileClose(m_InternalDump);
    xnDumpFileClose(m_InDump);

    if (!m_bFrameCorrupted)
    {
        // mark the buffer as stable
        XnUInt64 nTimestamp;
        if (m_pDevicePrivateData->pSensor->ShouldUseHostTimestamps())
        {
            // use the host timestamp of the first packet
            nTimestamp = m_nFirstPacketTimestamp;
        }
        else
        {
            // use timestamp in last packet
            nTimestamp = CreateTimestampFromDevice(pHeader->nTimeStamp);
        }

        OniFrame* pFrame = m_pTripleBuffer->GetWriteFrame();
        pFrame->timestamp = nTimestamp;

        XnUInt32 nFrameID;
        m_pTripleBuffer->MarkWriteBufferAsStable(&nFrameID);

        // let inheriting classes do their stuff
        OnFrameReady(nFrameID, nTimestamp);
    }
    else
    {
        // restart
        m_pTripleBuffer->GetWriteBuffer()->Reset();
    }

    // log bandwidth
    XnUInt64 nSysTime;
    xnOSGetTimeStamp(&nSysTime);
    xnDumpFileWriteString(m_pDevicePrivateData->BandwidthDump, "%llu,%s,%d,%d\n",
        nSysTime, m_csName, GetCurrentFrameID(), m_nBytesReceived);

    // re-init dumps
    m_InDump = xnDumpFileOpen(m_csInDumpMask, "%s_%d.raw", m_csInDumpMask, GetCurrentFrameID());
    m_InternalDump = xnDumpFileOpen(m_csInternalDumpMask, "%s_%d.raw", m_csInternalDumpMask, GetCurrentFrameID());
    m_nBytesReceived = 0;
}

void XnFrameStreamProcessor::FrameIsCorrupted()
{
    if (!m_bFrameCorrupted)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "%s frame is corrupt!", m_csName);
        m_bFrameCorrupted = TRUE;
    }
}

void XnFrameStreamProcessor::WriteBufferOverflowed()
{
    XnBuffer* pBuffer = GetWriteBuffer();
    xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "%s Frame Buffer overflow! current size: %d", m_csName, pBuffer->GetSize());
    FrameIsCorrupted();
}

void Channel2RowDownEven(const unsigned char* src_argb, int /*src_stride*/, int src_stepx,
    unsigned char* dst_argb, int dst_width) {
    const unsigned short* src = (const unsigned short*)(src_argb);
    unsigned short* dst = (unsigned short*)(dst_argb);

    int x;
    for (x = 0; x < dst_width - 1; x += 2) {
        dst[0] = src[0];
        dst[1] = src[src_stepx];
        src += (src_stepx << 1);
        dst += 2;
    }
    if (dst_width & 1) {
        dst[0] = src[0];
    }

}

void Channel2TransposeLeft(const unsigned char* src, int src_stride,
    unsigned char* dst, int dst_stride, int width, int height) {
    int i;
    int src_pixel_step = src_stride >> 1;

    for (i = 0; i < width; ++i) {
        Channel2RowDownEven(src, 0, src_pixel_step, dst, height);
        dst += dst_stride;
        src += 2;
    }

}

void XnFrameStreamProcessor::Channel2Rotate90(const unsigned char* src, int src_stride,
    unsigned char* dst, int dst_stride, int width, int height) {
    src += src_stride * (height - 1);
    src_stride = -src_stride;
    Channel2TransposeLeft(src, src_stride, dst, dst_stride, width, height);
}
