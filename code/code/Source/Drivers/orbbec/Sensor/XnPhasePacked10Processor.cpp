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
#ifdef XN_NEON
#include <arm_neon.h>
#endif
#include <XnProfiling.h>
#include "XnSensor.h"
#include "XnFormatsStatus.h"
#include "XnPhasePacked10Processor.h"


#define XN_MASK_PHASE_PACKED_10BIT_PROCESSOR "XnPhasePacked10Processor"

XnPhasePacked10Processor::XnPhasePacked10Processor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnPhaseProcessor(pStream, pHelper, pBufferManager)
    , m_stride(0)
    , m_paddedStride(0)
    , m_packetSize(0)
    , m_align_bytes(0)
{
}

XnPhasePacked10Processor::~XnPhasePacked10Processor()
{
}

XnStatus XnPhasePacked10Processor::Init()
{
    XnStatus nRetVal = XnPhaseProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    XnInt32 inputFormat = GetStream()->GetInputFormat();
    switch (inputFormat)
    {
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_ALIGN_256_EXTRA_LINE_1:
        m_align_bytes = 256; ///< 256-byte alignment for RK1608.
        break;
    case XN_IO_PHASE_FORMAT_MIPI_10_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1:
        m_align_bytes = 2;
        break;
    default:
        xnLogError(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR, "Not supported input format (%d)...", inputFormat);
        return XN_STATUS_IO_INVALID_STREAM_PHASE_FORMAT;
    }

    m_stride = GetStream()->GetXRes() * 10 / 8;
    m_paddedStride = m_stride;
    if (0 != m_stride % m_align_bytes)
        m_paddedStride = (m_stride / m_align_bytes + 1) * m_align_bytes;

    m_packetSize = m_paddedStride * (GetStream()->GetYRes() + GetStream()->GetMetadataLine());
    XN_VALIDATE_BUFFER_ALLOCATE(m_recvBuf, m_packetSize + 1);

    // All is good.
    return XN_STATUS_OK;
}

void XnPhasePacked10Processor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION("XnPhasePacked10Processor::ProcessFramePacketChunk")

    // Check if there is enough room to receive packet data.
    if (m_recvBuf.GetFreeSpaceInBuffer() >= nDataSize)
    {
        // Copy values. Make sure we do not get corrupted pixels.
        m_recvBuf.UnsafeWrite(pData, nDataSize);

        // All is good.
        return;
    }

    // Not expected...
    xnLogWarning(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR, "Bad overflow phase! size = %d, maxSize = %d", m_recvBuf.GetSize(), m_recvBuf.GetMaxSize());
    FrameIsCorrupted();
    m_recvBuf.Reset();

    XN_PROFILING_END_SECTION
}

XnStatus XnPhasePacked10Processor::UnpackRaw10To16(XnUInt16* pDst, const XnUInt8* pSrc)
{
    uint16_t* p = pDst;
    for (uint32_t hei = 0; hei < GetStream()->GetYRes(); ++hei)
    {
        const uint8_t* pLine = pSrc + m_paddedStride * hei;
        for (uint32_t wid = 0; wid < m_stride; wid += 5)
        {
            OniRaw10Packed* pPack = (OniRaw10Packed*)&pLine[wid];
            p[0] = pPack->p1 | ((pPack->p2 & 0x03) << 8);
            p[1] = (pPack->p2 >> 2) | ((pPack->p3 & 0x0F) << 6);
            p[2] = (pPack->p3 >> 4) | ((pPack->p4 & 0x3F) << 4);
            p[3] = (pPack->p4 >> 6) | (pPack->p5 << 2);
            p += 4;
        }
    }

    return XN_STATUS_OK;
}

#ifdef XN_NEON
XnStatus XnPhasePacked10Processor::UnpackRaw10To16Ex(XnUInt16* pDst, const XnUInt8* pSrc)
{
    uint16_t* p = pDst;
    uint8x8_t uint8x8x5[5] = { 0 };
    uint16x8x4_t u16x8x4 = { 0 };
    for (uint32_t i = 0; i < GetStream()->GetXRes() * GetStream()->GetYRes() * 10 / 8; i += 40, p += 32)
    {
        uint8x8x5[0] = vld1_u8(&pSrc[i]);
        uint8x8x5[1] = vld1_u8(&pSrc[i] + 8);
        uint8x8x5[2] = vld1_u8(&pSrc[i] + 16);
        uint8x8x5[3] = vld1_u8(&pSrc[i] + 24);
        uint8x8x5[4] = vld1_u8(&pSrc[i] + 32);

        u16x8x4.val[0][0] = uint8x8x5[0][0] | ((uint8x8x5[0][1] & 0x03) << 8);
        u16x8x4.val[0][1] = (uint8x8x5[0][1] >> 2) | ((uint8x8x5[0][2] & 0x0F) << 6);
        u16x8x4.val[0][2] = (uint8x8x5[0][2] >> 4) | ((uint8x8x5[0][3] & 0x3F) << 4);
        u16x8x4.val[0][3] = (uint8x8x5[0][3] >> 6) | (uint8x8x5[0][4] << 2);
        u16x8x4.val[0][4] = uint8x8x5[0][5] | ((uint8x8x5[0][6] & 0x03) << 8);
        u16x8x4.val[0][5] = (uint8x8x5[0][6] >> 2) | ((uint8x8x5[0][7] & 0x0F) << 6);
        u16x8x4.val[0][6] = (uint8x8x5[0][7] >> 4) | ((uint8x8x5[1][0] & 0x3F) << 4);
        u16x8x4.val[0][7] = (uint8x8x5[1][0] >> 6) | (uint8x8x5[1][1] << 2);

        u16x8x4.val[1][0] = uint8x8x5[1][2] | ((uint8x8x5[1][3] & 0x03) << 8);
        u16x8x4.val[1][1] = (uint8x8x5[1][3] >> 2) | ((uint8x8x5[1][4] & 0x0F) << 6);
        u16x8x4.val[1][2] = (uint8x8x5[1][4] >> 4) | ((uint8x8x5[1][5] & 0x3F) << 4);
        u16x8x4.val[1][3] = (uint8x8x5[1][5] >> 6) | (uint8x8x5[1][6] << 2);
        u16x8x4.val[1][4] = uint8x8x5[1][7] | ((uint8x8x5[2][0] & 0x03) << 8);
        u16x8x4.val[1][5] = (uint8x8x5[2][0] >> 2) | ((uint8x8x5[2][1] & 0x0F) << 6);
        u16x8x4.val[1][6] = (uint8x8x5[2][1] >> 4) | ((uint8x8x5[2][2] & 0x3F) << 4);
        u16x8x4.val[1][7] = (uint8x8x5[2][2] >> 6) | (uint8x8x5[2][3] << 2);

        u16x8x4.val[2][0] = uint8x8x5[2][4] | ((uint8x8x5[2][5] & 0x03) << 8);
        u16x8x4.val[2][1] = (uint8x8x5[2][5] >> 2) | ((uint8x8x5[2][6] & 0x0F) << 6);
        u16x8x4.val[2][2] = (uint8x8x5[2][6] >> 4) | ((uint8x8x5[2][7] & 0x3F) << 4);
        u16x8x4.val[2][3] = (uint8x8x5[2][7] >> 6) | (uint8x8x5[3][0] << 2);
        u16x8x4.val[2][4] = uint8x8x5[3][1] | ((uint8x8x5[3][2] & 0x03) << 8);
        u16x8x4.val[2][5] = (uint8x8x5[3][2] >> 2) | ((uint8x8x5[3][3] & 0x0F) << 6);
        u16x8x4.val[2][6] = (uint8x8x5[3][3] >> 4) | ((uint8x8x5[3][4] & 0x3F) << 4);
        u16x8x4.val[2][7] = (uint8x8x5[3][4] >> 6) | (uint8x8x5[3][5] << 2);

        u16x8x4.val[3][0] = uint8x8x5[3][6] | ((uint8x8x5[3][7] & 0x03) << 8);
        u16x8x4.val[3][1] = (uint8x8x5[3][7] >> 2) | ((uint8x8x5[4][0] & 0x0F) << 6);
        u16x8x4.val[3][2] = (uint8x8x5[4][0] >> 4) | ((uint8x8x5[4][1] & 0x3F) << 4);
        u16x8x4.val[3][3] = (uint8x8x5[4][1] >> 6) | (uint8x8x5[4][2] << 2);
        u16x8x4.val[3][4] = uint8x8x5[4][3] | ((uint8x8x5[4][4] & 0x03) << 8);
        u16x8x4.val[3][5] = (uint8x8x5[4][4] >> 2) | ((uint8x8x5[4][5] & 0x0F) << 6);
        u16x8x4.val[3][6] = (uint8x8x5[4][5] >> 4) | ((uint8x8x5[4][6] & 0x3F) << 4);
        u16x8x4.val[3][7] = (uint8x8x5[4][6] >> 6) | (uint8x8x5[4][7] << 2);

        vst1q_u16(p, u16x8x4.val[0]);
        vst1q_u16(p + 8, u16x8x4.val[1]);
        vst1q_u16(p + 16, u16x8x4.val[2]);
        vst1q_u16(p + 24, u16x8x4.val[3]);
    }

    return XN_STATUS_OK;
}
#else
XnStatus XnPhasePacked10Processor::UnpackRaw10To16Ex(XnUInt16* pDst, const XnUInt8* pSrc)
{
    uint16_t *p = pDst;
    for (uint32_t i = 0; i < GetStream()->GetXRes() * GetStream()->GetYRes() * 10 / 8; i += 5, p += 4)
    {
        OniRaw10Packed* item = (OniRaw10Packed*)&pSrc[i];
        p[0] = item->p1 | ((item->p2 & 0x03) << 8);
        p[1] = (item->p2 >> 2) | ((item->p3 & 0x0F) << 6);
        p[2] = (item->p3 >> 4) | ((item->p4 & 0x3F) << 4);
        p[3] = (item->p4 >> 6) | (item->p5 << 2);
    }

    return XN_STATUS_OK;
}
#endif

int XnPhasePacked10Processor::UnpackMipiRaw10(XnUInt16* pDst, const XnUInt8* pSrc)
{
    uint16_t* p = pDst;
    uint32_t size = GetStream()->GetXRes() * GetStream()->GetYRes() * 10 / 8;
    for (uint32_t i = 0; i < size; i += 5)
    {
        OniRaw10Packed* pItem = (OniRaw10Packed*)&pSrc[i];
        p[0] = (pItem->p1 << 2) | (pItem->p5 & 0x03);
        p[1] = (pItem->p2 << 2) | ((pItem->p5 >> 2) & 0x03);
        p[2] = (pItem->p3 << 2) | ((pItem->p5 >> 4) & 0x03);
        p[3] = (pItem->p4 << 2) | ((pItem->p5 >> 6) & 0x03);
        p += 4;
    }

    return XN_STATUS_OK;
}

void XnPhasePacked10Processor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR)

    if (m_recvBuf.GetSize() != m_packetSize)
    {
        xnLogError(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR, "Read: Phase buffer is corrupt. Size is %u (!= %u)", m_recvBuf.GetSize(), m_packetSize);
        FrameIsCorrupted();
    }
    else
    {
        // Get the extra data first.
        ProcessMetadata((OniMetadata*)GetWriteBuffer()->GetData(), m_recvBuf.GetData());

        // Then unpack the phase data.
        XnUInt32 srcOffset = m_paddedStride * GetStream()->GetMetadataLine();
        XnUInt32 dstOffset = GetStream()->GetXRes() * GetStream()->GetBytesPerPixel() * GetStream()->GetMetadataLine();
        switch (GetStream()->GetInputFormat())
        {
        case XN_IO_PHASE_FORMAT_MIPI_10_BIT_EXTRA_LINE_1:
            UnpackMipiRaw10((XnUInt16*)(GetWriteBuffer()->GetData() + dstOffset), m_recvBuf.GetData() + srcOffset);
            break;
        case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1:
            UnpackRaw10To16Ex((XnUInt16*)(GetWriteBuffer()->GetData() + dstOffset), m_recvBuf.GetData() + srcOffset);
            break;
        case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_ALIGN_256_EXTRA_LINE_1:
            UnpackRaw10To16((XnUInt16*)(GetWriteBuffer()->GetData() + dstOffset), m_recvBuf.GetData() + srcOffset);
            break;
        default:
            FrameIsCorrupted();
            xnLogError(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR, "Not supported input format (%d)...", GetStream()->GetInputFormat());
        }

        OniPixelFormat format = GetStream()->GetOutputFormat();
        switch (format)
        {
        case ONI_PIXEL_FORMAT_GRAY16:
            break;
        case ONI_PIXEL_FORMAT_RGB888:
            // TODO
            break;
        default:
            FrameIsCorrupted();
            xnLogError(XN_MASK_PHASE_PACKED_10BIT_PROCESSOR, "Not supported ouput format (%d)", format);
        }

        GetWriteBuffer()->UnsafeUpdateSize(CalculateExpectedSize());
    }

    m_recvBuf.Reset();
    XnPhaseProcessor::OnEndOfFrame(pHeader);

    XN_PROFILING_END_SECTION
}
