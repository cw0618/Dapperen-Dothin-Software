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
#include "XnDepthUnpacked16Processor.h"


#define XN_MASK_DEPTH_UNPACKED_16BIT_PROCESSOR "XnDepthUnpacked16Processor"

XnDepthUnpacked16Processor::XnDepthUnpacked16Processor(XnSensorDepthStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnDepthProcessor(pStream, pHelper, pBufferManager)
{
}

XnDepthUnpacked16Processor::~XnDepthUnpacked16Processor()
{
}

XnStatus XnDepthUnpacked16Processor::Init()
{
    XnStatus nRetVal = XnDepthProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    XnInt32 width = GetStream()->GetXRes();
    XnInt32 height = GetStream()->GetYRes();
    XnInt32 bitwide = GetStream()->GetBytesPerPixel();
    XnInt32 extraLine = GetStream()->GetMetadataLine();
    XnInt32 bufferSize = width * (height + extraLine) * bitwide;

    XN_VALIDATE_BUFFER_ALLOCATE(m_recvBuf, bufferSize);

    /// All is good.
    return XN_STATUS_OK;
}

void XnDepthUnpacked16Processor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION(XN_MASK_DEPTH_UNPACKED_16BIT_PROCESSOR)

    /// Check if there is enough room to receive packet data.
    if (m_recvBuf.GetFreeSpaceInBuffer() >= nDataSize)
    {
        /// Sometimes, when packets are lost, we get uneven number of bytes.
        /// So we need to complete one byte, in order to keep UINT16 alignment.
        if (0 != nDataSize % 2)
        {
            nDataSize--;
            pData++;
        }

        /// Copy values. Make sure we do not get corrupted pixels.
        m_recvBuf.UnsafeWrite(pData, nDataSize);

        /// All is good.
        return;
    }

    /// Not expected...
    xnLogWarning(XN_MASK_DEPTH_UNPACKED_16BIT_PROCESSOR, "Bad overflow depth: size = %d, maxSize = %d", m_recvBuf.GetSize(), m_recvBuf.GetMaxSize());
    FrameIsCorrupted();
    m_recvBuf.Reset();

    XN_PROFILING_END_SECTION
}

void XnDepthUnpacked16Processor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
#if 0 /// Call base (depth processor)
    XnBuffer* pWrite = GetWriteBuffer();
    XnIODepthFormats format = GetStream()->GetInputFormat();
    switch (format)
    {
    case XN_IO_DEPTH_FORMAT_UNCOMPRESSED_DEPTH_16_BIT:
        pWrite->UnsafeWrite(m_recvBuf.GetData(), m_recvBuf.GetSize());
        break;
    default:
        FrameIsCorrupted();
        xnLogError(XN_MASK_DEPTH_UNPACKED_16BIT_PROCESSOR, "Not supported ouput format (%d)", format);
    }

    m_recvBuf.Reset();

    /// Call base.
    XnDepthProcessor::OnEndOfFrame(pHeader);

#else /// Call base (frame processor)
    XnBuffer* pWrite = GetWriteBuffer();
    XnIODepthFormats format = GetStream()->GetInputFormat();
    switch (format)
    {
    case XN_IO_DEPTH_FORMAT_UNCOMPRESSED_DEPTH_16_BIT:
        pWrite->UnsafeWrite(m_recvBuf.GetData(), m_recvBuf.GetSize());
        break;
    default:
        FrameIsCorrupted();
        xnLogError(XN_MASK_DEPTH_UNPACKED_16BIT_PROCESSOR, "Not supported ouput format (%d)", format);
    }

    if (GetWriteBuffer()->GetSize() != GetExpectedSize())
    {
        xnLogWarning(XN_MASK_SENSOR_READ, "Read: Depth buffer is corrupt. Size is %u (!= %u)", GetWriteBuffer()->GetSize(), GetExpectedSize());
        FrameIsCorrupted();
    }
    else
    {
        // Parsing metadata if needed.
        if (GetStream()->GetMetadataLine())
            ProcessMetadata((OniMetadata*)GetWriteBuffer()->GetData(), m_recvBuf.GetData());

        OniFrame* pFrame = GetWriteFrame();
        pFrame->sensorType = ONI_SENSOR_DEPTH;
        pFrame->videoMode.fps = GetStream()->GetFPS();
        pFrame->extraLine = GetStream()->GetMetadataLine();
        pFrame->videoMode.resolutionX = GetStream()->GetXRes();
        pFrame->videoMode.resolutionY = GetStream()->GetYRes();
        pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();
        if (XN_FIRMWARE_CROPPING_MODE_DISABLED == GetStream()->GetFirmwareCropMode().GetValue())
        {
            pFrame->cropOriginX = 0;
            pFrame->cropOriginY = 0;
            pFrame->croppingEnabled = FALSE;
            pFrame->width = pFrame->videoMode.resolutionX;
            pFrame->height = pFrame->videoMode.resolutionY;
        }
        else
        {
            pFrame->croppingEnabled = TRUE;
            pFrame->width = (int)GetStream()->GetFirmwareCropSizeX().GetValue();
            pFrame->height = (int)GetStream()->GetFirmwareCropSizeY().GetValue();
            pFrame->cropOriginX = (int)GetStream()->GetFirmwareCropOffsetX().GetValue();
            pFrame->cropOriginY = (int)GetStream()->GetFirmwareCropOffsetY().GetValue();
        }
        pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

        if (ONI_PIXEL_FORMAT_DEPTH_100_UM == GetStream()->GetOutputFormat())
        {
            uint32_t scale = GetStream()->GetShiftScale();
            OniDepthPixel* pDepth = (OniDepthPixel*)((XnUInt8*)pFrame->data + pFrame->stride * pFrame->extraLine);
            for (int hei = 0; hei < pFrame->height; ++hei)
            {
                for (int wid = 0; wid < pFrame->width; ++wid)
                {
                    pDepth[hei * pFrame->width + wid] *= (XnUInt16)scale;
                }
            }
        }
    }

    m_recvBuf.Reset();

    /// Call base.
    XnFrameStreamProcessor::OnEndOfFrame(pHeader);
#endif
}
