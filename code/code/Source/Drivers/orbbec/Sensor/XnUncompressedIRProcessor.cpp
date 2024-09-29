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
#include "XnUncompressedIRProcessor.h"
#include <XnProfiling.h>
#include "XnSensor.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

/* The size of an input element for unpacking. */
#define XN_INPUT_ELEMENT_SIZE 2


/* The size of an output element for unpacking. */

#define XN_OUTPUT_ELEMENT_SIZE 2


#define XN_MAX_RES_WIDTH 1280
#define XN_MAX_RES_HEIGHT 1080

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

XnUncompressedIRProcessor::XnUncompressedIRProcessor(XnSensorIRStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_START, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_END)
    , m_nRefTimestamp(0)
    , m_DepthCMOSType(pHelper->GetFixedParams()->GetDepthCmosType())
    , m_metadataBytes(0)
{
}

XnUncompressedIRProcessor::~XnUncompressedIRProcessor()
{
}

XnStatus XnUncompressedIRProcessor::Init()
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnFrameStreamProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    XN_VALIDATE_BUFFER_ALLOCATE(m_ContinuousBuffer, XN_INPUT_ELEMENT_SIZE);

    switch (GetStream()->GetOutputFormat())
    {
    case ONI_PIXEL_FORMAT_GRAY16:
        break;
    case ONI_PIXEL_FORMAT_RGB888:
        XN_VALIDATE_BUFFER_ALLOCATE(m_UnpackedBuffer, GetExpectedOutputSize());
        break;
    case ONI_PIXEL_FORMAT_GRAY8:
        XN_VALIDATE_BUFFER_ALLOCATE(m_UnpackedBuffer, GetExpectedOutputSize() * 2);
        break;
    default:
        assert(0);
        return XN_STATUS_ERROR;
    }

    if (GetStream()->GetMetadataLine())
    {
        m_metadataBytes = GetStream()->GetXRes() * GetStream()->GetMetadataLine() * GetStream()->GetBytesPerPixel();
        XN_VALIDATE_BUFFER_ALLOCATE(m_metadataBuffer, m_metadataBytes);
    }

    return (XN_STATUS_OK);
}

XnStatus XnUncompressedIRProcessor::Unpack(const XnUInt8* pcInput, const XnUInt32 nInputSize, XnUInt16* pnOutput, XnUInt32* pnActualRead, XnUInt32* pnOutputSize)
{
    XnUInt32 nElements = nInputSize / XN_INPUT_ELEMENT_SIZE; // floored
    XnUInt32 nNeededOutput = nElements * XN_OUTPUT_ELEMENT_SIZE;
    *pnActualRead = 0;

    if (*pnOutputSize < nNeededOutput)
    {
        *pnOutputSize = 0;
        return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
    }

    *pnActualRead = nInputSize;
    *pnOutputSize = nInputSize;
    memcpy(pnOutput, pcInput, nInputSize);
    // moving points
    pcInput = pcInput + nInputSize;
    pnOutput = pnOutput + nInputSize / 2;
    return XN_STATUS_OK;
}

void XnUncompressedIRProcessor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION("XnUncompressedIRProcessor::ProcessFramePacketChunk")

    // if output format is Gray16, we can write directly to output buffer. otherwise, we need
    // to write to a temp buffer.
    XnBuffer* pWriteBuffer = (GetStream()->GetOutputFormat() == ONI_PIXEL_FORMAT_GRAY16) ? GetWriteBuffer() : &m_UnpackedBuffer;

    if (m_ContinuousBuffer.GetSize() != 0)
    {
        // fill in to a whole element
        XnUInt32 nReadBytes = XN_MIN(nDataSize, XN_INPUT_ELEMENT_SIZE - m_ContinuousBuffer.GetSize());
        m_ContinuousBuffer.UnsafeWrite(pData, nReadBytes);
        pData += nReadBytes;
        nDataSize -= nReadBytes;

        if (m_ContinuousBuffer.GetSize() == XN_INPUT_ELEMENT_SIZE)
        {
            // process it
            XnUInt32 nActualRead = 0;
            XnUInt32 nOutputSize = pWriteBuffer->GetFreeSpaceInBuffer();
            XnStatus status = XN_STATUS_OK;
            status = Unpack(m_ContinuousBuffer.GetData(), XN_INPUT_ELEMENT_SIZE, (XnUInt16*)pWriteBuffer->GetUnsafeWritePointer(), &nActualRead, &nOutputSize);
            if (XN_STATUS_OK != status)
                WriteBufferOverflowed();
            else
                pWriteBuffer->UnsafeUpdateSize(nOutputSize);

            m_ContinuousBuffer.Reset();
        }
    }

    XnUInt32 nActualRead = 0;
    XnUInt32 nOutputSize = pWriteBuffer->GetFreeSpaceInBuffer();
    XnStatus status = XN_STATUS_OK;

    status = Unpack(pData, nDataSize, (XnUInt16*)pWriteBuffer->GetUnsafeWritePointer(), &nActualRead, &nOutputSize);
    if (XN_STATUS_OK != status)
    {
        WriteBufferOverflowed();
    }
    else
    {
        pWriteBuffer->UnsafeUpdateSize(nOutputSize);

        pData += nActualRead;
        nDataSize -= nActualRead;

        // if we have any bytes left, store them for next packet
        if (nDataSize > 0)
        {
            // no need to check for overflow. there can not be a case in which more than XN_INPUT_ELEMENT_SIZE
            // are left.
            m_ContinuousBuffer.UnsafeWrite(pData, nDataSize);
        }
    }

    XN_PROFILING_END_SECTION
}



void XnUncompressedIRProcessor::IRto888(XnUInt16* pInput, XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize)
{
    XnUInt16* pInputEnd = pInput + nInputSize;
    XnUInt8* pOutputOrig = pOutput;
    XnUInt8* pOutputEnd = pOutput + *pnOutputSize;

    while (pInput != pInputEnd && pOutput < pOutputEnd)
    {

        *pOutput = (XnUInt8)((*pInput) >> 2);
        *(pOutput + 1) = *pOutput;
        *(pOutput + 2) = *pOutput;

        pOutput += 3;
        pInput++;
    }

    *pnOutputSize = (XnUInt32)(pOutput - pOutputOrig);
}

void XnUncompressedIRProcessor::IRtoGray8(XnUInt16* pInput, XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize)
{

    XnUInt16* pInputEnd = pInput + nInputSize;
    XnUInt8* pOutputOrig = pOutput;
    XnUInt8* pOutputEnd = pOutput + *pnOutputSize;

    while (pInput != pInputEnd && pOutput < pOutputEnd)
    {

        *pOutput = (XnUInt8)((*pInput) >> 2);

        pOutput += 1;
        pInput++;
    }

    *pnOutputSize = (XnUInt32)(pOutput - pOutputOrig);
}

void XnUncompressedIRProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION("XnUncompressedIRProcessor::OnEndOfFrame")

    // if there are bytes left in continuous buffer, then we have a corrupt frame
    if (m_ContinuousBuffer.GetSize() != 0)
    {
        xnLogWarning(XN_MASK_SENSOR_READ, "IR buffer is corrupt. There are left over bytes (invalid size)");
        FrameIsCorrupted();
    }

    // if data was written to temp buffer, convert it now
    switch (GetStream()->GetOutputFormat())
    {
    case ONI_PIXEL_FORMAT_GRAY16:
        break;
    case ONI_PIXEL_FORMAT_GRAY8:
    {
        XnUInt32 nOutputSize = GetWriteBuffer()->GetFreeSpaceInBuffer();
        IRtoGray8((XnUInt16*)m_UnpackedBuffer.GetData(), m_UnpackedBuffer.GetSize() / sizeof(XnUInt16), GetWriteBuffer()->GetUnsafeWritePointer(), &nOutputSize);
        GetWriteBuffer()->UnsafeUpdateSize(nOutputSize);
        m_UnpackedBuffer.Reset();
    }
    break;
    case ONI_PIXEL_FORMAT_RGB888:
    {
        XnUInt32 nOutputSize = GetWriteBuffer()->GetFreeSpaceInBuffer();
        IRto888((XnUInt16*)m_UnpackedBuffer.GetData(), m_UnpackedBuffer.GetSize() / sizeof(XnUInt16), GetWriteBuffer()->GetUnsafeWritePointer(), &nOutputSize);
        GetWriteBuffer()->UnsafeUpdateSize(nOutputSize);
        m_UnpackedBuffer.Reset();
    }
    break;
    default:
        assert(0);
        return;
    }

    // calculate expected size
    XnUInt32 width = GetStream()->GetXRes();
    XnUInt32 height = GetStream()->GetYRes();
    XnUInt32 actualHeight = height;

    // when cropping is turned on, actual depth size is smaller
    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
    {
        width = (XnUInt32)GetStream()->m_FirmwareCropSizeX.GetValue();
        height = (XnUInt32)GetStream()->m_FirmwareCropSizeY.GetValue();
        actualHeight = height;
    }
    //else if (GetStream()->GetResolution() != XN_RESOLUTION_SXGA)
    else if ((GetStream()->GetResolution() != XN_RESOLUTION_SXGA) &&
        (GetStream()->GetResolution() != XN_RESOLUTION_1280_960) &&
        (GetStream()->GetResolution() != XN_RESOLUTION_720P) &&
        (GetStream()->GetResolution() != XN_RESOLUTION_640_400) &&
        (GetStream()->GetResolution() != XN_RESOLUTION_1280_800))
    {
        if (m_DepthCMOSType == XN_DEPTH_CMOS_MT9M001)
        {
            // there are additional 8 rows (this is how the CMOS is configured)
            actualHeight += 8;
        }
    }
    else
    {
        if (m_DepthCMOSType == XN_DEPTH_CMOS_AR130)
        {
            // there missing 64 rows (this is how the CMOS is configured)
            actualHeight -= 64;
        }
    }

	xnLogWarning(XN_MASK_SENSOR_READ, "IR buffer width. is %u,height=%u ", width,height);
    XnUInt32 nExpectedBufferSize = width * (actualHeight + GetStream()->GetMetadataLine()) * GetStream()->GetBytesPerPixel();

    if (GetWriteBuffer()->GetSize() != nExpectedBufferSize)
    {
        xnLogWarning(XN_MASK_SENSOR_READ, "IR buffer is corrupt. Size is %u (!= %u)", GetWriteBuffer()->GetSize(), nExpectedBufferSize);
        FrameIsCorrupted();
    }

    // Parsing metadata if needed.
    if (GetStream()->GetMetadataLine())
    {
        m_metadataBuffer.UnsafeWrite(GetWriteBuffer()->GetData(), m_metadataBytes);
        ProcessMetadata((OniMetadata*)GetWriteBuffer()->GetData(), m_metadataBuffer.GetData());
        m_metadataBuffer.Reset();
    }

    // don't report additional rows out (so we're not using the expected buffer size)
    GetWriteBuffer()->UnsafeSetSize(width * (height + GetStream()->GetMetadataLine()) * GetStream()->GetBytesPerPixel());

    OniFrame* pFrame = GetWriteFrame();
    pFrame->sensorType = ONI_SENSOR_IR;
    pFrame->videoMode.fps = GetStream()->GetFPS();
    pFrame->extraLine = GetStream()->GetMetadataLine();
    pFrame->videoMode.resolutionX = GetStream()->GetXRes();
    pFrame->videoMode.resolutionY = GetStream()->GetYRes();
    pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();
    pFrame->width = (int)width;
    pFrame->height = (int)height;

    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
    {
        pFrame->cropOriginX = (int)GetStream()->m_FirmwareCropOffsetX.GetValue();
        pFrame->cropOriginY = (int)GetStream()->m_FirmwareCropOffsetY.GetValue();
        pFrame->croppingEnabled = TRUE;
    }
    else
    {
        pFrame->cropOriginX = 0;
        pFrame->cropOriginY = 0;
        pFrame->croppingEnabled = FALSE;
    }
    pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

    XnFrameStreamProcessor::OnEndOfFrame(pHeader);
    m_ContinuousBuffer.Reset();

    XN_PROFILING_END_SECTION
}

XnUInt64 XnUncompressedIRProcessor::CreateTimestampFromDevice(XnUInt32 nDeviceTimeStamp)
{
    XnUInt64 nNow;
    xnOSGetHighResTimeStamp(&nNow);

    // There's a firmware bug, causing IR timestamps not to advance if depth stream is off.
    // If so, we need to create our own timestamps.
    if (m_pDevicePrivateData->pSensor->GetFirmware()->GetParams()->m_Stream1Mode.GetValue() != XN_VIDEO_STREAM_DEPTH)
    {
        if (m_nRefTimestamp == 0)
        {
            m_nRefTimestamp = nNow;
        }

        return nNow - m_nRefTimestamp;
    }
    else
    {
        XnUInt64 nResult = XnFrameStreamProcessor::CreateTimestampFromDevice(nDeviceTimeStamp);

        // keep it as ref so that if depth is turned off, we'll continue from there
        m_nRefTimestamp = nNow - nResult;

        return nResult;
    }
}

void XnUncompressedIRProcessor::OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS)
{
    XnFrameStreamProcessor::OnFrameReady(nFrameID, nFrameTS);

    m_pDevicePrivateData->pSensor->GetFPSCalculator()->MarkIr(nFrameID, nFrameTS);
}
