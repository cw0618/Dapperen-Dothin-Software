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
#include "XnImageProcessor.h"
#include "XnSensor.h"
#include <XnProfiling.h>

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

XnImageProcessor::XnImageProcessor(XnSensorImageStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager, XnBool bCompressedOutput)
    : XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_START, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_END)
    , m_bCompressedOutput(bCompressedOutput)
{
}

XnImageProcessor::~XnImageProcessor()
{
    // unregister from properties (otherwise, callbacks will be called with deleted pointer...)
    GetStream()->XResProperty().OnChangeEvent().Unregister(m_hXResCallback);
    GetStream()->YResProperty().OnChangeEvent().Unregister(m_hYResCallback);
    GetStream()->m_FirmwareCropSizeX.OnChangeEvent().Unregister(m_hXCropCallback);
    GetStream()->m_FirmwareCropSizeY.OnChangeEvent().Unregister(m_hYCropCallback);
    GetStream()->m_FirmwareCropMode.OnChangeEvent().Unregister(m_hCropEnabledCallback);
}

XnStatus XnImageProcessor::Init()
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnFrameStreamProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetStream()->XResProperty().OnChangeEvent().Register(ActualResChangedCallback, this, m_hXResCallback);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetStream()->YResProperty().OnChangeEvent().Register(ActualResChangedCallback, this, m_hYResCallback);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetStream()->m_FirmwareCropSizeX.OnChangeEvent().Register(ActualResChangedCallback, this, m_hXCropCallback);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetStream()->m_FirmwareCropSizeY.OnChangeEvent().Register(ActualResChangedCallback, this, m_hYCropCallback);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetStream()->m_FirmwareCropMode.OnChangeEvent().Register(ActualResChangedCallback, this, m_hCropEnabledCallback);
    XN_IS_STATUS_OK(nRetVal);

    CalcActualRes();

    return (XN_STATUS_OK);
}

XnUInt32 XnImageProcessor::CalculateExpectedSize()
{
    XnUInt32 nExpectedBufferSize = GetStream()->GetXRes() * GetStream()->GetYRes();

    /// When cropping is turned on, actual depth size is smaller.
    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
        nExpectedBufferSize = (XnUInt32)(GetStream()->m_FirmwareCropSizeX.GetValue() * GetStream()->m_FirmwareCropSizeY.GetValue());

    nExpectedBufferSize = nExpectedBufferSize * GetStream()->GetBytesPerPixel() +
        GetStream()->GetXRes() * GetStream()->GetBytesPerPixel() * GetStream()->GetMetadataLine();

    return nExpectedBufferSize;
}

void XnImageProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    if (!m_bCompressedOutput)
    {
        /// Make sure data size is right.
        XnUInt32 nExpectedSize = CalculateExpectedSize();
        if (GetWriteBuffer()->GetSize() != nExpectedSize)
        {
            xnLogError(XN_MASK_SENSOR_READ, "Read: Image buffer is corrupt. Size is %u (!= %u)", GetWriteBuffer()->GetSize(), nExpectedSize);
            FrameIsCorrupted();
        }
    }

    OniFrame* pFrame = GetWriteFrame();
    pFrame->sensorType = ONI_SENSOR_COLOR;
    pFrame->videoMode.fps = GetStream()->GetFPS();
    pFrame->extraLine = GetStream()->GetMetadataLine();
    pFrame->videoMode.resolutionX = GetStream()->GetXRes();
    pFrame->videoMode.resolutionY = GetStream()->GetYRes();
    pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();

    /// If output in compressed format, the frame size should be set to the size of received data.
    if (m_bCompressedOutput)
        pFrame->dataSize = GetWriteBuffer()->GetSize();

    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
    {
        pFrame->width = (int)GetStream()->m_FirmwareCropSizeX.GetValue();
        pFrame->height = (int)GetStream()->m_FirmwareCropSizeY.GetValue();
        pFrame->cropOriginX = (int)GetStream()->m_FirmwareCropOffsetX.GetValue();
        pFrame->cropOriginY = (int)GetStream()->m_FirmwareCropOffsetY.GetValue();
        pFrame->croppingEnabled = TRUE;
    }
    else
    {
        pFrame->width = pFrame->videoMode.resolutionX;
        pFrame->height = pFrame->videoMode.resolutionY;
        pFrame->cropOriginX = 0;
        pFrame->cropOriginY = 0;
        pFrame->croppingEnabled = FALSE;
    }
    pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

    // call base
    XnFrameStreamProcessor::OnEndOfFrame(pHeader);
}

void XnImageProcessor::OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS)
{
    XnFrameStreamProcessor::OnFrameReady(nFrameID, nFrameTS);

    m_pDevicePrivateData->pSensor->GetFPSCalculator()->MarkColor(nFrameID, nFrameTS);
}

void XnImageProcessor::CalcActualRes()
{
    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
    {
        m_nActualXRes = (XnUInt32)GetStream()->m_FirmwareCropSizeX.GetValue();
        m_nActualYRes = (XnUInt32)GetStream()->m_FirmwareCropSizeY.GetValue();
    }
    else
    {
        m_nActualXRes = GetStream()->GetXRes();
        m_nActualYRes = GetStream()->GetYRes();
    }
}

XnStatus XnImageProcessor::ActualResChangedCallback(const XnProperty* /*pSender*/, void* pCookie)
{
    XnImageProcessor* pThis = (XnImageProcessor*)pCookie;
    pThis->CalcActualRes();
    return XN_STATUS_OK;
}

