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
#include "XnSensor.h"
#include "XnPhaseProcessor.h"


XnPhaseProcessor::XnPhaseProcessor(XnSensorPhaseStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_PHASE_START, XN_SENSOR_PROTOCOL_RESPONSE_PHASE_END)
{
}

XnPhaseProcessor::~XnPhaseProcessor()
{
}

XnStatus XnPhaseProcessor::Init()
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnFrameStreamProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    /// All is good.
    return XN_STATUS_OK;
}

XnUInt32 XnPhaseProcessor::CalculateExpectedSize()
{
    XnUInt32 imageBytes = GetStream()->GetXRes() * GetStream()->GetYRes() * GetStream()->GetBytesPerPixel();
    XnUInt32 extraBytes = GetStream()->GetXRes() * GetStream()->GetBytesPerPixel() * GetStream()->GetMetadataLine();

    return (imageBytes + extraBytes);
}

void XnPhaseProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION(XN_MASK_SENSOR_PROTOCOL_PHASE)

    XnUInt32 expectedSize = CalculateExpectedSize();
    XnUInt32 outputSize = GetWriteBuffer()->GetSize();
    if (outputSize != expectedSize)
    {
        xnLogError(XN_MASK_SENSOR_READ, "Read: Phase buffer is corrupt. Size is %u (!= %u)", outputSize, expectedSize);
        FrameIsCorrupted();
    }

    OniFrame* pFrame = GetWriteFrame();
    pFrame->sensorType = ONI_SENSOR_PHASE;

    pFrame->width = GetStream()->GetXRes();
    pFrame->height = GetStream()->GetYRes();
    pFrame->extraLine = GetStream()->GetMetadataLine();
    pFrame->stride = GetStream()->GetXRes() * GetStream()->GetBytesPerPixel();

    pFrame->videoMode.fps = GetStream()->GetFPS();
    pFrame->videoMode.resolutionX = GetStream()->GetXRes();
    pFrame->videoMode.resolutionY = GetStream()->GetYRes();
    pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();

    /// Call base.
    XnFrameStreamProcessor::OnEndOfFrame(pHeader);

    XN_PROFILING_END_SECTION
}

void XnPhaseProcessor::OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS)
{
    XnFrameStreamProcessor::OnFrameReady(nFrameID, nFrameTS);
    m_pDevicePrivateData->pSensor->GetFPSCalculator()->MarkPhase(nFrameID, nFrameTS);
}
