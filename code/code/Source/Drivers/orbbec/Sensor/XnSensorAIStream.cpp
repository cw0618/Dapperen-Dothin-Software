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
#include <XnOS.h>
#include "XnCmosInfo.h"
#include <XnFormatsStatus.h>
#include "XnDeviceSensorInit.h"
#include "XnSensorAIStream.h"
#include "XnAIProcessor.h"


#define XN_AI_STREAM_DEFAULT_FPS           (30)
#define XN_AI_STREAM_DEFAULT_RESOLUTION    (XN_RESOLUTION_3840_2160)
#define XN_AI_STREAM_DEFAULT_OUTPUT_FORMAT (ONI_PIXEL_FORMAT_JOINT_2D)

XnSensorAIStream::XnSensorAIStream(const XnChar* StreamName, XnSensorObjects* pObjects)
    : XnAIStream(StreamName, FALSE, XN_DEVICE_SENSOR_MAX_AI)
    , m_helper(pObjects)
    , m_firmwareMirror(0, "FirmwareMirror", FALSE, StreamName)
    , m_actualRead(XN_STREAM_PROPERTY_ACTUAL_READ_DATA, "ActualReadData", FALSE)
    , m_inputFormat(XN_STREAM_PROPERTY_INPUT_FORMAT, "InputFormat", XN_IO_AI_FORMAT_JOINT_2D)
{
    xnOSMemSet(m_drvCfgFile, 0, XN_FILE_MAX_PATH);
    m_actualRead.UpdateSetCallback(SetActualReadCallback, this);
    m_inputFormat.UpdateSetCallback(SetInputFormatCallback, this);
}

XnStatus XnSensorAIStream::Init()
{
    /// Init base.
    XnStatus nRetVal = XnAIStream::Init();
    XN_IS_STATUS_OK(nRetVal);

    /// Add properties.
    XN_VALIDATE_ADD_PROPERTIES(this, &m_inputFormat, &m_actualRead);

    /// Init helper.
    nRetVal = m_helper.Init(this, this);
    XN_IS_STATUS_OK(nRetVal);

    /// Register supported modes.
    XnUInt32 nSupportedModes = m_helper.GetPrivateData()->FWInfo.AIModes.GetSize();
    XnCmosPreset* pSupportedModes = m_helper.GetPrivateData()->FWInfo.AIModes.GetData();
    nRetVal = AddSupportedModes(pSupportedModes, nSupportedModes);
    XN_IS_STATUS_OK(nRetVal);

    /// Data processor.
    nRetVal = m_helper.RegisterDataProcessorProperty(m_inputFormat);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = m_helper.RegisterDataProcessorProperty(ResolutionProperty());
    XN_IS_STATUS_OK(nRetVal);

    /// register for mirror
    XnCallbackHandle hCallbackDummy;
    nRetVal = IsMirroredProperty().OnChangeEvent().Register(OnMirrorChangedCallback, this, hCallbackDummy);
    XN_IS_STATUS_OK(nRetVal);

    /// Set base properties default values.
    nRetVal = FPSProperty().UnsafeUpdateValue(XN_AI_STREAM_DEFAULT_FPS);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = ResolutionProperty().UnsafeUpdateValue(XN_AI_STREAM_DEFAULT_RESOLUTION);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = OutputFormatProperty().UnsafeUpdateValue(XN_AI_STREAM_DEFAULT_OUTPUT_FORMAT);
    XN_IS_STATUS_OK(nRetVal);

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::Free()
{
    m_helper.Free();
    XnAIStream::Free();

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::MapPropertiesToFirmware()
{
    XnStatus nRetVal = m_helper.MapFirmwareProperty(m_inputFormat, GetFirmwareParams()->m_AIFormat, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(FPSProperty(), GetFirmwareParams()->m_AIFPS, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(ResolutionProperty(), GetFirmwareParams()->m_AIResolution, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareMirror, GetFirmwareParams()->m_AIMirror, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::ConfigureStreamImpl()
{
    xnUSBShutdownReadThread(GetHelper()->GetPrivateData()->pSpecificAIUsb->pUsbConnection->UsbEp);

    XnStatus nRetVal = SetActualRead(TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.ConfigureFirmware(m_inputFormat);
    XN_IS_STATUS_OK(nRetVal);

    /// No need to set these attributes, or keep the same properties as the Depth stream ???
    // nRetVal = m_helper.ConfigureFirmware(FPSProperty());
    // XN_IS_STATUS_OK(nRetVal);
    // nRetVal = m_helper.ConfigureFirmware(ResolutionProperty());
    // XN_IS_STATUS_OK(nRetVal);

    /// CMOS.
    if (XN_RESOLUTION_SXGA != GetResolution())
    {
        nRetVal = m_helper.GetCmosInfo()->SetCmosConfig(XN_CMOS_TYPE_AI, GetResolution(), GetFPS());
        XN_IS_STATUS_OK(nRetVal);
    }

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::OpenStreamImpl()
{
    XnStatus nRetVal = GetFirmwareParams()->m_Stream3Mode.SetValue(XN_VIDEO_STREAM_AI);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = FixFirmwareBug();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnAIStream::Open();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::CloseStreamImpl()
{
    XnStatus nRetVal = GetFirmwareParams()->m_Stream3Mode.SetValue(XN_VIDEO_STREAM_OFF);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = SetActualRead(FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnAIStream::Close();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::CreateDataProcessor(XnDataProcessor** ppProcessor)
{
    XN_RET_IF_NULL(ppProcessor, XN_STATUS_ERROR);

    XnFrameBufferManager* pBufferManager;
    XnStatus nRetVal = StartBufferManager(&pBufferManager);
    XN_IS_STATUS_OK(nRetVal);

    XnStreamProcessor* pNew = NULL;
    switch (m_inputFormat.GetValue())
    {
    case XN_IO_AI_FORMAT_JOINT_2D:
    case XN_IO_AI_FORMAT_JOINT_3D:
    case XN_IO_AI_FORMAT_BODY_MASK:
    case XN_IO_AI_FORMAT_FLOOR_INFO:
    case XN_IO_AI_FORMAT_BODY_SHAPE:
    case XN_IO_AI_FORMAT_PHASE:
    case XN_IO_AI_FORMAT_DEPTH_IR:
        XN_VALIDATE_NEW_AND_INIT(pNew, XnAIProcessor, this, &m_helper, pBufferManager);
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_IO_INVALID_STREAM_AI_FORMAT, XN_MASK_SENSOR_PROTOCOL_AI, "Not supported AI input format: %d", m_inputFormat.GetValue());
    }

    *ppProcessor = pNew;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::Mirror(OniFrame* pFrame) const
{
    /// Only perform mirror if it's our job. If mirror is performed by FW, we don't need to do anything.
    if (m_firmwareMirror.GetValue())
        return XN_STATUS_OK;

    return XnAIStream::Mirror(pFrame);
}

XnStatus XnSensorAIStream::CalcRequiredSize(XnUInt32* pnRequiredSize) const
{
    XN_RET_IF_NULL(pnRequiredSize, XN_STATUS_ERROR);
    *pnRequiredSize = GetXRes() * GetYRes() * GetBytesPerPixel() + GetXRes() * GetBytesPerPixel() * GetMetadataLine();
    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::FixFirmwareBug()
{
    /// Firmware bug ugly workaround: in v5.1, IR 1.3 would not turn off decimation, so image is
    /// corrupted. we need to turn it off ourselves. The problem is that the firmware does not
    /// even provide a way to do so, so we need to directly change the register...
    /// the bug only happens when cropping is off.
    if (m_helper.GetFirmware()->GetInfo()->nFWVer == XN_SENSOR_FW_VER_5_1 && GetResolution() == XN_RESOLUTION_SXGA && !GetCropping()->enabled)
    {
        XnStatus nRetVal = XnHostProtocolWriteAHB(m_helper.GetPrivateData(), 0x2a003c00, 0x3ff0000, 0xffffffff);
        XN_IS_STATUS_OK(nRetVal);
    }

    return XN_STATUS_OK;
}

void XnSensorAIStream::SetDriverConfig(char* path, int size)
{
    if (NULL == path)
        return;

    int len = size > XN_FILE_MAX_PATH ? XN_FILE_MAX_PATH : size;
    xnOSStrNCopy(m_drvCfgFile, path, len, XN_FILE_MAX_PATH);
}

XnStatus XnSensorAIStream::SetActualRead(XnBool bRead)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if ((XnBool)m_actualRead.GetValue() != bRead)
    {
        if (bRead)
        {
            xnLogInfo(XN_MASK_SENSOR_PROTOCOL_AI, "Creating USB AI read thread...");
            XnSpecificUsbDevice* pUSB = GetHelper()->GetPrivateData()->pSpecificAIUsb;
            nRetVal = xnUSBInitReadThread(pUSB->pUsbConnection->UsbEp, pUSB->nChunkReadBytes, pUSB->nNumberOfBuffers, pUSB->nTimeout, XnDeviceSensorProtocolUsbEpCb, pUSB);
            XN_IS_STATUS_OK(nRetVal);
        }
        else
        {
            xnLogInfo(XN_MASK_SENSOR_PROTOCOL_AI, "Shutting down AI read thread...");
            xnUSBShutdownReadThread(GetHelper()->GetPrivateData()->pSpecificAIUsb->pUsbConnection->UsbEp);
        }

        nRetVal = m_actualRead.UnsafeUpdateValue(bRead);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensorAIStream::SetFPS(XnUInt32 nFPS)
{
    XnStatus nRetVal = m_helper.BeforeSettingFirmwareParam(FPSProperty(), (XnUInt16)nFPS);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnAIStream::SetFPS(nFPS);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingFirmwareParam(FPSProperty());
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::SetResolution(XnResolutions nResolution)
{
    XnStatus nRetVal = m_helper.BeforeSettingFirmwareParam(ResolutionProperty(), (XnUInt16)nResolution);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnAIStream::SetResolution(nResolution);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingFirmwareParam(ResolutionProperty());
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensorAIStream::SetOutputFormat(OniPixelFormat nOutputFormat)
{
    XnIOAIFormats inputFormat = XN_IO_AI_FORMAT_JOINT_2D;
    switch (nOutputFormat)
    {
    case ONI_PIXEL_FORMAT_JOINT_2D:
        inputFormat = XN_IO_AI_FORMAT_JOINT_2D;
        break;
    case ONI_PIXEL_FORMAT_JOINT_3D:
        inputFormat = XN_IO_AI_FORMAT_JOINT_3D;
        break;
    case ONI_PIXEL_FORMAT_BODY_MASK:
        inputFormat = XN_IO_AI_FORMAT_BODY_MASK;
        break;
    case ONI_PIXEL_FORMAT_FLOOR_INFO:
        inputFormat = XN_IO_AI_FORMAT_FLOOR_INFO;
        break;
    case ONI_PIXEL_FORMAT_BODY_SHAPE:
        inputFormat = XN_IO_AI_FORMAT_BODY_SHAPE;
        break;
    case ONI_PIXEL_FORMAT_PHASE:
        inputFormat = XN_IO_AI_FORMAT_PHASE;
        break;
    case ONI_PIXEL_FORMAT_DEPTH_IR:
        inputFormat = XN_IO_AI_FORMAT_DEPTH_IR;
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_DEVICE_BAD_PARAM, XN_MASK_SENSOR_PROTOCOL_AI, "Not supported AI output format: %d", nOutputFormat);
    }

    /// Note: for AI stream the output format is the same as input format.
    XnStatus nRetVal = SetInputFormat(inputFormat);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = DeviceMaxPixelProperty().UnsafeUpdateValue(XN_DEVICE_SENSOR_MAX_AI);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.BeforeSettingDataProcessorProperty();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnAIStream::SetOutputFormat(nOutputFormat);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingDataProcessorProperty();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::SetInputFormat(XnIOAIFormats nInputFormat)
{
    if (nInputFormat == m_inputFormat.GetValue())
        return XN_STATUS_OK;

    switch (nInputFormat)
    {
    case XN_IO_AI_FORMAT_JOINT_2D:
    case XN_IO_AI_FORMAT_JOINT_3D:
    case XN_IO_AI_FORMAT_BODY_MASK:
    case XN_IO_AI_FORMAT_FLOOR_INFO:
    case XN_IO_AI_FORMAT_BODY_SHAPE:
    case XN_IO_AI_FORMAT_PHASE:
    case XN_IO_AI_FORMAT_DEPTH_IR:
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_DEVICE_BAD_PARAM, XN_MASK_SENSOR_PROTOCOL_AI, "Not supported AI input format: %d", nInputFormat);
    }

    XnStatus nRetVal = m_helper.SimpleSetFirmwareParam(m_inputFormat, (XnUInt16)nInputFormat);
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorAIStream::SetMirror(XnBool bMirrored)
{
    /// Set firmware mirror.
    XnBool bFirmwareMirror = (bMirrored && m_helper.GetFirmwareVersion() >= XN_SENSOR_FW_VER_5_0);

    xnOSEnterCriticalSection(GetLock());
    XnStatus nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareMirror, (XnUInt16)bFirmwareMirror);
    if (XN_STATUS_OK != nRetVal)
    {
        xnOSLeaveCriticalSection(GetLock());
        return nRetVal;
    }

    /// Update property.
    nRetVal = XnAIStream::SetMirror(bMirrored);
    xnOSLeaveCriticalSection(GetLock());
    XN_IS_STATUS_OK(nRetVal);

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensorAIStream::OnMirrorChangedCallback(const XnProperty* pSender, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensorAIStream::SetActualReadCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorAIStream* pThis = (XnSensorAIStream*)pCookie;
    return pThis->SetActualRead(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensorAIStream::SetInputFormatCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorAIStream* pThis = (XnSensorAIStream*)pCookie;
    return pThis->SetInputFormat(XnIOAIFormats(nValue));
}
