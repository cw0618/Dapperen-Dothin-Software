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
#include "XnSensorPhaseStream.h"
#include "XnPhasePacked10Processor.h"
#include "XnPhaseUnpacked16Processor.h"


#define XN_SENSOR_PHASE_STREAM_EXTRA_LINE            (1)
#define XN_MASK_SENSOR_PHASE_STREAM                  "XnSensorPhaseStream"

XnSensorPhaseStream::XnSensorPhaseStream(const XnChar* streamName, XnSensorObjects* pObjects)
    : XnPhaseStream(streamName, FALSE, XN_DEVICE_SENSOR_MAX_PHASE)
    , m_helper(pObjects)
    , m_freqMode(XN_STREAM_PROPERTY_FREQUENCY_MODE, "FrequencyMode")
    , m_actualRead(XN_STREAM_PROPERTY_ACTUAL_READ_DATA, "ActualReadData", FALSE, streamName)
    , m_inputFormat(XN_STREAM_PROPERTY_INPUT_FORMAT, "InputFormat", XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_ALIGN_256_EXTRA_LINE_1, streamName)
    , m_firmwareMirror(0, "FirmwareMirror", TRUE, streamName)
    , m_croppingMode(XN_STREAM_PROPERTY_CROPPING_MODE, "CroppingMode", XN_CROPPING_MODE_NORMAL)
    , m_firmwareCropMode(0, "FirmwareCropMode", XN_FIRMWARE_CROPPING_MODE_DISABLED, streamName)
    , m_firmwareCropSizeX(0, "FirmwareCropSizeX", 0, streamName)
    , m_firmwareCropSizeY(0, "FirmwareCropSizeY", 0, streamName)
    , m_firmwareCropOffsetX(0, "FirmwareCropOffsetX", 0, streamName)
    , m_firmwareCropOffsetY(0, "FirmwareCropOffsetY", 0, streamName)
{
    m_freqMode.UpdateGetCallback(GetFrequencyModeCallback, this);
    m_freqMode.UpdateSetCallback(SetFrequencyModeCallback, this);
    m_actualRead.UpdateSetCallback(SetActualReadCallback, this);
    m_inputFormat.UpdateSetCallback(SetInputFormatCallback, this);
    m_croppingMode.UpdateSetCallback(SetCroppingModeCallback, this);
}

XnStatus XnSensorPhaseStream::Init()
{
    /// Init base.
    XnStatus nRetVal = XnPhaseStream::Init();
    XN_IS_STATUS_OK(nRetVal);

    /// Add properties.
    XN_VALIDATE_ADD_PROPERTIES(this, &m_inputFormat, &m_actualRead, &m_croppingMode, &m_freqMode);

    /// Init helper.
    nRetVal = m_helper.Init(this, this);
    XN_IS_STATUS_OK(nRetVal);

    /// Register supported modes.
    XnUInt32 nSupportedModes = m_helper.GetPrivateData()->FWInfo.phaseModes.GetSize();
    XnCmosPreset* pSupportedModes = m_helper.GetPrivateData()->FWInfo.phaseModes.GetData();
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

    int inputFormat = 0;
    nRetVal = xnOSReadIntFromINI(m_drvCfgFile, "Phase", "InputFormat", &inputFormat);
    XN_IS_STATUS_OK(nRetVal);
    switch (inputFormat)
    {
    case XN_IO_PHASE_FORMAT_MIPI_10_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_UNCOMPRESSED_16_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_ALIGN_256_EXTRA_LINE_1:
        nRetVal = m_inputFormat.UnsafeUpdateValue(inputFormat);
        XN_IS_STATUS_OK(nRetVal);
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_DEVICE_BAD_PARAM, XN_MASK_SENSOR_PHASE_STREAM, "Not supported Phase input format (%d)...", inputFormat);
    }

    XnInt32 resolution = 0;
    nRetVal = xnOSReadIntFromINI(m_drvCfgFile, "Phase", "Resolution", &resolution);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = SetResolution((XnResolutions)resolution);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = SetMetadataLine(XN_SENSOR_PHASE_STREAM_EXTRA_LINE);
    XN_IS_STATUS_OK(nRetVal);

    /// Set base properties default values.
    nRetVal = FPSProperty().UnsafeUpdateValue(XN_PHASE_STREAM_DEFAULT_FPS);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = ResolutionProperty().UnsafeUpdateValue(XN_PHASE_STREAM_DEFAULT_RESOLUTION);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = OutputFormatProperty().UnsafeUpdateValue(XN_PHASE_STREAM_DEFAULT_OUTPUT_FORMAT);
    XN_IS_STATUS_OK(nRetVal);

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::Free()
{
    m_helper.Free();
    XnPhaseStream::Free();

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::MapPropertiesToFirmware()
{
    XnStatus nRetVal = m_helper.MapFirmwareProperty(m_freqMode, GetFirmwareParams()->m_freqMode, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_inputFormat, GetFirmwareParams()->m_phaseFormat, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(FPSProperty(), GetFirmwareParams()->m_phaseFPS, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(ResolutionProperty(), GetFirmwareParams()->m_phaseResolution, FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareMirror, GetFirmwareParams()->m_phaseMirror, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareCropMode, GetFirmwareParams()->m_phaseCropMode, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareCropSizeX, GetFirmwareParams()->m_phaseCropSizeX, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareCropSizeY, GetFirmwareParams()->m_phaseCropSizeY, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareCropOffsetX, GetFirmwareParams()->m_phaseCropOffsetX, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.MapFirmwareProperty(m_firmwareCropOffsetY, GetFirmwareParams()->m_phaseCropOffsetY, TRUE);
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::ConfigureStreamImpl()
{
    xnUSBShutdownReadThread(GetHelper()->GetPrivateData()->pSpecificDepthUsb->pUsbConnection->UsbEp);

    XnStatus nRetVal = SetActualRead(TRUE);
    XN_IS_STATUS_OK(nRetVal);

    // nRetVal = m_helper.ConfigureFirmware(m_freqMode);
    // XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.ConfigureFirmware(m_inputFormat);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.ConfigureFirmware(FPSProperty());
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.ConfigureFirmware(ResolutionProperty());
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.ConfigureFirmware(m_firmwareMirror);
    XN_IS_STATUS_OK(nRetVal);

    /// CMOS.
    if (XN_RESOLUTION_SXGA != GetResolution())
    {
        nRetVal = m_helper.GetCmosInfo()->SetCmosConfig(XN_CMOS_TYPE_PHASE, GetResolution(), GetFPS());
        XN_IS_STATUS_OK(nRetVal);
    }

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::OpenStreamImpl()
{
    XnStatus nRetVal = GetFirmwareParams()->m_Stream1Mode.SetValue(XN_VIDEO_STREAM_PHASE);
    XN_IS_STATUS_OK(nRetVal);

    /// Cropping
    if (XN_FIRMWARE_CROPPING_MODE_DISABLED != m_firmwareCropMode.GetValue())
    {
        nRetVal = m_helper.ConfigureFirmware(m_firmwareCropSizeX);
        XN_IS_STATUS_OK(nRetVal);

        nRetVal = m_helper.ConfigureFirmware(m_firmwareCropSizeY);
        XN_IS_STATUS_OK(nRetVal);

        nRetVal = m_helper.ConfigureFirmware(m_firmwareCropOffsetX);
        XN_IS_STATUS_OK(nRetVal);

        nRetVal = m_helper.ConfigureFirmware(m_firmwareCropOffsetY);
        XN_IS_STATUS_OK(nRetVal);
    }

    nRetVal = FixFirmwareBug();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnPhaseStream::Open();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::CloseStreamImpl()
{
    XnStatus nRetVal = GetFirmwareParams()->m_Stream1Mode.SetValue(XN_VIDEO_STREAM_OFF);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = SetActualRead(FALSE);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnPhaseStream::Close();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::CreateDataProcessor(XnDataProcessor** ppProcessor)
{
    XN_RET_IF_NULL(ppProcessor, XN_STATUS_ERROR);

    XnFrameBufferManager* pBufferManager;
    XnStatus nRetVal = StartBufferManager(&pBufferManager);
    XN_IS_STATUS_OK(nRetVal);

    XnStreamProcessor* pNew = NULL;
    switch (m_inputFormat.GetValue())
    {
    case XN_IO_PHASE_FORMAT_UNCOMPRESSED_16_BIT_EXTRA_LINE_1:
        XN_VALIDATE_NEW_AND_INIT(pNew, XnPhaseUnpacked16Processor, this, &m_helper, pBufferManager);
        break;
    case XN_IO_PHASE_FORMAT_MIPI_10_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1:
    case XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_ALIGN_256_EXTRA_LINE_1:
        XN_VALIDATE_NEW_AND_INIT(pNew, XnPhasePacked10Processor, this, &m_helper, pBufferManager);
        break;
    default:
        return XN_STATUS_IO_INVALID_STREAM_PHASE_FORMAT;
    }

    *ppProcessor = pNew;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::Mirror(OniFrame* pFrame) const
{
    /// Only perform mirror if it's our job. If mirror is performed by FW, we don't need to do anything.
    if (m_firmwareMirror.GetValue())
        return XN_STATUS_OK;

    return XnPhaseStream::Mirror(pFrame);
}

XnStatus XnSensorPhaseStream::CalcRequiredSize(XnUInt32* pnRequiredSize) const
{
    XN_RET_IF_NULL(pnRequiredSize, XN_STATUS_ERROR);

    XnUInt32 imageBytes = GetXRes() * GetYRes() * GetBytesPerPixel();
    XnUInt32 extraBytes = GetXRes() * GetBytesPerPixel() * GetMetadataLine();
    *pnRequiredSize = imageBytes + extraBytes;

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::FixFirmwareBug()
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

void XnSensorPhaseStream::SetDriverConfig(char* path, int size)
{
    if (NULL == path)
        return;

    memset(m_drvCfgFile, 0, sizeof(m_drvCfgFile));
    memcpy(m_drvCfgFile, path, size);
}

XnStatus XnSensorPhaseStream::SetActualRead(XnBool bRead)
{
    XnStatus nRetVal = XN_STATUS_OK;
    if ((XnBool)m_actualRead.GetValue() != bRead)
    {
        if (bRead)
        {
            xnLogInfo(XN_MASK_SENSOR_PROTOCOL_PHASE, "Creating USB Phase read thread...");
            XnSpecificUsbDevice* pUSB = GetHelper()->GetPrivateData()->pSpecificDepthUsb;
            nRetVal = xnUSBInitReadThread(pUSB->pUsbConnection->UsbEp, pUSB->nChunkReadBytes, pUSB->nNumberOfBuffers, pUSB->nTimeout, XnDeviceSensorProtocolUsbEpCb, pUSB);
            XN_IS_STATUS_OK(nRetVal);
        }
        else
        {
            xnLogInfo(XN_MASK_SENSOR_PROTOCOL_PHASE, "Shutting down Phase read thread...");
            xnUSBShutdownReadThread(GetHelper()->GetPrivateData()->pSpecificDepthUsb->pUsbConnection->UsbEp);
        }

        nRetVal = m_actualRead.UnsafeUpdateValue(bRead);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensorPhaseStream::SetFPS(XnUInt32 nFPS)
{
    XnStatus nRetVal = m_helper.BeforeSettingFirmwareParam(FPSProperty(), (XnUInt16)nFPS);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnPhaseStream::SetFPS(nFPS);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingFirmwareParam(FPSProperty());
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetResolution(XnResolutions nResolution)
{
    XnStatus nRetVal = m_helper.BeforeSettingFirmwareParam(ResolutionProperty(), (XnUInt16)nResolution);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnPhaseStream::SetResolution(nResolution);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingFirmwareParam(ResolutionProperty());
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensorPhaseStream::SetOutputFormat(OniPixelFormat nOutputFormat)
{
    XnStatus nRetVal = XN_STATUS_ERROR;
    switch (nOutputFormat)
    {
    case ONI_PIXEL_FORMAT_GRAY16:
    case ONI_PIXEL_FORMAT_RGB888:
        nRetVal = DeviceMaxPixelProperty().UnsafeUpdateValue(XN_DEVICE_SENSOR_MAX_PHASE);
        XN_IS_STATUS_OK(nRetVal);
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_DEVICE_BAD_PARAM, XN_MASK_SENSOR_PHASE_STREAM, "Not supported Phase output format (%d)...", nOutputFormat);
    }

    nRetVal = m_helper.BeforeSettingDataProcessorProperty();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnPhaseStream::SetOutputFormat(nOutputFormat);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_helper.AfterSettingDataProcessorProperty();
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetInputFormat(XnIOPhaseFormats nInputFormat)
{
    XnStatus nRetVal = m_helper.SimpleSetFirmwareParam(m_inputFormat, (XnUInt16)nInputFormat);
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetMirror(XnBool bMirrored)
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
    nRetVal = XnPhaseStream::SetMirror(bMirrored);
    xnOSLeaveCriticalSection(GetLock());
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensorPhaseStream::CropImpl(OniFrame* pFrame, const OniCropping* pCropping)
{
    /// If firmware cropping is disabled then crop.
    if (XN_FIRMWARE_CROPPING_MODE_DISABLED == m_firmwareCropMode.GetValue())
    {
        XnStatus nRetVal = XnPhaseStream::CropImpl(pFrame, pCropping);
        XN_IS_STATUS_OK(nRetVal);
    }
    else if (IsMirrored())
    {
        /// Mirror is done in software and cropping in chip, so we crop the other side (see SetCroppingImpl()).
        pFrame->cropOriginX = GetXRes() - pFrame->cropOriginX - pFrame->width;
    }

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetCroppingImpl(const OniCropping* pCropping, XnCroppingMode mode)
{
    XnStatus nRetVal = ValidateCropping(pCropping);
    XN_IS_STATUS_OK(nRetVal);

    xnOSEnterCriticalSection(GetLock());
    if (m_helper.GetFirmwareVersion() > XN_SENSOR_FW_VER_3_0)
    {
        nRetVal = m_helper.StartFirmwareTransaction();
        if (XN_STATUS_OK != nRetVal)
        {
            xnOSLeaveCriticalSection(GetLock());
            return nRetVal;
        }

        /// Mirror is done by software (meaning AFTER cropping, which is bad). So we need to flip the cropping area
        /// to match requested area.
        XnUInt16 nXOffset = (XnUInt16)pCropping->originX;
        if (IsMirrored())
            nXOffset = (XnUInt16)(GetXRes() - pCropping->originX - pCropping->width);

        if (pCropping->enabled)
        {
            nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareCropSizeX, (XnUInt16)pCropping->width);
            if (XN_STATUS_OK == nRetVal)
                nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareCropSizeY, (XnUInt16)pCropping->height);

            if (XN_STATUS_OK == nRetVal)
                nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareCropOffsetX, (XnUInt16)nXOffset);

            if (XN_STATUS_OK == nRetVal)
                nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareCropOffsetY, (XnUInt16)pCropping->originY);
        }

        if (XN_STATUS_OK == nRetVal)
        {
            XnFirmwareCroppingMode firmwareMode = m_helper.GetFirmwareCroppingMode(mode, pCropping->enabled);
            nRetVal = m_helper.SimpleSetFirmwareParam(m_firmwareCropMode, (XnUInt16)firmwareMode);
        }

        if (XN_STATUS_OK != nRetVal)
        {
            m_helper.RollbackFirmwareTransaction();
            m_helper.UpdateFromFirmware(m_firmwareCropMode);
            m_helper.UpdateFromFirmware(m_firmwareCropOffsetX);
            m_helper.UpdateFromFirmware(m_firmwareCropOffsetY);
            m_helper.UpdateFromFirmware(m_firmwareCropSizeX);
            m_helper.UpdateFromFirmware(m_firmwareCropSizeY);
            xnOSLeaveCriticalSection(GetLock());
            return (nRetVal);
        }

        nRetVal = m_helper.CommitFirmwareTransactionAsBatch();
        if (XN_STATUS_OK != nRetVal)
        {
            m_helper.UpdateFromFirmware(m_firmwareCropMode);
            m_helper.UpdateFromFirmware(m_firmwareCropOffsetX);
            m_helper.UpdateFromFirmware(m_firmwareCropOffsetY);
            m_helper.UpdateFromFirmware(m_firmwareCropSizeX);
            m_helper.UpdateFromFirmware(m_firmwareCropSizeY);
            xnOSLeaveCriticalSection(GetLock());
            return (nRetVal);
        }
    }

    nRetVal = m_croppingMode.UnsafeUpdateValue(mode);
    XN_ASSERT(XN_STATUS_OK == nRetVal);

    nRetVal = XnSensorPhaseStream::SetCropping(pCropping);
    if (XN_STATUS_OK == nRetVal)
        nRetVal = FixFirmwareBug();

    xnOSLeaveCriticalSection(GetLock());
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetCropping(const OniCropping* pCropping)
{
    return SetCroppingImpl(pCropping, (XnCroppingMode)m_croppingMode.GetValue());
}

XnStatus XnSensorPhaseStream::OnMirrorChanged()
{
    /// If cropping is on, we need to flip it.
    OniCropping cropping = *GetCropping();
    if (cropping.enabled)
    {
        XnStatus nRetVal = SetCropping(&cropping);
        XN_IS_STATUS_OK(nRetVal);
    }

    return XN_STATUS_OK;
}

XnStatus XnSensorPhaseStream::SetCroppingMode(XnCroppingMode mode)
{
    switch (mode)
    {
    case XN_CROPPING_MODE_NORMAL:
    case XN_CROPPING_MODE_INCREASED_FPS:
    case XN_CROPPING_MODE_SOFTWARE_ONLY:
        break;
    default:
        XN_LOG_WARNING_RETURN(XN_STATUS_DEVICE_BAD_PARAM, XN_MASK_SENSOR_PHASE_STREAM, "Bad cropping mode (%d)...", mode);
    }

    return SetCroppingImpl(GetCropping(), mode);
}

XnStatus XnSensorPhaseStream::GetFrequencyMode(OniFrequencyMode* pMode)
{
    return XnHostProtocolGetParam(m_helper.GetPrivateData(), PARAM_FREQUENCY_MODE, *(XnUInt16*)pMode);
}

XnStatus XnSensorPhaseStream::SetFrequencyMode(const OniFrequencyMode mode)
{
    return m_helper.SimpleSetFirmwareParam(m_freqMode, (XnUInt16)mode);
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::OnMirrorChangedCallback(const XnProperty* pSender, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->OnMirrorChanged();
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::SetActualReadCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_ERROR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_ERROR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->SetActualRead(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::SetInputFormatCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->SetInputFormat(XnIOPhaseFormats(nValue));
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::SetCroppingModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->SetCroppingMode((XnCroppingMode)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::GetFrequencyModeCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
    XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->GetFrequencyMode((OniFrequencyMode*)pValue);
}

XnStatus XN_CALLBACK_TYPE XnSensorPhaseStream::SetFrequencyModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensorPhaseStream* pThis = (XnSensorPhaseStream*)pCookie;
    return pThis->SetFrequencyMode((OniFrequencyMode)nValue);
}
