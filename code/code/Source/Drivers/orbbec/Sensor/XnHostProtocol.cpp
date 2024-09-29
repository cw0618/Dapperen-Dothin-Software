/*****************************************************************************
*                                                                           *
*  OpenNI 2.x Alpha                                                         *
*  Copyright (C) 2012 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  Licensed under the Apache License, Version 2.0 (the "License");          *
*  you may not use this file except in compliance with the License.         *
*  You may obtain a copy of the License at                                  *
*                                                                           *
*      http://www.apache.org/licenses/LICENSE-2.0                           *
*                                                                           *
*  Unless required by applicable law or agreed to in writing, software      *
*  distributed under the License is distributed on an "AS IS" BASIS,        *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
*  See the License for the specific language governing permissions and      *
*  limitations under the License.                                           *
*                                                                           *
*****************************************************************************/
#include "XnDeviceSensorProtocol.h"
#include "XnHostProtocol.h"
#include <math.h>
#include <XnLog.h>
#include "XnSensorDepthStream.h"
#include "XnSensor.h"
#include <XnPsVersion.h>

// Control Protocol
#include "XnParams.h"

#define XN_RECEIVE_USB_DATA_TIMEOUT 20000
#define XN_USB_HOST_PROTOCOL_TIMEOUT 5000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_KEEP_ALIVE 5000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_GETVERSION 5000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_SETPARAM 5000

#define XN_USB_HOST_PROTOCOL_TIMEOUT_UPLOAD 180000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS 180000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_BIST 300000
#define XN_USB_HOST_PROTOCOL_TIMEOUT_EMITTER_DATA 60000

#define XN_USB_HOST_PROTOCOL_FILE_UPLOAD_PRE_DELAY 250
#define XN_USB_HOST_PROTOCOL_FILE_UPLOAD_POST_DELAY 0

#define XN_LOG_TEXT_MESSAGE_V1_2	0x1000
#define XN_LOG_OVERFLOW_V1_2		0x1001

#define XN_LOG_TEXT_MESSAGE_V3_0	0x1200
#define XN_LOG_OVERFLOW_V3_0		0x1201

#define XN_LOG_TEXT_MESSAGE_V4_0	0x1200
#define XN_LOG_OVERFLOW_V4_0		0x1201

#define XN_LOG_TEXT_MESSAGE_V5_0	0x5400
#define XN_LOG_OVERFLOW_V5_0		0x5401

#define XN_USB_HOST_PROTOCOL_SEND_RETRIES	5
#define XN_HOST_PROTOCOL_NOT_READY_RETRIES	3

#define XN_PROTOCOL_MAX_PACKET_SIZE_V5_0	512
#define XN_PROTOCOL_MAX_PACKET_SIZE_V0_17	64

#define MAX_PACKET_SIZE 512
#define MAX_RECEIVE_PACKET_SIZE 900
#define EATCH_PACKET_SIZE 32

#define MAX_PACKET_BULK_SIZE (256 * 1024)
#define EACH_PACKET_BULK_SIZE (MAX_PACKET_BULK_SIZE - 36)


inline XnInt32 CompareVersion(XnUInt8 nMajor1, XnUInt8 nMinor1, XnUInt16 nBuild1, XnUInt8 nMajor2, XnUInt8 nMinor2, XnUInt16 nBuild2)
{
    XnInt32 nResult = nMajor1 - nMajor2;

    if (nResult == 0)
    {
        nResult = nMinor1 - nMinor2;
    }

    if (nResult == 0)
    {
        nResult = nBuild1 - nBuild2;
    }

    return (nResult);
}

static XnFWVer GetFWVersion(XnUInt8 nMajor, XnUInt8 nMinor, XnUInt16 nBuild)
{
    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_8;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 7, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_7;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 6, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_6;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 5, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_5;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 4, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_4;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_3;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 2, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_2;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 1, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_1;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 5, 0, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_5_0;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 4, 0, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_4_0;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 3, 0, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_3_0;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 1, 2, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_1_2;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 1, 2, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_1_2;
    }
    else if (CompareVersion(nMajor, nMinor, nBuild, 1, 1, 0) >= 0)
    {
        return XN_SENSOR_FW_VER_1_1;
    }
    else
    {
        return XN_SENSOR_FW_VER_0_17;
    }
}

XnStatus XnHostProtocolUpdateSupportedImageModes(XnDevicePrivateData* pDevicePrivateData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (pDevicePrivateData->FWInfo.bGetPresetsSupported)
    {
        // ask the firmware
        const XnUInt32 nAllocSize = 100;
        XnUInt32 nCount = nAllocSize;
        XnCmosPreset aSupportedModes[nAllocSize] = { 0 };
        nRetVal = XnHostProtocolGetCmosPresets(pDevicePrivateData, XN_CMOS_TYPE_IMAGE, aSupportedModes, nCount);
        if (XN_STATUS_OK == nRetVal)
        {
            if (nCount == 0)
            {
                xnLogError(XN_MASK_DEVICE_SENSOR, "Device does not support any image mode!");
                return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
            }

            nRetVal = pDevicePrivateData->FWInfo.imageModes.SetData(aSupportedModes, nCount);
            XN_IS_STATUS_OK(nRetVal);
        }
        else
        {
            XnCmosPreset presets[] =
            {
                { XN_IO_IMAGE_FORMAT_H264, XN_RESOLUTION_WVGA, 30 },
                { XN_IO_IMAGE_FORMAT_H264, XN_RESOLUTION_720P, 30 },
                { XN_IO_IMAGE_FORMAT_H264, XN_RESOLUTION_1920_1080, 30 },
                { XN_IO_IMAGE_FORMAT_H264, XN_RESOLUTION_3840_2160, 30 },
            };
            nRetVal = pDevicePrivateData->FWInfo.imageModes.SetData(presets, sizeof(presets) / sizeof(presets[0]));
            XN_IS_STATUS_OK(nRetVal);
        }
    }
    else
    {
        // old firmware. Just use what we know
        switch (pDevicePrivateData->pSensor->GetCurrentUsbInterface())
        {
        case XN_SENSOR_USB_INTERFACE_BULK_ENDPOINTS:
            nRetVal = pDevicePrivateData->FWInfo.imageModes.SetData(pDevicePrivateData->FWInfo._imageBulkModes.GetData(), pDevicePrivateData->FWInfo._imageBulkModes.GetSize());
            XN_IS_STATUS_OK(nRetVal);
            break;
        case XN_SENSOR_USB_INTERFACE_ISO_ENDPOINTS:
            nRetVal = pDevicePrivateData->FWInfo.imageModes.SetData(pDevicePrivateData->FWInfo._imageIsoModes.GetData(), pDevicePrivateData->FWInfo._imageIsoModes.GetSize());
            XN_IS_STATUS_OK(nRetVal);
            break;
        default:
            xnLogError(XN_MASK_DEVICE_SENSOR, "Unknown interface in old firmware (%d)", pDevicePrivateData->pSensor->GetCurrentUsbInterface());
            XN_ASSERT(FALSE);
            return XN_STATUS_ERROR;
        }
    }

    // added high-res image modes (UXGA for 5.2, SXGA for 5.3 and newer)
    XnCmosPreset imageHighResBayerMode = { XN_IO_IMAGE_FORMAT_COMPRESSED_MJPEG, XN_RESOLUTION_1280_960, 30 };

    nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageHighResBayerMode);
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageHighResBayerMode);
    XN_IS_STATUS_OK(nRetVal);


    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolUpdateSupportedDepthModes(XnDevicePrivateData* pDevicePrivateData)
{
    XnStatus nRetVal = XN_STATUS_OK;
    if (pDevicePrivateData->FWInfo.bGetPresetsSupported)
    {
        // ask the firmware
        const XnUInt32 nAllocSize = 100;
        XnUInt32 nCount = nAllocSize;
        XnCmosPreset aSupportedModes[nAllocSize] = { 0 };
        nRetVal = XnHostProtocolGetCmosPresets(pDevicePrivateData, XN_CMOS_TYPE_DEPTH, aSupportedModes, nCount);
        XN_IS_STATUS_OK(nRetVal);

        if (nCount == 0)
        {
            xnLogError(XN_MASK_DEVICE_SENSOR, "Device does not support any depth mode!");
            return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
        }

        nRetVal = pDevicePrivateData->FWInfo.depthModes.SetData(aSupportedModes, nCount);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolUpdateSupportedIRModes(XnDevicePrivateData* pDevicePrivateData)
{
    XnStatus nRetVal = XN_STATUS_OK;
    if (pDevicePrivateData->FWInfo.bGetPresetsSupported)
    {
        // ask the firmware
        const XnUInt32 nAllocSize = 100;
        XnUInt32 nCount = nAllocSize;
        XnCmosPreset aSupportedModes[nAllocSize] = { 0 };
        nRetVal = XnHostProtocolGetCmosPresets(pDevicePrivateData, XN_CMOS_TYPE_IR, aSupportedModes, nCount);
        XN_IS_STATUS_OK(nRetVal);

        for (XnUInt32 i = 0; i < nCount; i++)
        {
            aSupportedModes[i].nFormat = 0;
        }

        if (nCount == 0)
        {
            xnLogError(XN_MASK_DEVICE_SENSOR, "Device does not support any ir mode!");
            return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
        }

        nRetVal = pDevicePrivateData->FWInfo.irModes.SetData(aSupportedModes, nCount);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolUpdateSupportedPhaseModes(XnDevicePrivateData* pDevicePrivateData)
{
    if (pDevicePrivateData->FWInfo.bGetPresetsSupported)
    {
        /// Ask the firmware.
        const XnUInt32 nAllocSize = 100;
        XnUInt32 nCount = nAllocSize;
        XnCmosPreset aSupportedModes[nAllocSize] = { 0 };
        XnStatus nRetVal = XnHostProtocolGetCmosPresets(pDevicePrivateData, XN_CMOS_TYPE_PHASE, aSupportedModes, nCount);
        if (XN_STATUS_OK == nRetVal)
        {
            if (nCount == 0)
            {
                xnLogError(XN_MASK_DEVICE_SENSOR, "Device does not support any phase mode!");
                return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
            }

            nRetVal = pDevicePrivateData->FWInfo.phaseModes.SetData(aSupportedModes, nCount);
            XN_IS_STATUS_OK(nRetVal);
        }
        else
        {
            XnCmosPreset presets[] =
            {
                { XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1, XN_RESOLUTION_VGA, 30 },
                { XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1, XN_RESOLUTION_960_240, 30 },
                { XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1, XN_RESOLUTION_1920_480, 30 },
                { XN_IO_PHASE_FORMAT_COMPRESSED_10_BIT_EXTRA_LINE_1, XN_RESOLUTION_1280_960, 30 },
            };
            nRetVal = pDevicePrivateData->FWInfo.phaseModes.SetData(presets, sizeof(presets) / sizeof(presets[0]));
            XN_IS_STATUS_OK(nRetVal);
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolUpdateSupportedAIModes(XnDevicePrivateData* pDevicePrivateData)
{
    if (pDevicePrivateData->FWInfo.bGetPresetsSupported)
    {
        /// Ask the firmware.
        const XnUInt32 nAllocSize = 100;
        XnUInt32 nCount = nAllocSize;
        XnCmosPreset aSupportedModes[nAllocSize] = { 0 };
        XnStatus nRetVal = XnHostProtocolGetCmosPresets(pDevicePrivateData, XN_CMOS_TYPE_AI, aSupportedModes, nCount);
        if (XN_STATUS_OK == nRetVal)
        {
            if (nCount == 0)
            {
                xnLogError(XN_MASK_DEVICE_SENSOR, "Device does not support any AI mode!");
                return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
            }

            nRetVal = pDevicePrivateData->FWInfo.AIModes.SetData(aSupportedModes, nCount);
            XN_IS_STATUS_OK(nRetVal);
        } 
        else
        {
            XnCmosPreset presets[] =
            {
                { XN_IO_AI_FORMAT_JOINT_2D, XN_RESOLUTION_3840_2160, 30 },
                { XN_IO_AI_FORMAT_JOINT_3D, XN_RESOLUTION_3840_2160, 30 },
                { XN_IO_AI_FORMAT_BODY_MASK, XN_RESOLUTION_3840_2160, 30 },
                { XN_IO_AI_FORMAT_FLOOR_INFO, XN_RESOLUTION_3840_2160, 30 },
                { XN_IO_AI_FORMAT_BODY_SHAPE, XN_RESOLUTION_3840_2160, 30 },
                { XN_IO_AI_FORMAT_PHASE, XN_RESOLUTION_3840_2160, 30 },
            };
            nRetVal = pDevicePrivateData->FWInfo.AIModes.SetData(presets, sizeof(presets) / sizeof(presets[0]));
            XN_IS_STATUS_OK(nRetVal);
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolInitFWParams(XnDevicePrivateData* pDevicePrivateData, XnUInt8 nMajor, XnUInt8 nMinor, XnUInt16 nBuild, XnHostProtocolUsbCore usb, XnBool bGuessed)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // we start with oldest settings (FW version 0.17), and change them for newer versions
    pDevicePrivateData->FWInfo.nFWMagic = XN_FW_MAGIC_25;
    pDevicePrivateData->FWInfo.nHostMagic = XN_HOST_MAGIC_25;
    pDevicePrivateData->FWInfo.nProtocolHeaderSize = sizeof(XnHostProtocolHeaderV25);
    pDevicePrivateData->FWInfo.nProtocolMaxPacketSize = XN_PROTOCOL_MAX_PACKET_SIZE_V0_17;
    pDevicePrivateData->FWInfo.bAudioSupported = FALSE;
    pDevicePrivateData->FWInfo.bPhaseSupported = FALSE;
    pDevicePrivateData->FWInfo.bAISupported = FALSE;
    pDevicePrivateData->FWInfo.bMirrorSupported = FALSE;
    pDevicePrivateData->FWInfo.bGetPresetsSupported = FALSE;
    pDevicePrivateData->FWInfo.bDeviceInfoSupported = FALSE;
    pDevicePrivateData->FWInfo.bImageAdjustmentsSupported = FALSE;

    pDevicePrivateData->FWInfo.nOpcodeGetVersion = OPCODE_V017_GET_VERSION;
    pDevicePrivateData->FWInfo.nOpcodeKeepAlive = OPCODE_V017_KEEP_ALIVE;
    pDevicePrivateData->FWInfo.nOpcodeGetParam = OPCODE_V017_GET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeSetParam = OPCODE_V017_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetFixedParams = OPCODE_V017_GET_FIXED_PARAMS;
    pDevicePrivateData->FWInfo.nOpcodeGetMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeAlgorithmParams = OPCODE_V017_ALGORITM_PARAMS;
    pDevicePrivateData->FWInfo.nOpcodeReset = OPCODE_V017_RESET;
    pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetSerialNumber = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetCfgProductNumber = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetCMOSReg = OPCODE_V017_GET_CMOS_REGISTER;
    pDevicePrivateData->FWInfo.nOpcodeSetCMOSReg = OPCODE_V017_SET_CMOS_REGISTER;
    pDevicePrivateData->FWInfo.nOpcodeWriteI2C = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeReadI2C = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeReadAHB = OPCODE_V017_READ_AHB;
    pDevicePrivateData->FWInfo.nOpcodeWriteAHB = OPCODE_V017_WRITE_AHB;
    pDevicePrivateData->FWInfo.nOpcodeGetPlatformString = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetUsbCore = OPCODE_GET_USB_CORE_TYPE;
    pDevicePrivateData->FWInfo.nOpcodeSetLedState = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeEnableEmitter = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeEnableIrflood = OPCODE_INVALID;

    //irgain
    pDevicePrivateData->FWInfo.nOpcodeSetIrGain = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetIrGain = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetIrExp = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetIrExp = OPCODE_INVALID;

    //ldp enable
    pDevicePrivateData->FWInfo.nOpcodeSetLdpEnable = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetLdpEnable = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnable = OPCODE_INVALID;

    //Auto ae
    pDevicePrivateData->FWInfo.nOpcodeSetAeEnable = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetAeEnable = OPCODE_INVALID;

	pDevicePrivateData->FWInfo.nOpcodeSetHdrModeEnable = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetHdrModeEnable = OPCODE_INVALID;
    //dabai
    pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1 = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1 = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1 = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeChangeSensor = OPCODE_INVALID;

    //ecc public key
    pDevicePrivateData->FWInfo.nOpcodeSetPublicKey = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetPublicKey = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeRandomString = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeRSKey = OPCODE_INVALID;

    //laser secure
    pDevicePrivateData->FWInfo.nOpcodeIsSupportLaserSecure = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetLaserSecure = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserSecure = OPCODE_INVALID;

    //laser current
    pDevicePrivateData->FWInfo.nOpcodeSetLaserCurrent = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserCurrent = OPCODE_INVALID;

    //soft reset
    pDevicePrivateData->FWInfo.nOpcodeSoftReset = OPCODE_INVALID;
    //switch dual camera left and right ir
    pDevicePrivateData->FWInfo.nOpcodeSetSwitchIr = OPCODE_INVALID;

    //get DisparityCoeff
    pDevicePrivateData->FWInfo.nOpcodeGetDisparityCoeff = OPCODE_INVALID;

    //rgb ae mode
    pDevicePrivateData->FWInfo.nOpcodeSetRgbAeMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetRgbAeMode = OPCODE_INVALID;

    //support sub cmd OB_OPCODE_READ_SUBCMD_PARAM
    pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams = OPCODE_INVALID;

    //support sub cmd OB_OPCODE_OPTIOM_READ_SUBCMD_PARAM
    pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams = OPCODE_INVALID;

    //support sub cmd PD
    pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd = OPCODE_INVALID;

    //support cmd bootloader protection status
    pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsRead = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsWrite = OPCODE_INVALID;

    //TOF
    pDevicePrivateData->FWInfo.nOpcodeToFSensorRead = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite = OPCODE_INVALID;

    //Motor
    pDevicePrivateData->FWInfo.nOpcodeMotorRead = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeMotorWrite = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeSetD2CResolution = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetD2CResolution = OPCODE_INVALID;

    //laser time
    pDevicePrivateData->FWInfo.nOpcodeSetLaserTime = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserTime = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeSetPostFilterThreshold = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetPostFilterThreshold = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeSetDepthIrMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetDepthIrMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetTecEnable = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeSetSubTractBGMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetSubTractBGMode = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeGetLog = OPCODE_V017_GET_LOG;
    pDevicePrivateData->FWInfo.nOpcodeTakeSnapshot = OPCODE_V017_TAKE_SNAPSHOT;
    pDevicePrivateData->FWInfo.nOpcodeInitFileUpload = OPCODE_V017_INIT_FILE_UPLOAD;
    pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload = OPCODE_V017_WRITE_FILE_UPLOAD;
    pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload = OPCODE_V017_FINISH_FILE_UPLOAD;
    pDevicePrivateData->FWInfo.nOpcodeDownloadFile = OPCODE_V017_DOWNLOAD_FILE;
    pDevicePrivateData->FWInfo.nOpcodeDeleteFile = OPCODE_V017_DELETE_FILE;
    pDevicePrivateData->FWInfo.nOpcodeGetFlashMap = OPCODE_V017_GET_FLASH_MAP;
    pDevicePrivateData->FWInfo.nOpcodeGetFileList = OPCODE_V017_GET_FILE_LIST;
    pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeExecuteFile = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeReadFlash = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeBIST = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetGMCParams = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetCPUStats = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeCalibrateTec = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetTecData = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetEmitterData = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nOpcodeFileTransfer = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeFileTransferPrepare = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeFileTransferFinish = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetPlatformVersion = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetPlatformSDKVersion = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeStreamSetQuery = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetTOFFreqMode = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSetTOFFreqMode = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFilterLevel = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFilterLevel = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorIntegrationTime = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorIntegrationTime = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorGain = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorGain = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorLaserInterference = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorLaserInterference = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorWorkingMode = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorWorkingMode = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFrequency = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFrequency = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDutyCycle = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDutyCycle = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDriverICReg = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDriverICReg = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorSensorReg = OPCODE_INVALID;
	pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorSensorReg = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeStartService = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeGetSensorID = OPCODE_INVALID;
    pDevicePrivateData->FWInfo.nOpcodeSendCommand = OPCODE_INVALID;

    pDevicePrivateData->FWInfo.nLogStringType = XN_LOG_TEXT_MESSAGE_V1_2;
    pDevicePrivateData->FWInfo.nLogOverflowType = XN_LOG_OVERFLOW_V1_2;

    pDevicePrivateData->FWInfo.nUSBDelayReceive = 100;
    pDevicePrivateData->FWInfo.nUSBDelayExecutePreSend = 1;
    pDevicePrivateData->FWInfo.nUSBDelayExecutePostSend = 10;
    pDevicePrivateData->FWInfo.nUSBDelaySoftReset = 800;
    pDevicePrivateData->FWInfo.nUSBDelaySetParamFlicker = 3000;
    pDevicePrivateData->FWInfo.nUSBDelaySetParamStream0Mode = 1;
    pDevicePrivateData->FWInfo.nUSBDelaySetParamStream1Mode = 300;
    pDevicePrivateData->FWInfo.nUSBDelaySetParamStream2Mode = 1;
    pDevicePrivateData->FWInfo.nUSBDelaySetParamStream3Mode = 1;

    pDevicePrivateData->FWInfo.bGetImageCmosTypeSupported = FALSE;
    pDevicePrivateData->FWInfo.bImageSupported = TRUE;
    pDevicePrivateData->FWInfo.bIncreasedFpsCropSupported = FALSE;
    pDevicePrivateData->FWInfo.bHasFilesystemLock = FALSE;

    pDevicePrivateData->FWInfo.nISOLowDepthAlternativeInterface = (XnUInt8)(-1);

    // depth cmos modes
    pDevicePrivateData->FWInfo.depthModes.Clear();
    XnCmosPreset depthModes[] =
    {
        { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_VGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_VGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_VGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_VGA, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_640_400, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_1280_800, 5 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_1280_800, 30 },

        { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_320_200, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_320_200, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_320_200, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_320_200, 30 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_10_BIT, XN_RESOLUTION_800_1280, 5 },
        { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_10_BIT, XN_RESOLUTION_400_640, 30 },
    };
    nRetVal = pDevicePrivateData->FWInfo.depthModes.AddLast(depthModes, sizeof(depthModes) / sizeof(depthModes[0]));
    XN_IS_STATUS_OK(nRetVal);

    // image cmos modes
    pDevicePrivateData->FWInfo._imageBulkModes.Clear();
    pDevicePrivateData->FWInfo._imageIsoModes.Clear();

    XnCmosPreset imageCommonModes[] =
    {
        { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_VGA, 30 },
    };
    nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageCommonModes, sizeof(imageCommonModes) / sizeof(imageCommonModes[0]));
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageCommonModes, sizeof(imageCommonModes) / sizeof(imageCommonModes[0]));
    XN_IS_STATUS_OK(nRetVal);

    XnCmosPreset imageIsoModes[] =
    {
        { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_QVGA, 30 },
        { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_QVGA, 60 },
        { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_VGA, 30 },
    };
    nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageIsoModes, sizeof(imageIsoModes) / sizeof(imageIsoModes[0]));
    XN_IS_STATUS_OK(nRetVal);

    // IR cmos modes
    pDevicePrivateData->FWInfo.irModes.Clear();
    XnCmosPreset irModes[] =
    {
        { 0, XN_RESOLUTION_QVGA, 30 },
        { 0, XN_RESOLUTION_QVGA, 60 },
        { 0, XN_RESOLUTION_VGA, 30 },
        { 0, XN_RESOLUTION_640_400, 30 },
        { 0, XN_RESOLUTION_1280_800, 5 },
        { 0, XN_RESOLUTION_1280_800, 30 },
        { 0, XN_RESOLUTION_800_1280, 5 },
        { 0, XN_RESOLUTION_400_640, 30 },
    };
    nRetVal = pDevicePrivateData->FWInfo.irModes.AddLast(irModes, sizeof(irModes) / sizeof(irModes[0]));
    XN_IS_STATUS_OK(nRetVal);

    if (CompareVersion(nMajor, nMinor, nBuild, 1, 1, 0) >= 0)
    {
        // opcodes were changed
        pDevicePrivateData->FWInfo.nOpcodeGetVersion = OPCODE_V110_GET_VERSION;
        pDevicePrivateData->FWInfo.nOpcodeKeepAlive = OPCODE_V110_KEEP_ALIVE;
        pDevicePrivateData->FWInfo.nOpcodeGetParam = OPCODE_V110_GET_PARAM;
        pDevicePrivateData->FWInfo.nOpcodeSetParam = OPCODE_V110_SET_PARAM;
        pDevicePrivateData->FWInfo.nOpcodeGetFixedParams = OPCODE_V110_GET_FIXED_PARAMS;
        pDevicePrivateData->FWInfo.nOpcodeGetMode = OPCODE_V110_GET_MODE;
        pDevicePrivateData->FWInfo.nOpcodeSetMode = OPCODE_V110_SET_MODE;
        pDevicePrivateData->FWInfo.nOpcodeAlgorithmParams = OPCODE_V110_ALGORITHM_PARAMS;
        pDevicePrivateData->FWInfo.nOpcodeReset = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCMOSReg = OPCODE_V110_GET_CMOS_REGISTER;
        pDevicePrivateData->FWInfo.nOpcodeSetCMOSReg = OPCODE_V110_SET_CMOS_REGISTER;
        pDevicePrivateData->FWInfo.nOpcodeWriteI2C = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeReadI2C = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeReadAHB = OPCODE_V110_READ_AHB;
        pDevicePrivateData->FWInfo.nOpcodeWriteAHB = OPCODE_V110_WRITE_AHB;
        pDevicePrivateData->FWInfo.nOpcodeGetLog = OPCODE_V110_GET_LOG;
        pDevicePrivateData->FWInfo.nOpcodeTakeSnapshot = OPCODE_V110_TAKE_SNAPSHOT;
        pDevicePrivateData->FWInfo.nOpcodeInitFileUpload = OPCODE_V110_INIT_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload = OPCODE_V110_WRITE_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload = OPCODE_V110_FINISH_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeDownloadFile = OPCODE_V110_DOWNLOAD_FILE;
        pDevicePrivateData->FWInfo.nOpcodeDeleteFile = OPCODE_V110_DELETE_FILE;
        pDevicePrivateData->FWInfo.nOpcodeGetFlashMap = OPCODE_V110_GET_FLASH_MAP;
        pDevicePrivateData->FWInfo.nOpcodeGetFileList = OPCODE_V110_GET_FILE_LIST;
        pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute = OPCODE_V110_SET_FILE_ATTRIBUTES;
        pDevicePrivateData->FWInfo.nOpcodeExecuteFile = OPCODE_V110_EXECUTE_FILE;
        pDevicePrivateData->FWInfo.nOpcodeReadFlash = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeBIST = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeSetGMCParams = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCPUStats = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateTec = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetTecData = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetEmitterData = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault = OPCODE_INVALID;

        // log system changed
        pDevicePrivateData->FWInfo.nLogStringType = XN_LOG_TEXT_MESSAGE_V1_2;
        pDevicePrivateData->FWInfo.nLogOverflowType = XN_LOG_OVERFLOW_V1_2;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 1, 2, 0) >= 0)
    {
        // protocol header was changed
        pDevicePrivateData->FWInfo.nFWMagic = XN_FW_MAGIC_26;
        pDevicePrivateData->FWInfo.nHostMagic = XN_HOST_MAGIC_26;
        pDevicePrivateData->FWInfo.nProtocolHeaderSize = sizeof(XnHostProtocolHeaderV26);
        pDevicePrivateData->FWInfo.nProtocolHeaderBulkSize = sizeof(XnHostProtocolBulkHeaderV26);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 3, 0, 0) >= 0)
    {
        // audio support!
        pDevicePrivateData->FWInfo.bAudioSupported = TRUE;

        // opcodes were changed
        pDevicePrivateData->FWInfo.nOpcodeGetVersion = OPCODE_GET_VERSION;
        pDevicePrivateData->FWInfo.nOpcodeKeepAlive = OPCODE_KEEP_ALIVE;
        pDevicePrivateData->FWInfo.nOpcodeGetParam = OPCODE_GET_PARAM;
        pDevicePrivateData->FWInfo.nOpcodeSetParam = OPCODE_SET_PARAM;
        pDevicePrivateData->FWInfo.nOpcodeGetFixedParams = OPCODE_GET_FIXED_PARAMS;
        pDevicePrivateData->FWInfo.nOpcodeGetMode = OPCODE_GET_MODE;
        pDevicePrivateData->FWInfo.nOpcodeSetMode = OPCODE_SET_MODE;
        pDevicePrivateData->FWInfo.nOpcodeAlgorithmParams = OPCODE_ALGORITM_PARAMS;
        pDevicePrivateData->FWInfo.nOpcodeReset = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCMOSReg = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeSetCMOSReg = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeWriteI2C = OPCODE_I2C_WRITE;
        pDevicePrivateData->FWInfo.nOpcodeReadI2C = OPCODE_I2C_READ;
        pDevicePrivateData->FWInfo.nOpcodeReadAHB = OPCODE_READ_AHB;
        pDevicePrivateData->FWInfo.nOpcodeWriteAHB = OPCODE_WRITE_AHB;
        pDevicePrivateData->FWInfo.nOpcodeGetLog = OPCODE_GET_LOG;
        pDevicePrivateData->FWInfo.nOpcodeTakeSnapshot = OPCODE_TAKE_SNAPSHOT;
        pDevicePrivateData->FWInfo.nOpcodeInitFileUpload = OPCODE_INIT_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload = OPCODE_WRITE_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload = OPCODE_FINISH_FILE_UPLOAD;
        pDevicePrivateData->FWInfo.nOpcodeDownloadFile = OPCODE_DOWNLOAD_FILE;
        pDevicePrivateData->FWInfo.nOpcodeDeleteFile = OPCODE_DELETE_FILE;
        pDevicePrivateData->FWInfo.nOpcodeGetFlashMap = OPCODE_GET_FLASH_MAP;
        pDevicePrivateData->FWInfo.nOpcodeGetFileList = OPCODE_GET_FILE_LIST;
        pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute = OPCODE_SET_FILE_ATTRIBUTES;
        pDevicePrivateData->FWInfo.nOpcodeExecuteFile = OPCODE_EXECUTE_FILE;
        pDevicePrivateData->FWInfo.nOpcodeReadFlash = OPCODE_READ_FLASH;
        pDevicePrivateData->FWInfo.nOpcodeBIST = OPCODE_V300_BIST;
        pDevicePrivateData->FWInfo.nOpcodeSetGMCParams = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetCPUStats = OPCODE_GET_CPU_STATS;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateTec = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetTecData = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeGetEmitterData = OPCODE_INVALID;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault = OPCODE_INVALID;

        // log system changed
        pDevicePrivateData->FWInfo.nLogStringType = XN_LOG_TEXT_MESSAGE_V3_0;
        pDevicePrivateData->FWInfo.nLogOverflowType = XN_LOG_OVERFLOW_V3_0;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 4, 0, 0) >= 0)
    {
        // audio removed...
        pDevicePrivateData->FWInfo.bAudioSupported = FALSE;

        // opcodes added
        pDevicePrivateData->FWInfo.nOpcodeBIST = OPCODE_BIST;
        pDevicePrivateData->FWInfo.nOpcodeSetGMCParams = OPCODE_SET_GMC_PARAMS;

        // log system changed
        pDevicePrivateData->FWInfo.nLogStringType = XN_LOG_TEXT_MESSAGE_V4_0;
        pDevicePrivateData->FWInfo.nLogOverflowType = XN_LOG_OVERFLOW_V4_0;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 0, 0) >= 0)
    {
        // max packet size changed
        pDevicePrivateData->FWInfo.nProtocolMaxPacketSize = XN_PROTOCOL_MAX_PACKET_SIZE_V5_0;
        // audio is back on
        pDevicePrivateData->FWInfo.bAudioSupported = TRUE;
        // mirror supported
        pDevicePrivateData->FWInfo.bMirrorSupported = TRUE;

        // opcodes changes
        pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking = OPCODE_SET_CMOS_BLANKING;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateTec = OPCODE_CALIBRATE_TEC;
        pDevicePrivateData->FWInfo.nOpcodeGetTecData = OPCODE_GET_TEC_DATA;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter = OPCODE_CALIBRATE_EMITTER;
        pDevicePrivateData->FWInfo.nOpcodeGetEmitterData = OPCODE_GET_EMITTER_DATA;
        pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault = OPCODE_CALIBRATE_PROJECTOR_FAULT;

        // log system changed
        pDevicePrivateData->FWInfo.nLogStringType = XN_LOG_TEXT_MESSAGE_V5_0;
        pDevicePrivateData->FWInfo.nLogOverflowType = XN_LOG_OVERFLOW_V5_0;

        // ISO endpoints interface was added
        pDevicePrivateData->FWInfo.nISOAlternativeInterface = 0;
        pDevicePrivateData->FWInfo.nBulkAlternativeInterface = 1;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 1, 0) >= 0)
    {
        // added high-res IR
        XnCmosPreset irHighResMode[] = {
            { 0, XN_RESOLUTION_SXGA, 30 },
            { 0, XN_RESOLUTION_640_400, 30 },
            { 0, XN_RESOLUTION_1280_960, 30 },
            { 0, XN_RESOLUTION_1280_800, 5 },
            { 0, XN_RESOLUTION_1280_800, 30 },
            { 0, XN_RESOLUTION_720P, 30 },
            { 0, XN_RESOLUTION_320_200, 30 },
            { 0, XN_RESOLUTION_800_1280, 5 },
            { 0, XN_RESOLUTION_400_640, 30 },
        };
        nRetVal = pDevicePrivateData->FWInfo.irModes.AddLast(irHighResMode, sizeof(irHighResMode) / sizeof(irHighResMode[0]));
        XN_IS_STATUS_OK(nRetVal);

        // opcode added
        pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking = OPCODE_GET_CMOS_BLANKING;

        // add SXGA depth modes
        XnCmosPreset aSXGAmodes[] =
        {
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_SXGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_SXGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_SXGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_SXGA, 30 },
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_1280_960, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_1280_960, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_1280_960, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_1280_960, 5 },

            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_640_400, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_640_400, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_640_400, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_640_400, 30 },

            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_1280_800, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_1280_800, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_1280_800, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_1280_800, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_800_1280, 5 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_400_640, 30 },
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_1280_800, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_1280_800, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_1280_800, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_1280_800, 30 },

        };
        nRetVal = pDevicePrivateData->FWInfo.depthModes.AddLast(aSXGAmodes, sizeof(aSXGAmodes) / sizeof(aSXGAmodes[0]));
        XN_IS_STATUS_OK(nRetVal)

    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 2, 0) >= 0 && CompareVersion(nMajor, nMinor, nBuild, 5, 6, 0) < 0)
    {
        // 25 FPS modes were added
        XnCmosPreset depthModes25FPS[] =
        {
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_VGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_VGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_VGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_VGA, 25 },
        };
        nRetVal = pDevicePrivateData->FWInfo.depthModes.AddLast(depthModes25FPS, sizeof(depthModes25FPS) / sizeof(depthModes25FPS[0]));
        XN_IS_STATUS_OK(nRetVal);

        XnCmosPreset imageModes25FpsCommon[] =
        {
            { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_VGA, 25 },
        };
        nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageModes25FpsCommon, sizeof(imageModes25FpsCommon) / sizeof(imageModes25FpsCommon[0]));
        XN_IS_STATUS_OK(nRetVal);
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageModes25FpsCommon, sizeof(imageModes25FpsCommon) / sizeof(imageModes25FpsCommon[0]));
        XN_IS_STATUS_OK(nRetVal);

        XnCmosPreset imageModes25FpsIso[] =
        {
            { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_QVGA, 25 },
            { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_VGA, 25 },
        };
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageModes25FpsIso, sizeof(imageModes25FpsIso) / sizeof(imageModes25FpsIso[0]));
        XN_IS_STATUS_OK(nRetVal);

        XnCmosPreset irModes25Fps[] =
        {
            { 0, XN_RESOLUTION_QVGA, 25 },
            { 0, XN_RESOLUTION_VGA, 25 },
        };
        nRetVal = pDevicePrivateData->FWInfo.irModes.AddLast(irModes25Fps, sizeof(irModes25Fps) / sizeof(irModes25Fps[0]));
        XN_IS_STATUS_OK(nRetVal);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 1, 0) >= 0)
    {
        // added high-res image modes (UXGA for 5.2, SXGA for 5.3 and newer)
        XnCmosPreset imageHighResBayerMode = { XN_IO_IMAGE_FORMAT_COMPRESSED_MJPEG, XN_RESOLUTION_1280_960, 30 };

        nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageHighResBayerMode);
        XN_IS_STATUS_OK(nRetVal);
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageHighResBayerMode);
        XN_IS_STATUS_OK(nRetVal);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 2, 0) >= 0)
    {
        // added high-res image modes (UXGA for 5.2, SXGA for 5.3 and newer)
        XnCmosPreset imageHighResBayerMode = { XN_IO_IMAGE_FORMAT_BAYER, XN_RESOLUTION_UXGA, 30 };
        if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 0) >= 0)
        {
            imageHighResBayerMode.nResolution = XN_RESOLUTION_SXGA;
        }
        nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageHighResBayerMode);
        XN_IS_STATUS_OK(nRetVal);
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageHighResBayerMode);
        XN_IS_STATUS_OK(nRetVal);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 15) == 0)
    {
        pDevicePrivateData->FWInfo.nUSBDelaySetParamFlicker = 300;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 16) >= 0 &&
        !pDevicePrivateData->pSensor->IsLowBandwidth())
    {
        if (usb == XN_USB_CORE_JANGO)
        {
            pDevicePrivateData->FWInfo.nUSBDelayReceive = 1;
            pDevicePrivateData->FWInfo.nUSBDelayExecutePreSend = 0;
            pDevicePrivateData->FWInfo.nUSBDelayExecutePostSend = 0;
        }

        pDevicePrivateData->FWInfo.nUSBDelaySoftReset = 1;
        pDevicePrivateData->FWInfo.nUSBDelaySetParamFlicker = 1;
        pDevicePrivateData->FWInfo.nUSBDelaySetParamStream0Mode = 1;
        pDevicePrivateData->FWInfo.nUSBDelaySetParamStream1Mode = 1;
        pDevicePrivateData->FWInfo.nUSBDelaySetParamStream2Mode = 1;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 25) >= 0)
    {
        pDevicePrivateData->FWInfo.bDeviceInfoSupported = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 28) >= 0)
    {
        // YUV is also supported in high-res
        XnCmosPreset imageHighResYuvMode = { XN_IO_IMAGE_FORMAT_YUV422, XN_RESOLUTION_SXGA, 30 };
        nRetVal = pDevicePrivateData->FWInfo._imageBulkModes.AddLast(imageHighResYuvMode);
        XN_IS_STATUS_OK(nRetVal);
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageHighResYuvMode);
        XN_IS_STATUS_OK(nRetVal);

        XnCmosPreset imageHighResYuvModeIso = { XN_IO_IMAGE_FORMAT_UNCOMPRESSED_YUV422, XN_RESOLUTION_SXGA, 30 };
        nRetVal = pDevicePrivateData->FWInfo._imageIsoModes.AddLast(imageHighResYuvModeIso);
        XN_IS_STATUS_OK(nRetVal);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 29) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets = OPCODE_GET_CMOS_PRESETS;
        pDevicePrivateData->FWInfo.bGetPresetsSupported = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 3, 31) >= 0 && CompareVersion(nMajor, nMinor, nBuild, 5, 4, 0) < 0)
    {
        // file system lock was also added in 5.3.31 (a maintenance release), but it's not in newer versions (5.4 and above)
        pDevicePrivateData->FWInfo.bHasFilesystemLock = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 4, 0) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber = OPCODE_GET_SERIAL_NUMBER;
        pDevicePrivateData->FWInfo.nOpcodeSetSerialNumber = OPCODE_SET_SERIAL_NUMBER;
        pDevicePrivateData->FWInfo.nOpcodeGetCfgProductNumber = OB_OPCODE_GET_CFG_PRODUCT_NUMBER;
        pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC = OPCODE_GET_FAST_CONVERGENCE_TEC;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 5, 0) >= 0)
    {
        // only difference is the interfaces order
        pDevicePrivateData->FWInfo.nBulkAlternativeInterface = 0;
        pDevicePrivateData->FWInfo.nISOAlternativeInterface = 1;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 6, 0) >= 0)
    {
        // audio is no longer supported - switched to UAC
        pDevicePrivateData->FWInfo.bAudioSupported = FALSE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 6, 2) >= 0)
    {
        // add QQVGA depth modes
        XnCmosPreset aQQmodes[] =
        {
            { XN_IO_DEPTH_FORMAT_COMPRESSED_PS, XN_RESOLUTION_QQVGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_11_BIT, XN_RESOLUTION_QQVGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_12_BIT, XN_RESOLUTION_QQVGA, 30 },
            { XN_IO_DEPTH_FORMAT_UNCOMPRESSED_16_BIT, XN_RESOLUTION_QQVGA, 30 },
        };
        nRetVal = pDevicePrivateData->FWInfo.depthModes.AddLast(aQQmodes, sizeof(aQQmodes) / sizeof(aQQmodes[0]));
        XN_IS_STATUS_OK(nRetVal);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 6, 9) >= 0)
    {
        pDevicePrivateData->FWInfo.bGetImageCmosTypeSupported = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 7, 0) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeGetPlatformString = OPCODE_GET_PLATFORM_STRING;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 7, 2) >= 0)
    {
        pDevicePrivateData->FWInfo.bIncreasedFpsCropSupported = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 0) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeSetLedState = OPCODE_SET_LED_STATE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 2) >= 0)
    {
        pDevicePrivateData->FWInfo.bHasFilesystemLock = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 9) >= 0)
    {
        pDevicePrivateData->FWInfo.bImageAdjustmentsSupported = TRUE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 15) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeEnableEmitter = OPCODE_ENABLE_EMITTER;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 15) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeEnableIrflood = OPCODE_ENABLE_IRFLOOD;
    }

    //irgain
    pDevicePrivateData->FWInfo.nOpcodeSetIrGain = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetIrGain = OB_OPCODE_GET_PARAM;

    //ldp enable
    pDevicePrivateData->FWInfo.nOpcodeSetLdpEnable = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetLdpEnable = OB_OPCODE_GET_PARAM;

    pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnable = OB_OPCODE_GET_PARAM;

    //Auto ae
    pDevicePrivateData->FWInfo.nOpcodeSetAeEnable = OB_OPCODE_SET_AE_ENABLE;
    pDevicePrivateData->FWInfo.nOpcodeGetAeEnable = OB_OPCODE_GET_AE_ENABLE;

	pDevicePrivateData->FWInfo.nOpcodeSetHdrModeEnable = OB_OPCODE_SET_HDRMODE_ENABLE;
	pDevicePrivateData->FWInfo.nOpcodeGetHdrModeEnable = OB_OPCODE_GET_HDRMODE_ENABLE;
    pDevicePrivateData->FWInfo.nOpcodeSetMipiTestEnable = OB_OPCODE_SET_MIPI_TEST_ENABLE;
    pDevicePrivateData->FWInfo.nOpcodeGetMipiTestEnable = OB_OPCODE_GET_MIPI_TEST_ENABLE;

    pDevicePrivateData->FWInfo.nOpcodeI2CReadFlash = OB_OPCODE_I2C_READ_FLASH_MIPI;

    pDevicePrivateData->FWInfo.nOpcodeGetIRSensorModel = OB_OPCODE_GET_IR_MODEl;
    pDevicePrivateData->FWInfo.nOpcodeGetRgbSensorModel = OB_OPCODE_GET_RGB_MODEl;

    //dabai ldp enable
    pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1 = OB_OPCODE_SET_LDP;
    pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1 = OB_OPCODE_GET_LDP;
    pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1 = OB_OPCODE_GET_EMITTER;

    pDevicePrivateData->FWInfo.nOpcodeSetIrExp = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetIrExp = OB_OPCODE_GET_PARAM;

    pDevicePrivateData->FWInfo.nOpcodeChangeSensor = OB_OPCODE_CHANGE_SENSOR;

    pDevicePrivateData->FWInfo.nOpcodeSetPublicKey = OB_OPCODE_SET_PUBLIC_KEY;
    pDevicePrivateData->FWInfo.nOpcodeGetPublicKey = OB_OPCODE_GET_PUBLIC_KEY;

    pDevicePrivateData->FWInfo.nOpcodeRandomString = OB_OPCODE_GET_RANDOM_STRING;
    pDevicePrivateData->FWInfo.nOpcodeRSKey = OB_OPCODE_SET_VERIFY_RS;

    //laser secure
    pDevicePrivateData->FWInfo.nOpcodeIsSupportLaserSecure = OB_OPCODE_GET_IS_SUPPORT_LASER_SECURE;
    pDevicePrivateData->FWInfo.nOpcodeSetLaserSecure = OB_OPCODE_SET_ENABLE_LASER_SECURE;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserSecure = OB_OPCODE_GET_ENABLE_LASER_SECURE;

    //laser current
    pDevicePrivateData->FWInfo.nOpcodeSetLaserCurrent = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserCurrent = OB_OPCODE_GET_PARAM;

    pDevicePrivateData->FWInfo.nOpcodeSoftReset = OB_OPCODE_SOFT_RESET;
    //swith dual camera left and right ir
    pDevicePrivateData->FWInfo.nOpcodeSetSwitchIr = OB_OPCODE_SET_SWITCH_IR;

    //get DisparityCoeff
    pDevicePrivateData->FWInfo.nOpcodeGetDisparityCoeff = OB_OPCODE_GET_DUAL_PARAM;

    //get/set rgb ae mode
    pDevicePrivateData->FWInfo.nOpcodeSetRgbAeMode = CMD_RGB_SET_AEMODE;
    pDevicePrivateData->FWInfo.nOpcodeGetRgbAeMode = CMD_RGB_GET_AEMODE;

    //support sub cmd OB_OPCODE_READ_SUBCMD_PARAM
    pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams = OB_OPCODE_READ_SUBCMD_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams = OB_OPCODE_WRITE_SUBCMD_PARAM;

    pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams = OB_OPCODE_OPTIM_READ_SUBCMD;
    pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams = OB_OPCODE_OPTIM_WRITE_SUBCMD;

    pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd = OB_OPCODE_GET_PD;
    pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd = OB_OPCODE_SET_PD;

    pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsRead = OB_OPCODE_BOOTLOADER_PTS_GET;
    pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsWrite = OB_OPCODE_BOOTLOADER_PTS_SET;

    pDevicePrivateData->FWInfo.nOpcodeToFSensorRead = OB_OPCODE_TOF_SENSOR_GET;
    pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite = OB_OPCODE_TOF_SENSOR_SET;

    pDevicePrivateData->FWInfo.nOpcodeMotorRead = OB_OPCODE_GET_MOTOR_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeMotorWrite = OB_OPCODE_SET_MOTOR_PARAM;


    pDevicePrivateData->FWInfo.nOpcodeSetD2CResolution = OB_OPCODE_SET_D2C_RESOLUTION;
    pDevicePrivateData->FWInfo.nOpcodeGetD2CResolution = OB_OPCODE_GET_D2C_RESOLUTION;

    pDevicePrivateData->FWInfo.nOpcodeSetLaserTime = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetLaserTime = OB_OPCODE_GET_PARAM;

    pDevicePrivateData->FWInfo.nOpcodeSetPostFilterThreshold = OB_OPCODE_SET_PARAM;
    pDevicePrivateData->FWInfo.nOpcodeGetPostFilterThreshold = OB_OPCODE_GET_PARAM;

    //set depthir mode
    pDevicePrivateData->FWInfo.nOpcodeSetDepthIrMode = OB_OPCODE_SET_DEPTH_NIR_MODE;
    pDevicePrivateData->FWInfo.nOpcodeGetDepthIrMode = OB_OPCODE_GET_DEPTH_NIR_MODE;

    pDevicePrivateData->FWInfo.nOpcodeSetTecEnable = CMD_TEC_ENABLE;

    //subtract backgroud mode
    pDevicePrivateData->FWInfo.nOpcodeSetSubTractBGMode = OB_OPCODE_SET_SUBTRACT_BG_MODE;
    pDevicePrivateData->FWInfo.nOpcodeGetSubTractBGMode = OB_OPCODE_GET_SUBTRACT_BG_MODE;

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 16) >= 0)
    {
        pDevicePrivateData->FWInfo.nISOLowDepthAlternativeInterface = 2;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 9, 0) >= 0)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Sensor version %d.%d.%x is newer than latest known. Trying to use 5.8 protocol...", nMajor, nMinor, nBuild);
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 24) >= 0)
    {
        pDevicePrivateData->FWInfo.nOpcodeGetSensorID = OPCODE_GET_SENSOR_ID;
        pDevicePrivateData->FWInfo.nOpcodeSendCommand = OPCODE_SEND_COMMAND;
        pDevicePrivateData->FWInfo.nOpcodeQueryTimestamp = OPCODE_QUERY_TIMESTAMP;
        pDevicePrivateData->FWInfo.nOpcodeStreamSetQuery = OPCODE_STREAM_SET_QUERY;
        pDevicePrivateData->FWInfo.nOpcodeGetTOFFreqMode = OPCODE_GET_TOF_FREQ_MODE;
        pDevicePrivateData->FWInfo.nOpcodeSetTOFFreqMode = OPCODE_SET_TOF_FREQ_MODE;
        pDevicePrivateData->FWInfo.nOpcodeStartService = OPCODE_START_SERVICE;
        pDevicePrivateData->FWInfo.nOpcodeGetPlatformVersion = OPCODE_GET_PLATFORM_VERSION;
        pDevicePrivateData->FWInfo.nOpcodeGetPlatformSDKVersion = OPCODE_GET_PLATFORM_SDK_VERSION;
        pDevicePrivateData->FWInfo.nOpcodeFileTransfer = OPCODE_GENERAL_FILE_TRANSFERRING;
        pDevicePrivateData->FWInfo.nOpcodeFileTransferFinish = OPCODE_GENERAL_FILE_TRANSFER_FINISH;
        pDevicePrivateData->FWInfo.nOpcodeFileTransferPrepare = OPCODE_GENERAL_FILE_TRANSFER_PREPARE;
    }

    if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 24) >= 0)
    {
        nRetVal = XnHostProtocolGetStreamSet(pDevicePrivateData);
        XN_IS_STATUS_OK(nRetVal);
    }

	if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 24) >= 0)
	{
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFilterLevel = OB_OPCODE_GET_TOF_SENSOR_FILTER_LEVEL;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFilterLevel = OB_OPCODE_SET_TOF_SENSOR_FILTER_LEVEL;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorIntegrationTime = OB_OPCODE_GET_TOF_SENSOR_INTEGRATION_TIME;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorIntegrationTime = OB_OPCODE_SET_TOF_SENSOR_INTEGRATION_TIME;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorGain = OB_OPCODE_GET_TOF_SENSOR_GAIN;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorGain = OB_OPCODE_SET_TOF_SENSOR_GAIN;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorLaserInterference = OB_OPCODE_GET_LASER_INTERFERENCE_ENABLE;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorLaserInterference = OB_OPCODE_SET_LASER_INTERFERENCE_ENABLE;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorWorkingMode = OB_OPCODE_GET_WORKING_MODE;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorWorkingMode = OB_OPCODE_SET_WORKING_MODE;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFrequency = OB_OPCODE_GET_FREQUENCY;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFrequency = OB_OPCODE_SET_FREQUENCY;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDutyCycle = OB_OPCODE_GET_DUTY_CYCLE;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDutyCycle = OB_OPCODE_SET_DUTY_CYCLE;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDriverICReg = OB_OPCODE_GET_DRIVER_IC_REG;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDriverICReg = OB_OPCODE_SET_DRIVER_IC_REG;
		pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorSensorReg = OB_OPCODE_GET_SENSOR_REG;
		pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorSensorReg = OB_OPCODE_SET_SENSOR_REG;
	}
    // If FW is already known, update image modes
    if (!bGuessed)
    {
        nRetVal = XnHostProtocolUpdateSupportedImageModes(pDevicePrivateData);
        XN_IS_STATUS_OK(nRetVal);

        if (CompareVersion(nMajor, nMinor, nBuild, 5, 8, 23) >= 0)
        {
            nRetVal = XnHostProtocolUpdateSupportedDepthModes(pDevicePrivateData);
            XN_IS_STATUS_OK(nRetVal);

            nRetVal = XnHostProtocolUpdateSupportedIRModes(pDevicePrivateData);
            XN_IS_STATUS_OK(nRetVal);
        }

        if (pDevicePrivateData->FWInfo.bPhaseSupported)
        {
            nRetVal = XnHostProtocolUpdateSupportedPhaseModes(pDevicePrivateData);
            XN_IS_STATUS_OK(nRetVal);
        }

        if (pDevicePrivateData->FWInfo.bAISupported)
        {
            nRetVal = XnHostProtocolUpdateSupportedAIModes(pDevicePrivateData);
            XN_IS_STATUS_OK(nRetVal);
        }
    }

    pDevicePrivateData->FWInfo.nCurrMode = XN_MODE_PS;
    pDevicePrivateData->FWInfo.nFWVer = GetFWVersion(nMajor, nMinor, nBuild);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolInitHeader(const XnDevicePrivateData* pDevicePrivateData, void* pBuffer, XnUInt32 nSize, XnUInt16 nOpcode)
{
    static XnUInt16 nId = 0;

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_2)
    {
        XnHostProtocolHeaderV26* pHeader = (XnHostProtocolHeaderV26*)pBuffer;
        pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->FWInfo.nHostMagic);
        pHeader->nSize = XN_PREPARE_VAR16_IN_BUFFER(XnUInt16(nSize / sizeof(XnUInt16)));
        pHeader->nOpcode = XN_PREPARE_VAR16_IN_BUFFER(nOpcode);
        pHeader->nId = XN_PREPARE_VAR16_IN_BUFFER(nId++);
    }
    else
    {
        XnHostProtocolHeaderV25* pHeader = (XnHostProtocolHeaderV25*)pBuffer;
        pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->FWInfo.nHostMagic);
        pHeader->nSize = XN_PREPARE_VAR16_IN_BUFFER(XnUInt16(nSize / sizeof(XnUInt16)));
        pHeader->nOpcode = XN_PREPARE_VAR16_IN_BUFFER(nOpcode);
        pHeader->nId = XN_PREPARE_VAR16_IN_BUFFER(nId++);
        pHeader->nCRC16 = XN_PREPARE_VAR16_IN_BUFFER(0);
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolInitBulkHeader(const XnDevicePrivateData* pDevicePrivateData, void* pBuffer, XnUInt32 nSize, XnUInt16 nOpcode)
{
    static XnUInt16 nId = 0;
    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_2)
    {
        XnHostProtocolBulkHeaderV26* pHeader = (XnHostProtocolBulkHeaderV26*)pBuffer;
        pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->FWInfo.nHostMagic);
        pHeader->nSize = XN_PREPARE_VAR32_IN_BUFFER(XnUInt32(nSize / sizeof(XnUInt16)));
        pHeader->nOpcode = XN_PREPARE_VAR16_IN_BUFFER(nOpcode);
        pHeader->nId = XN_PREPARE_VAR16_IN_BUFFER(nId++);
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolUSBSend(const XnDevicePrivateData* pDevicePrivateData,
    XnUChar* pBuffer, XnUInt16 nSize, XnUInt32 nTimeOut, XnBool bForceBulk)
{
    XnStatus nRetVal = XN_STATUS_OK;

    const XnUsbControlConnection* pCtrlConnection = &pDevicePrivateData->SensorHandle.ControlConnection;

    XnUInt32 nCounter = XN_USB_HOST_PROTOCOL_SEND_RETRIES;
    while (nCounter-- != 0)
    {
        if (pCtrlConnection->bIsBulk || bForceBulk)
            nRetVal = xnUSBWriteEndPoint(pCtrlConnection->ControlOutConnectionEp, pBuffer, nSize, nTimeOut);
        else
        {
            nRetVal = xnUSBSendControl(pDevicePrivateData->SensorHandle.USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0, 0, 0, pBuffer, nSize, nTimeOut);
        }

        if (nRetVal != XN_STATUS_USB_TRANSFER_TIMEOUT && nRetVal != XN_STATUS_USB_TRANSFER_STALL)
            break;

        xnOSSleep(100);
    }

    return nRetVal;
}

XnStatus XnHostProtocolUSBReceive(const XnDevicePrivateData* pDevicePrivateData,
    XnUChar* pBuffer, XnUInt nSize, XnUInt32& nRead, XnUInt32 nTimeOut, XnBool bForceBulk, XnUInt32 nFailTimeout)
{
    XnStatus nRetVal;
    XnUInt64 nMaxTime;
    XnUInt64 nCurrTime;

    const XnUsbControlConnection* pCtrlConnection = &pDevicePrivateData->SensorHandle.ControlConnection;

    xnOSGetHighResTimeStamp(&nMaxTime);
    nMaxTime += (nTimeOut * 1000);

    for (;;)
    {
        xnOSGetHighResTimeStamp(&nCurrTime);
        if (nCurrTime > nMaxTime)
        {
            return (XN_STATUS_USB_TRANSFER_TIMEOUT);
        }

        if (pCtrlConnection->bIsBulk || bForceBulk)
            nRetVal = xnUSBReadEndPoint(pCtrlConnection->ControlInConnectionEp, pBuffer, nSize, &nRead, nTimeOut);
        else
            nRetVal = xnUSBReceiveControl(pDevicePrivateData->SensorHandle.USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0, 0, 0, pBuffer, nSize, &nRead, nTimeOut);

        if (nRetVal != XN_STATUS_USB_TRANSFER_TIMEOUT && nRetVal != XN_STATUS_USB_TRANSFER_STALL && nRetVal != XN_STATUS_USB_NOT_ENOUGH_DATA)
        {
            break;
        }

        if (nFailTimeout != 0)
        {
            XnUInt64 nNow;
            XnUInt64 nNow2;
            xnOSGetHighResTimeStamp(&nNow);
            xnOSGetHighResTimeStamp(&nNow2);
            while (nNow2 - nNow < nFailTimeout)
            {
                xnOSGetHighResTimeStamp(&nNow2);
            }
        }
        else
        {
            xnOSSleep(pDevicePrivateData->FWInfo.nUSBDelayReceive);
        }
    }

    return nRetVal;
}

XnStatus ValidateReplyV26(const XnDevicePrivateData* pDevicePrivateData, XnUChar* pBuffer, XnUInt32 nBufferSize, XnUInt16 nExpectedOpcode, XnUInt16 nRequestId, XnUInt16& nDataSize, XnUChar** pDataBuf)
{
    XnUInt16 nHeaderOffset = 0;
    XnHostProtocolHeaderV26* pHeader = (XnHostProtocolHeaderV26*)pBuffer;

    pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nMagic);

    while (pHeader->nMagic != pDevicePrivateData->FWInfo.nFWMagic && nHeaderOffset < nBufferSize - pDevicePrivateData->FWInfo.nProtocolHeaderSize - sizeof(XnHostProtocolReplyHeader))
    {
        nHeaderOffset++;
        pHeader = (XnHostProtocolHeaderV26*)(pBuffer + nHeaderOffset);
        pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nMagic);
    }

    pHeader->nId = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nId);
    pHeader->nOpcode = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nOpcode);
    pHeader->nSize = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nSize);

    if (pHeader->nMagic != pDevicePrivateData->FWInfo.nFWMagic)
    {
        return XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC;
    }

    if (pHeader->nId != nRequestId)
    {
        //		printf("Response ID (%d) not the same as request id (%d)\n", pHeader->nId, nRequestId);
        return XN_STATUS_DEVICE_PROTOCOL_WRONG_ID;
    }

    if (pHeader->nOpcode != nExpectedOpcode)
    {
        //		printf("Unexpected opcode %d (expected: %d)\n", pHeader->nOpcode, nExpectedOpcode);
        return XN_STATUS_DEVICE_PROTOCOL_WRONG_OPCODE;
    }
    // CRC?
    // ...

    XnHostProtocolReplyHeader* pReply = (XnHostProtocolReplyHeader*)(pBuffer + nHeaderOffset + pDevicePrivateData->FWInfo.nProtocolHeaderSize);
    pReply->nErrorCode = XN_PREPARE_VAR16_IN_BUFFER(pReply->nErrorCode);

    if (pReply->nErrorCode != ACK)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Received NACK: %d", pReply->nErrorCode);

        switch (pReply->nErrorCode)
        {
        case NACK_INVALID_COMMAND:
            return XN_STATUS_DEVICE_PROTOCOL_INVALID_COMMAND;
        case NACK_BAD_PACKET_CRC:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PACKET_CRC;
        case NACK_BAD_PACKET_SIZE:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PACKET_SIZE;
        case NACK_BAD_PARAMS:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PARAMS;
        case NACK_I2C_TRANSACTION_FAILED:
            return XN_STATUS_DEVICE_PROTOCOL_I2C_TRANSACTION_FAILED;
        case NACK_FILE_NOT_FOUND:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_NOT_FOUND;
        case NACK_FILE_CREATE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_CREATE_FAILURE;
        case NACK_FILE_WRITE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_WRITE_FAILURE;
        case NACK_FILE_DELETE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_DELETE_FAILURE;
        case NACK_FILE_READ_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_READ_FAILURE;
        case NACK_BAD_COMMAND_SIZE:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_COMMAND_SIZE;
        case NACK_NOT_READY:
            return XN_STATUS_DEVICE_PROTOCOL_NOT_READY;
        case NACK_OVERFLOW:
            return XN_STATUS_DEVICE_PROTOCOL_OVERFLOW;
        case NACK_OVERLAY_NOT_LOADED:
            return XN_STATUS_DEVICE_PROTOCOL_OVERLAY_NOT_LOADED;
        case NACK_FILE_SYSTEM_LOCKED:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_SYSTEM_LOCKED;
        case NACK_NOT_WRITE_PUBLIC_KEY:
            return XN_STATUS_IO_DEVICE_NOT_WRITE_PUBLIC_KEY;
        case NACK_PUBLIC_KEY_MD5_VERIFY_FAILED:
            return XN_STATUS_IO_DEVICE_PUBLIC_KEY_MD5_VERIFY_FAIL;
        case NACK_NOT_WRITE_MD5:
            return XN_STATUS_IO_DEVICE_NOT_WRITE_MD5;
        case NACK_UNKNOWN_ERROR:
        default:
            return XN_STATUS_DEVICE_PROTOCOL_UNKNOWN_ERROR;
        }
    }
    // Check reply length is reasonable for opcode

    nDataSize = pHeader->nSize - sizeof(XnHostProtocolReplyHeader) / sizeof(XnUInt16);

    if (pDataBuf)
        *pDataBuf = pBuffer + nHeaderOffset + pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnHostProtocolReplyHeader);
    return XN_STATUS_OK;
}

XnStatus ValidateReplyV25(const XnDevicePrivateData* pDevicePrivateData, XnUChar* pBuffer, XnUInt32 nBufferSize, XnUInt16 nExpectedOpcode, XnUInt16 nRequestId, XnUInt16& nDataSize, XnUChar** pDataBuf)
{
    XnUInt16 nHeaderOffset = 0;
    XnHostProtocolHeaderV25* pHeader = (XnHostProtocolHeaderV25*)pBuffer;

    pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nMagic);

    while (pHeader->nMagic != pDevicePrivateData->FWInfo.nFWMagic && nHeaderOffset < nBufferSize - pDevicePrivateData->FWInfo.nProtocolHeaderSize - sizeof(XnHostProtocolReplyHeader))
    {
        nHeaderOffset++;
        pHeader = (XnHostProtocolHeaderV25*)(pBuffer + nHeaderOffset);
        pHeader->nMagic = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nMagic);
    }

    pHeader->nCRC16 = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nCRC16);
    pHeader->nId = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nId);
    pHeader->nOpcode = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nOpcode);
    pHeader->nSize = XN_PREPARE_VAR16_IN_BUFFER(pHeader->nSize);

    if (pHeader->nMagic != pDevicePrivateData->FWInfo.nFWMagic)
    {
        return XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC;
    }

    if (pHeader->nId != nRequestId)
    {
        //		printf("Response ID (%d) not the same as request id (%d)\n", pHeader->nId, nRequestId);
        return XN_STATUS_DEVICE_PROTOCOL_WRONG_ID;
    }

    if (pHeader->nOpcode != nExpectedOpcode)
    {
        //		printf("Unexpected opcode %d (expected: %d)\n", pHeader->nOpcode, nExpectedOpcode);
        return XN_STATUS_DEVICE_PROTOCOL_WRONG_OPCODE;
    }
    // CRC?
    // ...

    XnHostProtocolReplyHeader* pReply = (XnHostProtocolReplyHeader*)(pBuffer + nHeaderOffset + pDevicePrivateData->FWInfo.nProtocolHeaderSize);

    pReply->nErrorCode = XN_PREPARE_VAR16_IN_BUFFER(pReply->nErrorCode);

    if (pReply->nErrorCode != ACK)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Received NACK: %d", pReply->nErrorCode);

        switch (pReply->nErrorCode)
        {
        case NACK_INVALID_COMMAND:
            return XN_STATUS_DEVICE_PROTOCOL_INVALID_COMMAND;
        case NACK_BAD_PACKET_CRC:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PACKET_CRC;
        case NACK_BAD_PACKET_SIZE:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PACKET_SIZE;
        case NACK_BAD_PARAMS:
            return XN_STATUS_DEVICE_PROTOCOL_BAD_PARAMS;
        case NACK_I2C_TRANSACTION_FAILED:
            return XN_STATUS_DEVICE_PROTOCOL_I2C_TRANSACTION_FAILED;
        case NACK_FILE_NOT_FOUND:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_NOT_FOUND;
        case NACK_FILE_CREATE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_CREATE_FAILURE;
        case NACK_FILE_WRITE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_WRITE_FAILURE;
        case NACK_FILE_DELETE_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_DELETE_FAILURE;
        case NACK_FILE_READ_FAILURE:
            return XN_STATUS_DEVICE_PROTOCOL_FILE_READ_FAILURE;
        case NACK_NOT_WRITE_PUBLIC_KEY:
            return XN_STATUS_IO_DEVICE_NOT_WRITE_PUBLIC_KEY;
        case NACK_PUBLIC_KEY_MD5_VERIFY_FAILED:
            return XN_STATUS_IO_DEVICE_PUBLIC_KEY_MD5_VERIFY_FAIL;
        case NACK_NOT_WRITE_MD5:
            return XN_STATUS_IO_DEVICE_NOT_WRITE_MD5;
        case NACK_UNKNOWN_ERROR:
        default:
            return XN_STATUS_DEVICE_PROTOCOL_UNKNOWN_ERROR;
        }
    }
    // Check reply length is reasonable for opcode

    nDataSize = pHeader->nSize - sizeof(XnHostProtocolReplyHeader) / sizeof(XnUInt16);

    if (pDataBuf)
        *pDataBuf = pBuffer + nHeaderOffset + pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnHostProtocolReplyHeader);
    return XN_STATUS_OK;
}

XnUInt32 XnHostProtocolGetTimeOut(const XnDevicePrivateData* pDevicePrivateData, XnUInt16 nOpcode)
{
    if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeKeepAlive)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_KEEP_ALIVE;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeGetVersion)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_GETVERSION;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeSetParam)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_SETPARAM;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeInitFileUpload)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_UPLOAD;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeDeleteFile)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeDownloadFile)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeFileTransfer)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_FILE_OPS;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeBIST)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_BIST;
    else if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeGetEmitterData)
        return XN_USB_HOST_PROTOCOL_TIMEOUT_EMITTER_DATA;
    else
        return XN_USB_HOST_PROTOCOL_TIMEOUT;
}

XnUInt32 XnHostProtocolGetSetParamRecvTimeOut(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam)
{
    if (nParam == PARAM_IMAGE_FLICKER_DETECTION)
        return pDevicePrivateData->FWInfo.nUSBDelaySetParamFlicker;
    else if (nParam == PARAM_GENERAL_STREAM0_MODE)
        return pDevicePrivateData->FWInfo.nUSBDelaySetParamStream0Mode;
    else if (nParam == PARAM_GENERAL_STREAM1_MODE)
        return pDevicePrivateData->FWInfo.nUSBDelaySetParamStream1Mode;
    else if (nParam == PARAM_GENERAL_STREAM2_MODE)
        return pDevicePrivateData->FWInfo.nUSBDelaySetParamStream2Mode;
    else if (nParam == PARAM_GENERAL_STREAM3_MODE)
        return pDevicePrivateData->FWInfo.nUSBDelaySetParamStream3Mode;
    else
        return 0;
}

XnStatus XnHostProtocolGetRequestID(const XnDevicePrivateData* pDevicePrivateData, XnUChar* pBuffer, XnUInt16* pnRequestId)
{
    XnUInt16 nRequestId;

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_2)
    {
        nRequestId = ((XnHostProtocolHeaderV26*)(pBuffer))->nId;
    }
    else
    {
        nRequestId = ((XnHostProtocolHeaderV25*)(pBuffer))->nId;
    }

    *pnRequestId = XN_PREPARE_VAR16_IN_BUFFER(nRequestId);
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolReceiveReply(
    const XnDevicePrivateData* pDevicePrivateData,
    XnUChar* pBuffer,
    XnUInt32 nTimeOut,
    XnUInt16 nOpcode,
    XnUInt16 nRequestId,
    XnUInt32* pnReadBytes,
    XnUInt16* pnDataSize,
    XnUChar** ppRelevantBuffer,
    XnBool bForceBulk,
    XnUInt32 nRecvTimeout,
    XnUInt32 nFailTimeout)
{
    XnStatus rc = XN_STATUS_OK;

    XnUInt64 nStartWaitingTime;
    xnOSGetTimeStamp(&nStartWaitingTime);

    for (;;) // loop until timeout expires
    {
        do // loop until right reply ID is received
        {
            // receive reply
            if (nRecvTimeout != 0)
            {
                xnOSSleep(nRecvTimeout);
            }

            rc = XnHostProtocolUSBReceive(pDevicePrivateData, pBuffer, pDevicePrivateData->FWInfo.nProtocolMaxPacketSize, *pnReadBytes, nTimeOut, bForceBulk, nFailTimeout);
            XN_IS_STATUS_OK(rc);

            // Validate the reply
            if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_2)
            {
                rc = ValidateReplyV26(pDevicePrivateData, pBuffer, *pnReadBytes, nOpcode, nRequestId, *pnDataSize, ppRelevantBuffer);
            }
            else
            {
                rc = ValidateReplyV25(pDevicePrivateData, pBuffer, *pnReadBytes, nOpcode, nRequestId, *pnDataSize, ppRelevantBuffer);
            }
        } while (rc == XN_STATUS_DEVICE_PROTOCOL_WRONG_ID);

        XnUInt64 nNow;
        xnOSGetTimeStamp(&nNow);

        if (rc != XN_STATUS_OK && rc != XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC)
        {
            return rc;
        }
        else if (rc == XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC && (nNow - nStartWaitingTime) > XN_RECEIVE_USB_DATA_TIMEOUT)
        {
            // Timeout expired
            return XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC;
        }
        else if (rc == XN_STATUS_DEVICE_PROTOCOL_BAD_MAGIC)
        {
            // Timeout not expired yet
            xnOSSleep(10);
        }
        else
        {
            // OK
            break;
        }
    }

    return rc;
}

XnStatus XnHostProtocolExecute(
    const XnDevicePrivateData* pDevicePrivateData,
    XnUChar* pBuffer,
    XnUInt16 nSize,
    XnUInt16 nOpcode,
    XnUChar** ppRelevantBuffer,
    XnUInt16& nDataSize,
    XnUInt32 nRecvTimeout = 0)
{
    XnStatus rc;
    XnUInt32 nRead = 0;
    XnUInt32 nFailTimeout = 0;
    XnBool bForceBulk = FALSE;

    if (nOpcode == OPCODE_INVALID)
    {
        return (XN_STATUS_DEVICE_PROTOCOL_UNSUPPORTED_OPCODE);
    }

    // don't bother trying to communicate with the device if it was disconnected
    if (NULL == pDevicePrivateData->pSensor || pDevicePrivateData->pSensor->GetErrorState() == XN_STATUS_DEVICE_NOT_CONNECTED)
    {
        return (XN_STATUS_DEVICE_NOT_CONNECTED);
    }

    XnUInt32 nTimeOut = XnHostProtocolGetTimeOut(pDevicePrivateData, nOpcode);

    // store request (in case we need to retry it)
    XnUChar request[MAX_PACKET_SIZE];
    xnOSMemCopy(request, pBuffer, nSize);

    XnUInt16 nRequestId;
    rc = XnHostProtocolGetRequestID(pDevicePrivateData, pBuffer, &nRequestId);
    XN_IS_STATUS_OK(rc);

    XnUInt16 nRetriesLeft = XN_HOST_PROTOCOL_NOT_READY_RETRIES;
    while (nRetriesLeft-- > 0) // loop until device is ready
    {
        rc = xnOSLockMutex(pDevicePrivateData->hExecuteMutex, XN_WAIT_INFINITE);
        XN_IS_STATUS_OK(rc);

        // Sleep before sending the control
        if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload)
        {
            XnUInt64 nNow;
            XnUInt64 nNow2;
            xnOSGetHighResTimeStamp(&nNow);
            xnOSGetHighResTimeStamp(&nNow2);
            while (nNow2 - nNow < XN_USB_HOST_PROTOCOL_FILE_UPLOAD_PRE_DELAY)
            {
                xnOSGetHighResTimeStamp(&nNow2);
            }
        }
        else
        {
            xnOSSleep(pDevicePrivateData->FWInfo.nUSBDelayExecutePreSend);
        }

        // Send request
        rc = XnHostProtocolUSBSend(pDevicePrivateData, request, nSize, nTimeOut, bForceBulk);
        if (rc != XN_STATUS_OK)
        {
            xnOSUnLockMutex(pDevicePrivateData->hExecuteMutex);
            return rc;
        }

        // Sleep before trying to read the reply
        if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload)
        {
            nFailTimeout = XN_USB_HOST_PROTOCOL_FILE_UPLOAD_PRE_DELAY;
        }
        else
        {
            xnOSSleep(pDevicePrivateData->FWInfo.nUSBDelayExecutePostSend);
        }

        // receive reply
        rc = XnHostProtocolReceiveReply(pDevicePrivateData, pBuffer, nTimeOut, nOpcode, nRequestId, &nRead, &nDataSize, ppRelevantBuffer, bForceBulk, nRecvTimeout, nFailTimeout);

        if (rc == XN_STATUS_DEVICE_PROTOCOL_NOT_READY || rc == XN_STATUS_OK)
        {
            XnStatus unlockRC = xnOSUnLockMutex(pDevicePrivateData->hExecuteMutex);
            XN_IS_STATUS_OK(unlockRC);
        }
        else
        {
            xnOSUnLockMutex(pDevicePrivateData->hExecuteMutex);
            return rc;
        }

        if (rc == XN_STATUS_OK)
            break;

        xnOSSleep(1000);
        xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Device not ready. %d more retries...", nRetriesLeft);
    }

    XN_IS_STATUS_OK(rc);

    if (ppRelevantBuffer == NULL)
        return XN_STATUS_OK;

    // Get rest of data
    XnInt32 nCur = nRead; // Read so far

    nRead -= (pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnHostProtocolReplyHeader)); // Data read so far

    while (nRead < nDataSize * 2U)
    {
        XnUInt32 dummy = 0;

        rc = XnHostProtocolUSBReceive(pDevicePrivateData, pBuffer + nCur, pDevicePrivateData->FWInfo.nProtocolMaxPacketSize, dummy, nTimeOut, bForceBulk, 0);
        if (rc != XN_STATUS_OK)
        {
            return rc;
        }

        nCur += dummy;
        nRead += dummy;
    }

    return XN_STATUS_OK;
}


XnStatus XnHostProtocolUSBBulkSend(const XnDevicePrivateData* pDevicePrivateData, XnUChar* pBuffer, XnUInt32 nSize, XnUInt32 nTimeOut)
{
    XnStatus nRetVal = XN_STATUS_OK;
    XnInt32 nCounter = XN_USB_HOST_PROTOCOL_SEND_RETRIES;
    const XnUsbControlConnection* pCtrlConnection = &pDevicePrivateData->SensorHandle.ControlConnectionBulk;
    while (nCounter--)
    {
        if (pCtrlConnection->bIsBulk)
            nRetVal = xnUSBWriteEndPoint(pCtrlConnection->ControlOutConnectionEp, pBuffer, nSize, nTimeOut);
        else
            nRetVal = XN_STATUS_USB_CONTROL_SEND_FAILED;

        if (XN_STATUS_USB_TRANSFER_TIMEOUT != nRetVal && XN_STATUS_USB_TRANSFER_STALL != nRetVal)
            break;

        xnOSSleep(100);
    }

    return nRetVal;
}

XnStatus XnHostProtocolBulkExecute(
    const XnDevicePrivateData* pDevicePrivateData,
    XnUChar* pBuffer,
    XnUInt32 nSize,
    XnUInt16 nOpcode)
{
    XnStatus rc;
    if (OPCODE_INVALID == nOpcode)
        return (XN_STATUS_DEVICE_PROTOCOL_UNSUPPORTED_OPCODE);

    /// Don't bother trying to communicate with the device if it was disconnected.
    if (XN_STATUS_DEVICE_NOT_CONNECTED == pDevicePrivateData->pSensor->GetErrorState())
        return (XN_STATUS_DEVICE_NOT_CONNECTED);

    XnUInt32 nTimeOut = XnHostProtocolGetTimeOut(pDevicePrivateData, nOpcode);

    /// Store request (in case we need to retry it).
    XnUChar request[MAX_PACKET_BULK_SIZE] = { 0 };
    xnOSMemCopy(request, pBuffer, nSize);

    XnUInt16 nRequestId = 0;
    rc = XnHostProtocolGetRequestID(pDevicePrivateData, pBuffer, &nRequestId);
    XN_IS_STATUS_OK(rc);

    /// Loop until device is ready.
    XnInt16 nRetries = XN_HOST_PROTOCOL_NOT_READY_RETRIES;
    while (nRetries--)
    {
        rc = xnOSLockMutex(pDevicePrivateData->hExecuteMutex, XN_WAIT_INFINITE);
        XN_IS_STATUS_OK(rc);

        /// Sleep before sending the control.
        if (nOpcode == pDevicePrivateData->FWInfo.nOpcodeFileTransfer)
        {
            XnUInt64 nStart = 0;
            XnUInt64 nEnd = 0;
            xnOSGetHighResTimeStamp(&nStart);
            xnOSGetHighResTimeStamp(&nEnd);
            while (nEnd - nStart < XN_USB_HOST_PROTOCOL_FILE_UPLOAD_PRE_DELAY)
            {
                xnOSGetHighResTimeStamp(&nEnd);
            }
        }
        else
            xnOSSleep(pDevicePrivateData->FWInfo.nUSBDelayExecutePreSend);

        /// Send request.
        rc = XnHostProtocolUSBBulkSend(pDevicePrivateData, request, nSize, nTimeOut);
        if (XN_STATUS_OK != rc)
        {
            xnOSUnLockMutex(pDevicePrivateData->hExecuteMutex);
            return rc;
        }

        xnOSUnLockMutex(pDevicePrivateData->hExecuteMutex);
        if (XN_STATUS_OK == rc)
            break;

        xnOSSleep(1000);
    }

    return XN_STATUS_OK;
}

#pragma pack (push, 1)
typedef struct
{
    XnUInt16 nEntrySize;
    XnUInt32 nTimeStamp;
    XnUInt16 nLogType;
} XnLogEntryHeader;

typedef struct
{
    XnUInt16 nLine;
    XnUInt32 nParam;
} XnLogDefaultData;
#pragma pack(pop)

XnStatus XnHostProtocolGetLog(XnDevicePrivateData* pDevicePrivateData, XnChar* csBuffer, XnUInt32 nBufferSize)
{
    XnStatus rc = XN_STATUS_OK;
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnUChar allLogBuffer[XN_MAX_LOG_SIZE];

    XnUInt nAllLogBytes = 0;

    // loop until no more log is available
    for (;;)
    {
        XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetLog);

        XnUChar* pRelevantBuffer;
        XnUInt16 nDataSize;

        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetLog,
            &pRelevantBuffer, nDataSize);

        XN_IS_STATUS_OK(rc);

        if (nDataSize == 0) // no more log
            break;

        // translate to bytes
        nDataSize *= sizeof(XnUInt16);

        if (nAllLogBytes + nDataSize > XN_MAX_LOG_SIZE)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Log Buffer is too small. received %d bytes, but buffer is %d long", nAllLogBytes + nDataSize, XN_MAX_LOG_SIZE);
            return XN_STATUS_INTERNAL_BUFFER_TOO_SMALL;
        }

        xnOSMemCopy(allLogBuffer + nAllLogBytes, pRelevantBuffer, nDataSize);
        nAllLogBytes += nDataSize;
    }

    XnUChar* pCurrData = allLogBuffer;
    XnUChar* pEndData = allLogBuffer + nAllLogBytes;

    XnUInt32 nBufferUsed = 0;

    // now parse it
    while (pCurrData < pEndData)
    {
        XnLogEntryHeader* pLogEntryHeader = (XnLogEntryHeader*)pCurrData;
        pCurrData += sizeof(XnLogEntryHeader);

        pLogEntryHeader->nEntrySize = XN_PREPARE_VAR16_IN_BUFFER(pLogEntryHeader->nEntrySize);
        pLogEntryHeader->nLogType = XN_PREPARE_VAR16_IN_BUFFER(pLogEntryHeader->nLogType);
        pLogEntryHeader->nTimeStamp = XN_PREPARE_VAR32_IN_BUFFER(pLogEntryHeader->nTimeStamp);

        // lower byte contains error type and higher contains module ID
        XnUInt32 nCharsWritten = 0;

        if (pLogEntryHeader->nLogType == pDevicePrivateData->FWInfo.nLogStringType)
        {
            // text messages are in wide characters
            //XnWChar wcsMessage[600] = {0};

            if (pLogEntryHeader->nEntrySize*sizeof(XnUInt16) > 600)
            {
                xnLogError(XN_MASK_SENSOR_PROTOCOL, "Got a log entry with %d bytes!", pLogEntryHeader->nEntrySize*sizeof(XnUInt16));
                return XN_STATUS_INTERNAL_BUFFER_TOO_SMALL;
            }

            rc = xnOSStrFormat((XnChar*)csBuffer + nBufferUsed, nBufferSize - nBufferUsed, &nCharsWritten, "%u:\t", pLogEntryHeader->nTimeStamp);
            XN_IS_STATUS_OK(rc);
            nBufferUsed += nCharsWritten;

            if (nBufferSize - nBufferUsed < pLogEntryHeader->nEntrySize)
            {
                xnLogError(XN_MASK_SENSOR_PROTOCOL, "Not enough space in user buffer!");
                return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
            }

            for (XnUInt32 i = 0; i < pLogEntryHeader->nEntrySize; ++i)
            {
                csBuffer[nBufferUsed++] = pCurrData[i * 2];
            }

            csBuffer[nBufferUsed++] = '\n';
        }
        else if (pLogEntryHeader->nLogType == pDevicePrivateData->FWInfo.nLogOverflowType)
        {
            rc = xnOSStrFormat((XnChar*)csBuffer + nBufferUsed, nBufferSize - nBufferUsed, &nCharsWritten, "%u:\tLog Overflow\n", pLogEntryHeader->nTimeStamp);
            XN_IS_STATUS_OK(rc);

            nBufferUsed += nCharsWritten;
        }
        else
        {
            XnLogDefaultData* pData = (XnLogDefaultData*)pCurrData;

            pData->nLine = XN_PREPARE_VAR16_IN_BUFFER(pData->nLine);
            pData->nParam = XN_PREPARE_VAR16_IN_BUFFER(pData->nParam);

            rc = xnOSStrFormat((XnChar*)csBuffer + nBufferUsed, nBufferSize - nBufferUsed, &nCharsWritten,
                "%u:\tModule: [0x%X], Error: [0x%X], Param: 0x%X, (Line: %d)\n",
                pLogEntryHeader->nTimeStamp, XnChar(pLogEntryHeader->nLogType >> 8),
                XnChar(pLogEntryHeader->nLogType), pData->nParam, pData->nLine);
            XN_IS_STATUS_OK(rc);

            nBufferUsed += nCharsWritten;
        }

        pCurrData += pLogEntryHeader->nEntrySize*sizeof(XnUInt16);
    }

    if (nBufferUsed > 0)
    {
        // add null termination
        csBuffer[nBufferUsed] = '\0';
        nBufferUsed++;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetVersion(const XnDevicePrivateData* pDevicePrivateData, XnVersions& Version)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnVersions *pVersion = NULL;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting hardware versions...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetVersion);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetVersion,
        (XnUChar**)(&pVersion), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get version failed: %s", xnGetStatusString(rc));

        return rc;
    }

    xnOSMemCopy(&Version, pVersion, sizeof(XnVersions));

    Version.nBuild = XN_PREPARE_VAR16_IN_BUFFER(Version.nBuild);
    Version.nChip = XN_PREPARE_VAR32_IN_BUFFER(Version.nChip);
    Version.nFPGA = XN_PREPARE_VAR16_IN_BUFFER(Version.nFPGA);
    Version.nSystemVersion = XN_PREPARE_VAR16_IN_BUFFER(Version.nSystemVersion);

    *((XnUInt16*)&Version) = xnOSEndianSwapUINT16(*((XnUInt16*)pVersion));

    if (Version.nMajor >= 5)
    {
        XnChar cpBuffer[XN_MAX_OS_NAME_LENGTH];
        sprintf(cpBuffer, "%x", Version.nBuild);
        Version.nBuild = (XnUInt16)atoi(cpBuffer);
    }

    Version.SDK.nMajor = XN_PS_MAJOR_VERSION;
    Version.SDK.nMinor = XN_PS_MINOR_VERSION;
    Version.SDK.nMaintenance = XN_PS_MAINTENANCE_VERSION;
    Version.SDK.nBuild = XN_PS_BUILD_VERSION;

    // find out hardware version
    if (Version.nFPGA == XN_FPGA_VER_FPDB_26)
    {
        Version.HWVer = XN_SENSOR_HW_VER_FPDB_10;
    }
    else if (Version.nFPGA == XN_FPGA_VER_FPDB_25)
    {
        Version.HWVer = XN_SENSOR_HW_VER_FPDB_10;
    }
    else if (Version.nFPGA == XN_FPGA_VER_CDB)
    {
        Version.HWVer = XN_SENSOR_HW_VER_CDB_10;
    }
    else if (Version.nFPGA == XN_FPGA_VER_CDB)
    {
        Version.HWVer = XN_SENSOR_HW_VER_CDB_10;
    }
    else if (Version.nFPGA == XN_FPGA_VER_RD3)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD_3;
    }
    else if (Version.nFPGA == XN_FPGA_VER_RD5)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD_5;
    }
    else if (Version.nFPGA == XN_FPGA_VER_RD1081)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD1081;
    }
    else if (Version.nFPGA == XN_FPGA_VER_RD1082)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD1082;
    }
    else if (Version.nFPGA == XN_FPGA_VER_RD109)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD109;
    }
    else
    {
        Version.HWVer = XN_SENSOR_HW_VER_UNKNOWN;
    }

    // find out chip version
    if (Version.nChip == XN_CHIP_VER_PS1000)
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_PS1000;
    }
    else if (Version.nChip == XN_CHIP_VER_PS1080)
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_PS1080;
    }
    else if (Version.nChip == XN_CHIP_VER_PS1080A6)
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_PS1080A6;
    }
    else if (Version.nChip == XN_CHIP_VER_MX6000)
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_MX6000;
    }
    else if (Version.nChip == XN_CHIP_VER_DUAL_MX6000)
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_DUAL_MX6000;
    }
    else
    {
        Version.ChipVer = XN_SENSOR_CHIP_VER_UNKNOWN;
    }

    // find out sensor version
    Version.SensorVer = XN_SENSOR_VER_UNKNOWN;

    // in some firmwares, the HWVer was incorrect. Override according to firmware number
    Version.FWVer = GetFWVersion(Version.nMajor, Version.nMinor, Version.nBuild);
    if (Version.FWVer == XN_SENSOR_FW_VER_5_0)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD_5;
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_1)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD_5;
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_2)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD_5;
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_3)
    {
        if (Version.nBuild < 28)
        {
            Version.HWVer = XN_SENSOR_HW_VER_RD1081;
        }
        else if (Version.nBuild == 28)
        {
            Version.HWVer = XN_SENSOR_HW_VER_RD1082;
        }
        // 5.3.29 and up returns valid HW versions, so no need to override anything
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_4)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD1082;
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_5)
    {
        Version.HWVer = XN_SENSOR_HW_VER_RD1082;
    }
    else if (Version.FWVer == XN_SENSOR_FW_VER_5_6)
    {
        if (CompareVersion(Version.nMajor, Version.nMinor, Version.nBuild, 5, 6, 6) >= 0)
        {
            if (Version.nFPGA == 0)
            {
                Version.HWVer = XN_SENSOR_HW_VER_RD1081;
            }
            else if (Version.nFPGA == 1)
            {
                Version.HWVer = XN_SENSOR_HW_VER_RD1082;
            }
        }
        else
        {
            Version.HWVer = XN_SENSOR_HW_VER_RD1082;
        }
    }

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL,
        "Hardware versions: FW=%d.%d.%d (%d) HW=%d Chip=%d Sensor=%d SYS=%d",
        Version.nMajor, Version.nMinor, Version.nBuild,
        Version.FWVer, Version.HWVer, Version.ChipVer,
        Version.SensorVer, Version.nSystemVersion);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolKeepAlive(XnDevicePrivateData* pDevicePrivateData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Requesting KeepAlive...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeKeepAlive);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeKeepAlive,
        NULL, nDataSize);

    if (rc == XN_STATUS_OK)
        xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Got KeepAlive Reply.");
    else
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "KeepAlive failed: %s", xnGetStatusString(rc));

    return rc;
}

XnStatus XnHostProtocolReadAHB(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nAddress, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nAddress);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt32), pDevicePrivateData->FWInfo.nOpcodeReadAHB);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt32), pDevicePrivateData->FWInfo.nOpcodeReadAHB,
        (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    nValue = XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolWriteAHB(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nAddress, XnUInt32 nValue,
    XnUInt32 nMask)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Write AHB: 0x%08x 0x%08x 0x%08x", nAddress, nValue, nMask);

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nAddress);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);
    *(((XnUInt32*)pDataBuf) + 2) = XN_PREPARE_VAR32_IN_BUFFER(nMask);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt32) * 3, pDevicePrivateData->FWInfo.nOpcodeWriteAHB);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt32) * 3, pDevicePrivateData->FWInfo.nOpcodeWriteAHB,
        NULL, nDataSize);
    return rc;
}

XnStatus XnHostProtocolGetParam(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnUInt16& nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    //	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Getting Parameter [%d]...", nParam);
    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nParam);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetParam);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetParam,
        (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getting [%d]: %s", nParam, xnGetStatusString(rc));
        return rc;
    }

    //xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Param[%d] = %d", nParam, *pValue);
    nValue = XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetParam(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnUInt16 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    //	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Setting Parameter [%d] to %d", nParam, nValue);

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nParam);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(nValue);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16) * 2, pDevicePrivateData->FWInfo.nOpcodeSetParam);

    XnUInt16 nDataSize;

    XnInt32 nTimesLeft = 5;
    XnStatus rc = XN_STATUS_ERROR;
    while (nTimesLeft > 0)
    {
        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16) * 2, pDevicePrivateData->FWInfo.nOpcodeSetParam,
            NULL, nDataSize, XnHostProtocolGetSetParamRecvTimeOut(pDevicePrivateData, nParam));
        nTimesLeft--;

        if (rc == XN_STATUS_OK ||
            rc == XN_STATUS_DEVICE_PROTOCOL_BAD_PARAMS ||
            rc == XN_STATUS_DEVICE_NOT_CONNECTED ||
            rc == XN_STATUS_DEVICE_PROTOCOL_INVALID_COMMAND)
        {
            break;
        }

        xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Retrying to set the param... rc=%d", rc);
    }

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed setting [%d] to [%d]: %s", nParam, nValue, xnGetStatusString(rc));
    }
    else
    {
        //		xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Done.");
    }

    return rc;
}

void XnHostPrototcolAdjustFixedParamsV26(XnFixedParamsV26* pFixedParamsV26, XnFixedParams* pFixedParams)
{
    // the only difference from V2.6 to V3.0 is the 4 last parameters
    xnOSMemCopy(pFixedParams, pFixedParamsV26, sizeof(XnFixedParamsV26));
    pFixedParams->nUseExtPhy = pFixedParamsV26->nUseExtPhy;
    pFixedParams->bProjectorProtectionEnabled = FALSE;
    pFixedParams->nProjectorDACOutputVoltage = FALSE;
    pFixedParams->nTecEmitterDelay = pFixedParamsV26->nTecEmitterDelay;
}

void XnHostPrototcolAdjustFixedParamsV20(XnFixedParamsV20* pFixedParamsV20, XnFixedParams* pFixedParams)
{
    // the only difference from V2.0 to V2.6 is the addition of nUseExtPhy
    XnFixedParamsV26 fixedParamsV26;
    xnOSMemCopy(&fixedParamsV26, pFixedParamsV20, sizeof(XnFixedParamsV20));

    // now adjust from V2.6 to current
    XnHostPrototcolAdjustFixedParamsV26(&fixedParamsV26, pFixedParams);
}

XnStatus XnHostProtocolGetFixedParams(XnDevicePrivateData* pDevicePrivateData, XnFixedParams& FixedParams)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUChar* pRelevantBuffer;
    XnUInt16 nFixedParamSize = 0;

    XnChar FixedParamsBuffer[2048] = { 0 };
    XnChar* pData = FixedParamsBuffer;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting the fixed params...");

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        nFixedParamSize = sizeof(XnFixedParams);
    }
    else if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_1)
    {
        nFixedParamSize = sizeof(XnFixedParamsV26);
    }
    else // v0.17
    {
        nFixedParamSize = sizeof(XnFixedParamsV20);
    }

    xnOSMemSet(&FixedParams, 0, sizeof(XnFixedParams));

    XnInt16 nDataRead = 0;

    XnStatus rc;
    while (nDataRead < nFixedParamSize)
    {
        *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(XnUInt16(nDataRead / sizeof(XnUInt32)));
        XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetFixedParams);

        XnUInt16 nDataSize;

        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetFixedParams,
            &pRelevantBuffer, nDataSize);

        if (rc != XN_STATUS_OK)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get fixed params failed: %s", xnGetStatusString(rc));

            return rc;
        }

        XnUInt32 nReadNow = nDataSize*sizeof(XnUInt16);
        if (nReadNow == 0)
        {
            break;
        }

        xnOSMemCopy(pData + nDataRead, pRelevantBuffer, nReadNow);

        nDataRead += (XnUInt16)nReadNow;
    }

    for (XnUInt32 i = 0; i < nFixedParamSize / sizeof(XnUInt32); i++)
    {
        XnUInt32 temp = *((XnUInt32*)(&FixedParams) + i);
        *((XnUInt32*)(&FixedParams) + i) = XN_PREPARE_VAR32_IN_BUFFER(temp);
    }

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        xnOSMemCopy(&FixedParams, FixedParamsBuffer, sizeof(XnFixedParams));
    }
    else if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_1)
    {
        XnFixedParamsV26 fixedParamsV26;
        xnOSMemCopy(&fixedParamsV26, FixedParamsBuffer, nFixedParamSize);
        XnHostPrototcolAdjustFixedParamsV26(&fixedParamsV26, &FixedParams);
    }
    else if (pDevicePrivateData->FWInfo.nFWVer == XN_SENSOR_FW_VER_0_17)
    {
        XnFixedParamsV20 fixedParamsV20;
        xnOSMemCopy(&fixedParamsV20, FixedParamsBuffer, nFixedParamSize);
        XnHostPrototcolAdjustFixedParamsV20(&fixedParamsV20, &FixedParams);
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16& nMode)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetMode);

    XnUInt16 nDataSize = 0;
    XnUInt16* pMode = NULL;
    XnStatus rc = XnHostProtocolExecute(
        pDevicePrivateData,
        buffer,
        pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetMode,
        (XnUChar**)(&pMode),
        nDataSize);
    if (rc != XN_STATUS_OK)
    {
        nMode = XN_HOST_PROTOCOL_MODE_PS;
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get mode failed: %s, 'PROTOCOL_MODE_PS' would be returned by default.", xnGetStatusString(rc));
    }
    else
        nMode = XN_PREPARE_VAR16_IN_BUFFER(*pMode);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolReset(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nResetType)
{
    XnStatus rc = XN_STATUS_OK;

    if (pDevicePrivateData->FWInfo.nFWVer == XN_SENSOR_FW_VER_0_17)
    {
        XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
        XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

        *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nResetType);

        XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeReset);

        XnUInt16 nDataSize;

        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeReset,
            NULL, nDataSize);

        // Power reset can't fail, and device won't have time to send ACK.
        if (nResetType == XN_RESET_TYPE_POWER)
            rc = XN_STATUS_OK;

        return rc;
    }
    else
    {
        XnUInt16 nActualValue;
        switch (nResetType)
        {
        case XN_RESET_TYPE_POWER:
            nActualValue = XN_HOST_PROTOCOL_MODE_REBOOT;
            break;
        case XN_RESET_TYPE_SOFT:
        {
            // also kill streams before (in FW < 5.2)
            if (pDevicePrivateData->FWInfo.nFWVer < XN_SENSOR_FW_VER_5_2)
            {
                XnSensorFirmwareParams* pParams = pDevicePrivateData->pSensor->GetFirmware()->GetParams();
                rc = pParams->m_Stream0Mode.SetValue(XN_VIDEO_STREAM_OFF);
                XN_IS_STATUS_OK(rc);
                rc = pParams->m_Stream1Mode.SetValue(XN_VIDEO_STREAM_OFF);
                XN_IS_STATUS_OK(rc);
                rc = pParams->m_Stream2Mode.SetValue(XN_AUDIO_STREAM_OFF);
                XN_IS_STATUS_OK(rc);
                if (pDevicePrivateData->FWInfo.bAISupported)
                {
                    rc = pParams->m_Stream3Mode.SetValue(XN_VIDEO_STREAM_OFF);
                    XN_IS_STATUS_OK(rc);
                }
            }
        }

        nActualValue = XN_HOST_PROTOCOL_MODE_SOFT_RESET;
        break;
        case XN_RESET_TYPE_SOFT_FIRST:
            nActualValue = XN_HOST_PROTOCOL_MODE_SOFT_RESET;
            break;
        default:
            return XN_STATUS_DEVICE_UNSUPPORTED_MODE;
        }

        return XnHostProtocolSetMode(pDevicePrivateData, nActualValue);
    }
}

XnStatus XnHostProtocolSetMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nMode)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nMode);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Setting mode to %d...", nMode);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeSetMode);

    XnUInt16 nDataSize;

    XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeSetMode,
        NULL, nDataSize);

    // Patch: always return OK when switching modes (since the firmware is changing there is nobody to ACK the request...)
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetCMOSRegister(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress,
    XnUInt16& nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nCMOS);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(nAddress);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16) * 2, pDevicePrivateData->FWInfo.nOpcodeGetCMOSReg);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16) * 2,
        pDevicePrivateData->FWInfo.nOpcodeGetCMOSReg, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    nValue = XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetCMOSRegister(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress,
    XnUInt16 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nCMOS);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(nAddress);
    *(((XnUInt16*)pDataBuf) + 2) = XN_PREPARE_VAR16_IN_BUFFER(nValue);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16) * 3, pDevicePrivateData->FWInfo.nOpcodeSetCMOSReg);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16) * 3,
        pDevicePrivateData->FWInfo.nOpcodeSetCMOSReg, NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolReadI2C(XnDevicePrivateData* pDevicePrivateData, XnI2CReadData* pI2CReadData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(pI2CReadData->nBus);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(pI2CReadData->nSlaveAddress);
    *(((XnUInt16*)pDataBuf) + 2) = XN_PREPARE_VAR16_IN_BUFFER(pI2CReadData->nReadSize);
    for (int i = 0; i < pI2CReadData->nWriteSize; i++)
        *((XnUInt16*)pDataBuf + 3 + i) = XN_PREPARE_VAR16_IN_BUFFER(pI2CReadData->cpWriteBuffer[i]);

    XnUInt16 nOpSize = sizeof(XnUInt16) * 3 + (pI2CReadData->nWriteSize * sizeof(XnUInt16));
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nOpSize, pDevicePrivateData->FWInfo.nOpcodeReadI2C);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nOpSize,
        pDevicePrivateData->FWInfo.nOpcodeReadI2C, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    for (int i = 0; i < nDataSize; i++)
        pI2CReadData->cpReadBuffer[i] = XN_PREPARE_VAR16_IN_BUFFER(*(pValue + i));

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolWriteI2C(XnDevicePrivateData* pDevicePrivateData, const XnI2CWriteData* pI2CWriteData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(pI2CWriteData->nBus);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(pI2CWriteData->nSlaveAddress);
    for (int i = 0; i < pI2CWriteData->nWriteSize; i++)
        *((XnUInt16*)pDataBuf + 2 + i) = XN_PREPARE_VAR16_IN_BUFFER(pI2CWriteData->cpWriteBuffer[i]);

    XnUInt16 nOpSize = sizeof(XnUInt16) * 2 + (pI2CWriteData->nWriteSize * sizeof(XnUInt16));
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nOpSize, pDevicePrivateData->FWInfo.nOpcodeWriteI2C);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nOpSize,
        pDevicePrivateData->FWInfo.nOpcodeWriteI2C, NULL, nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetCMOSRegisterI2C(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress,
    XnUInt16& nValue)
{
    XnStatus nRetVal = XN_STATUS_OK;
    XnI2CReadData I2CReadData;

    nValue = 0;
    I2CReadData.cpReadBuffer[0] = 0;
    I2CReadData.cpReadBuffer[1] = 0;

    I2CReadData.nReadSize = XN_PREPARE_VAR16_IN_BUFFER(2);
    I2CReadData.nWriteSize = XN_PREPARE_VAR16_IN_BUFFER(1);
    I2CReadData.cpWriteBuffer[0] = XN_PREPARE_VAR16_IN_BUFFER(nAddress);

    if (nCMOS == XN_CMOS_TYPE_IMAGE)
    {
        I2CReadData.nBus = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetImageCmosI2CBus());
        I2CReadData.nSlaveAddress = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetImageCmosI2CSlaveAddress());
    }
    else if (nCMOS == XN_CMOS_TYPE_DEPTH)
    {
        I2CReadData.nBus = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetDepthCmosI2CBus());
        I2CReadData.nSlaveAddress = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetDepthCmosI2CSlaveAddress());
    }
    else
    {
        return (XN_STATUS_ERROR);
    }

    nRetVal = XnHostProtocolReadI2C(pDevicePrivateData, &I2CReadData);
    XN_IS_STATUS_OK(nRetVal);

    nValue = XN_PREPARE_VAR16_IN_BUFFER((I2CReadData.cpReadBuffer[0] << 8) + I2CReadData.cpReadBuffer[1]);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetCMOSRegisterI2C(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS, XnUInt16 nAddress,
    XnUInt16 nValue)

{
    XnStatus nRetVal = XN_STATUS_OK;
    XnI2CWriteData I2CWriteData;

    I2CWriteData.cpWriteBuffer[0] = XN_PREPARE_VAR16_IN_BUFFER(nAddress);
    I2CWriteData.cpWriteBuffer[1] = XN_PREPARE_VAR16_IN_BUFFER((nValue >> 8) & 0xff);
    I2CWriteData.cpWriteBuffer[2] = XN_PREPARE_VAR16_IN_BUFFER(nValue & 0xff);
    I2CWriteData.nWriteSize = XN_PREPARE_VAR16_IN_BUFFER(3);

    if (nCMOS == XN_CMOS_TYPE_IMAGE)
    {
        I2CWriteData.nBus = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetImageCmosI2CBus());
        I2CWriteData.nSlaveAddress = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetImageCmosI2CSlaveAddress());
    }
    else if (nCMOS == XN_CMOS_TYPE_DEPTH)
    {
        I2CWriteData.nBus = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetDepthCmosI2CBus());
        I2CWriteData.nSlaveAddress = XN_PREPARE_VAR16_IN_BUFFER(pDevicePrivateData->pSensor->GetFixedParams()->GetDepthCmosI2CSlaveAddress());
    }
    else
    {
        return (XN_STATUS_ERROR);
    }

    nRetVal = XnHostProtocolWriteI2C(pDevicePrivateData, &I2CWriteData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolInitUpload(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt16 nAttributes,
    XnUInt32 nSizeInWords, XN_FILE_HANDLE &FileToUpload, XnUInt32& nNextOffset)
{
    XnStatus rc = XN_STATUS_OK;

    if (pDevicePrivateData->FWInfo.bHasFilesystemLock)
    {
        rc = XnHostProtocolSetParam(pDevicePrivateData, PARAM_FILE_SYSTEM_LOCK, 0);
        if (rc != XN_STATUS_OK)
        {
            xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Failed to unlock file system: %s", xnGetStatusString(rc));
            return rc;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nOffset);
    XnUInt32 *Size = (XnUInt32*)pDataBuf + 1;
    *Size = XN_PREPARE_VAR32_IN_BUFFER(nSizeInWords);
    XnUInt16 nHeaderSize;

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_1_1)
    {
        *((XnUInt16*)((XnUInt32*)pDataBuf + 2)) = XN_PREPARE_VAR16_IN_BUFFER(nAttributes);
        nHeaderSize = sizeof(XnUInt32) + sizeof(XnUInt32) + sizeof(XnUInt16);
    }
    else
    {
        nHeaderSize = sizeof(XnUInt32) + sizeof(XnUInt32);
    }

    XnUInt32 nReadFromFile = pDevicePrivateData->FWInfo.nProtocolMaxPacketSize - pDevicePrivateData->FWInfo.nProtocolHeaderSize - nHeaderSize;
    xnOSSeekFile64(FileToUpload, XN_OS_SEEK_SET, 0);
    xnOSReadFile(FileToUpload, pDataBuf + nHeaderSize, &nReadFromFile);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nHeaderSize + nReadFromFile, pDevicePrivateData->FWInfo.nOpcodeInitFileUpload);

    XnUInt16 nDataSize;
    XnUInt32* pValue = 0;

    XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nHeaderSize + (XnUInt16)nReadFromFile,
        pDevicePrivateData->FWInfo.nOpcodeInitFileUpload, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    nNextOffset = XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolWriteUpload(XnDevicePrivateData* pDevicePrivateData, XN_FILE_HANDLE &FileToUpload,
    XnUInt32 nOffset, XnUInt32 nFileSize, XnUInt32& nNextOffset)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nChunkSize = pDevicePrivateData->FWInfo.nProtocolMaxPacketSize - pDevicePrivateData->FWInfo.nProtocolHeaderSize - sizeof(XnUInt32);

    if (nFileSize - nNextOffset*sizeof(XnUInt16) < nChunkSize)
    {
        nChunkSize = nFileSize - nNextOffset*sizeof(XnUInt16);
    }

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nOffset);
    pDataBuf += sizeof(XnUInt32);

    XnStatus rc = xnOSSeekFile64(FileToUpload, XN_OS_SEEK_SET, nNextOffset*sizeof(XnUInt16));
    XN_IS_STATUS_OK(rc);

    rc = xnOSReadFile(FileToUpload, pDataBuf, &nChunkSize);
    XN_IS_STATUS_OK(rc);

    if (nChunkSize == 1)
    {
        pDataBuf[nChunkSize] = 0;
        nChunkSize++;
    }

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt32) + nChunkSize, pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload);

    XnUInt16 nDataSize;
    XnUInt32* pValue;

    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, (XnUInt16)(pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt32) + nChunkSize),
        pDevicePrivateData->FWInfo.nOpcodeWriteFileUpload, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    nNextOffset = XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolFinishUpload(XnDevicePrivateData* pDevicePrivateData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeFinishFileUpload, NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolFileUpload(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset,
    const XnChar* strFileName, XnUInt16 nAttributes)
{
    XnStatus rc;
    XnUInt64 nFileSize;
    XN_FILE_HANDLE UploadFile;

    rc = xnOSGetFileSize64(strFileName, &nFileSize);
    XN_IS_STATUS_OK(rc);

    rc = xnOSOpenFile(strFileName, XN_OS_FILE_READ, &UploadFile);
    XN_IS_STATUS_OK(rc);

    if (nFileSize % 2 == 1)
        nFileSize++;

    XnUInt32 nNextOffset;

    XnUInt64 nBefore;
    xnOSGetTimeStamp(&nBefore);

    rc = XnHostProtocolInitUpload(pDevicePrivateData, nOffset, nAttributes, (XnUInt32)nFileSize / sizeof(XnUInt16), UploadFile, nNextOffset);
    if (rc != XN_STATUS_OK)
    {
        xnOSCloseFile(&UploadFile);
        return (rc);
    }

    XnUInt64 nNow;
    xnOSGetTimeStamp(&nNow);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Initialized upload of %llu bytes in %llu ms", nFileSize, nNow - nBefore);

    xnOSGetTimeStamp(&nBefore);

    XnUInt32 nLastPrintBytes = 0;
    while (nNextOffset*sizeof(XnUInt16) < nFileSize)
    {
        while ((nNextOffset*sizeof(XnUInt16) - nLastPrintBytes) > 5000)
        {
            printf(".");
            nLastPrintBytes += 5000;
        }

        rc = XnHostProtocolWriteUpload(pDevicePrivateData, UploadFile, nNextOffset, (XnUInt32)nFileSize, nNextOffset);
        if (rc != XN_STATUS_OK)
        {
            xnOSCloseFile(&UploadFile);
            return (rc);
        }
    }
    printf("\n");

    xnOSGetTimeStamp(&nNow);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Uploaded %llu bytes in %llu ms", nFileSize, nNow - nBefore);

    rc = XnHostProtocolFinishUpload(pDevicePrivateData);
    if (rc != XN_STATUS_OK)
    {
        xnOSCloseFile(&UploadFile);
        return (rc);
    }

    xnOSCloseFile(&UploadFile);

    return rc;
}

XnStatus XnHostProtocolDeleteFile(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileId)
{
    XnStatus rc = XN_STATUS_OK;

    if (pDevicePrivateData->FWInfo.bHasFilesystemLock)
    {
        rc = XnHostProtocolSetParam(pDevicePrivateData, PARAM_FILE_SYSTEM_LOCK, 0);
        if (rc != XN_STATUS_OK)
        {
            xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Failed to unlock file system: %s", xnGetStatusString(rc));
            return rc;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nFileId);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeDeleteFile);

    XnUInt16 nDataSize;

    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeDeleteFile,
        NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolSetFileAttributes(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileId, XnUInt16 nAttributes)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nFileId);
    *(((XnUInt16*)pDataBuf) + 1) = XN_PREPARE_VAR16_IN_BUFFER(nAttributes);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 2 * sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + 2 * sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeSetFileAttribute,
        NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolExecuteFile(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileId)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nFileId);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeExecuteFile);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeExecuteFile,
        NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolGetFlashMap(XnDevicePrivateData* pDevicePrivateData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetFlashMap);

    XnUInt nRead;
    XnUChar* pRelevantBuffer;
    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetFlashMap,
        &pRelevantBuffer, nDataSize);
    if (rc != XN_STATUS_OK)
    {
        //		printf("Get Flash MapExecution failed\n");
        return rc;
    }

    // Deal with specific log reply
#pragma pack(push, 1)
    typedef struct
    {
        XnUInt16 nFileType;
        XnUInt32 nOffsetInFlash;
        XnUInt32 nSizeInWords;
        struct
        {
            XnUInt8 nMajor;
            XnUInt8 nMinor;
            XnUInt16 nBuild;
        } Version;
    } XnFlashEntry;
#pragma pack (pop)

    nRead = pDevicePrivateData->FWInfo.nProtocolHeaderSize + nDataSize * 2;
    XnFlashEntry* pFlashEntry;

    // Go over log and print it out
    while (pRelevantBuffer < buffer + nRead)
    {
        pFlashEntry = (XnFlashEntry*)pRelevantBuffer;

        pFlashEntry->nFileType = XN_PREPARE_VAR16_IN_BUFFER(pFlashEntry->nFileType);
        pFlashEntry->nOffsetInFlash = XN_PREPARE_VAR32_IN_BUFFER(pFlashEntry->nOffsetInFlash);
        pFlashEntry->nSizeInWords = XN_PREPARE_VAR32_IN_BUFFER(pFlashEntry->nSizeInWords);
        pFlashEntry->Version.nBuild = XN_PREPARE_VAR16_IN_BUFFER(pFlashEntry->Version.nBuild);

        printf("File Type: %d\n", pFlashEntry->nFileType);
        printf("Offset: %u\n", pFlashEntry->nOffsetInFlash);
        printf("Size in Words: %u\n", pFlashEntry->nSizeInWords);
        printf("Version: %d.%d.%d\n", pFlashEntry->Version.nMajor, pFlashEntry->Version.nMinor,
            pFlashEntry->Version.nBuild);

        pRelevantBuffer += sizeof(XnFlashEntry);
    }

    return XN_STATUS_OK;
}

#pragma pack (push, 1)
typedef struct XnAlgorithmParamRequest
{
    XnUInt16 nParamID;
    XnUInt16 nFormat;
    XnUInt16 nResolution;
    XnUInt16 nFPS;
    XnUInt16 nOffset;
} XnAlgorithmParamRequest;

typedef struct XnAlgorithmParamRequestV4
{
    XnUInt8 nResolution;
    XnUInt8 nFPS;
    XnUInt8 nFormat;
    XnUInt8 nParamID;
    XnUInt16 nOffset;
} XnAlgorithmParamRequestV4;
#pragma pack (pop)

XnStatus XnHostProtocolAlgorithmParams(XnDevicePrivateData* pDevicePrivateData,
    XnHostProtocolAlgorithmType eAlgorithmType,
    void* pAlgorithmInformation, XnUInt16 nAlgInfoSize, XnResolutions nResolution, XnUInt16 nFPS)
{
    XnChar* pData = (XnChar*)pAlgorithmInformation;
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUChar* pRelevantBuffer;

    XnInt16 nDataRead = 0;
    XnUInt16 nRequestSize = 0;

    if (eAlgorithmType == XN_HOST_PROTOCOL_ALGORITHM_DEVICE_INFO &&
        !pDevicePrivateData->FWInfo.bDeviceInfoSupported)
    {
        XnDeviceInformation* pDeviceInfo = (XnDeviceInformation*)pAlgorithmInformation;
        strcpy(pDeviceInfo->strDeviceName, "PrimeSense Sensor");
        strcpy(pDeviceInfo->strVendorData, "");
        return XN_STATUS_OK;
    }

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting algorithm params 0x%x for resolution %d and fps %d....", eAlgorithmType, nResolution, nFPS);

    XnStatus rc;
    while (nDataRead < nAlgInfoSize)
    {
        if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_5_1)
        {
            XnAlgorithmParamRequest* pRequest = (XnAlgorithmParamRequest*)pDataBuf;
            pRequest->nParamID = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)eAlgorithmType);
            pRequest->nFormat = 0;
            pRequest->nResolution = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nResolution);
            pRequest->nFPS = XN_PREPARE_VAR16_IN_BUFFER(nFPS);
            pRequest->nOffset = XN_PREPARE_VAR16_IN_BUFFER(nDataRead / sizeof(XnUInt16));
            nRequestSize = sizeof(XnAlgorithmParamRequest);
        }
        else
        {
            XnAlgorithmParamRequestV4* pRequest = (XnAlgorithmParamRequestV4*)pDataBuf;
            pRequest->nParamID = (XnUInt8)eAlgorithmType;
            pRequest->nFormat = 0;
            pRequest->nResolution = (XnUInt8)nResolution;
            pRequest->nFPS = 0;
            pRequest->nOffset = XN_PREPARE_VAR16_IN_BUFFER(nDataRead / sizeof(XnUInt16));
            nRequestSize = sizeof(XnAlgorithmParamRequestV4);
        }

        XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeAlgorithmParams);

        XnUInt16 nDataSize;
        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize, pDevicePrivateData->FWInfo.nOpcodeAlgorithmParams,
            &pRelevantBuffer, nDataSize);


        if (rc != XN_STATUS_OK)
            return rc;

        XnUInt16 nReadNow = (XnUInt16)(nDataSize*sizeof(XnUInt16));
        if (nReadNow == 0)
        {
            break;
        }

        xnOSMemCopy(pData + nDataRead, pRelevantBuffer, nReadNow);

        nDataRead += nReadNow;
    }

    if (nDataRead != nAlgInfoSize)
    {
        XN_LOG_WARNING_RETURN(XN_STATUS_IO_DEVICE_INVALID_RESPONSE_SIZE, XN_MASK_SENSOR_PROTOCOL, "Failed getting algorithm params: expected %u bytes, but got only %u", nAlgInfoSize, nDataRead);
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolTakeSnapshot(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOS)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nCMOS);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeTakeSnapshot);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeTakeSnapshot,
        NULL, nDataSize);

    return rc;
}

XnStatus XnHostProtocolGetFileList(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFirstFileId, XnFlashFile* pFileList, XnUInt16& nNumOfEntries)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUChar* pRelevantBuffer;
    XnUInt32 nBytesRead = 0;
    XnBool bDone = false;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting file list");

    for (;;)
    {
        *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nFirstFileId);

        XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetFileList);

        XnUInt16 nDataSize;

        XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetFileList,
            &pRelevantBuffer, nDataSize);
        if (rc != XN_STATUS_OK)
        {
            return rc;
        }

        XnUInt32 DataSizeInBytes = nDataSize*sizeof(XnUInt16);

        if (DataSizeInBytes == 0)
        {
            // Done
            break;
        }
        if (nBytesRead + DataSizeInBytes > nNumOfEntries*sizeof(XnFlashFile))
        {
            DataSizeInBytes = nNumOfEntries*sizeof(XnFlashFile) - nBytesRead;
            bDone = true;
        }

        xnOSMemCopy(((XnChar*)pFileList) + nBytesRead, pRelevantBuffer, DataSizeInBytes);

        nBytesRead += DataSizeInBytes;
        nFirstFileId = XN_PREPARE_VAR16_IN_BUFFER(pFileList[nBytesRead / sizeof(XnFlashFile) - 1].nId) + 1;

        if (bDone)
            break;
    }

    nNumOfEntries = (XnUInt16)(nBytesRead / sizeof(XnFlashFile));

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolFileDownloadChunk(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileType,
    XnUInt32 nOffset, XnChar* pData, XnUInt16& nChunkSize)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUChar* pRelevantBuffer;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nFileType);
    *(XnUInt32*)((((XnUInt16*)pDataBuf) + 1)) = XN_PREPARE_VAR32_IN_BUFFER(XnUInt32(nOffset / sizeof(XnUInt16)));

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16) + sizeof(XnUInt32), pDevicePrivateData->FWInfo.nOpcodeDownloadFile);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16) + sizeof(XnUInt32), pDevicePrivateData->FWInfo.nOpcodeDownloadFile,
        &pRelevantBuffer, nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nChunkSize < nDataSize*sizeof(XnUInt16))
    {
        // received too much.
        return XN_STATUS_INTERNAL_BUFFER_TOO_SMALL;
    }

    nChunkSize = nDataSize*sizeof(XnUInt16);

    xnOSMemCopy(pData, pRelevantBuffer, nChunkSize);

    return XN_STATUS_OK;

}

XnStatus XnHostProtocolFileDownload(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nFileType,
    const XnChar* strFileName)
{
    XN_FILE_HANDLE File;
    XnStatus rc = XN_STATUS_OK;

    rc = xnOSOpenFile(strFileName, XN_OS_FILE_WRITE | XN_OS_FILE_TRUNCATE, &File);
    XN_IS_STATUS_OK(rc);

    XnChar Buffer[MAX_PACKET_SIZE];
    XnUInt16 nChunkSize = 0;
    XnUInt32 nOffset = 0;

    XnUInt32 nLastPrintBytes = 0;

    for (;;)
    {
        while ((nOffset - nLastPrintBytes) > 5000)
        {
            printf(".");
            nLastPrintBytes += 5000;
        }

        nChunkSize = MAX_PACKET_SIZE;

        rc = XnHostProtocolFileDownloadChunk(pDevicePrivateData, nFileType, nOffset, Buffer, nChunkSize);

        if (rc != XN_STATUS_OK || nChunkSize == 0)
        {
            break;
        }

        rc = xnOSWriteFile(File, Buffer, nChunkSize);
        if (rc != XN_STATUS_OK)
            break;
        nOffset += nChunkSize;
    }

    printf("\n");

    xnOSCloseFile(&File);

    return rc;
}

#define XN_HOST_PROTOCOL_INIT_BUFFER(pBuffer)	\
	XnUChar* __pBuffer = (XnUChar*)pBuffer;	\
	XnUInt16 __nBufferSize = 0;

#define XN_HOST_PROTOCOL_APPEND_PARAM(type, param)	\
	*(type*)__pBuffer = (type)param;				\
	__pBuffer += sizeof(type);						\
	__nBufferSize += sizeof(type);

#define XN_HOST_PROTOCOL_SIZE	__nBufferSize

XnStatus XnHostProtocolReadFlashChunk(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUChar* pData, XnUInt16* nChunkSize)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUChar* pRelevantBuffer;

    XN_HOST_PROTOCOL_INIT_BUFFER(pDataBuf);
    XN_HOST_PROTOCOL_APPEND_PARAM(XnUInt32, XN_PREPARE_VAR32_IN_BUFFER(nOffset));
    XN_HOST_PROTOCOL_APPEND_PARAM(XnUInt16, XN_PREPARE_VAR16_IN_BUFFER(*nChunkSize));

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, XN_HOST_PROTOCOL_SIZE, pDevicePrivateData->FWInfo.nOpcodeReadFlash);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + XN_HOST_PROTOCOL_SIZE, pDevicePrivateData->FWInfo.nOpcodeReadFlash,
        &pRelevantBuffer, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    // words to bytes
    if (*nChunkSize < nDataSize)
    {
        // received too much.
        return XN_STATUS_INTERNAL_BUFFER_TOO_SMALL;
    }

    *nChunkSize = nDataSize;

    // size is in words
    xnOSMemCopy(pData, pRelevantBuffer, nDataSize*sizeof(XnUInt16));

    return XN_STATUS_OK;

}

XnStatus XnHostProtocolReadFlash(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt32 nSize, XnUChar* pBuffer)
{
    XnStatus rc = XN_STATUS_OK;

    XnUInt32 nLoopOffset = nOffset;
    XnUInt16 nLoopChunkSize;
    XnUInt32 nReadSize = 0;

    XnUInt32 counter = 0;
    while (nReadSize < nSize)
    {
        if (counter % 100 == 0)
        {
            printf(".");
        }
        counter++;

        // don't ask for more than MAX UINT16
        nLoopChunkSize = (XnUInt16)XN_MIN(nSize - nReadSize, 0xFFFF);

        rc = XnHostProtocolReadFlashChunk(pDevicePrivateData, nLoopOffset, pBuffer + nReadSize*sizeof(XnUInt16), &nLoopChunkSize);

        if (rc != XN_STATUS_OK)
            return rc;

        if (nLoopChunkSize == 0)
            return XN_STATUS_ERROR;

        nLoopOffset += nLoopChunkSize;
        nReadSize += nLoopChunkSize;
    }

    printf("\n");

    return rc;
}

XnStatus XnHostProtocolRunBIST(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nTestsMask, XnUInt32* pnFailures)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32* pRelevantBuffer;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nTestsMask);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeBIST);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeBIST,
        (XnUChar**)&pRelevantBuffer, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    // the UINT32 received has a bit turned on for each failed module, so if all are off, everything is OK.
    *pnFailures = (*pRelevantBuffer);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetCPUStats(XnDevicePrivateData* pDevicePrivateData, XnTaskCPUInfo* pTasks, XnUInt32 *pnTimesCount)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt32* pRelevantBuffer;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetCPUStats);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetCPUStats,
        (XnUChar**)&pRelevantBuffer, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    // check how many numbers we got
    XnUInt32 nCount = nDataSize * sizeof(XnUInt16) / sizeof(XnTaskCPUInfo);

    // check if we have enough space in buffer
    if (nCount > *pnTimesCount)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "CPUStats: no space in buffer for all tasks. Dropping last %d", nCount - *pnTimesCount);
        nCount = *pnTimesCount;
    }

    xnOSMemCopy(pTasks, pRelevantBuffer, nCount * sizeof(XnTaskCPUInfo));
    for (XnUInt32 i = 0; i < nCount; i++)
    {
        pTasks[i].nTimesExecuted = XN_PREPARE_VAR32_IN_BUFFER(pTasks[i].nTimesExecuted);
        pTasks[i].nTimeInMicroSeconds = XN_PREPARE_VAR32_IN_BUFFER(pTasks[i].nTimeInMicroSeconds);
    }

    *pnTimesCount = nCount;

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetAudioSampleRate(XnDevicePrivateData* pDevicePrivateData, XnSampleRate nSampleRate)
{
    EA2d_SampleRate nSample;

    switch (nSampleRate)
    {
    case XN_SAMPLE_RATE_8K:
        nSample = A2D_SAMPLE_RATE_8KHZ;
        break;
    case XN_SAMPLE_RATE_11K:
        nSample = A2D_SAMPLE_RATE_11KHZ;
        break;
    case XN_SAMPLE_RATE_12K:
        nSample = A2D_SAMPLE_RATE_12KHZ;
        break;
    case XN_SAMPLE_RATE_16K:
        nSample = A2D_SAMPLE_RATE_16KHZ;
        break;
    case XN_SAMPLE_RATE_22K:
        nSample = A2D_SAMPLE_RATE_22KHZ;
        break;
    case XN_SAMPLE_RATE_24K:
        nSample = A2D_SAMPLE_RATE_24KHZ;
        break;
    case XN_SAMPLE_RATE_32K:
        nSample = A2D_SAMPLE_RATE_32KHZ;
        break;
    case XN_SAMPLE_RATE_44K:
        nSample = A2D_SAMPLE_RATE_44KHZ;
        break;
    case XN_SAMPLE_RATE_48K:
        nSample = A2D_SAMPLE_RATE_48KHZ;
        break;
    default:
        return XN_STATUS_DEVICE_UNSUPPORTED_MODE;
    }

    return XnHostProtocolSetParam(pDevicePrivateData, PARAM_AUDIO_SAMPLE_RATE, (XnUInt16)nSample);
}

XnStatus XnHostProtocolGetAudioSampleRate(XnDevicePrivateData* pDevicePrivateData, XnSampleRate* pSampleRate)
{
    XnUInt16 nValue;
    XnHostProtocolGetParam(pDevicePrivateData, PARAM_AUDIO_SAMPLE_RATE, nValue);
    XnSampleRate nSample;

    switch (nValue)
    {
    case A2D_SAMPLE_RATE_8KHZ:
        nSample = XN_SAMPLE_RATE_8K;
        break;
    case A2D_SAMPLE_RATE_11KHZ:
        nSample = XN_SAMPLE_RATE_11K;
        break;
    case A2D_SAMPLE_RATE_12KHZ:
        nSample = XN_SAMPLE_RATE_12K;
        break;
    case A2D_SAMPLE_RATE_16KHZ:
        nSample = XN_SAMPLE_RATE_16K;
        break;
    case A2D_SAMPLE_RATE_22KHZ:
        nSample = XN_SAMPLE_RATE_22K;
        break;
    case A2D_SAMPLE_RATE_24KHZ:
        nSample = XN_SAMPLE_RATE_24K;
        break;
    case A2D_SAMPLE_RATE_32KHZ:
        nSample = XN_SAMPLE_RATE_32K;
        break;
    case A2D_SAMPLE_RATE_44KHZ:
        nSample = XN_SAMPLE_RATE_44K;
        break;
    case A2D_SAMPLE_RATE_48KHZ:
        nSample = XN_SAMPLE_RATE_48K;
        break;
    default:
        return XN_STATUS_DEVICE_UNSUPPORTED_MODE;
    }

    *pSampleRate = nSample;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetMultipleParams(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nNumOfParams, XnInnerParamData* anParams)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt16* pCurData = (XnUInt16*)pDataBuf;
    for (XnUInt16 nIndex = 0; nIndex < nNumOfParams; ++nIndex)
    {
        *pCurData++ = XN_PREPARE_VAR16_IN_BUFFER(anParams[nIndex].nParam);
        *pCurData++ = XN_PREPARE_VAR16_IN_BUFFER(anParams[nIndex].nValue);
    }

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16)*nNumOfParams * 2, pDevicePrivateData->FWInfo.nOpcodeSetParam);

    XnUInt16 nDataSize;

    XnInt32 nTimesLeft = 5;
    XnStatus rc = XN_STATUS_ERROR;
    while (rc != XN_STATUS_OK && rc != XN_STATUS_DEVICE_PROTOCOL_BAD_PARAMS &&
        rc != XN_STATUS_DEVICE_PROTOCOL_INVALID_COMMAND && nTimesLeft > 0)
    {
        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16)*nNumOfParams * 2, pDevicePrivateData->FWInfo.nOpcodeSetParam,
            NULL, nDataSize);
        nTimesLeft--;
    }

    if (rc != XN_STATUS_OK)
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed: %s", xnGetStatusString(rc));

    return rc;
}

#pragma pack (push, 1)
typedef struct
{
    XnUInt16 nSetPoint;
} XnCalibrateTecRequest;
#pragma pack (pop)

XnStatus XnHostProtocolCalibrateTec(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nSetPoint)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Calibrating TEC. Set Point: %d", nSetPoint);

    XnCalibrateTecRequest* pRequest = (XnCalibrateTecRequest*)pDataBuf;
    pRequest->nSetPoint = XN_PREPARE_VAR16_IN_BUFFER(nSetPoint);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnCalibrateTecRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateTec);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnCalibrateTecRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateTec,
        NULL, nDataSize);

    if (rc != XN_STATUS_OK)
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed Calibrating TEC: %s", xnGetStatusString(rc));
    else
        xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Calibrating TEC succeeded.");

    return rc;
}

XnStatus XnHostProtocolGetTecData(XnDevicePrivateData* pDevicePrivateData, XnTecData* pTecData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnStatus rc;

    if (pDevicePrivateData->FWInfo.nFWVer < XN_SENSOR_FW_VER_5_4)
    {
        xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting TEC data...");

        XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetTecData);

        XnTecData* pResult;

        rc = XnHostProtocolExecute(pDevicePrivateData,
            buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetTecData,
            (XnUChar**)(&pResult), nDataSize);

        XN_IS_STATUS_OK(rc);

        pTecData->m_SetPointVoltage = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_SetPointVoltage);
        pTecData->m_CompensationVoltage = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_CompensationVoltage);
        pTecData->m_TecDutyCycle = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_TecDutyCycle);
        pTecData->m_HeatMode = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_HeatMode);
        pTecData->m_ProportionalError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_ProportionalError);
        pTecData->m_IntegralError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_IntegralError);
        pTecData->m_DerivativeError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_DerivativeError);
        pTecData->m_ScanMode = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_ScanMode);
    }
    else
    {
        XnTecFastConvergenceData TecFastConvergenceData;

        rc = XnHostProtocolGetTecFastConvergenceData(pDevicePrivateData, &TecFastConvergenceData);
        XN_IS_STATUS_OK(rc);

        pTecData->m_SetPointVoltage = 0;
        pTecData->m_CompensationVoltage = 0;
        pTecData->m_TecDutyCycle = TecFastConvergenceData.m_TecDutyCycle;
        pTecData->m_HeatMode = TecFastConvergenceData.m_HeatMode;
        pTecData->m_ProportionalError = TecFastConvergenceData.m_ProportionalError;
        pTecData->m_IntegralError = TecFastConvergenceData.m_IntegralError;
        pTecData->m_DerivativeError = TecFastConvergenceData.m_DerivativeError;

        // Convert the new modes (post FW 5.4) into the old modes (pre 5.4).
        pTecData->m_ScanMode = (TecFastConvergenceData.m_ScanMode) - 1;
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetTecFastConvergenceData(XnDevicePrivateData* pDevicePrivateData, XnTecFastConvergenceData* pTecData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting TEC Fast Convergence data...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC);

    XnTecFastConvergenceData* pResult;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetFastConvergenceTEC,
        (XnUChar**)(&pResult), nDataSize);

    XN_IS_STATUS_OK(rc);

    pTecData->m_SetPointTemperature = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_SetPointTemperature);
    pTecData->m_MeasuredTemperature = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_MeasuredTemperature);
    pTecData->m_ProportionalError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_ProportionalError);
    pTecData->m_IntegralError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_IntegralError);
    pTecData->m_DerivativeError = XN_PREPARE_VAR32_IN_BUFFER(pResult->m_DerivativeError);
    pTecData->m_ScanMode = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_ScanMode);
    pTecData->m_HeatMode = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_HeatMode);
    pTecData->m_TecDutyCycle = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_TecDutyCycle);
    pTecData->m_TemperatureRange = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_TemperatureRange);

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)
typedef struct
{
    XnUInt16 nSetPoint;
} XnCalibrateEmitterRequest;
#pragma pack (pop)

XnStatus XnHostProtocolCalibrateEmitter(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nSetPoint)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Calibrating Emitter. Set Point: %d", nSetPoint);

    XnCalibrateEmitterRequest* pRequest = (XnCalibrateEmitterRequest*)pDataBuf;
    pRequest->nSetPoint = XN_PREPARE_VAR16_IN_BUFFER(nSetPoint);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnCalibrateEmitterRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnCalibrateEmitterRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateEmitter,
        NULL, nDataSize);

    if (rc != XN_STATUS_OK)
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed Calibrating Emitter: %s", xnGetStatusString(rc));
    else
        xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Calibrating Emitter succeeded.");

    return rc;
}

XnStatus XnHostProtocolGetEmitterData(XnDevicePrivateData* pDevicePrivateData, XnEmitterData* pEmitterData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting Emitter data...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetEmitterData);

    XnEmitterData* pResult;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetEmitterData,
        (XnUChar**)(&pResult), nDataSize);

    XN_IS_STATUS_OK(rc);

    pEmitterData->m_State = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_State);
    pEmitterData->m_SetPointVoltage = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_SetPointVoltage);
    pEmitterData->m_SetPointClocks = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_SetPointClocks);
    pEmitterData->m_PD_Reading = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_PD_Reading);
    pEmitterData->m_EmitterSet = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_EmitterSet);
    pEmitterData->m_EmitterSettingLogic = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_EmitterSettingLogic);
    pEmitterData->m_LightMeasureLogic = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_LightMeasureLogic);
    pEmitterData->m_IsAPCEnabled = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_IsAPCEnabled);

    // set some version specific fields
    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_5_1)
    {
        pEmitterData->m_EmitterSetStepSize = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_EmitterSetStepSize);
        pEmitterData->m_ApcTolerance = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_ApcTolerance);
    }
    else
    {
        pEmitterData->m_EmitterSetStepSize = 0;
        pEmitterData->m_ApcTolerance = 0;
    }

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_5_3)
    {
        pEmitterData->m_SubClocking = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_SubClocking);
        pEmitterData->m_Precision = XN_PREPARE_VAR16_IN_BUFFER(pResult->m_Precision);
    }
    else
    {
        pEmitterData->m_SubClocking = 0;
        pEmitterData->m_Precision = 0;
    }

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)
typedef struct
{
    XnUInt16 nMinThreshold;
    XnUInt16 nMaxThreshold;
} XnProjectorFaultRequest;
#pragma pack (pop)

XnStatus XnHostProtocolCalibrateProjectorFault(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nMinThreshold, XnUInt16 nMaxThreshold, XnBool* pbProjectorFaultEvent)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 nDataSize;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Testing Projector Fault. Min Threshold: %u, Max Threshold: %u...", nMinThreshold, nMaxThreshold);

    XnProjectorFaultRequest* pRequest = (XnProjectorFaultRequest*)pDataBuf;
    pRequest->nMinThreshold = XN_PREPARE_VAR16_IN_BUFFER(nMinThreshold);
    pRequest->nMaxThreshold = XN_PREPARE_VAR16_IN_BUFFER(nMaxThreshold);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnProjectorFaultRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault);

    XnBool* pResult;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnProjectorFaultRequest), pDevicePrivateData->FWInfo.nOpcodeCalibrateProjectorFault,
        (XnUChar**)(&pResult), nDataSize);

    XN_IS_STATUS_OK(rc);

    *pbProjectorFaultEvent = *pResult;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Projector fault event: %d", *pResult);

    return (XN_STATUS_OK);
}

XnStatus XnDeviceSensorGetDepthAGCParams(XnUInt16 nBin, XnUInt16* pnMinParam, XnUInt16* pnMaxParam)
{
    switch (nBin)
    {
    case 0:
        *pnMinParam = PARAM_DEPTH_AGC_BIN0_LOW;
        *pnMaxParam = PARAM_DEPTH_AGC_BIN0_HIGH;
        break;
    case 1:
        *pnMinParam = PARAM_DEPTH_AGC_BIN1_LOW;
        *pnMaxParam = PARAM_DEPTH_AGC_BIN1_HIGH;
        break;
    case 2:
        *pnMinParam = PARAM_DEPTH_AGC_BIN2_LOW;
        *pnMaxParam = PARAM_DEPTH_AGC_BIN2_HIGH;
        break;
    case 3:
        *pnMinParam = PARAM_DEPTH_AGC_BIN3_LOW;
        *pnMaxParam = PARAM_DEPTH_AGC_BIN3_HIGH;
        break;
    default:
        return XN_STATUS_DEVICE_BAD_PARAM;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetDepthAGCBin(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nBin, XnUInt16 nMinShift, XnUInt16 nMaxShift)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XnUInt16 nMinParam;
    XnUInt16 nMaxParam;

    nRetVal = XnDeviceSensorGetDepthAGCParams(nBin, &nMinParam, &nMaxParam);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnHostProtocolSetParam(pDevicePrivateData, nMinParam, nMinShift);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnHostProtocolSetParam(pDevicePrivateData, nMaxParam, nMaxShift);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetDepthAGCBin(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nBin, XnUInt16* pnMinShift, XnUInt16* pnMaxShift)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XnUInt16 nMinParam;
    XnUInt16 nMaxParam;

    nRetVal = XnDeviceSensorGetDepthAGCParams(nBin, &nMinParam, &nMaxParam);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnHostProtocolGetParam(pDevicePrivateData, nMinParam, *pnMinShift);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnHostProtocolGetParam(pDevicePrivateData, nMaxParam, *pnMaxShift);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnVSyncRequest
{
    XnUInt16 nUnits;
    XnUInt16 nCmosID;
    XnUInt16 nNumberOfFrames;
} XnVSyncRequest;

#pragma pack (pop)

XnStatus XnHostProtocolSetCmosBlanking(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nUnits, XnCMOSType nCMOSID, XnUInt16 nNumberOfFrames)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    if (pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_5_1)
    {
        XnVSyncRequest* pRequest = (XnVSyncRequest*)pDataBuf;
        pRequest->nUnits = XN_PREPARE_VAR16_IN_BUFFER(nUnits);
        pRequest->nCmosID = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nCMOSID);
        pRequest->nNumberOfFrames = XN_PREPARE_VAR16_IN_BUFFER(nNumberOfFrames);
        nRequestSize = sizeof(XnVSyncRequest);
    }
    else
    {
        XN_LOG_WARNING_RETURN(XN_STATUS_IO_DEVICE_FUNCTION_NOT_SUPPORTED, XN_MASK_SENSOR_PROTOCOL, "Set Blanking is not supported by this firmware!");
    }

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Chaning CMOS %d Blanking to %hd (NumberOfFrames=%hu)...", nCMOSID, nUnits, nNumberOfFrames);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetCmosBlanking,
        NULL, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        XN_LOG_WARNING_RETURN(rc, XN_MASK_SENSOR_PROTOCOL, "Failed changing CMOS %d Blanking to %hd (NumberOfFrames=%hu): %s", nCMOSID, nUnits, nNumberOfFrames, xnGetStatusString(rc));
    }

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnGetCmosBlankingRequest
{
    XnUInt16 nCmosID;
} XnGetCmosBlankingRequest;

typedef struct XnGetCmosBlankingReply
{
    XnUInt32 nUnits;
} XnGetCmosBlankingReply;

#pragma pack (pop)

XnStatus XnHostProtocolGetCmosBlanking(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOSID, XnUInt16* pnLines)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnGetCmosBlankingRequest* pRequest = (XnGetCmosBlankingRequest*)pDataBuf;
    pRequest->nCmosID = (XnUInt16)nCMOSID;

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting Cmos %d VBlanking...", nCMOSID);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnGetCmosBlankingRequest), pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking);

    XnGetCmosBlankingReply* pReply;
    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnGetCmosBlankingRequest), pDevicePrivateData->FWInfo.nOpcodeGetCmosBlanking,
        (XnUChar**)&pReply, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        XN_LOG_WARNING_RETURN(rc, XN_MASK_SENSOR_PROTOCOL, "Failed getting Cmos %d Blanking: %s", nCMOSID, xnGetStatusString(rc));
    }

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Cmos %d VBlanking: %u", nCMOSID, pReply->nUnits);

    *pnLines = (XnUInt16)pReply->nUnits;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetStreamSet(XnDevicePrivateData* pDevicePrivateData)
{
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeStreamSetQuery;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, 0, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || recvSize <= 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to query stream set: error (%s).", xnGetStatusString(rc));
        return rc;
    }

    XnStreamSet* pSet = (XnStreamSet*)pRecv;
    XnUInt32 num = recvSize * 2 / sizeof(XnStreamSet);
    while (pSet < (XnStreamSet*)pRecv + num)
    {
        switch (pSet->type)
        {
        case XN_CMOS_TYPE_PHASE:
            pDevicePrivateData->FWInfo.bPhaseSupported = TRUE;
            break;
        case XN_CMOS_TYPE_AI:
            pDevicePrivateData->FWInfo.bAISupported = TRUE;
            break;
        default:
            break;
        }

        pDevicePrivateData->FWInfo.streamSets.AddLast(*pSet++);
    }

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetCmosPresets(XnDevicePrivateData* pDevicePrivateData, XnCMOSType nCMOSID, XnCmosPreset* aPresets, XnUInt32& nCount)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Reading CMOS %d supported presets...", nCMOSID);

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nCMOSID);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets);

    XnUInt16 nDataSize;
    XnCmosPreset* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + sizeof(XnUInt16), pDevicePrivateData->FWInfo.nOpcodeGetCmosPresets,
        (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getting CMOS %d presets: %s", nCMOSID, xnGetStatusString(rc));
        return rc;
    }

    XnUInt32 nReturnedCount = nDataSize * 2 / sizeof(XnCmosPreset);
    if (nReturnedCount > nCount)
    {
        return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
    }

    XnCmosPreset* pValueEnd = pValue + nReturnedCount;

    nCount = 0;

    while (pValue < pValueEnd)
    {
        // workaround a FW bug - an extra preset arrives with FPS 0. Ignore it.
        if (pValue->nFPS != 0)
        {
            aPresets[nCount].nFormat = XN_PREPARE_VAR16_IN_BUFFER(pValue->nFormat);
            aPresets[nCount].nResolution = XN_PREPARE_VAR16_IN_BUFFER(pValue->nResolution);
            aPresets[nCount].nFPS = XN_PREPARE_VAR16_IN_BUFFER(pValue->nFPS);
            ++nCount;
        }

        ++pValue;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetSerialNumber(XnDevicePrivateData* pDevicePrivateData, XnChar* cpSerialNumber)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Reading sensor serial number...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber);

    XnUInt16 nDataSize;
    XnUChar *serialNumberBuffer = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber,
        (XnUChar**)(&serialNumberBuffer), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getting the sensor serial number: %s", xnGetStatusString(rc));
        return rc;
    }

    serialNumberBuffer[nDataSize * 2] = 0;

    strcpy(cpSerialNumber, (XnChar*)serialNumberBuffer);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetPlatformString(XnDevicePrivateData* pDevicePrivateData, XnChar* cpPlatformString)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    cpPlatformString[0] = '\0';

    if (pDevicePrivateData->FWInfo.nOpcodeGetPlatformString == OPCODE_INVALID)
    {
        // for FW that doesn't support this opcode, we just return an empty string
        return XN_STATUS_OK;
    }

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Reading sensor platform string...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetPlatformString);

    XnUInt16 nDataSize;
    XnChar *platformStringBuffer = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetPlatformString,
        (XnUChar**)(&platformStringBuffer), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getting the sensor platform string: %s", xnGetStatusString(rc));
        return rc;
    }

    XnUInt32 nBufferUsed = 0;
    for (XnUInt32 i = 0; i < (XnUInt32)nDataSize * 2; ++i)
    {
        cpPlatformString[nBufferUsed++] = platformStringBuffer[i * 2];
    }

    cpPlatformString[nBufferUsed++] = '\0';

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetCfgProductNumber(XnDevicePrivateData* pDevicePrivateData, XnChar* cfgProductNumber)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetCfgProductNumber);

    XnUInt16 nDataSize;
    XnUChar *productNumber = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetCfgProductNumber,
        (XnUChar**)(&productNumber), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getting the cfg product number: %s", xnGetStatusString(rc));
        return rc;
    }

    productNumber[nDataSize * 2] = 0;
    strcpy(cfgProductNumber, (XnChar*)productNumber);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocalGetIRSensorModel(XnDevicePrivateData* pDevicePrivateData, XnUInt32& irModel)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetIRSensorModel);

    XnUInt16 nDataSize;
    XnUInt32 *irSensorModel = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetIRSensorModel,
        (XnUChar**)(&irSensorModel), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed get the IR sensor model: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 != 4)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed get the IR sensor model.");
        return XN_STATUS_ERROR;
    }

    irModel = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*irSensorModel);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocalGetRgbSensorModel(XnDevicePrivateData* pDevicePrivateData, XnUInt32& rgbModel)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetRgbSensorModel);

    XnUInt16 nDataSize;
    XnUInt32 *rgbSensorModel = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetRgbSensorModel,
        (XnUChar**)(&rgbSensorModel), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed get the RGB sensor model: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 != 4)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed get the RGB sensor model.");
        return XN_STATUS_ERROR;
    }

    rgbModel = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*rgbSensorModel);
    if (rgbModel == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed get the RGB sensor model, it is a uvc device.");
        return XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetUsbCoreType(XnDevicePrivateData* pDevicePrivateData, XnHostProtocolUsbCore& nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetUsbCore);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetUsbCore,
        (XnUChar**)(&pValue), nDataSize);
    XN_IS_STATUS_OK(rc);

    nValue = (XnHostProtocolUsbCore)XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

#pragma pack (push, 1)

typedef struct XnVSetLedStateRequest
{
    XnUInt16 nLedId;
    XnUInt16 nState;
} XnVSetLedStateRequest;

#pragma pack (pop)

XnStatus XnHostProtocolSetLedState(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nLedId, XnUInt16 nState)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetLedStateRequest* pRequest = (XnVSetLedStateRequest*)pDataBuf;
    pRequest->nLedId = XN_PREPARE_VAR16_IN_BUFFER(nLedId);
    pRequest->nState = XN_PREPARE_VAR16_IN_BUFFER(nState);
    nRequestSize = sizeof(XnVSetLedStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLedState);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLedState,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnVSetEmitterStateRequest
{
    XnUInt16 nActive;
} XnVSetEmitterStateRequest;

#pragma pack (pop)

XnStatus XnHostProtocolSetEmitterState(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetEmitterStateRequest* pRequest = (XnVSetEmitterStateRequest*)pDataBuf;
    pRequest->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);
    nRequestSize = sizeof(XnVSetEmitterStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeEnableEmitter);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeEnableEmitter,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetEmitterState(XnDevicePrivateData* pDevicePrivateData, XnUInt16* pState)
{
    XN_RET_IF_NULL(pState, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Emitter property getting of TX requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = 0;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get emitter state of TX sensor: opcode (%d), %s",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    if (recvSize * 2 != sizeof(XnUInt16))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get emitter state of TX: data size mismatch(%d != %d), opcode (%d).",
            recvSize, sizeof(XnUInt32), opcode);
        return XN_STATUS_ERROR;
    }

    /// All is good.
    *pState = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetIrfloodState(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetEmitterStateRequest* pRequest = (XnVSetEmitterStateRequest*)pDataBuf;
    pRequest->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);
    nRequestSize = sizeof(XnVSetEmitterStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeEnableIrflood);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeEnableIrflood,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

///////////////////////////////////////////////////////////////////////
//mx6000 add
//OB set irgain
XnStatus XnHostProtocolSetIrGain(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_IR_GAIN;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetIrGain);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetIrGain,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//get irgain
XnStatus XnHostProtocolGetIrGain(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_IR_GAIN;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    //*(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetIrGain);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetIrGain,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}


//set ir exp
XnStatus XnHostProtocolSetIrExp(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_IR_EXPOSURE;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetIrExp);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetIrExp,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//get ir exp
XnStatus XnHostProtocolGetIrExp(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_IR_EXPOSURE;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    //*(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetIrExp);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetIrExp,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}


//ado change sensor
XnStatus XnHostProtocolSetChangeSensor(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetEmitterStateRequest* pRequest = (XnVSetEmitterStateRequest*)pDataBuf;
    pRequest->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);
    nRequestSize = sizeof(XnVSetEmitterStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeChangeSensor);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeChangeSensor,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}


//add for mayi verify
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
XnStatus XnHostProtocolSetPublicKey(XnDevicePrivateData* pDevicePrivateData, const OBEccVerify* pPublicKey)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    int nEccVerify = sizeof(OBEccVerify);

    //copy
    //xnOSMemCopy(pDataBuf, (XnUChar*)&pPublicKey, nEccVerify);

    xnOSMemCopy(pDataBuf, (XnUChar*)pPublicKey->Pub_x, 48);
    xnOSMemCopy(pDataBuf + 48, (XnUChar*)pPublicKey->Pub_y, 48);
    xnOSMemCopy(pDataBuf + 48 * 2, (XnUChar*)pPublicKey->Batch_Ver, 12);

    XnUInt16 nOpSize = nEccVerify;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nOpSize, pDevicePrivateData->FWInfo.nOpcodeSetPublicKey);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nOpSize,
        pDevicePrivateData->FWInfo.nOpcodeSetPublicKey, NULL, nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed setPublicKey: %s", xnGetStatusString(rc));
        return rc;
    }



    return XN_STATUS_OK;
}

//get public key and batch version
XnStatus XnHostProtocolGetPublicKey(XnDevicePrivateData* pDevicePrivateData, OBEccVerify* pPublicKey)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetPublicKey);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetPublicKey, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getPublicKey: %s", xnGetStatusString(rc));
        return rc;
    }

    //
    int nEccVerify = sizeof(OBEccVerify);
    if (nEccVerify != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    xnOSMemCopy(pPublicKey->Pub_x, (XnUChar *)pValue, 48);
    xnOSMemCopy(pPublicKey->Pub_y, (XnUChar *)pValue + 48, 48);
    xnOSMemCopy(pPublicKey->Batch_Ver, (XnUChar *)pValue + 48 * 2, 12);
    return XN_STATUS_OK;
}


XnStatus XnHostProtocolSetRSKey(XnDevicePrivateData* pDevicePrivateData, const OBEccRSKey* pRSKey)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    int nRsKey = sizeof(OBEccRSKey);

    //copy
    xnOSMemCopy(pDataBuf, (XnUChar*)pRSKey->R_key, 48);
    xnOSMemCopy(pDataBuf + 48, (XnUChar*)pRSKey->S_key, 48);

    XnUInt16 nOpSize = nRsKey;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nOpSize, pDevicePrivateData->FWInfo.nOpcodeRSKey);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nOpSize,
        pDevicePrivateData->FWInfo.nOpcodeRSKey, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed setRsKey: %s", xnGetStatusString(rc));
        return rc;
    }

    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_IO_DEVICE_RSKEY_VERIFY_FAIL;
    }
}

//
XnStatus XnHostProtocolGetRandomString(XnDevicePrivateData* pDevicePrivateData, OBEccInit* pInitString)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeRandomString);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeRandomString, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed getRandomString: %s", xnGetStatusString(rc));
        return rc;
    }

    //
    int nInitString = sizeof(OBEccInit);
    if (nInitString != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    xnOSMemCopy((XnUChar*)pInitString->RandomStr, (XnUChar *)pValue, 48);
    xnOSMemCopy((XnUChar*)pInitString->Batch_Ver, (XnUChar *)pValue + 48, 12);

    return XN_STATUS_OK;
}

//laser secure
//nValue :1:support ,0:not support
XnStatus XnHostProtocolIsSupportLaserSecure(XnDevicePrivateData* pDevicePrivateData, XnBool &bIsSupport)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeIsSupportLaserSecure);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeIsSupportLaserSecure, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    //
    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bIsSupport = TRUE;
    }
    else
    {
        bIsSupport = FALSE;
    }

    return XN_STATUS_OK;
}

//
XnStatus XnHostProtocolSetLaserSecureStatus(XnDevicePrivateData* pDevicePrivateData, XnBool bStatus)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt16 nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bStatus);
    *(XnUInt16*)pDataBuf = nActive;
    XnUInt32 nRequestSize = sizeof(XnUInt16);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserSecure);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserSecure,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetLaserSecureStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bStatus)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetLaserSecure);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetLaserSecure, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bStatus = TRUE;
    }
    else
    {
        bStatus = FALSE;
    }

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetLaserCurrent(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_LASER_CURRENT;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserCurrent);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserCurrent,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetLaserCurrent(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_LASER_CURRENT;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    //*(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLaserCurrent);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLaserCurrent,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolSoftReset(XnDevicePrivateData* pDevicePrivateData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeSoftReset);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeSoftReset,
        NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//set switch dual left and right ir
XnStatus XnHostProtocolSetSwitchIr(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetEmitterStateRequest* pRequest = (XnVSetEmitterStateRequest*)pDataBuf;
    pRequest->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);
    nRequestSize = sizeof(XnVSetEmitterStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetSwitchIr);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetSwitchIr,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnGetDisparityCoeffRequest
{
    XnUInt16 nSubOpCode;
    XnUInt16 nParam;
} XnGetDisparityCoeffRequest;

typedef struct XnGetDisparityCoeffReply
{
    XnUInt32 nDisparityCoeff;
} XnGetDisparityCoeffReply;

#pragma pack (pop)

//get disparityCoeff
XnStatus XnHostProtocolGetDisparityCoeff(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nParam, XnFloat& fDisparityCoeff)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnGetDisparityCoeffRequest* pRequest = (XnGetDisparityCoeffRequest*)pDataBuf;

    XnUInt16 nOpCode = (XnUInt16)OB_DUAL_PARAM_DISPARTY_COFF;
    pRequest->nSubOpCode = XN_PREPARE_VAR16_IN_BUFFER(nOpCode);
    pRequest->nParam = XN_PREPARE_VAR16_IN_BUFFER(nParam);

    XnUInt16 nRequestSize = sizeof(XnGetDisparityCoeffRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetDisparityCoeff);

    XnUInt16 nDataSize;
    XnGetDisparityCoeffReply *pReply = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeGetDisparityCoeff, (XnUChar**)(&pReply), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    //
    int nSize = sizeof(XnGetDisparityCoeffReply);
    if (nSize != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nDisparityCoeff = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(pReply->nDisparityCoeff);

    //because float to int transfer,firmware x10000
    fDisparityCoeff = (XnFloat)(nDisparityCoeff / 10000.0);


    return XN_STATUS_OK;
}


XnStatus XnHostProtocolSetRgbAeMode(XnDevicePrivateData* pDevicePrivateData, const XnRgbAeMode* pRgbAeMode)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnRgbAeMode* pRequest = (XnRgbAeMode*)pDataBuf;

    pRequest->nAeMode = XN_PREPARE_VAR16_IN_BUFFER(pRgbAeMode->nAeMode);
    pRequest->nAeTarget = XN_PREPARE_VAR16_IN_BUFFER(pRgbAeMode->nAeTarget);

    XnUInt16 nRequestSize = sizeof(XnRgbAeMode);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetRgbAeMode);

    XnUInt16 nDataSize;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeSetRgbAeMode, NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolGetRgbAeMode(XnDevicePrivateData* pDevicePrivateData, XnRgbAeMode* pRgbAeMode)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetRgbAeMode);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetRgbAeMode, (XnUChar**)(&pValue), nDataSize);

    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    //
    int nRgbAeMode = sizeof(XnRgbAeMode);
    if (nRgbAeMode != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt16 nAeMode = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    XnUInt16 nAeTarget = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*(pValue + 2));
    pRgbAeMode->nAeMode = nAeMode;
    pRgbAeMode->nAeTarget = nAeTarget;

    return XN_STATUS_OK;
}


#pragma pack (push, 1)

typedef struct XnGetSubCmdRequest
{
    XnUInt32 replayValue;
} XnGetSubCmdRequest;

#pragma pack (pop)

//get calibration ir temperature
XnStatus XnHostProtocolGetSupportSubCmdValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 subcmd, XnDouble& nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams, subcmd, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)subcmd;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams);

    XnUInt16 nDataSize;
    XnGetSubCmdRequest *pReply = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams, (XnUChar**)(&pReply), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    //
    int nSize = sizeof(XnGetSubCmdRequest);
    if (nSize != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    XnInt32 newValue = (XnInt32)XN_PREPARE_VAR32_IN_BUFFER(pReply->replayValue);

    //because float to int transfer,firmware x1000
    nValue = (XnDouble)(newValue / 1000.0);


    return (XN_STATUS_OK);
}

//set calibration ir temperature
XnStatus XnHostProtocolSetSupportSubCmdValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 subcmd, XnDouble nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams, subcmd, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nValue<(XnDouble)supportSubCmdValue.nMinValue || nValue>(XnDouble)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;


    XnInt32 tempValue = (XnInt32)(nValue * 1000);
    XnUInt32 nSubCmd = (XnUInt32)subcmd;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = tempValue;

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnVSetTempCompStateRequest
{
    XnUInt32 nSubCmd;
    XnUInt32 nActive;
} XnVSetTempCompStateRequest;

#pragma pack (pop)

//Temperature compensation switch setting (0 :disable,1:enable)
XnStatus XnHostProtocolTemperatureCompSwitch(XnDevicePrivateData* pDevicePrivateData, XnBool bActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams,
        (XnUInt32)TEMPERATURE_DEFL_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetTempCompStateRequest* pRequest = (XnVSetTempCompStateRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(TEMPERATURE_DEFL_ENABLE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)bActive);
    nRequestSize = sizeof(XnVSetTempCompStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeWriteSubCmdParams,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//Temperature compensation status get(0 :disable,1:enable)
XnStatus XnHostProtocolGetTemperatureCompStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams,
        (XnUInt32)TEMPERATURE_DEFL_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(TEMPERATURE_DEFL_ENABLE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeReadSubCmdParams, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    //
    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }


    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnVSetDepthOptimStateRequest
{
    XnUInt32 nSubCmd;
    XnUInt32 nActive;
} XnVSetDepthOptimStateRequest;

#pragma pack (pop)

//Depth optimization enable switch(0 :disable,1:enable)
XnStatus XnHostProtocolDepthOptimSwitch(XnDevicePrivateData* pDevicePrivateData, XnBool bActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        (XnUInt32)DEPTH_OPTIMIZATION_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetDepthOptimStateRequest* pRequest = (XnVSetDepthOptimStateRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(DEPTH_OPTIMIZATION_ENABLE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)bActive);
    nRequestSize = sizeof(XnVSetDepthOptimStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//Depth optimization enable get(0 :disable,1:enable)
XnStatus XnHostProtocolGetDepthOptimStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive){
    bActive = FALSE;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams,
        (XnUInt32)DEPTH_OPTIMIZATION_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(DEPTH_OPTIMIZATION_ENABLE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    //
    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }


    return (XN_STATUS_OK);
}

#pragma pack (push, 1)

typedef struct XnSupportSubCmdRequest
{
    XnUInt32 opcode;
    XnUInt32 subCmd;
} XnSupportSubCmdRequest;

#pragma pack (pop)

XnStatus XnHostProtocolSupportSubCmdMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16 opcode, XnUInt32 subCmd, XnSupportSubCmdValue* supportValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnSupportSubCmdRequest* pRequest = (XnSupportSubCmdRequest*)pDataBuf;
    pRequest->opcode = XN_PREPARE_VAR32_IN_BUFFER(opcode);
    pRequest->subCmd = XN_PREPARE_VAR32_IN_BUFFER(subCmd);

    XnUInt16 nRequestSize = sizeof(XnSupportSubCmdRequest);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, (XnUInt16)OB_OPCODE_SUPPORT_SUB_CMD);

    XnUInt16 nDataSize;
    XnSupportSubCmdValue *pReply = NULL;
    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        (XnUInt16)OB_OPCODE_SUPPORT_SUB_CMD, (XnUChar**)(&pReply), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    int nSize = sizeof(XnSupportSubCmdValue);
    if (nSize != nDataSize * 2)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol support sub cmd datasize error!");
        return XN_STATUS_ERROR;
    }

    supportValue->nSupportRead = pReply->nSupportRead;
    supportValue->nSupportWrite = pReply->nSupportWrite;
    supportValue->nSupportRange = pReply->nSupportRange;
    supportValue->nPlaceholder = pReply->nPlaceholder;
    supportValue->nMinValue = pReply->nMinValue;
    supportValue->nMaxValue = pReply->nMaxValue;
    supportValue->nReservered1 = pReply->nReservered1;

    return (XN_STATUS_OK);
}


//Get optimization param
XnStatus XnHostProtocolGetDepthOptimizationParam(XnDevicePrivateData* pDevicePrivateData, XnDepthOptimizationParam* depthOptimParam)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams,
        (XnUInt32)DEPTH_OPTIMIZATION_PARAM, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)DEPTH_OPTIMIZATION_PARAM;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams);

    XnUInt16 nDataSize;
    XnDepthOptimizationParam *pReply = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams, (XnUChar**)(&pReply), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    //
    int nSize = sizeof(XnDepthOptimizationParam);
    if (nSize != nDataSize * 2)
    {
        return XN_STATUS_ERROR;
    }

    depthOptimParam->nParam1 = (XnDouble)(pReply->nParam1);
    depthOptimParam->nParam2 = (XnDouble)(pReply->nParam2);
    depthOptimParam->nParam3 = (XnDouble)(pReply->nParam3);

    return XN_STATUS_OK;
}

//Set optimization param
XnStatus XnHostProtocolSetDepthOptimizationParam(XnDevicePrivateData* pDevicePrivateData, const XnDepthOptimizationParam* depthOptimParam)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        (XnUInt32)DEPTH_OPTIMIZATION_PARAM, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange != 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "The value exceeds the subcommand range !");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;


    XnUInt32 nSubCmd = (XnUInt32)DEPTH_OPTIMIZATION_PARAM;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);

    XnDepthOptimizationParam* tempDepthOptimParam = (XnDepthOptimizationParam*)(pDataBuf + 4);
    tempDepthOptimParam->nParam1 = (XnDouble)(depthOptimParam->nParam1);
    tempDepthOptimParam->nParam2 = (XnDouble)(depthOptimParam->nParam2);
    tempDepthOptimParam->nParam3 = (XnDouble)(depthOptimParam->nParam3);

    XnUInt16 nRequestSize = sizeof(XnUInt32) + sizeof(XnDepthOptimizationParam);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);


    return (XN_STATUS_OK);

}
#pragma pack (push, 1)

typedef struct XnObSubParamRequest
{
    XnUInt32 nSubCmd;
    XnUInt32 nActive;
} XnObSubParamRequest;

#pragma pack (pop)

//ldp enable set
XnStatus XnHostProtocolSetLdpEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnObSubParamRequest* pRequest = (XnObSubParamRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(OB_PARAM_LDP_ENABLE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)bActive);
    XnUInt32 nRequestSize = sizeof(XnObSubParamRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnable);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnable,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//get ldp enable
XnStatus XnHostProtocolGetLdpEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(OB_PARAM_LDP_ENABLE);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnable);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnable,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return (XN_STATUS_OK);
}

//
XnStatus XnHostProtocolGetEmitterEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(OB_PARAM_LASER_STATUS);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnable);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnable,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize == 0)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return (XN_STATUS_OK);
}

//dabai ldp
//set ldp enable(0 :disable,1:enable)
XnStatus XnHostProtocolSetLdpEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool bActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        (XnUInt32)LDP_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnObSubParamRequest* pRequest = (XnObSubParamRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(LDP_ENABLE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)bActive);
    nRequestSize = sizeof(XnObSubParamRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

//get ldp enable
XnStatus XnHostProtocolGetLdpEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_ENABLE);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolGetLdpScaleV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nScale)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_SCALE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_SCALE);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);
    nScale = nValue;
    /*
        if (nValue == 0x01)
        {
        nScale = TRUE;
        }
        else
        {
        nScale = FALSE;
        }
        */
    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetLdpScaleV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nScale){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        (XnUInt32)LDP_SCALE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    //
    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nScale< (XnUInt32)supportSubCmdValue.nMinValue || nScale>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnObSubParamRequest* pRequest = (XnObSubParamRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(LDP_SCALE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)nScale);
    nRequestSize = sizeof(XnObSubParamRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolGetLdpStatusV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_STATUS, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_STATUS);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetLdpThresUpV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_THRES_UP, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_THRES_UP);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);
    value = nValue;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetLdpThresUpV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 value){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        (XnUInt32)LDP_THRES_UP, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    //
    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (value< (XnUInt32)supportSubCmdValue.nMinValue || value>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnObSubParamRequest* pRequest = (XnObSubParamRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(LDP_THRES_UP);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)value);
    nRequestSize = sizeof(XnObSubParamRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetLdpThresLowV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_THRES_LOW, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_THRES_LOW);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);
    value = nValue;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetLdpThresLowV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 value){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        (XnUInt32)LDP_THRES_LOW, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    //
    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (value< (XnUInt32)supportSubCmdValue.nMinValue || value>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnObSubParamRequest* pRequest = (XnObSubParamRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(LDP_THRES_LOW);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)value);
    nRequestSize = sizeof(XnObSubParamRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLdpEnableV1,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetLdpNoiseValueV1(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &value)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUInt32)LDP_NOISE_VALUE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }


    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(LDP_NOISE_VALUE);
    XnUInt16 nRequestSize = sizeof(XnUInt32);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLdpEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);
    value = nValue;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetEmitterEnableV1(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetEmitterEnableV1,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return (XN_STATUS_OK);
}

//Writedistortion param from flash
XnStatus XnHostProtocolWriteDistortionParam(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nSize, XnUChar* pBuffer)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        (XnUInt32)MULTI_DISTANCE_CALIBRATION_PARAM, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange != 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "The value exceeds the subcommand range !");
        return XN_STATUS_ERROR;
    }

    int tempSize = 0;
    if (nSize % 2 != 0)
    {
        tempSize = nSize + 1;
    }
    else
    {
        tempSize = nSize;
    }

    //4: zip buff size
    XnUChar *pSrcBuff = (XnUChar*)xnOSMalloc(tempSize + 4);
    *(XnUInt32*)pSrcBuff = XN_PREPARE_VAR32_IN_BUFFER(nSize);
    xnOSMemCopy(pSrcBuff + 4, pBuffer, tempSize);

    int addr_offset = 0;
    int sizeInBytes = tempSize + 4;
    int lastsizeInBytes = sizeInBytes % EATCH_PACKET_SIZE;

    for (int i = 0; i < sizeInBytes / EATCH_PACKET_SIZE; i++)
    {
        XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
        XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

        XnUInt32 nSubCmd = (XnUInt32)MULTI_DISTANCE_CALIBRATION_PARAM;
        *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
        *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(addr_offset);
        *(XnInt16*)(pDataBuf + 8) = XN_PREPARE_VAR32_IN_BUFFER(EATCH_PACKET_SIZE);

        xnOSMemCopy(pDataBuf + 12, pSrcBuff + addr_offset, EATCH_PACKET_SIZE);

        XnUInt16 nRequestSize = 3 * sizeof(XnUInt32) + EATCH_PACKET_SIZE;
        XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams);

        XnUInt16 nDataSize;
        rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize,
            pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams, NULL, nDataSize);

        if (rc != XN_STATUS_OK)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "send cmd write flash failed (%d)\n", rc);
            if (pSrcBuff)
            {
                xnOSFree(pSrcBuff);
                pSrcBuff = NULL;
            }
            return rc;
        }

        //
        addr_offset = addr_offset + EATCH_PACKET_SIZE;
    }

    if (lastsizeInBytes != 0)
    {
        XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
        XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

        XnUInt32 nSubCmd = (XnUInt32)MULTI_DISTANCE_CALIBRATION_PARAM;
        *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
        *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(addr_offset);
        *(XnInt16*)(pDataBuf + 8) = XN_PREPARE_VAR32_IN_BUFFER(lastsizeInBytes);

        memcpy(pDataBuf + 12, pSrcBuff + addr_offset, lastsizeInBytes);

        XnUInt16 nRequestSize = 3 * sizeof(XnUInt32) + lastsizeInBytes;
        XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams);

        XnUInt16 nDataSize;
        rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize,
            pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams, NULL, nDataSize);

        if (rc != XN_STATUS_OK)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "send cmd write flash failed (%d)\n", rc);
            if (pSrcBuff)
            {
                xnOSFree(pSrcBuff);
                pSrcBuff = NULL;
            }
            return rc;
        }
    }

    if (pSrcBuff)
    {
        xnOSFree(pSrcBuff);
        pSrcBuff = NULL;
    }

    return (XN_STATUS_OK);
}

#define MAX_MULTI_DISTANCE_CALIBRATION 1024*1024
//multi distance calibration read flash size
#define MULTI_DISTANCE_CALIBRATION_READ_SIZE 480

XnStatus XnHostProtocolReadMultiDistanceParam(XnDevicePrivateData* pDevicePrivateData, XnUChar* pBuffer, XnUInt32 nOffset, XnUInt32 nReadSize, XnUInt32 &nActualSize, XnUInt32 nSubCmd, XnBool bFirst)
{
    if (pBuffer == NULL)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "pBuffer is NULL\n");
        return XN_STATUS_ERROR;
    }

    XnUInt32 nHopeSendSize = nReadSize;
    if (nHopeSendSize % 2 != 0)
    {
        nHopeSendSize = nHopeSendSize + 1;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    //XnUInt32 nSubCmd = (XnUInt32)MULTI_DISTANCE_CALIBRATION_PARAM;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nOffset);
    *(XnInt16*)(pDataBuf + 8) = XN_PREPARE_VAR32_IN_BUFFER(nHopeSendSize);

    XnUInt16 nRequestSize = 3 * sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "send cmd read flash failed (%d)\n", rc);
        return rc;
    }

    if (nReadSize % 2 == 0)
    {
        nActualSize = nDataSize * 2;
    }
    else
    {
        nActualSize = nDataSize * 2 - 1;
    }


    if (nReadSize != nActualSize)
    {
        return XN_STATUS_ERROR;
    }
    else
    {
        if (bFirst)
        {
            memcpy(pBuffer, pValue, nActualSize);
        }
        else
        {
            memcpy(pBuffer + nOffset - 4, pValue, nActualSize);
        }

        return (XN_STATUS_OK);
    }

}

//Read distortion param from flash
XnStatus XnHostProtocolReadDistortionParam(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nSize, XnUChar* pBuffer)
{
    if (pBuffer == NULL)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "pBuffer = NULL !");
        return XN_STATUS_ERROR;
    }

    XnUInt32 nMaxBuffSize = nSize;

    xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam Read flash start\n");
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams,
        (XnUInt32)MULTI_DISTANCE_CALIBRATION_PARAM, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    int sizeInBytes = 0;
    int readSize = MULTI_DISTANCE_CALIBRATION_READ_SIZE;
    int addr_offset = 0;
    bool bFirstRead = true;
    XnUInt32 nflashDataSize = 0;

    if (bFirstRead)
    {
        XnUChar pFirstRevbuffer[MAX_PACKET_SIZE] = { 0 };
        XnUInt32 nActualSize = 0;

        rc = XnHostProtocolReadMultiDistanceParam(pDevicePrivateData, (XnUChar*)pFirstRevbuffer, addr_offset, readSize, nActualSize, MULTI_DISTANCE_CALIBRATION_PARAM, bFirstRead);
        if (rc != XN_STATUS_OK)
        {
            return rc;
        }

        xnOSMemCopy((XnUInt32*)(&nflashDataSize), (XnUInt32 *)pFirstRevbuffer, 4);
        if (nflashDataSize == 0 || nflashDataSize > nMaxBuffSize)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "send cmd read flash size failed.\n");
            return XN_STATUS_ERROR;
        }

        //xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam----nDataSize= %d\n", nflashDataSize);
        memcpy(pBuffer, pFirstRevbuffer + 4, nActualSize - 4);

        sizeInBytes = nflashDataSize - (nActualSize - 4);
        //	xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam----sizeInBytes= %d\n", sizeInBytes);

        addr_offset = addr_offset + nActualSize;
        //xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam----addr_offset= %d\n", addr_offset);

        bFirstRead = false;
    }

    nSize = nflashDataSize;
    int lastsizeInBytes = sizeInBytes % readSize;
    //xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam----lastsizeInBytes= %d\n", lastsizeInBytes);

    for (int i = 0; i < sizeInBytes / readSize; i++)
    {
        XnUInt32 nActualSize = 0;
        rc = XnHostProtocolReadMultiDistanceParam(pDevicePrivateData, (XnUChar*)pBuffer, addr_offset, readSize, nActualSize, MULTI_DISTANCE_CALIBRATION_PARAM, bFirstRead);
        if (rc != XN_STATUS_OK)
        {
            return rc;
        }

        addr_offset = addr_offset + nActualSize;
    }

    if (lastsizeInBytes != 0)
    {
        XnUInt32 nActualSize = 0;
        rc = XnHostProtocolReadMultiDistanceParam(pDevicePrivateData, (XnUChar*)pBuffer, addr_offset, lastsizeInBytes, nActualSize, MULTI_DISTANCE_CALIBRATION_PARAM, bFirstRead);
        if (rc != XN_STATUS_OK)
        {
            return rc;
        }

        if (nActualSize < (XnUInt32)lastsizeInBytes)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "send cmd read flash failed (%d)\n", rc);
            return XN_STATUS_ERROR;
        }

        addr_offset = addr_offset + lastsizeInBytes;
    }

    if ((XnUInt32)((addr_offset - 4)) != nflashDataSize)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, " Data outflow");
        return XN_STATUS_ERROR;
    }

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "XnHostProtocolReadDistortionParam read flash success, nflashDataSize =%d!\n", nflashDataSize);

    return (XN_STATUS_OK);
}


#pragma pack (push, 1)

typedef struct XnVSetAeEnabled
{
    XnInt16 nActive;
} XnVSetAeEnabled;

typedef struct XnVSetHdrModeEnabled
{
	XnInt16 nActive;
} XnVSetHdrModeEnabled;
#pragma pack (pop)

XnStatus XnHostProtocolSetAeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetAeEnabled *aeEnable = (XnVSetAeEnabled*)pDataBuf;
    aeEnable->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnInt16)bActive);
    XnUInt16 nRequestSize = sizeof(XnVSetAeEnabled);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetAeEnable);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeSetAeEnable, NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return rc;
}


XnStatus XnHostProtocolGetAeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetAeEnable);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetAeEnable, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (2 * nDataSize != sizeof(XnVSetAeEnabled))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol ae request failed!");
        return XN_STATUS_ERROR;
    }

    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return rc;
}
XnStatus XnHostProtocolSetHdrModeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
	XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
	XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnVSetHdrModeEnabled *hdrEnable = (XnVSetHdrModeEnabled*)pDataBuf;
	hdrEnable->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnInt16)bActive);
	XnUInt16 nRequestSize = sizeof(XnVSetHdrModeEnabled);
	XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetHdrModeEnable);
	XnUInt16 nDataSize;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
		pDevicePrivateData->FWInfo.nOpcodeSetHdrModeEnable, NULL, nDataSize);
	XN_IS_STATUS_OK(rc);
	return rc;
}
XnStatus XnHostProtocolGetHdrModeEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
	XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetHdrModeEnable);
	XnUInt16 nDataSize;
	XnUInt16* pValue = NULL;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
		pDevicePrivateData->FWInfo.nOpcodeGetHdrModeEnable, (XnUChar**)(&pValue), nDataSize);
	XN_IS_STATUS_OK(rc);
	if (2 * nDataSize != sizeof(XnVSetHdrModeEnabled))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol hdrmode request failed!");
		return XN_STATUS_ERROR;
	}
	XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
	if (nValue == 0x01)
	{
		bActive = TRUE;
	}
	else
	{
		bActive = FALSE;
	}
	return rc;
}

XnStatus XnHostProtocolSetFirmwareQN(XnDevicePrivateData* pDevicePrivateData, const OBFirmwareQN* qN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 nRequestSize = sizeof(OBFirmwareQN);
    xnOSMemCopy(pDataBuf, qN, nRequestSize);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_WRITE_QN);

    XnUInt16 nDataSize;
    return XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize, OB_OPCODE_WRITE_QN, NULL, nDataSize);
}

XnStatus XnHostProtocolGetFirmwareQN(XnDevicePrivateData* pDevicePrivateData, OBFirmwareQN* qN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OB_OPCODE_READ_QN);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OB_OPCODE_READ_QN, &pValue, nDataSize);
    XN_IS_STATUS_OK(rc);
    if (nDataSize * 2 == sizeof(OBFirmwareQN))
    {
        xnOSMemCopy(qN, pValue, sizeof(OBFirmwareQN));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }
}

XnStatus XnHostProtocolVerifyQN(XnDevicePrivateData* pDevicePrivateData, const OBFirmwareQN* qN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 nRequestSize = sizeof(OBFirmwareQN);
    xnOSMemCopy(pDataBuf, qN, nRequestSize);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_VERIFY_QN);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize, OB_OPCODE_VERIFY_QN, (XnUChar**)&pValue, nDataSize);
    XN_IS_STATUS_OK(rc);
    if (nDataSize != 1)
    {
        return XN_STATUS_ERROR;
    }

    XnUInt16 status = XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (status == 0x01)
    {
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_QN_VERIFY_FAILED;
    }
}

XnStatus XnHostProtocolGetPublicBoardVersion(const XnDevicePrivateData* pDevicePrivateData, OBPublicBoardVersion* pData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnUChar *pVersion = NULL;
    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting Public Board versions...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OPCODE_GET_PUBLIC_BOARD_VERSION);

    //GET VERSION
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OPCODE_GET_PUBLIC_BOARD_VERSION,
        (XnUChar**)(&pVersion), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get public board version failed: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 == sizeof(OBPublicBoardVersion))
    {
        xnOSMemCopy(pData, pVersion, sizeof(OBPublicBoardVersion));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }
}

XnStatus XnHostProtocolMx6300FirmewarGetVersion(const XnDevicePrivateData* pDevicePrivateData, ObMX6300Version* pData)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnUChar *pVersion = NULL;
    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting mx6300 versions...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OPCODE_GET_MX6300_VERSION);

    //GET VERSION
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OPCODE_GET_MX6300_VERSION,
        (XnUChar**)(&pVersion), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get version failed: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 == sizeof(ObMX6300Version))
    {
        xnOSMemCopy(pData, pVersion, sizeof(ObMX6300Version));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }
}

#pragma pack (push, 1)

typedef struct XnD2CResolutionRequest
{
    XnUInt16 nResolution;
} XnD2CResolutionRequest;

#pragma pack (pop)

XnStatus XnHostProtocolSetD2CResolution(XnDevicePrivateData* pDevicePrivateData, XnUInt16 nResolution)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnD2CResolutionRequest* pRequest = (XnD2CResolutionRequest*)pDataBuf;
    pRequest->nResolution = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)nResolution);
    nRequestSize = sizeof(XnD2CResolutionRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetD2CResolution);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetD2CResolution,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetD2CResolution(XnDevicePrivateData* pDevicePrivateData, XnUInt16 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetD2CResolution);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, pDevicePrivateData->FWInfo.nOpcodeGetD2CResolution,
        (XnUChar**)(&pValue), nDataSize);
    XN_IS_STATUS_OK(rc);

    if (nDataSize != 1)
    {
        return XN_STATUS_ERROR;
    }

    nValue = XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetUsbDeviceSpeed(XnDevicePrivateData* pDevicePrivateData, XnUInt16 &nValue)
{

    XnUSBDeviceSpeed deviceSpeed;

    XnStatus status = xnUSBGetDeviceSpeed(pDevicePrivateData->SensorHandle.USBDevice, &deviceSpeed);

    if (status == XN_STATUS_OK)
    {
        nValue = (XnUInt16)deviceSpeed;
    }

    return status;
}

XnStatus XnHostProtocolSetSerialNumber(XnDevicePrivateData* pDevicePrivateData, const OBSerialNumber* sN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 nRequestSize = sizeof(OBSerialNumber);
    xnOSMemCopy(pDataBuf, sN, nRequestSize);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_SET_SERIAL_NUMBER);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize, OB_OPCODE_SET_SERIAL_NUMBER, NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetSerialNumber(XnDevicePrivateData* pDevicePrivateData, OBSerialNumber* sN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OB_OPCODE_GET_SERIAL_NUMBER);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;

    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer,
        pDevicePrivateData->FWInfo.nProtocolHeaderSize, OB_OPCODE_GET_SERIAL_NUMBER, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(OBSerialNumber))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get serialNumber failed");
        return XN_STATUS_ERROR;
    }

    xnOSMemCopy(sN, pValue, sizeof(OBSerialNumber));

    return (XN_STATUS_OK);
}

//get ir flood switch
XnStatus XnHostProtocolGeminiGetIrFloodSwitchState(XnDevicePrivateData* pDevicePrivateData, XnUInt32* c_pValue)
{
    return XnHostProtocolGeminiGetIrFloodState(pDevicePrivateData, IR_READ_FLOOD_ENABLE, c_pValue);
}

//get ir flood level
XnStatus XnHostProtocolGeminiGetIrFloodLevelState(XnDevicePrivateData* pDevicePrivateData, XnUInt32* c_pValue)
{
    return XnHostProtocolGeminiGetIrFloodState(pDevicePrivateData, IR_READ_FLOOD_LEVEL, c_pValue);
}

#pragma pack (push, 1)
typedef struct IrFloodProtocolReadDataFormat
{
    XnUInt16 nSubCmd;
}IrFloodProtocolReadDataFormat;

typedef struct IrFloodProtocolWriteDataFormat
{
    XnUInt16 nSubCmd;
    XnUInt16 nData;
} IrFloodProtocolWriteDataFormat;
#pragma pack (pop)
XnStatus XnHostProtocolGeminiGetIrFloodState(XnDevicePrivateData* pDevicePrivateData, IrFloodSubCmd nSubType, /*IrFloodOptParam nOptParam, */XnUInt32* c_pValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, OB_OPCODE_IR_FLOOD_OPTION,//liuk-g
        (XnUInt32)nSubType, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Ir flood sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd : Read  not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    ((IrFloodProtocolReadDataFormat*)pDataBuf)->nSubCmd = XN_PREPARE_VAR16_IN_BUFFER(nSubType);
    XnUInt16 nRequestSize = sizeof(IrFloodProtocolReadDataFormat);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_IR_FLOOD_OPTION);

    XnUInt16 nDataSize = 0;
    XnUInt32* pValue = NULL;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, OB_OPCODE_IR_FLOOD_OPTION,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt16))
    {
        return XN_STATUS_ERROR;
    }

    *c_pValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    return (XN_STATUS_OK);
}

//set ir flood switch
XnStatus XnHostProtocolGeminiSetIrFloodSwitchState(XnDevicePrivateData* pDevicePrivateData, const XnUInt32& c_nValue)
{
    return XnHostProtocolGeminiSetIrFloodState(pDevicePrivateData, IR_WRITE_FLOOD_ENABLE, c_nValue);
}

//set ir flood level
XnStatus XnHostProtocolGeminiSetIrFloodLevelState(XnDevicePrivateData* pDevicePrivateData, const XnUInt32& c_nValue)
{
    return XnHostProtocolGeminiSetIrFloodState(pDevicePrivateData, IR_WRITE_FLOOD_LVEL, c_nValue);
}

XnStatus XnHostProtocolGeminiSetIrFloodState(XnDevicePrivateData* pDevicePrivateData, IrFloodSubCmd nSubType, const XnUInt32& c_nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, OB_OPCODE_IR_FLOOD_OPTION,
        (XnUInt32)nSubType, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        if (IR_WRITE_FLOOD_ENABLE == nSubType)
        {
            return XnHostProtocolSetIrfloodState(pDevicePrivateData, c_nValue == 1 ? TRUE : FALSE);
        }
        else
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Ir flood sub cmd error!");
            return rc;
        }
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd : Write  not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (c_nValue< (XnUInt32)supportSubCmdValue.nMinValue || c_nValue>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    (*(IrFloodProtocolWriteDataFormat*)(pDataBuf)).nSubCmd = XN_PREPARE_VAR16_IN_BUFFER(nSubType);
    (*(IrFloodProtocolWriteDataFormat*)(pDataBuf)).nData = XN_PREPARE_VAR16_IN_BUFFER(c_nValue);

    XnUInt16 nRequestSize = sizeof(IrFloodProtocolWriteDataFormat);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_IR_FLOOD_OPTION);


    XnUInt16 nDataSize = 0;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, OB_OPCODE_IR_FLOOD_OPTION, \
        NULL, nDataSize);

    XN_IS_STATUS_OK(rc);
    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolSetKT_PN(XnDevicePrivateData* pDevicePrivateData, const OBKTProductNumber* pN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 nRequestSize = sizeof(OBKTProductNumber);
    xnOSMemCopy(pDataBuf, pN, nRequestSize);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_WRITE_PN);

    XnUInt16 nDataSize;
    return XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize, OB_OPCODE_WRITE_PN, NULL, nDataSize);
}


XnStatus XnHostProtocolGetKT_PN(XnDevicePrivateData* pDevicePrivateData, OBKTProductNumber* pN)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OB_OPCODE_READ_PN);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OB_OPCODE_READ_PN, &pValue, nDataSize);
    XN_IS_STATUS_OK(rc);
    if (nDataSize * 2 == sizeof(OBKTProductNumber))
    {
        xnOSMemCopy(pN, pValue, sizeof(OBKTProductNumber));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }
}

#pragma pack (push, 1)

typedef struct XnVSetMipiTestEnabled
{
    XnInt16 nActive;
} XnVSetMipiTestEnabled;

#pragma pack (pop)

XnStatus XnHostProtocolSetMipiTestEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetMipiTestEnabled *mipiTestEnable = (XnVSetMipiTestEnabled*)pDataBuf;
    mipiTestEnable->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnInt16)bActive);
    XnUInt16 nRequestSize = sizeof(XnVSetMipiTestEnabled);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetMipiTestEnable);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeSetMipiTestEnable, NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return rc;
}


XnStatus XnHostProtocolGetMipiTestEnable(XnDevicePrivateData* pDevicePrivateData, XnBool &bActive)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeGetMipiTestEnable);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeGetMipiTestEnable, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (2 * nDataSize != sizeof(XnVSetMipiTestEnabled))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol get mipi test request failed!");
        return XN_STATUS_ERROR;
    }

    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        bActive = TRUE;
    }
    else
    {
        bActive = FALSE;
    }

    return rc;
}

XnStatus XnHostProtocolI2CReadFlashOnce(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt16 nSize, XnUChar* pBuffer)
{

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nOffset);
    *(XnUInt16*)(pDataBuf + 4) = XN_PREPARE_VAR16_IN_BUFFER(nSize / 2);

    XnUInt16 nRequestSize = sizeof(XnUInt32) + sizeof(XnUInt16);


    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeI2CReadFlash);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeI2CReadFlash,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (2 * nDataSize != nSize)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol I2CReadFlash failed!");
        return XN_STATUS_ERROR;
    }

    xnOSMemCopy(pBuffer, pValue, nSize);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolI2CReadFlash(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nOffset, XnUInt32 nSize, XnUChar* pBuffer)
{
    int addr_offset = 0;


    XnUInt8	Receivebuf[512] = { 0 };

    int sizeInBytes = nSize;

    int lastsizeInBytes = sizeInBytes % EATCH_PACKET_SIZE;

    for (int k = 0; k < sizeInBytes / EATCH_PACKET_SIZE; k++)
    {
        XnStatus nRetVal = XnHostProtocolI2CReadFlashOnce(pDevicePrivateData, nOffset + addr_offset, (XnUInt16)EATCH_PACKET_SIZE, (XnUChar *)Receivebuf);

        if (nRetVal != XN_STATUS_OK)
        {
            return nRetVal;
        }

        xnOSMemCopy(pBuffer + addr_offset, Receivebuf, EATCH_PACKET_SIZE);

        addr_offset = addr_offset + EATCH_PACKET_SIZE;
    }

    if (lastsizeInBytes != 0)
    {

        XnStatus nRetVal = XnHostProtocolI2CReadFlashOnce(pDevicePrivateData, nOffset + addr_offset, (XnUInt16)lastsizeInBytes, (XnUChar *)Receivebuf);
        if (nRetVal != XN_STATUS_OK)
        {
            return nRetVal;
        }

        xnOSMemCopy(pBuffer + addr_offset, Receivebuf, lastsizeInBytes);

    }

    return XN_STATUS_OK;
}


#pragma pack (push, 1)
typedef struct AEProtocolGetDataFormat
{
    XnUInt16 nSubCmd;
}AEProtocolGetDataFormat;

typedef struct AEProtocolSetDataFormat
{
    XnUInt16 nSubCmd;
    AeParamsStruct nData;
} AEProtocolSetDataFormat;
#pragma pack (pop)
XnStatus XnHostProtocolGetAEOption(XnDevicePrivateData* pDevicePrivateData, AEOption nSubType, AeParamsStruct* pAeValue)
{
    /*XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, OB_OPCODE_AE_OPTION,//liuk-g
    (XnUInt32)nSubType, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
    xnLogError(XN_MASK_SENSOR_PROTOCOL, "AE sub cmd error!");
    return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
    xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd : Read  not supported!");
    return XN_STATUS_ERROR;
    }*/

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    ((AEProtocolGetDataFormat*)pDataBuf)->nSubCmd = XN_PREPARE_VAR16_IN_BUFFER(nSubType);
    XnUInt16 nRequestSize = sizeof(AEProtocolGetDataFormat);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_AE_OPTION);

    XnUInt16 nDataSize = 0;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, OB_OPCODE_AE_OPTION,
        (XnUChar**)(&pValue), nDataSize);


    if (nDataSize * 2 != sizeof(AeParamsStruct))
    {
        return XN_STATUS_ERROR;
    }

    *pAeValue = *((AeParamsStruct*)pValue);
    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetAEOption(XnDevicePrivateData* pDevicePrivateData, AEOption nSubType, AeParamsStruct* pValue)
{
    /*XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, OB_OPCODE_AE_OPTION,
    (XnUInt32)nSubType, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
    xnLogError(XN_MASK_SENSOR_PROTOCOL, "AE sub cmd error!");
    return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
    xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd : Write  not supported!");
    return XN_STATUS_ERROR;
    }*/

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    (*(AEProtocolSetDataFormat*)(pDataBuf)).nSubCmd = XN_PREPARE_VAR16_IN_BUFFER(nSubType);
    (*(AEProtocolSetDataFormat*)(pDataBuf)).nData = *pValue;

    XnUInt16 nRequestSize = sizeof(AEProtocolSetDataFormat);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, OB_OPCODE_AE_OPTION);


    XnUInt16 nDataSize = 0;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, OB_OPCODE_AE_OPTION, \
        NULL, nDataSize);

    XN_IS_STATUS_OK(rc);
    return (XN_STATUS_OK);
}
#pragma pack (push, 1)

typedef struct XnVSetDistortionStateRequest
{
    XnUInt32 nSubCmd;
    XnUInt32 nActive;
} XnVSetDistortionStateRequest;

#pragma pack (pop)

XnStatus XnHostProtocolDistortionStateSwitch(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        (XnUInt32)MULTI_DISTANCE_CALIBRATION_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nActive< (XnUInt32)supportSubCmdValue.nMinValue || nActive>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt32 nRequestSize;

    XnVSetDistortionStateRequest* pRequest = (XnVSetDistortionStateRequest*)pDataBuf;
    pRequest->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(MULTI_DISTANCE_CALIBRATION_ENABLE);
    pRequest->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)nActive);
    nRequestSize = sizeof(XnVSetDistortionStateRequest);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimWriteSubCmdParams,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetDistortionState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive){
    nActive = 0;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams,
        (XnUInt32)MULTI_DISTANCE_CALIBRATION_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(MULTI_DISTANCE_CALIBRATION_ENABLE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeOptimReadSubCmdParams, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    //
    XnUInt32 nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    nActive = nValue;

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetCoreBroadFlashId(const XnDevicePrivateData* pDevicePrivateData, XnUInt32 & nBroadId)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnUInt32 *pValue = NULL;
    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting Core Board flash id...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OPCODE_GET_CORE_BOARD_FLASH_ID);


    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OPCODE_GET_CORE_BOARD_FLASH_ID,
        (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get core board flash id failed: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    nBroadId = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);
    return XN_STATUS_OK;
}


typedef struct XnVSetPdStateRequest
{
    XnUInt32 nSubCmd;
    XnUInt32 nActive;
} XnVSetPdStateRequest;

XnStatus XnHostProtocolSetPdEnableStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd,
        (XnUInt32)PD_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nActive< (XnUInt32)supportSubCmdValue.nMinValue || nActive>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetPdStateRequest* pdState = (XnVSetPdStateRequest*)pDataBuf;
    pdState->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(PD_ENABLE);
    pdState->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)nActive);

    XnUInt16 nRequestSize = sizeof(XnVSetPdStateRequest);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetPdEnableStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive){
    nActive = 0;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd,
        (XnUInt32)PD_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PD_ENABLE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        return XN_STATUS_ERROR;
    }

    nActive = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetPdAlertStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive){
    nActive = 0;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd,
        (XnUInt32)PD_ALERT_STATUS, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PD_ALERT_STATUS);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {

        return XN_STATUS_ERROR;
    }

    nActive = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolGetPdUpperTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive){
    nActive = 0;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd,
        (XnUInt32)PD_HTH_THRESHOLD, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PD_HTH_THRESHOLD);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {

        return XN_STATUS_ERROR;
    }

    nActive = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetPdUpperTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd,
        (XnUInt32)PD_HTH_THRESHOLD, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nActive< (XnUInt32)supportSubCmdValue.nMinValue || nActive>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetPdStateRequest* pdState = (XnVSetPdStateRequest*)pDataBuf;
    pdState->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(PD_HTH_THRESHOLD);
    pdState->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)nActive);

    XnUInt16 nRequestSize = sizeof(XnVSetPdStateRequest);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetPdLowerTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nActive){
    nActive = 0;
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd,
        (XnUInt32)PD_LTH_THRESHOLD, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PD_LTH_THRESHOLD);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {

        return XN_STATUS_ERROR;
    }

    nActive = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetPdLowerTlv(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nActive){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd,
        (XnUInt32)PD_LTH_THRESHOLD, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    if (supportSubCmdValue.nSupportRange == 1)
    {
        if (nActive< (XnUInt32)supportSubCmdValue.nMinValue || nActive>(XnUInt32)supportSubCmdValue.nMaxValue)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "the value exceeds the subcommand range !");
            return XN_STATUS_ERROR;
        }
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetPdStateRequest* pdState = (XnVSetPdStateRequest*)pDataBuf;
    pdState->nSubCmd = XN_PREPARE_VAR32_IN_BUFFER(PD_LTH_THRESHOLD);
    pdState->nActive = XN_PREPARE_VAR32_IN_BUFFER((XnUInt32)nActive);

    XnUInt16 nRequestSize = sizeof(XnVSetPdStateRequest);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdWriteSubCmd, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetPdCurrentThreshold(XnDevicePrivateData* pDevicePrivateData, OBPdThreshold* pd){
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd,
        (XnUInt32)PD_CURRENT_THRESHOLD, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PD_CURRENT_THRESHOLD);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd);

    XnUInt16 nDataSize;
    OBPdThreshold* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodePdReadSubCmd, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 == sizeof(OBPdThreshold))
    {
        xnOSMemCopy(pd, pValue, sizeof(OBPdThreshold));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }

}


#pragma pack (push, 1)

typedef struct XnVSetBootLoaderPtsEnabled
{
    XnInt16 nActive;
} XnVSetBootLoaderPtsEnabled;

#pragma pack (pop)

XnStatus XnHostProtocolSetBootLoaderPtsStatus(XnDevicePrivateData* pDevicePrivateData, XnBool bActive){
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnVSetBootLoaderPtsEnabled* bootLoaderPts = (XnVSetBootLoaderPtsEnabled*)pDataBuf;
    bootLoaderPts->nActive = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);

    XnUInt16 nRequestSize = sizeof(XnVSetBootLoaderPtsEnabled);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsWrite);

    XnUInt16 nDataSize;
    XnUChar* pValue = NULL;

    //
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsWrite, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}


XnStatus XnHostProtocolGetBootLoaderPtsStatus(XnDevicePrivateData* pDevicePrivateData, XnBool &nActive){
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsRead);

    XnUInt16 nDataSize;
    XnUInt16* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize,
        pDevicePrivateData->FWInfo.nOpcodeBootLoaderPtsRead, (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (2 * nDataSize != sizeof(XnVSetBootLoaderPtsEnabled))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol bootLoader protection status request failed!");
        return XN_STATUS_ERROR;
    }

    XnUInt16 nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);
    if (nValue == 0x01)
    {
        nActive = TRUE;
    }
    else
    {
        nActive = FALSE;
    }

    return (XN_STATUS_OK);
}

//set laser time
XnStatus XnHostProtocolSetLaserTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_LASER_TIME;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserTime);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetLaserTime,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetLaserTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_LASER_TIME;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    //*(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLaserTime);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetLaserTime,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetPostFilterThreshold(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_DEPTH_POSTFILTER_THRESHOLD;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);
    *(((XnUInt32*)pDataBuf) + 1) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetPostFilterThreshold);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetPostFilterThreshold,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetPostFilterThreshold(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUInt32 nSubCmd = (XnUInt32)OB_PARAM_DEPTH_POSTFILTER_THRESHOLD;
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nSubCmd);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetPostFilterThreshold);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetPostFilterThreshold,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return (XN_STATUS_OK);
}

//TOF
XnStatus XnHostProtocolSetTofSensorEnable(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite,
        (XnUInt32)PROTOCOL_TOF_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_ENABLE);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite);

    XnUInt16 nDataSize;
    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetTofSensorEnable(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead,
        (XnUInt32)PROTOCOL_TOF_ENABLE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_ENABLE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get TOF sensor enable status failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetTofSensorMeasureResult(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead,
        (XnUInt32)PROTOCOL_TOF_RESULT, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_RESULT);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get TOF sensor measurement result failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetTofSensorAppId(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead,
        (XnUInt32)PROTOCOL_TOF_APP_ID, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_APP_ID);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get TOF sensor app id failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetTofSensorCalibrationValue(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite,
        (XnUInt32)PROTOCOL_TOF_CALIBRATION, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_CALIBRATION);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite);

    XnUInt16 nDataSize;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetTofSensorAppEnableState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite,
        (XnUInt32)PROTOCOL_TOF_APP_START, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_APP_START);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite);

    XnUInt16 nDataSize;
    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

//Motor
XnStatus XnHostProtocolSetMotorTest(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorWrite,
        (XnUInt32)PROTOCOL_STEP_MOTOR_TEST, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_TEST);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue); // 1 motor start test

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorWrite);

    XnUInt16 nDataSize;
    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorTestResult(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_TEST, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_TEST);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor test result failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetMotorPosition(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorWrite,
        (XnUInt32)PROTOCOL_STEP_MOTOR_POSITION, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_POSITION);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorWrite);

    XnUInt16 nDataSize;
    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorPosition(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_POSITION, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_POSITION);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor position failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorStatus(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_STATUS, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_STATUS);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor status failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}


XnStatus XnHostProtocolGetMotorTestCount(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_TEST_COUNT, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_TEST_COUNT);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor test count failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetMotorRunTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
	XnSupportSubCmdValue supportSubCmdValue;
	XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorWrite,
		(XnUInt32)PROTOCOL_STEP_MOTOR_RUN_TIME, &supportSubCmdValue);

	if (rc != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
		return rc;
	}

	if (supportSubCmdValue.nSupportWrite == 0)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
		return XN_STATUS_ERROR;
	}

	XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
	XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

	*(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_RUN_TIME);
	*(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

	XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
	XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorWrite);

	XnUInt16 nDataSize;
	rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
		pDevicePrivateData->FWInfo.nOpcodeMotorWrite, NULL, nDataSize);
	XN_IS_STATUS_OK(rc);

	return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorRunTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
	XnSupportSubCmdValue supportSubCmdValue;
	XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
		(XnUInt32)PROTOCOL_STEP_MOTOR_RUN_TIME, &supportSubCmdValue);

	if (rc != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
		return rc;
	}

	if (supportSubCmdValue.nSupportRead == 0)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
		return XN_STATUS_ERROR;
	}

	XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
	XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

	*(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_RUN_TIME);

	XnUInt16 nRequestSize = sizeof(XnUInt32);
	XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

	XnUInt16 nDataSize;
	uint32_t* pValue = NULL;

	//
	rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
		pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
	if (rc != XN_STATUS_OK)
	{
		return rc;
	}

	if (nDataSize * 2 != sizeof(XnUInt32))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor runtime failed!");
		return XN_STATUS_ERROR;
	}

	nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

	return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorFeature(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_FEATURE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_FEATURE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor feature failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorUpdownState(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_UPDOWN_STATE, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_UPDOWN_STATE);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor updown state failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetMotorUpdownTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorRead,
        (XnUInt32)PROTOCOL_STEP_MOTOR_UPDOWN_TIME, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_UPDOWN_TIME);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorRead);

    XnUInt16 nDataSize;
    uint32_t* pValue = NULL;

    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorRead, (XnUChar**)(&pValue), nDataSize);
    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get motor updown time failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetMotorUpdown(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeMotorWrite,
        (XnUInt32)PROTOCOL_STEP_MOTOR_SET_UPDOWN, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_STEP_MOTOR_SET_UPDOWN);
    *(XnUInt32*)(pDataBuf + 4) = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) * 2;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeMotorWrite);

    XnUInt16 nDataSize;
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeMotorWrite, NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}

//china union pay certification
XnStatus XnHostProtocolGetCupVerifyVersion(const XnDevicePrivateData* pDevicePrivateData, CupCertify* pCupCertify)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUInt16 nDataSize;
    XnUChar *pVersion = NULL;
    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Getting Cup certification...");

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, 0, OB_OPCODE_CUP_CERTIFY_GET);

    //GET VERSION
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize, OB_OPCODE_CUP_CERTIFY_GET,
        (XnUChar**)(&pVersion), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Get Cup certification: %s", xnGetStatusString(rc));
        return rc;
    }

    if (nDataSize * 2 == sizeof(CupCertify))
    {
        xnOSMemCopy(pCupCertify, pVersion, sizeof(CupCertify));
        return XN_STATUS_OK;
    }
    else
    {
        return XN_STATUS_ERROR;
    }
}


XnStatus XnHostProtocolSetTofSensorCalibrationParams(XnDevicePrivateData* pDevicePrivateData, const OBTofSensorCalParams* tofCalParams)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite,
        (XnUInt32)PROTOCOL_TOF_CALIBRATION_DATA, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportWrite == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    int nValue = sizeof(tofCalParams->calParams);
    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_CALIBRATION_DATA);
    xnOSMemCopy(pDataBuf + 4, tofCalParams->calParams, nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32) + nValue;
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite);

    XnUInt16 nDataSize;
    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorWrite, NULL, nDataSize);

    XN_IS_STATUS_OK(rc);

    return XN_STATUS_OK;
}


XnStatus XnHostProtocolGetTofSensorCalibrationParams(XnDevicePrivateData* pDevicePrivateData, OBTofSensorCalParams* tofCalParams)
{
    XnSupportSubCmdValue supportSubCmdValue;
    XnStatus rc = XnHostProtocolSupportSubCmdMode(pDevicePrivateData, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead,
        (XnUInt32)PROTOCOL_TOF_CALIBRATION_DATA, &supportSubCmdValue);

    if (rc != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Support sub cmd error!");
        return rc;
    }

    if (supportSubCmdValue.nSupportRead == 0)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host Protocol sub cmd not supported!");
        return XN_STATUS_ERROR;
    }

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(PROTOCOL_TOF_CALIBRATION_DATA);

    XnUInt16 nRequestSize = sizeof(XnUInt32);
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeToFSensorRead);

    XnUInt16 nDataSize;
    uint8_t* pValue = NULL;

    //
    rc = XnHostProtocolExecute(pDevicePrivateData, buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + nRequestSize,
        pDevicePrivateData->FWInfo.nOpcodeToFSensorRead, (XnUChar**)(&pValue), nDataSize);
    if (rc != XN_STATUS_OK)
    {
        return rc;
    }

    if (nDataSize * 2 != 14)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get TOF sensor calibration params failed!");
        return XN_STATUS_ERROR;
    }

    xnOSMemCopy(tofCalParams->calParams, pValue, 14);
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetDepthIrMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt32*)pDataBuf = XN_PREPARE_VAR32_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt32);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetDepthIrMode);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetDepthIrMode,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Host protocol set depth and NIR success!");
    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetDepthIrMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnUInt16 nRequestSize = 0;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetDepthIrMode);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetDepthIrMode,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get depth and NIR failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pValue);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Host protocol get depth and NIR success!");

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolBulkDataSend(XnDevicePrivateData* pDevicePrivateData, const XnUChar* pData, const XnUInt32 nSize, const XnUInt16 opCode)
{
    XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    XnUChar send[MAX_PACKET_BULK_SIZE] = { 0 };
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderBulkSize;
    memcpy(send + headerSize, pData, nSize);

    XnHostProtocolInitBulkHeader(pDevicePrivateData, send, nSize, opCode);
    XnStatus rc = XnHostProtocolBulkExecute(
        pDevicePrivateData,
        send,
        headerSize + nSize,
        opCode);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send bulk data, opcode (%d), error (%d).\n", opCode, rc);
        return rc;
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolFileTransferPrepare(XnDevicePrivateData* pDevicePrivateData, const XnUsbFileAttributes* pAttributes)
{
    XN_RET_IF_NULL(pAttributes, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    XnUInt16 dataSize = sizeof(XnUsbFileAttributes);
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeFileTransferPrepare;
    
    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, pAttributes, dataSize);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send request at the beginning of file transfer: opcode (%d), error (%s).",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    // Check the response code received from deivce.
    if (NULL != pRecv)
    {
        XnUInt16 response = *(uint16_t*)pRecv;
        if (response >= XN_USB_FILE_RECEIVE_TIMEOUT)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send request at the beginning of file transfer: response (%d).", response);
            return response;
        }
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolFileTransferring(XnDevicePrivateData* pDevicePrivateData, const XnUsbGeneralFile* pUsbFile)
{
    XN_RET_IF_NULL(pUsbFile, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    // OpenNI protocol requires DWORD size.
    XnUInt32 totalSize = pUsbFile->attributes.size;
    if (0 != totalSize % 2)
        totalSize += 1;

    XnUInt32 offset = 0;
    XnUInt32 dataSize = 0;
    XnUChar sendBuf[MAX_PACKET_BULK_SIZE] = { 0 };
    XnUInt16 uidSize = sizeof(pUsbFile->attributes.uid);
    XnUInt16 opCode = pDevicePrivateData->FWInfo.nOpcodeFileTransfer;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderBulkSize;
    while (offset < totalSize)
    {
        dataSize = totalSize - offset;
        if (dataSize > EACH_PACKET_BULK_SIZE)
            dataSize = EACH_PACKET_BULK_SIZE;

        XnUChar* p = sendBuf + headerSize;
        *reinterpret_cast<XnUInt32*>(p) = XN_PREPARE_VAR32_IN_BUFFER(pUsbFile->attributes.uid);
        xnOSMemCopy(p + uidSize, pUsbFile->pContent + offset, dataSize);

        XnUInt32 packetSize = uidSize + dataSize;
        XnHostProtocolInitBulkHeader(pDevicePrivateData, sendBuf, packetSize, opCode);
        XnStatus rc = XnHostProtocolBulkExecute(pDevicePrivateData, sendBuf, headerSize + packetSize, opCode);
        if (XN_STATUS_OK != rc)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send file (%s), opcode (%d), error (%d).\n",
                pUsbFile->attributes.name, opCode, rc);
            return rc;
        }

        offset += dataSize;
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolFileTransferFinish(XnDevicePrivateData* pDevicePrivateData, const XnUsbFileAttributes* pAttributes)
{
    XN_RET_IF_NULL(pAttributes, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    XnUInt16 dataSize = sizeof(pAttributes->uid);
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeFileTransferFinish;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, &pAttributes->uid, dataSize);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send request at the end of file transfer: opcode (%d), error (%s).",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    // Check the response code received from deivce.
    if (NULL != pRecv)
    {
        XnUInt16 response = *(uint16_t*)pRecv;
        if (response >= XN_USB_FILE_RECEIVE_TIMEOUT)
        {
            xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send request at the end of file transfer: response code (%d).", response);
            return response;
        }
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSendUsbFile(XnDevicePrivateData* pDevicePrivateData, const XnUsbGeneralFile* pUsbFile)
{
    XN_RET_IF_NULL(pUsbFile, XN_STATUS_DEVICE_BAD_PARAM);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_DEVICE_BAD_PARAM);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeFileTransfer ||
        OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeFileTransferFinish ||
        OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeFileTransferPrepare)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "USB file sending requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    // Step 1: send a start request.
    XnStatus ret = XnHostProtocolFileTransferPrepare(pDevicePrivateData, &pUsbFile->attributes);
    XN_IS_STATUS_OK(ret);

    // Step 2: send the file data.
    ret = XnHostProtocolFileTransferring(pDevicePrivateData, pUsbFile);
    XN_IS_STATUS_OK(ret);

    // Step 3: send the end request.
    ret = XnHostProtocolFileTransferFinish(pDevicePrivateData, &pUsbFile->attributes);
    XN_IS_STATUS_OK(ret);

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolBulkDataUploadState(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 opCode, XnUInt32* pState)
{
    XN_RET_IF_NULL(pState, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    XnUInt16 requestSize = 0;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, buffer, requestSize, opCode);

    XnUInt16 recvSize = 0;
    XnUChar* pRecv = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, buffer, headerSize + requestSize, opCode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get the bulk data uploading state: opcode (%d), %s",
            opCode, xnGetStatusString(rc));
        return rc;
    }

    if (recvSize * 2 != sizeof(XnUInt32))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get the bulk data uploading state: data size mismatch(%d != %d), opcode (%d).",
            recvSize * 2, sizeof(XnUInt32), opCode);
        return XN_STATUS_ERROR;
    }

    // All is good.
    *pState = (XnUInt32)XN_PREPARE_VAR32_IN_BUFFER(*pRecv);
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolStartService(XnDevicePrivateData* pDevicePrivateData, const OniService* pService)
{
    XN_RET_IF_NULL(pService, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeStartService)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Service property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = sizeof(OniService);
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeStartService;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, pService, dataSize);

    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to start service (%d) status (%d): opcode (%d), error (%s).",
            pService->type, pService->value, opcode, xnGetStatusString(rc));
        return rc;
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetTOFFreqMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16* pMode)
{
    XN_RET_IF_NULL(pMode, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFFreqMode)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Frequency mode property getting for TOF sensor requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = 0;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFFreqMode;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get frequency mode of TOF sensor: opcode (%d), %s",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    if (recvSize * 2 != sizeof(XnUInt16))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get frequency mode of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
            recvSize, sizeof(XnUInt16), opcode);
        return XN_STATUS_ERROR;
    }

    // All is good.
    *pMode = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetTOFFreqMode(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 mode)
{
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFFreqMode)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Frequency mode property setting for TOF sensor requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = sizeof(XnUInt16);
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFFreqMode;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, &mode, dataSize);

    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set frequency mode for TOF sensor: opcode (%d), error (%s).",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetTOFSensorFilterLevel(XnDevicePrivateData* pDevicePrivateData, XnUInt16* level)
{
	XN_RET_IF_NULL(level, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFilterLevel)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "filter level property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFilterLevel;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get filter level of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get filter level of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	*level = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetTOFSensorFilterLevel(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 level)
{
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFilterLevel)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "filter level property setting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(XnUInt16);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFilterLevel;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, &level, dataSize);
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set filter level for TOF sensor: opcode (%d), error (%s).",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetTOFSensorIntegrationTime(XnDevicePrivateData* pDevicePrivateData, XnUInt32* integrationTime)
{
	XN_RET_IF_NULL(integrationTime, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorIntegrationTime)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "integration time property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt32 dataSize = sizeof(XnUInt32);
	XnUInt32 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorIntegrationTime;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get integration time of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != dataSize)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get integration time of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	*integrationTime = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt32*)pRecv);
	xnLogError(XN_MASK_SENSOR_PROTOCOL, "result=%d.",
		integrationTime);
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetTOFSensorIntegrationTime(XnDevicePrivateData* pDevicePrivateData, const XnUInt32 integrationTime)
{
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorIntegrationTime)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "integration time property setting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt32 dataSize = sizeof(XnUInt32);
	XnUInt32 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorIntegrationTime;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, &integrationTime, dataSize);
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set integration time for TOF sensor: opcode (%d), error (%s).",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetTOFSensorGain(XnDevicePrivateData* pDevicePrivateData, XnUInt16* gain)
{
	XN_RET_IF_NULL(gain, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorGain)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "gain property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorGain;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get gain of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get gain of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	*gain = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d,%d",
		gain,*gain);
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetTOFSensorLaserInterference(XnDevicePrivateData* pDevicePrivateData, XnUInt16* value)
{
	XN_RET_IF_NULL(value, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorLaserInterference)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "LaserInterference property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorLaserInterference;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get LaserInterference of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get LaserInterference of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	*value = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d,%d",
		value, *value);
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetTOFSensorWorkingMode(XnDevicePrivateData* pDevicePrivateData, XnUInt16* value) {
	XN_RET_IF_NULL(value, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorWorkingMode)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "working mode property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorWorkingMode;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get working mode of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get working mode of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	*value = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d,%d",
		value, *value);
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetGeneralFrequency(XnDevicePrivateData* pDevicePrivateData, ORBTofFrequency* pData) {
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFrequency)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "frequency property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorFrequency;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get frequency of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get frequency of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d",
		pRecv);
	xnOSMemCopy(pData, pRecv, sizeof(ORBTofFrequency));
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetGeneralDutyCycle(XnDevicePrivateData* pDevicePrivateData, ORBTofDuty* pData) {
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDutyCycle)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "duty cycle property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = 0;
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDutyCycle;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get duty cycle of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(XnUInt16))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get duty cycle of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(XnUInt16), opcode);
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d",
		pRecv);
	xnOSMemCopy(pData, pRecv, sizeof(ORBTofDuty));
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetGeneralDriverICReg(XnDevicePrivateData* pDevicePrivateData, ObReg8Map* pData) {
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDriverICReg)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "driver ic reg property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(ObReg8Map);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorDriverICReg;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pData, dataSize);
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get driver ic reg of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(ObReg8Map))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get driver ic reg of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(ObReg8Map), opcode);
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d",
		pRecv);
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "addr=%d,value=%d",
		((ObReg8Map*)pRecv)->addr, ((ObReg8Map*)pRecv)->value);
	xnOSMemCopy(pData, pRecv, sizeof(ObReg8Map));
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetGeneralSensorReg(XnDevicePrivateData* pDevicePrivateData, ObReg16Map* pData) {
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorSensorReg)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "sensor reg property getting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(ObReg16Map);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetTOFSensorSensorReg;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pData, dataSize);
	
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc || NULL == pRecv)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get sensor reg of TOF sensor: opcode (%d), %s",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	if (recvSize * 2 != sizeof(ObReg16Map))
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get sensor reg of TOF sensor: data size mismatch(%d != %d), opcode (%d).",
			recvSize, sizeof(ObReg16Map), opcode);
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "addr=%d,value=%d",
		((ObReg16Map*)pRecv)->addr, ((ObReg16Map*)pRecv)->value);
	xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "result=%d",
		pRecv);
	xnOSMemCopy(pData, pRecv, sizeof(ObReg16Map));
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetTOFSensorGain(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 gain)
{
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorGain)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "gain property setting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(XnUInt16);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorGain;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, &gain, dataSize);
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set gain for TOF sensor: opcode (%d), error (%s).",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetTOFSensorLaserInterference(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 value)
{
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorLaserInterference)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "laser interference property setting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(XnUInt16);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorLaserInterference;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, &value, dataSize);
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set laser interferrence for TOF sensor: opcode (%d), error (%s).",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetTOFSensorWorkingMode(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 value)
{
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorWorkingMode)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "working mode property setting for TOF sensor requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(XnUInt16);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorWorkingMode;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, &value, dataSize);
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, NULL, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set working mode for TOF sensor: opcode (%d), error (%s).",
			opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetSensorID(XnDevicePrivateData* pDevicePrivateData, OniSensorIDMap* pSensorID)
{
    XN_RET_IF_NULL(pSensorID, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetSensorID)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Sensor ID property getting requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 type = 0;
    switch (pSensorID->type)
    {
    case ONI_SENSOR_IR:
        type = XN_CMOS_TYPE_IR;
        break;
    case ONI_SENSOR_COLOR:
        type = XN_CMOS_TYPE_IMAGE;
        break;
    case ONI_SENSOR_DEPTH:
    case ONI_SENSOR_PHASE:
        type = XN_CMOS_TYPE_DEPTH;
        break;
    default:
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Invalid CMOS type...");
        return XN_STATUS_ERROR;
    }

    XnUInt16 dataSize = sizeof(XnUInt16);
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetSensorID;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, &type, dataSize);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get sensor ID: cmos type (%d), opcode (%d), error (%s).",
            type, opcode, xnGetStatusString(rc));
        return rc;
    }

    if (recvSize * 2 != dataSize)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get sensor ID: data size mismatch(%d != %d), opcode (%d).",
            recvSize, dataSize, opcode);
        return XN_STATUS_ERROR;
    }

    XnUInt16 val = XN_PREPARE_VAR16_IN_BUFFER(*(XnUInt16*)pRecv);
    switch (val)
    {
    case ONI_SENSOR_ID_PLECO:
    case ONI_SENSOR_ID_S5K33D:
        pSensorID->id = (OniSensorID)val;
        break;
    default:
        pSensorID->id = ONI_SENSOR_ID_NONE;
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Unknown sensor ID: 0x%x, opcode (%d).", val, opcode);
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolGetGeneralSerialNumber(XnDevicePrivateData* pDevicePrivateData, OniSerialNumberMap* pSerialMap)
{
    XN_RET_IF_NULL(pSerialMap, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General serial number property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = sizeof(XnUInt16);
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeGetSerialNumber;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, &pSerialMap->type, dataSize);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get general serial number: opcode (%d), error (%s)",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    pRecv[recvSize * 2] = '\0';
    xnOSStrNCopy(pSerialMap->serial, (XnChar*)pRecv, recvSize * 2, ONI_MAX_STR);

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetGeneralSerialNumber(XnDevicePrivateData* pDevicePrivateData, const OniSerialNumberMap* pSerialMap)
{
    XN_RET_IF_NULL(pSerialMap, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetSerialNumber)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General serial number property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    // OpenNI protocol requires DWORD size.
    XnUInt16 len = (XnUInt16)xnOSStrLen(pSerialMap->serial);
    if (0 != len % 2)
        len += 1;

    XnUInt16 dataSize = sizeof(XnUInt16) + len;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetSerialNumber;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, &pSerialMap->type, sizeof(XnUInt16));
    xnOSMemCopy(sendBuf + headerSize + sizeof(XnUInt16), pSerialMap->serial, len);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set general serial number: SN(%s), opcode (%d), error (%s).",
            pSerialMap->serial, opcode, xnGetStatusString(rc));
        return rc;
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetGeneralFrequency(XnDevicePrivateData* pDevicePrivateData, const ORBTofFrequency* pFrequency)
{
	XN_RET_IF_NULL(pFrequency, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFrequency)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General tof frequency property requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize =sizeof(ORBTofFrequency);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorFrequency;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pFrequency, dataSize);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set frequency: lowFreq(%d),highFreq(%d), opcode (%d), error (%s).",
			pFrequency->lowFrequncy,pFrequency->highFrequncy, opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetGeneralDutyCycle(XnDevicePrivateData* pDevicePrivateData, const ORBTofDuty* pDutyCycle)
{
	XN_RET_IF_NULL(pDutyCycle, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDutyCycle)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General tof duty cycle property requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(ORBTofDuty);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDutyCycle;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pDutyCycle, dataSize);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set duty cycle: lowDutyCycle(%d),highDutyCycle(%d), opcode (%d), error (%s).",
			pDutyCycle->lowDuty, pDutyCycle->highDuty, opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetGeneralDriverICReg(XnDevicePrivateData* pDevicePrivateData, const ObReg8Map* pData)
{
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDriverICReg)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General tof driver ic property requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(ObReg8Map);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorDriverICReg;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pData, dataSize);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set driver ic reg: addr(%d),val(%d), opcode (%d), error (%s).",
			pData->addr,pData->addr, opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolSetGeneralSensorReg(XnDevicePrivateData* pDevicePrivateData, const ObReg16Map* pData)
{
	XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);
	if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorSensorReg)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "General tof sensor reg property requires version 5.8.24 or higher...");
		return XN_STATUS_OK;
	}
	XnUInt16 dataSize = sizeof(ObReg16Map);
	XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSetTOFSensorSensorReg;
	XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;
	XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
	xnOSMemCopy(sendBuf + headerSize, pData, dataSize);
	XnUChar* pRecv = NULL;
	XnUInt16 recvSize = 0;
	XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);
	XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
	if (XN_STATUS_OK != rc)
	{
		xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to set sensor reg: addr(%d),val(%d), opcode (%d), error (%s).",
			pData->addr, pData->value, opcode, xnGetStatusString(rc));
		return rc;
	}
	return XN_STATUS_OK;
}
XnStatus XnHostProtocolGetPlatformVersion(XnDevicePrivateData* pDevicePrivateData, const XnUInt16 opcode, XnChar* pVersion)
{
    XN_RET_IF_NULL(pVersion, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeGetPlatformVersion)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Platform version property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = 0;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to get platform version: opcode (%d), error (%s).",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    pRecv[recvSize * 2] = '\0';
    xnOSStrNCopy(pVersion, (XnChar*)pRecv, recvSize * 2, ONI_MAX_STR);

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSendCommand(XnDevicePrivateData* pDevicePrivateData, OniSerialCmd* pCmd)
{
    XN_RET_IF_NULL(pCmd, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeSendCommand)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Command property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    // OpenNI protocol requires DWORD size.
    XnUInt16 len = (XnUInt16)xnOSStrLen(pCmd->cmd);
    if (0 != len % 2)
        len += 1;

    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeSendCommand;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    xnOSMemCopy(sendBuf + headerSize, pCmd->cmd, len);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, len, opcode);
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + len, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to send command <%s>: opcode (%d), error (%s).",
            pCmd->cmd, opcode, xnGetStatusString(rc));
        return rc;
    }

    if (NULL != pRecv)
    {
        pRecv[recvSize * 2] = '\0';
        xnOSStrNCopy(pCmd->resp, (XnChar*)pRecv, recvSize * 2, ONI_MAX_STR);
    }

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolQueryTimestamp(XnDevicePrivateData* pDevicePrivateData, XnUInt64* pTimestamp)
{
    XN_RET_IF_NULL(pTimestamp, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDevicePrivateData, XN_STATUS_NULL_INPUT_PTR);

    if (OPCODE_INVALID == pDevicePrivateData->FWInfo.nOpcodeQueryTimestamp)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "Command property requires version 5.8.24 or higher...");
        return XN_STATUS_OK;
    }

    XnUInt16 dataSize = 0;
    XnUInt16 opcode = pDevicePrivateData->FWInfo.nOpcodeQueryTimestamp;
    XnUInt16 headerSize = pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    XnUChar sendBuf[MAX_PACKET_SIZE] = { 0 };
    XnHostProtocolInitHeader(pDevicePrivateData, sendBuf, dataSize, opcode);

    XnUChar* pRecv = NULL;
    XnUInt16 recvSize = 0;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData, sendBuf, headerSize + dataSize, opcode, &pRecv, recvSize);
    if (XN_STATUS_OK != rc || NULL == pRecv)
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to query device timestamp: opcode (%d), error (%s).",
            opcode, xnGetStatusString(rc));
        return rc;
    }

    if (recvSize * 2 != sizeof(XnUInt64))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Failed to query device timestamp: data size mismatch(%d != %d), opcode (%d).",
            recvSize, sizeof(XnUInt64), opcode);
        return XN_STATUS_ERROR;
    }

    *pTimestamp = XN_PREPARE_VAR64_IN_BUFFER(*(XnUInt64*)pRecv);

    // All is good.
    return XN_STATUS_OK;
}

XnStatus XnHostProtocolSetTecEnable(XnDevicePrivateData* pDevicePrivateData, XnBool bActive)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER((XnUInt16)bActive);

    XnUInt16 nRequestSize = sizeof(XnUInt16);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetTecEnable);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetTecEnable,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Host protocol set Tec enable Success!,status =%d", bActive);
    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolSetSubtractBGMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 nValue)
{
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };
    XnUChar* pDataBuf = buffer + pDevicePrivateData->FWInfo.nProtocolHeaderSize;

    *(XnUInt16*)pDataBuf = XN_PREPARE_VAR16_IN_BUFFER(nValue);

    XnUInt16 nRequestSize = sizeof(XnUInt16);

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetSubTractBGMode);

    XnUInt16 nDataSize;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeSetSubTractBGMode,
        NULL, nDataSize);
    XN_IS_STATUS_OK(rc);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Host protocol set subtract backgroud mode  success!");

    return (XN_STATUS_OK);
}

XnStatus XnHostProtocolGetSubtractBGMode(XnDevicePrivateData* pDevicePrivateData, XnUInt32 &nValue)
{
    //
    XnUChar buffer[MAX_PACKET_SIZE] = { 0 };

    XnUInt16 nRequestSize = 0;

    XnHostProtocolInitHeader(pDevicePrivateData, buffer, nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetSubTractBGMode);

    XnUInt16 nDataSize;
    XnUInt32* pValue = NULL;
    XnStatus rc = XnHostProtocolExecute(pDevicePrivateData,
        buffer, pDevicePrivateData->FWInfo.nProtocolHeaderSize + (XnUInt16)nRequestSize, pDevicePrivateData->FWInfo.nOpcodeGetSubTractBGMode,
        (XnUChar**)(&pValue), nDataSize);

    XN_IS_STATUS_OK(rc);

    if (nDataSize * 2 != sizeof(XnUInt16))
    {
        xnLogError(XN_MASK_SENSOR_PROTOCOL, "Host protocol get subtract backgroud mode failed!");
        return XN_STATUS_ERROR;
    }

    nValue = (XnUInt16)XN_PREPARE_VAR16_IN_BUFFER(*pValue);

    xnLogVerbose(XN_MASK_SENSOR_PROTOCOL, "Host protocol get subtract backgroud success!");

    return (XN_STATUS_OK);
}