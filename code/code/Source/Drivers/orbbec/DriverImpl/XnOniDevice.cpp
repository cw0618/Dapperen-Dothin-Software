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
#include "XnOniDevice.h"
#include "XnOniStream.h"
#include "XnOniDriver.h"
#include "../Sensor/XnDeviceEnumeration.h"
#include "../DDK/XnPropertySetInternal.h"

#define OBEXT "OBExtensionDriv"
#define EATCH_PACKET_SIZE 32
#define HW_D2C_TYPE 0
#define HW_DISTORTION_TYPE 1
#define CFG_IR_GAIN_TYPE 0
#define CFG_IR_EXP_TYPE 1

#define CFG_LASER_CURRENT_TYPE 0
#define CFG_LASER_TIME_TYPE 1

#include "mx6000_cfg.h"
#include <iostream>
using namespace std;

//ORBBEC
#if 0
#define FLASH_SERIALNUMBER           0x30000
#define FLASH_CAM_PARAMS             0x70000
#define FLASH_RGB_INIT_TABLE_ADDR    0x80000
#define FLASH_D2C_MATRIX_ADDR        0x90000
#define FLASH_REF_ADDR               0xA0000
#define FLASH_IR_GAIN                0x130000
#define FLASH_TEC_LDP_FLAG           0x1A0000

#define MX6000_I2C_IR_GAIN 0x3509
#define MX4000_I2C_IR_GAIN 0x0035
#define I2C_IR_EXP 0x0009
#endif

#define XN_MASK_ONI_DEVICE_SENSOR "OniDeviceSensor"

//---------------------------------------------------------------------------
// XnOniDevice class
//---------------------------------------------------------------------------
XnOniDevice::XnOniDevice(const XnChar* uri, oni::driver::DriverServices& driverServices, XnOniDriver* pDriver)
    : m_driverServices(driverServices)
    , m_pDriver(pDriver)
    , m_pCalibration(NULL)
{
    xnOSMemCopy(&m_info, XnDeviceEnumeration::GetDeviceInfo(uri), sizeof(m_info));
    seq_num = 0x00;
    m_bUpdateFlag = FALSE;
}

XnOniDevice::~XnOniDevice()
{
    if (m_pCalibration)
    {
        XN_DELETE(m_pCalibration);
        m_pCalibration = NULL;
    }

    // free the allocated arrays
    for (int i = 0; i < m_numSensors; ++i)
    {
        XN_DELETE_ARR(m_sensors[i].pSupportedVideoModes);
    }

    m_sensor.Destroy();
}

XnStatus XnOniDevice::FillSupportedVideoModes()
{
    XnUInt32 nSupportedModes = 0;
    XnCmosPreset* pSupportedModes = NULL;

    int s = 0;

    // Depth
    nSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.depthModes.GetSize();
    pSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.depthModes.GetData();

    m_sensors[s].sensorType = ONI_SENSOR_DEPTH;
    m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * 3);
    XN_VALIDATE_ALLOC_PTR(m_sensors[s].pSupportedVideoModes);

    OniPixelFormat depthFormats[] = { ONI_PIXEL_FORMAT_DEPTH_1_MM, ONI_PIXEL_FORMAT_DEPTH_100_UM };
    XnSizeT depthFormatsCount = sizeof(depthFormats) / sizeof(depthFormats[0]);

    int writeIndex = 0;
    for (XnUInt32 i = 0; i < nSupportedModes; ++i)
    {
        for (XnSizeT formatIndex = 0; formatIndex < depthFormatsCount; ++formatIndex)
        {
            m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat = depthFormats[formatIndex];
            m_sensors[s].pSupportedVideoModes[writeIndex].fps = pSupportedModes[i].nFPS;
            XnBool bOK = XnDDKGetXYFromResolution(
                (XnResolutions)pSupportedModes[i].nResolution,
                (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX,
                (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY
                );
            XN_ASSERT(bOK);
            XN_REFERENCE_VARIABLE(bOK);

            bool foundMatch = false;
            for (int j = 0; j < writeIndex; ++j)
            {
                if (m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat == m_sensors[s].pSupportedVideoModes[j].pixelFormat &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].fps == m_sensors[s].pSupportedVideoModes[j].fps &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX == m_sensors[s].pSupportedVideoModes[j].resolutionX &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY == m_sensors[s].pSupportedVideoModes[j].resolutionY)
                {
                    // Already know this configuration
                    foundMatch = true;
                    break;
                }
            }
            if (!foundMatch)
            {
                ++writeIndex;
            }
        }
    }
    m_sensors[s].numSupportedVideoModes = writeIndex;

    // Image
    // first, make sure that our sensor actually supports Image
    XnUInt64 nImageSupported = FALSE;
    XnStatus nRetVal = m_sensor.GetProperty(XN_MASK_DEVICE, XN_MODULE_PROPERTY_IMAGE_SUPPORTED, &nImageSupported);
    XN_IS_STATUS_OK(nRetVal);
    if (nImageSupported)
    {
        ++s;
        nSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.imageModes.GetSize();
        pSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.imageModes.GetData();

        m_sensors[s].sensorType = ONI_SENSOR_COLOR;
        m_sensors[s].numSupportedVideoModes = 0; // to be changed later..
        m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * 10);
        XN_VALIDATE_ALLOC_PTR(m_sensors[s].pSupportedVideoModes);

        writeIndex = 0;
        for (XnUInt32 j = 0; j < nSupportedModes; ++j)
        {
            // make an OniVideoMode for each OniFormat supported by the input format
            OniPixelFormat aOniFormats[10];
            int	  nOniFormats = 0;
            XnOniColorStream::GetAllowedOniOutputFormatForInputFormat((XnIOImageFormats)pSupportedModes[j].nFormat, aOniFormats, &nOniFormats);
            for (int curOni = 0; curOni < nOniFormats; ++curOni)
            {
                m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat = aOniFormats[curOni];

                m_sensors[s].pSupportedVideoModes[writeIndex].fps = pSupportedModes[j].nFPS;
                XnBool bOK = XnDDKGetXYFromResolution(
                    (XnResolutions)pSupportedModes[j].nResolution,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY
                    );
                XN_ASSERT(bOK);
                XN_REFERENCE_VARIABLE(bOK);

                bool foundMatch = false;
                for (int i = 0; i < writeIndex; ++i)
                {
                    if (m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat == m_sensors[s].pSupportedVideoModes[i].pixelFormat &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].fps == m_sensors[s].pSupportedVideoModes[i].fps &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX == m_sensors[s].pSupportedVideoModes[i].resolutionX &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY == m_sensors[s].pSupportedVideoModes[i].resolutionY)
                    {
                        // Already know this configuration
                        foundMatch = true;
                        break;
                    }
                }
                if (!foundMatch)
                {
                    ++writeIndex;
                }
            }
        }
        m_sensors[s].numSupportedVideoModes = writeIndex;
    }

    // IR
    ++s;
    nSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.irModes.GetSize();
    pSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.irModes.GetData();

    m_sensors[s].sensorType = ONI_SENSOR_IR;
    m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * 3);
    XN_VALIDATE_ALLOC_PTR(m_sensors[s].pSupportedVideoModes);

    OniPixelFormat irFormats[] = { ONI_PIXEL_FORMAT_GRAY16, ONI_PIXEL_FORMAT_GRAY8, ONI_PIXEL_FORMAT_RGB888 };
    writeIndex = 0;
    for (XnUInt32 i = 0; i < nSupportedModes; ++i)
    {
        for (int fmt = 0; fmt <= 2; ++fmt)
        {
            m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat = irFormats[fmt];
            m_sensors[s].pSupportedVideoModes[writeIndex].fps = pSupportedModes[i].nFPS;
            XnBool bOK = XnDDKGetXYFromResolution(
                (XnResolutions)pSupportedModes[i].nResolution,
                (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX,
                (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY
                );
            XN_ASSERT(bOK);
            XN_REFERENCE_VARIABLE(bOK);

            bool foundMatch = false;
            for (int j = 0; j < writeIndex; ++j)
            {
                if (m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat == m_sensors[s].pSupportedVideoModes[j].pixelFormat &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].fps == m_sensors[s].pSupportedVideoModes[j].fps &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX == m_sensors[s].pSupportedVideoModes[j].resolutionX &&
                    m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY == m_sensors[s].pSupportedVideoModes[j].resolutionY)
                {
                    // Already know this configuration
                    foundMatch = true;
                    break;
                }
            }
            if (!foundMatch)
            {
                ++writeIndex;
            }
        }
    }
    m_sensors[s].numSupportedVideoModes = writeIndex;

    // Phase
    XnUInt64 nPhaseSupported = FALSE;
    nRetVal = m_sensor.GetProperty(XN_MASK_DEVICE, XN_MODULE_PROPERTY_PHASE_SUPPORTED, &nPhaseSupported);
    XN_IS_STATUS_OK(nRetVal);
    if (nPhaseSupported)
    {
        ++s;
        nSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.phaseModes.GetSize();
        pSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.phaseModes.GetData();

        OniPixelFormat formats[] = { ONI_PIXEL_FORMAT_GRAY16, ONI_PIXEL_FORMAT_RGB888 };
        XnUInt32 numFormats = sizeof(formats) / sizeof(formats[0]);

        m_sensors[s].sensorType = ONI_SENSOR_PHASE;
        m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * numFormats);
        XN_VALIDATE_ALLOC_PTR(m_sensors[s].pSupportedVideoModes);

        writeIndex = 0;
        for (XnUInt32 i = 0; i < nSupportedModes; ++i)
        {
            for (XnUInt32 j = 0; j < numFormats; ++j)
            {
                m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat = formats[j];
                m_sensors[s].pSupportedVideoModes[writeIndex].fps = pSupportedModes[i].nFPS;

                XnBool bOK = XnDDKGetXYFromResolution(
                    (XnResolutions)pSupportedModes[i].nResolution,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY);
                XN_ASSERT(bOK);
                XN_REFERENCE_VARIABLE(bOK);

                bool foundMatch = false;
                for (int z = 0; z < writeIndex; ++z)
                {
                    if (m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat == m_sensors[s].pSupportedVideoModes[z].pixelFormat &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].fps == m_sensors[s].pSupportedVideoModes[z].fps &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX == m_sensors[s].pSupportedVideoModes[z].resolutionX &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY == m_sensors[s].pSupportedVideoModes[z].resolutionY)
                    {
                        /// Already know this configuration
                        foundMatch = true;
                        break;
                    }
                }
                if (!foundMatch)
                    ++writeIndex;
            }
        }
    }
    m_sensors[s].numSupportedVideoModes = writeIndex;

    // AI
    XnUInt64 nAISupported = FALSE;
    nRetVal = m_sensor.GetProperty(XN_MASK_DEVICE, XN_MODULE_PROPERTY_AI_SUPPORTED, &nAISupported);
    XN_IS_STATUS_OK(nRetVal);
    if (nAISupported)
    {
        ++s;
        nSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.AIModes.GetSize();
        pSupportedModes = m_sensor.GetDevicePrivateData()->FWInfo.AIModes.GetData();

        OniPixelFormat formats[] =
        {
            ONI_PIXEL_FORMAT_JOINT_2D,
            ONI_PIXEL_FORMAT_JOINT_3D,
            ONI_PIXEL_FORMAT_BODY_MASK,
            ONI_PIXEL_FORMAT_FLOOR_INFO,
            ONI_PIXEL_FORMAT_BODY_SHAPE,
            ONI_PIXEL_FORMAT_PHASE,
            ONI_PIXEL_FORMAT_DEPTH_IR
        };
        XnUInt32 numFormats = sizeof(formats) / sizeof(formats[0]);

        m_sensors[s].sensorType = ONI_SENSOR_AI;
        m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * numFormats);
        XN_VALIDATE_ALLOC_PTR(m_sensors[s].pSupportedVideoModes);

        writeIndex = 0;
        for (XnUInt32 i = 0; i < nSupportedModes; ++i)
        {
            for (XnUInt32 j = 0; j < numFormats; ++j)
            {
                m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat = formats[j];
                m_sensors[s].pSupportedVideoModes[writeIndex].fps = pSupportedModes[i].nFPS;

                XnBool bOK = XnDDKGetXYFromResolution(
                    (XnResolutions)pSupportedModes[i].nResolution,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX,
                    (XnUInt32*)&m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY);
                XN_ASSERT(bOK);
                XN_REFERENCE_VARIABLE(bOK);

                bool foundMatch = false;
                for (int z = 0; z < writeIndex; ++z)
                {
                    if (m_sensors[s].pSupportedVideoModes[writeIndex].pixelFormat == m_sensors[s].pSupportedVideoModes[z].pixelFormat &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].fps == m_sensors[s].pSupportedVideoModes[z].fps &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionX == m_sensors[s].pSupportedVideoModes[z].resolutionX &&
                        m_sensors[s].pSupportedVideoModes[writeIndex].resolutionY == m_sensors[s].pSupportedVideoModes[z].resolutionY)
                    {
                        /// Already know this configuration
                        foundMatch = true;
                        break;
                    }
                }
                if (!foundMatch)
                    ++writeIndex;
            }
        }
    }
    m_sensors[s].numSupportedVideoModes = writeIndex;

    m_numSensors = s + 1;

    return XN_STATUS_OK;
}

XnStatus XnOniDevice::Init(const char* mode)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XN_PROPERTY_SET_CREATE_ON_STACK(initialValues);

    if (mode != NULL)
    {
        nRetVal = XnPropertySetAddModule(&initialValues, XN_MODULE_NAME_DEVICE);
        XN_IS_STATUS_OK(nRetVal);

        for (int i = 0; mode[i] != '\0'; ++i)
        {
            switch (mode[i])
            {
            case 'L':
                nRetVal = XnPropertySetAddIntProperty(&initialValues, XN_MODULE_NAME_DEVICE, XN_MODULE_PROPERTY_LEAN_INIT, TRUE);
                XN_IS_STATUS_OK(nRetVal);
                break;
            case 'R':
                nRetVal = XnPropertySetAddIntProperty(&initialValues, XN_MODULE_NAME_DEVICE, XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP, FALSE);
                XN_IS_STATUS_OK(nRetVal);
                break;
            }
        }
    }

    XnDeviceConfig config;
    config.cpConnectionString = m_info.uri;
    config.pInitialValues = &initialValues;
    XnStatus retVal = m_sensor.Init(&config);
    XN_IS_STATUS_OK(retVal);

    m_sensor.SetDevicePID(m_info.usbProductId);

    nRetVal = FillSupportedVideoModes();
    XN_IS_STATUS_OK(nRetVal);

    memset(&m_Ext, 0, sizeof(OBExtension));
    m_Ext.cam_tag = 0;
    m_Ext.vid = m_info.usbVendorId;
    m_Ext.pid = m_info.usbProductId;

    return XN_STATUS_OK;
}

OniStatus XnOniDevice::getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
{
    *numSensors = m_numSensors;
    *pSensors = m_sensors;

    return ONI_STATUS_OK;
}

oni::driver::StreamBase* XnOniDevice::createStream(OniSensorType sensorType)
{
    XnOniStream* pStream = NULL;
    switch (sensorType)
    {
    case ONI_SENSOR_IR:
        pStream = XN_NEW(XnOniIRStream, &m_sensor, this);
        break;
    case ONI_SENSOR_DEPTH:
        pStream = XN_NEW(XnOniDepthStream, &m_sensor, this);
        break;
    case ONI_SENSOR_COLOR:
        pStream = XN_NEW(XnOniColorStream, &m_sensor, this);
        break;
    case ONI_SENSOR_PHASE:
        pStream = XN_NEW(XnOniPhaseStream, &m_sensor, this);
        break;
    case ONI_SENSOR_AI:
        pStream = XN_NEW(XnOniAIStream, &m_sensor, this);
        break;
    default:
        m_driverServices.errorLoggerAppend("XnOniDevice: Can't create a stream of type %d", sensorType);
        return NULL;
    }

    XnStatus nRetVal = pStream->Init();
    if (nRetVal != XN_STATUS_OK)
    {
        m_driverServices.errorLoggerAppend("XnOniDevice: Can't initialize stream of type %d: %s", sensorType, xnGetStatusString(nRetVal));
        XN_DELETE(pStream);
        return NULL;
    }

    return pStream;
}

void XnOniDevice::destroyStream(oni::driver::StreamBase* pStream)
{
    XN_DELETE(pStream);
}

OniStatus XnOniDevice::getProperty(int propertyId, void* data, int* pDataSize)
{
    OniStatus rc = ONI_STATUS_OK;

    switch (propertyId)
    {
    case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        XnUInt32 nCharsWritten = 0;
        XnStatus rc = xnOSStrFormat((XnChar*)data, *pDataSize, &nCharsWritten, "%d.%d.%d", versions.nMajor, versions.nMinor, versions.nBuild);
        if (rc != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Couldn't get firmware version: %s\n", xnGetStatusString(rc));
            return ONI_STATUS_BAD_PARAMETER;
        }
        *pDataSize = nCharsWritten + 1;

        break;
    }
    case ONI_DEVICE_PROPERTY_HARDWARE_VERSION:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        int hwVer = versions.HWVer;
        if (*pDataSize == sizeof(int))
        {
            (*((int*)data)) = hwVer;
        }
        else if (*pDataSize == sizeof(short))
        {
            (*((short*)data)) = (short)hwVer;
        }
        else if (*pDataSize == sizeof(uint64_t))
        {
            (*((uint64_t*)data)) = (uint64_t)hwVer;
        }
        else
        {
            m_driverServices.errorLoggerAppend("Unexpected size: %d != %d or %d or %d\n", *pDataSize, sizeof(short), sizeof(int), sizeof(uint64_t));
            return ONI_STATUS_ERROR;
        }
        break;
    }
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case OBEXTENSION_ID_SERIALNUMBER:
    {
        XnStatus rc = m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_SERIAL_NUMBER, data, pDataSize);
        if (rc != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Couldn't get serial number: %s\n", xnGetStatusString(rc));
            return ONI_STATUS_BAD_PARAMETER;
        }

        break;
    }
    case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
    {
        if (*pDataSize == sizeof(OniVersion))
        {
            OniVersion* version = (OniVersion*)data;
            version->major = XN_PS_MAJOR_VERSION;
            version->minor = XN_PS_MINOR_VERSION;
            version->maintenance = XN_PS_MAINTENANCE_VERSION;
            version->build = XN_PS_BUILD_VERSION;
        }
        else
        {
            m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
            return ONI_STATUS_ERROR;
        }
    }
    break;
    case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
    {
        if (*pDataSize == sizeof(OniImageRegistrationMode))
        {
            OniImageRegistrationMode* mode = (OniImageRegistrationMode*)data;

            // Find the depth stream in the sensor.
            XnDeviceStream* pDepth = NULL;
            XnStatus xnrc = m_sensor.GetStream(XN_STREAM_NAME_DEPTH, &pDepth);
            if (xnrc != XN_STATUS_OK)
            {
                return ONI_STATUS_BAD_PARAMETER;
            }

            // Set the mode in the depth stream.
            XnUInt64 val;
            xnrc = pDepth->GetProperty(XN_STREAM_PROPERTY_REGISTRATION, &val);
            if (xnrc != XN_STATUS_OK)
            {
                return ONI_STATUS_ERROR;
            }

            // Update the return value.
            *mode = (val == 1) ? ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR : ONI_IMAGE_REGISTRATION_OFF;
        }
        else
        {
            m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniImageRegistrationMode));
            return ONI_STATUS_ERROR;
        }
    }
    break;
    case OBEXTENSION_ID_IR_GAIN:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //get irgain
            rc = ObGetIRGain(data, pDataSize);
        }
        else
        {
            XnUInt16 value = 0;
            XnHostProtocolGetCMOSRegisterI2C(m_sensor.GetDevicePrivateData(), XN_CMOS_TYPE_DEPTH, MX4000_I2C_IR_GAIN, value);
            *(XnUInt16*)data = value;
        }

    }
    break;
    case OBEXTENSION_ID_IR_EXP:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //get irexp
            rc = ObGetIRExp(data, pDataSize);
        }
        else
        {

            XnUInt16 value = 0;
            XnHostProtocolGetCMOSRegisterI2C(m_sensor.GetDevicePrivateData(), XN_CMOS_TYPE_DEPTH, I2C_IR_EXP, value);
            *(XnUInt16*)data = value;
        }
    }
    break;
    case OBEXTENSION_ID_LDP_EN:
    case OBEXTENSION_ID_CAM_PARAMS:
    case OBEXTENSION_ID_LASER_EN:
    case OBEXTENSION_ID_DEVICETYPE:
        rc = OBExtension_GetProperty(propertyId, data, *pDataSize);
        break;
    case XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK:
    {
        XnParamFlashData *flashData = (XnParamFlashData *)data;
        rc = UpdateFirmwareReadFlash((uint8_t *)flashData->pData, flashData->nSize, flashData->nOffset);
    }
    break;
    case XN_MODULE_PROPERTY_IRGAIN_FLASH:
    {

        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = GetCfgIrGainExp(data, pDataSize, CFG_IR_GAIN_TYPE);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_IREXP_FLASH:
    {

        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = GetCfgIrGainExp(data, pDataSize, CFG_IR_EXP_TYPE);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;

    case XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD_FLASH:
    {

        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = GetCfgPostfiterThreshold(data, pDataSize);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    //
    case XN_MODULE_PROPERTY_LASER_CURRENT_FLASH:
    {

        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = GetCfgLaserCurrentOrTime(data, pDataSize, CFG_LASER_CURRENT_TYPE);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_LASER_TIME_FLASH:
    {

        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = GetCfgLaserCurrentOrTime(data, pDataSize, CFG_LASER_TIME_TYPE);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case ONI_DEVICE_PROPERTY_STATE_SUBTRACT_BG:
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_STATE_SUBTRACT_BG, data, pDataSize);
        if (nRetVal != ONI_STATUS_OK){
            return ONI_STATUS_NOT_SUPPORTED;
        }
        return ONI_STATUS_OK;
    }
    break;
    case ONI_DEVICE_PROPERTY_FREQUENCY_MODE:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_TOF_FREQ_MODE, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_SENSOR_ID:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_SENSOR_ID, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_PLATFORM_VERSION:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_PLATFORM_VERSION, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_GENERAL_SERIAL_NUMBER:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_GENERAL_SERIAL_NUMBER, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_EMITTER_STATE:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_EMITTER_STATE, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_QUERY_DEVICE_TIMESTAMP:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_QUERY_DEVICE_TIMESTAMP, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_TEMPERATURE_COEFFICIENT_RX:
    {
        return (OniStatus)m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_IR_TEMP_COMP_CO, data, pDataSize);
    }
    case ONI_DEVICE_PROPERTY_CALIBRATION_CAMERA:
    {
        if (*pDataSize != sizeof(OniCalibrationCamera))
        {
            m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniCalibrationCamera));
            return ONI_STATUS_ERROR;
        }

        return getCalibration((OniCalibrationCamera *)data);
    }
    default:
        XnStatus nRetVal = m_sensor.DeviceModule()->GetProperty(propertyId, data, pDataSize);
        switch (nRetVal)
        {
        case XN_STATUS_OK:
            rc = ONI_STATUS_OK;
            break;
        case XN_STATUS_IO_DEVICE_NOT_WRITE_PUBLIC_KEY:
            rc = ONI_STATUS_NOT_WRITE_PUBLIC_KEY;
            break;
        case XN_STATUS_IO_DEVICE_PUBLIC_KEY_MD5_VERIFY_FAIL:
            rc = ONI_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED;
            break;
        case XN_STATUS_IO_DEVICE_NOT_WRITE_MD5:
            rc = ONI_STATUS_NOT_WRITE_MD5;
            break;
        case XN_STATUS_DEVICE_UNSUPPORTED_PARAMETER:
            rc = ONI_STATUS_NOT_SUPPORTED;
            break;
        default:
        {
            m_driverServices.errorLoggerAppend("Failed to get property %x: %s", propertyId, xnGetStatusString(nRetVal));
            rc = ONI_STATUS_BAD_PARAMETER;
        }
        break;
        }
        break;
    }

    return rc;
}

OniStatus XnOniDevice::setProperty(int propertyId, const void* data, int dataSize)
{
    OniStatus rc = ONI_STATUS_OK;

    switch (propertyId)
    {
    case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
    {
        if (dataSize == sizeof(OniImageRegistrationMode))
        {
            OniImageRegistrationMode* mode = (OniImageRegistrationMode*)data;

            // Find the depth stream in the sensor.
            XnDeviceStream* pDepth = NULL;
            XnStatus xnrc = m_sensor.GetStream(XN_STREAM_NAME_DEPTH, &pDepth);
            if (xnrc != XN_STATUS_OK)
            {
                return ONI_STATUS_BAD_PARAMETER;
            }

            // Set the mode in the depth stream.
            XnUInt64 val = (*mode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR) ? 1 : 0;
            xnrc = pDepth->SetProperty(XN_STREAM_PROPERTY_REGISTRATION, val);
            if (xnrc != XN_STATUS_OK)
            {
                return ONI_STATUS_ERROR;
            }
        }
        else
        {
            m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", dataSize, sizeof(OniImageRegistrationMode));
            return ONI_STATUS_ERROR;
        }
    }
    break;
    case OBEXTENSION_ID_IR_GAIN:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            // rc = (OniStatus)XnHostProtocolSetCMOSRegisterI2C(m_sensor.GetDevicePrivateData(), XN_CMOS_TYPE_DEPTH, MX6000_I2C_IR_GAIN, *((uint8_t*)data));
            //set gain
            rc = ObSetIRGain(data, dataSize);
        }
        else
        {
            rc = (OniStatus)XnHostProtocolSetCMOSRegisterI2C(m_sensor.GetDevicePrivateData(), XN_CMOS_TYPE_DEPTH, MX4000_I2C_IR_GAIN, *((uint8_t*)data));
        }
        //xnLogError(OBEXT, "setGain: %d, ret: %d !", *((uint16_t*)data), ret);
    }
    break;
    case OBEXTENSION_ID_IR_EXP:
    {
#if 0
        uint8_t readbuf[2];
        uint8_t cmdbuf[16];
        //cmdbuf[0] = *((int8_t*)data);
        *(uint16_t*)cmdbuf = 0x0001;
        *(((uint16_t*)cmdbuf)+1) = 0x0009;
        *(((uint16_t*)cmdbuf)+2) = *((uint16_t*)data);
        //rc = SendCmd(CMD_EXP_SET, cmdbuf, 2, readbuf, 2);
        rc = SendCmd(OPCODE_I2C_WRITE, cmdbuf, 6, readbuf, 2);
#endif
        //set exp
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = ObSetIRExp(data, dataSize);
        }
        else
        {
            XnHostProtocolSetCMOSRegisterI2C(m_sensor.GetDevicePrivateData(), XN_CMOS_TYPE_DEPTH, I2C_IR_EXP, *((uint16_t*)data));
        }

    }
    break;
    case OBEXTENSION_ID_LDP_EN:
    case OBEXTENSION_ID_CAM_PARAMS:
    case OBEXTENSION_ID_LASER_EN:
    case OBEXTENSION_ID_SERIALNUMBER:
    case OBEXTENSION_ID_DEVICETYPE:
    case OBEXTENSION_ID_UPDATE_FIRMWARE:
        rc = OBExtension_SetProperty(propertyId, data, dataSize);
        break;
    case XN_MODULE_PROPERTY_LASER_SECURE_KEEPALIVE:
    {
        XnStatus status = XnHostProtocolKeepAlive(m_sensor.GetDevicePrivateData());
        if (status != XN_STATUS_OK)
        {
            return  ONI_STATUS_ERROR;
        }
    }
    break;
    case XN_MODULE_PROPERTY_UPDATE_FIRMWARE_FLASH_CHUNK:
    {
        XnParamFlashData *flashData = (XnParamFlashData *)data;
        rc = UpdateFirmwareWriteFlash((uint8_t *)flashData->pData, flashData->nSize, flashData->nOffset);
    }
    break;
    case XN_MODULE_PROPERTY_CFG_SNPN:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //set cfg Sn Pn
            rc = SetCfgSn_Pn(data, dataSize);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_Z0_BASELINE:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //set z0 and baseline
            rc = SetCfgZ0_Baseline(data, dataSize);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_WRITE_REF:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //Write reference
            rc = UpdateRef(data, dataSize);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_WRITE_SW_ALIGN_PARAM:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //Write software alignment parameters
            rc = UpdateSwAlignParams(data, dataSize);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_WRITE_HW_ALIGN_PARAM:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //Write hardware alignment parameters
            int nSystem32 = sizeof(int *);
            if (nSystem32 == 4)
            {
                OBCfgHwD2cDistortion *d2cData = (OBCfgHwD2cDistortion *)data;
                rc = SetCfgParam(d2cData->data, d2cData->nSize, d2cData->nRegNum, HW_D2C_TYPE);
            }
            else
            {
                return ONI_STATUS_NOT_SUPPORTED;
            }

        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_WRITE_HW_DISTORTION_PARAM:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            int nSystem32 = sizeof(int *);
            if (nSystem32 == 4)
            {
                //Write hardware distortion parameters
                rc = SetCfgParam(data, dataSize, 1, HW_DISTORTION_TYPE);
            }
            else
            {
                return ONI_STATUS_NOT_SUPPORTED;
            }
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_WRITE_DEPTH_CONFIG:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = SetDepthConfig(data, dataSize);
        }
        else {
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_IRGAIN_FLASH:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //set cfg flash ir gain
            rc = SetCfgIrGainExp(data, dataSize, CFG_IR_GAIN_TYPE);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case XN_MODULE_PROPERTY_IREXP_FLASH:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //set cfg flash ir gain
            rc = SetCfgIrGainExp(data, dataSize, CFG_IR_EXP_TYPE);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;

    case XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD_FLASH:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //set post filter threshold
            rc = SetCfgPostfiterThreshold(data, dataSize);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;

    case XN_MODULE_PROPERTY_LASER_CURRENT_FLASH:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = SetCfgLaserCurrentOrTime(data, dataSize, CFG_LASER_CURRENT_TYPE);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;

    case XN_MODULE_PROPERTY_LASER_TIME_FLASH:
    {
        XnVersions &versions = m_sensor.GetDevicePrivateData()->Version;
        if (versions.ChipVer == XN_SENSOR_CHIP_VER_MX6000 || versions.ChipVer == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            rc = SetCfgLaserCurrentOrTime(data, dataSize, CFG_LASER_TIME_TYPE);
        }
        else{
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    break;
    case ONI_DEVICE_PROPERTY_ENABLE_SUBTRACT_BG:
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_ENABLE_SUBTRACT_BG, data, dataSize);
        if (nRetVal != ONI_STATUS_OK){
            return ONI_STATUS_NOT_SUPPORTED;
        }
        return ONI_STATUS_OK;
    }
    case ONI_DEVICE_PROPERTY_FREQUENCY_MODE:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_TOF_FREQ_MODE, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_GENERAL_SERIAL_NUMBER:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_GENERAL_SERIAL_NUMBER, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_SEND_COMMAND:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_SEND_COMMAND, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_EMITTER_STATE:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_EMITTER_STATE, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_LOAD_FILE:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_USB_GRNERAL_FILE, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_START_SERVICE:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_START_SERVICE, data, dataSize);
    }
    case ONI_DEVICE_PROPERTY_TEMPERATURE_COEFFICIENT_RX:
    {
        return (OniStatus)m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_IR_TEMP_COMP_CO, data, dataSize);
    }
    default:
        XnStatus nRetVal = m_sensor.DeviceModule()->SetProperty(propertyId, data, dataSize);
        switch (nRetVal)
        {
        case XN_STATUS_OK:
            rc = ONI_STATUS_OK;
            break;
        case XN_STATUS_IO_DEVICE_NOT_WRITE_PUBLIC_KEY:
            rc = ONI_STATUS_NOT_WRITE_PUBLIC_KEY;
            break;
        case XN_STATUS_IO_DEVICE_PUBLIC_KEY_MD5_VERIFY_FAIL:
            rc = ONI_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED;
            break;
        case XN_STATUS_IO_DEVICE_NOT_WRITE_MD5:
            rc = ONI_STATUS_NOT_WRITE_MD5;
            break;
        case XN_STATUS_IO_DEVICE_RSKEY_VERIFY_FAIL:
            rc = ONI_STATUS_RSKEY_VERIFY_FAILED;
            break;
        default:
        {
            m_driverServices.errorLoggerAppend("Failed to set property %x: %s", propertyId, xnGetStatusString(nRetVal));
            rc = ONI_STATUS_BAD_PARAMETER;
        }
        break;
        }
        break;
    }

    return rc;
}

OniBool XnOniDevice::isPropertySupported(int propertyId)
{
    if (propertyId == ONI_DEVICE_PROPERTY_DRIVER_VERSION ||
        propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION ||
        propertyId == ONI_DEVICE_PROPERTY_FIRMWARE_VERSION ||
        propertyId == ONI_DEVICE_PROPERTY_HARDWARE_VERSION ||
        propertyId == ONI_DEVICE_PROPERTY_SERIAL_NUMBER)
    {
        return TRUE;
    }
    else
    {
        XnBool propertyExists = FALSE;
        m_sensor.DeviceModule()->DoesPropertyExist(propertyId, &propertyExists);
        return propertyExists;
    }
}

void XnOniDevice::notifyAllProperties()
{
    XnUInt32 nValue = (XnUInt32)m_sensor.GetCurrentUsbInterface();
    int size = sizeof(nValue);
    raisePropertyChanged(XN_MODULE_PROPERTY_USB_INTERFACE, &nValue, sizeof(nValue));

    nValue = m_sensor.GetDeviceMirror();
    raisePropertyChanged(XN_MODULE_PROPERTY_MIRROR, &nValue, sizeof(nValue));

    nValue = m_sensor.GetDeviceMirror();
    raisePropertyChanged(XN_STREAM_PROPERTY_CLOSE_RANGE, &nValue, sizeof(nValue));

    getProperty(XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP, &nValue, &size);
    raisePropertyChanged(XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP, &nValue, sizeof(nValue));

    getProperty(XN_MODULE_PROPERTY_LEAN_INIT, &nValue, &size);
    raisePropertyChanged(XN_MODULE_PROPERTY_LEAN_INIT, &nValue, sizeof(nValue));

    XnChar strValue[XN_DEVICE_MAX_STRING_LENGTH] = { 0 };
    size = sizeof(strValue);
    getProperty(XN_MODULE_PROPERTY_SERIAL_NUMBER, strValue, &size);
    raisePropertyChanged(XN_MODULE_PROPERTY_SERIAL_NUMBER, strValue, size);

    XnVersions versions;
    size = sizeof(versions);
    getProperty(XN_MODULE_PROPERTY_VERSION, &versions, &size);
    raisePropertyChanged(XN_MODULE_PROPERTY_VERSION, &versions, size);
}

OniStatus XnOniDevice::EnableFrameSync(XnOniStream** pStreams, int streamCount)
{
    // Translate the XnOniStream to XnDeviceStream.
    xnl::Array<XnDeviceStream*> streams(streamCount);
    streams.SetSize(streamCount);
    for (int i = 0; i < streamCount; ++i)
    {
        streams[i] = pStreams[i]->GetDeviceStream();
    }

    // Set the frame sync group.
    XnStatus rc = m_sensor.SetFrameSyncStreamGroup(streams.GetData(), streamCount);
    if (rc != XN_STATUS_OK)
    {
        m_driverServices.errorLoggerAppend("Error setting frame-sync group (rc=%d)\n", rc);
        return ONI_STATUS_ERROR;
    }

    return ONI_STATUS_OK;
}

void XnOniDevice::DisableFrameSync()
{
    XnStatus rc = m_sensor.SetFrameSyncStreamGroup(NULL, 0);
    if (rc != XN_STATUS_OK)
    {
        m_driverServices.errorLoggerAppend("Error setting frame-sync group (rc=%d)\n", rc);
    }
}

OniBool XnOniDevice::isImageRegistrationModeSupported(OniImageRegistrationMode mode)
{
    return (mode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR || mode == ONI_IMAGE_REGISTRATION_OFF);
}

OniStatus XnOniDevice::OBExtension_GetProperty(int id, void* ptr_data, int dataSize)
{
    OniStatus ret;
    int nMX6000Id = m_sensor.GetDevicePrivateData()->ChipInfo.nChipVer;

#if 1
    if ((m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice == NULL) return ONI_STATUS_ERROR;
    switch (id)
    {
    case OBEXTENSION_ID_IR_GAIN:
        if (dataSize == 4)
        {
            ret = ReadFlash(FLASH_IR_GAIN, 2, (uint8_t *)ptr_data, 4);
            if (*(uint32_t*)ptr_data == 0xFFFFFFFF)
            {
                *(uint32_t*)ptr_data = 0x0060;
            }
        }
        else
        {
            ret = ONI_STATUS_ERROR;
        }
        break;
    case OBEXTENSION_ID_LDP_EN:
        //GetLdp
        if (nMX6000Id == XN_SENSOR_CHIP_VER_MX6000 || nMX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //mx6000
            uint32_t ldp_tec_flag = 0;
            if (dataSize == 4)
            {
                ret = GetLdp(ldp_tec_flag);
                if (ret == ONI_STATUS_OK)
                {
                    //success
                    if (ldp_tec_flag == 0x01)
                    {
                        //
                        *(uint32_t *)ptr_data = 0x01;
                    }
                    else
                    {
                        *(uint32_t *)ptr_data = 0x00;
                    }
                }
                else
                {
                    ret = ONI_STATUS_ERROR;
                }
            }
            else
            {
                ret = ONI_STATUS_ERROR;
            }
        }
        else
        {
            //mx400
            uint32_t ldp_tec_flag;
            if (dataSize == 4)
            {
                ret = ReadFlash(FLASH_TEC_LDP_FLAG, 2, (uint8_t *)&ldp_tec_flag, 4);//get the dword
                if (((ldp_tec_flag >> 13) & 0x01) == 0x01)
                {
                    *(uint32_t *)ptr_data = 0x01;
                }
                else
                {
                    *(uint32_t *)ptr_data = 0x00;
                }
            }
            else
            {
                ret = ONI_STATUS_ERROR;
            }
        }

        break;

    case OBEXTENSION_ID_CAM_PARAMS:
        if (nMX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //
            ret = GetDualCameraParam((uint8_t *)ptr_data, (uint16_t)dataSize);
        }
        else
        {
            uint16_t dwSize = (uint16_t)dataSize;
            if (m_sensor.GetDevicePrivateData()->Version.nMajor >= 5 &&
                m_sensor.GetDevicePrivateData()->Version.nMinor >= 8 &&
                m_sensor.GetDevicePrivateData()->Version.nBuild < 24)
            {
                dwSize = dwSize - sizeof(uint32_t) * 2;
            }
            ret = ReadFlash(FLASH_CAM_PARAMS, dwSize / 2, (uint8_t *)ptr_data, (uint16_t)dataSize);//dump the flash memory
        }

        break;
    case OBEXTENSION_ID_DEVICETYPE:
        if (m_Ext.vid == 0x2bc5)
        {
            switch (m_Ext.pid)
            {
            case 0x0400:
                strcpy((char*)ptr_data, "Unknow Device");
                break;
            case 0x0401:
                strcpy((char*)ptr_data, "Orbbec Astra");
                break;
            case 0x0402:
                strcpy((char*)ptr_data, "Orbbec Astra S");
                break;
            case 0x0403:
                strcpy((char*)ptr_data, "Orbbec Astra Pro");
                break;
            case 0x0404:
                strcpy((char*)ptr_data, "Orbbec Astra Mini");
                break;
            case 0x0405:
                strcpy((char*)ptr_data, "Orbbec Astra Orion");
                break;
            case 0x0406:
                strcpy((char*)ptr_data, "Orbbec Astra Hurley");
                break;
            case 0x0407:
                strcpy((char*)ptr_data, "Orbbec Astra mini S");
                break;
            case 0x0605:
                strcpy((char*)ptr_data, "Orbbec SuperD1");
                break;
            case 0x0606:
                strcpy((char*)ptr_data, "Orbbec LunaP1");
                break;
            case 0x0607:
                strcpy((char*)ptr_data, "Diweitai Ado");
                break;
            case 0x0608:
                strcpy((char*)ptr_data, "Orbbec Canglong");
                break;
            case 0x0609:
                strcpy((char*)ptr_data, "Orbbec P1Pro");
                break;
            case 0x060b:
                strcpy((char*)ptr_data, "Astra SL1000S_U3");
                break;
            case Tornado_D2_PID:
                strcpy((char*)ptr_data, "Orbbec TornadoD2");
                break;
            case Tornado_D2_PRO_PID:
                strcpy((char*)ptr_data, "Orbbec TornadoD2 Pro");
                break;
            case 0x060e:
                strcpy((char*)ptr_data, "Orbbec Dabai");
                break;
            case ASTRAPRO_PLUS:
                strcpy((char*)ptr_data, "Orbbec Astra Pro Plus");
                break;
            case BAIDU_Atlas:
                strcpy((char*)ptr_data, "Orbbec Atlas");
                break;
            case Gemini_PID:
                strcpy((char*)ptr_data, "Astra SV1301S_U3");
                break;
            case PROJECTOR_PID:
                strcpy((char*)ptr_data, "Astra SL1200S_CF");
                break;
            case BUTTERFLY_PID:
                strcpy((char*)ptr_data, "Astra SL1510S_U2");
                break;
            case Petrel_Pro:
                strcpy((char*)ptr_data, "Astra SL1502P_U2");
                break;
            case Petrel_Plus:
                strcpy((char*)ptr_data, "Astra SL1503P_U2");
                break;
            case ONEPLUS_TV:
                strcpy(static_cast<char*>(ptr_data), "Orbbec Oneplus TV");
                break;
            case KUNLUNSHAN_TV:
                strcpy(static_cast<char*>(ptr_data), "Orbbec Kunlunshan TV");
                break;
            default:
                strcpy((char*)ptr_data, "Orbbec Unknow Device");
                break;
            }
        }
        else
        {
            strcpy((char*)ptr_data, "Unknow Device");
        }
        ret = ONI_STATUS_OK;
        break;
    default:
        return ONI_STATUS_ERROR;
    }
#endif
    return ret;
}

OniStatus XnOniDevice::OBExtension_SetProperty(int id, const void* ptr_data, int dataSize){

    OniStatus ret = ONI_STATUS_ERROR;
    //get version
    int nMX6000Id = m_sensor.GetDevicePrivateData()->ChipInfo.nChipVer;


    if (NULL == (m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice) return ONI_STATUS_ERROR;

    switch (id)
    {
    case OBEXTENSION_ID_IR_GAIN:
        if (dataSize == 4)
        {
            ret = WriteFlash(FLASH_IR_GAIN, 2, (uint8_t *)ptr_data, 4);
        }
        else
        {
            ret = ONI_STATUS_ERROR;
        }
        break;
    case OBEXTENSION_ID_LDP_EN:
        if (nMX6000Id == XN_SENSOR_CHIP_VER_MX6000 || nMX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //MX6000 chip
            if (dataSize == 4)
            {
                if (*(uint8_t*)ptr_data == 0x01)
                {
                    ret = LdpSet(true);
                }
                else
                {
                    ret = LdpSet(false);
                }
            }

        }
        else
        {
            //mx400 chip
            uint32_t ldp_tec_flag;
            if (dataSize == 4)
            {
                ret = ReadFlash(FLASH_TEC_LDP_FLAG, 2, (uint8_t *)&ldp_tec_flag, 4);//get the dword
                if (*(uint8_t*)ptr_data == 0x01)
                {
                    ldp_tec_flag = ldp_tec_flag | (1 << 13);
                }
                else
                {
                    ldp_tec_flag = ldp_tec_flag &~(1 << 13);
                }
                ret = WriteFlash(FLASH_TEC_LDP_FLAG, 2, (uint8_t *)&ldp_tec_flag, 4);
            }
            else
            {
                ret = ONI_STATUS_ERROR;
            }

        }

        break;

    case OBEXTENSION_ID_CAM_PARAMS:
        if (nMX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            //
            ret = SetDualCameraParam((uint8_t *)ptr_data, (uint16_t)dataSize);
        }
        else
        {
            ret = WriteFlash(FLASH_CAM_PARAMS, (uint16_t)dataSize / 2, (uint8_t *)ptr_data, (uint16_t)dataSize);
        }

        break;
    case OBEXTENSION_ID_LASER_EN:
        ret = EnableLaser(*(uint32_t *)ptr_data);
        break;
    case OBEXTENSION_ID_UPDATE_FIRMWARE:
    {
        if (m_bUpdateFlag)
        {
            return  ONI_STATUS_NOT_SUPPORTED;
        }

        m_bUpdateFlag = TRUE;
        if (nMX6000Id == XN_SENSOR_CHIP_VER_MX6000 || nMX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)
        {
            ret = UpdateFirmware((uint8_t *)ptr_data, dataSize);
        }
        else
        {
            ret = MX400UpdateFirmware((uint8_t *)ptr_data, dataSize);
        }

        m_bUpdateFlag = FALSE;
    }
    break;
    default:
        return ONI_STATUS_ERROR;

    }

    return ret;
}

OniStatus XnOniDevice::SendCmd(uint16_t cmd, void *cmdbuf, uint16_t cmd_len, void *replybuf, uint16_t reply_len)
{
    int res;
    uint8_t obuf[0x400];
    uint8_t ibuf[0x200];
    unsigned int actual_len;
    cam_hdr *chdr = (cam_hdr*)obuf;
    cam_hdr *rhdr = (cam_hdr*)ibuf;

    if (0 == (m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice) return ONI_STATUS_ERROR;

    if (cmd_len & 1 || cmd_len > (0x400 - sizeof(*chdr))) {
        return ONI_STATUS_ERROR;
    }

    chdr->magic[0] = 0x47;
    chdr->magic[1] = 0x4d;
    chdr->cmd = cmd;
    //chdr->tag = *cam_tag;
    chdr->tag = m_Ext.cam_tag;
    chdr->len = cmd_len / 2;
    //copy the cmdbuf
    memcpy(obuf + sizeof(*chdr), cmdbuf, cmd_len);

    res = xnUSBSendControl((m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar*)obuf, cmd_len + sizeof(*chdr), 5000);
    if (res < 0)
    {
        xnLogError(OBEXT, "send_cmd: Output control transfer failed (%d)\n!", res);
        return ONI_STATUS_ERROR;
    }
    do
    {
        xnUSBReceiveControl((m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar *)ibuf, 0x200, &actual_len, 5000);
        //print_dbg("send_cmd: actual length = %d\n", actual_len);
    } while ((actual_len == 0) || (actual_len == 0x200));

    //print_dbg("Control reply: %d\n", res);
    if (actual_len < (int)sizeof(*rhdr)) {
        xnLogError(OBEXT, "send_cmd: Input control transfer failed (%d)\n", res);
        return ONI_STATUS_ERROR;
    }
    actual_len -= sizeof(*rhdr);

    if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) {
        xnLogError(OBEXT, "send_cmd: Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
        return ONI_STATUS_ERROR;
    }

    if (rhdr->cmd != chdr->cmd) {
        xnLogError(OBEXT, "send_cmd: Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
        return ONI_STATUS_ERROR;
    }

    if (rhdr->tag != chdr->tag) {
        xnLogError(OBEXT, "send_cmd: Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
        return ONI_STATUS_ERROR;
    }

    if (rhdr->len != (actual_len / 2)) {
        xnLogError(OBEXT, "send_cmd: Bad len %04x != %04x\n", rhdr->len, (int)(actual_len / 2));
        return ONI_STATUS_ERROR;
    }

    if (actual_len > reply_len) {
        xnLogError(OBEXT, "send_cmd: Data buffer is %d bytes long, but got %d bytes\n", reply_len, actual_len);
        memcpy(replybuf, ibuf + sizeof(*rhdr), reply_len);
    }
    else {
        memcpy(replybuf, ibuf + sizeof(*rhdr), actual_len);
    }

    //(*cam_tag)++;
    m_Ext.cam_tag++;


    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::ReadFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size)
{

    OniStatus ret = ONI_STATUS_OK;
    uint8_t cmdbuf[10];
    //create local buffer
    uint8_t * readbuf = NULL;
    if (((uint32_t)dw_size * 2 > buffer_size) || (dw_size > 256))
    {
        return ONI_STATUS_ERROR;
    }
    readbuf = (uint8_t *)malloc(2 + dw_size * 2);
    if (readbuf == NULL)
    {
        return ONI_STATUS_ERROR;
    }

    memset(readbuf, 0, 2 + dw_size * 2);
    //*(uint32_t *)&cmdbuf[0] = offset;
    uint32_t* pOffset = (uint32_t*)cmdbuf;
    *pOffset = offset;
    //*(uint16_t *)&cmdbuf[4] = dw_size;
    uint16_t* pCmd = (uint16_t*)(cmdbuf + 4);
    *pCmd = dw_size;
    ret = SendCmd(OPCODE_READ_FLASH, cmdbuf, 2 * 3, readbuf, dw_size * 2 + 2);
    if (ret != ONI_STATUS_OK){
        free(readbuf);
        return ret;
    }

    //copy to buffer
    memcpy(buffer, &readbuf[2], dw_size * 2);
    free(readbuf);
    return ret;
}

OniStatus XnOniDevice::WriteFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size)
{
    int ret;
    uint8_t readbuf[2];
    uint8_t *cmdbuf = NULL;
    if (((uint32_t)dw_size * 2 > buffer_size) || (dw_size > 256))
    {
        return ONI_STATUS_ERROR;
    }

    cmdbuf = (uint8_t *)malloc(4 + dw_size * 2 + 4);
    if (cmdbuf == NULL)
    {
        return ONI_STATUS_ERROR;
    }

    *(uint32_t *)&cmdbuf[0] = offset;
    *(uint16_t *)&cmdbuf[4] = dw_size;
    *(uint32_t *)&cmdbuf[6] = 0;
    ret = SendCmd(OPCODE_INIT_FILE_UPLOAD, cmdbuf, 2 * 5, readbuf, 2);
    if (ret == ONI_STATUS_ERROR)
    {
        free(cmdbuf);
        return ONI_STATUS_ERROR;
    }
    *(uint32_t *)&cmdbuf[0] = offset;
    memcpy(&cmdbuf[4], buffer, dw_size * 2);
    ret = SendCmd(OPCODE_WRITE_FILE_UPLOAD, cmdbuf, 4 + dw_size * 2, readbuf, 2);
    free(cmdbuf);
    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::EnableLaser(uint32_t enable)
{

    OniStatus ret = ONI_STATUS_OK;
    uint8_t readbuf[2];
    uint8_t cmdbuf[16];

    if (enable == 0x01)
    {
        cmdbuf[0] = 0x01;
    }
    else
    {
        cmdbuf[0] = 0x00;
    }

    cmdbuf[1] = 0x00;
    ret = SendCmd(CMD_ENABLE_EMITTER, cmdbuf, 2, readbuf, 2);

    return ret;
}

//////////////////////////////////////////////////////////////////////////////////////
OniStatus XnOniDevice::ObSetIRGain(const void* ptr_data, int dataSize)
{

    OniStatus ret = ONI_STATUS_OK;

    if (dataSize == 4)
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_IRGAIN, ptr_data, dataSize);
        if (nRetVal != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Failed to set property %x: %s", XN_MODULE_PROPERTY_IRGAIN, xnGetStatusString(nRetVal));
            return ONI_STATUS_BAD_PARAMETER;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }

    return ret;
}


OniStatus XnOniDevice::ObGetIRGain(void* data, int* pDataSize)
{
    OniStatus ret = ONI_STATUS_OK;
    //
    if (*pDataSize == 4)
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_IRGAIN, data, pDataSize);
        if (nRetVal != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Failed to get property %x: %s", XN_MODULE_PROPERTY_IRGAIN, xnGetStatusString(nRetVal));
            return ONI_STATUS_BAD_PARAMETER;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }
    return ret;
}



OniStatus XnOniDevice::ObGetIRExp(void* data, int* pDataSize)
{
    OniStatus ret = ONI_STATUS_OK;
    //
    if (*pDataSize == 4)
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->GetProperty(XN_MODULE_PROPERTY_IREXP, data, pDataSize);
        if (nRetVal != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Failed to get property %x: %s", XN_MODULE_PROPERTY_IREXP, xnGetStatusString(nRetVal));
            return ONI_STATUS_BAD_PARAMETER;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }
    return ret;
}


OniStatus XnOniDevice::ObSetIRExp(const void* ptr_data, int dataSize)
{

    OniStatus ret = ONI_STATUS_OK;

    if (dataSize == 4)
    {
        XnStatus nRetVal = m_sensor.DeviceModule()->SetProperty(XN_MODULE_PROPERTY_IREXP, ptr_data, dataSize);
        if (nRetVal != XN_STATUS_OK)
        {
            m_driverServices.errorLoggerAppend("Failed to set property %x: %s", XN_MODULE_PROPERTY_IREXP, xnGetStatusString(nRetVal));
            return ONI_STATUS_BAD_PARAMETER;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }

    return ret;
}

//////////////////////////////////////////////////////////////////////////////////////
//mx6000 add obExtension
uint32_t CalCRC32(XnUInt8 * /*data*/, XnInt /*data_size*/)
{
    return 0;
}

//get ldp
OniStatus XnOniDevice::GetLdp(XnUInt32 &ldp_status)
{
    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);

    //first dump the cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        //
        xnLogError(OBEXT, "GetLdp  read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //uint32_t crc32 = CalCRC32(buf_ptr, MX6000_CFG_SIZE - 4);
    //check tth crc32

    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
    DeviceInfo * device_info_ptr = (DeviceInfo *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->device_info_ptr) & 0x0000FFFF));

    ldp_status = device_info_ptr->ldp_enabl_flag;

    free(buf_ptr);
    return ONI_STATUS_OK;
}

//set ldp
OniStatus XnOniDevice::LdpSet(XnBool ldp_status)
{

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);

    //first dump the cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        //
        xnLogError(OBEXT, "LdpSet  read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //uint32_t crc32 = CalCRC32(buf_ptr, MX6000_CFG_SIZE - 4);
    //check tth crc32

    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;

    DeviceInfo * device_info_ptr = (DeviceInfo *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->device_info_ptr) & 0x0000FFFF));

    device_info_ptr->ldp_enabl_flag = (uint32_t)ldp_status;

    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "LdpSet  EraseFlash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "LdpSet  WriteFlash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    free(buf_ptr);
    return ONI_STATUS_OK;
}

//add by gaodw for factory updateFirmware interface.
OniStatus XnOniDevice::UpdateFirmwareWriteFlash(void *pBuff, int nSize, int flash_address)
{
    if (NULL == pBuff)
    {
        xnLogError(OBEXT, "UpdateFirmwareWriteFlash  param pBuff is NULL!!! \n");
        return ONI_STATUS_ERROR;
    }

    int nRet = EraseFlash(flash_address, nSize);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmwareWriteFlash EraseFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    nRet = WriteFlash(flash_address, nSize, (uint8_t *)pBuff);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmwareWriteFlash WriteFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    return ONI_STATUS_OK;
}


OniStatus XnOniDevice::UpdateFirmwareReadFlash(void *pBuff, int nSize, int flash_address)
{

    if (NULL == pBuff)
    {
        xnLogError(OBEXT, "UpdateFirmwareReadFlash param pBuff is NULL!!! \n");
        return ONI_STATUS_ERROR;
    }

    int nRet = ReadFlash(flash_address, nSize / 2, (uint8_t*)pBuff);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmwareReadFlash  error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::UpdateFirmware(void *pBuff, int nSize)
{
    if (nSize != MX6000_CODE_SIZE)
    {
        xnLogError(OBEXT, "UpdateFirmware error fileSize: (%d)\n", nSize);
        return ONI_STATUS_ERROR;
    }

    return UpdateFlash(MX6000_CODE_ADDRESS, pBuff, nSize);

    /*int nRet=EraseFlash(MX6000_CODE_ADDRESS,nSize);
    if(nRet !=0)
    {
    xnLogError(OBEXT, "UpdateFirmware EraseFlash error: (%d)\n", nRet);
    return ONI_STATUS_ERROR;
    }

    nRet = WriteFlash(MX6000_CODE_ADDRESS, nSize, (uint8_t *)pBuff);
    if (nRet != 0)
    {
    xnLogError(OBEXT, "UpdateFirmware WriteFlash error: (%d)\n", nRet);
    return ONI_STATUS_ERROR;
    }

    uint8_t* bufPtr=(uint8_t *)calloc(1,nSize);
    nRet = ReadFlash(MX6000_CODE_ADDRESS, nSize / 2, bufPtr);
    if (nRet != 0)
    {
    xnLogError(OBEXT, "UpdateFirmware ReadFlash error: (%d)\n", nRet);
    free(bufPtr);
    return ONI_STATUS_ERROR;
    }
    for(int i=0; i<nSize; i++)
    {
    if(bufPtr[i] != *((uint8_t *)pBuff+i))
    {
    xnLogError(OBEXT, "UpdateFirmware file verify error\n");
    free(bufPtr);
    return ONI_STATUS_ERROR;
    }
    }
    free(bufPtr);
    return ONI_STATUS_OK;*/
}

//Write reference
OniStatus XnOniDevice::UpdateRef(const void *pBuff, int nSize)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    return UpdateFlash(MX6000_REFERENCE_ADDRESS, (uint8_t *)pBuff, nSize);
}

//Write software alignment parameters
OniStatus XnOniDevice::UpdateSwAlignParams(const void *pBuff, int nSize)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }
    return UpdateFlash(MX6000_SW_ALIGN_ADDRESS, (uint8_t *)pBuff, nSize);
}

OniStatus XnOniDevice::SetDepthConfig(const void *pBuffer, int nSize)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set depth config read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //
    const int nBaseline = MX6000_BASELINE_OFFSET;
    memcpy(buf_ptr + nBaseline, pBuffer, nSize);

    //
    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set depth config erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set depth config write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set depth config read flash failed (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set depth config verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::UpdateFlash(XnUInt32 offset, void *pBuff, int nSize)
{
    int nRet = EraseFlash(offset, nSize);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFlash EraseFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    nRet = WriteFlash(offset, nSize, (uint8_t *)pBuff);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFlash WriteFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    uint8_t* bufPtr = (uint8_t *)calloc(1, nSize);
    nRet = ReadFlash(offset, nSize / 2, bufPtr);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFlash ReadFlash error: (%d)\n", nRet);
        free(bufPtr);
        return ONI_STATUS_ERROR;
    }
    for (int i = 0; i < nSize; i++)
    {
        if (bufPtr[i] != *((uint8_t *)pBuff + i))
        {
            xnLogError(OBEXT, "UpdateFlash file verify error\n");
            free(bufPtr);
            return ONI_STATUS_ERROR;
        }
    }

    free(bufPtr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::MX400UpdateFirmware(void *pBuff, int nSize)
{
    if (nSize != MX400_CODE_SIZE)
    {
        xnLogError(OBEXT, "UpdateFirmware error fileSize: (%d)\n", nSize);
        return ONI_STATUS_ERROR;
    }
    int nRet = EraseFlash(MX400_CODE_ADDRESS, nSize);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmware EraseFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    nRet = WriteFlash(MX400_CODE_ADDRESS, nSize, (uint8_t *)pBuff);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmware WriteFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    uint8_t* bufPtr = (uint8_t *)calloc(1, nSize);
    nRet = ReadFlash(MX400_CODE_ADDRESS, nSize / 2, bufPtr);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "UpdateFirmware ReadFlash error: (%d)\n", nRet);
        free(bufPtr);
        return ONI_STATUS_ERROR;
    }
    for (int i = 0; i < nSize; i++)
    {
        if (bufPtr[i] != *((uint8_t *)pBuff + i))
        {
            xnLogError(OBEXT, "UpdateFirmware file verify error\n");
            free(bufPtr);
            return ONI_STATUS_ERROR;
        }
    }
    free(bufPtr);
    return ONI_STATUS_OK;
}

//set dual camera param
OniStatus XnOniDevice::SetDualCameraParam(void *pBuff, int nSize)
{
    //
    /*
    OBCameraParams obCameraParam;
    if(nSize != sizeof(obCameraParam))
    {
    xnLogError(OBEXT, "obCameraParam len error(%d)\n", nSize);
    return ONI_STATUS_ERROR;
    }
    */
    ObContent_t obContent;
    int nContentSize = sizeof(obContent);
    xnOSMemSet(&obContent, 0, nContentSize);

    int nRet = ReadFlash(FLASH_CAM_PARAMS, nContentSize / 2, (XnUInt8 *)&obContent);
    if (nRet != 0)
    {
        //
        xnLogError(OBEXT, "get dual camera param failed (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    nRet = EraseFlash(FLASH_CAM_PARAMS, nContentSize);
    if (nRet != 0)
    {
        xnLogError(OBEXT, " SetDualCameraParam EraseFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    //convert OBCameraParams to ObContent_t
    int nD2cSize = sizeof(obContent.HOST.soft_d2c);
    int nCopySize = nD2cSize > nSize ? nSize : nD2cSize;

    xnOSMemCopy(&obContent.HOST.soft_d2c, pBuff, nCopySize);

    //write flash
    nRet = WriteFlash(FLASH_CAM_PARAMS, nContentSize, (uint8_t *)&obContent);
    if (nRet != 0)
    {
        xnLogError(OBEXT, "SetDualCameraParam WriteFlash error: (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }

    //read flash
    ObContent_t obContent1;
    xnOSMemSet(&obContent1, 0, nContentSize);
    nRet = ReadFlash(FLASH_CAM_PARAMS, nContentSize / 2, (XnUInt8 *)&obContent1);
    if (nRet != 0)
    {
        //
        xnLogError(OBEXT, "get dual camera param failed verify (%d)\n", nRet);
        return ONI_STATUS_ERROR;
    }
    //verify flash
    uint8_t *pSrc = (uint8_t *)&obContent;
    uint8_t *pDst = (uint8_t *)&obContent1;
    for (int i = 0; i < nContentSize; i++)
    {
        //
        if (pSrc[i] != pDst[i])
        {
            xnLogError(OBEXT, " SetDualCameraParam verify failed\n");
            return ONI_STATUS_ERROR;
        }
    }


    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::GetDualCameraParam(void *pBuff, int nSize)
{
    ObContent_t obContent;
    int nContentSize = sizeof(obContent);
    xnOSMemSet(&obContent, 0, nContentSize);

    int ret = ReadFlash(FLASH_CAM_PARAMS, nContentSize / 2, (XnUInt8 *)&obContent);
    if (ret != 0)
    {
        //
        xnLogError(OBEXT, "get dual camera param failed (%d)\n", ret);
        return ONI_STATUS_ERROR;
    }

    //convert ObContent_t to OBCameraParams
    int nD2cSize = sizeof(obContent.HOST.soft_d2c);
    int nCopySize = nD2cSize > nSize ? nSize : nD2cSize;

    xnOSMemCopy(pBuff, &obContent.HOST.soft_d2c, nCopySize);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::SetCfgSn_Pn(const void *pBuffer, int nSize)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    OBCfgSerialProductNumber *cfgSn_Pn = (OBCfgSerialProductNumber*)pBuffer;
    if (nSize != sizeof(OBCfgSerialProductNumber))
    {
        return ONI_STATUS_ERROR;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  sn and pn  read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    DeviceCfg *device_Cfg_Ptr = (DeviceCfg *)buf_ptr;
    DeviceInfo *device_Info_Ptr = (DeviceInfo *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_Cfg_Ptr->device_info_ptr) & 0x0000FFFF));

    memcpy(device_Info_Ptr->sn, cfgSn_Pn->SerialNumber, sizeof(cfgSn_Pn->SerialNumber));
    memcpy(device_Info_Ptr->pn, cfgSn_Pn->ProductNumber, sizeof(cfgSn_Pn->ProductNumber));

    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  sn and pn  erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  sn and pn  write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  sn and pn  read flash failed (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg  sn and pn verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::SetCfgZ0_Baseline(const void *pBuffer, int nSize)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    OBZ0Baseline *cfgSn_Pn = (OBZ0Baseline*)pBuffer;
    if (nSize != sizeof(OBZ0Baseline))
    {
        return ONI_STATUS_ERROR;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg   z0 and baseline   read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //
    const int nZ0Offset = MX6000_Z0_OFFSET;
    const int nBaseline = MX6000_BASELINE_OFFSET;

    memcpy(buf_ptr + nZ0Offset, &cfgSn_Pn->fZ0, sizeof(float));
    memcpy(buf_ptr + nBaseline, &cfgSn_Pn->fBaseline, sizeof(float));

    //

    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg   z0 and baseline   erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg   z0 and baseline   write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  z0 and baseline  read flash failed (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg  z0 and baseline verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::SetCfgParam(const void *pBuffer, int nSize, int nRegsNum, int nParamType)
{
    int nLen = sizeof(void*);

    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
    DepthCfg * detph_cfg_ptr = (DepthCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->depth_cfg_ptr) & 0x0000FFFF));
    if (nParamType == HW_D2C_TYPE)
    {
        detph_cfg_ptr->d2c_regs_num = nRegsNum;
        detph_cfg_ptr->d2c_regs_ptr = (RegVal*)(DEVICE_CONFIG_ADDR + (MX6000_CFG_D2C_ADDRESS & 0x0000FFFF));

        xnOSMemCopy(buf_ptr + MX6000_CFG_D2C_ADDRESS, pBuffer, nSize);
    }
    else if (nParamType == HW_DISTORTION_TYPE)
    {
        detph_cfg_ptr->rectify_regs_num = 1;
        detph_cfg_ptr->rectify_regs_ptr = (RegVal*)(DEVICE_CONFIG_ADDR + (MX6000_CFG_REC_ADDRESS & 0x0000FFFF));

        //RegVal * rec_regs_ptr = (RegVal *)(int)((int)buf_ptr + (MX6000_CFG_REC_ADDRESS & 0x0000FFFF));
        xnOSMemCopy(buf_ptr + MX6000_CFG_REC_ADDRESS, pBuffer, nSize);
    }
    else
    {
        xnLogError(OBEXT, "ParamType upsupport \n");
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  read flash failed (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::SetCfgIrGainExp(const void *pBuffer, int nSize, int nParamType)
{
    int nLen = sizeof(void*);
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    if (nSize != 4)
    {
        return ONI_STATUS_ERROR;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
    SensorCfg * sensor_cfg_ptr = (SensorCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->sensor_cfg_ptr[0]) & 0x0000FFFF));
    //SensorCfg * sensor_cfg_ptr = (SensorCfg *)((int)buf_ptr + ((int)device_cfg_ptr->sensor_cfg_ptr[0] & 0x0000FFFF));

    XnUInt32 *pValue = (XnUInt32 *)pBuffer;
    XnUInt32 nValue = (XnUInt32)(*pValue);
    if (nParamType == CFG_IR_GAIN_TYPE)
    {
        sensor_cfg_ptr->gain.def = nValue;
    }
    else if (nParamType == CFG_IR_EXP_TYPE)
    {
        sensor_cfg_ptr->exposure.def = nValue;
    }
    else
    {
        xnLogError(OBEXT, "ParamType upsupport \n");
        free(buf_ptr);
        return ONI_STATUS_NOT_SUPPORTED;
    }


    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp read flash failed1 (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg  gain or exp verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::GetCfgIrGainExp(void* pBuffer, int*pDataSize, int nParamType)
{
    OniStatus ret = ONI_STATUS_OK;
    int nLen = sizeof(void*);
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    //
    if (*pDataSize == 4)
    {
        //
        XnUInt8 *buf_ptr = NULL;
        int ret = 0;

        buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
        if (NULL == buf_ptr)
        {
            return ONI_STATUS_ERROR;
        }

        //first dump cfg part
        ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
        if (ret != 0)
        {
            xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }


        DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
        SensorCfg * sensor_cfg_ptr = (SensorCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->sensor_cfg_ptr[0]) & 0x0000FFFF));
        //SensorCfg * sensor_cfg_ptr = (SensorCfg *)((int)buf_ptr + ((int)device_cfg_ptr->sensor_cfg_ptr[0] & 0x0000FFFF));

        XnUInt32 *pValue = (XnUInt32 *)pBuffer;

        if (nParamType == CFG_IR_GAIN_TYPE)
        {
            *pValue = sensor_cfg_ptr->gain.def;
        }
        else if (nParamType == CFG_IR_EXP_TYPE)
        {
            *pValue = sensor_cfg_ptr->exposure.def;
        }
        else
        {
            xnLogError(OBEXT, "ParamType upsupport \n");
            free(buf_ptr);
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }
    return ret;
}

OniStatus XnOniDevice::SetCfgPostfiterThreshold(const void *pBuffer, int nSize)
{
    int nLen = sizeof(void*);

    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    if (nSize != 4)
    {
        return ONI_STATUS_ERROR;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
    //DepthCfg * detph_cfg_ptr = (DepthCfg *)((int)buf_ptr + ((int)device_cfg_ptr->depth_cfg_ptr & 0x0000FFFF));
    DepthCfg * detph_cfg_ptr = (DepthCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->depth_cfg_ptr) & 0x0000FFFF));

    XnUInt32 *pValue = (XnUInt32 *)pBuffer;
    XnUInt32 nValue = (XnUInt32)(*pValue);
    detph_cfg_ptr->postfilter = nValue;

    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp read flash failed1 (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg  gain or exp verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::GetCfgPostfiterThreshold(void* pBuffer, int*pDataSize)
{
    OniStatus ret = ONI_STATUS_OK;
    int nLen = sizeof(void*);
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    //
    if (*pDataSize == 4)
    {
        //
        XnUInt8 *buf_ptr = NULL;
        int ret = 0;

        buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
        if (NULL == buf_ptr)
        {
            return ONI_STATUS_ERROR;
        }

        //first dump cfg part
        ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
        if (ret != 0)
        {
            xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }


        DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
        //DepthCfg * detph_cfg_ptr = (DepthCfg *)((int)buf_ptr + ((int)device_cfg_ptr->depth_cfg_ptr & 0x0000FFFF));
        DepthCfg * detph_cfg_ptr = (DepthCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->depth_cfg_ptr) & 0x0000FFFF));

        XnUInt32 *pValue = (XnUInt32 *)pBuffer;

        *pValue = detph_cfg_ptr->postfilter;

    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }
    return ret;
}

OniStatus XnOniDevice::SetCfgLaserCurrentOrTime(const void *pBuffer, int nSize, int nParamType)
{
    int nLen = sizeof(void*);

    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    if (nSize != 4)
    {
        return ONI_STATUS_ERROR;
    }

    XnUInt8 *buf_ptr = NULL;
    int ret = 0;

    buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_ptr)
    {
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
    //LaserCfg * laser_cfg_ptr = (LaserCfg *)((int)buf_ptr + ((int)device_cfg_ptr->laser_cfg_ptr & 0x0000FFFF));
    LaserCfg * laser_cfg_ptr = (LaserCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->laser_cfg_ptr) & 0x0000FFFF));

    XnUInt32 *pValue = (XnUInt32 *)pBuffer;
    XnUInt32 nValue = (XnUInt32)(*pValue);
    if (nParamType == CFG_LASER_CURRENT_TYPE)
    {
        laser_cfg_ptr->current.def = nValue;
    }
    else if (nParamType == CFG_LASER_TIME_TYPE)
    {
        laser_cfg_ptr->time.def = nValue;
    }
    else
    {
        xnLogError(OBEXT, "ParamType upsupport \n");
        free(buf_ptr);
        return ONI_STATUS_NOT_SUPPORTED;
    }


    ret = EraseFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg gain or exp erase flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    ret = WriteFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE, buf_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp write flash failed (%d)\n", ret);
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }


    //cfg check
    XnUInt8 *buf_check_ptr = NULL;
    buf_check_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
    if (NULL == buf_check_ptr)
    {
        free(buf_ptr);
        return ONI_STATUS_ERROR;
    }

    //first dump cfg part
    ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_check_ptr);
    if (ret != 0)
    {
        xnLogError(OBEXT, "Set Cfg  gain or exp read flash failed1 (%d)\n", ret);
        free(buf_ptr);
        free(buf_check_ptr);
        return ONI_STATUS_ERROR;
    }

    int cSize = MX6000_CFG_SIZE;
    for (int i = 0; i < cSize; i++)
    {
        if (buf_check_ptr[i] != *((uint8_t *)buf_ptr + i))
        {
            xnLogError(OBEXT, "Set Cfg  gain or exp verify error\n");
            free(buf_check_ptr);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }
    }

    free(buf_check_ptr);
    free(buf_ptr);

    return ONI_STATUS_OK;
}

OniStatus XnOniDevice::GetCfgLaserCurrentOrTime(void* pBuffer, int*pDataSize, int nParamType)
{
    OniStatus ret = ONI_STATUS_OK;
    //only support 32bit program
    int nLen = sizeof(void*);
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION || nLen != 4){
        xnLogError(OBEXT, "Error: (%d)\n", ONI_STATUS_NOT_SUPPORTED);
        return ONI_STATUS_NOT_SUPPORTED;
    }

    //
    if (*pDataSize == 4)
    {
        //
        XnUInt8 *buf_ptr = NULL;
        int ret = 0;

        buf_ptr = (XnUInt8 *)calloc(1, MX6000_CFG_SIZE);
        if (NULL == buf_ptr)
        {
            return ONI_STATUS_ERROR;
        }

        //first dump cfg part
        ret = ReadFlash(MX6000_CFG_ADDRESS, MX6000_CFG_SIZE / 2, buf_ptr);
        if (ret != 0)
        {
            xnLogError(OBEXT, "Set Cfg gain or exp read flash failed (%d)\n", ret);
            free(buf_ptr);
            return ONI_STATUS_ERROR;
        }


        DeviceCfg *device_cfg_ptr = (DeviceCfg *)buf_ptr;
        //LaserCfg * laser_cfg_ptr = (LaserCfg *)((int)buf_ptr + ((int)device_cfg_ptr->laser_cfg_ptr & 0x0000FFFF));
        LaserCfg * laser_cfg_ptr = (LaserCfg *)(reinterpret_cast<uintptr_t>(buf_ptr)+(reinterpret_cast<uintptr_t>(device_cfg_ptr->laser_cfg_ptr) & 0x0000FFFF));

        XnUInt32 *pValue = (XnUInt32 *)pBuffer;

        if (nParamType == CFG_LASER_CURRENT_TYPE)
        {
            *pValue = laser_cfg_ptr->current.def;
        }
        else if (nParamType == CFG_LASER_TIME_TYPE)
        {
            *pValue = laser_cfg_ptr->time.def;
        }
        else
        {
            xnLogError(OBEXT, "ParamType upsupport \n");
            free(buf_ptr);
            return ONI_STATUS_NOT_SUPPORTED;
        }
    }
    else
    {
        ret = ONI_STATUS_ERROR;
    }
    return ret;
}

OniStatus XnOniDevice::getCalibration(OniCalibrationCamera *pCalibration)
{
    XN_RET_IF_NULL(pCalibration, ONI_STATUS_BAD_PARAMETER);

    if (m_pCalibration)
    {
        xnOSMemCopy(pCalibration, m_pCalibration, sizeof(OniCalibrationCamera));
        return ONI_STATUS_OK;
    }

    OBCameraParams params = { 0 };
    XnInt32 size = sizeof(OBCameraParams);

    XnUInt16 dwSize = (XnUInt16)(size / 2);
    if (m_sensor.GetDevicePrivateData()->Version.nMajor >= 5 &&
        m_sensor.GetDevicePrivateData()->Version.nMinor >= 8 &&
        m_sensor.GetDevicePrivateData()->Version.nBuild < 24)
    {
        dwSize = (XnUInt16)((size - sizeof(uint32_t) * 2) / 2);
    }

    OniStatus ret = ReadFlash(FLASH_CAM_PARAMS, dwSize, (uint8_t *)&params, size);
    XN_IS_STATUS_OK_RET(ret, ONI_STATUS_ERROR);

    m_pCalibration = new OniCalibrationCamera();
    XN_RET_IF_NULL(m_pCalibration, ONI_STATUS_BAD_PARAMETER);

    // Color camera
    m_pCalibration->color.width = params.width;
    m_pCalibration->color.height = params.height;
    m_pCalibration->color.intrinsics.k1 = params.r_k[0];
    m_pCalibration->color.intrinsics.k2 = params.r_k[1];
    m_pCalibration->color.intrinsics.p1 = params.r_k[2];
    m_pCalibration->color.intrinsics.p2 = params.r_k[3];
    m_pCalibration->color.intrinsics.k3 = params.r_k[4];
    m_pCalibration->color.intrinsics.fx = params.r_intr_p[0];
    m_pCalibration->color.intrinsics.fy = params.r_intr_p[1];
    m_pCalibration->color.intrinsics.cx = params.r_intr_p[2];
    m_pCalibration->color.intrinsics.cy = params.r_intr_p[3];
    xnOSMemCopy(m_pCalibration->color.extrinsics.rotation, params.r2l_r, sizeof(params.r2l_r));
    xnOSMemCopy(m_pCalibration->color.extrinsics.translation, params.r2l_t, sizeof(params.r2l_t));

    // Depth camera
    m_pCalibration->depth.width = 640;
    m_pCalibration->depth.height = 480;
    m_pCalibration->depth.intrinsics.k1 = params.l_k[0];
    m_pCalibration->depth.intrinsics.k2 = params.l_k[1];
    m_pCalibration->depth.intrinsics.p1 = params.l_k[2];
    m_pCalibration->depth.intrinsics.p2 = params.l_k[3];
    m_pCalibration->depth.intrinsics.k3 = params.l_k[4];
    m_pCalibration->depth.intrinsics.fx = params.l_intr_p[0];
    m_pCalibration->depth.intrinsics.fy = params.l_intr_p[1];
    m_pCalibration->depth.intrinsics.cx = params.l_intr_p[2];
    m_pCalibration->depth.intrinsics.cy = params.l_intr_p[3];
    xnOSMemCopy(m_pCalibration->depth.extrinsics.rotation, params.r2l_r, sizeof(params.r2l_r));
    xnOSMemCopy(m_pCalibration->depth.extrinsics.translation, params.r2l_t, sizeof(params.r2l_t));

    xnOSMemCopy(pCalibration, m_pCalibration, sizeof(OniCalibrationCamera));

    return ONI_STATUS_OK;
}

int XnOniDevice::ReadFlash(XnUInt32 offset, XnUInt32 size, XnUInt8 *data_ptr)
{
    int ret = -1;
    XnUInt16 data_len;
    XnUInt16 resp_len;
    int addr_offset = 0;
    XnUInt16 i;
    XnUInt32 *p = NULL;

    int sizeInBytes = size * 2;
    int lastsizeInBytes = sizeInBytes % EATCH_PACKET_SIZE;

    for (int k = 0; k < sizeInBytes / EATCH_PACKET_SIZE; k++)
    {
        data_len = 6;
        ret = InitHeader(req_buf, OPCODE_READ_FLASH, data_len);
        if (ret)
        {
            xnLogError(OBEXT, "init header of read flash failed (%d)\n", ret);
            return ret;
        }

        p = (XnUInt32 *)(req_buf + 8);
        *p = offset + addr_offset;
        req_buf[12] = EATCH_PACKET_SIZE / 2;
        req_buf[13] = (EATCH_PACKET_SIZE / 2) >> 8;

        ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
        if (ret)
        {
            //cout << "send cmd  read flash failed" << endl;
            xnLogError(OBEXT, "send cmd  read flash failed (%d)\n", ret);
            return ret;
        }

        for (int i = 0; i < EATCH_PACKET_SIZE; i++)
        {
            *(data_ptr + addr_offset + i) = *(resp_buf + 10 + i);
        }

        addr_offset = addr_offset + EATCH_PACKET_SIZE;
    }

    if (lastsizeInBytes != 0)
    {
        data_len = 6;
        ret = InitHeader(req_buf, OPCODE_READ_FLASH, data_len);
        if (ret)
        {
            //cout << "init header of read flash failed" << endl;
            xnLogError(OBEXT, "init header of read flash failed (%d)\n", ret);
            return ret;
        }

        p = (XnUInt32 *)(req_buf + 8);
        *p = offset + addr_offset;
        req_buf[12] = (XnUInt8)lastsizeInBytes / 2;
        req_buf[13] = (XnUInt8)((lastsizeInBytes / 2) >> 8);

        ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
        if (ret)
        {
            //cout << "send cmd  read flash failed" << endl;
            xnLogError(OBEXT, "send cmd  read flash failed (%d)\n", ret);
            return ret;
        }

        for (i = 0; i < lastsizeInBytes; i++)
        {
            *(data_ptr + addr_offset + i) = *(resp_buf + 10 + i);
        }
    }

    return ret;
}

int XnOniDevice::EraseFlash(XnUInt32 offset, XnUInt32 size)
{
    int ret;
    XnUInt16 data_len;
    XnUInt16 resp_len;
    XnUInt32 *p = NULL;

    data_len = 10;
    ret = InitHeader(req_buf, OPCODE_INIT_FILE_UPLOAD, data_len);
    p = (XnUInt32 *)(req_buf + 8);
    *p = offset;
    p = (XnUInt32 *)(req_buf + 12);
    *p = size;

    if (ret)
    {
        //cout << "init header of init upload file failed" << endl;
        xnLogError(OBEXT, "init header of init upload file failed (%d)\n", ret);
        return ret;
    }

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret)
    {
        //cout << "send cmd init upload file failed" << endl;
        xnLogError(OBEXT, "send cmd init upload file failed (%d)\n", ret);
    }

    return ret;
}


int XnOniDevice::WriteFlash(XnUInt32 offset, XnUInt32 size, const XnUInt8  *buf)
{
    int ret = 0;
    XnUInt16 data_len;
    XnUInt16 resp_len;

    int addr_offset = 0;
    XnUInt32 *p = NULL;

    uint8_t data_buf[256 + 12] = { 0 };

    //int sizeInBytes = size * 2;
    int sizeInBytes = size;
    int lastsizeInBytes = sizeInBytes % EATCH_PACKET_SIZE;


    for (int k = 0; k < sizeInBytes / EATCH_PACKET_SIZE; k++)
    {
        //memcpy(data_buf + 12, buf + addr_offset, 256);
        //memcpy(data_buf, buf + addr_offset, 256);

        data_len = EATCH_PACKET_SIZE + 4;
        ret = InitHeader(data_buf, OPCODE_WRITE_FILE_UPLOAD, data_len);
        memcpy(data_buf + 12, buf + addr_offset, EATCH_PACKET_SIZE);

        p = (XnUInt32 *)(data_buf + 8);
        *p = offset + addr_offset;

        if (ret)
        {
            //cout << "write flash failed" << endl;
            xnLogError(OBEXT, "write flash failed (%d)\n", ret);
            return ret;
        }

        ret = send(data_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
        if (ret)
        {
            //cout << "send cmd write flash failed" << endl;
            xnLogError(OBEXT, "send cmd write flash failed (%d)\n", ret);
            return ret;
        }

        addr_offset = addr_offset + EATCH_PACKET_SIZE;
    }

    if (lastsizeInBytes != 0)
    {
        memcpy(data_buf + 12, buf + addr_offset, lastsizeInBytes);

        data_len = (XnUInt16)(lastsizeInBytes + 4);
        ret = InitHeader(data_buf, OPCODE_WRITE_FILE_UPLOAD, data_len);
        p = (XnUInt32 *)(data_buf + 8);
        *p = offset + addr_offset;

        if (ret)
        {
            //cout << "write flash failed" << endl;
            xnLogError(OBEXT, "write flash failed (%d)\n", ret);
            return ret;
        }

        ret = send(data_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
        if (ret)
        {
            //cout << "send cmd write flash failed" << endl;
            xnLogError(OBEXT, "send cmd write flash failed (%d)\n", ret);
            return ret;
        }

    }

    return ret;
}


int XnOniDevice::InitHeader(void *buf, XnUInt16 cmd, XnUInt16 data_len)
{
    protocol_header *pheader = (protocol_header *)buf;
    pheader->magic = CMD_HEADER_MAGIC;
    pheader->size = data_len / 2;
    pheader->opcode = cmd;
    pheader->id = seq_num;
    seq_num++;

    return 0;
}

//
int XnOniDevice::send(void *cmd_req, XnUInt16 req_len, void *cmd_resp, XnUInt16 *resp_len)
{
    XnUInt32 actual_len;
    XnStatus nRetVal;
    XnStatus err = 0;
    XnUInt32 n;

    xnUSBSendControl((m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar*)cmd_req, req_len, 100000);
    n = 0;
    do
    {
        nRetVal = xnUSBReceiveControl((m_sensor.GetDevicePrivateData()->SensorHandle).USBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar *)cmd_resp, 0x200, &actual_len, 100000);
        if (nRetVal != 0)
        {
            n++;
            err = nRetVal;
        }
        if (n == 2)
        {
            err = nRetVal;
            break;
        }
    } while ((actual_len == 0) || (actual_len == 0x200));  //error?

    if (nRetVal == 0)
    {
        *resp_len = (XnUInt16)actual_len;
        err = *(XnUInt16*)((XnUChar *)cmd_resp + 8);
    }
    else
    {
        *resp_len = 0;
    }


    return err;
}
