/*****************************************************************************
*									     *
*  OpenNI 2.x Alpha							     *
*  Copyright (C) 2012 PrimeSense Ltd.					     *
*									     *
*  This file is part of OpenNI. 					     *
*									     *
*  Licensed under the Apache License, Version 2.0 (the "License");	     *
*  you may not use this file except in compliance with the License.	     *
*  You may obtain a copy of the License at				     *
*									     *
*      http://www.apache.org/licenses/LICENSE-2.0			     *
*									     *
*  Unless required by applicable law or agreed to in writing, software	     *
*  distributed under the License is distributed on an "AS IS" BASIS,	     *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and	     *
*  limitations under the License.					     *
*									     *
*****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "XnOniDothinDevice.h"
#include "XnOniDothinDriver.h"
#include "../DDK/XnPropertySetInternal.h"
#include "XnDothinDeviceEnumeration.h"
#include "XnMx6xResolution.h"
#include "XnOniDothinStream.h"
#include "XnOniDothinColorStream.h"
#include "OBCProperties.h"
#include <XnLog.h>
#include <vector>
#include <iostream>

using namespace std;

//---------------------------------------------------------------------------
// XnOniDevice class
//---------------------------------------------------------------------------
XnOniDothinDevice::XnOniDothinDevice(const XnChar* uri, oni::driver::DriverServices& driverServices, XnOniDothinDriver* pDriver) : m_driverServices(driverServices), m_pDriver(pDriver)
{
	xnOSStrCopy(m_info.uri, uri, sizeof(m_info.uri));
	xnOSStrCopy(m_info.vendor, "Dothinkey", sizeof(m_info.vendor));
	xnOSStrCopy(m_info.name, "rgb", sizeof(m_info.name));
}

XnStatus XnOniDothinDevice::Init(const char* mode)
{
	XnStatus nRetVal = XN_STATUS_OK;
	xnLogInfo(XN_MASK_ONI_DEVICE, "Load RGB XnOniDothinDevice howard");
	nRetVal = DeviceOpen();
	if (nRetVal == XN_STATUS_OK)
	{
		if (mode == nullptr)
		{
			nRetVal = DeviceInit(false);
		}
		else
		{
			nRetVal = DeviceInit(true);
		}
		
		XN_IS_STATUS_OK(nRetVal);
	}
	else
	{
		return nRetVal;
	}


	return XN_STATUS_OK;
}

XnStatus XnOniDothinDevice::DeviceOpen()
{
	if (xnOSStrLen(m_info.uri) == 0)
	{
		return XN_STATUS_ERROR;
	}
	XnStatus rc = XN_STATUS_OK;
	rc = dtConnectHelper.DeviceOpen(m_info.uri);


	//uint32_t sensor_id = 0;
	//int size = sizeof(sensor_id);
	//command_data_t cmd_t;
	//cmd_t.data = &sensor_id;
	//cmd_t.len = size;

	//m_pMudulesHelper.CommandGetProperty(SENSOR_ID, &cmd_t);
	//xnLogInfo(XN_MASK_ONI_DEVICE, "Load RGB sensor SENSOR_ID=%d", sensor_id);
	return XN_STATUS_OK;
}

XnStatus XnOniDothinDevice::DeviceInit(bool model)
{
	XnStatus res = XN_STATUS_OK;
	//1.Init sensor config
	res = dtConnectHelper.LoadSensorConfiguration();
	XN_IS_STATUS_OK(res);

	//2.Init sensor mx6x module
	res = m_pMudulesHelper.Init(dtConnectHelper.GetDothinId());
	XN_IS_STATUS_OK(res);
	if (res != XN_STATUS_OK)
	{
		deviceClose();
		return -1;
	}
	if (!model)
	{
		//3.New XnDothinDeviceProperties
		dotDeviceProperties = XN_NEW(XnDothinDeviceProperties, &m_pMudulesHelper);
		XN_VALIDATE_INPUT_PTR(dotDeviceProperties);

		res = FillSupportedVideoModes();
		XN_IS_STATUS_OK(res);
	}


	//Load Firmware and reference document, mx6300 need this procedure for  internal debug
	/*
	xnLogInfo(XN_MASK_ONI_DEVICE, "Start loadFirmware...");
	res = m_pMudulesHelper.LoadFirmware();
	XN_IS_STATUS_OK(res);
	xnLogInfo(XN_MASK_ONI_DEVICE, "LoadFirmware success...");

	res = FillSupportedVideoModes();
	XN_IS_STATUS_OK(res);

	/*
	xnLogInfo(XN_MASK_ONI_DEVICE, "Start load reference...");
	res = m_pMudulesHelper.LoadReference();
	XN_IS_STATUS_OK(res);
	xnLogInfo(XN_MASK_ONI_DEVICE, "Load reference success...");
	*/

	return XN_STATUS_OK;
}
uint32_t XnOniDothinDevice::getSensorId(){
	uint32_t sensor_id = 0;
	int size = sizeof(sensor_id);
	command_data_t cmd_t;
	cmd_t.data = &sensor_id;
	cmd_t.len = size;

	XnStatus ret = m_pMudulesHelper.CommandGetProperty(SENSOR_ID, &cmd_t);
	if (ret != XN_STATUS_OK)
	{
		return -1;
	}
	xnLogInfo(XN_MASK_ONI_DEVICE, "Load RGB sensor SENSOR_ID=%d", sensor_id);
	return sensor_id;
}

void XnOniDothinDevice::deviceClose()
{
	// free the allocated arrays
	//for (XnInt i = 0; i < m_numSensors; ++i)
	//{
	//	XN_DELETE_ARR(m_sensors[i].pSupportedVideoModes);
	//}
	xnLogInfo(XN_MASK_ONI_DEVICE, "DeviceClose.........");
	dtConnectHelper.DeviceClose();
	xnLogVerbose(XN_MASK_ONI_DEVICE, "Dothin device destroy...");
	if (dotDeviceProperties != nullptr)
	{
		//delete dotDeviceProperties;
		dotDeviceProperties = nullptr;
	}
}


XnStatus XnOniDothinDevice::ResetDevice()
{

	return XN_STATUS_OK;
}


XnStatus XnOniDothinDevice::FillSupportedVideoModes()
{
	xnLogInfo(XN_MASK_ONI_DEVICE, "Start RGB fill supported videoModes...");
	XnStatus nRetVal;
	obc_videomode mymodes[10];
	command_data_t cmd_modes;
	cmd_modes.data = &mymodes;
	cmd_modes.len = sizeof(mymodes) / sizeof(obc_videomode);
	nRetVal = m_pMudulesHelper.CommandGetProperty(SUPPORT_VIDEO_MODES, &cmd_modes);


	//if (m_sensors[0].pSupportedVideoModes != nullptr)
	//{
	//	XN_DELETE_ARR(m_sensors[0].pSupportedVideoModes);
	//}

	//for (int i = 0; i < cmd_modes.len; i++)
	//{
	obc_videomode videoMode = mymodes[0];
	m_sensors[s].sensorType = ONI_SENSOR_COLOR;
	m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
	m_sensors[s].pSupportedVideoModes[n].resolutionX = videoMode.width;
	m_sensors[s].pSupportedVideoModes[n].resolutionY = videoMode.height;
	m_sensors[s].pSupportedVideoModes[n].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
	m_sensors[s].pSupportedVideoModes[n].fps = 30;
	++n;
	m_sensors[s].numSupportedVideoModes = n;
	++s;
	//}
	m_numSensors = s;
	xnLogInfo(XN_MASK_ONI_DEVICE, "Fill supported videoModes end...");

	return XN_STATUS_OK;
}

OniStatus XnOniDothinDevice::getSensorInfoList(OniSensorInfo** pSensors, XnInt* numSensors)
{
	*numSensors = m_numSensors;
	*pSensors = m_sensors;

	return ONI_STATUS_OK;
}

oni::driver::StreamBase* XnOniDothinDevice::createStream(OniSensorType sensorType)
{
	XnOniDothinStream* dothinStream = NULL;
	if (sensorType == ONI_SENSOR_COLOR)
	{
		dothinStream = XN_NEW(XnOniDothinColorStream, this);
	}

	else
	{
		m_driverServices.errorLoggerAppend("XnOniDothinDevice: Can't create a stream of type %d", sensorType);
		return NULL;
	}

	OniStatus status = dothinStream->Init();
	if (status != ONI_STATUS_OK)
	{
		return NULL;
	}

	return dothinStream;
}

OniBool XnOniDothinDevice::isVideoModeSupport(OniVideoMode mode, OniSensorType sensorType)
{
	if (m_numSensors == 0)
	{
		return FALSE;
	}

	for (XnInt i = 0; i < m_numSensors; i++)
	{
		OniSensorInfo sensorInfo = m_sensors[i];

		if (sensorInfo.numSupportedVideoModes == 0)
		{
			return FALSE;
		}

		if (sensorType != sensorInfo.sensorType)
		{
			continue;
		}

		for (XnInt j = 0; j < sensorInfo.numSupportedVideoModes; j++)
		{
			OniVideoMode videoMode = sensorInfo.pSupportedVideoModes[j];
			if ((videoMode.resolutionX = mode.resolutionX && videoMode.resolutionY == mode.resolutionY))
			{
				if (sensorType == ONI_SENSOR_DEPTH)
				{
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_DEPTH_1_MM || mode.pixelFormat == ONI_PIXEL_FORMAT_DEPTH_100_UM ||
						mode.pixelFormat == ONI_PIXEL_FORMAT_SHIFT_9_2)
					{
						return TRUE;
					}
				}

				if (sensorType == ONI_SENSOR_IR)
				{
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_GRAY16 || mode.pixelFormat == ONI_PIXEL_FORMAT_RGB888)
					{
						return TRUE;
					}
				}

				if (sensorType == ONI_SENSOR_COLOR)
				{
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_RGB888 || mode.pixelFormat == ONI_PIXEL_FORMAT_YUV422 || mode.pixelFormat == ONI_PIXEL_FORMAT_YUYV)
					{
						return TRUE;
					}
				}
				if (sensorType == ONI_SENSOR_PHASE)
				{
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_GRAY16)
					{
						return TRUE;
					}
				}
			}
		}
	}

	return FALSE;
}

OniStatus XnOniDothinDevice::getDefaultVideoMode(OniVideoMode* mode, OniSensorType sensorType)
{
	if (m_numSensors == 0)
	{
		return ONI_STATUS_ERROR;
	}

	for (XnInt i = 0; i < m_numSensors; i++)
	{
		OniSensorInfo sensorInfo = m_sensors[i];
		if (sensorInfo.numSupportedVideoModes == 0)
		{
			return ONI_STATUS_ERROR;
		}

		if (sensorType != sensorInfo.sensorType)
		{
			continue;
		}

		OniVideoMode videoMode = sensorInfo.pSupportedVideoModes[0]; //ensure index 0 is original resolution, not binning or other mode
		xnOSMemCopy(mode, &videoMode, sizeof(OniVideoMode));
		break;
	}

	return ONI_STATUS_OK;
}


void XnOniDothinDevice::destroyStream(oni::driver::StreamBase* pStream)
{
	XN_DELETE(pStream);
}


OniStatus XnOniDothinDevice::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	if (dotDeviceProperties == NULL)
	{
		return ONI_STATUS_ERROR;
	}
	XnStatus rc = XN_STATUS_OK;
	//XnStatus rc = dotDeviceProperties->isPropertySupported(propertyId);
	//if (rc != XN_STATUS_OK)
	//{
	//	return ONI_STATUS_NOT_SUPPORTED;
	//}
	switch (propertyId)
	{
	case ONI_DEVICE_PROPERTY_EMITTER_STATE:
		rc = dotDeviceProperties->getProperty(XN_MODULE_PROPERTY_EMITTER_STATE, data, pDataSize);
		break;
	default:
		rc = dotDeviceProperties->getProperty(propertyId, data, pDataSize);
		if (rc != XN_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}
		break;
	}

	return ONI_STATUS_OK;
}


OniStatus XnOniDothinDevice::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	if (dotDeviceProperties == NULL)
	{
		return ONI_STATUS_ERROR;
	}
	XnStatus rc = XN_STATUS_OK;
	//XnStatus rc = dotDeviceProperties->isPropertySupported(propertyId);
	//if (rc != XN_STATUS_OK)
	//{
	//	return ONI_STATUS_NOT_SUPPORTED;
	//}

	switch (propertyId)
	{

	case OBC_LOAD_FW:
		rc = dotDeviceProperties->setProperty(propertyId, data, dataSize);
		if (rc != XN_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		//rc = FillSupportedVideoModes();
		//if (rc != XN_STATUS_OK)
		//{
		//	xnLogError(XN_MASK_ONI_DEVICE, "Fill supported videoModes failed!");
		//	return ONI_STATUS_ERROR;
		//}

		break;

	case OBC_FREQUENCY_MODE:
		rc = dotDeviceProperties->setProperty(propertyId, data, dataSize);
		if (rc != XN_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		//rc = FillSupportedVideoModes();
		//if (rc != XN_STATUS_OK)
		//{
		//	xnLogError(XN_MASK_ONI_DEVICE, "Fill supported videoModes failed!");
		//	return ONI_STATUS_ERROR;
		//}
		break;
	case ONI_DEVICE_PROPERTY_LOAD_FILE:
		//加载文件

		rc = dotDeviceProperties->setProperty(propertyId, data, dataSize);
		break;
	case ONI_DEVICE_PROPERTY_EMITTER_STATE:
		rc = dotDeviceProperties->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, data, dataSize);
		break;
	default:
		rc = dotDeviceProperties->setProperty(propertyId, data, dataSize);
		if (rc != XN_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}
		break;
	}
	return ONI_STATUS_OK;
}


OniBool XnOniDothinDevice::isPropertySupported(XnInt propertyId)
{
	if (dotDeviceProperties == NULL)
	{
		return FALSE;
	}

	XnStatus rc = dotDeviceProperties->isPropertySupported(propertyId);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_NOT_SUPPORTED;
	}

	return TRUE;
}


void XnOniDothinDevice::notifyAllProperties()
{

}


OniBool XnOniDothinDevice::isImageRegistrationModeSupported(OniImageRegistrationMode mode)
{
	return (mode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR || mode == ONI_IMAGE_REGISTRATION_OFF);
}

XnOniDothinDevice::~XnOniDothinDevice()
{


}