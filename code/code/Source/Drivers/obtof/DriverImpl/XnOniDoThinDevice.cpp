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
#include "XnOniDothinIRStream.h"
#include "XnOniDothinDepthStream.h"
#include "XnOniDothinColorStream.h"
#include "XnOniDothinPhaseStream.h"
#include "XnOniDothinAIStream.h"
#include "OBCProperties.h"
#include <XnLog.h>
#include <vector>
#include <iostream>
#include"tofinfo.h"
#include"SensorType.h"
using namespace std;

//---------------------------------------------------------------------------
// XnOniDevice class
//---------------------------------------------------------------------------
XnOniDothinDevice::XnOniDothinDevice(const XnChar* uri, oni::driver::DriverServices& driverServices, XnOniDothinDriver* pDriver) : m_driverServices(driverServices), m_pDriver(pDriver)
{

	xnOSStrCopy(m_info.uri, uri, sizeof(m_info.uri));
	xnOSStrCopy(m_info.vendor, "Dothinkey", sizeof(m_info.vendor));
	xnOSStrCopy(m_info.name, "Dothin", sizeof(m_info.name));
	dotDeviceProperties = nullptr;
}

XnStatus XnOniDothinDevice::Init(const char* mode)
{
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = DeviceOpen();
	if (nRetVal == XN_STATUS_OK)
	{
		if (mode==nullptr)
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
	XN_IS_STATUS_OK(rc);

	return XN_STATUS_OK;
}

XnStatus XnOniDothinDevice::DeviceInit(bool model)
{
	XnStatus res = XN_STATUS_OK;

	xnLogInfo(XN_MASK_ONI_DEVICE, "Load sensor configuration...");
	//1.Init sensor config
	res = dtConnectHelper.LoadSensorConfiguration();
	XN_IS_STATUS_OK(res);

	//2.Init sensor mx6x module
	res = m_pMudulesHelper.Init(dtConnectHelper.GetDothinId());
	if (res != XN_STATUS_OK)
	{
		deviceClose();
		return -1;
	}
	if (!model)
	{
		dotDeviceProperties = XN_NEW(XnDothinDeviceProperties, &m_pMudulesHelper);
		XN_VALIDATE_INPUT_PTR(dotDeviceProperties);

		res = FillSupportedVideoModes();
		XN_IS_STATUS_OK(res);
	}
	//3.New XnDothinDeviceProperties


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
void XnOniDothinDevice::deviceClose()
{
	// free the allocated arrays
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
	return dtConnectHelper.DothinDeviceReset();
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

XnStatus XnOniDothinDevice::FillSupportedVideoModes()
{
	xnLogInfo(XN_MASK_ONI_DEVICE, "Start fill supported videoModes...");
	XnStatus nRetVal;

	OniPixelFormat depthFormats[] = { ONI_PIXEL_FORMAT_DEPTH_1_MM };
	XnSizeT depthFormatsCount = sizeof(depthFormats) / sizeof(depthFormats[0]);

	for (XnInt i = 0; i < m_numSensors; ++i)
	{
		if (m_sensors[i].pSupportedVideoModes != nullptr)
		{
			m_sensors[i].pSupportedVideoModes == nullptr;
			//XN_DELETE_ARR(m_sensors[i].pSupportedVideoModes);
		}
		
	}
	xnLogInfo(XN_MASK_ONI_DEVICE, "Start TOF fill supported videoModes...");
	m_numSensors = 0;

	vector<hw_stream_type_t> support_streams;
	vector<obc_videomode> support_videomodes;

	//1.请求Hw支持的stream type
	command_data_t cmd_t;
	cmd_t.data = nullptr;
	cmd_t.len = 0;

	nRetVal = m_pMudulesHelper.CommandGetProperty(STREAMS_CAPACITY, &cmd_t);
	if (nRetVal != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_ONI_DEVICE, "Get property STREAMS_CAPACITY failed: %d", nRetVal);
		return XN_STATUS_ERROR;
	}

	//2.Get stream fps
	XnUInt32 fps = 0;

	//3.Get stream type video mode
	hw_stream_type_t* pstreams = (hw_stream_type_t*)cmd_t.data;

	XnInt s = 0;
	for (XnInt i = 0; i < cmd_t.len; i++) //STREAMS_CAPACITY @see support_stream_
	{
		support_videomodes.clear();
		support_streams.push_back(pstreams[i]);

		hw_stream_type_t hstype = pstreams[i];
		obc_stream_type_t dst = OBC_STREAM_INVALID;
		OniSensorType sensorType;

		string sensorStr;
		//match OniSensorType
		switch (hstype)
		{
		case STREAM_IR_LASER:
			dst = OBC_STREAM_IR_LASER;
			sensorType = ONI_SENSOR_IR;
			sensorStr = "IR";
			break;

		case STREAM_DEPTH:
			dst = OBC_STREAM_DEPTH;
			sensorType = ONI_SENSOR_DEPTH;
			sensorStr = "Depth";
			break;

		case STREAM_COLOR:
			dst = OBC_STREAM_COLOR;
			sensorType = ONI_SENSOR_COLOR;
			sensorStr = "Color";
			break;

		case STREAM_TOF_PHASE:
			dst = OBC_STREAM_TOF_PHASE;
			sensorType = ONI_SENSOR_PHASE;
			sensorStr = "Phase";
			break;
		}

		if (dst == OBC_STREAM_INVALID)
		{
			xnLogInfo(XN_MASK_ONI_DEVICE, "Stream type invalid.");
			continue;
		}

		xnLogInfo(XN_MASK_ONI_DEVICE, "HW stream type %d", hstype);
#if (PLECO_SENSOR == 1)
		2.Get stream fps
		command_data_t cmd_fps;
		cmd_fps.data = &fps;
		cmd_fps.len = sizeof(XnUInt32);
		nRetVal = m_pMudulesHelper.CommandGetProperty(FPS, &cmd_fps);
		if (nRetVal != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_ONI_DEVICE, "Get property FPS failed: %d", nRetVal);
			return XN_STATUS_ERROR;
		}

		command_data_t cmd_data;
		hw_ext_msg_t ext;
		cmd_data.data = &ext;
		cmd_data.len = sizeof(ext);

		XnUInt64 resolution_mask = 0;
		XnInt msg = hstype;
		ext.subcmd = GET_SUPPORT_RESOLUTION;
		ext.msg = &msg;
		ext.p_data = &resolution_mask;

		nRetVal = m_pMudulesHelper.CommandGetProperty(EXT_PARAMS, &cmd_data);
		if (nRetVal != XN_STATUS_OK) {
			xnLogError(XN_MASK_ONI_DEVICE, "Get property EXT_PARAMS failed:%d", nRetVal);
			return XN_STATUS_ERROR;
		}
		xnLogInfo(XN_MASK_ONI_DEVICE, "Get property resolutions mask : %x ", resolution_mask);

		这里的分辨率是单张相位图，实际输出是多张相位图的整合
		每个字节包含一个支持的分辨率枚举成员，每个sensor单个频率最多支持64/8个分辨率
		首个分辨率需要是原始分辨率
		XnUInt8 *resolution_mask_byte = reinterpret_cast<XnUInt8*>(&resolution_mask);
		for (XnInt i = 0; i < sizeof(resolution_mask); ++i)
		{
			if (resolution_mask_byte[i] == RESOLUTION_UNKNOWN)
			{
				break;
			}
			for (XnInt j = 0; j < sizeof(ob_res_arr) / sizeof(ob_res_arr[0]); ++j)
			{
				if (resolution_mask_byte[i] == ob_res_arr[j].key)
				{
					obc_videomode vm;
					vm.width = ob_res_arr[j].vmode.width;
					vm.height = ob_res_arr[j].vmode.height;
					vm.type = dst;
					support_videomodes.push_back(vm);
					xnLogInfo(XN_MASK_ONI_DEVICE, "%s support resolutions  %d x %d ", sensorStr.c_str(), vm.width, vm.height);
				}
			}
		}
#elif (MF_SENSOR == 1)
		if (hstype == STREAM_TOF_PHASE)
		{
			obc_videomode vm;
			vm.width = 1280;
			vm.height = 480;
			vm.type = OBC_STREAM_TOF_PHASE;
			support_videomodes.push_back(vm);
			obc_videomode vm2;
			vm2.width = 640;
			vm2.height = 480;
			vm2.type = OBC_STREAM_TOF_PHASE;
			support_videomodes.push_back(vm2);
			fps = 30;
		}
		else if (hstype == STREAM_DEPTH)
		{
			obc_videomode vm;
			vm.width = 640;
			vm.height = 480;
			vm.type = OBC_STREAM_DEPTH;
			support_videomodes.push_back(vm);
			fps = 30;
		}
		else if (hstype == STREAM_IR_LASER)
		{
			obc_videomode vm;
			vm.width = 640;
			vm.height = 480;
			vm.type = OBC_STREAM_IR_LASER;
			support_videomodes.push_back(vm);
			fps = 30;
		}
#elif (GAEA_SENSOR == 1)
		if (hstype == STREAM_TOF_PHASE)
		{
			obc_videomode vm;
			vm.width = 1200;
			vm.height = 1920;
			vm.type = OBC_STREAM_TOF_PHASE;
			support_videomodes.push_back(vm);
			fps = 30;
		}
#endif
		//3.Switch video mode to OniSensorInfo
		XnInt nSupportedModes = support_videomodes.size();
		m_sensors[s].sensorType = sensorType;
		if (sensorType == ONI_SENSOR_DEPTH)
		{
			m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes * depthFormatsCount);
		}
		else
		{
			m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, nSupportedModes);
		}

		XnInt n = 0;
		for (auto m : support_videomodes)//s: support_stream  n:support_videomodes
		{
			if (sensorType == ONI_SENSOR_DEPTH)
			{
				for (XnInt k = 0; k < depthFormatsCount; k++)
				{
					m_sensors[s].pSupportedVideoModes[n].resolutionX = m.width;
					m_sensors[s].pSupportedVideoModes[n].resolutionY = m.height;
					m_sensors[s].pSupportedVideoModes[n].pixelFormat = depthFormats[k];
					m_sensors[s].pSupportedVideoModes[n].fps = fps;
					++n;
				}
			}
			else if (sensorType == ONI_SENSOR_IR
				|| sensorType == ONI_SENSOR_PHASE)
			{
				m_sensors[s].pSupportedVideoModes[n].resolutionX = m.width;
				m_sensors[s].pSupportedVideoModes[n].resolutionY = m.height;
				m_sensors[s].pSupportedVideoModes[n].pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
				m_sensors[s].pSupportedVideoModes[n].fps = fps;
				++n;
			}
			else
			{
				m_sensors[s].pSupportedVideoModes[n].resolutionX = m.width;
				m_sensors[s].pSupportedVideoModes[n].resolutionY = m.height;
				m_sensors[s].pSupportedVideoModes[n].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
				m_sensors[s].pSupportedVideoModes[n].fps = fps;
				++n;
			}
		}

		m_sensors[s].numSupportedVideoModes = n;
		++s;
	}
	//
	//添加IR流信息,IR流的分辨率和深度流一样
	for (int i = 0; i < 4; i++)
	{
		OniSensorInfo sensorInfo = m_sensors[i];
		if (sensorInfo.sensorType == ONI_SENSOR_DEPTH)
		{
			obc_stream_type_t dst = OBC_STREAM_IR_LASER;
			OniSensorType sensorType = ONI_SENSOR_IR;
			int n = sensorInfo.numSupportedVideoModes;
			m_sensors[s].sensorType = sensorType;
			m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, n);
			for (int j = 0; j < n; j++)
			{
				OniVideoMode videoMode = sensorInfo.pSupportedVideoModes[j];
				m_sensors[s].pSupportedVideoModes[j].resolutionX = videoMode.resolutionX;
				m_sensors[s].pSupportedVideoModes[j].resolutionY = videoMode.resolutionY;
				m_sensors[s].pSupportedVideoModes[j].pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
				m_sensors[s].pSupportedVideoModes[j].fps = videoMode.fps;
			}
			m_sensors[s].numSupportedVideoModes = n;
			++s;
		}
	}
	//添加AI流信息，phase流走AI通道
	for (int i = 0; i < 3; i++)
	{
		OniSensorInfo sensorInfo = m_sensors[i];
		if (sensorInfo.sensorType == ONI_SENSOR_PHASE)
		{
			OniSensorType sensorType = ONI_SENSOR_AI;
			int n = sensorInfo.numSupportedVideoModes;
			m_sensors[s].sensorType = sensorType;
			m_sensors[s].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, n);
			//for (int j = 0; j < n; j++)
			//{
			//j = 0;
			OniVideoMode videoMode = sensorInfo.pSupportedVideoModes[0];
			m_sensors[s].pSupportedVideoModes[0].resolutionX = 3840;
			m_sensors[s].pSupportedVideoModes[0].resolutionY = 2160;
			m_sensors[s].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_PHASE;
			m_sensors[s].pSupportedVideoModes[0].fps = videoMode.fps;
			//}
			m_sensors[s].numSupportedVideoModes = n;
			++s;
		}
	}
	++s;
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
	if (sensorType == ONI_SENSOR_IR)
	{
		dothinStream = XN_NEW(XnOniDothinIRStream, this);
	}
	else if (sensorType == ONI_SENSOR_DEPTH)
	{
		dothinStream = XN_NEW(XnOniDothinDepthStream, this);
	}
	else if (sensorType == ONI_SENSOR_COLOR)
	{
		dothinStream = XN_NEW(XnOniDothinColorStream, this);
	}
	else if (sensorType == ONI_SENSOR_PHASE)
	{
		dothinStream = XN_NEW(XnOniDothinPhaseStream, this);
	}
	else if (sensorType == ONI_SENSOR_AI)
	{
		dothinStream = XN_NEW(XnOniDothinAIStream, this);
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
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_YUV422 || mode.pixelFormat == ONI_PIXEL_FORMAT_YUYV)
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
				if (sensorType == ONI_SENSOR_AI)
				{
					if (mode.pixelFormat == ONI_PIXEL_FORMAT_PHASE)
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
	case OBC_DEVICE_RESET:
		dtConnectHelper.DothinDeviceReset();
		break;

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
	//case OBC_MODULATION_FREQUENCY:
	//	ORBTofFrequency* freqMode = (ORBTofFrequency*)data;
	//	uint16_t frequency = 0;
	//	rc = dotDeviceProperties->setProperty(propertyId, data, dataSize);
	//	break;
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
	if (propertyId == 100)
	{
		return false;
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