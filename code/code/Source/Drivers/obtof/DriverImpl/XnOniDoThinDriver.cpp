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
#include "XnOniDothinDriver.h"
#include "XnOniDothinDevice.h"
#include <XnCommon.h>
#include <XnOS.h>
#include "XnDothinDeviceEnumeration.h"
#include <XnLogWriterBase.h>

//---------------------------------------------------------------------------
// XnOniDriver class
//---------------------------------------------------------------------------
XnOniDothinDriver::XnOpenNILogWriter::XnOpenNILogWriter(OniDriverServices* pDriverServices) : m_pDriverServices(pDriverServices)
{
}

void XnOniDothinDriver::XnOpenNILogWriter::WriteEntry(const XnLogEntry* pEntry)
{
	m_pDriverServices->log(m_pDriverServices, pEntry->nSeverity, pEntry->strFile, pEntry->nLine, pEntry->strMask, pEntry->strMessage);
}

void XnOniDothinDriver::XnOpenNILogWriter::WriteUnformatted(const XnChar* /*strMessage*/)
{
	// DO NOTHING
}

OniStatus XnOniDothinDriver::initialize(oni::driver::DeviceConnectedCallback deviceConnectedCallback, oni::driver::DeviceDisconnectedCallback deviceDisconnectedCallback, oni::driver::DeviceStateChangedCallback deviceStateChangedCallback, void* pCookie)
{
	OniStatus nRetVal = DriverBase::initialize(deviceConnectedCallback, deviceDisconnectedCallback, deviceStateChangedCallback, pCookie);
	if (nRetVal != ONI_STATUS_OK)
	{
		return (nRetVal);
	}

	xnLogSetMaskMinSeverity(XN_LOG_MASK_ALL, XN_LOG_VERBOSE);
	m_writer.Register();

	XnStatus rc = XnDothinDeviceEnumeration::DeviceConnectEvent().Register(OnDeviceConnected, this, m_connectedEventHandle);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	rc = XnDothinDeviceEnumeration::DeviceDisConnectEvent().Register(OnDeviceDisconnected, this, m_disconnectedEventHandle);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}
	//XnDothinDeviceEnumeration::Enumeration();
	XnDothinDeviceEnumeration* donthinDeviceEnumeration = NULL;
	donthinDeviceEnumeration = XN_NEW(XnDothinDeviceEnumeration);
	XnInt32 pDeviceNum = ENUM_MAX_DEVICES;
	XnChar *dothinDevices[ENUM_MAX_DEVICES];
	XnInt ret = 0;
	if ((ret = EnumerateDevice(dothinDevices, ENUM_MAX_DEVICES, &pDeviceNum)) != DT_ERROR_OK)
	{
		xnLogError(XN_MASK_DOTHIN_DEVICE_ENUM, "Dothin device not found!");
		return ONI_STATUS_ERROR;
	}
	for (int i = 0; i < pDeviceNum; i++)
	{
		XnOniDothinDevice* pDevice;
		char* uri = dothinDevices[i];
		if (m_devices.Get(uri, pDevice) == XN_STATUS_OK)
		{
			getServices().errorLoggerAppend("Device is already open.");
			nRetVal = ONI_STATUS_ERROR;
			continue;
		}
		
		pDevice = XN_NEW(XnOniDothinDevice, uri, getServices(), this);
		XnStatus ret = pDevice->Init("model");
		if (ret == XN_STATUS_OK && pDevice != nullptr)
		{
			uint32_t senserId=pDevice->getSensorId();
			xnLogError(XN_MASK_DOTHIN_DEVICE_ENUM, "Dothin tof device senserId=%d", senserId);
			//if (senserId == kSensorIdPleco)
			//{
				XnDothinDeviceEnumeration::OnDeviceConnectivityEvent(uri, XN_USB_EVENT_DEVICE_CONNECT);
				pDevice->deviceClose();
				XN_DELETE(pDevice);
			//	break;
			//}
		}
		else
		{
			nRetVal = ONI_STATUS_ERROR;
			pDevice->deviceClose();
			XN_DELETE(pDevice);
		}

	}
	return ONI_STATUS_OK;
}

void XnOniDothinDriver::shutdown()
{
	if (m_connectedEventHandle != NULL)
	{
		XnDothinDeviceEnumeration::DeviceConnectEvent().Unregister(m_connectedEventHandle);
		m_connectedEventHandle = NULL;
	}

	if (m_disconnectedEventHandle != NULL)
	{
		XnDothinDeviceEnumeration::DeviceDisConnectEvent().Unregister(m_disconnectedEventHandle);
		m_disconnectedEventHandle = NULL;
	}

	// Close all open devices and release the memory
	for (xnl::StringsHash<XnOniDothinDevice*>::Iterator it = m_devices.Begin(); it != m_devices.End(); ++it)
	{
		XN_DELETE(it->Value());
	}

	m_devices.Clear();

	XnDothinDeviceEnumeration::Shutdown();
}

oni::driver::DeviceBase* XnOniDothinDriver::deviceOpen(const char* uri, const char* mode)
{
	XnOniDothinDevice* pDevice=nullptr;
	// if device was already opened for this uri, return the previous one
	if (m_devices.Get(uri, pDevice) == XN_STATUS_OK)
	{
		getServices().errorLoggerAppend("Device is already open.");
		return NULL;
	}

	pDevice = XN_NEW(XnOniDothinDevice, uri, getServices(), this);
	int nRetVal = pDevice->Init(mode);
	if (nRetVal != XN_STATUS_OK)
	{
		getServices().errorLoggerAppend("Could not open \"%s\": %s", uri, xnGetStatusString(nRetVal));
		return NULL;
	}

	if (nRetVal != XN_STATUS_OK)
	{
		XN_DELETE(pDevice);
		return NULL;
	}

	// Add the device and return it.
	m_devices[uri] = pDevice;
	return pDevice;
}

void XnOniDothinDriver::deviceClose(oni::driver::DeviceBase* pDevice)
{
	for (xnl::StringsHash<XnOniDothinDevice*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
	{
		if (iter->Value() == pDevice)
		{
			pDevice->deviceClose();
			m_devices.Remove(iter);
			XN_DELETE(pDevice);
			return;
		}
	}

	// not our device?!
	XN_ASSERT(FALSE);
}

void* XnOniDothinDriver::enableFrameSync(oni::driver::StreamBase** pStreams, XnInt streamCount)
{
	return NULL;
}

void XnOniDothinDriver::disableFrameSync(void* frameSyncGroup)
{
	FrameSyncGroup* pFrameSyncGroup = (FrameSyncGroup*)frameSyncGroup;

	// Find device in driver.
	xnl::StringsHash<XnOniDothinDevice*>::ConstIterator iter = m_devices.Begin();
	while (iter != m_devices.End())
	{
		//Do nothing
		++iter;
	}
}

void XN_CALLBACK_TYPE XnOniDothinDriver::OnDevicePropertyChanged(const XnChar* ModuleName, XnUInt32 nPropertyId, void* pCookie)
{
	
}

void XN_CALLBACK_TYPE XnOniDothinDriver::OnDeviceConnected(const OniDeviceInfo& deviceInfo, void* pCookie)
{
	XnOniDothinDriver* pThis = (XnOniDothinDriver*)pCookie;
	pThis->deviceConnected(&deviceInfo);
}

void XN_CALLBACK_TYPE XnOniDothinDriver::OnDeviceDisconnected(const OniDeviceInfo& deviceInfo, void* pCookie)
{
	XnOniDothinDriver* pThis = (XnOniDothinDriver*)pCookie;
	pThis->deviceDisconnected(&deviceInfo);
}
