#include "XnDothinDeviceEnumeration.h"



XnBool XnDothinDeviceEnumeration::m_initialized = FALSE;
XnDothinDeviceEnumeration::DeviceConnectivityEvent XnDothinDeviceEnumeration::m_deviceConnectEvent;
XnDothinDeviceEnumeration::DeviceConnectivityEvent XnDothinDeviceEnumeration::m_deviceDisConnectEvent;
XnDothinDeviceEnumeration::DevicesHash XnDothinDeviceEnumeration::m_devices;
XN_CRITICAL_SECTION_HANDLE XnDothinDeviceEnumeration::ms_lock;

XnDothinDeviceEnumeration::XnDothinDeviceEnumeration()
{
}


XnDothinDeviceEnumeration::~XnDothinDeviceEnumeration()
{
}

XnStatus XnDothinDeviceEnumeration::Enumeration()
{
	XnStatus res = XN_STATUS_OK;

	if (m_initialized)
	{
		return XN_STATUS_OK;
	}

	res = xnOSCreateCriticalSection(&ms_lock);
	XN_IS_STATUS_OK(res);

	XnInt32 pDeviceNum = ENUM_MAX_DEVICES;
	XnChar *DeviceNames[ENUM_MAX_DEVICES];

	XnInt ret = 0;
	if ((ret = EnumerateDevice(DeviceNames, ENUM_MAX_DEVICES, &pDeviceNum)) != DT_ERROR_OK)
	{
		xnLogError(XN_MASK_DOTHIN_DEVICE_ENUM, "Dothin device not found!");
		return XN_STATUS_USB_DEVICE_NOT_FOUND;
	}

	if (pDeviceNum == 0)
	{
		xnLogError(XN_MASK_DOTHIN_DEVICE_ENUM, "Dothin device not found!");
		return XN_STATUS_USB_DEVICE_NOT_FOUND;
	}

	xnLogInfo(XN_MASK_DOTHIN_DEVICE_ENUM, "Dothin device is found...");

	//for (XnUInt32 j = 0; j < pDeviceNum; ++j)
	//{
	//	std::string tofType = "-rgb";
	//	std::string device = DeviceNames[j] + tofType;
	//	OnDeviceConnectivityEvent(device.c_str(), XN_USB_EVENT_DEVICE_CONNECT);
	//	GlobalFree(DeviceNames[j]);
	//}
	m_initialized = TRUE;
	return XN_STATUS_OK;
}


OniDeviceInfo* XnDothinDeviceEnumeration::GetDeviceInfo(const XnChar* deviceName)
{
	OniDeviceInfo* dInfo = NULL;
	xnl::AutoCSLocker lock(ms_lock);
	if (m_devices.Get(deviceName, dInfo) == XN_STATUS_OK)
	{
		return dInfo;
	}

	return NULL;
}


void XnDothinDeviceEnumeration::OnDeviceConnectivityEvent(const XnChar* deviceName, XnUSBEventType eventType)
{
	//xnl::AutoCSLocker lock(ms_lock);
	if (eventType == XN_USB_EVENT_DEVICE_CONNECT)
	{
		//if (m_devices.Find(deviceName) == m_devices.End())
		//{
			OniDeviceInfo deviceInfo;
			xnOSStrCopy(deviceInfo.uri, deviceName, sizeof(deviceInfo.uri));
			xnOSStrCopy(deviceInfo.vendor, "Dothinkey", sizeof(deviceInfo.vendor));
			xnOSStrCopy(deviceInfo.name, "rgb", sizeof(deviceInfo.name));

			// add it to hash
			m_devices.Set(deviceName, deviceInfo);

			// raise event
			m_deviceConnectEvent.Raise(deviceInfo);
		//}
	}
	else if (eventType == XN_USB_EVENT_DEVICE_DISCONNECT)
	{
		OniDeviceInfo deviceInfo;
		if (XN_STATUS_OK == m_devices.Get(deviceName, deviceInfo))
		{
			// raise event
			m_deviceDisConnectEvent.Raise(deviceInfo);

			// remove it
			m_devices.Remove(deviceName);
		}
	}

}
XnStatus XnDothinDeviceEnumeration::Shutdown()
{
	if (m_initialized)
	{
		m_deviceConnectEvent.Clear();
		m_deviceConnectEvent.Clear();

		xnOSCloseCriticalSection(&ms_lock);

		m_devices.Clear();
		m_initialized = FALSE;
	}

	return XN_STATUS_OK;
}