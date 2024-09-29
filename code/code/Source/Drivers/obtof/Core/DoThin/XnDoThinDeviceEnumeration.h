#pragma once

#ifndef XN_DOTHIN_DEVICE_ENUMERATION
#define XN_DOTHIN_DEVICE_ENUMERATION


#include <XnEvent.h>
#include <OniCTypes.h>
#include <XnHash.h>
#include <XnUSB.h>
#include "XnLog.h"
#include "XnOS.h"
#include "dtccm2.h"
#define  XN_MASK_DOTHIN_DEVICE_ENUM "DothinDeviceEnum"
#define  ENUM_MAX_DEVICES 2  //最大Dothin设备枚举数

class XnDothinDeviceEnumeration
{
public:
	typedef xnl::Event1Arg<const OniDeviceInfo&> DeviceConnectivityEvent;

	static XnStatus Enumeration();
	static OniDeviceInfo* GetDeviceInfo(const XnChar* deviceName);

	//return DeviceConnectivityEvent
	static DeviceConnectivityEvent::Interface& DeviceConnectEvent()
	{ 
		return m_deviceConnectEvent; 
	}

	static DeviceConnectivityEvent::Interface& DeviceDisConnectEvent()
	{
		return m_deviceDisConnectEvent;
	}
	static void OnDeviceConnectivityEvent(const XnChar* deviceName, XnUSBEventType eventType);
	static XnStatus Shutdown();
	
public:
	XnDothinDeviceEnumeration();

	~XnDothinDeviceEnumeration();

private:

	static XnBool m_initialized;

	typedef xnl::StringsHash<OniDeviceInfo> DevicesHash;
	//for cache enumeration device
	static DevicesHash m_devices;

	static DeviceConnectivityEvent m_deviceConnectEvent;
	static DeviceConnectivityEvent m_deviceDisConnectEvent;

	
	static XN_CRITICAL_SECTION_HANDLE ms_lock;

};

#endif  //XN_DOTHIN_DEVICE_ENUMERATION