#pragma once
#ifndef XN_DOTHIN_CONNECT_HELPER
#define XN_DOTHIN_CONNECT_HELPER

#include <XnStatus.h>
#include <XnMemory.h>
#include <XnLog.h>
#include "XnOS.h"
#include "dtccm2.h"
#include "imagekit.h"
#include "XnDothinConfig.h"

#define XN_MASK_DOTHIN_CONNECT "XnDothinConnect"


class XnDothinConnectHelper
{
public:
	XnDothinConnectHelper();
	~XnDothinConnectHelper();

public:
	XnStatus DeviceOpen(const char *pszDeviceName);
	XnStatus DothinDeviceReset();
	XnStatus LoadSensorConfiguration();
	XnStatus Restart(XnUInt32 XRes, XnUInt32 YRes, XnUInt32* disBit);
	XnStatus DeviceClose();
	XnInt32 GetDothinId(){ return dothin_id; };

	/*Read sensor data by Dothin*/
	XnStatus ReadFrame(XnChar* buf, XnInt32 GrabSize, unsigned long * framesize);
	
private:
	XnStatus InitSensorConfig();
	XnStatus ConfigSensor();
    XnStatus InitDiskInfo();
    XnStatus SetConfigType(XnInt bits);

private:
	XnInt32 dothin_id;
	SensorTab sensor_Config;
	i2c_power_t i2c_Config;
	FrameInfo frameInfo;
	XnBool i2c_write;

};


#endif // XN_DOTHIN_CONNECT_HELPER