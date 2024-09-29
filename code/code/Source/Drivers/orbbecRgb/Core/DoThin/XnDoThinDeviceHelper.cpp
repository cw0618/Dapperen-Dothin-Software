#include "XnDothinDeviceHelper.h"


XnDothinDeviceHelper::XnDothinDeviceHelper()
{

}


XnStatus XnDothinDeviceHelper::EnumerateDothinDevice(char *DeviceName[], XnInt *pDeviceNum)
{
	if (NULL == DeviceName || NULL == pDeviceNum)
	{
		return XN_STATUS_ERROR;
	}
	
	XnInt ret = 0;
	if ((ret = EnumerateDevice(DeviceName, *pDeviceNum, pDeviceNum)) != DT_ERROR_OK)
	{
		return XN_STATUS_ERROR;
	}

	return XN_STATUS_OK;
}

XnDothinDeviceHelper::~XnDothinDeviceHelper()
{

}
