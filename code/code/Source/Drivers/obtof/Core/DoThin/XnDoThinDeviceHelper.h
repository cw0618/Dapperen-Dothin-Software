#pragma once

#ifndef XN_DOTHINDEVICE_HELPER_H
#define XN_DOTHINDEVICE_HELPER_H


#include "XnOS.h"
#include "dtccm2.h"



class XnDothinDeviceHelper
{
public:
	XnDothinDeviceHelper();
	~XnDothinDeviceHelper();

public:

	XnStatus EnumerateDothinDevice(char *DeviceName[], XnInt *pDeviceNum);


};

#endif //XN_DOTHINDEVICE_HELPER_H
