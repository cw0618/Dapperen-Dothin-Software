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
#ifndef XNONIDEVICE_H
#define XNONIDEVICE_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <Driver/OniDriverAPI.h>
#include <XnLib.h>
#include <OBExtensionCommand.h>
#include "XnDothinConnectHelper.h"
#include "XnMx6xModulesHepler.h"
#include "DDK/XnDothinDeviceProperties.h"
#define XN_FIRMWARE_FILE "\\ov9282_1.8.90.bin"
//#define XN_REF_FILE "\\B5-tiger.ref"

#define XN_REF_FILE "\\082201T.ref"


//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

class XnOniStream;
class XnOniDothinDriver;

class XnOniDothinDevice :
	public oni::driver::DeviceBase
{
public:
	XnOniDothinDevice(const char* uri, oni::driver::DriverServices& driverServices, XnOniDothinDriver* pDriver);

	XnStatus Init(const char* mode);

	//Device open
	XnStatus DeviceOpen();

	//Device init
	XnStatus DeviceInit(bool model);

	//Device close
	void deviceClose();
	uint32_t getSensorId();
	//Device reset
	XnStatus ResetDevice();

	OniStatus getSensorInfoList(OniSensorInfo** pSensors, XnInt* numSensors);
	
	OniDeviceInfo* GetInfo() { return &m_info; }

	//Create this device stream
	oni::driver::StreamBase* createStream(OniSensorType sensorType);

	//Destroy this device stream
	void destroyStream(oni::driver::StreamBase* pStream);

	
	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);
	
	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);
	
	virtual OniBool isPropertySupported(XnInt propertyId);
	
	virtual void notifyAllProperties();

	virtual OniBool isImageRegistrationModeSupported(OniImageRegistrationMode mode);

	XnOniDothinDriver* GetDriver()
	{
		return m_pDriver;
	}

	XnDothinConnectHelper* getDothinConnectHelper()
	{
		return &dtConnectHelper;
	}

	XnMx6xModulesHelper* getMx6xModulesHelper()
	{
		return &m_pMudulesHelper;
	}

	OniBool isVideoModeSupport(OniVideoMode mode, OniSensorType sensorType);

	OniStatus getDefaultVideoMode(OniVideoMode* mode, OniSensorType sensorType);

	virtual ~XnOniDothinDevice();

private:
	const XnChar* XN_MASK_ONI_DEVICE = "XnOniDothinDevice";
	XnStatus FillSupportedVideoModes();

	OniDeviceInfo m_info;
	XnInt n = 0;
	XnInt s = 0;
	XnInt m_numSensors;
	OniSensorInfo m_sensors[10];

	oni::driver::DriverServices& m_driverServices;
	XnOniDothinDriver* m_pDriver;

	XnDothinConnectHelper dtConnectHelper;
	XnMx6xModulesHelper m_pMudulesHelper;
	XnDothinDeviceProperties* dotDeviceProperties=NULL;

	OBExtension m_Ext;

};

#endif // XNONIDEVICE_H
