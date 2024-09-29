#pragma once
#ifndef XN_DOTHIN_DEVICE_PROPERTIES
#define XN_DOTHIN_DEVICE_PROPERTIES

#include "XnLog.h"
#include "XnOS.h"
#include "XnStatus.h"
#include "OniCProperties.h"
#include "OniCTypes.h"
#include "OniCEnums.h"
#include "XnDothinConnectHelper.h"
#include "XnMx6xModulesHepler.h"
#include "XnMx6xModules.h"



class XnDothinDeviceProperties
{
public:
	XnDothinDeviceProperties(XnMx6xModulesHelper* mx6xMudules_Helper);

	~XnDothinDeviceProperties();


private:
	XnMx6xModulesHelper* m_pMudulesHelper;

	XnStatus PropertyLoadFirmware(XnInt propertyId, void* pData, XnInt dataSize);

	XnStatus PropertyLoadReference(XnInt propertyId, void* pData, XnInt dataSize);

	XnStatus getPropertyModuleVersion(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus getPropertyFirmwareVersion(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus getPropertyEepromData(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus setPropertyEepromData(XnInt propertyId, void* pData, XnInt dataSize);

	XnStatus getPropertyLdmpDpuble(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus setPropertyLdmpDpuble(XnInt propertyId, void* pData, XnInt dataSize);

	XnStatus getExtPropertyPrefilter(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus SetExtPropertyPrefilter(XnInt propertyId, void* pData, XnInt dataSize);

	XnStatus setPropertySoftFilter(XnInt propertyId, void* pData, XnInt dataSize);
	

public:
	XnStatus isPropertySupported(XnInt propertyId);

	XnStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	XnStatus OBCExtension_GetProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	XnStatus OBCExtension_SetProperty(XnInt propertyId, const void* data, XnInt dataSize);



};



#endif //XN_DOTHIN_DEVICE_PROPERTIES



