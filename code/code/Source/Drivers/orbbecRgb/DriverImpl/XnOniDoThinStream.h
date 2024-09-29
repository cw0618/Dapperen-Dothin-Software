#ifndef XN_ONI_DOTHIN_STREAM
#define XN_ONI_DOTHIN_STREAM

#include <Driver/OniDriverAPI.h>
#include "XnOS.h"
#include "OniCProperties.h"
#include "XnOniDothinDevice.h"
#include "DDK/XnDothinStreamProperties.h"
#include "XnMx6xModules.h"
#include "XnDDK.h"
#include "XnLog.h"
#include "XnDothinStreamImple.h"

class XnOniDothinStream : 
	public oni::driver::StreamBase
{
public:

	XnOniDothinStream(XnChar* m_strType, OniSensorType m_sensorType, XnOniDothinDevice* pDevice);

	virtual ~XnOniDothinStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	void setServices(oni::driver::StreamServices* pStreamServices);

	virtual OniStatus SetVideoMode(OniVideoMode* pVideoMode);

	virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode);

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();

    XnOniDothinDevice* GetDevice() { return m_pDevice; };

    XnBool IsStart() { return m_bIsStart; };


protected:

	OniSensorType m_sensorType;

	const XnChar* m_strType;

	XnOniDothinDevice* m_pDevice;

	XnBool m_bIsStart = FALSE;

	XnDothinStreamProperties* dtStreamProperties;

	XnDothinDataProcessor* dtDataProcessor;

	/*start stream or stop stream.l*/
	OniStatus SetStreamMode(obc_stream_type streamType);

	OniStatus SetActualRead(XnBool bRead);

	static void XN_CALLBACK_TYPE OnDotNewFrameCallback(OniFrame* pFrame, void* pCookie);

	void raiseNewDataFrame(OniFrame* pFrame);


private:
	const XnChar* XN_MASK_DOTHIN_STREAM = "DothinStream";
	oni::driver::StreamServices* m_pServices;

public:

	XnDothinConnectHelper* dtConnectHelper;
	XnDothinStreamImple* dothinStreamImple;

};


#endif //XN_ONI_DOTHIN_STREAM

