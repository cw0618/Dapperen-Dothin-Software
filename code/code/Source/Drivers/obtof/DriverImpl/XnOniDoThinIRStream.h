#ifndef XN_ONI_DOTHIN_IRSTREAM
#define XN_ONI_DOTHIN_IRSTREAM

#include "XnOniDothinStream.h"
#include "XnDoThinDataProcessor.h"
#include "XnDothinIRProcessor.h"
#include "DDK/XnDothinIRStreamProperties.h"

class XnOniDothinIRStream : public XnOniDothinStream
{
public:
	XnOniDothinIRStream(XnOniDothinDevice* pDevice);

	virtual ~XnOniDothinIRStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();

private: 
	const XnChar* XN_MASK_DOTHIN_IR_STREAM = "DothinIRStream";

	


};


#endif //XN_ONI_DOTHIN_IRSTREAM






