#ifndef XN_ONI_DOTHIN_DEPTHSTREAM
#define XN_ONI_DOTHIN_DEPTHSTREAM

#include "XnOniDothinStream.h"
#include "XnDoThinDataProcessor.h"
#include "XnDothinDepthProcessor.h"

class XnOniDothinDepthStream : public XnOniDothinStream
{
public:
	XnOniDothinDepthStream(XnOniDothinDevice* pDevice);

	virtual ~XnOniDothinDepthStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();

private: 
	const XnChar* XN_MASK_DOTHIN_DEPTH_STREAM = "DothinDepthStream";
	


};


#endif //XN_ONI_DOTHIN_DEPTHSTREAM






