#ifndef XN_ONI_DOTHIN_AI_STREAM
#define XN_ONI_DOTHIN_AI_STREAM

#include "XnOniDothinStream.h"
#include "DDK/XnDothinAIStreamProperties.h"
#include "XnDoThinDataProcessor.h"
#include "XnDothinAIProcessor.h"

class XnOniDothinAIStream : public XnOniDothinStream
{
public:
    XnOniDothinAIStream(XnOniDothinDevice* pDevice);

    virtual ~XnOniDothinAIStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();
private:
	const XnChar* XN_MASK_DOTHIN_AI_STREAM = "DothinAIStream";

};


#endif //XN_ONI_DOTHIN_AI_STREAM






