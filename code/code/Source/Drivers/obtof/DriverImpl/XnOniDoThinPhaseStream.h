#ifndef XN_ONI_DOTHIN_PHASE_STREAM
#define XN_ONI_DOTHIN_PHASE_STREAM

#include "XnOniDothinStream.h"
#include "DDK/XnDothinPhaseStreamProperties.h"
#include "XnDoThinDataProcessor.h"
#include "XnDothinPhaseProcessor.h"

class XnOniDothinPhaseStream : public XnOniDothinStream
{
public:
    XnOniDothinPhaseStream(XnOniDothinDevice* pDevice);

    virtual ~XnOniDothinPhaseStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();
private:
	const XnChar* XN_MASK_DOTHIN_PHASE_STREAM = "DothinPhaseStream";

};


#endif //XN_ONI_DOTHIN_PHASE_STREAM






