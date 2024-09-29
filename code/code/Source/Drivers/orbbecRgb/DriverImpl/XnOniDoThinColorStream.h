#pragma once
#ifndef XN_ONI_DOTHIN_COLOR_STREAM
#define XN_ONI_DOTHIN_COLOR_STREAM

#include "XnOniDothinStream.h"
#include "XnDoThinDataProcessor.h"
#include "XnDothinColorProcessor.h"

class XnOniDothinColorStream :public XnOniDothinStream
{
public:
	XnOniDothinColorStream(XnOniDothinDevice* pDevice);

	virtual ~XnOniDothinColorStream();

	virtual OniStatus Init();

	virtual OniStatus start();

	virtual void stop();

	virtual OniStatus getProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	virtual OniStatus setProperty(XnInt propertyId, const void* data, XnInt dataSize);

	virtual OniBool isPropertySupported(XnInt propertyId);

	virtual XnInt getRequiredFrameSize();


};

#endif //XN_ONI_DOTHIN_COLOR_STREAM


