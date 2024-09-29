#include "XnDoThinDataProcessor.h"


XnDothinDataProcessor::XnDothinDataProcessor()
{
}

XnStatus XnDothinDataProcessor::Init()
{
	return XN_STATUS_OK;
}

void XnDothinDataProcessor::setServices(oni::driver::StreamServices* dStreamServices)
{
}

XnStatus XnDothinDataProcessor::start()
{
	return XN_STATUS_OK;
}

void XnDothinDataProcessor::StartOfFrame()
{
}

void XnDothinDataProcessor::XnDothinFrameDataCallback(XnChar* grabBuffer, XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize)
{
    XnDothinFrameProcess(grabBuffer, grabBufferSize, requiredFrameSize);
}

void XnDothinDataProcessor::NewFrameAvailable(OniFrame* pFrame)
{
	if (newFrameDataAvailable != NULL && newFrameDataAvailableCookie != NULL)
	{
		newFrameDataAvailable(pFrame, newFrameDataAvailableCookie);
	}
}

void XnDothinDataProcessor::setNewFrameDataCallback(NewFrameDataAvailable func, void* pCookie)
{
	newFrameDataAvailable = func;
	newFrameDataAvailableCookie = pCookie;
}

XnStatus XnDothinDataProcessor::XnDothinFrameProcess(XnChar* dBuffer, XnUInt32 nBufferSize, XnUInt32 pBufferSize)
{
	return XN_STATUS_OK;
}

void XnDothinDataProcessor::EndOfFrame()
{

}

XnStatus XnDothinDataProcessor::stop()
{

	return XN_STATUS_OK;
}

XnDothinDataProcessor::~XnDothinDataProcessor()
{

}
