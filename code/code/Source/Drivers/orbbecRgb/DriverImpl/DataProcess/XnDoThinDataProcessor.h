#pragma once
#ifndef XN_DOTHIN_DATA_PROCESS
#define XN_DOTHIN_DATA_PROCESS

#include <Driver/OniDriverAPI.h>
#include <DDK/XnFrameBufferManager.h>
#include <DDK/XnDothinStreamProperties.h>
#include "XnOS.h"
#include "XnLog.h"

class XnDothinDataProcessor
{
public:
	XnDothinDataProcessor();

	virtual ~XnDothinDataProcessor();

public:
	virtual XnStatus Init();

	virtual void setServices(oni::driver::StreamServices* dStreamServices);

	void setDeviceProperties(XnDothinStreamProperties* doTStreamProperties)
	{ 
		dtStreamProperties = doTStreamProperties;
	}

	virtual XnStatus start();

		/*Data start*/
	virtual void StartOfFrame();

	/*pBufferSize£ºUnpack buffer size*/
    virtual void  XnDothinFrameDataCallback(XnUChar* grabBuffer, XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize);

	/*pBufferSize£ºUnpack buffer size*/
    virtual XnStatus XnDothinFrameProcess(XnUChar* grabBuffer, XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize);

	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();

public:
	void NewFrameAvailable(OniFrame* pFrame);

	typedef void (XN_CALLBACK_TYPE* NewFrameDataAvailable)(OniFrame* pFrame, void* pCookie);

	void setNewFrameDataCallback(NewFrameDataAvailable func, void* pCookie);

protected:
	XnDothinStreamProperties* dtStreamProperties;

	/*Frame data callback*/
	NewFrameDataAvailable newFrameDataAvailable;

	void* newFrameDataAvailableCookie;

};


#endif //XN_DOTHIN_DATA_PROCESS