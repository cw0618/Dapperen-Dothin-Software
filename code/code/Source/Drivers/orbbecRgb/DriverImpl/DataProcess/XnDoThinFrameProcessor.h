#pragma once
#ifndef XN_DOTHIN_FRAME_PROCESSOR
#define XN_DOTHIN_FRAME_PROCESSOR

#include "XnDoThinDataProcessor.h"

typedef enum  MIPI_RAW
{
	RAW_10 = 10,
    RAW_11 = 11,
    RAW_12 = 12,
}MIPI_RAW;


class XnDothinFrameProcessor :
	public XnDothinDataProcessor
{

public:
	XnDothinFrameProcessor();
	virtual ~XnDothinFrameProcessor();

	virtual XnStatus Init();

	virtual XnStatus start();

	void setServices(oni::driver::StreamServices* dStreamServices)
	{
		pStreamServices = dStreamServices;
	}

	/*Data start*/
	virtual void StartOfFrame();

	/*pBufferSize：Unpack buffer size*/
    virtual XnStatus XnDothinFrameProcess(XnUChar* grabBuffer, XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize);

	virtual XnStatus ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);

	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();

protected:
	oni::driver::StreamServices* pStreamServices;

	/* A pointer to the triple frame buffer of this stream. */
	XnFrameBufferManager m_pTripleBuffer;

	OniPixelFormat outPutFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;

    XnUInt64 frameTimestamp;



private:
	const XnChar* XN_MASK_DOTHIN_FRAME_PROCESSOR = "DothinFrameProcessor";
	XnBool CheckWriteBufferForOverflow(XnUInt32 nWriteSize);

	XnUInt64 GetHostTimestamp();

	static void XN_CALLBACK_TYPE OnTripleBufferNewFrame(OniFrame* pFrame, void* pCookie);

};

#endif //XN_DOTHIN_FRAME_PROCESSOR



