#pragma once
#ifndef XN_DOTHIN_DEPTH_PROCESSOR
#define XN_DOTHIN_DEPTH_PROCESSOR

#include "XnDothinFrameProcessor.h"

class XnDothinDepthProcessor :
	public XnDothinFrameProcessor
{
public:
	XnDothinDepthProcessor();

	virtual ~XnDothinDepthProcessor();

	virtual XnStatus Init();

	virtual XnStatus start();

	/*Data start*/
	virtual void StartOfFrame();

	virtual XnStatus ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);

	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();

protected:
    XnTofSensor *m_pSensor;
private:
	const XnChar* XN_MASK_DOTHIN_DEPTH_PROCESSOR = "DothinDepthProcessor";

};



#endif //XN_DOTHIN_DEPTH_PROCESSOR



