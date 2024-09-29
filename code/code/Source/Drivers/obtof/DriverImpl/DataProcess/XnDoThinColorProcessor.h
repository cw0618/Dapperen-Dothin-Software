#pragma once
#ifndef XN_DOTHIN_COLOR_PROCESSOR
#define XN_DOTHIN_COLOR_PROCESSOR

#include "XnDothinFrameProcessor.h"

typedef struct  RGB888_t
{
	XnUInt8 R;
	XnUInt8 G;
	XnUInt8 B;
}RGB888_t;

class XnDothinColorProcessor :
	public XnDothinFrameProcessor
{
public:
	XnDothinColorProcessor();

	virtual ~XnDothinColorProcessor();

	virtual XnStatus Init();

	/*Data start*/
	virtual void StartOfFrame();

	virtual XnStatus ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);

	/*Data end*/
	virtual void EndOfFrame();

private:

	XnUChar* unPackData;

	XnStatus unPackRaw10toRaw16(const XnUChar* dotInput, XnUInt16* dotOutput, XnUInt32 dotInputSize);

	XnStatus Raw16toBayer(const XnUChar* dotInput, XnUChar* m_unPackData, XnUInt32 nXRes, XnUInt32 nYRes);

	XnStatus BayertoRGB(XnUChar* src, XnUChar* dst, XnUInt32 nXRes, XnUInt32 nYRes);


};


#endif //XN_DOTHIN_COLOR_PROCESSOR





