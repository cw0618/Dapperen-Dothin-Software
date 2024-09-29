#pragma once
#ifndef XN_DOTHIN_AI_PROCESSOR
#define XN_DOTHIN_AI_PROCESSOR

#include "XnDothinFrameProcessor.h"
#include "DriverImpl/Sensor/XnTofSensor.h"

class XnDothinAIProcessor :
	public XnDothinFrameProcessor
{
public:

    XnDothinAIProcessor();

    virtual ~XnDothinAIProcessor();

	virtual XnStatus Init();

	virtual XnStatus start();

	/*Data start*/
	virtual void StartOfFrame();

	virtual XnStatus ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);
	virtual XnStatus XnDothinAIProcessor::ProcessAIChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, OniAIFrame* pAiFrame);
	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();

protected:
    XnUInt32 m_bitPerAIPixel;

    XnTofSensor *m_pSensor;
private:
	const XnChar* XN_MASK_DOTHIN_AI_PROCESSOR = "DothinAIProcessor";
};


#endif //XN_DOTHIN_AI_PROCESSOR

