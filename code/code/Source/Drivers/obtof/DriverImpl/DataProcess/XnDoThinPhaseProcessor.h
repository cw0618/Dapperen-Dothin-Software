#pragma once
#ifndef XN_DOTHIN_PHASE_PROCESSOR
#define XN_DOTHIN_PHASE_PROCESSOR

#include "XnDothinFrameProcessor.h"
#include "DriverImpl/Sensor/XnTofSensor.h"

class XnDothinPhaseProcessor :
	public XnDothinFrameProcessor
{
public:

    XnDothinPhaseProcessor();

    virtual ~XnDothinPhaseProcessor();

	virtual XnStatus Init();

	virtual XnStatus start();

	/*Data start*/
	virtual void StartOfFrame();

	virtual XnStatus ProcessPackChunk(const XnChar* dotInput, const XnUInt32 dotInputSize, XnInt16* dotOutput, XnUInt32* dotOutputSize);

	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();

protected:
    XnUInt32 m_bitPerPhasePixel;

    XnTofSensor *m_pSensor;
private:
	const XnChar* XN_MASK_DOTHIN_PHASE_PROCESSOR = "DothinPhaseProcessor";
};


#endif //XN_DOTHIN_PHASE_PROCESSOR

