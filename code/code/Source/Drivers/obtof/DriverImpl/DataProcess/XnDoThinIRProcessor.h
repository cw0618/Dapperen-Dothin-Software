#pragma once
#ifndef XN_DOTHIN_IR_PROCESSOR
#define XN_DOTHIN_IR_PROCESSOR

#include "XnDothinFrameProcessor.h"
#include "DriverImpl/Sensor/XnTofSensor.h"

class XnDothinIRProcessor :
	public XnDothinFrameProcessor
{
public:
	XnDothinIRProcessor();

	virtual ~XnDothinIRProcessor();

	virtual XnStatus Init();

	virtual XnStatus start();

	/*Data start*/
	virtual void StartOfFrame();

	virtual XnStatus ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);

	XnStatus Unpack12DataTo16(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize);

	/*Data end*/
	virtual void EndOfFrame();

	virtual XnStatus stop();
protected:
	XnUInt32 m_bitPerPhasePixel;

	XnTofSensor *m_pSensor;
private:
	const XnChar* XN_MASK_DOTHIN_IR_PROCESSOR = "DothinIRProcessor";

};


#endif //XN_DOTHIN_IR_PROCESSOR





