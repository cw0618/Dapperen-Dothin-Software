#include "XnDothinIRProcessor.h"
#include <iostream>


XnDothinIRProcessor::XnDothinIRProcessor()
{

}

XnStatus XnDothinIRProcessor::Init()
{
	XnStatus rc = XN_STATUS_OK;
	rc = XnDothinFrameProcessor::Init();
	XN_IS_STATUS_OK(rc);

	return XN_STATUS_OK;
}

XnStatus XnDothinIRProcessor::start()
{	
	if (nullptr == dtStreamProperties)
	{
		xnLogError(XN_MASK_DOTHIN_IR_PROCESSOR, "StreamProperties is nullptr, GetSensor fail");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	m_bitPerPhasePixel = dtStreamProperties->GetSensorInputFormat();

	m_pSensor = dtStreamProperties->GetSensor();
	if (nullptr == m_pSensor)
	{
		xnLogError(XN_MASK_DOTHIN_IR_PROCESSOR, "m_pSensor is nullptr, pack phase data fail");
		return XN_STATUS_NULL_OUTPUT_PTR;
	}

	XnStatus res = XnDothinFrameProcessor::start();
	XN_IS_STATUS_OK(res);
	//OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	//pFrame->extraLine = dtStreamProperties->GetExtraLine();
	return XN_STATUS_OK;
}

void XnDothinIRProcessor::StartOfFrame()
{
	XnDothinFrameProcessor::StartOfFrame();
}

XnStatus XnDothinIRProcessor::ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
	XnStatus ret;
	if (nullptr == m_pSensor)
	{
		xnLogError(XN_MASK_DOTHIN_IR_PROCESSOR, "m_pSensor is nullptr, pack data fail");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	ret = m_pSensor->GetIRFrame(dotInput, dotInputSize, dotOutput, dotOutputSize);
	if (ret != XN_STATUS_OK)
	{
		//xnLogError(XN_MASK_DOTHIN_IR_PROCESSOR, "GetDepthFrame fail, %d", ret);
		return ret;
	}

	return XN_STATUS_OK;
}


/*Raw 12 to 16*/
XnStatus XnDothinIRProcessor::Unpack12DataTo16(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
	if (dotInput == NULL || dotOutput == NULL)
	{
		return XN_STATUS_ERROR;
	}

	//XnInt raw12Size = width * height * 12 / 8;
	XnInt raw12Size = dotInputSize;
	XnInt i = 0;
	for (i = 0; i < raw12Size; i += 6) {
		XnChar* pStart = (XnChar*)&dotInput[i];

		XnUInt16 shift1 = pStart[0];
		XnUInt16 shift2 = pStart[1];
		dotOutput[0] = (shift1 << 4) | (pStart[2] & 0xF);
		dotOutput[1] = (shift2 << 4) | ((pStart[2] >> 4) & 0xF);

		XnUInt16 shift3 = pStart[3];
		XnUInt16 shift4 = pStart[4];
		dotOutput[2] = (shift3 << 4) | (pStart[5] & 0xF);
		dotOutput[3] = (shift4 << 4) | ((pStart[5] >> 4) & 0xF);
		dotOutput += 4;
	}

	return XN_STATUS_OK;
}


void XnDothinIRProcessor::EndOfFrame()
{
	OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	if (dtStreamProperties != nullptr && pFrame != nullptr)
	{
		pFrame->sensorType = dtStreamProperties->GetOniSensorType();

		XnTofSensor *pSensor = dtStreamProperties->GetSensor();
		dtStreamProperties->GetVideoMode(&pFrame->videoMode);
		pSensor->UpdateFrameInfo(pFrame);
		pFrame->timestamp = frameTimestamp;
	}
	XnUInt32 nFrameID;
	m_pTripleBuffer.MarkWriteBufferAsStable(&nFrameID);
}


XnStatus XnDothinIRProcessor::stop()
{
	return XnDothinFrameProcessor::stop();
}

XnDothinIRProcessor::~XnDothinIRProcessor()
{

}
