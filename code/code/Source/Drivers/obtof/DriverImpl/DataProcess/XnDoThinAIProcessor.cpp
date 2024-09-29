#include "XnDothinAIProcessor.h"
#include <iostream>


XnDothinAIProcessor::XnDothinAIProcessor() :
    m_bitPerAIPixel(0),
    m_pSensor(nullptr)
{

}


XnStatus XnDothinAIProcessor::Init()
{
	return XnDothinFrameProcessor::Init();
}


XnStatus XnDothinAIProcessor::start()
{
    if (nullptr == dtStreamProperties)
    {
        xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "StreamProperties is nullptr, GetSensor fail");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    m_bitPerAIPixel = dtStreamProperties->GetSensorInputFormat();

    m_pSensor = dtStreamProperties->GetSensor();
    if (nullptr == m_pSensor)
    {
        xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "m_pSensor is nullptr, pack ai data fail");
        return XN_STATUS_NULL_OUTPUT_PTR;
    }

	XnStatus res = XnDothinFrameProcessor::start();
	XN_IS_STATUS_OK(res);
	OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	pFrame->extraLine = dtStreamProperties->GetExtraLine();
	return XN_STATUS_OK;
}


void XnDothinAIProcessor::StartOfFrame()
{
	XnDothinFrameProcessor::StartOfFrame();
}


XnStatus XnDothinAIProcessor::ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
 //   XnStatus ret;
 //   if (nullptr == m_pSensor)
 //   {
 //       xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "m_pSensor is nullptr, pack ai data fail");
 //       return XN_STATUS_NULL_INPUT_PTR;
 //   }

 //   ret = m_pSensor->GetAIFrame(dotInput, dotInputSize, dotOutput, dotOutputSize);
 //   if (ret != XN_STATUS_OK)
	//{
 //       xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "process sensor ai data fail");
 //       return ret;
	//}
 //   
	return XN_STATUS_OK;
}
XnStatus XnDothinAIProcessor::ProcessAIChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, OniAIFrame* pAiFrame)
{
	XnStatus ret;
	if (nullptr == m_pSensor)
	{
		xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "m_pSensor is nullptr, pack ai data fail");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	ret = m_pSensor->GetAIFrame(dotInput, dotInputSize, pAiFrame);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_AI_PROCESSOR, "process sensor ai data fail");
		return ret;
	}

	return XN_STATUS_OK;
}

void XnDothinAIProcessor::EndOfFrame()
{
    OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	//pFrame->phase.
    if (dtStreamProperties != nullptr && pFrame != nullptr)
    {
        pFrame->sensorType = dtStreamProperties->GetOniSensorType();

        XnTofSensor *pSensor = dtStreamProperties->GetSensor();
        
        pSensor->UpdateFrameInfo(pFrame);
        
        dtStreamProperties->GetVideoMode(&pFrame->videoMode);
        pFrame->timestamp = frameTimestamp;
    }

    XnUInt32 nFrameID;
    m_pTripleBuffer.MarkWriteBufferAsStable(&nFrameID);
}


XnStatus XnDothinAIProcessor::stop()
{
	return XnDothinFrameProcessor::stop();
}


XnDothinAIProcessor::~XnDothinAIProcessor()
{

}
