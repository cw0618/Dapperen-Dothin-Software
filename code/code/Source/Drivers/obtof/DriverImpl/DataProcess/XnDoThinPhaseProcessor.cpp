#include "XnDothinPhaseProcessor.h"
#include <iostream>


XnDothinPhaseProcessor::XnDothinPhaseProcessor() :
    m_bitPerPhasePixel(0),
    m_pSensor(nullptr)
{

}


XnStatus XnDothinPhaseProcessor::Init()
{
	return XnDothinFrameProcessor::Init();
}


XnStatus XnDothinPhaseProcessor::start()
{
    if (nullptr == dtStreamProperties)
    {
        xnLogError(XN_MASK_DOTHIN_PHASE_PROCESSOR, "StreamProperties is nullptr, GetSensor fail");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    m_bitPerPhasePixel = dtStreamProperties->GetSensorInputFormat();

    m_pSensor = dtStreamProperties->GetSensor();
    if (nullptr == m_pSensor)
    {
        xnLogError(XN_MASK_DOTHIN_PHASE_PROCESSOR, "m_pSensor is nullptr, pack phase data fail");
        return XN_STATUS_NULL_OUTPUT_PTR;
    }

	XnStatus res = XnDothinFrameProcessor::start();
	XN_IS_STATUS_OK(res);
	OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	pFrame->extraLine = dtStreamProperties->GetExtraLine();
	return XN_STATUS_OK;
}


void XnDothinPhaseProcessor::StartOfFrame()
{
	XnDothinFrameProcessor::StartOfFrame();
}


XnStatus XnDothinPhaseProcessor::ProcessPackChunk(const XnChar* dotInput, const XnUInt32 dotInputSize, XnInt16* dotOutput, XnUInt32* dotOutputSize)
{
    XnStatus ret;
    if (nullptr == m_pSensor)
    {
        xnLogError(XN_MASK_DOTHIN_PHASE_PROCESSOR, "m_pSensor is nullptr, pack phase data fail");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    ret = m_pSensor->GetPhaseFrame(dotInput, dotInputSize, dotOutput, dotOutputSize);

    if (ret != XN_STATUS_OK)
	{
        //xnLogError(XN_MASK_DOTHIN_PHASE_PROCESSOR, "process sensor phase data fail");
        return ret;
	}
    
	return XN_STATUS_OK;
}

void XnDothinPhaseProcessor::EndOfFrame()
{
    OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
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


XnStatus XnDothinPhaseProcessor::stop()
{
	return XnDothinFrameProcessor::stop();
}


XnDothinPhaseProcessor::~XnDothinPhaseProcessor()
{

}
