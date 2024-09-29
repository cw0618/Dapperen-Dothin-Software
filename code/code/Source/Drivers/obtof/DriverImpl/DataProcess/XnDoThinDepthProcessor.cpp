#include <iostream>
#include "XnDothinDepthProcessor.h"

XnDothinDepthProcessor::XnDothinDepthProcessor():
    m_pSensor(nullptr)

{

}

XnStatus XnDothinDepthProcessor::Init()
{
	XnStatus rc = XN_STATUS_OK;
	rc = XnDothinFrameProcessor::Init();
	XN_IS_STATUS_OK(rc);

	return XN_STATUS_OK;
}

XnStatus XnDothinDepthProcessor::start()
{
    if (nullptr == dtStreamProperties)
    {
        xnLogError(XN_MASK_DOTHIN_DEPTH_PROCESSOR, "StreamProperties is nullptr, GetSensor fail");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    m_pSensor = dtStreamProperties->GetSensor();
    if (nullptr == m_pSensor)
    {
        xnLogError(XN_MASK_DOTHIN_DEPTH_PROCESSOR, "m_pSensor is nullptr, pack phase data fail");
        return XN_STATUS_NULL_OUTPUT_PTR;
    }

    XnStatus res = XnDothinFrameProcessor::start();
    XN_IS_STATUS_OK(res);
	OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	pFrame->extraLine = dtStreamProperties->GetExtraLine();
    return XN_STATUS_OK;
}

void XnDothinDepthProcessor::StartOfFrame()
{
	XnDothinFrameProcessor::StartOfFrame();
}

XnStatus XnDothinDepthProcessor::ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
    XnStatus ret;
    if (nullptr == m_pSensor)
    {
        xnLogError(XN_MASK_DOTHIN_DEPTH_PROCESSOR, "m_pSensor is nullptr, pack data fail");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    ret = m_pSensor->GetDepthFrame(dotInput, dotInputSize, dotOutput, dotOutputSize);
    if (ret != XN_STATUS_OK)
    {
        //xnLogError(XN_MASK_DOTHIN_DEPTH_PROCESSOR, "GetDepthFrame fail, %d", ret);
        return ret;
    }

	return XN_STATUS_OK;
}

void XnDothinDepthProcessor::EndOfFrame()
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

XnStatus XnDothinDepthProcessor::stop()
{
	return XnDothinFrameProcessor::stop();
}

XnDothinDepthProcessor::~XnDothinDepthProcessor()
{

}
