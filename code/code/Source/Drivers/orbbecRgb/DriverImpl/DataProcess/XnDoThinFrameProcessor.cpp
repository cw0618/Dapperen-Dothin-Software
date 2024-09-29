#include "XnDothinFrameProcessor.h"
#include <iostream>


XnDothinFrameProcessor::XnDothinFrameProcessor()
{

}


XnStatus XnDothinFrameProcessor::Init()
{
	XnStatus nRetVal = XN_STATUS_OK;
	nRetVal = m_pTripleBuffer.Init();
	XN_IS_STATUS_OK(nRetVal);

	m_pTripleBuffer.SetNewFrameCallback(OnTripleBufferNewFrame, this);
	return XN_STATUS_OK;
}

XnStatus XnDothinFrameProcessor::start()
{
	XnStatus res = m_pTripleBuffer.Start(*pStreamServices);
	XN_IS_STATUS_OK(res);
	return XN_STATUS_OK;
}

/*Data start*/
void XnDothinFrameProcessor::StartOfFrame()
{
	//Rest frame bnuffer
	XnBuffer* dotWriteBuffer = m_pTripleBuffer.GetWriteBuffer();
	dotWriteBuffer->Reset();

	frameTimestamp = GetHostTimestamp();
}


XnStatus XnDothinFrameProcessor::XnDothinFrameProcess(XnUChar* grabBuffer, XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize)

{
	StartOfFrame();

	XnStatus Status;
	XnBuffer* dotWriteBuffer = m_pTripleBuffer.GetWriteBuffer();

    if (!CheckWriteBufferForOverflow(requiredFrameSize))
	{
		return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
	}

	XnUInt16* dotOutput = (XnUInt16*)dotWriteBuffer->GetUnsafeWritePointer();
    Status = ProcessPackChunk(grabBuffer, grabBufferSize, dotOutput, &requiredFrameSize);
    dotWriteBuffer->UnsafeUpdateSize(requiredFrameSize);

	if (Status != XN_STATUS_OK)
	{
		return Status;
	}

	EndOfFrame();
	return XN_STATUS_OK;
}

XnStatus XnDothinFrameProcessor::ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
	return XN_STATUS_OK;
}


/*Data end*/
void XnDothinFrameProcessor::EndOfFrame()
{
	OniFrame* pFrame = m_pTripleBuffer.GetWriteFrame();
	if (dtStreamProperties != NULL && pFrame != NULL)
	{
		pFrame->sensorType = dtStreamProperties->GetOniSensorType();
        pFrame->width = dtStreamProperties->GetXRes();
        pFrame->height = dtStreamProperties->GetYRes();
        pFrame->stride = dtStreamProperties->GetStride();
		pFrame->extraLine = 0;
        dtStreamProperties->GetVideoMode(&pFrame->videoMode);
        pFrame->timestamp = frameTimestamp;
	}
	
	XnUInt32 nFrameID;
	m_pTripleBuffer.MarkWriteBufferAsStable(&nFrameID);
}


/*
* Checks if write buffer has overflowed, if so, a log will be issued and buffer will reset.
*/
XnBool XnDothinFrameProcessor::CheckWriteBufferForOverflow(XnUInt32 nWriteSize)
{
	if (m_pTripleBuffer.GetWriteBuffer()->GetFreeSpaceInBuffer() < nWriteSize)
	{
		XnBuffer* pBuffer = m_pTripleBuffer.GetWriteBuffer();
		xnLogWarning(XN_MASK_DOTHIN_FRAME_PROCESSOR, "%s Frame Buffer overflow! current size: %d", dtStreamProperties->GetSensorTypeName(), pBuffer->GetSize());
		return FALSE;
	}
	return TRUE;
}


XnUInt64 XnDothinFrameProcessor::GetHostTimestamp()
{
	XnUInt64 nNow;
	xnOSGetHighResTimeStamp(&nNow);
	return nNow;
}

XnStatus XnDothinFrameProcessor::stop()
{
	m_pTripleBuffer.Stop();
	return XN_STATUS_OK;
}


void XN_CALLBACK_TYPE XnDothinFrameProcessor::OnTripleBufferNewFrame(OniFrame* pFrame, void* pCookie)
{
	XnDothinFrameProcessor* frameProcessor = (XnDothinFrameProcessor*)pCookie;
    frameProcessor->NewFrameAvailable(pFrame);
}


XnDothinFrameProcessor::~XnDothinFrameProcessor()
{
	m_pTripleBuffer.Free();
}
