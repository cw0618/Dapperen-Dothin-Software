#include "XnMjpegToGray8Processor.h"
#include <XnProfiling.h>
#include <fstream>

#define XN_INPUT_ELEMENT_SIZE 5
/* The size of an output element for unpacking. */
#define XN_OUTPUT_ELEMENT_SIZE 8

using namespace std;

XnMjpegToGray8Processor::XnMjpegToGray8Processor(XnSensorIRStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager) :
	XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_START, XN_SENSOR_PROTOCOL_RESPONSE_IMAGE_END)
{
	m_ppStreamUncompJPEGContext = NULL;
}


XnMjpegToGray8Processor::~XnMjpegToGray8Processor()
{
	if (m_ppStreamUncompJPEGContext != NULL) {
		XnStreamFreeUncompressImageJ(&m_ppStreamUncompJPEGContext);
	}
}


XnStatus XnMjpegToGray8Processor::Init()
{
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = XnFrameStreamProcessor::Init();
	XN_IS_STATUS_OK(nRetVal);

	XN_VALIDATE_BUFFER_ALLOCATE(m_ContinuousBuffer, XN_INPUT_ELEMENT_SIZE);

	switch (GetStream()->GetOutputFormat())
	{
	case ONI_PIXEL_FORMAT_GRAY16:
		break;
	case ONI_PIXEL_FORMAT_RGB888:
		XN_VALIDATE_BUFFER_ALLOCATE(m_UnpackedBuffer, GetExpectedOutputSize());
		break;
	case ONI_PIXEL_FORMAT_GRAY8:
		XN_VALIDATE_BUFFER_ALLOCATE(m_UnpackedBuffer, GetExpectedOutputSize() * 2);
		break;
	default:
		assert(0);
		return XN_STATUS_ERROR;
	}
	XN_VALIDATE_BUFFER_ALLOCATE(m_MjpegData, GetExpectedOutputSize());

	nRetVal = XnStreamInitUncompressImageJ(&m_ppStreamUncompJPEGContext);

	return nRetVal;
}


void Gray8to888(XnUInt8* pInput, XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize)
{
	XnUInt8* pInputEnd = pInput + nInputSize;
	XnUInt8* pOutputOrig = pOutput;
	XnUInt8* pOutputEnd = pOutput + *pnOutputSize;

	while (pInput != pInputEnd && pOutput < pOutputEnd)
	{

		*pOutput = *pInput;
		*(pOutput + 1) = *pOutput;
		*(pOutput + 2) = *pOutput;

		pOutput += 3;
		pInput++;
	}

	*pnOutputSize = (XnUInt32)(pOutput - pOutputOrig);
}

void XnMjpegToGray8Processor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize)
{
	XN_PROFILING_START_SECTION("XnJpegToRGBImageProcessor::ProcessFramePacketChunk")

		// append to mjpeg data buffer
		if (m_MjpegData.GetFreeSpaceInBuffer() < nDataSize)
		{
			xnLogWarning(XN_MASK_SENSOR_PROTOCOL_IMAGE, "Bad overflow image! %d", m_MjpegData.GetSize());
			FrameIsCorrupted();
			m_MjpegData.Reset();
		}
		else
		{
			m_MjpegData.UnsafeWrite(pData, nDataSize);
		}

	XN_PROFILING_END_SECTION
}



void XnMjpegToGray8Processor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
	XN_PROFILING_START_SECTION("XnMjpegToGray8Processor::OnEndOfFrame")

	XnBuffer* pWriteBuffer = GetWriteBuffer();
	XnUInt32 nOutputSize = pWriteBuffer->GetMaxSize();
	XnUChar* nOutputData = pWriteBuffer->GetUnsafeWritePointer();
	XnUChar* nUnPackData = nOutputData;

	if (ONI_PIXEL_FORMAT_RGB888 == GetStream()->GetOutputFormat()) {
		nUnPackData = m_UnpackedBuffer.GetData();
	}

	//Need to uncompress MJPEG to gray
	XnStatus nRetVal = XnStreamUncompressImageJ(&m_ppStreamUncompJPEGContext, m_MjpegData.GetData(), m_MjpegData.GetSize(), nUnPackData, &nOutputSize);
	if (nRetVal != XN_STATUS_OK)
	{
		xnLogWarning(XN_MASK_SENSOR_PROTOCOL_IMAGE, "Failed to uncompress MJPEG for frame %d: %s (%d)\n", GetCurrentFrameID(), xnGetStatusString(nRetVal), pWriteBuffer->GetSize());
		FrameIsCorrupted();

		XnDumpFile* badImageDump = xnDumpFileOpen(XN_DUMP_BAD_IMAGE, "BadImage_%d.jpeg", GetCurrentFrameID());
		xnDumpFileWriteBuffer(badImageDump, m_MjpegData.GetData(), m_MjpegData.GetSize());
		xnDumpFileClose(badImageDump);
	}

	pWriteBuffer->UnsafeUpdateSize(nOutputSize);
	m_MjpegData.Reset();

	XnUInt32 width = GetStream()->GetXRes();
	XnUInt32 height = GetStream()->GetYRes();
	OniFrame* pFrame = GetWriteFrame();
	pFrame->sensorType = ONI_SENSOR_IR;

	pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();
	pFrame->videoMode.resolutionX = GetStream()->GetXRes();
	pFrame->videoMode.resolutionY = GetStream()->GetYRes();
	pFrame->videoMode.fps = GetStream()->GetFPS();
	pFrame->width = (int)width;
	pFrame->height = (int)height;

	pFrame->cropOriginX = 0;
	pFrame->cropOriginY = 0;
	pFrame->croppingEnabled = FALSE;
	pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

	if (ONI_PIXEL_FORMAT_RGB888 == GetStream()->GetOutputFormat()) {
		XnUInt32 outSize = GetExpectedOutputSize();
		 Gray8to888(nUnPackData, nOutputSize, nOutputData, &outSize);
	}

	XnFrameStreamProcessor::OnEndOfFrame(pHeader);

	XN_PROFILING_END_SECTION
}
