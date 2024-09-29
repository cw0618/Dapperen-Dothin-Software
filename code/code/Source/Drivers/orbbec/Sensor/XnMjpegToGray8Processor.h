#pragma once
#include "XnIRProcessor.h"
#include "XnJpeg.h"

class XnMjpegToGray8Processor : public XnFrameStreamProcessor
{
public:
	XnMjpegToGray8Processor(XnSensorIRStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
	~XnMjpegToGray8Processor();

	XnStatus Init();

	//---------------------------------------------------------------------------
	// Overridden Functions
	//---------------------------------------------------------------------------
protected:
	virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);
	virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);

	inline XnSensorIRStream* GetStream()
	{
		return (XnSensorIRStream*)XnFrameStreamProcessor::GetStream();
	}
	//---------------------------------------------------------------------------
	// Class Members
	//---------------------------------------------------------------------------
private:
	XnBuffer m_MjpegData;
	XnBuffer m_ContinuousBuffer;
	XnBuffer m_UnpackedBuffer;
	XnUInt64 m_nRefTimestamp; // needed for firmware bug workaround
	XnDepthCMOSType m_DepthCMOSType;
	XnStreamUncompJPEGContext* m_ppStreamUncompJPEGContext;

	OniIRPixel *m_pIrDataBuf;
};

