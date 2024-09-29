#include "XnDothinColorProcessor.h"
#include "Bayer.h"
#include <iostream>
#include <imagekit.h>
#include <dtccm2.h>

XnDothinColorProcessor::XnDothinColorProcessor()
{
	unPackData = NULL;
}

XnStatus XnDothinColorProcessor::Init()
{
	return XnDothinFrameProcessor::Init();
}

void XnDothinColorProcessor::StartOfFrame()
{
	XnDothinFrameProcessor::StartOfFrame();
}

XnStatus XnDothinColorProcessor::ProcessPackChunk(const XnUChar* dotInput, const XnUInt32 dotInputSize, XnUInt16* dotOutput, XnUInt32* dotOutputSize)
{
	XnUInt32  nXRes = dtStreamProperties->GetXRes();
	XnUInt32  nYRes = dtStreamProperties->GetYRes();

	//XnUChar* dst = new XnUChar[nXRes * nYRes * 2];
	XnInt inputFormat = dtStreamProperties->GetSensorInputFormat();
	//if (inputFormat == RAW_10)
	//{
	//	unPackRaw10toRaw16(dotInput, (XnUInt16*)dst, dotInputSize);
	//}
	//XnUInt32 data_size = nXRes*nYRes * 2;
	//std::string irStr_raw = "D:\\orbbecproject\\2020\\OpenniSDK\\openniMaster\\openni2.3\\Bin\\Win32-Debug\\color16.raw";
	//xnOSSaveFile(irStr_raw.c_str(), dst, data_size);
	SetRawFormat(OUTFORMAT_BGGR, 0);
	ImageProcess((BYTE*)dotInput, (BYTE*)dotOutput, dtStreamProperties->GetXRes(), dtStreamProperties->GetYRes(), nullptr, 0);

//#if 1
//	Bayer2RGB888(dst, (XnUInt8*)dotOutput, dtStreamProperties->GetXRes(), dtStreamProperties->GetYRes(), 1);
//#endif
//
//#if 0
//
//	if (unPackData == NULL)
//	{
//		unPackData = new XnUChar[nXRes * nYRes];
//	}
//	xnOSMemSet(unPackData, 0, nXRes * nYRes * sizeof(XnUChar));
//
//	Raw16toBayer(dst, unPackData, nXRes, nYRes);
//	BayertoRGB(unPackData, (XnUChar*)dotOutput, nXRes, nYRes);
//
//#endif

	
	//std::string irStr = "D:\\orbbecproject\\2020\\OpenniSDK\\openniMaster\\openni2.3\\Bin\\Win32-Debug\\color.raw";
	//xnOSSaveFile(irStr.c_str(), dotOutput, *dotOutputSize);

	//delete[] dst;
	//dst = NULL;

	return XN_STATUS_OK;
}


XnStatus XnDothinColorProcessor::unPackRaw10toRaw16(const XnUChar* src, XnUInt16* dst, XnUInt32 dotInputSize)
{
	if (NULL == src || NULL == dst) {
		return  XN_STATUS_ERROR;
	}
	
	return XN_STATUS_OK;
}


XnStatus XnDothinColorProcessor::Raw16toBayer(const XnUChar* dotInput, XnUChar* m_unPackData, XnUInt32 nXRes, XnUInt32 nYRes)
{
	uint16_t* praw8 = (uint16_t*)dotInput;
	for (XnInt i = 0; i < nXRes * nYRes; i++)
	{
		uint16_t value = praw8[i];
		XnUInt8 bayerValue = value >> 2 & 0XFF;
		m_unPackData[i] = bayerValue;
	}

	return XN_STATUS_OK;
}

XnStatus XnDothinColorProcessor::BayertoRGB(XnUChar* src, XnUChar* dst, XnUInt32 nXRes, XnUInt32 nYRes)
{
	for (size_t i = 0; i < nYRes; i++)
	{
		XnUChar* psrc = ((XnUChar*)src)+ (nXRes * i);
		RGB888_t* rgb888 = ((RGB888_t*)dst) + (nXRes * i);
		for (XnInt j = 0; j < nXRes; j++)
		{
			XnUChar value = psrc[j];
			rgb888[j].R = value;
			rgb888[j].G = value;
			rgb888[j].B = value;

		}
	}

	return XN_STATUS_OK;
}


void XnDothinColorProcessor::EndOfFrame()
{
	XnDothinFrameProcessor::EndOfFrame();
}


XnDothinColorProcessor::~XnDothinColorProcessor()
{
	if (unPackData != NULL)
	{
		delete unPackData;
		unPackData = NULL;
	}
}
