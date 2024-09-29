#include "XnDothinIRStreamProperties.h"
#include "OBCProperties.h"
#include "OpenNI.h"
#include "DriverImpl\Sensor\XnPlecoTofSensorCalc.h"

XnDothinIRStreamProperties::XnDothinIRStreamProperties(XnChar* strType, OniSensorType sensorType, XnUInt32 sensorId)
:XnDothinStreamProperties(strType, sensorType), m_sensorId(sensorId)
{

}

XnStatus XnDothinIRStreamProperties::Init(XnMx6xModulesHelper* mudulesHelper, XnDothinConnectHelper* dtConnectHelper)
{
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "Dothin stream properties init start...");

	XN_VALIDATE_INPUT_PTR(mudulesHelper);
	m_pMudulesHelper = mudulesHelper;

	XN_VALIDATE_INPUT_PTR(dtConnectHelper);
	m_pConnectHelper = dtConnectHelper;

	switch (m_sensorId)
	{
	case TOF_SENSOR_ID_PLECO:

		m_pSensor = new XnPlecoTofSensorCalc(mudulesHelper->getMx6xModulesHandle());

		break;
	case TOF_SENSOR_ID_MLX75027:

		m_pSensor = new XnMFTofSensor(mudulesHelper->getMx6xModulesHandle());

		break;
	default:
		xnLogError(XN_MASK_DOTHIN_IR_STREAM_PROP, "Sensor is not supported, sensorId: %x", m_sensorId);
		return XN_STATUS_ERROR;
		break;
	}

	m_dothinMipiPackBit = m_pSensor->GetDothinMipiPackBit();

	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "Sensor dothinMipiPackBit is %d", m_dothinMipiPackBit);
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "Dothin phase stream properties init end...");
	return XN_STATUS_OK;
}


void XnDothinIRStreamProperties::SetSensorInputFormat(const XnUInt32 inputFormat)
{
    m_dothinMipiPackBit = inputFormat;
}
XnStatus XnDothinIRStreamProperties::UpdateMode()
{
	XnStatus status = XN_STATUS_OK;

	status = m_pSensor->UpdateMode();
	XN_IS_STATUS_OK(status);

	m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);
	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
	CalcStride(XRes, m_bytesPerPixel);
	CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);
}


XnStatus XnDothinIRStreamProperties::SetDefaultVideoMode(OniVideoMode* pVideoMode)
{
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "XnDothinDepthStreamProperty SetDefaultVideoMode...");
	XN_VALIDATE_INPUT_PTR(pVideoMode);

	xnOSMemCopy(&m_defaultVideoMode, pVideoMode, sizeof(OniVideoMode));
	xnOSMemCopy(&m_curVideoMode, pVideoMode, sizeof(OniVideoMode));

	XnStatus nRetVal = ONI_STATUS_OK;

	m_outPixelFormat = m_curVideoMode.pixelFormat;

	m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);

	nRetVal = OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "IRSensor XRes is %d\n", XRes);
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "IRSensor YRes is %d\n", YRes);

	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "IRSensor m_dothinMipiPackBit is %d\n", m_dothinMipiPackBit);
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "IRSensor m_bytesPerPixel is %d\n", m_bytesPerPixel);

	//XnUInt32 binningFactor;
	//m_pSensor->GetBinningFactor(binningFactor);
	//CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);
	//CalcStride(XRes, m_bytesPerPixel);
	//CalcRequiredSize(m_pSensor->kVGADepthWidth / 2, m_pSensor->kVGADepthHeight / 2, m_bytesPerPixel);
	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
	CalcStride(XRes, m_bytesPerPixel);
	CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);
	return XN_STATUS_OK;
}

XnStatus XnDothinIRStreamProperties::ChangeVideoMode(OniVideoMode* pVideoMode)
{
	xnLogInfo(XN_MASK_DOTHIN_IR_STREAM_PROP, "XnDothinDepthStreamProperty ChangeVideoMode...");
	XN_VALIDATE_INPUT_PTR(pVideoMode);

	if (m_defaultVideoMode.resolutionX / pVideoMode->resolutionX != m_defaultVideoMode.resolutionY / pVideoMode->resolutionY)
	{
		xnLogError(XN_MASK_DOTHIN_IR_STREAM_PROP, "resolution is not matched");
		return XN_STATUS_NO_MATCH;
	}

	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = m_pSensor->SetBinningFactor(m_defaultVideoMode.resolutionX / pVideoMode->resolutionX);
	if (XN_STATUS_OK != nRetVal)
	{
		xnLogError(XN_MASK_DOTHIN_IR_STREAM_PROP, "SetBinningFactor fail %d", nRetVal);
		return XN_STATUS_PROPERTY_NOT_SET;
	}

	m_curVideoMode.pixelFormat = pVideoMode->pixelFormat;
	m_curVideoMode.resolutionX = pVideoMode->resolutionX;
	m_curVideoMode.resolutionY = pVideoMode->resolutionY;

	if (m_curVideoMode.fps != pVideoMode->fps)
	{
		nRetVal = SetFPS(&pVideoMode->fps);
		if (nRetVal == XN_STATUS_OK)
		{
			m_curVideoMode.fps = pVideoMode->fps;
		}
	}

	m_outPixelFormat = m_curVideoMode.pixelFormat;

	m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);

	OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);

	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
	CalcStride(XRes, m_bytesPerPixel);
	CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);

	return XN_STATUS_OK;
}


XnStatus XnDothinIRStreamProperties::GetVideoMode(OniVideoMode* pVideoMode)
{
	xnOSMemCopy(pVideoMode, &m_curVideoMode, sizeof(OniVideoMode));
	return XN_STATUS_OK;
}

XnStatus XnDothinIRStreamProperties::OnOutputFormatChanged(OniPixelFormat pixelFormat)
{
	OniStatus nRetVal = ONI_STATUS_OK;

	switch (pixelFormat)
	{
	case ONI_PIXEL_FORMAT_SHIFT_9_2:
		m_bytesPerPixel = sizeof(XnUInt16);
		break;
	case ONI_PIXEL_FORMAT_DEPTH_1_MM:
	case ONI_PIXEL_FORMAT_DEPTH_100_UM:
		m_bytesPerPixel = sizeof(OniDepthPixel);
		break;
	case ONI_PIXEL_FORMAT_GRAY8:
		m_bytesPerPixel = sizeof(XnUInt8);
		break;
	case ONI_PIXEL_FORMAT_GRAY16:
		m_bytesPerPixel = sizeof(XnUInt16);
		break;
	case ONI_PIXEL_FORMAT_YUV422:
	case ONI_PIXEL_FORMAT_YUYV:
		// YUV422 is actually 4 bytes for every 2 pixels
		m_bytesPerPixel = sizeof(XnUChar) * 2;
		break;
	case ONI_PIXEL_FORMAT_RGB888:
		m_bytesPerPixel = sizeof(XnUChar) * 3;
		break;
	case ONI_PIXEL_FORMAT_JPEG:
		// size is unknown.
		m_bytesPerPixel = 1;
		break;
	default:
		return (ONI_STATUS_NOT_SUPPORTED);
	}


	return XN_STATUS_OK;
}


void XnDothinIRStreamProperties::CalcRequiredSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 nBytesPerPixel)
{
	m_requiredFrameSize = XRes * YRes * nBytesPerPixel + XRes* nBytesPerPixel*m_extraLine.GetValue();
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Calc requiredSize is %d", m_requiredFrameSize);
}


void XnDothinIRStreamProperties::CalcGrabBufferSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 InputFormat)
{
	UNREFERENCED_PARAMETER(InputFormat);

	m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);//这里的参数用来设置度信，与分辨率无关，是抓取的buffer的宽和高

	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "GetGrabBufferSize is %d", m_grabBufferSize);
}


void XnDothinIRStreamProperties::CalcStride(XnUInt32 XRes, XnUInt32 nBytesPerPixel)
{
	stride = XRes * nBytesPerPixel;
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Calc stride is %d", stride);
}


XnStatus XnDothinIRStreamProperties::GetSensorInputFormatFromDot()
{
	XnStatus status = XN_STATUS_OK;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);

	XnUInt32 dispbits = 0;
	XnInt size = sizeof(XnUInt32);
	status = m_pMudulesHelper->CommandGetProperty(OBC_DISP_BITS, &dispbits, &size);
	XN_IS_STATUS_OK(status);

    m_dothinMipiPackBit = dispbits;

    xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Sensor dothinMipiPackBit is %d", m_dothinMipiPackBit);
	return XN_STATUS_OK;
}


XnStatus XnDothinIRStreamProperties::SetFPS(XnInt* fps)
{
	XnStatus status = XN_STATUS_OK;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->CommandSetProperty(OBC_FPS, fps, sizeof(XnInt));
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnStatus XnDothinIRStreamProperties::StreamMode(obc_stream_type streamType)
{
	obc_videomode mode;
	mode.width = GetXRes();
	mode.height = GetYRes();
	mode.type = streamType;

	XnStatus status = XN_STATUS_OK;
	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->CommandSetProperty(OBC_STREAM_MODE, &mode, sizeof(obc_videomode));
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnDothinIRStreamProperties::~XnDothinIRStreamProperties()
{

}
