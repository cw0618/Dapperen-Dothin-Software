#include "XnDothinStreamProperties.h"
#include "OBCProperties.h"


XnDothinStreamProperties::XnDothinStreamProperties(XnChar* strType, OniSensorType sensorType) :
    m_strType(strType),
	m_sensorType(sensorType),
    m_pSensor(nullptr)
{
    xnOSMemSet(&m_curVideoMode, 0, sizeof(OniVideoMode));
    xnOSMemSet(&m_defaultVideoMode, 0, sizeof(OniVideoMode));
}

XnStatus XnDothinStreamProperties::Init(XnMx6xModulesHelper* mudulesHelper, XnDothinConnectHelper* dtConnectHelper)
{
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Dothin stream properties init start...");

	XN_VALIDATE_INPUT_PTR(mudulesHelper);
	m_pMudulesHelper = mudulesHelper;

	XN_VALIDATE_INPUT_PTR(dtConnectHelper);
	m_pConnectHelper = dtConnectHelper;

	XnStatus rc = GetSensorInputFormatFromDot();
	XN_IS_STATUS_OK(rc);

	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Dothin stream properties init end...");
	return XN_STATUS_OK;
}


void XnDothinStreamProperties::SetSensorInputFormat(const XnUInt32 inputFormat)
{
	
    m_dothinMipiPackBit = inputFormat;
}


XnStatus XnDothinStreamProperties::SetDefaultVideoMode(OniVideoMode* pVideoMode)
{
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Dothin stream properties VideoMode set default...");
	XN_VALIDATE_INPUT_PTR(pVideoMode);

	xnOSMemCopy(&m_curVideoMode, pVideoMode, sizeof(OniVideoMode));
    xnOSMemCopy(&m_defaultVideoMode, pVideoMode, sizeof(OniVideoMode));

	XnStatus nRetVal = ONI_STATUS_OK;

	m_outPixelFormat = m_curVideoMode.pixelFormat;
	XRes = m_curVideoMode.resolutionX;
	YRes = m_curVideoMode.resolutionY;

	OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);
	
	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);

	CalcStride(XRes, m_bytesPerPixel);

    CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);

	return XN_STATUS_OK;
}

XnStatus XnDothinStreamProperties::ChangeVideoMode(OniVideoMode* pVideoMode)
{
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "XnDothinStreamProperties ChangeVideoMode...");
	XN_VALIDATE_INPUT_PTR(pVideoMode);

	XnStatus nRetVal = ONI_STATUS_OK;

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
	XRes = m_curVideoMode.resolutionX;
	YRes = m_curVideoMode.resolutionY;

	OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);

	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);

	CalcStride(XRes, m_bytesPerPixel);

    CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);

	return XN_STATUS_OK;
}


XnStatus XnDothinStreamProperties::GetVideoMode(OniVideoMode* pVideoMode)
{
	xnOSMemCopy(pVideoMode, &m_curVideoMode, sizeof(OniVideoMode));
	return XN_STATUS_OK;
}


XnStatus XnDothinStreamProperties::StreamMode(obc_stream_type streamType)
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


XnStatus XnDothinStreamProperties::UpdateMode()
{
    return XN_STATUS_NOT_IMPLEMENTED;
}


XnStatus XnDothinStreamProperties::OnOutputFormatChanged(OniPixelFormat pixelFormat)
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


void XnDothinStreamProperties::CalcGrabBufferSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 InputFormat)
{
    m_grabBufferSize = XRes * YRes * InputFormat / 8;
    xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Calc back frame Size is %d", m_grabBufferSize);
}


void XnDothinStreamProperties::CalcRequiredSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 nBytesPerPixel)
{
	m_requiredFrameSize = XRes * YRes * nBytesPerPixel;
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Calc requiredSize is %d", m_requiredFrameSize);
}


void XnDothinStreamProperties::CalcStride(XnUInt32 XRes, XnUInt32 nBytesPerPixel)
{
	stride = XRes * nBytesPerPixel;
	xnLogInfo(XN_MASK_DOTHIN_STREAM_PPS, "Calc stride is %d", stride);
}


XnStatus XnDothinStreamProperties::GetSensorInputFormatFromDot()
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


XnStatus XnDothinStreamProperties::SetFPS(XnInt* fps)
{
	XnStatus status = XN_STATUS_OK;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->CommandSetProperty(OBC_FPS, fps, sizeof(XnInt));
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnDothinStreamProperties::~XnDothinStreamProperties()
{
    if (nullptr != m_pSensor)
    {
        delete m_pSensor;
    }
}
