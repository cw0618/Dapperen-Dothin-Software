#include "XnDothinPhaseStreamProperties.h"
#include "OBCProperties.h"
#include "OpenNI.h"
#include"SensorType.h"

XnDothinPhaseStreamProperties::XnDothinPhaseStreamProperties(XnChar* strType, OniSensorType sensorType, XnUInt32 sensorId) : 
    XnDothinStreamProperties(strType, sensorType),
    m_sensorId(sensorId)
{

}


XnStatus XnDothinPhaseStreamProperties::Init(XnMx6xModulesHelper* mudulesHelper, XnDothinConnectHelper* dtConnectHelper)
{
	xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Dothin stream properties init start...");

	XN_VALIDATE_INPUT_PTR(mudulesHelper);
	m_pMudulesHelper = mudulesHelper;

	XN_VALIDATE_INPUT_PTR(dtConnectHelper);
	m_pConnectHelper = dtConnectHelper;

    switch (m_sensorId)
    {
    case TOF_SENSOR_ID_PLECO:
        m_pSensor = new XnPlecoTofSensor(mudulesHelper->getMx6xModulesHandle());
        break;
	case TOF_SENSOR_ID_MLX75027:
		m_pSensor = new XnMFTofSensor(mudulesHelper->getMx6xModulesHandle());
		break;
	case TOF_SENSOR_ID_GAEA:
		m_pSensor = new XnGaeaSensor(mudulesHelper->getMx6xModulesHandle());
		break;
    default:
        xnLogError(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Sensor is not supported, sensorId: %x", m_sensorId);
        return XN_STATUS_ERROR;
        break;
    }

    m_dothinMipiPackBit = m_pSensor->GetDothinMipiPackBit();
    xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Sensor dothinMipiPackBit is %d", m_dothinMipiPackBit);
    xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Dothin phase stream properties init end...");
	return XN_STATUS_OK;
}


void XnDothinPhaseStreamProperties::SetSensorInputFormat(const XnUInt32 inputFormat)
{
    UNREFERENCED_PARAMETER(inputFormat);
    xnLogError(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Unable to change sensor inputFormat, bitPerPhasePixel is decided by tof sensor");
}


XnStatus XnDothinPhaseStreamProperties::SetDefaultVideoMode(OniVideoMode* pVideoMode)
{
	xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "XnDothinPhaseStreamProperties SetDefaultVideoMode...");
	XN_VALIDATE_INPUT_PTR(pVideoMode);

    xnOSMemCopy(&m_defaultVideoMode, pVideoMode, sizeof(OniVideoMode));
	xnOSMemCopy(&m_curVideoMode, pVideoMode, sizeof(OniVideoMode));

	XnStatus nRetVal = ONI_STATUS_OK;

	m_outPixelFormat = m_curVideoMode.pixelFormat;

    m_pSensor->GetGrabBufferSize (&XRes, &YRes, &m_grabBufferSize);

    nRetVal = OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);

    CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);
	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
	CalcStride(XRes, m_bytesPerPixel);

	return XN_STATUS_OK;
}


XnStatus XnDothinPhaseStreamProperties::ChangeVideoMode(OniVideoMode* pVideoMode)
{
    xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "XnDothinPhaseStreamProperties ChangeVideoMode..."); 
    XN_VALIDATE_INPUT_PTR(pVideoMode);
    XnStatus nRetVal = XN_STATUS_OK;

    m_curVideoMode.pixelFormat = pVideoMode->pixelFormat;
	XRes = m_curVideoMode.resolutionX = pVideoMode->resolutionX;
	YRes = m_curVideoMode.resolutionY = pVideoMode->resolutionY;

	if (m_curVideoMode.fps != pVideoMode->fps)
	{
		nRetVal = SetFPS(&pVideoMode->fps);
		if (nRetVal == XN_STATUS_OK)
		{
			m_curVideoMode.fps = pVideoMode->fps;
		}
	}

    m_outPixelFormat = m_curVideoMode.pixelFormat;
	m_pSensor->setFrameResolution(pVideoMode->resolutionX, pVideoMode->resolutionY);
    m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);

	OnOutputFormatChanged(m_outPixelFormat);
	XN_IS_STATUS_OK(nRetVal);

	CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
	CalcStride(XRes, m_bytesPerPixel);
    CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);

	return XN_STATUS_OK;
}

XnStatus XnDothinPhaseStreamProperties::GetVideoMode(OniVideoMode* pVideoMode)
{
	xnOSMemCopy(pVideoMode, &m_curVideoMode, sizeof(OniVideoMode));
	return XN_STATUS_OK;
}

XnStatus XnDothinPhaseStreamProperties::StreamMode(obc_stream_type streamType)
{
    obc_videomode mode;//宽和高为无关参数
    mode.width = 0;
    mode.height = 0;
    mode.type = streamType;

    XnStatus status = XN_STATUS_OK;
    XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
    status = m_pMudulesHelper->CommandSetProperty(OBC_STREAM_MODE, &mode, sizeof(obc_videomode));

    return status;
}


XnStatus XnDothinPhaseStreamProperties::UpdateMode()
{
    XnStatus status = XN_STATUS_OK;

    status = m_pSensor->UpdateMode();
    XN_IS_STATUS_OK(status);
    m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);

    CalcGrabBufferSize(XRes, YRes, m_dothinMipiPackBit);
    CalcStride(XRes, m_bytesPerPixel);
    CalcRequiredSize(XRes, YRes, m_bytesPerPixel);
}


void XnDothinPhaseStreamProperties::CalcGrabBufferSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 InputFormat)
{
    UNREFERENCED_PARAMETER(InputFormat);

    m_pSensor->GetGrabBufferSize(&XRes, &YRes, &m_grabBufferSize);//这里的参数用来设置度信，与分辨率无关，是抓取的buffer的宽和高

    xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "GetGrabBufferSize is %d", m_grabBufferSize);
}


void XnDothinPhaseStreamProperties::CalcRequiredSize(XnUInt32 XRes, XnUInt32 YRes, XnUInt32 nBytesPerPixel)
{
    UNREFERENCED_PARAMETER(XRes);
    UNREFERENCED_PARAMETER(YRes);
    UNREFERENCED_PARAMETER(nBytesPerPixel);
    XnUInt32 outputBufferWidth = 0;
    XnUInt32 outputBufferHeight = 0;

    m_pSensor->GetOutputbufferSize(&outputBufferWidth, &outputBufferHeight, &m_requiredFrameSize);

	xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "m_requiredFrameSize %d", m_requiredFrameSize);
}


void XnDothinPhaseStreamProperties::CalcStride(XnUInt32 XRes, XnUInt32 nBytesPerPixel)
{
	stride = XRes * nBytesPerPixel;
	xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM_PPS, "Calc stride is %d", stride);
}


XnStatus XnDothinPhaseStreamProperties::SetFPS(XnInt* fps)
{
	XnStatus status = XN_STATUS_OK;
	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->CommandSetProperty(OBC_FPS, fps, sizeof(XnInt));

    return status;
}


XnDothinPhaseStreamProperties::~XnDothinPhaseStreamProperties()
{
}
