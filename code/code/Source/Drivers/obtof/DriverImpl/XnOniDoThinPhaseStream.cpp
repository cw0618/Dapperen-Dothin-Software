#include "XnOniDothinPhaseStream.h"


XnOniDothinPhaseStream::XnOniDothinPhaseStream(XnOniDothinDevice* pDevice) :XnOniDothinStream(XN_STREAM_TYPE_PHASE, ONI_SENSOR_PHASE, pDevice)
{
    XnStatus status;
    XnUInt32 sensorId;
    XnInt size = sizeof(sensorId);
    if (nullptr == m_pDevice)
    {
        xnLogError(XN_MASK_DOTHIN_PHASE_STREAM, "m_pDevice is nullptr");
    }
    else
    {
        status = m_pDevice->getProperty(SENSOR_ID, &sensorId, &size);
        if (XN_STATUS_OK != status)
        {
            xnLogError(XN_MASK_DOTHIN_PHASE_STREAM, "get sensor id fail");
        }
    }
	dtStreamProperties = XN_NEW(XnDothinPhaseStreamProperties, XN_STREAM_TYPE_PHASE, ONI_SENSOR_PHASE, sensorId);//如果sensorId不匹配，在Init()时会失败
	dtDataProcessor = XN_NEW(XnDothinPhaseProcessor);

    xnLogInfo(XN_MASK_DOTHIN_PHASE_STREAM, "construct XnOniDothinPhaseStream, sensorId %x", sensorId);
}


OniStatus XnOniDothinPhaseStream::Init()
{
	OniStatus status = ONI_STATUS_OK;

	status = XnOniDothinStream::Init();
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	return status;
}


OniStatus XnOniDothinPhaseStream::start()
{
	OniStatus status = ONI_STATUS_OK;

	if (!m_bIsStart)
	{
        //检测sensor模式， 更新sensor对应属性
        dtStreamProperties->UpdateMode();

		status = XnOniDothinStream::start();
        XN_IS_STATUS_OK(status);

        status = SetStreamMode(OBC_STREAM_TOF_PHASE);
        XN_IS_STATUS_OK(status);

		status = SetActualRead(TRUE);
		m_bIsStart = TRUE;
	}

	return status;
}


void XnOniDothinPhaseStream::stop()
{
	if (m_bIsStart)
	{
		m_bIsStart = FALSE;
		SetActualRead(FALSE);
		SetStreamMode(OBC_STREAM_DISABLED);
		XnOniDothinStream::stop();
	}
}


OniStatus XnOniDothinPhaseStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	switch (propertyId)
	{
	case OBEXTENSION_ID_UPDATE_FRAMEMODE:
		dtStreamProperties->UpdateMode();
		break;
	default:
		XnOniDothinStream::getProperty(propertyId, data, pDataSize);
		break;
	}
	return ONI_STATUS_OK;
}


OniStatus XnOniDothinPhaseStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{

	return XnOniDothinStream::setProperty(propertyId, data, dataSize);
}


OniBool XnOniDothinPhaseStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}


XnInt XnOniDothinPhaseStream::getRequiredFrameSize()
{
	return XnOniDothinStream::getRequiredFrameSize();
}


XnOniDothinPhaseStream::~XnOniDothinPhaseStream()
{
    XN_DELETE(dtStreamProperties);
	XN_DELETE(dtDataProcessor);
}
