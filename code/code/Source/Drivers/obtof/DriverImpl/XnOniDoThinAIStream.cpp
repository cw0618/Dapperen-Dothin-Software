#include "XnOniDothinAIStream.h"


XnOniDothinAIStream::XnOniDothinAIStream(XnOniDothinDevice* pDevice) :XnOniDothinStream(XN_STREAM_TYPE_AI, ONI_SENSOR_AI, pDevice)
{
    XnStatus status;
    XnUInt32 sensorId;
    XnInt size = sizeof(sensorId);
    if (nullptr == m_pDevice)
    {
        xnLogError(XN_MASK_DOTHIN_AI_STREAM, "m_pDevice is nullptr");
    }
    else
    {
        status = m_pDevice->getProperty(SENSOR_ID, &sensorId, &size);
        if (XN_STATUS_OK != status)
        {
            xnLogError(XN_MASK_DOTHIN_AI_STREAM, "get sensor id fail");
        }
    }
	dtStreamProperties = XN_NEW(XnDothinAIStreamProperties, XN_STREAM_TYPE_AI, ONI_SENSOR_AI, sensorId);//如果sensorId不匹配，在Init()时会失败
	dtDataProcessor = XN_NEW(XnDothinAIProcessor);

    xnLogInfo(XN_MASK_DOTHIN_AI_STREAM, "construct XnOniDothinAIStream, sensorId %x", sensorId);
}


OniStatus XnOniDothinAIStream::Init()
{
	OniStatus status = ONI_STATUS_OK;

	status = XnOniDothinStream::Init();
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	return status;
}


OniStatus XnOniDothinAIStream::start()
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


void XnOniDothinAIStream::stop()
{
	if (m_bIsStart)
	{
		m_bIsStart = FALSE;
		SetActualRead(FALSE);
		SetStreamMode(OBC_STREAM_DISABLED);
		XnOniDothinStream::stop();
	}
}


OniStatus XnOniDothinAIStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	return XnOniDothinStream::getProperty(propertyId, data, pDataSize);
}


OniStatus XnOniDothinAIStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{

	return XnOniDothinStream::setProperty(propertyId, data, dataSize);
}


OniBool XnOniDothinAIStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}


XnInt XnOniDothinAIStream::getRequiredFrameSize()
{
	return XnOniDothinStream::getRequiredFrameSize();
}


XnOniDothinAIStream::~XnOniDothinAIStream()
{
    XN_DELETE(dtStreamProperties);
	XN_DELETE(dtDataProcessor);
}
