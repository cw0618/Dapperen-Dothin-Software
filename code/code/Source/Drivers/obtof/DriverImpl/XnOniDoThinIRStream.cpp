#include "XnOniDothinIRStream.h"


XnOniDothinIRStream::XnOniDothinIRStream(XnOniDothinDevice* pDevice) :XnOniDothinStream(XN_STREAM_TYPE_IR, ONI_SENSOR_IR, pDevice)
{
	XnStatus status;
	XnUInt32 sensorId;
	XnInt size = sizeof(sensorId);
	if (nullptr == m_pDevice)
	{
		xnLogError(XN_MASK_DOTHIN_IR_STREAM, "m_pDevice is nullptr");
	}
	else
	{
		status = m_pDevice->getProperty(SENSOR_ID, &sensorId, &size);
		if (XN_STATUS_OK != status)
		{
			xnLogError(XN_MASK_DOTHIN_IR_STREAM, "get sensor id fail");
		}
	}
	dtStreamProperties = XN_NEW(XnDothinIRStreamProperties, XN_STREAM_TYPE_IR, ONI_SENSOR_IR, sensorId);
	dtDataProcessor = XN_NEW(XnDothinIRProcessor);
}

OniStatus XnOniDothinIRStream::Init()
{
	OniStatus status = ONI_STATUS_OK;

	status = XnOniDothinStream::Init();
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	return status;
}


OniStatus XnOniDothinIRStream::start()
{
	OniStatus status = ONI_STATUS_OK;

	if (!m_bIsStart)
	{
		//检测sensor模式， 更新sensor对应属性
		dtStreamProperties->UpdateMode();
		status = XnOniDothinStream::start();
		if (status != ONI_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		status = SetStreamMode(OBC_STREAM_IR_LASER);
		if (status != ONI_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		status = SetActualRead(TRUE);
		m_bIsStart = TRUE;
	}

	return status;
}


void XnOniDothinIRStream::stop()
{
	if (m_bIsStart)
	{
		m_bIsStart = FALSE;
		SetActualRead(FALSE);
		SetStreamMode(OBC_STREAM_DISABLED);
		XnOniDothinStream::stop();
	}
}


OniStatus XnOniDothinIRStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	return XnOniDothinStream::getProperty(propertyId, data, pDataSize);
}


OniStatus XnOniDothinIRStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	return XnOniDothinStream::setProperty(propertyId, data, dataSize);
}


OniBool XnOniDothinIRStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}


XnInt XnOniDothinIRStream::getRequiredFrameSize()
{
	return XnOniDothinStream::getRequiredFrameSize();
}

XnOniDothinIRStream::~XnOniDothinIRStream()
{
	XN_DELETE(dtStreamProperties);
	XN_DELETE(dtDataProcessor);
}
