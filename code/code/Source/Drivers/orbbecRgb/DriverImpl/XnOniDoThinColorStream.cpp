#include "XnOniDothinColorStream.h"

XnOniDothinColorStream::XnOniDothinColorStream(XnOniDothinDevice* pDevice) :XnOniDothinStream(XN_STREAM_TYPE_IMAGE, ONI_SENSOR_COLOR, pDevice)
{
	dtStreamProperties = XN_NEW(XnDothinStreamProperties, XN_STREAM_TYPE_IMAGE, ONI_SENSOR_COLOR);
	dtDataProcessor = XN_NEW(XnDothinColorProcessor);
}


OniStatus XnOniDothinColorStream::Init()
{
	OniStatus status = XnOniDothinStream::Init();
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}
	return ONI_STATUS_OK;
}


OniStatus XnOniDothinColorStream::start()
{
	OniStatus status = ONI_STATUS_OK;

	if (!m_bIsStart)
	{
		status = XnOniDothinStream::start();
		if (status != ONI_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		status = SetStreamMode(OBC_STREAM_COLOR);
		if (status != ONI_STATUS_OK)
		{
			return ONI_STATUS_ERROR;
		}

		status = SetActualRead(TRUE);
		m_bIsStart = TRUE;
	}

	return status;
}


void XnOniDothinColorStream::stop()
{
	if (m_bIsStart)
	{
		m_bIsStart = FALSE;
		SetActualRead(FALSE);
		SetStreamMode(OBC_STREAM_DISABLED);
		XnOniDothinStream::stop();
	}
}


OniStatus XnOniDothinColorStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	return XnOniDothinStream::getProperty(propertyId, data, pDataSize);
}


OniStatus XnOniDothinColorStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	return XnOniDothinStream::setProperty(propertyId, data, dataSize);
}


OniBool XnOniDothinColorStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}


XnInt XnOniDothinColorStream::getRequiredFrameSize()
{
	return XnOniDothinStream::getRequiredFrameSize();
}


XnOniDothinColorStream::~XnOniDothinColorStream()
{
	XN_DELETE(dtDataProcessor);
}