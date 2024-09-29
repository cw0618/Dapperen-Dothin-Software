#include "XnOniDothinDepthStream.h"
#include "DDK/XnDothinDepthStreamProperty.h"

XnOniDothinDepthStream::XnOniDothinDepthStream(XnOniDothinDevice* pDevice) :XnOniDothinStream(XN_STREAM_TYPE_DEPTH, ONI_SENSOR_DEPTH, pDevice)
{
    XnStatus status;
    XnUInt32 sensorId;
    XnInt size = sizeof(sensorId);
    if (nullptr == m_pDevice)
    {
        xnLogError(XN_MASK_DOTHIN_DEPTH_STREAM, "m_pDevice is nullptr");
    }
    else
    {
        status = m_pDevice->getProperty(SENSOR_ID, &sensorId, &size);
        if (XN_STATUS_OK != status)
        {
            xnLogError(XN_MASK_DOTHIN_DEPTH_STREAM, "get sensor id fail");
        }
    }
	dtStreamProperties = XN_NEW(XnDothinDepthStreamProperty, XN_STREAM_TYPE_DEPTH, ONI_SENSOR_DEPTH, sensorId);
	dtDataProcessor = XN_NEW(XnDothinDepthProcessor);
}

OniStatus XnOniDothinDepthStream::Init()
{
	OniStatus status = ONI_STATUS_OK;

	status = XnOniDothinStream::Init();
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	return status;
}

OniStatus XnOniDothinDepthStream::start()
{
	OniStatus status = ONI_STATUS_OK;

	if (!m_bIsStart)
	{
		//检测sensor模式， 更新sensor对应属性
		dtStreamProperties->UpdateMode();
        status = XnOniDothinStream::start();
        XN_IS_STATUS_OK(status);

		status = SetStreamMode(OBC_STREAM_DEPTH);
        XN_IS_STATUS_OK(status);

		status = SetActualRead(TRUE);
		m_bIsStart = TRUE;
	}

	return status;
}

void XnOniDothinDepthStream::stop()
{
	if (m_bIsStart)
	{
		m_bIsStart = FALSE;
		SetActualRead(FALSE);
		SetStreamMode(OBC_STREAM_DISABLED);
		XnOniDothinStream::stop();
	}
}

OniStatus XnOniDothinDepthStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	switch (propertyId)
	{
	case OBEXTENSION_ID_CAM_PARAMS:
		if (*pDataSize == sizeof(OBCameraParams))
		{
			XnTofSensor *m_pSensor = dtStreamProperties->GetSensor();
			OBCameraParams *cmameraParams = (OBCameraParams*)data;
			m_pSensor->getCameraParams(cmameraParams);
		}
		break;
	default:
		break;
	}
	return XnOniDothinStream::getProperty(propertyId, data, pDataSize);
}

OniStatus XnOniDothinDepthStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	switch (propertyId)
	{
	case ONI_STREAM_PROPERTY_LOAD_FILE:
		if (dataSize == sizeof(OniFileAttributes))
		{
			OniFileAttributes* fileAttributes=(OniFileAttributes*)data;
			OniFileCategory category = fileAttributes->category;
			XnTofSensor *m_pSensor = dtStreamProperties->GetSensor();
			XnChar *filePath = (XnChar*)fileAttributes->path;
			if (category == ONI_FILE_CATEGORY_FILTER)
			{
				m_pSensor->LoadFilterConfig(filePath);
			}
			else if (category == ONI_FILE_CATEGORY_CALIB)
			{
				m_pSensor->LoadCaliParam(filePath);
			}

			return ONI_STATUS_OK;
		}
		break;
	default:
		break;
	}
	return XnOniDothinStream::setProperty(propertyId, data, dataSize);
}

OniBool XnOniDothinDepthStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}

XnInt XnOniDothinDepthStream::getRequiredFrameSize()
{
	return XnOniDothinStream::getRequiredFrameSize();
}

XnOniDothinDepthStream::~XnOniDothinDepthStream()
{
    XN_DELETE(dtStreamProperties);
	XN_DELETE(dtDataProcessor);
}
