#include "XnOniDothinStream.h"


XnOniDothinStream::XnOniDothinStream(XnChar* m_strType, OniSensorType m_sensorType, XnOniDothinDevice* pDevice)
	:m_strType(m_strType),
	m_sensorType(m_sensorType),
	m_pDevice(pDevice),
	m_bIsStart(FALSE)
{
	dothinStreamImple = XN_NEW(XnDothinStreamImple);
}


OniStatus XnOniDothinStream::Init()
{
	OniStatus status = ONI_STATUS_OK;

	if (m_pDevice == NULL)
	{
		return ONI_STATUS_ERROR;
	}

	XnMx6xModulesHelper* mx6xModulesHelper = m_pDevice->getMx6xModulesHelper();
	dtConnectHelper = m_pDevice->getDothinConnectHelper();
	if (mx6xModulesHelper == NULL || dtConnectHelper == NULL)
	{
		return ONI_STATUS_ERROR;
	}

	XnStatus rc = dtStreamProperties->Init(mx6xModulesHelper, dtConnectHelper);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

    dtDataProcessor->setDeviceProperties(dtStreamProperties);
	dtDataProcessor->Init();

	OniVideoMode pVideoMode;
	xnOSMemSet(&pVideoMode, 0, sizeof(OniVideoMode));
	status = m_pDevice->getDefaultVideoMode(&pVideoMode, m_sensorType);
	if (status != ONI_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}
	dtStreamProperties->SetDefaultVideoMode(&pVideoMode);

	return ONI_STATUS_OK;
}


OniStatus XnOniDothinStream::start()
{
	XnUInt32 inputFormat = dtStreamProperties->GetSensorInputFormat();
	XnUInt32 XRes = dtStreamProperties->GetXRes();
	XnUInt32 YRes = dtStreamProperties->GetYRes();

	XnStatus rc = dtConnectHelper->Restart(XRes, YRes, &inputFormat);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}
	return ONI_STATUS_OK;
}


void XnOniDothinStream::stop()
{

}


void XnOniDothinStream::setServices(oni::driver::StreamServices* pStreamServices)
{
	oni::driver::StreamBase::setServices(pStreamServices);
	dtDataProcessor->setServices(pStreamServices);
	this->m_pServices = pStreamServices;
}


OniStatus XnOniDothinStream::SetVideoMode(OniVideoMode* pVideoMode)
{
    XnStatus status = XN_STATUS_OK;
	status = dtStreamProperties->ChangeVideoMode(pVideoMode);
	if (status != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}
	return ONI_STATUS_OK;
}

OniStatus XnOniDothinStream::GetVideoMode(OniVideoMode* pVideoMode)
{
	dtStreamProperties->GetVideoMode(pVideoMode);
	return ONI_STATUS_OK;
}


OniStatus XnOniDothinStream::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnStatus nRetVal = XN_STATUS_ERROR;
	switch (propertyId)
	{
	case ONI_STREAM_PROPERTY_VIDEO_MODE:
		if (*pDataSize != sizeof(OniVideoMode))
		{
			xnLogError(XN_MASK_DOTHIN_STREAM, "Unexpected size: %d != %d", *pDataSize, sizeof(OniVideoMode));
			return ONI_STATUS_ERROR;
		}

		nRetVal = GetVideoMode((OniVideoMode*)data);
		XN_IS_STATUS_OK_RET(nRetVal, ONI_STATUS_ERROR);
		break;
	case ONI_STREAM_PROPERTY_MIRRORING:

		break;
	default:
		break;
	}

	return ONI_STATUS_OK;
}


OniStatus XnOniDothinStream::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	if (m_bIsStart)//开流时，会设置失败
	{
		xnLogError(XN_MASK_DOTHIN_STREAM, "%s stream is running...", m_strType);
		return ONI_STATUS_OUT_OF_FLOW;
	}

	switch (propertyId)
	{
	case ONI_STREAM_PROPERTY_VIDEO_MODE:
		if (dataSize != sizeof(OniVideoMode))
		{
			xnLogError(XN_MASK_DOTHIN_STREAM, "Unexpected size: %d != %d", dataSize, sizeof(OniVideoMode));
			return ONI_STATUS_BAD_PARAMETER;
		}

		if (m_pDevice == NULL)
		{
			return ONI_STATUS_ERROR;
		}

		if (!m_pDevice->isVideoModeSupport(*(OniVideoMode*)data, m_sensorType))
		{
			return ONI_STATUS_NOT_SUPPORTED;
		}

		OniVideoMode videoMode;
		GetVideoMode(&videoMode);
		if (!xnOSMemCmp(&videoMode, data, sizeof(OniVideoMode)))
		{
			return (ONI_STATUS_OK);
		}

		return SetVideoMode((OniVideoMode*)data);
		break;
	default:
		break;
	}

	return ONI_STATUS_OK;
}


OniBool XnOniDothinStream::isPropertySupported(XnInt propertyId)
{
	return ONI_STATUS_OK;
}


XnInt XnOniDothinStream::getRequiredFrameSize()
{
    return dtStreamProperties->GetRequiredFrameSize();
}


OniStatus XnOniDothinStream::SetStreamMode(obc_stream_type streamType)
{
	if (dtStreamProperties == NULL)
	{
		return ONI_STATUS_ERROR;
	}

	XnStatus rc = dtStreamProperties->StreamMode(streamType);
	if (rc != XN_STATUS_OK)
	{
		return ONI_STATUS_ERROR;
	}

	return ONI_STATUS_OK;
}


OniStatus XnOniDothinStream::SetActualRead(XnBool bRead)
{
	if (dothinStreamImple == NULL || dtDataProcessor == NULL)
	{
		return ONI_STATUS_ERROR;
	}

	XnStatus nRetVal = XN_STATUS_OK;
	if (bRead)
	{
		dtDataProcessor->setNewFrameDataCallback(OnDotNewFrameCallback, this);
		nRetVal = dtDataProcessor->start();
		if (nRetVal != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTHIN_STREAM, "Dot data processor start  exception!");
			return ONI_STATUS_ERROR;
		}

		xnLogVerbose(XN_MASK_DOTHIN_STREAM, "Creating dothin %s read thread...", m_strType);
        nRetVal = dothinStreamImple->XnDothinInitReadThread(dtStreamProperties->GetGrabBufferSize(), 
            dtStreamProperties->GetRequiredFrameSize(),
            dtConnectHelper, dtDataProcessor);
		if (nRetVal != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTHIN_STREAM, "Dot read thread init exception!");
			return ONI_STATUS_ERROR;
		}
	}
	else
	{
		xnLogVerbose(XN_MASK_DOTHIN_STREAM, "Shutting down Dothin %s read thread...", m_strType);
		dtDataProcessor->setNewFrameDataCallback(NULL, this);

        dothinStreamImple->XnDothinShutDownThread();

        //stop data processor after shutting down thread of reading data
		dtDataProcessor->stop();
	}

	return ONI_STATUS_OK;
}


void XN_CALLBACK_TYPE XnOniDothinStream::OnDotNewFrameCallback(OniFrame* pFrame, void* pCookie)
{
	
	XnOniDothinStream* dtStream = (XnOniDothinStream*)pCookie;
	dtStream->raiseNewDataFrame(pFrame);
}


void XnOniDothinStream::raiseNewDataFrame(OniFrame* pFrame)
{
	if (m_bIsStart && (pFrame->sensorType == m_sensorType))
	{
		raiseNewFrame(pFrame);
	}
}



XnOniDothinStream::~XnOniDothinStream()
{
	XN_DELETE(dothinStreamImple);
	xnLogVerbose(XN_MASK_DOTHIN_STREAM, "Dothin stream destroy...");
}
