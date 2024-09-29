#include "OpenNI.h"
#include "OniCTypes.h"
#include "XnMFTofSensorCalc.h"

//const XnChar *kConfigFilePathAndName = ".\\config\\filter_config_pleco.ini";
//const XnChar *kCalibFilePathAndNameDualShuffle = ".\\config\\DepthParams.bin";
//const XnChar *kCalibFilePathAndNameSingleShuffle = "..\\..\\Config\\DepthCalc\\DepthParams_pleco\\single_shuffle\\DepthParams.bin";

XnMFTofSensorCalc::XnMFTofSensorCalc(mx6x_module_t *mx6xModule) :
    XnMFTofSensor(mx6xModule)
{
    //m_configFilePathAndName = kConfigFilePathAndName;
    //m_calibFilePathAndName = kCalibFilePathAndNameDualShuffle;
	int maxSize = 640 * 480 ;
	m_phaseBuf = new XnInt16[maxSize];
	memset(m_phaseBuf, 0, maxSize);
}

XnMFTofSensorCalc::~XnMFTofSensorCalc() 
{
    m_configFilePathAndName = nullptr;
    m_calibFilePathAndName = nullptr;
}

XnStatus XnMFTofSensorCalc::UpdateFrameInfo(OniFrame *frame)
{

	if (frame->sensorType == ONI_SENSOR_AI)
	{
		frame->frameIndex = m_pFrameGroup->frames[0]->group_index;

		frame->extraLine = m_pFrameGroup->real_count;

		frame->width = frame->width;
		frame->stride = frame->width * sizeof(XnUInt16);
		frame->height = frame->height;
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16);
		OniAIFrame* pAIFrame = (OniAIFrame*)frame->data;
        pAIFrame->frameSet.size = 1;
        pAIFrame->frameSet.status = ONI_AI_STATUS_OK;
		OniTOFFrame* phase = &pAIFrame->frameSet.frames[0];
		phase->width = aiPhaseWidth;
		phase->height = aiPhaseHeight;
		phase->stride = phase->width * sizeof(XnUInt16);
		phase->planeNum = m_pFrameGroup->real_count;
		for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
		{
			OniMetadata* pMetadata = &phase->meta[i];
			//pMetadata = static_cast<XnUChar*>(frame.data) + frame.stride * i;
			SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
		}
	}
	else if (frame->sensorType == ONI_SENSOR_IR || frame->sensorType == ONI_SENSOR_DEPTH)
	{
		frame->extraLine = 1;
		frame->frameIndex = m_pFrameGroup->frames[0]->group_index;
		frame->width = frame->videoMode.resolutionX;
		frame->stride = frame->width * sizeof(XnUInt16);
		frame->height = frame->videoMode.resolutionY;
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16)+frame->width*frame->extraLine*sizeof(XnUInt16);

		OniMetadata *pMetadata = nullptr;
		for (XnInt i = 0; i < frame->extraLine; ++i)
		{
			pMetadata = static_cast<OniMetadata*>(frame->data) + frame->stride * i;
			SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
			pMetadata->width = frame->width;
			pMetadata->height = frame->height;
		}
	}
	else
	{
		frame->frameIndex = m_pFrameGroup->frames[0]->group_index;
		
		frame->width = m_depthWidth;
		frame->stride = frame->width * sizeof(XnUInt16);
		frame->height = m_depthHeight;
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16)+frame->stride * m_pFrameGroup->real_count;
		OniMetadata *pMetadata = nullptr;
		for (XnInt i = 0; i < frame->extraLine; ++i)
		{
			pMetadata = static_cast<OniMetadata*>(frame->data) + frame->stride * i;
			SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
		}
	}

    return XN_STATUS_OK;
}

XnStatus XnMFTofSensorCalc::CalcSensorProperty()
{
	XnMFTofSensor::CalcSensorProperty();

    m_depthWidth = kVGADepthWidth / m_binningFactor;
    m_depthHeight = kVGADepthHeight / m_binningFactor;
    xnLogInfo(XN_MASK_MF_TOF_SENSOR_CALC, "engine CalcSensorProperty, m_depthWidth %d, m_depthHeight %d", m_depthWidth, m_depthHeight);
    return XN_STATUS_OK;
}

XnStatus XnMFTofSensorCalc::UpdateMode()
{
	XnStatus nRet = XN_STATUS_OK;

	nRet = XnMFTofSensor::UpdateMode();
	XN_IS_STATUS_OK(nRet);
#if 0
	switch (m_binningFactor)
	{
	case  1 : // TOF_BINNING_MODE_1X1
		m_modeConfig.height = 480;
		m_modeConfig.width = 640;
		break;
	case 2:// TOF_BINNING_MODE_2X2:
		m_modeConfig.height = 240;
		m_modeConfig.width = 320;
		break;
	case 4:// TOF_BINNING_MODE_4X4:
		m_modeConfig.height = 120;
		m_modeConfig.width = 160;
		break;
	default:
		xnLogError(XN_MASK_MF_TOF_SENSOR_CALC, "bad case m_binningFactor %d", m_binningFactor);
		return -1;
		break;
	}
#endif
	return XN_STATUS_OK;
}

XnStatus XnMFTofSensorCalc::UpdateCalcConfig()
{

	if (m_sensorId == TOF_SENSOR_ID_PLECO)
	{
		//pleco传入的相位顺序为[0, 240, 120, 0, 240, 120]
		if (TOF_SINGLE_FREQUENCY_MODE == m_freqMode)
		{
			m_phaseFrameIndex = { 0, 1, 2 };
		}
		else
		{
			// 深度引擎v3.1.2 目前会把3ns，5ns  其中3ns视作低频，按数值大小进行得处理。
			if (m_pFrameGroup->frames[0]->frequency < m_pFrameGroup->frames[3]->frequency)
			{
				m_phaseFrameIndex = { 0, 1, 2, 3, 4, 5 };
			}
			else
			{
				m_phaseFrameIndex = { 3, 4, 5, 0, 1, 2 };
			}
		}
	}
	else if (m_sensorId == TOF_SENSOR_ID_MLX75027)
	{
		if (TOF_SINGLE_FREQUENCY_MODE == m_freqMode)
		{
			m_phaseFrameIndex = { 0, 1, 2, 3 };
		}
		else
		{
			if (m_pFrameGroup->frames[0]->frequency < m_pFrameGroup->frames[3]->frequency)
			{
				m_phaseFrameIndex = { 0, 1, 2, 3};
			}
			else
			{
				m_phaseFrameIndex = { 0, 1, 2, 3 };
			}
		}
	}

    return XN_STATUS_OK;
}

XnStatus XnMFTofSensorCalc::GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize)
{
	return this->XnTofSensor::GetOutputbufferSize(outputBufferWidth, outputBufferHeight, outputBufferSize);
    //*outputBufferWidth = m_depthWidth;
   // *outputBufferHeight = m_depthHeight + m_phaseCount;//每帧对应一行扩展信息
    //*outputBufferSize = *outputBufferWidth * *outputBufferHeight * sizeof(XnUInt16);

    //return  XN_STATUS_OK;
}

XnStatus XnMFTofSensorCalc::GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize)
{
    *outputImageWidth = m_depthWidth;
    *outputImageHeight = m_depthHeight;
    *outputImageSize = *outputImageWidth * *outputImageHeight * sizeof(XnUInt16);

    return  XN_STATUS_OK;
}
int XnMFTofSensorCalc::getWidth(){
	return kVGADepthWidth;
}
int XnMFTofSensorCalc::getHeight(){
	return kVGADepthHeight;
}
