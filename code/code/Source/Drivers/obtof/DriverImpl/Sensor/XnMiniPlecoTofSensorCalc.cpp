#include "OpenNI.h"
#include "OniCTypes.h"
#include "XnMiniPlecoTofSensorCalc.h"

//const XnChar *kConfigFilePathAndName = ".\\config\\filter_config_pleco.ini";
//const XnChar *kCalibFilePathAndNameDualShuffle = ".\\config\\DepthParams.bin";
//const XnChar *kCalibFilePathAndNameSingleShuffle = "..\\..\\Config\\DepthCalc\\DepthParams_pleco\\single_shuffle\\DepthParams.bin";


XnMiniPlecoTofSensorCalc::XnMiniPlecoTofSensorCalc(mx6x_module_t *mx6xModule) :
	XnMiniPlecoTofSensor(mx6xModule)
{
	//m_configFilePathAndName = kConfigFilePathAndName;
	//m_calibFilePathAndName = kCalibFilePathAndNameDualShuffle;
}


XnMiniPlecoTofSensorCalc::~XnMiniPlecoTofSensorCalc()
{
	m_configFilePathAndName = nullptr;
	m_calibFilePathAndName = nullptr;
}


XnStatus XnMiniPlecoTofSensorCalc::UpdateFrameInfo(OniFrame *frame)
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
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16) + frame->width*frame->extraLine*sizeof(XnUInt16);
		xnLogInfo(XN_MASK_TOF_SENSOR, "XnMiniPlecoTofSensorCalc updateFrameInfo sensor ir/depth width=%d,height=%d\n", frame->videoMode.resolutionX, frame->videoMode.resolutionY);
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
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16) + frame->stride * m_pFrameGroup->real_count;
		OniMetadata *pMetadata = nullptr;
		for (XnInt i = 0; i < frame->extraLine; ++i)
		{
			pMetadata = static_cast<OniMetadata*>(frame->data) + frame->stride * i;
			SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
		}
	}


	return XN_STATUS_OK;
}

XnStatus XnMiniPlecoTofSensorCalc::CalcSensorProperty()
{
	XnMiniPlecoTofSensor::CalcSensorProperty();

	m_depthWidth = m_phaseWidth / 3;
	m_depthHeight = m_phaseHeight;
//	m_depthWidth = kVGADepthWidth / m_binningFactor;
//	m_depthHeight = kVGADepthHeight / m_binningFactor;
	xnLogInfo(XN_MASK_TOF_SENSOR_CALC, "XnPlecoTofSensorCalc engine CalcSensorProperty, m_depthWidth %d, m_depthHeight %d", m_depthWidth, m_depthHeight);
	return XN_STATUS_OK;
}

XnStatus XnMiniPlecoTofSensorCalc::UpdateMode()
{
	XnStatus nRet = XN_STATUS_OK;

	nRet = XnMiniPlecoTofSensor::UpdateMode();
	XN_IS_STATUS_OK(nRet);
	return XN_STATUS_OK;
}

XnStatus XnMiniPlecoTofSensorCalc::UpdateCalcConfig()
{

	//sensor模式改变后，传入深度计算接口的buffer大小需要更新
	if (m_phasePixelNum != m_phaseWidth * m_phaseHeight * m_phaseCount)
	{
		m_phasePixelNum = m_phaseWidth * m_phaseHeight * m_phaseCount;
		delete[] m_phaseBuf;
		m_phaseBuf = nullptr;
		m_phaseBuf = new XnUInt16[m_phasePixelNum];
	}

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
			m_phaseFrameIndex = { 0, 1, 2, 3, 4, 5,6,7,8,9,10,11,12 };
			//m_phaseFrameIndex = { 0, 1, 2, 3, 4, 5 };
		}
		else
		{
			//	m_phaseFrameIndex = { 3, 4, 5, 0, 1, 2 };
			m_phaseFrameIndex = { 3, 4, 5, 0, 1, 2,9,10,11,6,7,8 };
		}
	}
	return XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensorCalc::GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize)
{
	return this->XnTofSensor::GetOutputbufferSize(outputBufferWidth, outputBufferHeight, outputBufferSize);
	//*outputBufferWidth = m_depthWidth;
	// *outputBufferHeight = m_depthHeight + m_phaseCount;//每帧对应一行扩展信息
	//*outputBufferSize = *outputBufferWidth * *outputBufferHeight * sizeof(XnUInt16);

	//return  XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensorCalc::GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize)
{
	*outputImageWidth = m_depthWidth;
	*outputImageHeight = m_depthHeight;
	*outputImageSize = *outputImageWidth * *outputImageHeight * sizeof(XnUInt16);

	return  XN_STATUS_OK;
}
int XnMiniPlecoTofSensorCalc::getWidth() {
	return m_depthWidth;
	//return kVGADepthWidth;
}
int XnMiniPlecoTofSensorCalc::getHeight() {
	return m_depthHeight;
	//return kVGADepthHeight;
}
