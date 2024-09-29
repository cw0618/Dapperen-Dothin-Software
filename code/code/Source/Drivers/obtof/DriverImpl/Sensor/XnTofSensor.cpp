// Copyright 2020 ORBBEC Technology., Inc.
// This file belongs to ORBBEC Technology., Inc.
// It is considered a trade secret, and is not to be divulged or used by
// parties who have NOT received written authorization from the owner.
// see https:
/**********************************************************************************************
***           C O N F I D E N T I A L  ---  O R B B E C   Technology                        ***
***********************************************************************************************
*                                                                                             *
*  @project   :
*  @file      :
*  @brief     :    tof sensor的各自处理的基类
*  @author    :    Xia Xue Guang && ZhuoWeifeng                                               *
*  @version   :    0.0.0.1                                                                    *
*  @date      :    2021.05.7                                                                  *
*  @update    :                                                                               *
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																		     	  *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#include "XnTofSensor.h"

XnTofSensor::XnTofSensor(mx6x_module_t* mx6xModule) :
m_sensorId(0),
m_binningFactor(1),
m_freqMode(TOF_DUAL_FREQUENCY_MODE),
m_phaseWidth(0),
m_phaseHeight(0),
m_extraLine(0),
m_bitPerPhasePixel(10),
m_grabWidth(0),
m_grabHeight(0),
m_dothinMipiPackBit(10),
m_configFilePathAndName(nullptr),
m_calibFilePathAndName(nullptr),
m_configParamBuf(nullptr),
m_phaseBuf(nullptr),
m_phasePixelNum(0),
m_phaseFrameIndex(0)
{
	m_pModule = mx6xModule;
	m_pCompressedFrameGroup = new ObFrameGroup;
	m_pFrameGroup = new ObFrameGroup;
	MallocFrameGroupBuffer(m_pCompressedFrameGroup, 16);
	MallocFrameGroupBuffer(m_pFrameGroup,16);

	m_calibParamBuf = nullptr;
	m_configParamBuf = new XnChar[kOutputParamBufSize];
	//memset(&m_modeConfig, 0, sizeof(Depth_Mode_Config));
	//memset(&m_outputConfig, 0, sizeof(Output_Config));

	m_depthWidth = kVGADepthWidth / m_binningFactor;
	m_depthHeight = kVGADepthHeight / m_binningFactor;
	m_depth = new XnFloat[kVGADepthWidth * kVGADepthHeight];
	m_amplitude = new XnFloat[kVGADepthWidth * kVGADepthHeight];
	m_intensity = new XnFloat[kVGADepthWidth * kVGADepthHeight];
	m_tmpBuf = new XnFloat[kVGADepthWidth * kVGADepthHeight * kMaxFrequencyNum];
}

XnTofSensor::~XnTofSensor()
{
	FreeFrameGroupBuffer(m_pCompressedFrameGroup);
	FreeFrameGroupBuffer(m_pFrameGroup);
#if USE_FAKE_TOF_DEPTH_LIB
	//ReleaseCaliParamsBuffer(m_calibParamBuf);
#else 
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine fake ReleaseCaliParamsBuffer");
#endif
	xnLogInfo(XN_MASK_TOF_SENSOR, "ReleaseCaliParamsBuffer over");
	delete[] m_configParamBuf;
	delete[] m_depth;
	delete[] m_amplitude;
	delete[] m_intensity;
	delete[] m_tmpBuf;
	delete[] m_phaseBuf;
}

XnStatus XnTofSensor::Init()
{
	return XN_STATUS_NOT_IMPLEMENTED;
};

XnStatus XnTofSensor::MallocFrameGroupBuffer(ObFrameGroup *pFrameGroup, int frameCount)
{
	if (nullptr == pFrameGroup)
	{
		return  XN_STATUS_NULL_INPUT_PTR;
	}

	//对于带rk1608的情况，用深度的宽和高计算内存空间会偏小，故这里用定值
	const int maxFrameBufferSize = 640 * 480 * 2 * 16;
	pFrameGroup->frame_count = frameCount;
	pFrameGroup->frames = new (std::nothrow)ObTofFrameInfo*[pFrameGroup->frame_count];

	for (XnUInt i = 0; i < pFrameGroup->frame_count; ++i)
	{
		pFrameGroup->frames[i] = new (std::nothrow)ObTofFrameInfo();
		if (nullptr == pFrameGroup->frames[i])
		{
			return  XN_STATUS_NULL_OUTPUT_PTR;
		}
		pFrameGroup->frames[i]->buffer = new (std::nothrow) char[maxFrameBufferSize];
		if (nullptr == pFrameGroup->frames[i]->buffer)
		{
			return  XN_STATUS_INVALID_BUFFER_SIZE;
		}
		if ((XnUInt32)(pFrameGroup->frames[i]->buffer) % 4 != 0)
		{
			return  XN_STATUS_NO_MATCH;
		}
		pFrameGroup->frames[i]->buffer_size = maxFrameBufferSize;
		pFrameGroup->frames[i]->data_offset = pFrameGroup->frames[i]->buffer;
	}
	return  XN_STATUS_OK;
}

XnStatus  XnTofSensor::FreeFrameGroupBuffer(ObFrameGroup *pFrameGroup)
{
	if (nullptr == pFrameGroup || nullptr == pFrameGroup->frames)
	{
		return  XN_STATUS_NULL_INPUT_PTR;
	}
	for (XnUInt32 i = 0; i < pFrameGroup->frame_count; i++)
	{
		if (nullptr != pFrameGroup->frames[i])
		{
			if (nullptr != pFrameGroup->frames[i]->buffer)
			{
				delete pFrameGroup->frames[i]->buffer;
				pFrameGroup->frames[i]->data_offset = nullptr;
			}
			delete  pFrameGroup->frames[i];
		}
	}
	delete[] pFrameGroup->frames;
	return  XN_STATUS_OK;
}

int XnTofSensor::ObcInt8ToInt16(XnInt16 * src, XnInt16 * dst, int w, int h) {
	if (NULL == src || NULL == dst) {
		return  -STATUS_DEPTH_LIB_NULL_INPUT_PTR;
	}
	for (int i = 0; i < w*h; i++) {
		dst[i] = (((src[i]*1) & 0xFF00) ? ((src[i]*1) & 0x00FF) : (src[i]*1));
	}
	return STATUS_OK;
}

XnStatus XnTofSensor::UnpackRaw8Data(XnInt8 *in, XnInt16 *out, XnInt width, XnInt height)
{
	if (nullptr == in || nullptr == out)
	{
		return XN_STATUS_NULL_INPUT_PTR;
	}
	int raw8Size = width * height * 8 / kBitPerByte;

	for (XnInt i = 0; i < raw8Size; i += 1)
	{
		const XnInt8* pStart = &in[i];

		XnInt8 shift0 = pStart[0];
		out[0] = shift0;
		out += 1;
	}
	return  XN_STATUS_OK;
}



int XnTofSensor::ObcInt10ToInt16(XnInt16 * src, XnInt16 * dst, int w, int h) {
	if (NULL == src || NULL == dst) {
		return  -STATUS_DEPTH_LIB_NULL_INPUT_PTR;
	}
	for (int i = 0; i < w*h; i++) {
		dst[i] = ((src[i] & 0xfc00) ? (src[i] & 0x03FF) : src[i]);
	}
	return STATUS_OK;
}

XnStatus XnTofSensor::UnpackRaw10Data(XnInt8 *in, XnInt16 *out, XnInt width, XnInt height)
{
	if (nullptr == in || nullptr == out)
	{
		return XN_STATUS_NULL_INPUT_PTR;
	}

	int raw10Size = width * height * 10 / kBitPerByte;
	for (XnInt i = 0; i < raw10Size; i += 5)
	{
		const XnInt8 *pStart = &in[i];
		XnInt16 shift0 = pStart[0];
		XnInt16 shift1 = pStart[1];
		XnInt16 shift2 = pStart[2];
		XnInt16 shift3 = pStart[3];
		XnInt16 shift4 = pStart[4];

		out[0] = (shift0 << 2) | (shift4 & 0x03);
		out[1] = (shift1 << 2) | ((shift4 >> 2) & 0x03);
		out[2] = (shift2 << 2) | ((shift4 >> 4) & 0x03);
		out[3] = (shift3 << 2) | ((shift4 >> 6) & 0x03);

		out += 4;
	}
	return  XN_STATUS_OK;

}

int XnTofSensor::ObcInt12ToInt16(XnInt16 * src, XnInt16 * dst, int w, int h) {
	if (NULL == src || NULL == dst) {
		return  -STATUS_DEPTH_LIB_NULL_INPUT_PTR;
	}
	for (int i = 0; i < w*h; i++) {
		 dst[i] = ((src[i] & 0xf000) ? (src[i] & 0x0FFF) : src[i]);
	}
	return STATUS_OK;
}

XnStatus XnTofSensor::UnpackRaw12Data(XnInt8 *in, XnInt16 *out, XnInt width, XnInt height)
{
	if (nullptr == in || nullptr == out)
	{
		return XN_STATUS_NULL_INPUT_PTR;
	}
	int raw12Size = width * height * 12 / kBitPerByte;

	for (XnInt i = 0; i < raw12Size; i += 6)
	{
		const XnInt8* pStart = &in[i];

		XnInt16 shift0 = pStart[0];
		XnInt16 shift1 = pStart[1];
		XnInt16 shift2 = pStart[2];
		out[0] = (shift0 << 4) | (shift2 & 0xF);
		out[1] = ((shift1 << 4) | ((shift2 >> 4) & 0xF))+70;

		XnInt16 shift3 = pStart[3];
		XnInt16 shift4 = pStart[4];
		XnInt16 shift5 = pStart[5];
		out[2] = (shift3 << 4) | (shift5 & 0xF);
		out[3] = ((shift4 << 4) | ((shift5 >> 4) & 0xF))+70;

		out += 4;
	}
	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::SortMetaData(const ObTofFrameInfo &frameInfo, OniMetadata *pMetaData)
{

	pMetaData->dutyCycle[0] = frameInfo.duty_cycle;
	pMetaData->dutyCycle[1] = frameInfo.duty_cycle;
	pMetaData->frameIndex = frameInfo.frame_index;
	pMetaData->type = (OniFrameType)frameInfo.frame_type;
	pMetaData->frequency[0] = frameInfo.frequency;
	pMetaData->frequency[1] = frameInfo.frequency;
	pMetaData->groupIndex = frameInfo.group_index;
	pMetaData->height = frameInfo.height;
	pMetaData->integration[0] = frameInfo.integration_time;
	pMetaData->integration[1] = frameInfo.integration_time;
	pMetaData->mode = (OniTOFSensorMode)frameInfo.out_mode;
	pMetaData->shuffle = false;
	pMetaData->id = (OniSensorID)frameInfo.sensor_type;
	pMetaData->temperDelay = frameInfo.time_delay_circuit_temp;
	pMetaData->temperRX = frameInfo.rx_temp;
	pMetaData->temperTX = frameInfo.driver_ic_temp;
	pMetaData->width = frameInfo.width;

	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::SetProperty(XnInt id, void*data, XnInt size)
{
	command_data_t  command_data;
	command_data.len = size;
	command_data.data = data;
	XnInt ret = m_pModule->set_property((hw_command_t)id, &command_data);
	// LOG_DEBUG("DothinDevice::setProperty %d, dothinId = %d,ret=%d\n", id, dev_id_, ret);
	return  ret;
}

XnStatus XnTofSensor::GetProperty(XnInt id, void*data, XnInt *size)
{
	command_data_t  command_data;
	command_data.len = *size;
	command_data.data = data;
	XnInt ret = m_pModule->get_property((hw_command_t)id, &command_data);
	// LOG_DEBUG("DothinDevice::getProperty %d, dothinId = %d,ret=%d\n", id, dev_id_, ret);
	*size = command_data.len;
	return  ret;
}

XnStatus XnTofSensor::UpdateBinningFactor()
{
	uint8_t binning_mode;
	int size = sizeof(uint8_t);
	int ret = GetProperty(BINNING_MODE, &binning_mode, &size);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "BINNING_MODE fail %d", ret);
		return ret;
	}
	switch (binning_mode)
	{
	case TOF_BINNING_MODE_1X1:
		m_binningFactor = 1;
		break;
	case TOF_BINNING_MODE_2X2:
		m_binningFactor = 2;
		break;
	case TOF_BINNING_MODE_4X4:
		m_binningFactor = 3;
		break;
	default:

		m_binningFactor = 1;
	}
	return XN_STATUS_OK;
}

XnStatus XnTofSensor::UpdateWindowFactor()
{
	uint32_t window_height;
	uint32_t window_width;
	int size = sizeof(uint32_t);
	int ret1 = GetProperty(WINDOW_HEIGHT, &window_height, &size);
	int ret2 = GetProperty(WINDOW_WIDTH, &window_width, &size);
	if (XN_STATUS_OK != ret1)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "Window Get Height fail %d", ret1);
		return ret1;
	}
	if (XN_STATUS_OK != ret2)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "Window Get Width fail %d", ret2);
		return ret2;
	}
	m_windowHeight = window_height;
	m_windowWidth = window_width;
	return XN_STATUS_OK;
}

XnStatus XnTofSensor::UpdateSampFactor()
{
	uint8_t SubSamp, SubSampv;
	int size = sizeof(uint8_t);
	int ret1 = GetProperty(SUB_SAMP, &SubSamp, &size);
	int ret2 = GetProperty(SUB_SAMP_V, &SubSampv, &size);

	if (XN_STATUS_OK != ret1)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "Get SubSamp fail %d", ret1);
		return ret1;
	}
	m_subsamp = SubSamp;
	m_subsampv = SubSampv;

	return XN_STATUS_OK;
}

XnStatus XnTofSensor::UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup)
{
	if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "pFrameGroup is nullptr");
		return  XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret = 0;
	pFramegroup->frame_count = pCompressedFrameGroup->frame_count;
	pFramegroup->real_count = pCompressedFrameGroup->real_count;
	//xnLogWarning(XN_MASK_TOF_SENSOR, "engine UnpackCompressedFrameGroup frame_count %d,real_count=%d", pFramegroup->frame_count, pFramegroup->real_count);
	//xnLogWarning(XN_MASK_TOF_SENSOR, "engine out (%d,%d)", pCompressedFrameGroup->frames[0]->width, pCompressedFrameGroup->frames[0]->height );

	for (XnInt i = 0; i < pCompressedFrameGroup->real_count; ++i)
	{
		if (nullptr == pCompressedFrameGroup->frames[i] || nullptr == pFramegroup->frames[i])
		{
			xnLogError(XN_MASK_TOF_SENSOR, "pFrame is nullptr, index: %d", i);
			return  XN_STATUS_NULL_INPUT_PTR;
		}
		ObTofFrameInfo *inputFrame = pCompressedFrameGroup->frames[i];
		ObTofFrameInfo *outputFrame = pFramegroup->frames[i];

		outputFrame->CloneParam(inputFrame);
		outputFrame->mipi_pack = 16;
		outputFrame->data_offset = outputFrame->buffer;

		switch (pCompressedFrameGroup->frames[0]->mipi_pack)
		{
		case 10:
			ret = UnpackRaw10Data((XnInt8*)inputFrame->data_offset, (XnInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
			break;
		case 12:
			ret = UnpackRaw12Data((XnInt8*)inputFrame->data_offset, (XnInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
			break;
		default:
			break;
		}
		if (XN_STATUS_OK != ret)
		{
			xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail %d", ret);
			return  ret;
		}
	}
	return 0;
}

XnStatus XnTofSensor::CalcSensorProperty()
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::UpdateCalcConfig()
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetDutyCyclePercentFormat(XnUInt32 freq,
	XnUInt32 ducy_cle,
	XnInt sensor_id,
	XnFloat *pduty_cycle_percent_format)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::SetModulationFrequency(XnUInt32 freq)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::SetIntegrationTime(XnUInt32 integration_time)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::SetDutyCycle(XnUInt32 integration_time)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::SetTriggerSignal()
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::SetBinningFactor(XnUInt32 binningFactor)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetBinningFactor(XnUInt32 &binningFactor)
{
	binningFactor = m_binningFactor;
	return XN_STATUS_OK;
}

XnStatus XnTofSensor::getCameraParams(OBCameraParams *cmameraParams){
	float l_intrin[5];
	float l_distort_coeff[5];
	XnStatus ret_status = XN_STATUS_ERROR;
	//int ret = GetToFIntrinsic(m_calibParamBuf, l_intrin, l_distort_coeff);
	//if (ret == 0){
	//	cmameraParams->l_intr_p[0] = l_intrin[0];
	//	cmameraParams->l_intr_p[1] = l_intrin[1];
	//	cmameraParams->l_intr_p[2] = l_intrin[2];
	//	cmameraParams->l_intr_p[3] = l_intrin[3];
	//	cmameraParams->l_k[0] = l_distort_coeff[0];
	//	cmameraParams->l_k[1] = l_distort_coeff[1];
	//	cmameraParams->l_k[2] = l_distort_coeff[2];
	//	cmameraParams->l_k[3] = l_distort_coeff[3];
	//	cmameraParams->l_k[4] = l_distort_coeff[4];
	//	ret_status = XN_STATUS_OK;
	//}
	return ret_status;
}

XnStatus XnTofSensor::LoadFilterConfig(XnChar* filePath)
{
#if 0
	if (nullptr == filePath || nullptr == m_configParamBuf)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "nullptr == LoadConfigFile || nullptr == m_configParamBuf");
		return -STATUS_DEPTH_LIB_NULL_INPUT_PTR;
	}
	XnBool isExist = false;
	XnInt ret = xnOSDoesFileExist(filePath, &isExist);
	if (XN_STATUS_OK != ret || false == isExist)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadConfigFile %s does not exist", filePath);
		return -STATUS_DEPTH_LIB_INVALID_PARAM;
	}
	{
		//TODO XXGDEBUG
		XnChar dirname[256];
		memset(dirname, 0, sizeof(dirname));
		xnOSGetCurrentDir(dirname, sizeof(dirname));
		xnLogInfo(XN_MASK_TOF_SENSOR, "engine xnOSGetCurrentDir:%s", dirname);
	}
	assert(_CrtCheckMemory());
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine LoadToFFilterParam filePath:%s", filePath);
	//加载参数配置文件
#if USE_FAKE_TOF_DEPTH_LIB
	ret = LoadToFFilterParam(const_cast<char*>(filePath), m_configParamBuf, m_modeConfig, m_outputConfig);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadToFConfig fail %d\n", ret);
		return ret;
	}
	else
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadToFConfig success\n");
	}
#else 
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine fake LoadToFFilterParam\n");
#endif
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine _CrtCheckMemory");
	assert(_CrtCheckMemory());
#endif
	return XN_STATUS_OK;
}

XnStatus XnTofSensor::LoadCaliParam(XnChar* filePath)
{
#if 0
	//m_calibParamBuf 传入空指针即可
	if (nullptr == filePath)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "nullptr == LoadCaliParamFile");
		return -STATUS_DEPTH_LIB_NULL_INPUT_PTR;
	}
	{
		//TODO XXGDEBUG
		XnChar dirname[256];
		memset(dirname, 0, sizeof(dirname));
		xnOSGetCurrentDir(dirname, sizeof(dirname));
		xnLogInfo(XN_MASK_TOF_SENSOR, "engine xnOSGetCurrentDir:%s", dirname);
	}
	XnBool isExist = false;
	XnInt ret = xnOSDoesFileExist(filePath, &isExist);
	if (XN_STATUS_OK != ret || false == isExist)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadCaliParamFile %s does not exist", filePath);
		return -STATUS_DEPTH_LIB_INVALID_PARAM;
	}
	//assert(_CrtCheckMemory());
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine LoadToFCaliParam filePath:%s", filePath);
#if USE_FAKE_TOF_DEPTH_LIB
	//加载标定配置文件
	ret = LoadToFCaliParam(const_cast<char*>(filePath), m_calibParamBuf, m_modeConfig);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadToFCaliParam fail %d\n", ret);
		return ret;
	}
	else
	{
		xnLogError(XN_MASK_TOF_SENSOR, "LoadToFCaliParam success \n");
	}
#else 
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine fake LoadToFCaliParam\n");
#endif
#endif
	return XN_STATUS_OK;
}

//XnStatus XnTofSensor::SetOutputConfig(Output_Config outputConfig)
//{
//	xnOSMemCopy(&m_outputConfig, &outputConfig, sizeof(Output_Config));
//	return XN_STATUS_OK;
//}

XnStatus XnTofSensor::ChangeFreqMode(ObFrequencyMode &mode)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::UpdateFrameInfo(OniFrame *frame)
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
		frame->frameIndex = m_pFrameGroup->frames[0]->group_index;
		frame->width = frame->videoMode.resolutionX;
		frame->stride = frame->width * sizeof(XnUInt16);
		frame->height = frame->videoMode.resolutionY;
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16)+frame->width*frame->extraLine*sizeof(XnUInt16);
		frame->extraLine = 1;
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
		frame->extraLine = m_extraLine;
		frame->width = m_phaseWidth;
		frame->stride = frame->width * sizeof(XnUInt16);
		frame->height = m_phaseHeight * m_phaseOutputCount;
		frame->dataSize = frame->width * frame->height * sizeof(XnUInt16)+frame->stride * m_phaseOutputCount;
		OniMetadata *pMetadata = nullptr;
		for (XnInt i = 0; i < m_phaseOutputCount; ++i)
		{
			pMetadata = static_cast<OniMetadata*>(frame->data) + frame->stride * i;
			if (pMetadata != nullptr)
			{
				SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
			}
			else
			{
				xnLogError(XN_MASK_TOF_SENSOR, "pMetadata is null\n");
			}
			
		}
	}

	return XN_STATUS_OK;
}

XnStatus XnTofSensor::UpdateMode()
{
	XnStatus ret = XN_STATUS_OK;
	ret = UpdateWindowFactor();
	ret = UpdateBinningFactor();
	ret = UpdateSampFactor();
	XN_IS_STATUS_OK(ret);

	return XN_STATUS_OK;
}



XnStatus XnTofSensor::updateFrameInfo()
{
	return XN_STATUS_OK;
}
XnStatus XnTofSensor::ParseExtendedData(ObTofFrameInfo *framegroup, XnInt index)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height){
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetAIFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, OniAIFrame* pAiFrame)
{
	return XN_STATUS_NOT_IMPLEMENTED;

}

XnStatus XnTofSensor::GetPhaseFrame(const XnChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnInt16 *frameData, XnUInt32 *frameDataSize)
{
	return XN_STATUS_NOT_IMPLEMENTED;

}

XnStatus XnTofSensor::GetIRFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::GetDepthFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{

	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::GetAmplitudeFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetInfraredFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	return XN_STATUS_NOT_IMPLEMENTED;
}

XnStatus XnTofSensor::GetGrabBufferSize(XnUInt32 *grabWidth, XnUInt32 *grabHeight, XnUInt32 *grabBufferSize)
{
	*grabWidth = m_grabWidth;
	*grabHeight = m_grabHeight;
	*grabBufferSize = (*grabWidth) * (*grabHeight) * m_bitPerPhasePixel / kBitPerByte;

	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize)
{
	*outputBufferWidth = m_phaseWidth;
	*outputBufferHeight = m_phaseHeight * m_phaseOutputCount + m_phaseOutputCount;//每帧对应一行扩展信息
	*outputBufferSize = (*outputBufferWidth) * (*outputBufferHeight) * sizeof(XnUInt16);

	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize)
{
	*outputImageWidth = m_phaseWidth;
	//*outputImageHeight = m_phaseHeight * m_extraLine;
	*outputImageHeight = m_phaseHeight * m_phaseOutputCount;
	*outputImageSize = *outputImageWidth * *outputImageHeight * sizeof(XnUInt16);

	return  XN_STATUS_OK;
}

XnUInt32 XnTofSensor::getExtraLine(){
	return m_extraLine;
}
XnStatus XnTofSensor::setFrameResolution(int width, int height)
{

	return  XN_STATUS_OK;
}
