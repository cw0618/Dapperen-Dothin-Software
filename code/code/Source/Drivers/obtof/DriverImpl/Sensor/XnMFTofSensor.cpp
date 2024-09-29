#include "XnMFTofSensor.h"
#include "OniCTypes.h"
#include "OpenNI.h"
#include"tofinfo.h"

XnMFTofSensor::XnMFTofSensor(mx6x_module_t *mx6xModule) :
XnTofSensor(mx6xModule)
{
	m_sensorId = TOF_SENSOR_ID_MLX75027;
	m_freqMode = TOF_SINGLE_FREQUENCY_MODE;

	m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
	m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
	m_phaseInputCount = kFreqPhaseInputNum;
	m_phaseOutputCount = kFreqPhaseOutputNum;
	m_bitPerPhasePixel = kPixelBit,
	m_extraLine = m_phaseOutputCount;
	m_grabWidth = m_phaseWidth;
	m_grabHeight = (kOriginalPhaseHeight + kGrabExtendedDataLine) * m_phaseInputCount;
	m_dothinMipiPackBit = kMipiPackBit;
}

XnMFTofSensor::~XnMFTofSensor()
{
	m_configFilePathAndName = nullptr;
	m_calibFilePathAndName = nullptr;
}

XnStatus XnMFTofSensor::Init()
{
	return XN_STATUS_OK;
}

XnStatus XnMFTofSensor::CalcSensorProperty()
{
	//switch (m_freqMode)
	//{
	//case TOF_SINGLE_FREQUENCY_MODE:
	//	m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
	//	m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
	//	m_phaseInputCount = kFreqPhaseInputNum;
	//	m_phaseOutputCount = kFreqPhaseOutputNum;
	//	m_grabWidth = m_phaseWidth;
	//	m_grabHeight = (m_phaseHeight + kGrabExtendedDataLine) * m_phaseInputCount;
	//	break;
	//case TOF_DUAL_FREQUENCY_MODE:
	//case TOF_AF_FREQUENCY_MODE:
	//	m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
	//	m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
	//	m_phaseInputCount = kFreqPhaseInputNum;
	//	m_phaseOutputCount = kFreqPhaseOutputNum;
	//	m_grabWidth = m_phaseWidth;
	//	m_grabHeight = (m_phaseHeight + kGrabExtendedDataLine) * m_phaseInputCount;
	//	break;
	//default:
	//	break;
	//}
	xnLogInfo(XN_MASK_MF_TOF_SENSOR, "CalcSensorProperty, m_freqMode %d, m_binningFactor %d, grab_w %d, grab_h %d", m_freqMode, m_binningFactor, m_grabWidth, m_grabHeight);

	return XN_STATUS_OK;
}

XnStatus XnMFTofSensor::GetDutyCyclePercentFormat(uint32_t freq, uint32_t ducy_cle, XnInt sensor_id, float *pduty_cycle_percent_format)
{
	UNREFERENCED_PARAMETER(sensor_id);
	if (nullptr == pduty_cycle_percent_format)
	{
		return  -1;
	}
	if (ducy_cle < 0 || ducy_cle >30)
	{
		return  -2;
	}
	float list[31];

	GetIllumDutyCycleList((XnUInt8)freq, list);
	*pduty_cycle_percent_format = list[ducy_cle];
	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list)
{
	ObDutyCycleStep value;
	value.frequency = mod_freq;
	int size = sizeof(value);

	XnStatus ret = GetProperty(DUTY_CYCLE_LIST, &value, &size);
	if (XN_STATUS_OK == ret)
	{
		if (nullptr != duty_cycle_list)
		{
			memcpy(duty_cycle_list, value.duty_cycle_steps, sizeof(value.duty_cycle_steps));
		}
	}
	return ret;
}

XnStatus XnMFTofSensor::UnpackCompressedIRFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup, int phaseIndex)
{
	if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "pFrameGroup is nullptr");
		return  XN_STATUS_NULL_INPUT_PTR;
	}
	XnStatus ret;
	pFramegroup->frame_count = m_phaseOutputCount;
	pFramegroup->real_count = m_phaseOutputCount;

	if (nullptr == pCompressedFrameGroup->frames[0] || nullptr == pFramegroup->frames[phaseIndex])
		{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "pFrame is nullptr, index: %d", phaseIndex);
			return  XN_STATUS_NULL_INPUT_PTR;
		}
	ObTofFrameInfo *inputFrame = pCompressedFrameGroup->frames[0];
	ObTofFrameInfo *outputFrame = pFramegroup->frames[phaseIndex];

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
		xnLogError(XN_MASK_MF_TOF_SENSOR, "UnpackCompressedFrameGroup fail %d", ret);
		return  ret;
	}
	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup)
{
	if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "pFrameGroup is nullptr");
		return  XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret;
	pFramegroup->frame_count = pCompressedFrameGroup->frame_count;
	pFramegroup->real_count = pCompressedFrameGroup->real_count;

	for (XnInt i = 0; i < pCompressedFrameGroup->real_count; ++i)
	{
		if (nullptr == pCompressedFrameGroup->frames[i] || nullptr == pFramegroup->frames[i])
		{
			xnLogError(XN_MASK_MF_TOF_SENSOR, "pFrame is nullptr, index: %d", i);
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

			if (inputFrame->out_mode == TOF_OUTPUT_MODE_A_SUB_B)
			{
				ret = ObcInt12ToInt16((XnInt16*)outputFrame->data_offset, (XnInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
			}
			break;
		default:
			break;
		}
		if (XN_STATUS_OK != ret)
		{
			xnLogError(XN_MASK_MF_TOF_SENSOR, "UnpackCompressedFrameGroup fail %d", ret);
			return  ret;
		}
	}
	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::SetIntegrationTime(uint32_t integration_time)
{
	XnStatus ret = SetProperty(INTEGRATION_TIME, &integration_time, sizeof(integration_time));
	return ret;
}

XnStatus XnMFTofSensor::SetDutyCycle(uint32_t cycle)
{
	XnStatus ret = SetProperty(DUTY_CYCLE, &cycle, sizeof(cycle));
	return ret;
}

XnStatus XnMFTofSensor::SetTriggerSignal()
{
	XnStatus ret = SetProperty(SOFTWARE_TRIGGER, nullptr, 0);
	return ret;
}

XnStatus XnMFTofSensor::SetBinningFactor(XnUInt32 binningFactor)
{
	XnUInt8 bm;

	switch (binningFactor)
	{
	case 1:
		bm = TOF_BINNING_MODE_1X1;
		break;
	case 2:
		bm = TOF_BINNING_MODE_2X2;
		break;
	case 4:
		bm = TOF_BINNING_MODE_4X4;
		break;
	default:
		xnLogError(XN_MASK_MF_TOF_SENSOR, "SetBinningFactor invalid binningFactor: %d", binningFactor);
		return XN_STATUS_BAD_PARAM;
	}

	command_data_t  command_data;
	command_data.data = &bm;
	command_data.len = sizeof(XnUInt8);

	XnInt ret = m_pModule->set_property(BINNING_MODE, &command_data);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "set_property BINNING_MODE fail %d", ret);
		return XN_STATUS_PROPERTY_NOT_SET;
	}

	ret = UpdateMode();
	XN_IS_STATUS_OK(ret);

	return XN_STATUS_OK;
}

XnStatus XnMFTofSensor::UpdateMode()
{
	XnStatus nRet = XN_STATUS_OK;
	updateFrameInfo();
	nRet = XnTofSensor::UpdateMode();
	
	XN_IS_STATUS_OK(nRet);

	return XN_STATUS_OK;
}

XnStatus XnMFTofSensor::updateFrameInfo()
{
	int ret = XN_STATUS_OK;
	int size = 0;
	int freq_mode = 0;
	int freq = 0;
	
	int freq_mode_size = sizeof(freq_mode);
	ret = GetProperty(FREQUENCY_MODE, &freq_mode, &freq_mode_size);

	int freqsize = sizeof(freq);
	ret = GetProperty(MODULATION_FREQUENCY, &freq, &freqsize);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "FREQUENCY_MODE fail %d", ret);
		return ret;
	}
	m_freqMode = freq_mode;
	m_frequencyOne = freq >> 8;
	m_frequencyTwo = freq & 0xff;
	xnLogError(XN_MASK_TOF_SENSOR, "freqMode %d, frequencyOne %d, frequencyTwo %d", m_freqMode, m_frequencyOne, m_frequencyTwo);

	int phase_count = 0;
	size = sizeof(phase_count);
	ret = GetProperty(PHASE_COUNT, &phase_count, &size);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "PHASE_COUNT fail %d", ret);
		return -1;
	}

	xnLogError(XN_MASK_TOF_SENSOR, "phase_count %d", phase_count);

	if (m_freqMode == TOF_DUAL_FREQUENCY_MODE)
	{
		//双频
		if (phase_count == 4)
		{
			m_phaseOutputCount = 8;
			m_extraLine = 8;
			kFreqPhaseOutputNum = 8;
		}
		else
		{
			m_phaseOutputCount = 16;
			m_extraLine = 16;
			kFreqPhaseOutputNum = 16;
		}

	}
	else
	{
		//单频
		m_phaseOutputCount = phase_count;
		m_extraLine = phase_count;
		kFreqPhaseOutputNum = phase_count;
	}
	return ret;
}

XnStatus XnMFTofSensor::ParseExtendedData(ObTofFrameInfo* tof_frames, XnInt index)
{
	XnStatus ret = 0;
	tof_frames->mipi_pack = m_dothinMipiPackBit;
	tof_frames->width = m_phaseWidth;
	tof_frames->height = m_phaseHeight;
	tof_frames->frame_index = index % 3;
	tof_frames->group_index = 0;
	tof_frames->sensor_type = m_sensorId;

	uint32_t framesize = (tof_frames->width * tof_frames->height *tof_frames->mipi_pack) / kBitPerByte;
	XnInt kEbdLength = m_phaseWidth * kGrabExtendedDataLine * m_dothinMipiPackBit / kBitPerByte;
	// LOG_DEBUG("ParserExpandData framesize = %d,out=%d\n", framesize, cur_vmode_.out_mode);
	XnChar* expand_data = (XnChar*)tof_frames->buffer + framesize;
	uint8_t output_mode = expand_data[31 * 3];
	tof_frames->rx_temp = (float)((uint8_t)expand_data[39 * 3] - 40);
	tof_frames->duty_cycle = 0.5f;
	tof_frames->driver_ic_temp = 0;
	// line2
	uint8_t user_id = expand_data[29 * 3];

	uint8_t frame_counter = expand_data[kEbdLength / 2 + 45 * 3]; //group
	uint8_t phase_counter = expand_data[kEbdLength / 2 + 48 * 3]; //frame no
	
	tof_frames->frequency = user_id;
	tof_frames->frame_index = phase_counter;
	tof_frames->group_index = frame_counter;
	tof_frames->out_mode = output_mode;
	tof_frames->data_offset = tof_frames->buffer;
	//xnLogError(XN_MASK_MF_TOF_SENSOR, "group_index = %d ,frame_index = %d frequency = %d", tof_frames->group_index, tof_frames->frame_index, tof_frames->frequency);
	return ret;
}

XnStatus XnMFTofSensor::CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height){
	if (nullptr == p_phase_framegroup->frames || nullptr == p_IR_frame) {
		return  -STATUS_NULL_INPUT_PTR;
	}

	memset(p_IR_frame, 0, width *height * 2);

	for (uint32_t index = 0; index < p_phase_framegroup->real_count; index++) {
		uint16_t *frame = (uint16_t*)p_phase_framegroup->frames[index]->data_offset;

		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				p_IR_frame[i*width + j] += frame[i * 2 * width + j * 2]
					+ frame[i * 2 * width + j * 2 + 1];
			}
		}
	}

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			p_IR_frame[i*width + j] = (uint16_t)((p_IR_frame[i*width + j] / (p_phase_framegroup->real_count * 2)));
		}
	}

	return STATUS_OK;
}

XnStatus XnMFTofSensor::GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height)
{
	switch (p_phase_framegroup->frames[0]->sensor_type)
	{
	case OBC_SENSOR_ID_S5K33D:
	case OBC_SENSOR_ID_RK1608_S5K33D:
	case OBC_SENSOR_ID_MLX75027:
	case OBC_SENSOR_ID_IMX456:
	case OBC_SENSOR_ID_IMX516:
	case OBC_SENSOR_ID_IMX518:
	case OBC_SENSOR_ID_PLECO:
	case OBC_SENSOR_ID_RK1608_PLECO:
	{
		*width = kIRWidth;
		*height = kIRHeight;
		break;
	}
	case OBC_SENSOR_ID_IMX316:
	{
		*width = 240;
		*height = 180;
		break;
	}
	default:
	{
			   xnLogError(XN_MASK_MF_TOF_SENSOR, "unsupport sensor: %x", p_phase_framegroup->frames[0]->sensor_type);
			   return -STATUS_DEPTH_LIB_NO_SUPPORT;
	}
	}
	return STATUS_OK;
}

XnStatus XnMFTofSensor::GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup)
{
	XnStatus ret = XN_STATUS_OK;
	if (nullptr == p_framegroup || nullptr == p_framegroup->frames)
	{
		return  XN_STATUS_ERROR;
	}
	if (p_framegroup->frame_count < m_phaseInputCount)
	{
		return  XN_STATUS_ERROR;
	}
	/*
	LOG_DEBUG("m_freqMode = %d, frame_count = %d, phase_cout = %d",
	m_freqMode,
	p_framegroup->frame_count,
	m_phaseCount);
	*/
	/**
	* EBD  数据为2行，位于每帧数据的后面
	* 数据存储格式  phase_img + embedded_data
	* 出图顺序为 0 240 120
	*/
	const XnUInt8 phase_info[8] = { 0, 240, 120, 0, 240, 120 };

	XnInt phase_img_mipi_size = m_phaseWidth * m_phaseHeight * m_dothinMipiPackBit / kBitPerByte;
	XnInt embedded_data_size = m_phaseWidth * kGrabExtendedDataLine * m_dothinMipiPackBit / kBitPerByte;
	//LOG_DEBUG("phase_img_mipi_size  %d,embedded_data_size =%d", phase_img_mipi_size, embedded_data_size);

	if (m_freqMode == TOF_SINGLE_FREQUENCY_MODE)
	{
		//m_phaseInputCount为1，mf2202项目度信是一帧帧的读取，不是读取整个帧组
		p_framegroup->real_count = m_phaseInputCount;

		for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
		{
			p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
			memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[i*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
			ret = ParseExtendedData(p_framegroup->frames[i], i);
			if (ret < 0)
			{
				xnLogError(XN_MASK_MF_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
				return  ret;
			}
			//TODO,mf2202项目out_mode从属性获取
			p_framegroup->frames[i]->phase_map = phase_info[i];
		}
		return XN_STATUS_OK;
	}
	else if (m_freqMode == TOF_DUAL_FREQUENCY_MODE || m_freqMode == TOF_AF_FREQUENCY_MODE)
	{
		p_framegroup->real_count = m_phaseInputCount;

		for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
		{
			p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
			//整理成前一半为第一个频率，后一半为第二个频率
			switch (i)
			{
			case 0:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[0], embedded_data_size + phase_img_mipi_size);
				break;
			case 1:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(2)*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				break;
			case 2:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(4)*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				break;
			case 3:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(1)*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				break;
			case 4:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(3)*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				break;
			case 5:
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(5)*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				break;
			}

			ret = ParseExtendedData(p_framegroup->frames[i], i);
			if (ret < 0)
			{
				xnLogError(XN_MASK_MF_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
				return  ret;
			}
			p_framegroup->frames[i]->out_mode = TOF_OUTPUT_MODE_3TAP;
			p_framegroup->frames[i]->phase_map = phase_info[i];
		}
		return  XN_STATUS_OK;
	}
	else
	{
		return XN_STATUS_ERROR;
	}
}

XnStatus XnMFTofSensor::GetPhaseFrame(const XnChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnInt16 *frameData, XnUInt32 *frameDataSize)
{
	if (nullptr == dothinInputBuf || nullptr == frameData
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "GetPhaseFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);
	ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
		return ret;
	}

	XnUInt32 outputImageWidth, outputImageHeight, outputImageSize;
	GetOutputImageSize(&outputImageWidth, &outputImageHeight, &outputImageSize);

	//oniframe的data中包含metadata，这里需要更改帧数据地址，加上偏移量
	XnChar *frameDataAddr = static_cast<XnChar *>(static_cast<void*>(frameData)) + outputImageWidth * sizeof(XnUInt16)* m_phaseOutputCount;
	return organizeFrame(frameDataAddr, outputImageSize);
}

XnStatus XnMFTofSensor::organizeFrame(XnChar *frameDataAddr, XnUInt32 outputImageSize)
{
	ObTofFrameInfo* frameInfo = m_pFrameGroup->frames[0];
	frameInfo->out_mode = m_freqMode;
	UpdateCalcConfig();
	//单频组帧
	if (m_freqMode == TOF_SINGLE_FREQUENCY_MODE)
	{
		return singleFreqorganize(frameDataAddr, outputImageSize);
	}
	else if (m_freqMode == TOF_DUAL_FREQUENCY_MODE)
	{
		//mf2202双频组帧
		return dualFreqorganize(frameDataAddr, outputImageSize);
	}
}

//单频组帧
XnStatus XnMFTofSensor::singleFreqorganize(XnChar *frameDataAddr, XnUInt32 outputImageSize)
{
	ObTofFrameInfo* frameInfo = m_pFrameGroup->frames[0];
	if (frameInfo->frame_index == 0)
	{
		//帧组中的第一帧phase
		mGroupIndex = frameInfo->group_index;
		xnOSMemCopy(frameDataAddr, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
		mPhaseStep = PHASE_STEP_ONE;
		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 1)
	{
		//帧组中的第二帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_ONE)
		{
			mPhaseStep = PHASE_STEP_TWO;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 1, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
		}
		else
		{
			mPhaseStep = MAX_PHASE_STEP;
			mGroupIndex = -1;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 2)
	{
		//帧组中的第三帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_TWO)
		{
			mPhaseStep = PHASE_STEP_THREE;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 2, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 3)
	{
		//帧组中的第四帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_THREE)
		{
			mPhaseStep = PHASE_STEP_FOUR;

			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 3, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			//xnOSSaveFile("D:\\orbbecproject\\PhaseFrameData4.raw", frameDataAddr, outputImageSize);
			if (frameInfo->frequency == 0x80)
			{
				//8个相位一个帧组
				return XN_STATUS_CONTINUE;
			}
			else
			{
				return  XN_STATUS_OK;
			}
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	else if (frameInfo->frame_index == 4)
	{
		//帧组中的第五帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_FOUR)
		{
			mPhaseStep = PHASE_STEP_FIVE;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 4, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

			return XN_STATUS_CONTINUE;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	else if (frameInfo->frame_index == 5)
	{
		//帧组中的第六帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_FIVE)
		{
			mPhaseStep = PHASE_STEP_SIX;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 5, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

			return XN_STATUS_CONTINUE;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	else if (frameInfo->frame_index == 6)
	{
		//帧组中的第七帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_SIX)
		{
			mPhaseStep = PHASE_STEP_SEVEN;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 6, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

			return XN_STATUS_CONTINUE;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	else if (frameInfo->frame_index == 7)
	{
		//帧组中的第八帧phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_SEVEN)
		{
			mPhaseStep = PHASE_STEP_FIVE;
			xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount * 7, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			//xnOSSaveFile("D:\\orbbecproject\\PhaseFrameData4.raw", frameDataAddr, 640*480*2);
			return  XN_STATUS_OK;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}

	return XN_STATUS_CONTINUE;
}

//双频的组帧
XnStatus XnMFTofSensor::dualFreqorganize(XnChar *frameDataAddr, XnUInt32 outputImageSize)
{
	ObTofFrameInfo* frameInfo = m_pFrameGroup->frames[0];
	if (m_phaseOutputCount == 8)
	{
		//频率1，frameIndex是0，1，2，3，频率2，frameIndex是0，1，2，3。总共8帧一个帧组
		if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 0))
		{
			//帧组中的第一帧phase
			mGroupIndex = frameInfo->group_index;
			xnOSMemCopy(frameDataAddr, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			mPhaseStep = PHASE_STEP_ONE;

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_ONE)
		{
			//帧组中的第二帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 1))
			{
				mPhaseStep = PHASE_STEP_TWO;
				xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mPhaseStep = MAX_PHASE_STEP;
				mGroupIndex = -1;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_TWO)
		{
			//帧组中的第三帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 2))
			{
				mPhaseStep = PHASE_STEP_THREE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 2, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_THREE)
		{
			//帧组中的第四帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 3))
			{
				mPhaseStep = PHASE_STEP_FOUR;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 3, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_FOUR)
		{
			//帧组中的第五帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 0))
			{
				mPhaseStep = PHASE_STEP_FIVE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 4, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_FIVE)
		{
			//帧组中的第六帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 1))
			{
				mPhaseStep = PHASE_STEP_SIX;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 5, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_SIX)
		{
			//帧组中的第七帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 2))
			{
				mPhaseStep = PHASE_STEP_SEVEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 6, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_SEVEN)
		{
			//帧组中的第八帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 3))
			{
				mPhaseStep = PHASE_STEP_EIGHT;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 7, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
				//xnOSSaveFile("D:\\orbbecproject\\PhaseFrameData.raw", frameDataAddr, 640*480*2);

				return XN_STATUS_OK;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
	}
    else
    {
		//频率1，frameIndex是0，1，2，3，4，5，6，7，频率2，frameIndex是0，1，2，3，4，5，6，7，总共16帧一个帧组
		if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 0))
		{
			//帧组中的第一帧phase
			mGroupIndex = frameInfo->group_index;
			xnOSMemCopy(frameDataAddr, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			mPhaseStep = PHASE_STEP_ONE;

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_ONE)
		{
			//帧组中的第二帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 1))
			{
				mPhaseStep = PHASE_STEP_TWO;
				xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseOutputCount, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mPhaseStep = MAX_PHASE_STEP;
				mGroupIndex = -1;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_TWO)
		{
			//帧组中的第三帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 2))
			{
				mPhaseStep = PHASE_STEP_THREE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 2, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_THREE)
		{
			//帧组中的第四帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 3))
			{
				mPhaseStep = PHASE_STEP_FOUR;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 3, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_FOUR)
		{
			//帧组中的第五帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 4))
			{
				mPhaseStep = PHASE_STEP_FIVE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 4, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_FIVE)
		{
			//帧组中的第六帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 5))
			{
				mPhaseStep = PHASE_STEP_SIX;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 5, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_SIX)
		{
			//帧组中的第七帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 6))
			{
				mPhaseStep = PHASE_STEP_SEVEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 6, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
			}

			return XN_STATUS_CONTINUE;
		}
		else if (mPhaseStep == PHASE_STEP_SEVEN)
		{
			//帧组中的第八帧phase
			if ((m_frequencyOne == frameInfo->frequency) && (frameInfo->frame_index == 7))
			{
				mPhaseStep = PHASE_STEP_EIGHT;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 7, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);
				
				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_EIGHT)
		{
			//帧组中的第九帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 0))
			{
				mPhaseStep = PHASE_STEP_NINE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 8, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_NINE)
		{
			//帧组中的第十帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 1))
			{
				mPhaseStep = PHASE_STEP_TEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 9, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_TEN)
		{
			//帧组中的第十一帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 2))
			{
				mPhaseStep = PHASE_STEP_ELEVEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 10, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_ELEVEN)
		{
			//帧组中的第十二帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 3))
			{
				mPhaseStep = PHASE_STEP_TWELVE;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 11, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_TWELVE)
		{
			//帧组中的第十三帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 4))
			{
				mPhaseStep = PHASE_STEP_THIRTEEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 12, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_THIRTEEN)
		{
			//帧组中的第十四帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 5))
			{
				mPhaseStep = PHASE_STEP_FOURTEEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 13, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_FOURTEEN)
		{
			//帧组中的第十五帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 6))
			{
				mPhaseStep = PHASE_STEP_FIFTEEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 14, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_CONTINUE;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
		else if (mPhaseStep == PHASE_STEP_FIFTEEN)
		{
			//帧组中的第十六帧phase
			if ((m_frequencyTwo == frameInfo->frequency) && (frameInfo->frame_index == 7))
			{
				mPhaseStep = PHASE_STEP_SIXTEEN;
				xnOSMemCopy(frameDataAddr + (outputImageSize / m_phaseOutputCount) * 15, frameInfo->data_offset, outputImageSize / m_phaseOutputCount);

				return XN_STATUS_OK;
			}
			else
			{
				mGroupIndex = -1;
				mPhaseStep = MAX_PHASE_STEP;
				xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
				return XN_STATUS_CONTINUE;
			}
		}
    }
	return XN_STATUS_CONTINUE;
}

XnStatus XnMFTofSensor::GetIRFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	if (nullptr == dothinInputBuf || nullptr == frameData
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "GetIRFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}
	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

	ObTofFrameInfo* frameInfo = m_pCompressedFrameGroup->frames[0];
	if (frameInfo->frame_index == 0)
	{
		//帧组中的第一张phase
		mGroupIndex = frameInfo->group_index;
		ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 0);
		if (XN_STATUS_OK != ret)
		{
			xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
			return ret;
		}
		mPhaseStep = PHASE_STEP_ONE;
		
		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 1)
	{
		//帧组中的第二张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_ONE)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 1);
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
				
			}
			mPhaseStep = PHASE_STEP_TWO;
			return XN_STATUS_CONTINUE;
		}
		else
		{
			mPhaseStep = MAX_PHASE_STEP;
			mGroupIndex = -1;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "IR PhaseFrame Lost frameIndex %d", frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 2)
	{
		//帧组中的第三张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_TWO)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 2);
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
			}
			mPhaseStep = PHASE_STEP_THREE;
			
			return XN_STATUS_CONTINUE;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost frameIndex %d", frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 3)
	{
		//帧组中的第四张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_THREE)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 3);
			
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
			}

			XnUInt16* frameDataIR = (XnUInt16*)((XnUInt8*)frameData + kVGADepthWidth*kBytePerDepthPixel*kIRExtendedDataLine);
			ret = CalcIRFramePleco(m_pFrameGroup, frameDataIR, kVGADepthWidth, kVGADepthHeight);

			return  XN_STATUS_OK;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost frameIndex %d", frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::GetDepthFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	if (nullptr == dothinInputBuf || nullptr == frameData
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "GetIRFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}
	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

	ObTofFrameInfo* frameInfo = m_pCompressedFrameGroup->frames[0];
	if (frameInfo->frame_index == 0)
	{
		//帧组中的第一张phase
		mGroupIndex = frameInfo->group_index;
		ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 0);
		if (XN_STATUS_OK != ret)
		{
			xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
			return ret;
		}

		mPhaseStep = PHASE_STEP_ONE;

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 1)
	{
		//帧组中的第二张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_ONE)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 1);
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
			}
			mPhaseStep = PHASE_STEP_TWO;
			return XN_STATUS_CONTINUE;
		}
		else
		{
			mPhaseStep = MAX_PHASE_STEP;
			mGroupIndex = -1;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 2)
	{
		//帧组中的第三张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_TWO)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 2);
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
			}
			mPhaseStep = PHASE_STEP_THREE;
			return XN_STATUS_CONTINUE;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);
		}

		return XN_STATUS_CONTINUE;
	}
	else if (frameInfo->frame_index == 3)
	{
		//帧组中的第四张phase
		if (frameInfo->group_index == mGroupIndex && mPhaseStep == PHASE_STEP_THREE)
		{
			ret = UnpackCompressedIRFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup, 3);
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
				return ret;
			}
			ret = UpdateCalcConfig();
			if (XN_STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "UpdateCalcConfig fail, %d", ret);
				return ret;
			}
			for (int i = 0; i < m_pFrameGroup->real_count; ++i)
			{
				memcpy(m_phaseBuf + (m_phasePixelNum / m_pFrameGroup->real_count) * i, m_pFrameGroup->frames[m_phaseFrameIndex[i]]->data_offset, m_phasePixelNum * sizeof(uint16_t) / m_pFrameGroup->real_count);
			}
			//xnOSSaveFile("D:\\orbbecproject\\2020\\MF2202SDK\\openni2.3\\Bin\\Win32-Debug\\PhaseFrameData.raw", m_phaseBuf, kOriginalPhaseWidth * kOriginalPhaseHeight * kBytePerDepthPixel*4);
#if USE_FAKE_TOF_DEPTH_LIB
			//XnUInt64 startStamp;
			//xnOSGetTimeStamp(&startStamp);
			//ret = DepthCalc(m_phaseBuf, m_phasePixelNum,
			//	m_calibParamBuf, m_configParamBuf, m_tmpBuf,
			//	m_depth, m_intensity, m_amplitude,
			//	m_modeConfig, m_outputConfig);

#else
			xnLogError(XN_MASK_TOF_SENSOR, "engine fake DepthCalc fake");
#endif

			if (STATUS_OK != ret)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "DepthCalc fail, %d", ret);
				return ret;
			}
			XnUInt16* frameDataDepth = (XnUInt16*)((XnUInt8*)frameData + kVGADepthWidth*sizeof(XnUInt16)*kIRExtendedDataLine);
			for (int i = 0; i < kVGADepthWidth * kVGADepthHeight; i++){
				frameDataDepth[i] = (uint16_t)m_depth[i];
			}
			return  XN_STATUS_OK;
		}
		else
		{
			mGroupIndex = -1;
			mPhaseStep = MAX_PHASE_STEP;
			xnLogError(XN_MASK_MF_TOF_SENSOR, "PhaseFrame Lost group_index %d frameIndex %d", frameInfo->group_index, frameInfo->frame_index);

			return XN_STATUS_CONTINUE;
		}
	}
	//xnOSSaveFile(".\\DepthFrameData.raw", frameData, kVGADepthWidth * kVGADepthHeight * kBytePerDepthPixel);

	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::GetAIFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, OniAIFrame* pAiFrame)
{
	if (nullptr == dothinInputBuf || nullptr == pAiFrame
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "GetPhaseFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

	ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_MF_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
		return ret;
	}

	//XnUInt32 outputImageWidth, outputImageHeight, outputImageSize;
	//GetOutputImageSize(&outputImageWidth, &outputImageHeight, &outputImageSize);

	//oniframe的data中包含metadata，这里需要更改帧数据地址，加上偏移量
	XnUInt8* pAddr = (XnUInt8*)pAiFrame + sizeof(OniAIFrame);
	pAiFrame->frameSet.size = 1;
	pAiFrame->frameSet.status = ONI_AI_STATUS_OK;
	OniTOFFrame &phase = pAiFrame->frameSet.frames[0];
	phase.address = (XnUInt64)pAddr;
	//XnUInt8* frameDataAddr = (XnUInt8*)(pAiFrame->phase.address + sizeof(OniAIFrame));
	int phaseSize = phase.width * phase.height*sizeof(XnUInt16);

	for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
	{
		xnOSMemCopy((void*)(pAddr + phaseSize*i), m_pFrameGroup->frames[i]->data_offset, phaseSize);
	}

	//xnOSSaveFile("D:\\orbbecproject\\2020\\OpenniSDK\\kunlunshan\\openni2.3\\Bin\\Win32-Debug\\config\\plecoFrameData.raw", (void *)pAiFrame->phase.address, phaseSize*6);

	return  XN_STATUS_OK;
}

XnStatus XnMFTofSensor::setFrameResolution(int width, int height)
{
	m_phaseWidth = m_grabWidth = width;
	m_phaseHeight = height;
	m_grabHeight = (height + kGrabExtendedDataLine) * m_phaseInputCount;
	xnLogInfo(XN_MASK_MF_TOF_SENSOR, "setFrameResolution  width %d， height %d", width, height);
	return  XN_STATUS_OK;
}

