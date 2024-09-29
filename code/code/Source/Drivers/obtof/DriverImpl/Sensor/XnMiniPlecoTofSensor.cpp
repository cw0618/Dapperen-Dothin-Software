#include "XnMiniPlecoTofSensor.h"
#include "OniCTypes.h"
#include "OpenNI.h"
#include"tofinfo.h"
#include "../Core/Mx6x/XnMx6xResolution.h"



XnMiniPlecoTofSensor::XnMiniPlecoTofSensor(mx6x_module_t *mx6xModule) :
	XnTofSensor(mx6xModule)
{
	m_sensorId = TOF_SENSOR_ID_MINI_PLECO;

	m_freqMode = TOF_DUAL_FREQUENCY_MODE;

	m_phaseWidth = kMiniOriginalPhaseWidth / m_binningFactor;
	m_phaseHeight = kMiniOriginalPhaseHeight / m_binningFactor;
	m_phaseCount = kMiniDualFreqPhaseNum;
	m_bitPerPhasePixel = kMiniPixelBit,

		m_grabWidth = m_phaseWidth;
	m_grabHeight = (kMiniOriginalPhaseHeight + kMiniGrabExtendedDataLine) * m_phaseCount;
	m_dothinMipiPackBit = kMiniMipiPackBit;
}


XnMiniPlecoTofSensor::~XnMiniPlecoTofSensor()
{
	m_configFilePathAndName = nullptr;
	m_calibFilePathAndName = nullptr;
}


XnStatus XnMiniPlecoTofSensor::Init()
{
	return XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::CalcSensorProperty()
{
	if (disableFrequencyMode) {

		for (int i = 0; i < sizeof(ob_res_arr) / sizeof(ob_res_arr[0]); i++) {
			if (m_curResIndex == ob_res_arr[i].key) {
				m_phaseWidth = ob_res_arr[i].vmode.width;
				m_phaseHeight = ob_res_arr[i].vmode.height / 6;
				break;
			}
		}

		m_phaseCount = kMiniDualFreqPhaseNum;
		m_grabWidth = m_phaseWidth;
		m_grabHeight = (m_phaseHeight + kMiniGrabExtendedDataLine) * m_phaseCount;

		xnLogInfo(XN_MASK_TOF_SENSOR, "XnMiniPlecoTofSensor CalcSensorProperty, m_freqMode %d, grab_w %d, grab_h %d", m_curResIndex, m_grabWidth, m_grabHeight);
	}
	else {


		switch (m_freqMode)
		{
		case TOF_SINGLE_FREQUENCY_MODE:
			if (m_binningFactor == 4) {
				m_phaseWidth = 192;
			}
			else {
				m_phaseWidth = kMiniOriginalPhaseWidth / m_binningFactor;
			}
			m_phaseHeight = kMiniOriginalPhaseHeight / m_binningFactor;

			if (m_shuffle) {
				m_phaseCount = kMiniSingleShuffleFreqPhaseNum;
			}
			else {
				m_phaseCount = kMiniSingleNonShuffleFreqPhaseNum;
			}
			m_phaseCount *= m_hdrFactor;
			m_grabWidth = m_phaseWidth;
			m_grabHeight = (m_phaseHeight + kMiniGrabExtendedDataLine) * m_phaseCount;
			break;
		case TOF_DUAL_FREQUENCY_MODE:
		case TOF_AF_FREQUENCY_MODE:
			if (m_binningFactor == 4) {
				m_phaseWidth = 192;
			}
			else {
				m_phaseWidth = kMiniOriginalPhaseWidth / m_binningFactor;
			}
			m_phaseHeight = kMiniOriginalPhaseHeight / m_binningFactor;

			if (m_shuffle) {
				m_phaseCount = kMiniDualShuffleFreqPhaseNum;
			}
			else {
				m_phaseCount = kMiniDualNonShuffleFreqPhaseNum;
			}
			m_phaseCount *= m_hdrFactor;

			m_grabWidth = m_phaseWidth;
			m_grabHeight = (m_phaseHeight + kMiniGrabExtendedDataLine) * m_phaseCount;
			break;
		default:
			break;
		}
		xnLogInfo(XN_MASK_TOF_SENSOR, "XnMiniPlecoTofSensor CalcSensorProperty, m_freqMode %d, m_binningFactor %d,m_shuffle %d,mHdrMode %d,m_phaseCount %d,mFrequencyOrder %d, grab_w %d, grab_h %d", m_freqMode, m_binningFactor, m_shuffle, m_hdrFactor, m_phaseCount, m_frequencyOrderFactor, m_grabWidth, m_grabHeight);

	}


	return XN_STATUS_OK;

}


XnStatus XnMiniPlecoTofSensor::GetDutyCyclePercentFormat(uint32_t freq, uint32_t ducy_cle, XnInt sensor_id, float *pduty_cycle_percent_format)
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


XnStatus XnMiniPlecoTofSensor::GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list)
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


XnStatus XnMiniPlecoTofSensor::UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup)
{
	if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "pFrameGroup is nullptr");
		return  XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret;
	pFramegroup->frame_count = pCompressedFrameGroup->frame_count;
	pFramegroup->real_count = pCompressedFrameGroup->real_count;

	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "unpackcompressframegroup: %d,%d", pFramegroup->frame_count, pFramegroup->real_count);

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
			ret = UnpackRaw10Data((XnUInt8*)inputFrame->data_offset, (XnUInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
			//	xnLogError(XN_MASK_TOF_SENSOR, "XnPlecoTofSensor UnpackRaw10Data***********");
			break;
		case 12:
			ret = UnpackRaw12Data((XnUInt8*)inputFrame->data_offset, (XnUInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
			//	xnLogError(XN_MASK_TOF_SENSOR, "XnTofSensorXnPlecoTofSensor UnpackRaw12Data***********");
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
	return  XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::SetIntegrationTime(uint32_t integration_time)
{
	XnStatus ret = SetProperty(INTEGRATION_TIME, &integration_time, sizeof(integration_time));
	return ret;
}


XnStatus XnMiniPlecoTofSensor::SetDutyCycle(uint32_t cycle)
{
	XnStatus ret = SetProperty(DUTY_CYCLE, &cycle, sizeof(cycle));
	// LOG_DEBUG(" %s SetDutyCycle cycle = %d\n",__func__, cycle);
	return ret;
}


XnStatus XnMiniPlecoTofSensor::SetTriggerSignal()
{
	XnStatus ret = SetProperty(SOFTWARE_TRIGGER, nullptr, 0);
	// LOG_DEBUG("OBC_TOF_SOFTWARE_TRIGGER ret= %d\n", ret);
	return ret;
}


XnStatus XnMiniPlecoTofSensor::SetBinningFactor(XnUInt32 binningFactor)
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
		xnLogError(XN_MASK_TOF_SENSOR, "SetBinningFactor invalid binningFactor: %d", binningFactor);
		return XN_STATUS_BAD_PARAM;
	}

	command_data_t  command_data;
	command_data.data = &bm;
	command_data.len = sizeof(XnUInt8);

	XnInt ret = m_pModule->set_property(BINNING_MODE, &command_data);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "set_property BINNING_MODE fail %d", ret);
		return XN_STATUS_PROPERTY_NOT_SET;
	}

	ret = UpdateMode();
	XN_IS_STATUS_OK(ret);

	return XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::UpdateMode()
{
	XnStatus nRet = XN_STATUS_OK;

	nRet = XnTofSensor::UpdateMode();
	XN_IS_STATUS_OK(nRet);

	return XN_STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::ParseExtendedData(ObTofFrameInfo* tof_frames, XnInt index, XnBool ifHdrMode, XnInt groupCount)
{
	XnStatus ret = 0;
	tof_frames->mipi_pack = m_dothinMipiPackBit;
	tof_frames->width = m_phaseWidth;
	tof_frames->height = m_phaseHeight;
	//tof_frames->frame_index = index % 3;
	tof_frames->frame_index = index;
	tof_frames->group_index = 0;
	tof_frames->sensor_type = m_sensorId;

	uint32_t framesize = (tof_frames->width * tof_frames->height *tof_frames->mipi_pack) / kBitPerByte;
	XnUChar* expand_data = (XnUChar*)tof_frames->buffer + framesize;

	xnLogInfo(XN_MASK_TOF_SENSOR, "sensor_type = %d", m_sensorId);
	//LOG_DEBUG("frame counter = %d, subframe counter = %d\n", frame_counter, subframe_counter);
	// 扩展信息每一行的长度
	XnInt  line_size = m_phaseWidth * m_dothinMipiPackBit / kBitPerByte;

	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "extend line size= %d", line_size);

#if 1
	XnUInt8 userId1 = expand_data[0];
	XnUInt8 userId2 = expand_data[1];
	XnUInt8 userId3 = expand_data[3];
	XnUInt8 userId4 = expand_data[4];
	XnUInt8 freqModeMg = expand_data[6];
	XnUInt8  subframe_counter = (XnUInt8)expand_data[7];
	xnLogInfo(XN_MASK_TOF_SENSOR, "subframe_counter= %d", subframe_counter);

	XnUInt16 valueH = (((XnUInt8)expand_data[70]) << 8);
	XnUInt8 valueL = (XnUInt8)expand_data[69];

	XnUInt16 frame_counter = (XnUInt8)expand_data[1 * line_size];
	xnLogInfo(XN_MASK_TOF_SENSOR, "frame_counter= %d", frame_counter);
	//tof_frames->group_index = (XnUInt8)frame_counter;
	tof_frames->group_index = frame_counter;

	tof_frames->sensor_temp = (valueH | valueL)*0.16f - 272;

	tof_frames->rx_temp = tof_frames->sensor_temp;

	valueH = (((XnUInt8)expand_data[1 * line_size + 36] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 37];
	tof_frames->driver_ic_temp = 25 + ((valueH | valueL) - 296) / 5.4f;

	valueH = (((XnUInt8)expand_data[1 * line_size + 46] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 54];
	XnUInt16 pd_h2 = (valueH | valueL);

	valueH = (((XnUInt8)expand_data[1 * line_size + 45] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 48];
	XnUInt16 pd_bg = (valueH | valueL);
	tof_frames->driver_ic_pd = (pd_h2 << 16) + pd_bg;

	tof_frames->driver_ic_error_flag = (XnUInt8)expand_data[1 * line_size + 58];
	{
		// 温度与png中的保持一致   损失部分精度
		XnUInt16 ir_temp = (XnUInt16)((tof_frames->rx_temp + 273.16) * 100);
		XnUInt16 vcsel_temp = (XnUInt16)((tof_frames->driver_ic_temp + 273.16) * 100);
		// LOG_DEBUG("tof_frames->driver_ic_temp = %f, --%f", tof_frames->driver_ic_temp, ((float)vcsel_temp / 10.0 - 273.16));
		tof_frames->sensor_temp = (float)((float)ir_temp / 100.0 - 273.16);
		tof_frames->driver_ic_temp = (float)((float)vcsel_temp / 100.0 - 273.16);
		tof_frames->rx_temp = tof_frames->sensor_temp;
	}
	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "RX temp transf:   %.6f, TX temp transf:   %.6f",
	//		tof_frames->rx_temp, tof_frames->driver_ic_temp);
	//LOG_DEBUG("RX temp transf:   %.6f, TX temp transf:   %.6f", 
	//tof_frames->rx_temp, tof_frames->driver_ic_temp);

	XnUInt8 pll2_clk_div_b = (XnUInt8)expand_data[13];
	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "index = %d,pll2_clk_div_b=%d", index, pll2_clk_div_b);

	/**
	single non  shuffle   1张   0
	hdr	single  non shuffle  2张  两个积分时间  0  1

	single shuffle  3张  012
	hdr single shuffle   6张  分两个积分时间  012  345

	dual non shuffle  2张  两个脉宽  0  1
	hdr dual non shuffle 4张 两个脉宽，两个积分时间  02    13

	dual shuffle  6张 两个脉宽  024   135
	hdr dual shuffle 12张 两个脉宽，两个积分时间    0246810     1357911

	*/


	bool useFreq0 = false;

	switch (m_freqMode) {
	case TOF_SINGLE_FREQUENCY_MODE:
		useFreq0 = true;
		break;
	case TOF_DUAL_FREQUENCY_MODE:
	case TOF_AF_FREQUENCY_MODE:
		if (groupCount <= 4) {
			if (index % 2 == 0) {
				useFreq0 = true;
			}
			else {
				useFreq0 = false;
			}
		}
		else {
			if (index < 3 || ((index >= 6) && ((index % 6)<3))) {
				useFreq0 = true;
			}
			else {
				useFreq0 = false;
			}
		}

	}

	//if (ifHdrMode) {

	//	switch (groupCount)
	//	{
	//	case 2:
	//	case 6:  //hdr single shuffle
	//		if (index %2==0) {
	//			useFreq0 = true;
	//		}
	//		else {
	//			useFreq0 = false;
	//		}
	//		break;
	//	case 4:
	//	case 12:
	//		if (index % 2 == 0) {
	//			useFreq0 = true;
	//		}
	//		else {
	//			useFreq0 = false;
	//		}
	//		break;
	//
	//	}


	//}
	//else {
	//	switch (groupCount)
	//	{
	//	case 1:
	//	case 3:
	//		useFreq0 = true;
	//		break;
	//	case 2:
	//	case 6:
	//		if (index %2==0) {
	//			useFreq0 = true;
	//		} else{
	//			useFreq0 = false;
	//		}
	//		break;
	//	}

	//}


	//if (index < 3 || ((index % 6)<3))
	//if (index == 0 || index == 2)
	if (useFreq0)
	{ // 

		tof_frames->frequency = (XnUInt8)expand_data[21] + 3;
		//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "index = %d,frequency=%d", index, expand_data[21]);
		uint16_t tempIntegrationTime = (XnUInt8)expand_data[15] | ((XnUInt8)expand_data[16] << 8);
		//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "index = %d,tempIntegrationTime=%d", index, tempIntegrationTime);
		//tof_frames->integration_time = tof_frames->frequency * 750 * tempIntegrationTime / 1000;
		tof_frames->integration_time = tempIntegrationTime;


		XnUInt8 duty = (XnUInt8)expand_data[24];
		tof_frames->duty_cycle = ((duty >> 3) > 0) ? ((duty >> 3)*0.2f) : (-(duty & 0x07)*0.2f);
		//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "F0: integration_time = %d, frequency = %d, duty_cycle = %f, duty = %d", tof_frames->integration_time, tof_frames->frequency, tof_frames->duty_cycle, duty);
	}
	//	else if (index == 1 || index == 3)
	else
	{ // F1
		tof_frames->frequency = (XnUInt8)expand_data[22] + 3;
		//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "index = %d,frequency=%d", index, expand_data[22]);
		uint16_t tempIntegrationTime = (XnUInt8)expand_data[18] | ((XnUInt8)expand_data[19] << 8);
		//xnLogError(XN_MASK_PLECO_TOF_SENSOR, "index = %d,tempIntegrationTime=%d", index, tempIntegrationTime);
		//	tof_frames->integration_time = tof_frames->frequency * 750 * tempIntegrationTime / 1000;
		tof_frames->integration_time = tempIntegrationTime;


		XnUInt8 duty = (XnUInt8)expand_data[25];
		tof_frames->duty_cycle = ((duty >> 3) > 0) ? ((duty >> 3)*0.2f) : (-(duty & 0x07)*0.2f);
		//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "F1: integration_time = %d, frequency = %d, duty_cycle = %f, duty = %d", tof_frames->integration_time, tof_frames->frequency, tof_frames->duty_cycle, duty);
	}



#else
	XnUInt16 frame_counter = (XnUInt8)expand_data[3] | ((XnUInt8)expand_data[4] << kBitPerByte);
	XnUInt8  subframe_counter = (XnUInt8)expand_data[5];
	XnUInt16 valueH = (((XnUInt8)expand_data[2]) << 8);
	XnUInt8 valueL = (XnUInt8)expand_data[1];

	tof_frames->group_index = (XnUInt8)frame_counter;
	//        LOG_DEBUG("RX valueH:   %d", valueH);
	//        LOG_DEBUG("RX valueL:   %d", valueL);
	tof_frames->sensor_temp = (valueH | valueL)*0.16f - 272;

	tof_frames->rx_temp = tof_frames->sensor_temp;

	valueH = (((XnUInt8)expand_data[1 * line_size + 24] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 25];
	tof_frames->driver_ic_temp = 25 + ((valueH | valueL) - 296) / 5.4f;
	//LOG_DEBUG("RX temp:   %.6f, TX temp:   %.6f", 
	//tof_frames->rx_temp, tof_frames->driver_ic_temp);

	//LOG_DEBUG("ParserExpandData H:%x.L%x    tof_frames->driver_ic_temp= %f,tof_frames->sensor_temp=%f\n",
	//valueH, valueL,tof_frames->driver_ic_temp, tof_frames->sensor_temp);

	valueH = (((XnUInt8)expand_data[1 * line_size + 31] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 36];
	XnUInt16 pd_h2 = (valueH | valueL);

	valueH = (((XnUInt8)expand_data[1 * line_size + 30] & 0x03) << 8);
	valueL = (XnUInt8)expand_data[1 * line_size + 32];
	XnUInt16 pd_bg = (valueH | valueL);
	tof_frames->driver_ic_pd = (pd_h2 << 16) + pd_bg;
	//LOG_DEBUG("PLECO  pd_h2: %d, pd_bg: %d , %d, %d\r\n", pd_h2, pd_bg, valueH, valueL);		

	tof_frames->driver_ic_error_flag = (XnUInt8)expand_data[1 * line_size + 39];
	{
		// 温度与png中的保持一致   损失部分精度
		XnUInt16 ir_temp = (XnUInt16)((tof_frames->rx_temp + 273.16) * 100);
		XnUInt16 vcsel_temp = (XnUInt16)((tof_frames->driver_ic_temp + 273.16) * 100);
		// LOG_DEBUG("tof_frames->driver_ic_temp = %f, --%f", tof_frames->driver_ic_temp, ((float)vcsel_temp / 10.0 - 273.16));
		tof_frames->sensor_temp = (float)((float)ir_temp / 100.0 - 273.16);
		tof_frames->driver_ic_temp = (float)((float)vcsel_temp / 100.0 - 273.16);
		tof_frames->rx_temp = tof_frames->sensor_temp;
	}
	//LOG_DEBUG("RX temp transf:   %.6f, TX temp transf:   %.6f", 
	//tof_frames->rx_temp, tof_frames->driver_ic_temp);

	if (index < 3)
	{ // F0
		tof_frames->integration_time = (XnUInt8)expand_data[10] | ((XnUInt8)expand_data[11] << 8);
		tof_frames->frequency = (XnUInt8)expand_data[14] + 3;
		XnUInt8 duty = (XnUInt8)expand_data[16];
		tof_frames->duty_cycle = ((duty >> 3) > 0) ? ((duty >> 3)*0.2f) : (-(duty & 0x07)*0.2f);
		//LOG_DEBUG("F0: integration_time = %d, frequency = %d, duty_cycle = %f, duty = %d\n", tof_frames->integration_time, tof_frames->frequency, tof_frames->duty_cycle, duty);
	}
	else
	{ // F1
		tof_frames->integration_time = (XnUInt8)expand_data[12] | ((XnUInt8)expand_data[13] << 8);
		tof_frames->frequency = (XnUInt8)expand_data[15] + 3;
		XnUInt8 duty = (XnUInt8)expand_data[17];
		tof_frames->duty_cycle = ((duty >> 3) > 0) ? ((duty >> 3)*0.2f) : (-(duty & 0x07)*0.2f);
		//LOG_DEBUG("F1: integration_time = %d, frequency = %d, duty_cycle = %f, duty = %d\n", tof_frames->integration_time, tof_frames->frequency, tof_frames->duty_cycle, duty);
	}
#endif
	return ret;
}
XnStatus XnMiniPlecoTofSensor::CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height) {
	if (nullptr == p_phase_framegroup->frames || nullptr == p_IR_frame) {
		return  -STATUS_NULL_INPUT_PTR;
	}
	//int width = 0;
	//int height = 0;
	//int ret = GetIRFrameSize(p_phase_framegroup, &width, &height);
	//OB_IS_STATUS_OK(ret);
	//xnLogError(XN_MASK_PLECO_TOF_SENSOR, "CalcIRFramePleco width= %d, height=%d ,groupCount=%d", width, height, p_phase_framegroup->real_count);
	memset(p_IR_frame, 0, width *height * 2);

	for (uint32_t index = 0; index < p_phase_framegroup->real_count; index++) {
		uint16_t *frame = (uint16_t*)p_phase_framegroup->frames[index]->data_offset;
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				p_IR_frame[i*width + j] += frame[i * 3 * width + j * 3]
					+ frame[i * 3 * width + j * 3 + 1]
					+ frame[i * 3 * width + j * 3 + 2];
			}
		}
	}

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			p_IR_frame[i*width + j] = (uint16_t)((p_IR_frame[i*width + j] / (p_phase_framegroup->real_count * 3)));
		}
	}
	return STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::CalcAmplitudeFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_Amplitude_frame, int width, int height) {
	
	return STATUS_OK;
}

XnStatus XnMiniPlecoTofSensor::GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height)
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
		*width = 640;
		*height = 480;
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
		xnLogError(XN_MASK_TOF_SENSOR, "unsupport sensor: %x", p_phase_framegroup->frames[0]->sensor_type);
		return -STATUS_DEPTH_LIB_NO_SUPPORT;
	}
	}
	return STATUS_OK;
}


XnStatus XnMiniPlecoTofSensor::GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup)
{
	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "GetFrameGroup**********************************");
	XnStatus ret = XN_STATUS_OK;
	if (nullptr == p_framegroup || nullptr == p_framegroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "p_framegroup or  p_framegroup->frames is null");
		return  XN_STATUS_ERROR;
	}
	if (p_framegroup->frame_count < m_phaseCount)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "p_framegroup  frame_count=%d,m_phaseCount=%d", p_framegroup->frame_count, m_phaseCount);
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
	const XnUInt8 phase_info[12] = { 0, 240, 120, 0, 240, 120,0,240,120,0,240,120 };
	const XnUInt8 m_phase_index[2][12] = { { 0, 2, 4, 1, 3, 5,6,8,10,7,9,11 }, //F0F1F0F1F0F1
	{ 0, 1, 2, 3, 4, 5,6,7,8,9,10,11 }  //F0F0F0F1F1F1
	};


	XnInt phase_img_mipi_size = m_phaseWidth * m_phaseHeight * m_dothinMipiPackBit / kBitPerByte;
	XnInt embedded_data_size = m_phaseWidth * kMiniGrabExtendedDataLine * m_dothinMipiPackBit / kBitPerByte;

	//	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "phase_img_mipi_size  %d,embedded_data_size =%d", phase_img_mipi_size, embedded_data_size);
	//LOG_DEBUG("phase_img_mipi_size  %d,embedded_data_size =%d", phase_img_mipi_size, embedded_data_size);

	if (m_freqMode == TOF_SINGLE_FREQUENCY_MODE)
	{
		p_framegroup->real_count = m_phaseCount;

		for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
		{
			p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
			memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[i*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
			ret = ParseExtendedData(p_framegroup->frames[i], i, (m_hdrFactor == 2) ? true : false, p_framegroup->real_count);
			if (ret < 0)
			{
				xnLogError(XN_MASK_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
				return  ret;
			}
			//p_framegroup->frames[i]->out_mode = TOF_OUTPUT_MODE_3TAP;
			if (!m_shuffle) {
				p_framegroup->frames[i]->out_mode = ONI_PLECO_3TAP_SINGLE_FREQ_1FRAME;
			}
			else {
				p_framegroup->frames[i]->out_mode = ONI_PLECO_3TAP_SINGLE_FREQ_3FRAME;
			}

			p_framegroup->frames[i]->phase_map = phase_info[i];
		}
		return XN_STATUS_OK;
	}

	else if (m_freqMode == TOF_DUAL_FREQUENCY_MODE || m_freqMode == TOF_AF_FREQUENCY_MODE)
	{
		p_framegroup->real_count = m_phaseCount;

		if (!m_shuffle) {
			p_framegroup->real_count = m_phaseCount;

			for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
			{
				p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[i*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				ret = ParseExtendedData(p_framegroup->frames[i], i, true, p_framegroup->real_count);
				if (ret < 0)
				{
					xnLogError(XN_MASK_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
					return  ret;
				}
				//p_framegroup->frames[i]->out_mode = TOF_OUTPUT_MODE_3TAP;
				p_framegroup->frames[i]->out_mode = ONI_PLECO_3TAP_DUAL_FREQ_2FRAME;
				p_framegroup->frames[i]->phase_map = phase_info[i];
			}
		}
		else {

			int cur_phase_index = 0;
			//shuffler
			if (0 == m_frequencyOrderFactor) {
				cur_phase_index = 0;
			}
			else if (1 == m_frequencyOrderFactor) {
				cur_phase_index = 1;
			}


			for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
			{

				p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
				memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[(m_phase_index[cur_phase_index][i])*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
				//		 xnLogError(XN_MASK_PLECO_TOF_SENSOR, "dual shuffle **************************************** cur_phase_index=%d  m_phase_index[cur_phase_index][i]= %d", cur_phase_index, m_phase_index[cur_phase_index][i]);
				//memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[i*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);

				ret = ParseExtendedData(p_framegroup->frames[i], i, false, p_framegroup->real_count);
				if (ret < 0)
				{
					xnLogError(XN_MASK_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
					return  ret;
				}
				p_framegroup->frames[i]->out_mode = ONI_PLECO_3TAP_DUAL_FREQ_6FRAME;
				//p_framegroup->frames[i]->out_mode = TOF_OUTPUT_MODE_3TAP;
				p_framegroup->frames[i]->phase_map = phase_info[i];
			}


		}

		return  XN_STATUS_OK;
	}
	else
	{
		return XN_STATUS_ERROR;
	}
}


XnStatus XnMiniPlecoTofSensor::GetPhaseFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	if (nullptr == dothinInputBuf || nullptr == frameData
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "GetPhaseFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

	ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
		return ret;
	}

	XnUInt32 outputImageWidth, outputImageHeight, outputImageSize;
	GetOutputImageSize(&outputImageWidth, &outputImageHeight, &outputImageSize);

	//oniframe的data中包含metadata，这里需要更改帧数据地址，加上偏移量
	XnUChar *frameDataAddr = static_cast<XnUChar *>(static_cast<void*>(frameData)) + outputImageWidth * sizeof(XnUInt16)* m_pFrameGroup->real_count;

	//char postfix[100];
	for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
	{
		xnOSMemCopy(frameDataAddr + outputImageSize / m_phaseCount * i, m_pFrameGroup->frames[i]->data_offset, outputImageSize / m_phaseCount);

		/*show each phase frame
		std::string phaseStr = ".\\phase.raw";
		sprintf(postfix, "_%d", i);
		phaseStr.insert(phaseStr.size() - 4, postfix);
		xnOSSaveFile(phaseStr.c_str(), m_pFrameGroup->frames[i]->data_offset, outputImageSize / m_phaseCount);
		*/
	}

	//xnOSSaveFile(".\\plecoFrameData.raw", frameDataAddr, outputImageSize);

	return  XN_STATUS_OK;
}
XnStatus XnMiniPlecoTofSensor::GetAIFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, OniAIFrame* pAiFrame)
{
	if (nullptr == dothinInputBuf || nullptr == pAiFrame
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "GetPhaseFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}

	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

	ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
		return ret;
	}

	//XnUInt32 outputImageWidth, outputImageHeight, outputImageSize;
	//GetOutputImageSize(&outputImageWidth, &outputImageHeight, &outputImageSize);

	//oniframe的data中包含metadata，这里需要更改帧数据地址，加上偏移量



	XnUInt8* pAddr = (XnUInt8*)pAiFrame + sizeof(OniAIFrame);
	pAiFrame->frameSet.size = 1;
	pAiFrame->frameSet.status = ONI_AI_STATUS_OK;
	OniTOFFrame &phase = pAiFrame->frameSet.frames[0];
	phase.width = m_phaseWidth;
	phase.height = m_phaseHeight;
	phase.address = (XnUInt64)pAddr;
	//XnUInt8* frameDataAddr = (XnUInt8*)(pAiFrame->phase.address + sizeof(OniAIFrame));
	int phaseSize = phase.width * phase.height*sizeof(XnUInt16);
	//xnLogError(XN_MASK_PLECO_TOF_SENSOR, "XnPlecoTofSensor::GetAIFrame phaseSize  %d,width=%d,height=%d,%d", phaseSize, phase.width, phase.height, m_pFrameGroup->real_count);
	for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
	{
		xnOSMemCopy((void*)(pAddr + phaseSize*i), m_pFrameGroup->frames[i]->data_offset, phaseSize);

		//show each phase frame
		/*std::string phasestr = ".\\phase.raw";
		std::string phasestr1 = ".\\phase1.raw";*/
		/*		sprintf(postfix, "_%d", i);
		phasestr.insert(phasestr.size() - 4, postfix);
		xnossavefile(phasestr.c_str(), m_pframegroup->frames[i]->data_offset, phasesize);*/
		/*if (i == 1) {
		xnOSSaveFile(phasestr.c_str(), m_pFrameGroup->frames[i]->data_offset, phaseSize);

		}*/



	}

	//计算AE
return	AeAlgorithm(phase.width, phase.height);
#if 0
	//判断ae是否打开了
	bool ae_status = 0;
	int size = sizeof(uint8_t);
	XnStatus status = GetProperty(OBC_AE, &ae_status, &size);
	//	ae_status = 1;
	if (status == XN_STATUS_OK&&ae_status) {
		xnLogInfo(XN_MASK_TOF_SENSOR, "ae_status = %d", ae_status);
		ret = UpdateCalcConfig();
		int src_phase_size = phase.width*phase.height;
		int phasePixelNum = phase.width*phase.height*m_pFrameGroup->real_count;
		int integration_time = m_pFrameGroup->frames[0]->integration_time;
		xnLogInfo(XN_MASK_TOF_SENSOR, "integration_time = %d,phaseSize=%d,width=%d,height=%d,phasePixelNum=%d,sizeof(uint16_t)=%d", integration_time, phaseSize, phase.width, phase.height, phasePixelNum, sizeof(uint16_t));
		for (int i = 0; i < m_pFrameGroup->real_count; ++i)
		{
			xnLogInfo(XN_MASK_TOF_SENSOR, "m_phasePixelNum = %d", m_phasePixelNum);
			memcpy(m_phaseBuf + m_phasePixelNum / m_pFrameGroup->real_count * i, m_pFrameGroup->frames[i]->data_offset, m_phasePixelNum * sizeof(uint16_t) / m_pFrameGroup->real_count);
		}


		xnLogInfo(XN_MASK_TOF_SENSOR, "hConfig_flood = %d", hConfig_flood);
#if USE_FAKE_TOF_DEPTH_LIB
		//	if (NULL != hConfig_flood) {

		int calc_integration_time = ExposureTimeCalcWithConfig(integration_time, m_phaseBuf, hConfig_flood);
		xnLogInfo(XN_MASK_TOF_SENSOR, "integration_time = %d,calc_integration_time=%d", integration_time, calc_integration_time);
		SetIntegrationTime(calc_integration_time);

		/*}
		else {
		xnLogError(XN_MASK_TOF_SENSOR, "hConfig_flood is null");
		char * fileName = "D:/tf/OrbbecToolkit_latest/OrbbecToolKit_Inuitive/config/filter_config_pleco_flood.ini";
		hConfig_flood = LoadToFFilterParam(fileName);
		}*/
#endif
	}
#endif


	//xnOSSaveFile("D:\\orbbecproject\\2020\\OpenniSDK\\kunlunshan\\openni2.3\\Bin\\Win32-Debug\\config\\plecoFrameData.raw", (void *)pAiFrame->phase.address, phaseSize*6);

	//return  XN_STATUS_OK;
}

//XnStatus XnMiniPlecoTofSensor::AeAlgorithm(int width,int height) {
//#if 1
//	//判断ae是否打开了
//	int ret = 0;
//	bool ae_status = 0;
//	int size = sizeof(uint8_t);
//	XnStatus status = GetProperty(OBC_AE, &ae_status, &size);
//	if (status == XN_STATUS_OK&&ae_status) {
//		xnLogInfo(XN_MASK_TOF_SENSOR, "ae_status = %d", ae_status);
//		ret = UpdateCalcConfig();
//		int src_phase_size = width*height;
//		int phasePixelNum = width*height*m_pFrameGroup->real_count;
//		int integration_time = m_pFrameGroup->frames[0]->integration_time;
//
//		for (int i = 0; i < m_pFrameGroup->real_count; ++i)
//		{
//			xnLogInfo(XN_MASK_TOF_SENSOR, "m_phasePixelNum = %d", m_phasePixelNum);
//			memcpy(m_phaseBuf + m_phasePixelNum / m_pFrameGroup->real_count * i, m_pFrameGroup->frames[i]->data_offset, m_phasePixelNum * sizeof(uint16_t) / m_pFrameGroup->real_count);
//		}
//
//
//		xnLogInfo(XN_MASK_TOF_SENSOR, "hConfig_flood = %d", hConfig_flood);
//#if USE_FAKE_TOF_DEPTH_LIB
//			if (NULL != hConfig_flood) {
//
//		int calc_integration_time = ExposureTimeCalcWithConfig(integration_time, m_phaseBuf, hConfig_flood);
//		xnLogInfo(XN_MASK_TOF_SENSOR, "integration_time = %d,calc_integration_time=%d", integration_time, calc_integration_time);
//		SetIntegrationTime(calc_integration_time);
//
//		}
//		else {
//		xnLogError(XN_MASK_TOF_SENSOR, "hConfig_flood is null");
//		return XN_STATUS_ERROR;
//	
//		}
//#endif
//	}
//#endif
//
//
//	//xnOSSaveFile("D:\\orbbecproject\\2020\\OpenniSDK\\kunlunshan\\openni2.3\\Bin\\Win32-Debug\\config\\plecoFrameData.raw", (void *)pAiFrame->phase.address, phaseSize*6);
//
//	return  XN_STATUS_OK;
//}

XnStatus XnMiniPlecoTofSensor::UpdateCalcConfig()
{
	/* if (TOF_PLECO != m_modeConfig.device_type)
	{
	xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "ConfigFile wrong, device_type should be TOF_PLECO");
	return STATUS_DEPTH_LIB_NO_MATCH;
	}*/
	/*if (false == m_modeConfig.with_shuffle)
	{
	xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "ConfigFile wrong, TOF_PLECO, with_shuffle should not be false");
	return STATUS_DEPTH_LIB_NO_IMPLEMENT;
	}*/

	//if (kOriginalPhaseWidth / m_phaseWidth > kVGADepthWidth / m_modeConfig.width)
	//{
	//    xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "output width and height is too large, phase width*height is (%d * %d), output width*height is (%d * %d)",
	//        m_phaseWidth, m_phaseHeight, m_modeConfig.width, m_modeConfig.height);
	//    return STATUS_DEPTH_LIB_NO_MATCH;
	//}
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
		}
		else
		{
			m_phaseFrameIndex = { 3, 4, 5, 0, 1, 2,9,10,11,6,7,8 };
		}
	}
	return XN_STATUS_OK;
}

