#include "XnPlecoTofSensor.h"
#include "OniCTypes.h"
#include "OpenNI.h"
#include"tofinfo.h"


XnPlecoTofSensor::XnPlecoTofSensor(mx6x_module_t *mx6xModule) :
    XnTofSensor(mx6xModule)
{
    m_sensorId = TOF_SENSOR_ID_PLECO;

    m_freqMode = TOF_DUAL_FREQUENCY_MODE;

    m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
    m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
    m_phaseCount = kDualFreqPhaseNum;
    m_bitPerPhasePixel = kPixelBit,

    m_grabWidth = m_phaseWidth;
    m_grabHeight = (kOriginalPhaseHeight + kGrabExtendedDataLine) * m_phaseCount;
    m_dothinMipiPackBit = kMipiPackBit;
}


XnPlecoTofSensor::~XnPlecoTofSensor() 
{
    m_configFilePathAndName = nullptr;
    m_calibFilePathAndName = nullptr;
}


XnStatus XnPlecoTofSensor::Init()
{
    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensor::CalcSensorProperty()
{
    switch (m_freqMode)
    {
    case TOF_SINGLE_FREQUENCY_MODE:
        m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
        m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
        m_phaseCount = kSingleFreqPhaseNum;
        m_grabWidth = m_phaseWidth;
        m_grabHeight = (m_phaseHeight + kGrabExtendedDataLine) * m_phaseCount;
        break;
    case TOF_DUAL_FREQUENCY_MODE:
    case TOF_AF_FREQUENCY_MODE:
        m_phaseWidth = kOriginalPhaseWidth / m_binningFactor;
        m_phaseHeight = kOriginalPhaseHeight / m_binningFactor;
        m_phaseCount = kDualFreqPhaseNum;
        m_grabWidth = m_phaseWidth;
        m_grabHeight = (m_phaseHeight + kGrabExtendedDataLine) * m_phaseCount;
        break;
    default:
        break;
    }
    xnLogInfo(XN_MASK_PLECO_TOF_SENSOR, "XnPlecoTofSensor CalcSensorProperty, m_freqMode %d, m_binningFactor %d, grab_w %d, grab_h %d", m_freqMode, m_binningFactor, m_grabWidth, m_grabHeight);
    
    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensor::GetDutyCyclePercentFormat(uint32_t freq, uint32_t ducy_cle, XnInt sensor_id, float *pduty_cycle_percent_format)
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


XnStatus XnPlecoTofSensor::GetIllumDutyCycleList(XnUInt8 mod_freq, XnFloat *duty_cycle_list)
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


XnStatus XnPlecoTofSensor::UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup)
{
    if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR, "pFrameGroup is nullptr");
        return  XN_STATUS_NULL_INPUT_PTR;
    }

    XnStatus ret;
    pFramegroup->frame_count = pCompressedFrameGroup->frame_count;
    pFramegroup->real_count = pCompressedFrameGroup->real_count;

    for (XnInt i = 0; i < pCompressedFrameGroup->real_count; ++i)
    {
        if (nullptr == pCompressedFrameGroup->frames[i] || nullptr == pFramegroup->frames[i])
        {
            xnLogError(XN_MASK_PLECO_TOF_SENSOR, "pFrame is nullptr, index: %d", i);
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
            break;
        case 12:
            ret = UnpackRaw12Data((XnUInt8*)inputFrame->data_offset, (XnUInt16*)outputFrame->data_offset, inputFrame->height, inputFrame->width);
            break;
        default:
            break;
        }
        if (XN_STATUS_OK != ret)
        {
            xnLogError(XN_MASK_PLECO_TOF_SENSOR, "UnpackCompressedFrameGroup fail %d", ret);
            return  ret;
        }
    }
    return  XN_STATUS_OK;
}


XnStatus XnPlecoTofSensor::SetIntegrationTime(uint32_t integration_time)
{
    XnStatus ret = SetProperty(INTEGRATION_TIME, &integration_time, sizeof(integration_time));
    return ret;
}


XnStatus XnPlecoTofSensor::SetDutyCycle(uint32_t cycle)
{
    XnStatus ret = SetProperty(DUTY_CYCLE, &cycle, sizeof(cycle));
    // LOG_DEBUG(" %s SetDutyCycle cycle = %d\n",__func__, cycle);
    return ret;
}


XnStatus XnPlecoTofSensor::SetTriggerSignal()
{
    XnStatus ret = SetProperty(SOFTWARE_TRIGGER, nullptr, 0);
    // LOG_DEBUG("OBC_TOF_SOFTWARE_TRIGGER ret= %d\n", ret);
    return ret;
}


XnStatus XnPlecoTofSensor::SetBinningFactor(XnUInt32 binningFactor)
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
        xnLogError(XN_MASK_PLECO_TOF_SENSOR, "SetBinningFactor invalid binningFactor: %d", binningFactor);
        return XN_STATUS_BAD_PARAM;
    }

    command_data_t  command_data;
    command_data.data = &bm;
    command_data.len = sizeof(XnUInt8);

    XnInt ret = m_pModule->set_property(BINNING_MODE, &command_data);
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR, "set_property BINNING_MODE fail %d", ret);
        return XN_STATUS_PROPERTY_NOT_SET;
    }

    ret = UpdateMode();
    XN_IS_STATUS_OK(ret);

    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensor::UpdateMode()
{
    XnStatus nRet = XN_STATUS_OK;

    nRet = XnTofSensor::UpdateMode();
    XN_IS_STATUS_OK(nRet);

    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensor::ParseExtendedData(ObTofFrameInfo* tof_frames, XnInt index)
{
    XnStatus ret = 0;
    tof_frames->mipi_pack = m_dothinMipiPackBit;
    tof_frames->width = m_phaseWidth;
    tof_frames->height = m_phaseHeight;
    tof_frames->frame_index = index % 3;
    tof_frames->group_index = 0;
    tof_frames->sensor_type = m_sensorId;

    uint32_t framesize = (tof_frames->width * tof_frames->height *tof_frames->mipi_pack) / kBitPerByte;
    // LOG_DEBUG("ParserExpandData framesize = %d,out=%d\n", framesize, cur_vmode_.out_mode);
    XnChar* expand_data = (XnChar*)tof_frames->buffer + framesize;

    XnUInt16 frame_counter = (XnUInt8)expand_data[3] | ((XnUInt8)expand_data[4] << kBitPerByte);
    XnUInt8  subframe_counter = (XnUInt8)expand_data[5];
    //LOG_DEBUG("frame counter = %d, subframe counter = %d\n", frame_counter, subframe_counter);
    // 扩展信息每一行的长度
    XnInt  line_size = m_phaseWidth * m_dothinMipiPackBit / kBitPerByte;
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
    return ret;
}
XnStatus XnPlecoTofSensor::CalcIRFramePleco(ObFrameGroup *p_phase_framegroup, uint16_t *p_IR_frame, int width, int height){
	if (nullptr == p_phase_framegroup->frames || nullptr == p_IR_frame) {
		return  -STATUS_NULL_INPUT_PTR;
	}
	//int width = 0;
	//int height = 0;
	//int ret = GetIRFrameSize(p_phase_framegroup, &width, &height);
	//OB_IS_STATUS_OK(ret);
	xnLogError(XN_MASK_PLECO_TOF_SENSOR, "CalcIRFramePleco width= %d, height=%d", width,height);
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
XnStatus XnPlecoTofSensor::GetIRFrameSize(const ObFrameGroup *p_phase_framegroup, int *width, int *height)
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
			   xnLogError(XN_MASK_PLECO_TOF_SENSOR, "unsupport sensor: %x", p_phase_framegroup->frames[0]->sensor_type);
			   return -STATUS_DEPTH_LIB_NO_SUPPORT;
	}
	}
	return STATUS_OK;
}
XnStatus XnPlecoTofSensor::GetFrameGroup(const XnChar *GrabBuffer, ObFrameGroup *p_framegroup)
{
    XnStatus ret = XN_STATUS_OK;
    if (nullptr == p_framegroup || nullptr == p_framegroup->frames)
    {
        return  XN_STATUS_ERROR;
    }
    if (p_framegroup->frame_count < m_phaseCount)
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
        p_framegroup->real_count = m_phaseCount;

        for (uint32_t i = 0; i < p_framegroup->real_count; ++i)
        {
            p_framegroup->frames[i]->data_offset = p_framegroup->frames[i]->buffer;
            memcpy(p_framegroup->frames[i]->buffer, &GrabBuffer[i*(phase_img_mipi_size + embedded_data_size)], embedded_data_size + phase_img_mipi_size);
            ret = ParseExtendedData(p_framegroup->frames[i], i);
            if (ret < 0)
            {
                xnLogError(XN_MASK_PLECO_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
                return  ret;
            }
            p_framegroup->frames[i]->out_mode = TOF_OUTPUT_MODE_3TAP;
            p_framegroup->frames[i]->phase_map = phase_info[i];
        }
        return XN_STATUS_OK;
    }

    else if (m_freqMode == TOF_DUAL_FREQUENCY_MODE || m_freqMode == TOF_AF_FREQUENCY_MODE)
    {
        p_framegroup->real_count = m_phaseCount;

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
                xnLogError(XN_MASK_PLECO_TOF_SENSOR, "ParserExpandData ret=%d  i= %d", ret, i);
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


XnStatus XnPlecoTofSensor::GetPhaseFrame(const XnUChar* dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
    if (nullptr == dothinInputBuf || nullptr == frameData
        || nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
        || nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR, "GetPhaseFrame fail, null input or output ptr");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);

    ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
        return ret;
    }

    XnUInt32 outputImageWidth, outputImageHeight, outputImageSize;
    GetOutputImageSize(&outputImageWidth, &outputImageHeight, &outputImageSize);

    //oniframe的data中包含metadata，这里需要更改帧数据地址，加上偏移量
    XnUChar *frameDataAddr = static_cast<XnUChar *>(static_cast<void*>(frameData)) + outputImageWidth * sizeof(XnUInt16) * m_pFrameGroup->real_count;

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


