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
    m_phaseCount(0),
    m_bitPerPhasePixel(10),
    m_grabWidth(0),
    m_grabHeight(0),
    m_dothinMipiPackBit(10),
    m_configFilePathAndName(nullptr),
    m_calibFilePathAndName(nullptr),
    m_phaseBuf(nullptr),
    m_phasePixelNum(0),
    m_phaseFrameIndex(0)
{
    m_pModule = mx6xModule;
    m_pCompressedFrameGroup = new ObFrameGroup;
    m_pFrameGroup = new ObFrameGroup;
    MallocFrameGroupBuffer(m_pCompressedFrameGroup);
    MallocFrameGroupBuffer(m_pFrameGroup);

 //   m_calibParamBuf = nullptr;
	//m_configParamBuf = new XnChar[kOutputParamBufSize];

    memset(&m_modeConfig, 0, sizeof(Depth_Mode_Config));
    memset(&m_outputConfig, 0, sizeof(Output_Config));

    m_depthWidth = kVGADepthWidth / m_binningFactor;
    m_depthHeight = kVGADepthHeight / m_binningFactor;
    //m_depth = new XnFloat[kVGADepthWidth * kVGADepthHeight];
    //m_amplitude = new XnFloat[kVGADepthWidth * kVGADepthHeight];
    //m_intensity = new XnFloat[kVGADepthWidth * kVGADepthHeight];
    //m_tmpBuf = new XnFloat[kVGADepthWidth * kVGADepthHeight * kMaxFrequencyNum];
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
    //delete[] m_configParamBuf;
    //delete[] m_depth;
    //delete[] m_amplitude;
    //delete[] m_intensity;
    //delete[] m_tmpBuf;
    delete[] m_phaseBuf;
}


XnStatus XnTofSensor::Init()
{ 
    return XN_STATUS_NOT_IMPLEMENTED;
};


XnStatus XnTofSensor::MallocFrameGroupBuffer(ObFrameGroup *pFrameGroup)
{
    if (nullptr == pFrameGroup)
    {
        return  XN_STATUS_NULL_INPUT_PTR;
    }

    //对于带rk1608的情况，用深度的宽和高计算内存空间会偏小，故这里用定值
    const int maxFrameBufferSize = 1280 * 960 * 10;
    const XnUInt maxFrameCount = 8;
    pFrameGroup->frame_count = maxFrameCount;
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


XnStatus XnTofSensor::UnpackRaw10Data(XnUInt8 *in, XnUInt16 *out, XnInt width, XnInt height)
{
    if (nullptr == in || nullptr == out)
    {
        return XN_STATUS_NULL_INPUT_PTR;
    }

    int raw10Size = width * height * 10 / kBitPerByte;
    for (XnInt i = 0; i < raw10Size; i += 5)
    {
        const XnUInt8 *pStart = &in[i];
        XnUInt16 shift0 = pStart[0];
        XnUInt16 shift1 = pStart[1];
        XnUInt16 shift2 = pStart[2];
        XnUInt16 shift3 = pStart[3];
        XnUInt16 shift4 = pStart[4];

        out[0] = (shift0 << 2) | (shift4 & 0x03);
        out[1] = (shift1 << 2) | ((shift4 >> 2) & 0x03);
        out[2] = (shift2 << 2) | ((shift4 >> 4) & 0x03);
        out[3] = (shift3 << 2) | ((shift4 >> 6) & 0x03);

        out += 4;
    }
    return  XN_STATUS_OK;

}


XnStatus XnTofSensor::UnpackRaw12Data(XnUInt8 *in, XnUInt16 *out, XnInt width, XnInt height)
{
    if (nullptr == in || nullptr == out) 
    {
        return XN_STATUS_NULL_INPUT_PTR;
    }
    int raw12Size = width * height * 12 / kBitPerByte;

    for (XnInt i = 0; i < raw12Size; i += 6) 
    {
        const XnUInt8* pStart = &in[i];

        XnUInt16 shift0 = pStart[0];
        XnUInt16 shift1 = pStart[1];
        XnUInt16 shift2 = pStart[2];
        out[0] = (shift0 << 4) | (shift2 & 0xF);
        out[1] = (shift1 << 4) | ((shift2 >> 4) & 0xF);

        XnUInt16 shift3 = pStart[3];
        XnUInt16 shift4 = pStart[4];
        XnUInt16 shift5 = pStart[5];
        out[2] = (shift3 << 4) | (shift5 & 0xF);
        out[3] = (shift4 << 4) | ((shift5 >> 4) & 0xF);

        out += 4;
    }
    return  XN_STATUS_OK;
}


XnStatus XnTofSensor::SortMetaData(const ObTofFrameInfo &frameInfo, void *pMetaData)
{
    if (nullptr == pMetaData)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "pMetaData is nullptr");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    OniMetadata *phaseMetaData = static_cast<OniMetadata*>(pMetaData);
    phaseMetaData->dutyCycle[0] = frameInfo.duty_cycle;
    phaseMetaData->frameIndex = frameInfo.frame_index;
    phaseMetaData->type = (OniFrameType)frameInfo.frame_type;
    phaseMetaData->frequency[0] = frameInfo.frequency;
    phaseMetaData->groupIndex = frameInfo.group_index;
    phaseMetaData->height = frameInfo.height;
    phaseMetaData->integration[0] = frameInfo.integration_time;
    phaseMetaData->mode = (OniTOFSensorMode)frameInfo.out_mode;
    phaseMetaData->shuffle = 0x1B == frameInfo.phase_map ? true : false;
    phaseMetaData->id = (OniSensorID)frameInfo.sensor_type;
    phaseMetaData->temperDelay = frameInfo.time_delay_circuit_temp;
    phaseMetaData->temperRX = frameInfo.rx_temp;
    phaseMetaData->temperTX = frameInfo.driver_ic_temp;
    phaseMetaData->width = frameInfo.width;

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
        m_binningFactor = 4;
        break;
    default:

        m_binningFactor = 1;
    }
    return XN_STATUS_OK;
}

XnStatus XnTofSensor::UpdateFrequecyMode()
{
    int ret = XN_STATUS_OK;
    int size = 0;
    uint8_t freq_mode = 0;
    size = sizeof(uint8_t);

    ret = GetProperty(FREQUENCY_MODE, &freq_mode, &size);
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "FREQUENCY_MODE fail %d", ret);
        return ret;
    }
    m_freqMode = freq_mode;

    return XN_STATUS_OK;
}


XnStatus XnTofSensor::UnpackCompressedFrameGroup(ObFrameGroup *pCompressedFrameGroup, ObFrameGroup *pFramegroup)
{
    if (nullptr == pCompressedFrameGroup || nullptr == pFramegroup)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "pFrameGroup is nullptr");
        return  XN_STATUS_NULL_INPUT_PTR;
    }
	
    XnStatus ret=0;
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
	//float l_intrin[5];
	//float l_distort_coeff[5];
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
	if (nullptr == filePath)
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
	//ret = LoadToFFilterParam(const_cast<char*>(filePath), m_configParamBuf, m_modeConfig, m_outputConfig);
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "LoadToFConfig fail %d\n", ret);
        return ret;
    }
#else 
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine fake LoadToFFilterParam\n");
#endif
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine _CrtCheckMemory");
	assert(_CrtCheckMemory());
    return XN_STATUS_OK;
}


XnStatus XnTofSensor::LoadCaliParam(XnChar* filePath)
{
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
	//ret = LoadToFCaliParam(const_cast<char*>(filePath), m_calibParamBuf, m_modeConfig);
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "LoadToFCaliParam fail %d\n", ret);
        return ret;
    }
#else 
	xnLogInfo(XN_MASK_TOF_SENSOR, "engine fake LoadToFCaliParam\n");
#endif
    return XN_STATUS_OK;
}


XnStatus XnTofSensor::SetOutputConfig(Output_Config outputConfig)
{
    xnOSMemCopy(&m_outputConfig, &outputConfig, sizeof(Output_Config));
    return XN_STATUS_OK;
}


XnStatus XnTofSensor::ChangeFreqMode(ObFrequencyMode &mode)
{
    return XN_STATUS_NOT_IMPLEMENTED;
}


XnStatus XnTofSensor::UpdateFrameInfo(OniFrame *frame)
{
    frame->frameIndex = m_pFrameGroup->frames[0]->group_index;

	frame->extraLine = 0;

	frame->width = m_phaseWidth;
	frame->stride = frame->width * sizeof(XnUInt16);
	frame->height = m_phaseHeight * m_phaseCount;
	frame->dataSize = frame->width * frame->height * sizeof(XnUInt16)+frame->stride * m_pFrameGroup->real_count;

    XnUChar *pMetadata = nullptr;
    for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
    {
		pMetadata = static_cast<XnUChar*>(frame->data) + frame->stride * i;
        SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
    }

    return XN_STATUS_OK;
}


XnStatus XnTofSensor::UpdateMode()
{
    XnStatus ret = XN_STATUS_OK;

    ret = UpdateFrequecyMode();
    XN_IS_STATUS_OK(ret);

    ret = UpdateBinningFactor();
    XN_IS_STATUS_OK(ret);

    ret = CalcSensorProperty();
    XN_IS_STATUS_OK(ret);

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
XnStatus XnTofSensor::GetPhaseFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
    return XN_STATUS_NOT_IMPLEMENTED;

}
XnStatus XnTofSensor::GetIRFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
	if (nullptr == dothinInputBuf || nullptr == frameData
		|| nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
		|| nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "GetIRFrame fail, null input or output ptr");
		return XN_STATUS_NULL_INPUT_PTR;
	}
	XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);
	ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
	if (XN_STATUS_OK != ret)
	{
		xnLogError(XN_MASK_TOF_SENSOR, "UnpackCompressedFrameGroup fail, %d", ret);
		return ret;
	}
	ret = CalcIRFramePleco(m_pFrameGroup, frameData, kVGADepthWidth, kVGADepthHeight);
	//xnOSSaveFile(".\\irFrameData.raw", frameData, kVGADepthWidth * kVGADepthHeight * kBytePerDepthPixel);
	return  XN_STATUS_OK;
}

XnStatus XnTofSensor::GetDepthFrame(const XnUChar *dothinInputBuf, const XnUInt32 dothinInputBufSize, XnUInt16 *frameData, XnUInt32 *frameDataSize)
{
    if (nullptr == dothinInputBuf || nullptr == frameData
        || nullptr == m_pFrameGroup || nullptr == m_pFrameGroup->frames
        || nullptr == m_pCompressedFrameGroup || nullptr == m_pCompressedFrameGroup->frames)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "GetDepthFrame fail, null input or output ptr");
        return XN_STATUS_NULL_INPUT_PTR;
    }

    XnStatus ret = GetFrameGroup(reinterpret_cast<const XnChar*>(dothinInputBuf), m_pCompressedFrameGroup);
    ret = UnpackCompressedFrameGroup(m_pCompressedFrameGroup, m_pFrameGroup);
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
        memcpy(m_phaseBuf + m_phasePixelNum / m_pFrameGroup->real_count * i, m_pFrameGroup->frames[m_phaseFrameIndex[i]]->data_offset, m_phasePixelNum * sizeof(uint16_t) / m_pFrameGroup->real_count);
    }
	
#if USE_FAKE_TOF_DEPTH_LIB
    //ret = DepthCalc(m_phaseBuf, m_phasePixelNum,
    //    m_calibParamBuf, m_configParamBuf, m_tmpBuf,
    //    m_depth, m_intensity, m_amplitude,
    //    m_modeConfig, m_outputConfig);
#else
	xnLogError(XN_MASK_TOF_SENSOR, "engine fake DepthCalc fake");
#endif
	
    if (STATUS_OK != ret)
    {
        xnLogError(XN_MASK_TOF_SENSOR, "DepthCalc fail, %d", ret);
        return ret;
    }
	//for (int i = 0; i < kVGADepthWidth * kVGADepthHeight; i++){
	//	frameData[i] = (uint16_t)m_depth[i];
	//}
    //xnOSSaveFile(".\\DepthFrameData.raw", frameData, kVGADepthWidth * kVGADepthHeight * kBytePerDepthPixel);

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
    *grabBufferSize = *grabWidth * *grabHeight * m_bitPerPhasePixel / kBitPerByte;

    return  XN_STATUS_OK;
}


XnStatus XnTofSensor::GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize)
{
    *outputBufferWidth = m_phaseWidth;
    *outputBufferHeight = m_phaseHeight * m_phaseCount + m_phaseCount;//每帧对应一行扩展信息
    *outputBufferSize = *outputBufferWidth * *outputBufferHeight * sizeof(XnUInt16);

    return  XN_STATUS_OK;
}


XnStatus XnTofSensor::GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize)
{
    *outputImageWidth = m_phaseWidth;
    *outputImageHeight = m_phaseHeight * m_phaseCount;
    *outputImageSize = *outputImageWidth * *outputImageHeight * sizeof(XnUInt16);

    return  XN_STATUS_OK;
}

