#include "OpenNI.h"
#include "OniCTypes.h"
#include "XnPlecoTofSensorCalc.h"

//const XnChar *kConfigFilePathAndName = ".\\config\\filter_config_pleco.ini";
//const XnChar *kCalibFilePathAndNameDualShuffle = ".\\config\\DepthParams.bin";
//const XnChar *kCalibFilePathAndNameSingleShuffle = "..\\..\\Config\\DepthCalc\\DepthParams_pleco\\single_shuffle\\DepthParams.bin";


XnPlecoTofSensorCalc::XnPlecoTofSensorCalc(mx6x_module_t *mx6xModule) :
    XnPlecoTofSensor(mx6xModule)
{
    //m_configFilePathAndName = kConfigFilePathAndName;
    //m_calibFilePathAndName = kCalibFilePathAndNameDualShuffle;
}


XnPlecoTofSensorCalc::~XnPlecoTofSensorCalc() 
{
    m_configFilePathAndName = nullptr;
    m_calibFilePathAndName = nullptr;
}


XnStatus XnPlecoTofSensorCalc::UpdateFrameInfo(OniFrame &frame)
{
    //frame.sensorType = ONI_SENSOR_DEPTH;
    frame.frameIndex = m_pFrameGroup->frames[0]->group_index;

    frame.extraLine = m_pFrameGroup->real_count;

    frame.width = m_depthWidth;
    frame.stride = frame.width * sizeof(XnUInt16);
    frame.height = m_depthHeight;
    frame.dataSize = frame.width * frame.height * sizeof(XnUInt16) + frame.stride * m_pFrameGroup->real_count;

    XnUChar *pMetadata = nullptr;
    for (XnInt i = 0; i < m_pFrameGroup->real_count; ++i)
    {
        pMetadata = static_cast<XnUChar*>(frame.data) + frame.stride * i;
        SortMetaData(*m_pFrameGroup->frames[i], pMetadata);
    }

    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensorCalc::CalcSensorProperty()
{
    XnPlecoTofSensor::CalcSensorProperty();

    m_depthWidth = kVGADepthWidth / m_binningFactor;
    m_depthHeight = kVGADepthHeight / m_binningFactor;
//#if USE_FAKE_TOF_DEPTH_LIB
//	xnLogInfo(XN_MASK_PLECO_TOF_SENSOR_CALC, "engine ReleaseCaliParamsBuffer");
//    ReleaseCaliParamsBuffer(m_calibParamBuf);
//#else 
//	xnLogInfo(XN_MASK_PLECO_TOF_SENSOR_CALC, "fake ReleaseCaliParamsBuffer");
//#endif

  //  if (TOF_SINGLE_FREQUENCY_MODE == m_freqMode)
  //  { 
		//m_calibFilePathAndName = kCalibFilePathAndNameDualShuffle;
  //  }
  //  else
  //  {
  //      m_calibFilePathAndName = kCalibFilePathAndNameDualShuffle;
  //  }
  //  XnInt ret = LoadCaliParam();
    //if (XN_STATUS_OK != ret)
    //{
    //    xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "CalcSensorProperty, LoadCaliParam fail %d, path %s,", ret, m_calibFilePathAndName);
    //}
    xnLogInfo(XN_MASK_PLECO_TOF_SENSOR_CALC, "engine CalcSensorProperty, m_depthWidth %d, m_depthHeight %d", m_depthWidth, m_depthHeight);
    return XN_STATUS_OK;
}

XnStatus XnPlecoTofSensorCalc::UpdateMode()
{
	XnStatus nRet = XN_STATUS_OK;

	nRet = XnPlecoTofSensor::UpdateMode();
	XN_IS_STATUS_OK(nRet);

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
		xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "bad case m_binningFactor %d", m_binningFactor);
		return -1;
		break;
	}

	return XN_STATUS_OK;
}

XnStatus XnPlecoTofSensorCalc::UpdateCalcConfig()
{
    if (TOF_PLECO != m_modeConfig.device_type)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "ConfigFile wrong, device_type should be TOF_PLECO");
        return STATUS_DEPTH_LIB_NO_MATCH;
    }
    if (false == m_modeConfig.with_shuffle)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "ConfigFile wrong, TOF_PLECO, with_shuffle should not be false");
        return STATUS_DEPTH_LIB_NO_IMPLEMENT;
    }

    if (kOriginalPhaseWidth / m_phaseWidth > kVGADepthWidth / m_modeConfig.width)
    {
        xnLogError(XN_MASK_PLECO_TOF_SENSOR_CALC, "output width and height is too large, phase width*height is (%d * %d), output width*height is (%d * %d)",
            m_phaseWidth, m_phaseHeight, m_modeConfig.width, m_modeConfig.height);
        return STATUS_DEPTH_LIB_NO_MATCH;
    }
    //sensor模式改变后，传入深度计算接口的buffer大小需要更新
    if (m_phasePixelNum != m_phaseWidth * m_phaseHeight * m_phaseCount)
    {
        m_phasePixelNum = m_phaseWidth * m_phaseHeight * m_phaseCount;
        delete[] m_phaseBuf;
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
            m_phaseFrameIndex = { 0, 1, 2, 3, 4, 5 };
        }
        else
        {
            m_phaseFrameIndex = { 3, 4, 5, 0, 1, 2 };
        }
    }
    return XN_STATUS_OK;
}


XnStatus XnPlecoTofSensorCalc::GetOutputbufferSize(XnUInt32 *outputBufferWidth, XnUInt32 *outputBufferHeight, XnUInt32 *outputBufferSize)
{
	return this->XnTofSensor::GetOutputbufferSize(outputBufferWidth, outputBufferHeight, outputBufferSize);
    //*outputBufferWidth = m_depthWidth;
   // *outputBufferHeight = m_depthHeight + m_phaseCount;//每帧对应一行扩展信息
    //*outputBufferSize = *outputBufferWidth * *outputBufferHeight * sizeof(XnUInt16);

    //return  XN_STATUS_OK;
}


XnStatus XnPlecoTofSensorCalc::GetOutputImageSize(XnUInt32 *outputImageWidth, XnUInt32 *outputImageHeight, XnUInt32 *outputImageSize)
{
    *outputImageWidth = m_depthWidth;
    *outputImageHeight = m_depthHeight;
    *outputImageSize = *outputImageWidth * *outputImageHeight * sizeof(XnUInt16);

    return  XN_STATUS_OK;
}

