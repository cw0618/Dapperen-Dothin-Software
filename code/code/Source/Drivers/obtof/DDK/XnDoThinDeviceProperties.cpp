#include "XnDothinDeviceProperties.h"
#include "OBCProperties.h"
#include"PS1080.h"
#include <iostream>


#define XN_MASK_DOTIN_DEVICE_PPS "DothinDeviceProperties" 
#define XN_OBC_STR_MAX_LEN 64 

using namespace std;


XnDothinDeviceProperties::XnDothinDeviceProperties(XnMx6xModulesHelper* mx6xMudules_Helper) :m_pMudulesHelper(mx6xMudules_Helper)
{

}


XnStatus XnDothinDeviceProperties::isPropertySupported(XnInt propertyId)
{
	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);

	switch (propertyId)
	{
		case OBC_SOFT_FILTER:
			return XN_STATUS_OK;
		default:
			break;
	}

	XnStatus status = m_pMudulesHelper->IsCommandPropertySupport((hw_command_t)propertyId);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnStatus  XnDothinDeviceProperties::getProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnStatus status = XN_STATUS_OK;

	switch (propertyId)
	{
	case OBC_MODULE_VERSION:
		return getPropertyModuleVersion(propertyId, data, pDataSize);

	case OBC_FW_VERSION:
		return getPropertyFirmwareVersion(propertyId, data, pDataSize);

	case OBC_EEPROM_RW:
		return getPropertyEepromData(propertyId, data, pDataSize);

	case OBC_REG_RW:
		if (*pDataSize != sizeof(obc_reg_map))
		{
			return XN_STATUS_INVALID_BUFFER_SIZE;
		}

		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(propertyId, data, pDataSize);
		XN_IS_STATUS_OK(status);
		break;
	case OBC_LDMP_DOUBLE:
		return getPropertyLdmpDpuble(propertyId, data, pDataSize);

	case OBC_MODULE_NAME:
	case OBC_AE_ENABLE:
	case OBC_IR_EXP:
	case OBC_IR_GAIN:
	case OBC_PULSE_WIDTH:
	case OBC_FLOOD_LED:
	case OBC_LASER:
	case OBC_IR_SN:
	case OBC_LDMP_SN:
	case OBC_LASER_CURRENT:
	case OBC_FLOOD_CURRENT:
	case OBC_PACK_FMT:
	case OBC_DISP_BITS:
	case OBC_DISP_SUB_BITS:
	case OBC_DEPTH_ROTATION_TYPE:
	case OBC_DEPTH_MIRROR_TYPE:
    case OBC_ENGINE_ID:
	case OBC_CAMERA_PARAMS:
	case OBC_IR_TEMP:
	case OBC_LDMP_TEMP:
	case OBC_ENGINE_NAME:
	case OBC_NTC_COEFF:
	case OBC_DEP_RECTIFY:
	case OBC_AE_AVG:
	case OBC_DATA_FMT:
	case OBC_ENGINE_IS_VALID:

    case OBC_SUPPORT_VIDEO_MODES:
    case OBC_SENSOR_TEMP:
    case OBC_RX_TEMP:
    case OBC_TX_TEMP:
    case OBC_ILLUM_POWER:
    case OBC_ILLUM_POWER_CTL:
    case OBC_INTEGRATION_TIME:
    case OBC_MODULATION_FREQUENCY:
    case OBC_DATA_OUTPUT_MODE:
    case OBC_DUTY_CYCLE:
    case OBC_MIRROR_FLIP:
    case OBC_TEST_PATTERN:
    case OBC_SOFTWARE_TRIGGER:
    case OBC_HARDWARE_TRIGGER:

    case OBC_SENSOR_ID:
    case OBC_SENSOR_INFO:
    case OBC_DUTY_CYCLE_LIST:
    case OBC_VCSEL_PD:
    case OBC_ILLUM_POWER_TEST:
    case OBC_DEFAULT_PARAMS:
    case OBC_OPS_PTR:
    case OBC_AE:
    case OBC_DRIVER_IC_DETECT:
    case OBC_REG16_RW:
    case OBC_DELAY_CIRCUIT_TIME:
    case OBC_DELAY_CIRCUIT_ID:
    case OBC_TX_A_B_POWER:
    case OBC_SENSOR_FREQUENCY_DUTY_CONFIG:
    case OBC_SHUFFLE:
    case OBC_CHIP_ID:
    case OBC_BINNING_MODE:
    case OBC_BURST_MODE:
    case OBC_FREQUENCY_MODE:
    case OBC_ITO_EM:
    case OBC_DRIVER_IC_REG8_RW:
    case OBC_VMGHI_VOLTAGE:
    case OBC_AF_DEPTH:
	case OBC_DELAY_POWER:
	case OBC_DELAY_FW_VERSION:
	case OBC_DELAY_DELAY_SWITCH:
	case OBC_DELAY_DELAY_TIME:
	case OBC_DELAY_TEMPERATURE:
	case OBC_PHASE_COUNT:
	case OBC_WINDOW_ORIGINY:
	case OBC_WINDOW_ORIGINX:
	case OBC_WINDOW_HEIGHT:
	case OBC_WINDOW_WIDTH:
	case OBC_SUB_SAMP:
	case OBC_DEEPSLEEP_MODE:
	case OBC_HDR_ALGORITHM:
	case OBC_HIST_ALGORITHM:
	case OBC_MEDIAN_ALGORITHM:
	case OBC_SUB_SAMP_V:
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(propertyId, data, pDataSize);
		XN_IS_STATUS_OK(status);
		break;

	case OBC_EXT_DEP_PREFILTER:
	case OBC_EXT_DEP_PREFILTER_LEVEL:
	case OBC_EXT_DEP_SMFILTER:
	case OBC_EXT_DEP_THRFILTER:
	case OBC_EXT_DEP_UNIDIV:
	case OBC_EXT_DEP_PTFILTER:
	case OBC_EXT_DEP_ENGINE:
	case OBC_EXT_GPM_STATUS:
	case OBC_EXT_GPM:
	case OBC_EXT_GPM_IS_ENABLE:

		return OBCExtension_GetProperty(propertyId, data, pDataSize);
	case XN_MODULE_PROPERTY_AE:
	case XN_MODULE_PROPERTY_EMITTER_STATE:
	case XN_MODULE_PROPERTY_EEPROM_STATE:
		if (propertyId == XN_MODULE_PROPERTY_AE)
		{
			propertyId = OBC_AE;
		}
		else if (propertyId == XN_MODULE_PROPERTY_EMITTER_STATE)
		{
			propertyId = OBC_ILLUM_POWER;
		}
		else if (propertyId == XN_MODULE_PROPERTY_EEPROM_STATE)
		{
			propertyId = OBC_EEPROM_RW;
			if (*pDataSize != sizeof(eeprom_data_t)) {
				return -XN_STATUS_ERROR;
			}
		}
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(propertyId, data, pDataSize);
		XN_IS_STATUS_OK(status);
		break;
	case OBC_GROUP_FRAME_MODE:
		status = getGroupFrameMode((ObcTofFrameMode *)data,*pDataSize);
		XN_IS_STATUS_OK(status);
		break;
	default:
		xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "This property getting is not supported!");
		return XN_STATUS_NO_MATCH;
	}
	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::setProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	XnStatus status = XN_STATUS_OK;
	switch (propertyId)
	{
	case OBC_LOAD_FW:
		return PropertyLoadFirmware(propertyId, (void*)data, dataSize);

	case OBC_LOAD_REF:
		return PropertyLoadReference(propertyId, (void*)data, dataSize);

	case OBC_EEPROM_RW:
		return setPropertyEepromData(propertyId, (void*)data, dataSize);

	case OBC_REG_RW:
		if (dataSize != sizeof(obc_reg_map))
		{
			return XN_STATUS_INVALID_BUFFER_SIZE;
		}

		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandSetProperty(propertyId, (void*)data, dataSize);
		XN_IS_STATUS_OK(status);

		break;
	case OBC_LDMP_DOUBLE:

		return setPropertyLdmpDpuble(propertyId, (void*)data, dataSize);

	case OBC_AE_ENABLE:
	case OBC_IR_EXP:
	case OBC_IR_GAIN:
	case OBC_PULSE_WIDTH:
	case OBC_FLOOD_LED:
	case OBC_LASER:
	case OBC_LASER_CURRENT:
	case OBC_FLOOD_CURRENT:
	case OBC_CAMERA_PARAMS:
	case OBC_CHIP_RESET:
	case OBC_NTC_SVC:
	case OBC_NTC_COEFF:
	case OBC_DEP_RECTIFY:
    case OBC_SUPPORT_VIDEO_MODES:
    case OBC_SENSOR_TEMP:
    case OBC_RX_TEMP:
    case OBC_TX_TEMP:
    case OBC_ILLUM_POWER:
    case OBC_ILLUM_POWER_CTL:
    case OBC_INTEGRATION_TIME:
    case OBC_MODULATION_FREQUENCY:
    case OBC_DATA_OUTPUT_MODE:
    case OBC_DUTY_CYCLE:
    case OBC_MIRROR_FLIP:
    case OBC_TEST_PATTERN:
    case OBC_SOFTWARE_TRIGGER:
    case OBC_HARDWARE_TRIGGER:

    case OBC_SENSOR_ID:
    case OBC_SENSOR_INFO:
    case OBC_DUTY_CYCLE_LIST:
    case OBC_VCSEL_PD:
    case OBC_ILLUM_POWER_TEST:
    case OBC_DEFAULT_PARAMS:
    case OBC_OPS_PTR:
    case OBC_AE:
    case OBC_DRIVER_IC_DETECT:
    case OBC_REG16_RW:
    case OBC_DELAY_CIRCUIT_TIME:
    case OBC_DELAY_CIRCUIT_ID:
    case OBC_TX_A_B_POWER:
    case OBC_SENSOR_FREQUENCY_DUTY_CONFIG:
    case OBC_SHUFFLE:
    case OBC_CHIP_ID:
    case OBC_BINNING_MODE:
    case OBC_BURST_MODE:
    case OBC_FREQUENCY_MODE:
    case OBC_ITO_EM:
    case OBC_DRIVER_IC_REG8_RW:
    case OBC_VMGHI_VOLTAGE:
	case OBC_DELAY_POWER:
	case OBC_DELAY_FW_VERSION:
	case OBC_DELAY_DELAY_SWITCH:
	case OBC_DELAY_DELAY_TIME:
	case OBC_DELAY_TEMPERATURE:
	case OBC_PHASE_COUNT:
	case OBC_WINDOW_ORIGINY:
	case OBC_WINDOW_ORIGINX:
	case OBC_WINDOW_HEIGHT:
	case OBC_WINDOW_WIDTH:
	case OBC_ODD_DGAIN:
	case OBC_EVEN_DGAIN:
	case OBC_SUB_SAMP:
	case OBC_DEEPSLEEP_MODE:
	case OBC_HDR_ALGORITHM:
	case OBC_HIST_ALGORITHM:
	case OBC_MEDIAN_ALGORITHM:
	case OBC_EBC_ALGORITHM:
	case OBC_LSC_ALGORITHM:
	case OBC_SUB_SAMP_V:
	case OBC_CORRECTION_ALGORITHM:
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandSetProperty(propertyId, (void*)data, dataSize);
		XN_IS_STATUS_OK(status);
		break;

	case OBC_EXT_DEP_PREFILTER:
	case OBC_EXT_DEP_PREFILTER_LEVEL:
	case OBC_EXT_DEP_SMFILTER:
	case OBC_EXT_DEP_THRFILTER:
	case OBC_EXT_DEP_UNIDIV:
	case OBC_EXT_DEP_PTFILTER:
	case OBC_EXT_DEP_ENGINE:
	case OBC_EXT_GPM:
	case OBC_EXT_GPM_IS_ENABLE:
		return OBCExtension_SetProperty(propertyId, data, dataSize);

	case OBC_SOFT_FILTER:
		return setPropertySoftFilter(propertyId, (void*)data, dataSize);
	case XN_MODULE_PROPERTY_AE:
	case XN_MODULE_PROPERTY_EMITTER_STATE:
	case XN_MODULE_PROPERTY_EEPROM_STATE:
		if (propertyId == XN_MODULE_PROPERTY_AE)
		{
			propertyId = OBC_AE;
		}
		else if (propertyId == XN_MODULE_PROPERTY_EMITTER_STATE)
		{
			propertyId = OBC_ILLUM_POWER;
		}
		else if (propertyId == XN_MODULE_PROPERTY_EEPROM_STATE)
		{
			propertyId = OBC_EEPROM_RW;
			if (dataSize != sizeof(eeprom_data_t)) {
				return -XN_STATUS_ERROR;
			}
		}
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandSetProperty(propertyId, (void*)data, dataSize);
		XN_IS_STATUS_OK(status);
		break;
	case OBC_GROUP_FRAME_MODE:
		if (dataSize == sizeof(ObcTofFrameMode))
		{
			ObcTofFrameMode *tofFrameMode = (ObcTofFrameMode *)data;
			setGroupFrameMode(*tofFrameMode);
		}
		break;
	default:
		xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "This property setting is not supported!");
		return XN_STATUS_NO_MATCH;
	}

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::PropertyLoadFirmware(XnInt propertyId, void* pData, XnInt dataSize)
{
	block_buf_t blockbf;
	xnOSMemSet(&blockbf, 0, sizeof(block_buf_t));
	blockbf.data = pData;
	blockbf.len = dataSize;
	blockbf.blocksize = 500;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandSetProperty(propertyId, &blockbf, sizeof(block_buf_t));
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::PropertyLoadReference(XnInt propertyId, void* pData, XnInt dataSize)
{
	xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "Start load reference...");

	block_buf_t blockbf;
	xnOSMemSet(&blockbf, 0, sizeof(block_buf_t));
	blockbf.data = pData;
	blockbf.len = dataSize;
	blockbf.blocksize = 500;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandSetProperty(propertyId, &blockbf, sizeof(block_buf_t));
	XN_IS_STATUS_OK(status);

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->LoadReference(&blockbf);
	XN_IS_STATUS_OK(status);

	xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "Load reference success...");

	return XN_STATUS_OK;
}



XnStatus XnDothinDeviceProperties::getPropertyModuleVersion(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnInt mName = 0;
	XnInt size = sizeof(mName);
	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandGetProperty(propertyId, &mName, &size);
	XN_IS_STATUS_OK(status);

	sprintf((XnChar*)data, "%d.%d.%d.%d", mName >> 24, mName >> 16 & 0xff, mName >> 8 & 0xff, mName & 0xff);

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::getPropertyFirmwareVersion(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnInt mName = 0;
	XnInt size = sizeof(mName);

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandGetProperty(propertyId, &mName, &size);
	XN_IS_STATUS_OK(status);

	sprintf((XnChar*)data, "%d.%d.%d.%d", mName >> 24, mName >> 16 & 0xff, mName >> 8 & 0xff, mName & 0xff);

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::getPropertyEepromData(XnInt propertyId, void* data, XnInt* pDataSize)
{
	if (*pDataSize != sizeof(eeprom_data_t))
	{
		return XN_STATUS_INVALID_BUFFER_SIZE;
	}

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandGetProperty(propertyId, data, pDataSize);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::setPropertyEepromData(XnInt propertyId, void* pData, XnInt dataSize)
{	
	if (dataSize != sizeof(eeprom_data_t))
	{
		return XN_STATUS_INVALID_BUFFER_SIZE;
	}

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandSetProperty(propertyId, (void*)pData, dataSize);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}


XnStatus XnDothinDeviceProperties::getPropertyLdmpDpuble(XnInt propertyId, void* data, XnInt* pDataSize)
{
	if (*pDataSize != sizeof(LdmpDouble_T))
	{
		return XN_STATUS_INVALID_BUFFER_SIZE;
	}

	LdmpDouble_T* ldmpDoule = (LdmpDouble_T*)data;
	XnUInt16 ldmpdoubleRead = 0;
	XnInt size = sizeof(XnUInt16);

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandGetProperty(propertyId, &ldmpdoubleRead, &size);
	XN_IS_STATUS_OK(status);

	ldmpDoule->a_en = ldmpdoubleRead >> 8 & 0xff;
	ldmpDoule->b_en = ldmpdoubleRead & 0xff;

	printf("getProperty ldmpdouble ldmpdoubleRead : %d, a_en: %d, b_en: %d\n", ldmpdoubleRead, ldmpDoule->a_en, ldmpDoule->b_en);

	return XN_STATUS_OK;

}

XnStatus XnDothinDeviceProperties::setPropertyLdmpDpuble(XnInt propertyId, void* pData, XnInt dataSize)
{
	if (dataSize != sizeof(LdmpDouble_T))
	{
		return XN_STATUS_INVALID_BUFFER_SIZE;
	}

	LdmpDouble_T* ldmpDoule = (LdmpDouble_T*)pData;

	uint16_t  ldmpdouble = 0;
	XnUInt8 a = ldmpDoule->a_en;
	XnUInt8 b = ldmpDoule->b_en;
	ldmpdouble = a << 8 | b;
	XnInt size = sizeof(ldmpdouble);

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	XnStatus status = m_pMudulesHelper->CommandSetProperty(propertyId, &ldmpdouble, size);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}

XnStatus XnDothinDeviceProperties::OBCExtension_GetProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnStatus status = XN_STATUS_OK;
	XnInt msg = 0;

	switch (propertyId)
	{
	case OBC_EXT_DEP_PREFILTER://
		if (*pDataSize != sizeof(obc_prefilter_t))
		{
			return XN_STATUS_INVALID_BUFFER_SIZE;
		}

		return getExtPropertyPrefilter(propertyId, data, pDataSize);

	case OBC_EXT_DEP_PREFILTER_LEVEL://
	case OBC_EXT_DEP_SMFILTER://
	case OBC_EXT_DEP_THRFILTER://
	case OBC_EXT_DEP_UNIDIV://
	case OBC_EXT_DEP_PTFILTER://
	case OBC_EXT_DEP_ENGINE://
	case OBC_EXT_GPM://
	case OBC_EXT_GPM_STATUS://
	case OBC_EXT_GPM_IS_ENABLE://

		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->ExtCommandGetProperty(propertyId, &msg, data, pDataSize);
		XN_IS_STATUS_OK(status);

		break;
	default:
		xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "This extension property getting is not supported!");
		return XN_STATUS_NO_MATCH;
	}

	return XN_STATUS_OK;
}



XnStatus XnDothinDeviceProperties::OBCExtension_SetProperty(XnInt propertyId, const void* data, XnInt dataSize)
{
	XnStatus status = XN_STATUS_OK;
	XnInt msg = 0;

	switch (propertyId)
	{
	case OBC_EXT_DEP_PREFILTER://
		if (dataSize != sizeof(obc_prefilter_t))
		{
			return XN_STATUS_INVALID_BUFFER_SIZE;
		}
		
		return SetExtPropertyPrefilter(propertyId, (void*)data, dataSize);
	case OBC_EXT_DEP_SMFILTER://
	case OBC_EXT_DEP_THRFILTER://
	case OBC_EXT_DEP_UNIDIV://
	case OBC_EXT_DEP_PTFILTER://
	case OBC_EXT_DEP_ENGINE://
	case OBC_EXT_GPM://
	case OBC_EXT_GPM_IS_ENABLE://

		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->ExtCommandSetProperty(propertyId, &msg, (void*)data, dataSize);
		XN_IS_STATUS_OK(status);

		break;
	default:
		xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "This extension property getting is not supported!");
		return XN_STATUS_NO_MATCH;
	}

	

	return XN_STATUS_OK;
}



XnStatus XnDothinDeviceProperties::getExtPropertyPrefilter(XnInt propertyId, void* data, XnInt* pDataSize)
{
	XnStatus status = XN_STATUS_OK;

	obc_prefilter_t* obc_prefilter = (obc_prefilter_t*)data;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->ExtCommandGetProperty(propertyId, &obc_prefilter->level, &obc_prefilter->obc_Prefilter, pDataSize);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}

XnStatus XnDothinDeviceProperties::SetExtPropertyPrefilter(XnInt propertyId, void* pData, XnInt dataSize)
{
	XnStatus status = XN_STATUS_OK;

	obc_prefilter_t* obc_prefilter = (obc_prefilter_t*)pData;

	XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
	status = m_pMudulesHelper->ExtCommandSetProperty(propertyId, &obc_prefilter->level, &obc_prefilter->obc_Prefilter, dataSize);
	XN_IS_STATUS_OK(status);

	return XN_STATUS_OK;
}

XnStatus XnDothinDeviceProperties::setPropertySoftFilter(XnInt propertyId, void* pData, XnInt dataSize)
{
	XnInt* value = (XnInt*)pData;
	XnInt enable = *value;

    /*TODO 删除obtof中 XnDothinDeviceProperties中有关ir模组深度的函数
	if (enable == TRUE)
	{
		enable_filter(TRUE);
	}
	else
	{
		enable_filter(FALSE);
	}
    */

	xnLogInfo(XN_MASK_DOTIN_DEVICE_PPS, "Set soft filter %d success...", enable);
	return XN_STATUS_OK;
}
XnStatus XnDothinDeviceProperties::setGroupFrameMode(ObcTofFrameMode tofFrameMode)
{
	XnStatus status = -1;
	if (m_pMudulesHelper != nullptr)
	{
		int outMode = tofFrameMode.out_mode;
		status = m_pMudulesHelper->CommandSetProperty(OBC_DATA_OUTPUT_MODE, &outMode, sizeof(int));
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "Set OBC_DATA_OUTPUT_MODE error...%d", status);
			return status;
		}
		int freqMode = tofFrameMode.freqmode;
		status = m_pMudulesHelper->CommandSetProperty(OBC_FREQUENCY_MODE, &freqMode, sizeof(int));
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "Set OBC_FREQUENCY_MODE error...%d", status);
			return status;
		}

		int freq = 0;
		if (tofFrameMode.freqmode == OBC_DUAL_FREQUENCY)
		{
			freq = (tofFrameMode.requestfreq[0].frequency << 8) | (tofFrameMode.requestfreq[1].frequency);
		}
		else
		{
			freq = tofFrameMode.requestfreq[0].frequency;
		}
		status = m_pMudulesHelper->CommandSetProperty(OBC_MODULATION_FREQUENCY, &freq, sizeof(int));
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "Set OBC_MODULATION_FREQUENCY error...%d", status);
			return status;
		}

		int dutyCycle = 0;
		if (tofFrameMode.freqmode == OBC_DUAL_FREQUENCY)
		{
			dutyCycle = (tofFrameMode.requestfreq[0].duty_cycle << 8) | (tofFrameMode.requestfreq[1].duty_cycle & 0xFF);
		}
		else
		{
			dutyCycle = tofFrameMode.requestfreq[0].duty_cycle;
		}

		status = m_pMudulesHelper->CommandSetProperty(OBC_DUTY_CYCLE, &dutyCycle, sizeof(int));
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "Set OBC_DUTY_CYCLE error...%d", status);
			return status;
		}
		int integrationTime = tofFrameMode.requestfreq[0].integration_time;
		status = m_pMudulesHelper->CommandSetProperty(OBC_INTEGRATION_TIME, &integrationTime, sizeof(int));
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "Set OBC_INTEGRATION_TIME error...%d", status);
			return status;
		}
	}
	
	return status;
}

XnStatus XnDothinDeviceProperties::getGroupFrameMode(ObcTofFrameMode *tofFrameMode, int dataSize)
{
	XnStatus status = -1;
	if ((m_pMudulesHelper != nullptr) || (dataSize != sizeof(ObcTofFrameMode)))
	{
		int outMode = 0;
		ObcTofFrameMode obcFrameMode;
		memset(&obcFrameMode, 0, sizeof(ObcTofFrameMode));
		int outModeSize = sizeof(outMode);
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(OBC_DATA_OUTPUT_MODE, &outMode, &outModeSize);
		XN_IS_STATUS_OK(status);
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "get OBC_DATA_OUTPUT_MODE error...%d", status);
			return status;
		}
		obcFrameMode.out_mode = (ObcTofOutMode)outMode;

		int freqMode = 0;
		int freqModeSize = sizeof(freqModeSize);
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(OBC_FREQUENCY_MODE, &freqMode, &freqModeSize);
		XN_IS_STATUS_OK(status);
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "get OBC_FREQUENCY_MODE error...%d", status);
			return status;
		}
		obcFrameMode.freqmode = (ObcFrequencyMode)freqMode;

		int freq = 0;
		int freqSize = sizeof(freq);
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(OBC_MODULATION_FREQUENCY, &freq, &freqSize);
		XN_IS_STATUS_OK(status);
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "get OBC_MODULATION_FREQUENCY error...%d", status);
			return status;
		}
		if (obcFrameMode.freqmode == OBC_DUAL_FREQUENCY)
		{
			obcFrameMode.requestfreq[0].frequency = freq >> 8;
			obcFrameMode.requestfreq[1].frequency = freq & 0xff;
		}
		else
		{
			obcFrameMode.requestfreq[0].frequency = freq & 0xff;
		}

		int dutyCycle = 0;
		int dutyCycleSize = sizeof(dutyCycle);
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(OBC_DUTY_CYCLE, &dutyCycle, &dutyCycleSize);
		XN_IS_STATUS_OK(status);
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "get OBC_MODULATION_FREQUENCY error...%d", status);
			return status;
		}
		if (obcFrameMode.freqmode == OBC_DUAL_FREQUENCY)
		{
			obcFrameMode.requestfreq[0].duty_cycle = dutyCycle >> 8;
			obcFrameMode.requestfreq[1].duty_cycle = dutyCycle & 0xff;
		}
		else
		{
			obcFrameMode.requestfreq[0].duty_cycle = dutyCycle & 0xff;
		}

		int integrationTime = 0;
		int timeSize = sizeof(integrationTime);
		XN_VALIDATE_INPUT_PTR(m_pMudulesHelper);
		status = m_pMudulesHelper->CommandGetProperty(OBC_INTEGRATION_TIME, &integrationTime, &timeSize);
		XN_IS_STATUS_OK(status);
		if (status != XN_STATUS_OK)
		{
			xnLogError(XN_MASK_DOTIN_DEVICE_PPS, "get OBC_INTEGRATION_TIME error...%d", status);
			return status;
		}
		obcFrameMode.requestfreq[0].integration_time = integrationTime;
		xnOSMemCopy(tofFrameMode, &obcFrameMode, sizeof(ObcTofFrameMode));
	}
	return status;
}

XnDothinDeviceProperties::~XnDothinDeviceProperties()
{

}
