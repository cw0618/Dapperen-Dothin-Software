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

XnDothinDeviceProperties::~XnDothinDeviceProperties()
{

}
