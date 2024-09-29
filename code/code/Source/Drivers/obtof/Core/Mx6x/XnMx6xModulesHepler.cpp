
#include "XnMx6xModulesHepler.h"
#include <iostream>
#include <fstream>
#include <string>
#include "XnOS.h"
#include "XnLog.h"
#include "XnDothinProtocol.h"
#include <OBCProperties.h>
using namespace std;


#define OniGetProcAddress(function)																\
{																								\
	rc = xnOSGetProcAddress(m_libHandle, XN_STRINGIFY(function), (XnFarProc*)&funcs.function);	\
if (rc != ONI_STATUS_OK)																	\
{																							\
	xnLogWarning(XN_MASK_MODULE_HEPLER, "LibraryHandler: Couldn't find function %s in %s. Stopping", XN_STRINGIFY(function), library);	\
	return;																					\
}																							\
}


XnMx6xModulesHelper::XnMx6xModulesHelper() :dothin_id(-1), m_valid(false)
{

}


XnStatus XnMx6xModulesHelper::Init(XnInt32 dothin_id_)
{
	this->dothin_id = dothin_id_;
	if (this->dothin_id == -1)
	{
		return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Load modules library...");
	//1.Load module library
	XnStatus res = LoadModulesLibrary();
	if (res != XN_STATUS_OK)
	{
		return -1;
	}
	XN_IS_STATUS_OK(res);

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Resolve path to config...");
	res = resolvePathToConfig();
	XN_IS_STATUS_OK(res);

	AddModuleProperties();

	return XN_STATUS_OK;
}

void XnMx6xModulesHelper::AddModuleProperties()
{
	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Add module properties...");
	obcCmdPropertiesHash.Clear();

	DEFINE_OBCMD_MAP(MODULE_NAME, CreateObcCmdT("Module name", MODULE_NAME), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(MODULE_VERSION, CreateObcCmdT("Module version", MODULE_VERSION), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(FW_VERSION, CreateObcCmdT("Firmware version", FW_VERSION), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LOAD_FW, CreateObcCmdT("Load firmware", LOAD_FW), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LOAD_REF, CreateObcCmdT("Load reference", LOAD_REF), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(EEPROM_RW, CreateObcCmdT("EEPROW", EEPROM_RW), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(REG_RW, CreateObcCmdT("Register", REG_RW), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(STREAM_MODE, CreateObcCmdT("StreamMode", STREAM_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DATA_FMT, CreateObcCmdT("Data format", DATA_FMT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(FPS, CreateObcCmdT("FPS", FPS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(AE_ENABLE, CreateObcCmdT("AE enable", AE_ENABLE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(IR_EXP, CreateObcCmdT("IR exposure", IR_EXP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(IR_GAIN, CreateObcCmdT("IR Gain", IR_GAIN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(PULSE_WIDTH, CreateObcCmdT("Pulse width", PULSE_WIDTH), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(FLOOD_LED, CreateObcCmdT("Flood status", FLOOD_LED), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LASER, CreateObcCmdT("Laser status", LASER), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(IR_SN, CreateObcCmdT("IR sn", IR_SN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LDMP_SN, CreateObcCmdT("LDMP sn", LDMP_SN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LASER_CURRENT, CreateObcCmdT("Laser current", LASER_CURRENT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(FLOOD_CURRENT, CreateObcCmdT("Flood current", FLOOD_CURRENT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(PACK_FMT, CreateObcCmdT("Pack format", PACK_FMT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DISP_BITS, CreateObcCmdT("DISP bits", DISP_BITS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DISP_SUB_BITS, CreateObcCmdT("DISP sub bits", DISP_SUB_BITS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DEPTH_ROTATION_TYPE, CreateObcCmdT("Depth rotation", DEPTH_ROTATION_TYPE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DEPTH_MIRROR_TYPE, CreateObcCmdT("Mirror", DEPTH_MIRROR_TYPE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ENGINE_ID, CreateObcCmdT("Engine id", ENGINE_ID), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(CAMERA_PARAMS, CreateObcCmdT("Camera params", CAMERA_PARAMS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(IR_TEMP, CreateObcCmdT("IR temperature", IR_TEMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LDMP_TEMP, CreateObcCmdT("LDMP temperature", LDMP_TEMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(EXT_PARAMS, CreateObcCmdT("ext params", EXT_PARAMS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LDMP_DOUBLE, CreateObcCmdT("LDMP double", LDMP_DOUBLE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(CHIP_RESET, CreateObcCmdT("Chip reset", CHIP_RESET), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ENGINE_NAME, CreateObcCmdT("Engine name", ENGINE_NAME), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(NTC_SVC, CreateObcCmdT("NTC SVC", NTC_SVC), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(NTC_COEFF, CreateObcCmdT("NTC COEFF", NTC_COEFF), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DEP_RECTIFY, CreateObcCmdT("DEP rectify", DEP_RECTIFY), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(AE_AVG, CreateObcCmdT("AE avg", AE_AVG), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ENGINE_IS_VALID, CreateObcCmdT("Engine is valid", ENGINE_IS_VALID), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(WINDOW_ORIGINY, CreateObcCmdT("WINDOW_ORIGINY", WINDOW_ORIGINY), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(WINDOW_ORIGINX, CreateObcCmdT("WINDOW_ORIGINX", WINDOW_ORIGINX), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(WINDOW_HEIGHT, CreateObcCmdT("WINDOW_HEIGHT", WINDOW_HEIGHT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(WINDOW_WIDTH, CreateObcCmdT("WINDOW_WIDTH", WINDOW_WIDTH), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ODD_DGAIN, CreateObcCmdT("ODD_DGAIN", ODD_DGAIN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(EVEN_DGAIN, CreateObcCmdT("EVEN_DGAIN", EVEN_DGAIN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SUB_SAMP, CreateObcCmdT("SUB_SAMP", SUB_SAMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DEEPSLEEP_MODE, CreateObcCmdT("DEEPSLEEP_MODE", DEEPSLEEP_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(HDR_ALGORITHM, CreateObcCmdT("HDR_ALGORITHM", HDR_ALGORITHM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(HIST_ALGORITHM, CreateObcCmdT("HIST_ALGORITHM", HIST_ALGORITHM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(MEDIAN_ALGORITHM, CreateObcCmdT("MEDIAN_ALGORITHM", MEDIAN_ALGORITHM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(EBC_ALGORITHM, CreateObcCmdT("EBC_ALGORITHM", EBC_ALGORITHM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(LSC_ALGORITHM, CreateObcCmdT("LSC_ALGORITHM", LSC_ALGORITHM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(TEST_PATTERN, CreateObcCmdT("TEST_PATTERN", TEST_PATTERN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SUB_SAMP_V, CreateObcCmdT("SUB_SAMP_V", SUB_SAMP_V), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(CORRECTION_ALGORITHM, CreateObcCmdT("CORRECTION_ALGORITHM", CORRECTION_ALGORITHM), obcCmdPropertiesHash);
	/**
	*add for tof property
	*/
	DEFINE_OBCMD_MAP(SUPPORT_VIDEO_MODES, CreateObcCmdT("SUPPORT_VIDEO_MODES", SUPPORT_VIDEO_MODES), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SENSOR_TEMP, CreateObcCmdT("SENSOR_TEMP", SENSOR_TEMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(RX_TEMP, CreateObcCmdT("RX_TEMP", RX_TEMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(TX_TEMP, CreateObcCmdT("TX_TEMP", TX_TEMP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ILLUM_POWER, CreateObcCmdT("ILLUM_POWER", ILLUM_POWER), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ILLUM_POWER_CTL, CreateObcCmdT("ILLUM_POWER_CTL", ILLUM_POWER_CTL), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(INTEGRATION_TIME, CreateObcCmdT("INTEGRATION_TIME", INTEGRATION_TIME), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(MODULATION_FREQUENCY, CreateObcCmdT("MODULATION_FREQUENCY", MODULATION_FREQUENCY), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DATA_OUTPUT_MODE, CreateObcCmdT("DATA_OUTPUT_MODE", DATA_OUTPUT_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DUTY_CYCLE, CreateObcCmdT("DUTY_CYCLE", DUTY_CYCLE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(MIRROR_FLIP, CreateObcCmdT("COMMAND_END", MIRROR_FLIP), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(TEST_PATTERN, CreateObcCmdT("TEST_PATTERN", TEST_PATTERN), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SOFTWARE_TRIGGER, CreateObcCmdT("SOFTWARE_TRIGGER", SOFTWARE_TRIGGER), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(HARDWARE_TRIGGER, CreateObcCmdT("HARDWARE_TRIGGER", HARDWARE_TRIGGER), obcCmdPropertiesHash);

	DEFINE_OBCMD_MAP(SENSOR_ID, CreateObcCmdT("SENSOR_ID", SENSOR_ID), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SENSOR_INFO, CreateObcCmdT("SENSOR_INFO", SENSOR_INFO), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DUTY_CYCLE_LIST, CreateObcCmdT("DUTY_CYCLE_LIST", DUTY_CYCLE_LIST), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(VCSEL_PD, CreateObcCmdT("VCSEL_PD", VCSEL_PD), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ILLUM_POWER_TEST, CreateObcCmdT("ILLUM_POWER_TEST", ILLUM_POWER_TEST), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DEFAULT_PARAMS, CreateObcCmdT("DEFAULT_PARAMS", DEFAULT_PARAMS), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(OPS_PTR, CreateObcCmdT("OPS_PTR", OPS_PTR), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(AE, CreateObcCmdT("AE", AE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DRIVER_IC_DETECT, CreateObcCmdT("DRIVER_IC_DETECT", DRIVER_IC_DETECT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(REG16_RW, CreateObcCmdT("REG16_RW", REG16_RW), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DELAY_CIRCUIT_TIME, CreateObcCmdT("DELAY_CIRCUIT_TIME", DELAY_CIRCUIT_TIME), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DELAY_CIRCUIT_ID, CreateObcCmdT("DELAY_CIRCUIT_ID", DELAY_CIRCUIT_ID), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(TX_A_B_POWER, CreateObcCmdT("TX_A_B_POWER", TX_A_B_POWER), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SENSOR_FREQUENCY_DUTY_CONFIG, CreateObcCmdT("SENSOR_FREQUENCY_DUTY_CONFIG", SENSOR_FREQUENCY_DUTY_CONFIG), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(SHUFFLE, CreateObcCmdT("SHUFFLE", SHUFFLE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(CHIP_ID, CreateObcCmdT("CHIP_ID", CHIP_ID), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(BINNING_MODE, CreateObcCmdT("BINNING_MODE", BINNING_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(BURST_MODE, CreateObcCmdT("BURST_MODE", BURST_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(FREQUENCY_MODE, CreateObcCmdT("FREQUENCY_MODE", FREQUENCY_MODE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(ITO_EM, CreateObcCmdT("ITO_EM", ITO_EM), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DRIVER_IC_REG8_RW, CreateObcCmdT("DRIVER_IC_REG8_RW", DRIVER_IC_REG8_RW), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(VMGHI_VOLTAGE, CreateObcCmdT("VMGHI_VOLTAGE", VMGHI_VOLTAGE), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(AF_DEPTH, CreateObcCmdT("AF_DEPTH", AF_DEPTH), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(DELAY_DELAY_TIME, CreateObcCmdT("Delay time", DELAY_DELAY_TIME), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(PHASE_COUNT, CreateObcCmdT("PHASE_COUNT", PHASE_COUNT), obcCmdPropertiesHash);
	DEFINE_OBCMD_MAP(COMMAND_END, CreateObcCmdT("COMMAND_END", COMMAND_END), obcCmdPropertiesHash);
	
	//DEFINE_OBCMD_MAP(DELAY_DELAY_SWITCH, CreateObcCmdT("Delay switch", DELAY_DELAY_SWITCH), obcCmdPropertiesHash);
	//DEFINE_OBCMD_MAP(DELAY_TEMPERATURE, CreateObcCmdT("Delay temperature", DELAY_TEMPERATURE), obcCmdPropertiesHash);

	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Add module properties end...");

	/**********************************************************Ext command******************************************************************/

	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Add module ext properties...");

	ext_cmdPropertiesHash.Clear();

	DEFINE_OB_EXTCMD_MAP(DEP_PREFILTER, CreateObcExtCmdT("Depth prefilter", EXT_PARAMS, DEP_PREFILTER), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_PREFILTER_LEVEL, CreateObcExtCmdT("Depth prefilter level", EXT_PARAMS, DEP_PREFILTER_LEVEL), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_SMFILTER, CreateObcExtCmdT("Depth smfilter", EXT_PARAMS, DEP_SMFILTER), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_THRFILTER, CreateObcExtCmdT("Depth thrfilter", EXT_PARAMS, DEP_THRFILTER), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_UNIDIV, CreateObcExtCmdT("Depth unidiv", EXT_PARAMS, DEP_UNIDIV), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_PTFILTER, CreateObcExtCmdT("Depth ptfilter", EXT_PARAMS, DEP_PTFILTER), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(DEP_ENGINE, CreateObcExtCmdT("Depth engine", EXT_PARAMS, DEP_ENGINE), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(GPM, CreateObcExtCmdT("GPM", EXT_PARAMS, GPM), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(GPM_STATUS, CreateObcExtCmdT("GPM status", EXT_PARAMS, GPM_STATUS), ext_cmdPropertiesHash);
	DEFINE_OB_EXTCMD_MAP(GPM_IS_ENABLE, CreateObcExtCmdT("GPM enable", EXT_PARAMS, GPM_IS_ENABLE), ext_cmdPropertiesHash);

	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Add module ext properties end...");

}

OBC_Command_t XnMx6xModulesHelper::CreateObcCmdT(char* propertyName, mx6x_command cmd)
{
	OBC_Command_t obc_Cmd;
	strcpy(obc_Cmd.propertyName, propertyName);
	obc_Cmd.command = cmd;
	return obc_Cmd;
}


OBC_Ext_Command_t XnMx6xModulesHelper::CreateObcExtCmdT(char* propertyName, mx6x_command cmd, hw_ext_params_t ext_command)
{
	OBC_Ext_Command_t obc_Ext_Cmd;
	strcpy(obc_Ext_Cmd.propertyName, propertyName);
	obc_Ext_Cmd.command = cmd;
	obc_Ext_Cmd.ext_command = ext_command;

	return obc_Ext_Cmd;
}


XnStatus XnMx6xModulesHelper::LoadModulesLibrary()
{
	XnStatus res = XN_STATUS_OK;

	//1.Get modules dir
	res = GetModulesDir();
	XN_IS_STATUS_OK(res);

	//2.Load module(load library)
	res = LoadModules();
	if (res != XN_STATUS_OK)
	{
		return -1;
	}
	XN_IS_STATUS_OK(res);

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Load moduls library success...");
	return XN_STATUS_OK;
}

// Dummy function used only for taking its address for the sake of xnOSGetModulePathForProcAddress.
static void dummyFunctionToTakeModulesAddress() {}


XnStatus XnMx6xModulesHelper::GetModulesDir()
{
	XnStatus res = XN_STATUS_OK;

	XnChar strdothinDriverPath[XN_FILE_MAX_PATH];
	res = xnOSGetModulePathForProcAddress(reinterpret_cast<void*>(&dummyFunctionToTakeModulesAddress), strdothinDriverPath);
	if (res != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Get modules parent path failed!");
		return res;
	}

	string temStr = strdothinDriverPath;
	XnInt32 resIndex = -1;
	if ((resIndex = temStr.rfind(XN_STRING_OPENNI)) == string::npos)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Get modules OpenNI2 directory failed!");
		return XN_STATUS_ERROR;
	}

	string pModuleDir = temStr.substr(0, resIndex);
	xnOSMemSet(rootDir, 0, sizeof(rootDir));
	strcpy(rootDir, pModuleDir.c_str());

	string modulesDirStr = pModuleDir + XN_MODULES_DIR;
	xnOSMemSet(modulesDir, 0, sizeof(modulesDir));
	strcpy(modulesDir, modulesDirStr.c_str());

	XnBool bDirExists = FALSE;
	res = xnOSDoesDirectoryExist(modulesDir, &bDirExists);
	if (res != XN_STATUS_OK || !bDirExists)
	{
		xnLogInfo(XN_MASK_MODULE_HEPLER, "Find modules directory failed!");
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_MODULE_HEPLER, "Find the modules directory：%s", modulesDir);

	CreateMx6xSLogDir(rootDir);

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::CreateMx6xSLogDir(XnChar* rootDir)
{
	XnStatus nRetVal;

	XnChar strLog[XN_FILE_MAX_PATH] = "";
	XN_VALIDATE_STR_APPEND(strLog, rootDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(strLog, XN_SLOG_DIR, XN_FILE_MAX_PATH, nRetVal);

	XnBool bDirExists = FALSE;
	nRetVal = xnOSDoesDirectoryExist(strLog, &bDirExists);
	if (bDirExists)
	{
		strcpy(sLogDir, strLog);
		return XN_STATUS_OK;
	}

	nRetVal = xnOSCreateDirectory(strLog);
	XN_IS_STATUS_OK(nRetVal);
	strcpy(sLogDir, strLog);
	xnLogInfo(XN_MASK_MODULE_HEPLER, "Create Mx6x Log directory success");

	return XN_STATUS_OK;
}


XnStatus XnMx6xModulesHelper::LoadModules()
{
	XnStatus nRetVal;

	// search modules
	XnInt32 nFileCount = 0;
	XnChar cpSearchString[XN_FILE_MAX_PATH] = "";

	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Looking for modules at '%s'", modulesDir);

	// Build the search pattern string
	XN_VALIDATE_STR_APPEND(cpSearchString, modulesDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpSearchString, XN_FILE_DIR_SEP, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpSearchString, XN_SHARED_LIBRARY_PREFIX, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpSearchString, XN_FILE_ALL_WILDCARD, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpSearchString, XN_SHARED_LIBRARY_POSTFIX, XN_FILE_MAX_PATH, nRetVal);

	nRetVal = xnOSCountFiles(cpSearchString, &nFileCount);
	if (nRetVal != XN_STATUS_OK || nFileCount == 0)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Found no modules matching '%s'", cpSearchString);
		return XN_STATUS_NO_MODULES_FOUND;
	}


	typedef XnChar MyMudules[XN_FILE_MAX_PATH];
	MyMudules* acsFileList = XN_NEW_ARR(MyMudules, nFileCount);

	nRetVal = xnOSGetFileList(cpSearchString, NULL, acsFileList, nFileCount, &nFileCount);
	XN_IS_STATUS_OK(nRetVal);

	for (size_t i = 0; i < nFileCount; i++)
	{
		std::string moduleName = acsFileList[i];
		if (strcmp(moduleName.c_str(), "tof.dll") == 0)
		{
			char* moduleNamePath = acsFileList[i];
			XnChar rscModulePath[XN_FILE_MAX_PATH] = "";
			XN_VALIDATE_STR_APPEND(rscModulePath, modulesDir, XN_FILE_MAX_PATH, nRetVal);
			XN_VALIDATE_STR_APPEND(rscModulePath, XN_FILE_DIR_SEP, XN_FILE_MAX_PATH, nRetVal);
			XN_VALIDATE_STR_APPEND(rscModulePath, moduleNamePath, XN_FILE_MAX_PATH, nRetVal);

			XnChar desModulePath[XN_FILE_MAX_PATH];
			nRetVal = CreateModuleFilesStr(moduleNamePath, desModulePath);
			if (nRetVal != XN_STATUS_OK)
			{
				xnLogError(XN_MASK_MODULE_HEPLER, "Create module file failed!");
				continue;
			}

			nRetVal = CopyModuleFile(rscModulePath, desModulePath);
			if (nRetVal != XN_STATUS_OK)
			{
				xnLogError(XN_MASK_MODULE_HEPLER, "Copy module file failed!");
				continue;
			}

			xnLogInfo(XN_MASK_MODULE_HEPLER, "Load module '%s'", desModulePath);
			nRetVal = LoadModuleDriver(desModulePath, &p_mx6x_handle_);
			if (nRetVal != XN_STATUS_OK)
			{
				xnLogError(XN_MASK_MODULE_HEPLER, "Load module driver failed!");
				continue;
			}
			else
			{
				strcpy(realLibFile, desModulePath);
				m_valid = true;
				break;
			}
		}

		
	}
	XN_DELETE_ARR(acsFileList);
	return nRetVal;
}


XnStatus XnMx6xModulesHelper::CreateModuleFilesStr(char* moduleName, char* llmoduleName)
{
	if (dothin_id < 0)
	{
		return XN_STATUS_ERROR;
	}

	XnStatus nRetVal;

	//Convert XnInt to char 
	XnChar dothin_id_Str[XN_FILE_MAX_PATH];
	sprintf(dothin_id_Str, "%d", dothin_id);

	string tempName = moduleName;
	//Filtering copy file
	if (tempName.find(XN_MUDULES_SEG) != string::npos)
	{
		return XN_STATUS_ERROR;
	}

	XnInt32 dIndex = tempName.find(".");
	if (dIndex < 0)
	{
		return XN_STATUS_ERROR;
	}

	tempName = tempName.substr(0, dIndex);
	XnChar maName[XN_FILE_MAX_PATH];
	strcpy(maName, tempName.c_str());

	XnChar cpAppedStr[XN_FILE_MAX_PATH] = "";

	XN_VALIDATE_STR_APPEND(cpAppedStr, modulesDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpAppedStr, XN_FILE_DIR_SEP, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpAppedStr, maName, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpAppedStr, XN_MUDULES_SEG, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpAppedStr, dothin_id_Str, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(cpAppedStr, XN_SHARED_LIBRARY_POSTFIX, XN_FILE_MAX_PATH, nRetVal);

	xnOSStrCopy(llmoduleName, cpAppedStr, sizeof(cpAppedStr));

	return nRetVal;
}


XnStatus XnMx6xModulesHelper::CopyModuleFile(const char* src, const char* dst)
{
	using namespace std;
	ifstream in(src, ios::binary);
	ofstream out(dst, ios::binary);
	if (!in.is_open()) {
		//cout << "error open file " << src << endl;
		return -1;
	}
	if (!out.is_open()) {
		//cout << "error open file " << dst << endl;
		return  -2;
	}
	if (src == dst) {
		//cout << "the src file can't be same with dst file" << endl;
		//exit(EXIT_FAILURE);
		return  -3;
	}
	char buf[2048];
	long long totalBytes = 0;
	while (in)
	{
		in.read(buf, 2048);
		out.write(buf, in.gcount());
		totalBytes += in.gcount();
	}

	in.close();
	out.close();

	return  0;
}


XnStatus XnMx6xModulesHelper::LoadModuleDriver(char* moduleName, mx6x_module_t** p_module)
{
	support_protocol_type_t* p_sensortype = NULL;

	moduleLibhandle_ = LoadLibraryEx(moduleName, NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
	// Make sure it succeeded (return value is not NULL). If not return an error....
	if (moduleLibhandle_ == NULL)
	{
		remove(moduleName);
		xnLogWarning(XN_MASK_MODULE_HEPLER, "Failed to load library '%s'. Error code: %d.", moduleName, GetLastError());
		return (XN_STATUS_OS_CANT_LOAD_LIB);
	}

	XnStatus rc;
	rc = xnOSGetProcAddress(moduleLibhandle_, HAL_PROTOCOL_SYM_AS_STR, (XnFarProc*)&p_sensortype);
	if (rc != XN_STATUS_OK) {
		xnLogError(XN_MASK_MODULE_HEPLER, "%s ObGetDlAddr err:  ret: %d.", moduleName, rc);
		remove(moduleName);
		xnOSFreeLibrary(moduleLibhandle_);
		return rc;
	}


	if (NULL == p_sensortype) {
		xnLogError(XN_MASK_MODULE_HEPLER, " p_module  is null.");
		remove(moduleName);
		xnOSFreeLibrary(moduleLibhandle_);
		return XN_STATUS_ERROR;
	}

	//Init obc log
	init_logger(sLogDir, S_TRACE, LOCATION_STDOUT);


	if (SPI_PROTOCOL == p_sensortype->PROTOCOL) {
		rc = dothinProtocol.InitSpi(&ob_ops_, dothin_id);
		xnLogVerbose(XN_MASK_MODULE_HEPLER, "Init dothin_id(%d) spi api, ret= %d.", dothin_id, rc);
		dothinProtocol.InitApOps(&ob_ops_, dothin_id);
	}
	else
	{
		rc = dothinProtocol.InitI2C(&ob_ops_, dothin_id);
		xnLogVerbose(XN_MASK_MODULE_HEPLER, "init dothin_id_(%d) i2c api, ret= %d.", dothin_id, rc);
		if (rc != XN_STATUS_OK)
		{
			return XN_STATUS_ERROR;
		}
		dothinProtocol.InitApOps(&ob_ops_, dothin_id);
	}


	rc = xnOSGetProcAddress(moduleLibhandle_, HAL_MODULE_SYM_AS_STR, (XnFarProc*)p_module);
	if (rc != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "GetProcAddress err:  ret: %d.", rc);
		xnOSFreeLibrary(moduleLibhandle_);
		remove(moduleName);
	}
	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Load mx6x module success...");

	rc = (*p_module)->init_ops(&ob_ops_);
	if (rc != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, " Init ops failed!.");
		remove(moduleName);
		xnOSFreeLibrary(moduleLibhandle_);
		return XN_STATUS_ERROR;
	}
	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Init ob ops success...");

	char device_id[20];
	memset(device_id, 0, sizeof(device_id));
	command_data_t cmd_data;
	cmd_data.data = device_id;
	cmd_data.len = sizeof(device_id);

	rc = (*p_module)->get_property(ENGINE_ID, &cmd_data);
	xnLogVerbose(XN_MASK_MODULE_HEPLER, "Compare device Id: %s, request: %s ret: %d.", device_id, p_sensortype->sensorname, rc);

	if (strncmp(device_id, p_sensortype->sensorname, 4) == 0)
	{
		xnLogVerbose(XN_MASK_MODULE_HEPLER, "Get device Id: %s, request: %s...", device_id, p_sensortype->sensorname);
		return XN_STATUS_OK;
	}
	else
	{
		xnLogVerbose(XN_MASK_MODULE_HEPLER, "Get device no match Id: %s, request: %s ret: %d.", device_id, p_sensortype->sensorname);

	   xnOSFreeLibrary(moduleLibhandle_);
	   remove(moduleName);

		moduleLibhandle_ = NULL;
		*p_module = NULL;

		xnLogVerbose(XN_MASK_MODULE_HEPLER, "free handle success ret = %d ..", rc);
		rc = XN_STATUS_NO_MATCH;
	}

	return rc;
}


XnStatus XnMx6xModulesHelper::resolvePathToConfig()
{
	XN_VALIDATE_INPUT_PTR(rootDir);

	XnStatus nRetVal;

	xnOSMemSet(configDir, 0, sizeof(configDir));
	XN_VALIDATE_STR_APPEND(configDir, rootDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(configDir, XN_CONFIG_DIR, XN_FILE_MAX_PATH, nRetVal);

	xnLogVerbose(XN_MASK_MODULE_HEPLER, "resolvePathToConfig : %s ..", configDir);
	return XN_STATUS_OK;
}


XnStatus XnMx6xModulesHelper::LoadFirmware()
{
	if (xnOSStrLen(configDir) == 0)
	{
		return XN_STATUS_ERROR;
	}

	XnStatus nRetVal;
	XnChar firmwareFile[XN_FILE_MAX_PATH];

	xnOSMemSet(firmwareFile, 0, sizeof(firmwareFile));
	XN_VALIDATE_STR_APPEND(firmwareFile, configDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(firmwareFile, XN_FIRMWARE_FILE, XN_FILE_MAX_PATH, nRetVal);

	//1.load config firmware
	block_buf_t blockbf;
	xnOSMemSet(&blockbf, 0, sizeof(block_buf_t));

	XnUInt64 nFileSize = 0;
	nRetVal = xnOSGetFileSize64(firmwareFile, &nFileSize);
	if (nRetVal != XN_STATUS_OK)
	{
		return XN_STATUS_ERROR;
	}

	// 2.read file
	blockbf.len = nFileSize;
	blockbf.data = (XnChar*)xnOSCalloc((XnSizeT)(nFileSize), sizeof(XnChar));
	nRetVal = xnOSLoadFile(firmwareFile, blockbf.data, blockbf.len);
	if (nRetVal != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "LoadFirmware loadFile failed:%s.", xnGetStatusString(nRetVal));
		if (blockbf.data != NULL)
		{
			xnOSFree(blockbf.data);
		}

		return nRetVal;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "LoadFirmware:%s ....size: %d.", firmwareFile, blockbf.len);

	//3.Load firmware
	blockbf.blocksize = 500;
	nRetVal = CommandSetProperty(OBC_LOAD_FW, &blockbf, sizeof(block_buf_t));
	free(blockbf.data);
	blockbf.data = NULL;
	if (nRetVal != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "set property LoadFirmware failed:%d", nRetVal);
		return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "set property LoadFirmware success...");

	return XN_STATUS_OK;
}


XnStatus XnMx6xModulesHelper::LoadReference()
{
	/*TODO 删除IR模组相关函数 预留加载1608固件及标定文件的函数
	if (xnOSStrLen(configDir) == 0)
	{
	return XN_STATUS_ERROR;
	}

	XnStatus nRetVal = XN_STATUS_OK;

	XnChar refFile[XN_FILE_MAX_PATH];

	xnOSMemSet(refFile, 0, sizeof(refFile));
	XN_VALIDATE_STR_APPEND(refFile, configDir, XN_FILE_MAX_PATH, nRetVal);
	XN_VALIDATE_STR_APPEND(refFile, XN_REF_FILE, XN_FILE_MAX_PATH, nRetVal);

	//1.load config firmware
	block_buf_t blockbf;
	xnOSMemSet(&blockbf, 0, sizeof(block_buf_t));

	XnUInt64 nFileSize = 0;
	nRetVal = xnOSGetFileSize64(refFile, &nFileSize);
	if (nRetVal != XN_STATUS_OK)
	{
	return XN_STATUS_ERROR;
	}

	// 2.read file
	blockbf.len = nFileSize;
	blockbf.data = (XnChar*)xnOSCalloc((XnSizeT)(nFileSize), sizeof(XnChar));
	nRetVal = xnOSLoadFile(refFile, blockbf.data, blockbf.len);
	if (nRetVal != XN_STATUS_OK)
	{
	xnLogError(XN_MASK_MODULE_HEPLER, "Reference loadFile failed:%s.", xnGetStatusString(nRetVal));
	if (blockbf.data != NULL)
	{
	xnOSFree(blockbf.data);
	}
	return nRetVal;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "LoadRef:%s ....size: %d.", refFile, blockbf.len);

	//3.Set reference
	blockbf.blocksize = 500;
	nRetVal = CommandSetProperty(OBC_LOAD_REF, &blockbf, sizeof(block_buf_t));
	XN_IS_STATUS_OK(nRetVal);

	//4.Init Depth library
	XnUInt32 v = 0;
	//XnInt ret = obc_getversion(&v);
	xnLogInfo(XN_MASK_MODULE_HEPLER, "obc library version %d", v);

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Init obc library start...");
	//ret = obc_init_by_ref((XnUChar*)blockbf.data, blockbf.len, &ob_ops_);
	free(blockbf.data);
	blockbf.data = NULL;
	//if (ret != 0)
	{
	xnLogError(XN_MASK_MODULE_HEPLER, "Init obc library failed %d", ret);
	return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Init obc library success...");
	*/

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::LoadReference(block_buf_t* blockbf)
{
	XN_VALIDATE_INPUT_PTR(blockbf);

	/*TODO 删除IR模组相关函数 预留加载1608固件及标定文件的函数
	//Init Depth library
	XnUInt32 v = 0;
	XnInt ret = obc_getversion(&v);
	xnLogInfo(XN_MASK_MODULE_HEPLER, "obc library version %d", v);

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Init obc library start...");
	ret = obc_init_by_ref((XnUChar*)blockbf->data, blockbf->len, &ob_ops_);
	if (ret != 0)
	{
	xnLogError(XN_MASK_MODULE_HEPLER, "Init obc library failed %d", ret);
	return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Init obc library success...");
	*/
	return XN_STATUS_OK;
}


XnBool XnMx6xModulesHelper::isEngineValid()
{
	XnInt isValid = -1;
	XnInt size = sizeof(XnInt);
	XnStatus status = CommandGetProperty(OBC_ENGINE_IS_VALID, &isValid, &size);
	if (status != XN_STATUS_OK)
	{
		return FALSE;
	}

	if (isValid == 1)
	{
		return TRUE;
	}

	return FALSE;
}

XnStatus XnMx6xModulesHelper::IsCommandPropertySupport(XnInt propertyId)
{
	xnl::Hash<XnUInt32, OBC_Command_t>::ConstIterator it = obcCmdPropertiesHash.Find(propertyId);
	if (it == obcCmdPropertiesHash.End())
	{
		XnStatus status = XnMx6xModulesHelper::IsExtCommandPropertySupport(propertyId);
		if (status == XN_STATUS_OK)
		{
			return XN_STATUS_OK;
		}

		xnLogInfo(XN_MASK_MODULE_HEPLER, "Command property is not supported!");
		return XN_STATUS_NO_MATCH;
	}

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::CommandSetProperty(hw_command_t command_t, command_data_t* data_t)
{
	if (p_mx6x_handle_ == NULL)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	XnInt ret = p_mx6x_handle_->set_property(command_t, data_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Command set property failed : %d，%d.", command_t, ret);
		return XN_STATUS_ERROR;
	}

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::CommandGetProperty(hw_command_t command_t, command_data_t* data_t)
{
	if (p_mx6x_handle_ == NULL)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	XnInt ret = p_mx6x_handle_->get_property(command_t, data_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Command get property failed : %d，%d.", command_t, ret);
		return XN_STATUS_ERROR;
	}

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::CommandSetProperty(XnInt propertyId, void* data, XnInt dataSize)
{
	if (p_mx6x_handle_ == nullptr)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	xnl::Hash<XnUInt32, OBC_Command_t>::ConstIterator it = obcCmdPropertiesHash.Find(propertyId);
	OBC_Command_t obc_Cmd = it->Value();
	int value = *((int *)data);
	xnLogInfo(XN_MASK_MODULE_HEPLER, "Set property to %s , propertyId：%d ,value=%d", obc_Cmd.propertyName, obc_Cmd.command, value);

	command_data_t cmd_t;
	cmd_t.data = data;
	cmd_t.len = (XnUInt32)dataSize;

	int ret = p_mx6x_handle_->set_property(obc_Cmd.command, &cmd_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Command set property failed : %d，%d.", obc_Cmd.command, ret);
		return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Set property to %s success...", obc_Cmd.propertyName);

	return XN_STATUS_OK;
}


XnStatus XnMx6xModulesHelper::CommandGetProperty(XnInt propertyId, void* data, XnInt* pDataSize)
{
	if (p_mx6x_handle_ == NULL)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	xnl::Hash<XnUInt32, OBC_Command_t>::ConstIterator it = obcCmdPropertiesHash.Find(propertyId);
	OBC_Command_t obc_Cmd = it->Value();

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Get property to %s , propertyId：%d...", obc_Cmd.propertyName, obc_Cmd.command);

	command_data_t cmd_t;
	cmd_t.data = data;
	cmd_t.len = (XnUInt32)(*pDataSize);

	XnInt ret = p_mx6x_handle_->get_property((hw_command_t)obc_Cmd.command, &cmd_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Command get property failed : %d，%d.", propertyId, ret);
		return XN_STATUS_ERROR;
	}

	*pDataSize = cmd_t.len;

	{
		uint8_t test_val = 0;
		test_val = *(uint8_t *)data;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Get property to %s success...", obc_Cmd.propertyName);

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::IsExtCommandPropertySupport(XnInt propertyId)
{
	xnl::Hash<XnUInt32, OBC_Ext_Command_t>::ConstIterator it = ext_cmdPropertiesHash.Find(propertyId);
	if (it == ext_cmdPropertiesHash.End())
	{
		xnLogInfo(XN_MASK_MODULE_HEPLER, "Ext command property is not supported!");
		return XN_STATUS_NO_MATCH;
	}

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::ExtCommandGetProperty(XnInt propertyId, void* msg, void* data, XnInt* pDataSize)
{
	if (p_mx6x_handle_ == NULL)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	xnl::Hash<XnUInt32, OBC_Ext_Command_t>::ConstIterator it = ext_cmdPropertiesHash.Find(propertyId);
	OBC_Ext_Command_t obc_ext_Cmd = it->Value();

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Get extension property to %s , propertyId：%d, ext propertyId：%d...", obc_ext_Cmd.propertyName, obc_ext_Cmd.command, obc_ext_Cmd.ext_command);


	hw_ext_msg_t ext;
	ext.subcmd = obc_ext_Cmd.ext_command;
	ext.msg = msg;
	ext.p_data = data;

	command_data_t cmd_t;
	cmd_t.data = &ext;
	cmd_t.len = sizeof(ext);

	XnInt ret = p_mx6x_handle_->get_property((hw_command_t)obc_ext_Cmd.command, &cmd_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Extension Command get property failed : %d，%d.", propertyId, ret);
		return XN_STATUS_ERROR;
	}

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Get extension property to %s success...", obc_ext_Cmd.propertyName);

	return XN_STATUS_OK;
}

XnStatus XnMx6xModulesHelper::ExtCommandSetProperty(XnInt propertyId, void* msg, void* data, XnInt dataSize)
{
	if (p_mx6x_handle_ == NULL)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Mx6x module handle is null!");
		return XN_STATUS_ERROR;
	}

	xnl::Hash<XnUInt32, OBC_Ext_Command_t>::ConstIterator it = ext_cmdPropertiesHash.Find(propertyId);
	OBC_Ext_Command_t obc_ext_Cmd = it->Value();

	xnLogInfo(XN_MASK_MODULE_HEPLER, "Set extension property to %s , propertyId：%d, ext propertyId：%d...", obc_ext_Cmd.propertyName, obc_ext_Cmd.command, obc_ext_Cmd.ext_command);

	hw_ext_msg_t ext;
	ext.subcmd = obc_ext_Cmd.ext_command;
	ext.msg = msg;
	ext.p_data = data;

	command_data_t cmd_t;
	cmd_t.data = &ext;
	cmd_t.len = sizeof(ext);

	XnInt ret = p_mx6x_handle_->set_property(obc_ext_Cmd.command, &cmd_t);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_MODULE_HEPLER, "Extension command set property failed : %d，%d.", obc_ext_Cmd.ext_command, ret);
		return XN_STATUS_ERROR;
	}
	xnLogInfo(XN_MASK_MODULE_HEPLER, "Set extension property to %s success...", obc_ext_Cmd.propertyName);
	return XN_STATUS_OK;
}

XnMx6xModulesHelper::~XnMx6xModulesHelper()
{
	this->dothin_id = -1;
	if (m_valid)
	{
		xnOSFreeLibrary(moduleLibhandle_);
		//remove(realLibFile);
		moduleLibhandle_ = NULL;
		m_valid = false;
	}
}