#ifndef XN_MX6X_MODULES_HELPER
#define XN_MX6X_MODULES_HELPER

#include <XnStatus.h>
#include <XnLog.h>
#include "XnOS.h"
#include "XnMx6xModules.h"
#include "XnMx6xOBCTeeFuns.h"
#include "XnDothinProtocol.h"
#include "XnMx6xSLog.h"
#include <XnHash.h>
#include <iostream>
#include <fstream>


using namespace std;


#define XN_MASK_MODULE_HEPLER "XnModulesHelper"
#define XN_STRING_OPENNI "\\OpenNI2"
#define XN_MODULES_DIR "\\Modules"
#define XN_CONFIG_DIR "\\config"
#define XN_SLOG_DIR "\\Log"
#define XN_MUDULES_SEG "_"



//#define XN_FIRMWARE_FILE "\\ov9282_1.8.90.bin"
#define XN_FIRMWARE_FILE "\\orfw_app.bin" //ov9282固件

//#define XN_REF_FILE "\\B5-tiger.ref"
//#define XN_REF_FILE "\\082201T.ref"
#define XN_REF_FILE "\\004.bin" //ov9282模组对应标定文件

#define DEFINE_OBCMD_MAP(NAME, OBCMD_T, OBJ) OBJ.Set(OBC_##NAME, OBCMD_T)

#define DEFINE_OB_EXTCMD_MAP(NAME, OB_EXTCMD_T, OBJ) OBJ.Set(OBC_EXT_##NAME, OB_EXTCMD_T)



class XnMx6xModulesHelper
{
public:
	XnMx6xModulesHelper();

	~XnMx6xModulesHelper();

public:

	XnStatus Init(XnInt32 dothin_id_);
	
	mx6x_module_t* getMx6xModulesHandle(){ return p_mx6x_handle_; };

	obc_ops_t getOB_OPS(){ return ob_ops_; };
	
	XnChar* getMx6xConfigDir(){ return configDir; };

	XnStatus LoadFirmware();

	XnStatus LoadReference();

	XnStatus LoadReference(block_buf_t* blockbf);

	XnBool isEngineValid();

	/*Check if command properties are supported*/
	XnStatus IsCommandPropertySupport(XnInt propertyId);

	/*Send commands to set property values*/
	XnStatus CommandSetProperty(hw_command_t command_t, command_data_t* data_t);

	/*Send commands to get property values*/
	XnStatus CommandGetProperty(hw_command_t command_t, command_data_t* data_t);

	/*Send commands to set property values*/
	XnStatus CommandSetProperty(XnInt propertyId, void* data, XnInt dataSize);

	/*Send commands to get property values*/
	XnStatus CommandGetProperty(XnInt propertyId, void* data, XnInt* pDataSize);

	/*Send ext commands to set property values*/
	XnStatus ExtCommandSetProperty(XnInt propertyId, void* msg, void* data, XnInt dataSize);

	/*Send ext commands to get property values*/
	XnStatus ExtCommandGetProperty(XnInt propertyId, void* msg, void* data, XnInt* pDataSize);


private:
	XnBool m_valid = false;
	void AddModuleProperties();
	XnStatus LoadModulesLibrary();
	XnStatus resolvePathToConfig();
	XnStatus GetModulesDir();
	XnStatus CreateMx6xSLogDir(XnChar* rootDir);
	XnStatus LoadModules();
	XnStatus CreateModuleFilesStr(char* moduleName, char* llmoduleName);
	XnStatus CopyModuleFile(const char* src, const char* dst);
	XnStatus LoadModuleDriver(char* moduleName, mx6x_module_t** p_module);
	OBC_Command_t CreateObcCmdT(char* propertyName, mx6x_command cmd);
	OBC_Ext_Command_t CreateObcExtCmdT(char* propertyName, mx6x_command cmd, hw_ext_params_t ext_command);

	/*Check if ext command properties are supported*/
	XnStatus IsExtCommandPropertySupport(XnInt propertyId);
	
	XnInt32 dothin_id;

	mx6x_module_t* p_mx6x_handle_;

	XnChar rootDir[XN_FILE_MAX_PATH];
	XnChar modulesDir[XN_FILE_MAX_PATH];
	XnChar realLibFile[XN_FILE_MAX_PATH];
	XnChar sLogDir[XN_FILE_MAX_PATH];
	XnChar configDir[XN_FILE_MAX_PATH];

	XN_LIB_HANDLE moduleLibhandle_;
	XnDothinProtocol dothinProtocol;
	obc_ops_t ob_ops_;


	xnl::Hash<XnUInt32, OBC_Command_t> obcCmdPropertiesHash;
	xnl::Hash<XnUInt32, OBC_Ext_Command_t> ext_cmdPropertiesHash;
	





};

#endif //XN_MX6X_MODULES_HELPER