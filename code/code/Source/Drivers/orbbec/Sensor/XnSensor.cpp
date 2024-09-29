/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <string>
#include "XnSensor.h"
#include "XnSensorDepthStream.h"
#include "XnSensorImageStream.h"
#include "XnSensorIRStream.h"
#include "XnSensorAudioStream.h"
#include "XnSensorPhaseStream.h"
#include "XnSensorAIStream.h"
#include "XnDeviceSensor.h"
#include "XnHostProtocol.h"
#include "XnDeviceSensorInit.h"
#include "XnDeviceEnumeration.h"
#include <XnPsVersion.h>
#include "XnIOFileStream.h"


//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define XN_SENSOR_MAX_STREAM_COUNT                      5
#define XN_SENSOR_FRAME_SYNC_MAX_DIFF                   3
#define XN_SENSOR_DEFAULT_CLOSE_STREAMS_ON_SHUTDOWN     TRUE
#define XN_SENSOR_DEFAULT_HOST_TIMESTAMPS               FALSE
#define XN_GLOBAL_CONFIG_FILE_NAME                      "orbbec.ini"

#define FRAME_SYNC_MAX_FRAME_TIME_DIFF                  30000


#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#define XN_SENSOR_DEFAULT_USB_INTERFACE             XN_SENSOR_USB_INTERFACE_ISO_ENDPOINTS
#else
// on weak platforms (Arm), we prefer to use BULK
// BULK seems to be more stable on linux
#define XN_SENSOR_DEFAULT_USB_INTERFACE             XN_SENSOR_USB_INTERFACE_BULK_ENDPOINTS
#endif

#define MUL_DISTACNE_PARAM_SIZE 1024*1024

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
typedef struct XnWaitForSycnhedFrameData
{
    XnSensor* pThis;
    const XnChar* strDepthStream;
    const XnChar* strImageStream;
} XnWaitForSycnhedFrameData;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
XnSensor::XnSensor(XnBool bResetOnStartup /* = TRUE */, XnBool bLeanInit /* = FALSE */)
    : XnDeviceBase(),
    m_hDisconnectedCallback(NULL),
    m_ErrorState(XN_MODULE_PROPERTY_ERROR_STATE, "ErrorState", XN_STATUS_OK),
    m_ResetSensorOnStartup(XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP, "ResetOnStartup", bResetOnStartup),
    m_LeanInit(XN_MODULE_PROPERTY_LEAN_INIT, "LeanInit", bLeanInit),
    m_Interface(XN_MODULE_PROPERTY_USB_INTERFACE, "UsbInterface", bResetOnStartup ? XN_SENSOR_DEFAULT_USB_INTERFACE : XN_SENSOR_USB_INTERFACE_DEFAULT),
    m_ReadData(0, "ReadData", FALSE),
    m_FrameSync(XN_MODULE_PROPERTY_FRAME_SYNC, "FrameSync", FALSE),
    m_FirmwareFrameSync(XN_MODULE_PROPERTY_FIRMWARE_FRAME_SYNC, "FirmwareFrameSync", FALSE),
    m_CloseStreamsOnShutdown(XN_MODULE_PROPERTY_CLOSE_STREAMS_ON_SHUTDOWN, "CloseStreamsOnShutdown", XN_SENSOR_DEFAULT_CLOSE_STREAMS_ON_SHUTDOWN),
    m_HostTimestamps(XN_MODULE_PROPERTY_HOST_TIMESTAMPS, "HostTimestamps", XN_SENSOR_DEFAULT_HOST_TIMESTAMPS),
    m_FirmwareParam(XN_MODULE_PROPERTY_FIRMWARE_PARAM, "FirmwareParam", NULL),
    m_CmosBlankingUnits(XN_MODULE_PROPERTY_CMOS_BLANKING_UNITS, "BlankingUnits", NULL),
    m_CmosBlankingTime(XN_MODULE_PROPERTY_CMOS_BLANKING_TIME, "BlankingTime", NULL),
    m_Reset(XN_MODULE_PROPERTY_RESET, "Reset"),
    m_Version(XN_MODULE_PROPERTY_VERSION, "Version", &m_DevicePrivateData.Version, sizeof(m_DevicePrivateData.Version), NULL),
    m_FixedParam(XN_MODULE_PROPERTY_FIXED_PARAMS, "FixedParams", NULL),
    m_ID(XN_MODULE_PROPERTY_SERIAL_NUMBER, "ID"),
    m_DeviceName(XN_MODULE_PROPERTY_PHYSICAL_DEVICE_NAME, "PhysicalDeviceName"),
    m_VendorSpecificData(XN_MODULE_PROPERTY_VENDOR_SPECIFIC_DATA, "VendorData"),
    m_PlatformString(XN_MODULE_PROPERTY_SENSOR_PLATFORM_STRING, "SensorPlatformString"),
    m_AudioSupported(XN_MODULE_PROPERTY_AUDIO_SUPPORTED, "IsAudioSupported"),
    m_ImageSupported(XN_MODULE_PROPERTY_IMAGE_SUPPORTED, "IsImageSupported"),
    m_PhaseSupported(XN_MODULE_PROPERTY_PHASE_SUPPORTED, "IsPhaseSupported"),
    m_AISupported(XN_MODULE_PROPERTY_AI_SUPPORTED, "IsAISupported"),
    m_ImageControl(XN_MODULE_PROPERTY_IMAGE_CONTROL, "ImageControl", NULL),
    m_DepthControl(XN_MODULE_PROPERTY_DEPTH_CONTROL, "DepthControl", NULL),
    m_AHB(XN_MODULE_PROPERTY_AHB, "AHB", NULL),
    m_LedState(XN_MODULE_PROPERTY_LED_STATE, "LedState", NULL),
    m_EmitterEnabled(XN_MODULE_PROPERTY_EMITTER_STATE, "EmitterState"),
    m_EmitterEnabled_V1(XN_MODULE_PROPERTY_EMITTER_STATE_V1, "EmitterStateV1", NULL),
    //ir flood
    m_IrfloodEnabled(XN_MODULE_PROPERTY_IRFLOOD_STATE, "IrfloodState", NULL),
    m_IrfloodLevel(XN_MODULE_PROPERTY_IRFLOOD_LEVEL, "IrfloodLevel", NULL),
    //ir gain
    m_IrGain(XN_MODULE_PROPERTY_IRGAIN, "IrGain", NULL),
    m_IrExp(XN_MODULE_PROPERTY_IREXP, "IrExp", NULL),
    m_LdpEnable(XN_MODULE_PROPERTY_LDP_ENABLE, "LdpEnable", NULL),
    m_LdpScale(XN_MODULE_PROPERTY_LDP_SCALE, "LdpScale", NULL),
    m_LdpStatus(XN_MODULE_PROPERTY_LDP_STATUS, "LdpStatus", NULL),
    m_LdpThresUp(XN_MODULE_PROPERTY_LDP_THRES_UP, "LdpThresUp", NULL),
    m_LdpThresLow(XN_MODULE_PROPERTY_LDP_THRES_LOW, "LdpThresLow", NULL),
    m_LdpNoise(XN_MODULE_PROPERTY_LDP_NOIST_VALUE, "LdpNoise", NULL),
    m_AeEnabled(XN_MODULE_PROPERTY_AE, "AeEnable", NULL),
	m_HdrModeEnabled(XN_MODULE_PROPERTY_TOF_SENSOR_HDR_MODE, "HdrModeEnable", NULL),
    m_MipiTestEnabled(XN_MODULE_PROPERTY_MIPI_TEST, "MipiTestEnable", NULL),
    m_I2CReadFlash(XN_MODULE_PROPERTY_I2C_READ_FLASH_MIPI, "I2CReadFlashMipi", NULL),
    //ado change sensor
    m_ChangeSensor(XN_MODULE_PROPERTY_SENSOR_CHANGE, "ChangeSensor", NULL),
    m_PublicKey(XN_MODULE_PROPERTY_PUBLIC_KEY, "publickey", NULL),
    m_RandomString(XN_MODULE_PROPERTY_RANDOM_STRING, "randomstring", NULL),
    m_RSKey(XN_MODULE_PROPERTY_RS_KEY, "rskey", NULL),
    //laser secure
    m_IsSupportLaserSecure(XN_MODULE_PROPERTY_IS_SUPPORT_LASER_SECURE, "IsSupportLaserSecure", NULL),
    m_LaserSecureStatus(XN_MODULE_PROPERTY_LASER_SECURE_STATUS, "LaserSecureStatus", NULL),
    //laser current
    m_LaserCurrent(XN_MODULE_PROPERTY_LASER_CURRENT, "LaserCurrent", NULL),
    //soft reset
    m_SoftReset(XN_MODULE_PROPERTY_SOFT_RESET, "SoftReset", NULL),
    //switch dual camera left and right ir
    m_SwitchIr(XN_MODULE_PROPERTY_SWITCH_IR, "SwitchIr", NULL),
    //rgb ae mode
    m_RgbAeMode(XN_MODULE_PROPERTY_RGB_AE_MODE, "RgbAeMode", NULL),
    //cal ir temperature
    m_CalIrTemperature(XN_MODULE_PROPERTY_CAL_IR_TEMP, "CalIrTemperautre", NULL),
    //cal Ldmp temperature
    m_CalLdmpTemperature(XN_MODULE_PROPERTY_CAL_LDMP_TEMP, "CalLdmpTemperautre", NULL),
    //IR real time temperature
    m_RtIrTemperature(XN_MODULE_PROPERTY_RT_IR_TEMP, "RtIrTemperature", NULL),
    //Ldmp real time temperature
    m_RtLdmpTemperature(XN_MODULE_PROPERTY_RT_LDMP_TEMP, "RtLdmpTemperature", NULL),
    //IR temperature compensation coefficient
    m_IrTEMPCO(XN_MODULE_PROPERTY_IR_TEMP_COMP_CO, "IrTemperatureCompCo", NULL),
    //Ldmp temperature compensation coefficient
    m_LdmpTEMPCO(XN_MODULE_PROPERTY_LDMP_TEMP_COMP_CO, "LdmpIrTemperatureCompCo", NULL),
    //Temperature compensate
    m_TemperatureCompEnabled(XN_MODULE_PROPERTY_TEMP_COMP, "TemperautreCompensate", NULL),
    //m_DepthOptimizationEnabled
    m_DepthOptimizationEnabled(XN_MODULE_PROPERTY_DEPTH_OPTIM_STATE, "DepthOptimizationState", NULL),
    //m_DepthOptimizationEnabled
    m_ObDepthOptimParam(XN_MODULE_PROPERTY_DEPTH_OPTIM_PARAM, "ObDepthOptimizationParam", NULL),
    //Flash Distortion Param
    m_FlashDistortionParam(XN_MODULE_PROPERTY_DISTORTION_PARAM, "ObDistortionParam", NULL),
    //Multi distance calibration enable
    ObDistortionEnabled(XN_MODULE_PROPERTY_DISTORTION_STATE, "ObDistortionEnable", NULL),
    m_FirmwareLogFilter(XN_MODULE_PROPERTY_FIRMWARE_LOG_FILTER, "FirmwareLogFilter", 0),
    m_FirmwareLogInterval(XN_MODULE_PROPERTY_FIRMWARE_LOG_INTERVAL, "FirmwareLogInterval", 0),
    m_FirmwareLogPrint(XN_MODULE_PROPERTY_PRINT_FIRMWARE_LOG, "PrintFirmwareLog", FALSE),
    m_FirmwareCPUInterval(XN_MODULE_PROPERTY_FIRMWARE_CPU_INTERVAL, "FirmwareCPUInterval", 0),
    m_APCEnabled(XN_MODULE_PROPERTY_APC_ENABLED, "APCEnabled", TRUE),
    m_FirmwareTecDebugPrint(XN_MODULE_PROPERTY_FIRMWARE_TEC_DEBUG_PRINT, "TecDebugPrint", FALSE),
    m_ReadAllEndpoints(XN_MODULE_PROPERTY_READ_ALL_ENDPOINTS, "ReadAllEndpoints", 0),
    m_I2C(XN_MODULE_PROPERTY_I2C, "I2C", NULL),
    m_DeleteFile(XN_MODULE_PROPERTY_DELETE_FILE, "DeleteFile"),
    m_TecSetPoint(XN_MODULE_PROPERTY_TEC_SET_POINT, "TecSetPoint"),
    m_TecStatus(XN_MODULE_PROPERTY_TEC_STATUS, "TecStatus", NULL),
    m_TecFastConvergenceStatus(XN_MODULE_PROPERTY_TEC_FAST_CONVERGENCE_STATUS, "TecFastConvergenceStatus", NULL),
    m_EmitterSetPoint(XN_MODULE_PROPERTY_EMITTER_SET_POINT, "EmitterSetPoint"),
    m_EmitterStatus(XN_MODULE_PROPERTY_EMITTER_STATUS, "EmitterStatus", NULL),
    m_FileAttributes(XN_MODULE_PROPERTY_FILE_ATTRIBUTES, "FileAttributes", NULL),
    m_FlashFile(XN_MODULE_PROPERTY_FILE, "File", NULL),
    m_FirmwareLog(XN_MODULE_PROPERTY_FIRMWARE_LOG, "FirmwareLog", NULL),
    m_FlashChunk(XN_MODULE_PROPERTY_FLASH_CHUNK, "FlashChunk", NULL),
    m_FileList(XN_MODULE_PROPERTY_FILE_LIST, "FileList", NULL),
    m_BIST(XN_MODULE_PROPERTY_BIST, "BIST", NULL),
    m_ProjectorFault(XN_MODULE_PROPERTY_PROJECTOR_FAULT, "ProjectorFault", NULL),
    m_deviceQN(XN_MODULE_PROPERTY_QN_INFO, "FirmwareQN", NULL),
    m_verifyQN(XN_MODULE_PROPERTY_QN_VERIFY, "VerifyQN", NULL),
    m_PublicBoardVersion(XN_MODULE_PROPERTY_PUBLIC_BOARD_VERSION, "PublicBoardVersion", NULL),
    m_CupVerifyVersion(XN_MODULE_PROPERTY_CUP_VERIFY_VERSION, "CupVerifyVersion", NULL),
    m_VersionMX6300(XN_MODULE_PROPERTY_VERSION_MX6300, "VersionMX6300", NULL),
    m_D2CResolution(XN_MODULE_PROPERTY_D2C_RESOLUTION, "D2CResolution", NULL),
    m_UsbDeviceSpeed(XN_MODULE_PROPERTY_USB_SPEED, "UsbSpeed", NULL),
    m_deviceSN(XN_MODULE_PROPERTY_PRODUCT_SERIAL_NUMBER, "DeviceSN", NULL),
    m_devicePN(XN_MODULE_PROPERTY_PRODUCT_NUMBER, "DevicePN", NULL),
    m_Z0Baselie(XN_MODULE_PROPERTY_Z0_BASELINE, "Z0Baseline", NULL),
    m_cfgPN(XN_MODULE_PROPERTY_CFG_PN, "ConfigPN"),
    m_irSensorModel(XN_MODULE_PROPERTY_IRS_MODEl, "IRSensorModel"),
    m_rgbSensorModel(XN_MODULE_PROPERTY_RGBS_MODEl, "RgbSensorModel"),
    m_floodAEOptions(XN_MODULE_PROPERTY_FLOOD_AE_OPTIONS, "floodAe", NULL),
    m_emitterAEOptions(XN_MODULE_PROPERTY_EMITTER_AE_OPTIONS, "emitterAe", NULL),
    m_CoreBroadFlashId(XN_MODULE_PROPERTY_CORE_BROAD_FLASH_ID, "CoreBroadFlashId", NULL),
    m_PdEnable(XN_MODULE_PROPERTY_PD_ENABLE_STATUS, "PdEnableStatus", NULL),
    m_PdAlertStatus(XN_MODULE_PROPERTY_PD_ALERT_STATUS, "PdAlertStatus", NULL),
    m_PdUpperTlv(XN_MODULE_PROPERTY_PD_UPPER_TLV, "PdUpperThreshold", NULL),
    m_PdLowerTlv(XN_MODULE_PROPERTY_PD_LOWER_TLV, "PdLowerThreshold", NULL),
    m_PdCurTlv(XN_MODULE_PROPERTY_PD_CUR_TLV, "PdCurrentThreshold", NULL),
    m_BootLoaderPts(XN_MODULE_PROPERTY_BOOTLOADER_PTS, "BootLoaderProtectionStatus", NULL),
    m_LaserTime(XN_MODULE_PROPERTY_LASER_TIME, "LaserTime", NULL),
    m_PostFilterThreshold(XN_MODULE_PROPERTY_POSTFILTER_THRESHOLD, "PostFilterThreshold", NULL),
    m_Zpps(XN_MODULE_PROPERTY_ZPPS, "Zpps", NULL),
    m_TofSensorEnabled(XN_MODULE_PROPERTY_TOF_SENSOR_ENABLE, "TofSensorEnable", NULL),
    m_TofSensorMEAResult(XN_MODULE_PROPERTY_TOF_SENSOR_MEA_RESULT, "TofSensorMeasureResult", NULL),
    m_TofSensorAppId(XN_MODULE_PROPERTY_TOF_SENSOR_APP_ID, "TofSensorAppId", NULL),
    m_TofSensorCal(XN_MODULE_PROPERTY_TOF_SENSOR_CAL, "TofSensorCalibration", NULL),
    m_TofSensorAppEnable(XN_MODULE_PROPERTY_TOF_SENSOR_APP_START, "TofSensorAppEnable", NULL),
    m_TofSensorCalParams(XN_MODULE_PROPERTY_TOF_SENSOR_CAL_PARAMS, "TofSensorCalibrationParams", NULL),
    m_DepthIrMode(XN_MODULE_PROPERTY_DEPTH_IR_MODE, "DepthIrMode", NULL),
    m_TecEnable(XN_MODULE_PROPERTY_TEC_ENABLE, "TecEnable", NULL),
    m_EnableSubtractBG(XN_MODULE_PROPERTY_ENABLE_SUBTRACT_BG, "EnableSubtract", NULL),
    m_StateSubtractBG(XN_MODULE_PROPERTY_STATE_SUBTRACT_BG, "StateSubtract", NULL),
    m_freqMode(XN_MODULE_PROPERTY_TOF_FREQ_MODE, "FreqMode", ONI_SINGLE_FREQ_NOSHUFFLE),
    m_service(XN_MODULE_PROPERTY_START_SERVICE, "Service"),
    m_sensorID(XN_MODULE_PROPERTY_SENSOR_ID, "SensorID"),
    m_serialNumber(XN_MODULE_PROPERTY_GENERAL_SERIAL_NUMBER, "SerialNumber"),
    m_sendCmd(XN_MODULE_PROPERTY_SEND_COMMAND, "SendCmd"),
    m_deviceTimestamp(XN_MODULE_PROPERTY_QUERY_DEVICE_TIMESTAMP, "DeviceTimestamp"),
    m_platformVerion(XN_MODULE_PROPERTY_PLATFORM_VERSION, "PlatformVersion"),
    m_platformSDKVerion(XN_MODULE_PROPERTY_PLATFORM_SDK_VERSION, "PlatformSDKVersion"),
    m_sendUsbFile(XN_MODULE_PROPERTY_USB_GRNERAL_FILE, "UsbFileSend"),
    m_MotorTest(XN_MODULE_PROPERTY_MOTOR_TEST, "MotorTest", NULL),
    m_MotorPosition(XN_MODULE_PROPERTY_MOTOR_POSITION, "MotorPosition", NULL),
    m_MotorStaus(XN_MODULE_PROPERTY_MOTOR_STATUS, "MotorStatus", NULL),
    m_MotorTestCount(XN_MODULE_PROPERTY_MOTOR_TEST_COUNT, "MotorTestCount", NULL),
	m_MotorRunTime(XN_MODULE_PROPERTY_MOTOR_RUN_TIME, "MotorRunTime", NULL),
    m_MotorFeature(XN_MODULE_PROPERTY_MOTOR_FEATURE, "MotorFeature", NULL),
    m_MotorUpdownState(XN_MODULE_PROPERTY_MOTOR_UPDOWN_STATE, "MotorUpdownState", NULL),
    m_MotorUpdownTime(XN_MODULE_PROPERTY_MOTOR_UPDOWN_TIME, "MotorUpdownTime", NULL),
    m_MotorUpdownControl(XN_MODULE_PROPERTY_MOTOR_UPDOWN_CONTROL, "MotorUpdownControl", NULL),
	m_FilterLevel(XN_MODULE_PROPERTY_TOF_SENSOR_FILTER_LEVEL, "FilterLevel", 0),
	m_TofSensorIntegrationTime(XN_MODULE_PROPERTY_TOF_SENSOR_INTEGRATION_TIME, "TofSensorIntegrationTime", 0),
	m_TofSensorGain(XN_MODULE_PROPERTY_TOF_SENSOR_GAIN, "TofSensorGain", 0),
	m_TofSensorLaserInterference(XN_MODULE_PROPERTY_TOF_SENSOR_LASER_INTERFERENCE, "TofSensorLaserInterference", 0),
	m_TofSensorWorkingMode(XN_MODULE_PROPERTY_TOF_SENSOR_WORKINGMODE, "TofSensorWorkingMode", 0),
	m_TofSensorFrequency(XN_MODULE_PROPERTY_TOF_SENSOR_FREQUENCY,"TofSensorFrequency"),
	m_TofSensorDutycycle(XN_MODULE_PROPERTY_TOF_SENSOR_DUTYCYCLE,"TofSensorDutyCycle"),
	m_TofSensorDriverIcReg(XN_MODULE_PROPERTY_TOF_SENSOR_DRIVER_IC_REG8,"TofSensorDriverIcReg"),
	m_TofSensorSensorReg(XN_MODULE_PROPERTY_TOF_SENSOR_SENSOR_REG16,"TofSensorSensorReg"),
    m_javaVM(XN_MODULE_PROPERTY_JAVA_VM, "JavaVM", NULL),
    m_Firmware(&m_DevicePrivateData),
    m_FPS(),
    m_CmosInfo(&m_Firmware, &m_DevicePrivateData),
    m_SensorIO(&m_DevicePrivateData.SensorHandle),
    m_Objects(&m_Firmware, &m_DevicePrivateData, &m_FPS, &m_CmosInfo),
    m_pScheduler(NULL),
    m_pLogTask(NULL),
    m_pCPUTask(NULL),
    m_FirmwareLogDump(NULL),
    m_FrameSyncDump(NULL),
    m_bInitialized(FALSE)
{
    // reset all data
    xnOSMemSet(&m_DevicePrivateData, 0, sizeof(XnDevicePrivateData));
    ResolveGlobalConfigFileName(m_strGlobalConfigFile, sizeof(m_strGlobalConfigFile), NULL);

    m_ResetSensorOnStartup.UpdateSetCallbackToDefault();
    m_LeanInit.UpdateSetCallbackToDefault();
    m_Interface.UpdateSetCallback(SetInterfaceCallback, this);
    m_ReadData.UpdateSetCallback(SetReadDataCallback, this);
    m_FrameSync.UpdateSetCallbackToDefault();
    m_FirmwareFrameSync.UpdateSetCallback(SetFirmwareFrameSyncCallback, this);
    m_FirmwareParam.UpdateSetCallback(SetFirmwareParamCallback, this);
    m_FirmwareParam.UpdateGetCallback(GetFirmwareParamCallback, this);
    m_CmosBlankingUnits.UpdateSetCallback(SetCmosBlankingUnitsCallback, this);
    m_CmosBlankingUnits.UpdateGetCallback(GetCmosBlankingUnitsCallback, this);
    m_CmosBlankingTime.UpdateSetCallback(SetCmosBlankingTimeCallback, this);
    m_CmosBlankingTime.UpdateGetCallback(GetCmosBlankingTimeCallback, this);
    m_Reset.UpdateSetCallback(ResetCallback, this);
    m_FixedParam.UpdateGetCallback(GetFixedParamsCallback, this);
    m_CloseStreamsOnShutdown.UpdateSetCallbackToDefault();
    m_HostTimestamps.UpdateSetCallbackToDefault();

    if (COMPILE_VERSION_TYPE == COMPILE_PRODUCT_TEST_VERSION)
    {
        m_PlatformString.UpdateGetCallback(GetPlatformStringCallback, this);
    }

    m_AudioSupported.UpdateGetCallback(GetAudioSupportedCallback, this);
    m_ImageSupported.UpdateGetCallback(GetImageSupportedCallback, this);
    m_PhaseSupported.UpdateGetCallback(GetPhaseSupportedCallback, this);
    m_AISupported.UpdateGetCallback(GetBodySupportedCallback, this);
    m_ImageControl.UpdateSetCallback(SetImageCmosRegisterCallback, this);
    m_ImageControl.UpdateGetCallback(GetImageCmosRegisterCallback, this);
    m_DepthControl.UpdateSetCallback(SetDepthCmosRegisterCallback, this);
    m_DepthControl.UpdateGetCallback(GetDepthCmosRegisterCallback, this);
    m_AHB.UpdateSetCallback(WriteAHBCallback, this);
    m_AHB.UpdateGetCallback(ReadAHBCallback, this);
    m_LedState.UpdateSetCallback(SetLedStateCallback, this);

    m_EmitterEnabled.SetAlwaysSet(TRUE);
    m_EmitterEnabled.UpdateSetCallback(SetEmitterStateCallback, this);
    m_EmitterEnabled_V1.UpdateGetCallback(GetEmitterEnableCallback, this);

    //irflood switch
    m_IrfloodEnabled.UpdateGetCallback(GetIrfloodStateCallback, this);
    m_IrfloodEnabled.UpdateSetCallback(SetIrfloodStateCallback, this);

    //irflood level
    m_IrfloodLevel.UpdateGetCallback(GetIrfloodLevelCallback, this);
    m_IrfloodLevel.UpdateSetCallback(SetIrfloodLevelCallback, this);

    //irgain
    m_IrGain.UpdateSetCallback(SetIrGainCallback, this);
    m_IrGain.UpdateGetCallback(GetIrGainCallback, this);

    //ir exp
    m_IrExp.UpdateSetCallback(SetIrExpCallback, this);
    m_IrExp.UpdateGetCallback(GetIrExpCallback, this);

    //ldp enable
    m_LdpEnable.UpdateSetCallback(SetLdpEnableCallback, this);
    m_LdpEnable.UpdateGetCallback(GetLdpEnableCallback, this);

    m_LdpScale.UpdateSetCallback(SetLdpScaleCallback, this);
    m_LdpScale.UpdateGetCallback(GetLdpScaleCallback, this);

    m_LdpStatus.UpdateGetCallback(GetLdpStatusCallback, this);
    m_LdpThresUp.UpdateGetCallback(GetLdpThresUpCallback, this);
    m_LdpThresUp.UpdateSetCallback(SetLdpThresUpCallback, this);
    m_LdpThresLow.UpdateGetCallback(GetLdpThresLowCallback, this);
    m_LdpThresLow.UpdateSetCallback(SetLdpThresLowCallback, this);
    m_LdpNoise.UpdateGetCallback(GetLdpNoiseValueCallback, this),

    m_AeEnabled.UpdateSetCallback(SetAeEnableCallback, this);
    m_AeEnabled.UpdateGetCallback(GetAeStateCallback, this);

	m_HdrModeEnabled.UpdateSetCallback(SetHdrModeEnableCallback, this);
	m_HdrModeEnabled.UpdateGetCallback(GetHdrModeEnableCallback, this);
    m_MipiTestEnabled.UpdateSetCallback(SetMipiTestEnableCallback, this);
    m_MipiTestEnabled.UpdateGetCallback(GetMipiTestStateCallback, this);
    m_I2CReadFlash.UpdateGetCallback(GetI2CReadFlashCallback, this);

    //ado change sensor
    m_ChangeSensor.UpdateSetCallback(SetAdoChangeSensorCallback, this);

    //public key
    m_PublicKey.UpdateSetCallback(SetPublicKeyCallback, this);
    m_PublicKey.UpdateGetCallback(GetPublicKeyCallback, this);

    m_RandomString.UpdateGetCallback(GetRandomStringCallback, this);
    m_RSKey.UpdateSetCallback(SetRSKeyCallback, this);

    //laser secure
    m_IsSupportLaserSecure.UpdateGetCallback(IsSupportLaserSecureCallback, this);
    m_LaserSecureStatus.UpdateSetCallback(SetLaserSecureStatusCallback, this);
    m_LaserSecureStatus.UpdateGetCallback(GetLaserSecureStatusCallback, this);

    //laser current
    m_LaserCurrent.UpdateSetCallback(SetLaserCurrentCallback, this);
    m_LaserCurrent.UpdateGetCallback(GetLaserCurrentCallback, this);

    //soft reset
    m_SoftReset.UpdateSetCallback(SetSoftResetCallback, this);

    //switch ir
    m_SwitchIr.UpdateSetCallback(SetSwitchIrCallback, this);

    //rgb ae mode
    m_RgbAeMode.UpdateSetCallback(SetRgbAeModeCallback, this);
    m_RgbAeMode.UpdateGetCallback(GetRgbAeModeCallback, this);

    //cal ir temperature
    m_CalIrTemperature.UpdateSetCallback(SetCalIrTempertureCallback, this);
    m_CalIrTemperature.UpdateGetCallback(GetCalIrTempertureCallback, this);

    //cal Ldmp temperature
    m_CalLdmpTemperature.UpdateSetCallback(SetCalLdmpTempertureCallback, this);
    m_CalLdmpTemperature.UpdateGetCallback(GetCalLdmpTempertureCallback, this);
    //IR real time temperature
    m_RtIrTemperature.UpdateGetCallback(GetRtIrTempertureCallback, this);
    //Ldmp real time temperature
    m_RtLdmpTemperature.UpdateGetCallback(GetRtLdmpTempertureCallback, this);

    //IR temperature compensation coefficient
    m_IrTEMPCO.UpdateSetCallback(SetIrTempertureCoCallback, this);
    m_IrTEMPCO.UpdateGetCallback(GetIrTempertureCoCallback, this);

    //Ldmp temperature compensation coefficient
    m_LdmpTEMPCO.UpdateSetCallback(SetLdmpTempertureCoCallback, this);
    m_LdmpTEMPCO.UpdateGetCallback(GetLdmpTempertureCoCallback, this);

    //Temperature compensation enable
    m_TemperatureCompEnabled.UpdateSetCallback(SetTemperatureCompStateCallback, this);
    m_TemperatureCompEnabled.UpdateGetCallback(GetTemperatureCompStateCallback, this);

    //Depth optimization enable
    m_DepthOptimizationEnabled.UpdateSetCallback(SetDepthOptimizationStateCallback, this);
    m_DepthOptimizationEnabled.UpdateGetCallback(GetDepthOptimizationStateCallback, this);

    m_ObDepthOptimParam.UpdateSetCallback(setObDepthOptimizationParamCallback, this);
    m_ObDepthOptimParam.UpdateGetCallback(GetObDepthOptimizationParamCallback, this);

    //Distortion Param
    m_FlashDistortionParam.UpdateSetCallback(WriteFlashDistortionParamCallback, this);
    m_FlashDistortionParam.UpdateGetCallback(ReadFlashDistortionParamCallback, this);

    ObDistortionEnabled.UpdateSetCallback(SetDepthDistortionStateCallback, this);
    ObDistortionEnabled.UpdateGetCallback(GetDepthDistortionStateCallback, this);

    m_FirmwareLogInterval.UpdateSetCallback(SetFirmwareLogIntervalCallback, this);
    m_FirmwareLogPrint.UpdateSetCallback(SetFirmwareLogPrintCallback, this);
    m_FirmwareCPUInterval.UpdateSetCallback(SetFirmwareCPUIntervalCallback, this);
    m_DeleteFile.UpdateSetCallback(DeleteFileCallback, this);
    m_FirmwareLogFilter.UpdateSetCallback(SetFirmwareLogFilterCallback, this);
    m_APCEnabled.UpdateSetCallback(SetAPCEnabledCallback, this);
    m_TecSetPoint.UpdateSetCallback(SetTecSetPointCallback, this);
    m_TecStatus.UpdateGetCallback(GetTecStatusCallback, this);
    m_TecFastConvergenceStatus.UpdateGetCallback(GetTecFastConvergenceStatusCallback, this);
    m_EmitterSetPoint.UpdateSetCallback(SetEmitterSetPointCallback, this);
    m_EmitterStatus.UpdateGetCallback(GetEmitterStatusCallback, this);
    m_I2C.UpdateSetCallback(SetI2CCallback, this);
    m_I2C.UpdateGetCallback(GetI2CCallback, this);
    m_FileAttributes.UpdateSetCallback(SetFileAttributesCallback, this);
    m_FlashFile.UpdateSetCallback(WriteFlashFileCallback, this);
    m_FlashFile.UpdateGetCallback(ReadFlashFileCallback, this);
    m_FirmwareLog.UpdateGetCallback(GetFirmwareLogCallback, this);
    m_FlashChunk.UpdateGetCallback(ReadFlashChunkCallback, this);
    m_FileList.UpdateGetCallback(GetFileListCallback, this);
    m_BIST.UpdateSetCallback(RunBISTCallback, this);
    m_ProjectorFault.UpdateSetCallback(SetProjectorFaultCallback, this);
    m_FirmwareTecDebugPrint.UpdateSetCallbackToDefault();
    m_ReadAllEndpoints.UpdateSetCallback(SetReadAllEndpointsCallback, this);
    m_deviceQN.UpdateGetCallback(GetFirmwareQNCallback, this);
    m_deviceQN.UpdateSetCallback(SetFirmwareQNCallback, this);
    m_verifyQN.UpdateSetCallback(VerifyQNCallback, this);
    m_VersionMX6300.UpdateGetCallback(GetMX6300VersionCallback, this);
    m_PublicBoardVersion.UpdateGetCallback(GetPublicBoardVersionCallback, this);
    m_CupVerifyVersion.UpdateGetCallback(GetCupVerifyVersionCallback, this);
    m_D2CResolution.UpdateSetCallback(SetD2CResolutionCallback, this);
    m_D2CResolution.UpdateGetCallback(GetD2CResolutionCallback, this);
    m_UsbDeviceSpeed.UpdateGetCallback(GetUsbDeviceSpeedCallback, this);
    m_deviceSN.UpdateSetCallback(SetDeviceSerialNumberCallback, this);
    m_deviceSN.UpdateGetCallback(GetDeviceSerialNumberCallback, this);

    m_devicePN.UpdateSetCallback(SetDevicePNCallback, this);
    m_devicePN.UpdateGetCallback(GetDevicePNCallback, this);
    m_Z0Baselie.UpdateGetCallback(GetZ0BaselineCallback, this);

    m_PdEnable.UpdateGetCallback(GetPdEnableCallback, this);
    m_PdEnable.UpdateSetCallback(SetPdEnableCallback, this);

    m_PdAlertStatus.UpdateGetCallback(GetPdAlertCallback, this);

    m_PdUpperTlv.UpdateGetCallback(GetPdUpperTlvCallback, this);
    m_PdUpperTlv.UpdateSetCallback(SetPdUpperTlvCallback, this);

    m_PdLowerTlv.UpdateGetCallback(GetPdLowerTlvCallback, this);
    m_PdLowerTlv.UpdateSetCallback(SetPdLowerTlvCallback, this);

    m_PdCurTlv.UpdateGetCallback(GetDevicePdCurrenthresholdCallback, this);

    m_BootLoaderPts.UpdateGetCallback(GetBootLoaderPtsCallback, this);
    m_BootLoaderPts.UpdateSetCallback(SetBootLoaderPtsCallback, this);

    m_LaserTime.UpdateSetCallback(SetLaserTimeCallback, this);
    m_LaserTime.UpdateGetCallback(GetLaserTimeCallback, this);

    m_PostFilterThreshold.UpdateSetCallback(SetPostFilterThresholdCallback, this);
    m_PostFilterThreshold.UpdateGetCallback(GetPostFilterThresholdCallback, this);

    m_Zpps.UpdateGetCallback(GetZppsCallback, this);


    m_TofSensorEnabled.UpdateGetCallback(GetTofSensorEnableStateCallback, this);
    m_TofSensorEnabled.UpdateSetCallback(SetTofSensorEnableStateCallback, this);
    m_TofSensorMEAResult.UpdateGetCallback(GetTofSensorMeasureResultCallback, this);
    m_TofSensorAppId.UpdateGetCallback(GetTofSensorAppIdCallback, this);
    m_TofSensorCal.UpdateSetCallback(SetTofSensorcalibrationCallback, this);
    m_TofSensorAppEnable.UpdateSetCallback(SetTofSensorAppEnableStateCallback, this);
    m_TofSensorCalParams.UpdateGetCallback(GetTofSensorcalibrationParamsCallback, this);
    m_TofSensorCalParams.UpdateSetCallback(SetTofSensorcalibrationParamsCallback, this);

    //Motor
    m_MotorTest.UpdateGetCallback(GetMotorTestResultCallback, this);
    m_MotorTest.UpdateSetCallback(SetMotorTestCallback, this);
    m_MotorPosition.UpdateGetCallback(GetMotorPositionCallback, this);
    m_MotorPosition.UpdateSetCallback(SetMotorPositionCallback, this);
    m_MotorStaus.UpdateGetCallback(GetMotorStatusCallback, this);
    //m_MotorStaus.UpdateSetCallback(SetMotorStatusCallback, this);
	m_MotorTestCount.UpdateGetCallback(GetMotorTestCountCallback, this);
	m_MotorRunTime.UpdateSetCallback(SetMotorRunTimeCallback, this);
	m_MotorRunTime.UpdateGetCallback(GetMotorRunTimeCallback, this);
    m_MotorFeature.UpdateGetCallback(GetMotorFeatureCallback, this);
    m_MotorUpdownState.UpdateGetCallback(GetMotorUpdownStateCallback, this);
    m_MotorUpdownTime.UpdateGetCallback(GetMotorUpdownTimeCallback, this);
    m_MotorUpdownControl.UpdateSetCallback(SetMotorUpdownCallback, this);

    m_DepthIrMode.UpdateSetCallback(SetDepthIrModeCallback, this);
    m_DepthIrMode.UpdateGetCallback(GetDepthIrModeCallback, this);
    m_TecEnable.UpdateSetCallback(SetSetTecEnableCallback, this);

    m_EnableSubtractBG.UpdateSetCallback(SetSubtractBGCallback, this);
    m_StateSubtractBG.UpdateGetCallback(GetSubtractBGCallback, this);

    m_cfgPN.UpdateGetCallback(GetDeviceCfgPNCallback, this);
    m_irSensorModel.UpdateGetCallback(GetIRSensorModelCallback, this);
    m_rgbSensorModel.UpdateGetCallback(GetRgbSensorModelCallback, this);

    m_floodAEOptions.UpdateSetCallback(SetFloodAEOptionsCallback, this);
    m_floodAEOptions.UpdateGetCallback(GetFloodAEOptionsCallback, this);
    m_emitterAEOptions.UpdateSetCallback(SetEmitterAEOptionsCallback, this);
    m_emitterAEOptions.UpdateGetCallback(GetEmitterAEOptionsCallback, this);

    m_CoreBroadFlashId.UpdateGetCallback(GetCoreBroadFlashIdCallback, this);

    m_freqMode.SetAlwaysSet(TRUE);
    m_freqMode.UpdateGetCallback(GetTOFFreqModeCallback, this);
    m_freqMode.UpdateSetCallback(SetTOFFreqModeCallback, this);

    m_sensorID.UpdateGetCallback(GetSensorIDCallback, this);
    m_serialNumber.UpdateGetCallback(GetGeneralSerialNumberCallback, this);
    m_serialNumber.UpdateSetCallback(SetGeneralSerialNumberCallback, this);
    m_platformVerion.UpdateGetCallback(GetPlatformVersionCallback, this);
    m_platformSDKVerion.UpdateGetCallback(GetPlatformSDKVersionCallback, this);

    m_sendCmd.UpdateSetCallback(SendCommandCallback, this);
    m_sendUsbFile.UpdateSetCallback(SendUsbFileCallback, this);

	m_FilterLevel.UpdateGetCallback(GetTOFSensorFilterLevelCallback, this),
	m_FilterLevel.UpdateSetCallback(SetTOFSensorFilterLevelCallback, this),
	m_TofSensorIntegrationTime.UpdateGetCallback(GetTOFSensorIntegrationTimeCallback, this),
	m_TofSensorIntegrationTime.UpdateSetCallback(SetTOFSensorIntegrationTimeCallback, this),
	m_TofSensorGain.UpdateGetCallback(GetTOFSensorGainCallback, this),
	m_TofSensorGain.UpdateSetCallback(SetTOFSensorGainCallback, this),
	m_TofSensorLaserInterference.UpdateGetCallback(GetTOFSensorLaserInterferenceCallback, this),
	m_TofSensorLaserInterference.UpdateSetCallback(SetTOFSensorLaserInterferenceCallback, this),
	m_TofSensorWorkingMode.UpdateGetCallback(GetTOFSensorWorkingModeCallback, this),
	m_TofSensorWorkingMode.UpdateSetCallback(SetTOFSensorWorkingModeCallback, this),
	m_TofSensorFrequency.UpdateGetCallback(GetTOFSensorFrequencyCallback,this),
	m_TofSensorFrequency.UpdateSetCallback(SetTOFSensorFrequencyCallback,this),
	m_TofSensorDutycycle.UpdateGetCallback(GetTOFSensorDutyCycleCallback,this),
	m_TofSensorDutycycle.UpdateSetCallback(SetTOFSensorDutyCycleCallback,this),
	m_TofSensorDriverIcReg.UpdateGetCallback(GetTOFSensorDriverICRegCallback,this),
	m_TofSensorDriverIcReg.UpdateSetCallback(SetTOFSensorDriverICRegCallback,this),
	m_TofSensorSensorReg.UpdateGetCallback(GetTOFSensorSensorRegCallback,this),
	m_TofSensorSensorReg.UpdateSetCallback(SetTOFSensorSensorRegCallback,this),
    m_javaVM.UpdateSetCallback(SetJavaVMCallback, this);
    m_service.UpdateSetCallback(StartServiceCallback, this);

    m_deviceTimestamp.UpdateGetCallback(QueryDeviceTimestampCallback, this);

    // Clear the frame-synced streams.
    m_nFrameSyncEnabled = FALSE;
    m_nFrameSyncLastFrameID = 0;
    m_nPId = 0;


    m_distortionParam.data = new XnUInt8[MUL_DISTACNE_PARAM_SIZE];
    m_distortionParam.nSize = MUL_DISTACNE_PARAM_SIZE;
    m_bReadDistortParam = FALSE;

    m_bReadCameraParam = FALSE;
    m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;

    xnOSMemSet(&m_ObContent, 0, sizeof(m_ObContent));
    xnOSMemSet(&m_ObCameraParams, 0, sizeof(m_ObCameraParams));
}

XnSensor::~XnSensor()
{

    if (m_distortionParam.data != NULL)
    {
        delete[]m_distortionParam.data;
        m_distortionParam.data = NULL;
    }
    XnSensor::Destroy();
}

XnStatus XnSensor::InitImpl(const XnDeviceConfig *pDeviceConfig)
{
    XnStatus nRetVal = XN_STATUS_OK;

    xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Initializing device sensor...");

    nRetVal = xnSchedulerStart(&m_pScheduler);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_PropSynchronizer.RegisterSynchronization(&m_Firmware.GetParams()->m_APCEnabled, &m_APCEnabled);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_PropSynchronizer.RegisterSynchronization(&m_Firmware.GetParams()->m_LogFilter, &m_FirmwareLogFilter);
    XN_IS_STATUS_OK(nRetVal);

    // Frame Sync
    XnCallbackHandle hCallbackDummy;
    nRetVal = m_FrameSync.OnChangeEvent().Register(FrameSyncPropertyChangedCallback, this, hCallbackDummy);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetFirmware()->GetParams()->m_Stream0Mode.OnChangeEvent().Register(FrameSyncPropertyChangedCallback, this, hCallbackDummy);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = GetFirmware()->GetParams()->m_Stream1Mode.OnChangeEvent().Register(FrameSyncPropertyChangedCallback, this, hCallbackDummy);
    XN_IS_STATUS_OK(nRetVal);

    // other stuff
    m_FrameSyncDump = xnDumpFileOpen(XN_DUMP_FRAME_SYNC, "FrameSync.csv");
    xnDumpFileWriteString(m_FrameSyncDump, "HostTime(us),DepthNewData,DepthTimestamp(ms),ImageNewData,ImageTimestamp(ms),Diff(ms),Action\n");

    nRetVal = XnDeviceBase::InitImpl(pDeviceConfig);
    XN_IS_STATUS_OK(nRetVal);

    // now that everything is configured, open the sensor
    nRetVal = InitSensor(pDeviceConfig);
    if (nRetVal != XN_STATUS_OK)
    {
        Destroy();
        return (nRetVal);
    }

    // configure it from global file.
    XnDeviceModule* pModule = NULL;
    FindModule(XN_MODULE_NAME_DEVICE, &pModule);
    if (pModule && '\0' != m_strGlobalConfigFile[0])
    {
        nRetVal = pModule->LoadConfigFromFile(m_strGlobalConfigFile);
        XN_IS_STATUS_OK(nRetVal);
    }

    nRetVal = XnDeviceEnumeration::DisconnectedEvent().Register(OnDeviceDisconnected, this, m_hDisconnectedCallback);
    XN_IS_STATUS_OK(nRetVal);

    XnDeviceEnumeration::DisconnectedEvent().DisconnectCallbackSet(m_hDisconnectedCallback);

    xnLogInfo(XN_MASK_DEVICE_SENSOR, "Device sensor initialized");

    return (XN_STATUS_OK);
}

XnStatus XnSensor::InitSensor(const XnDeviceConfig* pDeviceConfig)
{
    XnStatus nRetVal = XN_STATUS_OK;
    XnDevicePrivateData* pDevicePrivateData = GetDevicePrivateData();

    pDevicePrivateData->pSensor = this;

    // open IO
    nRetVal = m_SensorIO.OpenDevice(pDeviceConfig->cpConnectionString);
    XN_IS_STATUS_OK(nRetVal);

    // initialize
    nRetVal = XnDeviceSensorInit(pDevicePrivateData);
    XN_IS_STATUS_OK(nRetVal);

    // init firmware
    nRetVal = m_Firmware.Init((XnBool)m_ResetSensorOnStartup.GetValue(), (XnBool)m_LeanInit.GetValue());
    XN_IS_STATUS_OK(nRetVal);
    m_bInitialized = TRUE;

    m_ResetSensorOnStartup.UpdateSetCallback(NULL, NULL);
    m_LeanInit.UpdateSetCallback(NULL, NULL);

    // update device info properties
    nRetVal = m_DeviceName.UnsafeUpdateValue(GetFixedParams()->GetDeviceName());
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = m_VendorSpecificData.UnsafeUpdateValue(GetFixedParams()->GetVendorData());
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = m_ID.UnsafeUpdateValue(GetFixedParams()->GetSensorSerial());
    XN_IS_STATUS_OK(nRetVal);
    nRetVal = m_PlatformString.UnsafeUpdateValue(GetFixedParams()->GetPlatformString());
    XN_IS_STATUS_OK(nRetVal);

    // Add supported streams
    AddSupportedStream(XN_STREAM_TYPE_DEPTH);
    AddSupportedStream(XN_STREAM_TYPE_IR);

    if (GetFirmware()->GetInfo()->bImageSupported)
    {
        AddSupportedStream(XN_STREAM_TYPE_IMAGE);
    }

    // if (GetFirmware()->GetInfo()->bPhaseSupported)
    // {
    //     AddSupportedStream(XN_STREAM_TYPE_PHASE);
    // }

    if (GetFirmware()->GetInfo()->bAISupported)
    {
        AddSupportedStream(XN_STREAM_TYPE_AI);
    }

    if (GetFirmware()->GetInfo()->bAudioSupported)
    {
        AddSupportedStream(XN_STREAM_TYPE_AUDIO);
    }

    XnStatus rcState = GetObDistortionEnableState(m_bMultiDisCalEnable);
    if ((XN_STATUS_OK == rcState) && (m_bMultiDisCalEnable == MULTI_DISCAL_ENABLE))
    {
        rcState = ReadFlashDistortionParam(&m_distortionParam);
        if (rcState != XN_STATUS_OK)
        {
            m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;
        }
    }

    return XN_STATUS_OK;
}

XnStatus XnSensor::Destroy()
{
    XnDevicePrivateData* pDevicePrivateData = GetDevicePrivateData();

    if (m_hDisconnectedCallback != NULL)
    {
        XnDeviceEnumeration::DisconnectedEvent().Unregister(m_hDisconnectedCallback);
        m_hDisconnectedCallback = NULL;
    }

    // close Commands.txt thread
    if (pDevicePrivateData->LogThread.hThread != NULL)
    {
        pDevicePrivateData->LogThread.bKillThread = TRUE;

        xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Shutting down Sensor commands.txt thread...");
        xnOSWaitAndTerminateThread(&pDevicePrivateData->LogThread.hThread, XN_DEVICE_SENSOR_THREAD_KILL_TIMEOUT);
        pDevicePrivateData->LogThread.hThread = NULL;
    }

    // if needed, close the streams
    if (m_bInitialized && m_CloseStreamsOnShutdown.GetValue() == TRUE &&
        m_ReadData.GetValue() == TRUE && m_ErrorState.GetValue() != XN_STATUS_DEVICE_NOT_CONNECTED)
    {
        m_Firmware.GetParams()->m_Stream0Mode.SetValue(XN_VIDEO_STREAM_OFF);
        m_Firmware.GetParams()->m_Stream1Mode.SetValue(XN_VIDEO_STREAM_OFF);
        m_Firmware.GetParams()->m_Stream2Mode.SetValue(XN_AUDIO_STREAM_OFF);
        if (pDevicePrivateData->FWInfo.bAISupported)
            m_Firmware.GetParams()->m_Stream3Mode.SetValue(XN_VIDEO_STREAM_OFF);
    }

    // close IO (including all reading threads)
    m_SensorIO.CloseDevice();
    m_bInitialized = FALSE;

    // shutdown scheduler
    if (m_pScheduler != NULL)
    {
        xnSchedulerShutdown(&m_pScheduler);
        m_pScheduler = NULL;
    }

    if (pDevicePrivateData->hEndPointsCS != NULL)
    {
        xnOSCloseCriticalSection(&pDevicePrivateData->hEndPointsCS);
        pDevicePrivateData->hEndPointsCS = NULL;
    }

    // free buffers
    XnDeviceSensorFreeBuffers(pDevicePrivateData);

    if (pDevicePrivateData->hExecuteMutex != NULL)
    {
        xnOSCloseMutex(&pDevicePrivateData->hExecuteMutex);
        pDevicePrivateData->hExecuteMutex = NULL;
    }

    XnDeviceBase::Destroy();

    // close dumps
    xnDumpFileClose(pDevicePrivateData->TimestampsDump);
    xnDumpFileClose(pDevicePrivateData->BandwidthDump);
    xnDumpFileClose(pDevicePrivateData->MiniPacketsDump);
    xnDumpFileClose(m_FrameSyncDump);
    xnDumpFileClose(m_FirmwareLogDump);

    m_Firmware.Free();

    return (XN_STATUS_OK);
}

XnStatus XnSensor::CreateDeviceModule(XnDeviceModuleHolder** ppModuleHolder)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnDeviceBase::CreateDeviceModule(ppModuleHolder);
    XN_IS_STATUS_OK(nRetVal);

    // add sensor properties
    XnDeviceModule* pModule = (*ppModuleHolder)->GetModule();
    XnProperty* pProps[] =
    {
        &m_ErrorState, &m_ResetSensorOnStartup, &m_LeanInit, &m_Interface, &m_ReadData, &m_FirmwareParam, &m_CmosBlankingUnits,
        &m_CmosBlankingTime, &m_Reset, &m_Version, &m_FixedParam, &m_FrameSync, &m_FirmwareFrameSync, &m_CloseStreamsOnShutdown,
        &m_ID, &m_VendorSpecificData, &m_AudioSupported, &m_ImageSupported, &m_ImageControl, &m_DepthControl, &m_AHB, &m_LedState,
        &m_EmitterEnabled, &m_IrfloodEnabled, &m_IrfloodLevel, &m_IrGain, &m_IrExp, &m_ChangeSensor, &m_PublicKey, &m_RandomString,
        &m_RSKey, &m_IsSupportLaserSecure, &m_LaserSecureStatus, &m_LaserCurrent, &m_SwitchIr, &m_SoftReset, &m_RgbAeMode, &m_CalIrTemperature,
        &m_CalLdmpTemperature, &m_RtIrTemperature, &m_RtLdmpTemperature, &m_IrTEMPCO, &m_LdmpTEMPCO, &m_TemperatureCompEnabled, &m_LdpEnable,
        &m_EmitterEnabled_V1, &m_LdpScale, &m_LdpStatus, &m_LdpThresUp, &m_LdpThresLow, &m_LdpNoise, &m_DepthOptimizationEnabled, &m_ObDepthOptimParam,
        &m_HostTimestamps, &m_PlatformString, &m_FirmwareLogInterval, &m_FirmwareLogPrint, &m_FirmwareCPUInterval, &m_DeleteFile, &m_APCEnabled,
        &m_TecSetPoint, &m_TecStatus, &m_TecFastConvergenceStatus, &m_EmitterSetPoint, &m_EmitterStatus, &m_I2C, &m_FileAttributes, &m_FlashFile,
        &m_FirmwareLogFilter, &m_FirmwareLog, &m_FlashChunk, &m_FileList, &m_ProjectorFault, &m_BIST, &m_FirmwareTecDebugPrint, &m_DeviceName,
        &m_ReadAllEndpoints, &m_FlashDistortionParam, &ObDistortionEnabled, &m_deviceQN, &m_verifyQN, &m_PublicBoardVersion, &m_CupVerifyVersion,
		&m_VersionMX6300, &m_D2CResolution, &m_UsbDeviceSpeed, &m_deviceSN, &m_devicePN, &m_cfgPN, &m_irSensorModel, &m_rgbSensorModel, &m_AeEnabled,&m_HdrModeEnabled,
        &m_MipiTestEnabled, &m_Z0Baselie, &m_I2CReadFlash, &m_PdEnable, &m_PdAlertStatus, &m_PdUpperTlv, &m_PdLowerTlv, &m_PdCurTlv, &m_BootLoaderPts,
        &m_floodAEOptions, &m_emitterAEOptions, &m_CoreBroadFlashId, &m_LaserTime, &m_PostFilterThreshold, &m_Zpps, &m_TofSensorEnabled, &m_TofSensorMEAResult,
        &m_TofSensorAppId, &m_TofSensorCal, &m_TofSensorAppEnable, &m_TofSensorCalParams, &m_DepthIrMode, &m_TecEnable, &m_EnableSubtractBG, &m_StateSubtractBG,
        &m_platformVerion, &m_platformSDKVerion, &m_sendUsbFile, &m_MotorTest, &m_MotorPosition, &m_MotorStaus, &m_MotorTestCount,
        &m_MotorRunTime, &m_PhaseSupported, &m_AISupported, &m_freqMode, &m_sensorID, &m_serialNumber, &m_service, &m_sendCmd, &m_deviceTimestamp, &m_javaVM,
		&m_MotorFeature, &m_MotorUpdownState, &m_MotorUpdownTime, &m_MotorUpdownControl,&m_FilterLevel,&m_TofSensorIntegrationTime,&m_TofSensorGain,&m_TofSensorLaserInterference,
		&m_TofSensorWorkingMode,&m_TofSensorFrequency,&m_TofSensorDutycycle,&m_TofSensorDriverIcReg,&m_TofSensorSensorReg
    };

    nRetVal = pModule->AddProperties(pProps, sizeof(pProps) / sizeof(XnProperty*));
    if (nRetVal != XN_STATUS_OK)
    {
        DestroyModule(*ppModuleHolder);
        *ppModuleHolder = NULL;
        return (nRetVal);
    }

    // configure it from global file
    /*if (m_strGlobalConfigFile[0] != '\0')
    {
        nRetVal = pModule->LoadConfigFromFile(m_strGlobalConfigFile);
        XN_IS_STATUS_OK(nRetVal);
    }*/

    return (XN_STATUS_OK);
}

XnStatus XnSensor::CreateStreamImpl(const XnChar* strType, const XnChar* strName, const XnActualPropertiesHash* pInitialSet)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnDeviceBase::CreateStreamImpl(strType, strName, pInitialSet);
    XN_IS_STATUS_OK(nRetVal);

    // and configure it from global config file
    nRetVal = ConfigureModuleFromGlobalFile(strName, strType);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::CreateStreamModule(const XnChar* StreamType, const XnChar* StreamName, XnDeviceModuleHolder** ppStreamHolder)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // make sure reading from streams is turned on
    if (!m_ReadData.GetValue())
    {
        nRetVal = m_ReadData.SetValue(TRUE);
        XN_IS_STATUS_OK(nRetVal);
    }

    XnDeviceStream* pStream;
    XnSensorStreamHelper* pHelper;

    // create stream
    if (strcmp(StreamType, XN_STREAM_TYPE_DEPTH) == 0)
    {
        XnSensorDepthStream* pDepthStream;
        XN_VALIDATE_NEW(pDepthStream, XnSensorDepthStream, StreamName, &m_Objects);
        pDepthStream->SetDriverConfig(m_strGlobalConfigFile, XN_FILE_MAX_PATH);
        pStream = pDepthStream;
        pHelper = pDepthStream->GetHelper();
    }
    else if (strcmp(StreamType, XN_STREAM_TYPE_IMAGE) == 0)
    {
        XnSensorImageStream* pImageStream;
        XN_VALIDATE_NEW(pImageStream, XnSensorImageStream, StreamName, &m_Objects);
        pStream = pImageStream;
        pHelper = pImageStream->GetHelper();
    }
    else if (strcmp(StreamType, XN_STREAM_TYPE_IR) == 0)
    {
        XnSensorIRStream* pIRStream;
        XN_VALIDATE_NEW(pIRStream, XnSensorIRStream, StreamName, &m_Objects);
        pIRStream->SetDriverConfig(m_strGlobalConfigFile, XN_FILE_MAX_PATH);
        pStream = pIRStream;
        pHelper = pIRStream->GetHelper();
    }
    else if (strcmp(StreamType, XN_STREAM_TYPE_AUDIO) == 0)
    {
        // TODO: enable
        XN_ASSERT(FALSE);
        pStream = NULL;
        pHelper = NULL;
        /*if (!m_Firmware.GetInfo()->bAudioSupported)
        {
        XN_LOG_WARNING_RETURN(XN_STATUS_UNSUPPORTED_STREAM, XN_MASK_DEVICE_SENSOR, "Audio is not supported by this FW!");
        }

        // TODO: use the allow other users property when constructing the audio stream
        XnSensorAudioStream* pAudioStream;
        XN_VALIDATE_NEW(pAudioStream, XnSensorAudioStream, GetUSBPath(), StreamName, &m_Objects, FALSE);
        pStream = pAudioStream;
        pHelper = pAudioStream->GetHelper();*/
    }
    else if (strcmp(StreamType, XN_STREAM_TYPE_AI) == 0)
    {
        XnSensorAIStream* pAIStream = NULL;
        XN_VALIDATE_NEW(pAIStream, XnSensorAIStream, StreamName, &m_Objects);
        pAIStream->SetDriverConfig(m_strGlobalConfigFile, XN_FILE_MAX_PATH);
        pStream = pAIStream;
        pHelper = pAIStream->GetHelper();
    }
    // @deprecated Used @ref XnSensorAIStream instead.
    // else if (strcmp(StreamType, XN_STREAM_TYPE_PHASE) == 0)
    // {
    //     XnSensorPhaseStream* pPhaseStream;
    //     XN_VALIDATE_NEW(pPhaseStream, XnSensorPhaseStream, StreamName, &m_Objects);
    //     pPhaseStream->SetDriverConfig(m_strGlobalConfigFile, XN_FILE_MAX_PATH);
    //     pStream = pPhaseStream;
    //     pHelper = pPhaseStream->GetHelper();
    // }
    else
    {
        XN_LOG_WARNING_RETURN(XN_STATUS_UNSUPPORTED_STREAM, XN_MASK_DEVICE_SENSOR, "Unsupported stream type: %s", StreamType);
    }

    *ppStreamHolder = XN_NEW(XnSensorStreamHolder, pStream, pHelper);

    return (XN_STATUS_OK);
}

void XnSensor::DestroyStreamModule(XnDeviceModuleHolder* pStreamHolder)
{
    XN_DELETE(pStreamHolder->GetModule());
    XN_DELETE(pStreamHolder);
}

XnStatus XnSensor::OpenAllStreams()
{
    XnStatus nRetVal = XN_STATUS_OK;

    xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Opening all streams...");

    // take a list of all the streams
    const XnChar* astrStreams[XN_SENSOR_MAX_STREAM_COUNT];
    XnUInt32 nStreamCount = XN_SENSOR_MAX_STREAM_COUNT;
    XnDeviceStream* apStreams[XN_SENSOR_MAX_STREAM_COUNT];
    XnSensorStreamHolder* apSensorStreams[XN_SENSOR_MAX_STREAM_COUNT];

    nRetVal = GetStreamNames(astrStreams, &nStreamCount);
    XN_IS_STATUS_OK(nRetVal);

    for (XnUInt32 i = 0; i < nStreamCount; ++i)
    {
        XnDeviceModuleHolder* pHolder;
        nRetVal = FindStream(astrStreams[i], &pHolder);
        XN_IS_STATUS_OK(nRetVal);

        apSensorStreams[i] = (XnSensorStreamHolder*)(pHolder);
        apStreams[i] = apSensorStreams[i]->GetStream();
    }

    // NOTE: the following is an ugly patch. When depth and IR both exist, Depth stream MUST be configured
    // and opened BEFORE IR stream. So, generally, if one of the streams is depth, we move it to be first.
    for (XnUInt32 i = 1; i < nStreamCount; ++i)
    {
        if (strcmp(apStreams[i]->GetType(), XN_STREAM_TYPE_DEPTH) == 0)
        {
            // switch it with the one in location 0
            const XnChar* strTempName = astrStreams[0];
            XnDeviceStream* pTempStream = apStreams[0];
            XnSensorStreamHolder* pTempHolder = apSensorStreams[0];

            astrStreams[0] = astrStreams[i];
            apStreams[0] = apStreams[i];
            apSensorStreams[0] = apSensorStreams[i];

            astrStreams[i] = strTempName;
            apStreams[i] = pTempStream;
            apSensorStreams[i] = pTempHolder;
            break;
        }
    }

    // now configure them all
    for (XnUInt32 i = 0; i < nStreamCount; ++i)
    {
        if (!apStreams[i]->IsOpen())
        {
            xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Configuring stream %s...", apStreams[i]->GetName());
            nRetVal = apSensorStreams[i]->Configure();
            XN_IS_STATUS_OK(nRetVal);
            xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Stream %s is configured", apStreams[i]->GetName());
        }
        else
        {
            xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Stream %s is already open.", apStreams[i]->GetName());
        }
    }

    // and open them all
    for (XnUInt32 i = 0; i < nStreamCount; ++i)
    {
        if (!apStreams[i]->IsOpen())
        {
            nRetVal = apSensorStreams[i]->FinalOpen();
            XN_IS_STATUS_OK(nRetVal);
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetStream(const XnChar* strStream, XnDeviceStream** ppStream)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XnDeviceModuleHolder* pHolder;
    nRetVal = FindStream(strStream, &pHolder);
    XN_IS_STATUS_OK(nRetVal);

    XnSensorStreamHolder* pSensorStreamHolder = (XnSensorStreamHolder*)(pHolder);
    *ppStream = pSensorStreamHolder->GetStream();

    return XN_STATUS_OK;
}

XnStatus XnSensor::LoadConfigFromFile(const XnChar* csINIFilePath, const XnChar* csSectionName)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XN_VALIDATE_INPUT_PTR(csINIFilePath);
    XN_VALIDATE_INPUT_PTR(csSectionName);

    // we first need to configure the USB interface (we want to do so BEFORE creating streams)
    nRetVal = m_Interface.ReadValueFromFile(csINIFilePath, XN_MODULE_NAME_DEVICE);
    XN_IS_STATUS_OK(nRetVal);

    // now configure DEVICE module (primary stream, global mirror, etc.)
    nRetVal = DeviceModule()->LoadConfigFromFile(csINIFilePath, XN_MODULE_NAME_DEVICE);
    XN_IS_STATUS_OK(nRetVal);

    // and now configure the streams
    XnDeviceModuleHolderList streams;
    nRetVal = GetStreamsList(streams);
    XN_IS_STATUS_OK(nRetVal);

    for (XnDeviceModuleHolderList::Iterator it = streams.Begin(); it != streams.End(); ++it)
    {
        XnDeviceModuleHolder* pHolder = *it;
        nRetVal = pHolder->GetModule()->LoadConfigFromFile(csINIFilePath);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::InitReading()
{
    XnStatus nRetVal = XN_STATUS_OK;

    XnSensorUsbInterface prevInterface = GetCurrentUsbInterface();

    // open data endpoints
    nRetVal = m_SensorIO.OpenDataEndPoints((XnSensorUsbInterface)m_Interface.GetValue(), *m_Firmware.GetInfo());
    XN_IS_STATUS_OK(nRetVal);

    XnSensorUsbInterface currInterface = GetCurrentUsbInterface();
    nRetVal = m_Interface.UnsafeUpdateValue(currInterface);
    XN_IS_STATUS_OK(nRetVal);

    if (prevInterface != currInterface)
    {
        nRetVal = XnHostProtocolUpdateSupportedImageModes(&m_DevicePrivateData);
        XN_IS_STATUS_OK(nRetVal);
    }

    // take frequency information
    XnFrequencyInformation FrequencyInformation;

    nRetVal = XnHostProtocolAlgorithmParams(&m_DevicePrivateData, XN_HOST_PROTOCOL_ALGORITHM_FREQUENCY, &FrequencyInformation, sizeof(XnFrequencyInformation), (XnResolutions)0, 0);
    if (nRetVal != XN_STATUS_OK)
        return nRetVal;

    m_DevicePrivateData.fDeviceFrequency = XN_PREPARE_VAR_FLOAT_IN_BUFFER(FrequencyInformation.fDeviceFrequency);

    // Init Dumps
    m_DevicePrivateData.BandwidthDump = xnDumpFileOpen(XN_DUMP_BANDWIDTH, "Bandwidth.csv");
    xnDumpFileWriteString(m_DevicePrivateData.BandwidthDump, "Timestamp,Frame Type,Frame ID,Size\n");
    m_DevicePrivateData.TimestampsDump = xnDumpFileOpen(XN_DUMP_TIMESTAMPS, "Timestamps.csv");
    xnDumpFileWriteString(m_DevicePrivateData.TimestampsDump, "Host Time (us),Stream,Device TS,Time (ms),Comments\n");
    m_DevicePrivateData.MiniPacketsDump = xnDumpFileOpen(XN_DUMP_MINI_PACKETS, "MiniPackets.csv");
    xnDumpFileWriteString(m_DevicePrivateData.MiniPacketsDump, "HostTS,Type,ID,Size,Timestamp\n");

    m_DevicePrivateData.nGlobalReferenceTS = 0;
    nRetVal = xnOSCreateCriticalSection(&m_DevicePrivateData.hEndPointsCS);
    XN_IS_STATUS_OK(nRetVal);

    // NOTE: when we go up, some streams might be open, and so we'll receive lots of garbage.
    // wait till streams are turned off, and then start reading.
    //	pDevicePrivateData->bIgnoreDataPackets = TRUE;

    // open input threads
    nRetVal = XnDeviceSensorOpenInputThreads(GetDevicePrivateData());
    XN_IS_STATUS_OK(nRetVal);

    // open 'commands.txt' thread
    nRetVal = xnOSCreateThread(XnDeviceSensorProtocolScriptThread, (XN_THREAD_PARAM)&m_DevicePrivateData, &m_DevicePrivateData.LogThread.hThread);
    XN_IS_STATUS_OK(nRetVal);

    return XN_STATUS_OK;
}

XnStatus XnSensor::ChangeTaskInterval(XnScheduledTask** ppTask, XnTaskCallbackFuncPtr pCallback, XnUInt32 nInterval)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (*ppTask == NULL)
    {
        nRetVal = xnSchedulerAddTask(m_pScheduler, nInterval, pCallback, this, ppTask);
        XN_IS_STATUS_OK(nRetVal);
    }
    else // already scheduled
    {
        if (nInterval == 0)
        {
            nRetVal = xnSchedulerRemoveTask(m_pScheduler, ppTask);
            XN_IS_STATUS_OK(nRetVal);

            *ppTask = NULL;
        }
        else
        {
            nRetVal = xnSchedulerRescheduleTask(m_pScheduler, *ppTask, nInterval);
            XN_IS_STATUS_OK(nRetVal);
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ValidateSensorID(XnChar* csSensorID)
{
    if (strcmp(csSensorID, XN_DEVICE_SENSOR_DEFAULT_ID) != 0)
    {
        if (strcmp(csSensorID, GetFixedParams()->GetSensorSerial()) != 0)
        {
            return (XN_STATUS_IO_DEVICE_WRONG_SERIAL);
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ResolveGlobalConfigFileName(XnChar* strConfigFile, XnUInt32 nBufSize, const XnChar* strConfigDir)
{
    XnStatus rc = XN_STATUS_OK;

    // If strConfigDir is NULL, tries to resolve the config file based on the driver's directory
    XnChar strBaseDir[XN_FILE_MAX_PATH];
    if (strConfigDir == NULL)
    {
#if XN_PLATFORM == XN_PLATFORM_ANDROID_ARM
        // support for applications
        xnOSGetApplicationFilesDir(strBaseDir, nBufSize);

        XnChar strTempFileName[XN_FILE_MAX_PATH];
        xnOSStrCopy(strTempFileName, strBaseDir, sizeof(strTempFileName));
        rc = xnOSAppendFilePath(strTempFileName, XN_GLOBAL_CONFIG_FILE_NAME, sizeof(strTempFileName));
        XN_IS_STATUS_OK(rc);

        XnBool bExists;
        xnOSDoesFileExist(strTempFileName, &bExists);

        if (bExists)
        {
            strConfigDir = strBaseDir;
        }
        else
        {
            // support for native use - search in current dir
            strConfigDir = ".";
        }
#else
        if (xnOSGetModulePathForProcAddress(reinterpret_cast<void*>(&XnSensor::ResolveGlobalConfigFileName), strBaseDir) == XN_STATUS_OK &&
            xnOSGetDirName(strBaseDir, strBaseDir, XN_FILE_MAX_PATH) == XN_STATUS_OK)
        {
            // Successfully obtained the driver's path
            strConfigDir = strBaseDir;
        }
        else
        {
            // Something wrong happened. Use the current directory as the fallback.
            strConfigDir = ".";
        }
#endif
    }

    XN_VALIDATE_STR_COPY(strConfigFile, strConfigDir, nBufSize, rc);
    return xnOSAppendFilePath(strConfigFile, XN_GLOBAL_CONFIG_FILE_NAME, nBufSize);
}

XnStatus XnSensor::SetGlobalConfigFile(const XnChar* strConfigFile)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = xnOSStrCopy(m_strGlobalConfigFile, strConfigFile, XN_FILE_MAX_PATH);
    XN_IS_STATUS_OK(nRetVal);

    XnBool bExists;
    nRetVal = xnOSDoesFileExist(m_strGlobalConfigFile, &bExists);
    XN_IS_STATUS_OK(nRetVal);

    if (!bExists)
    {
        xnLogVerbose(XN_MASK_DEVICE_SENSOR, "Global configuration file '%s' was not found.", m_strGlobalConfigFile);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ConfigureModuleFromGlobalFile(const XnChar* strModule, const XnChar* strSection /* = NULL */)
{
    XnStatus nRetVal = XN_STATUS_OK;

    XnDeviceModule* pModule;
    nRetVal = FindModule(strModule, &pModule);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = pModule->LoadConfigFromFile(m_strGlobalConfigFile, strSection);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetFirmwareParam(XnInnerParamData* pParam)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetParam(&m_DevicePrivateData, pParam->nParam, pParam->nValue);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ReadAHB(XnAHBData* pAHB)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolReadAHB(&m_DevicePrivateData, pAHB->nRegister, pAHB->nValue);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::WriteAHB(const XnAHBData* pAHB)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolWriteAHB(&m_DevicePrivateData, pAHB->nRegister, pAHB->nValue, pAHB->nMask);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetLedState(XnUInt16 nLedId, XnUInt16 nState)
{
    return XnHostProtocolSetLedState(&m_DevicePrivateData, nLedId, nState);
}

XnStatus XnSensor::SetEmitterState(XnBool bActive)
{
    XnStatus ret = XnHostProtocolSetEmitterState(&m_DevicePrivateData, bActive);
    XN_IS_STATUS_OK(ret);

    return m_EmitterEnabled.UnsafeUpdateValue(bActive);
}

XnStatus XnSensor::GetEmitterState(XnBool* pState)
{
    return XnHostProtocolGetEmitterState(&m_DevicePrivateData, (XnUInt16*)pState);
}

XnStatus XnSensor::GetIrfloodState(XnUInt32* bActive)
{
    return XnHostProtocolGeminiGetIrFloodSwitchState(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::SetIrfloodState(const XnUInt32& bActive)
{
    return XnHostProtocolGeminiSetIrFloodSwitchState(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetIrfloodLevel(XnUInt32* nLevel)
{
    return XnHostProtocolGeminiGetIrFloodLevelState(&m_DevicePrivateData, nLevel);
}

XnStatus XnSensor::SetIrfloodLevel(const XnUInt32& nLevel)
{
    return XnHostProtocolGeminiSetIrFloodLevelState(&m_DevicePrivateData, nLevel);
}


//irgain
XnStatus XnSensor::SetIrGain(XnUInt32 nIrGain)
{
    return XnHostProtocolSetIrGain(&m_DevicePrivateData, nIrGain);
}

//get irgain
XnStatus XnSensor::getIrGain(XnUInt32 &nIrGain)
{
    return XnHostProtocolGetIrGain(&m_DevicePrivateData, nIrGain);
}

//Ldp enable
XnStatus XnSensor::SetLdpEnable(XnBool bActive)
{
    XnStatus status = XN_STATUS_ERROR;

    if (m_nPId == ASTRAPRO_PLUS)
    {
        status = XnHostProtocolSetLdpEnable(&m_DevicePrivateData, bActive);
    }
    else
    {
        status = XnHostProtocolSetLdpEnableV1(&m_DevicePrivateData, bActive);
    }


    return status;
}

XnStatus XnSensor::getLdpEnable(XnBool &bActive)
{
    XnStatus status = XN_STATUS_ERROR;

    if (m_nPId == ASTRAPRO_PLUS)
    {
        status = XnHostProtocolGetLdpEnable(&m_DevicePrivateData, bActive);
    }
    else
    {
        status = XnHostProtocolGetLdpEnableV1(&m_DevicePrivateData, bActive);
    }

    return status;

}

XnStatus XnSensor::GetEmitterEnable(XnBool &bActive)
{
    XnStatus status = XN_STATUS_ERROR;

    if (m_nPId == ASTRAPRO_PLUS)
    {
        status = XnHostProtocolGetEmitterEnable(&m_DevicePrivateData, bActive);
    }
    else
    {
        status = XnHostProtocolGetEmitterEnableV1(&m_DevicePrivateData, bActive);
    }

    return status;
}

XnStatus XnSensor::SetLdpScale(XnUInt32 nScale)
{
    return  XnHostProtocolSetLdpScaleV1(&m_DevicePrivateData, nScale);
}

XnStatus XnSensor::GetLdpScale(XnUInt32 &nScale)
{
    return XnHostProtocolGetLdpScaleV1(&m_DevicePrivateData, nScale);
}

XnStatus XnSensor::GetLdpStatus(XnBool &bActive)
{
    return XnHostProtocolGetLdpStatusV1(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetLdpThresUp(XnUInt32 &value)
{
    return XnHostProtocolGetLdpThresUpV1(&m_DevicePrivateData, value);
}

XnStatus XnSensor::SetLdpThresUp(XnUInt32 nScale)
{
    return  XnHostProtocolSetLdpThresUpV1(&m_DevicePrivateData, nScale);
}

XnStatus XnSensor::GetLdpThresLow(XnUInt32 &value)
{
    return XnHostProtocolGetLdpThresLowV1(&m_DevicePrivateData, value);
}

XnStatus XnSensor::SetLdpThresLow(XnUInt32 value)
{
    return  XnHostProtocolSetLdpThresLowV1(&m_DevicePrivateData, value);
}

XnStatus XnSensor::GetLdpNoiseValue(XnUInt32 &value)
{
    return XnHostProtocolGetLdpNoiseValueV1(&m_DevicePrivateData, value);
}

//Auto ae
XnStatus XnSensor::SetAeStatus(XnBool bActive)
{
    return XnHostProtocolSetAeEnable(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetAeStatus(XnBool &bActive)
{
    return XnHostProtocolGetAeEnable(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::SetHdrModeStatus(XnBool bActive)
{
	return XnHostProtocolSetHdrModeEnable(&m_DevicePrivateData, bActive);
}
XnStatus XnSensor::GetHdrModeStatus(XnBool &bActive)
{
	return XnHostProtocolGetAeEnable(&m_DevicePrivateData, bActive);
}
//set cal IR Temperature
XnStatus XnSensor::SetCalIrTemperature(XnDouble nCalIrTemp)
{
    return XnHostProtocolSetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_CAL_TIR, nCalIrTemp);
}

//get cal IR Temperature
XnStatus XnSensor::getCalIrTemperature(XnDouble &nCalIrTemp)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_CAL_TIR, nCalIrTemp);
}

//set cal LDMP Temperature
XnStatus XnSensor::SetCalLdmpTemperature(XnDouble nCalLdmpTemp)
{
    return XnHostProtocolSetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_CAL_TLDMP, nCalLdmpTemp);
}

//get cal LDMP Temperature
XnStatus XnSensor::getCalLdmpTemperature(XnDouble &nCalLdmpTemp)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_CAL_TLDMP, nCalLdmpTemp);
}

//get ir real time temperature
XnStatus XnSensor::getRtIrTemperature(XnDouble &RtIrTemp)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_TIR, RtIrTemp);
}

//get ldmp real time temperature
XnStatus XnSensor::getRtLdmpTemperature(XnDouble &RtLdmpTemp)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_TLDMP, RtLdmpTemp);
}

//set ir temperature compensation coefficient
XnStatus XnSensor::SetIrTemperatureCo(XnDouble nTempCo)
{
    return XnHostProtocolSetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_NCOST_IR, nTempCo);
}

//get ir temperature compensation coefficient
XnStatus XnSensor::getIrTemperatureCo(XnDouble &nTempco)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_NCOST_IR, nTempco);
}

//set ldmp temperature compensation coefficient
XnStatus XnSensor::SetLdmpTemperatureCo(XnDouble nTempCo)
{
    return XnHostProtocolSetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_NCOST_LDMP, nTempCo);
}

//get ldmp temperature compensation coefficient
XnStatus XnSensor::getLdmpTemperatureCo(XnDouble &nTempco)
{
    return XnHostProtocolGetSupportSubCmdValue(&m_DevicePrivateData, (XnUInt32)TEMPERATURE_DEFL_NCOST_LDMP, nTempco);
}

//set temperature comp state
XnStatus XnSensor::SetTemperatureCompState(XnBool bActive)
{
    return XnHostProtocolTemperatureCompSwitch(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetTemperatureCompState(XnBool &bActive)
{
    return XnHostProtocolGetTemperatureCompStatus(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::SetDepthOptimizationState(XnBool bActive)
{
    return XnHostProtocolDepthOptimSwitch(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetDepthOptimizationState(XnBool &bActive)
{
    return XnHostProtocolGetDepthOptimStatus(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::setObDepthOptimizationParam(const XnDepthOptimizationParam* depthOptimParam)
{
    return XnHostProtocolSetDepthOptimizationParam(&m_DevicePrivateData, depthOptimParam);
}

XnStatus XnSensor::GetObDepthOptimizationParam(XnDepthOptimizationParam* depthOptimParam)
{
    return XnHostProtocolGetDepthOptimizationParam(&m_DevicePrivateData, depthOptimParam);
}

//Multi distance calibration switch
XnStatus XnSensor::SetObDistortionEnableState(XnUInt32 nActive)
{
    if (m_DevicePrivateData.ChipInfo.nChipVer == XN_SENSOR_CHIP_VER_UNKNOWN)
    {
        xnLogWarning(XN_MASK_DEVICE_SENSOR, "set Multi distance calibration unsupport");
        return XN_STATUS_ERROR;
    }
    else
    {
        return XnHostProtocolDistortionStateSwitch(&m_DevicePrivateData, nActive);
    }

}

XnStatus XnSensor::GetObDistortionEnableState(XnUInt32 &nActive)
{
    if (m_DevicePrivateData.ChipInfo.nChipVer == XN_SENSOR_CHIP_VER_UNKNOWN)
    {
        xnLogWarning(XN_MASK_DEVICE_SENSOR, "get Multi distance calibration unsupport");
        return XN_STATUS_ERROR;
    }
    else
    {
        return XnHostProtocolGetDistortionState(&m_DevicePrivateData, nActive);
    }
}

//irexp set
XnStatus XnSensor::SetIrExp(XnUInt32 nIrExp)
{
    return XnHostProtocolSetIrExp(&m_DevicePrivateData, nIrExp);
}

//get irexp
XnStatus XnSensor::getIrExp(XnUInt32 &nIrExp)
{
    return XnHostProtocolGetIrExp(&m_DevicePrivateData, nIrExp);
}

//ado change sensor
XnStatus XnSensor::SetAdoChangeSensor(XnBool bActive)
{
    return XnHostProtocolSetChangeSensor(&m_DevicePrivateData, bActive);
}

//set public key
XnStatus XnSensor::SetPublicKey(const OBEccVerify* pPublicKey)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolSetPublicKey(&m_DevicePrivateData, pPublicKey);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetPublicKey(OBEccVerify* pPublicKey)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetPublicKey(&m_DevicePrivateData, pPublicKey);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}


//set RS key
XnStatus XnSensor::SetRSKey(const OBEccRSKey* pRSKey)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolSetRSKey(&m_DevicePrivateData, pRSKey);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

//get random string
XnStatus XnSensor::GetRandomString(OBEccInit* pRandomString)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetRandomString(&m_DevicePrivateData, pRandomString);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

//laser secure
XnStatus XnSensor::IsSupportLaserSecure(XnBool &bIsSupport)
{
    return XnHostProtocolIsSupportLaserSecure(&m_DevicePrivateData, bIsSupport);
}

XnStatus XnSensor::SetLaserSecureStatus(XnBool bStatus)
{
    return XnHostProtocolSetLaserSecureStatus(&m_DevicePrivateData, bStatus);
}

XnStatus XnSensor::GetLaserSecureStatus(XnBool &bStatus)
{
    return XnHostProtocolGetLaserSecureStatus(&m_DevicePrivateData, bStatus);
}

//laser current
XnStatus XnSensor::SetLaserCurrent(XnUInt32 nLaserCurrent)
{
    return XnHostProtocolSetLaserCurrent(&m_DevicePrivateData, nLaserCurrent);
}

XnStatus XnSensor::GetLaserCurrent(XnUInt32 &nLaserCurrent)
{
    return XnHostProtocolGetLaserCurrent(&m_DevicePrivateData, nLaserCurrent);
}

XnStatus XnSensor::SetSoftReset()
{
    return XnHostProtocolSoftReset(&m_DevicePrivateData);
}

//switch left and right ir
XnStatus XnSensor::SetSwitchIr(XnBool bActive)
{
    return XnHostProtocolSetSwitchIr(&m_DevicePrivateData, bActive);
}

//rgb ae mode
XnStatus XnSensor::SetRgbAeMode(const XnRgbAeMode* pRgbAeMode)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolSetRgbAeMode(&m_DevicePrivateData, pRgbAeMode);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetRgbAeMode(XnRgbAeMode* pRgbAeMode)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetRgbAeMode(&m_DevicePrivateData, pRgbAeMode);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}


XnStatus XnSensor::SetFirmwareFrameSync(XnBool bOn)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = GetFirmware()->GetParams()->m_FrameSyncEnabled.SetValue(bOn);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_FirmwareFrameSync.UnsafeUpdateValue(bOn);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

void XnSensor::ReadFirmwareLog()
{
    // get log
    XnChar LogBuffer[XN_MAX_LOG_SIZE] = "";
    XnHostProtocolGetLog(&m_DevicePrivateData, LogBuffer, XN_MAX_LOG_SIZE);

    // write sensor log to dump
    xnDumpFileWriteString(m_FirmwareLogDump, LogBuffer);

    // print
    if (m_FirmwareLogPrint.GetValue())
    {
        printf("%s", LogBuffer);
    }
}

void XnSensor::ReadFirmwareCPU()
{
    XnTaskCPUInfo aTasks[100];
    XnUInt32 nTasksCount = 100;
    XnStatus nRetVal = XnHostProtocolGetCPUStats(&m_DevicePrivateData, aTasks, &nTasksCount);
    if (nRetVal != XN_STATUS_OK)
    {
        xnLogWarning(XN_MASK_SENSOR_PROTOCOL, "GetCPUStats failed execution: %s", xnGetStatusString(nRetVal));
        return;
    }

    // sum it all up
    XnUInt64 nSum = 0;
    for (XnUInt32 nIndex = 0; nIndex < nTasksCount; ++nIndex)
        nSum += aTasks[nIndex].nTimeInMicroSeconds;

    // print
    printf("Task ID  Total Time (us)  Percentage  Times    Avg. Time Per Call\n");
    printf("=======  ===============  ==========  =======  ==================\n");
    for (XnUInt32 nIndex = 0; nIndex < nTasksCount; ++nIndex)
    {
        printf("%7u  %15u  %10.3f  %7u	%18.3f\n",
            nIndex, aTasks[nIndex].nTimeInMicroSeconds,
            aTasks[nIndex].nTimeInMicroSeconds * 100.0 / nSum,
            aTasks[nIndex].nTimesExecuted,
            (double)aTasks[nIndex].nTimeInMicroSeconds / (double)aTasks[nIndex].nTimesExecuted);
    }
}

XnStatus XnSensor::GetI2C(XnI2CReadData* pI2CReadData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolReadI2C(&m_DevicePrivateData, pI2CReadData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetTecStatus(XnTecData* pTecData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetTecData(&m_DevicePrivateData, pTecData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetTecFastConvergenceStatus(XnTecFastConvergenceData* pTecData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetTecFastConvergenceData(&m_DevicePrivateData, pTecData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetEmitterStatus(XnEmitterData* pEmitterData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetEmitterData(&m_DevicePrivateData, pEmitterData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ReadFlashFile(const XnParamFileData* pFile)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolFileDownload(&m_DevicePrivateData, (XnUInt16)pFile->nOffset, pFile->strFileName);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ReadFlashChunk(XnParamFlashData* pFlash)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolReadFlash(&m_DevicePrivateData, pFlash->nOffset, pFlash->nSize, pFlash->pData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}


XnStatus XnSensor::WriteFlashDistortionParam(const XnDistortionParam* pValue)
{
    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = XnHostProtocolWriteDistortionParam(&m_DevicePrivateData, pValue->nSize, pValue->data);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::ReadFlashDistortionParam(XnDistortionParam* pValue)
{
    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = XnHostProtocolReadDistortionParam(&m_DevicePrivateData, pValue->nSize, pValue->data);
    XN_IS_STATUS_OK(nRetVal);

    m_bReadDistortParam = TRUE;
    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetCmosBlankingUnits(XnCmosBlankingUnits* pBlanking)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer < XN_SENSOR_FW_VER_5_1)
    {
        return (XN_STATUS_IO_DEVICE_FUNCTION_NOT_SUPPORTED);
    }

    nRetVal = XnHostProtocolGetCmosBlanking(&m_DevicePrivateData, pBlanking->nCmosID, &pBlanking->nUnits);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetCmosBlankingTime(XnCmosBlankingTime* pBlanking)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // check version
    if (m_Firmware.GetInfo()->nFWVer < XN_SENSOR_FW_VER_5_1)
    {
        return (XN_STATUS_IO_DEVICE_FUNCTION_NOT_SUPPORTED);
    }

    // get value in units
    XnCmosBlankingUnits blankingUnits;
    blankingUnits.nCmosID = pBlanking->nCmosID;
    nRetVal = GetCmosBlankingUnits(&blankingUnits);
    XN_IS_STATUS_OK(nRetVal);

    // get coefficients
    const XnCmosBlankingCoefficients* pCoeffs = m_CmosInfo.GetBlankingCoefficients(pBlanking->nCmosID);

    // translate to time
    pBlanking->nTimeInMilliseconds = (pCoeffs->fA * blankingUnits.nUnits + pCoeffs->fB) / 1000;

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetFirmwareMode(XnParamCurrentMode* pnMode)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer == XN_SENSOR_FW_VER_0_17)
    {
        *pnMode = m_Firmware.GetInfo()->nCurrMode;
    }
    else
    {
        XnUInt16 nMode;
        nRetVal = XnHostProtocolGetMode(&m_DevicePrivateData, nMode);
        XN_IS_STATUS_OK(nRetVal);

        switch (nMode)
        {
        case XN_HOST_PROTOCOL_MODE_PS:
            *pnMode = XN_MODE_PS;
            break;
        case XN_HOST_PROTOCOL_MODE_MAINTENANCE:
            *pnMode = XN_MODE_MAINTENANCE;
            break;
        case XN_HOST_PROTOCOL_MODE_SAFE_MODE:
            *pnMode = XN_MODE_SAFE_MODE;
            break;
        default:
            printf("Got Unknown Firmware Mode %d\n", nMode);
            return XN_STATUS_DEVICE_BAD_PARAM;
        }
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetDepthCmosRegister(XnControlProcessingData* pRegister)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        nRetVal = XnHostProtocolGetCMOSRegisterI2C(&m_DevicePrivateData, XN_CMOS_TYPE_DEPTH, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }
    else
    {
        nRetVal = XnHostProtocolGetCMOSRegister(&m_DevicePrivateData, XN_CMOS_TYPE_DEPTH, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetImageCmosRegister(XnControlProcessingData* pRegister)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        nRetVal = XnHostProtocolGetCMOSRegisterI2C(&m_DevicePrivateData, XN_CMOS_TYPE_IMAGE, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }
    else
    {
        nRetVal = XnHostProtocolGetCMOSRegister(&m_DevicePrivateData, XN_CMOS_TYPE_IMAGE, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetFirmwareLog(XnChar* csLog, XnUInt32 nSize)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetLog(&m_DevicePrivateData, csLog, nSize);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetFileList(XnFlashFileList* pFileList)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolGetFileList(&m_DevicePrivateData, 0, pFileList->pFiles, pFileList->nFiles);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::GetFixedParams(XnDynamicSizeBuffer* pBuffer)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (pBuffer->nMaxSize < sizeof(XnFixedParams))
    {
        return (XN_STATUS_OUTPUT_BUFFER_OVERFLOW);
    }

    XnFixedParams fixed;
    nRetVal = XnHostProtocolGetFixedParams(GetDevicePrivateData(), fixed);
    XN_IS_STATUS_OK(nRetVal);

    xnOSMemCopy(pBuffer->pData, &fixed, sizeof(XnFixedParams));
    pBuffer->nDataSize = sizeof(XnFixedParams);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::RunBIST(XnUInt32 nTestsMask, XnUInt32* pnFailures)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // always perform soft reset before running bist
    nRetVal = XnHostProtocolReset(&m_DevicePrivateData, XN_RESET_TYPE_SOFT);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnHostProtocolRunBIST(&m_DevicePrivateData, nTestsMask, pnFailures);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetReadAllEndpoints(XnBool bEnabled)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_ReadAllEndpoints.GetValue() == (XnUInt64)bEnabled)
    {
        return XN_STATUS_OK;
    }

    if (bEnabled)
    {
        xnLogInfo(XN_MASK_DEVICE_SENSOR, "Creating USB depth read thread...");
        XnSpecificUsbDevice* pUSB = m_DevicePrivateData.pSpecificDepthUsb;
        nRetVal = xnUSBInitReadThread(pUSB->pUsbConnection->UsbEp, pUSB->nChunkReadBytes, pUSB->nNumberOfBuffers, pUSB->nTimeout, XnDeviceSensorProtocolUsbEpCb, pUSB);
        XN_IS_STATUS_OK(nRetVal);

        xnLogInfo(XN_MASK_DEVICE_SENSOR, "Creating USB image read thread...");
        pUSB = m_DevicePrivateData.pSpecificImageUsb;
        nRetVal = xnUSBInitReadThread(pUSB->pUsbConnection->UsbEp, pUSB->nChunkReadBytes, pUSB->nNumberOfBuffers, pUSB->nTimeout, XnDeviceSensorProtocolUsbEpCb, pUSB);
        XN_IS_STATUS_OK(nRetVal);

        if (IsAISupported())
        {
            xnLogInfo(XN_MASK_DEVICE_SENSOR, "Creating USB AI read thread...");
            pUSB = m_DevicePrivateData.pSpecificAIUsb;
            nRetVal = xnUSBInitReadThread(pUSB->pUsbConnection->UsbEp, pUSB->nChunkReadBytes, pUSB->nNumberOfBuffers, pUSB->nTimeout, XnDeviceSensorProtocolUsbEpCb, pUSB);
            XN_IS_STATUS_OK(nRetVal);
        }
    }
    else
    {
        xnLogInfo(XN_MASK_DEVICE_SENSOR, "Shutting down USB depth read thread...");
        xnUSBShutdownReadThread(m_DevicePrivateData.pSpecificDepthUsb->pUsbConnection->UsbEp);

        xnLogInfo(XN_MASK_DEVICE_SENSOR, "Shutting down USB image read thread...");
        xnUSBShutdownReadThread(m_DevicePrivateData.pSpecificImageUsb->pUsbConnection->UsbEp);

        if (IsAISupported())
        {
            xnLogInfo(XN_MASK_DEVICE_SENSOR, "Shutting down USB AI read thread...");
            xnUSBShutdownReadThread(m_DevicePrivateData.pSpecificAIUsb->pUsbConnection->UsbEp);
        }
    }

    nRetVal = m_ReadAllEndpoints.UnsafeUpdateValue(bEnabled);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetErrorState(XnStatus errorState)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (errorState != GetErrorState())
    {
        if (errorState == XN_STATUS_OK)
        {
            xnLogInfo(XN_MASK_DEVICE_SENSOR, "Device is back to normal state.");
        }
        else
        {
            xnLogError(XN_MASK_DEVICE_SENSOR, "Device has entered error mode: %s", xnGetStatusString(errorState));
        }

        nRetVal = m_ErrorState.UnsafeUpdateValue((XnUInt64)errorState);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetInterface(XnSensorUsbInterface nInterface)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // we don't allow change if requested value is specific and different than current
    if (m_ReadData.GetValue() == TRUE &&
        nInterface != XN_SENSOR_USB_INTERFACE_DEFAULT &&
        nInterface != GetCurrentUsbInterface())
    {
        return (XN_STATUS_DEVICE_PROPERTY_READ_ONLY);
    }

    nRetVal = m_Interface.UnsafeUpdateValue(nInterface);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetHostTimestamps(XnBool bHostTimestamps)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // we don't allow change if requested value is specific and different than current
    if (m_ReadData.GetValue() == TRUE &&
        bHostTimestamps != (XnBool)m_HostTimestamps.GetValue())
    {
        return (XN_STATUS_DEVICE_PROPERTY_READ_ONLY);
    }

    nRetVal = m_HostTimestamps.UnsafeUpdateValue(bHostTimestamps);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetReadData(XnBool bRead)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (!bRead)
    {
        return XN_STATUS_ERROR;
    }
    else
    {
        nRetVal = InitReading();
        XN_IS_STATUS_OK(nRetVal);

        nRetVal = m_ReadData.UnsafeUpdateValue(TRUE);
        XN_IS_STATUS_OK(nRetVal);

        // no longer needed
        m_ReadData.UpdateSetCallback(NULL, NULL);

        XnHostProtocolUpdateSupportedImageModes(&m_DevicePrivateData);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetDepthCmosRegister(const XnControlProcessingData* pRegister)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        nRetVal = XnHostProtocolSetCMOSRegisterI2C(&m_DevicePrivateData, XN_CMOS_TYPE_DEPTH, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }
    else
    {
        nRetVal = XnHostProtocolSetCMOSRegister(&m_DevicePrivateData, XN_CMOS_TYPE_DEPTH, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetImageCmosRegister(const XnControlProcessingData* pRegister)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer >= XN_SENSOR_FW_VER_3_0)
    {
        nRetVal = XnHostProtocolSetCMOSRegisterI2C(&m_DevicePrivateData, XN_CMOS_TYPE_IMAGE, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }
    else
    {
        nRetVal = XnHostProtocolSetCMOSRegister(&m_DevicePrivateData, XN_CMOS_TYPE_IMAGE, pRegister->nRegister, pRegister->nValue);
        XN_IS_STATUS_OK(nRetVal);
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareLogFilter(XnUInt32 nFilter)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // set the firmware param (this prop will be updated accordingly)
    nRetVal = m_Firmware.GetParams()->m_LogFilter.SetValue(nFilter);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareLogInterval(XnUInt32 nMilliSeconds)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = ChangeTaskInterval(&m_pLogTask, ExecuteFirmwareLogTask, nMilliSeconds);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_FirmwareLogInterval.UnsafeUpdateValue(nMilliSeconds);
    XN_IS_STATUS_OK(nRetVal);

    if (nMilliSeconds == 0)
    {
        xnDumpFileClose(m_FirmwareLogDump);
    }
    else
    {
        m_FirmwareLogDump = xnDumpFileOpenEx("FirmwareLog", TRUE, TRUE, "Sensor.log");
    }

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareLogPrint(XnBool bPrint)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = m_FirmwareLogPrint.UnsafeUpdateValue(bPrint);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareCPUInterval(XnUInt32 nMilliSeconds)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = ChangeTaskInterval(&m_pCPUTask, ExecuteFirmwareCPUTask, nMilliSeconds);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_FirmwareCPUInterval.UnsafeUpdateValue(nMilliSeconds);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetAPCEnabled(XnBool bEnabled)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // set firmware param (we will be updated via synch mechanism)
    nRetVal = m_Firmware.GetParams()->m_APCEnabled.SetValue(bEnabled);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetI2C(const XnI2CWriteData* pI2CWriteData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolWriteI2C(&m_DevicePrivateData, pI2CWriteData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::DeleteFile(XnUInt16 nFileID)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolDeleteFile(&m_DevicePrivateData, nFileID);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetTecSetPoint(XnUInt16 nSetPoint)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolCalibrateTec(&m_DevicePrivateData, nSetPoint);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetEmitterSetPoint(XnUInt16 nSetPoint)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolCalibrateEmitter(&m_DevicePrivateData, nSetPoint);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFileAttributes(const XnFileAttributes* pAttributes)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolSetFileAttributes(&m_DevicePrivateData, pAttributes->nId, pAttributes->nAttribs);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareParam(const XnInnerParamData* pParam)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolSetParam(&m_DevicePrivateData, pParam->nParam, pParam->nValue);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::WriteFlashFile(const XnParamFileData* pFile)
{
    XnStatus nRetVal = XN_STATUS_OK;

    xnLogInfo(XN_MASK_SENSOR_PROTOCOL, "Upload file %s (offset %d)", pFile->strFileName, pFile->nOffset);

    nRetVal = XnHostProtocolFileUpload(&m_DevicePrivateData, pFile->nOffset, pFile->strFileName, pFile->nAttributes);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetCmosBlankingUnits(const XnCmosBlankingUnits* pBlanking)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer < XN_SENSOR_FW_VER_5_1)
    {
        return (XN_STATUS_IO_DEVICE_FUNCTION_NOT_SUPPORTED);
    }

    nRetVal = XnHostProtocolSetCmosBlanking(&m_DevicePrivateData, pBlanking->nUnits, pBlanking->nCmosID, pBlanking->nNumberOfFrames);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetCmosBlankingTime(const XnCmosBlankingTime* pBlanking)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // check version
    if (m_Firmware.GetInfo()->nFWVer < XN_SENSOR_FW_VER_5_1)
    {
        return (XN_STATUS_IO_DEVICE_FUNCTION_NOT_SUPPORTED);
    }

    // get coefficients
    const XnCmosBlankingCoefficients* pCoeffs = m_CmosInfo.GetBlankingCoefficients(pBlanking->nCmosID);

    // translate to units request
    XnCmosBlankingUnits blankingUnits;
    blankingUnits.nCmosID = pBlanking->nCmosID;
    blankingUnits.nNumberOfFrames = pBlanking->nNumberOfFrames;
    blankingUnits.nUnits = XnUInt16((pBlanking->nTimeInMilliseconds * 1000 - pCoeffs->fB) / pCoeffs->fA);

    nRetVal = SetCmosBlankingUnits(&blankingUnits);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::Reset(XnParamResetType nType)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolReset(&m_DevicePrivateData, (XnUInt16)nType);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetFirmwareMode(XnParamCurrentMode nMode)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_Firmware.GetInfo()->nFWVer == XN_SENSOR_FW_VER_0_17)
    {
        m_Firmware.GetInfo()->nCurrMode = nMode;
        return (XN_STATUS_OK);
    }

    XnHostProtocolModeType nActualValue;

    switch (nMode)
    {
    case XN_MODE_PS:
        nActualValue = XN_HOST_PROTOCOL_MODE_PS;
        break;
    case XN_MODE_MAINTENANCE:
        nActualValue = XN_HOST_PROTOCOL_MODE_MAINTENANCE;
        break;
    default:
        return XN_STATUS_DEVICE_UNSUPPORTED_MODE;
    }

    nRetVal = XnHostProtocolSetMode(&m_DevicePrivateData, (XnUInt16)nActualValue);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnSensor::SetProjectorFault(XnProjectorFaultData* pProjectorFaultData)
{
    XnStatus nRetVal = XN_STATUS_OK;

    nRetVal = XnHostProtocolCalibrateProjectorFault(&m_DevicePrivateData, pProjectorFaultData->nMinThreshold, pProjectorFaultData->nMaxThreshold, &pProjectorFaultData->bProjectorFaultEvent);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

void XnSensor::ExecuteFirmwareLogTask(void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    pThis->ReadFirmwareLog();
}

void XnSensor::ExecuteFirmwareCPUTask(void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    pThis->ReadFirmwareCPU();
}

XnStatus XnSensor::OnFrameSyncPropertyChanged()
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (m_ReadData.GetValue() == TRUE)
    {
        // decide firmware frame sync - both streams are on, and user asked for it
        XnBool bFrameSync = (
            m_FrameSync.GetValue() == TRUE &&
            GetFirmware()->GetParams()->m_Stream0Mode.GetValue() == XN_VIDEO_STREAM_COLOR &&
            GetFirmware()->GetParams()->m_Stream1Mode.GetValue() == XN_VIDEO_STREAM_DEPTH
            );

        nRetVal = SetFirmwareFrameSync(bFrameSync);
        XN_IS_STATUS_OK(nRetVal);

        // Set frame sync enabled flag (so mechanism will not be activated in case a stream is turned off).
        m_frameSyncCs.Lock();
        m_nFrameSyncEnabled = bFrameSync;
        m_frameSyncCs.Unlock();
    }

    return (XN_STATUS_OK);
}

void XnSensor::OnNewStreamData(XnDeviceStream* pStream, OniFrame* pFrame)
{
    // Lock critical section.
    m_frameSyncCs.Lock();

    // Find the relevant stream in the frame-synced streams.
    FrameSyncedStream* pFrameSyncedStream = NULL;
    XnUInt32 nValidFrameCount = 0; // received frame
    XnUInt32 nFrameSyncStreamCount = m_FrameSyncedStreams.GetSize();
    int receivedFrameId = pFrame->frameIndex;
    for (XnUInt32 i = 0; m_nFrameSyncEnabled && (i < nFrameSyncStreamCount); ++i)
    {
        if (pStream == m_FrameSyncedStreams[i].pStream)
        {
            // Verify frame is valid.
            if (pFrame != NULL)
            {
                // Release old frame and assign new frame.
                if (m_FrameSyncedStreams[i].pFrame != NULL)
                {
                    m_FrameSyncedStreams[i].pStream->ReleaseFrame(m_FrameSyncedStreams[i].pFrame);
                }

                // Store the frame, timestamp and frame ID.
                m_FrameSyncedStreams[i].pFrame = pFrame;
                pStream->AddRefToFrame(pFrame);
                pFrameSyncedStream = &m_FrameSyncedStreams[i];
                nValidFrameCount++;
            }
        }
        else if (m_FrameSyncedStreams[i].pFrame != NULL)
        {
            // Check if there is a stored frame which has older timestamp than allowed.
            XnUInt64 diff = (pFrame->timestamp > m_FrameSyncedStreams[i].pFrame->timestamp) ?
                pFrame->timestamp - m_FrameSyncedStreams[i].pFrame->timestamp :
                m_FrameSyncedStreams[i].pFrame->timestamp - pFrame->timestamp;
            if (diff > FRAME_SYNC_MAX_FRAME_TIME_DIFF)
            {
                // Check if received frame has newer timestamp than stored one.
                if (pFrame->timestamp > m_FrameSyncedStreams[i].pFrame->timestamp)
                {
                    // Release the last frame.
                    m_FrameSyncedStreams[i].pStream->ReleaseFrame(m_FrameSyncedStreams[i].pFrame);
                    m_FrameSyncedStreams[i].pFrame = NULL;
                }
                // Newest frame should be released.
                else
                {
                    // Check whether frame was updated in the relevant frame synced stream.
                    if (pFrameSyncedStream != NULL)
                    {
                        // Release the stored new frame.
                        pFrameSyncedStream->pStream->ReleaseFrame(pFrameSyncedStream->pFrame);
                        pFrameSyncedStream->pFrame = NULL;
                        nValidFrameCount--;
                    }
                    else
                    {
                        // Release the new frame.
                        pFrame = NULL;
                    }
                }
                break;
            }
            else
            {
                ++nValidFrameCount;
            }
        }
    }

    // Check whether stream is frame synced.
    if (m_nFrameSyncEnabled && (pFrameSyncedStream != NULL))
    {
        // Check if all the frames arrived.
        if (nValidFrameCount == nFrameSyncStreamCount)
        {
            // Send all the frames.
            ++m_nFrameSyncLastFrameID;
            for (XnUInt32 i = 0; i < nFrameSyncStreamCount; ++i)
            {
                // Send the frame.
                m_FrameSyncedStreams[i].pFrame->frameIndex = m_nFrameSyncLastFrameID;
                XnDeviceBase::OnNewStreamData(m_FrameSyncedStreams[i].pStream, m_FrameSyncedStreams[i].pFrame);
                m_FrameSyncedStreams[i].pStream->ReleaseFrame(m_FrameSyncedStreams[i].pFrame);
                m_FrameSyncedStreams[i].pFrame = NULL;
            }
        }

        // Unlock critical section.
        m_frameSyncCs.Unlock();
    }
    else
    {
        // Unlock critical section.
        m_frameSyncCs.Unlock();

        // Set frame ID to higher of self and frame ID, to make sure IDs are incremental.
        m_nFrameSyncLastFrameID = (m_nFrameSyncLastFrameID < receivedFrameId) ? receivedFrameId : m_nFrameSyncLastFrameID;

        if (pFrame != NULL)
        {
            // Send the frame (it is not frame-synced).
            XnDeviceBase::OnNewStreamData(pStream, pFrame);
        }
    }
}

XnStatus XnSensor::SetFrameSyncStreamGroup(XnDeviceStream** ppStreamList, XnUInt32 numStreams)
{
    // Lock critical section.
    m_frameSyncCs.Lock();

    // Set the frame sync property in the device.
    XnStatus rc = SetProperty(XN_MODULE_NAME_DEVICE, XN_MODULE_PROPERTY_FRAME_SYNC,
        (XnUInt64)((numStreams > 0) ? TRUE : FALSE));
    if (rc != XN_STATUS_OK)
    {
        // Unlock critical section.
        m_frameSyncCs.Unlock();
        return rc;
    }

    // Clear all the streams from frame-sync list.
    XnUInt32 nFrameSyncStreamCount = m_FrameSyncedStreams.GetSize();
    for (XnUInt32 i = 0; i < nFrameSyncStreamCount; ++i)
    {
        // Release stored frame.
        if (m_FrameSyncedStreams[i].pFrame != NULL)
        {
            // Release the frame.
            m_FrameSyncedStreams[i].pStream->ReleaseFrame(m_FrameSyncedStreams[i].pFrame);
            m_FrameSyncedStreams[i].pFrame = NULL;
        }

        // Clear pointer to stream, timestamp and frame ID.
        m_FrameSyncedStreams[i].pStream = NULL;
    }

    // Check if creating group.
    if (numStreams > 0)
    {
        m_FrameSyncedStreams.SetSize(numStreams);

        // Add streams to frame-sync list.
        for (XnUInt32 i = 0; i < numStreams; ++i)
        {
            m_FrameSyncedStreams[i].pStream = ppStreamList[i];
            m_FrameSyncedStreams[i].pFrame = NULL;
        }
    }
    else
    {
        // Zero the size of the frame sync group.
        m_FrameSyncedStreams.SetSize(0);
    }

    // Unlock critical section.
    m_frameSyncCs.Unlock();

    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetInterfaceCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->XnSensor::SetInterface((XnSensorUsbInterface)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetHostTimestampsCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->XnSensor::SetHostTimestamps(nValue == 1);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetReadDataCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->XnSensor::SetReadData((XnBool)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareParamCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnInnerParamData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareParam((const XnInnerParamData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetCmosBlankingUnitsCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnCmosBlankingUnits);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetCmosBlankingUnits((const XnCmosBlankingUnits*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetCmosBlankingTimeCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnCmosBlankingTime);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetCmosBlankingTime((const XnCmosBlankingTime*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::ResetCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->Reset((XnParamResetType)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareModeCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareMode((XnParamCurrentMode)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFirmwareParamCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnInnerParamData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetFirmwareParam((XnInnerParamData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetCmosBlankingUnitsCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnCmosBlankingUnits);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetCmosBlankingUnits((XnCmosBlankingUnits*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetCmosBlankingTimeCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnCmosBlankingTime);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetCmosBlankingTime((XnCmosBlankingTime*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFirmwareModeCallback(const XnIntProperty* /*pSender*/, XnUInt64* pnValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnParamCurrentMode nMode;
    XnStatus nRetVal = pThis->GetFirmwareMode(&nMode);
    XN_IS_STATUS_OK(nRetVal);

    *pnValue = nMode;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetAudioSupportedCallback(const XnIntProperty* /*pSender*/, XnUInt64* pnValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    *pnValue = pThis->m_Firmware.GetInfo()->bAudioSupported;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetImageSupportedCallback(const XnIntProperty* /*pSender*/, XnUInt64* pnValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    *pnValue = pThis->m_Firmware.GetInfo()->bImageSupported;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPhaseSupportedCallback(const XnIntProperty* /*pSender*/, XnUInt64* pnValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    *pnValue = pThis->m_Firmware.GetInfo()->bPhaseSupported;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetBodySupportedCallback(const XnIntProperty* /*pSender*/, XnUInt64* pnValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    *pnValue = pThis->m_Firmware.GetInfo()->bAISupported;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::FrameSyncPropertyChangedCallback(const XnProperty* /*pSender*/, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->OnFrameSyncPropertyChanged();
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFixedParamsCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnDynamicSizeBuffer);
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDynamicSizeBuffer* pBuffer = (XnDynamicSizeBuffer*)gbValue.data;
    return pThis->GetFixedParams(pBuffer);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetInstanceCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    if (gbValue.dataSize != sizeof(void*))
    {
        return XN_STATUS_DEVICE_PROPERTY_SIZE_DONT_MATCH;
    }

    *(void**)gbValue.data = pCookie;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetDepthCmosRegisterCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnControlProcessingData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetDepthCmosRegister((const XnControlProcessingData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetImageCmosRegisterCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnControlProcessingData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetImageCmosRegister((const XnControlProcessingData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDepthCmosRegisterCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnControlProcessingData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetDepthCmosRegister((XnControlProcessingData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetImageCmosRegisterCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnControlProcessingData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetImageCmosRegister((XnControlProcessingData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::WriteAHBCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnAHBData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->WriteAHB((const XnAHBData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::ReadAHBCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnAHBData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->ReadAHB((XnAHBData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLedStateCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnLedState);
    XnSensor* pThis = (XnSensor*)pCookie;
    const XnLedState* pLedState = (const XnLedState*)gbValue.data;
    return pThis->SetLedState(pLedState->nLedID, pLedState->nState);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetEmitterStateCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetEmitterState(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetEmitterStateCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
    XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetEmitterState((XnBool*)pValue);
}

//ir_flood
XnStatus XN_CALLBACK_TYPE XnSensor::SetIrfloodStateCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetIrfloodState((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetIrfloodStateCallback(const XnIntProperty* /*pSender*/, XnUInt64* nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nIrfloodLevel = 0;
    XnStatus rc = pThis->GetIrfloodState(&nIrfloodLevel);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIrfloodLevel;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetIrfloodLevelCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetIrfloodLevel((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetIrfloodLevelCallback(const XnIntProperty* /*pSender*/, XnUInt64* nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nIrfloodState = 0;
    XnStatus rc = pThis->GetIrfloodLevel(&nIrfloodState);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIrfloodState;
    }
    return rc;
}
//end ir_flood

//irgain set

XnStatus XN_CALLBACK_TYPE XnSensor::SetIrGainCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetIrGain((XnUInt32)nValue);
}

//get irgain
XnStatus XN_CALLBACK_TYPE XnSensor::GetIrGainCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nIrgain = 0;
    XnStatus rc = pThis->getIrGain(nIrgain);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIrgain;
    }

    return rc;
}

//set cal ir temperature
XnStatus XN_CALLBACK_TYPE XnSensor::SetCalIrTempertureCallback(XnRealProperty* /*pSender*/, XnDouble nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetCalIrTemperature(nValue);
}

//get cal ir temperature
XnStatus XN_CALLBACK_TYPE XnSensor::GetCalIrTempertureCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nCalIrTemp = 0;
    XnStatus rc = pThis->getCalIrTemperature(nCalIrTemp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nCalIrTemp;
    }

    return rc;
}

//set cal ldmp temperature
XnStatus XN_CALLBACK_TYPE XnSensor::SetCalLdmpTempertureCallback(XnRealProperty* /*pSender*/, XnDouble nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetCalLdmpTemperature(nValue);
}

//get cal ldmp temperature
XnStatus XN_CALLBACK_TYPE XnSensor::GetCalLdmpTempertureCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nCalLdmpTemp = 0;
    XnStatus rc = pThis->getCalLdmpTemperature(nCalLdmpTemp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nCalLdmpTemp;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetRtIrTempertureCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nRtIrTemp = 0;
    XnStatus rc = pThis->getRtIrTemperature(nRtIrTemp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nRtIrTemp;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetRtLdmpTempertureCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nRtLdmpTemp = 0;
    XnStatus rc = pThis->getRtLdmpTemperature(nRtLdmpTemp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nRtLdmpTemp;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetIrTempertureCoCallback(XnRealProperty* /*pSender*/, XnDouble nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetIrTemperatureCo(nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetIrTempertureCoCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nTempCo = 0;
    XnStatus rc = pThis->getIrTemperatureCo(nTempCo);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nTempCo;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLdmpTempertureCoCallback(XnRealProperty* /*pSender*/, XnDouble nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLdmpTemperatureCo(nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLdmpTempertureCoCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDouble nTempCo = 0;
    XnStatus rc = pThis->getLdmpTemperatureCo(nTempCo);
    if (XN_STATUS_OK == rc)
    {
        *nValue = nTempCo;
    }

    return rc;
}

//Set Temperature CompState
XnStatus XN_CALLBACK_TYPE XnSensor::SetTemperatureCompStateCallback(XnIntProperty*  /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTemperatureCompState(nValue == TRUE);
}

//Get Temperature CompState
XnStatus XN_CALLBACK_TYPE XnSensor::GetTemperatureCompStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool enableTempComp = 0;
    XnStatus rc = pThis->GetTemperatureCompState(enableTempComp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)enableTempComp;
    }
    return rc;
}

//Set depth optimization enable
XnStatus XN_CALLBACK_TYPE XnSensor::SetDepthOptimizationStateCallback(XnIntProperty*  /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetDepthOptimizationState(nValue == TRUE);
}

//Get depth optimization enable
XnStatus XN_CALLBACK_TYPE XnSensor::GetDepthOptimizationStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool enableTempComp = 0;
    XnStatus rc = pThis->GetDepthOptimizationState(enableTempComp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)enableTempComp;
    }
    return rc;
}

//Set depth optimization param
XnStatus XN_CALLBACK_TYPE XnSensor::setObDepthOptimizationParamCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnDepthOptimizationParam);
    XnSensor* pThis = (XnSensor*)pCookie;
    const XnDepthOptimizationParam *depthOptim = (const XnDepthOptimizationParam *)gbValue.data;
    return pThis->setObDepthOptimizationParam(depthOptim);
}

//Get depth optimization param
XnStatus XN_CALLBACK_TYPE XnSensor::GetObDepthOptimizationParamCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnDepthOptimizationParam);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetObDepthOptimizationParam((XnDepthOptimizationParam *)gbValue.data);
}

//irExp set
XnStatus XN_CALLBACK_TYPE XnSensor::SetIrExpCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetIrExp((XnUInt32)nValue);
}

//get irExp
XnStatus XN_CALLBACK_TYPE XnSensor::GetIrExpCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nIrExp = 0;
    XnStatus rc = pThis->getIrExp(nIrExp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIrExp;
    }

    return rc;
}

//
XnStatus XN_CALLBACK_TYPE XnSensor::SetLdpEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLdpEnable((XnBool)nValue);
}


XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpEnableCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool nLdpEnable = 0;
    XnStatus rc = pThis->getLdpEnable(nLdpEnable);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLdpEnable;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetEmitterEnableCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool nEmitterEnable = 0;
    XnStatus rc = pThis->GetEmitterEnable(nEmitterEnable);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nEmitterEnable;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLdpScaleCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLdpScale((XnBool)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpScaleCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nScale = 0;
    XnStatus rc = pThis->GetLdpScale(nScale);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nScale;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpStatusCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool nLdpStatus = 0;
    XnStatus rc = pThis->GetLdpStatus(nLdpStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLdpStatus;
    }

    return rc;
}


XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpThresUpCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nLdpThresUp = 0;
    XnStatus rc = pThis->GetLdpThresUp(nLdpThresUp);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLdpThresUp;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLdpThresUpCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLdpThresUp((XnUInt32)nValue);
}


XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpThresLowCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nLdpThresLow = 0;
    XnStatus rc = pThis->GetLdpThresLow(nLdpThresLow);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLdpThresLow;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLdpThresLowCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLdpThresLow((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLdpNoiseValueCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nLdpNoise = 0;
    XnStatus rc = pThis->GetLdpNoiseValue(nLdpNoise);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLdpNoise;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetAeEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetAeStatus(nValue == TRUE);
}


XnStatus XN_CALLBACK_TYPE XnSensor::GetAeStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool autoAeStatus = 0;
    XnStatus rc = pThis->GetAeStatus(autoAeStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt16)autoAeStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetHdrModeEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
	XnSensor* pThis = (XnSensor*)pCookie;
	xnLogInfo(XN_MASK_DEVICE_SENSOR, "set hdr mode status:%d", nValue);
	return pThis->SetHdrModeStatus(nValue == TRUE);
}
XnStatus XN_CALLBACK_TYPE XnSensor::GetHdrModeEnableCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
	XnSensor* pThis = (XnSensor*)pCookie;
	XnBool autoHdrStatus = 0;
	XnStatus rc = pThis->GetHdrModeStatus(autoHdrStatus);
	if (XN_STATUS_OK == rc)
	{
		*nValue = (XnUInt16)autoHdrStatus;
	}
	return rc;
}

//ado change sensor set callback
XnStatus XN_CALLBACK_TYPE XnSensor::SetAdoChangeSensorCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetAdoChangeSensor(nValue == TRUE);
}


XnStatus XN_CALLBACK_TYPE XnSensor::SetPublicKeyCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBEccVerify);
    XnSensor* pThis = (XnSensor*)pCookie;
    const OBEccVerify *pObEccVerify = (const OBEccVerify *)gbValue.data;
    return pThis->SetPublicKey(pObEccVerify);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPublicKeyCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBEccVerify);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetPublicKey((OBEccVerify*)gbValue.data);
}

//
XnStatus XN_CALLBACK_TYPE XnSensor::SetRSKeyCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBEccRSKey);
    XnSensor* pThis = (XnSensor*)pCookie;
    const OBEccRSKey *pObEccRSKey = (const OBEccRSKey *)gbValue.data;
    return pThis->SetRSKey(pObEccRSKey);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetRandomStringCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBEccInit);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetRandomString((OBEccInit*)gbValue.data);
}

//laser secure
XnStatus XN_CALLBACK_TYPE XnSensor::IsSupportLaserSecureCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool nIsSupport = 0;
    XnStatus rc = pThis->IsSupportLaserSecure(nIsSupport);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIsSupport;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLaserSecureStatusCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLaserSecureStatus(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLaserSecureStatusCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool bLaserSecure = 0;
    XnStatus rc = pThis->GetLaserSecureStatus(bLaserSecure);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)bLaserSecure;
    }

    return rc;
}

//laser current
XnStatus XN_CALLBACK_TYPE XnSensor::SetLaserCurrentCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLaserCurrent((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLaserCurrentCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nLaserCurrent = 0;
    XnStatus rc = pThis->GetLaserCurrent(nLaserCurrent);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLaserCurrent;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetSoftResetCallback(XnIntProperty* /*pSender*/, XnUInt64 /*nValue*/, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetSoftReset();
}

//switch dual camera ir
XnStatus XN_CALLBACK_TYPE XnSensor::SetSwitchIrCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetSwitchIr(nValue == TRUE);
}

//rgb ae mode
XnStatus XN_CALLBACK_TYPE XnSensor::SetRgbAeModeCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnRgbAeMode);
    XnSensor* pThis = (XnSensor*)pCookie;
    const XnRgbAeMode *pRgbAemode = (const XnRgbAeMode *)gbValue.data;
    return pThis->SetRgbAeMode(pRgbAemode);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetRgbAeModeCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnRgbAeMode);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetRgbAeMode((XnRgbAeMode*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareFrameSyncCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareFrameSync(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareLogFilterCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareLogFilter((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareLogIntervalCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareLogInterval((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareLogPrintCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareLogPrint((XnBool)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareCPUIntervalCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFirmwareCPUInterval((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetReadAllEndpointsCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetReadAllEndpoints((XnBool)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetAPCEnabledCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetAPCEnabled((XnBool)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetI2CCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnI2CWriteData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetI2C((const XnI2CWriteData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::DeleteFileCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->DeleteFile((XnUInt16)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetTecSetPointCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTecSetPoint((XnUInt16)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetEmitterSetPointCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetEmitterSetPoint((XnUInt16)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFileAttributesCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnFileAttributes);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetFileAttributes((const XnFileAttributes*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::WriteFlashFileCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnParamFileData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->WriteFlashFile((const XnParamFileData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetProjectorFaultCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnProjectorFaultData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetProjectorFault((XnProjectorFaultData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetI2CCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnI2CReadData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetI2C((XnI2CReadData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTecStatusCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnTecData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetTecStatus((XnTecData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTecFastConvergenceStatusCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnTecFastConvergenceData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetTecFastConvergenceStatus((XnTecFastConvergenceData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetEmitterStatusCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnEmitterData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetEmitterStatus((XnEmitterData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::ReadFlashFileCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnParamFileData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->ReadFlashFile((const XnParamFileData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFirmwareLogCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetFirmwareLog((XnChar*)gbValue.data, gbValue.dataSize);
}

XnStatus XN_CALLBACK_TYPE XnSensor::ReadFlashChunkCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnParamFlashData);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->ReadFlashChunk((XnParamFlashData*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFileListCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnFlashFileList);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetFileList((XnFlashFileList*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::WriteFlashDistortionParamCallback(XnGeneralProperty*  /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnDistortionParam);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->WriteFlashDistortionParam((XnDistortionParam*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::ReadFlashDistortionParamCallback(const XnGeneralProperty*  /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnDistortionParam);
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->ReadFlashDistortionParam((XnDistortionParam*)gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetDepthDistortionStateCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetObDistortionEnableState((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDepthDistortionStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 tempState = 0;
    XnStatus rc = pThis->GetObDistortionEnableState(tempState);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)tempState;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::RunBISTCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnBist);
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBist* pBist = (XnBist*)gbValue.data;
    XnStatus nRetVal = pThis->RunBIST(pBist->nTestsMask, &pBist->nFailures);
    XN_IS_STATUS_OK(nRetVal);
    return XN_STATUS_OK;
}

void XN_CALLBACK_TYPE XnSensor::OnDeviceDisconnected(const OniDeviceInfo& deviceInfo, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PRODUCT_TEST_VERSION)
    {
        return;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    if (pThis == NULL)
    {
        return;
    }

    if (xnOSStrCmp(deviceInfo.uri, pThis->GetUSBPath()) == 0)
    {
        pThis->SetErrorState(XN_STATUS_DEVICE_NOT_CONNECTED);
    }
}

void XnSensor::SetDevicePID(XnUInt16 nPid)
{
    m_nPId = nPid;
}

XnUInt16 XnSensor::GetDevicePID()
{
    return m_nPId;
}

XnStatus XnSensor::SetFirmwareQN(const OBFirmwareQN* qN)
{
    return XnHostProtocolSetFirmwareQN(&m_DevicePrivateData, qN);
}
XnStatus XnSensor::GetFirmwareQN(OBFirmwareQN* qN)
{
    return XnHostProtocolGetFirmwareQN(&m_DevicePrivateData, qN);
}
XnStatus XnSensor::VerifyQN(const OBFirmwareQN* qN)
{
    return XnHostProtocolVerifyQN(&m_DevicePrivateData, qN);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFirmwareQNCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBFirmwareQN);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBFirmwareQN* qN = (OBFirmwareQN*)gbValue.data;
    return pThis->SetFirmwareQN(qN);
}
XnStatus XN_CALLBACK_TYPE XnSensor::GetFirmwareQNCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBFirmwareQN);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBFirmwareQN* qN = (OBFirmwareQN*)gbValue.data;
    return pThis->GetFirmwareQN(qN);
}
XnStatus XN_CALLBACK_TYPE XnSensor::VerifyQNCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBFirmwareQN);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBFirmwareQN* qN = (OBFirmwareQN*)gbValue.data;
    return pThis->VerifyQN(qN);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPublicBoardVersionCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{

    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnDevicePrivateData* pPrivateData = pThis->GetDevicePrivateData();
    if (XN_STATUS_OK == XnHostProtocolGetPublicBoardVersion(pPrivateData, (OBPublicBoardVersion*)(gbValue.data)))
    {
        return XN_STATUS_OK;
    }

    return XN_STATUS_ERROR;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMX6300VersionCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDevicePrivateData* pPrivateData = pThis->GetDevicePrivateData();
    if (XN_STATUS_OK == XnHostProtocolMx6300FirmewarGetVersion(pPrivateData, (ObMX6300Version*)(gbValue.data)))
    {
        return XN_STATUS_OK;
    }

    return XN_STATUS_ERROR;
}



XnStatus XnSensor::SetD2CResolution(XnUInt16 nResolution)
{
    return XnHostProtocolSetD2CResolution(&m_DevicePrivateData, nResolution);
}


XnStatus XnSensor::GetD2CResolution(XnUInt16 &nResolution)
{
    return XnHostProtocolGetD2CResolution(&m_DevicePrivateData, nResolution);
}


XnStatus XN_CALLBACK_TYPE XnSensor::SetD2CResolutionCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetD2CResolution((XnUInt16)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetD2CResolutionCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt16 nResolution = 0;
    XnStatus rc = pThis->GetD2CResolution(nResolution);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nResolution;
    }

    return rc;
}

XnStatus XnSensor::GetUsbDeviceSpeed(XnUInt16 &nSpeed)
{
    return XnHostProtocolGetUsbDeviceSpeed(&m_DevicePrivateData, nSpeed);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetUsbDeviceSpeedCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt16 nSpeed = 0;
    XnStatus rc = pThis->GetUsbDeviceSpeed(nSpeed);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nSpeed;
    }

    return rc;
}

XnStatus XnSensor::SetDeviceSN(const OBSerialNumber* sN)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    return XnHostProtocolSetSerialNumber(&m_DevicePrivateData, sN);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetDeviceSerialNumberCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBSerialNumber);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBSerialNumber* sN = (OBSerialNumber*)gbValue.data;
    return pThis->SetDeviceSN(sN);
}

//Get Device sensor SN
XnStatus XnSensor::GetDeviceSN(OBSerialNumber* sN)
{
    return XnHostProtocolGetSerialNumber(&m_DevicePrivateData, sN);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDeviceSerialNumberCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBSerialNumber);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBSerialNumber* sN = (OBSerialNumber*)gbValue.data;
    return pThis->GetDeviceSN(sN);
}


XnStatus XnSensor::SetDevicePN(const OBKTProductNumber* qN)
{
    return XnHostProtocolSetKT_PN(&m_DevicePrivateData, qN);
}

XnStatus XnSensor::GetDevicePN(OBKTProductNumber* qN)
{
    return XnHostProtocolGetKT_PN(&m_DevicePrivateData, qN);
}

XnStatus XnSensor::SetObPdEnableState(XnUInt32 nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolSetPdEnableStatus(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetObPdEnableState(XnUInt32 &nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetPdEnableStatus(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetObPdAlertState(XnUInt32 &nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetPdAlertStatus(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::SetObPdUpperTlv(XnUInt32 nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolSetPdUpperTlv(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetObPdUpperTlv(XnUInt32 &nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetPdUpperTlv(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::SetObPdLowerTlv(XnUInt32 nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolSetPdLowerTlv(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetObPdLowerTlv(XnUInt32 &nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetPdLowerTlv(&m_DevicePrivateData, nActive);
}



XnStatus XnSensor::GetPdCurrentThreshold(OBPdThreshold* pd)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetPdCurrentThreshold(&m_DevicePrivateData, pd);
}

XnStatus XnSensor::SetObBootLoaderPtsState(XnBool nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolSetBootLoaderPtsStatus(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetObBootLoaderPtsState(XnBool &nActive)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }
    return XnHostProtocolGetBootLoaderPtsStatus(&m_DevicePrivateData, nActive);
}

XnStatus XnSensor::GetDeviceCfgPN(XnChar* cfgPn)
{
    return XnHostProtocolGetCfgProductNumber(&m_DevicePrivateData, cfgPn);
}

XnStatus XnSensor::SetAEOptions(AeParamsStruct* pValue, XnUInt32 nType)
{
    return XnHostProtocolSetAEOption(&m_DevicePrivateData, (AEOption)nType, pValue);
}

XnStatus XnSensor::GetAEOptions(AeParamsStruct* pValue, XnUInt32 nType)
{
    return XnHostProtocolGetAEOption(&m_DevicePrivateData, (AEOption)nType, pValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetDevicePNCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBKTProductNumber);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBKTProductNumber* pN = (OBKTProductNumber*)gbValue.data;
    return pThis->SetDevicePN(pN);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDevicePNCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBKTProductNumber);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBKTProductNumber* pN = (OBKTProductNumber*)gbValue.data;
    return pThis->GetDevicePN(pN);
}

XnStatus XN_CALLBACK_TYPE  XnSensor::SetPdEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetObPdEnableState((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPdEnableCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 pdStatus = 0;
    XnStatus rc = pThis->GetObPdEnableState(pdStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)pdStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPdAlertCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 pdAlertStatus = 0;
    XnStatus rc = pThis->GetObPdAlertState(pdAlertStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)pdAlertStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPdUpperTlvCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 pdAlertStatus = 0;
    XnStatus rc = pThis->GetObPdUpperTlv(pdAlertStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)pdAlertStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE  XnSensor::SetPdUpperTlvCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetObPdUpperTlv((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPdLowerTlvCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 pdAlertStatus = 0;
    XnStatus rc = pThis->GetObPdLowerTlv(pdAlertStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)pdAlertStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE  XnSensor::SetPdLowerTlvCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetObPdLowerTlv((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDevicePdCurrenthresholdCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBPdThreshold);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBPdThreshold* pd = (OBPdThreshold*)gbValue.data;
    return pThis->GetPdCurrentThreshold(pd);
}

XnStatus XN_CALLBACK_TYPE  XnSensor::SetBootLoaderPtsCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetObBootLoaderPtsState(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetBootLoaderPtsCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool btPtsStatus = 0;
    XnStatus rc = pThis->GetObBootLoaderPtsState(btPtsStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)btPtsStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetDeviceCfgPNCallback(const XnActualStringProperty* /*pSender*/, XnChar* csValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetDeviceCfgPN(csValue);
}

XnStatus XnSensor::GetIRSensorModel(XnUInt32 &irModel)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    return XnHostProtocalGetIRSensorModel(&m_DevicePrivateData, irModel);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetIRSensorModelCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nIrModel = 0;
    XnStatus rc = pThis->GetIRSensorModel(nIrModel);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nIrModel;
    }
    return rc;
}

XnStatus XnSensor::GetRgbSensorModel(XnUInt32 &rgbModel)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    return XnHostProtocalGetRgbSensorModel(&m_DevicePrivateData, rgbModel);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetRgbSensorModelCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nRgbModel = 0;
    XnStatus rc = pThis->GetRgbSensorModel(nRgbModel);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nRgbModel;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetFloodAEOptionsCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, AeParamsStruct);
    XnSensor* pThis = (XnSensor*)pCookie;
    AeParamsStruct* pAEOptions = (AeParamsStruct*)gbValue.data;
    return pThis->SetAEOptions(pAEOptions, AE_SET_FLOOD_OPTION);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetFloodAEOptionsCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, AeParamsStruct);
    XnSensor* pThis = (XnSensor*)pCookie;
    AeParamsStruct* pAEOptions = (AeParamsStruct*)gbValue.data;
    return pThis->GetAEOptions(pAEOptions, AE_GET_FLOOD_OPTION);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetEmitterAEOptionsCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, AeParamsStruct);
    XnSensor* pThis = (XnSensor*)pCookie;
    AeParamsStruct* pAEOptions = (AeParamsStruct*)gbValue.data;
    return pThis->SetAEOptions(pAEOptions, AE_SET_EMITTER_OPTION);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetEmitterAEOptionsCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, AeParamsStruct);
    XnSensor* pThis = (XnSensor*)pCookie;
    AeParamsStruct* pAEOptions = (AeParamsStruct*)gbValue.data;
    return pThis->GetAEOptions(pAEOptions, AE_GET_EMITTER_OPTION);
}

XnStatus XnSensor::SetMipiTestStatus(XnBool bActive)
{
    return XnHostProtocolSetMipiTestEnable(&m_DevicePrivateData, bActive);
}

XnStatus XnSensor::GetMipiTestStatus(XnBool &bActive)
{
    return XnHostProtocolGetMipiTestEnable(&m_DevicePrivateData, bActive);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetMipiTestEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetMipiTestStatus(nValue == TRUE);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMipiTestStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool bMipiTestStatus = 0;
    XnStatus rc = pThis->GetMipiTestStatus(bMipiTestStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt16)bMipiTestStatus;
    }
    return rc;
}


XnStatus XnSensor::I2CReadFlash(XnParamFlashData* pValue)
{
    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = XnHostProtocolI2CReadFlash(&m_DevicePrivateData, pValue->nOffset, pValue->nSize, pValue->pData);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetI2CReadFlashCallback(const XnGeneralProperty*  /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    //XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, XnParamFlashData);
    XnParamFlashData *pFlashParam = (XnParamFlashData*)gbValue.data;
    XnSensor* pThis = (XnSensor*)pCookie;
    XnVersions &versions = pThis->m_DevicePrivateData.Version;
    if (XN_SENSOR_CHIP_VER_MX6000 == versions.ChipVer){
        return pThis->I2CReadFlash(pFlashParam);
    }
    //  XN_SENSOR_CHIP_VER_DUAL_MX6000  Device 
    ObContent_t obContent;
    OniStatus ret = pThis->GetDualCameraParam(obContent);
    if (ret != ONI_STATUS_OK){
        return ret;
    }
    int nSize = pFlashParam->nSize;
    int nD2cSize = sizeof(obContent.HOST.soft_d2c);
    int nCopySize = nD2cSize > nSize ? nSize : nD2cSize;
    xnOSMemCopy(pFlashParam->pData, &obContent.HOST.soft_d2c, nCopySize);

    return ret;
}

XnStatus XnSensor::GetI2CDualCameraParam(ObContent_t &obContent){
    int nContentSize = sizeof(obContent);
    int offset = sizeof(obContent.DPU) + FLASH_CAM_PARAMS;
    int nSize = sizeof(obContent.HOST.virCam);
    XnParamFlashData cameraFlash_temp;
    memset(&cameraFlash_temp, 0, sizeof(cameraFlash_temp));
    cameraFlash_temp.nOffset = offset;
    cameraFlash_temp.nSize = nSize;
    cameraFlash_temp.pData = (uint8_t*)&(obContent.HOST.virCam);
    int nRetVal = I2CReadFlash((XnParamFlashData*)&cameraFlash_temp);
    if (nRetVal != XN_STATUS_OK)
    {
        xnLogError(XN_MASK_SENSOR_READ, "get dual camera param failed 1(%d)\n", nRetVal);
        return nRetVal;
    }
    nSize = sizeof(obContent.HOST.soft_d2c);
    offset = (nContentSize - nSize) + FLASH_CAM_PARAMS;
    memset(&cameraFlash_temp, 0, sizeof(cameraFlash_temp));
    cameraFlash_temp.nOffset = offset;
    cameraFlash_temp.nSize = nSize;
    cameraFlash_temp.pData = (uint8_t*)&(obContent.HOST.soft_d2c);
    nRetVal = I2CReadFlash((XnParamFlashData*)&cameraFlash_temp);
    if (nRetVal != 0)
    {
        xnLogError(XN_MASK_SENSOR_READ, "get dual camera param failed 2(%d)\n", nRetVal);
        return nRetVal;
    }
    return nRetVal;
}

OniStatus XnSensor::GetDualCameraParam(ObContent_t &obContent)
{
    int nContentSize = sizeof(obContent);
    xnOSMemSet(&obContent, 0, nContentSize);
    if (m_bReadCameraParam)
    {
        xnOSMemCopy(&obContent, &m_ObContent, sizeof(obContent));
        return ONI_STATUS_OK;
    }

    XnStatus ret = GetI2CDualCameraParam(obContent);
    if (ret != 0){
        //spi
        xnOSMemSet(&obContent, 0, nContentSize);
        ret = ReadFlash(FLASH_CAM_PARAMS, nContentSize / 2, (XnUInt8 *)&obContent);
        if (ret != 0)
        {
            xnLogError(XN_MASK_SENSOR_READ, "get dual camera param failed (%d)\n", ret);
            return ONI_STATUS_ERROR;
        }
    }
    xnOSMemCopy(&m_ObContent, &obContent, sizeof(obContent));
    m_bReadCameraParam = TRUE;
    return ONI_STATUS_OK;
}

XnStatus XnSensor::GetCameraParam(OBCameraParams &obCameraParams)
{
    int nSize = sizeof(OBCameraParams);
    xnOSMemSet(&obCameraParams, 0, nSize);

    if (m_bReadCameraParam)
    {
        xnOSMemCopy(&obCameraParams, &m_ObCameraParams, sizeof(OBCameraParams));
        return ONI_STATUS_OK;
    }

    XnStatus nRetVal = XN_STATUS_OK;
    XnVersions &versions = m_DevicePrivateData.Version;
    if (XN_SENSOR_CHIP_VER_MX6000 == versions.ChipVer)
    {
        //mipi public board
        XnParamFlashData cameraFlash;
        memset(&cameraFlash, 0, sizeof(cameraFlash));
        cameraFlash.nOffset = FLASH_CAM_PARAMS;
        cameraFlash.nSize = nSize;
        cameraFlash.pData = (uint8_t*)&obCameraParams;

        XnStatus nRetVal = I2CReadFlash((XnParamFlashData*)&cameraFlash);
        if (nRetVal != XN_STATUS_OK)
        {
            nRetVal = ReadFlash(FLASH_CAM_PARAMS, (uint16_t)nSize / 2, (uint8_t *)&obCameraParams, (uint16_t)nSize);
        }
    }
    else
    {
        nRetVal = ReadFlash(FLASH_CAM_PARAMS, (uint16_t)nSize / 2, (uint8_t *)&obCameraParams, (uint16_t)nSize);
    }

    if (nRetVal == XN_STATUS_OK)
    {
        xnOSMemCopy(&m_ObCameraParams, &obCameraParams, nSize);
        m_bReadCameraParam = TRUE;
    }

    return nRetVal;
}

XnStatus XnSensor::ReadFlash(XnUInt32 offset, XnUInt32 size, XnUInt8 *data_ptr)
{
    int addr_offset = 0;
    XnUInt8	Receivebuf[512] = { 0 };
    int sizeInBytes = size * 2;
    int lastsizeInBytes = sizeInBytes % EATCH_PACKET_SIZE;

    for (int k = 0; k < sizeInBytes / EATCH_PACKET_SIZE; k++)
    {
        XnStatus nRetVal = ReadFlash(offset + addr_offset, (uint16_t)EATCH_PACKET_SIZE / 2, (uint8_t *)Receivebuf, EATCH_PACKET_SIZE);

        if (nRetVal != XN_STATUS_OK)
        {
            return nRetVal;
        }

        xnOSMemCopy(data_ptr + addr_offset, Receivebuf, EATCH_PACKET_SIZE);

        addr_offset = addr_offset + EATCH_PACKET_SIZE;
    }

    if (lastsizeInBytes != 0)
    {

        XnStatus nRetVal = ReadFlash(offset + addr_offset, (uint16_t)lastsizeInBytes / 2, (uint8_t *)Receivebuf, lastsizeInBytes);
        if (nRetVal != XN_STATUS_OK)
        {
            return nRetVal;
        }

        xnOSMemCopy(data_ptr + addr_offset, Receivebuf, lastsizeInBytes);

    }

    return XN_STATUS_OK;
}

XnStatus XnSensor::ReadFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size)
{

    XnStatus ret = XN_STATUS_OK;
    uint8_t cmdbuf[10];
    //create local buffer
    uint8_t * readbuf = NULL;
    if (((uint32_t)dw_size * 2 > buffer_size) || (dw_size > 256))
    {
        return XN_STATUS_ERROR;
    }
    readbuf = (uint8_t *)malloc(2 + dw_size * 2);
    if (readbuf == NULL)
    {
        return XN_STATUS_ERROR;
    }

    memset(readbuf, 0, 2 + dw_size * 2);
    //*(uint32_t *)&cmdbuf[0] = offset;
    uint32_t* pOffset = (uint32_t*)cmdbuf;
    *pOffset = offset;
    //*(uint16_t *)&cmdbuf[4] = dw_size;
    uint16_t* pCmd = (uint16_t*)(cmdbuf + 4);
    *pCmd = dw_size;
    ret = SendCmd(OPCODE_READ_FLASH, cmdbuf, 2 * 3, readbuf, dw_size * 2 + 2);
    if (ret != XN_STATUS_OK){
        free(readbuf);
        return ret;
    }

    //copy to buffer
    memcpy(buffer, &readbuf[2], dw_size * 2);
    free(readbuf);
    return ret;
}

XnStatus XnSensor::SendCmd(uint16_t cmd, void *cmdbuf, uint16_t cmd_len, void *replybuf, uint16_t reply_len)
{
    int res;
    uint8_t obuf[0x400];
    uint8_t ibuf[0x200];
    unsigned int actual_len;
    cam_hdr *chdr = (cam_hdr*)obuf;
    cam_hdr *rhdr = (cam_hdr*)ibuf;

    XN_USB_DEV_HANDLE hUSBDevice = (m_DevicePrivateData.SensorHandle).USBDevice;
    if (hUSBDevice == 0) return XN_STATUS_ERROR;

    if (cmd_len & 1 || cmd_len > (0x400 - sizeof(*chdr))) {
        return XN_STATUS_ERROR;
    }

    chdr->magic[0] = 0x47;
    chdr->magic[1] = 0x4d;
    chdr->cmd = cmd;
    //chdr->tag = *cam_tag;
    chdr->tag = 0;
    chdr->len = cmd_len / 2;
    //copy the cmdbuf
    memcpy(obuf + sizeof(*chdr), cmdbuf, cmd_len);

    res = xnUSBSendControl(hUSBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar*)obuf, cmd_len + sizeof(*chdr), 5000);
    if (res < 0)
    {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Output control transfer failed (%d)\n!", res);
        return XN_STATUS_ERROR;
    }
    do
    {
        xnUSBReceiveControl(hUSBDevice, XN_USB_CONTROL_TYPE_VENDOR, 0x00, 0x0000, 0x0000, (XnUChar *)ibuf, 0x200, &actual_len, 5000);
        //print_dbg("send_cmd: actual length = %d\n", actual_len);
    } while ((actual_len == 0) || (actual_len == 0x200));

    //print_dbg("Control reply: %d\n", res);
    if (actual_len < (int)sizeof(*rhdr)) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Input control transfer failed (%d)\n", res);
        return XN_STATUS_ERROR;
    }
    actual_len -= sizeof(*rhdr);

    if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
        return XN_STATUS_ERROR;
    }

    if (rhdr->cmd != chdr->cmd) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
        return XN_STATUS_ERROR;
    }

    if (rhdr->tag != chdr->tag) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
        return XN_STATUS_ERROR;
    }

    if (rhdr->len != (actual_len / 2)) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Bad len %04x != %04x\n", rhdr->len, (int)(actual_len / 2));
        return XN_STATUS_ERROR;
    }

    if (actual_len > reply_len) {
        xnLogError(XN_MASK_SENSOR_READ, "send_cmd: Data buffer is %d bytes long, but got %d bytes\n", reply_len, actual_len);
        memcpy(replybuf, ibuf + sizeof(*rhdr), reply_len);
    }
    else {
        memcpy(replybuf, ibuf + sizeof(*rhdr), actual_len);
    }

    //(*cam_tag)++;

    return XN_STATUS_OK;
}
XnStatus XN_CALLBACK_TYPE XnSensor::GetZ0BaselineCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OBZ0Baseline);
    XnSensor* pThis = (XnSensor*)pCookie;
    OBZ0Baseline* pZ0Baseline = (OBZ0Baseline*)gbValue.data;
    pZ0Baseline->fZ0 = (float)pThis->GetFixedParams()->GetZeroPlaneDistance();
    pZ0Baseline->fBaseline = (float)pThis->GetFixedParams()->GetEmitterDCmosDistance();

    return XN_STATUS_OK;
}

XnStatus XnSensor::GetDistortionParam(XnDistortionParam &distortionParam)
{
    XnStatus status = XN_STATUS_ERROR;
    if (m_bReadDistortParam)
    {
        distortionParam = m_distortionParam;
        status = XN_STATUS_OK;
    }

    return status;
}

XnUInt32 XnSensor::GetMultiDisCalEnable(void)
{
    return m_bMultiDisCalEnable;
}

XnStatus XnSensor::GetPlatformString(XnChar* csPlatform)
{
    return XnHostProtocolGetPlatformString(&m_DevicePrivateData, csPlatform);
}

XnStatus XnSensor::GetTOFFreqMode(XnUInt16* pMode)
{
    return XnHostProtocolGetTOFFreqMode(&m_DevicePrivateData, pMode);
}

XnStatus XnSensor::GetTOFSensorFilterLevel(XnUInt16* level)
{
	return XnHostProtocolGetTOFSensorFilterLevel(&m_DevicePrivateData, level);
}
XnStatus XnSensor::GetTOFSensorIntegrationTime(XnUInt32* integrationTime) {
	return XnHostProtocolGetTOFSensorIntegrationTime(&m_DevicePrivateData, integrationTime);
}
XnStatus XnSensor::GetTOFSensorGain(XnUInt16* gain) {
	return XnHostProtocolGetTOFSensorGain(&m_DevicePrivateData, gain);
}
XnStatus XnSensor::GetTOFSensorLaserInterference(XnUInt16* value) {
	return XnHostProtocolGetTOFSensorLaserInterference(&m_DevicePrivateData, value);
}
XnStatus XnSensor::GetTOFSensorWorkingMode(XnUInt16* value) {
	return XnHostProtocolGetTOFSensorWorkingMode(&m_DevicePrivateData, value);
}
XnStatus XnSensor::GetGeneralFrequency(ORBTofFrequency* pData) {
	return XnHostProtocolGetGeneralFrequency(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::GetGeneralDutyCycle(ORBTofDuty* pData) {
	return XnHostProtocolGetGeneralDutyCycle(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::GetGeneralDriverICReg(ObReg8Map* pData) {
	return XnHostProtocolGetGeneralDriverICReg(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::GetGeneralSensorReg(ObReg16Map* pData) {
	return XnHostProtocolGetGeneralSensorReg(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::GetTOFFreqModeCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
    XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetTOFFreqMode((XnUInt16*)pValue);
}

XnStatus XnSensor::SetTOFFreqMode(const XnUInt16 mode)
{
    XnStatus ret = XnHostProtocolSetTOFFreqMode(&m_DevicePrivateData, mode);
    XN_IS_STATUS_OK(ret);

    return m_freqMode.UnsafeUpdateValue(mode);
}

XnStatus XnSensor::SetTOFSensorFilterLevel(const XnUInt16 filterlevel) {
	XnStatus ret = XnHostProtocolSetTOFSensorFilterLevel(&m_DevicePrivateData, filterlevel);
	XN_IS_STATUS_OK(ret);
	return m_FilterLevel.UnsafeUpdateValue(filterlevel);
}
XnStatus XnSensor::SetTOFSensorIntegrationTime(const XnUInt32 integrationTime) {
	XnStatus ret = XnHostProtocolSetTOFSensorIntegrationTime(&m_DevicePrivateData, integrationTime);
	XN_IS_STATUS_OK(ret);
	return m_TofSensorIntegrationTime.UnsafeUpdateValue(integrationTime);
}
XnStatus XnSensor::SetTOFSensorGain(const XnUInt16 gain) {
	XnStatus ret = XnHostProtocolSetTOFSensorGain(&m_DevicePrivateData, gain);
	XN_IS_STATUS_OK(ret);
	return m_TofSensorGain.UnsafeUpdateValue(gain);
}
XnStatus XnSensor::SetTOFSensorLaserInterference(const XnUInt16 value) {
	XnStatus ret = XnHostProtocolSetTOFSensorLaserInterference(&m_DevicePrivateData, value);
	XN_IS_STATUS_OK(ret);
	return m_TofSensorLaserInterference.UnsafeUpdateValue(value);
}
XnStatus XnSensor::SetTOFSensorWorkingMode(const XnUInt16 value) {
	XnStatus ret = XnHostProtocolSetTOFSensorWorkingMode(&m_DevicePrivateData, value);
	XN_IS_STATUS_OK(ret);
	return m_TofSensorWorkingMode.UnsafeUpdateValue(value);
}
XnStatus XnSensor::SetTOFFreqModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTOFFreqMode((XnUInt16)nValue);
}
XnStatus XnSensor::GetTOFSensorFilterLevelCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
	XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetTOFSensorFilterLevel((XnUInt16*)pValue);
}
XnStatus XnSensor::SetTOFSensorFilterLevelCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetTOFSensorFilterLevel((XnUInt16)nValue);
}
XnStatus XnSensor::GetTOFSensorIntegrationTimeCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
	XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetTOFSensorIntegrationTime((XnUInt32*)pValue);
}
XnStatus XnSensor::GetTOFSensorGainCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
	XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetTOFSensorGain((XnUInt16*)pValue);
}
XnStatus XnSensor::GetTOFSensorLaserInterferenceCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
	XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetTOFSensorLaserInterference((XnUInt16*)pValue);
}
XnStatus XnSensor::GetTOFSensorWorkingModeCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie) {
	XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetTOFSensorWorkingMode((XnUInt16*)pValue);
}
XnStatus XnSensor::GetTOFSensorFrequencyCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie) {
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetGeneralFrequency((ORBTofFrequency*)gbValue.data);
}
XnStatus XnSensor::GetTOFSensorDutyCycleCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie) {
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetGeneralDutyCycle((ORBTofDuty*)gbValue.data);
}
XnStatus XnSensor::GetTOFSensorDriverICRegCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie) {
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetGeneralDriverICReg((ObReg8Map*)gbValue.data);
}
XnStatus XnSensor::GetTOFSensorSensorRegCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie) {
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->GetGeneralSensorReg((ObReg16Map*)gbValue.data);
}
XnStatus XnSensor::SetTOFSensorIntegrationTimeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetTOFSensorIntegrationTime((XnUInt32)nValue);
}
XnStatus XnSensor::SetTOFSensorGainCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetTOFSensorGain((XnUInt16)nValue);
}
XnStatus XnSensor::SetTOFSensorLaserInterferenceCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetTOFSensorLaserInterference((XnUInt16)nValue);
}
XnStatus XnSensor::SetTOFSensorWorkingModeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie) {
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetTOFSensorWorkingMode((XnUInt16)nValue);
}
XnStatus XnSensor::SetTOFSensorFrequencyCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, ORBTofFrequency);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetGeneralFrequency((ORBTofFrequency*)gbValue.data);
}
XnStatus XnSensor::SetTOFSensorDutyCycleCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, ORBTofDuty);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetGeneralDutyCycle((ORBTofDuty*)gbValue.data);
}
XnStatus XnSensor::SetTOFSensorDriverICRegCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, ObReg8Map);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetGeneralDriverICReg((ObReg8Map*)gbValue.data);
}
XnStatus XnSensor::SetTOFSensorSensorRegCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
	XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
	XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
	XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, ObReg16Map);
	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetGeneralSensorReg((ObReg16Map*)gbValue.data);
}

XnStatus XnSensor::GetSensorID(OniSensorIDMap* pSensorID)
{
    return XnHostProtocolGetSensorID(&m_DevicePrivateData, pSensorID);
}

XnStatus XnSensor::GetSensorIDCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OniSensorIDMap);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetSensorID((OniSensorIDMap*)gbValue.data);
}

XnStatus XnSensor::GetGeneralSerialNumber(OniSerialNumberMap* pSerialMap)
{
    return XnHostProtocolGetGeneralSerialNumber(&m_DevicePrivateData, pSerialMap);
}

XnStatus XnSensor::GetGeneralSerialNumberCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OniSerialNumberMap);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetGeneralSerialNumber((OniSerialNumberMap*)gbValue.data);
}

XnStatus XnSensor::SetGeneralSerialNumber(const OniSerialNumberMap* pSerialNumber)
{
    return XnHostProtocolSetGeneralSerialNumber(&m_DevicePrivateData, pSerialNumber);
}
XnStatus XnSensor::SetGeneralFrequency(const ORBTofFrequency* pData) {
	return XnHostProtocolSetGeneralFrequency(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::SetGeneralDutyCycle(const ORBTofDuty* pData) {
	return XnHostProtocolSetGeneralDutyCycle(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::SetGeneralDriverICReg(const ObReg8Map* pData) {
	return XnHostProtocolSetGeneralDriverICReg(&m_DevicePrivateData, pData);
}
XnStatus XnSensor::SetGeneralSensorReg(const ObReg16Map* pData) {
	return XnHostProtocolSetGeneralSensorReg(&m_DevicePrivateData, pData);
}

XnStatus XnSensor::SetGeneralSerialNumberCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OniSerialNumberMap);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetGeneralSerialNumber((OniSerialNumberMap*)gbValue.data);
}

XnStatus XnSensor::SendCommand(OniSerialCmd* pCmd)
{
    return XnHostProtocolSendCommand(&m_DevicePrivateData, pCmd);
}

XnStatus XnSensor::SendCommandCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OniSerialCmd);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SendCommand((OniSerialCmd*)gbValue.data);
}

XnStatus XnSensor::QueryDeviceTimestamp(XnUInt64* pTimestamp)
{
    return XnHostProtocolQueryTimestamp(&m_DevicePrivateData, pTimestamp);
}

XnStatus XnSensor::QueryDeviceTimestampCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie)
{
    XN_RET_IF_NULL(pValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->QueryDeviceTimestamp(pValue);
}

XnStatus XnSensor::GetPlatformVersion(XnChar* pValue)
{
    return XnHostProtocolGetPlatformVersion(&m_DevicePrivateData, m_DevicePrivateData.FWInfo.nOpcodeGetPlatformVersion, pValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPlatformVersionCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie)
{
    XN_RET_IF_NULL(csValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetPlatformVersion(csValue);
}

XnStatus XnSensor::GetPlatformSDKVersion(XnChar* pValue)
{
    return XnHostProtocolGetPlatformVersion(&m_DevicePrivateData, m_DevicePrivateData.FWInfo.nOpcodeGetPlatformSDKVersion, pValue);
}

XnStatus XnSensor::GetPlatformSDKVersionCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie)
{
    XN_RET_IF_NULL(csValue, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetPlatformSDKVersion(csValue);
}

XnStatus XnSensor::StartService(const OniService* pService)
{
    return XnHostProtocolStartService(&m_DevicePrivateData, pService);
}

XnStatus XnSensor::StartServiceCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);
    XN_VALIDATE_GENERAL_BUFFER_TYPE(gbValue, OniService);

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->StartService((const OniService*)gbValue.data);
}

XnStatus XnSensor::SendUsbFile(const XnUsbGeneralFile* pUsbFile)
{
    return XnHostProtocolSendUsbFile(&m_DevicePrivateData, pUsbFile);
}

XnStatus XnSensor::SendUsbFile(const OniFileAttributes * pFileAttributes)
{
    XN_RET_IF_NULL(pFileAttributes, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pFileAttributes->path, XN_STATUS_NULL_INPUT_PTR);

    std::string filePath(pFileAttributes->path);
    size_t pos = filePath.find_last_of("/");
    if (pos != std::string::npos)
        pos += 1;
    else
    {
        pos = filePath.find_last_of("\\");
        if (pos != std::string::npos)
            pos += 1;
        else
            pos = 0;
    }
    std::string fileName = filePath.substr(pos);
    XnInt32 fileNameLen = fileName.size();
    if (fileNameLen >= XN_USB_MAX_FILE_NAME_LENGTH)
    {
        fileNameLen = XN_USB_MAX_FILE_NAME_LENGTH - 1;
        xnLogWarning(XN_MASK_SENSOR_READ, "The file name <%s> is too long <%d>, it would be truncated to <%d> bytes.",
            fileName.c_str(), fileName.size(), XN_USB_MAX_FILE_NAME_LENGTH);
    }

    XnIOFileStream ioFile(filePath.c_str(), XN_OS_FILE_READ);
    XnStatus ret = ioFile.Init();
    if (XN_STATUS_OK != ret)
    {
        xnLogError(XN_MASK_SENSOR_READ, "Failed to open file <%s>...", filePath.c_str());
        return ret;
    }
    XnUInt64 fileSize = 0;
    ioFile.Size(&fileSize);

    XnUsbGeneralFile usbFile = { 0 };
    usbFile.attributes.size = (XnUInt32)fileSize;
    usbFile.attributes.suffix = (XnUInt16)pFileAttributes->suffix;
    usbFile.attributes.category = (XnUInt16)pFileAttributes->category;
    xnOSStrNCopy(usbFile.attributes.name, fileName.c_str(), fileNameLen, XN_USB_MAX_FILE_NAME_LENGTH);
    xnLogInfo(XN_MASK_SENSOR_READ, "Loading file <%s, %d bytes>", filePath.c_str(), usbFile.attributes.size);

	try {
    usbFile.pContent = new XnUInt8[usbFile.attributes.size + 1]();
	}
	catch (std::bad_alloc) {
		xnLogError(XN_MASK_SENSOR_READ, "mem alloc error:" );
		delete[] usbFile.pContent;
		return XN_STATUS_ALLOC_FAILED;
	}
	if (NULL == usbFile.pContent) {
		return XN_STATUS_ALLOC_FAILED;
	}
    ret = ioFile.ReadData(usbFile.pContent, usbFile.attributes.size);
    if (XN_STATUS_OK != ret)
    {
        delete[] usbFile.pContent;
        xnLogError(XN_MASK_SENSOR_READ, "Failed to read file <%s>...", filePath.c_str());
        return ret;
    }

    ret = SendUsbFile(&usbFile);
    delete[] usbFile.pContent;
	usbFile.pContent = NULL;
    return ret;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SendUsbFileCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pThis = (XnSensor*)pCookie;
    if (gbValue.dataSize == sizeof(XnUsbGeneralFile))
        return pThis->SendUsbFile((XnUsbGeneralFile*)gbValue.data);
    else if (gbValue.dataSize == sizeof(OniFileAttributes))
        return pThis->SendUsbFile((OniFileAttributes*)gbValue.data);
    else
    {
        xnLogError(XN_MASK_SENSOR_READ, "Bad parameters...");
        return XN_STATUS_INVALID_BUFFER_SIZE;
    }
}

XnStatus XnSensor::SetJavaVM(const void* pJavaVM)
{
    m_pJavaVM = pJavaVM;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetJavaVMCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XN_RET_IF_NULL(pSender, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pCookie, XN_STATUS_NULL_INPUT_PTR);

    XnSensor* pSelf = (XnSensor*)pCookie;
    return pSelf->SetJavaVM(gbValue.data);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPlatformStringCallback(const XnActualStringProperty* /*pSender*/, XnChar* csValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetPlatformString(csValue);
}

XnStatus XnSensor::GetCoreBroadFlashId(XnUInt32 &nId)
{
    return XnHostProtocolGetCoreBroadFlashId(&m_DevicePrivateData, nId);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetCoreBroadFlashIdCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nId = 0;
    XnStatus rc = pThis->GetCoreBroadFlashId(nId);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nId;
    }

    return rc;
}


//laser time
XnStatus XnSensor::SetLaserTime(XnUInt32 nLaserTime)
{
    return XnHostProtocolSetLaserTime(&m_DevicePrivateData, nLaserTime);
}

XnStatus XnSensor::GetLaserTime(XnUInt32 &nLaserTime)
{
    return XnHostProtocolGetLaserTime(&m_DevicePrivateData, nLaserTime);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetLaserTimeCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetLaserTime((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetLaserTimeCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nLaserTime = 0;
    XnStatus rc = pThis->GetLaserTime(nLaserTime);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nLaserTime;
    }

    return rc;
}

XnStatus XnSensor::SetPostFilterThreshold(XnUInt32 nPostFilterThreshold)
{
    return XnHostProtocolSetPostFilterThreshold(&m_DevicePrivateData, nPostFilterThreshold);
}

XnStatus XnSensor::GetPostFilterThreshold(XnUInt32 &nPostFilterThreshold)
{
    return XnHostProtocolGetPostFilterThreshold(&m_DevicePrivateData, nPostFilterThreshold);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetPostFilterThresholdCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetPostFilterThreshold((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetPostFilterThresholdCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nThreshold = 0;
    XnStatus rc = pThis->GetPostFilterThreshold(nThreshold);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nThreshold;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetZppsCallback(const XnRealProperty* /*pSender*/, XnDouble *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    *nValue = pThis->GetFixedParams()->GetZeroPlanePixelSize();

    return XN_STATUS_OK;
}


//TOF
XnStatus XnSensor::SetTofSensorEnableState(XnUInt32 nValue)
{
    return XnHostProtocolSetTofSensorEnable(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetTofSensorEnableState(XnUInt32 &nValue)
{
    return XnHostProtocolGetTofSensorEnable(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetTofSensorMeasureResult(XnUInt32 &nValue)
{
    return XnHostProtocolGetTofSensorMeasureResult(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetTofSensorAppId(XnUInt32 &nValue)
{
    return XnHostProtocolGetTofSensorAppId(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::SetTofSensorCalibrationValue(XnUInt32 nValue)
{
    return XnHostProtocolSetTofSensorCalibrationValue(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::SetTofSensorAppEnableState(XnUInt32 nValue)
{
    return XnHostProtocolSetTofSensorAppEnableState(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetTofSensorCalibrationParams(OBTofSensorCalParams* tofCalParams)
{
    return XnHostProtocolGetTofSensorCalibrationParams(&m_DevicePrivateData, tofCalParams);
}

XnStatus XnSensor::SetTofSensorCalibrationParams(const OBTofSensorCalParams* tofCalParams)
{
    return XnHostProtocolSetTofSensorCalibrationParams(&m_DevicePrivateData, tofCalParams);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTofSensorEnableStateCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 pdStatus = 0;
    XnStatus rc = pThis->GetTofSensorEnableState(pdStatus);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)pdStatus;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetTofSensorEnableStateCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTofSensorEnableState((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTofSensorMeasureResultCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetTofSensorMeasureResult(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTofSensorAppIdCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetTofSensorAppId(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetTofSensorcalibrationCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTofSensorCalibrationValue((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetTofSensorAppEnableStateCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTofSensorAppEnableState((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetCupVerifyVersionCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnDevicePrivateData* pPrivateData = pThis->GetDevicePrivateData();
    if (XN_STATUS_OK == XnHostProtocolGetCupVerifyVersion(pPrivateData, (CupCertify*)(gbValue.data)))
    {
        return XN_STATUS_OK;
    }

    return XN_STATUS_ERROR;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetTofSensorcalibrationParamsCallback(XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    OBTofSensorCalParams* tofCalParams = (OBTofSensorCalParams*)gbValue.data;
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetTofSensorCalibrationParams(tofCalParams);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetTofSensorcalibrationParamsCallback(const XnGeneralProperty* /*pSender*/, const OniGeneralBuffer& gbValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    OBTofSensorCalParams* tofCalParams = (OBTofSensorCalParams*)gbValue.data;
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->GetTofSensorCalibrationParams(tofCalParams);
}

//Motor
XnStatus XnSensor::SetMotorTest(XnUInt32 nValue)
{
    return XnHostProtocolSetMotorTest(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorTestResult(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorTestResult(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::SetMotorPosition(XnUInt32 nValue)
{
    return XnHostProtocolSetMotorPosition(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorPosition(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorPosition(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorStatus(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorStatus(&m_DevicePrivateData, nValue);
}


XnStatus XnSensor::GetMotorTestCount(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorTestCount(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::SetMotorRunTime(XnUInt32 nValue)
{
	return XnHostProtocolSetMotorRunTime(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorRunTime(XnUInt32 &nValue)
{
	return XnHostProtocolGetMotorRunTime(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorFeature(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorFeature(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorUpdownState(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorUpdownState(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::GetMotorUpdownTime(XnUInt32 &nValue)
{
    return XnHostProtocolGetMotorUpdownTime(&m_DevicePrivateData, nValue);
}

XnStatus XnSensor::SetMotorUpdown(XnUInt32 nValue)
{
    return XnHostProtocolSetMotorUpdown(&m_DevicePrivateData, nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetMotorTestCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetMotorTest((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorTestResultCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorTestResult(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetMotorPositionCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetMotorPosition((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorPositionCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorPosition(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorStatusCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorStatus(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}


XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorTestCountCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorTestCount(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetMotorRunTimeCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
	if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
	{
		return XN_STATUS_ERROR;
	}

	XnSensor* pThis = (XnSensor*)pCookie;
	return pThis->SetMotorRunTime((XnUInt32)nValue);
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorRunTimeCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
	if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
	{
		return XN_STATUS_ERROR;
	}

	XnSensor* pThis = (XnSensor*)pCookie;
	XnUInt32 result = 0;
	XnStatus rc = pThis->GetMotorRunTime(result);
	if (XN_STATUS_OK == rc)
	{
		*nValue = (XnUInt64)result;
	}
	return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorFeatureCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorFeature(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorUpdownStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorUpdownState(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::GetMotorUpdownTimeCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 result = 0;
    XnStatus rc = pThis->GetMotorUpdownTime(result);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)result;
    }
    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetMotorUpdownCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie)
{
    if (COMPILE_VERSION_TYPE == COMPILE_PUBLISH_VERSION)
    {
        return XN_STATUS_ERROR;
    }

    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetMotorUpdown((XnUInt32)nValue);
}

//set depthir mode
XnStatus XN_CALLBACK_TYPE XnSensor::SetDepthIrModeCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetDepthIrMode((XnUInt32)nValue);
}

//get depthir mode
XnStatus XN_CALLBACK_TYPE XnSensor::GetDepthIrModeCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nMode = 0;
    XnStatus rc = pThis->getDepthIrMode(nMode);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nMode;
    }

    return rc;
}

XnStatus XnSensor::SetDepthIrMode(XnUInt32 nMode)
{
    return XnHostProtocolSetDepthIrMode(&m_DevicePrivateData, nMode);
}

XnStatus XnSensor::getDepthIrMode(XnUInt32 &nMode)
{
    return XnHostProtocolGetDepthIrMode(&m_DevicePrivateData, nMode);
}

XnStatus XnSensor::SetTecEnable(XnBool bActive)
{
    return XnHostProtocolSetTecEnable(&m_DevicePrivateData, bActive);
}


//set depthir mode
XnStatus XN_CALLBACK_TYPE XnSensor::SetSubtractBGCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    return pThis->SetSubTractBGMode((XnUInt32)nValue);
}

//get depthir mode
XnStatus XN_CALLBACK_TYPE XnSensor::GetSubtractBGCallback(const XnIntProperty* /*pSender*/, XnUInt64 *nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnUInt32 nMode = 0;
    XnStatus rc = pThis->GetSubTractBGMode(nMode);
    if (XN_STATUS_OK == rc)
    {
        *nValue = (XnUInt64)nMode;
    }

    return rc;
}

XnStatus XN_CALLBACK_TYPE XnSensor::SetSetTecEnableCallback(XnIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnSensor* pThis = (XnSensor*)pCookie;
    XnBool bEnable = FALSE;
    if (nValue == 1)
    {
        bEnable = TRUE;
    }
    else
    {
        bEnable = FALSE;
    }

    return pThis->SetTecEnable((XnBool)bEnable);
}


XnStatus XnSensor::SetSubTractBGMode(XnUInt32 nMode)
{
    return XnHostProtocolSetSubtractBGMode(&m_DevicePrivateData, nMode);
}

XnStatus XnSensor::GetSubTractBGMode(XnUInt32 &nMode)
{
    return XnHostProtocolGetSubtractBGMode(&m_DevicePrivateData, nMode);
}
