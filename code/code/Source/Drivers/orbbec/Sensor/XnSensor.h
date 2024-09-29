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
#ifndef XNSENSOR_H
#define XNSENSOR_H

#define Tornado_D2_PID 0x060d
#define Tornado_D2_PRO_PID 0x061f
#define ASTRAPRO_PLUS 0x060f
#define BAIDU_Atlas 0x0613
#define Gemini_PID 0x0614
#define PROJECTOR_PID 0x0617
#define BUTTERFLY_PID 0x0618
#define Petrel_Pro 0x062b
#define Petrel_Plus 0x062c
#define ONEPLUS_TV 0x053F
#define KUNLUNSHAN_TV 0x0641

#define MULTI_DISCAL_DISABLE 0
#define MULTI_DISCAL_ENABLE 1
#define MULTI_DISCAL_TEST 2
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <DDK/XnDeviceBase.h>
#include "XnDeviceSensorIO.h"
#include "XnParams.h"
#include "XnDeviceSensor.h"
#include "XnSensorFixedParams.h"
#include "XnSensorFirmwareParams.h"
#include <DDK/XnDeviceStream.h>
#include "XnSensorFirmware.h"
#include "XnCmosInfo.h"
#include "IXnSensorStream.h"
#include <DDK/XnIntPropertySynchronizer.h>
#include "XnArray.h"
#include <OBExtensionCommand.h>

//---------------------------------------------------------------------------
// XnSensor class
//---------------------------------------------------------------------------
typedef enum
{
    COMPILE_PUBLISH_VERSION = 0,
    COMPILE_PRODUCT_TEST_VERSION = 1,
}CompileVersionType;

//compile for product software
//COMPILE_PRODUCT_TEST_VERSION:product software;
//COMPILE_PUBLISH_VERSION :publish software
#define COMPILE_VERSION_TYPE COMPILE_PRODUCT_TEST_VERSION

class XnSensor : public XnDeviceBase
{
    friend class XnServerSensorInvoker;

public:
    XnSensor(XnBool bResetOnStartup = TRUE, XnBool bLeanInit = FALSE);
    ~XnSensor();

    virtual XnStatus InitImpl(const XnDeviceConfig* pDeviceConfig);
    virtual XnStatus Destroy();
    virtual XnStatus OpenAllStreams();
    virtual XnStatus LoadConfigFromFile(const XnChar* csINIFilePath, const XnChar* csSectionName);

public:
    inline XnSensorFixedParams* GetFixedParams() { return GetFirmware()->GetFixedParams(); }
    inline XnSensorFirmware* GetFirmware() { return &m_Firmware; }
    inline XnSensorFPS* GetFPSCalculator() { return &m_FPS; }

    XnStatus SetCmosConfiguration(XnCMOSType nCmos, XnResolutions nRes, XnUInt32 nFPS);

    inline XnDevicePrivateData* GetDevicePrivateData() { return &m_DevicePrivateData; }

    XnStatus ConfigPropertyFromFile(XnStringProperty* pProperty, const XnChar* csINIFilePath, const XnChar* csSectionName);
    XnStatus ConfigPropertyFromFile(XnIntProperty* pProperty, const XnChar* csINIFilePath, const XnChar* csSectionName);

    inline XnBool IsLowBandwidth() const { return m_SensorIO.IsLowBandwidth(); }
    inline XnBool IsMiscSupported() const { return m_SensorIO.IsMiscEndpointSupported(); }
    inline XnBool IsAISupported() const { return m_SensorIO.IsAIEndpointSupported(); }
    inline XnSensorUsbInterface GetCurrentUsbInterface() const { return m_SensorIO.GetCurrentInterface(*m_Firmware.GetInfo()); }

    XnStatus GetStream(const XnChar* strStream, XnDeviceStream** ppStream);

    inline XnStatus GetErrorState() { return (XnStatus)m_ErrorState.GetValue(); }
    XnStatus SetErrorState(XnStatus errorState);

    /**
     * Resolves the config file's path.
     * Specify NULL to strConfigDir to resolve it based on the driver's directory.
     */
    static XnStatus ResolveGlobalConfigFileName(XnChar* strConfigFile, XnUInt32 nBufSize, const XnChar* strConfigDir);

    XnStatus SetGlobalConfigFile(const XnChar* strConfigFile);
    XnStatus ConfigureModuleFromGlobalFile(const XnChar* strModule, const XnChar* strSection = NULL);

    const XnChar* GetUSBPath() { return m_SensorIO.GetDevicePath(); }
    XnBool ShouldUseHostTimestamps() { return (m_HostTimestamps.GetValue() == TRUE); }
    XnBool HasReadingStarted() { return (m_ReadData.GetValue() == TRUE); }
    inline XnBool IsTecDebugPring() const { return (XnBool)m_FirmwareTecDebugPrint.GetValue(); }

    XnStatus SetFrameSyncStreamGroup(XnDeviceStream** ppStreamList, XnUInt32 numStreams);
    void SetDevicePID(XnUInt16 nPid);
    XnUInt16 GetDevicePID();

    XnStatus setObDepthOptimizationParam(const XnDepthOptimizationParam* depthOptimParam);
    XnStatus GetObDepthOptimizationParam(XnDepthOptimizationParam* depthOptimParam);

    //Depth optimization stats
    XnStatus SetDepthOptimizationState(XnBool bActive);
    XnStatus GetDepthOptimizationState(XnBool &bActive);

    //multi distance calibration
    XnStatus WriteFlashDistortionParam(const XnDistortionParam* pValue);
    XnStatus ReadFlashDistortionParam(XnDistortionParam* pValue);

    XnStatus SetObDistortionEnableState(XnUInt32 nActive);
    XnStatus GetObDistortionEnableState(XnUInt32 &nActive);


    XnStatus GetDistortionParam(XnDistortionParam &distortionParam);
    XnUInt32 GetMultiDisCalEnable(void);

    XnStatus SetObPdEnableState(XnUInt32 nActive);
    XnStatus GetObPdEnableState(XnUInt32 &nActive);

    XnStatus GetObPdAlertState(XnUInt32 &nActive);

    XnStatus SetObPdUpperTlv(XnUInt32 nActive);
    XnStatus GetObPdUpperTlv(XnUInt32 &nActive);

    XnStatus SetObPdLowerTlv(XnUInt32 nActive);
    XnStatus GetObPdLowerTlv(XnUInt32 &nActive);

    XnStatus SetObBootLoaderPtsState(XnBool nActive);
    XnStatus GetObBootLoaderPtsState(XnBool &nActive);

    const void* GetJavaVm() { return m_pJavaVM; }

protected:
    virtual XnStatus CreateStreamImpl(const XnChar* strType, const XnChar* strName, const XnActualPropertiesHash* pInitialSet);

    XnStatus CreateDeviceModule(XnDeviceModuleHolder** ppModuleHolder);
    XnStatus CreateStreamModule(const XnChar* StreamType, const XnChar* StreamName, XnDeviceModuleHolder** ppStream);
    void DestroyStreamModule(XnDeviceModuleHolder* pStreamHolder);

    virtual void OnNewStreamData(XnDeviceStream* pStream, OniFrame* pFrame);

private:
    XnStatus GetI2CDualCameraParam(ObContent_t &obContent);
    XnStatus InitSensor(const XnDeviceConfig* pDeviceConfig);
    XnStatus ValidateSensorID(XnChar* csSensorID);
    XnStatus SetMirrorForModule(XnDeviceModule* pModule, XnUInt64 nValue);
    XnStatus FindSensorStream(const XnChar* StreamName, IXnSensorStream** ppStream);
    XnStatus InitReading();
    XnStatus OnFrameSyncPropertyChanged();

    static XnStatus XN_CALLBACK_TYPE GetInstanceCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    XnStatus ChangeTaskInterval(XnScheduledTask** ppTask, XnTaskCallbackFuncPtr pCallback, XnUInt32 nInterval);
    void ReadFirmwareLog();
    void ReadFirmwareCPU();

    //---------------------------------------------------------------------------
    // Getters
    //---------------------------------------------------------------------------
    XnStatus GetFirmwareParam(XnInnerParamData* pParam);
    XnStatus GetCmosBlankingUnits(XnCmosBlankingUnits* pBlanking);
    XnStatus GetCmosBlankingTime(XnCmosBlankingTime* pBlanking);
    XnStatus GetFirmwareMode(XnParamCurrentMode* pnMode);
    XnStatus GetLastRawFrame(const XnChar* strStream, XnUChar* pBuffer, XnUInt32 nDataSize);
    XnStatus GetFixedParams(XnDynamicSizeBuffer* pBuffer);
    XnStatus GetDepthCmosRegister(XnControlProcessingData* pRegister);
    XnStatus GetImageCmosRegister(XnControlProcessingData* pRegister);
    XnStatus ReadAHB(XnAHBData* pData);
    XnStatus GetI2C(XnI2CReadData* pI2CReadData);
    XnStatus GetTecStatus(XnTecData* pTecData);
    XnStatus GetTecFastConvergenceStatus(XnTecFastConvergenceData* pTecData);
    XnStatus GetEmitterStatus(XnEmitterData* pEmitterData);
    XnStatus ReadFlashFile(const XnParamFileData* pFile);
    XnStatus ReadFlashChunk(XnParamFlashData* pFlash);
    XnStatus GetFirmwareLog(XnChar* csLog, XnUInt32 nSize);
    XnStatus GetFileList(XnFlashFileList* pFileList);

    //---------------------------------------------------------------------------
    // Setters
    //---------------------------------------------------------------------------
    XnStatus SetInterface(XnSensorUsbInterface nInterface);
    XnStatus SetHostTimestamps(XnBool bHostTimestamps);
    XnStatus SetNumberOfBuffers(XnUInt32 nCount);
    XnStatus SetReadData(XnBool bRead);
    XnStatus SetFirmwareParam(const XnInnerParamData* pParam);
    XnStatus SetCmosBlankingUnits(const XnCmosBlankingUnits* pBlanking);
    XnStatus SetCmosBlankingTime(const XnCmosBlankingTime* pBlanking);
    XnStatus Reset(XnParamResetType nType);
    XnStatus SetFirmwareMode(XnParamCurrentMode nMode);
    XnStatus SetDepthCmosRegister(const XnControlProcessingData* pRegister);
    XnStatus SetImageCmosRegister(const XnControlProcessingData* pRegister);
    XnStatus WriteAHB(const XnAHBData* pData);
    XnStatus SetLedState(XnUInt16 nLedId, XnUInt16 nState);
    XnStatus SetEmitterState(XnBool bActive);
    XnStatus GetEmitterState(XnBool* pState);

    //ir flood switch
    XnStatus GetIrfloodState(XnUInt32* bActive);
    XnStatus SetIrfloodState(const XnUInt32& bActive);

    //ir flood level
    XnStatus GetIrfloodLevel(XnUInt32* nLevel);
    XnStatus SetIrfloodLevel(const XnUInt32& nLevel);

    //irgain
    XnStatus SetIrGain(XnUInt32 nIrGain);
    XnStatus getIrGain(XnUInt32 &nIrGain);
    XnStatus SetIrExp(XnUInt32 nIrExp);
    XnStatus getIrExp(XnUInt32 &nIrExp);

    //ldp
    XnStatus SetLdpEnable(XnBool bActive);
    XnStatus getLdpEnable(XnBool &bActive);

    XnStatus GetEmitterEnable(XnBool &bActive);
    XnStatus SetLdpScale(XnUInt32 nScale);
    XnStatus GetLdpScale(XnUInt32 &nScale);

    XnStatus GetLdpStatus(XnBool &bActive);
    XnStatus GetLdpThresUp(XnUInt32 &value);
    XnStatus SetLdpThresUp(XnUInt32 value);
    XnStatus GetLdpThresLow(XnUInt32 &value);
    XnStatus SetLdpThresLow(XnUInt32 value);
    XnStatus GetLdpNoiseValue(XnUInt32 &value);

    XnStatus SetAeStatus(XnBool bActive);
    XnStatus GetAeStatus(XnBool &bActive);
	XnStatus SetHdrModeStatus(XnBool bActive);
	XnStatus GetHdrModeStatus(XnBool &bActive);
    //u1
    XnStatus SetMipiTestStatus(XnBool bActive);
    XnStatus GetMipiTestStatus(XnBool &bActive);


    //ado change sensor
    XnStatus SetAdoChangeSensor(XnBool bActive);

    XnStatus SetPublicKey(const OBEccVerify* pPublicKey);
    XnStatus GetPublicKey(OBEccVerify* pPublicKey);

    XnStatus SetRSKey(const OBEccRSKey* pRSKey);
    XnStatus GetRandomString(OBEccInit* pRandomString);

    //laser secure
    XnStatus IsSupportLaserSecure(XnBool &bIsSupport);
    XnStatus SetLaserSecureStatus(XnBool bStatus);
    XnStatus GetLaserSecureStatus(XnBool &bStatus);

    //laser current
    XnStatus SetLaserCurrent(XnUInt32 nLaserCurrent);
    XnStatus GetLaserCurrent(XnUInt32 &nLaserCurrent);

    //soft reset
    XnStatus SetSoftReset();

    //switch dual camera left and right ir
    XnStatus SetSwitchIr(XnBool bActive);

    //rgb ae mode
    XnStatus SetRgbAeMode(const XnRgbAeMode* pRgbAeMode);
    XnStatus GetRgbAeMode(XnRgbAeMode* pRgbAeMode);

    //IR Temperature
    XnStatus SetCalIrTemperature(XnDouble nCalIrTemp);
    XnStatus getCalIrTemperature(XnDouble &nCalIrTemp);

    //LDMP Temperature
    XnStatus SetCalLdmpTemperature(XnDouble nCalLdmpTemp);
    XnStatus getCalLdmpTemperature(XnDouble &CalLdmpTemp);

    //get Ir real time temperature
    XnStatus getRtIrTemperature(XnDouble &RtIrTemp);
    //get Ldmp real time temperature
    XnStatus getRtLdmpTemperature(XnDouble &RtLdmpTemp);

    //set ir temperature compensation coefficient
    XnStatus SetIrTemperatureCo(XnDouble nTempCo);
    //get ir temperature compensation coefficient
    XnStatus getIrTemperatureCo(XnDouble &nTempco);
    //set ldmp temperature compensation coefficient
    XnStatus SetLdmpTemperatureCo(XnDouble nTempco);
    //get ldmp temperature compensation coefficient
    XnStatus getLdmpTemperatureCo(XnDouble &nTempco);

    //Temperature comp
    XnStatus SetTemperatureCompState(XnBool bActive);
    XnStatus GetTemperatureCompState(XnBool &bActive);

    XnStatus GetCoreBroadFlashId(XnUInt32 &nId);

    //laser time
    XnStatus SetLaserTime(XnUInt32 nLaserTime);
    XnStatus GetLaserTime(XnUInt32 &nLaserTime);

    XnStatus SetPostFilterThreshold(XnUInt32 nPostFilterThreshold);
    XnStatus GetPostFilterThreshold(XnUInt32 &nPostFilterThreshold);

    XnStatus SetFirmwareFrameSync(XnBool bOn);
    XnStatus SetI2C(const XnI2CWriteData* pI2CWriteData);
    XnStatus SetFirmwareLogFilter(XnUInt32 nFilter);
    XnStatus SetFirmwareLogInterval(XnUInt32 nMilliSeconds);
    XnStatus SetFirmwareLogPrint(XnBool bPrint);
    XnStatus SetFirmwareCPUInterval(XnUInt32 nMilliSeconds);
    XnStatus SetAPCEnabled(XnBool bEnabled);
    XnStatus DeleteFile(XnUInt16 nFileID);
    XnStatus SetTecSetPoint(XnUInt16 nSetPoint);
    XnStatus SetEmitterSetPoint(XnUInt16 nSetPoint);
    XnStatus SetFileAttributes(const XnFileAttributes* pAttributes);
    XnStatus WriteFlashFile(const XnParamFileData* pFile);
    XnStatus SetProjectorFault(XnProjectorFaultData* pProjectorFaultData);
    XnStatus RunBIST(XnUInt32 nTestsMask, XnUInt32* pnFailures);
    XnStatus SetReadAllEndpoints(XnBool bEnabled);
    XnStatus SetFirmwareQN(const OBFirmwareQN* qN);
    XnStatus GetFirmwareQN(OBFirmwareQN* qN);
    XnStatus VerifyQN(const OBFirmwareQN* qN);
    XnStatus GetFirmwareVersion(OBFirmwareQN* pFriemwareVersion);

    XnStatus SetD2CResolution(XnUInt16 nResolution);
    XnStatus GetD2CResolution(XnUInt16 &nResolution);

    XnStatus GetUsbDeviceSpeed(XnUInt16 &nSpeed);
    XnStatus SetDeviceSN(const OBSerialNumber* sN);
    XnStatus GetDeviceSN(OBSerialNumber* sN);

    XnStatus SetDevicePN(const OBKTProductNumber* pN);
    XnStatus GetDevicePN(OBKTProductNumber* pN);
    XnStatus GetDeviceCfgPN(XnChar* cfgPn);
    XnStatus GetIRSensorModel(XnUInt32 &irModel);
    XnStatus GetRgbSensorModel(XnUInt32 &rgbModel);

    XnStatus GetPdCurrentThreshold(OBPdThreshold* pd);

    //TOF
    XnStatus SetTofSensorEnableState(XnUInt32 nValue);
    XnStatus GetTofSensorEnableState(XnUInt32 &nValue);
    XnStatus GetTofSensorMeasureResult(XnUInt32 &nValue);
    XnStatus GetTofSensorAppId(XnUInt32 &nValue);
    XnStatus SetTofSensorCalibrationValue(XnUInt32 nValue);
    XnStatus SetTofSensorAppEnableState(XnUInt32 nValue);
    XnStatus GetTofSensorCalibrationParams(OBTofSensorCalParams* tofCalParams);
    XnStatus SetTofSensorCalibrationParams(const OBTofSensorCalParams* tofCalParams);

    // Motor
    XnStatus SetMotorTest(XnUInt32 nValue);
    XnStatus GetMotorTestResult(XnUInt32 &nValue);
    XnStatus SetMotorPosition(XnUInt32 nValue);
    XnStatus GetMotorPosition(XnUInt32 &nValue);
    XnStatus GetMotorStatus(XnUInt32 &nValue);
	XnStatus GetMotorTestCount(XnUInt32 &nValue);
	XnStatus SetMotorRunTime(XnUInt32 nValue);
	XnStatus GetMotorRunTime(XnUInt32 &nValue);
    XnStatus GetMotorFeature(XnUInt32 &nValue);
    XnStatus GetMotorUpdownState(XnUInt32 &nValue);
    XnStatus GetMotorUpdownTime(XnUInt32 &nValue);
    XnStatus SetMotorUpdown(XnUInt32 nValue);

    XnStatus SetDepthIrMode(XnUInt32 nMode);
    XnStatus getDepthIrMode(XnUInt32 &nMode);

    XnStatus SetSubTractBGMode(XnUInt32 nMode);
    XnStatus GetSubTractBGMode(XnUInt32 &nMode);

    XnStatus SetTecEnable(XnBool bActive);

    XnStatus SetAEOptions(AeParamsStruct* pValue, XnUInt32 nType);
    XnStatus GetAEOptions(AeParamsStruct* pValue, XnUInt32 nType);

    XnStatus GetPlatformString(XnChar* csPlatform);

    XnStatus GetTOFFreqMode(XnUInt16* pMode);
    XnStatus SetTOFFreqMode(const XnUInt16 mode);

	XnStatus GetTOFSensorFilterLevel(XnUInt16* filterLevel);
	XnStatus SetTOFSensorFilterLevel(const XnUInt16 filterLevel);
	XnStatus GetTOFSensorIntegrationTime(XnUInt32* integrationTime);
	XnStatus SetTOFSensorIntegrationTime(const XnUInt32 integrationTime);
	XnStatus GetTOFSensorGain(XnUInt16* gain);
	XnStatus SetTOFSensorGain(const XnUInt16 gain);
	XnStatus GetTOFSensorLaserInterference(XnUInt16* laserInterference);
	XnStatus SetTOFSensorLaserInterference(const XnUInt16 laserInterference);
	XnStatus GetTOFSensorWorkingMode(XnUInt16* workingMode);
	XnStatus SetTOFSensorWorkingMode(const XnUInt16 workingMode);
    XnStatus SendCommand(OniSerialCmd* pCmd);
    XnStatus QueryDeviceTimestamp(XnUInt64* pTimestamp);

    XnStatus GetPlatformVersion(XnChar* pValue);
    XnStatus GetPlatformSDKVersion(XnChar* pValue);

    XnStatus GetSensorID(OniSensorIDMap* pSensorID);
    XnStatus GetGeneralSerialNumber(OniSerialNumberMap* pSerialMap);
    XnStatus SetGeneralSerialNumber(const OniSerialNumberMap* pSerialMap);

	XnStatus SetGeneralFrequency(const ORBTofFrequency* pData);
	XnStatus GetGeneralFrequency(ORBTofFrequency* pData);
	XnStatus SetGeneralDutyCycle(const ORBTofDuty* pData);
	XnStatus GetGeneralDutyCycle(ORBTofDuty* pData);
	XnStatus SetGeneralDriverICReg(const ObReg8Map* pData);
	XnStatus GetGeneralDriverICReg(ObReg8Map* pData);
	XnStatus SetGeneralSensorReg(const ObReg16Map* pData);
	XnStatus GetGeneralSensorReg(ObReg16Map* pData);
    XnStatus SendUsbFile(const XnUsbGeneralFile* pUsbFile);
    XnStatus SendUsbFile(const OniFileAttributes * pFileAttributes);
    XnStatus StartService(const OniService* pService);
    XnStatus SetJavaVM(const void* pJavaVM);

    //---------------------------------------------------------------------------
    // Callbacks
    //---------------------------------------------------------------------------
    static void XN_CALLBACK_TYPE OnDeviceDisconnected(const OniDeviceInfo& deviceInfo, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetInterfaceCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetHostTimestampsCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetNumberOfBuffersCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetReadDataCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareParamCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetCmosBlankingUnitsCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetCmosBlankingTimeCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE ResetCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareModeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFixedParamsCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE FrameSyncPropertyChangedCallback(const XnProperty* pSender, void* pCookie);
    static XnBool XN_CALLBACK_TYPE HasSynchedFrameArrived(void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFirmwareParamCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetCmosBlankingUnitsCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetCmosBlankingTimeCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFirmwareModeCallback(const XnIntProperty* pSender, XnUInt64* pnValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetAudioSupportedCallback(const XnIntProperty* pSender, XnUInt64* pnValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetImageSupportedCallback(const XnIntProperty* pSender, XnUInt64* pnValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPhaseSupportedCallback(const XnIntProperty* pSender, XnUInt64* pnValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetBodySupportedCallback(const XnIntProperty* pSender, XnUInt64* pnValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetDepthCmosRegisterCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetImageCmosRegisterCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDepthCmosRegisterCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetImageCmosRegisterCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE ReadAHBCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE WriteAHBCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetLedStateCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetEmitterStateCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetEmitterStateCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie);

    //ir flood
    static XnStatus XN_CALLBACK_TYPE SetIrfloodStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIrfloodStateCallback(const XnIntProperty* pSender, XnUInt64* nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetIrfloodLevelCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIrfloodLevelCallback(const XnIntProperty* pSender, XnUInt64* nValue, void* pCookie);

    //irgain
    static XnStatus XN_CALLBACK_TYPE SetIrGainCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIrGainCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //ir exp
    static XnStatus XN_CALLBACK_TYPE SetIrExpCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIrExpCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //ldp
    static XnStatus XN_CALLBACK_TYPE SetLdpEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdpEnableCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //
    static XnStatus XN_CALLBACK_TYPE SetLdpScaleCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdpScaleCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetLdpStatusCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdpThresUpCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetLdpThresUpCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdpThresLowCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetLdpThresLowCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdpNoiseValueCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetEmitterEnableCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //Auto ae
    static XnStatus XN_CALLBACK_TYPE SetAeEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetAeStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetHdrModeEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetHdrModeEnableCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    //u1
    static XnStatus XN_CALLBACK_TYPE SetMipiTestEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMipiTestStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetI2CReadFlashCallback(const XnGeneralProperty*  pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //ado project change sensor
    static XnStatus XN_CALLBACK_TYPE SetAdoChangeSensorCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    //public key
    static XnStatus XN_CALLBACK_TYPE SetPublicKeyCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPublicKeyCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetRandomStringCallback(const XnGeneralProperty*pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetRSKeyCallback(XnGeneralProperty*pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //laser secure
    static XnStatus XN_CALLBACK_TYPE IsSupportLaserSecureCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetLaserSecureStatusCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLaserSecureStatusCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //laser current
    static XnStatus XN_CALLBACK_TYPE SetLaserCurrentCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLaserCurrentCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //soft reset
    static XnStatus XN_CALLBACK_TYPE SetSoftResetCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    //switch dual camera left and right ir
    static XnStatus XN_CALLBACK_TYPE SetSwitchIrCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    //rgb ae mode
    static XnStatus XN_CALLBACK_TYPE SetRgbAeModeCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetRgbAeModeCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //Cal ir  temperture
    static XnStatus XN_CALLBACK_TYPE SetCalIrTempertureCallback(XnRealProperty* pSender, XnDouble nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetCalIrTempertureCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    //Cal ldmp temperture
    static XnStatus XN_CALLBACK_TYPE SetCalLdmpTempertureCallback(XnRealProperty* pSender, XnDouble nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetCalLdmpTempertureCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    //IR real time temperature
    static XnStatus XN_CALLBACK_TYPE GetRtIrTempertureCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);
    //Ldmp real timne temperature
    static XnStatus XN_CALLBACK_TYPE GetRtLdmpTempertureCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    //IR temperature compensation coefficient
    static XnStatus XN_CALLBACK_TYPE SetIrTempertureCoCallback(XnRealProperty* pSender, XnDouble nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIrTempertureCoCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    //Ldmp temperature compensation coefficient
    static XnStatus XN_CALLBACK_TYPE SetLdmpTempertureCoCallback(XnRealProperty* pSender, XnDouble nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLdmpTempertureCoCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetTemperatureCompStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTemperatureCompStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetDepthOptimizationStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDepthOptimizationStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    //
    static XnStatus XN_CALLBACK_TYPE setObDepthOptimizationParamCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetObDepthOptimizationParamCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetCoreBroadFlashIdCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetLaserTimeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetLaserTimeCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetPostFilterThresholdCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPostFilterThresholdCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetZppsCallback(const XnRealProperty* pSender, XnDouble *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetTofSensorEnableStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTofSensorEnableStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTofSensorMeasureResultCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTofSensorAppIdCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetTofSensorcalibrationCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetTofSensorAppEnableStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetTofSensorcalibrationParamsCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTofSensorcalibrationParamsCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //Motor
    static XnStatus XN_CALLBACK_TYPE SetMotorTestCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorTestResultCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetMotorPositionCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorPositionCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorStatusCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorTestCountCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetMotorRunTimeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetMotorRunTimeCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorFeatureCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorUpdownStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetMotorUpdownTimeCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetMotorUpdownCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetDepthIrModeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDepthIrModeCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetSetTecEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetSubtractBGCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetSubtractBGCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareFrameSyncCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareLogFilterCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareLogIntervalCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareLogPrintCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFirmwareCPUIntervalCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetReadAllEndpointsCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetAPCEnabledCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetI2CCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE DeleteFileCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetTecSetPointCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetEmitterSetPointCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFileAttributesCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE WriteFlashFileCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetProjectorFaultCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE RunBISTCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFileListCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static void XN_CALLBACK_TYPE ExecuteFirmwareLogTask(void* pCookie);
    static void XN_CALLBACK_TYPE ExecuteFirmwareCPUTask(void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetI2CCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTecStatusCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTecFastConvergenceStatusCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetEmitterStatusCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE ReadFlashFileCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFirmwareLogCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE ReadFlashChunkCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE ReadFlashDistortionParamCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE WriteFlashDistortionParamCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //read/write QN callback
    static XnStatus XN_CALLBACK_TYPE SetFirmwareQNCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFirmwareQNCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    //verify QN callback
    static XnStatus XN_CALLBACK_TYPE VerifyQNCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetPublicBoardVersionCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetMX6300VersionCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetD2CResolutionCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetD2CResolutionCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetUsbDeviceSpeedCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetDeviceSerialNumberCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDeviceSerialNumberCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetDevicePNCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDevicePNCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetZ0BaselineCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetPdEnableCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPdEnableCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetPdAlertCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPdUpperTlvCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetPdUpperTlvCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPdLowerTlvCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetPdLowerTlvCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetDevicePdCurrenthresholdCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetBootLoaderPtsCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetBootLoaderPtsCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetDeviceCfgPNCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetIRSensorModelCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetRgbSensorModelCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetCupVerifyVersionCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetFloodAEOptionsCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFloodAEOptionsCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetEmitterAEOptionsCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetEmitterAEOptionsCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetDepthDistortionStateCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetDepthDistortionStateCallback(const XnIntProperty* pSender, XnUInt64 *nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPlatformStringCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetTOFFreqModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetTOFFreqModeCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie);

	static XnStatus XN_CALLBACK_TYPE SetTOFSensorFilterLevelCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorFilterLevelCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorIntegrationTimeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorIntegrationTimeCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorGainCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorGainCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorLaserInterferenceCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorLaserInterferenceCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorWorkingModeCallback(XnIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorWorkingModeCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorFrequencyCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorFrequencyCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorDutyCycleCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorDutyCycleCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorDriverICRegCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorDriverICRegCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetTOFSensorSensorRegCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetTOFSensorSensorRegCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SendUsbFileCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE StartServiceCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE GetPlatformVersionCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetPlatformSDKVersionCallback(const XnActualStringProperty* pSender, XnChar* csValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetSensorIDCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetGeneralSerialNumberCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetGeneralSerialNumberCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

    static XnStatus XN_CALLBACK_TYPE SetJavaVMCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SendCommandCallback(XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE QueryDeviceTimestampCallback(const XnIntProperty* pSender, XnUInt64* pValue, void* pCookie);

    //---------------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------------
    XnCallbackHandle m_hDisconnectedCallback;
    XnActualIntProperty m_ErrorState;
    XnActualIntProperty m_ResetSensorOnStartup;
    XnActualIntProperty m_LeanInit;
    XnActualIntProperty m_Interface;
    XnActualIntProperty m_ReadData;
    XnActualIntProperty m_FrameSync;
    XnActualIntProperty m_FirmwareFrameSync;
    XnActualIntProperty m_CloseStreamsOnShutdown;
    XnActualIntProperty m_HostTimestamps;
    XnGeneralProperty m_FirmwareParam;
    XnGeneralProperty m_CmosBlankingUnits;
    XnGeneralProperty m_CmosBlankingTime;
    XnIntProperty m_Reset;

    XnVersions m_VersionData;
    XnDevicePrivateData m_DevicePrivateData;

    XnActualGeneralProperty m_Version;
    XnGeneralProperty m_FixedParam;

    XnActualStringProperty m_ID;
    XnActualStringProperty m_DeviceName;
    XnActualStringProperty m_VendorSpecificData;
    XnActualStringProperty m_PlatformString;

    XnGeneralProperty m_service;
    XnGeneralProperty m_sendCmd;
    XnGeneralProperty m_sensorID;
    XnGeneralProperty m_sendUsbFile;
    XnGeneralProperty m_serialNumber;
    XnActualIntProperty m_freqMode;
    XnIntProperty m_deviceTimestamp;
    XnActualStringProperty m_platformVerion;
    XnActualStringProperty m_platformSDKVerion;

    XnIntProperty m_AudioSupported;
    XnIntProperty m_ImageSupported;
    XnIntProperty m_PhaseSupported;
    XnIntProperty m_AISupported;
    XnGeneralProperty m_ImageControl;
    XnGeneralProperty m_DepthControl;
    XnGeneralProperty m_AHB;
    XnGeneralProperty m_LedState;
    XnActualIntProperty m_EmitterEnabled;
    //get emitter enable
    XnIntProperty m_EmitterEnabled_V1;

    //ir flood
    XnIntProperty m_IrfloodEnabled;
    XnIntProperty m_IrfloodLevel;

    //irGain
    XnIntProperty m_IrGain;
    XnIntProperty m_IrExp;
    //ldp enable
    XnIntProperty m_LdpEnable;
    XnIntProperty m_LdpScale;
    XnIntProperty m_LdpStatus;      //Ldp status
    XnIntProperty m_LdpThresUp;     //Ldp thres up
    XnIntProperty m_LdpThresLow;    //Ldp thres low
    XnIntProperty m_LdpNoise;       //Ldp noise

    XnIntProperty m_AeEnabled; //AE enabled
	XnIntProperty m_HdrModeEnabled; //Hdr mode
    XnIntProperty m_MipiTestEnabled;
    //use i2c read flash ,only use in mipi
    XnGeneralProperty m_I2CReadFlash;
    //ado change sensor
    XnIntProperty m_ChangeSensor;
    //public key and batch version
    XnGeneralProperty m_PublicKey;

    //random init string
    XnGeneralProperty m_RandomString;
    XnGeneralProperty m_RSKey;

    //laser secure
    XnIntProperty	m_IsSupportLaserSecure;
    XnIntProperty	m_LaserSecureStatus;

    //laser current
    XnIntProperty m_LaserCurrent;

    //soft reset
    XnIntProperty m_SoftReset;

    //switch dual camera ir
    XnIntProperty m_SwitchIr;

    //rgb ae mode
    XnGeneralProperty m_RgbAeMode;

    //Cal ir temperature
    XnRealProperty m_CalIrTemperature;
    //Cal Ldmp temperature
    XnRealProperty m_CalLdmpTemperature;

    //IR real time temperature
    XnRealProperty m_RtIrTemperature;
    //Ldmp real timne temperature
    XnRealProperty m_RtLdmpTemperature;

    //IR temperature compensation coefficient
    XnRealProperty m_IrTEMPCO;
    //Ldmp temperature compensation coefficient
    XnRealProperty m_LdmpTEMPCO;

    //Temperature comp
    XnIntProperty m_TemperatureCompEnabled;

    //Depth Optimization enable
    XnIntProperty m_DepthOptimizationEnabled;

    //Set Depth Optimization param
    XnGeneralProperty m_ObDepthOptimParam;

    //Distortion Param
    XnGeneralProperty m_FlashDistortionParam;


    XnIntProperty ObDistortionEnabled;

    XnActualIntProperty m_FirmwareLogFilter;
    XnActualIntProperty m_FirmwareLogInterval;
    XnActualIntProperty m_FirmwareLogPrint;
    XnActualIntProperty m_FirmwareCPUInterval;
    XnActualIntProperty m_APCEnabled;
    XnActualIntProperty m_FirmwareTecDebugPrint;
    XnActualIntProperty m_ReadAllEndpoints;
    XnGeneralProperty m_I2C;
    XnIntProperty m_DeleteFile;
    XnIntProperty m_TecSetPoint;
    XnGeneralProperty m_TecStatus;
    XnGeneralProperty m_TecFastConvergenceStatus;
    XnIntProperty m_EmitterSetPoint;
    XnGeneralProperty m_EmitterStatus;
    XnGeneralProperty m_FileAttributes;
    XnGeneralProperty m_FlashFile;
    XnGeneralProperty m_FirmwareLog;
    XnGeneralProperty m_FlashChunk;
    XnGeneralProperty m_FileList;
    XnGeneralProperty m_BIST;
    XnGeneralProperty m_ProjectorFault;
    //Device QN property
    XnGeneralProperty m_deviceQN;
    //Set QN verify
    XnGeneralProperty m_verifyQN;
    XnGeneralProperty m_PublicBoardVersion;
    XnGeneralProperty m_CupVerifyVersion; //
    XnGeneralProperty m_VersionMX6300;
    XnIntProperty m_D2CResolution;
    XnIntProperty m_UsbDeviceSpeed;
    XnGeneralProperty m_deviceSN;
    XnGeneralProperty m_devicePN;
    XnGeneralProperty m_Z0Baselie;

    XnActualStringProperty m_cfgPN;
    XnIntProperty m_irSensorModel;
    XnIntProperty m_rgbSensorModel;
    XnGeneralProperty m_floodAEOptions;
    XnGeneralProperty m_emitterAEOptions;
    //mipi project core broad flash id
    XnIntProperty m_CoreBroadFlashId;

    //PD enable
    XnIntProperty m_PdEnable;
    XnIntProperty m_PdAlertStatus;
    XnIntProperty m_PdUpperTlv;
    XnIntProperty m_PdLowerTlv;
    XnGeneralProperty m_PdCurTlv;

    XnIntProperty m_BootLoaderPts;

    //laser time
    XnIntProperty m_LaserTime;
    XnIntProperty m_PostFilterThreshold;
    XnRealProperty m_Zpps;

    //TOF sensor
    XnIntProperty m_TofSensorEnabled;
    XnIntProperty m_TofSensorMEAResult;
    XnIntProperty m_TofSensorAppId;
    XnIntProperty m_TofSensorCal;
    XnIntProperty m_TofSensorAppEnable;

    // Motor
    XnIntProperty m_MotorTest;
    XnIntProperty m_MotorPosition;
    XnIntProperty m_MotorStaus;
    XnIntProperty m_MotorTestCount;
	XnIntProperty m_MotorRunTime;
    XnIntProperty m_MotorFeature;
    XnIntProperty m_MotorUpdownState;
    XnIntProperty m_MotorUpdownTime;
    XnIntProperty m_MotorUpdownControl;

    XnGeneralProperty m_TofSensorCalParams;
    XnIntProperty m_DepthIrMode;
    XnIntProperty m_TecEnable;
    XnIntProperty m_EnableSubtractBG;
    XnIntProperty m_StateSubtractBG;

	XnIntProperty m_FilterLevel; //Hdr mode
	XnIntProperty m_TofSensorIntegrationTime;
	XnIntProperty m_TofSensorGain;
	XnIntProperty m_TofSensorLaserInterference;
	XnIntProperty m_TofSensorWorkingMode;
	XnGeneralProperty m_TofSensorFrequency;
	XnGeneralProperty m_TofSensorDutycycle;
	XnGeneralProperty m_TofSensorDriverIcReg;
	XnGeneralProperty m_TofSensorSensorReg;
    XnGeneralProperty m_javaVM;

    XnSensorFirmware m_Firmware;
    XnSensorFPS m_FPS;
    XnCmosInfo m_CmosInfo;
    XnSensorIO m_SensorIO;

    XnSensorObjects m_Objects;

    /** A scheduler to be used for performing periodic tasks. */
    XnScheduler* m_pScheduler;
    XnScheduledTask* m_pLogTask;
    XnScheduledTask* m_pCPUTask;
    XnDumpFile* m_FirmwareLogDump;
    XnDumpFile* m_FrameSyncDump;
    XnBool m_nFrameSyncEnabled;
    typedef struct
    {
        XnDeviceStream* pStream;
        OniFrame* pFrame;
    } FrameSyncedStream;
    xnl::Array<FrameSyncedStream> m_FrameSyncedStreams;
    int m_nFrameSyncLastFrameID;
    xnl::CriticalSection m_frameSyncCs;

    XnBool m_bInitialized;

    XnIntPropertySynchronizer m_PropSynchronizer;

    XnChar m_strGlobalConfigFile[XN_FILE_MAX_PATH];
    XnUInt16 m_nPId;

    //multi distance cal
    XnDistortionParam m_distortionParam;
    XnBool m_bReadDistortParam;
    XnUInt32 m_bMultiDisCalEnable;

    const void* m_pJavaVM;

public:
    OniStatus GetDualCameraParam(ObContent_t &obContent);
    XnStatus GetCameraParam(OBCameraParams &obCameraParams);


private:
    int ReadFlash(XnUInt32 offset, XnUInt32 size, XnUInt8 *data_ptr);
    XnStatus SendCmd(uint16_t cmd, void *cmdbuf, uint16_t cmd_len, void *replybuf, uint16_t reply_len);
    XnBool m_bReadCameraParam;
    ObContent_t m_ObContent;
    OBCameraParams m_ObCameraParams;
    XnStatus ReadFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size);
    XnStatus I2CReadFlash(XnParamFlashData* pValue);
};

#endif // XNSENSOR_H
