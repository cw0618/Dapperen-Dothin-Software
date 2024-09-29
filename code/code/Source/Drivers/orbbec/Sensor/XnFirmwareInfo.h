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
#ifndef XNFIRMWAREINFO_H
#define XNFIRMWAREINFO_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnStreamParams.h>
#include <XnArray.h>

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
class XnFirmwareInfo
{
public:
    XnFWVer nFWVer;
    XnUInt16 nHostMagic;
    XnUInt16 nFWMagic;
    XnUInt16 nProtocolHeaderSize;
    XnUInt16 nProtocolHeaderBulkSize;
    XnUInt16 nProtocolMaxPacketSize;

    XnParamCurrentMode nCurrMode;

    XnBool bAudioSupported;
    XnBool bPhaseSupported;
    XnBool bAISupported;
    XnBool bGetPresetsSupported;
    XnBool bDeviceInfoSupported;
    XnBool bImageAdjustmentsSupported;

    XnUInt16 nOpcodeGetVersion;
    XnUInt16 nOpcodeKeepAlive;
    XnUInt16 nOpcodeGetParam;
    XnUInt16 nOpcodeSetParam;
    XnUInt16 nOpcodeGetFixedParams;
    XnUInt16 nOpcodeGetMode;
    XnUInt16 nOpcodeSetMode;
    XnUInt16 nOpcodeAlgorithmParams;
    XnUInt16 nOpcodeReset;
    XnUInt16 nOpcodeSetCmosBlanking;
    XnUInt16 nOpcodeGetCmosBlanking;
    XnUInt16 nOpcodeGetCmosPresets;
    XnUInt16 nOpcodeGetSerialNumber;
    XnUInt16 nOpcodeSetSerialNumber;
    XnUInt16 nOpcodeGetCfgProductNumber;
    XnUInt16 nOpcodeGetFastConvergenceTEC;
    XnUInt16 nOpcodeGetCMOSReg;
    XnUInt16 nOpcodeSetCMOSReg;
    XnUInt16 nOpcodeWriteI2C;
    XnUInt16 nOpcodeReadI2C;
    XnUInt16 nOpcodeReadAHB;
    XnUInt16 nOpcodeWriteAHB;
    XnUInt16 nOpcodeGetPlatformString;
    XnUInt16 nOpcodeGetUsbCore;
    XnUInt16 nOpcodeSetLedState;
    XnUInt16 nOpcodeEnableEmitter;
    XnUInt16 nOpcodeEnableIrflood;

    //irgain
    XnUInt16 nOpcodeSetIrGain;
    //get irgain
    XnUInt16 nOpcodeGetIrGain;

    //ldp enable
    XnUInt16 nOpcodeSetLdpEnable;
    XnUInt16 nOpcodeGetLdpEnable;
    XnUInt16 nOpcodeGetEmitterEnable;

    //Auto ae
    XnUInt16 nOpcodeSetAeEnable;
    XnUInt16 nOpcodeGetAeEnable;
	XnUInt16 nOpcodeSetHdrModeEnable;
	XnUInt16 nOpcodeGetHdrModeEnable;
    //set u1 test mode
    XnUInt16 nOpcodeSetMipiTestEnable;
    XnUInt16 nOpcodeGetMipiTestEnable;
    XnUInt16 nOpcodeI2CReadFlash;

    XnUInt16 nOpcodeGetIRSensorModel;
    XnUInt16 nOpcodeGetRgbSensorModel;

    XnUInt16 nOpcodeSetLdpEnableV1;
    XnUInt16 nOpcodeGetLdpEnableV1;
    XnUInt16 nOpcodeGetEmitterEnableV1;

    //ado change sensor
    XnUInt16 nOpcodeChangeSensor;

    XnUInt16 nOpcodeSetIrExp;
    XnUInt16 nOpcodeGetIrExp;

    //set public key and batch version
    XnUInt16 nOpcodeSetPublicKey;
    XnUInt16 nOpcodeGetPublicKey;

    //get random init string
    XnUInt16 nOpcodeRandomString;
    XnUInt16 nOpcodeRSKey;

    //laser secure
    XnUInt16 nOpcodeIsSupportLaserSecure;
    XnUInt16 nOpcodeSetLaserSecure;
    XnUInt16 nOpcodeGetLaserSecure;

    //laser current
    XnUInt16 nOpcodeSetLaserCurrent;
    XnUInt16 nOpcodeGetLaserCurrent;

    //soft reset
    XnUInt16 nOpcodeSoftReset;
    //switch dual camera left and right ir
    XnUInt16 nOpcodeSetSwitchIr;   //0:left ir ;1:right ir

    //get DisparityCoeff
    XnUInt16 nOpcodeGetDisparityCoeff;

    //set rgb ae mode
    XnUInt16 nOpcodeSetRgbAeMode;
    XnUInt16 nOpcodeGetRgbAeMode;

    XnUInt16 nOpcodeReadSubCmdParams;
    XnUInt16 nOpcodeWriteSubCmdParams;

    XnUInt16 nOpcodeOptimReadSubCmdParams;
    XnUInt16 nOpcodeOptimWriteSubCmdParams;

    XnUInt16 nOpcodePdReadSubCmd;
    XnUInt16 nOpcodePdWriteSubCmd;

    XnUInt16 nOpcodeBootLoaderPtsRead;
    XnUInt16 nOpcodeBootLoaderPtsWrite;

    XnUInt16 nOpcodeToFSensorRead;
    XnUInt16 nOpcodeToFSensorWrite;

    XnUInt16 nOpcodeMotorRead;
    XnUInt16 nOpcodeMotorWrite;

    XnUInt16 nOpcodeSetD2CResolution;
    XnUInt16 nOpcodeGetD2CResolution;

    //laser timer
    XnUInt16 nOpcodeSetLaserTime;
    XnUInt16 nOpcodeGetLaserTime;

    //post filter threshold
    XnUInt16 nOpcodeSetPostFilterThreshold;
    XnUInt16 nOpcodeGetPostFilterThreshold;

    //set depthir mode
    XnUInt16 nOpcodeSetDepthIrMode;
    XnUInt16 nOpcodeGetDepthIrMode;

    //
    XnUInt16 nOpcodeSetTecEnable;

    XnUInt16 nOpcodeSetSubTractBGMode;
    XnUInt16 nOpcodeGetSubTractBGMode;

    XnUInt16 nOpcodeGetLog;
    XnUInt16 nOpcodeTakeSnapshot;
    XnUInt16 nOpcodeInitFileUpload;
    XnUInt16 nOpcodeWriteFileUpload;
    XnUInt16 nOpcodeFinishFileUpload;
    XnUInt16 nOpcodeDownloadFile;
    XnUInt16 nOpcodeDeleteFile;
    XnUInt16 nOpcodeGetFlashMap;
    XnUInt16 nOpcodeGetFileList;
    XnUInt16 nOpcodeSetFileAttribute;
    XnUInt16 nOpcodeExecuteFile;
    XnUInt16 nOpcodeReadFlash;
    XnUInt16 nOpcodeBIST;
    XnUInt16 nOpcodeSetGMCParams;
    XnUInt16 nOpcodeGetCPUStats;
    XnUInt16 nOpcodeCalibrateTec;
    XnUInt16 nOpcodeGetTecData;
    XnUInt16 nOpcodeCalibrateEmitter;
    XnUInt16 nOpcodeGetEmitterData;
    XnUInt16 nOpcodeCalibrateProjectorFault;

    XnUInt16 nOpcodeGetSensorID;
    XnUInt16 nOpcodeSendCommand;
    XnUInt16 nOpcodeFileTransfer;
    XnUInt16 nOpcodeFileTransferPrepare;
    XnUInt16 nOpcodeFileTransferFinish;
    XnUInt16 nOpcodeGetPlatformVersion;
    XnUInt16 nOpcodeGetPlatformSDKVersion;
    XnUInt16 nOpcodeGetTOFFreqMode;
    XnUInt16 nOpcodeSetTOFFreqMode;
	XnUInt16 nOpcodeGetTOFSensorFilterLevel;
	XnUInt16 nOpcodeSetTOFSensorFilterLevel;
	XnUInt32 nOpcodeGetTOFSensorIntegrationTime;
	XnUInt32 nOpcodeSetTOFSensorIntegrationTime;
	XnUInt16 nOpcodeGetTOFSensorGain;
	XnUInt16 nOpcodeSetTOFSensorGain;
	XnUInt16 nOpcodeGetTOFSensorLaserInterference;
	XnUInt16 nOpcodeSetTOFSensorLaserInterference;
	XnUInt16 nOpcodeSetTOFSensorWorkingMode;
	XnUInt16 nOpcodeGetTOFSensorWorkingMode;
	XnUInt16 nOpcodeSetTOFSensorFrequency;
	XnUInt16 nOpcodeGetTOFSensorFrequency;
	XnUInt16 nOpcodeSetTOFSensorDutyCycle;
	XnUInt16 nOpcodeGetTOFSensorDutyCycle;
	XnUInt16 nOpcodeSetTOFSensorDriverICReg;
	XnUInt16 nOpcodeGetTOFSensorDriverICReg;
	XnUInt16 nOpcodeSetTOFSensorSensorReg;
	XnUInt16 nOpcodeGetTOFSensorSensorReg;
    XnUInt16 nOpcodeStartService;
    XnUInt16 nOpcodeStreamSetQuery;
    XnUInt16 nOpcodeQueryTimestamp;

    XnUInt16 nLogStringType;
    XnUInt16 nLogOverflowType;

    XnBool bMirrorSupported;

    XnUInt16 nUSBDelayReceive;
    XnUInt16 nUSBDelayExecutePreSend;
    XnUInt16 nUSBDelayExecutePostSend;
    XnUInt16 nUSBDelaySoftReset;
    XnUInt16 nUSBDelaySetParamFlicker;
    XnUInt16 nUSBDelaySetParamStream0Mode;
    XnUInt16 nUSBDelaySetParamStream1Mode;
    XnUInt16 nUSBDelaySetParamStream2Mode;
    XnUInt16 nUSBDelaySetParamStream3Mode;

    XnUInt8 nISOAlternativeInterface;
    XnUInt8 nBulkAlternativeInterface;
    XnUInt8 nISOLowDepthAlternativeInterface;

    XnBool bGetImageCmosTypeSupported;
    XnBool bImageSupported;
    XnBool bIncreasedFpsCropSupported;
    XnBool bHasFilesystemLock;

    xnl::Array<XnCmosPreset> depthModes;
    xnl::Array<XnCmosPreset> _imageBulkModes;
    xnl::Array<XnCmosPreset> _imageIsoModes;
    xnl::Array<XnCmosPreset> imageModes;
    xnl::Array<XnCmosPreset> irModes;
    xnl::Array<XnCmosPreset> phaseModes;
    xnl::Array<XnCmosPreset> AIModes;
    xnl::Array<XnStreamSet> streamSets;
};

#endif // XNFIRMWAREINFO_H
