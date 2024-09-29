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
#ifndef XNONIDEVICE_H
#define XNONIDEVICE_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <Driver/OniDriverAPI.h>
#include <XnLib.h>
#include "XnOniDepthStream.h"
#include "XnOniColorStream.h"
#include "XnOniIRStream.h"
#include "XnOniPhaseStream.h"
#include "XnOniAIStream.h"
#include "../Sensor/XnSensor.h"

#include <OBExtensionCommand.h>

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

class XnOniStream;
class XnOniDriver;

class XnOniDevice : public oni::driver::DeviceBase
{
public:
    XnOniDevice(const char* uri, oni::driver::DriverServices& driverServices, XnOniDriver* pDriver);
    virtual ~XnOniDevice();

    XnStatus Init(const char* mode);

    OniDeviceInfo* GetInfo() { return &m_info; }

    OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors);

    oni::driver::StreamBase* createStream(OniSensorType sensorType);
    void destroyStream(oni::driver::StreamBase* pStream);

    virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize);
    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize);
    virtual OniBool isPropertySupported(int propertyId);
    virtual void notifyAllProperties();

    virtual OniStatus EnableFrameSync(XnOniStream** pStreams, int streamCount);
    virtual void DisableFrameSync();

    virtual OniBool isImageRegistrationModeSupported(OniImageRegistrationMode mode);

    XnSensor* GetSensor()
    {
        return &m_sensor;
    }

    XnOniDriver* GetDriver()
    {
        return m_pDriver;
    }

private:
    OniStatus OBExtension_GetProperty(int id, void* data, int dataSize);
    OniStatus OBExtension_SetProperty(int id, const void* data, int dataSize);
    OniStatus SendCmd(uint16_t cmd, void *cmdbuf, uint16_t cmd_len, void *replybuf, uint16_t reply_len);
    OniStatus ReadFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size);
    OniStatus WriteFlash(uint32_t offset, uint16_t dw_size, uint8_t *buffer, uint32_t buffer_size);
    OniStatus EnableLaser(uint32_t enable);

    XnStatus FillSupportedVideoModes();

private:
    //mx6000 add
    OniStatus LdpSet(XnBool ldp_status);
    OniStatus GetLdp(XnUInt32 &ldp_status);

    int ReadFlash(XnUInt32 offset, XnUInt32 size, XnUInt8 *data_ptr);
    int EraseFlash(XnUInt32 offset, XnUInt32 size);
    int WriteFlash(XnUInt32 offset, XnUInt32 size, const XnUInt8  *buf);
    int InitHeader(void *buf, XnUInt16 cmd, XnUInt16 data_len);
    XnUInt16 seq_num;
    XnUInt8	req_buf[512];
    XnUInt8	resp_buf[512];
    int send(void *cmd_req, XnUInt16 req_len, void *cmd_resp, XnUInt16 *resp_len);

    //irgain
    OniStatus ObSetIRGain(const void* ptr_data, int dataSize);
    OniStatus ObGetIRGain(void* data, int* pDataSize);

    OniStatus ObGetIRExp(void* data, int* pDataSize);
    OniStatus ObSetIRExp(const void* ptr_data, int dataSize);
    OniStatus UpdateFirmware(void *pBuff, int nSize);
    OniStatus UpdateRef(const void *pBuff, int nSize);
    OniStatus UpdateSwAlignParams(const void *pBuff, int nSize);
    OniStatus UpdateFlash(XnUInt32 offset, void *pBuff, int nSize);
    OniStatus MX400UpdateFirmware(void *pBuff, int nSize);
    OniStatus UpdateFirmwareWriteFlash(void *pBuff, int nSize, int address);
    OniStatus UpdateFirmwareReadFlash(void *pBuff, int nSize, int address);
    //dual camera read/write flash
    OniStatus SetDualCameraParam(void *pBuff, int nSize);
    OniStatus GetDualCameraParam(void *pBuff, int nSize);
    OniStatus SetCfgSn_Pn(const void *pBuff, int nSize);
    //set z0-baseline
    OniStatus SetCfgZ0_Baseline(const void *pBuffer, int nSize);
    OniStatus SetCfgParam(const void *pBuffer, int nSize, int nRegsNum, int nParamType);

    // Set depth config.
    OniStatus SetDepthConfig(const void *pBuf, int nSize);

    //set firmware config flash irgain or irexp
    OniStatus SetCfgIrGainExp(const void *pBuffer, int nSize, int nParamType);

    //get irgain/irexp default from flash
    OniStatus GetCfgIrGainExp(void* pBuffer, int* pDataSize, int nParamType);

    //set post filter threshold
    OniStatus SetCfgPostfiterThreshold(const void *pBuffer, int nSize);

    //get post filter threshold
    OniStatus GetCfgPostfiterThreshold(void* pBuffer, int*pDataSize);

    //set laser current or laser time
    OniStatus SetCfgLaserCurrentOrTime(const void *pBuffer, int nSize, int nParamType);

    //set cfg laser current or laser time
    OniStatus GetCfgLaserCurrentOrTime(void* pBuffer, int*pDataSize, int nParamType);

    OniStatus getCalibration(OniCalibrationCamera *pCalibration);

private:
    OniDeviceInfo m_info;
    int m_numSensors;
    OniSensorInfo m_sensors[10];
    oni::driver::DriverServices& m_driverServices;
    XnSensor m_sensor;
    XnOniDriver* m_pDriver;
    XnBool m_bUpdateFlag;
    OBExtension m_Ext;

    OniCalibrationCamera* m_pCalibration;
};

#endif // XNONIDEVICE_H
