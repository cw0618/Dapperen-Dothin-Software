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
#ifndef _XN_SENSOR_PHASE_STREAM_H_
#define _XN_SENSOR_PHASE_STREAM_H_

#include <DDK/XnPhaseStream.h>
#include "XnSensorStreamHelper.h"
#include "XnSensor.h"


#define XN_PHASE_STREAM_DEFAULT_FPS           (30)
#define XN_PHASE_STREAM_DEFAULT_RESOLUTION    (XN_RESOLUTION_1280_960)
#define XN_PHASE_STREAM_DEFAULT_OUTPUT_FORMAT (ONI_PIXEL_FORMAT_GRAY16)

class XnSensorPhaseStream : public XnPhaseStream, public IXnSensorStream
{
public:
    XnSensorPhaseStream(const XnChar* streamName, XnSensorObjects* pObjects);
    ~XnSensorPhaseStream() { Free(); }

    XnStatus Init();
    XnStatus Free();
    void SetDriverConfig(char* path, int size);

    inline XnSensorStreamHelper* GetHelper() { return &m_helper; }

    inline XnIOPhaseFormats GetInputFormat() { return (XnIOPhaseFormats)m_inputFormat.GetValue(); }

    XnStatus BatchConfig(const XnActualPropertiesHash& props) { return m_helper.BatchConfig(props); }

    friend class XnPhaseProcessor;
    friend class XnPhasePacked10Processor;

protected:
    inline XnSensorFirmwareParams* GetFirmwareParams() const { return m_helper.GetFirmware()->GetParams(); }

    void GetFirmwareStreamConfig(XnResolutions* pnRes, XnUInt32* pnFPS) { *pnRes = GetResolution(); *pnFPS = GetFPS(); }

    /// Overridden Methods
    XnStatus Open() { return m_helper.Open(); }
    XnStatus Close() { return m_helper.Close(); }

    XnStatus OpenStreamImpl();
    XnStatus CloseStreamImpl();

    XnStatus ConfigureStreamImpl();
    XnStatus MapPropertiesToFirmware();
    XnStatus CreateDataProcessor(XnDataProcessor** ppProcessor);
    
    XnStatus Mirror(OniFrame* pFrame) const;
    XnStatus CalcRequiredSize(XnUInt32* pnRequiredSize) const;
    XnStatus CropImpl(OniFrame* pFrame, const OniCropping* pCropping);

    /// Setters
    virtual XnStatus SetFPS(XnUInt32 nFPS);
    virtual XnStatus SetMirror(XnBool bMirrored);
    virtual XnStatus SetActualRead(XnBool bRead);
    virtual XnStatus SetResolution(XnResolutions nResolution);
    virtual XnStatus SetOutputFormat(OniPixelFormat nOutputFormat);
    virtual XnStatus SetInputFormat(XnIOPhaseFormats nInputFormat);
    virtual XnStatus SetCroppingMode(XnCroppingMode mode);
    virtual XnStatus SetCropping(const OniCropping* pCropping);
  
    virtual XnStatus GetFrequencyMode(OniFrequencyMode* pMode);
    virtual XnStatus SetFrequencyMode(const OniFrequencyMode mode);

private:
    XnStatus FixFirmwareBug();

    XnStatus OnMirrorChanged();
    XnStatus SetCroppingImpl(const OniCropping* pCropping, XnCroppingMode mode);

    static XnStatus XN_CALLBACK_TYPE OnMirrorChangedCallback(const XnProperty* pSender, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetActualReadCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetInputFormatCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetCroppingModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE GetFrequencyModeCallback(const XnActualIntProperty* pSender, XnUInt64* pValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetFrequencyModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);

private:
    char m_drvCfgFile[XN_FILE_MAX_PATH];

    XnSensorStreamHelper m_helper;

    XnActualIntProperty m_freqMode;
    XnActualIntProperty m_actualRead;
    XnActualIntProperty m_inputFormat;

    XnActualIntProperty m_croppingMode;
    XnActualIntProperty m_firmwareMirror;
    XnActualIntProperty m_firmwareCropMode;
    XnActualIntProperty m_firmwareCropSizeX;
    XnActualIntProperty m_firmwareCropSizeY;
    XnActualIntProperty m_firmwareCropOffsetX;
    XnActualIntProperty m_firmwareCropOffsetY;
};

#endif /// _XN_SENSOR_PHASE_STREAM_H_
