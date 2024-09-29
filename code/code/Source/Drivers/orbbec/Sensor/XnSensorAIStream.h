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
#ifndef _XN_SENSOR_AI_STREAM_H_
#define _XN_SENSOR_AI_STREAM_H_

#include <DDK/XnAIStream.h>
#include "XnSensorStreamHelper.h"
#include "XnSensor.h"


class XnSensorAIStream : public XnAIStream, public IXnSensorStream
{
public:
    XnSensorAIStream(const XnChar* StreamName, XnSensorObjects* pObjects);
    ~XnSensorAIStream() { Free(); }

    XnStatus Init();
    XnStatus Free();
    void SetDriverConfig(char* path, int size);

    inline XnSensorStreamHelper* GetHelper() { return &m_helper; }

    XnStatus BatchConfig(const XnActualPropertiesHash& props) { return m_helper.BatchConfig(props); }

    friend class XnBodyProcessor;
    friend class XnUncompressedIRProcessor;

protected:
    virtual XnStatus SetFPS(XnUInt32 nFPS);
    virtual XnStatus SetMirror(XnBool bMirrored);
    virtual XnStatus SetActualRead(XnBool bRead);
    virtual XnStatus SetResolution(XnResolutions nResolution);
    virtual XnStatus SetOutputFormat(OniPixelFormat nOutputFormat);
    virtual XnStatus SetInputFormat(XnIOAIFormats nInputFormat);

    inline XnSensorFirmwareParams* GetFirmwareParams() const { return m_helper.GetFirmware()->GetParams(); }

    XnStatus Open() { return m_helper.Open(); }
    XnStatus Close() { return m_helper.Close(); }

    XnStatus Mirror(OniFrame* pFrame) const;
    XnStatus CalcRequiredSize(XnUInt32* pnRequiredSize) const;

    XnStatus OpenStreamImpl();
    XnStatus CloseStreamImpl();

    XnStatus ConfigureStreamImpl();
    XnStatus MapPropertiesToFirmware();
    XnStatus CreateDataProcessor(XnDataProcessor** ppProcessor);

    void GetFirmwareStreamConfig(XnResolutions* pnRes, XnUInt32* pnFPS) { *pnRes = GetResolution(); *pnFPS = GetFPS(); }

private:
    XnStatus FixFirmwareBug();

    static XnStatus XN_CALLBACK_TYPE OnMirrorChangedCallback(const XnProperty* pSender, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetActualReadCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
    static XnStatus XN_CALLBACK_TYPE SetInputFormatCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);

private:
    char m_drvCfgFile[XN_FILE_MAX_PATH];

    XnSensorStreamHelper m_helper;
    XnActualIntProperty m_actualRead;
    XnActualIntProperty m_inputFormat;
    XnActualIntProperty m_firmwareMirror;
};

#endif /// _XN_SENSOR_AI_STREAM_H_
