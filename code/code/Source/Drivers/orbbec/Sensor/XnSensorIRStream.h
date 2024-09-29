/*****************************************************************************
*									     *
*  OpenNI 2.x Alpha							     *
*  Copyright (C) 2012 PrimeSense Ltd.					     *
*									     *
*  This file is part of OpenNI. 					     *
*									     *
*  Licensed under the Apache License, Version 2.0 (the "License");	     *
*  you may not use this file except in compliance with the License.	     *
*  You may obtain a copy of the License at				     *
*									     *
*      http://www.apache.org/licenses/LICENSE-2.0			     *
*									     *
*  Unless required by applicable law or agreed to in writing, software	     *
*  distributed under the License is distributed on an "AS IS" BASIS,	     *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and	     *
*  limitations under the License.					     *
*									     *
*****************************************************************************/
#ifndef XNSENSORIRSTREAM_H
#define XNSENSORIRSTREAM_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <DDK/XnIRStream.h>
#include "XnSensorStreamHelper.h"
#include "XnSensor.h"
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define XN_IR_STREAM_DEFAULT_FPS				30
#define XN_IR_STREAM_DEFAULT_RESOLUTION 		XN_RESOLUTION_QVGA
#define XN_IR_STREAM_DEFAULT_OUTPUT_FORMAT		ONI_PIXEL_FORMAT_GRAY16
#define XN_IR_STREAM_DEFAULT_MIRROR				FALSE

//---------------------------------------------------------------------------
// XnSensorIRStream class
//---------------------------------------------------------------------------
class XnSensorIRStream : public XnIRStream, public IXnSensorStream
{
public:
	XnSensorIRStream(const XnChar* StreamName, XnSensorObjects* pObjects);
	~XnSensorIRStream() { Free(); }

	//---------------------------------------------------------------------------
	// Overridden Methods
	//---------------------------------------------------------------------------
	XnStatus Init();
	XnStatus Free();
	XnStatus BatchConfig(const XnActualPropertiesHash& props) { return m_Helper.BatchConfig(props); }
	void SetDriverConfig(char* path, int size);
	inline XnSensorStreamHelper* GetHelper() { return &m_Helper; }

	friend class XnIRProcessor;
	friend class XnUncompressedIRProcessor;
protected:
	virtual XnStatus SetInputFormat(XnIOIRFormats nInputFormat);
	inline XnSensorFirmwareParams* GetFirmwareParams() const { return m_Helper.GetFirmware()->GetParams(); }

	//---------------------------------------------------------------------------
	// Overridden Methods
	//---------------------------------------------------------------------------
	XnStatus Open() { return m_Helper.Open(); }
	XnStatus Close() { return m_Helper.Close(); }
	XnStatus CalcRequiredSize(XnUInt32* pnRequiredSize) const;
	XnStatus CropImpl(OniFrame* pFrame, const OniCropping* pCropping);
	XnStatus ConfigureStreamImpl();
	XnStatus OpenStreamImpl();
	XnStatus CloseStreamImpl();
	XnStatus CreateDataProcessor(XnDataProcessor** ppProcessor);
	XnStatus MapPropertiesToFirmware();
    XnStatus Mirror(OniFrame* pFrame) const;

	void GetFirmwareStreamConfig(XnResolutions* pnRes, XnUInt32* pnFPS) { *pnRes = GetResolution(); *pnFPS = GetFPS(); }

	//---------------------------------------------------------------------------
	// Setters
	//---------------------------------------------------------------------------
    virtual XnStatus SetFPS(XnUInt32 nFPS);
    virtual XnStatus SetMirror(XnBool bMirrored);
    virtual XnStatus SetActualRead(XnBool bRead);
    virtual XnStatus SetResolution(XnResolutions nResolution);
    virtual XnStatus SetOutputFormat(OniPixelFormat nOutputFormat);
    virtual XnStatus SetCroppingMode(XnCroppingMode mode);
	virtual XnStatus SetCropping(const OniCropping* pCropping);

private:
	XnStatus OnIsMirroredChanged();
	XnStatus SetCroppingImpl(const OniCropping* pCropping, XnCroppingMode mode);

	XnStatus FixFirmwareBug();

	static XnStatus XN_CALLBACK_TYPE IsMirroredChangedCallback(const XnProperty* pSender, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetActualReadCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetCroppingModeCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetInputFormatCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);

    XnActualIntProperty m_InputFormat;
	XnActualIntProperty m_CroppingMode;

	XnSensorStreamHelper m_Helper;
    XnActualIntProperty m_firmwareMirror;
	XnActualIntProperty m_FirmwareCropSizeX;
	XnActualIntProperty m_FirmwareCropSizeY;
	XnActualIntProperty m_FirmwareCropOffsetX;
	XnActualIntProperty m_FirmwareCropOffsetY;
	XnActualIntProperty m_FirmwareCropMode;

	XnActualIntProperty m_ActualRead;
	char m_driverConfig[XN_FILE_MAX_PATH];
};


#endif // XNSENSORIRSTREAM_H
