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
#ifndef __XN_DEPTH_PROCESSOR_H__
#define __XN_DEPTH_PROCESSOR_H__

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "XnFrameStreamProcessor.h"
#include "XnSensorDepthStream.h"


#include "SoftwareRegistrator.h"

#include "apply_undistortion_capi.h"



//---------------------------------------------------------------------------
// Compilation Checks
//---------------------------------------------------------------------------

// Optimization: in order to save branches in the code itself, we create a shift-to-depth
// map which will actually translate shift-to-shift. This optimization relies on the
// fact that both shifts and depths are 16-bit long. If this is not the case,
// this optimization should be re-written.
// Then, any processor can always go through this LUT, no matter what the output format is.
#if (OniDepthPixel != XnUInt16)
	#error "Depth and Shift do not have the same size. Need to reconsider optimization!"
#endif

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
typedef struct ShiftToDepthConfig
{
	/** The zero plane distance in depth units. */
	unsigned short nZeroPlaneDistance;
	/** The zero plane pixel size */
	float fZeroPlanePixelSize;
	/** The distance between the emitter and the Depth Cmos */
	float fEmitterDCmosDistance;
	/** The maximum possible shift value from this device. */
	unsigned int nDeviceMaxShiftValue;
	/** The maximum possible depth from this device (as opposed to a cut-off). */
	unsigned int nDeviceMaxDepthValue;

	unsigned int nConstShift;
	unsigned int nPixelSizeFactor;
	unsigned int nParamCoeff;
	unsigned int nShiftScale;

	unsigned short nDepthMinCutOff;
	unsigned short nDepthMaxCutOff;

} ShiftToDepthConfig;

typedef struct ShiftToDepthTables
{
	XnBool bIsInitialized;
	/** The shift-to-depth table. */
	unsigned short* pShiftToDepthTable;
	/** The number of entries in the shift-to-depth table. */
	unsigned int nShiftsCount;
	/** The depth-to-shift table. */
	unsigned short* pDepthToShiftTable;
	/** The number of entries in the depth-to-shift table. */
	unsigned int nDepthsCount;
} ShiftToDepthTables;

class XnDepthProcessor : public XnFrameStreamProcessor
{
public:
	XnDepthProcessor(XnSensorDepthStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
	virtual ~XnDepthProcessor();

	XnStatus Init();

protected:
	//---------------------------------------------------------------------------
	// Overridden Functions
	//---------------------------------------------------------------------------
	virtual void OnStartOfFrame(const XnSensorProtocolResponseHeader* pHeader);
	virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
	virtual void OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS);

	//---------------------------------------------------------------------------
	// Helper Functions
	//---------------------------------------------------------------------------
	inline XnSensorDepthStream* GetStream()
	{
		return (XnSensorDepthStream*)XnFrameStreamProcessor::GetStream();
	}

	inline OniDepthPixel GetOutput(XnUInt16 nShift)
	{
		return nShift;//m_pShiftToDepthTable[nShift];
	}

	inline XnUInt32 GetExpectedSize()
	{
		return m_nExpectedFrameSize;
	}

	void CheckIgnoreEOF(); // ADD by ZW

	void filterSpeckles(void* srcbuf, int newVal, int maxSpeckleSize, int maxDiff, int resolutionX, int resolutionY);

	XnStatus GetDepthCoordinatesOfC2D(XnUInt32 x, XnUInt32 y, OniDepthPixel z, XnUInt32 imageXRes, XnUInt32 imageYRes, XnUInt32& depthX, XnUInt32& depthY);
	XnStatus GetColorCoordinatesOfD2C(XnUInt32 x, XnUInt32 y, OniDepthPixel z, XnUInt32 imageXRes, XnUInt32 imageYRes, XnUInt32& depthX, XnUInt32& depthY);
	static XnStatus XN_CALLBACK_TYPE GetColorCoordinatesOfD2CCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE GetDepthCoordinatesOfC2DCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie);

protected:
    void PadPixels(XnUInt32 nPixels);
    XnUInt32 CalculateExpectedSize();

protected:
    XnUInt32 m_nPaddingPixelsOnEnd;

private:
	XnGeneralProperty m_PixelC2DPropery;
	XnGeneralProperty m_PixelD2CPropery;

	XnBool m_applyRegistrationOnEnd;
	XnUInt32 m_nExpectedFrameSize;
	XnBool m_bShiftToDepthAllocated;
	//TRUE :shift ; FALSE:depth
	XnBool m_bShiftOrDepth;
	OniDepthPixel* m_pShiftToDepthTable;
	OniDepthPixel m_noDepthValue;
	OniDepthPixel* DepthBuf;
	OniDepthPixel* pDepthToShiftTable_org;
	// get config
	ShiftToDepthConfig config;
	ShiftToDepthTables m_ShiftToDepth;
	unsigned char* _buf;
	unsigned char* tag_Buf;
	uint8_t* m_filterCache;
	int m_MX6000Id;
	bool m_paramRead;
	double m_dualforcalRota;
	OBCameraParams m_CamParams;
	SoftwareRegistrator m_softRegistrator;

	//multi distance cal
	ApplyUndistHandle m_auh;
	XnUInt32 m_bMultiDisCalEnable;

	//new softfilter alg
	int m_maxDiff;
	int m_maxSpeckleSize;

};

#endif // XNDEPTHPROCESSOR_H
