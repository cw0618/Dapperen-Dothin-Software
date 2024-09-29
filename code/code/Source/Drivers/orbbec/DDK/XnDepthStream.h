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
#ifndef XNDEPTHSTREAM_H
#define XNDEPTHSTREAM_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <DDK/XnPixelStream.h>
#include <DDK/XnActualRealProperty.h>
#include <DDK/XnShiftToDepthStreamHelper.h>

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

class XnDepthStream : public XnPixelStream
{
public:
	XnDepthStream(const XnChar* csName, XnBool bAllowCustomResolutions, OniDepthPixel nDeviceMaxDepth, XnUInt16 nDeviceMaxShift);
	~XnDepthStream() { Free(); }

	//---------------------------------------------------------------------------
	// Overridden Methods
	//---------------------------------------------------------------------------
	XnStatus Init();
	XnStatus Free();

	//---------------------------------------------------------------------------
	// Getters
	//---------------------------------------------------------------------------
	inline OniDepthPixel GetMinDepth() const { return (OniDepthPixel)m_MinDepth.GetValue(); }
	inline OniDepthPixel GetMaxDepth() const { return (OniDepthPixel)m_MaxDepth.GetValue(); }
	inline XnUInt32 GetConstShift() const { return (XnUInt32)m_ConstShift.GetValue(); }
	inline XnUInt32 GetPixelSizeFactor() const { return (XnUInt32)m_PixelSizeFactor.GetValue(); }
	inline XnUInt16 GetMaxShift() const { return (XnUInt16)m_MaxShift.GetValue(); }
	inline OniDepthPixel GetDeviceMaxDepth() const { return (OniDepthPixel)m_DeviceMaxDepth.GetValue(); }
	inline XnUInt32 GetParamCoefficient() const { return (XnUInt32)m_ParamCoefficient.GetValue(); }
	inline XnUInt32 GetShiftScale() const { return (XnUInt32)m_ShiftScale.GetValue(); }
	inline XnDouble GetZeroPlaneDistance() const { return m_ZeroPlaneDistance.GetValue(); }
	inline XnDouble GetZeroPlanePixelSize() const { return m_ZeroPlanePixelSize.GetValue(); }
	inline XnDouble GetEmitterDCmosDistance() const { return m_EmitterDCmosDistance.GetValue(); }
	inline XnDouble GetDCmosRCmosDistance() const { return m_GetDCmosRCmosDistance.GetValue(); }

	inline OniDepthPixel* GetShiftToDepthTable() const { return m_S2DHelper.GetShiftToDepthTable(); }
	inline XnUInt16* GetDepthToShiftTable() const { return m_S2DHelper.GetDepthToShiftTable(); }

	inline int getSoftwareRegistratorMode() const { return (XnBool)m_SoftwareRegistrator.GetValue(); }
	inline int getSoftFilterMode() const { return (XnBool)m_SoftwareFilter.GetValue(); }
	inline int getDepthRotateMode() const { return (int)m_DepthRotate.GetValue(); }

	inline int getDepthMaxDiff() const { return (int)m_DepthMaxDiff.GetValue(); }

	inline int getMaxSpeckleSize() const { return (int)m_MaxSpeckleSize.GetValue(); }

protected:
	//---------------------------------------------------------------------------
	// Properties Getters
	//---------------------------------------------------------------------------
	inline XnActualIntProperty& MinDepthProperty() { return m_MinDepth; }
	inline XnActualIntProperty& MaxDepthProperty() { return m_MaxDepth; }
	inline XnActualIntProperty& ConstShiftProperty() { return m_ConstShift; }
	inline XnActualIntProperty& PixelSizeFactorProperty() { return m_PixelSizeFactor; }
	inline XnActualIntProperty& MaxShiftProperty() { return m_MaxShift; }
	inline XnActualIntProperty& DeviceMaxDepthProperty() { return m_DeviceMaxDepth; }
	inline XnActualIntProperty& ParamCoefficientProperty() { return m_ParamCoefficient; }
	inline XnActualIntProperty& ShiftScaleProperty() { return m_ShiftScale; }
	inline XnActualRealProperty& ZeroPlaneDistanceProperty() { return m_ZeroPlaneDistance; }
	inline XnActualRealProperty& ZeroPlanePixelSizeProperty() { return m_ZeroPlanePixelSize; }
	inline XnActualRealProperty& EmitterDCmosDistanceProperty() { return m_EmitterDCmosDistance; }
	inline XnActualRealProperty& GetDCmosRCmosDistanceProperty() { return m_GetDCmosRCmosDistance; }

	inline XnActualRealProperty& DualFocallengthProperty() { return m_DualFocallength; }
	inline XnActualRealProperty& DualCoeffDisparityProperty() { return m_DualCoeffDisparity; }

	virtual XnStatus SetSoftwareRegistrator(XnBool bRegistration);
	//---------------------------------------------------------------------------
	// Setters
	//---------------------------------------------------------------------------
	virtual XnStatus SetMinDepth(OniDepthPixel nMinDepth);
	virtual XnStatus SetMaxDepth(OniDepthPixel nMaxDepth);
	inline XnActualIntProperty& SoftFilterProperty() { return m_SoftwareFilter; }
	inline XnActualIntProperty& DepthRotateProperty() { return m_DepthRotate; }
	virtual XnStatus SetSoftFilter(XnBool bSoftFilter);
	virtual XnStatus SetDepthRotate(XnBool bDepthRotate);

	inline XnActualIntProperty& DepthMaxDiffProperty() { return m_DepthMaxDiff; }
	virtual XnStatus SetDepthMaxDiff(XnInt nMaxDiff);

	inline XnActualIntProperty& DepthMaxSpeckleSizeProperty() { return m_MaxSpeckleSize; }
	virtual XnStatus SetDepthMaxSpeckleSize(XnInt nMaxSpeckleSize);
protected:
	//---------------------------------------------------------------------------
	// Helper functions
	//---------------------------------------------------------------------------
	XnStatus ValidateDepthValue(OniDepthPixel nDepth);

private:
	// callbacks
	static XnStatus XN_CALLBACK_TYPE SetMinDepthCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetMaxDepthCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetSoftwareRegistratorCallback(XnActualIntProperty* pSender, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetSoftFilterCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetDepthRotateCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetDepthMaxDiffCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie);
	static XnStatus XN_CALLBACK_TYPE SetDepthMaxSpeckleSizeCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie);
	//---------------------------------------------------------------------------
	// Members
	//---------------------------------------------------------------------------

	XnActualIntProperty m_MinDepth;
	XnActualIntProperty m_MaxDepth;
	XnActualIntProperty m_ConstShift;
	XnActualIntProperty m_PixelSizeFactor;
	XnActualIntProperty m_MaxShift;
	XnActualIntProperty m_DeviceMaxDepth;
	XnActualIntProperty m_ParamCoefficient;
	XnActualIntProperty m_ShiftScale;
	XnActualRealProperty m_ZeroPlaneDistance;
	XnActualRealProperty m_ZeroPlanePixelSize;
	XnActualRealProperty m_EmitterDCmosDistance;
	XnActualRealProperty m_GetDCmosRCmosDistance;
	XnActualRealProperty m_DualFocallength;
	XnActualRealProperty m_DualCoeffDisparity;
	XnActualIntProperty m_SoftwareRegistrator;
	XnActualIntProperty m_SoftwareFilter;
	XnActualIntProperty m_DepthRotate;
	//softfilter MaxDiff param
	XnActualIntProperty m_DepthMaxDiff;
	//softfilter MaxSpeckleSize param
	XnActualIntProperty m_MaxSpeckleSize;
	XnShiftToDepthStreamHelper m_S2DHelper;
};

#endif // XNDEPTHSTREAM_H
