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
#include "XnDepthStream.h"
#include <XnLog.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define XN_DEPTH_STREAM_MAX_DEPTH_VALUE 			XN_MAX_UINT16
#define XN_DEPTH_STREAM_DEFAULT_PIXEL_SIZE_FACTOR	1

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
XnDepthStream::XnDepthStream(const XnChar* csName, XnBool bAllowCustomResolutions, OniDepthPixel nDeviceMaxDepth, XnUInt16 nDeviceMaxShift)
    : XnPixelStream(XN_STREAM_TYPE_DEPTH, csName, bAllowCustomResolutions)
    , m_MinDepth(XN_STREAM_PROPERTY_MIN_DEPTH, "MinDepth")
    , m_MaxDepth(XN_STREAM_PROPERTY_MAX_DEPTH, "MaxDepth", nDeviceMaxDepth)
    , m_ConstShift(XN_STREAM_PROPERTY_CONST_SHIFT, "ConstShift")
    , m_PixelSizeFactor(XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR, "PixelSizeFactor", 1)
    , m_MaxShift(XN_STREAM_PROPERTY_MAX_SHIFT, "MaxShift", nDeviceMaxShift)
    , m_DeviceMaxDepth(XN_STREAM_PROPERTY_DEVICE_MAX_DEPTH, "DeviceMaxDepth", nDeviceMaxDepth)
    , m_ParamCoefficient(XN_STREAM_PROPERTY_PARAM_COEFF, "ParamCoeff")
    , m_ShiftScale(XN_STREAM_PROPERTY_SHIFT_SCALE, "ShiftScale")
    , m_ZeroPlaneDistance(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, "ZPD")
    , m_ZeroPlanePixelSize(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, "ZPPS")
    , m_EmitterDCmosDistance(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, "LDDIS")
    , m_GetDCmosRCmosDistance(XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE, "DCRCDIS")
    , m_DualFocallength(XN_STREAM_PROPERTY_DUAL_FOCALL_LENGTH, "DualFocallength")
    , m_DualCoeffDisparity(XN_STREAM_PROPERTY_DUAL_COEFF_DISPARITY, "DualCoeffDisparity")
    , m_SoftwareRegistrator(XN_STREAM_PROPERTY_SOFTWARE_REGISTRATION, "SoftwareRegistrator", 0)
    , m_SoftwareFilter(XN_STREAM_PROPERTY_SOFTWARE_FILTER, "SoftFilter", 1)
    , m_DepthRotate(XN_STREAM_PROPERTY_DEPTH_ROTATE, "DepthRotate")
    , m_DepthMaxDiff(XN_STREAM_PROPERTY_DEPTH_MAX_DIFF, "MaxDiff", 0)
    , m_MaxSpeckleSize(XN_STREAM_PROPERTY_DEPTH_MAX_SPECKLE_SIZE, "MaxSpeckleSize", 0)
{
    m_MinDepth.UpdateSetCallback(SetMinDepthCallback, this);
    m_MaxDepth.UpdateSetCallback(SetMaxDepthCallback, this);
}

XnStatus XnDepthStream::Init()
{
    XnStatus nRetVal = XN_STATUS_OK;

    // init base
    nRetVal = XnPixelStream::Init();
    XN_IS_STATUS_OK(nRetVal);

    // add properties
    XN_VALIDATE_ADD_PROPERTIES(this, &m_MinDepth, &m_MaxDepth, &m_ConstShift, &m_PixelSizeFactor,
        &m_MaxShift, &m_ParamCoefficient, &m_ShiftScale, &m_ZeroPlaneDistance, &m_ZeroPlanePixelSize,
        &m_EmitterDCmosDistance, &m_GetDCmosRCmosDistance, &m_DeviceMaxDepth, &m_DualFocallength, &m_DualCoeffDisparity, &m_SoftwareRegistrator, &m_SoftwareFilter, &m_DepthRotate, &m_DepthMaxDiff, &m_MaxSpeckleSize);

    m_SoftwareRegistrator.UpdateSetCallback(SetSoftwareRegistratorCallback, this);
    m_SoftwareFilter.UpdateSetCallback(SetSoftFilterCallback, this);
    m_DepthRotate.UpdateSetCallback(SetDepthRotateCallback, this);

    m_DepthMaxDiff.UpdateSetCallback(SetDepthMaxDiffCallback, this);
    m_MaxSpeckleSize.UpdateSetCallback(SetDepthMaxSpeckleSizeCallback, this);

    nRetVal = OutputFormatProperty().UnsafeUpdateValue(ONI_PIXEL_FORMAT_DEPTH_1_MM);
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = m_S2DHelper.Init(this);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnDepthStream::Free()
{
    XnPixelStream::Free();
    return (XN_STATUS_OK);
}

XnStatus XnDepthStream::SetMinDepth(OniDepthPixel nMinDepth)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (nMinDepth > GetDeviceMaxDepth())
    {
        return XN_STATUS_DEVICE_BAD_PARAM;
    }

    nRetVal = m_MinDepth.UnsafeUpdateValue(nMinDepth);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnDepthStream::SetMaxDepth(OniDepthPixel nMaxDepth)
{
    XnStatus nRetVal = XN_STATUS_OK;

    if (nMaxDepth > GetDeviceMaxDepth())
    {
        return XN_STATUS_DEVICE_BAD_PARAM;
    }

    nRetVal = m_MaxDepth.UnsafeUpdateValue(nMaxDepth);
    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XnDepthStream::ValidateDepthValue(OniDepthPixel nDepth)
{
    if (nDepth > GetDeviceMaxDepth())
    {
        return XN_STATUS_DEVICE_BAD_PARAM;
    }

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetMinDepthCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetMinDepth((OniDepthPixel)nValue);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetMaxDepthCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetMaxDepth((OniDepthPixel)nValue);
}

XnStatus XnDepthStream::SetSoftwareRegistrator(XnBool bSoftwareRegist)
{
    XnStatus nRetVal = XN_STATUS_OK;

    m_SoftwareRegistrator.UnsafeUpdateValue(bSoftwareRegist);

    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetSoftwareRegistratorCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetSoftwareRegistrator((XnBool)nValue);
}

XnStatus XnDepthStream::SetSoftFilter(XnBool bSoftFilter)
{
    XnStatus nRetVal = XN_STATUS_OK;

    m_SoftwareFilter.UnsafeUpdateValue(bSoftFilter);

    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetSoftFilterCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetSoftFilter((XnBool)nValue);
}

XnStatus XnDepthStream::SetDepthRotate(XnBool bDepthRotate)
{
    XnStatus nRetVal = XN_STATUS_OK;

    m_DepthRotate.UnsafeUpdateValue(bDepthRotate);

    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetDepthRotateCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetDepthRotate((XnBool)nValue);
}


XnStatus XnDepthStream::SetDepthMaxDiff(XnInt nMaxDiff)
{
    XnStatus nRetVal = XN_STATUS_OK;

    m_DepthMaxDiff.UnsafeUpdateValue(nMaxDiff);

    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetDepthMaxDiffCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetDepthMaxDiff((XnInt)nValue);
}

XnStatus XnDepthStream::SetDepthMaxSpeckleSize(XnInt nMaxSpeckleSize)
{
    XnStatus nRetVal = XN_STATUS_OK;

    m_MaxSpeckleSize.UnsafeUpdateValue(nMaxSpeckleSize);

    XN_IS_STATUS_OK(nRetVal);

    return (XN_STATUS_OK);
}

XnStatus XN_CALLBACK_TYPE XnDepthStream::SetDepthMaxSpeckleSizeCallback(XnActualIntProperty* /*pSender*/, XnUInt64 nValue, void* pCookie)
{
    XnDepthStream* pStream = (XnDepthStream*)pCookie;
    return pStream->SetDepthMaxSpeckleSize((XnInt)nValue);
}