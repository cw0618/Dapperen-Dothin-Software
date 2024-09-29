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
#include "XnAIStream.h"


XnAIStream::XnAIStream(const XnChar* csName, XnBool bAllowCustomResolutions, OniBodyPixel nDeviceMaxPixel)
    : XnPixelStream(XN_STREAM_TYPE_AI, csName, bAllowCustomResolutions)
    , m_deviceMaxPixel(XN_STREAM_PROPERTY_DEVICE_MAX_AI, "DeviceMaxBody", nDeviceMaxPixel)
{
}

XnStatus XnAIStream::Init()
{
    XnStatus nRetVal = XnPixelStream::Init();
    XN_IS_STATUS_OK(nRetVal);

    XN_VALIDATE_ADD_PROPERTIES(this, &m_deviceMaxPixel);

    return XN_STATUS_OK;
}
