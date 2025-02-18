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
#ifndef XNIRSTREAM_H
#define XNIRSTREAM_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <DDK/XnPixelStream.h>

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

/** Represents a base class for an image stream. */
class XnIRStream : public XnPixelStream
{
public:
    XnIRStream(const XnChar* csName, XnBool bAllowCustomResolutions, OniIRPixel nDeviceMaxIR);

    XnStatus Init();

    //---------------------------------------------------------------------------
    // Getters
    //---------------------------------------------------------------------------
    inline OniIRPixel GetDeviceMaxIR() const { return (OniIRPixel)m_DeviceMaxIR.GetValue(); }

protected:
    //---------------------------------------------------------------------------
    // Properties Getters
    //---------------------------------------------------------------------------
    inline XnActualIntProperty& DeviceMaxIRProperty() { return m_DeviceMaxIR; }

private:
    //---------------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------------
    XnActualIntProperty m_DeviceMaxIR;
};

#endif // XNIRSTREAM_H
