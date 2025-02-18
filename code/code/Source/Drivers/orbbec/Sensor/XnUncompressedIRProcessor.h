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
#ifndef XNCOMPRESSEDIRPROCESSOR_H
#define XNCOMPRESSEDIRPROCESSOR_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "XnFrameStreamProcessor.h"
#include "XnSensorIRStream.h"

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

class XnUncompressedIRProcessor : public XnFrameStreamProcessor
{
public:
    XnUncompressedIRProcessor(XnSensorIRStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    virtual ~XnUncompressedIRProcessor();

    XnStatus Init();

protected:
    //---------------------------------------------------------------------------
    // Overridden Functions
    //---------------------------------------------------------------------------
    virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
    virtual XnUInt64 CreateTimestampFromDevice(XnUInt32 nDeviceTimeStamp);
    virtual void OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS);

    //---------------------------------------------------------------------------
    // Internal Functions
    //---------------------------------------------------------------------------
private:
    XnStatus Unpack(const XnUInt8* pcInput, const XnUInt32 nInputSize, XnUInt16* pnOutput, XnUInt32* pnActualRead, XnUInt32* pnOutputSize);
    void IRto888(XnUInt16* pInput, XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);
    void IRtoGray8(XnUInt16* pInput, XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);
    inline XnSensorIRStream* GetStream()
    {
        return (XnSensorIRStream*)XnFrameStreamProcessor::GetStream();
    }

    //---------------------------------------------------------------------------
    // Class Members
    //---------------------------------------------------------------------------
private:
    /* A buffer to store bytes till we have enough to unpack. */
    XnBuffer m_ContinuousBuffer;
    XnBuffer m_UnpackedBuffer;
    XnBuffer m_metadataBuffer;
    XnUInt64 m_nRefTimestamp; // needed for firmware bug workaround
    XnDepthCMOSType m_DepthCMOSType;

    XnInt32 m_metadataBytes;
};

#endif // XNCOMPRESSEDIRPROCESSOR_H
