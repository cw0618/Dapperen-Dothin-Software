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
#ifndef _XN_AI_PROCESSOR_H_
#define _XN_AI_PROCESSOR_H_
#include "XnSensorAIStream.h"
#include "XnFrameStreamProcessor.h"


class XnAIProcessor : public XnFrameStreamProcessor
{
public:
    XnAIProcessor(XnSensorAIStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    virtual ~XnAIProcessor();

    XnStatus Init();

protected:
    virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
    virtual void OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS);

private:
    inline XnSensorAIStream* GetStream()
    {
        return (XnSensorAIStream*)XnFrameStreamProcessor::GetStream();
    }

    virtual XnStatus OnProtocolParse(XnBuffer* pDstBuf, XnBuffer* pSrcBuf);

    XnStatus OnPhaseParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus OnDepthIRParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus OnBodyMaskParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus OnBodyJointParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus OnBodyShapeParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus OnFloorInfoParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader);
    XnStatus UnpackRaw10To16(XnUInt16* pDst, const XnUInt8* pSrc, const XnInt32 width, const XnInt32 height);

private:
    XnInt32 m_offset;
    XnInt32 m_minSize;
    XnBuffer m_recvBuf;
};

#endif /// _XN_AI_PROCESSOR_H_
