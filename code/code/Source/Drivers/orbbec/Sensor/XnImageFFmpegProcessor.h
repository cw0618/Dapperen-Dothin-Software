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
#ifndef _XN_IMAGE_FFMPEG_PROCESSOR_H_
#define _XN_IMAGE_FFMPEG_PROCESSOR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "libavcodec/avcodec.h"
#include "libavutil/dict.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"
#if defined(ANDROID) || defined(__ANDROID__)
#include "libavcodec/jni.h"
#endif
#ifdef __cplusplus
}
#endif

#include "XnImageProcessor.h"


class XnImageFFmpegProcessor : public XnImageProcessor
{
public:
    XnImageFFmpegProcessor(XnSensorImageStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager);
    ~XnImageFFmpegProcessor();

    XnStatus Init();

protected:
    /// Overridden Functions.
    virtual void OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader);
    virtual void ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* pHeader, const XnUChar* pData, XnUInt32 nDataOffset, XnUInt32 nDataSize);

    XnStatus OnFrameDecode(XnBuffer* pDst, const XnUInt8* pSrc, const XnUInt32 dataSize);

private:
    XnStatus OnReceiveFrame();
    XnStatus OnExtractExtradata();
    XnStatus OnFrameFormat(XnBuffer* pBuffer);
    XnStatus OnSendPacket(const XnUInt8* pData, const XnUInt32 dataSize);

    static void FFmpegLogger(void* ptr, int level, const char* fmt, va_list vl);

private:
    bool m_hasHwDecoder;
    XnUInt32 m_imageSize;
    XnBuffer m_recvBuf;

    AVPacket* m_pAvpkt;

    AVFrame* m_pDecodeFrame;
    AVFrame* m_pOutPutFrame;
    
    AVPixelFormat m_iPixelFormat;
    AVPixelFormat m_oPixelFormat;

    AVCodec* m_pSwCodec;
    AVCodecContext* m_pSwCtx;

    AVCodec* m_pHwCodec;
    AVCodecContext* m_pHwCtx;

    SwsContext* m_pSwsCtx;
};

#endif /// _XN_IMAGE_FFMPEG_PROCESSOR_H_
