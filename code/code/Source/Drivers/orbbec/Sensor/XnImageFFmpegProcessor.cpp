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
#include "YUV.h"
#include <XnProfiling.h>
#include "XnFormatsStatus.h"
#include "XnSensor.h"
#include "XnImageFFmpegProcessor.h"


#define XN_MASK_IMAGE_FFMPEG_PROCESSOR "XnImageFFmpegProcessor"

XnImageFFmpegProcessor::XnImageFFmpegProcessor(XnSensorImageStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnImageProcessor(pStream, pHelper, pBufferManager)
    , m_hasHwDecoder(false)
    , m_imageSize(0)
    , m_pAvpkt(NULL)
    , m_pDecodeFrame(NULL)
    , m_pOutPutFrame(NULL)
    , m_iPixelFormat(AV_PIX_FMT_NONE)
    , m_oPixelFormat(AV_PIX_FMT_NONE)
    , m_pHwCodec(NULL)
    , m_pSwsCtx(NULL)
    , m_pHwCtx(NULL)
    , m_pSwCodec(NULL)
    , m_pSwCtx(NULL)
{
}

XnImageFFmpegProcessor::~XnImageFFmpegProcessor()
{
    av_packet_free(&m_pAvpkt);
    av_frame_free(&m_pDecodeFrame);
    av_frame_free(&m_pOutPutFrame);
    sws_freeContext(m_pSwsCtx);

    if (m_hasHwDecoder && NULL != m_pHwCtx->extradata)
        av_freep(&m_pHwCtx->extradata);

    avcodec_close(m_pHwCtx);
    avcodec_free_context(&m_pHwCtx);
}

void XnImageFFmpegProcessor::FFmpegLogger(void* ptr, int level, const char* fmt, va_list vl)
{
    if (NULL == ptr || NULL == fmt)
        return;

    char content[1024] = { 0 };
    vsnprintf(content, sizeof(content), fmt, vl);

    switch (level)
    {
    case AV_LOG_DEBUG:
    case AV_LOG_TRACE:
    case AV_LOG_VERBOSE:
        // xnLogVerbose(XN_MASK_H264_IMAGE_PROCESSOR, "ffmpeg : %s", content);
        break;
    case AV_LOG_INFO:
        xnLogInfo(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "ffmpeg : %s", content);
        break;
    case AV_LOG_WARNING:
        xnLogWarning(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "ffmpeg : %s", content);
        break;
    case AV_LOG_ERROR:
    case AV_LOG_FATAL:
    case AV_LOG_PANIC:
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "ffmpeg : %s", content);
        break;
    default:
        break;
    }
}

XnStatus XnImageFFmpegProcessor::Init()
{
    XnStatus nRetVal = XnImageProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    XnUInt32 width = GetStream()->GetXRes();
    XnUInt32 height = GetStream()->GetYRes();
    XnUInt32 pixelBytes = GetStream()->GetBytesPerPixel();
    m_imageSize = width * height * pixelBytes;

    av_log_set_level(AV_LOG_TRACE);
    av_log_set_callback(XnImageFFmpegProcessor::FFmpegLogger);

    /// Looks like there is no need to register in the latest FFMPEG ?
    // av_register_all();
    // avcodec_register_all();

    /// Try to find a hardware decoder on Android.
#if defined(ANDROID) || defined(__ANDROID__)
    const void* pVM = GetStream()->GetHelper()->GetPrivateData()->pSensor->GetJavaVm();
    if (NULL != pVM)
    {
        nRetVal = av_jni_set_java_vm((void*)pVM, NULL);
        if (XN_STATUS_OK == nRetVal)
        {
            m_pHwCodec = avcodec_find_decoder_by_name("h264_mediacodec");
            if (NULL != m_pHwCodec)
            {
                m_hasHwDecoder = true;
                m_pHwCtx = avcodec_alloc_context3(m_pHwCodec);
                xnLogInfo(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Find h264_mediacodec success, codec (%p), context (%p).", m_pHwCodec->name, m_pHwCtx);
            }
        }
    }
#endif

    AVCodecID codecID = AV_CODEC_ID_NONE;
    switch (GetStream()->GetInputFormat())
    {
    case XN_IO_IMAGE_FORMAT_H264:
        codecID = AV_CODEC_ID_H264;
        break;
    case XN_IO_IMAGE_FORMAT_COMPRESSED_MJPEG:
        codecID = AV_CODEC_ID_MJPEG;
        break;
    default:
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Not supported input format (%d)...", GetStream()->GetInputFormat());
        return XN_STATUS_ERROR;
    }

    m_oPixelFormat = AV_PIX_FMT_RGB24;
    m_iPixelFormat = AV_PIX_FMT_YUV420P;
    switch (GetStream()->GetOutputFormat())
    {
    case ONI_PIXEL_FORMAT_H264:
    case ONI_PIXEL_FORMAT_MJPEG:
        SetCompressedOutput(TRUE);
        break;
    case ONI_PIXEL_FORMAT_RGB888:
        m_oPixelFormat = AV_PIX_FMT_RGB24;
        break;
    case ONI_PIXEL_FORMAT_YUYV:
        m_oPixelFormat = AV_PIX_FMT_YUYV422;
        break;
    case ONI_PIXEL_FORMAT_YUV422:
        m_oPixelFormat = AV_PIX_FMT_UYVY422;
        break;
    default:
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Not supported output format (%d)...", GetStream()->GetOutputFormat());
        return XN_STATUS_ERROR;
    }

    /// Look for a software decoder.
    m_pSwCodec = avcodec_find_decoder(codecID);
    XN_RET_IF_NULL(m_pSwCodec, XN_STATUS_BAD_PARAM);

    m_pSwCtx = avcodec_alloc_context3(m_pSwCodec);
    XN_RET_IF_NULL(m_pSwCtx, XN_STATUS_BAD_PARAM);

    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    nRetVal = avcodec_open2(m_pSwCtx, m_pSwCodec, NULL);
    if (0 != nRetVal)
    {
        av_strerror(nRetVal, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to open decoder '%s'..., error (%d | %s).", m_pSwCodec->name, nRetVal, errBuf);
    }

    m_pAvpkt = av_packet_alloc();
    XN_RET_IF_NULL(m_pAvpkt, XN_STATUS_BAD_PARAM);

    m_pDecodeFrame = av_frame_alloc();
    XN_RET_IF_NULL(m_pDecodeFrame, XN_STATUS_BAD_PARAM);

    m_pOutPutFrame = av_frame_alloc();
    XN_RET_IF_NULL(m_pOutPutFrame, XN_STATUS_BAD_PARAM);

    m_pSwsCtx = sws_getContext(width, height, m_iPixelFormat, width, height, m_oPixelFormat, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    XN_RET_IF_NULL(m_pSwsCtx, XN_STATUS_NULL_OUTPUT_PTR);

    XN_VALIDATE_BUFFER_ALLOCATE(m_recvBuf, m_imageSize);

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnImageFFmpegProcessor::OnExtractExtradata()
{
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    const AVBitStreamFilter* pBsf = av_bsf_get_by_name("extract_extradata");
    if (NULL == pBsf)
    {
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Error on getting 'extract_extradata'.");
        return XN_STATUS_ERROR;
    }

    AVBSFContext* pBsfContext = NULL;
    int ret = av_bsf_alloc(pBsf, &pBsfContext);
    if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to allocate BSF..., error (%d | %s)", ret, errBuf);
        return ret;
    }

    ret = avcodec_parameters_from_context(pBsfContext->par_in, m_pSwCtx);
    if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "avcodec parameters from context failed, error (%d | %s).", ret, errBuf);
        av_bsf_free(&pBsfContext);
        return ret;
    }

    ret = av_bsf_init(pBsfContext);
    if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "BSF init failed..., error (%d | [%s]).", ret, errBuf);
        av_bsf_free(&pBsfContext);
        return ret;
    }

    AVPacket* pPacketRef = av_packet_alloc();
    ret = av_packet_ref(pPacketRef, m_pAvpkt);
    if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to reference packet..., error (%d | %s)", ret, errBuf);
        av_bsf_free(&pBsfContext);
        return ret;
    }

    ret = av_bsf_send_packet(pBsfContext, pPacketRef);
    if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to send BSF packet..., error (%d | %s).", ret, errBuf);
        av_packet_unref(pPacketRef);
        av_packet_free(&pPacketRef);
        av_bsf_free(&pBsfContext);
        return ret;
    }

    bool done = false;
    while (ret >= 0 && !done)
    {
        ret = av_bsf_receive_packet(pBsfContext, pPacketRef);
        if (ret < 0)
        {
            if (AVERROR(EAGAIN) == ret || AVERROR_EOF == ret)
                continue;

            av_packet_unref(pPacketRef);
            av_packet_free(&pPacketRef);
            av_bsf_free(&pBsfContext);
            av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
            xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to receive BSF packet..., error (%d | %s).", ret, errBuf);
            return ret;
        }

        int dataSize = 0;
        uint8_t* pExtraData = av_packet_get_side_data(pPacketRef, AV_PKT_DATA_NEW_EXTRADATA, &dataSize);
        if (NULL != pExtraData)
        {
            if (NULL == m_pHwCtx->extradata)
            {
                m_pHwCtx->extradata = (uint8_t *)av_mallocz(dataSize + AV_INPUT_BUFFER_PADDING_SIZE);
                if (NULL == m_pHwCtx->extradata)
                {
                    av_packet_unref(pPacketRef);
                    av_packet_free(&pPacketRef);
                    av_bsf_free(&pBsfContext);
                    xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Error on allocating extra data buffer.");
                    return AVERROR(ENOMEM);
                }
            }

            memcpy(m_pHwCtx->extradata, pExtraData, dataSize);
            m_pHwCtx->extradata_size = dataSize;

            done = true;
            xnLogInfo(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Got extra data (%p), size (%d)\n", m_pHwCtx->extradata, dataSize);
        }
    }

    av_packet_unref(pPacketRef);
    av_packet_free(&pPacketRef);
    av_bsf_free(&pBsfContext);

    return (done ? XN_STATUS_OK : XN_STATUS_ERROR);
}

XnStatus XnImageFFmpegProcessor::OnSendPacket(const XnUInt8* pData, const XnUInt32 dataSize)
{
    XN_RET_IF_NULL(pData, XN_STATUS_NULL_INPUT_PTR);

    int ret = XN_STATUS_ERROR;
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    if (m_hasHwDecoder)
    {
#if 0
        int ret = OnExtractExtradata();
        XN_IS_STATUS_OK(ret);

        if (!avcodec_is_open(m_pHwCtx))
        {
            m_pHwCtx->width = GetStream()->GetXRes();
            m_pHwCtx->height = GetStream()->GetYRes();
            m_pHwCtx->coded_width = m_pHwCtx->width;
            m_pHwCtx->coded_height = m_pHwCtx->height;
            m_pHwCtx->pix_fmt = AV_PIX_FMT_YUV420P;
            m_pHwCtx->profile = FF_PROFILE_H264_HIGH;

            // int index = 0;
            // const AVCodecHWConfig *pHwCfg = NULL;
            // while (NULL != (pHwCfg = avcodec_get_hw_config(m_pHwCodec, index++)))
            // {
            //     if (AV_HWDEVICE_TYPE_MEDIACODEC == pHwCfg->device_type)
            //     {
            //         m_pHwCtx->pix_fmt = pHwCfg->pix_fmt;
            //         break;
            //     }
            // }

            ret = avcodec_open2(m_pHwCtx, m_pHwCodec, NULL);
            if (ret < 0)
            {
                m_hasHwDecoder = false;
                av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to open h264_mediacodec..., error (%d | %s).", ret, errBuf);
            }
        }
#else
        /// FIXME: currently use software decoding on Android due to some issue of hardware decoding.
        m_hasHwDecoder = false;
#endif
    }

    /// If hardware decoding is not supported then choose software decoding.
    m_pAvpkt->data = (XnUInt8*)pData;
    m_pAvpkt->size = (XnInt32)dataSize;
    AVCodecContext* pCtx = m_hasHwDecoder ? m_pHwCtx : m_pSwCtx;
    ret = avcodec_send_packet(pCtx, m_pAvpkt);
    if (AVERROR(EAGAIN) == ret || AVERROR_EOF == ret)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogVerbose(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Input is not accepted in the current state, must try to read output first..., error(%d | %s)", ret, errBuf);
        return XN_STATUS_OK;
    }
    else if (ret < 0)
    {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to send packet..., error (%d | %s)", ret, errBuf);
        return ret;
    }

    return XN_STATUS_OK;
}

XnStatus XnImageFFmpegProcessor::OnReceiveFrame()
{
    int ret = 0;
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    while (ret >= 0)
    {
        /// If hardware decoding is not supported then choose software decoding.
        AVCodecContext* pCtx = m_hasHwDecoder ? m_pHwCtx : m_pSwCtx;
        ret = avcodec_receive_frame(pCtx, m_pDecodeFrame);
        if (0 == ret)
        {
            /// Got the frame.
            return XN_STATUS_OK;
        }
        else if (AVERROR(EAGAIN) == ret || AVERROR_EOF == ret)
        {
            /// EOF - the decoder was flushed, no more data.
            /// EAGAIN - we need to push more data with avcodec_send_packet.

            /// Be nice to the user and prepare the decoder for new stream for him if he wants to continue the decoding (startover).
            if (AVERROR_EOF == ret)
                avcodec_flush_buffers(pCtx);

            av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
            xnLogVerbose(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Output is not available in this state, must try to send new packet..., error (%d | %s)", ret, errBuf);
            return XN_STATUS_OK;
        }
        else if (ret < 0)
        {
            av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
            xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Failed to receive frame..., error (%d | %s)", ret, errBuf);
            return ret;
        }
    }

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnImageFFmpegProcessor::OnFrameFormat(XnBuffer* pBuffer)
{
    XN_RET_IF_NULL(pBuffer, XN_STATUS_NULL_INPUT_PTR);

    /// Make sure the resolution of this current frame which received from device is match what we expected.
    if ((uint32_t)m_pDecodeFrame->width != GetStream()->GetXRes() && (uint32_t)m_pDecodeFrame->height != GetStream()->GetYRes())
    {
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Resolution (%d x %d) decoded from this current frame was not match what we expected (%d x %d)",
            m_pDecodeFrame->width,
            m_pDecodeFrame->height,
            GetStream()->GetXRes(),
            GetStream()->GetYRes());
        return XN_STATUS_NO_MATCH;
    }

    av_image_fill_arrays(
        m_pOutPutFrame->data,
        m_pOutPutFrame->linesize,
        pBuffer->GetUnsafeWritePointer(),
        m_oPixelFormat,
        m_pDecodeFrame->width,
        m_pDecodeFrame->height,
        8);

    sws_scale(
        m_pSwsCtx,
        m_pDecodeFrame->data,
        m_pDecodeFrame->linesize,
        0,
        m_pDecodeFrame->height,
        m_pOutPutFrame->data,
        m_pOutPutFrame->linesize);

    pBuffer->UnsafeUpdateSize(m_imageSize);

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnImageFFmpegProcessor::OnFrameDecode(XnBuffer* pDst, const XnUInt8* pSrc, const XnUInt32 dataSize)
{
    XN_RET_IF_NULL(pSrc, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDst, XN_STATUS_NULL_INPUT_PTR);

    int ret = OnSendPacket(pSrc, dataSize);
    if (XN_STATUS_OK != ret)
    {
        FrameIsCorrupted();
        return ret;
    }

    ret = OnReceiveFrame();
    if (XN_STATUS_OK != ret)
    {
        FrameIsCorrupted();
        return ret;
    }

    ret = OnFrameFormat(pDst);
    if (XN_STATUS_OK != ret)
    {
        FrameIsCorrupted();
        return ret;
    }

    /// All is good.
    return XN_STATUS_OK;
}

void XnImageFFmpegProcessor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    /// Check if there is enough room to receive packet data.
    if (m_recvBuf.GetFreeSpaceInBuffer() >= nDataSize)
    {
        /// Copy values. Make sure we do not get corrupted pixels.
        m_recvBuf.UnsafeWrite(pData, nDataSize);

        /// All is good.
        return;
    }

    /// Not expected...
    xnLogWarning(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Bad overflow image! size = %d, maxSize = %d", m_recvBuf.GetSize(), m_recvBuf.GetMaxSize());
    FrameIsCorrupted();
    m_recvBuf.Reset();
}

void XnImageFFmpegProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION(XN_MASK_IMAGE_FFMPEG_PROCESSOR)

    XnBuffer* pWrite = GetWriteBuffer();
    OniPixelFormat format = GetStream()->GetOutputFormat();
    switch (format)
    {
    case ONI_PIXEL_FORMAT_YUYV:
    case ONI_PIXEL_FORMAT_YUV422:
    case ONI_PIXEL_FORMAT_RGB888:
        OnFrameDecode(pWrite, m_recvBuf.GetData(), m_recvBuf.GetSize());
        break;
    case ONI_PIXEL_FORMAT_H264:
    case ONI_PIXEL_FORMAT_MJPEG:
        /// No need to decode compressed data, just output directly.
        pWrite->UnsafeWrite(m_recvBuf.GetData(), m_recvBuf.GetSize());
        break;
    default:
        FrameIsCorrupted();
        xnLogError(XN_MASK_IMAGE_FFMPEG_PROCESSOR, "Not supported ouput format (%d)", format);
    }

    m_recvBuf.Reset();

    /// Call base.
    XnImageProcessor::OnEndOfFrame(pHeader);

    XN_PROFILING_END_SECTION
}
