#include "YUV.h"
#include <XnProfiling.h>
#include "XnImageMjpegProcessor.h"


#define XN_MASK_IMAGE_MJPEG_PROCESSOR "XnImageMjpegProcessor"

XnImageMjpegProcessor::XnImageMjpegProcessor(XnSensorImageStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnImageProcessor(pStream, pHelper, pBufferManager, TRUE)
    , m_pCtx(NULL)
{
}

XnStatus XnImageMjpegProcessor::Init()
{
    XnStatus nRetVal = XnImageProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    nRetVal = XnStreamInitUncompressImageJ(&m_pCtx);
    XN_IS_STATUS_OK(nRetVal);

    XnUInt32 width = GetStream()->GetXRes();
    XnUInt32 height = GetStream()->GetYRes();
    XnUInt32 bufferSize = width * height * 2;
    XN_VALIDATE_BUFFER_ALLOCATE(m_yuvBuf, bufferSize);
    XN_VALIDATE_BUFFER_ALLOCATE(m_recvBuf, bufferSize);

    return XN_STATUS_OK;
}

XnImageMjpegProcessor::~XnImageMjpegProcessor()
{
    XnStreamFreeUncompressImageJ(&m_pCtx);
}

void XnImageMjpegProcessor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION(XN_MASK_IMAGE_MJPEG_PROCESSOR)

    /// Check if there is enough room to receive packet data.
    if (m_recvBuf.GetFreeSpaceInBuffer() >= nDataSize)
    {
        /// Copy values. Make sure we do not get corrupted pixels.
        m_recvBuf.UnsafeWrite(pData, nDataSize);

        /// All is good.
        return;
    }

    FrameIsCorrupted();
    m_recvBuf.Reset();
    xnLogWarning(XN_MASK_IMAGE_MJPEG_PROCESSOR, "Bad overflow mjpeg image! %d", m_recvBuf.GetSize());

    XN_PROFILING_END_SECTION
}

void XnImageMjpegProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION(XN_MASK_IMAGE_MJPEG_PROCESSOR)

    XnUInt32 yuvOutputSize = 0;
    OniPixelFormat format = GetStream()->GetOutputFormat();
    switch (format)
    {
    case ONI_PIXEL_FORMAT_MJPEG:
    {
        GetWriteBuffer()->UnsafeWrite(m_recvBuf.GetData(), m_recvBuf.GetSize());
        break;
    }
    case ONI_PIXEL_FORMAT_YUYV:
    {
        XnStatus nRetVal = XnStreamUncompressImjpegToYuyv(
            &m_pCtx,
            m_recvBuf.GetData(),
            m_recvBuf.GetSize(),
            GetWriteBuffer()->GetUnsafeWritePointer(),
            &yuvOutputSize,
            GetStream()->GetXRes(),
            GetStream()->GetYRes());
        if (XN_STATUS_OK == nRetVal)
            GetWriteBuffer()->UnsafeUpdateSize(yuvOutputSize);
        else
        {
            FrameIsCorrupted();
            xnLogWarning(XN_MASK_IMAGE_MJPEG_PROCESSOR, "Failed to uncompress mjpeg to yuyv for frame %d, error (%s)",
                GetCurrentFrameID(), xnGetStatusString(nRetVal));
        }
        break;
    }
    case ONI_PIXEL_FORMAT_RGB888:
    {
        XnStatus nRetVal = XnStreamUncompressImjpegToYuyv(
            &m_pCtx,
            m_recvBuf.GetData(),
            m_recvBuf.GetSize(),
            m_yuvBuf.GetUnsafeWritePointer(),
            &yuvOutputSize,
            GetStream()->GetXRes(),
            GetStream()->GetYRes());
        if (XN_STATUS_OK == nRetVal)
        {
            XnUInt32 actualRead = 0;
            XnUInt32 nOutputSize = GetExpectedOutputSize();
            m_yuvBuf.UnsafeUpdateSize(yuvOutputSize);
            YUYVToRGB888(m_yuvBuf.GetData(), GetWriteBuffer()->GetUnsafeWritePointer(), m_yuvBuf.GetSize(), &actualRead, &nOutputSize);
            GetWriteBuffer()->UnsafeUpdateSize(nOutputSize);
        } 
        else
        {
            FrameIsCorrupted();
            xnLogWarning(XN_MASK_IMAGE_MJPEG_PROCESSOR, "Failed to uncompress mjpeg to rgb for frame %d, error (%s)",
                GetCurrentFrameID(), xnGetStatusString(nRetVal));
        }
        break;
    }
    default:
        FrameIsCorrupted();
        xnLogError(XN_MASK_IMAGE_MJPEG_PROCESSOR, "Not supported ouput format (%d)", format);
        break;
    }

    m_yuvBuf.Reset();
    m_recvBuf.Reset();

    XnImageProcessor::OnEndOfFrame(pHeader);
    XN_PROFILING_END_SECTION
}
