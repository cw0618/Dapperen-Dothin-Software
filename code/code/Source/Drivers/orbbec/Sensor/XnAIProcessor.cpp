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
#ifdef XN_NEON
#include <arm_neon.h>
#endif
#include <XnProfiling.h>
#include "XnSensor.h"
#include "XnAIProcessor.h"


#define XN_MASK_AI_PROCESSOR            "XnAIProcessor"

#define XN_AI_PHASE_DEFAULT_WIDTH       (1920)
#define XN_AI_PHASE_DEFAULT_HEIGHT      (480)
#define XN_AI_PHASE_DEFAULT_SIZE        (XN_AI_PHASE_DEFAULT_WIDTH * XN_AI_PHASE_DEFAULT_HEIGHT * 10 / 8)

#define XN_AI_BODY_MASK_DEFAULT_WIDTH   (640)
#define XN_AI_BODY_MASK_DEFAULT_HEIGHT  (384)
#define XN_AI_BODY_MASK_DEFAULT_SIZE    (XN_AI_BODY_MASK_DEFAULT_WIDTH * XN_AI_BODY_MASK_DEFAULT_HEIGHT * sizeof(XnUInt8))

#define XN_AI_VGA_WIDTH  (320)
#define XN_AI_VGA_HEIGHT (240)
#define XN_AI_VGA_SIZE                  (XN_AI_VGA_WIDTH * XN_AI_VGA_HEIGHT * sizeof(XnUInt16))
#define XN_AI_FLOOR_MASK_DEFAULT_SIZE   (XN_AI_VGA_WIDTH * XN_AI_VGA_HEIGHT * sizeof(XnUInt8))

XnAIProcessor::XnAIProcessor(XnSensorAIStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_AI_START, XN_SENSOR_PROTOCOL_RESPONSE_AI_END)
    , m_offset(0)
    , m_minSize(0)
{
}

XnAIProcessor::~XnAIProcessor()
{
}

XnStatus XnAIProcessor::Init()
{
    XnStatus nRetVal = XnFrameStreamProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);

    XnUInt32 bufferSize = GetStream()->GetXRes() * GetStream()->GetYRes() * GetStream()->GetBytesPerPixel();
    XN_VALIDATE_BUFFER_ALLOCATE(m_recvBuf, bufferSize + 1);

    /// All is good.
    return XN_STATUS_OK;
}

#ifdef XN_NEON
XnStatus XnAIProcessor::UnpackRaw10To16(XnUInt16* pDst, const XnUInt8* pSrc, const XnInt32 width, const XnInt32 height)
{
    uint16_t* p = pDst;
    uint8x8_t uint8x8x5[5] = { 0 };
    uint16x8x4_t u16x8x4 = { 0 };
    for (XnInt32 i = 0; i < width * height * 10 / 8; i += 40, p += 32)
    {
        uint8x8x5[0] = vld1_u8(&pSrc[i]);
        uint8x8x5[1] = vld1_u8(&pSrc[i] + 8);
        uint8x8x5[2] = vld1_u8(&pSrc[i] + 16);
        uint8x8x5[3] = vld1_u8(&pSrc[i] + 24);
        uint8x8x5[4] = vld1_u8(&pSrc[i] + 32);

        u16x8x4.val[0][0] = uint8x8x5[0][0] | ((uint8x8x5[0][1] & 0x03) << 8);
        u16x8x4.val[0][1] = (uint8x8x5[0][1] >> 2) | ((uint8x8x5[0][2] & 0x0F) << 6);
        u16x8x4.val[0][2] = (uint8x8x5[0][2] >> 4) | ((uint8x8x5[0][3] & 0x3F) << 4);
        u16x8x4.val[0][3] = (uint8x8x5[0][3] >> 6) | (uint8x8x5[0][4] << 2);
        u16x8x4.val[0][4] = uint8x8x5[0][5] | ((uint8x8x5[0][6] & 0x03) << 8);
        u16x8x4.val[0][5] = (uint8x8x5[0][6] >> 2) | ((uint8x8x5[0][7] & 0x0F) << 6);
        u16x8x4.val[0][6] = (uint8x8x5[0][7] >> 4) | ((uint8x8x5[1][0] & 0x3F) << 4);
        u16x8x4.val[0][7] = (uint8x8x5[1][0] >> 6) | (uint8x8x5[1][1] << 2);

        u16x8x4.val[1][0] = uint8x8x5[1][2] | ((uint8x8x5[1][3] & 0x03) << 8);
        u16x8x4.val[1][1] = (uint8x8x5[1][3] >> 2) | ((uint8x8x5[1][4] & 0x0F) << 6);
        u16x8x4.val[1][2] = (uint8x8x5[1][4] >> 4) | ((uint8x8x5[1][5] & 0x3F) << 4);
        u16x8x4.val[1][3] = (uint8x8x5[1][5] >> 6) | (uint8x8x5[1][6] << 2);
        u16x8x4.val[1][4] = uint8x8x5[1][7] | ((uint8x8x5[2][0] & 0x03) << 8);
        u16x8x4.val[1][5] = (uint8x8x5[2][0] >> 2) | ((uint8x8x5[2][1] & 0x0F) << 6);
        u16x8x4.val[1][6] = (uint8x8x5[2][1] >> 4) | ((uint8x8x5[2][2] & 0x3F) << 4);
        u16x8x4.val[1][7] = (uint8x8x5[2][2] >> 6) | (uint8x8x5[2][3] << 2);

        u16x8x4.val[2][0] = uint8x8x5[2][4] | ((uint8x8x5[2][5] & 0x03) << 8);
        u16x8x4.val[2][1] = (uint8x8x5[2][5] >> 2) | ((uint8x8x5[2][6] & 0x0F) << 6);
        u16x8x4.val[2][2] = (uint8x8x5[2][6] >> 4) | ((uint8x8x5[2][7] & 0x3F) << 4);
        u16x8x4.val[2][3] = (uint8x8x5[2][7] >> 6) | (uint8x8x5[3][0] << 2);
        u16x8x4.val[2][4] = uint8x8x5[3][1] | ((uint8x8x5[3][2] & 0x03) << 8);
        u16x8x4.val[2][5] = (uint8x8x5[3][2] >> 2) | ((uint8x8x5[3][3] & 0x0F) << 6);
        u16x8x4.val[2][6] = (uint8x8x5[3][3] >> 4) | ((uint8x8x5[3][4] & 0x3F) << 4);
        u16x8x4.val[2][7] = (uint8x8x5[3][4] >> 6) | (uint8x8x5[3][5] << 2);

        u16x8x4.val[3][0] = uint8x8x5[3][6] | ((uint8x8x5[3][7] & 0x03) << 8);
        u16x8x4.val[3][1] = (uint8x8x5[3][7] >> 2) | ((uint8x8x5[4][0] & 0x0F) << 6);
        u16x8x4.val[3][2] = (uint8x8x5[4][0] >> 4) | ((uint8x8x5[4][1] & 0x3F) << 4);
        u16x8x4.val[3][3] = (uint8x8x5[4][1] >> 6) | (uint8x8x5[4][2] << 2);
        u16x8x4.val[3][4] = uint8x8x5[4][3] | ((uint8x8x5[4][4] & 0x03) << 8);
        u16x8x4.val[3][5] = (uint8x8x5[4][4] >> 2) | ((uint8x8x5[4][5] & 0x0F) << 6);
        u16x8x4.val[3][6] = (uint8x8x5[4][5] >> 4) | ((uint8x8x5[4][6] & 0x3F) << 4);
        u16x8x4.val[3][7] = (uint8x8x5[4][6] >> 6) | (uint8x8x5[4][7] << 2);

        vst1q_u16(p, u16x8x4.val[0]);
        vst1q_u16(p + 8, u16x8x4.val[1]);
        vst1q_u16(p + 16, u16x8x4.val[2]);
        vst1q_u16(p + 24, u16x8x4.val[3]);
    }

    return XN_STATUS_OK;
}
#else
XnStatus XnAIProcessor::UnpackRaw10To16(XnUInt16* pDst, const XnUInt8* pSrc, const XnInt32 width, const XnInt32 height)
{
    uint16_t *p = pDst;
    for (XnInt32 i = 0; i < width * height * 10 / 8; i += 5, p += 4)
    {
        OniRaw10Packed* item = (OniRaw10Packed*)&pSrc[i];
        p[0] = item->p1 | ((item->p2 & 0x03) << 8);
        p[1] = (item->p2 >> 2) | ((item->p3 & 0x0F) << 6);
        p[2] = (item->p3 >> 4) | ((item->p4 & 0x3F) << 4);
        p[3] = (item->p4 >> 6) | (item->p5 << 2);
    }

    return XN_STATUS_OK;
}
#endif

XnStatus XnAIProcessor::OnPhaseParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

	//if (XN_STATUS_OK != pHeader->status || pHeader->lenght < XN_AI_PHASE_DEFAULT_SIZE)
		if (XN_STATUS_OK != pHeader->status )
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid phase data..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    XnUInt8* pAddr = (XnUInt8*)pFrame + m_offset;
    XnInt32 totalBytes = pHeader->lenght - sizeof(XnUInt32) * 3;
    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);

    pFrame->frameSet.size = 1;
    pFrame->frameSet.status = ONI_AI_STATUS_OK;

    // Note the storage order(@see OniFrameType):
    // 0 - Phase
    // 1 - IR
    // 2 - Depth
    OniTOFFrame &phase = pFrame->frameSet.frames[ONI_FRAME_TYPE_PHASE];
    phase.width = *(XnInt32*)pStart; pStart += sizeof(XnInt32);
    phase.height = *(XnInt32*)pStart; pStart += sizeof(XnInt32);

    XnInt32 nodeStride = *(XnInt32*)pStart; pStart += sizeof(XnInt32);
    phase.planeNum = totalBytes / (nodeStride * (phase.height + 1));

    XnUInt8* pImage = pAddr;
    XnInt32 imageStride = phase.width * sizeof(XnUInt16);
    XnInt32 imageBytes = imageStride * phase.height;
    XnInt32 nodeBytes = nodeStride * phase.height;
    for (XnInt32 i = 0; i < phase.planeNum; ++i)
    {
        ProcessMetadata(&phase.meta[i], pStart);
        pStart += nodeStride;
        UnpackRaw10To16((XnUInt16*)pImage, pStart, phase.width, phase.height);
        pStart += nodeBytes;
        pImage += imageBytes;
    }

    phase.stride = imageStride;
    phase.address = (XnUInt64)pAddr;
    m_offset += imageBytes * phase.planeNum;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnDepthIRParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

    if (XN_STATUS_OK != pHeader->status || pHeader->lenght < XN_AI_VGA_SIZE * 2)
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid frame set..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    XnUInt8* pAddr = (XnUInt8*)pFrame + m_offset;
    XnInt32 totalBytes = pHeader->lenght - sizeof(XnUInt32) * 3;

    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);
    XnInt32 width = *(XnInt32*)pStart; pStart += sizeof(XnInt32);
    XnInt32 height = *(XnInt32*)pStart; pStart += sizeof(XnInt32);
    XnInt32 stride = *(XnInt32*)pStart; pStart += sizeof(XnInt32);
    XnInt32 imageBytes = stride * height / 2;

    OniFrameSet &frameSet = pFrame->frameSet;
    frameSet.status = ONI_AI_STATUS_OK;
    frameSet.size = totalBytes / imageBytes;

    const XnUInt8* pMeta = pStart;
    pStart += stride;

    // Note the storage order(@see OniFrameType):
    // 0 - Phase
    // 1 - IR
    // 2 - Depth
    for (XnInt32 i = ONI_FRAME_TYPE_DEPTH; i >= ONI_FRAME_TYPE_IR; --i)
    {
        frameSet.frames[i].width = width;
        frameSet.frames[i].stride = stride;
        frameSet.frames[i].height = height / 2;
        frameSet.frames[i].planeNum = 1;
        ProcessMetadata(&frameSet.frames[i].meta[0], pMeta);
        frameSet.frames[i].meta[0].type = (OniFrameType)i;

        frameSet.frames[i].address = (XnUInt64)(pAddr + i * imageBytes);
        memcpy((XnUInt8*)frameSet.frames[i].address, pStart, imageBytes);
        pStart += imageBytes;
    }

    m_offset += imageBytes * frameSet.size;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnBodyJointParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

    if (XN_STATUS_OK != pHeader->status || pHeader->lenght < sizeof(OniJoint) * ONI_JOINT_MAX)
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid joint info..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);
    const XnUInt8* pEnd = pStart + pHeader->lenght;

    XnUInt32 width = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    XnUInt32 height = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    XnUInt32 num = *(XnUInt32*)pStart; pStart += sizeof(XnUInt32);
    pFrame->bodyList.num = num;
    if (pFrame->bodyList.num > ONI_MAX_BODIES)
    {
        pFrame->bodyList.num = ONI_MAX_BODIES;
        xnLogWarning(XN_MASK_AI_PROCESSOR, "More than %d skeletons were detected...", num);
    }

    XnUInt32 jointNum = 0;
    XnUInt32 dSize = sizeof(double);
    for (XnInt32 i = 0; i < pFrame->bodyList.num && pStart < pEnd; ++i)
    {
        OniBody* pBody = &pFrame->bodyList.bodies[i];
        pBody->jointWidth = width;
        pBody->jointHeight = height;
        pBody->jointStatus = ONI_AI_STATUS_TRACKING;
        pBody->jointFormat = (OniPixelFormat)pHeader->tag;
        pBody->id = *(XnUInt32*)pStart; pStart += sizeof(XnUInt32);

        jointNum = *(uint16_t*)pStart; pStart += sizeof(uint16_t);
        if (jointNum > ONI_JOINT_MAX)
        {
            jointNum = ONI_JOINT_MAX;
            xnLogWarning(XN_MASK_AI_PROCESSOR, "More than %d joints were detected...", jointNum);
        }

        for (XnUInt32 j = 0; j < jointNum && pStart < pEnd; ++j)
        {
            OniJoint* pJoint = &pBody->joints[j];
            pJoint->type = (OniJointType)j;
            xnOSMemCopy(&pJoint->position.x, pStart, dSize); pStart += dSize;
            xnOSMemCopy(&pJoint->position.y, pStart, dSize); pStart += dSize;
            xnOSMemCopy(&pJoint->position.z, pStart, dSize); pStart += dSize;
            xnOSMemCopy(&pJoint->score, pStart, dSize); pStart += dSize;
        }
    }

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnBodyMaskParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

    if (XN_STATUS_OK != pHeader->status || pHeader->lenght < XN_AI_BODY_MASK_DEFAULT_SIZE)
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid body mask..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    XnUInt8* pDataAddr = (XnUInt8*)pFrame + m_offset;
    XnInt32 bytes = pHeader->lenght - sizeof(XnUInt16) * 2;
    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);

    pFrame->bodyMask.status = ONI_AI_STATUS_TRACKING;
    pFrame->bodyMask.mask.width = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    pFrame->bodyMask.mask.height = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    pFrame->bodyMask.mask.address = (XnUInt64)pDataAddr;
    xnOSMemCopy(pDataAddr, pStart, bytes);
    m_offset += bytes;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnBodyShapeParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

    if (XN_STATUS_OK != pHeader->status || pHeader->lenght < sizeof(XnBodyShape))
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid body shape info..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    XnUInt32 num = pHeader->lenght / sizeof(XnBodyShape);
    pFrame->bodyList.num = num;
    if (pFrame->bodyList.num > ONI_MAX_BODIES)
    {
        pFrame->bodyList.num = ONI_MAX_BODIES;
        xnLogWarning(XN_MASK_AI_PROCESSOR, "More than %d body shape were detected...", num);
    }

    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);
    const XnUInt8* pEnd = pStart + pHeader->lenght;
    for (XnUInt32 i = 0; i < num && pStart < pEnd; ++i)
    {
        XnBodyShape* pShape = (XnBodyShape*)pStart;
        OniBody* pBody = &pFrame->bodyList.bodies[i];
        pBody->bodyShapeStatus = ONI_AI_STATUS_TRACKING;
        pBody->bodyShape.ratio = pShape->ratio;
        pBody->bodyShape.height = pShape->height;
        pBody->bodyShape.waist = pShape->waist;
        pBody->bodyShape.waistline = pShape->waistline;
        pBody->bodyShape.bust = pShape->bust;
        pBody->bodyShape.hips = pShape->hips;
        pBody->bodyShape.shoulder = pShape->shoulder;
        pBody->bodyShape.figure = (OniFigureTypes)pShape->figure;
    }

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnFloorInfoParse(OniAIFrame* pFrame, const XnTTLVHeader* pHeader)
{
    XN_RET_IF_NULL(pFrame, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pHeader, XN_STATUS_NULL_INPUT_PTR);

    if (XN_STATUS_OK != pHeader->status || pHeader->lenght < XN_AI_FLOOR_MASK_DEFAULT_SIZE)
    {
        xnLogWarning(XN_MASK_AI_PROCESSOR, "Invalid floor plane info..., status code (%d)", pHeader->status);
        return XN_STATUS_ERROR;
    }

    XnUInt8* pDataAddr = (XnUInt8*)pFrame + m_offset;
    const XnUInt8* pStart = (XnUInt8*)pHeader + sizeof(XnTTLVHeader);
    XnInt32 bytes = pHeader->lenght - sizeof(OniPoint3D<float>) * 2 - sizeof(XnUInt32) * 2;

    pFrame->floorInfo.status = ONI_AI_STATUS_TRACKING;
    pFrame->floorInfo.plane.center = *(OniPoint3D<float>*)pStart; pStart += sizeof(OniPoint3D<float>);
    pFrame->floorInfo.plane.normal = *(OniPoint3D<float>*)pStart; pStart += sizeof(OniPoint3D<float>);
    pFrame->floorInfo.mask.address = (uint64_t)pDataAddr;
    pFrame->floorInfo.mask.width = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    pFrame->floorInfo.mask.height = *(XnUInt16*)pStart; pStart += sizeof(XnUInt16);
    xnOSMemCopy(pDataAddr, pStart, bytes);
    m_offset += bytes;

    /// All is good.
    return XN_STATUS_OK;
}

XnStatus XnAIProcessor::OnProtocolParse(XnBuffer* pDstBuf, XnBuffer* pSrcBuf)
{
    XN_RET_IF_NULL(pSrcBuf, XN_STATUS_NULL_INPUT_PTR);
    XN_RET_IF_NULL(pDstBuf, XN_STATUS_NULL_INPUT_PTR);

    xnOSMemSet(pDstBuf->GetData(), 0, pDstBuf->GetMaxSize());
    OniAIFrame* pFrame = (OniAIFrame*)pDstBuf->GetData();

    m_offset = sizeof(OniAIFrame);
    const XnUInt8* pStart = pSrcBuf->GetData();
    const XnUInt8* pEnd = pStart + pSrcBuf->GetSize();
    while (pStart < pEnd)
    {
        XnStatus ret = XN_STATUS_ERROR;
        XnTTLVHeader* pHeader = (XnTTLVHeader*)pStart;
        switch (pHeader->tag)
        {
        case XN_IO_AI_FORMAT_JOINT_2D:
        case XN_IO_AI_FORMAT_JOINT_3D:
            ret = OnBodyJointParse(pFrame, pHeader);
            break;
        case XN_IO_AI_FORMAT_BODY_MASK:
            ret = OnBodyMaskParse(pFrame, pHeader);
            break;
        case XN_IO_AI_FORMAT_BODY_SHAPE:
            ret = OnBodyShapeParse(pFrame, pHeader);
            break;
        case XN_IO_AI_FORMAT_FLOOR_INFO:
            ret = OnFloorInfoParse(pFrame, pHeader);
            break;
        case XN_IO_AI_FORMAT_PHASE:
            ret = OnPhaseParse(pFrame, pHeader);
            break;
        case XN_IO_AI_FORMAT_DEPTH_IR:
            ret = OnDepthIRParse(pFrame, pHeader);
            break;
        }

        if (XN_STATUS_OK != ret)
            ++pStart;
        else
            pStart += (sizeof(XnTTLVHeader) + pHeader->lenght);
    }

    /// All is good.
    pDstBuf->UnsafeUpdateSize(pSrcBuf->GetSize());
    return XN_STATUS_OK;
}

void XnAIProcessor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
    XN_PROFILING_START_SECTION(XN_MASK_SENSOR_PROTOCOL_AI)

    /// Check if there is enough room to receive packet data.
    if (m_recvBuf.GetFreeSpaceInBuffer() >= nDataSize)
    {
        /// Copy values. Make sure we do not get corrupted pixels.
        m_recvBuf.UnsafeWrite(pData, nDataSize);

        /// All is good.
        return;
    }

    /// Not expected...
    xnLogWarning(XN_MASK_AI_PROCESSOR, "Bad overflow AI: size = %d, maxSize = %d", m_recvBuf.GetSize(), m_recvBuf.GetMaxSize());
    FrameIsCorrupted();
    m_recvBuf.Reset();

    XN_PROFILING_END_SECTION
}

void XnAIProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    XN_PROFILING_START_SECTION(XN_MASK_SENSOR_PROTOCOL_AI)

    OniPixelFormat format = GetStream()->GetOutputFormat();
    switch (format)
    {
    case ONI_PIXEL_FORMAT_JOINT_2D:
    case ONI_PIXEL_FORMAT_BODY_MASK:
    case ONI_PIXEL_FORMAT_FLOOR_INFO:
    case ONI_PIXEL_FORMAT_JOINT_3D:
    case ONI_PIXEL_FORMAT_BODY_SHAPE:
    case ONI_PIXEL_FORMAT_PHASE:
    case ONI_PIXEL_FORMAT_DEPTH_IR:
        OnProtocolParse(GetWriteBuffer(), &m_recvBuf);
        break;
    default:
        FrameIsCorrupted();
        xnLogError(XN_MASK_AI_PROCESSOR, "Not supported ouput format (%d)", format);
    }

    OniFrame* pFrame = GetWriteFrame();
    pFrame->sensorType = ONI_SENSOR_AI;
    pFrame->width = GetStream()->GetXRes();
    pFrame->height = GetStream()->GetYRes();
    pFrame->videoMode.fps = GetStream()->GetFPS();
    pFrame->extraLine = GetStream()->GetMetadataLine();
    pFrame->videoMode.resolutionX = GetStream()->GetXRes();
    pFrame->videoMode.resolutionY = GetStream()->GetYRes();
    pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();
    pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

    m_recvBuf.Reset();

    /// Call base.
    XnFrameStreamProcessor::OnEndOfFrame(pHeader);

    XN_PROFILING_END_SECTION
}

void XnAIProcessor::OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS)
{
    XnFrameStreamProcessor::OnFrameReady(nFrameID, nFrameTS);
    m_pDevicePrivateData->pSensor->GetFPSCalculator()->MarkBody(nFrameID, nFrameTS);
}
