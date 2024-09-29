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
#include <fstream>
#include "XnDepthProcessor.h"
#include "XnSensor.h"
#include <XnDevice.h>
#include <XnProfiling.h>
#include <XnLog.h>
#include "OniOrbbecTypes.hpp"
#include <cstdlib>
#include "depthOptimization.h"

#ifndef _DEBUG
#if (XN_PLATFORM == XN_PLATFORM_WIN32 || XN_PLATFORM == XN_PLATFORM_LINUX_X86 || XN_PLATFORM == XN_PLATFORM_LINUX_ARM)
#include <softlib.h>
#endif

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#include <softfilter-window.h>
#else 
#include <softfilter.h>
#endif

#endif

using namespace std;

#define ATLAS_DEPTH_ROTATE 1
#define MUL_DISTACNE_PARAM_SIZE 1024*1024

//filter algorithm
typedef enum
{
    NO_FILTER = 0,		//not use filter
    SOFT_FILTER = 1,    //old soft filter
    POST_FILTER = 2,	//new soft filter
}FilterAlg;
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
XnDepthProcessor::XnDepthProcessor(XnSensorDepthStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager)
    : XnFrameStreamProcessor(pStream, pHelper, pBufferManager, XN_SENSOR_PROTOCOL_RESPONSE_DEPTH_START, XN_SENSOR_PROTOCOL_RESPONSE_DEPTH_END)
    , m_PixelC2DPropery(XN_STREAM_PROPERTY_PIXEL_C2D_REGISTRATION, "C2DPixelRegistration")
    , m_PixelD2CPropery(XN_STREAM_PROPERTY_PIXEL_D2C_REGISTRATION, "D2CPixelRegistration")
    , m_nPaddingPixelsOnEnd(0)
    , m_applyRegistrationOnEnd(FALSE)
    , m_nExpectedFrameSize(0)
    , m_bShiftToDepthAllocated(FALSE)
    , m_bShiftOrDepth(FALSE)
    , m_pShiftToDepthTable(pStream->GetShiftToDepthTable())
    , pDepthToShiftTable_org(pStream->GetDepthToShiftTable())
{
    DepthBuf = new OniDepthPixel[1328 * 1120](); //max resolution

    //#if (XN_PLATFORM == XN_PLATFORM_WIN32 || XN_PLATFORM == XN_PLATFORM_LINUX_X86 || XN_PLATFORM == XN_PLATFORM_LINUX_ARM)
    size_t bufSize = 1328 * 1120 * (int)(sizeof(int) + sizeof(int) + sizeof(char));
    _buf = new unsigned char[bufSize]();
    m_maxDiff = 16;

    //#endif
    tag_Buf = new unsigned char[16]();
    m_filterCache = NULL;
    config.nZeroPlaneDistance = 120;
    config.fZeroPlanePixelSize = 0.10419999808073044;
    config.fEmitterDCmosDistance = 7.5;
    config.nDeviceMaxShiftValue = 2047;

    config.nDeviceMaxDepthValue = 10000;

    config.nConstShift = 200;
    config.nPixelSizeFactor = 1;
    config.nParamCoeff = 4;
    config.nShiftScale = 10;

    config.nDepthMinCutOff = 0;
    config.nDepthMaxCutOff = static_cast<short>(config.nDeviceMaxDepthValue);

    unsigned short nIndex = 0;
    short  nShiftValue = 0;
    double dFixedRefX = 0;
    double dMetric = 0;
    double dDepth = 0;
    double dPlanePixelSize = config.fZeroPlanePixelSize;
    double dPlaneDsr = config.nZeroPlaneDistance;
    double dPlaneDcl = config.fEmitterDCmosDistance;
    int nConstShift = config.nParamCoeff * config.nConstShift;

    dPlanePixelSize *= config.nPixelSizeFactor;
    nConstShift /= config.nPixelSizeFactor;



    //	XN_VALIDATE_ALIGNED_CALLOC(m_ShiftToDepth.pShiftToDepthTable, XnDepthPixel, config.nDeviceMaxShiftValue+1, XN_DEFAULT_MEM_ALIGN);
    //	XN_VALIDATE_ALIGNED_CALLOC(m_ShiftToDepth.pDepthToShiftTable, XnUInt16, config.nDeviceMaxDepthValue+1, XN_DEFAULT_MEM_ALIGN);

    //	m_ShiftToDepth.pShiftToDepthTable=(unsigned short*)malloc( (config.nDeviceMaxShiftValue+1)*sizeof(unsigned short));
    //	m_ShiftToDepth.pDepthToShiftTable=(unsigned short*)malloc( (config.nDeviceMaxDepthValue+1)*sizeof(unsigned short));

    m_ShiftToDepth.pShiftToDepthTable = new unsigned short[config.nDeviceMaxShiftValue + 1]();
    m_ShiftToDepth.pDepthToShiftTable = new unsigned short[config.nDeviceMaxDepthValue + 1]();


    m_ShiftToDepth.bIsInitialized = TRUE;

    // store allocation sizes
    m_ShiftToDepth.nShiftsCount = config.nDeviceMaxShiftValue + 1;
    m_ShiftToDepth.nDepthsCount = config.nDeviceMaxDepthValue + 1;
    unsigned short* pShiftToDepthTable = m_ShiftToDepth.pShiftToDepthTable;
    unsigned short* pDepthToShiftTable = m_ShiftToDepth.pDepthToShiftTable;

    memset(pShiftToDepthTable, 0, m_ShiftToDepth.nShiftsCount * sizeof(unsigned short));
    memset(pDepthToShiftTable, 0, m_ShiftToDepth.nDepthsCount * sizeof(unsigned short));

    unsigned short nLastDepth = 0;
    unsigned short nLastIndex = 0;

    for (nIndex = 1; nIndex < config.nDeviceMaxShiftValue; nIndex++)
    {
        nShiftValue = nIndex;

        dFixedRefX = (double)(nShiftValue - nConstShift) / (double)config.nParamCoeff;
        dFixedRefX -= 0.375;
        dMetric = dFixedRefX * dPlanePixelSize;
        dDepth = config.nShiftScale * ((dMetric * dPlaneDsr / (dPlaneDcl - dMetric)) + dPlaneDsr);

        // check cut-offs
        if ((dDepth > config.nDepthMinCutOff) && (dDepth < config.nDepthMaxCutOff))
        {
            pShiftToDepthTable[nIndex] = (unsigned short)dDepth;


            for (unsigned short i = nLastDepth; i < dDepth; i++)
                pDepthToShiftTable[i] = nLastIndex;

            nLastIndex = nIndex;
            nLastDepth = (unsigned short)dDepth;

            // std::cout<<nIndex<<" "<<dDepth<<endl;
        }
    }

    for (unsigned short i = nLastDepth; i <= config.nDeviceMaxDepthValue; i++){
        pDepthToShiftTable[i] = nLastIndex;
        //std::cout<<pDepthToShiftTable[i] <<" "<<i<<endl;

    }


    m_auh = NULL;
    m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;

}

XnDepthProcessor::~XnDepthProcessor()
{
    if (m_bShiftToDepthAllocated)
    {
        xnOSFree(m_pShiftToDepthTable);
        m_pShiftToDepthTable = NULL;
    }

    if (DepthBuf != NULL)
    {
        delete[] DepthBuf;
        DepthBuf = NULL;
    }

    if (m_ShiftToDepth.pShiftToDepthTable != NULL)
    {
        delete[]m_ShiftToDepth.pShiftToDepthTable;
        m_ShiftToDepth.pShiftToDepthTable = NULL;
    }

    if (m_ShiftToDepth.pDepthToShiftTable != NULL)
    {
        delete[]m_ShiftToDepth.pDepthToShiftTable;
        m_ShiftToDepth.pDepthToShiftTable = NULL;
    }

    //#if (XN_PLATFORM == XN_PLATFORM_WIN32 || XN_PLATFORM == XN_PLATFORM_LINUX_X86 || XN_PLATFORM == XN_PLATFORM_LINUX_ARM)
    if (_buf != NULL)
    {
        delete[]  _buf;
        _buf = NULL;
    }
    //#endif


    if (m_auh != NULL)
    {
        DestroyApplyUndistHandle(&m_auh);
    }

    if (tag_Buf != NULL)
    {
        delete[] tag_Buf;
        tag_Buf = NULL;
    }
}

#define MAX_SHIFT_TO_DEPTH_TABLE 4096


XnStatus XnDepthProcessor::Init()
{
    XnStatus nRetVal = XN_STATUS_OK;

    // init base
    nRetVal = XnFrameStreamProcessor::Init();
    XN_IS_STATUS_OK(nRetVal);


    m_bMultiDisCalEnable = GetStreamHelper()->GetPrivateData()->pSensor->GetMultiDisCalEnable();
    xnLogVerbose(XN_MASK_SENSOR_READ, "Read flash distortion enable state %d", m_bMultiDisCalEnable);
    if (m_bMultiDisCalEnable == MULTI_DISCAL_ENABLE)
    {
        //Create Distortion processor
        m_auh = CreateApplyUndistHandle();

        XnDistortionParam distortionParam;
        XnStatus rcState = GetStreamHelper()->GetPrivateData()->pSensor->GetDistortionParam(distortionParam);

        if (XN_STATUS_OK == rcState)
        {
            //Input zip buffer
            rcState = ReadBinaryFile(m_auh, (const char*)(distortionParam.data), distortionParam.nSize);
            //xnOSSaveFile("D:\\v1.bin", distortionParam.data, distortionParam.nSize);
            xnLogVerbose(XN_MASK_SENSOR_READ, "Distortion param readBinaryFile end!");
            if (XN_STATUS_OK == rcState)
            {
                //get depth resolution
                XnResolutions nResolutions = (XnResolutions)GetStream()->GetResolution();
                XnUInt32 nWidth = 0;
                XnUInt32 nHeight = 0;
                XnBool bOk = XnDDKGetXYFromResolution(nResolutions, &nWidth, &nHeight);
                if (bOk && undistortion(m_auh, nWidth, nHeight))
                {
                    m_bMultiDisCalEnable = MULTI_DISCAL_ENABLE;
                }
                else
                {
                    m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;
                    xnLogWarning(XN_MASK_SENSOR_READ, "undistortion resolution unsupport");
                }
            }
            else
            {
                m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;
                xnLogWarning(XN_MASK_SENSOR_READ, "ReadBinaryFile failed");
            }
        }
        else
        {
            m_bMultiDisCalEnable = MULTI_DISCAL_DISABLE;
            xnLogWarning(XN_MASK_SENSOR_READ, "ReadFlashDistortionParam failed");
        }
    }

    xnLogVerbose(XN_MASK_SENSOR_READ, "Multi distance calibration enable %d", m_bMultiDisCalEnable);
    //////////////////////////////////////////////////////////////////////////////////////////
    //new softfilter
    XnResolutions nResolutions = (XnResolutions)GetStream()->GetResolution();
    XnUInt32 nWidth = 0;
    XnUInt32 nHeight = 0;
    XnBool bOk = XnDDKGetXYFromResolution(nResolutions, &nWidth, &nHeight);
    if (nWidth == 1280 || nHeight == 1280)
    {
        m_maxSpeckleSize = 1920;
    }
    else if (nWidth == 640 || nHeight == 640)
    {
        m_maxSpeckleSize = 480;
    }
    else if (nWidth == 320 || nHeight == 320)
    {
        m_maxSpeckleSize = 120;
    }
    else if (nWidth == 160 || nHeight == 160)
    {
        m_maxSpeckleSize = 30;
    }
    else
    {
        m_maxSpeckleSize = 480;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////

    m_MX6000Id = GetStreamHelper()->GetPrivateData()->ChipInfo.nChipVer;

    if (XN_SENSOR_CHIP_VER_DUAL_MX6000 == m_MX6000Id){
        m_pShiftToDepthTable = (OniDepthPixel*)xnOSMalloc(sizeof(OniDepthPixel)*MAX_SHIFT_TO_DEPTH_TABLE);
        xnOSMemSet(m_pShiftToDepthTable, 0, MAX_SHIFT_TO_DEPTH_TABLE);
        m_bShiftToDepthAllocated = TRUE;
        ObContent_t params;
        xnOSMemSet(&params.HOST.virCam, 0, sizeof(params.HOST.virCam));
        //int nContentSize = sizeof(params);

        nRetVal = GetStreamHelper()->GetPrivateData()->pSensor->GetDualCameraParam(params);
        if (nRetVal != XN_STATUS_OK)
        {
            m_paramRead = false;
            xnLogError(XN_MASK_SENSOR_READ, "Read sensor params failed");
        }
        else
        {
            //
            int datasize = sizeof(m_CamParams);
            xnOSMemSet(&m_CamParams, 0, datasize);
            m_paramRead = true;
            //
            xnOSMemCopy(&m_CamParams, &params.HOST.soft_d2c, datasize);
            if (m_softRegistrator.IsNaN(m_CamParams.l_intr_p[0]) || m_softRegistrator.IsNaN(m_CamParams.l_intr_p[1])){
                xnLogWarning(XN_MASK_SENSOR_READ, "Sensor params invalid(NaN value)\n");
                m_paramRead = false;
            }
            else{
                m_softRegistrator.Init(m_CamParams);
            }
        }

        double forcalllength = params.HOST.virCam.fx;
        double baseLine = params.HOST.virCam.bl;
        double fCoefficient = (double)GetStream()->ParamCoefficientProperty().GetValue();
        double disparityCoeff = GetStream()->DualCoeffDisparityProperty().GetValue();
        double fb = baseLine * forcalllength;


        int nShiftScale = 1;
        if (GetStream()->GetOutputFormat() == ONI_PIXEL_FORMAT_DEPTH_100_UM)
        {
            nShiftScale = 10;
        }

        xnLogVerbose(XN_MASK_SENSOR_READ, "Read forcalllength: %f,baseline: %f, fbcoeff: %f, fittingCoeff: %f ,fCoefficient: %f,nShiftScale: %d",
            forcalllength, baseLine, fb, disparityCoeff, fCoefficient, nShiftScale);

        XnDouble dDepth = 0;
        XnUInt32 nMinDepthValue = GetStream()->GetMinDepth();
        nMinDepthValue = nMinDepthValue*nShiftScale;

        XnUInt32 nMaxDepthValue = GetStream()->GetMaxDepth();
        nMaxDepthValue = nMaxDepthValue*nShiftScale;
        XnUInt32 nDeviceMaxDepth = GetStream()->GetDeviceMaxDepth();

        nMaxDepthValue = XN_MIN(nDeviceMaxDepth, nMaxDepthValue);

        for (uint16_t i = 0; i < 256 * 16; i++){
#if 1
            //openni
            double disparityArg = (disparityCoeff - i) / fCoefficient;
            if (disparityArg < 255 && disparityArg > 0)
            {
                dDepth = (nShiftScale)*(fb / disparityArg);
                if (dDepth > nMinDepthValue && dDepth < nMaxDepthValue)
                {
                    m_pShiftToDepthTable[i] = (OniDepthPixel)dDepth;
                }
                else
                {
                    m_pShiftToDepthTable[i] = 0;
                }
            }
            else
            {
                m_pShiftToDepthTable[i] = 0;
            }
#else
            //original 12bit
            double disparityArg = ((i + 128 * 16) % 4096)*0.0625;
            if (disparityArg != 128)
            {
                dDepth = (nShiftScale)*(fb / disparityArg);
                if (dDepth >nMinDepthValue && dDepth <nMaxDepthValue)
                {
                    m_pShiftToDepthTable[i] = (OniDepthPixel)dDepth;
                }
                else
                {
                    m_pShiftToDepthTable[i] = 0;
                }
            }
            else
            {
                m_pShiftToDepthTable[i] = 0;
            }
#endif
        }

    }
    else
    {
        nRetVal = GetStreamHelper()->GetPrivateData()->pSensor->GetCameraParam(m_CamParams);

        if (XN_STATUS_OK != nRetVal){
            m_paramRead = false;
            xnLogError(XN_MASK_SENSOR_READ, "Read sensor params failed");
        }
        else{

            m_paramRead = true;
            if (m_softRegistrator.IsNaN(m_CamParams.l_intr_p[0]) || m_softRegistrator.IsNaN(m_CamParams.l_intr_p[1])){
                xnLogWarning(XN_MASK_SENSOR_READ, "Sensor params invalid(NaN value)\n");
                m_paramRead = false;
            }
            else{
                m_softRegistrator.Init(m_CamParams);
            }
        }
    }


    switch (GetStream()->GetOutputFormat())
    {
    case ONI_PIXEL_FORMAT_SHIFT_9_2:
    {
        if (m_bShiftToDepthAllocated)
        {
            xnOSFree(m_pShiftToDepthTable);
            m_pShiftToDepthTable = NULL;
            m_bShiftToDepthAllocated = FALSE;
        }
        // optimization. We create a LUT shift-to-shift. See comment up.
        XnUInt32 nMaxShift = GetStream()->GetMaxShift();
        m_pShiftToDepthTable = (OniDepthPixel*)xnOSMalloc(sizeof(OniDepthPixel)*(nMaxShift + 1));
        XN_VALIDATE_ALLOC_PTR(m_pShiftToDepthTable);
        for (XnUInt32 i = 0; i < (nMaxShift + 1); ++i)
        {
            m_pShiftToDepthTable[i] = (OniDepthPixel)i;
        }
        m_bShiftToDepthAllocated = TRUE;
        m_bShiftOrDepth = TRUE;
        m_noDepthValue = nMaxShift;
    }
    break;
    case ONI_PIXEL_FORMAT_DEPTH_1_MM:
    case ONI_PIXEL_FORMAT_DEPTH_100_UM:
        m_noDepthValue = 0;
        break;
    default:
        XN_ASSERT(FALSE);
        XN_LOG_WARNING_RETURN(XN_STATUS_ERROR, XN_MASK_SENSOR_PROTOCOL_DEPTH, "Unknown Depth output: %d", GetStream()->GetOutputFormat());
    }


    m_PixelC2DPropery.UpdateGetCallback(GetDepthCoordinatesOfC2DCallback, this);
    m_PixelD2CPropery.UpdateGetCallback(GetColorCoordinatesOfD2CCallback, this);
    XnProperty* _aProps[] = { &m_PixelC2DPropery, &m_PixelD2CPropery };
    GetStream()->AddProperties(_aProps, sizeof(_aProps) / sizeof(XnProperty*));

    return (XN_STATUS_OK);
}

void XnDepthProcessor::OnStartOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    // call base
    XnFrameStreamProcessor::OnStartOfFrame(pHeader);

    m_nExpectedFrameSize = CalculateExpectedSize();

    m_applyRegistrationOnEnd = (
        (GetStream()->GetOutputFormat() == ONI_PIXEL_FORMAT_DEPTH_1_MM || GetStream()->GetOutputFormat() == ONI_PIXEL_FORMAT_DEPTH_100_UM) &&
        GetStream()->m_DepthRegistration.GetValue() == TRUE &&
        GetStream()->m_FirmwareRegistration.GetValue() == FALSE);

    if (m_pDevicePrivateData->FWInfo.nFWVer >= XN_SENSOR_FW_VER_5_1 && pHeader->nTimeStamp != 0)
    {
        // PATCH: starting with v5.1, the timestamp field of the SOF packet, is the number of pixels
        // that should be prepended to the frame.
        XnUInt32 nPaddingPixelsOnStart = pHeader->nTimeStamp >> 16;
        m_nPaddingPixelsOnEnd = pHeader->nTimeStamp & 0x0000FFFF;

        PadPixels(nPaddingPixelsOnStart);
    }
}

XnUInt32 XnDepthProcessor::CalculateExpectedSize()
{
    XnUInt32 nExpectedDepthBufferSize = GetStream()->GetXRes() * GetStream()->GetYRes();

    // when cropping is turned on, actual depth size is smaller
    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
        nExpectedDepthBufferSize = (XnUInt32)(GetStream()->m_FirmwareCropSizeX.GetValue() * GetStream()->m_FirmwareCropSizeY.GetValue());

    nExpectedDepthBufferSize = nExpectedDepthBufferSize * GetStream()->GetBytesPerPixel() +
        GetStream()->GetXRes() * GetStream()->GetBytesPerPixel() * GetStream()->GetMetadataLine();

    return nExpectedDepthBufferSize;
}

void XnDepthProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
    //xnLogVerbose(XN_MASK_SENSOR_READ, "OnEndOfFrame----------------------------start");
    // pad pixels
    if (m_nPaddingPixelsOnEnd != 0)
    {
        PadPixels(m_nPaddingPixelsOnEnd);
        m_nPaddingPixelsOnEnd = 0;
    }

    bool frameIsCorrupted = false;
    if (GetWriteBuffer()->GetSize() != GetExpectedSize())
    {
        xnLogWarning(XN_MASK_SENSOR_READ, "Read: Depth buffer is corrupt. Size is %u (!= %u)", GetWriteBuffer()->GetSize(), GetExpectedSize());
        FrameIsCorrupted();
        frameIsCorrupted = true;
    }
    else
    {
        if (m_applyRegistrationOnEnd)
        {
            GetStream()->ApplyRegistration((OniDepthPixel*)GetWriteBuffer()->GetData());
        }
    }

    OniFrame* pFrame = GetWriteFrame();
    pFrame->sensorType = ONI_SENSOR_DEPTH;
    pFrame->videoMode.fps = GetStream()->GetFPS();
    pFrame->extraLine = GetStream()->GetMetadataLine();
    pFrame->videoMode.resolutionX = GetStream()->GetXRes();
    pFrame->videoMode.resolutionY = GetStream()->GetYRes();
    pFrame->videoMode.pixelFormat = GetStream()->GetOutputFormat();
    if (GetStream()->m_FirmwareCropMode.GetValue() != XN_FIRMWARE_CROPPING_MODE_DISABLED)
    {
        pFrame->width = (int)GetStream()->m_FirmwareCropSizeX.GetValue();
        pFrame->height = (int)GetStream()->m_FirmwareCropSizeY.GetValue();
        pFrame->cropOriginX = (int)GetStream()->m_FirmwareCropOffsetX.GetValue();
        pFrame->cropOriginY = (int)GetStream()->m_FirmwareCropOffsetY.GetValue();
        pFrame->croppingEnabled = TRUE;
    }
    else
    {
        pFrame->width = pFrame->videoMode.resolutionX;
        pFrame->height = pFrame->videoMode.resolutionY;
        pFrame->cropOriginX = 0;
        pFrame->cropOriginY = 0;
        pFrame->croppingEnabled = FALSE;
    }
    pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();

    //Tornado-D2 Rotate depth,save tag data
    XnUInt16 nPid = GetStreamHelper()->GetPrivateData()->pSensor->GetDevicePID();
    if (nPid == Tornado_D2_PID || nPid == Tornado_D2_PRO_PID)
    {
        xnOSMemCopy(tag_Buf, pFrame->data, 16);
    }

    //softfilter
    OniDepthPixel* pDepth = (OniDepthPixel*)((XnUInt8*)pFrame->data + pFrame->stride *  pFrame->extraLine);
    xnOSMemCopy(DepthBuf, pDepth, pFrame->width*pFrame->height*sizeof(OniDepthPixel));

#ifndef _DEBUG
    XnUInt32 bFilter = (XnUInt32)GetStream()->getSoftFilterMode();
    if (bFilter == POST_FILTER)
    {
        XnInt nMaxDiff = (XnInt)GetStream()->getDepthMaxDiff();
        if (nMaxDiff != 0)
        {
            m_maxDiff = nMaxDiff;
        }

        XnInt nMaxSpeckleSize = (XnInt)GetStream()->getMaxSpeckleSize();
        if (nMaxSpeckleSize != 0)
        {
            m_maxSpeckleSize = nMaxSpeckleSize;
        }

        softfilter(_buf, (unsigned short*)(DepthBuf), pFrame->width, pFrame->height, m_maxDiff, m_maxSpeckleSize, 0);
    }
    else if (bFilter == SOFT_FILTER)
    {
#if (XN_PLATFORM == XN_PLATFORM_WIN32 || XN_PLATFORM == XN_PLATFORM_LINUX_X86 || XN_PLATFORM == XN_PLATFORM_LINUX_ARM)
        Softfilter(_buf, (unsigned short*)(DepthBuf), pFrame->width, pFrame->height);
#endif
    }
#endif

    if (!frameIsCorrupted) {
        int index = 0;
        int isRegist = GetStream()->getSoftwareRegistratorMode();
        if (isRegist && m_paramRead) {
            memset(pFrame->data, 0, pFrame->width * pFrame->height * sizeof(OniDepthPixel));
            for (int v = 0; v < pFrame->height; v++) {
                for (int u = 0; u < pFrame->width; u++) {
                    OniDepthPixel depthValue = m_pShiftToDepthTable[DepthBuf[index]];
                    index++;
                    if (0 == depthValue) {
                        pDepth[index] = 0;
                        continue;
                    }

                    if (!(m_MX6000Id == XN_SENSOR_CHIP_VER_MX6000 || m_MX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)) {
                        if (5506 == pDepth[index] || 288 == pDepth[index]) {
                            pDepth[index] = 0;
                            continue;
                        }
                    }
                    m_softRegistrator.MappingDepth2Color(*pFrame, u, v, depthValue, (isRegist == XN_PARAMS_REGISTRATION_USE_DISTORTION));
                }
            }
        }
        else {
            int nMaxShift = GetStream()->GetMaxShift();
            for (int i = 0; i < pFrame->height; i++) {
                for (int j = 0; j < pFrame->width; j++) {
                    if (DepthBuf[index] > nMaxShift)
                    {
                        pDepth[index] = 0;
                        index++;
                        continue;
                    }
                    pDepth[index] = m_pShiftToDepthTable[DepthBuf[index]];
                    if (!m_bShiftOrDepth)
                    {
                        //only depth data do multi distance calibration
                        if (pDepth[index] != 0 && m_bMultiDisCalEnable == MULTI_DISCAL_ENABLE)
                        {
                            //add
                            XnFloat d_depth = (XnFloat)pDepth[index], ud_depth = 0;
                            if (XN_SENSOR_CHIP_VER_DUAL_MX6000 == m_MX6000Id)
                            {
                                GetUndistortionDepthStereo(m_auh, j, i, d_depth, ud_depth);
                            }
                            else
                            {
                                GetUndistortionDepthMonocular(m_auh, j, i, d_depth, &ud_depth);
                            }
                            pDepth[index] = (OniDepthPixel)ud_depth;
                            ///////////////////////////////////////
                        }
                    }
#if 0
                    if (pDepth[index] <= 150){
                        pDepth[index] = 0;
                    }
#endif
                    if (!(m_MX6000Id == XN_SENSOR_CHIP_VER_MX6000 || m_MX6000Id == XN_SENSOR_CHIP_VER_DUAL_MX6000)) {
                        if (pDepth[index] == 5506 || pDepth[index] == 288) {
                            pDepth[index] = 0;
                        }
                    }

                    index++;
                }
            }
        }
    }

    //Tornado-D2 Rotate depth
    int nDepthRotate = GetStream()->getDepthRotateMode();
    if (nPid == Tornado_D2_PID || nPid == Tornado_D2_PRO_PID || ((nPid == BAIDU_Atlas) && nDepthRotate == ATLAS_DEPTH_ROTATE))
    {
        xnOSMemCopy(DepthBuf, pDepth, pFrame->width*pFrame->height*sizeof(OniDepthPixel));
        Channel2Rotate90((unsigned char*)DepthBuf, (pFrame->width * 2), (unsigned char *)(pFrame->data), (pFrame->height * 2), pFrame->width, pFrame->height);

        int nTempResolution = pFrame->videoMode.resolutionX;
        pFrame->videoMode.resolutionX = pFrame->videoMode.resolutionY;
        pFrame->videoMode.resolutionY = nTempResolution;


        int tempW = pFrame->width;
        pFrame->width = pFrame->height;
        pFrame->height = tempW;
        pFrame->stride = pFrame->width * GetStream()->GetBytesPerPixel();
    }

    if (nPid == Tornado_D2_PID || nPid == Tornado_D2_PRO_PID)
    {
        xnOSMemCopy((char*)pFrame->data, tag_Buf, 16);
    }

    // call base
    XnFrameStreamProcessor::OnEndOfFrame(pHeader);

    //xnLogVerbose(XN_MASK_SENSOR_READ, "OnEndOfFrame----------------------------end");
}

// ADD by ZW
void XnDepthProcessor::CheckIgnoreEOF()
{
    if (GetWriteBuffer()->GetSize() == GetExpectedSize())
    {
        m_bIgnoreEOF = TRUE;
    }
    return;
}
// ADD by ZW end

void XnDepthProcessor::PadPixels(XnUInt32 nPixels)
{
    XnBuffer* pWriteBuffer = GetWriteBuffer();

    // check for overflow
    if (!CheckWriteBufferForOverflow(nPixels * sizeof(OniDepthPixel)))
    {
        return;
    }

    OniDepthPixel* pDepth = (OniDepthPixel*)GetWriteBuffer()->GetUnsafeWritePointer();

    // place the no-depth value
    for (XnUInt32 i = 0; i < nPixels; ++i, ++pDepth)
    {
        *pDepth = m_noDepthValue;
    }
    pWriteBuffer->UnsafeUpdateSize(nPixels * sizeof(OniDepthPixel));
}

void XnDepthProcessor::OnFrameReady(XnUInt32 nFrameID, XnUInt64 nFrameTS)
{
    XnFrameStreamProcessor::OnFrameReady(nFrameID, nFrameTS);

    m_pDevicePrivateData->pSensor->GetFPSCalculator()->MarkDepth(nFrameID, nFrameTS);
}


XnStatus XnDepthProcessor::GetDepthCoordinatesOfC2D(XnUInt32 x, XnUInt32 y, OniDepthPixel z, XnUInt32 imageXRes, XnUInt32 imageYRes, XnUInt32& depthX, XnUInt32& depthY)
{
    XnStatus ret = XN_STATUS_OK;
    if (!m_paramRead){
        return XN_STATUS_ERROR;
    }
    int dx = 0;
    int dy = 0;
    int isRegist = GetStream()->getSoftwareRegistratorMode();
    m_softRegistrator.CoordinateConverterColorToDepth((int)x, (int)y, z, dx, dy, (isRegist == XN_PARAMS_REGISTRATION_USE_DISTORTION));
#if 0
    if(dy < 0 || dy > (int)imageYRes || dx < 0 || dx > (int)imageXRes){
        return XN_STATUS_ERROR;
    }
#endif
    depthX = dx;
    depthY = dy;

    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnDepthProcessor::GetDepthCoordinatesOfC2DCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XnDepthProcessor* pThis = (XnDepthProcessor*)pCookie;

    if (gbValue.dataSize != sizeof(XnPixelRegistration))
    {
        return XN_STATUS_DEVICE_PROPERTY_SIZE_DONT_MATCH;
    }

    XnPixelRegistration* pArgs = (XnPixelRegistration*)gbValue.data;

    return pThis->GetDepthCoordinatesOfC2D(pArgs->nImageX, pArgs->nImageY, pArgs->nDepthValue, pArgs->nImageXRes, pArgs->nImageYRes, pArgs->nDepthX, pArgs->nDepthY);
}

XnStatus XnDepthProcessor::GetColorCoordinatesOfD2C(XnUInt32 x, XnUInt32 y, OniDepthPixel z, XnUInt32 imageXRes, XnUInt32 imageYRes, XnUInt32& colorX, XnUInt32& colorY)
{
    XnStatus ret = XN_STATUS_OK;
    if (!m_paramRead){
        return XN_STATUS_ERROR;
    }
    int cx = 0;
    int cy = 0;
    int isRegist = GetStream()->getSoftwareRegistratorMode();
    m_softRegistrator.CoordinateConverterDepthToColor((int)x, (int)y, z, cx, cy, (isRegist == XN_PARAMS_REGISTRATION_USE_DISTORTION));
#if 0
    if(cy < 0 || cy > (int)imageYRes || cx < 0 || cx > (int)imageXRes){
        return XN_STATUS_ERROR;
    }
#endif
    colorX = cx;
    colorY = cy;
    return XN_STATUS_OK;
}

XnStatus XN_CALLBACK_TYPE XnDepthProcessor::GetColorCoordinatesOfD2CCallback(const XnGeneralProperty* pSender, const OniGeneralBuffer& gbValue, void* pCookie)
{
    XnDepthProcessor* pThis = (XnDepthProcessor*)pCookie;

    if (gbValue.dataSize != sizeof(XnPixelRegistration))
    {
        return XN_STATUS_DEVICE_PROPERTY_SIZE_DONT_MATCH;
    }

    XnPixelRegistration* pArgs = (XnPixelRegistration*)gbValue.data;

    return pThis->GetColorCoordinatesOfD2C(pArgs->nDepthX, pArgs->nDepthY, pArgs->nDepthValue, pArgs->nImageXRes, pArgs->nImageYRes, pArgs->nImageX, pArgs->nImageY);
}

void XnDepthProcessor::filterSpeckles(void* srcbuf, int newVal, int maxSpeckleSize, int maxDiff, int resolutionX, int resolutionY)
{
    int width = resolutionX, height = resolutionY, npixels = width*height;

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
    uint16_t* img = (uint16_t*)srcbuf;
#else
    typedef uint16_t (*Mat)[width];
    Mat img = (Mat)srcbuf;
#endif

    if (m_filterCache == NULL){
        size_t bufSize = npixels*(int)(sizeof(short) + sizeof(int) + sizeof(uchar));
        m_filterCache = new uint8_t[bufSize];
    }

    uchar* buf = m_filterCache;
    int i, j, dstep = (int)(width);
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf->x);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for (i = 0; i < height; i++)
    {
        short* ds = (short*)srcbuf + i * width;
        int* ls = labels + width*i;

        for (j = 0; j < width; j++)
        {
            if (ds[j] != newVal)   // not a bad disparity
            {
                if (ls[j])     // has a label, check for bad label
                {
                    if (rtype[ls[j]]) // small region, zero out disparity
                        ds[j] = (uint16_t)newVal;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((uint16_t)j, (uint16_t)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while (ws >= wbuf) // wavefront not empty
                    {
                        count++;
                        // put neighbors onto wavefront
                        //uint16_t* dpp = &img.at<T>(p.y, p.x);
#if (XN_PLATFORM == XN_PLATFORM_WIN32)
                        uint16_t* dpp = &img[p.y * width + p.x];
#else
                        uint16_t* dpp = &img[p.y][p.x];
#endif
                        uint16_t dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if (p.y < height - 1 && !lpp[+width] && dpp[+dstep] != newVal && abs(dp - dpp[+dstep]) <= maxDiff)
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y + 1);
                        }

                        if (p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && abs(dp - dpp[-dstep]) <= maxDiff)
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y - 1);
                        }

                        if (p.x < width - 1 && !lpp[+1] && dpp[+1] != newVal && abs(dp - dpp[+1]) <= maxDiff)
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x + 1, p.y);
                        }

                        if (p.x > 0 && !lpp[-1] && dpp[-1] != newVal && abs(dp - dpp[-1]) <= maxDiff)
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x - 1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if (count <= maxSpeckleSize)   // speckle region
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (uint16_t)newVal;
                    }
                    else
                        rtype[ls[j]] = 0;   // large region label
                }
            }
        }
    }
}

