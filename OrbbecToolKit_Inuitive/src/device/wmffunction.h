/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef WMFFUNCTION_H
#define WMFFUNCTION_H

#include "qthread.h"
#include "qdebug.h"

#include "mfmediacapture.h"
#include "mfapi.h"
#include "mfobjects.h"
#include "initguid.h"
#include "mfidl.h"
#include "vidcap.h"
#include "stdio.h"
#include "mfreadwrite.h"
#include "wmcodecdsp.h"
//#include "evr.h"
#include "utils.h"
#include "Mferror.h"
//#include "combaseapi.h"
#include "QLatin1Char"
//#include "mfmediaengine.h"
#include "mftransform.h"

#include "d3d11.h"
#include <comip.h>
#include <codecapi.h>
#include"opencv2/opencv.hpp"
#include "qmutex.h"

#define CHECK_HR(hr, msg) if (hr != S_OK) { qDebug()<<msg; qDebug()<<"error:"<<hr;goto done; }

#pragma comment(lib,"mf.lib")
#pragma comment(lib,"mfplat.lib")
#pragma comment(lib,"Mfreadwrite.lib")
#pragma comment(lib,"Mfuuid.lib")
#pragma comment(lib,"wmcodecdspuuid.lib")
#pragma comment(lib,"Ole32.lib")
#pragma comment(lib,"d3d11.lib")

template <class T> void SAFE_RELEASE(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

template <class T> inline void SAFE_RELEASE(T*& pT)
{
    if (pT != NULL)
    {
        pT->Release();
        pT = NULL;
    }
}

typedef _com_ptr_t < _com_IIID<IMFTransform, &__uuidof(IMFTransform) > > _com_ptr_IMFTransform;


class WMFFunction:public QThread
{
    Q_OBJECT
public:
    WMFFunction(QObject *parent=nullptr);
    ~WMFFunction();

    bool mThreadStarted=false;
	bool mDeviceCreate = false;
    void startThread();
    void stopThread();
	void closeDevice();
    boolean getThreadStatus();
	boolean getDeviceStatus();
    HRESULT initWMF();
    QList<Device_Resolution> enumResolutions();
    HRESULT setResolution(int cameraId,int indexId,int width,int height,int fps,QString format);


protected:
    void run();

private:
    IMFTransform* mPDecoderTransform=NULL;
    IMFMediaType *mPDecInputMediaType=nullptr,*mPDecOutputMediaType=nullptr;
    IMFAttributes *mPAttributes=nullptr;
    IMFActivate **mPpDevices=nullptr;
    IMFMediaSource *mPSource=nullptr;
    IMFSourceReader *mReader=nullptr;
    UINT32 mCount=0;

    QList<Uvc_Device> mUvcDevices;

    int mWidth;
    int mHeight;
    int mFrameRate;
    QString mFormat;
    QString mCurrentFormat="";

    byte *mRgbBuffer=nullptr,*mRenderBuffer=nullptr;
    int mDataWidth=0,mDataHeight=0;
    int mCurrentIndexId=0;
    double mAdditionalLine=0;
    QMutex mMutex;


    HRESULT enumDevice();
    void __stdcall NV12_T_BGR(unsigned int width, unsigned int height, unsigned char *yuyv,
                              unsigned char *bgr);
    HRESULT __stdcall CreateAndCopySingleBufferIMFSample(IMFSample* pSrcSample,IMFSample** pDstSample);
    HRESULT __stdcall CreateSingleBufferIMFSample(DWORD bufferSize, IMFSample** pSample);
    HRESULT __stdcall GetTransformOutput(IMFTransform* pTransform, IMFSample** pOutSample, BOOL* transformFlushed);

signals:
    void uvcDevice(QList<Uvc_Device> mDevices);
    void dataCallback(int width,int height,unsigned char* data,int fps,QString format);
    void stopRender();


};

#endif // WMFFUNCTION_H
