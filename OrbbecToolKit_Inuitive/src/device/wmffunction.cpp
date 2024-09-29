#include "wmffunction.h"
/** \class WMFFunction
*
* uvc数据流控制逻辑类
*
*/
WMFFunction::WMFFunction(QObject *parent):QThread(parent)
{
    mThreadStarted=false;
}

WMFFunction::~WMFFunction(){
    mThreadStarted=false;

    mUvcDevices.clear();

    if(mRgbBuffer!=nullptr){
        delete[] mRgbBuffer;
        mRgbBuffer=nullptr;
    }

    SAFE_RELEASE(mPSource);
    SAFE_RELEASE(mReader);
    SAFE_RELEASE(mPAttributes);

    SAFE_RELEASE(mPDecoderTransform);
    qDebug()<<"release wmffunction";
}
boolean WMFFunction::getThreadStatus(){
    return mThreadStarted;
}
boolean WMFFunction::getDeviceStatus() {
	return mDeviceCreate;
}
HRESULT WMFFunction::initWMF(){
    HRESULT hr;
    hr=CoInitializeEx(nullptr,COINIT_APARTMENTTHREADED|COINIT_DISABLE_OLE1DDE);
    if(FAILED(hr)){
        qDebug()<<"CoInitializeEx failed";
        return hr;
    }

    //create h264 decoder
    _com_ptr_IMFTransform mft;
    hr=CoCreateInstance(__uuidof(CMSH264DecoderMFT),NULL,CLSCTX_INPROC_SERVER,IID_IMFTransform,(VOID**)&mft);
    if(FAILED(hr)){
        qDebug()<<"create h264 decoder failed";
        return hr;
    }
    qDebug()<<"create h264 decoder success";
    hr=mft.QueryInterface(IID_PPV_ARGS(&mPDecoderTransform));
    if(FAILED(hr)){
        qDebug()<<"failed to get imftransform interface from h264 decoder";
    }
    qDebug()<<"get imftransfor interface from h264 success";
    hr=MFStartup(MF_VERSION);
    if(FAILED(hr)){
        qDebug()<<"mf startup failed";
        return hr;
    }
    qDebug()<<"success";

    hr=MFCreateAttributes(&mPAttributes,1);
    if(FAILED(hr)){
        qDebug()<<"create attributes failed";
        mPAttributes->Release();
        return hr;
    }

    hr=mPAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if(FAILED(hr)){
        qDebug()<<"set guid failed";
        mPAttributes->Release();
        return hr;
    }

    hr=enumDevice();
	mDeviceCreate = true;
    return hr;
}

HRESULT WMFFunction::enumDevice(){
    HRESULT hr;
    hr=MFEnumDeviceSources(mPAttributes,&mPpDevices,&mCount);
    qDebug()<<"device num:"<<mCount;

    for(int i=0;i<mCount;i++){
        TCHAR* pwszSymbolicLink_enum=NULL;
        UINT32 cchSymbolicLink_enum;
        hr=mPpDevices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,&pwszSymbolicLink_enum,&cchSymbolicLink_enum);
        QString deviceSymbolic=QString::fromStdWString(pwszSymbolicLink_enum);
        qDebug()<<"deviceSymbolic:"<<deviceSymbolic;

        TCHAR* name=NULL;
        hr=mPpDevices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,&name,&cchSymbolicLink_enum);
        QString deviceName=QString::fromStdWString(name);
        qDebug()<<"deviceName:"<<deviceName;
        if(deviceName.startsWith("UVC")){
            Uvc_Device mUvcDevice;
            mUvcDevice.cameraId=i;
            mUvcDevice.deviceName=deviceName;
            hr=mPpDevices[i]->ActivateObject(__uuidof(IMFMediaSource),(void**)&mPSource);
            if(FAILED(hr)){
                qDebug()<<"can not activate object";
            }

            mPAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING,TRUE);
            hr=MFCreateSourceReaderFromMediaSource(mPSource,mPAttributes,&mReader);
            if(SUCCEEDED(hr)){
                qDebug()<<"enum resolutions";
                mUvcDevice.resolutions=enumResolutions();
            }

            mUvcDevices.append(mUvcDevice);
            mCurrentIndexId=i;
        }
    }

    emit uvcDevice(mUvcDevices);

    return hr;
}

QList<Device_Resolution> WMFFunction::enumResolutions(){
    QList<Device_Resolution> mResolutions;
    HRESULT hr=S_OK;
    GUID subType{0};
    UINT32 frameRate=0;
    UINT32 frameRateMin=0;
    UINT32 frameRateMax=0;
    UINT32 denominator=0;
    DWORD32 width,height;
    DWORD index=0;
    //qDebug()<<"begin:"<<index;
    while(hr==S_OK){
        IMFMediaType *mediaType=NULL;
        hr=mReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,index,&mediaType);
        if(hr!=S_OK){
            //qDebug()<<"end enum stream:"<<index;
            break;
        }

        hr=mediaType->GetGUID(MF_MT_SUBTYPE,&subType);

        if(subType==MFVideoFormat_NV12){
            mediaType->Release();
            ++index;
            continue;
        }

        QString mTag;
        if(subType==MFVideoFormat_H264){
            mTag="H264";

        }else if(subType==MFVideoFormat_MJPG){
            mTag="MJPG";
        }else if(subType==MFVideoFormat_RGB32){
            mTag="RGB32";
        }else if(subType==MFVideoFormat_YUY2){
            mTag="YUY2";
        }else if(subType==MFVideoFormat_RGB24){
            mTag="RGB24";
        }else if(subType==MFVideoFormat_NV12){
            mTag="NV12";
        }else{
            mTag="Others";
        }

        hr=MFGetAttributeSize(mediaType,MF_MT_FRAME_SIZE,&width,&height);
        hr=MFGetAttributeRatio(mediaType,MF_MT_FRAME_RATE,&frameRate,&denominator);
        hr=MFGetAttributeRatio(mediaType,MF_MT_FRAME_RATE_RANGE_MIN,&frameRateMin,&denominator);
        hr=MFGetAttributeRatio(mediaType,MF_MT_FRAME_RATE_RANGE_MAX,&frameRateMax,&denominator);

        Device_Resolution mResolution;
        mResolution.indexId=index;
        mResolution.width=width;
        mResolution.height=height;
        mResolution.fps=frameRate;
        mResolution.format=mTag;
        mResolutions.append(mResolution);
        //qDebug()<<"index:"<<index<<",width:"<<width<<",height:"<<height<<",frameRate:"<<frameRate<<",mTag:"<<mTag;
        mediaType->Release();
        ++index;
    }

    return mResolutions;

}


HRESULT WMFFunction::setResolution(int cameraIndex,int indexId,int width,int height,int fps,QString format){
    HRESULT hr;
    if(mCurrentIndexId!=cameraIndex){
        qDebug()<<"recreate obj:"<<mCurrentIndexId<<","<<cameraIndex;
        SAFE_RELEASE(mPSource);
        SAFE_RELEASE(mReader);
        SAFE_RELEASE(mPAttributes);
        qDebug()<<"before detach device:"<<mCount;
        for(int i=0;i<mCount;i++){
            mPpDevices[i]->DetachObject();
            SAFE_RELEASE(mPpDevices[i]);
        }
        qDebug()<<"after detach device";
        hr=MFCreateAttributes(&mPAttributes,1);
        if(FAILED(hr)){
            qDebug()<<"create attributes failed";
            mPAttributes->Release();
            return hr;
        }
        qDebug()<<"create attributes success";
        hr=mPAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if(FAILED(hr)){
            qDebug()<<"set guid failed";
            mPAttributes->Release();
            return hr;
        }
        qDebug()<<"set guid success";

        hr=MFEnumDeviceSources(mPAttributes,&mPpDevices,&mCount);
        if(FAILED(hr)){
            qDebug()<<"enum device source failed";
            mPAttributes->Release();

        }

        qDebug()<<"enum device source success:"<<mCount;

        hr=mPpDevices[cameraIndex]->ActivateObject(__uuidof(IMFMediaSource),(void**)&mPSource);
        if(FAILED(hr)){
            qDebug()<<"can not activate object";
            return hr;
        }

        mPAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING,TRUE);
        hr=MFCreateSourceReaderFromMediaSource(mPSource,mPAttributes,&mReader);
        mCurrentIndexId=cameraIndex;
    }

    mWidth=width;
    mHeight=height;
    mFrameRate=fps;
    mFormat=format;

    IMFMediaType *pNativeType=NULL;
    IMFMediaType *pType=NULL;
    GUID subType;

    DWORD mIndex=0;
    if(format=="H264"){
        hr=mReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM,indexId,&pNativeType);
    }else{
        hr=mReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM,0,&pNativeType);
    }

    if(FAILED(hr)){
        qDebug()<<"get native media type failed";
        return hr;
    }

    UINT32 w,h;
    hr=MFGetAttributeSize(pNativeType,MF_MT_FRAME_SIZE,&w,&h);
    if(FAILED(hr)){
        qDebug()<<"get attribute size failed";
        return hr;
    }

    UINT32 framerate=0;
    UINT32 frameRateMin=0;
    UINT32 frameRateMax=0;
    UINT32 denominator=0;

    hr=MFGetAttributeRatio(pNativeType,MF_MT_FRAME_RATE,&framerate,&denominator);
    hr=MFGetAttributeRatio(pNativeType,MF_MT_FRAME_RATE,&frameRateMin,&denominator);
    hr=MFGetAttributeRatio(pNativeType,MF_MT_FRAME_RATE,&frameRateMax,&denominator);
    qDebug()<<"width:"<<w<<",height:"<<h<<",frameRate:"<<framerate;

    hr=pNativeType->GetGUID(MF_MT_SUBTYPE,&subType);
    if(subType==MFVideoFormat_MJPG){
        qDebug()<<"jpeg";
    }else if(subType==MFVideoFormat_NV12){
        qDebug()<<"nv12";
    }else if(subType==MFVideoFormat_H264){
        qDebug()<<"h264";
    }

    GUID majorType;
    hr=pNativeType->GetGUID(MF_MT_MAJOR_TYPE,&majorType);
    if(FAILED(hr)){
        qDebug()<<"get guid failed";
        return hr;
    }

    if(majorType==MFMediaType_Video){
        subType=MFVideoFormat_NV12;
    }else{
        return hr;
    }




    hr=mReader->SetStreamSelection(MF_SOURCE_READER_ALL_STREAMS,false);
    if(FAILED(hr)){
        qDebug()<<"set all stream failed";
        return hr;
    }

    hr=mReader->SetStreamSelection(MF_SOURCE_READER_FIRST_VIDEO_STREAM,TRUE);
    if(FAILED(hr)){
        qDebug()<<"Set stream selection failed";
        return hr;
    }

    if(format!="H264"){
        MFCreateMediaType(&pType);
        pType->SetGUID(MF_MT_MAJOR_TYPE,MFMediaType_Video);
        pType->SetGUID(MF_MT_SUBTYPE,subType);
        MFSetAttributeRatio(pType,MF_MT_FRAME_RATE,framerate,1);
        MFSetAttributeSize(pType,MF_MT_FRAME_SIZE,width,height);
        mReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,NULL, pType);
        SAFE_RELEASE(pType);
    }else{
        mReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,NULL, pNativeType);

        MFCreateMediaType(&mPDecInputMediaType);
        pNativeType->CopyAllItems(mPDecInputMediaType);
        MFSetAttributeSize(mPDecInputMediaType,MF_MT_FRAME_SIZE,width,height);
        MFSetAttributeRatio(mPDecInputMediaType,MF_MT_PIXEL_ASPECT_RATIO,width/height,1);
        MFSetAttributeRatio(mPDecInputMediaType,MF_MT_FRAME_RATE,framerate,1);
        mPDecInputMediaType->SetUINT32(MF_MT_INTERLACE_MODE,MFVideoInterlace_MixedInterlaceOrProgressive);

        hr=mPDecoderTransform->SetInputType(0,mPDecInputMediaType,0);
        if(FAILED(hr)){
            qDebug()<<"set input type failed";
            return hr;
        }
        qDebug()<<"set input type success";

        DWORD maxIn,minIn,maxOut,minOut,inCnt,outCnt;
        hr=mPDecoderTransform->GetStreamLimits(&minIn,&maxIn,&minOut,&maxOut);
        qDebug()<<"minIn:"<<minIn<<",maxIn:"<<maxIn<<",minOut:"<<minOut<<",maxOut:"<<maxOut;

        hr=mPDecoderTransform->GetStreamCount(&inCnt,&outCnt);
        qDebug()<<"inCnt:"<<inCnt<<",outCnt:"<<outCnt;

        DWORD inId,outId;
        hr=mPDecoderTransform->GetStreamIDs(1,&inId,1,&outId);
        if(FAILED(hr)){
            qDebug()<<"get stream id failed";
            //return hr;
        }

        DWORD mIndexes=0;
        IMFMediaType *mMediaType;
        hr=mPDecoderTransform->GetOutputAvailableType(0,mIndexes,&mMediaType);
        if(hr==MF_E_TRANSFORM_TYPE_NOT_SET){
            qDebug()<<"need set input type first:"<<hr;
            // break;
        }else if(hr==E_NOTIMPL){
            qDebug()<<"The MFT does not have a list of available output types:"<<hr;

        }else if(hr==MF_E_NO_MORE_TYPES){
            qDebug()<<"output stream index is out of bound:"<<mIndexes<<","<<hr;
            // break;
        }else if(hr==MF_E_INVALIDSTREAMNUMBER){
            qDebug()<<"Invalid stream identifier."<<hr;
        }else if(hr==S_OK){
            qDebug()<<"get output available type success";
        }else{
            qDebug()<<"get output available type failed:"<<hr;
        }

        hr=mMediaType->GetGUID(MF_MT_SUBTYPE,&subType);
        if(subType==MFVideoFormat_H264){
            qDebug()<<"*********** h264";
        }else if(subType==MFVideoFormat_NV12){
            qDebug()<<"************** nv12";
        }else if(subType==MFVideoFormat_RGB32){
            qDebug()<<"***************** rgb32";
        }else{
            qDebug()<<"*********** others";
        }

        MFCreateMediaType(&mPDecOutputMediaType);
        pNativeType->CopyAllItems(mPDecOutputMediaType);
        mPDecOutputMediaType->SetGUID(MF_MT_MAJOR_TYPE,MFMediaType_Video);

        hr=mPDecOutputMediaType->SetGUID(MF_MT_SUBTYPE,subType);
        if(FAILED(hr)){
            qDebug()<<"set subtype nv12 failed:"<<hr;

            hr=mPDecOutputMediaType->SetGUID(MF_MT_SUBTYPE,MFVideoFormat_RGB24);
            if(FAILED(hr)){
                qDebug()<<"set subtype rgb24 failed:"<<hr;
                return hr;
            }else{
                qDebug()<<"set subtype rgb24 success";
            }
        }else{
            qDebug()<<"set subtype nv12 success";
        }

        mPDecOutputMediaType->SetUINT32(CODECAPI_AVDecVideoAcceleration_H264,1);
        mPDecOutputMediaType->SetUINT32(CODECAPI_AVDecVideoThumbnailGenerationMode,TRUE);
        mPDecOutputMediaType->SetUINT32(MF_SA_D3D_AWARE,TRUE);
        MFSetAttributeRatio(mPDecOutputMediaType,MF_MT_PIXEL_ASPECT_RATIO,mWidth/mHeight,1);

        hr=mPDecoderTransform->SetOutputType(0,mPDecOutputMediaType,0);
    if(hr==MF_E_UNSUPPORTED_D3D_TYPE){
            qDebug()<<"The MFT could not find a suitable DirectX Video Acceleration (DXVA) configuration:"<<hr;
        }else if(hr==MF_E_TRANSFORM_TYPE_NOT_SET){
            qDebug()<<"You must set the input types before setting the output types.:"<<hr;
        }else if(hr==MF_E_TRANSFORM_CANNOT_CHANGE_MEDIATYPE_WHILE_PROCESSING){
            qDebug()<<"The MFT cannot switch types while processing data. Try draining or flushing the MFT:"<<hr;
        }else if(hr==MF_E_INVALIDTYPE){
            qDebug()<<"The proposed type is not valid. This error code indicates that the media type itself is not configured correctly; for example, it might contain mutually contradictory flags."<<hr;
        }else if(hr==MF_E_INVALIDSTREAMNUMBER){
            qDebug()<<"Invalid stream identifier."<<hr;
        }else if(hr==MF_E_INVALIDMEDIATYPE){
            qDebug()<<"The transform cannot use the proposed media type."<<hr;
        }else if(hr==S_OK){
            qDebug()<<"set outputtype success:"<<hr;
        }else{
            qDebug()<<"set outputtype failed:"<<hr;
        }

        IMFMediaType *currentType;
        mPDecoderTransform->GetOutputCurrentType(0,&currentType);
        GUID type;
        currentType->GetGUID(MF_MT_SUBTYPE,&type);
        if(type==MFVideoFormat_H264){
            qDebug()<<"pDecOutputMediaType*********** h264";
        }else if(type==MFVideoFormat_NV12){
            qDebug()<<"pDecOutputMediaType************** nv12";
        }else if(type==MFVideoFormat_RGB32){
            qDebug()<<"pDecOutputMediaType***************** rgb32";
        }else{
            qDebug()<<"pDecOutputMediaType*********** others";
        }

        DWORD status;
        mPDecoderTransform->GetInputStatus(0,&status);
        if(MFT_INPUT_STATUS_ACCEPT_DATA!=status){
            qDebug()<<"h264 decoder is not accept data";
            return  hr;
        }

        hr=mPDecoderTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM,NULL);
        hr=mPDecoderTransform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
        if(FAILED(hr)){
            qDebug()<<"failed to process flush command on h264 decoder";
            return hr;
        }
        qDebug()<<"process flush command on h264 decoder success";
        hr=mPDecoderTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING,NULL);
        if(FAILED(hr)){
            qDebug()<<"begin streaming command failed";
            return hr;
        }
        qDebug()<<"begin streaming command success";

        hr=mPDecoderTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM,NULL);
        if(FAILED(hr)){
            qDebug()<<"send start of stream command failed";
            return hr;
        }
        qDebug()<<"send start stream command";

        SAFE_RELEASE(mMediaType);
        SAFE_RELEASE(currentType);
        SAFE_RELEASE(mPDecInputMediaType);
        SAFE_RELEASE(mPDecOutputMediaType);

    }

    SAFE_RELEASE(pNativeType);
    return hr;
}


void WMFFunction::startThread(){
    mThreadStarted=true;
    if(!this->isRunning()){
        start();
    }
}
void WMFFunction::closeDevice() {
	if (mDeviceCreate)
	{
		mDeviceCreate = false;
		mUvcDevices.clear();
		SAFE_RELEASE(mPSource);
		SAFE_RELEASE(mReader);
		SAFE_RELEASE(mPAttributes);

		for (int i = 0; i<mCount; i++) {
			mPpDevices[i]->DetachObject();
			SAFE_RELEASE(mPpDevices[i]);
		}
	}


	//    ppDevices[currentIndexId]->DetachObject();
	//    SAFE_RELEASE(ppDevices[currentIndexId]);

	//SAFE_RELEASE(pDecoderTransform);
}
void WMFFunction::stopThread(){
    mThreadStarted=false;
    emit stopRender();
}

void WMFFunction::run(){
    while (mThreadStarted && mDeviceCreate) {
        HRESULT hr;
        IMFSample *pSample=nullptr,*pCopyVideoSample=nullptr,*pH264DecodeOutSample=nullptr;
        int sampleCount=0;
        DWORD streamIndex,flags,sampleFlags;
        LONGLONG llTimeStamp=0,llSampleDuration=0;
        IMFMediaType *pType;
        LONGLONG duration;
        BOOL h264DecodeTransformFlushed = FALSE;

        hr=mReader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM,&pType);
        if(FAILED(hr)){
            qDebug()<<"get current media type failed";
            continue;
        }

        UINT32 frameRate,denominator;
        MFGetAttributeRatio(pType,MF_MT_FRAME_RATE,&frameRate,&denominator);
        mReader->ReadSample((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,0,&streamIndex,&flags,&llTimeStamp,&pSample);
        //qDebug()<<"streamIndex:"<<streamIndex;
        if(FAILED(hr)){
            qDebug()<<"read sample failed";
            mThreadStarted=false;
        }else{

            if(pSample){
                if(mFormat=="H264"){

                    pSample->SetSampleTime(llTimeStamp);
                    pSample->GetSampleDuration(&llSampleDuration);
                    pSample->GetSampleFlags(&sampleFlags);
                    qDebug()<<"sample flag:"<<sampleFlags<<",sampleDuration:"<<llSampleDuration;

                    hr=CreateAndCopySingleBufferIMFSample(pSample,&pCopyVideoSample);
                    if(FAILED(hr)){
                        qDebug()<<"create and copy single buffer sample failed";
                        break;
                    }

                    hr=mPDecoderTransform->ProcessInput(0,pCopyVideoSample,0);
                    if(hr==S_OK){
                        qDebug()<<"process input success";
                    }else{
                        qDebug()<<"process input failed";
                    }

                    while(hr==S_OK){
                        hr=GetTransformOutput(mPDecoderTransform,&pH264DecodeOutSample,&h264DecodeTransformFlushed);
                        if(hr!=S_OK&&hr!=MF_E_TRANSFORM_NEED_MORE_INPUT){
                            qDebug()<<"error get h264 decoder";
                            SAFE_RELEASE(pSample);
                            SAFE_RELEASE(pCopyVideoSample);
                            SAFE_RELEASE(pH264DecodeOutSample);
                        }

                        if(h264DecodeTransformFlushed==TRUE){
                            qDebug()<<"success";
                        }else if(pH264DecodeOutSample!=NULL){
                            qDebug()<<"decoded buffer is here";

                            IMFMediaBuffer* buf=nullptr;
                            DWORD bufLen;
                            hr=pH264DecodeOutSample->ConvertToContiguousBuffer(&buf);
                            buf->GetCurrentLength(&bufLen);
                            qDebug()<<"bufLen:"<<bufLen;

                            byte *byteBuffer=nullptr;
                            DWORD bufCurrLen=0;
                            DWORD buffMaxLen=0;

                            if(mDataWidth!=mWidth||mDataHeight!=mHeight||mCurrentFormat!=mFormat){
                                if(mRgbBuffer!=nullptr){
                                    delete[] mRgbBuffer;
                                }

                                if(mRenderBuffer!=nullptr){
                                    delete[] mRenderBuffer;
                                    mRenderBuffer=nullptr;
                                }

                                mDataWidth=mWidth;
                                mDataHeight=mHeight;
                                mCurrentFormat=mFormat;

                                double additionBufSize=(bufLen-(mWidth*mHeight*1.5));

                                if(additionBufSize==0){
                                    mAdditionalLine=0;
                                    mRgbBuffer=new byte[mDataWidth*mDataHeight*3];
                                }else{
                                    mAdditionalLine=additionBufSize/(mWidth*1.5);

                                    mRgbBuffer=new byte[mDataWidth*(mDataHeight+mAdditionalLine)*3];
                                    mRenderBuffer=new byte[mDataWidth*mDataHeight*3];
                                }


                            }

                            hr=buf->Lock(&byteBuffer,&buffMaxLen,&bufCurrLen);
                            if(FAILED(hr)){
                                qDebug()<<"failed to lock video sample buffer:"<<hr;
                                buf->Unlock();
                                buf->Release();
                                pH264DecodeOutSample->Release();
                                pSample->Release();
                                pCopyVideoSample->Release();
                                continue;
                            }

                            NV12_T_BGR(mWidth,mHeight+mAdditionalLine,byteBuffer,mRgbBuffer);
                            if(mAdditionalLine==0){
                                emit dataCallback(mWidth,mHeight,mRgbBuffer,frameRate,mFormat);
                            }else{
                                mMutex.lock();
                                cv::Mat mat=cv::Mat::zeros(mHeight+mAdditionalLine,mWidth,CV_8UC3);
                                int nByte=mWidth*(mHeight+mAdditionalLine)*3;
                                memcpy(mat.data,mRgbBuffer,nByte);
                                cv::Mat cutMat=cv::Mat(mat,cv::Rect(0,mAdditionalLine,mWidth,mHeight));
                                memcpy(mRenderBuffer,cutMat.data,mWidth*mHeight*3);
                                emit dataCallback(mWidth,mHeight,mRenderBuffer,frameRate,mFormat);
                                mMutex.unlock();
                            }

                            buf->Unlock();
                            buf->Release();

                        }

                        SAFE_RELEASE(pH264DecodeOutSample);
                    }
                    sampleCount++;
                    SAFE_RELEASE(pCopyVideoSample);

                }else{
                    pSample->GetSampleDuration(&duration);
                    IMFMediaBuffer *buf=NULL;
                    DWORD bufLength;
                    hr=pSample->ConvertToContiguousBuffer(&buf);
                    if(FAILED(hr)){
                        qDebug()<<"convert to Contiguousbuffer failed";
                    }

                    hr=buf->GetCurrentLength(&bufLength);
                    if(FAILED(hr)){
                        qDebug()<<"get current length failed";
                    }
                    //qDebug()<<"get current len:"<<bufLength;

                    byte *byteBuffer;
                    DWORD buffCurrentLen=0;
                    DWORD buffMaxLen=0;
                    if(mDataWidth!=mWidth||mDataHeight!=mHeight){
                        if(mRgbBuffer!=nullptr){
                            delete[] mRgbBuffer;
                            mRgbBuffer=nullptr;
                        }

                        mDataWidth=mWidth;
                        mDataHeight=mHeight;
                        mRgbBuffer=new byte[mDataWidth*mDataHeight*3];
                    }

                    hr=buf->Lock(&byteBuffer,&buffMaxLen,&buffCurrentLen);
                    if(FAILED(hr)){
                        qDebug()<<"failed to lock video sample buffer:"<<hr;
                        buf->Unlock();
                        buf->Release();
                        pSample->Release();
                        continue;
                    }

                    NV12_T_BGR(mDataWidth,mDataHeight,byteBuffer,mRgbBuffer);
                    emit dataCallback(mDataWidth,mDataHeight,mRgbBuffer,frameRate,mFormat);
                    buf->Unlock();
                    buf->Release();
                    sampleCount++;

                }

            }
        }
        SAFE_RELEASE(pType);
        SAFE_RELEASE(pSample);
    }
}


void __stdcall WMFFunction::NV12_T_BGR(unsigned int width, unsigned int height, unsigned char *yuyv,
                                       unsigned char *bgr) {
	if (mDeviceCreate)
	{
		const int nv_start = width * height;
		int i, j, index = 0, rgb_index = 0;
		unsigned char y, u, v;
		int r, g, b, nv_index = 0;

		for (i = 0; i < height; i++) {
			for (j = 0; j < width; j++) {
				if (!mDeviceCreate)
				{
					break;
				}
				//nv_index = (rgb_index / 2 - width / 2 * ((i + 1) / 2)) * 2;
				nv_index = i / 2 * width + j - j % 2;

				y = yuyv[rgb_index];
				v = yuyv[nv_start + nv_index];
				u = yuyv[nv_start + nv_index + 1];
				//            u = yuyv[nv_start + nv_index ];
				//            v = yuyv[nv_start + nv_index + 1];

				r = y + (140 * (v - 128)) / 100;  //r
				g = y - (34 * (u - 128)) / 100 - (71 * (v - 128)) / 100; //g
				b = y + (177 * (u - 128)) / 100; //b

				if (r > 255)
					r = 255;
				if (g > 255)
					g = 255;
				if (b > 255)
					b = 255;
				if (r < 0)
					r = 0;
				if (g < 0)
					g = 0;
				if (b < 0)
					b = 0;

				index = rgb_index % width + (height - i - 1) * width;
				bgr[index * 3 + 2] = r;
				bgr[index * 3 + 1] = g;
				bgr[index * 3 + 0] = b;
				rgb_index++;
			}
		}
	}


}

HRESULT __stdcall WMFFunction::CreateSingleBufferIMFSample(DWORD bufferSize, IMFSample** pSample)
{
    IMFMediaBuffer* pBuffer = NULL;

    HRESULT hr = S_OK;

    hr = MFCreateSample(pSample);
    CHECK_HR(hr, "Failed to create MF sample.");

    // Adds a ref count to the pBuffer object.
    hr = MFCreateMemoryBuffer(bufferSize, &pBuffer);
    CHECK_HR(hr, "Failed to create memory buffer.");

    // Adds another ref count to the pBuffer object.
    hr = (*pSample)->AddBuffer(pBuffer);
    CHECK_HR(hr, "Failed to add sample to buffer.");

done:
    // Leave the single ref count that will be removed when the pSample is released.
    SAFE_RELEASE(pBuffer);
    return hr;
}

HRESULT __stdcall WMFFunction::CreateAndCopySingleBufferIMFSample(IMFSample* pSrcSample,IMFSample** pDstSample){
    IMFMediaBuffer* pDstBuffer=NULL;
    DWORD srcBufLength;
    HRESULT hr;
    hr=pSrcSample->GetTotalLength(&srcBufLength);
    if(FAILED(hr)){
        qDebug()<<"get total buffer length failed";
        return hr;
    }

    hr=CreateSingleBufferIMFSample(srcBufLength,pDstSample);
    CHECK_HR(hr, "Failed to create new single buffer IMF sample.");

    hr = pSrcSample->CopyAllItems(*pDstSample);
    CHECK_HR(hr, "Failed to copy IMFSample items from src to dst.");

    hr = (*pDstSample)->GetBufferByIndex(0, &pDstBuffer);
    CHECK_HR(hr, "Failed to get buffer from sample.");

    hr = pSrcSample->CopyToBuffer(pDstBuffer);
    CHECK_HR(hr, "Failed to copy IMF media buffer.");

done:
    SAFE_RELEASE(pDstBuffer);
    return hr;
}

HRESULT __stdcall WMFFunction::GetTransformOutput(IMFTransform* pTransform, IMFSample** pOutSample, BOOL* transformFlushed)
{
	if (mDeviceCreate)
	{

    MFT_OUTPUT_STREAM_INFO StreamInfo = { 0 };
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer = { 0 };
    DWORD processOutputStatus = 0;
    IMFMediaType* pChangedOutMediaType = NULL;

    HRESULT hr = S_OK;
    *transformFlushed = FALSE;

    hr = pTransform->GetOutputStreamInfo(0, &StreamInfo);
    //CHECK_HR(hr, "Failed to get output stream info from MFT.");
    if(FAILED(hr)){
        qDebug()<<"failed to get output stream info from mft";
        return  hr;
    }
    outputDataBuffer.dwStreamID = 0;
    outputDataBuffer.dwStatus = 0;
    outputDataBuffer.pEvents = NULL;

    if ((StreamInfo.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES) == 0) {
        hr = CreateSingleBufferIMFSample(StreamInfo.cbSize, pOutSample);
        //CHECK_HR(hr, "Failed to create new single buffer IMF sample.");
        if(FAILED(hr)){
            qDebug()<<"failed to create new single buffer IMF sample";
            return hr;
        }

        outputDataBuffer.pSample = *pOutSample;
        qDebug()<<"create new single buffer IMF sample success:"<<outputDataBuffer.dwStreamID;
    }

    auto mftProcessOutput = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &processOutputStatus);
    //1072861856
    qDebug()<<QString("Process output result %1, MFT status %2.").arg(mftProcessOutput).arg(processOutputStatus);

    if (mftProcessOutput == S_OK) {
        // Sample is ready and allocated on the transform output buffer.
        *pOutSample = outputDataBuffer.pSample;
    }
    else if (mftProcessOutput == MF_E_TRANSFORM_STREAM_CHANGE) {
        qDebug()<<"The format has changed on an output stream, or there is a new preferred format, or there is a new output stream.";
        // Format of the input stream has changed. https://docs.microsoft.com/en-us/windows/win32/medfound/handling-stream-changes
        if (outputDataBuffer.dwStatus == MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE) {
            qDebug()<<"MFT stream changed.\n";
            qDebug()<<"stream id:"<<outputDataBuffer.dwStreamID;

            hr = pTransform->GetOutputAvailableType(0, 0, &pChangedOutMediaType);
            CHECK_HR(hr, "Failed to get the MFT output media type after a stream change.");

            //std::cout << "MFT output media type: " << GetMediaTypeDescription(pChangedOutMediaType) << std::endl << std::endl;
            GUID subType;
            pChangedOutMediaType->GetGUID(MF_MT_SUBTYPE,&subType);
            if(subType==MFVideoFormat_H264){
                qDebug()<<"TF*********** h264";
            }else if(subType==MFVideoFormat_NV12){
                qDebug()<<"TF************** nv12";
            }else if(subType==MFVideoFormat_RGB32){
                qDebug()<<"TF***************** rgb32";
            }else{
                qDebug()<<"TF*********** others";
            }


            hr = pChangedOutMediaType->SetGUID(MF_MT_SUBTYPE, subType);
            CHECK_HR(hr, "Failed to set media sub type.");


            hr = pTransform->SetOutputType(0, pChangedOutMediaType, 0);
            CHECK_HR(hr, "Failed to set new output media type on MFT.");

            hr = pTransform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
            // hr = pTransform->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, NULL);
            CHECK_HR(hr, "Failed to process FLUSH command on MFT.");
            hr = pTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, NULL);
            hr = pTransform->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, NULL);
            //   hr = pTransform->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);

            *transformFlushed = TRUE;
        }
        else {
            qDebug()<<"MFT stream changed but didn't have the data format change flag set. Don't know what to do.";
            printf("MFT stream changed but didn't have the data format change flag set. Don't know what to do.\n");
            hr = E_NOTIMPL;
        }

        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }
    else if (mftProcessOutput == MF_E_TRANSFORM_NEED_MORE_INPUT) {
        // More input is not an error condition but it means the allocated output sample is empty.
        SAFE_RELEASE(pOutSample);
        qDebug()<<"The transform cannot produce output data until it receives more input data.";
        *pOutSample = NULL;
        hr = MF_E_TRANSFORM_NEED_MORE_INPUT;
    }else if(mftProcessOutput==E_UNEXPECTED){
        qDebug()<<"The ProcessOutput method was called on an asynchronous MFT that was not expecting this method call.";
        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }else if(mftProcessOutput==MF_E_INVALIDSTREAMNUMBER){
        qDebug()<<"Invalid stream identifier in the dwStreamID member of one or more MFT_OUTPUT_DATA_BUFFER structures.";
        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }else if(mftProcessOutput==MF_E_TRANSFORM_TYPE_NOT_SET){
        qDebug()<<"You must set the media type on one or more streams of the MFT";
        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }
    else {
        qDebug()<<"process out error:"<<mftProcessOutput<<","<<processOutputStatus;
        printf("MFT ProcessOutput error result %.2X, MFT status %.2X.\n", mftProcessOutput, processOutputStatus);
        hr = mftProcessOutput;

        SAFE_RELEASE(pOutSample);
        *pOutSample = NULL;
    }

done:

    SAFE_RELEASE(pChangedOutMediaType);

    return hr;

	}
	else {
		return S_FALSE;
	}
}


