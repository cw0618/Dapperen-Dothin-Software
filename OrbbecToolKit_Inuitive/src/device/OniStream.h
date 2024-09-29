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
#ifndef __ONI_STREAM_H__
#define __ONI_STREAM_H__


#include "OpenNI.h"
//#include "XnUSB.h"
#include "PS1080.h"
#include<QString>
#include <thread>
#include <mutex>

#include"src/device/common.h"
#define HISTSIZE 0xFFFF
using namespace openni;
typedef struct Point3D_ {
	float u;
	float v;
	float z;
}Point_3D;
class ObPtr
{
public:
	ObPtr();
	ObPtr(unsigned int _num);
	virtual ~ObPtr() { ObDelete(); }

	void UpdataDate(const int8_t *_src, unsigned int _size);
	void UpdataPhaseDate(const int8_t *_src, unsigned int _size);
	bool UpdataDate888bit();
	bool UpdataDatePhase();
    bool UpdataDatePhaseAB();
	bool UpdataDateIR(int w,int h);
	bool UpdataDateOtherIR(int w, int h,int min,int max);
	bool UpdataDateDepthIR();
    bool UpdataDateHistogram(int width, int height, bool y_flip = false);
	bool UpdateDatePointCould(int width, int height);
	bool UpdateDatePointCould(int width, int height,float * bufferData);
	/**
	* 保持和ImageJ打开的颜色一致
	* 
	*/
    bool UpdateDataImageJ();
	void calculateHistogram(int16_t *src, int w, int h);
    void convertRaw16ToRGB(int16_t* src, char* dst, int w, int h, int bitmov);
    void ObDelete();
    void ObDelete16bit();
    void ObDelete888bit();
	void ObDeleteDepth888bit();
    void ObDeleteHistogram888bit();
    void ObDeleteRainbow888bit();
	void ObDeleteD2C888bit();
	void ObDeletePointCouldData();

	void convertProjectiveToWorld(int u, int v, int z, Point_3D &world, bool use_distort_coef);

	void addPointFilterToDepthData();

	int8_t* mDataPtr{nullptr};                     // 原始深度数据
	int8_t* mDataDepthPtr{ nullptr };
	int8_t* mData888bitPtr{ nullptr };              // 灰度图数据
	int8_t* mDataHistogram888Ptr{ nullptr };       // 直方图数据
	int8_t* mDataRainbow888Ptr{ nullptr };         // 五彩模式数据
	int8_t* mDataD2c888Ptr{ nullptr };         // 五彩模式数据
	int8_t *mDataDepth888bitPtr{ nullptr };

	float * mDataPointCouldPtr{nullptr};    // 点云数据
	float mPointCloudFilterParams[3]{0};

 
public:
	void initGrayColor();
	uint32_t SizeOfData() const;
	uint32_t SizeOf888bit() const;
	uint32_t SizeOfHist888bit() const;
	uint32_t SizeOfRainbow888bit() const;
	uint32_t SizeOfD2C888bit() const;
	uint32_t SizeOfPointCloud() const;

	void SetRainbow888bitSize(uint32_t _size) { mSizeOfRainbow888bit = _size; };

	void SetD2C888bitSize(uint32_t _size) { mSizeOfD2c888bit = _size; };
	void SetRangeOfDepth(int value) { mRangeOfDepth = value; }
	void SetTofCameraParams(float *intrin, float *distort);

	uint32_t    mSizePointCouldData{ 0 };    // 点云  6个float
private:

	ObPtr& operator = (const ObPtr&) = delete;
	/// 画板
	uint8_t gray_color_value[256];
	// 记录指针大小，方便释放内存是判断
	uint32_t    mSizeOfData;              // 原始深度数据 16bit
	uint32_t    mSizeOf888bit;            // 灰度图  888bit
	uint32_t    mSizeOfHistogram888Bit;     // 直方图  888bit
	uint32_t    mSizeOfRainbow888bit;       // 五彩图  888bit
	uint32_t    mSizeOfD2c888bit;

	
	

	int mRangeOfDepth{8000};
	float mIntrin[4]{257.325f,257.235f,157.948f,120.483f};
	float mDistortCoeff[5]{ -0.0242094f ,0.189771f ,-0.000342234f ,-0.00048198f ,-0.388708f };
};


// class OniData *********************************
class OniData
{
public:
    int mWidth{ 0 };
    int mHeight{ 0 };
    int mPixelInBytes{ 0 };
    uint32_t mDataSizeBytes{ 0 };
    uint64_t mFrameTimeStamp{0};
	uint64_t mCustomTimeStamp{ 0 };
    bool mIsOffline{ false };
    int mFrameIndex{ 0 };
    int mPhaseNumber{0};
	int mLogicId{ 0 };
	int mOutMode;  // 帧输出模式
	bool isCaptureSuccess{ false };
	bool isCapturePCSuccess{ false };

    ObPtr oData;

	openni::Metadata mTofExteraline;

	OniData()
	{
		mFrameIndex = mWidth = mHeight = mDataSizeBytes = mLogicId = 0;
		mPixelInBytes = 2;
		mIsOffline = false;
	};

	virtual ~OniData() {
		Rst();
	};
	void static copyTofExtInfo(openni::Metadata* in, openni::Metadata* out) {
		*out = *in;
	}

    void Rst()
    {
        
        if (oData.mDataPtr != nullptr)
        {
            oData.ObDelete();
        }
		mFrameIndex = mWidth = mHeight = mDataSizeBytes = 0;
		mPixelInBytes = 2;
    }

    OniData& operator=(const OniData& R)
    {
        if (mWidth != R.mWidth || mHeight != R.mHeight || mDataSizeBytes != R.mDataSizeBytes)
        {
            Rst();
        }

        mWidth = R.mWidth;
        mHeight = R.mHeight;
        mDataSizeBytes = R.mDataSizeBytes;
        mFrameIndex = R.mFrameIndex;
        mPixelInBytes = R.mPixelInBytes;
		mCustomTimeStamp = R.mCustomTimeStamp;
		mFrameTimeStamp = R.mFrameTimeStamp;
		mLogicId = R.mLogicId;
		mOutMode = R.mOutMode;
		
        if (nullptr != R.oData.mDataPtr && 0 != mDataSizeBytes)
        {
            oData.UpdataDate(R.oData.mDataPtr, mDataSizeBytes);

        }
        return *this;
    };

    void copyPointCloud(OniData &_data)
    {
    	if (mWidth != _data.mWidth || mHeight != _data.mHeight || oData.mSizePointCouldData != _data.oData.mSizePointCouldData)
    	{
    		Rst();
    	}
		mWidth = _data.mWidth;
		mHeight = _data.mHeight;
		mDataSizeBytes = _data.mDataSizeBytes;
		mFrameIndex = _data.mFrameIndex;
		mPixelInBytes = _data.mPixelInBytes;
		mCustomTimeStamp = _data.mCustomTimeStamp;
		mLogicId = _data.mLogicId;
		mFrameTimeStamp = _data.mFrameTimeStamp;
		oData.mSizePointCouldData = _data.oData.mSizePointCouldData;
		if (nullptr != oData.mDataPointCouldPtr)
		{
			delete[] oData.mDataPointCouldPtr;
			
		}
		oData.mDataPointCouldPtr = new float[oData.mSizePointCouldData]();
		if (nullptr != _data.oData.mDataPointCouldPtr)
		{
			memcpy(oData.mDataPointCouldPtr, _data.oData.mDataPointCouldPtr, sizeof(float) * oData.mSizePointCouldData);
		}


    
    }

	void copy(const OniData& R)
	{
		if (mWidth != R.mWidth || mHeight != R.mHeight || mDataSizeBytes != R.mDataSizeBytes)
		{
			Rst();
		}

		mWidth = R.mWidth;
		mHeight = R.mHeight;
		mDataSizeBytes = R.mDataSizeBytes;
		mFrameIndex = R.mFrameIndex;
		mPixelInBytes = R.mPixelInBytes;
		mLogicId = R.mLogicId;
		mCustomTimeStamp = R.mCustomTimeStamp;

		if (nullptr != R.oData.mDataPtr && 0 != mDataSizeBytes)
		{
			oData.UpdataDate(R.oData.mDataPtr, mDataSizeBytes);
		}
		
	};
};

class OniStreamBase
{
public:
	openni::VideoStream         mVideoStream;
	openni::VideoMode           mStreamMode;
	openni::VideoFrameRef       mVideoFrameRef;
	openni::AIFrameRef          mFrameRefAi;
	const openni::SensorInfo*   mStreamInfo = nullptr;
	bool mStreamStatus = false;
	bool mCaptureFull = false;
	OniData mOniDataInfo;


	OniStreamBase();
	~OniStreamBase();


	void Rst();
	int StartStream();
	int StopStream();
	void DestroyStream();
	void CleanData();

	int UpdateInfo();
	int UpdateIRInfo(Metadata metaData, openni::VideoFrameRef * videoFrameRef,int outMode);
	int UpdateDepthInfo(int frameIndex, openni::VideoFrameRef * videoFrameRef);
	int UpdatePhaseInfo(TOFFrame phase);
	int UpdatePhaseInfo(Metadata metadata);
	int GetVideoFrameData(OniData &oniData ,int index);
private:

};


#endif //__ONI_STREAM_h__
