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

#ifndef _sensorbase_
#define _sensorbase_
#pragma once

#include "3rd/openni2/Include/OpenNI.h"
//#include "3rd/xnlib/include/XnLib.h"
//#include "3rd/xnlib/include/XnUSB.h"
#include "3rd/openni2/Include/PS1080.h"
#include "src/device/SubStream.h"
#include"3rd/OpenNI2/Include/tofinfo.h"
#include "src/calculate/calc.h"
#include "src/device/capturethread.h"
#include"3rd/OpenNI2/Include/OBCProperties.h"
#include"src/device/aibody.h"
#include "src/config/RegisterIniFile.h"
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <map>
#include <mutex>
#include "qfile.h"
#include <QFileDialog>
#include <queue>
#include <stdio.h>
#include <chrono>
#define ChangeToStr(className) #className

#define FILTER_INI_FILE_AB ".\\config\\filter_config_new.ini"
#define FILTER_INI_FILE_A_B ".\\config\\filter_config_new.ini"
#define CALI_INI_FILE ".\\config\\DepthParamsNew.bin"
#define CALI_INI_FILE_EEPROM 0x64
using cb_functor = std::function<void(bool)>;
using cb_16bit_functor = std::function<void(OniData&)>;
using cb_888bit_functor = std::function<void(OniData&)>;

using namespace std;
using namespace openni;
using namespace std::chrono;
//class OniSensor;
class TuningWindow;
class CaptureDialog;
/**
*模组的类型
*/
typedef enum
{
	kDevicePleco = 0, /**< pleco整机类型 */
	KDeviceDothin = 1, /**< 度信模组类型 */
	kDeviceLX = 2, /**< 理想模组 */
	KDeviceMLX = 3, /**< 迈来星模组类型 */
} DeviceModeType;

/**
*深度的处理状态
*/
typedef enum
{
	kInvalid = 0, /**< 无效的phase数据 */
	kValid = 1, /**< 有效可用的phase数据 */
	kUsing = 2, /**< phase数据正在转深度中 */
} DepthDataType;

struct CameraInternalParam
{
    float ir_fx, ir_fy, ir_cx, ir_cy;
    float rgb_fx, rgb_fy, rgb_cx, rgb_cy;

    bool is_valid;

    CameraInternalParam& operator=(CameraInternalParam& R)
    {
        ir_fx = R.ir_fx;
        ir_fy = R.ir_fy;
        ir_cx = R.ir_cx;
        ir_cy = R.ir_cy;
        rgb_fx = R.rgb_fx;
        rgb_fy = R.rgb_fy;
        rgb_cx = R.rgb_cx;
        rgb_cy = R.rgb_cy;
        is_valid = R.is_valid;

        return *this;
    }

    void Rst()
    {
        ir_fx = ir_fy = ir_cx = ir_cy = 0.0f;
        rgb_fx = rgb_fy = rgb_cx = rgb_cy = 0.0f;
        is_valid = false;
    }
    CameraInternalParam() { Rst(); }
};

/* 缓存的phase数据，用于转深度 */
typedef struct _PHASE_GROUP_INFO
{
	int phaseSize;
	DepthDataType depthDataType;
	Metadata metaData;
	int8_t * phaseGroup;
	uint64_t frameTimeStamp;
}PHASE_GROUP_INFO;

struct RecorderInfo {
	int width;
	int height;
	int streamType;
	int fps;
	bool needStart;
};

class SensorBase : public openni::OpenNI::DeviceDisconnectedListener, public openni::OpenNI::DeviceConnectedListener
{
    //friend OniSensor;
    friend TuningWindow;    // 调试类
    friend CaptureDialog;
public:
	SensorBase();
    ~SensorBase();
public:
	void captureUVCColor();
    bool InitOpenNI2();
    void initialize();

    inline void CloseOpenNI2() { openni::OpenNI::shutdown(); };

    std::pair<int, std::string> OpenDevice(const char* uri);       /// 根据uri打开设备

    std::pair<int, std::string> OpenAnyDevice();                   /// 打开任意设备

    std::pair<int, std::string> InitFromOni(const char* oni_file);/// 打开.oni数据

    void CloseDevice();
    void onlyCloseDevice();
    bool SetTempFunc(bool enable);

    std::pair<int, std::string> QueryTempSupport();

    std::pair<int, std::string> QueryCamParam(CameraInternalParam &);

    int GetParamFromOni(const char* oni_file_name);

    CameraInternalParam GetObCamParams() const {
        return mCameraParam;
    }

    OBCameraParams GetDetailParams() const {
        return mCamParam;
    }
	bool GetObCameraParams(OBCameraParams &params);

	bool GetDothinCameraParams(OBCameraParams &params);

    bool QuerySupportedModes(vector<string> &support_mode_vec, openni::SensorType sensor_type);

    void SetConnectStatusCallback(const cb_functor& cb) { mCbFunc = cb; };

    void SetDepthCallback(const cb_16bit_functor& cb) { mCbDepthFunc = cb; };

    void SetIrCallback(const cb_16bit_functor& cb) { mCbIrFunc = cb; };
    void SetPhaseCallback(const cb_16bit_functor& cb) { mCbPhaseFunc = cb; };
    void SetColorCallback(const cb_888bit_functor& cb) { mCbRgbFunc = cb; };
	void SetOniCallback(const cb_888bit_functor& cb) { mCbOniFunc = cb; };
    //void SetDefaultRes(string depth_res, string ir_res, string color_res);

    int SwitchStreamMode(openni::SensorType sensor_type, std::string str_mode);
    int changeStreamResolution(openni::SensorType sensor_type,std::string str_mode);
	openni::VideoMode getStreamResolution(openni::SensorType sensor_type);
	int changeAIStreamType(PixelFormat pixexFormat);
    int StartStream(openni::SensorType sensor_type);
    int StopStream(openni::SensorType sensor_type);
	Status GetTofFrameMode(ObcTofFrameMode &frameMode);
	int SetTofFrameMode(ObcTofFrameMode frameMode,int phaseCount);
	int setDutyOneValue(float dutyValue);
	int setDutyTwoValue(float dutyValue);
    /**
     * @brief SetHardD2C 设置硬件对齐
     * @param on true - on, false - off
     */
    void SetHardD2C(bool on);

    /**
     * @brief SetSoftD2C 设置软件对齐
     * @param on true - on, false - off
     */
    void SetSoftD2C(bool on);


    /// Capture 采图

    /// 设置深度图采集的回调
    void SetDepthCaptureCallback(const cb_16bit_functor& cb) { mCbDepthCaptureFunc = cb; };

    /// 设置IR图采集的回调
    void SetIRCaptureCallback(const cb_16bit_functor& cb) { mCbIrCaptureFunc = cb; };
    /// 设置右IR图采集的回调
    void SetPhaseCaptureCallback(const cb_16bit_functor& cb) { mCbPhaseCaptureFunc = cb; };
    /// 设置Color图采集的回调
    void SetColorCaptureCallback(const cb_888bit_functor& cb) { mCbColorCaptureFunc = cb; };

    /// 设置pc采集的回调
    void SetPcCaptureCallback(const cb_16bit_functor& cb) { mCbPcCaptureFunc = cb; };

    void SetCSVCaptureCallback(const cb_16bit_functor& cb) { mCbCsvCaptureFunc = cb; }
    /**
     * @brief StartDepthCapture 启动深度采图
     * @param total 采集图片总数
     * @param step  采图的帧间隔
     */
    void StartDepthCapture(int total, int step);

    /**
     * @brief StartDepthCapture 启动IR采图
     * @param total 采集图片总数
     * @param step  采图的帧间隔
     */
    void StartIRCapture(int total, int step);
    /**
    * @brief StartDepthCapture 启动右IR采图
    * @param total 采集图片总数
    * @param step  采图的帧间隔
    */
    void StartPhaseCapture(int total, int step);
	void StartCSVCapture(int total);
    /**
     * @brief StartDepthCapture 启动Color采图
     * @param total 采集图片总数
     * @param step  采图的帧间隔
     */
    void StartColorCapture(int total, int step);

    /**
    * @brief StartDepthCapture 启动PC采图
    * @param total 采集图片总数
    * @param step  采图的帧间隔
    */
    void StartPcCapture(int total, int step);
    int ToggleMirror(bool &depth, bool &ir_color);

    bool IsOpened() { return mDeviceOpened; };
    bool setD2CProperty(bool status);
	bool setFrequencyForMore(int value);
    bool setAEProperty(bool status);
	bool getAEProperty(); 
	bool getMirrorProperty(int &mirrorType);
	bool setMirrorProperty(int mirrorType);
	bool setLaserProperty(bool status);
	bool setDothinLaser(bool status);
	bool getDothinLaser();
	bool setDothinFlood(bool status);
	bool getDothinFlood();
	bool setDsleepMode(bool status);
	bool setHdrAlgorithm(bool status);
	bool setHistAlgorithm(bool status);
	bool setMedianAlgorithm(bool status);
	bool setEbcAlgorithm(bool status);
	bool setLscAlgorithm(bool status);
	bool setCorrectionAlgorithm(bool status);
	bool setOutputMode(bool status);
	bool setOutputMode2(int mode);
	bool getOutputMode();
	int getOutputMode2();

	int setBiningMode(int mode);
	int setMirrorFlip(int mode);
	int setSubSamp(int val);
	int setSubSampv(int val);
	int setOddDgain(int mode);
	int setEvenDgain(int mode);
	int setWindowOriginy(int originy);
	int setWindowOriginx(int originx);
	int setWindowHeight(int height);
	int setWindowWidth(int width);
	bool setChipSoftReset();
	bool setChipHardReset();
	bool setLaserPower(int value);
	bool getLaserPower(int* value);
	bool setFloodPower(int value);
	bool getFloodPower(int* value);
	bool getLaserProperty();
	bool getD2CProperty();
	int getFrequencyProperty();
	bool loadAllRegister();
	SensorID getSensorId(const SensorType type);
	int getDonthinSensorId(int &sensor_id);
	int getColorCaptureTotal() const { return mColorCaptureTotal; }
	int getPhaseCaptureTotal() const { return mPhaseCaptureTotal; }
	int getDepthCaptureTotal() const { return mDepthCaptureTotal; }
	int getIrCaptureTotal() const { return mIrCaptureTotal; }
	int getPcCaptureTotal() const { return mPcCaptureTotal; }

	void setPointCloudState(bool isNeedCalc) { mNeedCalcPointCloud = isNeedCalc; }
	bool operateEEPROM(bool isWrite, int addr, int len, uint8_t *data);
	int setIntegrationTime(uint32_t  time);
	int getIntegrationTime();
	int getTXTemperature();
	//占空比列表
	int getDutyCycleList(OBDutyCycleSteps *steps);
	int getIndexByDutyCycle(uint32_t frequency, float duty_cycle);
	bool setSensorFreqDutyConfig(int index);
	int createStream(openni::SensorType sensor_type, shared_ptr<OniStreamBase> cur_stream);
	bool getChipID(int *value);
	bool getDelayTime(int *value);
	bool setDelayTime(int value);
	bool setIRGain(int value);
	bool getIRGain(int *value);
	bool setIRExposure(int value);
	bool getIRExposure(int *value);
	bool operateSensorReg(bool isWrite, int addr, int *value);
	int getSensorInfo(ObcSensorInfo &sensorInfo);
public:

    bool mDepthStreamOpen = false;
    bool mColorStreamOpen;
	bool mIrStreamOpen = false;
    bool mPhaseStreamOpen = false;
	bool mUvcColorStreamOpen;
	bool mAiStreamOpen=false;
	bool mLoadCaliterSuccess = false;
	bool mSoftwareAE = true;
	bool mSensorId = 0;
	int mDothinSensorId = 0;
	int mTxTempCount = 0;
	ObcTofFrameMode mTofFrameMode;
	DeviceModeType mDeviceModeType = kDevicePleco;
	shared_ptr<OniStreamBase>  mIrStreamInfo;
	shared_ptr<OniStreamBase>  mPhaseStreamInfo;
	shared_ptr<OniStreamBase>  mColorStreamInfo;
	shared_ptr<OniStreamBase>  mDepthStreamInfo;
	shared_ptr<OniStreamBase>  mOniStreamInfo;
	shared_ptr<OniStreamBase>  mAiStreamInfo;
	QList<AIBody>  mBodyLists;
	AIBody mAiSekeleton2d;
	std::mutex mMutexAiFrame;
	std::mutex mPointCloudMutex;

private:
    const char* GetFormatName(openni::PixelFormat format);
    void SnapDataThread(shared_ptr<OniStreamBase> cur_stream, int index);

    void ReadAnyStreamThread();	// it will use call_back function
	void ReadTXTemperatureThread();	// it will use call_back function
    virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo);

    virtual void onDeviceConnected(const openni::DeviceInfo* pInfo);
    /**
     * @brief CalcSoftD2DC 根据D2C矩阵对深度图做软件对齐
     * @param src_data 原始深度数据
     * @param dis_data 目标数据， 即转换结果
     * @param width 数据宽
     * @param height 数据高
     * @param d2c_mat 4x4 float矩阵
     * @return
     */
    bool CalcSoftD2DC(const uint16_t* src_data, uint16_t* dest_data, int width, int height, const float d2c_mat[16]);
    void snapOneDepthPicture(OniData &cur_data);
    void snapOneIrLeftPicture(OniData &cur_data);
    void snapOnePhasePicture(OniData &cur_data);
    void snapOneRgbPicture(OniData &cur_data);
    QString getCsvFileName(QString name);
    void saveCSVData(QString csv_name, QString file_path);
    //void copyTofData();
	void ResetMember();
private:
	std::string REGISTER_CONFIG_INI = "./config/RegisterConfig.ini";
	RegisterIniFile mInitRegister;
	RecorderInfo mPhaseRecorderInfo{ 0 };
	RecorderInfo mIRRecorderInfo{ 0 };
	RecorderInfo mDepthRecorderInfo{ 0 };
    cb_functor mCbFunc;
    cb_16bit_functor mCbDepthFunc;
    cb_16bit_functor mCbIrFunc;
    cb_16bit_functor mCbPhaseFunc;
    cb_888bit_functor mCbRgbFunc;
	cb_16bit_functor mCbOniFunc;
	
	float mDutyOneValue = 0;
	float mDutyTwoValue = 0;
    bool depthRecord{ false };
    bool iRRecord{ false };
    bool phaseRecord{ false };
    bool mSaveCsvStart{ false };
	int mDepthWidth = 640;
	int mDepthHeight = 480;
	float *mDepthBuffer;
	float *mPointcloudBuffer;
	float *mAmplitude;
	float *mIntensity;
    QFile *infoFile;
	OniData mPhaseTemp;
	OniData mIRTemp;
	PHASE_GROUP_INFO mPhaseGroupOne;
	PHASE_GROUP_INFO mPhaseGroupTwo;
    /// 用于连续采图的回调
    cb_16bit_functor mCbDepthCaptureFunc;
    
    cb_16bit_functor mCbIrCaptureFunc;
    cb_16bit_functor mCbPhaseCaptureFunc;

    cb_888bit_functor mCbColorCaptureFunc;
    cb_16bit_functor mCbPcCaptureFunc;
    cb_16bit_functor mCbCsvCaptureFunc;

	HANDLE mFilterConfig = nullptr;
	HANDLE mCaliParam = nullptr;
	const int kOutputParamBufSize = sizeof(float) * 100;
	int mRoi[4] = { 20,620,15,465 };
public:
    openni::Device  mOniDevice;
	openni::Metadata *mObTofInfo{ nullptr };
	std::queue<OniData> mPhaseQue;
	std::queue<OniData> mPhaseQueTwo;
	bool mDepthExitedThread = true;
	bool mTXTemperatureThread = true;
	bool mPhaseQueFinish{ false };//用来切换队列
	bool mPhaseCaptureStart{ false };
	bool mPhaseCaptureEnd{ false };
	int mPhaseCaputreCount = 0;
	int mPhaseCaputreTotal = 0;
	int mPhaseCaputreLastIndex = 0;
	int mDepthCaptureIndex = 0;
	int mIrCaptureIndex = 0;
	int mDepthStepIndex = 0;
	int mIrStepIndex = 0;
	int mPhaseCaptureIndex = 0;
	int mPhaseStepIndex = 0;
	int mColorCaptureIndex = 0;
	int mPcCaptureIndex = 0;
	int mIntegrationTime = 0;
	int mCalcIntegrationTime = 992;
	float mTXTemperature = 0;
	bool mDoExitedThread = false;
	bool mThreadHasExited = true;
	bool mOniThreadHasExited = true;
protected:
    char mUri[256];
    bool mDeviceOpened{ false };

    openni::Status  mOniStatus;
    
    OBCameraParams mCamParam;

    CameraInternalParam mCameraParam;

    float mD2cMat16[16];

    std::map <std::string, const openni::VideoMode*> mStreamVideoMap;

    std::mutex mMutex;
	std::mutex mMutexPhaseGroupOne;
	std::mutex mMutexPhaseGroupTwo;
    std::mutex mMutexPhase;
	std::mutex mMutexSetting;
    bool mEnableSoftD2c;

    Calc* mCalc;
    std::string mDepthCurrentRes;
    std::string mColorCurrentRes;
    std::string ir_current_res_;
    std::string mPhaseCurrentRes;
	OniData mOniDataCsv;
    int mDepthCaptureTotal{ 0 };
    int mDepthCaptureStep{ 0 };
    int mIrCaptureTotal{ 0 };
    int mPhaseCaptureTotal{ 0 };
	int mSaveCsvTotal{ 0 };
    int mIrCaptureStep{ 0 };
    int mPhaseCaptureStep{ 0 };
    int mColorCaptureTotal{ 0 };
    int mColorCaptureStep{ 0 };
    int mPcCaptureTotal{ 0 };
    int mPcCaptureStep{ 0 };
	int mCurrentGroupIndex{ -1 };
	int mNextFrameIndex{ -1 };
	bool mNeedCalcPointCloud{ false };
	
};



#endif
