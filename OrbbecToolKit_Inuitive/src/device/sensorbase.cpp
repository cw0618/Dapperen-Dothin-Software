#include "sensorbase.h"
#include <sstream>
#include <thread>
#include <time.h>
#include<QDateTime>
#include "src/orbbec/ObPng.h"
#include "src/calculate/calc.h"
#include"src/stb_image_write.h"
#include <QMessageBox>
SensorBase::SensorBase()
{
	initialize();
	mCalc = Calc::Instance();
}
SensorBase::~SensorBase()
{
	ResetMember();
}

void SensorBase::initialize() {
	//ResetMember();
	mAiStreamInfo = shared_ptr<OniStreamBase>(new OniStream16bit());
	mDepthStreamInfo = shared_ptr<OniStreamBase>(new OniStream16bit());
	mIrStreamInfo = shared_ptr<OniStreamBase>(new OniStream16bit());
	mPhaseStreamInfo = shared_ptr<OniStreamBase>(new OniStream16bit());
	mOniStreamInfo = shared_ptr<OniStreamBase>(new OniStream16bit());
	mColorStreamInfo = shared_ptr<OniStreamBase>(new OniStream888bit());

	mDepthBuffer = new float[mDepthWidth * mDepthHeight]();
	mPointcloudBuffer = new float[mDepthWidth * mDepthHeight * 3]();
	mAmplitude = new float[mDepthWidth * mDepthHeight]();
	mIntensity = new float[mDepthWidth * mDepthHeight]();

	mPhaseGroupOne.depthDataType = kInvalid;
	mPhaseGroupOne.phaseGroup = nullptr;
	mPhaseGroupTwo.depthDataType = kInvalid;
	mPhaseGroupTwo.phaseGroup = nullptr;
	if (false == InitOpenNI2()) {
		qDebug() << QString("Failed to init openni2().");
	}
	mAiSekeleton2d.setJointFormat(PixelFormat::PIXEL_FORMAT_NONE);
	memset(mD2cMat16, 0xff, sizeof(mD2cMat16));
	memset(&mTofFrameMode,0,sizeof(mTofFrameMode));
	mTofFrameMode.out_mode = OB_TOF_OUT_MODE_3TAP;
}
std::pair<int, std::string> SensorBase::OpenDevice(const char * uri)
{
	if (nullptr == uri) {
		return std::make_pair(-1, std::string("failed to open device, uri is nullptr."));
	}

	openni::Status ret = mOniDevice.open(uri);
	if (openni::STATUS_OK != ret) {
		return std::make_pair(-1, std::string("failed to open device, ret is").append(std::to_string(ret)));
	}

	return std::make_pair(0, std::string());
}

std::pair<int, std::string> SensorBase::InitFromOni(const char* oni_file)
{
	if (nullptr == oni_file) {
		return std::make_pair(-1, std::string("init frome oni filed, oni_file is nullptr."));
	}

	if (mOniDevice.isValid())
	{
		CloseDevice();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	openni::Status ret = mOniDevice.open(oni_file);
	if (openni::STATUS_OK != ret) {
		return std::make_pair(-2, std::string("failed to open device by oni, ret is").append(std::to_string(ret)));
	}

	int reti = GetParamFromOni(oni_file);
	if (0 != reti) {
		return std::make_pair(-3, std::string("failed to get paramter from oni, ret is ").append(std::to_string(reti)));
	}

	return std::make_pair(0, std::string());
}

std::pair<int, std::string> SensorBase::OpenAnyDevice()
{
	if (true == mDeviceOpened) {
		return std::make_pair(-1, std::string("failed to open any device, is_device_opened_ = true."));    /// 不要重复打开
	}

	int i = 0;
	openni::Status ret = mOniDevice.open(openni::ANY_DEVICE);
	//while (openni::STATUS_OK != ret)
	//{
	//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//	ret = mOniDevice.open(openni::ANY_DEVICE);

	//	if (++i > 10) {
	//		break;
	//	}
	//}
	mDeviceOpened = (openni::STATUS_OK == ret);
	if (openni::STATUS_OK != ret) {
		return std::make_pair(-3, std::string("failed to OpenAnyDevice, ret is ").append(std::to_string(ret)));
	}
	//const char* deviceName=mOniDevice.getDeviceInfo().getName();
	//if (strcmp(deviceName,"Dothin")==0)
	//{
	//	mDeviceModeType = KDeviceDothin;
	//}
	return std::make_pair(0, std::string());
}
void SensorBase::onlyCloseDevice() {
	mDepthStreamOpen = false;
	mIrStreamOpen = false;
	mPhaseStreamOpen = false;
	mUvcColorStreamOpen = false;
	mColorStreamOpen = false;
	if (mOniDevice.isValid())
	{
		mOniDevice.close();
		mDeviceOpened = false;
	}
}
void SensorBase::CloseDevice()
{
	mStreamVideoMap.clear();
	mDoExitedThread = true;
	mDepthExitedThread = true;
	clock_t start, end;
	start = clock();

	while (!mThreadHasExited)
	{
		end = clock();
		if (((end - start) / CLOCKS_PER_SEC) > 6)
		{
			break;
		}
	}
	mAiStreamInfo->DestroyStream();
	mDepthStreamInfo->DestroyStream();
	mIrStreamInfo->DestroyStream();
	mPhaseStreamInfo->DestroyStream();
	mColorStreamInfo->DestroyStream();
	mOniStreamInfo->DestroyStream();

	mDepthStreamOpen = false;
	mIrStreamOpen = false;
	mPhaseStreamOpen = false;
	mUvcColorStreamOpen = false;
	mColorStreamOpen = false;
	mDepthCurrentRes.clear();
	ir_current_res_.clear();
	mPhaseCurrentRes.clear();
	mColorCurrentRes.clear();

	if (mOniDevice.isValid())
	{
		mOniDevice.close();
		mDeviceOpened = false;
	}
}

bool SensorBase::SetTempFunc(bool enable)
{
	if (!mOniDevice.isValid())
		return false;

	int dataSize = 4;
	int temperature_en = enable ? 1 : 0; //1:开启温度补偿; 0:关闭温度补偿功能
	openni::Status rc = mOniDevice.setProperty(XN_MODULE_PROPERTY_TEMP_COMP, (uint8_t *)&temperature_en, dataSize);
	if (openni::STATUS_OK != rc) {
		return false;
	}

	return true;
}

std::pair<int, std::string> SensorBase::QueryTempSupport()
{
	if (!mOniDevice.isValid()) {
		return std::make_pair(-1, std::string("failed to query sn, oni_device_ is nullptr."));
	}

	int dataSize = 4;
	int temperature_en = 0;
	openni::Status ret = mOniDevice.getProperty(XN_MODULE_PROPERTY_TEMP_COMP, (uint8_t *)&temperature_en, &dataSize);

	if (openni::STATUS_OK != ret) {
		return std::make_pair(-2, std::string("failed to query TEMP_COMP, ret is ").append(std::to_string(ret)));
	}

	return std::make_pair(0, std::string());
}

std::pair<int, std::string> SensorBase::QueryCamParam(CameraInternalParam & par)
{
	if (!mOniDevice.isValid()) {
		return std::make_pair(-1, std::string("failed to query paramter, oni_device_ is nullptr."));
	}

	int size = sizeof(mCamParam);
	openni::Status ret = mOniDevice.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&mCamParam, &size);
	if (openni::STATUS_OK != ret) {
		return std::make_pair(-2, std::string("failed to query paramter, get property return ").append(std::to_string(ret)));
	}

	par.ir_fx = mCamParam.l_intr_p[0] = isnan(mCamParam.l_intr_p[0]) ? 0 : mCamParam.l_intr_p[0];
	par.ir_fy = mCamParam.l_intr_p[1] = isnan(mCamParam.l_intr_p[1]) ? 0 : mCamParam.l_intr_p[1];
	par.ir_cx = mCamParam.l_intr_p[2] = isnan(mCamParam.l_intr_p[2]) ? 0 : mCamParam.l_intr_p[2];
	par.ir_cy = mCamParam.l_intr_p[3] = isnan(mCamParam.l_intr_p[3]) ? 0 : mCamParam.l_intr_p[3];

	par.rgb_fx = mCamParam.r_intr_p[0] = isnan(mCamParam.r_intr_p[0]) ? 0 : mCamParam.r_intr_p[0];
	par.rgb_fy = mCamParam.r_intr_p[1] = isnan(mCamParam.r_intr_p[1]) ? 0 : mCamParam.r_intr_p[1];
	par.rgb_cx = mCamParam.r_intr_p[2] = isnan(mCamParam.r_intr_p[2]) ? 0 : mCamParam.r_intr_p[2];
	par.rgb_cy = mCamParam.r_intr_p[3] = isnan(mCamParam.r_intr_p[3]) ? 0 : mCamParam.r_intr_p[3];

	if (par.ir_fx < 1 && par.ir_fy < 1 && par.ir_cx < 1 && par.ir_cy < 1
		&& par.rgb_fx < 1 && par.rgb_fy < 1 && par.rgb_cx < 1 && par.rgb_cy < 1)
	{
		memset(&mCamParam, 0, sizeof(mCamParam));
		par.is_valid = false;
		return std::make_pair(-2, std::string("the paramter is empty."));
	}

	mCalc->calcParamMat(mCamParam, mD2cMat16);

	mCameraParam = par;

	par.is_valid = true;
	return std::make_pair(0, std::string());
}

bool SensorBase::GetObCameraParams(OBCameraParams & params)
{
	if (!mOniDevice.isValid()) {
		return false;
	}
	float l_intrin[5];
	float l_distort_coeff[5];
	//int ret = GetToFIntrinsic(m_calibParamBuf, l_intrin, l_distort_coeff);
	//if (ret == 0) {
	//	mCamParam.l_intr_p[0] = l_intrin[0];
	//	mCamParam.l_intr_p[1] = l_intrin[1];
	//	mCamParam.l_intr_p[2] = l_intrin[2];
	//	mCamParam.l_intr_p[3] = l_intrin[3];
	//	mCamParam.l_k[0] = l_distort_coeff[0];
	//	mCamParam.l_k[1] = l_distort_coeff[1];
	//	mCamParam.l_k[2] = l_distort_coeff[2];
	//	mCamParam.l_k[3] = l_distort_coeff[3];
	//	mCamParam.l_k[4] = l_distort_coeff[4];
	//}
	if (mCamParam.l_intr_p[0] < 1 || mCamParam.l_intr_p[1] < 1)
	{
		return false;
	}
	params = mCamParam;
	return true;
}

bool SensorBase::GetDothinCameraParams(OBCameraParams &params)
{
	if (!mOniDevice.isValid()) {
		return false;
	}
	float l_intrin[5];
	float l_distort_coeff[5];
	//int ret = GetToFIntrinsic(m_calibParamBuf, l_intrin, l_distort_coeff);
	//if (ret == 0) {
	//	mCamParam.l_intr_p[0] = l_intrin[0];
	//	mCamParam.l_intr_p[1] = l_intrin[1];
	//	mCamParam.l_intr_p[2] = l_intrin[2];
	//	mCamParam.l_intr_p[3] = l_intrin[3];
	//	mCamParam.l_k[0] = l_distort_coeff[0];
	//	mCamParam.l_k[1] = l_distort_coeff[1];
	//	mCamParam.l_k[2] = l_distort_coeff[2];
	//	mCamParam.l_k[3] = l_distort_coeff[3];
	//	mCamParam.l_k[4] = l_distort_coeff[4];
	//}
	if (mCamParam.l_intr_p[0] < 1 || mCamParam.l_intr_p[1] < 1)
	{
		return false;
	}
	params = mCamParam;
	return true;
}

bool SensorBase::QuerySupportedModes(vector<string> &support_mode_vec, openni::SensorType sensor_type)
{
	if (!mOniDevice.isValid()) {
		return false;
	}

	std::string str_stream_type;
	const openni::SensorInfo *stream_info_ = mOniDevice.getSensorInfo(sensor_type);
	switch (sensor_type)
	{
	case openni::SENSOR_IR:
		mIrStreamInfo->mStreamInfo = stream_info_;
		str_stream_type = "IR";
		break;
	case openni::SENSOR_PHASE:
		mPhaseStreamInfo->mStreamInfo = stream_info_;
		mOniStreamInfo->mStreamInfo = stream_info_;
		str_stream_type = "Phase";
		break;
	case openni::SENSOR_COLOR:
		mColorStreamInfo->mStreamInfo = stream_info_;
		str_stream_type = "Color";
		break;
	case openni::SENSOR_DEPTH:
		mDepthStreamInfo->mStreamInfo = stream_info_;
		str_stream_type = "Depth";
		break;
	case openni::SENSOR_AI:
		mAiStreamInfo->mStreamInfo = stream_info_;
		str_stream_type = "Ai";
		break;

	default:
		break;
	}

	support_mode_vec.clear();

	std::string first_str;
	if (nullptr != stream_info_)
	{
		const openni::Array<openni::VideoMode>& SupportedModes = stream_info_->getSupportedVideoModes();
		int sz = SupportedModes.getSize();
		for (int i = 0; i < sz; ++i)
		{
			const openni::VideoMode* pSupportedMode = &SupportedModes[i];

			if ((sensor_type == openni::SENSOR_COLOR && pSupportedMode->getPixelFormat() == openni::PIXEL_FORMAT_RGB888)
				|| (sensor_type == openni::SENSOR_DEPTH && pSupportedMode->getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM)
				|| (sensor_type == openni::SENSOR_PHASE && pSupportedMode->getPixelFormat() == openni::PIXEL_FORMAT_GRAY16)
				|| (sensor_type == openni::SENSOR_IR && pSupportedMode->getPixelFormat() == openni::PIXEL_FORMAT_GRAY16))
			{
				stringstream ss;
				ss << pSupportedMode->getResolutionX() << "x"
					<< pSupportedMode->getResolutionY();

				std::string str_tmp = ss.str();
				support_mode_vec.push_back(str_tmp);
				if (0 == i) {
					first_str = str_tmp;
				}
				//QString str_stream = QString::fromStdString(str_tmp);
				//qDebug() << "howard sensor_type =" << sensor_type << "resolution = " << str_stream << pSupportedMode->getPixelFormat();
				mStreamVideoMap.insert(std::make_pair(str_tmp, pSupportedMode));
			}

			else if (sensor_type == openni::SENSOR_AI)
			{

				PixelFormat format = pSupportedMode->getPixelFormat();
				stringstream ss;
				ss << format;
				std::string str_tmp = ss.str();

				support_mode_vec.push_back(str_tmp);
				mStreamVideoMap.insert(std::make_pair(str_tmp, pSupportedMode));
			}
			else {
				continue;
			}
		}
	}

	return true;
}

// Snap one frame hread
void SensorBase::SnapDataThread(shared_ptr<OniStreamBase> cur_stream, int index)
{
	clock_t start, end;
	start = clock();

	int stream_index = -1;
	int fream_index = 0;
	while (true)
	{
		end = clock();
		if (((end - start) / CLOCKS_PER_SEC) > 180) {
			break;
		}
		if (fream_index > index) {
			break;
		}

		openni::VideoStream* stream[] = { &(cur_stream->mVideoStream) };
		openni::Status ret = openni::OpenNI::waitForAnyStream(stream, 1, &stream_index, 200);
		if (openni::STATUS_OK == ret)
		{
			mMutex.lock();
			ret = cur_stream->mVideoStream.readFrame(&cur_stream->mVideoFrameRef);
			mMutex.unlock();
			if (openni::STATUS_OK != ret) {
				stream_index = 0;
			}
			else {
				++fream_index;
			}
		}
		else {
			stream_index = 0;
		}
	}
}
void SensorBase::ReadTXTemperatureThread()
{
	while (!mTXTemperatureThread)
	{
		if (mTxTempCount >= 10)
		{
			mTxTempCount = 0;
			getTXTemperature();
		}
		else 
		{
			mTxTempCount++;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(400));
	}
	mTXTemperatureThread = true;
}

void SensorBase::ReadAnyStreamThread()
{
	mMutex.lock();
	mThreadHasExited = false;
	mMutex.unlock();
	int stream_index = -1;
	int phaseCount = 1;
	while (!mDoExitedThread)
	{
		openni::VideoStream* streams[] = {&(mPhaseStreamInfo->mVideoStream) };
		uint32_t numStream = sizeof(streams) / sizeof(streams[0]);
		mOniStatus = openni::OpenNI::waitForAnyStream(streams, numStream, &stream_index, 0);
		QString msg;
		if (openni::STATUS_OK == mOniStatus)
		{
			mMutex.lock();
			system_clock::time_point tp = system_clock::now();
			system_clock::duration dtn = tp.time_since_epoch();
			milliseconds mil = duration_cast<milliseconds>(dtn);
			uint64_t time_s = mil.count();
			switch (stream_index)
			{
			case 0:	// phase
			{
				mPhaseStreamInfo->mStreamStatus = true;
				mPhaseStreamInfo->mVideoStream.readFrame(&mPhaseStreamInfo->mVideoFrameRef);
				mPhaseStreamInfo->mOniDataInfo.mFrameTimeStamp = mPhaseStreamInfo->mVideoFrameRef.getTimestamp();
				OniFrame* oniFrame = mPhaseStreamInfo->mVideoFrameRef._getFrame();
				if (oniFrame != nullptr)
				{
					phaseCount = oniFrame->extraLine;
				}
				Metadata ob_tof_data = mPhaseStreamInfo->mVideoFrameRef.getMetadata();
				//qDebug() << "getFrameIndex = " << ob_tof_data.getFrameIndex() << "getLowFreq = " << ob_tof_data.getLowFreq();;
				if (mPhaseStreamOpen)
				{
					mPhaseStreamInfo->mOniDataInfo.mTofExteraline = ob_tof_data;
					mPhaseStreamInfo->UpdatePhaseInfo(ob_tof_data);

					mCbPhaseFunc(mPhaseStreamInfo->mOniDataInfo);
					//if (ob_tof_data.getSensorMode() == 5 || ob_tof_data.getSensorMode() == 1 || ob_tof_data.getSensorMode() == 2)
					//{
					//	//减背景模式，根据ebd数据中的context
					//	if (ob_tof_data.getFrameType() == 0)
					//	{
					//		mCbPhaseFunc(mPhaseStreamInfo->mOniDataInfo);
					//	}
					//	else
					//	{
					//		mCbIrFunc(mPhaseStreamInfo->mOniDataInfo);
					//	}
					//}
					//else 
					//{
					//	mCbPhaseFunc(mPhaseStreamInfo->mOniDataInfo);
					//}
					//采图
					if (mPhaseCaptureIndex < mPhaseCaptureTotal)
					{
						if (!mPhaseStreamInfo->mCaptureFull)
						{
							if (mPhaseCaptureStep == 0 || mPhaseCaptureStep == mPhaseStepIndex)
							{
								mPhaseTemp.mLogicId = mPhaseCaptureIndex;
								mPhaseTemp.mCustomTimeStamp = time_s;
								mPhaseTemp.mWidth = mPhaseStreamInfo->mOniDataInfo.mWidth;
								mPhaseTemp.mHeight = mPhaseStreamInfo->mOniDataInfo.mHeight;
								mPhaseTemp.mDataSizeBytes = mPhaseStreamInfo->mOniDataInfo.mDataSizeBytes;
								mPhaseTemp.mFrameTimeStamp = mPhaseStreamInfo->mOniDataInfo.mFrameTimeStamp;
								mPhaseTemp.mPixelInBytes = mPhaseStreamInfo->mOniDataInfo.mPixelInBytes;
								mPhaseTemp.mTofExteraline = ob_tof_data;
								mPhaseTemp.mPhaseNumber = phaseCount;
								mPhaseTemp.mOutMode = mTofFrameMode.out_mode;
								for (int i = 0; i < phaseCount; i++)
								{
									mPhaseTemp.mTofExteraline.frameIndex = i;
									mPhaseStreamInfo->GetVideoFrameData(mPhaseTemp, i);
									mCbPhaseCaptureFunc(mPhaseTemp);
								}
								mPhaseCaptureIndex++;
								mPhaseStepIndex = 0;
							}
							else
							{
								mPhaseStepIndex++;
							}

						}
					}
					else {
						mPhaseCaptureIndex = 0;
						mPhaseCaptureTotal = 0;
					}
					if (mSaveCsvTotal == -1)
					{
						mPhaseTemp.mWidth = mPhaseStreamInfo->mOniDataInfo.mWidth;
						mPhaseTemp.mHeight = mPhaseStreamInfo->mOniDataInfo.mHeight;
						mPhaseTemp.mDataSizeBytes = mPhaseStreamInfo->mOniDataInfo.mDataSizeBytes;
						mPhaseTemp.mFrameTimeStamp = mPhaseStreamInfo->mOniDataInfo.mFrameTimeStamp;
						mPhaseTemp.mPixelInBytes = mPhaseStreamInfo->mOniDataInfo.mPixelInBytes;
						mPhaseTemp.mTofExteraline.mode = ONI_S5K_4TAP_DUAL_FREQ_4FRAME;
						mPhaseTemp.mTofExteraline = ob_tof_data;
						mPhaseTemp.mPhaseNumber = phaseCount;
						mPhaseTemp.mOutMode = mTofFrameMode.out_mode;
						for (int i = 0; i < phaseCount; i++)
						{
							mPhaseTemp.mTofExteraline.frameIndex = i;
							mPhaseStreamInfo->GetVideoFrameData(mPhaseTemp, i);
							mCbCsvCaptureFunc(mPhaseTemp);
						}
					}
				}
				break;
			}
			default:
				break;
			}

			mMutex.unlock();
		}
	}
	mMutex.lock();
	mThreadHasExited = true;
	mMutex.unlock();
}
void SensorBase::captureUVCColor() {
	mMutex.lock();
	system_clock::time_point tp = system_clock::now();
	system_clock::duration dtn = tp.time_since_epoch();
	milliseconds mil = duration_cast<milliseconds>(dtn);
	uint64_t time_s = mil.count();
	mColorStreamInfo->mOniDataInfo.mCustomTimeStamp = time_s;
	if (mColorCaptureIndex < mColorCaptureTotal)
	{
		if (0 == (mColorCaptureIndex % (mColorCaptureStep + 1))) {
			mCbColorCaptureFunc(mColorStreamInfo->mOniDataInfo); //
		}
		if (mColorStreamInfo->mOniDataInfo.isCaptureSuccess) {
			mColorCaptureIndex++;
		}

	}
	else {
		mColorCaptureIndex = 0;
		mColorCaptureTotal = 0;
	}
	mMutex.unlock();
}
void SensorBase::StartDepthCapture(int total, int step)
{
	mDepthCaptureStep = step;
	mDepthCaptureTotal = total;
	mDepthCaptureIndex = 0;
	mDepthStepIndex = 0;
}

void SensorBase::StartIRCapture(int total, int step)
{
	mIrCaptureStep = step;
	mIrCaptureTotal = total;
	mIrCaptureIndex = 0;
	mIrStepIndex = 0;
}
void SensorBase::StartPhaseCapture(int total, int step)
{
	mPhaseCaptureStep = step;
	mPhaseCaptureTotal = total;
	mPhaseCaptureIndex = 0;
	mPhaseStepIndex = 0;
}
void SensorBase::StartCSVCapture(int total)
{
	mSaveCsvTotal = total;
}
void SensorBase::StartColorCapture(int total, int step)
{
	mColorCaptureStep = step;
	mColorCaptureTotal = total;
}

void SensorBase::StartPcCapture(int total, int step)
{
	mPcCaptureStep = step;
	mPcCaptureTotal = total;
}

int SensorBase::ToggleMirror(bool &b_depth_mirror, bool &b_ir_color_mirror)
{
	mMutex.lock();
	if (mDepthStreamInfo->mVideoStream.isValid())
	{
		mDepthStreamInfo->mVideoStream.setMirroringEnabled(!mDepthStreamInfo->mVideoStream.getMirroringEnabled());
		b_depth_mirror = mDepthStreamInfo->mVideoStream.getMirroringEnabled();
	}
	if (mIrStreamInfo->mVideoStream.isValid())
	{
		mIrStreamInfo->mVideoStream.setMirroringEnabled(!mIrStreamInfo->mVideoStream.getMirroringEnabled());
		b_ir_color_mirror = mIrStreamInfo->mVideoStream.getMirroringEnabled();
	}
	if (mPhaseStreamInfo->mVideoStream.isValid())
	{
		mPhaseStreamInfo->mVideoStream.setMirroringEnabled(!mPhaseStreamInfo->mVideoStream.getMirroringEnabled());
		b_ir_color_mirror = mPhaseStreamInfo->mVideoStream.getMirroringEnabled();
	}
	if (mColorStreamInfo->mVideoStream.isValid())
	{
		mColorStreamInfo->mVideoStream.setMirroringEnabled(!mColorStreamInfo->mVideoStream.getMirroringEnabled());
		b_ir_color_mirror = mColorStreamInfo->mVideoStream.getMirroringEnabled();
	}
	mMutex.unlock();

	return 0;
}

void SensorBase::onDeviceDisconnected(const openni::DeviceInfo * pInfo)
{
	int cp_res = strcmp(pInfo->getUri(), mUri);
	if (0 == cp_res) {
		memset(mUri, 0, sizeof(mUri));
	}

	mCbFunc(false);
}

void SensorBase::onDeviceConnected(const openni::DeviceInfo* pInfo)
{
	const char* uri = pInfo->getUri();
	if (strstr(uri, ".oni") <= 0)
	{
		strcpy_s(mUri, pInfo->getUri());
		mCbFunc(true);
	}
}
int SensorBase::changeAIStreamType(PixelFormat pixexFormat) {
	int ret_status = 0;
	if (!mOniDevice.isValid()) {
		ret_status - 1;
	}
	openni::VideoMode mode = mAiStreamInfo->mVideoStream.getVideoMode();
	mode.setPixelFormat(pixexFormat);
	ret_status = mAiStreamInfo->mVideoStream.setVideoMode(mode);
	return ret_status;
}
openni::VideoMode SensorBase::getStreamResolution(openni::SensorType sensor_type) {
	openni::VideoMode mode;
	if (!mOniDevice.isValid())
		return mode;
	shared_ptr<OniStreamBase> sp_cur_stream = mDepthStreamInfo;

	switch (sensor_type)
	{
	case openni::SensorType::SENSOR_DEPTH:

		sp_cur_stream = mDepthStreamInfo;
		break;
	case openni::SensorType::SENSOR_IR:
		sp_cur_stream = mIrStreamInfo;
		break;
	case openni::SensorType::SENSOR_PHASE:
		sp_cur_stream = mPhaseStreamInfo;
		break;
	case openni::SensorType::SENSOR_COLOR:
		sp_cur_stream = mColorStreamInfo;
		break;
	}
	mode = sp_cur_stream->mVideoStream.getVideoMode();
	return mode;
}
int SensorBase::changeStreamResolution(openni::SensorType sensor_type, std::string str_mode) {
	int ret_status = 0;
	if (!mOniDevice.isValid())
	{
		return -1;
	}
	shared_ptr<OniStreamBase> sp_cur_stream = mDepthStreamInfo;

	switch (sensor_type)
	{
	case openni::SensorType::SENSOR_DEPTH:
		if (str_mode == "") {
			str_mode = mDepthCurrentRes;
		}
		else {
			mDepthCurrentRes = str_mode;
		}
		sp_cur_stream = mDepthStreamInfo;
		break;
	case openni::SensorType::SENSOR_IR:
		if (str_mode == "") {
			str_mode = ir_current_res_;
		}
		else {
			ir_current_res_ = str_mode;
		}
		sp_cur_stream = mIrStreamInfo;
		break;
	case openni::SensorType::SENSOR_PHASE:

		if (str_mode == "") {
			str_mode = mPhaseCurrentRes;
		}
		else {
			mPhaseCurrentRes = str_mode;
		}
		sp_cur_stream = mPhaseStreamInfo;
		break;
	case openni::SensorType::SENSOR_COLOR:
		if (str_mode == "") {
			str_mode = mColorCurrentRes;
		}
		else {
			mColorCurrentRes = str_mode;
		}
		sp_cur_stream = mColorStreamInfo;
		break;
	}

	QString mode_value = QString::fromStdString(str_mode);
	QStringList list = mode_value.split("x");

	openni::VideoMode mode = sp_cur_stream->mVideoStream.getVideoMode();
	//mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	int width = list[0].toInt();
	int height = list[1].toInt();
	qDebug() << "changeresolution==" << width << " x " << height;
	mode.setResolution(width, height);
	ret_status = sp_cur_stream->mVideoStream.setVideoMode(mode);

	ret_status = StartStream(sensor_type);
	return ret_status;
}
int SensorBase::SwitchStreamMode(openni::SensorType sensor_type, std::string str_mode)
{
	int ret_status = 0;
	if (!mOniDevice.isValid())
		ret_status = -1;
	shared_ptr<OniStreamBase> sp_cur_stream = mDepthStreamInfo;
	switch (sensor_type)
	{
	case openni::SensorType::SENSOR_DEPTH:
		if (str_mode == "") {
			str_mode = mDepthCurrentRes;
		}
		else {
			mDepthCurrentRes = str_mode;
		}
		sp_cur_stream = mDepthStreamInfo;
		break;
	case openni::SensorType::SENSOR_IR:
		if (str_mode == "") {
			str_mode = ir_current_res_;
		}
		else {
			ir_current_res_ = str_mode;
		}
		sp_cur_stream = mIrStreamInfo;
		break;
	case openni::SensorType::SENSOR_PHASE:

		if (str_mode == "") {
			str_mode = mPhaseCurrentRes;
		}
		else {
			mPhaseCurrentRes = str_mode;
		}
		sp_cur_stream = mPhaseStreamInfo;
		break;
	case openni::SensorType::SENSOR_COLOR:
		if (str_mode == "") {
			str_mode = mColorCurrentRes;
		}
		else {
			mColorCurrentRes = str_mode;
		}
		sp_cur_stream = mColorStreamInfo;
		break;
	case openni::SensorType::SENSOR_AI:

		sp_cur_stream = mAiStreamInfo;
		break;
	}
	QString resolution_str = QString::fromStdString(str_mode);
	//qDebug() << "howard str_mode=" << resolution_str;
	const openni::VideoMode* model_to_open = nullptr;
	std::map <std::string, const openni::VideoMode*>::iterator it = mStreamVideoMap.find(str_mode);
	if (it != mStreamVideoMap.end()) {
		model_to_open = it->second;
	}
	if (nullptr == model_to_open && sensor_type != SENSOR_AI) {
		ret_status = -2;
	}
	int ret = createStream(sensor_type, sp_cur_stream);
	if (0 != ret) {
		string err = openni::OpenNI::getExtendedError();
		QString  msg = QString("OpenStream error: %1").arg(QString::fromStdString(err));
		qDebug() << msg;
		ret_status = -3;
	}
	if (sensor_type == openni::SensorType::SENSOR_DEPTH)
	{
		//QFileInfo filterFile(FILTER_INI_FILE_AB);
		//if (filterFile.exists()) {
		//	ret_status = loadFilterFile(filterFile.absoluteFilePath().toLocal8Bit().data());
		//}
		//QFileInfo caliFile(CALI_INI_FILE);
		//if ((ret_status ==0) && filterFile.exists()) {
		//	ret_status = loadCaliterFile(caliFile.absoluteFilePath().toLocal8Bit().data());
		//}
	}
	return ret_status;
}

int SensorBase::StopStream(openni::SensorType sensor_type)
{
	int ret_status = 0;
	if (openni::SENSOR_DEPTH == sensor_type)
	{
		ret_status = mDepthStreamInfo->StopStream();
		mDepthStreamInfo->mStreamStatus = false;
		mDepthStreamOpen = false;
	}
	else if (openni::SENSOR_IR == sensor_type)
	{
		ret_status = mIrStreamInfo->StopStream();
		mIrStreamInfo->mStreamStatus = false;
		mIrStreamOpen = false;
	}
	else if (openni::SENSOR_COLOR == sensor_type)
	{
		ret_status = mColorStreamInfo->StopStream();
		mColorStreamInfo->mStreamStatus = false;
		mColorStreamOpen = false;
	}
	else if (openni::SENSOR_PHASE == sensor_type)
	{
		ret_status = mPhaseStreamInfo->StopStream();
		mPhaseStreamOpen = false;
	}
	else if (openni::SENSOR_AI == sensor_type)
	{
		if (mAiStreamOpen)
		{
			ret_status = mAiStreamInfo->StopStream();
		}
		mPhaseStreamOpen = false;
		mAiStreamInfo->mStreamStatus = false;
		mAiStreamOpen = false;
	}
	return ret_status;
}
int SensorBase::StartStream(openni::SensorType sensor_type)
{
	int ret_status = 0;
	bool *stream_open_flag = &mDepthStreamOpen;
	// ir and color stream cann't open on the same time
	if (openni::SENSOR_DEPTH == sensor_type)
	{
		mDepthStreamInfo->mStreamStatus = true;
		ret_status = mDepthStreamInfo->StartStream();
		mDepthStreamOpen = true;
	}
	else if (openni::SENSOR_COLOR == sensor_type)
	{
		if (!mColorStreamOpen)
		{
			stream_open_flag = &mColorStreamOpen;
			mColorStreamInfo->mStreamStatus = true;
			ret_status = mColorStreamInfo->StartStream();
			mColorStreamOpen = true;
		}

		//std::this_thread::yield();
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	else if (openni::SENSOR_PHASE == sensor_type)
	{
		stream_open_flag = &mPhaseStreamOpen;
		mPhaseStreamInfo->mStreamStatus = true;
		ret_status = mPhaseStreamInfo->StartStream();
		mPhaseStreamOpen = true;

	}
	else if (openni::SENSOR_IR == sensor_type)
	{
		if (!mIrStreamOpen)
		{
			stream_open_flag = &mIrStreamOpen;
			mIrStreamInfo->mStreamStatus = true;
			ret_status = mIrStreamInfo->StartStream();
			mIrStreamOpen = true;
		}
	}
	else if (openni::SENSOR_AI == sensor_type)
	{
		if (!mAiStreamOpen)
		{
			stream_open_flag = &mAiStreamOpen;
			mAiStreamInfo->mStreamStatus = true;
			ret_status = mAiStreamInfo->StartStream();
			mAiStreamOpen = true;
			//mPhaseStreamOpen = true;
		}
	}
	if (mThreadHasExited)
	{
		qDebug() << "start thread";
		mDoExitedThread = false;
		std::thread t_(&SensorBase::ReadAnyStreamThread, this);
		t_.detach();
	}
	*stream_open_flag = true;
	return ret_status;
}

void SensorBase::SetHardD2C(bool on)
{
	if (true == on) {
		mOniDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	else {
		mOniDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	}
}

void SensorBase::SetSoftD2C(bool on)
{
	mEnableSoftD2c = on;
}

int SensorBase::createStream(openni::SensorType sensor_type, shared_ptr<OniStreamBase>cur_stream)
{
	if (!mOniDevice.isValid())
		return false;

	cur_stream->mStreamInfo = mOniDevice.getSensorInfo(sensor_type);
	if (nullptr == cur_stream->mStreamInfo) {
		return -1;
	}

	if (cur_stream->mVideoStream.isValid())
	{
		cur_stream->mVideoStream.stop();
		cur_stream->mVideoFrameRef.release(); /// ?
	}
	else
	{

		openni::Status st = cur_stream->mVideoStream.create(mOniDevice, sensor_type);
		if (openni::STATUS_OK != st) {
			return -2;
		}

	}
	return 0;
}

void SensorBase::ResetMember()
{
	mOniStatus = openni::Status::STATUS_ERROR;
	mDoExitedThread = true;
	mDepthExitedThread = true;
	memset(mUri, 0, sizeof(mUri));
	memset(&mCamParam, 0, sizeof(mCamParam));

	memset(mD2cMat16, 0, sizeof(mD2cMat16));

	mThreadHasExited = true;

	mTXTemperatureThread = true;

	mEnableSoftD2c = false;

	mDepthStreamOpen = false;
	mColorStreamOpen = false;
	mPhaseStreamOpen = false;
	mUvcColorStreamOpen = false;
	mIrStreamOpen = false;
	if (nullptr != mAmplitude)
	{
		delete[] mAmplitude;
		mAmplitude = nullptr;
	}
	if (nullptr != mIntensity)
	{
		delete[] mIntensity;
		mIntensity = nullptr;
	}
	if (mDepthBuffer != nullptr)
	{
		delete[] mDepthBuffer;
	}
	if (mPointcloudBuffer != nullptr)
	{
		delete[] mPointcloudBuffer;
	}

	if (mPhaseGroupOne.phaseGroup != nullptr)
	{
		delete[] mPhaseGroupOne.phaseGroup;
		mPhaseGroupOne.phaseGroup = nullptr;
	}
	if (mPhaseGroupTwo.phaseGroup != nullptr)
	{
		delete[] mPhaseGroupTwo.phaseGroup;
		mPhaseGroupTwo.phaseGroup = nullptr;
	}
}

const char * SensorBase::GetFormatName(openni::PixelFormat format)
{
	switch (format)
	{
	case openni::PIXEL_FORMAT_DEPTH_1_MM:
		return "1 mm";
	case openni::PIXEL_FORMAT_DEPTH_100_UM:
		return "100 um";
	case openni::PIXEL_FORMAT_SHIFT_9_2:
		return "Shifts 9.2";
	case openni::PIXEL_FORMAT_SHIFT_9_3:
		return "Shifts 9.3";
	case openni::PIXEL_FORMAT_RGB888:
		return "RGB 888";
	case openni::PIXEL_FORMAT_YUV422:
		return "YUV 422";
	case openni::PIXEL_FORMAT_YUYV:
		return "YUYV";
	case openni::PIXEL_FORMAT_GRAY8:
		return "Gray 8-bit";	//return "Grayscale 8-bit";
	case openni::PIXEL_FORMAT_GRAY16:
		return "Gray 16-bit";	// return "Grayscale 16-bit";
	case openni::PIXEL_FORMAT_JPEG:
		return "JPEG";
	default:
		return "Unknown";
	}
}

bool SensorBase::SensorBase::InitOpenNI2()
{
	mOniStatus = openni::OpenNI::initialize();
	if (openni::STATUS_OK != mOniStatus) {
		return false;
	}

	// Register to OpenNI events.
	mOniStatus = openni::OpenNI::addDeviceDisconnectedListener(this);
	if (openni::STATUS_OK != mOniStatus) {
		return false;
	}

	mOniStatus = openni::OpenNI::addDeviceConnectedListener(this);
	if (openni::STATUS_OK != mOniStatus) {
		return false;
	}

	return true;
};


bool SensorBase::CalcSoftD2DC(const uint16_t* src_data, uint16_t* dest_data, int width, int height, const float d2c_mat[16])
{
	uint16_t* dest_buffer = new uint16_t[width * height * sizeof(uint16_t)]();

	uint16_t x_d2c = 0, y_d2c = 0;
	double dZ = 0.0;
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			uint32_t iIndex = h * width + w;
			uint16_t depth_value = src_data[iIndex];
			if (depth_value < 10)
				continue;

			dZ = (double)depth_value;
			x_d2c = (uint16_t)(d2c_mat[0] * (double)w + d2c_mat[1] * (double)h + d2c_mat[2] + d2c_mat[3] / dZ);
			y_d2c = (uint16_t)(d2c_mat[4] * (double)w + d2c_mat[5] * (double)h + d2c_mat[6] + d2c_mat[7] / dZ);
			if (x_d2c < 0 || x_d2c >= width || y_d2c < 0 || y_d2c >= height)
				continue;

			uint32_t d2c_Index = y_d2c * width + x_d2c;
			dest_buffer[d2c_Index] = depth_value;
		}
	}

	memcpy(dest_data, dest_buffer, width * height * sizeof(uint16_t));

	delete dest_buffer;
	dest_buffer = nullptr;

	return true;
}

// 获取内外参
int SensorBase::GetParamFromOni(const char* oniFileName)
{
	FILE *file = NULL;
	fopen_s(&file, oniFileName, "rb");
	if (file == NULL)
	{
		fclose(file);
		return -1;
	}

	fseek(file, 0, SEEK_END);           //定位到文件末
	uint32_t file_size = ftell(file);   //文件长度
	if (file_size < 1000)
	{
		fclose(file);
		return -2;
	}

	int iParamsSize = sizeof(OBCameraParams);
	memset(&mCamParam, 0, iParamsSize);

	char* mack = new char[sizeof(char) * 3]();
	std::shared_ptr<char> smart_ptr_mack = std::shared_ptr<char>(mack); //使用智能指针管理buf对象, 可自动释放


	fseek(file, -3, SEEK_END);           //定位到文件末
	fread(mack, 1, 3, file);

	char* exitMack = new char[sizeof(char) * 3]();
	memset(exitMack, 0xAB, 3);
	std::shared_ptr<char> smart_ptr_exitmack = std::shared_ptr<char>(exitMack); //使用智能指针管理buf对象, 可自动释放

	//比较
	int iDiff = 0;
	for (int i = 0; i < 3; ++i)
	{
		if (exitMack[i] != mack[i]) {
			iDiff++;
		}
	}

	if (0 == iDiff)
	{
		// 读取内外参到cam_param_
		fseek(file, -(3 + iParamsSize), SEEK_END);           //定位到文件末
		fread(&mCamParam, 1, iParamsSize, file);

		mCameraParam.ir_fx = mCamParam.l_intr_p[0] = isnan(mCamParam.l_intr_p[0]) ? 0 : mCamParam.l_intr_p[0];
		mCameraParam.ir_fy = mCamParam.l_intr_p[1] = isnan(mCamParam.l_intr_p[1]) ? 0 : mCamParam.l_intr_p[1];
		mCameraParam.ir_cx = mCamParam.l_intr_p[2] = isnan(mCamParam.l_intr_p[2]) ? 0 : mCamParam.l_intr_p[2];
		mCameraParam.ir_cy = mCamParam.l_intr_p[3] = isnan(mCamParam.l_intr_p[3]) ? 0 : mCamParam.l_intr_p[3];

		mCameraParam.rgb_fx = mCamParam.r_intr_p[0] = isnan(mCamParam.r_intr_p[0]) ? 0 : mCamParam.r_intr_p[0];
		mCameraParam.rgb_fy = mCamParam.r_intr_p[1] = isnan(mCamParam.r_intr_p[1]) ? 0 : mCamParam.r_intr_p[1];
		mCameraParam.rgb_cx = mCamParam.r_intr_p[2] = isnan(mCamParam.r_intr_p[2]) ? 0 : mCamParam.r_intr_p[2];
		mCameraParam.rgb_cy = mCamParam.r_intr_p[3] = isnan(mCamParam.r_intr_p[3]) ? 0 : mCamParam.r_intr_p[3];

		mCameraParam.is_valid = true;

		mCalc->calcParamMat(mCamParam, mD2cMat16);
	}
	else
	{
		fclose(file);
		return -3;
	}

	fclose(file);
	return 0;
}

bool SensorBase::loadAllRegister()
{
	QString register_ini = QString::fromStdString(REGISTER_CONFIG_INI);
	QByteArray ba = register_ini.toLocal8Bit();
	char *c_str = ba.data();
	QString registerINI = QString::fromLocal8Bit(c_str);
	int iret = mInitRegister.LoadRegisterIni(registerINI);
	if (0 != iret)
	{
		QString msg = QString("Loading %1 failed").arg(registerINI);
		qDebug() << msg;
		return false;
	}
	int allSize = mInitRegister.mRegisterList.size();
	qDebug() << "mRegisterList.size() = " << allSize;
	for (int i = 0; i < allSize; i++)
	{
		REGISTER_DATA registerData = mInitRegister.mRegisterList.at(i);
		int addr = registerData.address.toInt(nullptr, 16);
		int value = registerData.value.toInt(nullptr, 16);
		bool status = operateSensorReg(true, addr, &value);
		if (status)
		{
			
			qDebug() << "set register success addr = " << registerData.address << " value = " << registerData.value;
		}
		else
		{
			qDebug() << "set register failure addr = " << registerData.address << " value = " << registerData.value;
		}
	}
	return true;
}

void SensorBase::snapOneDepthPicture(OniData &cur_data) {
	QDateTime date_time;
	int64_t  time_sec = QDateTime::currentMSecsSinceEpoch();
	QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
	QString path = QDir::currentPath() + "/Data/Picture";
	QDir cur_file_dir_g;
	if (!cur_file_dir_g.exists(path)) {
		cur_file_dir_g.mkpath(path);
	}
	ObPng ob_png;
	// save depth png
	//depth_分辨率_距离_顺序_时间（ms）
	QString full_name_png = QString("%1/%2_%3_%4_%5x%6_%7_%8.png")
		.arg(path).arg(timestamp).arg("0000").arg("DEPTH").arg(cur_data.mWidth).arg(cur_data.mHeight).arg("400mm").arg(cur_data.mFrameIndex);
	QByteArray ba = full_name_png.toLocal8Bit();
	char* c_full_name_png = ba.data();

	writeInfo write_info;
	write_info.width = cur_data.mWidth;
	write_info.height = cur_data.mHeight;
	write_info.bit_depth = 16;
	write_info.channels = 1;
	write_info.compression_level = 2;
	ob_png.WriteObpng(c_full_name_png, reinterpret_cast<uint16_t*>(cur_data.oData.mDataPtr), write_info);
}
void SensorBase::snapOneIrLeftPicture(OniData &cur_data) {
	QDateTime date_time;
	int64_t  time_sec = QDateTime::currentMSecsSinceEpoch();
	QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
	QString path = QDir::currentPath() + "/Data/Picture";
	QDir cur_file_dir_g;
	if (!cur_file_dir_g.exists(path)) {
		cur_file_dir_g.mkpath(path);
	}
	/// 保存PNG
	//depth_分辨率_距离_顺序_时间（ms）
	QString full_name_png = QString("%1/left_nomo_%2x%3_%4_%5_%6.png")
		.arg(path).arg(cur_data.mWidth).arg(cur_data.mHeight).arg("400mm").arg(timestamp).arg(cur_data.mFrameIndex);
	//.arg(temp);

	QByteArray ba = full_name_png.toLocal8Bit();
	char* c_full_name_png = ba.data();
	ObPng ob_png;
	writeInfo write_info;
	write_info.width = cur_data.mWidth;
	write_info.height = cur_data.mHeight;

	write_info.compression_level = 2;
	if (2 == cur_data.mPixelInBytes)
	{
		write_info.bit_depth = 16;
		write_info.channels = 1;
		ob_png.WriteObpng(c_full_name_png, reinterpret_cast<uint16_t*>(cur_data.oData.mDataPtr), write_info);
		QString file_path = QString("/Data/ir_%1x%2_%3_%4_%5.png")
			.arg(cur_data.mWidth).arg(cur_data.mHeight).arg("400mm").arg(cur_data.mFrameIndex).arg(timestamp);
		//		QString csv_name = getCsvFileName("left_nomo");
		//		saveCSVData(csv_name, file_path);
	}
	else if (3 == cur_data.mPixelInBytes) {
		write_info.bit_depth = 8;
		write_info.channels = 3;
	}
}
void SensorBase::snapOnePhasePicture(OniData &cur_data) {

}
void SensorBase::snapOneRgbPicture(OniData &cur_data) {
	QDateTime date_time;
	int64_t  time_sec = QDateTime::currentMSecsSinceEpoch();
	QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
	QString path = QDir::currentPath() + "/capture/Picture";
	QDir cur_file_dir_g;
	if (!cur_file_dir_g.exists(path)) {
		cur_file_dir_g.mkpath(path);
	}
	QString full_name_png = QString("%1/%2_%3_%4_%5x%6_%7_%8.png")
		.arg(path).arg(timestamp).arg("0000").arg("COLOR").arg(cur_data.mWidth).arg(cur_data.mHeight).arg("400mm").arg(cur_data.mFrameIndex);

	QByteArray ba = full_name_png.toLocal8Bit();
	char* c_full_name_png = ba.data();

	auto size = cur_data.mWidth * cur_data.mHeight * 3;
	stbi_write_png(c_full_name_png, cur_data.mWidth, cur_data.mHeight, 3, cur_data.oData.mDataPtr, cur_data.mWidth * 3);
}
QString SensorBase::getCsvFileName(QString name) {
	QString csv_name = name.append("_").append(QDateTime::currentDateTime().toString("yyyy-MM-dd")).append(".csv");
	return csv_name;
}
void SensorBase::saveCSVData(QString csv_name, QString file_path) {

	QString save_csv_path = QDir::currentPath() + "/CSV";
	QDir dir(save_csv_path);
	if (!dir.exists()) {
		dir.mkdir(save_csv_path);
	}
	save_csv_path.append("/").append(csv_name);
	infoFile = new QFile(save_csv_path);
	if (!infoFile->exists()) {
		infoFile->open(QIODevice::Append | QIODevice::Text | QIODevice::ReadWrite);
		QString title = "picture Path,Time,\n";
		infoFile->write(title.toUtf8().data());
		infoFile->close();
	}
	//QString time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
	QDateTime* data_time = new QDateTime(QDateTime::currentDateTime());
	QString timestamp = data_time->toString("MMddhhmmsszzz");
	infoFile->open(QIODevice::Append | QIODevice::Text | QIODevice::ReadWrite);
	infoFile->write(QString("%1%2").arg(file_path).arg(",").toUtf8().data());
	infoFile->write(QString("%1%2").arg(timestamp).arg(",").toUtf8().data());
	infoFile->write("\n");
	infoFile->close();

}

bool SensorBase::setFrequencyForMore(int value) {
	openni::Status rc = openni::STATUS_ERROR;
	openni::FrequencyMode mode;
	qDebug() << "howard setFrequencyForMore" << value;
	switch (value)
	{
		//case -1:
		//	mode = openni::FrequencyMode::FREQUENCY_MODE_NONE;
		//	break;
	case 0:
		mode = openni::FrequencyMode::SINGLE_FREQ_NOSHUFFLE;
		break;
	case 1:
		mode = openni::FrequencyMode::DUAL_FREQ_NOSHUFFLE;
		break;
	case 2:
		mode = openni::FrequencyMode::SINGLE_FREQ_SHUFFLE;
		break;
	case 3:
		mode = openni::FrequencyMode::DUAL_FREQ_SHUFFLE;
		break;
	case 4:
		mode = openni::FrequencyMode::SINGLE_FREQ_NOSHUFFLE_BINNING;
		break;
	case 5:
		mode = openni::FrequencyMode::DUAL_FREQ_NOSHUFFLE_BINNING;
		break;
	case 6:
		mode = openni::FrequencyMode::SINGLE_FREQ_SHUFFLE_BINNING;
		break;
	case 7:
		mode = openni::FrequencyMode::DUAL_FREQ_SHUFFLE_BINNING;
		break;
	default:
		break;
	}
	rc = mOniDevice.setFrequencyMode(mode);

	qDebug() << "set Frequency Mode :" << mode;

	if (openni::STATUS_OK != rc)
	{
		qDebug("Error: %s\n", openni::OpenNI::getExtendedError());

	}
	return openni::STATUS_OK == rc;
}
bool SensorBase::setD2CProperty(bool status) {
	openni::Status rc = openni::STATUS_ERROR;
	if (status) {
		rc = mOniDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	else {
		rc = mOniDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	}
	return openni::STATUS_OK == rc;
}
bool SensorBase::setAEProperty(bool status) {

	//openni::Status rc = mOniDevice.setProperty(XN_MODULE_PROPERTY_AE, status);
	//return openni::STATUS_OK == rc;
	mSoftwareAE = status;
	return true;
}
bool SensorBase::getAEProperty() {
	//int nValue = 0;
	//openni::Status rc = mOniDevice.getProperty(XN_MODULE_PROPERTY_AE, &nValue);
	//if (rc != openni::STATUS_OK)
	//{
	//	qDebug("Error: %s\n", openni::OpenNI::getExtendedError());
	//}
	//else
	//{
	//	qDebug("Ae status: %d\n", nValue);
	//}
	//return nValue == 1;
	return mSoftwareAE;
}

bool SensorBase::getMirrorProperty(int &mirrorType)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}

	uint16_t get_value = 0;
	int size = sizeof(get_value);
	int ret = mOniDevice.getProperty(OBC_MIRROR_FLIP, (void *)&get_value, &size);

	if (ret == 0) {
		mirrorType = get_value;
		qDebug() << "get OBC_TOF_MIRROR_FLIP success, flag:" << get_value;
		return true;
	}
	else {
		qDebug() << "get OBC_TOF_MIRROR_FLIP failed ret=:" << ret;
	}
	return false;
}

bool SensorBase::setMirrorProperty(int mirrorType)
{
	if (!IsOpened()) {
		return false;
	}

	int size = sizeof(mirrorType);
	int ret = mOniDevice.setProperty(OBC_MIRROR_FLIP, (void *)&mirrorType, size);

	if (ret == 0) {

		qDebug() << "set OBC_TOF_MIRROR_FLIP success, flag:" << mirrorType;
		return true;
	}
	else {
		qDebug() << "set OBC_TOF_MIRROR_FLIP failed ret=:" << ret;
	}
	return false;
}

bool SensorBase::setLaserProperty(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	openni::Status rc = mOniDevice.emitterEnable(status);
	return openni::STATUS_OK == rc;
}

bool SensorBase::setDothinLaser(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = status ? 1 : 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.setProperty(OBC_LASER, (void *)&illumPower, size);
	if (ret != 0) {
		qDebug() << "setDothinLaser failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::getDothinLaser()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.getProperty(OBC_LASER, (void *)&illumPower, &size);
	if (ret != 0) {
		qDebug() << "getDothinLaser failed ret:" << ret;
	}
	else {
		return illumPower == 1;
	}
	return false;
}

bool SensorBase::setDothinFlood(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = status ? 1 : 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.setProperty(OBC_FLOOD_LED, (void *)&illumPower, size);
	if (ret != 0) {
		qDebug() << "setDothinFlood failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::getDothinFlood()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.getProperty(OBC_FLOOD_LED, (void *)&illumPower, &size);
	if (ret != 0) {
		qDebug() << "getDothinFlood failed ret:" << ret;
	}
	else {
		return illumPower == 1;
	}
	return false;
}

bool SensorBase::setDsleepMode(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_DEEPSLEEP_MODE, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setDsleepMode failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setHdrAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_HDR_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setHdrAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setHistAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_HIST_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setHistAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setMedianAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_MEDIAN_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setMedianAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setEbcAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_EBC_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setEbcAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setLscAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_LSC_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setLscAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setCorrectionAlgorithm(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int mode = status ? 1 : 0;
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_CORRECTION_ALGORITHM, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setCorrectionAlgorithm failed ret:" << ret;
	}

	return ret == 0;
}


bool SensorBase::setOutputMode(bool status) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = status ? 5 : 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.setProperty(OBC_DATA_OUTPUT_MODE, (void *)&illumPower, size);
	if (ret != 0) {
		qDebug() << "setOutputMode failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setOutputMode2(int mode) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_DATA_OUTPUT_MODE, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setOutputMode2 failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::getOutputMode()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.getProperty(OBC_DATA_OUTPUT_MODE, (void *)&illumPower, &size);
	if (ret != 0) {
		qDebug() << "getOutputMode failed ret:" << ret;
	}
	else {
		return illumPower == 5;
	}
	return false;
}

int SensorBase::getOutputMode2()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 0;
	int size = sizeof(illumPower);
	int ret = mOniDevice.getProperty(OBC_DATA_OUTPUT_MODE, (void *)&illumPower, &size);
	qDebug() << "getOutputMode success, mode:" << illumPower;
	if (ret != 0) {
		qDebug() << "getOutputMode failed ret:" << ret;
	}
	else {
		return illumPower;
	}
	return false;
}

int SensorBase::setBiningMode(int mode) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_BINNING_MODE, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setBiningMode failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setMirrorFlip(int mode) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_MIRROR_FLIP, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setMirrorFlipMode failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setSubSamp(int val) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(val);
	int ret = mOniDevice.setProperty(OBC_SUB_SAMP, (void *)&val, size);
	if (ret != 0) {
		qDebug() << "setSubSamp failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setSubSampv(int val) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(val);
	int ret = mOniDevice.setProperty(OBC_SUB_SAMP_V, (void *)&val, size);
	if (ret != 0) {
		qDebug() << "setSubSampv failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setOddDgain(int mode) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_ODD_DGAIN, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setOddDgain failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setEvenDgain(int mode) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(mode);
	int ret = mOniDevice.setProperty(OBC_EVEN_DGAIN, (void *)&mode, size);
	if (ret != 0) {
		qDebug() << "setEvenDgain failed ret:" << ret;
	}
	return ret == 0;
}


int SensorBase::setWindowOriginy(int originy) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(originy);
	int ret = mOniDevice.setProperty(OBC_WINDOW_ORIGINY, (void *)&originy, size);
	if (ret != 0) {
		qDebug() << "setWindowOriginy failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setWindowOriginx(int originx) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(originx);
	int ret = mOniDevice.setProperty(OBC_WINDOW_ORIGINX, (void *)&originx, size);
	if (ret != 0) {
		qDebug() << "setWindowOriginx failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setWindowHeight(int height) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(height);
	int ret = mOniDevice.setProperty(OBC_WINDOW_HEIGHT, (void *)&height, size);
	if (ret != 0) {
		qDebug() << "setWindowHeight failed ret:" << ret;
	}
	return ret == 0;
}

int SensorBase::setWindowWidth(int width) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int size = sizeof(width);
	int ret = mOniDevice.setProperty(OBC_WINDOW_WIDTH, (void *)&width, size);
	if (ret != 0) {
		qDebug() << "setWindowWidth failed ret:" << ret;
	}
	return ret == 0;
}
bool SensorBase::setChipHardReset()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 1;
	int size = sizeof(illumPower);
	int ret = mOniDevice.setProperty(OBC_DEVICE_RESET, (void *)&illumPower, size);
	if (ret != 0) {
		qDebug() << "setChipHardReset failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setChipSoftReset()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!IsOpened()) {
		return false;
	}
	int illumPower = 1;
	int size = sizeof(illumPower);
	int ret = mOniDevice.setProperty(OBC_CHIP_RESET, (void *)&illumPower, size);
	if (ret != 0) {
		qDebug() << "setChipSoftReset failed ret:" << ret;
	}

	return ret == 0;
}

bool SensorBase::setLaserPower(int value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	int ret = 0;
	if (!IsOpened()) {
		return false;
	}
	int laserValue = value;
	int size = sizeof(laserValue);
	ret = mOniDevice.setProperty(OBC_LASER_CURRENT, (void *)&laserValue, size);
	if (ret == 0) {
		
		qDebug() << "setLaserPower, flag:" << laserValue;
		return true;
	}
	else {
		qDebug() << "setLaserPower failed ret=:" << ret;
	}
	 
    return false;
}

bool SensorBase::getLaserPower(int* value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	int ret = 0;
	if (!IsOpened()) {
		return false;
	}
	int laserValue = 0;
	int size = sizeof(laserValue);
	ret = mOniDevice.getProperty(OBC_LASER_CURRENT, (void *)&laserValue, &size);
	if (ret != 0) {
		qDebug() << "getIllumPower failed ret:" << ret;
	}
	else {
		*value = laserValue;
		qDebug() << "getIllumPower success laserValue:" << laserValue;
		return true;
	}
	return false;
}

bool SensorBase::setFloodPower(int value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	int ret = 0;
	if (!IsOpened()) {
		return false;
	}
	int floodValue = value;
	int size = sizeof(floodValue);
	ret = mOniDevice.setProperty(OBC_FLOOD_CURRENT, (void *)&floodValue, size);
	if (ret == 0) {
		qDebug() << "setFloodPower success ret:" << floodValue;
		return true;
	}
	else
	{
		qDebug() << "setFloodPower failed ret:" << ret;
	}
	return false;
}

bool SensorBase::getFloodPower(int* value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	int ret = 0;
	if (!IsOpened()) {
		return false;
	}
	int floodValue = 0;
	int size = sizeof(floodValue);
	ret = mOniDevice.getProperty(OBC_FLOOD_CURRENT, (void *)&floodValue, &size);
	if (ret != 0) {
		qDebug() << "getFloodPower failed ret:" << ret;
	}
	else {
		*value = floodValue;
		qDebug() << "getFloodPower success laserValue:" << floodValue;
		return true;
	}
	return false;
}

bool SensorBase::getLaserProperty() {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	return mOniDevice.emitterState();
}

bool SensorBase::getD2CProperty() {

	openni::ImageRegistrationMode rc = mOniDevice.getImageRegistrationMode();
	qDebug() << "getD2CProperty " << rc;
	return rc == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
}
SensorID SensorBase::getSensorId(const SensorType type) {
	return mOniDevice.getSensorID(type);
}
int SensorBase::getFrequencyProperty() {

	openni::FrequencyMode mode = mOniDevice.getFrequencyMode();
	return mode;

}

int SensorBase::setIntegrationTime(uint32_t  time)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return -5001;
	}
	int ret = 0;
	int size = sizeof(time);
	ret = mOniDevice.setProperty(OBC_INTEGRATION_TIME, (void *)&time, size);
	if (ret != 0) {
		qDebug() << "setIntegrationTime failed ret:" << ret;;
	}
	else {
		//qDebug() << "setIntegrationTime success value:" << time;
	}
	return ret;
}
int SensorBase::getIntegrationTime()
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return -5001;
	}
	int ret = 0;
	openni::Status rc = mOniDevice.getProperty(OBC_INTEGRATION_TIME, &mIntegrationTime);
	if (rc != openni::STATUS_OK)
	{
		qDebug("integrationTime Error: %s\n", openni::OpenNI::getExtendedError());
	}
	else
	{
		//qDebug("integrationTime status: %d\n", mIntegrationTime);
	}
	return mIntegrationTime;
}
int SensorBase::getTXTemperature() {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return -5001;
	}
	openni::Status rc = mOniDevice.getProperty(OBC_TX_TEMP, &mTXTemperature);
	if (rc != openni::STATUS_OK)
	{
		qDebug("mTXTemperature Error: %s\n", openni::OpenNI::getExtendedError());
	}
	else
	{
		//qDebug("mTXTemperature status: %d\n", mTXTemperature);
	}
	return mTXTemperature;
}
//占空比列表
int SensorBase::getDutyCycleList(OBDutyCycleSteps *steps)
{
	int ret = 0;
	if (!mDeviceOpened) {
		return -5001;
	}
	int size = sizeof(OBDutyCycleSteps);
	ret = mOniDevice.getProperty(OBC_DUTY_CYCLE_LIST, (void *)steps, &size);
	if (ret != 0) {
		qDebug() << "getDutyCycleList failed ret:" << ret;
	}
	else
	{
		qDebug() << "getDutyCycleList success";
	}
	return ret;
}

int SensorBase::getDonthinSensorId(int &sensor_id)
{
	int iSize = sizeof(int);
	int iRet = mOniDevice.getProperty(OBC_SENSOR_ID, &sensor_id, &iSize);
	if (iRet == 0)
	{
		mDothinSensorId = sensor_id;
	}
	else
	{
		qDebug() << "MODULATION_FREQUENCY  ret= " << iRet;
	}
	ObcSensorInfo sensorInfo;
	getSensorInfo(sensorInfo);
	if (sensorInfo.vcsel_driver_id == 1)
	{
		mDeviceModeType = kDeviceLX;
	}
	else if (sensorInfo.vcsel_driver_id == 2)
	{
		mDeviceModeType = KDeviceMLX;
	}
	else
	{
		mDeviceModeType = KDeviceDothin;
	}
	qDebug() << "mDeviceModeType  ret= " << mDeviceModeType;
	return iRet;
}

int SensorBase::SetTofFrameMode(ObcTofFrameMode frameMode, int phaseCount)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return -5001;
	}

	int iRet = mOniDevice.setProperty(OBC_GROUP_FRAME_MODE, &frameMode, sizeof(ObcTofFrameMode));
	if (iRet == 0)
	{
		mIntegrationTime = frameMode.requestfreq[0].integration_time;
	}
	else
	{
		qDebug() << "SetTofFrameMode failure ret= " << iRet;
	}
	if (frameMode.out_mode == OB_TOF_OUT_MODE_A_B)
	{
		openni::VideoMode mode = mPhaseStreamInfo->mVideoStream.getVideoMode();
		mode.setResolution(640, 480);
		mPhaseStreamInfo->mVideoStream.setVideoMode(mode);
	}
	else if (frameMode.out_mode == OB_TOF_OUT_MODE_A_AND_B)
	{
		openni::VideoMode mode = mPhaseStreamInfo->mVideoStream.getVideoMode();
		mode.setResolution(1280, 480);
		mPhaseStreamInfo->mVideoStream.setVideoMode(mode);
	}
	else
	{
		openni::VideoMode mode = mPhaseStreamInfo->mVideoStream.getVideoMode();
		mode.setResolution(1280, 480);
		mPhaseStreamInfo->mVideoStream.setVideoMode(mode);
	}
	//if (mTofFrameMode.out_mode != frameMode.out_mode)
	//{
	//	if (mTofFrameMode.out_mode == OB_TOF_OUT_MODE_A_B)
	//	{
	//		QFileInfo filterFile(FILTER_INI_FILE_A_B);
	//		if (filterFile.exists()) {
	//			char *path = filterFile.absoluteFilePath().toLocal8Bit().data();
	//			mFilterConfig = LoadToFFilterParam(const_cast<char*>(path));
	//		}
	//	}
	//	else
	//	{
	//		QFileInfo filterFile(FILTER_INI_FILE_AB);
	//		if (filterFile.exists()) {
	//			char *path = filterFile.absoluteFilePath().toLocal8Bit().data();
	//			mFilterConfig = LoadToFFilterParam(const_cast<char*>(path));
	//		}
	//	}
	//}
	uint8_t mUpdateFrameMode = 0;
	int iSize = sizeof(mUpdateFrameMode);
	mPhaseStreamInfo->mVideoStream.getProperty(OBEXTENSION_ID_UPDATE_FRAMEMODE, &mUpdateFrameMode, &iSize);
	mTofFrameMode = frameMode;
	return iRet;
}

Status SensorBase::GetTofFrameMode(ObcTofFrameMode & frameMode)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return STATUS_ERROR;
	}
	ObcTofFrameMode tofFrameMode;
	int frameModeSize = sizeof(tofFrameMode);
	Status iRet = mOniDevice.getProperty(OBC_GROUP_FRAME_MODE, &tofFrameMode, &frameModeSize);
	if (iRet == STATUS_OK)
	{
		frameMode = tofFrameMode;
		mTofFrameMode = tofFrameMode;
	}
	else
	{
		qDebug() << "GetTofFrameMode failed ret:" << iRet;
	}

	return iRet;
}

int SensorBase::getIndexByDutyCycle(uint32_t frequency, float duty_cycle)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	OBDutyCycleSteps steps0{ 0 };
	steps0.frequency = frequency;
	qDebug() << "getIndexByDutyCycle" << frequency;
	int ret = getDutyCycleList(&steps0);

	if (ret != 0) {
		qDebug() << "getDutyCycleList failed.";
		return -1;
	}
	for (int i = 0; i < OB_DUTY_CYCLE_LIST_NUM; i++) {
		float duty0 = steps0.duty_cycle_steps[i];
		if (duty0 == duty_cycle) {
			return i;
		}
	}
	qDebug() << "getIndexByDutyCycle failed. duty_cycle:" << duty_cycle;
	return -1;

}

bool SensorBase::setSensorFreqDutyConfig(int index)
{
	if (!mDeviceOpened) {
		return false;
	}
	if (mDothinSensorId != OBC_SENSOR_ID_IMX518 && mDothinSensorId != OBC_SENSOR_ID_IMX316 && mDothinSensorId != OBC_SENSOR_ID_IMX627)
	{
		return false;
	}
	ObcSensorFreqDutyConfig sensorFreqDutyConfig = { 1 };
	sensorFreqDutyConfig.index = index;
	int size = sizeof(sensorFreqDutyConfig);

	int ret = mOniDevice.setProperty(OBC_SENSOR_FREQUENCY_DUTY_CONFIG, (void *)&sensorFreqDutyConfig, size);
	if (ret == 0) {

		qDebug() << "set OBC_SENSOR_FREQUENCY_DUTY_CONFIG success: " << index;

		return true;
	}
	else {
		qDebug() << "set OBC_SENSOR_FREQUENCY_DUTY_CONFIG failed ret=:" << ret;
	}
	return false;
}
int SensorBase::setDutyOneValue(float dutyValue)
{
	mDutyOneValue = ((int)dutyValue)/100.0f;
	return 0;
}

int SensorBase::setDutyTwoValue(float dutyValue)
{
	mDutyTwoValue = ((int)dutyValue) / 100.0f;
	return 0;
}

bool SensorBase::operateEEPROM(bool isWrite, int addr, int len, uint8_t *data)
{
	return false;
	int size, ret;
	if (!IsOpened() && (mDeviceModeType != KDeviceMLX)) {
		return false;
	}

	if (isWrite) {
		ObcEEPROMData eeprom_data;
		eeprom_data.addr = addr;
		eeprom_data.len = len;
		eeprom_data.data = data;

		size = sizeof(eeprom_data);
		ret = mOniDevice.setProperty(OBC_EEPROM_RW, (void *)&eeprom_data, size);
		if (ret != 0)
		{
			qDebug() << "Write eeprom failed ,ret :" << ret;
			return false;
		}
	}
	else {
		ObcEEPROMData eeprom_data_read;
		eeprom_data_read.addr = addr;
		eeprom_data_read.len = len;
		eeprom_data_read.data = data;
		size = sizeof(eeprom_data_read);

		ret = mOniDevice.getProperty(OBC_EEPROM_RW, (void *)&eeprom_data_read, &size);
		if (ret != 0)
		{
			qDebug() << "Read eeprom failed ,ret :" << ret;
			return false;
		}
	}

	qDebug() << "operateEEPROM OK";

	return true;
}

bool SensorBase::setIRGain(int value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}
	int size = sizeof(value);
	int ret = mOniDevice.setProperty(OBC_IR_GAIN, (void *)&value, size);
	
	if (ret == 0) {
		qDebug() << "Set gain success, mode: " << value;
		return true;
	}
	else 
	{
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("Set gain error: %1").arg(QString::fromStdString(err));
		qDebug() << msg;
	}
	return false;
}

bool SensorBase::getIRGain(int *value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}

	int get_value = 0;
	int size = sizeof(get_value);
	int ret = mOniDevice.getProperty(OBC_IR_GAIN, (void *)&get_value, &size);

	if (ret == 0) {
		*value = get_value;
		qDebug() << "get gain success, flag:" << get_value;
		return true;
	}
	else {
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("get gain error: %1").arg(QString::fromStdString(err));
		qDebug() << msg;
	}
	return false;
}

bool SensorBase::setIRExposure(int value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}
	int size = sizeof(value);
	int ret = mOniDevice.setProperty(OBC_IR_EXP, (void *)&value, size);

	if (ret == 0) {
		qDebug() << "Set exposure success, mode: " << value;
		return true;
	}
	else
	{
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("Set exposure error: %1").arg(QString::fromStdString(err));
		qDebug() << msg;
	}
	return false;
}

bool SensorBase::getIRExposure(int *value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}

	int get_value = 0;
	int size = sizeof(get_value);
	int ret = mOniDevice.getProperty(OBC_IR_EXP, (void *)&get_value, &size);

	if (ret == 0) {
		*value = get_value;
		qDebug() << "get exposure success, flag:" << get_value;
		return true;
	}
	else {
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("get exposure error: %1").arg(QString::fromStdString(err));
		qDebug() << msg;
	}
	return false;
}

bool SensorBase::operateSensorReg(bool isWrite, int addr, int *value)
{
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}
	obc_reg_16_map_t reg_map;
	reg_map.addr = addr;
	reg_map.value = *value;
	int size = sizeof(reg_map);
	int ret = -1;
	if (isWrite) {
		ret = mOniDevice.setProperty(OBC_REG16_RW, (void *)&reg_map, size);

		if (ret != 0) {
			qDebug() << " operate OBC_REG16_RW failed,ret = " << ret;
		}
	}
	else {
		ret = mOniDevice.getProperty(OBC_REG16_RW, (void *)&reg_map, &size);
		*value = reg_map.value;
		if (ret != 0) {
			qDebug() << " operate OBC_REG16_RW failed,ret = " << ret;
		}
	}
	return ret == 0;
}

bool SensorBase::setDelayTime(int value)
{
	if (!mDeviceOpened) {
		return false;
	}
	int size = sizeof(value);
	Status ret = mOniDevice.setProperty(OBC_DELAY_DELAY_TIME, (void *)&value, size);

	if (ret == STATUS_OK) {

		qDebug() << "setDelayTime success, mode: " << value;

		return true;
	}
	else {
		qDebug() << "setDelayTime failed ret=:" << ret;
		return false;
	}
}

bool SensorBase::getChipID(int *value) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}
	int get_value = 0;
	int size = sizeof(get_value);
	int ret = mOniDevice.getProperty(OBC_CHIP_ID, (void *)&get_value, &size);

	if (ret == 0) {
		*value = get_value;
		qDebug() << " getChipID success, flag:" << get_value;
		return true;
	}
	else {
		qDebug() << "getChipID failed ret=:" << ret;
	}
	return false;
}

bool SensorBase::getDelayTime(int *value) {
	std::lock_guard<std::mutex> locker(mMutexSetting);
	if (!mDeviceOpened) {
		return false;
	}
	int get_value = 0;
	int size = sizeof(get_value);
	int ret = mOniDevice.getProperty(OBC_DELAY_DELAY_TIME, (void *)&get_value, &size);

	if (ret == 0) {
		*value = get_value;
		qDebug() << " getDelayTime success, flag:" << get_value;
		return true;
	}
	else {
		qDebug() << "getDelayTime failed ret=:" << ret;
	}
	return false;
}

int SensorBase::getSensorInfo(ObcSensorInfo &sensorInfo)
{
	if (!mDeviceOpened) {
		return false;
	}
	int size = sizeof(ObcSensorInfo);
	int ret = mOniDevice.getProperty(OBC_SENSOR_INFO, (void *)&sensorInfo, &size);
	if (ret != 0) 
	{
		qDebug() << "get OBC_SENSOR_INFO failed ret=:" << ret;
	}
	return ret;
}