#include "OniStream.h"
#include <algorithm>

#include "src/calculate/calc.h"


Calc *calc_g = Calc::Instance();

void ObPtr::ObDelete888bit()
{
	if (mSizeOf888bit != 0 && nullptr != mData888bitPtr)
	{
		delete[] mData888bitPtr;
		mData888bitPtr = nullptr;

		mSizeOf888bit = 0;
	}
}
void ObPtr::ObDeleteDepth888bit()
{
	if (mSizeOf888bit != 0 && nullptr != mDataDepth888bitPtr)
	{
		delete[] mDataDepth888bitPtr;
		mDataDepth888bitPtr = nullptr;

		mSizeOf888bit = 0;
	}
}
void ObPtr::ObDeletePointCouldData()
{
	if (mSizePointCouldData != 0 && nullptr != mDataPointCouldPtr)
	{
		delete mDataPointCouldPtr;
		mDataPointCouldPtr = nullptr;

		mSizePointCouldData = 0;
	}
}

void ObPtr::ObDeleteHistogram888bit()
{
	if (mSizeOfHistogram888Bit != 0 && nullptr != mDataHistogram888Ptr)
	{
		delete[] mDataHistogram888Ptr;
		mDataHistogram888Ptr = nullptr;

		mSizeOfHistogram888Bit = 0;
	}
}

void ObPtr::ObDeleteRainbow888bit()
{
	if (mSizeOfRainbow888bit != 0 && nullptr != mDataRainbow888Ptr)
	{
		delete mDataRainbow888Ptr;
		mDataRainbow888Ptr = nullptr;

		mSizeOfRainbow888bit = 0;
	}
}
void ObPtr::ObDeleteD2C888bit()
{
	if (mSizeOfD2c888bit != 0 && nullptr != mDataRainbow888Ptr)
	{
		delete[] mDataD2c888Ptr;
		mDataD2c888Ptr = nullptr;

		mSizeOfD2c888bit = 0;
	}
}
void ObPtr::ObDelete16bit()
{
	if (nullptr != mDataPtr)
	{
		delete[] mDataPtr;
		mDataPtr = nullptr;

		mSizeOfData = 0;
	}
}

void ObPtr::ObDelete()
{
	ObDelete16bit();
	ObDelete888bit();
	ObDeleteHistogram888bit();
	ObDeleteD2C888bit();
};

ObPtr::ObPtr()
{
	mSizeOfData = 0;

	mDataPtr = nullptr;
	mData888bitPtr = nullptr;
	mDataHistogram888Ptr = nullptr;
	mDataRainbow888Ptr = nullptr;
	mDataD2c888Ptr = nullptr;

	mSizeOfData = 0;
	mSizeOf888bit = 0;
	mSizeOfHistogram888Bit = 0;
	mSizeOfRainbow888bit = 0;
	mSizeOfD2c888bit = 0;
	initGrayColor();
	mPointCloudFilterParams[2] = 1;
};
void ObPtr::initGrayColor() {
	for (int i = 0; i < 256; i++) {
		gray_color_value[i] = i;
	}
}

ObPtr::ObPtr(unsigned int _num)
{
	mSizeOfData = 0;
	mDataPtr = new int8_t[_num * sizeof(int8_t)]();
	memset(mDataPtr, 0x00, _num * sizeof(int8_t));

	if (nullptr == mDataPtr) {
		mSizeOfData = 0;
	}
	else {
		mSizeOfData = _num * sizeof(int8_t);
	}
};
void ObPtr::UpdataPhaseDate(const int8_t *_src, unsigned int _size) {
	//_size =  _size*sizeof(uint16_t);
	if (_size != mSizeOfData)
	{
		ObDelete16bit();
		mDataPtr = new int8_t[_size]();
		memset(mDataPtr, 0, _size);
		mSizeOfData = _size;
	}

	if (nullptr == _src)
	{
		memset(mDataPtr, 0, _size);
		mSizeOfData = _size;
	}
	else
	{
		::memcpy((int8_t*)mDataPtr, _src, _size);
	}
	int16_t *phase_data = (int16_t*)mDataPtr;
	if (nullptr != mDataPtr && mSizeOfData != _size) {
		mSizeOfData = _size;
	}
}

void ObPtr::UpdataDate(const int8_t *_src, unsigned int _size)
{
	if (_size != mSizeOfData)
	{
		ObDelete16bit();
		mDataPtr = new int8_t[_size]();
	}

	if (nullptr == _src)
	{
		memset(mDataPtr, 0, _size);
		mSizeOfData = _size;
	}
	else
	{
		::memcpy((int8_t*)mDataPtr, _src, _size);
	}

	if (nullptr != mDataPtr && mSizeOfData != _size) {
		mSizeOfData = _size;
	}
}

uint32_t ObPtr::SizeOfData() const
{
	return mSizeOfData;
};

uint32_t ObPtr::SizeOf888bit() const
{
	return mSizeOf888bit;
};

uint32_t ObPtr::SizeOfHist888bit() const
{
	return mSizeOfHistogram888Bit;
};

uint32_t ObPtr::SizeOfRainbow888bit() const
{
	return mSizeOfRainbow888bit;
};
uint32_t ObPtr::SizeOfD2C888bit() const
{
	return mSizeOfD2c888bit;
}
uint32_t ObPtr::SizeOfPointCloud() const
{
	return mSizePointCouldData;
}
void ObPtr::SetTofCameraParams(float * in, float * dist)
{
	for (int i = 0; i < 4; i++) {
		mIntrin[i] = in[i];
	}
	for (int i = 0; i < 5; i++) {
		mDistortCoeff[i] = dist[i];
	}

}
;
bool ObPtr::UpdataDate888bit()
{
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
	{
		ObDelete888bit();
		mData888bitPtr = new int8_t[size_tmp]();
	}

	if (mSizeOfData / sizeof(int16_t) > size_tmp)
		return false;

	mSizeOf888bit = size_tmp;


	int16_t *temp_16bit = reinterpret_cast<int16_t*>(mDataPtr);
	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mData888bitPtr);
	for (int32_t i = 0; i < (mSizeOfData / sizeof(int16_t)); ++i)
	{
		int32_t depth_value = temp_16bit[i];
		float val = 0;
		if (depth_value > 0) {

			val = 255 - (depth_value * 255.0 / mRangeOfDepth);
		}
		tmp_888bit[i].r = val;
		tmp_888bit[i].g = val;
		tmp_888bit[i].b = val;


	}
	return true;
}

bool ObPtr::UpdataDatePhase() {
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
	{
		ObDelete888bit();
		mData888bitPtr = new int8_t[size_tmp]();
	}

	if (mSizeOfData / sizeof(int16_t) > size_tmp)
		return false;

	mSizeOf888bit = size_tmp;


	int16_t *temp_16bit = reinterpret_cast<int16_t*>(mDataPtr);

	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mData888bitPtr);
	for (uint32_t i = 0; i < (mSizeOfData / sizeof(int16_t)); ++i)
	{
		int16_t depth_value = temp_16bit[i];
		uint8_t rgb_value = abs(static_cast<uint8_t>((depth_value >> 2) & 0xff));//修改最大亮度，12位右移4位，10位右移2位
		tmp_888bit[i].r = rgb_value;
		tmp_888bit[i].g = rgb_value;
		tmp_888bit[i].b = rgb_value;
	}

	return true;
}

bool ObPtr::UpdataDatePhaseAB() {
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
	{
		ObDelete888bit();
		mData888bitPtr = new int8_t[size_tmp]();
	}

	if (mSizeOfData / sizeof(int16_t) > size_tmp)
		return false;

	mSizeOf888bit = size_tmp;


	int16_t *temp_16bit = reinterpret_cast<int16_t*>(mDataPtr);
	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mData888bitPtr);
	for (uint32_t i = 0; i < (mSizeOfData / sizeof(int16_t)); ++i)
	{
		int16_t depth_value = temp_16bit[i];
		uint8_t rgb_value = 0;
		if (depth_value >0)
		{
			rgb_value = abs(static_cast<uint8_t>((depth_value >> 2) & 0xff));
		}
		else
		{
			if (i > 1)
			{
				depth_value = temp_16bit[i - 1];
				
				rgb_value = abs(static_cast<uint8_t>((depth_value >> 2) & 0xff));
			}
			else
			{
				rgb_value = 0;
			}
			if (rgb_value > 200)
			{
				rgb_value = 0;
			}
			
		}
		//太暗了，加大40显示
		if (rgb_value < 250)
		{
			rgb_value += 40;
		}

		tmp_888bit[i].r = rgb_value;
		tmp_888bit[i].g = rgb_value;
		tmp_888bit[i].b = rgb_value;


	}
	//std::ofstream mbfs("./config/plan_data_640x480.raw", std::ios::binary);
	//mbfs.write((char *)tmp_888bit[0].r, 640 * 480);
	//mbfs.close();
	return true;
}

bool ObPtr::UpdataDateDepthIR() {
	//智能家居深度显示
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData;

	if (size_tmp != mSizeOf888bit || 0 == mDataDepth888bitPtr)
	{
		ObDelete888bit();
		mDataDepth888bitPtr = new int8_t[size_tmp]();
	}
	mSizeOf888bit = size_tmp;

	C888RGB *temp_16bit = reinterpret_cast<C888RGB*>(mDataPtr);
	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mDataDepth888bitPtr);
	for (uint32_t i = 0; i < (mSizeOfData / sizeof(C888RGB)); ++i)
	{

		tmp_888bit[i].r = temp_16bit[i].b;
		tmp_888bit[i].g = temp_16bit[i].g;
		tmp_888bit[i].b = temp_16bit[i].r;
	}

	return true;
}
float *m_histogram = nullptr;
bool ObPtr::UpdataDateIR(int w, int h) {
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
	{
		ObDelete888bit();
		mData888bitPtr = new int8_t[size_tmp]();
	}

	if (mSizeOfData / sizeof(int16_t) > size_tmp)
		return false;

	mSizeOf888bit = size_tmp;

	int16_t *temp_16bit = reinterpret_cast<int16_t*>(mDataPtr);
	/*直方图的形式显示IR*/
	calculateHistogram(temp_16bit, w, h);
	int8_t *tmp_8bit = reinterpret_cast<int8_t*>(mData888bitPtr);

	calc_g->getHistogram(temp_16bit, tmp_8bit, w, h, false);

#if 0
	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mData888bitPtr);
	/*
	  ImageJ的形式显示IR
		uint16_t maxIR = 0;

		for (uint32_t i = 0; i < (mSizeOfData / sizeof(uint16_t)); ++i) {
			if (temp_16bit[i] > maxIR) {
				maxIR = temp_16bit[i];
			}
		}
		*/

	for (uint32_t i = 0; i < (mSizeOfData / sizeof(uint16_t)); ++i)
	{
		uint8_t rgb_value = 0;
		uint16_t depth_value = temp_16bit[i];
		/*
		ImageJ的形式显示IR
		if (maxIR <= 255) {
			rgb_value = depth_value * 255 / 200;
		}
		else {
			rgb_value = static_cast<uint8_t>((depth_value >> 3) & 0xff);
		}
*/
/* 直方图的形式显示IR*/
		rgb_value = m_histogram[depth_value] * 255;


#if 0
		//东海pleco给客户演示工具，修改ir渲染，解决近处过曝，远处看不到的问题，发布版本v1.0.2.0
		if (depth_value > 0 && depth_value < 500)
		{
			rgb_value = depth_value * 255 / 500;
		}
		else if (depth_value > 500 && depth_value < 2000) {
			rgb_value = (depth_value >> 3) & 0xff;
		}
		else {
			rgb_value = 255;
		}
#endif
		//	rgb_value = static_cast<uint8_t>((depth_value >> 4) & 0xff);
		tmp_888bit[i].r = rgb_value;
		tmp_888bit[i].g = rgb_value;
		tmp_888bit[i].b = rgb_value;


		//	UpdataDateHistogram(w, h, false);
	}
#endif
	return true;
}
bool ObPtr::UpdataDateOtherIR(int w, int h, int min, int max)
{
	if (nullptr == mDataPtr) {
		return false;
	}
	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOf888bit || nullptr == mData888bitPtr)
	{
		ObDelete888bit();
		mData888bitPtr = new int8_t[size_tmp]();
	}

	if (mSizeOfData / sizeof(uint16_t) > size_tmp)
		return false;

	mSizeOf888bit = size_tmp;

	int16_t *temp_16bit = reinterpret_cast<int16_t*>(mDataPtr);
	C888RGB* tmp_888bit = reinterpret_cast<C888RGB*>(mData888bitPtr);
	for (uint32_t i = 0; i < (mSizeOfData / sizeof(int16_t)); ++i)
	{
		int16_t depth_value = temp_16bit[i];
		uint8_t rgb_value = abs(static_cast<uint8_t>((depth_value >> 4) & 0xff));
		tmp_888bit[i].r = rgb_value;
		tmp_888bit[i].g = rgb_value;
		tmp_888bit[i].b = rgb_value;
	}

	return true;
}
void ObPtr::calculateHistogram(int16_t *src, int w, int h) {
	// Calculate the accumulative histogram (the yellow display...)
	//memset(m_histogram, 0, HISTSIZE*sizeof(float));

	if (m_histogram == NULL) {
		m_histogram = new float[HISTSIZE];
	}
	if (m_histogram == NULL) {
		return;
	}
	memset(m_histogram, 0, HISTSIZE * sizeof(float));

	int nNumberOfPoints = 0;
	unsigned int value;
	int index = 0;
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			value = src[index];
			if (value != 0) {
				m_histogram[value]++;
				nNumberOfPoints++;
			}
			index++;
		}
	}

	int nIndex;
	for (nIndex = 1; nIndex < HISTSIZE; nIndex++) {
		m_histogram[nIndex] += m_histogram[nIndex - 1];
	}

	for (nIndex = 1; nIndex < HISTSIZE; nIndex++)
	{
		if (m_histogram[nIndex] != 0)
		{
			m_histogram[nIndex] = (nNumberOfPoints - m_histogram[nIndex]) / nNumberOfPoints;
		}
	}

}


void ObPtr::convertRaw16ToRGB(int16_t* src, char* dst, int w, int h, int bitmov) {
	for (int i = 0; i < h; i++)
	{
		int16_t* psrc = ((int16_t*)src) + (w * i);
		C888RGB* p_rgb = ((C888RGB*)dst) + (w * i);
		for (int j = 0; j < w; j++) {
			uint8_t value = (psrc[j] >> bitmov) & 0xff;
			p_rgb[j].r = value;
			p_rgb[j].g = value;
			p_rgb[j].b = value;
		}
	}

}
bool ObPtr::UpdateDataImageJ()
{
	if (nullptr == mDataPtr) {
		return false;
	}

	uint16_t max_depth = *std::max_element((uint16_t*)(mDataPtr), ((uint16_t*)mDataPtr + (mSizeOfData / sizeof(uint16_t))));
	if (max_depth > 10)
	{
		uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

		if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
		{
			ObDelete888bit();
			mData888bitPtr = new int8_t[size_tmp]();
		}

		mSizeOf888bit = size_tmp;

		double f_scale = 255.0 / max_depth;
		uint16_t *temp_16bit = (uint16_t*)mDataPtr;
		for (uint32_t i = 0; i < (mSizeOfData / sizeof(uint16_t)); ++i)
		{
			uint32_t depth_value = temp_16bit[i];

			char rgb_value = static_cast<char>(depth_value * f_scale);
			mData888bitPtr[3 * i] = mData888bitPtr[3 * i + 1] = mData888bitPtr[3 * i + 2] = rgb_value;
		}
	}
	else
	{
		uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

		if (size_tmp != mSizeOf888bit || 0 == mData888bitPtr)
		{
			ObDelete888bit();
			mData888bitPtr = new int8_t[size_tmp]();
		}
	}

	return true;
}

bool ObPtr::UpdataDateHistogram(int width, int height, bool y_flip)
{
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != mSizeOfHistogram888Bit)
	{
		ObDeleteHistogram888bit();
		mSizeOf888bit = size_tmp;

		mDataHistogram888Ptr = new int8_t[mSizeOf888bit]();
	}

	if (nullptr == mDataHistogram888Ptr)
	{
		mSizeOfHistogram888Bit = 0;
		return false;
	}

	calc_g->getHistogram(reinterpret_cast<const int16_t*>(mDataPtr), mDataHistogram888Ptr, width, height, y_flip);

	if (size_tmp != mSizeOfHistogram888Bit)
		mSizeOfHistogram888Bit = size_tmp;
	return true;
}

bool ObPtr::UpdateDatePointCould(int width, int height, float * bufferData)
{
	if (nullptr == bufferData) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 6;
	int iPointSize = width *height;

	if (nullptr == mDataPointCouldPtr)
	{
		ObDeletePointCouldData();
		mDataPointCouldPtr = new float[size_tmp]();
	}
	mSizePointCouldData = size_tmp;
	memset(mDataPointCouldPtr,0, size_tmp);
	float* pointCloudPtr = mDataPointCouldPtr;
	float* originalPointCloudPtr = bufferData;
	for (int i = 0; i < iPointSize; i++)
	{
		pointCloudPtr[0] = originalPointCloudPtr[0];
		pointCloudPtr[1] = originalPointCloudPtr[1];
		pointCloudPtr[2] = originalPointCloudPtr[2];
		pointCloudPtr[3] = 1.0f;
		pointCloudPtr[4] = 1.0f;
		pointCloudPtr[5] = 0.0f;
		pointCloudPtr += 6;
		originalPointCloudPtr += 3;
	}

	return true;
}

bool ObPtr::UpdateDatePointCould(int width, int height)
{
	if (nullptr == mDataPtr) {
		return false;
	}

	uint32_t size_tmp = mSizeOfData / 2 * 6;

	if (nullptr == mDataPointCouldPtr)
	{
		ObDeletePointCouldData();
		mDataPointCouldPtr = new float[size_tmp]();
	}

	if (mSizeOfData / sizeof(uint16_t) > size_tmp)
		return false;

	mSizePointCouldData = size_tmp;

	uint16_t *pDepth = (uint16_t*)mDataPtr;
	float* pointCloudPtr = mDataPointCouldPtr;


	for (int nY = 0; nY < height; nY++) {
		for (int nX = 0; nX < width; nX++, pDepth++) {
			Point_3D world;
			convertProjectiveToWorld(nX, nY, *pDepth, world, false);
			if (world.z > 60 && world.z <= 60000) {
				pointCloudPtr[0] = world.u;
				pointCloudPtr[1] = world.v;
				pointCloudPtr[2] = world.z;
				pointCloudPtr[3] = 1.0f;
				pointCloudPtr[4] = 1.0f;
				pointCloudPtr[5] = 0.0f;
			}
			else {
				pointCloudPtr[0] = 0;
				pointCloudPtr[1] = 0;
				pointCloudPtr[2] = 0;
				pointCloudPtr[3] = 1.0f;
				pointCloudPtr[4] = 0;
				pointCloudPtr[5] = 0;
			}

			pointCloudPtr += 6;
		}
	}

	return false;
}
void ObPtr::convertProjectiveToWorld(int u, int v, int z, Point_3D& world, bool use_distort_coef)
{
	float fx = mIntrin[0];
	float fy = mIntrin[1];
	float cx = mIntrin[2];
	float cy = mIntrin[3];

	float k1 = mDistortCoeff[0];
	float k2 = mDistortCoeff[1];
	float p1 = mDistortCoeff[2];
	float p2 = mDistortCoeff[3];
	float k3 = mDistortCoeff[4];

	float ifx, ify;
	ifx = 1. / (fx);
	ify = 1. / (fy);

	float tx = (u - cx) * ifx;
	float ty = (v - cy) * ify;

	float x0 = tx;
	float y0 = ty;

	if (use_distort_coef)
	{
		for (int j = 0; j < 5; j++)
		{
			double r2 = tx * tx + ty * ty;
			double icdist = (1) / (1 + ((k3 * r2 + k2)*r2 + k1)*r2);
			double deltaX = 2 * p1 * tx*ty + p2* (r2 + 2 * tx*tx);
			double deltaY = p1 * (r2 + 2 * ty*ty) + 2 * p2 * tx*ty;
			tx = (x0 - deltaX)*icdist;
			ty = (y0 - deltaY)*icdist;
		}
	}

	world.u = z * tx;
	world.v = z * ty;
	world.z = z;
}

void ObPtr::addPointFilterToDepthData()
{

	if (nullptr == mDataPtr) {
		return;
	}
}
// class OniStream_base ****************************************
void OniStreamBase::Rst()
{
	DestroyStream();
	mOniDataInfo.Rst();
}

OniStreamBase::OniStreamBase()
{
	mOniDataInfo.Rst();
}

OniStreamBase::~OniStreamBase()
{
	mOniDataInfo.Rst();
}

int OniStreamBase::StopStream()
{
	int ret_status = 0;
	if (mVideoStream.isValid()) {
		mVideoStream.stop();
		mVideoFrameRef.release();
	}
	else {
		ret_status = -1;
	}
	return ret_status;
}
int OniStreamBase::StartStream()
{
	int ret_status = 0;
	if (mVideoStream.isValid()) {
		openni::Status status = mVideoStream.start();
		ret_status = status;
	}
	else
	{
		ret_status = -1;
	}
	return ret_status;
}

void OniStreamBase::DestroyStream()
{
	if (mVideoStream.isValid())
	{
		mVideoStream.stop();
		mVideoStream.destroy();
	}
}

void OniStreamBase::CleanData()
{
	mOniDataInfo.oData.ObDelete();
}

int OniStreamBase::UpdateDepthInfo(int frameIndex, openni::VideoFrameRef * videoFrameRef)
{
	mOniDataInfo.mWidth = 640;
	mOniDataInfo.mHeight = 480;
	if (videoFrameRef == nullptr)
	{
		return -1;
	}
	openni::VideoFrameRef * depthVideoFrameRef = videoFrameRef;
	mOniDataInfo.mDataSizeBytes = mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t);
	mOniDataInfo.mPixelInBytes = 2;

	mOniDataInfo.oData.UpdataDate(nullptr, mOniDataInfo.mDataSizeBytes);
	mOniDataInfo.mFrameIndex = frameIndex;
	return 0;
}

int OniStreamBase::UpdateIRInfo(Metadata metaData, openni::VideoFrameRef * videoFrameRef,int outMode)
{
	mOniDataInfo.mWidth = 640;
	mOniDataInfo.mHeight = 480;
	int phaseCount = 0;
	if (videoFrameRef ==  nullptr)
	{
		return -1;
	}

	if (outMode == 4)
	{
		//A&B模式，输出IR图
		openni::VideoFrameRef * phaseVideoFrameRef = videoFrameRef;
		mOniDataInfo.mDataSizeBytes = mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t);
		mOniDataInfo.mPixelInBytes = 2;
		OniFrame* oniFrame = videoFrameRef->_getFrame();
		if (oniFrame != nullptr)
		{
			phaseCount = oniFrame->extraLine;
		}
		mOniDataInfo.oData.UpdataDate(nullptr, mOniDataInfo.mDataSizeBytes);
		mOniDataInfo.mFrameTimeStamp = phaseVideoFrameRef->getTimestamp();
		uint8_t* phaseGroup = (uint8_t*)phaseVideoFrameRef->getData();
		uint16_t *pIRframe = (uint16_t*)mOniDataInfo.oData.mDataPtr;
		for (uint32_t index = 0; index < phaseCount; index++) {
			uint16_t *frame = (uint16_t*)(phaseGroup + index * 1280 * 480 * 2);

			for (int i = 0; i < mOniDataInfo.mHeight; i++) {
				for (int j = 0; j < mOniDataInfo.mWidth; j++) {
					pIRframe[i * mOniDataInfo.mWidth + j] += (frame[i * 2 * mOniDataInfo.mWidth + j * 2]
						+ frame[i * 2 * mOniDataInfo.mWidth + j * 2 + 1]);
				}
			}
		}

		for (int i = 0; i < mOniDataInfo.mHeight; i++) {
			for (int j = 0; j < mOniDataInfo.mWidth; j++) {
				pIRframe[i * mOniDataInfo.mWidth + j] = (uint16_t)((pIRframe[i * mOniDataInfo.mWidth + j] / (phaseCount * 2)));
			}
		}
	}
	else if (outMode == 0)
	{
		//A-B模式，输出幅值图
		OniFrame* oniFrame = videoFrameRef->_getFrame();
		if (oniFrame != nullptr)
		{
			phaseCount = oniFrame->extraLine;
		}
		openni::VideoFrameRef * phaseVideoFrameRef = videoFrameRef;
		mOniDataInfo.mDataSizeBytes = mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t);
		mOniDataInfo.mPixelInBytes = 2;

		mOniDataInfo.oData.UpdataDate(nullptr, mOniDataInfo.mDataSizeBytes);
		mOniDataInfo.mFrameIndex = metaData.getGroupIndex();
		mOniDataInfo.mFrameTimeStamp = phaseVideoFrameRef->getTimestamp();
		int8_t* phaseGroup = (int8_t*)phaseVideoFrameRef->getData();
		int16_t *pIRframe = (int16_t*)mOniDataInfo.oData.mDataPtr;
		int piexlSize = metaData.width * metaData.height;
		if (phaseCount == 4)
		{
			int16_t *frameOne = (int16_t*)(phaseGroup);
			int16_t *frameTwo = (int16_t*)(phaseGroup + 1 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameThree = (int16_t*)(phaseGroup + 2 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameFour = (int16_t*)(phaseGroup + 3 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			
			int midDataOne = 0;
			int midDataTwo = 0;
			for (int i = 0; i < piexlSize; i++)
			{
				midDataOne = (int)(frameOne[i]) - (int)(frameThree[i]);
				midDataTwo = (int)(frameTwo[i]) - (int)(frameFour[i]);
				pIRframe[i] = (int)sqrt(pow(midDataOne, 2) + pow(midDataTwo, 2));
				pIRframe[i] = pIRframe[i] / 2;
			}
		}
		else if (phaseCount == 8)
		{
			int16_t *frameOne = (int16_t*)(phaseGroup);
			int16_t *frameTwo = (int16_t*)(phaseGroup + 1 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameThree = (int16_t*)(phaseGroup + 2 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameFour = (int16_t*)(phaseGroup + 3 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameFive = (int16_t*)(phaseGroup + 4 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameSix = (int16_t*)(phaseGroup + 5 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameSeven = (int16_t*)(phaseGroup + 6 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			int16_t *frameEight = (int16_t*)(phaseGroup + 7 * mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t));
			
			int midDataOne = 0;
			int midDataTwo = 0;
			int midDataThree = 0;
			int midDataFour = 0;
			for (int i = 0; i < piexlSize; i++)
			{
				midDataOne = (int)(frameOne[i]) - (int)(frameThree[i]);
				midDataTwo = (int)(frameTwo[i]) - (int)(frameFour[i]);
				midDataThree = (int)(frameFive[i]) - (int)(frameSeven[i]);
				midDataFour = (int)(frameSix[i]) - (int)(frameEight[i]);
				pIRframe[i] = (int)sqrt(pow(midDataOne, 2) + pow(midDataTwo, 2)) / 4 + (int)sqrt(pow(midDataThree, 2) + pow(midDataFour, 2)) / 4;
			}
		}
	}

	return 0;
}
int OniStreamBase::UpdateInfo()
{
	int cur_frame_index = mVideoFrameRef.getFrameIndex();
	if (cur_frame_index <= mOniDataInfo.mFrameIndex)
	{
		mOniDataInfo.mFrameIndex = cur_frame_index;
		return -1;
	}

	mOniDataInfo.mWidth = mVideoFrameRef.getWidth();
	mOniDataInfo.mHeight = mVideoFrameRef.getHeight();
	openni::PixelFormat format = mVideoFrameRef.getVideoMode().getPixelFormat();
	mOniDataInfo.mDataSizeBytes = mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(int16_t);
	mOniDataInfo.mPixelInBytes = mVideoFrameRef.getStrideInBytes() / mVideoFrameRef.getWidth();

	mOniDataInfo.oData.UpdataDate(reinterpret_cast<const int8_t*>(mVideoFrameRef.getData()), mOniDataInfo.mDataSizeBytes);
	mOniDataInfo.mFrameIndex = cur_frame_index;
	mOniDataInfo.mFrameTimeStamp = mVideoFrameRef.getTimestamp();

	return 0;
}

int OniStreamBase::UpdatePhaseInfo(TOFFrame phase) {

	mOniDataInfo.mWidth = phase.getWidth();
	mOniDataInfo.mHeight = phase.getHeight();
	mOniDataInfo.mDataSizeBytes = phase.getPlaneSize();
	mOniDataInfo.mPixelInBytes = phase.getStride() / phase.getWidth();
	openni::Metadata metadata = phase.getMetadata(0);
	mOniDataInfo.copyTofExtInfo(&metadata, &mOniDataInfo.mTofExteraline);
	mOniDataInfo.oData.UpdataPhaseDate(reinterpret_cast<const int8_t*>(phase.getPlane(0)), mOniDataInfo.mDataSizeBytes);
	return 0;
}
int OniStreamBase::UpdatePhaseInfo(Metadata metadata) {
	mOniDataInfo.mWidth = metadata.getWidth();
	mOniDataInfo.mHeight = metadata.getHeight();
	//phase_data = static_cast<const uint16_t*>(mVideoFrameRef.getData())/*+ frame_ref_.getWidth()*/;
	mOniDataInfo.mDataSizeBytes = mOniDataInfo.mWidth * mOniDataInfo.mHeight * sizeof(uint16_t);
	mOniDataInfo.mPixelInBytes = 2;
	mOniDataInfo.oData.UpdataPhaseDate(reinterpret_cast<const int8_t*>(mVideoFrameRef.getData()), mOniDataInfo.mDataSizeBytes);
	mOniDataInfo.mFrameTimeStamp = mVideoFrameRef.getTimestamp();
	return 0;
}
int OniStreamBase::GetVideoFrameData(OniData &oniData, int index) {


	int8_t* frameDataAddr = (int8_t*)mVideoFrameRef.getData() + mOniDataInfo.mDataSizeBytes * index;
	oniData.oData.UpdataDate(frameDataAddr, mOniDataInfo.mDataSizeBytes);
	return 0;
}