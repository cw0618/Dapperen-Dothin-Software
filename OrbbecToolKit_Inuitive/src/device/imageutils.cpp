#include "imageutils.h"
#include<string.h>
#include <QMutex>
#include <QDebug>

#define HISTSIZE 0xFFFF
#define POINT_MAX_DISTANCE 12000

#define NUM_COLORS  30000

/** \class ImageUtils
*
* 图像数据的处理类
*
*/
ImageUtils::ImageUtils() :
	mLegendR2B(1, NUM_COLORS, QImage::Format_ARGB32_Premultiplied),
	mLegendB2R(1, NUM_COLORS, QImage::Format_ARGB32_Premultiplied)
{
	mHistogram = NULL;
	memset(&mRgbBean, 0, sizeof(BufBean));
	mRgbBean.pointSize = 3; //rgb
	mRgbBean.valid = false;

	memset(&mCloudBean, 0, sizeof(PCloudBean));
	mCloudBean.pointSize = 6; //xyz rgb
	mCloudBean.valid = false;

	mPcMinValue = 1000;
	mPcMaxValue = 30000;

	initColorize();
}

ImageUtils::~ImageUtils()
{
	if (mHistogram != NULL) {
		delete mHistogram;
		mHistogram = NULL;
	}
}


void ImageUtils::initColorize()
{
	int numSteps = NUM_COLORS;
	unsigned char red, green, blue;
	mColorVector = QVector<QColor>();

	for (int i = 0; i < numSteps; i++) {
		createColorMapPixel(numSteps, i, red, green, blue);
		mColorVector << QColor(red, green, blue);
	}

	for (int i = 0; i < mColorVector.size(); i++) {
		mLegendB2R.setPixel(0, mColorVector.size() - i - 1, mColorVector.at(i).rgb());
		mLegendR2B.setPixel(0, i, mColorVector.at(i).rgb());
	}

	setRange(100, 60000);
}

int ImageUtils::grayRGB(uint16_t* src, int w, int h, int bitmov)
{
	int ret = 0;
	initRGBBuf(w, h);
	convertRaw16ToRGB(src, mRgbBean.data, w, h, bitmov);
	mRgbBean.valid = true;

	return ret;
}

int ImageUtils::histogramRGB(uint16_t* src, int w, int h, int rgbType)
{
	int ret = 0;
	initRGBBuf(w, h);
	//    init_PCloudBuf(w, h);
	depthFormatRgb888(src, mRgbBean.data, w, h, NULL);
	mRgbBean.valid = true;
	return ret;
}


int ImageUtils::linearRGB(uint16_t* src, int w, int h)
{
	int ret = 0;
	initRGBBuf(w, h);
	depthFormatLinearRGB(src, mRgbBean.data, w, h);
	mRgbBean.valid = true;
	return ret;
}


BufBean* ImageUtils::getRgbBean()
{
	return &mRgbBean;
}

PCloudBean* ImageUtils::getCloudBean()
{
	return &mCloudBean;
}

void ImageUtils::initRGBBuf(int w, int h) {
	int size = w * h * mRgbBean.pointSize;
	if (mRgbBean.data == NULL || mRgbBean.size != size) {
		if (mRgbBean.data != NULL) {
			delete mRgbBean.data;
			mRgbBean.data = NULL;
		}
		mRgbBean.size = size;
		mRgbBean.data = new char[size];
	}

	mRgbBean.w = w;
	mRgbBean.h = h;
	memset(mRgbBean.data, 0, mRgbBean.size);
	mRgbBean.valid = false;
}


void ImageUtils::initPCloudBuf(int w, int h)
{
	int new_size = w * h * mCloudBean.pointSize;
	int old_size = mCloudBean.w * mCloudBean.h * mCloudBean.pointSize;
	if (mCloudBean.data == NULL || old_size != new_size) {
		if (mCloudBean.data != NULL) {
			delete mCloudBean.data;
			mCloudBean.data = NULL;
		}
		mCloudBean.data = new float[new_size];
	}

	mCloudBean.w = w;
	mCloudBean.h = h;
	mCloudBean.pointCount = w * h;
	memset(mCloudBean.data, 0, new_size);
	mCloudBean.valid = false;

}

void ImageUtils::rotationRaw16(uint16_t *src, uint16_t* dst, int w, int h, int degress) {
	int x = 0;
	int y = 0;
	if (degress == 90) {
		//顺时针旋转90度
		for (y = 0; y < h; y++)
		{
			for (x = 0; x < w; x++)
			{
				dst[y + (w - x - 1)*h] = src[x + y*w];
			}
		}

	}
	else if (degress == -90) {
		//逆时针90度翻转
		for (y = 0; y < h; y++)
		{
			for (x = 0; x < w; x++)
			{
				dst[h*(x + 1) - 1 - y] = src[x + y*w];
			}
		}
	}
}

void ImageUtils::setPcMinValue(uint32_t value)
{
	mPcMinValue = value;
}

void ImageUtils::setPcMaxValue(uint32_t value)
{
	mPcMaxValue = value;
}

void ImageUtils::convertRaw16ToRGB(uint16_t* src, char* dst, int w, int h, int bitmov) {
	for (int i = 0; i < h; i++)
	{
		uint16_t* psrc = ((uint16_t*)src) + (w * i);
		RGB888* p_rgb = ((RGB888*)dst) + (w * i);
		for (int j = 0; j < w; j++) {
			uint8_t value = (psrc[j] >> bitmov) & 0xff;
			p_rgb[j].R = value;
			p_rgb[j].G = value;
			p_rgb[j].B = value;
		}
	}
}


void ImageUtils::depthFormatRgb888(uint16_t *src, void *dst, int w, int h, int rgbType) {
	// copy depth into texture-map
	QMutexLocker locker(&mutex);

	calculateHistogram(src, w, h);

	uint16_t *pDepth = src;
	float* pointCloudPtr = mCloudBean.data;
	for (int nY = 0; nY < h; nY++) {
		uint8_t *pTexture = (uint8_t *)((uint8_t*)dst + (w * nY * 3));

		for (int nX = 0; nX < w; nX++, pDepth++, pTexture += 3) {

			if (rgbType == 0) {
				uint8_t nRed = 0;

				nRed = mHistogram[*pDepth] * 255;
				pTexture[0] = nRed;
				pTexture[1] = nRed;
				pTexture[2] = nRed;

			}
			else if (rgbType == 1) {

				int index = mHistogram[*pDepth] * NUM_COLORS;
				QColor color = mColorVector.at(index);
				pTexture[0] = color.red();
				pTexture[1] = color.green();
				pTexture[2] = color.blue();
			}
			else {

				float val = 65536 - (*pDepth * 255.0 / 65536.0);
				pTexture[0] = val;
				pTexture[1] = val;
				pTexture[2] = val;
			}
		}
	}
}

void ImageUtils::calculateHistogram(uint16_t *src, int w, int h) {
	// Calculate the accumulative histogram (the yellow display...)
	//memset(m_histogram, 0, HISTSIZE*sizeof(float));

	if (mHistogram == NULL) {
		mHistogram = new float[HISTSIZE];
	}
	if (mHistogram == NULL) {
		qDebug("m_histogram  new malloc null");
		return;
	}
	memset(mHistogram, 0, HISTSIZE * sizeof(float));

	int nNumberOfPoints = 0;
	unsigned int value;
	int nIndex = 0;
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			value = src[nIndex];
			if (value != 0) {
				mHistogram[value]++;
				nNumberOfPoints++;
			}
			nIndex++;
		}
	}

	for (nIndex = 1; nIndex < HISTSIZE; nIndex++) {
		mHistogram[nIndex] += mHistogram[nIndex - 1];
		if (mHistogram[nIndex - 1] != 0) {
			mHistogram[nIndex - 1] = (nNumberOfPoints - mHistogram[nIndex - 1]) / nNumberOfPoints;
		}

		if (nIndex == (HISTSIZE - 1)) {
			if (mHistogram[nIndex] != 0) {
				mHistogram[nIndex] = (nNumberOfPoints - mHistogram[nIndex]) / nNumberOfPoints;
			}
		}
	}

	//    for (nIndex = 1; nIndex< HISTSIZE; nIndex++)
	//    {
	//        if (m_histogram[nIndex] != 0) {
	//            m_histogram[nIndex] = (nNumberOfPoints - m_histogram[nIndex]) / nNumberOfPoints;
	//        }
	//    }
}


void ImageUtils::depthFormatLinearRGB(uint16_t* src, char* dst, int w, int h)
{

	uint16_t *pDepth = src;
	for (int nY = 0; nY < h; nY++) {
		uint8_t *pTexture = (uint8_t *)((uint8_t*)dst + (w * nY * 3));
		for (int nX = 0; nX < w; nX++, pDepth++, pTexture += 3) {

			int value = *pDepth;

			value -= mBegin;
			if (value < 0 || value > mEnd) {
				pTexture[0] = 0;
				pTexture[1] = 0;
				pTexture[2] = 0;
				continue;

			}

			int index = mColorVector.size() - (value * mIndexFactorColor);

			if (index <= 0)
				index = 0;
			else if (index >= mColorVector.size())
				index = mColorVector.size() - 1;

			QColor color = mColorVector.at(index);
			pTexture[0] = color.red();
			pTexture[1] = color.green();
			pTexture[2] = color.blue();

		}
	}
}


void ImageUtils::setRange(int start, int stop) {
	mBegin = start;
	mEnd = stop;
	int diff = stop - start;
	if (diff <= 0) {
		diff = 1;
	}
	mIndexFactorColor = NUM_COLORS / (double)(diff);
}

double ImageUtils::interpolate(double x, double x0, double y0, double x1, double y1) {

	if (x1 == x0) {
		return y0;
	}
	else {
		return ((x - x0)*(y1 - y0) / (x1 - x0) + y0);
	}
}


void ImageUtils::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue) {

	double k = 1;
	double B0 = -0.125 * k - 0.25;
	double B1 = B0 + 0.25 * k;
	double B2 = B1 + 0.25 * k;
	double B3 = B2 + 0.25 * k;

	double G0 = B1;
	double G1 = G0 + 0.25 * k;
	double G2 = G1 + 0.25 * k;
	double G3 = G2 + 0.25 * k + 0.125;

	double R0 = B2;
	double R1 = R0 + 0.25 * k;
	double R2 = R1 + 0.25 * k;
	double R3 = R2 + 0.25 * k + 0.25;

	double i = (double)indx / (double)numSteps - 0.25 * k;

	if (i >= R0 && i < R1) {
		red = interpolate(i, R0, 0, R1, 255);
	}
	else if ((i >= R1) && (i < R2)) {
		red = 255;
	}
	else if ((i >= R2) && (i < R3)) {
		red = interpolate(i, R2, 255, R3, 0);
	}
	else {
		red = 0;
	}

	if (i >= G0 && i < G1) {
		green = interpolate(i, G0, 0, G1, 255);
	}
	else if ((i >= G1) && (i < G2)) {
		green = 255;
	}
	else if ((i >= G2) && (i < G3)) {
		green = interpolate(i, G2, 255, G3, 0);
	}
	else {
		green = 0;
	}


	if (i >= B0 && i < B1) {
		blue = interpolate(i, B0, 0, B1, 255);
	}
	else if ((i >= B1) && (i < B2)) {
		blue = 255;
	}
	else if ((i >= B2) && (i < B3)) {
		blue = interpolate(i, B2, 255, B3, 0);
	}
	else {
		blue = 0;
	}

}




