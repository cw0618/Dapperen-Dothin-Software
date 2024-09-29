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
#ifndef IMAGEUTILS_H
#define IMAGEUTILS_H

#include <QColor>
#include <QImage>
#include <QMutex>
#include<stdint.h>
#include "common.h"

#define POINT_MOVE_TO_CENTER 4200

class ImageUtils
{
private:

	QVector<QColor> mColorVector;
	QImage mLegendR2B;
	QImage mLegendB2R;
	int mBegin;
	int mEnd;
	double mIndexFactorColor;

	BufBean mRgbBean;
	PCloudBean mCloudBean;
	float* mHistogram = NULL;
	QMutex mutex;

	uint32_t mPcMinValue;
	uint32_t mPcMaxValue;

public:
	ImageUtils();
	~ImageUtils();

	//鐏板害鍥?
	int grayRGB(uint16_t* src, int w, int h, int bitmov = 0);
	//histogram 鐩存柟鍥?
	int histogramRGB(uint16_t* src, int w, int h, int rgbType);

	int linearRGB(uint16_t* src, int w, int h);

	void rotationRaw16(uint16_t *src, uint16_t* dst, int w, int h, int degress);

	void setPcMinValue(uint32_t value);
	void setPcMaxValue(uint32_t value);

	BufBean* getRgbBean();

	PCloudBean* getCloudBean();

private:
	void initColorize();
	void initRGBBuf(int w, int h);
	void initPCloudBuf(int w, int h);
	void convertRaw16ToRGB(uint16_t* src, char* dst, int w, int h, int bitmov = 0);
	void depthFormatRgb888(uint16_t *src, void *dst, int w, int h, int rgbType);
	void depthFormatLinearRGB(uint16_t* src, char* dst, int w, int h);

	void setRange(int start, int stop);
	double interpolate(double x, double x0, double y0, double x1, double y1);
	void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	void calculateHistogram(uint16_t *src, int w, int h);
};

#endif // IMAGEUTILS_H
