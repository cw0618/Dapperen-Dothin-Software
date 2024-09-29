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
#ifndef TEXTUREBUFFER_H
#define TEXTUREBUFFER_H
#include<stdint.h>
#include <mutex>
#include<qDebug>
class TextureBuffer
{
private:
	int8_t * data;
	int w = 0;
	int h = 0;
	int mPointCount = 0;
	int mPointSize = 0;
	int aiSize = 0;
	int8_t * mDepthData = nullptr;
	int8_t*  mAiData = nullptr;
	
	int mDepthWidth = 0;
	int mDepthHeight = 0;
	
	int mDepthPointSize = 0;
	bool mFrameUpdate=false;
	bool mDepthUpdate = false;
public:
    TextureBuffer();
    ~TextureBuffer();
	bool mD2cStart = false;
	std::mutex mMutexAiData;
	void setData(int8_t *buf, int width, int height, int pointSize);
	void setAIData(int8_t *buf,int size_);
	void setDepthData(int8_t *buf, int width, int height, int pointSize);
	int8_t* Data() const {
		return data;
	}
	int8_t* DepthData() const {
		return mDepthData;
	}
	int8_t* AIData() const {
		return mAiData;
	}
	int Width() const {
		return w;
	}

	int Height() const {
		return h;
	}
	int DWidth() const {
		return mDepthWidth;
	}

	int DHeight() const {
		return mDepthHeight;
	}
	int PointSize() const {
		return mPointSize;
	}
	int AISize() const {
		return aiSize;
	}

	void setFrameUpdate(bool tag) {
		mFrameUpdate = tag;
	}
	void setDepthUpdate(bool tag) {
		mDepthUpdate = tag;
	}
	void setD2CStart(bool state) {
		mD2cStart = state;
	}

	bool isFrameUpdate() {
		return mFrameUpdate;
	}
	bool isDepthUpdate() {
		return mDepthUpdate;
	}

};

#endif // TEXTUREBUFFER_H
