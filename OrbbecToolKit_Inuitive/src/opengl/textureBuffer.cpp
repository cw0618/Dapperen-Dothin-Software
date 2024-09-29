#include "texturebuffer.h"
#include <string.h>

TextureBuffer::TextureBuffer()
{
	data = NULL;
	w = 0;
	h = 0;
	mPointSize = 0;
	mFrameUpdate = false;
	mDepthUpdate = false;
	mPointCount = 0;
}

TextureBuffer::~TextureBuffer() {
	if (data != NULL) {
		delete[] data;
		data = NULL;
	}
}

void TextureBuffer::setAIData(int8_t *buf, int size_) {
	if (buf == NULL) return;
	if (size_ <= 0) return;
	if (aiSize != size_) {


		if (mAiData != NULL) {
			delete[] mAiData;
			mAiData = NULL;
		}
		aiSize = size_;
		mAiData = new int8_t[aiSize];
	}
	memcpy(mAiData, buf, aiSize);
}
void TextureBuffer::setData(int8_t *buf, int width, int height, int _pointSize) {
	if (buf == NULL) return;
	if (_pointSize <= 0) return;

	//int count = width * height;
	//if(count <= 0) return;

	w = width;
	h = height;

	if (data == NULL) {
		mPointSize = _pointSize;
		data = new int8_t[_pointSize];
	}
	mPointSize = _pointSize;
	memcpy(data, buf, _pointSize);
	mFrameUpdate = true;

}
void TextureBuffer::setDepthData(int8_t *buf, int width, int height, int _pointSize) {
	if (buf == NULL) return;
	if (_pointSize <= 0) return;
	mDepthWidth = width;
	mDepthHeight = height;
	if (mDepthData == NULL) {
		mDepthPointSize = _pointSize;
		mDepthData = new int8_t[mDepthPointSize];
	}
	memcpy(mDepthData, buf, mDepthPointSize);
	mDepthHeight = height;
	mDepthUpdate = true;
}
