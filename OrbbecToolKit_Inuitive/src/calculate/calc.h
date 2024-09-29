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

#ifndef CALC_H
#define CALC_H

#include "OniCTypes.h"
#include <memory>
#include <QThread>
#include <QMatrix4x4>
#include <QVector>
#include <QString>
#include <fstream>
#include <QMutex>
#include "src/device/OniStream.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


using namespace std;

/**
* @file calc.h
* @brief 计算类Calc头文件
* @details 用于计算（生成画板、深度图添加颜色、对齐颜色）
* @mainpage 工程概览
* @author zlb
* @email 邮箱
* @version 版本号
* @date 2019-6-12 14:37:56
* @license 版权
*/

struct TextInfo
{
	int x;
	int y;

	int fontSize;
	Qt::GlobalColor penColor;

	QString text;

	TextInfo& operator=(TextInfo& R)
	{
		x = R.x;
		y = R.y;
		fontSize = R.fontSize;
		penColor = R.penColor;

		text = R.text;

		return *this;
	}

	TextInfo()
	{
		text = "";
		x = y = 0;
		fontSize = 18;
		penColor = Qt::GlobalColor::red;
	}
};


enum DepthDrawModel
{
	RAINBOW = 0,
	GRAY,
	HISTOGRAM,
	NONE
};

enum class tagPCColor
{
	NONE = 0,
	CUR = 1,
	DEPTH = 2
};

struct PcYXZ
{
	float x, y, z;
	void Reset() {
		x = y = z = 0.0f;
	}

	PcYXZ() {
		Reset();
	}

	PcYXZ& operator=(const PcYXZ& R) {
		x = R.x;
		y = R.y;
		z = R.z;

		return *this;
	}
};

struct C888RGB
{
	uint8_t r, g, b;
	void Reset() {
		r = g = b = 0.0f;
	}

	C888RGB() {
		Reset();
	}

	C888RGB& operator=(const C888RGB& R) {
		r = R.r;
		g = R.g;
		b = R.b;

		return *this;
	}
};

struct PcRGB
{
	float r, g, b;
	void Reset() {
		r = g = b = 0.0f;
	}

	PcRGB() {
		Reset();
	}

	PcRGB& operator=(const PcRGB& R) {
		r = R.r;
		g = R.g;
		b = R.b;

		return *this;
	}
};

struct OBPoint
{
	PcYXZ Pos;
	PcRGB Color;

	OBPoint& operator=(const OBPoint& R)
	{
		Pos = R.Pos;
		Color = R.Color;

		return *this;
	}
};
struct C888DATA
{
	uint8_t * data8bitPtr;
	int width;
	int height;
	uint32_t dataSizeOfByte;

	C888DATA& operator=(const C888DATA& R)
	{
		UpdateData(R.data8bitPtr, width, height);
	}

	void UpdateData(const uint8_t* src, int w, int h)
	{
		if (nullptr == src)
			return;
		if (w != width || h != height)
		{
			DeleteData();
			width = w;
			height = h;
			dataSizeOfByte = w * h * 2 * sizeof(uint8_t);
			data8bitPtr = new uint8_t[dataSizeOfByte]();
		}
		memcpy(data8bitPtr, src, dataSizeOfByte);
	}

	void DeleteData()
	{
		if (nullptr != data8bitPtr)
		{
			delete data8bitPtr;
			data8bitPtr = nullptr;
		}
	}

	void Reset()
	{
		width = height = dataSizeOfByte = 0;
		DeleteData();
	}

	C888DATA()
	{
		width = height = dataSizeOfByte = 0;
		data8bitPtr = nullptr;
	}
};

struct MappingData
{
	C888DATA depthPtr;
	C888DATA colorPtr;
	C888DATA mappingPtr;

	void UpdateD(const uint8_t* src, int w, int h)
	{
		depthPtr.UpdateData(src, w, h);
	}

	void UpdateC(const uint8_t* src, int w, int h)
	{
		//color_ptr.DeleteData();
		if (w != depthPtr.width)
		{
			colorPtr.Reset();
			return;
		}
		else
		{
			colorPtr.UpdateData(src, w, h);
		}
	}

	void Reset()
	{
		depthPtr.Reset();
		colorPtr.Reset();
		mappingPtr.Reset();
	}

	MappingData()
	{
		Reset();
	}
};

struct WorldTransform
{
	const uint16_t * depthDataPtr;

	int width, height;

	DepthDrawModel model;

	uint8_t *color_data_ptr;

	WorldTransform()
	{
		depthDataPtr = nullptr;
		color_data_ptr = nullptr;

		width = height = 0;

		model = DepthDrawModel::NONE;

	}

	WorldTransform & operator=(WorldTransform& R)
	{
		depthDataPtr = R.depthDataPtr;

		width = R.width;
		height = R.height;

		model = R.model;

		color_data_ptr = R.color_data_ptr;

		return *this;
	}
};

using cb_update_pc = std::function<void(QVector<OBPoint>)>;
using cb_update_depth = std::function<void(OniData&)>;
using cb_update_mapping = std::function<void(C888DATA&)>;

class Calc : public QThread
{
	Q_OBJECT
public:

	static Calc* Instance();
	const float kPcScale = 800;
	void CreateRainbowPallet();
	void createColorPallet();
	void getHistogram(const int16_t* src, int8_t* dst, int width, int height, bool YFlip = true);

	int calcParamMat(const OBCameraParams& cam_par, float *mat_16);

	QVector<OBPoint> calcPointCloud(WorldTransform wt_);      

	void loadPly(const QString ply_filename);
	void calcMapping(MappingData& mapping_data);
	void setDrawModel(DepthDrawModel draw_mode);

	void setRainbow(int r_value, int r_step);
	void setD2CDistance(int distance);
	bool toWorldSpace(const uint16_t* depth_16bit, QVector<OBPoint> &obp_vec, const uint8_t* rgb_888bit, bool save_color,
		int width, int height, float fxy, float cx, float cy);

	bool writePly(const char* ply_name, const float *obp_vec, int size, bool isPly,int w,int h);

	bool rainbowFunc(OniData& oni_data);         // ThreadDepthCalc()
	bool updateD2CFunc(OniData& oni_data);
	void setCoordZ(int z_diff) { mCoordDiff = z_diff; }

	void setMaxDepth(int max_depth) { mMaxDepth = max_depth; }

	void setDepthRatio(float d_ratio) { mDepthRatio = d_ratio; };

	std::vector<std::string> stringSplit(std::string strtem, char a);

	void setInteriParameter(const float &fx, const float &cx, const float &cy) {
		fx_ = fx; cx_ = cx; cy_ = cy;
		emit SigInterParameterChanged();
	}

	bool interiParamValid() {
		return (fx_ > 10 && cy_ > 10 && cy_ > 10);
	};

	void getInteriParam(float &fx, float &cx, float &cy) const
	{
		fx = fx_; cx = cx_; cy = cy_;
	}

	double correl(std::vector<double> &A, std::vector<double> &B);

	double Calc::interpolate(double x, double x0, double y0, double x1, double y1);
	void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);

	void setMaxDepthColorDistance(int value);
	void setMinDepthColorDistance(int value);
signals:
	void SigInterParameterChanged();
protected:
	void ThreadLoadPly();
private:
	Calc(); 
	static Calc* gInstance;

private:

	QMatrix4x4 mMatrix4;
	QMutex mQmutexCalc;

	uint8_t mPalletIntsRed[256];
	uint8_t mPalletIntsGreen[256];
	uint8_t mPalletIntsBlue[256];
	int mD2cRenderDistance = 2000;
	QString mPlyFilename;

	// oni data 
	bool mDepthFinishFlag{ true };

	bool mMappingFinishFlag{ true };

	bool mLoadPlyFinishFlag{ true };

	WorldTransform wt_; 
	cb_update_depth mUpdateDepth;

	cb_update_mapping mUpdateMapping;

	DepthDrawModel mDeptMode{ DepthDrawModel::GRAY };

	int mRainbowValue{ 2000 };

	int mRainbowStep{ 0 };

	float mDepthRatio{ 0.5f };

	int mCoordDiff{ 300 };

	uint32_t mMaxDepth{ 8000 };
	int mMaxDepthColorDistance{ 3000 };
	int mMinDepthColorDistance{ 0 };
	float fx_{ 0.f };
	float cx_{ 0.f };
	float cy_{ 0.f };
};

#endif // CALC_H
