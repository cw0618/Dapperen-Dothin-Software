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
#ifndef VIDEOWINDOW_H
#define VIDEOWINDOW_H

#include <QWidget>
#include "src/device/onisensor.h"
#include"src/dialog/messagetip.h"
#include "src/calculate/calc.h"
#include"src/device/imageutils.h"
#include"src/opengl/mainGLWidget.h"
namespace Ui {
	class VideoWindow;
}

class VideoWindow : public QWidget
{
	Q_OBJECT

public:
	explicit VideoWindow(QWidget *parent = nullptr);
	~VideoWindow();
	const int kVideoCloseSwitchType = 1;
	const int kVideoFullScreenType = 2;
	const int kVideoCaptureType = 3;
	const int kVideoPauseType = 4;
	const int kStrPhase = 20;
	const int kStrIR = 21;
	const int kStrAMP = 22;
	const int mStrDepth = 23;
	const int kStrColor = 24;
	uint16_t mIR16bit = 0;
	const int kStrUvcColor = 26;
	const int kStrCloudPoint = 25;
	bool d2cStart = false;
	void setStreamMode(int mode);
	bool getFullScreenStatus();
	void setVideoTitle(QString title);
	void updateDepthData(OniData* data);
	void UpdateIrData(OniData *data, int w, int h, RenderType renderType);
	void UpdateColorData(OniData *data);

	void UpdateD2CData(int8_t* raw16, int w, int h, int size);

	void updatePointCouldData(OniData* data);
	void setDepthMode(DepthDrawModel depthMode);
	void setFrameRate(int frame_rate);
	void setD2CStart(bool state);
	void setAIBodyShape(bool state);
	void showCaptureFinishTip();
	void updata2DBody(AIBody& body_);
	void setAIRenderType(RenderType type);
	virtual void timerEvent(QTimerEvent * event);
	private slots:
	void OnFullScreenSwitch();
	void OnVideoCloseSwitch();
	void OnCaptureClick();
	void OnPauseClick();
signals:
	void videoViewCallback(int type, int value);
private:
	Ui::VideoWindow *ui;
	MessageTip * mMessageTip{ nullptr };
	OniData mDepthData;
	OniData mIrColorData;
	bool mIsFullScreen{ false };
	bool mIsPause{ false };
	Calc* mCalcBean{ nullptr };
	int mTimerId;
	int mStreamMode = 20;
	ImageUtils mImageUtilsBean;
	DepthDrawModel  mDepthDrawMode = DepthDrawModel::GRAY;
};

#endif // VIDEOWINDOW_H
