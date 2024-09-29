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
#ifndef COMMONCONTROLWIDGET_H
#define COMMONCONTROLWIDGET_H

#include <QWidget>
#include<QTimer>
#include <QFileDialog>
#include <qfile.h>
#include<QTimerEvent>
#include <QTimer>
#include <QThread>
namespace Ui {
	class CommonControlWidget;
}

class CommonControlWidget : public QWidget
{
	Q_OBJECT

public:
	explicit CommonControlWidget(QWidget *parent = nullptr);
	~CommonControlWidget();
	const int kCommonStreanTestChange = 1;
	//const int kCommonLaserChange = 2;
	//const int kCommonFloodChange = 3;
	//const int kCommonBackgrounpChange = 4;
	const int kCommonGainChange = 5;
	const int kCommonExposureChange = 6;
	const int kCommonSoftwareReset = 7;
	const int kCommonHardwareReset = 8;
	const int kCommonModeChange = 9;
	const int kCommonBiningModeChange = 10;
	const int kCommonFlipModeChange = 11;
	const int kCommonWindowOriginyChange = 12;
	const int kCommonWindowOriginxChange = 13;
	const int kCommonWindowHeightChange = 14;
	const int kCommonWindowWidthChange = 15;
	const int kCommonOddDgainChange = 16;
	const int kCommonEvenDgainChange = 17;
	const int kCommonSubSampChange = 18;
	const int kCommonDsleepChange = 19;
	const int kCommonHdrChange = 20;
	const int kCommonHistChange = 21;
	const int kCommonMedianChange = 22;
	const int kCommonEbcChange = 23;
	const int kCommonLscChange = 24;
	const int kCommonSubSampvChange = 25;
	const int kCommonRowCorrection = 26;
	bool mLaserState = true;
	bool mFloodState = true;
	bool mBackgrounpState = false;
	//void setLaserStatus(bool value); 
	//void setFloodStatus(bool value);
	//void setBackgrounpStatus(bool value);
	//bool IRGainSet(int value);

	private slots:
	void onSoftwareResetClick();
	void onHardwareResetClick();
	//void onFloodStateChanged(bool state);
	//void onLaserStateChanged(bool state);
	//void onBackgrounpStateChanged(bool state);
	void onModeChanged(int mode);
	void onBiningModeChanged(int mode);
	void onWindowSet();
	void onFlipModeChanged(int mode);
	void onSubSampChanged(int value);
	void onSubSampvChanged(int value);
	void onGainChanged(int value);
	//void OnIRGainSet();
	//void OnChangeSliderIRGain(int value);
	void onDgainChanged(bool state);
	void onDsleepChanged(bool state);
	void onHdrChanged(bool state);
	void onHistChanged(bool state);
	void onMedianChanged(bool state);
	void onEbcChanged(bool state);
	void onLscChanged(bool state);
	void onRowCorrectionChanged(bool state);

signals:
	void commonWidgetChangeCallback(int type, QString value);
private:
	Ui::CommonControlWidget *ui;
	bool mCameraStatus{ false };
	bool mFrequencyChange = true;
	int mIRGain = 0;
	int mIRExposure = 0;
};

#endif // COMMONCONTROLWIDGET_H
