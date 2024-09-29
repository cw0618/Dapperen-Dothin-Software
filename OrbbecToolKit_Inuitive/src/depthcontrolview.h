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
#ifndef DEPTHCONTROLVIEW_H
#define DEPTHCONTROLVIEW_H

#include <QWidget>
#include <QDir>
#include <qmessagebox.h>


namespace Ui {
	class DepthControlView;
}

class DepthControlView : public QWidget
{
	Q_OBJECT

public:
	explicit DepthControlView(QWidget *parent = nullptr);
	~DepthControlView();
	const int kResolutionChange = 1;
	const int kVideoMode = 2;
	const int kPointCloundSwitch = 3;
	const int kDepthColorMaxDistance = 4;
	const int kDepthColorMinDistance = 5;
	void addResolution(QString value);
	void resolutionClear();
	void setCurrentResolution(QString resolution);
	void setResolutionChange(bool change);
	void setPointCloundStatus(bool status);
	int getCurrentResolution();
	QString getVideoMode();
	void GetFilterFileName();
	void readPointFilterFromIni();
	void hidePointFilterSet();
	void setDepthEnable(bool state);

	private slots:
	void OnVideoModeChange(int index);
	void OnResolutionChange(int index);
	void OnSliderChange(int val);
	void onPointCloundChanged(bool state);
	void OnDepthMaxDistance();
	void OnDepthMinDistance();
	void OnChangeSliderMaxValue(int value);
	void OnChangeSliderMinValue(int value);
	public slots:
	void on_comboBox_filter_bin_currentIndexChanged(const QString &text);

signals:
	void depthChangeCallback(int type, QString value);
	void sliderChange(int val);
	void filterNameChange(QString);
	void changeFilterParams(double*);
	void ChangeSliderDepthValue(int value);

private:
	Ui::DepthControlView *ui;
	bool change_status = true;
	int nMin = 3000;
	int nMax = 10000;
	int nSingleStep = 10;
	QString currentFilterFile{ "" };
	double pointCloudFilterParams[3];
	int mMaxDepthDistanceValue = 3000;
	int mMinDepthDistanceValue = 0;
};

#endif // DEPTHCONTROLVIEW_H
