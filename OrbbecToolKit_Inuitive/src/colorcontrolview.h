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
#ifndef CAMERACONTROLVIEW_H
#define CAMERACONTROLVIEW_H

#include <QWidget>

namespace Ui {
	class ColorControlView;
}

class ColorControlView : public QWidget
{
	Q_OBJECT

public:
	explicit ColorControlView(QWidget *parent = nullptr);
	~ColorControlView();
	const int kResolutionChange = 1;
	const int kD2cRenderChange = 2;
	const int kD2cDistanceChange = 3;
	const int kPlaneRenderChange = 4;

	bool mD2cState = false;
	void addResolution(QString value);
	void resolutionClear();
	void setCurrentResolution(QString resolution);
	void setResolutionIndex(int index);
	int getCurrentResolution();
	QString getStrResolution();
	void setResolutionChange(bool change);
	void setD2CStatus(int value);
	bool getD2CState() const;
	int getD2CDistance();
	void setDistance(bool state);
	void setD2CEnable(bool enable);

	private slots:
	void OnResolutionChange(int index);
	void onD2CRenderChanged(bool state);
	void onPlaneRenderChanged(bool state);
	void ChangeSliderD2CValue(int value);
	void setD2CDistance();
signals:
	void rgbChangeCallback(int type, QString value);
private:
	Ui::ColorControlView *ui;
	bool mChangeStatus;
	bool mPlaneState = false;
	int mD2cDistanceValue = 2000;
	//QString currentItem;
};

#endif // CAMERACONTROLVIEW_H
