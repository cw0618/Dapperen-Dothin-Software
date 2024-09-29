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
#ifndef IRCONTROLVIEW_H
#define IRCONTROLVIEW_H

#include <QWidget>

namespace Ui {
	class IRControlView;
}

class IRControlView : public QWidget
{
	Q_OBJECT

public:
	explicit IRControlView(QWidget *parent = nullptr);
	~IRControlView();
	const int mResolutionChange = 1;
	const int mIRMaxDistance = 2;
	const int mIRMinDistance = 3;
	void addResolution(QString value);
	void resolutionClear();
	void setCurrentResolution(QString resolution);
	void setResolutionChange(bool change);
	int getCurrentResolution();

signals:
	void IRChangeCallback(int type, QString value);
	private slots:
	void OnResolutionChange(int index);
	void OnIRMaxDistance();
	void OnIRMinDistance();
	void OnChangeSliderIRMax(int value);
	void OnChangeSliderIRMin(int value);
private:
	Ui::IRControlView *ui;
	bool mChangeStatus = true;

	int mMaxIRDistanceValue = 1800;
	int mMinIRDistanceValue = 0;
};

#endif // IRCONTROLVIEW_H
