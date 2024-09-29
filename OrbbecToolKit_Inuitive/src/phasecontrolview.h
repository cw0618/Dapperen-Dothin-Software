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
#ifndef PHASECONTROLVIEW_H
#define PHASECONTROLVIEW_H

#include <QWidget>

namespace Ui {
	class PhaseControlView;
}

class PhaseControlView : public QWidget
{
	Q_OBJECT

public:
	explicit PhaseControlView(QWidget *parent = nullptr);
	~PhaseControlView();
	const int kResolutionChange = 1;

	void setCurrentResolution(QString resolution);
	int getCurrentResolution();
	void addResolution(QString value);
	void resolutionClear();
	int resolutionSize();
	void setResolutionChange(bool change);
	void setFreqMode(int mode);
private:
	Ui::PhaseControlView *ui;
	bool mChangeStatus = true;
	private slots:
	void OnResolutionChange(int index);
signals:
	void phaseChangeCallback(int type, QString value);
};

#endif // PHASECONTROLVIEW_H
