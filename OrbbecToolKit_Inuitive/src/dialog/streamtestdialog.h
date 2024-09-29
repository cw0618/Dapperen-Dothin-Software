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
#ifndef STREAMTESTDIALOG_H
#define STREAMTESTDIALOG_H

#include <QDialog>
#include<QTimerEvent>
#include <QTimer>
#include <QDebug>
#include <queue>
#include <mutex>
#include"src/device/capturethread.h"
namespace Ui {
	class StreamTestDialog;
}

class StreamTestDialog : public QDialog
{
	Q_OBJECT

public:
	explicit StreamTestDialog(QWidget *parent = 0);
	~StreamTestDialog();
	const int kSwitchChange = 1;
	const int kTimerChange = 2;
	const int kColorError = 10;
	const int kDepthError = 11;
	const int kPhaseError = 12;
	const int kIrError = 13;
	const int kUvcError = 14;

	void startThreadTime(int time, bool close);
	void stopThreadTime();
	void setTaskTotalCount(int count);
	void setTaskCurrentCount(int count);
	int getTaskCount();
	void ShowInfo(QString info);
	void clearLogMethod();
	void finishOneTask();
	void setShowStatus(bool status);
	void setButtonStatus(bool status);
	void addErrorStreamCount(int type);
	void clearErrorCount();
	int getErrorCount();
private:
	Ui::StreamTestDialog *ui;
	int mRefreshTime = 0;
	//一次任务要执行的动作数
	int mItemListCount = 0;
	int mTaskCount = 0;
	int mTaskTotalTount = 0;
	int mTimerId = 0;
	int mErrorAllCount = 0;
	int mErrorColorCount = 0;
	int mErrorDepthCount = 0;
	int mErrorPhaseCount = 0;
	int mErrorIrCount = 0;
	int mErrorUvcCount = 0;
	uint32_t mLogIndex{ 0 };
	std::mutex mMutexLock;
	CaptureThread mTimeThreadOut;
	bool mShowing = false;
	void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;


	private slots:
	void onOneTimeFinish(bool finish);
	void onTestSwitch(bool state);
	void clearLogReleased();

signals:
	void StreamTestCallback(int type, QString value);
};

#endif // STREAMTESTDIALOG_H
