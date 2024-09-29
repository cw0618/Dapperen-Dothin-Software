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
#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H
#include <QThread>
#include<qdebug.h>
#include <mutex>
#include <QTimer>
class CaptureThread : public QThread
{
	Q_OBJECT

public:
	explicit CaptureThread();
	~CaptureThread();
protected:
	void run();
private:
	int mTotalTime = 0;
	bool mTimeRun{ false };
	QTimer *mCountTimer{ NULL };
	std::mutex mMutex;
public:
	void startTimerThread(int time);
	void stopTimerThread();
	bool getThreadStatus();
signals:
	void onStepFinish(bool status);
	public slots:
	void onTimeoutControl();
};
#endif // CAPTURETHREAD_H