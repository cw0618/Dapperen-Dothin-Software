#include "capturethread.h"
#include <QDateTime>
/** \class CaptureThread
*
* 延时子线程
*
*/
CaptureThread::CaptureThread()
{
	mCountTimer = new QTimer(this);
	connect(mCountTimer, SIGNAL(timeout()), this, SLOT(onTimeoutControl()));
}

CaptureThread::~CaptureThread()
{
}
void CaptureThread::run() {
	//mMutex.lock();
	//if (mTotalTime >= 0)
	//{
	//	this->msleep(mTotalTime);
	//	if (mTimeRun)
	//	{
	//		emit onStepFinish(true);
	//	}
	//}
	//mMutex.unlock();
	if (!mCountTimer->isActive())
	{
		mCountTimer->start(mTotalTime);
	}
}
void CaptureThread::startTimerThread(int time) {
	qDebug() << "howard startTimerThread=" << time;
	mMutex.lock();
	mTotalTime = time;
	mTimeRun = true;

	start();
	mMutex.unlock();
}
void CaptureThread::stopTimerThread() {
	mMutex.lock();
	mTimeRun = false;
	if (mCountTimer->isActive())
	{
		mCountTimer->stop();
	}
	mTotalTime = 0;
	mMutex.unlock();
}
bool CaptureThread::getThreadStatus() {
	return mTimeRun;
}
void CaptureThread::onTimeoutControl()
{
	emit onStepFinish(true);
}