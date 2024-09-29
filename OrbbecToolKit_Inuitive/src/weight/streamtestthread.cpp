#include "streamtestthread.h"

StreamTestThread::StreamTestThread()
{
	mPTimer = new QTimer();
	mPTimer->moveToThread(this);
	connect(mPTimer, SIGNAL(timeout()), this, SLOT(timeOutSlot()), Qt::DirectConnection);
}
StreamTestThread::~StreamTestThread()
{

}
void StreamTestThread::stopThreadTime() {
	mIsAlive = false;
	start();
}
void StreamTestThread::startThreadTime(int time) {
	mRefreshTime = time;
	mIsAlive = true;
	start();
}
void StreamTestThread::run() {
	if (mIsAlive)
	{
		mPTimer->setTimerType(Qt::PreciseTimer);
		mPTimer->setInterval(mRefreshTime);
		mPTimer->start();
	}
	else
	{
		mPTimer->stop();
	}
}
void StreamTestThread::timeThreadRelease() {
	mPTimer->stop();
	mPTimer->destroyed();
}
void StreamTestThread::timeOutSlot() {
	emit threadMessage();
}