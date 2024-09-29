#include "csvfile.h"
#include <qDebug>
#include <QDir>
#include <QTimer>
/** \class CaptureDialog
*
* csv数据的保存类
*
*/
CSVFile::CSVFile(QObject *parent) : QThread(parent)
{
	mFileDir = QString(".\\capture");
	QDir dir(mFileDir);
	if (!dir.exists()) {
		dir.mkpath(".\\");
	}
	mQueue.front = mQueue.rear = 0;

}

CSVFile::~CSVFile()
{
	stopRecord();
}
void CSVFile::setFileDir(QString path)
{
	mFileDir = path;
	QDir dir(mFileDir);
	if (!dir.exists()) {
		dir.mkpath(".\\");
	}
}
int CSVFile::startRecord(QString fileName, int timeup)
{
	int ret = 0;
	if (!mAlive) {
		this->mFileName = fileName;
		mFilePath = mFileDir + "\\" + fileName + ".csv";

		mQueue.front = mQueue.rear = 0;
		mTitledRecorded = false;

		mFileBean = new QFile(mFilePath);
		mAlive = mFileBean->open(QIODevice::WriteOnly | QIODevice::Append);
		if (mAlive) {
			start();
			if (timeup != -1) {

				QTimer::singleShot(timeup * 1000, this, SLOT(timeUp()));
			}
		}
		else {
			ret = -1;
		}
	}

	return ret;
}


void CSVFile::stopRecord()
{
	mAlive = false;
	mSemaphore.release();

	requestInterruption();
	quit();
	wait();
}

void CSVFile::run()
{
	qDebug() << "CSVFile run start";
	while (mAlive && !isInterruptionRequested()) {
		mSemaphore.acquire();
		mMutex.lock();
		if (!obQueue_isEmpty() && mAlive) {
			mQueue.front = (mQueue.front + 1) % MESSAGE_QUEUE_SIZE;
			QString msg = (mQueue.msgs[mQueue.front]);

			QTextStream text_stream(mFileBean);
			text_stream << msg << "\r\n";
		}
		mMutex.unlock();
		mFileBean->flush();

	}

	if (mFileBean->isOpen()) {
		mFileBean->close();
	}

	emit record_finish();
	qDebug() << "CSVFile run over";

}

void CSVFile::timeUp()
{
	qDebug() << "CSVFile timeUp";
	if (mAlive) {
		stopRecord();
	}
}

bool CSVFile::isRecording()
{
	return mAlive;
}

bool CSVFile::hasRecordTitle()
{
	return mTitledRecorded;
}

void CSVFile::recordTitle(QString msg)
{
	mTitledRecorded = true;
	record(msg);
}

void CSVFile::record(QString _msg)
{
	if (mAlive) {
		mMutex.lock();
		mQueue.rear = (mQueue.rear + 1) % MESSAGE_QUEUE_SIZE;
		mQueue.msgs[mQueue.rear] = _msg;
		mMutex.unlock();
		mSemaphore.release();
	}
}

int CSVFile::obQueue_isEmpty()
{
	return mQueue.front == mQueue.rear;
}

int CSVFile::obQueue_isFull()
{
	return mQueue.front == (mQueue.rear + 1) % MESSAGE_QUEUE_SIZE;
}
