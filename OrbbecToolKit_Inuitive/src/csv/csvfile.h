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
#ifndef CSVFILE_H
#define CSVFILE_H

#include <QFile>
#include <QMutex>
#include <QSemaphore>
#include <QThread>

#define MESSAGE_QUEUE_SIZE 40
/**
 * 
 */
typedef struct _MSGQueue {
	QString  msgs[MESSAGE_QUEUE_SIZE];
	int front;
	int rear;
} MSGQueue;


class CSVFile : public QThread
{
	Q_OBJECT
private:
	QString mFileName;
	QString mFileDir;
	QString mFilePath;

	QFile *mFileBean{ NULL };
	QSemaphore mSemaphore;
	QMutex mMutex;
	MSGQueue mQueue;

	bool mAlive{ false };
	bool mTitledRecorded{ false };

public:
	explicit CSVFile(QObject *parent = nullptr);
	~CSVFile();

	int startRecord(QString fileName, int timeup);
	bool isRecording();
	void record(QString msg);
	bool hasRecordTitle();
	void recordTitle(QString msg);
	void stopRecord();
	void setFileDir(QString path);
	int roi_rows_num{ 0 };

private:
	int obQueue_isEmpty();
	int obQueue_isFull();

signals:
	void record_finish();

	public slots:
	void timeUp();
protected:
	void run();
};

#endif // CSVFILE_H
