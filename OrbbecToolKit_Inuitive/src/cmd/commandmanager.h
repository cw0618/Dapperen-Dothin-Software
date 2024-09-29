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
#ifndef COMMANDMANAGER_H
#define COMMANDMANAGER_H

#define CMD_HOST "obkit"
#define WELCOME_INFO (("welcome to command mode."))

#include <QMap>
#include <QSemaphore>
#include <QThread>
#include <src/device/common.h>

class CommandManager : public QThread
{
	Q_OBJECT
public:
	explicit CommandManager(QObject *parent = nullptr);
	~CommandManager();

	void startWork();

	void add_cmdSupport(QString name, Cmder cmder);

	QString _cmdhelp();

private:
	bool alive{ false };
	QString userName{ "orbbec " };
	QString cmdTag{ "" };

	QSemaphore m_sem;

	QString cmdIn;
	QString cmdFetch;

	int argc;
	QStringList argv;

	//支持的命令池
	QMap<QString, Cmder> cmdPool;
	/**
	* fetch 取指
	* 
	*/
	void fetch();
	/**
	* fetch 解码
	*
	*/
	void decode();
	/**
	* execute 取指
	*
	*/
	void execute();


signals:
	void updateInfo(QString info);
	void wait_cmd(QString tag);

	public slots:
	void on_input_cmd(QString in);
	// QThread interface
protected:
	void run();
};

#endif // COMMANDMANAGER_H
