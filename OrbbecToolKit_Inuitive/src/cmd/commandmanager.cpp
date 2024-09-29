/**
* @projectName   OBKit
*   命令管理
* @author        Yuxuewen
* @date          2019-06-20
*/

#include "commandmanager.h"
//#include "common.h"
#include <QDebug>



QString cmd_help(int argc, QStringList argv, void* handle)
{
	CommandManager* manager = (CommandManager*)handle;
	return manager->_cmdhelp();
}

CommandManager::CommandManager(QObject *parent) : QThread(parent), m_sem(0)
{
	cmdTag = QString("%1>>> ").arg(userName);
	cmdPool.clear();

	/*  Cmder cmder;
	  cmder.cmd_executer = cmd_help;
	  cmder.handle = this;

	  add_cmdSupport("help", cmder);*/
}

CommandManager::~CommandManager()
{
	alive = false;
	m_sem.release();
	requestInterruption();
	quit();
	wait();
}

void CommandManager::startWork()
{
	alive = true;
	start();
}


void CommandManager::run()
{
	emit updateInfo(WELCOME_INFO);
	while (alive && !isInterruptionRequested()) {

		fetch();
		decode();
		execute();
	}
}

void CommandManager::on_input_cmd(QString in)
{
	cmdIn = in;
	m_sem.release();
}

//取指
void CommandManager::fetch()
{
	cmdFetch.clear();
	emit wait_cmd(cmdTag);
	m_sem.acquire();

	cmdFetch = cmdIn;
}

//解码
void CommandManager::decode()
{
	argc = 0;
	argv.clear();

	if (!cmdFetch.isEmpty() && alive) {
		// argv = cmdFetch.split(' ',QString::SkipEmptyParts);
		argv << cmdFetch;
		argc = argv.length();
	}
}

//执行
void CommandManager::execute()
{
	if (argc == 0 || !alive) {
		return;
	}
	if (argv[0].contains("rm")) {
		emit updateInfo("You do not have permission to execute the delete command!\n");
		return;
	}

	QString result;
	QString cmdName = argv[0];
	if (cmdPool.contains(cmdName)) {
		Cmder cmder = cmdPool[cmdName];
		if (cmder.cmd_executer != NULL) {
			result = cmder.cmd_executer(argc, argv, cmder.handle);
			result = WELCOME_INFO;
		}

	}
	else {
		//result = QString("'%1' 不支持此命令.\n").arg(cmdName);
		Cmder cmder = cmdPool["common"];
		if (cmder.cmd_executer != NULL) {
			result = cmder.cmd_executer(argc, argv, cmder.handle);
		}
	}

	emit updateInfo(result);
}

void CommandManager::add_cmdSupport(QString name, Cmder cmder)
{
	cmdPool.insert(name, cmder);
}

QString CommandManager::_cmdhelp()
{
	QString helpInfo;
	helpInfo.clear();
	int count = 1;

	QMapIterator<QString, Cmder> it(cmdPool);
	while (it.hasNext()) {
		it.next();
		helpInfo.append(QString("[%1].%2\n").arg(count).arg(it.key()));
		count++;
	}
	return helpInfo;
}
