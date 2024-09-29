#include "widgetcmd.h"
#include "ui_widgetcmd.h"
#include <QDebug>
/** \class WidgetCmd
*
* cmd指令窗口
*
*/

QString cmd_clear(int argc, QStringList argv, void* handle)
{
	MPlainTextEdit* th = (MPlainTextEdit*)handle;
	th->clear();

	return QString("");
}

WidgetCmd::WidgetCmd(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::WidgetCmd)
{
	ui->setupUi(this);
	mCommandManager = new CommandManager(this);
	connect(mCommandManager, SIGNAL(updateInfo(QString)), ui->et_screen, SLOT(mOnShowInfo(QString)));
	connect(mCommandManager, SIGNAL(wait_cmd(QString)), ui->et_screen, SLOT(mOnWaitCommand(QString)));
	connect(ui->et_screen, SIGNAL(cmdStr(QString)), mCommandManager, SLOT(on_input_cmd(QString)));


	Cmder cmder;
	cmder.cmd_executer = cmd_clear;
	cmder.handle = ui->et_screen;
	mCommandManager->add_cmdSupport("clear", cmder);


	mCommandManager->startWork();
}

WidgetCmd::~WidgetCmd()
{
	delete ui;
	if (mCommandManager != NULL) {
		delete mCommandManager;
		mCommandManager = NULL;
	}
}

void WidgetCmd::showEvent(QShowEvent *event)
{

}
void WidgetCmd::add_cmdSupport(QString name, Cmder cmder)
{
	mCommandManager->add_cmdSupport(name, cmder);
}