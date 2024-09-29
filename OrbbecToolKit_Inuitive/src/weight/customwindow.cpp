#include "customwindow.h"
#include "ui_customwindow.h"
#include <QDebug>
/** \class CustomWindow
*
* 各路数据流的基类
*
*/
CustomWindow::CustomWindow(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::CustomWindow)
{
	ui->setupUi(this);
	ui->gb_content->setVisible(false);
	connect(ui->tb_content_switch, SIGNAL(clicked(bool)), this, SLOT(OnShowContent(bool)));
	connect(ui->tb_camera_switch, SIGNAL(clicked()), this, SLOT(OnCameraSwitch()));
}
CustomWindow::~CustomWindow()
{
	delete ui;
}
void CustomWindow::setCameraStatus(bool status) {
	if (status) {
		mCameraStatus = true;
		ui->tb_camera_switch->setText("on");
		QString str;
		str.append("QToolButton{background-color: rgba(255, 255, 255, 0);}");
		str.append("QToolButton{color: rgb(40,132,224);}");
		ui->tb_camera_switch->setStyleSheet(str);
		ui->tb_camera_switch->setIcon(QIcon(":/res/image/turn_on.png"));
	}
	else
	{
		mCameraStatus = false;
		ui->tb_camera_switch->setIcon(QIcon(":/res/image/turn_off.png"));
		QString str;
		ui->tb_camera_switch->setText("off");
		str.append("QToolButton{background-color: rgba(255, 255, 255, 0);}");
		str.append("QToolButton{color: rgb(255,101,99);}");
		ui->tb_camera_switch->setStyleSheet(str);
	}
}
void CustomWindow::setSwitchVisible(bool enable)
{
	ui->tb_camera_switch->setVisible(enable);
}
int CustomWindow::OnCameraSwitch() {
	if (mCameraStatus) {
		mCameraStatus = false;
		ui->tb_camera_switch->setIcon(QIcon(":/res/image/turn_off.png"));
		QString str;
		ui->tb_camera_switch->setText("off");
		str.append("QToolButton{background-color: rgba(255, 255, 255, 0);}");
		str.append("QToolButton{color: rgb(255,101,99);}");
		ui->tb_camera_switch->setStyleSheet(str);
		emit clickViewCallback(kCameraSwitchType, 0);
	}
	else
	{
		mCameraStatus = true;
		ui->tb_camera_switch->setText("on");
		QString str;
		str.append("QToolButton{background-color: rgba(255, 255, 255, 0);}");
		str.append("QToolButton{color: rgb(40,132,224);}");
		ui->tb_camera_switch->setStyleSheet(str);
		ui->tb_camera_switch->setIcon(QIcon(":/res/image/turn_on.png"));
		emit clickViewCallback(kCameraSwitchType, 1);
	}

	return 0;
}
void CustomWindow::OnOnlyShowContent(bool show) {
	mContentStatus = show;
	ui->gb_content->setVisible(show);
	if (show)
	{
		ui->tb_content_switch->setIcon(QIcon(":/res/image/content_down.png"));
		
	}
	else
	{
		ui->tb_content_switch->setIcon(QIcon(":/res/image/content_up.png"));
	}
}
int CustomWindow::OnShowContent(bool status) {
	//qDebug()<<"howard"<<status;
	if (mContentStatus) {
		mContentStatus = false;
		ui->gb_content->setVisible(false);
		ui->tb_content_switch->setIcon(QIcon(":/res/image/content_up.png"));
	}
	else
	{
		mContentStatus = true;
		ui->gb_content->setVisible(true);
		ui->tb_content_switch->setIcon(QIcon(":/res/image/content_down.png"));
	}

	return 0;
}
int CustomWindow::addLayout(QLayout *layout) {
	//ui->gb_content-
	ui->gb_content->setLayout(layout);

	return 0;
}
void CustomWindow::setTitle(QString title) {
	ui->tb_title->setText(title);
}
int CustomWindow::addWidget(QWidget *widget) {
	//    ui->gb_content->add(widget);
	return 0;
}

void CustomWindow::showSwitch(bool show) {
	if (show) {
		ui->tb_camera_switch->show();
	}
	else {
		ui->tb_camera_switch->hide();
	}
}
void CustomWindow::setContentEnable(bool enabled) {
	ui->tb_content_switch->setEnabled(enabled);
}
void CustomWindow::setSwitchEnable(bool enabled) {
	ui->tb_camera_switch->setEnabled(enabled);
}