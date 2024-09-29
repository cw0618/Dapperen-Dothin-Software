#include "devicestatuswidget.h"
#include "ui_devicestatuswidget.h"
#include<QPropertyAnimation>
/** \class DeviceStatusWidget
*
* 设备状态的显示类
*
*/
DeviceStatusWidget::DeviceStatusWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::DeviceStatusWidget)
{
	ui->setupUi(this);
}
DeviceStatusWidget::~DeviceStatusWidget()
{
	delete ui;
}
void DeviceStatusWidget::setDeviceStatus(QString contect, int status) {
	if (status !=0)
	{
		QString str;
		str.append("color: rgb(255, 0, 0);").append("font: 18pt ""Agency FB"";");
		ui->label_device_status->setStyleSheet(str);
	}
	else {
		QString str;
		str.append("color: rgb(195, 213, 229);").append("font: 18pt ""Agency FB"";");
		ui->label_device_status->setStyleSheet(str);
	}

	ui->label_device_status->setText(contect);
}
void DeviceStatusWidget::startAnimation() {
}
void DeviceStatusWidget::stopAnimation() {

}