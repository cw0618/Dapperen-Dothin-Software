
#include "aboutdialog.h"
#include "ui_aboutdialog.h"
/** \class AboutDialog
* 
* 版本信息显示类
*
*/
AboutDialog::AboutDialog(SensorBase *sensorbase_, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AboutDialog)
{
    ui->setupUi(this);
	mSensorDevice = sensorbase_;
    setWindowTitle("About");
	ui->label_firmware_version->setVisible(false);
	ui->label_6->setVisible(false);
	getVersion();
}

AboutDialog::~AboutDialog()
{
    delete ui;
}
void AboutDialog::getVersion() {

	//QString tool_version = "V0.0.1.
	QString toolVersion = "iToFViewer: "+ QString(TOOL_VERSION);
	ui->label_orbbec_version->setText(toolVersion);

	openni::Version openniVersion = openni::OpenNI::getVersion();
	QString openniSdk = QString("v%1.%2.%3.%4").arg(openniVersion.major).arg(openniVersion.minor).arg(openniVersion.maintenance).arg(openniVersion.build);
	ui->label_driver_version->setText(openniSdk);

	if (true == mSensorDevice->IsOpened()) {
		char version[256] = { 0 };

		QString versionStr;
		mSensorDevice->mOniDevice.getPlatformVersion(version);
		versionStr.append(QString("platform version: %1\n").arg(version));
		openni::SensorID sensorid;
		sensorid = mSensorDevice->mOniDevice.getSensorID(openni::SensorType::SENSOR_DEPTH);
		versionStr.append(QString("sensor id: 0x%1\n").arg((uint16_t)sensorid, 4, 16, QLatin1Char('0')));
		openni::SerialNumberMap serialNumber = { openni::SerialType::SERIAL_DEVICE, 0 };
		mSensorDevice->mOniDevice.getSerialNumber(&serialNumber);
		versionStr.append(QString("serial number: %1\n").arg(serialNumber.serial));
		//ui->version->setText(versionStr);

	}
}
