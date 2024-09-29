#include "opendevicethread.h"
/** \class OpenDeviceThread
*
* 设备连接类
*
*/
OpenDeviceThread::OpenDeviceThread(QObject *parent)
	: QThread(parent)
{
}

OpenDeviceThread::~OpenDeviceThread()
{
}
int OpenDeviceThread::CreateThread(SensorBase *sensorbase_) {
	mSensorDevice = sensorbase_;
	mOperateTye = DEVICE_OPEN;
	start(); 
	return 0;
}	

int OpenDeviceThread::loadRegisterFile(DeviceOperateType operateType)
{
	mOperateTye = operateType;
	start();
	return 0;
}

void OpenDeviceThread::run() {
	if (mOperateTye == DEVICE_OPEN)
	{
		if (mSensorDevice != nullptr)
		{
			auto ret = mSensorDevice->OpenAnyDevice();
			qDebug() << QString::fromStdString(ret.second);
			if (0 == ret.first)
			{
				emit openDeviceStatus(kDeviceOpenSuccess);
			}
			else {
				emit openDeviceStatus(kDeviceOpenFailure);
			}
		}
	}
	else if (mOperateTye == LOAD_REGISTER)
	{
		bool ret = mSensorDevice->loadAllRegister();
		emit openDeviceStatus(kLoadRegisterSuccess);
	}
}