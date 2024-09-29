#include "iniconfig.h"

#include <QFileInfo>
#include <QtDebug>
/** \class IniConfig
*
* config.ini文件的属性类，在config/config.ini
*
*/
IniConfig::IniConfig()
{
	//InitResMap();
}

int IniConfig::LoadIni(QString ini_file)
{
	QFileInfo Ifile(ini_file);
	if (!Ifile.isFile())
	{
		QString msg = QString("Can't find %1").arg(ini_file);
		qDebug() << msg;
		return -1;
	}

	settingsread = new QSettings(ini_file, QSettings::IniFormat);
	int ir_open = settingsread->value("IR/Open").toInt();
	config_res_.mOpenIr = ir_open == 1 ? true : false;
	int right_ir_open = settingsread->value("Phase/Open").toInt();
	config_res_.mOpenPhase = right_ir_open == 1 ? true : false;

	int color_open = settingsread->value("Color/Open").toInt();
	config_res_.mOpenColor = (color_open == 1) ? true : false;

	int depth_open = settingsread->value("Depth/Open").toInt();
	config_res_.mOpenDepth = (depth_open == 1) ? true : false;

	int phase_hide = settingsread->value("Phase/Hide").toInt();
	config_res_.mHidePhase = phase_hide == 1 ? true : false;

	int ir_hide = settingsread->value("IR/Hide").toInt();
	config_res_.mHideIr = ir_hide == 1 ? true : false;
	int color_hide = settingsread->value("Color/Hide").toInt();
	config_res_.mHideColor = color_hide == 1 ? true : false;
	int depth_hide = settingsread->value("Depth/Hide").toInt();
	config_res_.mHideDepth = depth_hide == 1 ? true : false;
	int ai_stream_hide_value = settingsread->value("AIStream/Hide").toInt();
	config_res_.mHideAiStream = ai_stream_hide_value == 1 ? true : false;
	int uvc_color_value = settingsread->value("UVCColor/Hide").toInt();
	config_res_.mHideUvcColor = uvc_color_value == 1 ? true : false;
	int password_state = settingsread->value("Setting/Password").toInt();
	config_res_.mPassword = password_state == 99 ? true : false;
	int stream_test_state = settingsread->value("Setting/StreamTest").toInt();
	config_res_.mStreamTest = stream_test_state == 1 ? true : false;
	config_res_.mStreamMode = settingsread->value("Setting/StreamMode").toInt();
	int filterSet = settingsread->value("Setting/FilterSetOnUI").toInt();
	config_res_.mFilterSetOnUI = (filterSet == 1 ? true : false);

	return 0;
}
void IniConfig::setPasswordValue(int value) {
	if (settingsread != nullptr)
	{
		settingsread->setValue("Setting/Password", value);
	}
}
void IniConfig::InitResMap()
{
}
IniConfig::~IniConfig()
{
	if (settingsread != nullptr)
	{
		delete settingsread;

	}
}