#include "RegisterIniFile.h"
#include <QFileInfo>
#include <QtDebug>

RegisterIniFile::RegisterIniFile()
{
}


RegisterIniFile::~RegisterIniFile()
{
	if (settingsread != nullptr)
	{
		delete settingsread;

	}
}

int RegisterIniFile::LoadRegisterIni(QString ini_file)
{
	QFileInfo Ifile(ini_file);
	if (!Ifile.isFile())
	{
		QString msg = QString("Can't find %1").arg(ini_file);
		qDebug() << msg;
		return -1;
	}
	mRegisterList.clear();
	settingsread = new QSettings(ini_file, QSettings::IniFormat);
	QStringList allKey = settingsread->allKeys();
	foreach(QString key, allKey)
	{
		REGISTER_DATA registerData;
		registerData.address = key;
		registerData.value = settingsread->value(key).toString();
		mRegisterList.append(registerData);
	}
	return 0;
}