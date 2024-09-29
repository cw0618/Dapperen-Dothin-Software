#pragma once

#include <QSettings>
#include <QMap>
typedef struct REGISTER_DATA_
{
	QString address;
	QString value;

}REGISTER_DATA;
class RegisterIniFile
{
public:
	RegisterIniFile();
	~RegisterIniFile();
	int LoadRegisterIni(QString ini_file);
	QList<REGISTER_DATA> mRegisterList;
private:
	QSettings *settingsread = nullptr;
};

