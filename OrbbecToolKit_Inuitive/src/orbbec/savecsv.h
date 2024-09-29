#ifndef SAVECSV_H
#define SAVECSV_H

#include <QObject>
#include <iostream>
#include <fstream>
#include <string>
#include <QDir>
#include <QString>
#include <QFileInfo>
#include <QDebug>
#include <QDate>
#include"src/config/GBK.h"

class Savecsv : public QObject
{
	Q_OBJECT

public:
	Savecsv(QObject *parent);
	~Savecsv();
	bool WriteCSV(QString path);
	QString getCurrentDate();
private:
	std::ofstream csv_file_;

	QDir dir;
	QString csv_Folder;
};
#endif // SAVECSV_H