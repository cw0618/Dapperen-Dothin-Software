#include "savecsv.h"

Savecsv::Savecsv(QObject *parent)
	: QObject(parent)
{
	csv_Folder = QString("CSV");
	dir.mkdir(csv_Folder);
}

Savecsv::~Savecsv()
{
}
QString Savecsv::getCurrentDate()
{
	//return QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
	return QDateTime::currentDateTime().toString("yyyy-MM-dd");
}
bool Savecsv::WriteCSV(QString path) {
	QString qcsv_name = QString("%1/%2-%3.csv").arg(csv_Folder).arg("data").arg(getCurrentDate());
	QFileInfo csv_file(qcsv_name);
	QByteArray ba = qcsv_name.toLocal8Bit();
	const char* csv_name = ba.data();
	if (csv_file.isFile())
	{
		QFile qf(csv_name);
		while (true)
		{
			if (!qf.open(QFile::ReadWrite))
			{
				QString msg = QString("%1\n Is opened, please close it!").arg(qcsv_name);
				qDebug() << msg;
				return false;
			}
			else
			{
				break;
			}
		}
	}

	if (csv_file.isFile())
	{
		QFile qf(csv_name);
		if (!qf.open(QFile::ReadWrite))
		{
			QString msg = QString("%1\n Is opened, please close it!").arg(qcsv_name);
			qDebug() << msg;
			return false;
		}
		csv_file_.open(csv_name, std::ios::out);
		QString time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
		string filepath = GBK::FromUnicode(path);
		string file_time = GBK::FromUnicode(time);
		csv_file_ << filepath.c_str() << ","
			<< file_time.c_str()
			<< endl;
	}
	else
	{
		csv_file_.open(csv_name, std::ios::out);
		csv_file_ << "Picture path," << "time" << endl;
		QString time=QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
		string filepath=GBK::FromUnicode(path);
		string file_time= GBK::FromUnicode(time);
		csv_file_ << filepath.c_str() << ","
			<< file_time.c_str()
			<< endl;
	}
	csv_file_.close();
	return true;
}