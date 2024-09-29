#include"mainwindowstyle.h"
#include <QApplication>
#include"src/weight/videowindow.h"
#include <QDir>
#include <QTextStream>
#include <QSharedMemory>
#include <QMessageBox>
#include <QMutex>
#include <QDateTime>
#pragma execution_character_set("utf-8")

void outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	static QMutex mutex;
	mutex.lock();

	QString msgLevel;
	switch (type)
	{
	case QtDebugMsg:
		msgLevel = QString("Debug");
		break;

	case QtWarningMsg:
		msgLevel = QString("Warning");
		break;

	case QtCriticalMsg:
		msgLevel = QString("Critical");
		break;

	case QtFatalMsg:
		msgLevel = QString("Fatal");
	}
	QString cpp_file = QString(context.file).mid(QString(context.file).lastIndexOf("\\") + 1);
	int index1 = QString(context.function).indexOf("::") + 2;
	int count = QString(context.function).indexOf("(") - index1;
	QString func_name = QString(context.function).mid(index1, count);

	QString context_info = QString("[%1 %2(%3)]:").arg(cpp_file).arg(func_name).arg(context.line);
	QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz");
	QString curr_date = QDateTime::currentDateTime().toString("yyyy-MM-dd");
	QString current_date = QString("(%1)").arg(current_date_time);

	QString message = QString("%1[%2]%3 %4").arg(current_date).arg(msgLevel).arg(context_info).arg(msg);

	fprintf(stdout, "%s\n", message.toLocal8Bit().constData());
	fflush(stdout);

	QString log_path = ".\\OrbbecSensor_Log\\";
	QDir dir(log_path);
	if (!dir.exists()) {
		dir.mkpath(".\\");
	}

	QFile file(log_path + curr_date + ".log");
	file.open(QIODevice::WriteOnly | QIODevice::Append);
	QTextStream text_stream(&file);
	text_stream << message << "\r\n";
	file.flush();
	file.close();

	mutex.unlock();
}
int main(int argc, char *argv[])
{
	qInstallMessageHandler(outputMessage);
    QApplication app(argc, argv);
    //娉ㄥ唽MessageHandler
   
//    MainWindow w;
    MainWindowStyle mainWindow;
//    CameraControlView w;
	app.installNativeEventFilter(&mainWindow);
	mainWindow.show();
	QSharedMemory shared("OrbbecSensor");//璁剧疆鍙兘杩愯涓€涓疄渚?
	if (shared.attach())
	{
		QMessageBox::warning(NULL, "warning", "Only one tool instance can be opened!");
		//system("taskkill /f /im OBKit.exe");
		return 0;
	}
	shared.create(1);
    return app.exec();
}
