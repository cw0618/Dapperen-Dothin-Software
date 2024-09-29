#include "streamtestdialog.h"
#include "ui_streamtestdialog.h"
/** \class StreamTestDialog
*
* 数据流压测类
*
*/
StreamTestDialog::StreamTestDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::StreamTestDialog)
{
	ui->setupUi(this);
	ui->switch_stream_test->setText("Stop", "Start");
	//connect(&mTimeThreadOut, SIGNAL(onStepFinish(bool)), this, SLOT(onOneTimeFinish(bool)));
	connect(ui->switch_stream_test, SIGNAL(checkedChanged(bool)), this, SLOT(onTestSwitch(bool)));
}

StreamTestDialog::~StreamTestDialog()
{
	delete ui;
}
void StreamTestDialog::startThreadTime(int time, bool close) {
	if (mShowing)
	{

		mTimeThreadOut.startTimerThread(time);
	}

}
void StreamTestDialog::stopThreadTime() {

}
void StreamTestDialog::onOneTimeFinish(bool finish) {
	stopThreadTime();
	bool stream_status = ui->switch_stream_test->getChecked();
	if (mShowing && stream_status)
	{
		emit StreamTestCallback(kTimerChange, "1");
	}
	else
	{
		mTaskCount = 0;
	}

}
void StreamTestDialog::onTestSwitch(bool state) {
	if (state)
	{
		mShowing = true;
		//开始压测
		emit StreamTestCallback(kSwitchChange, "1");
	}
	else
	{
		//关闭压测
		emit StreamTestCallback(kSwitchChange, "0");
		mShowing = false;
	}
}
void StreamTestDialog::closeEvent(QCloseEvent *event) {
	//
	mShowing = false;
	mTaskCount = 0;
	ui->switch_stream_test->setChecked(false);
	//timeThreadOut.stop_timer();
	qDebug() << "howard ==StreamTestDialog=closeEvent";
}
void StreamTestDialog::setTaskTotalCount(int count) {
	mTaskTotalTount = count;
	ui->label_task_total_count->setText(QString("%1").arg(count));
}
void StreamTestDialog::setTaskCurrentCount(int count) {
	ui->label_task_count->setText(QString("%1").arg(count));
	mTaskCount = 0;
}
int StreamTestDialog::getTaskCount() {
	return mTaskCount;
}
void StreamTestDialog::ShowInfo(QString info)
{
	std::lock_guard<std::mutex> lg(mMutexLock);
	ui->stream_text_browser->insertPlainText(QString("%1 :%2%3").arg(mLogIndex++).arg(info).arg("\n"));
	ui->stream_text_browser->moveCursor(QTextCursor::End);
}
void StreamTestDialog::clearLogReleased()
{
	clearLogMethod();
}
void StreamTestDialog::clearLogMethod() {
	std::lock_guard<std::mutex> lg(mMutexLock);
	ui->stream_text_browser->clear();
	mLogIndex = 0;
}
void StreamTestDialog::finishOneTask() {
	if (mShowing)
	{

		ui->label_task_count->setText(QString("%1").arg(mTaskCount + 1));
		mTaskCount++;
	}

}
void StreamTestDialog::setShowStatus(bool status) {
	mShowing = status;
}
void StreamTestDialog::setButtonStatus(bool status) {
	ui->switch_stream_test->setChecked(false);
}
void StreamTestDialog::clearErrorCount() {
	mErrorAllCount = 0;
	mErrorColorCount = 0;
	mErrorDepthCount = 0;
	mErrorPhaseCount = 0;
	mErrorIrCount = 0;
	mErrorUvcCount = 0;
	ui->label_errod_count_->setText("0");
	ui->label_color_error_count->setText("0");
	ui->label_depth_error_count->setText("0");
	ui->label_phase_error_count->setText("0");
	ui->label_ir_error_count->setText("0");
	ui->label_uvc_error_count->setText("0");
}
void StreamTestDialog::addErrorStreamCount(int type) {
	mErrorAllCount++;
	ui->label_errod_count_->setText(QString("%1").arg(mErrorAllCount));
	if (type == kColorError)
	{
		mErrorColorCount++;
		ui->label_color_error_count->setText(QString("%1").arg(mErrorColorCount));
	}
	else if (type == kDepthError)
	{
		mErrorDepthCount++;
		ui->label_depth_error_count->setText(QString("%1").arg(mErrorDepthCount));
	}
	else if (type == kPhaseError)
	{
		mErrorPhaseCount++;
		ui->label_phase_error_count->setText(QString("%1").arg(mErrorPhaseCount));
	}
	else if (type == kIrError)
	{
		mErrorIrCount++;
		ui->label_ir_error_count->setText(QString("%1").arg(mErrorIrCount));
	}
	else if (type == kUvcError)
	{
		mErrorUvcCount++;
		ui->label_uvc_error_count->setText(QString("%1").arg(mErrorUvcCount));
	}
}
int StreamTestDialog::getErrorCount() {
	return mErrorAllCount;
}