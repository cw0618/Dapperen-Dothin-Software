#include "mainwindowstyle.h"
#include "ui_mainwindowstyle.h"
#include <QInputDialog>
#include <QRadioButton>
#include <QMessageBox>
#include <QDebug>
#include"src/weight/videowindow.h"
#include <Dbt.h>
#include"src/device/obutils.h"
#pragma execution_character_set("utf-8")//解决中文乱码

/** \class MainWindowStyle
*
* 主页面逻辑控制类
*
*/
static const GUID GUID_DEVINTERFACE_LIST[] =
{
	// GUID_DEVINTERFACE_USB_DEVICE
	{ 0xA5DCBF10, 0x6530, 0x11D2,{ 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED } },
};
MainWindowStyle::MainWindowStyle(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindowStyle)
{
	setAcceptDrops(true);//设置拖拽访问
	ui->setupUi(this);
	setWindowTitle(QString("iToFViewer ") + QString(TOOL_VERSION));
	initWindow();
	initAction();
	registerUSBNotification();
}

MainWindowStyle::~MainWindowStyle()
{
	delete ui;
}

void MainWindowStyle::initWindow() {
	QWidget *device_status_page = ui->sw_video_window->widget(kConnectingPage);
	mDeviceStatusWidget = new DeviceStatusWidget();
	QVBoxLayout *device_status_layout = new QVBoxLayout(device_status_page);
	device_status_layout->addWidget(mDeviceStatusWidget);
	device_status_page->setLayout(device_status_layout);
	ui->cw_common_wiget->setTitle("Common Setting");
	ui->cw_common_wiget->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_common_wiget->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_depth_senser->setTitle("Depth sensor");
	ui->cw_depth_senser->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_depth_senser->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_left_ir->setTitle("IR senser");
	ui->cw_left_ir->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_left_ir->setStyleSheet("background-color: rgb(36, 44, 51)");
	//ui->cw_left_ir->hide();
	ui->cw_phase->setTitle("Phase sensor");
	ui->cw_phase->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_phase->setStyleSheet("background-color: rgb(36, 44, 51)");
	//ui->cw_phase->hide();
	ui->cw_rgb_camera->setTitle("Color sensor");
	ui->cw_rgb_camera->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_rgb_camera->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_ai_stream->setTitle("AI Stream");
	ui->cw_ai_stream->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_ai_stream->setStyleSheet("background-color: rgb(36, 44, 51)");

	ui->cw_uvc_content->setTitle("UVC Devices");
	ui->cw_uvc_content->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_uvc_content->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_uvc_content->showSwitch(false);
	ui->cw_uvc_content->setContentEnable(false);
	ui->cw_uvc_content->hide();
	ui->cw_rgb_camera_uvc->setTitle("Color sensor(UVC)");
	ui->cw_rgb_camera_uvc->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_rgb_camera_uvc->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_rgb_camera_uvc->hide();

	ui->cw_point_could->setTitle("Point Cloud");
	ui->cw_point_could->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_point_could->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_point_could->setVisible(false);

	ui->cw_filter->setTitle("application scene");
	ui->cw_filter->setAttribute(Qt::WA_StyledBackground, true);
	ui->cw_filter->setStyleSheet("background-color: rgb(36, 44, 51)");
	ui->cw_filter->setSwitchVisible(false);
	ui->cw_filter->setVisible(false);

	mVideoContainer = ui->sw_video_window->widget(kVideoPage);
	mVideoLayout = new QVBoxLayout(mVideoContainer);
	mVideoLayout->setSpacing(0);
	mVideoOneLayout = new QHBoxLayout();
	mVideoOneLayout->setSpacing(5);
	VideoTwoLayout = new QHBoxLayout();
	VideoTwoLayout->setSpacing(5);
	mVideoLayout->addLayout(mVideoOneLayout);
	mVideoLayout->addLayout(VideoTwoLayout);
	mVideoContainer->setLayout(mVideoLayout);
	//默认设置的分辨率
	QString runPath = QCoreApplication::applicationDirPath();
	qDebug() << "Run path : " << runPath;
	
	QString config_ini = QString::fromStdString(ORBBEC_CONFIG_INI);
	int iret = mInitConfig.LoadIni(config_ini);

	if (mInitConfig.config_res_.mHidePhase)
	{
		ui->cw_phase->hide();
	}
	if (mInitConfig.config_res_.mHideDepth)
	{
		ui->cw_depth_senser->hide();
	}
	if (mInitConfig.config_res_.mHideIr)
	{
		ui->cw_left_ir->hide();
	}
	if (mInitConfig.config_res_.mHideColor)
	{
		ui->cw_rgb_camera->hide();
	}
	if (mInitConfig.config_res_.mHideAiStream)
	{
		ui->cw_ai_stream->hide();
	}
	mCalc = Calc::Instance();
	mDepthDrawMode = DepthDrawModel::RAINBOW;
	mDeviceSensorBase = new SensorBase();
	mDeviceSensorBase->SetConnectStatusCallback(std::bind(&MainWindowStyle::SwithOnLineStatus, this, std::placeholders::_1));
	mDeviceSensorBase->SetIrCallback(std::bind(&MainWindowStyle::CbSnapIR, this, std::placeholders::_1));
	mDeviceSensorBase->SetPhaseCallback(std::bind(&MainWindowStyle::CbSnapPhase, this, std::placeholders::_1));
	mDeviceSensorBase->SetDepthCallback(std::bind(&MainWindowStyle::CbSnapDepth, this, std::placeholders::_1));
	mDeviceSensorBase->SetColorCallback(std::bind(&MainWindowStyle::CbSnapColor, this, std::placeholders::_1));
	mDeviceSensorBase->SetOniCallback(std::bind(&MainWindowStyle::CbSnapOni, this, std::placeholders::_1));
	/// 切换线程用
	connect(this, SIGNAL(SigUpdateIr(OniData*, int, int)), this, SLOT(CallbackUpdateIr(OniData*, int, int)));           /// 左IR
	connect(this, SIGNAL(SigUpdatePhase(OniData*, int, int)), this, SLOT(CallbackUpdatePhase(OniData*, int, int)));     /// 右IR
	connect(this, SIGNAL(SigDrawDepth(OniData*)), this, SLOT(DrawDepthVideo(OniData*)));                /// Depth
	connect(this, SIGNAL(SigUpdateColor(OniData*)), this, SLOT(CallbackUpdateColorData(OniData*)));     /// Color
	connect(this, SIGNAL(SigUpdateOni(OniData*)), this, SLOT(CallbackUpdateOniData(OniData*)));
	connect(this, SIGNAL(SigConnecte(bool)), this, SLOT(CallbackConnectStatus(bool)));                  /// 连接状态切换
	//connect(&mPlayOniVideo, SIGNAL(frameUpdate(char*, int, int, int, int)), this, SLOT(CallbackUpdateOniData(char*, int, int, int, int)), Qt::DirectConnection);
	mCaptureDialog = new CaptureDialog(mDeviceSensorBase, this);
	mTuningWindow = new tuningwindow(mDeviceSensorBase, this);

	mPngWindowQueue.clear();
	mPngWindowList.clear();
	for (int i = 0; i < 20; i++) {
		mPngWindowQueue.push_front(NULL);
	}
	mBoxCommon = new QVBoxLayout;
	mCommonControl = new CommonControlWidget();
	mBoxCommon->addWidget(mCommonControl);
	ui->cw_common_wiget->addLayout(mBoxCommon);
	ui->cw_common_wiget->showSwitch(false);
	ui->cw_common_wiget->OnOnlyShowContent(true);
	mBoxColor = new QVBoxLayout;
	mColorControl = new ColorControlView();
	connect(mColorControl, SIGNAL(rgbChangeCallback(int, QString)), this, SLOT(onRgbResolutionCallback(int, QString)));
	mBoxColor->addWidget(mColorControl);
	ui->cw_rgb_camera->addLayout(mBoxColor);

	mBoxPhase = new QVBoxLayout;
	mPhaseControl = new PhaseControlView();
	mBoxPhase->addWidget(mPhaseControl);
	ui->cw_phase->addLayout(mBoxPhase);

	mBoxDepth = new QVBoxLayout;
	mDepthControl = new DepthControlView();
	connect(mDepthControl, SIGNAL(depthChangeCallback(int, QString)), this, SLOT(onDepthControlCallback(int, QString)));
	connect(mDepthControl, SIGNAL(sliderChange(int)), this, SLOT(onSliderChange(int)));
	connect(mDepthControl, SIGNAL(filterNameChange(QString)), mTuningWindow, SLOT(onFilterNameChange(QString)));
	connect(mTuningWindow, SIGNAL(SignalsCloseStream(bool)), this, SLOT(settingCloseStream(bool)));
	connect(mDepthControl, SIGNAL(changeFilterParams(double*)), this, SLOT(onChangeFilterParams(double*)));
	mBoxDepth->addWidget(mDepthControl);
	ui->cw_depth_senser->addLayout(mBoxDepth);
	if (!mInitConfig.config_res_.mFilterSetOnUI) {
		mDepthControl->hidePointFilterSet();
	}

	mBoxIr = new QVBoxLayout;
	mIrControl = new IRControlView();
	connect(mIrControl, SIGNAL(IRChangeCallback(int, QString)), this, SLOT(onLeftIrControlCallback(int, QString)));
	mBoxIr->addWidget(mIrControl);
	ui->cw_left_ir->addLayout(mBoxIr);

}
void MainWindowStyle::initAction() {
	mStreamTestDialog = new StreamTestDialog();
	connect(mStreamTestDialog, SIGNAL(StreamTestCallback(int, QString)), this, SLOT(onStreamTestCallback(int, QString)));
	connect(ui->cw_depth_senser, SIGNAL(clickViewCallback(int, int)), this, SLOT(onDepthCameraSwitch(int, int)));
	connect(ui->cw_left_ir, SIGNAL(clickViewCallback(int, int)), this, SLOT(onIrCameraSwitch(int, int)));
	connect(ui->cw_phase, SIGNAL(clickViewCallback(int, int)), this, SLOT(onPhaseCameraSwitch(int, int)));
	connect(ui->cw_rgb_camera, SIGNAL(clickViewCallback(int, int)), this, SLOT(onRGBCameraSwitch(int, int)));

	connect(ui->cw_ai_stream, SIGNAL(clickViewCallback(int, int)), this, SLOT(onAIStreamSwitch(int, int)));
	connect(&mUvcRgbCamera, SIGNAL(colorResolutionCallback(int, QString)), this, SLOT(onUVCColorChang(int, QString)));
	connect(ui->cw_rgb_camera_uvc, SIGNAL(clickViewCallback(int, int)), this, SLOT(onUVCRGBSwitch(int, int)));

	connect(ui->button_2d_window, SIGNAL(clicked()), this, SLOT(on2DWindowClick()));
	connect(ui->button_3d_window, SIGNAL(clicked()), this, SLOT(on3DWindowClick()));
	connect(ui->button_capture, SIGNAL(clicked()), this, SLOT(onCaptureViewClick()));
	connect(ui->button_tuning_window, SIGNAL(clicked()), this, SLOT(onsettingButtionClick()));
	connect(ui->tb_device_close, SIGNAL(clicked()), this, SLOT(onDeviceCloseClick()));
	connect(ui->tb_info, SIGNAL(clicked()), this, SLOT(onButtonInfoClick()));
	connect(ui->tb_more, SIGNAL(clicked()), this, SLOT(onButtonMoreClick()));
	connect(ui->button_add_device, SIGNAL(clicked()), this, SLOT(onButtonAddDeviceClick()));
	connect(&mOpenDeviceThread, SIGNAL(openDeviceStatus(int)), this, SLOT(DeviceOpenStatus(int)));
	connect(mCommonControl, SIGNAL(commonWidgetChangeCallback(int, QString)), this, SLOT(onCommonWidgetCallback(int, QString)));
	initStart();
	this->startTimer(1000);     ///  启动定时器， 1秒调用一次TimerEvent
}
void MainWindowStyle::initStart() {
	mDeviceConnecting = true;

	int index = ui->sw_video_window->currentIndex();
	if (kConnectingPage != index) {
		ui->sw_video_window->setCurrentIndex(kConnectingPage);
	}
	mOpenDeviceThread.CreateThread(mDeviceSensorBase);
}
void MainWindowStyle::DeviceOpenStatus(int status) {
	mDeviceConnecting = false;
	qDebug() << "howard DeviceOpenStatus===" << status;
	if (status == mOpenDeviceThread.kDeviceOpenSuccess)
	{
		//设备开启成功
		QString str;
		str.append("color: rgb(193, 211, 227);").append("font: 18pt ""Agency FB"";");
		ui->label_device_name->setStyleSheet(str);
		ui->label_device_name->setText("Camera device");
		mDeviceStatusWidget->setDeviceStatus("Connect Device Success", 0);
		firstStart();
	}
	else if (status == mOpenDeviceThread.kDeviceOpenFailure){

		//设备开启失败
		mDeviceStatusWidget->setDeviceStatus("Connect Device Failure", -1);
	}
	else if (status == mOpenDeviceThread.kLoadRegisterSuccess)
	{
		ui->cw_phase->setSwitchVisible(true);
	}
}

void MainWindowStyle::onSliderChange(int value)
{

	mRangeOfDepth = value;
}
void MainWindowStyle::onChangeFilterParams(double* value)
{
	qDebug() << "onChangeFilterParams:" << value[0] << value[1] << value[2];
	mPointCloudFilterParams[0] = value[0];
	mPointCloudFilterParams[1] = value[1];
	mPointCloudFilterParams[2] = value[2];
	if (value[2] <= 0) { //byPass = 0
		mPointCloudFilterBuff = true;
	}
	else {
		mPointCloudFilterBuff = false;
	}

}
void MainWindowStyle::onPngInfoWindowClose(void * handle)
{
	mPngWindowQueue.enqueue((PngInfoWindow*)handle);
}
void MainWindowStyle::dragEnterEvent(QDragEnterEvent * event)
{
	// 判断拖拽文件类型，文件名 接收该动作
	QString filePath = event->mimeData()->urls()[0].fileName();
	if ((filePath.right(3).compare("png") == 0) || (filePath.right(3).compare("oni") == 0)) {
		event->acceptProposedAction();
	}
	else {
		event->ignore();
	}
}
void MainWindowStyle::dropEvent(QDropEvent *event)
{
	QList<QUrl> urls = event->mimeData()->urls();
	if (urls.isEmpty())
		return;
	//往文本框中追加文件名
	QStringList fileNames;
	foreach(QUrl url, urls) {
		QString fileStr = url.toLocalFile();
		fileNames << fileStr;

	}
	if ((fileNames.size() > 0) && (fileNames[0].right(3).compare("oni") == 0))
	{
	}
	else
	{
		showPngWindow(fileNames);
	}
}

/**
* 显示png图片
* \param fileNames
*  文件路径
*
*/
void MainWindowStyle::showPngWindow(QStringList& fileNames)
{
	if (fileNames.size() > 0) {

		if (mPngWindowQueue.size() <= 0) {
			QMessageBox::information(this, tr("Warning"), QString(tr("max count: %1")).arg(20), QMessageBox::Close);
			return;
		}

		QString errPng = "";
		for (int i = 0; i < fileNames.size(); i++) {
			if (mPngWindowQueue.size() <= 0) {
				break;
			}

			QString fileStr = fileNames[i];
			QByteArray ba = fileStr.toLocal8Bit();
			char* cStr = ba.data();
			PngInfoWindow* pngInfoWindow = mPngWindowQueue.dequeue();
			if (pngInfoWindow == NULL) {
				pngInfoWindow = new PngInfoWindow();
				connect(pngInfoWindow, SIGNAL(pngInfoWindowClose(void*)), this, SLOT(onPngInfoWindowClose(void*)));
				mPngWindowList.push_back(pngInfoWindow);
			}

			pngInfoWindow->setPngFile(fileNames[i]);
			pngInfoWindow->show();
		}
		//if (!errPng.isEmpty()) {
		//	errPng = QString(tr("not orbbec png:\n")) + errPng;
		//	QMessageBox::information(this, tr("Warning"), errPng, QMessageBox::Close);
		//}
	}
}

void MainWindowStyle::depthControlLayout() {

	// 深度数据支持的分辨率
	mDthSpportMde.clear();
	mDeviceSensorBase->QuerySupportedModes(mDthSpportMde, openni::SENSOR_DEPTH);
	//设置默认分辨率
	mDepthControl->setResolutionChange(false);
	mDepthControl->resolutionClear();
	mDepthControl->GetFilterFileName();

	for (auto &it : mDthSpportMde)
	{
		QString str_stream = QString::fromStdString(it);
		mDepthControl->addResolution(str_stream);
		qDebug() << "SENSOR_DEPTH str_stream===" << str_stream;
	}
	mDepthControl->setResolutionChange(true);

	int resolution_index = mDepthControl->getCurrentResolution();
	std::string cur_resolution = mDthSpportMde.at(resolution_index);
	mDeviceSensorBase->SwitchStreamMode(openni::SENSOR_DEPTH, cur_resolution);
}
void MainWindowStyle::IrControlLayout() {

	// ir 数据支持的分辨率
	mIrSupportMode.clear();
	mDeviceSensorBase->QuerySupportedModes(mIrSupportMode, openni::SENSOR_IR);
	//设置默认分辨率
	mIrControl->setResolutionChange(false);
	mIrControl->resolutionClear();

	for (auto &it : mIrSupportMode)
	{
		QString str_stream = QString::fromStdString(it);
		mIrControl->addResolution(str_stream);
		qDebug() << "SENSOR_IR str_stream===" << str_stream;
	}
	mIrControl->setResolutionChange(true);

	int resolution_index = mIrControl->getCurrentResolution();
	std::string cur_resolution = mIrSupportMode.at(resolution_index);
	mDeviceSensorBase->SwitchStreamMode(openni::SENSOR_IR, cur_resolution);
}
void MainWindowStyle::aiControlLayout() {

	if (!mAiStreamControl.mWidgetExist)
	{
		mAiStreamControl.mWidgetExist = true;
		connect(&mAiStreamControl, SIGNAL(aiChangeCallback(int, QString)), this, SLOT(onAiControlCallback(int, QString)));
		mBoxAiAtream.addWidget(&mAiStreamControl);
		ui->cw_ai_stream->addLayout(&mBoxAiAtream);
	}

	//phase数据支持的分辨率
	mAiSupportMode.clear();
	mDeviceSensorBase->QuerySupportedModes(mAiSupportMode, openni::SENSOR_AI);
	//设置默认分辨率
	mAiStreamControl.setAIStatueChange(false);
	mAiStreamControl.AITypeClear();

	for (int i = 0; i < mAiSupportMode.size(); i++)
	{
		int type = QString::fromStdString(mAiSupportMode.at(i)).toInt();
		QString str_stream = "";
		switch (type)
		{
		case PIXEL_FORMAT_JOINT_2D:
			str_stream = "2D SKELETON";
			break;
		case PIXEL_FORMAT_JOINT_3D:
			str_stream = "3D SKELETON";
			break;
		case PIXEL_FORMAT_BODY_MASK:
			str_stream = "BODY MASK";
			break;
		case PIXEL_FORMAT_FLOOR_INFO:
			str_stream = "FLOOR INFO";
			break;
		case PIXEL_FORMAT_BODY_SHAPE:
			str_stream = "BODY SHAPE";
			break;
		case PIXEL_FORMAT_FACE:
			str_stream = "FACE";
			break;
		case PIXEL_FORMAT_GESTURE:
			str_stream = "GESTURE";
			break;
		default:
			break;
		}
		mAiStreamControl.addAIType(str_stream);

		qDebug() << "SENSOR_AI str_stream===" << str_stream;
	}
	mAiStreamControl.setAIStatueChange(true);
	//创建流
	//mDeviceSensorBase->SwitchStreamMode(openni::SENSOR_AI, "");
}
void MainWindowStyle::phaseControlLayout() {
	//phase数据支持的分辨率
	mPhaseSupportMode.clear();
	mDeviceSensorBase->QuerySupportedModes(mPhaseSupportMode, openni::SENSOR_PHASE);
	//设置默认分辨率
	mPhaseControl->setResolutionChange(false);
	mPhaseControl->resolutionClear();
	for (auto &it : mPhaseSupportMode)
	{
		QString str_stream = QString::fromStdString(it);
		mPhaseControl->addResolution(str_stream);
		qDebug() << "SENSOR_PHASE str_stream===" << str_stream;
	}
	mPhaseControl->setResolutionChange(true);
	//创建流
	int resolution_index = mPhaseControl->getCurrentResolution();
	std::string cur_resolution = mPhaseSupportMode.at(resolution_index);
	int ret = mDeviceSensorBase->SwitchStreamMode(openni::SENSOR_PHASE, cur_resolution);
}
void MainWindowStyle::rgbControlLayout() {

	// color 数据支持的分辨率
	mColorSupportMode.clear();
	mColorControl->setResolutionChange(false);
	mColorControl->resolutionClear();

	mColorControl->setD2CStatus(false);

	mDeviceSensorBase->QuerySupportedModes(mColorSupportMode, openni::SENSOR_COLOR);

	for (auto &it : mColorSupportMode)
	{
		QString str_stream = QString::fromStdString(it);
		if (str_stream.contains("2560") || str_stream.contains("3840")) {
			continue;
		}
		mColorControl->addResolution(str_stream);
		qDebug() << "SENSOR_COLOR str_stream===" << str_stream;
	}
	openni::VideoMode mode = mDeviceSensorBase->getStreamResolution(openni::SENSOR_COLOR);
	QString rgb_resolution = QString("%1x%2").arg(mode.getResolutionX()).arg(mode.getResolutionX());
	qDebug() << "current rgb resolution=" << rgb_resolution;
	mColorControl->setCurrentResolution("1280x720");
	mColorControl->setResolutionChange(true);
}
void MainWindowStyle::on2DWindowClick() {
	int index = ui->sw_video_window->currentIndex();
	if (kVideoPage != index) {
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mColorOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		else
		{
			ui->sw_video_window->setCurrentIndex(kVideoPage);
		}
	}
	QString str_click;
	QString str;
	str_click.append("color: rgb(3, 172, 235);").append("background-color: rgb(62, 77, 89);").append("font: 12pt \"Agency FB\";");
	str.append("color: rgb(193, 211, 227);").append("background-color: rgb(62, 77, 89);").append("font: 12pt \"Agency FB\";");
	ui->button_2d_window->setStyleSheet(str_click);
	ui->button_3d_window->setStyleSheet(str);
}
void MainWindowStyle::onCaptureViewClick() {
	if (!mCaptureDialog->isVisible()) {
		mCaptureDialog->show();
	}
	else {
		mCaptureDialog->activateWindow();
	}
	mCaptureDialog->startTimerCount();
	//capture_dialog_->GetStartStatus();
}
void MainWindowStyle::onButtonInfoClick() {

	AboutDialog *about_dialog = new AboutDialog(mDeviceSensorBase, this);
	about_dialog->show();

}
void MainWindowStyle::onButtonMoreClick() {
	MoreDialog *about_dialog = new MoreDialog(this);
	about_dialog->show();
}
void MainWindowStyle::onButtonAddDeviceClick() {
	//qDebug() << "howard==onButtonAddDeviceClick";
	if (nullptr == mDeviceSensorBase || mDeviceSensorBase->IsOpened() == false)
	{
		CallbackConnectStatus(false);
	}
}
void MainWindowStyle::onDeviceCloseClick() {
	QString str;
	str.append("color: rgb(255, 0, 0);").append("font: 12pt ""Agency FB"";");
	ui->label_device_name->setStyleSheet(str);
	ui->label_device_name->setText("No device");
	if (nullptr != mDeviceSensorBase)
	{
		//mDeviceSensorBase->CloseDevice();
	}
	if (0 != mIrOpen)
	{
		mDeviceSensorBase->StopStream(openni::SENSOR_IR);
		if (mViewLocation.contains(kIrLocationKey)) {
			int left_if_location_value = mViewLocation.value(kIrLocationKey);
			if (10 == left_if_location_value && nullptr != mIRWindow) {
				//从容器中删除
				mIRWindow->hide();
				mVideoOneLayout->removeWidget(mIRWindow);
				delete mIRWindow;
				mIRWindow = nullptr;
			}
			else if (20 == left_if_location_value && nullptr != mIRWindow) {
				mIRWindow->hide();
				VideoTwoLayout->removeWidget(mIRWindow);
				delete mIRWindow;
				mIRWindow = nullptr;
			}
			mIrOpen = 0;
			ui->cw_left_ir->setCameraStatus(false);
		}
	}

	if (0 != mColorOpen) {
		mDeviceSensorBase->StopStream(openni::SENSOR_COLOR);
		if (mViewLocation.contains(kRgbLocationKey)) {
			int rgb_location_value = mViewLocation.value(kRgbLocationKey);
			if (10 == rgb_location_value && nullptr != mRgbWindow) {
				//从容器中删除
				mRgbWindow->hide();
				mVideoOneLayout->removeWidget(mRgbWindow);
				delete mRgbWindow;
				mRgbWindow = nullptr;
			}
			else if (20 == rgb_location_value && nullptr != mRgbWindow) {
				mRgbWindow->hide();
				VideoTwoLayout->removeWidget(mRgbWindow);
				delete mRgbWindow;
				mRgbWindow = nullptr;
			}
		}
		mColorOpen = 0;
		ui->cw_rgb_camera->setCameraStatus(false);
	}
	if (0 != mDepthOpen) {
		mDeviceSensorBase->StopStream(openni::SENSOR_DEPTH);
		if (mViewLocation.contains(kDepthLocationKey)) {
			int depth_location_value = mViewLocation.value(kDepthLocationKey);
			if (10 == depth_location_value && nullptr != mDepthWindow) {
				//从容器中删除
				mDepthWindow->hide();
				mVideoOneLayout->removeWidget(mDepthWindow);
				delete mDepthWindow;
				mDepthWindow = nullptr;
			}
			else if (20 == depth_location_value && nullptr != mDepthWindow) {
				mDepthWindow->hide();
				VideoTwoLayout->removeWidget(mDepthWindow);
				delete mDepthWindow;
				mDepthWindow = nullptr;
			}
		}
		mDepthOpen = 0;
		ui->cw_depth_senser->setCameraStatus(false);
	}
	ui->sw_video_window->setCurrentIndex(kAllClosePage);
}
void MainWindowStyle::onsettingButtionClick() {
	//bool ok;
	//QString password = QInputDialog::getText(this, tr("password"), tr("password:"), QLineEdit::Password, tr("test"), &ok);
	//if (!mPasswordShow)
	//{
	//	mPasswordShow = true;
	//	mInitConfig.setPasswordValue(99);
	//}
	if (nullptr != mTuningWindow)
	{
		mTuningWindow->InitializeStatus();
		mTuningWindow->show();
	}
}
void MainWindowStyle::on3DWindowClick() {
	int index = ui->sw_video_window->currentIndex();
	if (2 != index) {
		ui->sw_video_window->setCurrentIndex(kThreeDPage);
	}
	QString str_click;
	QString str;
	str_click.append("color: rgb(3, 172, 235);").append("background-color: rgb(62, 77, 89);").append("font: 12pt ""Agency FB"";");
	str.append("color: rgb(193, 211, 227);").append("background-color: rgb(62, 77, 89);").append("font: 12pt ""Agency FB"";");
	ui->button_3d_window->setStyleSheet(str_click);
	ui->button_2d_window->setStyleSheet(str);
}
int MainWindowStyle::onIrCameraSwitch(int type, int value) {
	int ret_status = 0;
	if (ui->cw_left_ir->kCameraSwitchType == type && KOpen == value) {
		//打开左IR
		if (mONIOpen == KOpen)
		{
			closeONIVideo();
			mOpenDeviceThread.CreateThread(mDeviceSensorBase);
		}
		ui->cw_left_ir->setSwitchEnable(false);
		if (true == mDeviceSensorBase->IsOpened()) {
			int resolution_index = mIrControl->getCurrentResolution();
			std::string cur_resolution = mIrSupportMode.at(resolution_index);
			mDeviceSensorBase->changeStreamResolution(openni::SENSOR_IR, cur_resolution);
			//mCommonControl->setLaserStatus(true);
		}
		else
		{
			closeIRVideo();
			CallbackConnectStatus(false);
			ret_status = -1;
			ui->cw_left_ir->setSwitchEnable(true);
			return ret_status;
		}
		int index = ui->sw_video_window->currentIndex();
		if (kVideoPage != index) {
			ui->sw_video_window->setCurrentIndex(kVideoPage);
		}
		if (nullptr != mIRWindow) {
			delete mIRWindow;
			mIRWindow = nullptr;
		}
		mIRWindow = new VideoWindow;
		mIRWindow->setStreamMode(mIRWindow->kStrIR);
		mIRWindow->setAttribute(Qt::WA_StyledBackground, true);
		mIRWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
		mIRWindow->setVideoTitle("IR sensor");
		connect(mIRWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onLeftIrVideoCallback(int, int)));
		int one_layout_count = mVideoOneLayout->count();
		//一个layout只能插入二个widget
		if (1 < one_layout_count) {
			VideoTwoLayout->addWidget(mIRWindow);
			mViewLocation.insert(kIrLocationKey, 20);
		}
		else
		{
			mVideoOneLayout->addWidget(mIRWindow);
			mViewLocation.insert(kIrLocationKey, 10);
		}
		mIrOpen = 1;
		ui->cw_left_ir->setSwitchEnable(true);
	}
	else if (ui->cw_left_ir->kCameraSwitchType == type && 0 == value) {
		//关闭左ir
		ui->cw_left_ir->setSwitchEnable(false);
		closeIRVideo();
		ui->cw_left_ir->setSwitchEnable(true);
	}
	return ret_status;
}
int MainWindowStyle::onPhaseCameraSwitch(int type, int value) {
	int ret_status = 0;
	if (ui->cw_phase->kCameraSwitchType == type && KOpen == value) {
		if (mONIOpen == KOpen)
		{
			closeONIVideo();
			mOpenDeviceThread.CreateThread(mDeviceSensorBase);
		}
		ui->cw_phase->setSwitchEnable(false);

		if (true == mDeviceSensorBase->IsOpened()) {
			int resolution_index = mPhaseControl->getCurrentResolution();
			std::string cur_resolution = mPhaseSupportMode.at(resolution_index);
			mDeviceSensorBase->StartStream(openni::SENSOR_PHASE);
		}
		else
		{
			ui->cw_phase->setSwitchEnable(true);
			CallbackConnectStatus(false);
			ret_status = -1;
			return ret_status;
		}

		int index = ui->sw_video_window->currentIndex();
		if (kVideoPage != index) {
			ui->sw_video_window->setCurrentIndex(kVideoPage);
		}
		if (nullptr != mPhaseWindow) {
			delete mPhaseWindow;
			mPhaseWindow = nullptr;
		}

		mPhaseWindow = new VideoWindow;
		mPhaseWindow->setStreamMode(mPhaseWindow->kStrPhase);
		//phase_window->setStreamMode(phase_window->STR_PHASE);
		mPhaseWindow->setAttribute(Qt::WA_StyledBackground, true);
		mPhaseWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
		mPhaseWindow->setVideoTitle("Phase Image");
		connect(mPhaseWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onPhaseVideoCallback(int, int)));
		mVideoOneLayout->addWidget(mPhaseWindow);
		mViewLocation.insert(kPhaseLocationKey, 10);

		//bool backgrounp = mDeviceSensorBase->getOutputMode();
		int backgrounp = mDeviceSensorBase->getOutputMode2();
		/*if (backgrounp == 5 || backgrounp == 1 || backgrounp == 2)
		{
			if (nullptr != mIRWindow) {
				delete mIRWindow;
				mIRWindow = nullptr;
			}
			mIRWindow = new VideoWindow;
			mIRWindow->setStreamMode(mIRWindow->kStrIR);
			mIRWindow->setAttribute(Qt::WA_StyledBackground, true);
			mIRWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
			mIRWindow->setVideoTitle("减背景");
			connect(mIRWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onLeftIrVideoCallback(int, int)));
			mVideoOneLayout->addWidget(mIRWindow);
			mViewLocation.insert(kIrLocationKey, 10);
		}*/
		mPhaseOpen = KOpen;
		ui->cw_phase->setSwitchEnable(true);
	}
	else if (ui->cw_phase->kCameraSwitchType == type && KClose == value) {
		//关闭phase
		ui->cw_phase->setSwitchEnable(false);
		closePhaseVideo(true);
		ret_status = -1;
		ui->cw_phase->setSwitchEnable(true);
	}
	return ret_status;
}

int MainWindowStyle::onUVCRGBSwitch(int type, int value) {
	int ret_status = 0;
	qDebug() << "howard uvc type=" << type << " value=" << value;
	if (true == mDeviceSensorBase->IsOpened()) {
		if (ui->cw_rgb_camera_uvc->kCameraSwitchType == type && KOpen == value) {
			//打开UVC RGB camera
			mColorHideOpen = false;
			ui->cw_rgb_camera_uvc->setSwitchEnable(false);
			if (!mWmfFunction.getDeviceStatus())
			{
				getUVCDevicesList();
			}
			if (mWmfFunction.getThreadStatus()) {
				mWmfFunction.stopThread();
				QThread::msleep(2000);
			}
			if (mColorOpen == KOpen) {
				closeRgbVideo(true);
			}
			int device_index = mUvcRgbCamera.getCurrentDevices();
			int resolution_index = mUvcRgbCamera.getCurrentResolution();
			Uvc_Device uvc_device = mUvcDevices.at(device_index);
			int resolutions_size = mUvcDevices.at(device_index).resolutions.size();
			Device_Resolution device_resolution = mUvcDevices.at(device_index).resolutions.at(resolution_index);
			HRESULT hr = mWmfFunction.setResolution(device_index, device_resolution.indexId,
				device_resolution.width, device_resolution.height, device_resolution.fps, device_resolution.format);
			if (SUCCEEDED(hr)) {
				ret_status = 0;
				qDebug() << "start preview";
				mWmfFunction.startThread();
				int index = ui->sw_video_window->currentIndex();
				if (kVideoPage != index) {
					ui->sw_video_window->setCurrentIndex(kVideoPage);
				}
				if (nullptr == mUvcRgbWindow) {
					mUvcRgbWindow = new VideoWindow;
				}
				if (mAiStreamOpen == KOpen)
				{
					mUvcRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
				}
				mUvcRgbWindow->setStreamMode(mUvcRgbWindow->kStrUvcColor);
				mUvcRgbWindow->setAttribute(Qt::WA_StyledBackground, true);
				mUvcRgbWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
				mUvcRgbWindow->setVideoTitle("uvc camera");
				connect(mUvcRgbWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onUVCRGBVideoCallback(int, int)));
				int one_layout_count = mVideoOneLayout->count();
				//一个layout只能插入二个widget
				if (1 < one_layout_count) {
					VideoTwoLayout->addWidget(mUvcRgbWindow);
					mViewLocation.insert(kRgbLocationKey, 20);
				}
				else
				{
					mVideoOneLayout->addWidget(mUvcRgbWindow);
					mViewLocation.insert(kRgbLocationKey, 10);
				}
				mDeviceSensorBase->mUvcColorStreamOpen = true;
				mUvcColorOpen = KOpen;
			}
			else {
				ret_status = -1;
			}
			ui->cw_rgb_camera_uvc->setSwitchEnable(true);
			if (mColorControl != nullptr && mDepthOpen == KOpen)
			{
				mColorControl->setD2CEnable(true);

			}
		}
		else if (ui->cw_rgb_camera_uvc->kCameraSwitchType == type && KClose == value) {
			//关闭UVC RGB camer
			ui->cw_rgb_camera_uvc->setSwitchEnable(false);
			closeUVCRgbVideo();
			ui->cw_rgb_camera_uvc->setSwitchEnable(true);
		}
	}
	return ret_status;
}
int MainWindowStyle::onAIStreamSwitch(int type, int value) {
	int ret_status = 0;
	if (ui->cw_ai_stream->kCameraSwitchType == type && KOpen == value) {
		//打开ai 流
		if (mPhaseOpen == KOpen)
		{
			ui->cw_phase->setSwitchEnable(false);
			closePhaseVideo(true);
			ui->cw_phase->setSwitchEnable(true);
		}
		ui->cw_ai_stream->setSwitchEnable(false);
		if (true == mDeviceSensorBase->IsOpened()) {
			if (mColorOpen == KClose && mUvcColorOpen == KClose) {
				ui->cw_ai_stream->setCameraStatus(false);
				ui->cw_ai_stream->setSwitchEnable(true);
				return -1;
			}
			else if (mUvcColorOpen == KOpen)
			{
				mUvcRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
			}
			else if (mColorOpen == KOpen)
			{
				mRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
			}
			//2D骨架
			int ai_type_index = mAiStreamControl.getCurrentAIType();
			std::string cur_resolution = mAiSupportMode.at(ai_type_index);
			int type = QString::fromStdString(cur_resolution).toInt();
			PixelFormat pixel_format = getPixelFormat(type);
			if (pixel_format == PixelFormat::PIXEL_FORMAT_BODY_SHAPE)
			{
				mRgbWindow->setAIBodyShape(true);
			}
			int index = ui->sw_video_window->currentIndex();
			if (kVideoPage != index) {
				ui->sw_video_window->setCurrentIndex(kVideoPage);
			}

			ret_status = mDeviceSensorBase->changeAIStreamType(pixel_format);
			if (ret_status == 0)
			{
				ret_status = mDeviceSensorBase->StartStream(openni::SENSOR_AI);
			}
			mAiStreamOpen = KOpen;

		}
		ui->cw_ai_stream->setSwitchEnable(true);
	}
	else {
		//关闭AI流
		if (true == mDeviceSensorBase->IsOpened()) {
			//关闭骨架
			ui->cw_ai_stream->setSwitchEnable(false);
			mDeviceSensorBase->StopStream(openni::SENSOR_AI);
			mAiStreamOpen = KClose;
			if (mRgbWindow != nullptr)
			{
				mDeviceSensorBase->mAiSekeleton2d.mJointFormat = PixelFormat::PIXEL_FORMAT_NONE;
				mRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
				mRgbWindow->setAIBodyShape(false);
				mRgbWindow->setAIRenderType(RenderType::kRenderNull);
			}
			ui->cw_ai_stream->setSwitchEnable(true);
			//if(rgb_camera_open==CLOSE){
			//    closeRgbVideo(false);
			//}
		}

	}
	return ret_status;
}
int MainWindowStyle::onRGBCameraSwitch(int type, int value) {
	int ret_status = 0;
	if (ui->cw_rgb_camera->kCameraSwitchType == type && KOpen == value) {
		//打开RGB camera

		ui->cw_rgb_camera->setSwitchEnable(false);
		if (mColorOpen == KClose) {
			if (true == mDeviceSensorBase->IsOpened()) {
				qDebug() << "Open color sensor";
				mColorHideOpen = false;
				if (mUvcColorOpen == KOpen)
				{
					closeUVCRgbVideo();
				}

				if (!mColorHideOpen)
				{
					//打开1280*720分辨率
					int resolution_index = mColorControl->getCurrentResolution();
					std::string cur_resolution = mColorSupportMode.at(resolution_index);
					ret_status = mDeviceSensorBase->changeStreamResolution(openni::SENSOR_COLOR, cur_resolution);
				}

			}
			else
			{
				ui->cw_rgb_camera->setSwitchEnable(true);
				CallbackConnectStatus(false);
				ret_status = -1;
				return ret_status;

			}
		}
		int index = ui->sw_video_window->currentIndex();
		if (kVideoPage != index) {
			ui->sw_video_window->setCurrentIndex(kVideoPage);
		}
		if (nullptr != mRgbWindow) {
			delete mRgbWindow;
			mRgbWindow = nullptr;
		}
		mRgbWindow = new VideoWindow;
		if (mAiStreamOpen == KOpen)
		{
			mRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
		}
		mRgbWindow->setStreamMode(mRgbWindow->kStrColor);
		mRgbWindow->setAttribute(Qt::WA_StyledBackground, true);
		mRgbWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
		mRgbWindow->setVideoTitle("Color Camera");
		connect(mRgbWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onRGBVideoCallback(int, int)));
		int one_layout_count = mVideoOneLayout->count();
		//一个layout只能插入二个widget
		if (1 < one_layout_count) {
			VideoTwoLayout->addWidget(mRgbWindow);
			mViewLocation.insert(kRgbLocationKey, 20);
		}
		else
		{
			mVideoOneLayout->addWidget(mRgbWindow);
			mViewLocation.insert(kRgbLocationKey, 10);
		}


		mColorOpen = KOpen;
		ui->cw_rgb_camera->setSwitchEnable(true);
		if (mColorControl != nullptr && mDepthOpen == KOpen)
		{
			mColorControl->setD2CEnable(true);

		}
	}
	else if (ui->cw_rgb_camera->kCameraSwitchType == type && 0 == value) {
		//关闭RGB流
		ui->cw_rgb_camera->setSwitchEnable(false);
		closeRgbVideo(true);
		ui->cw_rgb_camera->setSwitchEnable(true);

	}
	return ret_status;
}
int MainWindowStyle::onDepthCameraSwitch(int type, int value) {
	int ret_status = 0;
	if (ui->cw_depth_senser->kCameraSwitchType == type && 1 == value) {
		if (mONIOpen == KOpen)
		{
			closeONIVideo();
			mOpenDeviceThread.CreateThread(mDeviceSensorBase);
		}
		if (!mDeviceSensorBase->mLoadCaliterSuccess)
		{
			ui->cw_depth_senser->blockSignals(true);
			ui->cw_depth_senser->setCameraStatus(false);
			ui->cw_depth_senser->blockSignals(false);
			QMessageBox::warning(NULL, "warning", "请先加载滤波和标定文件!");
			return -1;
		}
		ui->cw_depth_senser->setSwitchEnable(false);
		if (true == mDeviceSensorBase->IsOpened()) {
			int resolution_index = mDepthControl->getCurrentResolution();
			std::string cur_resolution = mDthSpportMde.at(resolution_index);
			mDeviceSensorBase->changeStreamResolution(openni::SENSOR_DEPTH, cur_resolution);
			//mCommonControl->setLaserStatus(true);
			if (mDepthControl != nullptr)
			{
				mDepthControl->setDepthEnable(true);
			}
		}
		else
		{
			CallbackConnectStatus(false);
			ret_status = -1;
			ui->cw_depth_senser->setSwitchEnable(true);
			return ret_status;
		}
		int index = ui->sw_video_window->currentIndex();
		if (kVideoPage != index) {
			ui->sw_video_window->setCurrentIndex(kVideoPage);
		}
		if (nullptr != mDepthWindow) {
			delete mDepthWindow;
			mDepthWindow = nullptr;
		}
		mDepthWindow = new VideoWindow;
		mDepthWindow->setStreamMode(mDepthWindow->mStrDepth);
		mDepthWindow->setAttribute(Qt::WA_StyledBackground, true);
		mDepthWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
		mDepthWindow->setVideoTitle("Depth sensor");
		mDepthWindow->setDepthMode(DepthDrawModel::RAINBOW);
		connect(mDepthWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onDepthVideoCallback(int, int)));
		int one_layout_count = mVideoOneLayout->count();
		//qDebug() << "howard layout count =" << one_layout_count;
		//一个layout只能插入二个widget
		if (1 < one_layout_count) {
			VideoTwoLayout->addWidget(mDepthWindow);
			mViewLocation.insert(kDepthLocationKey, 20);
		}
		else
		{
			mVideoOneLayout->addWidget(mDepthWindow);
			mViewLocation.insert(kDepthLocationKey, 10);
		}
		mDepthOpen = KOpen;
		if (mDepthControl != nullptr)
		{
			QString value = mDepthControl->getVideoMode();
			setDepthMode(value);
		}
		ui->cw_depth_senser->setSwitchEnable(true);
		if (mColorControl != nullptr && (mColorOpen == KOpen || mUvcColorOpen == KOpen))
		{
			mColorControl->setD2CEnable(true);
		}
		//if (mDeviceSensorBase->mDeviceModeType == KDeviceDothin)
		//{
		//	bool bSuccess = mDeviceSensorBase->GetDothinCameraParams(mCameraParams);
		//	qDebug() << "oBCameraParams" << mCameraParams.l_intr_p[0] << mCameraParams.l_intr_p[1];
		//	if (!bSuccess) {
		//		qDebug() << "GetObCameraParams failure -------";
		//	}
		//}
	}
	else if (ui->cw_depth_senser->kCameraSwitchType == type && 0 == value) {
		//关闭depth流
		ui->cw_depth_senser->setSwitchEnable(false);
		closeDepthVideo(true);
		if (mDepthControl != nullptr)
		{
			mDepthControl->setDepthEnable(false);
		}
		ret_status = -1;
		//if (mColorHideOpen) {
		//	mColorHideOpen = false;
		//	ret_status = mDeviceSensorBase->StopStream(openni::SENSOR_COLOR);
		//}
		ui->cw_depth_senser->setSwitchEnable(true);

	}

	return ret_status;
}
void MainWindowStyle::onDepthVideoCallback(int type, int value) {

	if (mDepthWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mIrOpen == 1) {
				mIRWindow->hide();
				mIrOpen = 2;
			}
			if (mPhaseOpen == 1) {
				mPhaseWindow->hide();
				mPhaseOpen = 2;
			}
			if (mColorOpen == 1) {
				mRgbWindow->hide();
				mColorOpen = 2;
			}

			if (mUvcColorOpen == 1) {
				mUvcRgbWindow->hide();
				mUvcColorOpen = 2;
			}

			if (mPointCouldOpen == 1) {
				mPointCouldWindow->hide();
				mPointCouldOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
			if (mColorOpen == 2) {
				mRgbWindow->show();
				mColorOpen = 1;
			}

			if (mUvcColorOpen == 2) {
				mUvcRgbWindow->show();
				mUvcColorOpen = 1;
			}

			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();
				mPointCouldOpen = 1;
			}
		}
	}
	else if (mDepthWindow->kVideoCloseSwitchType == type) {
		//关闭流
		closeDepthVideo(true);
	}
	else if (mDepthWindow->kVideoCaptureType == type) {
		mDepthWindow->showCaptureFinishTip();
	}
	else if (mDepthWindow->kVideoPauseType == type) {
		//暂停与继续
		if (1 == value)
		{
			mDeviceSensorBase->StopStream(openni::SENSOR_DEPTH);
		}
		else
		{
			mDeviceSensorBase->StartStream(openni::SENSOR_DEPTH);
		}
	}
}
void MainWindowStyle::onPointCouldVideoCallback(int type, int value) {

	if (mPointCouldWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mIrOpen == 1) {
				mIRWindow->hide();
				mIrOpen = 2;
			}
			if (mPhaseOpen == 1) {
				mPhaseWindow->hide();
				mPhaseOpen = 2;
			}
			if (mColorOpen == 1) {
				mRgbWindow->hide();
				mColorOpen = 2;
			}
			if (mUvcColorOpen == 1) {
				mUvcRgbWindow->hide();
				mUvcColorOpen = 2;
			}
			if (mDepthOpen == 1) {
				mDepthWindow->hide();
				mDepthOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
			if (mColorOpen == 2) {
				mRgbWindow->show();
				mColorOpen = 1;
			}
			if (mUvcColorOpen == 2) {
				mUvcRgbWindow->show();
				mUvcColorOpen = 1;
			}
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
		}
	}
	else if (mPointCouldWindow->kVideoCloseSwitchType == type) {
		//关闭流
		closePointCloudVideo(true);
	}
	else if (mPointCouldWindow->kVideoCaptureType == type) {
		mPointCouldWindow->showCaptureFinishTip();
	}
	else if (mPointCouldWindow->kVideoPauseType == type) {
		//暂停与继续
		if (1 == value)
		{
			mDeviceSensorBase->StopStream(openni::SENSOR_DEPTH);
		}
		else
		{
			mDeviceSensorBase->StartStream(openni::SENSOR_DEPTH);
		}
	}
}
void MainWindowStyle::onDepthControlCallback(int type, QString value) {
	if (true == mDeviceSensorBase->IsOpened()) {
		if (mDepthControl->kResolutionChange == type)
		{
			//修改深度分辨率
			int resolution_index = value.toInt();
			if (mDepthOpen == KOpen)
			{
				std::string cur_resolution = mDthSpportMde.at(resolution_index);
				mDeviceSensorBase->changeStreamResolution(openni::SENSOR_DEPTH, cur_resolution);
			}
		}
		else if (mDepthControl->kVideoMode == type)
		{
			setDepthMode(value);
		}
		else if (mDepthControl->kPointCloundSwitch == type)
		{
			int clound_status = value.toInt();
			if (clound_status == KOpen)
			{
				//打开点云
				if (mDepthOpen == KOpen)
				{
					if (nullptr != mPointCouldWindow) {
						delete mPointCouldWindow;
						mPointCouldWindow = nullptr;
					}
					mPointCouldWindow = new VideoWindow;
					mPointCouldWindow->setStreamMode(mPointCouldWindow->kStrCloudPoint);
					mPointCouldWindow->setAttribute(Qt::WA_StyledBackground, true);
					mPointCouldWindow->setStyleSheet("background-color: rgb(36, 44, 51)");
					mPointCouldWindow->setVideoTitle("Point Cloud");

					connect(mPointCouldWindow, SIGNAL(videoViewCallback(int, int)), this, SLOT(onPointCouldVideoCallback(int, int)));
					int one_layout_count = mVideoOneLayout->count();
					//qDebug() << "howard layout count =" << one_layout_count;
					//一个layout只能插入二个widget
					if (1 < one_layout_count) {
						VideoTwoLayout->addWidget(mPointCouldWindow);
						mViewLocation.insert(kPointCouldLocationKey, 20);
					}
					else
					{
						mVideoOneLayout->addWidget(mPointCouldWindow);
						mViewLocation.insert(kPointCouldLocationKey, 10);
					}
					mPointCouldOpen = KOpen;
					mDeviceSensorBase->setPointCloudState(true);
				}
				else
				{
					mDepthControl->setPointCloundStatus(false);
				}
			}
			else
			{
				//关闭点云
				closePointCloudVideo(true);
				mDeviceSensorBase->setPointCloudState(false);
			}
		}
		else if (mDepthControl->kDepthColorMaxDistance == type)
		{
			int depthColorDistance = value.toInt();
			mCalc->setMaxDepthColorDistance(depthColorDistance);
		}
		else if (mDepthControl->kDepthColorMinDistance == type)
		{
			int depthColorDistance = value.toInt();
			mCalc->setMinDepthColorDistance(depthColorDistance);
		}
	}
}
void MainWindowStyle::setDepthMode(QString value) {
	if (mDepthWindow != nullptr)
	{
		if (value.contains("HISTOGRAM"))
		{
			mDepthDrawMode = DepthDrawModel::HISTOGRAM;
			mDepthWindow->setDepthMode(DepthDrawModel::HISTOGRAM);

		}
		else if (value.contains("GRAY"))
		{
			mDepthDrawMode = DepthDrawModel::GRAY;
			mDepthWindow->setDepthMode(DepthDrawModel::GRAY);

		}
		else if (value.contains("RAINBOW"))
		{
			mDepthDrawMode = DepthDrawModel::RAINBOW;
			mDepthWindow->setDepthMode(DepthDrawModel::RAINBOW);
		}
	}
}
void MainWindowStyle::onLeftIrControlCallback(int type, QString value) {
	if (true == mDeviceSensorBase->IsOpened()) {
		int dataValue = value.toInt();
		if (mIrControl->mResolutionChange == type) {
		}
		else if (mIrControl->mIRMaxDistance == type)
		{
			mIRDistanceMax = dataValue;
		}
		else if (mIrControl->mIRMinDistance == type)
		{
			mIRDistanceMin = dataValue;
		}
	}
}
void MainWindowStyle::onPhaseResolutionCallback(int type, QString value) {
}
void MainWindowStyle::onCommonWidgetCallback(int type, QString value) {
	qDebug() << "commom callback status:" << type << value;
	if (true == mDeviceSensorBase->IsOpened()) {
		if (mCommonControl->kCommonStreanTestChange == type) {
			//开关流压测
		}
		//else if (mCommonControl->kCommonLaserChange == type)
		//{
		//	//激光设置
		//	int laser_value = value.toInt();
		//	bool ret = false;

		//	ret = mDeviceSensorBase->setDothinLaser(laser_value == 1 ? true : false);
		//	if (!ret)
		//	{
		//		QMessageBox::warning(nullptr, "error", "set laser failed.");
		//	}
		//	else
		//	{
		//		qDebug() << "howard laser success---";
		//	}
		//}

		//else if (mCommonControl->kCommonFloodChange == type)
		//{
		//	//泛光灯设置
		//	int laser_value = value.toInt();
		//	bool ret = false;

		//	ret = mDeviceSensorBase->setDothinFlood(laser_value == 1 ? true : false);
		//}

		//else if (mCommonControl->kCommonBackgrounpChange == type)
		//{
		//	//减背景设置
		//	int laser_value = value.toInt();
		//	bool ret = false;

		//	ret = mDeviceSensorBase->setOutputMode(laser_value == 1 ? true : false);
		//	if (ret)
		//	{
		//		//关闭phase
		//		ui->cw_phase->setSwitchEnable(false);
		//		closePhaseVideo(true);
		//		ui->cw_phase->setSwitchEnable(true);
		//	}
		//}

		else if (mCommonControl->kCommonDsleepChange == type)
		{
			//deepsleep模式
			int dsleep_mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setDsleepMode(dsleep_mode == 1 ? true : false);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonHdrChange == type)
		{
			//hdr算法
			int hdr_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setHdrAlgorithm(hdr_en == 1 ? true : false);
			//if (ret)
			//{
			//	//关闭phase
			//	ui->cw_phase->setSwitchEnable(false);
			//	closePhaseVideo(true);
			//	ui->cw_phase->setSwitchEnable(true);
			//}
		}

		else if (mCommonControl->kCommonHistChange == type)
		{
			//直方图算法
			int hist_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setHistAlgorithm(hist_en == 1 ? true : false);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonMedianChange == type)
		{
			//median算法
			int midian_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setMedianAlgorithm(midian_en == 1 ? true : false);
			//if (ret)
			//{
			//	//关闭phase
			//	ui->cw_phase->setSwitchEnable(false);
			//	closePhaseVideo(true);
			//	ui->cw_phase->setSwitchEnable(true);
			//}
		}

		else if (mCommonControl->kCommonEbcChange == type)
		{
			//ebc算法
			int ebc_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setEbcAlgorithm(ebc_en == 1 ? true : false);
			//if (ret)
			//{
			//	//关闭phase
			//	ui->cw_phase->setSwitchEnable(false);
			//	closePhaseVideo(true);
			//	ui->cw_phase->setSwitchEnable(true);
			//}
		}

		else if (mCommonControl->kCommonLscChange == type)
		{
			//lsc算法
			int lsc_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setLscAlgorithm(lsc_en == 1 ? true : false);
			//if (ret)
			//{
			//	//关闭phase
			//	ui->cw_phase->setSwitchEnable(false);
			//	closePhaseVideo(true);
			//	ui->cw_phase->setSwitchEnable(true);
			//}
		}

		else if (mCommonControl->kCommonRowCorrection == type)
		{
			//行校正算法
			int correction_en = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setCorrectionAlgorithm(correction_en == 1 ? true : false);
			//if (ret)
			//{
			//	//关闭phase
			//	ui->cw_phase->setSwitchEnable(false);
			//	closePhaseVideo(true);
			//	ui->cw_phase->setSwitchEnable(true);
			//}
		}

		else if (mCommonControl->kCommonModeChange == type)
		{
			//工作模式
			int mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setOutputMode2(mode);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonBiningModeChange == type)
		{
			//像素合并
			int mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setBiningMode(mode);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonFlipModeChange == type)
		{
			//翻转
			int mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setMirrorFlip(mode);
		}

		else if (mCommonControl->kCommonSubSampChange == type)
		{
			//X方向间隔采样
			int val = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setSubSamp(val);
		}

		else if (mCommonControl->kCommonSubSampvChange == type)
		{
			//Y方向间隔采样
			int val = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setSubSampv(val);
		}

		else if (mCommonControl->kCommonOddDgainChange == type)
		{
			//奇数列数字增益
			int mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setOddDgain(mode);
		}

		else if (mCommonControl->kCommonEvenDgainChange == type)
		{
			//偶数列数字增益
			int mode = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setEvenDgain(mode);
		}

		else if (mCommonControl->kCommonWindowOriginyChange == type)
		{
			//开窗,改变originy
			int originy = value.toInt();
			bool ret = false;
			ret = mDeviceSensorBase->setWindowOriginy(originy);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonWindowOriginxChange == type)
		{
			//开窗，改变originx
			int originx = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setWindowOriginx(originx);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonWindowHeightChange == type)
		{
			//开窗，改变height
			int height = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setWindowHeight(height);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonWindowWidthChange == type)
		{
			//开窗，改变width
			int width = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setWindowWidth(width);
			if (ret)
			{
				//关闭phase
				ui->cw_phase->setSwitchEnable(false);
				closePhaseVideo(true);
				ui->cw_phase->setSwitchEnable(true);
			}
		}

		else if (mCommonControl->kCommonGainChange == type)
		{
			//增益景设置
			int gain_value = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setIRGain(gain_value);
		}

		else if (mCommonControl->kCommonExposureChange == type)
		{
			//曝光时间设置
			int exposure_value = value.toInt();
			bool ret = false;

			ret = mDeviceSensorBase->setIRExposure(exposure_value);
		}

		else if (mCommonControl->kCommonSoftwareReset == type)
		{
			//软件复位,先关phase
			ui->cw_phase->setSwitchEnable(false);
			closePhaseVideo(true);
			ui->cw_phase->setSwitchEnable(true);
			bool ret = false;
			ret = mDeviceSensorBase->setChipSoftReset();
		}

		else if (mCommonControl->kCommonHardwareReset == type)
		{
			//硬件复位,先关phase
			ui->cw_phase->setSwitchEnable(false);
			closePhaseVideo(true);
			ui->sw_video_window->setCurrentIndex(kConnectingPage);
			mDeviceStatusWidget->setDeviceStatus("Connect Device Failure", -1);
			ui->cw_phase->setSwitchEnable(true);
			bool ret = false;
			ret = mDeviceSensorBase->setChipHardReset();
			//QThread::sleep(2);
			//mOpenDeviceThread.CreateThread(mDeviceSensorBase);
		}
	}
}
PixelFormat MainWindowStyle::getPixelFormat(int index) {
	PixelFormat pixel_format = PIXEL_FORMAT_JOINT_2D;
	switch (index)
	{
	case PIXEL_FORMAT_JOINT_2D:
		pixel_format = PIXEL_FORMAT_JOINT_2D;
		break;
	case PIXEL_FORMAT_JOINT_3D:
		pixel_format = PIXEL_FORMAT_JOINT_3D;
		break;
	case PIXEL_FORMAT_BODY_MASK:
		pixel_format = PIXEL_FORMAT_BODY_MASK;
		break;
	case PIXEL_FORMAT_FLOOR_INFO:
		pixel_format = PIXEL_FORMAT_FLOOR_INFO;
		break;
	case PIXEL_FORMAT_BODY_SHAPE:
		pixel_format = PIXEL_FORMAT_BODY_SHAPE;
		break;
	case PIXEL_FORMAT_FACE:
		pixel_format = PIXEL_FORMAT_FACE;
		break;
	case PIXEL_FORMAT_GESTURE:
		pixel_format = PIXEL_FORMAT_GESTURE;
		break;
	default:
		break;
	}
	return pixel_format;
}
void MainWindowStyle::onAiControlCallback(int type, QString value) {
	if (true == mDeviceSensorBase->IsOpened()) {
		if (mAiStreamControl.kAiTypeChange == type)
		{
			//骨架类型
			int ret_status = 0;
			mAiStreamType = value.toInt();

			qDebug() << "howard skeleton_type_value=" << mAiStreamType;
			std::string cur_resolution = mAiSupportMode.at(mAiStreamType);
			int type = QString::fromStdString(cur_resolution).toInt();
			PixelFormat pixel_format = getPixelFormat(type);
			if (mColorOpen == KOpen)
			{
				mRgbWindow->setAIBodyShape(false);
			}

			else if (mUvcColorOpen == KOpen)
			{
				mUvcRgbWindow->setAIBodyShape(false);
			}

			if (mAiStreamOpen == KOpen)
			{
				ret_status = mDeviceSensorBase->changeAIStreamType(pixel_format);
				if (ret_status == 0)
				{
					ret_status = mDeviceSensorBase->StartStream(openni::SENSOR_AI);
				}
				mAiStreamOpen = KOpen;
				if (mAiStreamType == mAiStreamControl.kAi2DSkeleton)
				{
					//2D骨架
					if (mColorOpen == KOpen)
					{
						mRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
					}
					else if (mUvcColorOpen == KOpen)
					{
						mUvcRgbWindow->setAIRenderType(RenderType::kSkeletoneRender);
					}

				}
				else if (mAiStreamType == mAiStreamControl.kAi3DSkeleton)
				{
					//3D骨架

				}
				else if (mAiStreamType == mAiStreamControl.kAiBodyMask) {

				}
				else if (mAiStreamType == mAiStreamControl.kAiFloorInfo) {

				}
				else if (mAiStreamType == mAiStreamControl.kAiBodyShape) {
					if (mColorOpen == KOpen)
					{
						mRgbWindow->setAIBodyShape(true);
					}
					else if (mUvcColorOpen == KOpen)
					{
						mUvcRgbWindow->setAIBodyShape(true);
					}
				}
			}

		}
		else if (mAiStreamControl.kAiHeightChange == type) {
			if (mAiStreamOpen == KOpen && mAiStreamType == mAiStreamControl.kAiBodyShape) {
				QString body_height = QString("%1 CM").arg((int)(mDeviceSensorBase->mAiSekeleton2d.mBodyHeight / 10));
				//qDebug()<<"howard body_height="<<body_height;
				mAiStreamControl.setBodyHeight(body_height);
			}
		}
	}
}
void MainWindowStyle::onRgbResolutionCallback(int type, QString value) {
	if (true == mDeviceSensorBase->IsOpened()) {
		if (mColorControl->kResolutionChange == type)
		{
			//修改color分辨率
			int resolution_index = value.toInt();
			if (mColorOpen == KOpen)
			{
				std::string cur_resolution = mColorSupportMode.at(resolution_index);
				mDeviceSensorBase->changeStreamResolution(openni::SENSOR_COLOR, cur_resolution);
			}

		}
		else if (mColorControl->kD2cRenderChange == type)
		{
			int d2c_state = value.toInt();
			if (mDepthOpen == KOpen && mColorOpen == KOpen && mCommonD2CStatus &&d2c_state == KOpen)

			{
				if (mRgbWindow != nullptr)
				{
					mRgbWindow->setD2CStart(true);
					mD2CRenDer = true;
				}
			}
			else {
				if (mRgbWindow != nullptr)
				{
					mRgbWindow->setD2CStart(false);
				}
				mD2CRenDer = false;
				mColorControl->mD2cState = false;
				mColorControl->setD2CStatus(false);
			}

		}
		else if (mColorControl->kD2cDistanceChange == type)
		{
			int d2c_state_distance = value.toInt();
			mCalc->setD2CDistance(d2c_state_distance);
		}
	}
	else {

	}
}
void MainWindowStyle::onIrVideoCallback(int type, int value) {
	if (mIRWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mDepthOpen == 1) {
				mDepthWindow->hide();
				mDepthOpen = 2;
			}
			if (mPhaseOpen == 1) {
				mPhaseWindow->hide();
				mPhaseOpen = 2;
			}
			if (mColorOpen == 1) {
				mRgbWindow->hide();
				mColorOpen = 2;
			}

			if (mUvcColorOpen == 1) {
				mUvcRgbWindow->hide();
				mUvcColorOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
			if (mColorOpen == 2) {
				mRgbWindow->show();
				mColorOpen = 1;
			}

			if (mUvcColorOpen == 2) {
				mUvcRgbWindow->show();
				mUvcColorOpen = 1;
			}
		}
	}
	else if (mIRWindow->kVideoCloseSwitchType == type) {
		closeIRVideo();
	}
	else if (mIRWindow->kVideoCaptureType == type) {
		mIRWindow->showCaptureFinishTip();
	}
	else if (mIRWindow->kVideoPauseType == type) {
		//暂停
		if (1 == value)
		{
			mDeviceSensorBase->StopStream(openni::SENSOR_IR);
		}
		else
		{
			mDeviceSensorBase->StartStream(openni::SENSOR_IR);
		}
	}
}

void MainWindowStyle::onOniVideoCallback(int type, int value) {
   if (mONIWindow->kVideoCloseSwitchType == type) {
	   closeONIVideo();
	}
}

void MainWindowStyle::onPhaseVideoCallback(int type, int value) {
	if (mPhaseWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mDepthOpen == 1) {
				mDepthWindow->hide();
				mDepthOpen = 2;
			}
			if (mColorOpen == 1) {
				mRgbWindow->hide();
				mColorOpen = 2;
			}
			if (mIrOpen == 1) {
				mIRWindow->hide();
				mIrOpen = 2;
			}

			if (mUvcColorOpen == 1) {
				mUvcRgbWindow->hide();
				mUvcColorOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
			if (mColorOpen == 2) {
				mRgbWindow->show();
				mColorOpen = 1;
			}
			if (mUvcColorOpen == 2) {
				mUvcRgbWindow->show();
				mUvcColorOpen = 1;
			}
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();
				mPointCouldOpen = 1;
			}
		}
	}
	else if (mPhaseWindow->kVideoCloseSwitchType == type) {
		closePhaseVideo(true);
	}
	else if (mPhaseWindow->kVideoCaptureType == type) {
		mPhaseWindow->showCaptureFinishTip();
	}
	else if (mPhaseWindow->kVideoPauseType == type) {
		//暂停
		//        if (1 == value)
		//        {
		//            device_sensorbase->StopStream(openni::SENSOR_IR_R);
		//        }
		//        else
		//        {
		//            device_sensorbase->SwitchStreamMode(openni::SENSOR_IR_R, "");
		//        }
	}
}
void MainWindowStyle::onUVCRGBVideoCallback(int type, int value) {

	if (mUvcRgbWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mIrOpen == 1) {
				mIRWindow->hide();
				mIrOpen = 2;
			}
			if (mPhaseOpen == 1) {
				mPhaseWindow->hide();
				mPhaseOpen = 2;
			}
			if (mDepthOpen == 1) {
				mDepthWindow->hide();
				mDepthOpen = 2;
			}
			if (mPointCouldOpen == 1) {
				mPointCouldWindow->hide();
				mPointCouldOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();
				mPointCouldOpen = 1;
			}
		}
	}
	else if (mUvcRgbWindow->kVideoCloseSwitchType == type) {
		closeUVCRgbVideo();
	}
	else if (mUvcRgbWindow->kVideoCaptureType == type) {

	}
	else if (mUvcRgbWindow->kVideoPauseType == type) {
		//暂停
		if (1 == value)
		{
			mWmfFunction.stopThread();
		}
		else
		{
			HRESULT hr = mWmfFunction.setResolution(mUvcRgbCamera.getCurrentDevices(), mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).indexId, mUvcDevices.at(mUvcRgbCamera.getCurrentResolution()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).width, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).height, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).fps, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).format);
			if (SUCCEEDED(hr)) {
				qDebug() << "start preview";
				mWmfFunction.startThread();
			}
		}
	}

}
void MainWindowStyle::onRGBVideoCallback(int type, int value) {
	if (mRgbWindow->kVideoFullScreenType == type) {
		if (1 == value) {
			//开全屏
			if (mIrOpen == 1) {
				mIRWindow->hide();
				mIrOpen = 2;
			}
			if (mPhaseOpen == 1) {
				mPhaseWindow->hide();
				mPhaseOpen = 2;
			}
			if (mDepthOpen == 1) {
				mDepthWindow->hide();
				mDepthOpen = 2;
			}
			if (mPointCouldOpen == 1) {
				mPointCouldWindow->hide();
				mPointCouldOpen = 2;
			}
		}
		else
		{
			//关全屏
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();
				mPointCouldOpen = 1;
			}
		}
	}
	else if (mRgbWindow->kVideoCloseSwitchType == type) {
		closeRgbVideo(true);
	}
	else if (mRgbWindow->kVideoCaptureType == type) {
		mRgbWindow->showCaptureFinishTip();
	}
	else if (mRgbWindow->kVideoPauseType == type) {
		//暂停
		if (1 == value)
		{
			mDeviceSensorBase->StopStream(openni::SENSOR_COLOR);
		}
		else
		{
			mDeviceSensorBase->StartStream(openni::SENSOR_COLOR);
		}
	}
}
int MainWindowStyle::closePointCloudVideo(bool stop_stream) {
	int ret_status = 0;
	if (stop_stream) {
		//窗口变化先隐藏再显示，避免异常
		qDebug() << "depth open" << mDepthOpen;
		if (mPointCouldWindow == nullptr) {
			mPointCouldOpen = 0;
			return ret_status;
		}
		if (mDepthOpen == KOpen) {
			mDepthWindow->hide();
		}
		if (mIrOpen == 1) {
			mIRWindow->hide();
		}
		if (mPhaseOpen == 1) {
			mPhaseWindow->hide();
		}
	}
	mDepthControl->setPointCloundStatus(false);
	bool full_screen = mPointCouldWindow->getFullScreenStatus();
	if (mViewLocation.contains(kPointCouldLocationKey)) {
		int point_could_location_value = mViewLocation.value(kPointCouldLocationKey);
		if (10 == point_could_location_value && nullptr != mPointCouldWindow) {
			//从容器中删除
			mPointCouldWindow->hide();
			mVideoOneLayout->removeWidget(mPointCouldWindow);
			delete mPointCouldWindow;
			mPointCouldWindow = nullptr;
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == point_could_location_value && nullptr != mPointCouldWindow) {
			mPointCouldWindow->hide();
			VideoTwoLayout->removeWidget(mPointCouldWindow);
			delete mPointCouldWindow;
			mPointCouldWindow = nullptr;
		}
	}
	mPointCouldOpen = 0;
	if (stop_stream) {
		qDebug() << "howard close point cloud full_screen" << full_screen;

		//如果是全屏，要把之前隐藏的显示出来
		if (full_screen) {
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = 1;
			}
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = 1;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = 1;
			}
		}

		if (mDepthOpen == 1) {
			mDepthWindow->show();
		}
		if (mIrOpen == 1) {
			mIRWindow->show();
		}
		if (mPhaseOpen == 1) {
			mPhaseWindow->show();
		}
	}
	return ret_status;
}
int MainWindowStyle::closeDepthVideo(bool stop_stream) {
	int ret_status = 0;

	if (stop_stream) {
		mDeviceSensorBase->mDepthStreamOpen = false;
		mDeviceSensorBase->mDepthExitedThread = true;
		//窗口变化先隐藏再显示，避免异常
		if (mIrOpen == 1) {
			mIRWindow->hide();
			//device_sensorbase->StopStream(openni::SENSOR_IR);
		}
		if (mPhaseOpen == 1) {
			mPhaseWindow->hide();
		}
		if (mPointCouldOpen == 1 && mPointCouldWindow != nullptr) {
			mPointCouldWindow->hide();

		}
		if (mDepthWindow == nullptr) {
			mDepthOpen = 0;
			ui->cw_depth_senser->setCameraStatus(false);
			if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen) {
				ui->sw_video_window->setCurrentIndex(kAllClosePage);
			}
			return ret_status;
		}
	}
	closePointCloudVideo(true);
	bool full_screen = mDepthWindow->getFullScreenStatus();
	if (mViewLocation.contains(kDepthLocationKey)) {
		int depth_location_value = mViewLocation.value(kDepthLocationKey);
		if (10 == depth_location_value && nullptr != mDepthWindow) {
			//从容器中删除
			mDepthWindow->hide();
			mVideoOneLayout->removeWidget(mDepthWindow);
			delete mDepthWindow;
			mDepthWindow = nullptr;
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == depth_location_value && nullptr != mDepthWindow) {
			mDepthWindow->hide();
			VideoTwoLayout->removeWidget(mDepthWindow);
			delete mDepthWindow;
			mDepthWindow = nullptr;
		}
	}
	mDepthOpen = 0;
	mDeviceSensorBase->mDepthStreamInfo->mStreamStatus = false;
	ret_status = mDeviceSensorBase->StopStream(openni::SENSOR_DEPTH);
	ui->cw_depth_senser->setCameraStatus(false);
	if (mColorControl != nullptr)
	{
		mColorControl->setD2CEnable(false);

	}
	if (stop_stream) {
		qDebug() << "howard closeDepth full_screen" << full_screen;
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		else {
			//如果是全屏，要把之前隐藏的显示出来
			if (full_screen) {
				if (mIrOpen == 2) {
					mIRWindow->show();
					mIrOpen = 1;
				}
				if (mPhaseOpen == 2) {
					mPhaseWindow->show();
					mPhaseOpen = 1;
				}
			}
		}
		if (mIrOpen == KOpen) {
			mIRWindow->show();
		}
		if (mPhaseOpen == KOpen) {
			mPhaseWindow->show();
		}
	}
	return ret_status;
}
int MainWindowStyle::closeIRVideo() {
	int ret_status = 0;
	mDeviceSensorBase->mIrStreamOpen = false;
	if (mDepthOpen == KOpen) {
		mDepthWindow->hide();
	}
	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->hide();
	}
	if (mIRWindow == nullptr) {
		mIrOpen = 0;
		ui->cw_left_ir->setCameraStatus(false);
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		return ret_status;
	}
	bool fullScreen = mIRWindow->getFullScreenStatus();
	if (mViewLocation.contains(kIrLocationKey)) {
		int left_if_location_value = mViewLocation.value(kIrLocationKey);
		if (10 == left_if_location_value && nullptr != mIRWindow) {
			//从容器中删除
			mIRWindow->hide();
			mVideoOneLayout->removeWidget(mIRWindow);
			delete mIRWindow;
			mIRWindow = nullptr;
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == left_if_location_value && nullptr != mIRWindow) {
			mIRWindow->hide();
			VideoTwoLayout->removeWidget(mIRWindow);
			delete mIRWindow;
			mIRWindow = nullptr;
		}
	}
	mDeviceSensorBase->mIrStreamInfo->mStreamStatus = false;
	mDeviceSensorBase->StopStream(openni::SENSOR_IR);
	mIrOpen = 0;
	ui->cw_left_ir->setCameraStatus(false);
	if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen) {
		ui->sw_video_window->setCurrentIndex(kAllClosePage);
	}
	else
	{
		//如果是全屏，要把之前隐藏的显示出来
		if (fullScreen) {
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = KOpen;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = KOpen;
			}
			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();
				mPointCouldOpen = KOpen;
			}
		}
	}
	if (mDepthOpen == KOpen) {
		mDepthWindow->show();
	}
	if (mPhaseOpen == KOpen) {
		mPhaseWindow->show();
	}
	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->show();
	}
	return ret_status;

}
int MainWindowStyle::closePhaseVideo(bool stop_stream) {
	int ret_status = 0;

	if (stop_stream) {
		mDeviceSensorBase->mPhaseStreamOpen = false;
		if (mIrOpen == KOpen) {
			mIRWindow->hide();
		}
		if (mPhaseWindow == nullptr) {
			mPhaseOpen = 0;
			ui->cw_phase->setCameraStatus(false);
			if (0 == mIrOpen && 0 == mPhaseOpen) {
				ui->sw_video_window->setCurrentIndex(kAllClosePage);
			}
			return ret_status;
		}

	}
	bool full_screen = mPhaseWindow->getFullScreenStatus();
	if (mViewLocation.contains(kPhaseLocationKey)) {
		int right_ir_location_value = mViewLocation.value(kPhaseLocationKey);
		if (10 == right_ir_location_value && nullptr != mPhaseWindow) {
			//从容器中删除
			mPhaseWindow->hide();
			mVideoOneLayout->removeWidget(mPhaseWindow);
			delete mPhaseWindow;
			mPhaseWindow = nullptr;

			if (mIRWindow != nullptr)
			{
				mIRWindow->hide();
				mVideoOneLayout->removeWidget(mIRWindow);
				delete mIRWindow;
				mIRWindow = nullptr;
			}
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == right_ir_location_value && nullptr != mPhaseWindow) {
			mPhaseWindow->hide();
			VideoTwoLayout->removeWidget(mPhaseWindow);
			delete mPhaseWindow;
			mPhaseWindow = nullptr;
		}
	}
	mPhaseOpen = 0;
	mDeviceSensorBase->mAiStreamInfo->mStreamStatus = false;
	ui->cw_phase->setCameraStatus(false);
	ret_status = mDeviceSensorBase->StopStream(openni::SENSOR_PHASE);
	if (stop_stream) {
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		else
		{
			//如果是全屏，要把之前隐藏的显示出来
			if (full_screen) {
				if (mIrOpen == 2) {
					mIRWindow->show();
					mIrOpen = 1;
				}
			}
		}
		if (mIrOpen == KOpen) {
			mIRWindow->show();
		}
	}
	return ret_status;
}

int MainWindowStyle::closeONIVideo()
{
	if (mONIOpen == KOpen)
	{
		//mPlayOniVideo.close();
		//从容器中删除
		if (mONIWindow != nullptr)
		{
			mONIWindow->hide();
			mVideoOneLayout->removeWidget(mONIWindow);
			delete mONIWindow;
			mONIWindow = nullptr;
		}
		mONIOpen = KClose;
		mDeviceSensorBase->mOniStreamInfo->mVideoStream.stop();
		ui->sw_video_window->setCurrentIndex(kAllClosePage);
	}
	return 0;
}
int MainWindowStyle::closeUVCRgbVideo() {
	int ret_status = 0;
	mDeviceSensorBase->mUvcColorStreamOpen = false;
	mWmfFunction.stopThread();
	mWmfFunction.closeDevice();
	if (mIrOpen == KOpen) {
		mIRWindow->hide();
	}
	if (mPhaseOpen == KOpen) {
		mPhaseWindow->hide();
	}
	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->hide();
	}
	if (mDepthOpen == KOpen) {
		mDepthWindow->hide();
	}
	if (mUvcRgbWindow == nullptr) {
		mUvcColorOpen = 0;
		ui->cw_rgb_camera_uvc->setCameraStatus(false);
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mUvcColorOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		return ret_status;
	}
	bool full_screen = mUvcRgbWindow->getFullScreenStatus();
	if (mViewLocation.contains(kRgbLocationKey)) {
		int rgb_location_value = mViewLocation.value(kRgbLocationKey);
		if (10 == rgb_location_value && nullptr != mUvcRgbWindow) {
			//从容器中删除
			mUvcRgbWindow->hide();
			mVideoOneLayout->removeWidget(mUvcRgbWindow);
			delete mUvcRgbWindow;
			mUvcRgbWindow = nullptr;
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == rgb_location_value && nullptr != mUvcRgbWindow) {
			mUvcRgbWindow->hide();
			VideoTwoLayout->removeWidget(mUvcRgbWindow);
			delete mUvcRgbWindow;
			mUvcRgbWindow = nullptr;
		}
	}
	mUvcColorOpen = 0;
	mDeviceSensorBase->mColorStreamInfo->mStreamStatus = false;
	ui->cw_rgb_camera_uvc->setCameraStatus(false);
	if (mColorControl != nullptr)
	{
		mColorControl->setD2CEnable(false);
	}
	if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mUvcColorOpen) {
		ui->sw_video_window->setCurrentIndex(kAllClosePage);
	}
	else {
		//如果是全屏，要把之前隐藏的显示出来
		if (full_screen) {
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = KOpen;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = KOpen;
			}
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = KOpen;
			}
			// if (point_could_open == 2) {
			// point_could_window->show();
			// point_could_open = OPEN;
			// }
		}
	}

	if (mIrOpen == KOpen) {
		mIRWindow->show();


	}
	if (mPhaseOpen == KOpen) {
		mPhaseWindow->show();
	}
	if (mDepthOpen == KOpen) {
		mDepthWindow->show();
	}
	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->show();
	}
	return ret_status;
}
int MainWindowStyle::closeRgbVideo(bool stop_stream) {
	int ret_status = 0;
	if (stop_stream) {
		ret_status = mDeviceSensorBase->StopStream(openni::SENSOR_COLOR);
	}


	if (mIrOpen == KOpen) {
		mIRWindow->hide();
	}
	if (mPhaseOpen == KOpen) {
		mPhaseWindow->hide();
	}
	if (mDepthOpen == KOpen) {
		mDepthWindow->hide();
	}

	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->hide();
	}

	if (mRgbWindow == nullptr) {
		mColorOpen = 0;
		ui->cw_rgb_camera->setCameraStatus(false);
		if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mColorOpen) {
			ui->sw_video_window->setCurrentIndex(kAllClosePage);
		}
		return ret_status;
	}
	bool full_screen = mRgbWindow->getFullScreenStatus();
	if (mViewLocation.contains(kRgbLocationKey)) {
		int rgb_location_value = mViewLocation.value(kRgbLocationKey);
		if (10 == rgb_location_value && nullptr != mRgbWindow) {
			//从容器中删除
			mRgbWindow->hide();
			mVideoOneLayout->removeWidget(mRgbWindow);
			delete mRgbWindow;
			mRgbWindow = nullptr;
			//修改视频窗口
			int video_two_layout = VideoTwoLayout->count();
			int video_one_layout = mVideoOneLayout->count();
			if (video_two_layout == 1 && video_one_layout == 1)
			{
				QLayoutItem *child = VideoTwoLayout->takeAt(0);
				mVideoOneLayout->addItem(child);
				VideoTwoLayout->removeItem(child);
				mViewLocation.insert(kDepthLocationKey, 10);
				mViewLocation.insert(kIrLocationKey, 10);
				mViewLocation.insert(kPhaseLocationKey, 10);
				mViewLocation.insert(kRgbLocationKey, 10);
				mViewLocation.insert(kPointCouldLocationKey, 10);
			}
		}
		else if (20 == rgb_location_value && nullptr != mRgbWindow) {
			mRgbWindow->hide();
			VideoTwoLayout->removeWidget(mRgbWindow);
			delete mRgbWindow;
			mRgbWindow = nullptr;
		}
	}
	mColorOpen = 0;
	ui->cw_rgb_camera->setCameraStatus(false);
	if (mColorControl != nullptr)
	{
		mColorControl->setD2CEnable(false);
	}
	if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mColorOpen) {
		ui->sw_video_window->setCurrentIndex(kAllClosePage);
	}
	else {
		//如果是全屏，要把之前隐藏的显示出来
		if (full_screen) {
			if (mIrOpen == 2) {
				mIRWindow->show();
				mIrOpen = KOpen;
			}
			if (mPhaseOpen == 2) {
				mPhaseWindow->show();
				mPhaseOpen = KOpen;
			}
			if (mDepthOpen == 2) {
				mDepthWindow->show();
				mDepthOpen = KOpen;
			}
			if (mPointCouldOpen == 2) {
				mPointCouldWindow->show();

				mPointCouldOpen = KOpen;

			}
		}
	}

	if (mIrOpen == KOpen) {
		mIRWindow->show();

	}
	if (mDepthOpen == KOpen) {
		mDepthWindow->show();

	}
	if (mPhaseOpen == KOpen) {
		mPhaseWindow->show();
	}
	if (mPointCouldOpen == KOpen) {
		mPointCouldWindow->show();
	}

	return ret_status;
}
/// 修改在线状态
void MainWindowStyle::SwithOnLineStatus(bool b_online)
{
	qDebug() << "howard SwithOnLineStatus =" << b_online;
	if (b_online)
	{
		QThread::sleep(3);
		emit SigConnecte(b_online);
	}
	else
	{
		emit SigConnecte(b_online);
	}
}

void MainWindowStyle::firstStart() {

	mDeviceStatusWidget->setDeviceStatus("Connect Device Success", 0);
	//设备开启成功
	QString str;
	str.append("color: rgb(193, 211, 227);").append("font: 18pt ""Agency FB"";");
	ui->label_device_name->setStyleSheet(str);
	ui->label_device_name->setText("Camera device");
	int sensorID;
	mDeviceSensorBase->getDonthinSensorId(sensorID);
	getDeviceState();
	ui->cw_phase->setSwitchVisible(false);
	mOpenDeviceThread.loadRegisterFile(LOAD_REGISTER);
}
void MainWindowStyle::getAllResolution() {
	openni::SensorType sensorTypes[] = { openni::SENSOR_IR, openni::SENSOR_COLOR, openni::SENSOR_DEPTH, openni::SENSOR_PHASE, openni::SENSOR_AI };
	mSupportedSensors.clear();

	for (auto type : sensorTypes)
	{
		if (mDeviceSensorBase->mOniDevice.hasSensor(type))
		{
			mSupportedSensors.push_back(type);
			qDebug("Support sensor type (%d)\n", type);
		}
		else
			qDebug("Not support sensor type (%d)\n", type);
	}
	size_t len = mSupportedSensors.size();

	for (size_t i = 0; i < len; i++) {

		openni::SensorType sensor_type = mSupportedSensors[i];
		qDebug() << "sensor type: " << sensor_type;
		if (sensor_type == openni::SENSOR_PHASE)
		{
			phaseControlLayout();
		}

	}
}
void MainWindowStyle::getDeviceState() {

	getAllResolution();
	if (mDeviceSensorBase->mDeviceModeType != kDevicePleco)
	{
		
		//int IRGain = 0;
		//bool gainStatus = mDeviceSensorBase->getIRGain(&IRGain);
		//if (gainStatus)
		//{
			//mCommonControl->IRGainSet(IRGain);
		//}

		//bool laserStatus = mDeviceSensorBase->getDothinLaser();
		//mCommonControl->setLaserStatus(laserStatus);
		//bool floodStatus = mDeviceSensorBase->getDothinFlood();
		//mCommonControl->setFloodStatus(floodStatus); 
		//bool backgrounpStatus = mDeviceSensorBase->getOutputMode();
		//mCommonControl->setBackgrounpStatus(backgrounpStatus);
	}
	else {
		//bool ae_state = mDeviceSensorBase->getAEProperty();
		//mCommonD2CStatus = mDeviceSensorBase->getD2CProperty();
		//bool laser = mDeviceSensorBase->getLaserProperty();
		//mCommonControl->setLaserStatus(laser);
		//mDeviceSensorBase->setFrequencyForMore(mInitConfig.config_res_.mStreamMode);
		//ui->cw_common_wiget->setFrequencyEnable(false);
		//ui->cw_common_wiget->setQrequencyStatus(mInitConfig.config_res_.mStreamMode+1);
		//ui->cw_common_wiget->setFrequencyEnable(true);
		//mDeviceSensorBase->mSensorId = mDeviceSensorBase->getSensorId(openni::SENSOR_DEPTH);
	}
}

/**
* @brief MainWindow::CallbackConnectStatus 设备拔插响应
* @param is_online 在线状态
*
*/
void MainWindowStyle::CallbackConnectStatus(bool is_online)
{
	if (is_online)
	{
		qDebug() << "device connect";
		mOpenDeviceThread.CreateThread(mDeviceSensorBase);

	}
	else
	{
		qDebug() << "device disconnect";
		QString str;
		str.append("color: rgb(255, 0, 0);").append("font: 12pt ""Agency FB"";");
		ui->label_device_name->setStyleSheet(str);
		ui->label_device_name->setText("No device");
		ui->sw_video_window->setCurrentIndex(kConnectingPage);
		mDeviceStatusWidget->setDeviceStatus("Connect Device Failure", -1);
		mColorControl->setDistance(false);
		closePhaseVideo(true);
		// 5. 关闭设备
		mDeviceSensorBase->CloseDevice();	// has to close device
	}
}

/**
* @brief MainWindow::CallbackUpdateColorData 显示Color图
* @param data 彩色数据
*/
void MainWindowStyle::CallbackUpdateColorData(OniData *data)
{
	if (!mClosewindow)
	{
		if (nullptr != mRgbWindow && mColorOpen == KOpen) {
			mRgbWindow->setD2CStart(mD2CRenDer);
			mRgbWindow->UpdateColorData(data);
			if (mAiStreamOpen == KOpen)
			{
				if (mAiStreamType == mAiStreamControl.kAi2DSkeleton)
				{
					mRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
					mDeviceSensorBase->mAiSekeleton2d.mJointFormat = PixelFormat::PIXEL_FORMAT_NONE;
				}
				else if (mAiStreamType == mAiStreamControl.kAiFloorInfo) {
					mDeviceSensorBase->mMutexAiFrame.lock();
					QString str_center_x = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[0], 'f', 2);
					QString str_center_y = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[1], 'f', 2);
					QString str_center_z = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[2], 'f', 2);
					QString str_vector_x = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[0], 'f', 2);
					QString str_vector_y = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[1], 'f', 2);
					QString str_vector_z = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[2], 'f', 2);

					mAiStreamControl.setPlaneCenter(QString("x:%1 y:%2 z:%3").arg(str_center_x).arg(str_center_y).arg(str_center_z));
					mAiStreamControl.setPlaneVector(QString("x:%1 y:%2 z:%3").arg(str_vector_x).arg(str_vector_y).arg(str_vector_z));
					mRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
					mDeviceSensorBase->mAiSekeleton2d.mJointFormat = PixelFormat::PIXEL_FORMAT_NONE;
					mDeviceSensorBase->mMutexAiFrame.unlock();
				}
				else if (mAiStreamType == mAiStreamControl.kAiBodyShape)
				{
					mRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);

				}
				else if (mAiStreamType == mAiStreamControl.kAiBodyMask)
				{
					//抠图
					mDeviceSensorBase->mMutexAiFrame.lock();
					mRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
					//device_sensorbase->ai_sekeleton_2d.jointFormat = PixelFormat::PIXEL_FORMAT_NONE;
					mDeviceSensorBase->mMutexAiFrame.unlock();
				}
			}
		}
	}
}

//注册USB监听
void MainWindowStyle::registerUSBNotification()
{
	qDebug() << "开始注册USB监听";

	HDEVNOTIFY hDevNotify;
	DEV_BROADCAST_DEVICEINTERFACE NotifacationFiler;

	ZeroMemory(&NotifacationFiler, sizeof(DEV_BROADCAST_DEVICEINTERFACE));
	NotifacationFiler.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
	NotifacationFiler.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;

	for (int i = 0; i < sizeof(GUID_DEVINTERFACE_LIST) / sizeof(GUID); i++)
	{
		NotifacationFiler.dbcc_classguid = GUID_DEVINTERFACE_LIST[i];//GetCurrentUSBGUID();//m_usb->GetDriverGUID();
		hDevNotify = RegisterDeviceNotification((HANDLE)this->winId(), &NotifacationFiler, DEVICE_NOTIFY_WINDOW_HANDLE);
		if (!hDevNotify)
		{
			int Err = GetLastError();
			qDebug() << "注册失败:" << Err;
		}
	}
}
bool MainWindowStyle::nativeEventFilter(const QByteArray &eventType, void *message, long *result) {

	Q_UNUSED(result);
	//其中eventType表明了此次消息的类型，message存储着具体是什么信息，result是个传出变量；
	//eventType在windows操作系统下是"windows_generic_MSG"字符串，可以查看Qt的文档知道
	//message表明这个信号附带哪些信息，在热插拔事件中是WM_DEVICECHANGE类型，具体windows定义了哪些，可以查看“Dbt.h”文件
	if (eventType == QByteArray("windows_generic_MSG")) {
		MSG *pMsg = reinterpret_cast<MSG*>(message);

		if (pMsg->message == WM_DEVICECHANGE) {

			qDebug("wParam:%d",pMsg->wParam) ;
			switch (pMsg->wParam)
			{
				//设备连上
			case DBT_DEVICEARRIVAL:
			{
				//检查设备连接
				auto ret = mDeviceSensorBase->OpenAnyDevice();
				if (0 == ret.first)
				{
					mDeviceStatusWidget->setDeviceStatus("Connect Device Success", 0);
					//设备开启成功
					QString str;
					str.append("color: rgb(193, 211, 227);").append("font: 18pt ""Agency FB"";");
					ui->label_device_name->setStyleSheet(str);
					ui->label_device_name->setText("Camera device");
					getDeviceState();
				}
			}
				//CallbackConnectStatus(true);
				break;

				//设备断开
			case DBT_DEVICEREMOVECOMPLETE:
				//检查设备连接
				CallbackConnectStatus(false);
				break;
			}
		}
	}
	return false;
}
/**
* @brief MainWindow::DrawDepthVideo 显示深度图，同时出发点云处理
* @param data  已在线程里处理好的深度数据
*/
void MainWindowStyle::DrawDepthVideo(OniData * data)
{
	if (!mClosewindow)
	{
		if (mDepthOpen == 1 || mPointCouldOpen == 1) {
			if (mPointCloudFilterBuff && mInitConfig.config_res_.mFilterSetOnUI) {
				mPointCloudFilterBuff = false;

				data->oData.mPointCloudFilterParams[0] = mPointCloudFilterParams[0];
				data->oData.mPointCloudFilterParams[1] = mPointCloudFilterParams[1];
				data->oData.mPointCloudFilterParams[2] = mPointCloudFilterParams[2];
				readFileBuffer(data);

				qDebug() << " mPointCloudFilterParams[2] " << mPointCloudFilterParams[2];
			}
		}

		if (nullptr != mDepthWindow && mDepthOpen == KOpen) {
			mDepthWindow->updateDepthData(data);
		}

		if (nullptr != mPointCouldWindow && mPointCouldOpen == KOpen) {
			mDeviceSensorBase->mPointCloudMutex.lock();
			mPointCouldWindow->updatePointCouldData(data);
			mDeviceSensorBase->mPointCloudMutex.unlock();
		}
	}
}
/**
* @brief MainWindow::CallbackUpdateIr 显示IR图, 主线程
* @param data IR数据， 已转换过888bit
*/
void MainWindowStyle::CallbackUpdateIr(OniData *data, int w, int h)
{
	if (!mClosewindow)
	{
		if (nullptr != mIRWindow) {
			mIRWindow->UpdateIrData(data, w, h, kRenderIR);

		}
	}

}
/**
* @brief MainWindow::CallbackUpdateIr 显示IR图, 主线程
* @param data IR数据， 已转换过888bit
*/
void MainWindowStyle::CallbackUpdatePhase(OniData *data, int w, int h)
{
	if (!mClosewindow)
	{
		if (nullptr != mPhaseWindow && mPhaseOpen == KOpen) {
			mPhaseWindow->UpdateIrData(data, w, h, kRenderPhase);
			if (mPhaseControl != nullptr && mPhaseControl->resolutionSize() < 1)
			{
				mPhaseControl->addResolution(QString("%1x_%2").arg(data->mWidth).arg(data->mHeight));
			}

		}
	}

}

/**
* @brief MainWindow::CbSnapIR sensorbase的线程里调用此回调函数， 注意这个函数是在非主线程运行的。
* @param data
*/
void MainWindowStyle::CbSnapPhase(OniData &data)
{
	/// 转换到888bit，用于显示
	//qDebug()<<"howard CbSnapPhase";

	if (mPhaseRender) {
		if (2 == data.mPixelInBytes)
		{

			if (!data.oData.UpdataDatePhase()) {
				return;
			}
		}

		emit SigUpdatePhase(&data, data.mWidth, data.mHeight);

		mFramePhase++;
	}
};
/**
* @IR sensorbase的线程里调用此回调函数， 注意这个函数是在非主线程运行的。
* @param data
*/
void MainWindowStyle::CbSnapIR(OniData &data)
{
	/// 转换到888bit，用于显示
		if (2 == data.mPixelInBytes)
		{
			if (!data.oData.UpdataDateOtherIR(data.mWidth, data.mHeight, mIRDistanceMin, mIRDistanceMax)) {
				return;
			}
		}

		emit SigUpdateIr(&data, data.mWidth, data.mHeight);
		mFrameIR++;
};

void MainWindowStyle::CbSnapColor(OniData &data)
{
	if (mColorRender) {
		emit SigUpdateColor(&data);
		/// 统计帧率
		mFrameRGB++;
	}
};

void MainWindowStyle::CbSnapOni(OniData &data)
{
	if (mONIOpen) {
		emit SigUpdateOni(&data);
		/// 统计帧率
		mFrameONI++;
	}
};

/**
* @brief MainWindow::CbSnapDepth sensorbase的线程里调用此回调函数， 注意这个函数是在非主线程运行的。
* @param depth_data_
*/
void MainWindowStyle::CbSnapDepth(OniData &depth_data_)
{
	if (mDepthRender) {
		int8_t* depth_rgb_ptr = nullptr;
		if (nullptr != depth_data_.oData.mDataPtr)
		{
			try
			{
				if (DepthDrawModel::GRAY == mDepthDrawMode)
				{

					depth_data_.oData.SetRangeOfDepth(mRangeOfDepth);
					depth_data_.oData.UpdataDate888bit();
					depth_rgb_ptr = depth_data_.oData.mData888bitPtr;
				}
				else if (DepthDrawModel::HISTOGRAM == mDepthDrawMode) {
					depth_data_.oData.UpdataDateHistogram(depth_data_.mWidth, depth_data_.mHeight, false);
					depth_rgb_ptr = depth_data_.oData.mDataHistogram888Ptr;
				}
				else if (DepthDrawModel::RAINBOW == mDepthDrawMode) {
					mCalc->rainbowFunc(depth_data_);
					depth_rgb_ptr = depth_data_.oData.mDataRainbow888Ptr;
				}
				/* 多线程数据的复制*/
				unsigned int sizeDepth = depth_data_.mWidth * depth_data_.mHeight * sizeof(uint16_t);
				if (depth_data_.oData.mDataDepthPtr == nullptr)
				{
					depth_data_.oData.mDataDepthPtr = new int8_t[sizeDepth]();
				}
				memset(depth_data_.oData.mDataDepthPtr,0, sizeDepth);
				memcpy(depth_data_.oData.mDataDepthPtr, depth_data_.oData.mDataPtr, sizeDepth);
				if (mD2CRenDer && mRgbWindow != nullptr) {
					bool state = mCalc->updateD2CFunc(depth_data_);
					if (state)
					{
						mRgbWindow->UpdateD2CData(depth_data_.oData.mDataD2c888Ptr, depth_data_.mWidth, depth_data_.mHeight, depth_data_.oData.SizeOfD2C888bit());
					}

				}

			}
			catch (...)
			{
				qDebug() << __FILE__ << __LINE__ << __FUNCTION__ << "Error: memcpy error.";
			}
		}

		emit SigDrawDepth(&depth_data_);

		/// 统计深度帧率
		mFrameDepth++;
	}
}
void MainWindowStyle::timerEvent(QTimerEvent *event)
{
	//mDepthOpen = mDeviceSensorBase->mDepthStreamOpen;
	//mIrOpen = mDeviceSensorBase->mIrStreamOpen;
	//mPhaseOpen = mDeviceSensorBase->mPhaseStreamOpen;
	//mColorOpen = mDeviceSensorBase->mColorStreamOpen;
	/// 更新帧率
	if (mDepthOpen == 1)
	{
		mDepthWindow->setFrameRate(mFrameDepth);
		mFrameDepth = 0;
	}
	if (mONIOpen == 1)
	{
		mONIWindow->setFrameRate(mFrameONI);
		mFrameONI = 0;
	}
	if (mIrOpen == 1)
	{

	}
	if (mPhaseOpen == 1)
	{
		mPhaseWindow->setFrameRate(mFramePhase);
		mFramePhase = 0;
		if (mIRWindow!=nullptr)
		{
			mIRWindow->setFrameRate(mFrameIR);
			mFrameIR = 0;
		}
	}
	if (mColorOpen == 1 && !mColorHideOpen)
	{
		mRgbWindow->setFrameRate(mFrameRGB);
		mFrameRGB = 0;
	}
}
void MainWindowStyle::closeAllCamera() {
	//关闭rgb流
	if (mColorOpen == KOpen) {

		if (mRgbWindow == nullptr) {
			mColorOpen = 0;
			ui->cw_rgb_camera->setCameraStatus(false);
			if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mColorOpen) {
				ui->sw_video_window->setCurrentIndex(kAllClosePage);
			}
			return;
		}
		if (mViewLocation.contains(kRgbLocationKey)) {
			int rgb_location_value = mViewLocation.value(kRgbLocationKey);
			if (10 == rgb_location_value && nullptr != mRgbWindow) {
				//从容器中删除
				mRgbWindow->hide();
				mVideoOneLayout->removeWidget(mRgbWindow);
				delete mRgbWindow;
				mRgbWindow = nullptr;
				//修改视频窗口
				int video_two_layout = VideoTwoLayout->count();
				int video_one_layout = mVideoOneLayout->count();
				if (video_two_layout == 1 && video_one_layout == 1)
				{
					QLayoutItem *child = VideoTwoLayout->takeAt(0);
					mVideoOneLayout->addItem(child);
					VideoTwoLayout->removeItem(child);
					mViewLocation.insert(kDepthLocationKey, 10);
					mViewLocation.insert(kIrLocationKey, 10);
					mViewLocation.insert(kPhaseLocationKey, 10);
					mViewLocation.insert(kRgbLocationKey, 10);
					mViewLocation.insert(kPointCouldLocationKey, 10);
				}
			}
			else if (20 == rgb_location_value && nullptr != mRgbWindow) {
				mRgbWindow->hide();
				VideoTwoLayout->removeWidget(mRgbWindow);
				delete mRgbWindow;
				mRgbWindow = nullptr;
			}
		}

		mColorOpen = KClose;
		mDeviceSensorBase->StopStream(openni::SENSOR_COLOR);
		ui->cw_rgb_camera->setCameraStatus(false);
	}
	//关闭depth流
	if (mDepthOpen == KOpen) {

		if (mDepthWindow == nullptr) {
			mDepthOpen = 0;
			ui->cw_depth_senser->setCameraStatus(false);
			if (0 == mDepthOpen && 0 == mIrOpen && 0 == mPhaseOpen && 0 == mColorOpen) {
				ui->sw_video_window->setCurrentIndex(kAllClosePage);
			}
			return;
		}

		if (mViewLocation.contains(kDepthLocationKey)) {
			int depth_location_value = mViewLocation.value(kDepthLocationKey);
			if (10 == depth_location_value && nullptr != mDepthWindow) {
				//从容器中删除
				mDepthWindow->hide();
				mVideoOneLayout->removeWidget(mDepthWindow);
				delete mDepthWindow;
				mDepthWindow = nullptr;
				//修改视频窗口
				int video_two_layout = VideoTwoLayout->count();
				int video_one_layout = mVideoOneLayout->count();
				if (video_two_layout == 1 && video_one_layout == 1)
				{
					QLayoutItem *child = VideoTwoLayout->takeAt(0);
					mVideoOneLayout->addItem(child);
					VideoTwoLayout->removeItem(child);
					mViewLocation.insert(kDepthLocationKey, 10);
					mViewLocation.insert(kIrLocationKey, 10);
					mViewLocation.insert(kPhaseLocationKey, 10);
					mViewLocation.insert(kRgbLocationKey, 10);
					mViewLocation.insert(kPointCouldLocationKey, 10);
				}
			}
			else if (20 == depth_location_value && nullptr != mDepthWindow) {
				mDepthWindow->hide();
				VideoTwoLayout->removeWidget(mDepthWindow);
				delete mDepthWindow;
				mDepthWindow = nullptr;
			}
		}
		mDepthOpen = 0;
		mDeviceSensorBase->StopStream(openni::SENSOR_DEPTH);
		ui->cw_depth_senser->setCameraStatus(false);
	}
	//关闭相位流
	if (mPhaseOpen == KOpen) {


		if (mViewLocation.contains(kPhaseLocationKey)) {
			int right_ir_location_value = mViewLocation.value(kPhaseLocationKey);
			if (10 == right_ir_location_value && nullptr != mPhaseWindow) {
				//从容器中删除
				mPhaseWindow->hide();
				mVideoOneLayout->removeWidget(mPhaseWindow);
				delete mPhaseWindow;
				mPhaseWindow = nullptr;
				//修改视频窗口
				int video_two_layout = VideoTwoLayout->count();
				int video_one_layout = mVideoOneLayout->count();
				if (video_two_layout == 1 && video_one_layout == 1)
				{
					QLayoutItem *child = VideoTwoLayout->takeAt(0);
					mVideoOneLayout->addItem(child);
					VideoTwoLayout->removeItem(child);
					mViewLocation.insert(kDepthLocationKey, 10);
					mViewLocation.insert(kIrLocationKey, 10);
					mViewLocation.insert(kPhaseLocationKey, 10);
					mViewLocation.insert(kRgbLocationKey, 10);
					mViewLocation.insert(kPointCouldLocationKey, 10);
				}
			}
			else if (20 == right_ir_location_value && nullptr != mPhaseWindow) {
				mPhaseWindow->hide();
				VideoTwoLayout->removeWidget(mPhaseWindow);
				delete mPhaseWindow;
				mPhaseWindow = nullptr;
			}
		}
		mPhaseOpen = 0;
		mDeviceSensorBase->StopStream(openni::SENSOR_AI);
		ui->cw_phase->setCameraStatus(false);
	}
	if (mIrOpen == KOpen) {
		closeIRVideo();
	}
	if (mAiStreamOpen == KOpen)
	{

		mAiStreamOpen = KClose;
		int ret_status = mDeviceSensorBase->StopStream(openni::SENSOR_AI);
	}
	if (mUvcColorOpen == KOpen)
	{
		closeUVCRgbVideo();
	}
	ui->sw_video_window->setCurrentIndex(kAllClosePage);
}

int MainWindowStyle::openUVCRgbVideo(QString resolution) {
	int ret_status = 0;
	mUvcRgbCamera.setResolutionChange(false);
	mUvcRgbCamera.setCurrentResolution(resolution);
	mUvcRgbCamera.setResolutionChange(true);
	ret_status = onUVCRGBSwitch(1, 1);
	if (ret_status == 0) {
		ret_status = 0;
		QString show_content = QString("%1=%2").arg("change uvc resolution").arg(resolution);
		qDebug() << "change uvc resolution " << resolution;
		mStreamTestDialog->ShowInfo(show_content);
	}
	else
	{
		ret_status = -1;
		QString show_content = QString("%1=%2%3%4").arg("change uvc resolution").arg(resolution).arg("error code=").arg(ret_status);
		qDebug() << "change uvc resolution error code=" << ret_status;
		mStreamTestDialog->ShowInfo(show_content);
	}

	return ret_status;
}
void MainWindowStyle::UvcDevicesCallback(QList<Uvc_Device> mDevices) {
	int devices_size = mDevices.size();
	qDebug() << "howard devices_size=" << devices_size;
	mUvcDevices = mDevices;
	if (devices_size > 0) {
		//有UVC设备，显示UVC设备的布局
		ui->cw_uvc_content->show();
		ui->cw_rgb_camera_uvc->show();
		int layout_count = mUvcRgbContent.count();
		if (layout_count == 0)
		{
			mUvcRgbContent.addWidget(&mUvcRgbCamera);
			ui->cw_rgb_camera_uvc->addLayout(&mUvcRgbContent);
		}
		mUvcRgbCamera.clearDeviceList();
		for (int i = 0; i < devices_size; i++) {
			Uvc_Device uvc_device = mDevices.at(i);
			mUvcRgbCamera.setDevicesChange(false);
			mUvcRgbCamera.addDeviceList(uvc_device.deviceName);
			mUvcRgbCamera.setDevicesChange(true);
		}
		int device_index = mUvcRgbCamera.getCurrentDevices();
		Uvc_Device current_device = mDevices.at(device_index);
		QList<Device_Resolution> resolution_list = current_device.resolutions;
		mUvcRgbCamera.setResolutionChange(false);
		mUvcRgbCamera.resolutionClear();
		for (int j = 0; j < resolution_list.size(); j++) {
			Device_Resolution resolution = resolution_list.at(j);
			mUvcRgbCamera.addResolution(QString("%1_%2x%3_%4").arg(resolution.format).arg(resolution.width).arg(resolution.height).arg(resolution.fps));
		}
		mUvcRgbCamera.setCurrentResolution("MJPG_1280x720_30");
		mUvcRgbCamera.setResolutionChange(true);
	}
}
void MainWindowStyle::uvcDataCallback(int width, int height, unsigned char* data, int fps, QString format) {
	//qDebug()<<"howard width="<<width<<" height="<<height;
	if (mUvcRgbWindow != nullptr)
	{
		QImage image((uchar*)data, width, height, QImage::Format_RGB888);
		QImage previewImage = image.mirrored(false, true);
		mDeviceSensorBase->mColorStreamInfo->mOniDataInfo.mWidth = previewImage.width();
		mDeviceSensorBase->mColorStreamInfo->mOniDataInfo.mHeight = previewImage.height();
		mDeviceSensorBase->mColorStreamInfo->mOniDataInfo.mDataSizeBytes = previewImage.sizeInBytes();;
		mDeviceSensorBase->mColorStreamInfo->mOniDataInfo.oData.UpdataDate(reinterpret_cast<const int8_t*>(previewImage.bits()), mDeviceSensorBase->mColorStreamInfo->mOniDataInfo.mDataSizeBytes);
		mUvcRgbWindow->setFrameRate(fps);
		mDeviceSensorBase->captureUVCColor();
		if (mUvcColorRender)
		{
			mUvcRgbWindow->UpdateColorData(&mDeviceSensorBase->mColorStreamInfo->mOniDataInfo);
		}
		if (mAiStreamOpen == KOpen)
		{
			if (mAiStreamType == mAiStreamControl.kAi2DSkeleton)
			{
				mUvcRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
				mDeviceSensorBase->mAiSekeleton2d.mJointFormat = PixelFormat::PIXEL_FORMAT_NONE;
			}
			else if (mAiStreamType == mAiStreamControl.kAiFloorInfo) {
				mDeviceSensorBase->mMutexAiFrame.lock();
				QString str_center_x = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[0], 'f', 2);
				QString str_center_y = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[1], 'f', 2);
				QString str_center_z = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneCenter[2], 'f', 2);
				QString str_vector_x = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[0], 'f', 2);
				QString str_vector_y = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[1], 'f', 2);
				QString str_vector_z = QString::number(mDeviceSensorBase->mAiSekeleton2d.mPlaneVector[2], 'f', 2);

				mAiStreamControl.setPlaneCenter(QString("x:%1 y:%2 z:%3").arg(str_center_x).arg(str_center_y).arg(str_center_z));
				mAiStreamControl.setPlaneVector(QString("x:%1 y:%2 z:%3").arg(str_vector_x).arg(str_vector_y).arg(str_vector_z));
				mUvcRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
				mDeviceSensorBase->mAiSekeleton2d.mJointFormat = PixelFormat::PIXEL_FORMAT_NONE;
				mDeviceSensorBase->mMutexAiFrame.unlock();
			}
			else if (mAiStreamType == mAiStreamControl.kAiBodyShape)
			{
				mUvcRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);

			}
			else if (mAiStreamType == mAiStreamControl.kAiBodyMask)
			{
				//抠图
				mDeviceSensorBase->mMutexAiFrame.lock();
				mUvcRgbWindow->updata2DBody(mDeviceSensorBase->mAiSekeleton2d);
				//device_sensorbase->ai_sekeleton_2d.jointFormat = PixelFormat::PIXEL_FORMAT_NONE;
				mDeviceSensorBase->mMutexAiFrame.unlock();
			}
		}
	}
}
void MainWindowStyle::uvcdataStopRender() {

}
void MainWindowStyle::getUVCDevicesList() {

	connect(&mWmfFunction, SIGNAL(uvcDevice(QList<Uvc_Device>)), this, SLOT(UvcDevicesCallback(QList<Uvc_Device>)));
	connect(&mWmfFunction, SIGNAL(dataCallback(int, int, unsigned char*, int, QString)), this, SLOT(uvcDataCallback(int, int, unsigned char*, int, QString)));
	connect(&mWmfFunction, SIGNAL(stopRender()), this, SLOT(uvcdataStopRender()));
	mWmfFunction.initWMF();
}
void MainWindowStyle::onUVCColorChang(int type, QString value) {
	if (mUvcRgbCamera.kDevicesChange == type)
	{
		if (!mWmfFunction.getDeviceStatus())
		{
			return;
		}
		//切换uvc设备
		mWmfFunction.stopThread();
		QThread::msleep(2000);
		int device_index = mUvcRgbCamera.getCurrentDevices();
		Uvc_Device current_device = mUvcDevices.at(device_index);
		QList<Device_Resolution> resolution_list = current_device.resolutions;
		mUvcRgbCamera.setResolutionChange(false);
		mUvcRgbCamera.resolutionClear();
		for (int j = 0; j < resolution_list.size(); j++) {
			Device_Resolution resolution = resolution_list.at(j);
			mUvcRgbCamera.addResolution(QString("%1_%2x%3_%4").arg(resolution.format).arg(resolution.width).arg(resolution.height).arg(resolution.fps));
		}
		mUvcRgbCamera.setCurrentResolution("MJPG_1280x720_30");
		mUvcRgbCamera.setResolutionChange(true);
		//切换分辨率
		HRESULT hr = mWmfFunction.setResolution(mUvcRgbCamera.getCurrentDevices(), mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).indexId, mUvcDevices.at(mUvcRgbCamera.getCurrentResolution()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).width, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).height, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).fps, mUvcDevices.at(mUvcRgbCamera.getCurrentDevices()).resolutions.at(mUvcRgbCamera.getCurrentResolution()).format);
		if (SUCCEEDED(hr)) {
			qDebug() << "start preview";
			mWmfFunction.startThread();
		}
	}
	else if (mUvcRgbCamera.kResolutionChange == type)
	{
		if (!mWmfFunction.getDeviceStatus())
		{
			return;
		}
		//切换分辨率
		int device_index = mUvcRgbCamera.getCurrentDevices();
		int resolution_index = mUvcRgbCamera.getCurrentResolution();
		Uvc_Device uvc_device = mUvcDevices.at(device_index);
		int resolutions_size = mUvcDevices.at(device_index).resolutions.size();
		if (resolutions_size > resolution_index)
		{
			mWmfFunction.stopThread();
			QThread::msleep(2000);
			Device_Resolution device_resolution = mUvcDevices.at(device_index).resolutions.at(resolution_index);
			HRESULT hr = mWmfFunction.setResolution(device_index, device_resolution.indexId,
				device_resolution.width, device_resolution.height, device_resolution.fps, device_resolution.format);
			if (SUCCEEDED(hr)) {
				qDebug() << "start preview";
				mWmfFunction.startThread();
			}
		}

	}
}
int MainWindowStyle::uvcResolutionIndex(Device_Resolution resolution) {
	//查找uvc对应的分辨率索引
	int ret = -1;
	int device_index = mUvcRgbCamera.getCurrentDevices();
	Uvc_Device uvc_device = mUvcDevices.at(device_index);
	int resolutions_size = mUvcDevices.at(device_index).resolutions.size();
	for (int i = 0; i < resolutions_size; i++)
	{
		Device_Resolution resolution_index = mUvcDevices.at(device_index).resolutions.at(i);
		if (resolution_index.width == resolution.width && resolution_index.height == resolution.height && resolution_index.fps == resolution.fps && resolution_index.format.startsWith(resolution.format))
		{
			ret = i;
			break;

		}
	}
	return ret;
}
// 窗口关闭事件
void MainWindowStyle::closeEvent(QCloseEvent *event)
{
	mClosewindow = true;
	if (mUvcColorOpen == KOpen)
	{
		mWmfFunction.stopThread();
		if (mWmfFunction.getDeviceStatus())
		{
			mWmfFunction.closeDevice();
		}
		mUvcColorOpen = KClose;
	}
	if (nullptr != mDeviceSensorBase)
	{
		mDeviceSensorBase->CloseDevice();
	}
	if (mTuningWindow != nullptr) {
		delete mTuningWindow;
	}

	if (mDeviceSensorBase != nullptr) {
		delete mDeviceSensorBase;
	}

	for (int i = 0; i < mPngWindowList.size(); i++) {
		PngInfoWindow* pngInfoWindow = mPngWindowList.at(i);
		if (pngInfoWindow != NULL) {
			delete pngInfoWindow;
		}
	}
	QApplication::exit(0);
	closeONIVideo();
}

void MainWindowStyle::readFileBuffer(OniData * data) {
	////点云滤波使用
	//std::string cali_filename = "./config/DepthParams.bin";
	////加载标定文件并转为结构体
	//int cali_buffer_size = 0;
	//int ret = ObUniformV2CalcBufferSize(cali_filename, CRC_32_CHECKSUM_V2, cali_buffer_size);
	//if (0 != ret)
	//{
	//	qDebug() << "error:" << ret;
	//	return;
	//}
	//signed char* caliparam_buffer = (signed char*)malloc(cali_buffer_size * sizeof(signed char));
	//ret = ObUniformV2StructToBuffer(cali_filename, caliparam_buffer);
	//if (0 != ret)
	//{
	//	qDebug() << "error:" << ret;
	//	return;
	//}

	//BufferStruct(caliparam_buffer, &data->oData.mTofCaliParams);

	//free(caliparam_buffer);

}

void MainWindowStyle::settingCloseStream(bool status)
{
	if (mONIOpen == KOpen)
	{
		closeONIVideo();
	}
	if (mPhaseOpen = KOpen)
	{
		closePhaseVideo(true);
	}
	if (mDepthOpen = KOpen)
	{
		closeDepthVideo(true);
	}
	if (mIrOpen = KOpen)
	{
		closeIRVideo();
	}
}