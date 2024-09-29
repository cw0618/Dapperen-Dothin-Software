#include "videowindow.h"
#include "ui_videowindow.h"
/** \class VideoWindow
*
* 数据流显示窗口
*
*/
VideoWindow::VideoWindow(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::VideoWindow)
{
	ui->setupUi(this);
	connect(ui->tb_video_full_screen, SIGNAL(clicked()), this, SLOT(OnFullScreenSwitch()));
	connect(ui->tb_video_close, SIGNAL(clicked()), this, SLOT(OnVideoCloseSwitch()));
	connect(ui->tb_video_capture, SIGNAL(clicked()), this, SLOT(OnCaptureClick()));
	connect(ui->tb_video_play, SIGNAL(clicked()), this, SLOT(OnPauseClick()));
}

VideoWindow::~VideoWindow()
{
	delete ui;
}
void VideoWindow::setVideoTitle(QString title) {
	ui->label_camera_sn->setText(title);
}
bool VideoWindow::getFullScreenStatus() {
	return mIsFullScreen;
}
void VideoWindow::setDepthMode(DepthDrawModel depthMode) {
	mDepthDrawMode = depthMode;

}
void VideoWindow::setD2CStart(bool state) {
	d2cStart = state;
	ui->gl_color_render->setD2CStart(state);
}
void VideoWindow::setAIBodyShape(bool state) {
	ui->gl_color_render->ai_body_shape = state;
}
void VideoWindow::setStreamMode(int mode) {

	const int STR_UVC_COLOR = 26;
	mStreamMode = mode;
}
void VideoWindow::OnFullScreenSwitch() {
	if (mIsFullScreen) {
		//关闭全屏
		mIsFullScreen = false;
		ui->tb_video_full_screen->setIcon(QIcon(":/res/image/single_screen.png"));
		emit videoViewCallback(kVideoFullScreenType, 0);
	}
	else
	{
		//打开全屏
		mIsFullScreen = true;
		ui->tb_video_full_screen->setIcon(QIcon(":/res/image/full_screen.png"));
		emit videoViewCallback(kVideoFullScreenType, 1);
	}

}
void VideoWindow::OnVideoCloseSwitch() {
	emit videoViewCallback(kVideoCloseSwitchType, 0);
}
void VideoWindow::OnCaptureClick() {
	emit videoViewCallback(kVideoCaptureType, 0);
}
void VideoWindow::OnPauseClick() {
	if (mIsPause)
	{
		mIsPause = false;
		ui->tb_video_play->setIcon(QIcon(":/res/image/video_pause.png"));
		emit videoViewCallback(kVideoPauseType, 0);
	}
	else
	{
		mIsPause = true;
		ui->tb_video_play->setIcon(QIcon(":/res/image/video_play.png"));
		emit videoViewCallback(kVideoPauseType, 1);
	}
}

void VideoWindow::updateDepthData(OniData* data) {
	int8_t* depth_ptr = nullptr;
	QString mouse_info;
	QPoint mp = ui->gl_color_render->GetMousePos();
	if (mp.x() >= 0 && mp.y() >= 0)
	{
		uint32_t index = mp.y() * data->mWidth + mp.x();
		int dataSize = data->mWidth* data->mHeight;
		if ((index <= dataSize) && (data->oData.mDataDepthPtr != nullptr))
		{
			uint16_t depth_value = reinterpret_cast<uint16_t*>(data->oData.mDataDepthPtr)[index];
			mouse_info = QString("Depth(x:%1 y:%2 z:%3) resolution %4x%5").arg(mp.x()).arg(mp.y()).arg(depth_value).arg(data->mWidth).arg(data->mHeight);
		}
	}
	else
	{
		mouse_info = QString("resolution %1x%2").arg(data->mWidth).arg(data->mHeight);
	}
	if (DepthDrawModel::HISTOGRAM == mDepthDrawMode) {
		depth_ptr = data->oData.mDataHistogram888Ptr;
	}
	else if (DepthDrawModel::GRAY == mDepthDrawMode) {

		depth_ptr = data->oData.mData888bitPtr;
	}
	else if (DepthDrawModel::RAINBOW == mDepthDrawMode) {

		depth_ptr = data->oData.mDataRainbow888Ptr;
	}

	TextInfo ti_mouse_info;
	ti_mouse_info.penColor = Qt::green;
	
	if (nullptr != depth_ptr)
	{
		TextInfo ti_mouse_info;
		ti_mouse_info.penColor = Qt::green;
		ti_mouse_info.text = mouse_info;
		ui->gl_color_render->updateTextureBufferData(depth_ptr, data->mWidth, data->mHeight, 3 * data->mWidth* data->mHeight);
		ui->gl_color_render->UpdateMouseInfo(ti_mouse_info);
	}
}
void VideoWindow::updatePointCouldData(OniData* data) {
	ui->gl_color_render->updatePointCloudData(data->oData.mDataPointCouldPtr, data->mWidth*data->mHeight);
	ui->gl_color_render->update();
}
void VideoWindow::UpdateIrData(OniData *data,int w,int h, RenderType renderType) {
	QString mouse_info;
	QPoint mp = ui->gl_color_render->GetMousePos();
	if (mp.x() >= 0 && mp.y() >= 0 && mp.x() < data->mWidth && mp.y() < data->mHeight)
	{
		uint32_t index = mp.y() * data->mWidth + mp.x();
		int dataSize = data->mWidth* data->mHeight;
		if (index <= dataSize)
		{
			mIR16bit = reinterpret_cast<int16_t*>(data->oData.mDataPtr)[index];
			
		}
		mouse_info = QString("IR(x:%1 y:%2) 16bit:%3 resolution %4x%5").arg(mp.x()).arg(mp.y()).arg(mIR16bit).arg(data->mWidth).arg(data->mHeight);
	}
	else
	{
		mouse_info = QString("resolution %1x%2").arg(data->mWidth).arg(data->mHeight);
	}

	if (2 == data->mPixelInBytes)
	{
		if (nullptr != data->oData.mData888bitPtr)
		{
			if (nullptr != data->oData.mDataPtr)
			{
				mIrColorData = *data;         /// 备份，用于离线或暂停模式下查看鼠标点位置
				TextInfo ti_mouse_info;
				ti_mouse_info.penColor = Qt::green;

				ti_mouse_info.text = mouse_info;
				ui->gl_color_render->updateTextureBufferData(data->oData.mData888bitPtr, data->mWidth, data->mHeight, 3 * data->mWidth* data->mHeight);
				ui->gl_color_render->UpdateMouseInfo(ti_mouse_info);
			}
		}
	}
}
void VideoWindow::setAIRenderType(RenderType type) {
	ui->gl_color_render->setAIRenderType(type);
}
void VideoWindow::updata2DBody(AIBody& body) {
	ui->gl_color_render->updata2DBody(body);
}
void VideoWindow::UpdateD2CData(int8_t* raw16, int w, int h, int size) {
	ui->gl_color_render->updateDepthBufferData(raw16, w, h, size);

}
void VideoWindow::UpdateColorData(OniData *data) {
	QString mouse_info;
	if (mStreamMode == kStrColor)
	{
		mouse_info = QString("resolution %1x%2").arg(data->mWidth).arg(data->mHeight);
	}
	else if (mStreamMode == kStrUvcColor)
	{
		mouse_info = QString("resolution %3x%4").arg(data->mWidth).arg(data->mHeight);
	}
	if (data->oData.SizeOfData() != (3 * data->mWidth* data->mHeight))
	{
		return;
	}
	if (nullptr != data->oData.mDataPtr)
	{
		mIrColorData = *data;         /// 备份，用于离线或暂停模式下查看鼠标点位置
		TextInfo ti_mouse_info;
		ti_mouse_info.penColor = Qt::green;
		ti_mouse_info.text = mouse_info;
		ui->gl_color_render->updateTextureBufferData(data->oData.mDataPtr, data->mWidth, data->mHeight, data->oData.SizeOfData());
		ui->gl_color_render->UpdateMouseInfo(ti_mouse_info);
	}
}
void VideoWindow::setFrameRate(int frame_rate) {
	QString fps_depth = QString("fps: ") + QString::number(frame_rate);
	TextInfo ti_depth;
	ti_depth.penColor = Qt::green;
	ti_depth.text = fps_depth;
	ui->gl_color_render->UpdateFPS(ti_depth);
}
void VideoWindow::showCaptureFinishTip() {
	mMessageTip = new MessageTip(this);
	QPoint point_ = ui->tb_video_capture->mapFromGlobal(QCursor::pos());
	int point_x = ui->tb_video_capture->geometry().x() - mMessageTip->width() * 2 / 3;
	int point_y = ui->tb_video_capture->geometry().y() + ui->tb_video_capture->geometry().height() + 10;
	mMessageTip->setAttribute(Qt::WA_StyledBackground, true);
	mMessageTip->setMessageTip("capture success");
	mMessageTip->setGeometry(point_x, point_y, mMessageTip->width(), mMessageTip->height());
	mMessageTip->show();
	// 启动定时器, 指定时间间隔(毫秒）
	mTimerId = startTimer(2000);
}
// 定时器处理函数
void VideoWindow::timerEvent(QTimerEvent * event)
{
	if (event->timerId() == mTimerId) {

		if (mMessageTip != nullptr) {
			mMessageTip->hide();
			delete mMessageTip;
			mMessageTip = nullptr;
		}
		killTimer(mTimerId);
	}

}
