/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef MAINWINDOWSTYLE_H
#define MAINWINDOWSTYLE_H

#include <QMainWindow>
#include<QVBoxLayout>
#include<QHBoxLayout>
#include <QMap>
#include <QtGui\qevent.h>
#include <QQueue>
#include <QSignalMapper>
#include <QMimeData>
#include"src/weight/videowindow.h"
#include "src/device/onisensor.h"
#include "src/config/iniconfig.h"
#include "src/device/tuningwindow.h"
#include"src/colorcontrolview.h"
#include "src/dialog/capturedialog.h"
#include "src/dialog/streamtestdialog.h"
#include "src/dialog/aboutdialog.h"
#include "src/dialog/moredialog.h"
#include"src/device/sensorbase.h"
#include"src/phasecontrolview.h"
#include"src/depthcontrolview.h"
#include"src/weight/commoncontrolwidget.h"
#include"src/device/opendevicethread.h"
#include"src/weight/devicestatuswidget.h"
#include"src/ircontrolview.h"
#include"src/config/xmlconfig.h"
#include"src/weight/uvccolorwidget.h"
#include"src/device/wmffunction.h"
#include"src/opengl/mainGLWidget.h"
#include"src/weight/aistreamcontrol.h"
#include"3rd/OpenNI2/Include/tofinfo.h"
#include <QAbstractNativeEventFilter>
#include "pnginfowindow.h"
namespace Ui {
	class MainWindowStyle;
}

class MainWindowStyle : public QMainWindow, public QAbstractNativeEventFilter
{
	Q_OBJECT

public:
	explicit MainWindowStyle(QWidget *parent = nullptr);
	~MainWindowStyle();
	bool nativeEventFilter(const QByteArray &eventType, void *message, long *result);
public slots:
	void onPointCouldVideoCallback(int type, int value);
	void onDepthControlCallback(int type, QString value);
	void onLeftIrControlCallback(int type, QString value);
	void onPhaseResolutionCallback(int type, QString value);
	void onRgbResolutionCallback(int type, QString value);
	void onAiControlCallback(int type, QString value);
	int onDepthCameraSwitch(int type, int value);
	int onIrCameraSwitch(int type, int value);
	int onPhaseCameraSwitch(int type, int value);
	int onRGBCameraSwitch(int type, int value);

	int onAIStreamSwitch(int type, int value);
	int onUVCRGBSwitch(int type, int value);
	void onDepthVideoCallback(int type, int value);
	void onIrVideoCallback(int type, int value);
	void onOniVideoCallback(int type, int value);
	void onPhaseVideoCallback(int type, int value);
	void onCommonWidgetCallback(int type, QString value);
	void onRGBVideoCallback(int type, int value);
	void onUVCRGBVideoCallback(int type, int value);
	void onCaptureViewClick();
	void on2DWindowClick();
	void on3DWindowClick();
	void onsettingButtionClick();
	void onDeviceCloseClick();
	void onButtonInfoClick();
	void onButtonMoreClick();
	void onButtonAddDeviceClick();
	// 回调事件
	void CallbackConnectStatus(bool is_online);
	void DrawDepthVideo(OniData* data);
	void CallbackUpdateIr(OniData *data,int w,int h);
	void CallbackUpdatePhase(OniData *data,int w,int h);
	void CallbackUpdateColorData(OniData *data);
	void DeviceOpenStatus(int status);
	void UvcDevicesCallback(QList<Uvc_Device> mDevices);
	void uvcDataCallback(int width, int height, unsigned char* data, int fps, QString format);
	void uvcdataStopRender();
	void onUVCColorChang(int type, QString value);
	void onSliderChange(int value);
	void showPngWindow(QStringList& fileNames);
	void onChangeFilterParams(double* value);
	void onPngInfoWindowClose(void *handle);
	void settingCloseStream(bool status);
signals:
	void SigConnecte(bool is_online);
	void SigUpdateIr(OniData* data,int w,int h);
	void SigUpdatePhase(OniData* data,int w,int h);
	void SigUpdateColor(OniData* data);
    void SigDrawDepth(OniData *data);
	void SigUpdateOni(OniData* data);
protected:
	void dragEnterEvent(QDragEnterEvent *event) Q_DECL_OVERRIDE;
	void dropEvent(QDropEvent *event) Q_DECL_OVERRIDE;

private:
	Ui::MainWindowStyle *ui;
	std::string ORBBEC_CONFIG_INI = "./config/config.ini";
	int mFrameRGB = 0;
	int mFrameONI = 0;
	int mFrameIR = 0;
	int mFramePhase = 0;
	int mFrameDepth = 0;
	int mAiStreamType = 0;
	int SUCCESS_STATE = 0;

	const int kConnectingPage = 0;
	const int kAllClosePage = 1;
	const int kVideoPage = 2;
	const int kThreeDPage = 3;
	const int KOpen = 1;
	const int KClose = 0;
	int mStreamTestIndex = 0;
	int mRgbResolutionIndex = 0;
	int mIrResolutionIndex = 0;
	int mDepthResolutionIndex = 0;
	int mPhaseResolutionIndex = 0;
	IniConfig mInitConfig;
	XmlConfig mXmlConfig;
	bool mDeviceConnecting{ false };
	OpenDeviceThread mOpenDeviceThread;
	//PlayOni mPlayOniVideo;
	DeviceStatusWidget *mDeviceStatusWidget{ nullptr };

	VideoWindow * mDepthWindow{ nullptr };
	VideoWindow * mIRWindow{ nullptr };
	VideoWindow * mPhaseWindow{ nullptr };
	VideoWindow * mONIWindow{ nullptr };
	VideoWindow * mRgbWindow{ nullptr };
	VideoWindow * mUvcRgbWindow{ nullptr };
	QVBoxLayout *mVideoLayout{ nullptr };
	VideoWindow * mPointCouldWindow{ nullptr };
	QHBoxLayout *mVideoOneLayout{ nullptr };
	QHBoxLayout *VideoTwoLayout{ nullptr };
	tuningwindow *  mTuningWindow{ nullptr };
	StreamTestDialog *mStreamTestDialog{ nullptr };
	QVBoxLayout mUvcRgbContent;

	SensorBase * mDeviceSensorBase{ nullptr };
	Calc* mCalc{ nullptr };
	bool  mClosewindow{ false };
	WMFFunction mWmfFunction;
	QList<Uvc_Device> mUvcDevices;

	QVBoxLayout *mBoxPhase{ nullptr };
	QVBoxLayout *mBoxColor{ nullptr };
	QVBoxLayout *mBoxIr{ nullptr };
	QVBoxLayout *mBoxDepth{ nullptr };
	QVBoxLayout *mBoxCommon{ nullptr };
	QVBoxLayout mBoxAiAtream;
	QVBoxLayout mBoxRgbStream;

	CommonControlWidget *mCommonControl{ nullptr };
	DepthControlView *mDepthControl{ nullptr };
	IRControlView *mIrControl{ nullptr };
	PhaseControlView *mPhaseControl{ nullptr };
	ColorControlView *mColorControl{ nullptr };
	AIStreamControl mAiStreamControl;

	UVCColorWidget mUvcRgbCamera;
	std::vector<openni::SensorType> mSupportedSensors;
	DepthDrawModel  mDepthDrawMode;
	//10,11为在hb_video_one_layout的位置，20，21为在hb_video_two_layout的位置
	QMap<QString, int> mViewLocation;
	//0为关闭1为显示2为隐藏
	int mDepthOpen = 0;
	int mPointCouldOpen = 0;
	int mIrOpen = 0;
	int mPhaseOpen = 0;
	int mColorOpen = 0;
	int mAiStreamOpen = 0;
	int mUvcColorOpen = 0;
	int mONIOpen = 0;
	int mIRDistanceMin = 0;
	int mIRDistanceMax = 1800;
	//压测是否渲染
	bool mColorRender = true;
	bool mUvcColorRender = true;
	bool mDepthRender = true;
	bool mPhaseRender = true;
	bool mRrRender = true;
	bool mAmplitudeRender = true;
	bool mCommonD2CStatus = false;
	bool mD2CRenDer = false;
	bool mColorHideOpen = false;
	SensorType mOniVideoType;
	const QString kDepthLocationKey = "depth";
	const QString kIrLocationKey = "left_ir";
	const QString kPhaseLocationKey = "right_ir";
	const QString kRgbLocationKey = "rgb";
	const QString kPointCouldLocationKey = "clound";
	const QString kOniLocationKey = "oni";
	QWidget *mVideoContainer{ nullptr };
	/// 数据流支持的分辨率
	vector<string> mDthSpportMde;
	vector<string> mIrSupportMode;
	vector<string> mPhaseSupportMode;
	vector<string> mColorSupportMode;
	vector<string> mAiSupportMode;

	CaptureDialog *mCaptureDialog{ nullptr };
	int mRangeOfDepth{ 8000 };
	OBCameraParams mCameraParams;

	//png显示窗口
	QQueue<PngInfoWindow*> mPngWindowQueue;
	QList<PngInfoWindow*> mPngWindowList;

	bool mPointCloudFilterBuff{ false };
	float mPointCloudFilterParams[3];

private:
	void getUVCDevicesList();
	void firstStart();
	void getAllResolution();
	void SwithOnLineStatus(bool b);
	void CbSnapDepth(OniData &data);
	void CbSnapIR(OniData &data);
	void CbSnapPhase(OniData &data);
	void CbSnapColor(OniData &data);
	void CbSnapOni(OniData &data);
	void initStart();
	void initWindow();
	void initAction();
	int closeDepthVideo(bool stop_stream);
	int closePointCloudVideo(bool stop_stream);
	int closeIRVideo();
	int closePhaseVideo(bool stop_stream);
	int closeONIVideo();
	void setDepthMode(QString value);

	int closeRgbVideo(bool stop_stream);
	int closeUVCRgbVideo();
	int openUVCRgbVideo(QString resolution);
	void closeAllCamera();
	void depthControlLayout();
	void IrControlLayout();
	void phaseControlLayout();
	void aiControlLayout();
	void rgbControlLayout();
	int uvcResolutionIndex(Device_Resolution resolution);
	PixelFormat getPixelFormat(int index);
	void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;
	void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
	void getDeviceState();
	//int handleStreamTest(int index);
	void readFileBuffer(OniData * data);
	void registerUSBNotification();
};

#endif // MAINWINDOWSTYLE_H
