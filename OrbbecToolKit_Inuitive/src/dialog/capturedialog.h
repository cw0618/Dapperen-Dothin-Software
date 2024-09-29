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

#ifndef CAPTUREDIALOG_H
#define CAPTUREDIALOG_H

#include <QMainWindow>
#include <QFileDialog>
#include <queue>
#include <mutex>
#include <atomic>
#include <qbuttongroup.h>
#include <QTimer>
#include <QSettings>
#include "src/orbbec/ObPng.h"
#include "src/calculate/calc.h"
#include"src/device/sensorbase.h"
#include "src/tempcontroler/tempcontroler.h"
#include "qfile.h"
#include"src/config/minIni.h"
#include "src\csv\csvfile.h"
#include "src/csv/csvbean.h"

#define CAPTURE_INI_FILE ".\\config\\CaptureConfig.ini"
#define TEMPERATURE_DRIFT_CAP ".\\config\\TemperatureDriftCap.ini"
#pragma pack(4)
/** \struct PngHolderInfo
* 图片存储的信息
*/
struct PngHolderInfo {
	int channels;
	int compression_level; /**< 压缩的比例 */

	int sensorId;
	uint8_t frameIndex; /**< 帧序号 */
	uint8_t groupIndex; /**< 组序号，没有组概念时为0 */

	DEVICE_INFO       dev;
	TOF_DRIVER_INFO   drv;
	TOF_ENV_INFO      env;
	TOF_DATA_INFO     data;
	TOF_IRSENSOR_INFO irSensor;
	DISTORTION_INFO   dist;
	OB_PNG_INFO       ob_info;

};
/**
*温漂采集的方式
*/
typedef enum
{
	KStop = 0, /**< 停止采集数据 */
	KManualMode = 1, /**< 手动方式采集数据 */
	kAutoMode = 2,  /**< 自动方式采集数据 */
} TemperatureModeType;

struct raw_parm
{
	int width;
	int height;
	uint32_t dist;

	float irFx;
	float irFy;
	float irCx;
	float irCy;

	raw_parm()
	{
		width = height = dist = 0;
		irFx = irFy = irCx = irCy = 0.0;
	}
};

namespace Ui {
	class CaptureDialog;
}

class CaptureDialog : public QMainWindow
{
	Q_OBJECT

public:
	explicit CaptureDialog(SensorBase *sensorbase_, QWidget *parent = nullptr);
	~CaptureDialog();

	void initialize();
	void startTimerCount();

signals:
	void SigInputDepthData();
	void SigInputPcData();
	void SigInputIrData();
	void SigInputPhaseData();
	void SigInputColorData();
	void SigInputCSVData();
	void setNextTempState(bool needSet, int offset);
	void SigShowInfo(QString);
	void captureChangeCallback(int type, QString value);

	public slots:
	void slotInputDepthData();
	void slotInputPcData();
	void slotInputIrData();
	void slotInputPhaseData();
	void OnFilePathClick();
	void OnSavePictureClick();
	void OnPtcCaptureClick();
	void OnDarkCurrentCaptureClick();
	void updateUIState();
	void slotShowInfo(QString);
	void onOneTimeFinish();

	void onCearLogReleased();
	void GetStartStatus();
	QString getCsvFileName(QString name);
	void saveCSVData(QString csv_name, QString file_path);
	void onRecordFinish();
	void onTempCaptureClicked();
	void onPcRadioClicked(bool);
	void onlogClearClicked(bool);
	void onPhaseRadioClicked(bool);
	void onButtonStatusClick();
	void onDepthRadioClicked(bool);
	void onTemperatureChange(int);
	void onTimeoutTempControl();

protected:
	void SaveDepthThread();
	void SavePcThread();
	void SaveIrThread();
	void SavePhaseThread();
	void SaveColorThread(int thread_id);

protected:

	void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

private:
	SensorBase *mSensorDevice;
	QString mSaveCsvPath;
	QFile *infoFile;
	QMutex m_qMutex;
	float mRxTemp{ 0.0 };
	float mTxTemp{ 0.0 };
	std::mutex mMutexPhase;
	std::mutex mMutexIR;
	std::mutex mMutexDepth;
	std::mutex mMutexCsv;
	std::string mDeviceName;
	std::string mRrModelName;
	std::string mIrSensorName;
	std::string mDeviceSerial; 
	std::string mIRSerial; 
	std::string mLdmpSerial;
	std::string mDepthEngine;
	std::string mVcselName;
	std::string mDriverVersion;
	CSVBean *mCsvBean{nullptr};
	int mCaptureDequeSize = 100;
	int mTimerId = -1;
	struct TempControlParams {
		bool isTempControl{ false };
		float temp_max{ 0 };
		float temp_min{ 0 };
		float temp_interval{ 0 };
		float temp_step{ -999.9f };
		float temp_judge_interval;
		int frameTotal;
		int frameStep;
		int distance;
		int timer_interval;
		QTimer *temp_timer{ NULL };
		bool isAutoTempControl{ false };
		float mCurrentRxTempr{ 0 };
		bool bFirstReadTempr{ false };
	};
	TempControlParams mTempControl;
	TempControler *mTempControler{ nullptr };
	QTimer *mCountTimer{ NULL };
	bool bRcordVideoState{ false };
	/**
	* CbSnapDepthData 回调函数，从sensor_base传会一帧深度数据。在这里保存数据，断开连接。
	* \param data
	*/
	void CbSnapDepthData(OniData &data);

	/**
	*CbSnapDepthData 回调函数，从sensor_base传会一帧深度数据。在这里保存数据，断开连接。
	* \param data
	*/
	void CbSnapPcData(OniData &data);

	/**
	* CbSnapDepthData 回调函数，从sensor_base传会一帧IR数据。在这里保存数据，断开连接。
	* \param data
	*/
	void CbSnapIRData(OniData &data);
	/**
	* CbSnapDepthData 回调函数，从sensor_base传会一帧IR数据。在这里保存数据，断开连接。
	* \param data
	*/
	void CbSnapPhaseData(OniData &data);
	/**
	* @brief CbSnapColorData 回调函数，从sensor_base传会一帧深度数据。在这里保存数据，断开连接。
	* \param data
	*/
	void CbSnapColorData(OniData &data);

	void CbSnapCsvForPhase(OniData &data);

	/**
	* @brief WriteRawData 用于保存裸数据的方法
	*
	* \param raw_name
    * 保存的路径和文件名
	*
	* \param data
	* 文件内容
	*
	* \param byte_size
	* 文件大小
	*/
	void WriteRawData(const char* raw_name, const int8_t* data, uint32_t byte_size);

	void WriteInteriParam2Raw(const char* raw_name, const raw_parm &rp);

	void CaptureDialog::ReadTemp();

	void CaptureDialog::ShowInfo(QString info);
	int createPNGForPhase(OniData &data, QString file_name, int distance, TOF_DATA_TYPE dataType);
	void csvDataArray(OniData &data, uint16_t *array, int num);
	void recordTempCsv();

private:
	Ui::CaptureDialog *ui;

	QDir mCurFileDir;

	QList<OniData> mDepthQue;
	QList<OniData> mPcQue;
	QList<OniData> mIrQue;
	QList<OniData> mPhaseQue;

	std::queue<OniData> mColorQue;
	std::queue<CSVBean> mPhaseCsvBean;
	
	std::mutex mMutex;
	std::mutex mMutexColor;
	std::mutex mMutexPc;
	QString mCsvItem = nullptr;
	bool mDepthThreadFinish{ true };
	bool mThreadFinish{ true };
	bool mPhaseThreadFinish{ true };
	bool mColorThreadFinish{ true };
	bool CsvThreadFinish{ true };
	bool nPcThreadFinish{ true };
	int mCurrentCaptureTime{ 0 };
	//CaptureThread mTimeThreadOut;
	uint32_t mLogIndex{ 0 };

	CameraInternalParam mCip;

	char* mFullNameOni{ nullptr };

	Calc* mCalc{ nullptr };

	tagPCColor mPCColorEnable;

	QString mRrTemp;
	QString mLdmpTemp;

	CSVFile *mCsvFile;

	int phaseGroupIndex{ -1 };
	QString phaseTimestamp;

	void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;

	QString path;
	QString mTimeCapturePath;
	/**
	*点云的保存格式
	*/
	enum PicSaveFormat {
		RAW,
		PNG,
		BMP,
		XYZ,
		PLY
	};
	PicSaveFormat depthFormat{ PicSaveFormat::PNG };
	PicSaveFormat irFormat{ PicSaveFormat::PNG };
	PicSaveFormat phaseFormat{ PicSaveFormat::PNG };
	PicSaveFormat colorFormat{ PicSaveFormat::PNG };
	PicSaveFormat pointCloudFormat{ PicSaveFormat::XYZ };
	QString distance;
	QButtonGroup *depthRadioGroup;
	QButtonGroup *phaseRadioGroup;
	QButtonGroup *pcRadioGroup;

};

#endif // CAPTUREDIALOG_H
