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
#ifndef TUNINGWINDOW_H
#define TUNINGWINDOW_H

#include "sensorbase.h"
#include "src/device/mx6000.h"
#include "src/calculate/calc.h"
#include "src/device/sensorbase.h"
#include "src/config/GBK.h"
#include"3rd/OpenNI2/Include/tofinfo.h"
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMainWindow>
#include <QButtonGroup>
using namespace openni;
#include "src/cmd/widgetcmd.h"

namespace Ui {
	class tuningwindow;
}
typedef struct {
	int illumPower;
	uint32_t dacA;
	uint32_t dacB;
}DACBean;
typedef struct {
	uint16_t addr;
	uint8_t value;
}RegConfig;
class tuningwindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit tuningwindow(SensorBase *sensorbase_, QWidget *parent = nullptr);
	virtual ~tuningwindow();
	
	bool SendGeneralFile(openni::Device& device, const std::string& filePath, const uint16_t category, const uint16_t suffix);

	void ThreadForSendFile(QString filename);
	void ThreadForSendCommonFile(QString filename);
	void InitializeStatus();

	//当前tof sensor配置项
	bool isNeedSetDefaultConfig{false};
private:
	Ui::tuningwindow *ui;
	void updateLaserComb();

	bool setTxAbPower(uint16_t aState, uint16_t bState);
	bool getTxAbPower(uint16_t *aState, uint16_t *bState);
	SensorBase *mSensorDeviceBean;
	//shared_ptr<sensorbase> oni_sensor_;
	int mTimerId;
	std::string mSensorName;
	Calc* mCalcBean{ nullptr };
	uint32_t mLogIndex{ 0 };
	int current_vcsel_on_num{ 1 };
	void Initialize();
	void GetStartStatus();
	QString currentFilterFile;
	const QString kTunningInfoFile = ".\\config\\TunningCfg.ini";
	ObcFrequencyMode freqMode{ OBC_SINGLE_FREQUENCY };
	int mSensorId{ 0 };
	ObcSensorInfo sensorInfo{ 0 };
	// 调制频率
	uint32_t freq0{ 0 };
	uint32_t freq1{ 0 };
	// 占空比
	uint32_t duty0{ 0 };
	uint32_t duty1{ 0 };
	int mPhaseCount = 4;
	ObcTofOutMode out_mode{ OB_TOF_OUT_MODE_A_B };
	uint32_t pd_value_from_frame{ 0 };
	QList<float> duty_1;
	QList<float> duty_1_strip;
	QList<float> duty_2;
	QList<float> duty_2_strip;
	WidgetCmd * widgetCmd{ nullptr };
	bool isModifiConfig{ false };
	DACBean m_dac{ 0 };
	QMap<QString, float> mDefaultList;
	QButtonGroup *mFreqRadioGroup;

signals:
	void SigSetingShowInfo(QString);
	void SignalsCloseStream(bool status);
public slots:
	void on_checkbox_laser_toggled(bool checked);
	void on_checkbox_ldp_toggled(bool checked);
	void on_checkbox_flood_toggled(bool checked);
	void ButtonClicked();
	void slotsPowerTest(bool checked);
	void slotsAeDothin(bool checked);
	void slotsEEproWrite();
	void slotsEEproRead();
	void slotsGroupBoxLaser(bool checked);
	void on_btn_clear_log_released();
	void on_btn_choose_firm_bin_clicked();
	void on_btn_choose_d2c_bin_clicked();
	void on_btn_choose_calib_bin_clicked();
	void on_btn_choose_common_bin_clicked();
	void on_btn_choose_register_clicked();
	void onFilterNameChange(QString text);
	void onShowInfo(QString);
	void OnDutyOneChange(int index);
	void OnDytyTwoChange(int index);
	void OnOutModelChange(int index);
	void ShowSettingInfo(QString info);
};

#endif // TUNINGWINDOW_H
