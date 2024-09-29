#include "commoncontrolview.h"
#include "ui_commoncontrolwidget.h"
#include "commoncontrolwidget.h"
#include<QStyledItemDelegate>
#include <QDebug>
#include<qtoolbutton.h>
#include<QRegExpValidator>
#include<QDateTime>
/** \class CommonControlWidget
*
* 公共属性的设置和获取
*
*/
CommonControlWidget::CommonControlWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::CommonControlWidget)
{
	ui->setupUi(this);
	//ui->switch_laser->setText("OFF", "ON");
	//ui->switch_flood->setText("OFF", "ON");
	//ui->switch_background->setText("OFF", "ON");
	ui->switch_dsleep->setText("OFF", "ON");
	ui->switch_hdr->setText("OFF", "ON");
	ui->switch_hist->setText("OFF", "ON");
	ui->switch_median->setText("OFF", "ON");
	ui->switch_ebc->setText("OFF", "ON");
	ui->switch_lsc->setText("OFF", "ON");
	ui->switch_dgainen->setText("OFF", "ON");
	ui->switch_correction->setText("OFF", "ON");
	//ui->switch_laser->setChecked(true);
	//ui->switch_flood->setChecked(true);
	//ui->switch_background->setChecked(false);
	ui->switch_dsleep->setChecked(false);
	ui->switch_hdr->setChecked(false);
	ui->switch_hist->setChecked(false);
	ui->switch_median->setChecked(false);
	ui->switch_ebc->setChecked(false);
	ui->switch_lsc->setChecked(false);
	ui->switch_dgainen->setChecked(false);
	ui->switch_correction->setChecked(false);
	//connect(ui->switch_laser, SIGNAL(checkedChanged(bool)), this, SLOT(onLaserStateChanged(bool)));
	//connect(ui->switch_flood, SIGNAL(checkedChanged(bool)), this, SLOT(onFloodStateChanged(bool)));
	//connect(ui->switch_background, SIGNAL(checkedChanged(bool)), this, SLOT(onBackgrounpStateChanged(bool)));
	connect(ui->switch_dsleep, SIGNAL(checkedChanged(bool)), this, SLOT(onDsleepChanged(bool)));
	connect(ui->switch_hdr, SIGNAL(checkedChanged(bool)), this, SLOT(onHdrChanged(bool)));
	connect(ui->switch_hist, SIGNAL(checkedChanged(bool)), this, SLOT(onHistChanged(bool)));
	connect(ui->switch_median, SIGNAL(checkedChanged(bool)), this, SLOT(onMedianChanged(bool)));
	connect(ui->switch_ebc, SIGNAL(checkedChanged(bool)), this, SLOT(onEbcChanged(bool)));
	connect(ui->switch_lsc, SIGNAL(checkedChanged(bool)), this, SLOT(onLscChanged(bool)));
	connect(ui->switch_dgainen, SIGNAL(checkedChanged(bool)), this, SLOT(onDgainChanged(bool)));
	connect(ui->switch_correction, SIGNAL(checkedChanged(bool)), this, SLOT(onRowCorrectionChanged(bool)));

	//按钮
	connect(ui->tb_software_reset, SIGNAL(clicked()), this, SLOT(onSoftwareResetClick()));
	connect(ui->tb_hardware_reset, SIGNAL(clicked()), this, SLOT(onHardwareResetClick()));
	connect(ui->tb_window, SIGNAL(clicked()), this, SLOT(onWindowSet()));
	//connect(ui->tb_gain_set, SIGNAL(clicked()), this, SLOT(OnIRGainSet()));
	//connect(ui->slider_gain, SIGNAL(valueChanged(int)), this, SLOT(OnChangeSliderIRGain(int)));

	//下拉框
	connect(ui->cb_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(onModeChanged(int)));
	connect(ui->cb_bining_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(onBiningModeChanged(int)));
	connect(ui->cb_flip, SIGNAL(currentIndexChanged(int)), this, SLOT(onFlipModeChanged(int)));
	connect(ui->cb_subsamp, SIGNAL(currentIndexChanged(int)), this, SLOT(onSubSampChanged(int)));
	connect(ui->cb_subsamp_v, SIGNAL(currentIndexChanged(int)), this, SLOT(onSubSampvChanged(int)));
	connect(ui->cb_gain, SIGNAL(currentIndexChanged(int)), this, SLOT(onGainChanged(int)));
}

CommonControlWidget::~CommonControlWidget()
{
	delete ui;
}

//void CommonControlWidget::setLaserStatus(bool value) {
//
//	mLaserState = value;
//	ui->switch_laser->blockSignals(true);
//	if (value){
//		ui->switch_laser->setChecked(true);
//	}
//	else {
//		ui->switch_laser->setChecked(false);
//	}
//	ui->switch_laser->blockSignals(false);
//}

//void CommonControlWidget::setFloodStatus(bool value) {
//
//	mLaserState = value;
//	ui->switch_flood->blockSignals(true);
//	if (value){
//		ui->switch_flood->setChecked(true);
//	}
//	else {
//		ui->switch_flood->setChecked(false);
//	}
//	ui->switch_flood->blockSignals(false);
//}

//void CommonControlWidget::setBackgrounpStatus(bool value) {
//
//	mBackgrounpState = value;
//	ui->switch_background->blockSignals(true);
//	if (value){
//		ui->switch_background->setChecked(true);
//	}
//	else {
//		ui->switch_background->setChecked(false);
//	}
//	ui->switch_background->blockSignals(false);
//}

//槽函数
//void CommonControlWidget::onFloodStateChanged(bool state) {
//
//	bool mFloodState = state;
//	if (state)
//	{
//		emit commonWidgetChangeCallback(kCommonFloodChange, "1");
//	}
//	else {
//		emit commonWidgetChangeCallback(kCommonFloodChange, "0");
//	}
//}

//void CommonControlWidget::onLaserStateChanged(bool state) {
//
//	bool mLaserState = state;
//	if (state)
//	{
//		emit commonWidgetChangeCallback(kCommonLaserChange, "1");
//	}
//	else {
//		emit commonWidgetChangeCallback(kCommonLaserChange, "0");
//	}
//}

//void CommonControlWidget::onBackgrounpStateChanged(bool state) {
//
//	mBackgrounpState = state;
//	if (state)
//	{
//		emit commonWidgetChangeCallback(kCommonBackgrounpChange, "1");
//	}
//	else {
//		emit commonWidgetChangeCallback(kCommonBackgrounpChange, "0");
//	}
//}

void CommonControlWidget::onModeChanged(int mode) {

	QString str_value = QString::number(mode);
	emit commonWidgetChangeCallback(kCommonModeChange, str_value);

}

void CommonControlWidget::onBiningModeChanged(int mode) {
	QString str_value = QString::number(mode);
	emit commonWidgetChangeCallback(kCommonBiningModeChange, str_value);
}

void CommonControlWidget::onFlipModeChanged(int mode) {
	QString str_value = QString::number(mode);
	emit commonWidgetChangeCallback(kCommonFlipModeChange, str_value);
}

void CommonControlWidget::onSubSampChanged(int value) {
	QString str_value = QString::number(value);
	emit commonWidgetChangeCallback(kCommonSubSampChange, str_value);
}

void CommonControlWidget::onSubSampvChanged(int value) {
	QString str_value = QString::number(value);
	emit commonWidgetChangeCallback(kCommonSubSampvChange, str_value);
}

void CommonControlWidget::onGainChanged(int value) {
	QString str_value = QString::number(value);
	emit commonWidgetChangeCallback(kCommonGainChange, str_value);
}

//void CommonControlWidget::OnIRGainSet()
//{
//	QString gain_value = ui->le_gain_value->text();
//	int value = gain_value.toInt();
//	mIRGain = value;
//	ui->slider_gain->setValue(value);
//}
//
//bool CommonControlWidget::IRGainSet(int value)
//{
//	mIRGain = value;
//	QString str_value = QString::number(value);
//	ui->le_gain_value->setText(str_value);
//	ui->slider_gain->blockSignals(true);
//	ui->slider_gain->setValue(value);
//	ui->slider_gain->blockSignals(false);
//	return true;
//}
//
//void CommonControlWidget::OnChangeSliderIRGain(int value)
//{
//	mIRGain = value;
//	QString str_value = QString::number(value);
//	ui->le_gain_value->setText(str_value);
//	emit commonWidgetChangeCallback(kCommonGainChange, str_value);
//}

void CommonControlWidget::onDgainChanged(bool state) {
	QString OddDgain = ui->le_dgainval0->text();
	if (state)
	{
		emit commonWidgetChangeCallback(kCommonOddDgainChange, OddDgain);
	}
	else {
		emit commonWidgetChangeCallback(kCommonOddDgainChange, "20");
	}
}

void CommonControlWidget::onSoftwareResetClick()
{
	emit commonWidgetChangeCallback(kCommonSoftwareReset, "1");
}

void CommonControlWidget::onHardwareResetClick()
{
	emit commonWidgetChangeCallback(kCommonHardwareReset, "1");
}

void CommonControlWidget::onWindowSet()
{
	QString originy = ui->le_originy->text();
	emit commonWidgetChangeCallback(kCommonWindowOriginyChange, originy);
	QString originx = ui->le_originx->text();
	emit commonWidgetChangeCallback(kCommonWindowOriginxChange, originx);
	QString height = ui->le_height->text();
	emit commonWidgetChangeCallback(kCommonWindowHeightChange, height);
	QString width = ui->le_width->text();
	emit commonWidgetChangeCallback(kCommonWindowWidthChange, width);
}

void CommonControlWidget::onDsleepChanged(bool state) {
	if (state){
		emit commonWidgetChangeCallback(kCommonDsleepChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonDsleepChange, "0");
	}
}

void CommonControlWidget::onHdrChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonHdrChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonHdrChange, "0");
	}
}

void CommonControlWidget::onHistChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonHistChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonHistChange, "0");
	}
}

void CommonControlWidget::onMedianChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonMedianChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonMedianChange, "0");
	}
}

void CommonControlWidget::onEbcChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonEbcChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonEbcChange, "0");
	}
}

void CommonControlWidget::onLscChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonLscChange, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonLscChange, "0");
	}
}

void CommonControlWidget::onRowCorrectionChanged(bool state) {
	if (state) {
		emit commonWidgetChangeCallback(kCommonRowCorrection, "1");
	}
	else {
		emit commonWidgetChangeCallback(kCommonRowCorrection, "0");
	}
}