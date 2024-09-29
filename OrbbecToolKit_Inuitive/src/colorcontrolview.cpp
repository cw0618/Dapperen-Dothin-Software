#include "colorcontrolview.h"
#include "ui_colorcontrolview.h"
#include<QStyledItemDelegate>
#include <QDebug>
/** \class ColorControlView
*
* color数据流的控制类
*
*/
ColorControlView::ColorControlView(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::ColorControlView)
{
	ui->setupUi(this);
	QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
	ui->switch_d2c_render->setText("OFF", "ON");
	//ui->switch_plane_render->setText("OFF", "ON");
	ui->cb_resolution_rgb->setItemDelegate(itemDelegate);

	connect(ui->switch_d2c_render, SIGNAL(checkedChanged(bool)), this, SLOT(onD2CRenderChanged(bool)));
	//connect(ui->switch_plane_render, SIGNAL(checkedChanged(bool)), this, SLOT(onPlaneRenderChanged(bool)));
	connect(ui->cb_resolution_rgb, SIGNAL(currentIndexChanged(int)), this, SLOT(OnResolutionChange(int)));

	connect(ui->tb_d2c_distance_set, SIGNAL(clicked()), this, SLOT(setD2CDistance()));
	connect(ui->slider_d2c_distance, SIGNAL(valueChanged(int)), this, SLOT(ChangeSliderD2CValue(int)));
	ui->le_d2c_distance_value->setValidator(new QIntValidator(0, 9000, this));
	ui->tb_d2c_distance_set->setIcon(QIcon(":/res/image/write.png"));
	setDistance(false);
	setD2CEnable(false);
}

ColorControlView::~ColorControlView()
{
	delete ui;
}

void ColorControlView::addResolution(QString value) {
	ui->cb_resolution_rgb->addItem(value);
}
void ColorControlView::resolutionClear() {
	ui->cb_resolution_rgb->clear();
}
void ColorControlView::setResolutionIndex(int index) {
	ui->cb_resolution_rgb->setCurrentIndex(index);
}
void ColorControlView::setCurrentResolution(QString resolution) {
	//qDebug() << "howard setCurrentResolution=" << resolution;
	ui->cb_resolution_rgb->setCurrentText(resolution);
}
QString ColorControlView::getStrResolution() {
	return ui->cb_resolution_rgb->currentText();
}
int ColorControlView::getCurrentResolution() {
	return ui->cb_resolution_rgb->currentIndex();
}
void ColorControlView::setResolutionChange(bool change) {
	mChangeStatus = change;
}
void ColorControlView::OnResolutionChange(int index) {
	//    QString text = ui->cb_resolution_rgb->itemText(index);
	QString value_index = QString::number(index);
	//qDebug() << "howard OnResolutionChange="<< index;
	if (mChangeStatus)
	{
		emit rgbChangeCallback(kResolutionChange, value_index);
	}
}
void ColorControlView::onD2CRenderChanged(bool state) {

	mD2cState = state;
	if (state)
	{
		setDistance(true);
		emit rgbChangeCallback(kD2cRenderChange, "1");
		mD2cState = true;
	}
	else {
		setDistance(false);
		emit rgbChangeCallback(kD2cRenderChange, "0");
		mD2cState = false;
	}

}
void ColorControlView::onPlaneRenderChanged(bool state) {

	mPlaneState = state;
	if (state)
	{
		emit rgbChangeCallback(kPlaneRenderChange, "1");
	}
	else {
		emit rgbChangeCallback(kPlaneRenderChange, "0");
	}

}
void ColorControlView::setD2CStatus(int value) {

	if (value == 0)
	{
		ui->switch_d2c_render->setChecked(false);
		setDistance(false);
		mD2cState = false;
	}
	else {
		ui->switch_d2c_render->setChecked(true);
		setDistance(true);
		mD2cState = true;
	}

}
bool ColorControlView::getD2CState() const {

	return mD2cState;
}
void ColorControlView::setDistance(bool state) {
	ui->tb_d2c_distance_set->setEnabled(state);
	ui->le_d2c_distance_value->setEnabled(state);
	ui->slider_d2c_distance->setEnabled(state);
}
void ColorControlView::setD2CEnable(bool enable)
{
	ui->switch_d2c_render->setEnabled(enable);
	if (!enable) {
		setD2CStatus(0);
	}

}
void ColorControlView::setD2CDistance() {
	QString distance_value = ui->le_d2c_distance_value->text();
	int value = distance_value.toInt();
	//qDebug() << "howard setDepthDistance==" << value;
	mD2cDistanceValue = value;
	ui->slider_d2c_distance->setValue(value);

}
void ColorControlView::ChangeSliderD2CValue(int value)
{
	//qDebug() << "howard ChangeSliderDepthValue==" << value;
	mD2cDistanceValue = value;
	QString str_value = QString::number(value);
	ui->le_d2c_distance_value->setText(str_value);
	emit rgbChangeCallback(kD2cDistanceChange, str_value);
}
int ColorControlView::getD2CDistance() {
	return mD2cDistanceValue;
}
