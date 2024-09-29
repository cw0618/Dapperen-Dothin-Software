#include "depthcontrolview.h"
#include "ui_depthcontrolview.h"
#include<QStyledItemDelegate>
#include <QDebug>
#include <qsettings.h>
/** \class DepthControlView
*
* 深度数据和属性的控制类
*
*/
DepthControlView::DepthControlView(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::DepthControlView)
{
	ui->setupUi(this);
	QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
	ui->cb_video_mode->setItemDelegate(itemDelegate);
	ui->switch_clound_point->setText("OFF", "ON");
	ui->cb_resolution_depth->setItemDelegate(itemDelegate);
	ui->comboBox_filter_bin->setItemDelegate(itemDelegate);
	connect(ui->cb_video_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(OnVideoModeChange(int)));
	connect(ui->cb_resolution_depth, SIGNAL(currentIndexChanged(int)), this, SLOT(OnResolutionChange(int)));
	connect(ui->switch_clound_point, SIGNAL(checkedChanged(bool)), this, SLOT(onPointCloundChanged(bool)));
	//connect(ui->bt_measure_height, SIGNAL(clicked()), this, SLOT(getMeasureHeight()));

	ui->spinBox->setMinimum(nMin);  // 最小值
	ui->spinBox->setMaximum(nMax);  // 最大值
	ui->spinBox->setSingleStep(nSingleStep);  // 步长
	ui->spinBox->setStyleSheet("color:white;");

	ui->hSlider_bar->setOrientation(Qt::Horizontal);  // 水平方向
	ui->hSlider_bar->setMinimum(nMin);  // 最小值
	ui->hSlider_bar->setMaximum(nMax);  // 最大值
	ui->hSlider_bar->setSingleStep(nSingleStep);  // 步长

	// 连接信号槽（相互改变）
	connect(ui->spinBox, SIGNAL(valueChanged(int)), ui->hSlider_bar, SLOT(setValue(int)));
	connect(ui->hSlider_bar, SIGNAL(valueChanged(int)), ui->spinBox, SLOT(setValue(int)));

	connect(ui->hSlider_bar, SIGNAL(valueChanged(int)), this, SLOT(OnSliderChange(int)));

	ui->hSlider_bar->setValue(8000);
	ui->hSlider_bar->setVisible(false);
	ui->spinBox->setVisible(false);

	connect(ui->tb_depth_distance_set, SIGNAL(clicked()), this, SLOT(OnDepthMaxDistance()));
	connect(ui->slider_depth_distance, SIGNAL(valueChanged(int)), this, SLOT(OnChangeSliderMaxValue(int)));
	ui->le_depth_distance_value->setValidator(new QIntValidator(0, 3000, this));
	ui->tb_depth_distance_set->setIcon(QIcon(":/res/image/write.png"));
	connect(ui->tb_depth_distance_set_min, SIGNAL(clicked()), this, SLOT(OnDepthMinDistance()));
	connect(ui->slider_depth_distance_min, SIGNAL(valueChanged(int)), this, SLOT(OnChangeSliderMinValue(int)));
	ui->le_depth_distance_value_min->setValidator(new QIntValidator(0, 3000, this));
	ui->tb_depth_distance_set_min->setIcon(QIcon(":/res/image/write.png"));
	//setDistance(false);
	setDepthEnable(false);
	//adPointFilterFromIni();
}

DepthControlView::~DepthControlView()
{
	delete ui;
}

void DepthControlView::OnVideoModeChange(int index) {
	QString text = ui->cb_video_mode->itemText(index);
	//qDebug() << "howard OnVideoModeChange=" << text << index;;
	if (text.contains("GRAY"))
	{
		ui->spinBox->setVisible(true);
		ui->hSlider_bar->setVisible(true);
	}
	else {
		ui->hSlider_bar->setVisible(false);
		ui->spinBox->setVisible(false);
	}

	emit depthChangeCallback(kVideoMode, text);
}
QString DepthControlView::getVideoMode() {
	int index = ui->cb_video_mode->currentIndex();
	QString text = ui->cb_video_mode->itemText(index);
	return text;
}
void DepthControlView::addResolution(QString value) {
	ui->cb_resolution_depth->addItem(value);
}
void DepthControlView::resolutionClear() {
	ui->cb_resolution_depth->clear();
}
void DepthControlView::setCurrentResolution(QString resolution) {
	//qDebug() << "howard setCurrentResolution=" << resolution;
	ui->cb_resolution_depth->setCurrentText(resolution);

}
int DepthControlView::getCurrentResolution() {
	int index = ui->cb_resolution_depth->currentIndex();
	return index;
}

void DepthControlView::setResolutionChange(bool change) {
	change_status = change;
}
void DepthControlView::OnResolutionChange(int index) {
	QString value_index(index);
	//    QString text = ui->cb_resolution_depth->itemText(index);
		//qDebug() << "howard OnResolutionChange=" << index;;
	if (change_status)
	{
		emit depthChangeCallback(kResolutionChange, value_index);
	}
}

void DepthControlView::OnSliderChange(int val)
{
	emit sliderChange(val);
}

void DepthControlView::GetFilterFileName()
{
	QDir dir("./config/");
	QStringList nameFilters;
	nameFilters << "*filter*.*n*";
	QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);

	if (files.empty()) {
		QMessageBox::warning(nullptr, "error", "Not filter file found, please check!");
		return;
	}


	ui->comboBox_filter_bin->blockSignals(true);
	ui->comboBox_filter_bin->clear();
	ui->comboBox_filter_bin->addItem("please select");
	ui->comboBox_filter_bin->addItems(files);

	qDebug() << "get filter bin :" << files;

	//qDebug() << currentFilterFile;
	//if (files.contains(currentFilterFile) && currentFilterFile != "")
	//{

	//	ui->comboBox_filter_bin->setCurrentText(currentFilterFile);

	//}
	//else if (!files.filter("default").empty()) {

	//	currentFilterFile = files.filter("default").at(0);
	//	ui->comboBox_filter_bin->setCurrentText(currentFilterFile);
	//	emit filterNameChange(currentFilterFile);

	//}
	//else {
	//	//当没有default时,取第一个
	//	currentFilterFile = files.at(0);
	//	ui->comboBox_filter_bin->setCurrentText(currentFilterFile);
	//	emit filterNameChange(currentFilterFile);
	//}

	ui->comboBox_filter_bin->setCurrentIndex(0);
	ui->comboBox_filter_bin->blockSignals(false);


	readPointFilterFromIni();


}
void DepthControlView::readPointFilterFromIni()
{
	float value1, value2;
	QSettings *configSet = new QSettings("./config/"+currentFilterFile, QSettings::IniFormat);
	value1 = configSet->value("FILTER_CONFIG/point_cloud_filter_statistical_max_slope", 80.0).toFloat();
	value2 = configSet->value("FILTER_CONFIG/point_cloud_filter_statistical_sigma", 2.0).toFloat();
	int value3 = 0;
	value3 = configSet->value("FILTER_CONFIG/point_cloud_filter_bypass", 1).toInt();
	ui->led_pointcloud_param1->setText(QString("%1").arg(value1));
	ui->led_pointcloud_param2->setText(QString("%1").arg(value2));
	pointCloudFilterParams[0] = value1;
	pointCloudFilterParams[1] = value2;
	
	pointCloudFilterParams[2] = value3;
	
	
	

	emit changeFilterParams(pointCloudFilterParams);
}
void DepthControlView::hidePointFilterSet()
{
	bool visiable = false;
	for (int i = 0; i < ui->horizontalLayout_6->count(); i++) {
		auto item = ui->horizontalLayout_6->itemAt(i);
		if (item && item->widget())
		{
			item->widget()->setVisible(visiable);
		}
	}

	for (int i = 0; i < ui->horizontalLayout_7->count(); i++) {
		auto item = ui->horizontalLayout_7->itemAt(i);
		if (item && item->widget())
		{
			item->widget()->setVisible(visiable);
		}
	}
}
void DepthControlView::setPointCloundStatus(bool status) {
	ui->switch_clound_point->setChecked(status);
}
void DepthControlView::onPointCloundChanged(bool state) {
	if (state)
	{
		pointCloudFilterParams[0] = ui->led_pointcloud_param1->text().toDouble();
		pointCloudFilterParams[1] = ui->led_pointcloud_param2->text().toDouble();
		emit changeFilterParams(pointCloudFilterParams);
		emit depthChangeCallback(kPointCloundSwitch, "1");

	}
	else {
		emit depthChangeCallback(kPointCloundSwitch, "0");
	}
}

void DepthControlView::on_comboBox_filter_bin_currentIndexChanged(const QString & text)
{
	if (text.contains("please select"))
	{
		return;
	}

	currentFilterFile = text;
	readPointFilterFromIni();
	emit filterNameChange(currentFilterFile);
}

void DepthControlView::setDepthEnable(bool state) {
	ui->tb_depth_distance_set->setEnabled(state);
	ui->le_depth_distance_value->setEnabled(state);
	ui->slider_depth_distance->setEnabled(state);
	ui->tb_depth_distance_set_min->setEnabled(state);
	ui->le_depth_distance_value_min->setEnabled(state);
	ui->slider_depth_distance_min->setEnabled(state);
}

void DepthControlView::OnDepthMaxDistance()
{
	QString distance_value = ui->le_depth_distance_value->text();
	int value = distance_value.toInt();
	//qDebug() << "howard setDepthDistance==" << value;
	mMaxDepthDistanceValue = value;
	ui->slider_depth_distance->setValue(value);
}

void DepthControlView::OnChangeSliderMaxValue(int value)
{
	mMaxDepthDistanceValue = value;
	QString str_value = QString::number(value);
	ui->le_depth_distance_value->setText(str_value);
	emit depthChangeCallback(kDepthColorMaxDistance, str_value);
}
void DepthControlView::OnDepthMinDistance()
{
	QString distance_value = ui->le_depth_distance_value_min->text();
	int value = distance_value.toInt();
	//qDebug() << "howard setDepthDistance==" << value;
	mMinDepthDistanceValue = value;
	ui->slider_depth_distance_min->setValue(value);
}
void DepthControlView::OnChangeSliderMinValue(int value)
{
	mMinDepthDistanceValue = value;
	QString str_value = QString::number(value);
	ui->le_depth_distance_value_min->setText(str_value);
	emit depthChangeCallback(kDepthColorMinDistance, str_value);
}