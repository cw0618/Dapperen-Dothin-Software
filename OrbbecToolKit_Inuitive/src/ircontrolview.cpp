#include "ircontrolview.h"
#include "ui_ircontrolview.h"
#include<QStyledItemDelegate>
/** \class IRControlView
*
* IR数据和属性的控制类
*
*/
IRControlView::IRControlView(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::IRControlView)
{
	ui->setupUi(this);
	QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
	ui->cb_resolution_ir->setItemDelegate(itemDelegate);
	connect(ui->cb_resolution_ir, SIGNAL(currentIndexChanged(int)), this, SLOT(OnResolutionChange(int)));

	connect(ui->tb_ir_distance_set, SIGNAL(clicked()), this, SLOT(OnIRMaxDistance()));
	connect(ui->slider_ir_distance, SIGNAL(valueChanged(int)), this, SLOT(OnChangeSliderIRMax(int)));
	ui->le_ir_distance_value->setValidator(new QIntValidator(0, 2000, this));
	ui->tb_ir_distance_set->setIcon(QIcon(":/res/image/write.png"));
	connect(ui->tb_ir_distance_set_min, SIGNAL(clicked()), this, SLOT(OnIRMinDistance()));
	connect(ui->slider_ir_distance_min, SIGNAL(valueChanged(int)), this, SLOT(OnChangeSliderIRMin(int)));
	ui->le_ir_distance_value_min->setValidator(new QIntValidator(0, 2000, this));
	ui->tb_ir_distance_set_min->setIcon(QIcon(":/res/image/write.png"));
}

IRControlView::~IRControlView()
{
	delete ui;
}
void IRControlView::addResolution(QString value) {
	ui->cb_resolution_ir->addItem(value);
}
void IRControlView::resolutionClear() {
	ui->cb_resolution_ir->clear();
}
void IRControlView::setCurrentResolution(QString resolution) {
	//qDebug() << "howard setCurrentResolution=" << resolution;
	ui->cb_resolution_ir->setCurrentText(resolution);

}
int IRControlView::getCurrentResolution() {
	int index = ui->cb_resolution_ir->currentIndex();
	return index;
}
void IRControlView::setResolutionChange(bool change) {
	mChangeStatus = change;
}
void IRControlView::OnResolutionChange(int index) {
	QString value_index(index);
	//    QString text = ui->cb_resolution_depth->itemText(index);
	//qDebug() << "howard OnResolutionChange=" << index;;
	if (mChangeStatus)
	{
		emit IRChangeCallback(mResolutionChange, value_index);
	}
}

void IRControlView::OnIRMaxDistance()
{
	QString distance_value = ui->le_ir_distance_value->text();
	int value = distance_value.toInt();
	mMaxIRDistanceValue = value;
	ui->slider_ir_distance->setValue(value);
}

void IRControlView::OnIRMinDistance()
{
	QString distance_value = ui->le_ir_distance_value_min->text();
	int value = distance_value.toInt();
	//qDebug() << "howard setDepthDistance==" << value;
	mMinIRDistanceValue = value;
	ui->slider_ir_distance_min->setValue(value);
}

void IRControlView::OnChangeSliderIRMax(int value)
{
	mMaxIRDistanceValue = value;
	QString str_value = QString::number(value);
	ui->le_ir_distance_value->setText(str_value);
	emit IRChangeCallback(mIRMaxDistance, str_value);
}

void IRControlView::OnChangeSliderIRMin(int value)
{
	mMinIRDistanceValue = value;
	QString str_value = QString::number(value);
	ui->le_ir_distance_value_min->setText(str_value);
	emit IRChangeCallback(mIRMinDistance, str_value);
}