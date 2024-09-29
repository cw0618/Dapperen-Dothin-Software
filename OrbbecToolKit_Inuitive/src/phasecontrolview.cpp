#include "phasecontrolview.h"
#include "ui_phasecontrolview.h"
#include<QStyledItemDelegate>
#pragma execution_character_set("utf-8")//解决中文乱码
/** \class PhaseControlView
*
* phase数据流逻辑控制类
*
*/
PhaseControlView::PhaseControlView(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::PhaseControlView)
{
	ui->setupUi(this);
	QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
	ui->cb_resolution_phase->setItemDelegate(itemDelegate);
	connect(ui->cb_resolution_phase, SIGNAL(currentIndexChanged(int)), this, SLOT(OnResolutionChange(int)));
	ui->label_freq->setVisible(false);
}

PhaseControlView::~PhaseControlView()
{
	delete ui;
}

void PhaseControlView::setCurrentResolution(QString resolution) {
	//qDebug() << "howard setCurrentResolution=" << resolution;
	ui->cb_resolution_phase->setCurrentText(resolution);

}
int PhaseControlView::getCurrentResolution() {
	int index = ui->cb_resolution_phase->currentIndex();
	
	return index;
}
void PhaseControlView::addResolution(QString value) {
	ui->cb_resolution_phase->addItem(value);
}
void PhaseControlView::resolutionClear() {
	ui->cb_resolution_phase->clear();
}
int PhaseControlView::resolutionSize() {
	return ui->cb_resolution_phase->count();
}

void PhaseControlView::setResolutionChange(bool change) {
	mChangeStatus = change;
}
void PhaseControlView::setFreqMode(int mode)
{
	QString text;
	if (mode == 1) {
		text = "频率模式: 双频";
	}
	else {
		text = "频率模式: 单频";
	}

	text = " ";

	ui->label_freq->setText(text);
}
void PhaseControlView::OnResolutionChange(int index) {
	QString value_index(index);
	//    QString text = ui->cb_resolution_depth->itemText(index);
		//qDebug() << "howard OnResolutionChange=" << index;;
	if (mChangeStatus)
	{
		emit phaseChangeCallback(kResolutionChange, value_index);
	}
}
