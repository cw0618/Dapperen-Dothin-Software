#include "aistreamcontrol.h"
#include "ui_aistreamcontrol.h"
#include<QStyledItemDelegate>
/** \class AIStreamControl
*
* AI数据控制类
*
*/
AIStreamControl::AIStreamControl(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AIStreamControl)
{
    ui->setupUi(this);
    QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
    ui->cb_skeleton_type->setItemDelegate(itemDelegate);
    connect(ui->cb_skeleton_type, SIGNAL(currentIndexChanged(int)), this, SLOT(OnAITypeChange(int)));
    connect(ui->tb_body_height_test, SIGNAL(clicked()), this, SLOT(OnBodyHeightChange()));
}

AIStreamControl::~AIStreamControl()
{
    delete ui;
}
void AIStreamControl::addAIType(QString value){
    ui->cb_skeleton_type->addItem(value);
}
void AIStreamControl::AITypeClear() {
    ui->cb_skeleton_type->clear();
}
void AIStreamControl::setAITypeIndex(int index) {
    ui->cb_skeleton_type->setCurrentIndex(index);
}
void AIStreamControl::setCurrentAIType(QString resolution) {
    //qDebug() << "howard setCurrentResolution=" << resolution;
    ui->cb_skeleton_type->setCurrentText(resolution);
}
int AIStreamControl::getCurrentAIType(){
    return ui->cb_skeleton_type->currentIndex();
}
void AIStreamControl::setAIStatueChange(bool change) {
    mAIStatus = change;
}
void AIStreamControl::OnAITypeChange(int index) {
    QString value_index = QString::number(index);
    if (mAIStatus)
    {
        emit aiChangeCallback(kAiTypeChange, value_index);
    }

}
void AIStreamControl::setBodyHeight(QString height){
   ui->label_height_value->setText(height);
}
void AIStreamControl::setPlaneCenter(QString value){
   ui->label_center_value->setText(value);
}
void AIStreamControl::setPlaneVector(QString value){
     ui->label_vector_value->setText(value);
}
void AIStreamControl::OnBodyHeightChange(){
   emit aiChangeCallback(kAiHeightChange, "0");
}
