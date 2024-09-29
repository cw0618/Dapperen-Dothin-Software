#include "uvccolorwidget.h"
#include "ui_uvccolorwidget.h"
#include<QStyledItemDelegate>
#include <QDebug>
/** \class CaptureDialog
*
* uvc相机逻辑控制类
*
*/
UVCColorWidget::UVCColorWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UVCColorWidget)
{
    ui->setupUi(this);
    QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
    ui->cb_uvc_devices->setItemDelegate(itemDelegate);
    ui->cb_resolution_uvc->setItemDelegate(itemDelegate);
    connect(ui->cb_uvc_devices, SIGNAL( currentIndexChanged (int)),this, SLOT(OnUVCDevicesChange(int)) );
    connect(ui->cb_resolution_uvc, SIGNAL( currentIndexChanged (int)),this, SLOT(OnResolutionChange(int)) );
}

UVCColorWidget::~UVCColorWidget()
{
    delete ui;
}
void UVCColorWidget::addDeviceList(QString device){
    ui->cb_uvc_devices->addItem(device);
}
void UVCColorWidget::clearDeviceList(){
    ui->cb_uvc_devices->clear();
}
void UVCColorWidget::addResolution(QString value){
    ui->cb_resolution_uvc->addItem(value);
}
void UVCColorWidget::resolutionClear(){
    ui->cb_resolution_uvc->clear();
}
void UVCColorWidget::setCurrentResolution(QString resolution){
    ui->cb_resolution_uvc->setCurrentText(resolution);
}
void UVCColorWidget::setResolutionIndex(int index){
    ui->cb_resolution_uvc->setCurrentIndex(index);
}
int UVCColorWidget::getCurrentResolution(){
    return ui->cb_resolution_uvc->currentIndex();
}
QString UVCColorWidget::getCurrentResolutionStr(){
    QString value = ui->cb_resolution_uvc->currentText();
    return value;
}
int UVCColorWidget::getCurrentDevices(){
    return ui->cb_uvc_devices->currentIndex();
}
void UVCColorWidget::setResolutionChange(bool change){
     resolution_change=change;
}
void UVCColorWidget::setDevicesChange(bool change){
    devices_change=change;
}
void UVCColorWidget::OnUVCDevicesChange(int index){
    QString value_index = QString::number(index);
     //qDebug() << "howard OnResolutionChange="<< index;
     if (devices_change)
     {
         emit deviceChangeCallback(kDevicesChange, value_index);
     }
}
void UVCColorWidget::OnResolutionChange(int index){
    QString value_index = QString::number(index);
     //qDebug() << "howard OnResolutionChange="<< index;
     if (resolution_change)
     {
         emit colorResolutionCallback(kResolutionChange, value_index);
     }
}
