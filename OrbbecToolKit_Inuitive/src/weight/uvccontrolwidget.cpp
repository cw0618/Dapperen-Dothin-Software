#include "uvccontrolwidget.h"
#include "ui_uvccontrolwidget.h"

UVCControlWidget::UVCControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UVCControlWidget)
{
    ui->setupUi(this);
}

UVCControlWidget::~UVCControlWidget()
{
    delete ui;
}
