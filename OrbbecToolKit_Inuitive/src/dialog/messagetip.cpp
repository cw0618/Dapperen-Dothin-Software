#include "messagetip.h"
#include "ui_messagetip.h"

MessageTip::MessageTip(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MessageTip)
{
    ui->setupUi(this);
}

MessageTip::~MessageTip()
{
    delete ui;
}
void MessageTip::setMessageTip(QString info) {
    ui->label_content->setText(info);
}
