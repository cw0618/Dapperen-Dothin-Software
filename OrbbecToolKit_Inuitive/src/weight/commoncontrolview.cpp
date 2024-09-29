#include "commoncontrolview.h"
#include "ui_commoncontrolview.h"
#include <QDebug>
#include<qtoolbutton.h>
CommonControlView::CommonControlView(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::CommonControlView)
{
	ui->setupUi(this);
	
}

CommonControlView::~CommonControlView()
{
	delete ui;
}
