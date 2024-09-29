#include "stylecombobox.h"
#include "ui_stylecombobox.h"
StyleComboBox::StyleComboBox(QWidget *parent):
	QComboBox(parent),
  ui(new Ui::StyleComboBox)
{
	ui->setupUi(this);
}

StyleComboBox::~StyleComboBox()
{
	delete ui;
}
void StyleComboBox::wheelEvent(QWheelEvent *e)
{
	
}