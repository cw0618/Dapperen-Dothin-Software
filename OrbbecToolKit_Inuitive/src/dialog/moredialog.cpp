#include "moredialog.h"

MoreDialog::MoreDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowTitle("Company information");
	//设置窗口没有最小化和最大化按钮
	setWindowFlags(this->windowFlags() & ~Qt::WindowMinMaxButtonsHint);
}

MoreDialog::~MoreDialog()
{
}
