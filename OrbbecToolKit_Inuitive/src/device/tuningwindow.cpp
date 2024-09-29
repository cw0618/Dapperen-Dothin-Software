#include "tuningwindow.h"
#include "ui_tuningwindow.h"

#include <QMessageBox>
#include<QStyledItemDelegate>
#include"src/config/xmlregisterbean.h"
#include"src/config/xmlconfig.h"
#pragma execution_character_set("utf-8")//解决中文乱码
/** \class tuningwindow
*
* 设备属性设置类
*
*/
QString cmd_common(int argc, QStringList argv, void* handle)
{
	SensorBase* sensor = (SensorBase*)handle;
	openni::SerialCmd serial_cmd = { 0 };
	QString argv_s = argv[0];

	memset(serial_cmd.cmd, 64, 0);

	if (argv_s.length() > 64) {
		return QString("commond is too long. ");
	}
	QByteArray ba = argv_s.toLatin1();
	char * c_cmd = ba.data();
	strcpy(serial_cmd.cmd, c_cmd);
	sensor->mOniDevice.sendCommand(&serial_cmd);

	qDebug() << QString::fromLocal8Bit(serial_cmd.resp);
	return QString::fromLocal8Bit(serial_cmd.resp);
}

tuningwindow::tuningwindow(SensorBase *sensorbase_, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::tuningwindow)
{
	ui->setupUi(this);
	setWindowTitle("Tuning");
	setWindowFlags(this->windowFlags() & ~Qt::WindowMinMaxButtonsHint);
	mSensorDeviceBean = sensorbase_;
	mTimerId = 0;
	Initialize();
}

tuningwindow::~tuningwindow()
{
	delete ui;
}
void tuningwindow::Initialize() {
	connect(ui->btn_register_write, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_register_read, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_laser_dac_get, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_laser_dac_set, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_flood_dac_get, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_flood_dac_set, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_exposure_value_get, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(ui->btn_exposure_value_set, SIGNAL(clicked()), this, SLOT(ButtonClicked()));
	connect(this, SIGNAL(SigSetingShowInfo(QString)), this, SLOT(ShowSettingInfo(QString)), Qt::QueuedConnection);
}
void tuningwindow::InitializeStatus() {

}

void tuningwindow::GetStartStatus() {
	if (!mSensorDeviceBean->mOniDevice.isValid())
	{
		ShowSettingInfo(QString("Device is not valid."));
		return;
	}
}
void tuningwindow::on_btn_clear_log_released()
{
	ui->textBrowser->clear();
	mLogIndex = 0;
}
void tuningwindow::ShowSettingInfo(QString info)
{
	int index = ui->tabWidget->currentIndex();
	if (index == 0)
	{
		ui->textBrowser->insertPlainText(QString("%1 :%2%3").arg(mLogIndex++).arg(info).arg("\n"));
		ui->textBrowser->moveCursor(QTextCursor::End);
	}
	else if (index == 1)
	{
		ui->textBrowser_2->insertPlainText(QString("%1 :%2%3").arg(mLogIndex++).arg(info).arg("\n"));
		ui->textBrowser_2->moveCursor(QTextCursor::End);
	}
	else if (index == 3)
	{
		ui->textBrowser_dothin->insertPlainText(QString("%1 :%2%3").arg(mLogIndex++).arg(info).arg("\n"));
		ui->textBrowser_dothin->moveCursor(QTextCursor::End);
	}

}
/**
 * @brief TuningWindow::on_checkbox_laser_toggled
 * @param checked
 */
void tuningwindow::on_checkbox_laser_toggled(bool checked)
{

	if (!mSensorDeviceBean->mOniDevice.isValid())
	{
		ShowSettingInfo(QString("Device is not valid."));
		return;
	}

	int dataSize = 4;
	int laser_en = checked == true ? 1 : 0;//1:表示开激光;0:表示关激光
	openni::Status rc = mSensorDeviceBean->mOniDevice.setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, (uint8_t *)&laser_en, dataSize);
	if (rc != openni::STATUS_OK)
	{
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("%1 LDM  error: %2").
			arg(checked == true ? QString("Enable ") : QString("Disable")).arg(QString::fromStdString(err));

		qDebug() << msg;
		ShowSettingInfo(msg);
	}
	else
	{
		QString msg = QString("%1 LDM ok.").arg(checked == true ? QString("Enable ") : QString("Disable"));
		qDebug() << msg;
		ShowSettingInfo(msg);
	}
}
/**
 * @brief TuningWindow::on_checkbox_ldp_toggled 鍒囨崲LDP寮€鍏?
 * @param checked
 */
void tuningwindow::on_checkbox_ldp_toggled(bool checked)
{
	if (!mSensorDeviceBean->mOniDevice.isValid())
	{
		ShowSettingInfo(QString("Device is not valid."));
		return;
	}

	int dataSize = 4;
	int ldp_enable = checked == true ? 1 : 0; //0：表示关闭 LDP， 1：表示打开 LDP
	openni::Status rc = mSensorDeviceBean->mOniDevice.setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (uint8_t *)&ldp_enable, dataSize);
	if (rc != openni::STATUS_OK)
	{
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("%1 LDP  error: %2").
			arg(checked == true ? QString("Enable ") : QString("Disable")).arg(QString::fromStdString(err));

		qDebug() << msg;
		ShowSettingInfo(msg);
	}
	else
	{
		QString msg = QString("%1 LDP ok.").arg(checked == true ? QString("Enable ") : QString("Disable"));
		qDebug() << msg;
		ShowSettingInfo(msg);
	}
}
/**
 * @brief TuningWindow::on_checkbox_flood_toggled 切换泛光灯开关
 * @param checked
 */
void tuningwindow::on_checkbox_flood_toggled(bool checked)
{
	if (!mSensorDeviceBean->mOniDevice.isValid())
	{
		ShowSettingInfo(QString("Device is not valid."));
		return;
	}

	int dataSize = 4;
	int flood_en = checked == true ? 1 : 0; //1:表示开泛光灯;0:表示关泛光灯
	openni::Status rc = mSensorDeviceBean->mOniDevice.setProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, (uint8_t *)&flood_en, dataSize);
	if (rc != openni::STATUS_OK)
	{
		string err = openni::OpenNI::getExtendedError();
		QString msg = QString("%1 Flood  error: %2").
			arg(checked == true ? QString("Open ") : QString("Close")).arg(QString::fromStdString(err));

		qDebug() << msg;
		ShowSettingInfo(msg);
	}
	else
	{
		QString msg = QString("%1 Flood ok.").arg(checked == true ? QString("Open ") : QString("Close"));
		qDebug() << msg;
		ShowSettingInfo(msg);
	}
}

void tuningwindow::slotsGroupBoxLaser(bool checked) {
	if (!(mSensorDeviceBean->mIrStreamOpen || mSensorDeviceBean->mPhaseStreamOpen || mSensorDeviceBean->mDepthStreamOpen)) {
		QMessageBox::warning(this, tr("警告"), tr("请先开流!"), QMessageBox::Close);
		return;
	}
	mSensorDeviceBean->setDothinLaser(checked);

}

void tuningwindow::slotsPowerTest(bool checked) {
	if (mSensorDeviceBean->IsOpened())
	{
		uint32_t value = checked ? 1 : 0;
		int size = sizeof(value);
		int ret = mSensorDeviceBean->mOniDevice.setProperty(OBC_ILLUM_POWER_TEST, (void *)&value, size);

		if (ret == 0) {

			qDebug() << "OBC_TOF_ILLUM_POWER_TEST value: " << value;
		}
		else {
			qDebug() << "set OBC_TOF_ILLUM_POWER_TEST ret=:" << ret;
		}
	}
}
void tuningwindow::slotsEEproWrite() {
	if (mSensorDeviceBean->IsOpened() || mSensorDeviceBean->mDeviceModeType != kDevicePleco)
	{

		int addr = ui->led_register_addr->text().toInt(nullptr, 16);
		int value = ui->led_register_value->text().toInt(nullptr, 16);
	}
}
void tuningwindow::slotsEEproRead() {
	if (mSensorDeviceBean->IsOpened() || mSensorDeviceBean->mDeviceModeType != kDevicePleco)
	{
		uint8_t *buffer = new uint8_t[4]();
		memset(buffer, 0, 4);
		float offset;
		int addr = ui->led_register_addr->text().toInt(nullptr, 16);
		buffer = nullptr;
	}
}
void tuningwindow::slotsAeDothin(bool checked) {
	if (mSensorDeviceBean->IsOpened())
	{
		if (mSensorId != OBC_SENSOR_ID_PLECO)
		{
			uint32_t value = (checked ? 1 : 0);
			int size = sizeof(value);
			int ret = mSensorDeviceBean->mOniDevice.setProperty(OBC_AE, (void *)&value, size);

			if (ret == 0) {

				qDebug() << "OBC_AE value: " << value;
			}
			else {
				qDebug() << "set OBC_AE ret=:" << ret;
			}
		}
		mDefaultList["AE"] = checked ? 1 : 0;
		isModifiConfig = true;
	}

}

void tuningwindow::ButtonClicked()
{
	if (!mSensorDeviceBean->mOniDevice.isValid())
	{
		ShowSettingInfo(QString("Device is not valid."));
		return;
	}

	QString btn_name = sender()->objectName();
	QString msg;

	QPushButton * cur_btn = qobject_cast<QPushButton*>(sender());
	cur_btn->setEnabled(false);
	// 设置寄存器
	if (0 == btn_name.compare(ui->btn_register_write->objectName()))
	{
		int addr = ui->led_register_addr->text().toInt(nullptr, 16);
		int value = ui->led_register_value->text().toInt(nullptr, 16);
		mSensorDeviceBean->operateSensorReg(true,addr, &value);
	}
	else if (0 == btn_name.compare(ui->btn_register_read->objectName()))
	{
		// 读取寄存器
		int addr = ui->led_register_addr->text().toInt(nullptr, 16);
		int value = 0;
		bool status = mSensorDeviceBean->operateSensorReg(false, addr, &value);
		if (status)
		{
			QString str = QString::number(value, 16).toUpper();
			ui->led_register_value->setText(str);
		}
		else
		{
			ui->led_register_value->setText("");
		}
	}
	else if (0 == btn_name.compare(ui->btn_laser_dac_get->objectName()))
	{
		int dacA = 0;
		bool status = mSensorDeviceBean->getLaserPower(&dacA);
		if (status)
		{
			QString strDacA = QString::number(dacA);
			ui->et_laser_dac_a->setText(strDacA);
		}
	}
	else if (0 == btn_name.compare(ui->btn_laser_dac_set->objectName()))
	{
		int dacA = ui->et_laser_dac_a->text().toInt();
		bool status = mSensorDeviceBean->setLaserPower(dacA);
		if (!status)
		{
			ui->et_laser_dac_a->setText("");
		}
	}
	else if (0 == btn_name.compare(ui->btn_flood_dac_get->objectName()))
	{
		int dacA = 0;

		bool status = mSensorDeviceBean->getFloodPower( &dacA);
		if (status)
		{
			QString strDacA = QString::number(dacA);
			ui->et_flood_dac_a->setText(strDacA);
		}
	}
	else if (0 == btn_name.compare(ui->btn_flood_dac_set->objectName()))
	{
		int dacA = ui->et_flood_dac_a->text().toInt();
		bool status = mSensorDeviceBean->setFloodPower(dacA);
		if (!status)
		{
			ui->et_flood_dac_a->setText("");
		}
	}
	else if (0 == btn_name.compare(ui->btn_exposure_value_get->objectName()))
	{
		int exposureVlaue = 0;

		bool status = mSensorDeviceBean->getIRExposure(&exposureVlaue);
		if (status)
		{
			QString strExposure = QString::number(exposureVlaue);
			ui->et_exposure_value->setText(strExposure);
		}
	}
	else if (0 == btn_name.compare(ui->btn_exposure_value_set->objectName()))
	{
		int dacA = ui->et_exposure_value->text().toInt();
		bool status = mSensorDeviceBean->setIRExposure(dacA);
		if (!status)
		{
			ui->et_exposure_value->setText("");
		}
	}
	cur_btn->setEnabled(true);
}

void tuningwindow::on_btn_choose_firm_bin_clicked()
{
	QString file_path = QFileDialog::getOpenFileName(this, "Please select file to load.", "./", "*.img");
	if (!file_path.isEmpty()) {
		ui->eb_firmFile->setText(file_path);
	}

	std::thread sendFileThread(&tuningwindow::ThreadForSendFile, this, file_path);
	sendFileThread.detach();
	//emit SigShowInfo("loading...");

}
void tuningwindow::ThreadForSendFile(QString filePath) {


	emit SigSetingShowInfo("setting start");
	QByteArray ba = filePath.toLocal8Bit();
	char *c_str = ba.data();
	RegisterIniFile mInitRegister;
	QString registerConfig = QString::fromLocal8Bit(c_str);
	int iret = mInitRegister.LoadRegisterIni(registerConfig);
	int count = mInitRegister.mRegisterList.size();
	for (int i = 0; i < count; i++)
	{
		REGISTER_DATA registerBean = mInitRegister.mRegisterList.at(i);
		int addr = registerBean.address.toInt(nullptr, 16);
		int value = registerBean.value.toInt(nullptr, 16);
		bool status = mSensorDeviceBean->operateSensorReg(true, addr, &value);
		if (status)
		{
			qDebug() << "set register success addr = " << addr << " value = " << value;
		}
		else
		{
			qDebug() << "set register failure addr = " << addr << " value = " << value;
		}
	}
	emit SigSetingShowInfo("setting finish");
}

void tuningwindow::on_btn_choose_d2c_bin_clicked()
{
	QString file_path = QFileDialog::getOpenFileName(this, "Please select file to load.", "./", "*.bin");
	if (!file_path.isEmpty()) {
		ui->eb_D2CFile->setText(file_path);

		bool ret = false;
		QByteArray ba = file_path.toLocal8Bit();
		const char *c_str = ba.data();
		ret = SendGeneralFile(mSensorDeviceBean->mOniDevice, c_str, XN_USB_FILE_CATEGORY_D2C, XN_USB_FILE_SUFFIX_BINARY);
		QString result = "load D2C bin success";
		if (!ret) {
			//QMessageBox::warning(this, "error", "load failed!");
			result = "load failed!";
		}

		ShowSettingInfo(result);

	}
}

void tuningwindow::on_btn_choose_calib_bin_clicked()
{

	QString file_path = QFileDialog::getOpenFileName(this, "Please select file to load.", "./", "*.bin");
	if (!file_path.isEmpty()) {
		//        QString str_file = GBK::ToUnicode(file_path.toStdString());
		ui->eb_calibFile->setText(file_path);

		bool ret = false;
		QByteArray ba = file_path.toLocal8Bit();
		char *c_str = ba.data();
		QString result = "load calib bin success";
		if (mSensorDeviceBean->mDeviceModeType != kDevicePleco)
		{
		}
		else
		{
			ret = SendGeneralFile(mSensorDeviceBean->mOniDevice, c_str, XN_USB_FILE_CATEGORY_CALIB, XN_USB_FILE_SUFFIX_BINARY);

			if (!ret) {
				//QMessageBox::warning(this, "error", "load failed!");
				result = "load failed!";
			}
		}
		ShowSettingInfo(result);

	}
}

void tuningwindow::on_btn_choose_register_clicked()
{
	QString file_path = QFileDialog::getOpenFileName(this, "Please select file to load.", "./", "*.*");
	if (!file_path.isEmpty()) {
		int lastIndex = file_path.lastIndexOf("/") + 1;
		QString fileName = file_path.mid(lastIndex);
		bool filterFile = fileName.startsWith("RegisterConfig");
		if (filterFile)
		{
			QByteArray ba = file_path.toLocal8Bit();
			char *c_str = ba.data();
			ui->eb_register_file->setText(QString::fromLocal8Bit(c_str));
			std::thread sendFileThread(&tuningwindow::ThreadForSendFile, this, QString::fromLocal8Bit(c_str));
			sendFileThread.detach();
		}
		else
		{
			emit SigSetingShowInfo("please select RegisterConfig.xml");
		}
	}
}

void tuningwindow::on_btn_choose_common_bin_clicked()
{
	QString file_path = QFileDialog::getOpenFileName(this, "Please select file to load.", "./", "*.*");
	if (!file_path.isEmpty()) {
		//        QString str_file = GBK::ToUnicode(file_path.toStdString());
		ui->eb_commonFile->setText(file_path);
		int lastIndex = file_path.lastIndexOf("/") + 1;
		QString fileName = file_path.mid(lastIndex);
		bool filterFile = fileName.startsWith("filter");
		if (mSensorDeviceBean->mDeviceModeType != kDevicePleco && filterFile)
		{
		}
		else {
			std::thread sendFileThread(&tuningwindow::ThreadForSendCommonFile, this, file_path);
			sendFileThread.detach();
			emit SigSetingShowInfo("loading...");
		}
	}
}

void tuningwindow::ThreadForSendCommonFile(QString filePath) {
	bool ret = false;
	QByteArray ba = filePath.toLocal8Bit();
	const char *c_str = ba.data();
	ret = SendGeneralFile(mSensorDeviceBean->mOniDevice, c_str, XN_USB_FILE_CATEGORY_COMMON, XN_USB_FILE_SUFFIX_BINARY);
	QString result = "load common file success";
	if (!ret) {
		//QMessageBox::warning(this, "error", "load failed!");
		result = "load failed!";
	}

	emit SigSetingShowInfo(result);
}

void tuningwindow::onFilterNameChange(QString text)
{

	qDebug() << "start set filter bin :" << text;

	bool ret = false;
	QByteArray ba = (QString("./config/") + text).toLocal8Bit();
	const char *c_str = ba.data();
	ret = SendGeneralFile(mSensorDeviceBean->mOniDevice, c_str, XN_USB_FILE_CATEGORY_FILTER, XN_USB_FILE_SUFFIX_BINARY);
	QString result = "load filter bin success";
	if (!ret) {

		QMessageBox::warning(nullptr, "error", "load filter file failed! Please check the log.");

		result = "load failed!";
	}
	else {
		QMessageBox::warning(nullptr, "filter", "load success.");
	}

	ShowSettingInfo(result);
}

void tuningwindow::onShowInfo(QString info)
{
	ShowSettingInfo(info);
}

bool tuningwindow::SendGeneralFile(openni::Device& device, const std::string& filePath, const uint16_t category, const uint16_t suffix)
{
	if (!device.isValid())
	{
		return false;
	}
	if (filePath.empty())
	{
		std::printf("Bad command...\n");
		qDebug() << "Bad command...\n";
		return false;
	}

	size_t pos = filePath.find_last_of("/");
	if (pos == std::string::npos)
	{
		pos = filePath.find_last_of("\\");
		if (pos != std::string::npos)
			pos += 1;
		else
			pos = 0;
	}
	else {
		pos += 1;
	}

	uint32_t fileSize = 0;

	std::ifstream is;
	is.open(filePath, std::ios::binary);
	if (is.is_open()) {
		std::streampos spos = is.tellg();
		is.seekg(0, std::ios::end);
		fileSize = is.tellg();
		is.seekg(spos);


	}
	else {
		qDebug() << "Open bin file failed!";
		return false;
	}

	std::string fileName = filePath.substr(pos);
	//qDebug() << "fileName:" << fileName.data();
	if (fileName.size() > XN_USB_MAX_FILE_NAME_LENGTH)
	{
		std::printf("The file name (%s) is too long (%d), it must be limited to 64 bytes.", fileName.c_str(), fileName.size());
		qDebug() << "it must be limited to 64 bytes";
		return false;
	}

	XnUsbGeneralFile usbFile = { 0 };
	usbFile.attributes.size = fileSize;
	usbFile.pContent = new uint8_t[fileSize + 1]();
	usbFile.attributes.suffix = suffix;
	usbFile.attributes.category = category;
	strcpy(usbFile.attributes.name, fileName.c_str());
	qDebug() << "fileName:" << usbFile.attributes.name;
	is.read((char*)usbFile.pContent, fileSize);

	is.close();

	std::printf("file: %s, size: %d\n", filePath.c_str(), fileSize);
	openni::Status rc = device.setProperty(XN_MODULE_PROPERTY_USB_GRNERAL_FILE, usbFile);
	qDebug() << "result:" << rc;
	if (openni::STATUS_OK != rc)
	{
		qDebug() << "Failed to send file , error :" << openni::OpenNI::getExtendedError();
		std::printf("Failed to send file (%s), error (%s)\n", filePath.c_str(), openni::OpenNI::getExtendedError());
		delete[] usbFile.pContent;
		return false;
	}

	std::printf("File (%s) sent successfully\n", filePath.c_str());
	qDebug() << "File sent successfully";
	delete[] usbFile.pContent;
	return true;
}

void tuningwindow::updateLaserComb()
{
#if 0
	int index = -1;
	if (current_vcsel_on_num >= 1 && current_vcsel_on_num <= context.sensorInfo.vcsel_num) {
		index = current_vcsel_on_num - 1;
	}

	if (index >= 0 && index < context.sensorInfo.vcsel_num) {
		ui->comb_vcsel_num->blockSignals(true);
		ui->comb_vcsel_num->clear();
		for (int i = 1; i <= context.sensorInfo.vcsel_num; i++) {
			QString item = QString("%1").arg(i);
			ui->comb_vcsel_num->addItem(item, QVariant(item));
		}
		ui->comb_vcsel_num->setCurrentIndex(index);
		ui->comb_vcsel_num->blockSignals(false);
	}

	if (context.sensorInfo.vcsel_num > 1) {
		ui->widget_vcsel->setVisible(true);
	}
	else {
		ui->widget_vcsel->setVisible(false);
	}
#endif
}


bool tuningwindow::setTxAbPower(uint16_t aState, uint16_t bState)
{
	if (!mSensorDeviceBean->IsOpened()) {
		return false;
	}
	ObcTxABPower tx_a_b = { 0,0 };
	tx_a_b.a_state = aState;
	tx_a_b.b_state = bState;
	int size = sizeof(tx_a_b);
	int ret = mSensorDeviceBean->mOniDevice.setProperty(OBC_TX_A_B_POWER, (void *)&tx_a_b, size);

	if (ret == 0) {

		qDebug() << "set OBC_TX_A_B_POWER a & b: " << tx_a_b.a_state << tx_a_b.b_state;
		return true;
	}
	else {
		qDebug() << "set OBC_TX_A_B_POWER failed ret=:" << ret;
	}
	return false;
}

bool tuningwindow::getTxAbPower(uint16_t *aState, uint16_t *bState)
{
	if (!mSensorDeviceBean->IsOpened()) {
		return false;
	}
	ObcTxABPower tx_a_b = { 0,0 };

	int size = sizeof(tx_a_b);
	int ret = mSensorDeviceBean->mOniDevice.getProperty(OBC_TX_A_B_POWER, (void *)&tx_a_b, &size);

	if (ret == 0) {

		qDebug() << "get OBC_TX_A_B_POWER a & b: " << tx_a_b.a_state << tx_a_b.b_state;

		*aState = tx_a_b.a_state;
		*bState = tx_a_b.b_state;
		return true;
	}
	else {
		qDebug() << "get OBC_TX_A_B_POWER failed ret=:" << ret;
	}
	aState = 0;
	bState = 0;
	return false;
}

void tuningwindow::OnDutyOneChange(int index)
{
	float duty_1_value = duty_1_strip.at(index);
	int duty_1_step = duty_1.indexOf(duty_1_value);
	duty0 = duty_1_step;
}
void tuningwindow::OnDytyTwoChange(int index)
{
	float duty_2_value = duty_2_strip.at(index);
	int duty_2_step = duty_2.indexOf(duty_2_value);
	duty1 = duty_2_step;
}
void tuningwindow::OnOutModelChange(int index)
{
	if (mSensorId == OBC_SENSOR_ID_MLX75027 || mSensorId == OBC_SENSOR_ID_IMX516 \
		|| mSensorId == OBC_SENSOR_ID_IMX456 || mSensorId == OBC_SENSOR_ID_IMX518
		|| mSensorId == OBC_SENSOR_ID_IMX627) {
		out_mode = ObcTofOutMode(index);

	}
	else if (mSensorId == OBC_SENSOR_ID_S5K33D || mSensorId == OBC_SENSOR_ID_RK1608_S5K33D) {
		out_mode = ObcTofOutMode(index + 10);
	}
}
