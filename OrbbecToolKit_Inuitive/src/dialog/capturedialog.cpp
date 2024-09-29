#include "capturedialog.h"
#include "ui_capturedialog.h"

#include <QDateTime>
#include <QMessageBox>
#include <QButtonGroup>
#include <QRegExpValidator>
/// stb_image
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "src/stb_image_write.h"
#include"src/device/obutils.h"
#include<QStyledItemDelegate>
/** \class CaptureDialog
*
* 采图逻辑控制类
*
*/
CaptureDialog::CaptureDialog(SensorBase *sensorbase_, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::CaptureDialog)
{
	ui->setupUi(this);
	setWindowTitle("Save Dialog");

	setWindowFlags(this->windowFlags() & ~Qt::WindowMinMaxButtonsHint);
	mSensorDevice = sensorbase_;
	//oni_sensor_ = shared_ptr<sensorbase>(mx6000::Instance());
	QStyledItemDelegate* itemDelegate = new QStyledItemDelegate();
	ui->cb_tempe_mode->setItemDelegate(itemDelegate);
	QFileInfo iniFile(CAPTURE_INI_FILE);
	if (!iniFile.exists()) {
		qDebug() << "loadCaptureConfig failed, " << iniFile.absoluteFilePath() << " not exist.";
		return;
	}
	char version[100] = { 0 };
	minIni _minIni(CAPTURE_INI_FILE);
	//[Device]
	mDeviceName = _minIni.gets("Device", "device_name", "");
	mRrModelName = _minIni.gets("Device", "ir_model_name", "");
	mIrSensorName = _minIni.gets("Device", "ir_sensor_name", "");
	mVcselName = _minIni.gets("Device", "vcsel_name", "");

	mDeviceSerial = _minIni.gets("Device", "dev_serial", "");
	mIRSerial = _minIni.gets("Device", "ir_serial", "");
	mLdmpSerial = _minIni.gets("Device", "ldmp_serial", "");
	mDepthEngine = _minIni.gets("Device", "depth_engine", "");
	mDriverVersion = _minIni.gets("Device", "driver_version", "");
	ui->et_roi->setValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

	mDepthQue.reserve(mCaptureDequeSize);
	mIrQue.reserve(mCaptureDequeSize);
	mPhaseQue.reserve(mCaptureDequeSize);
	initialize();
}

CaptureDialog::~CaptureDialog()
{
	//oni_sensor_.reset();
	if (mCountTimer != nullptr)
	{
		delete mCountTimer;
		mCountTimer = nullptr;
	}
	delete ui;
}
void CaptureDialog::startTimerCount() {
	mTimerId = startTimer(800);
}
void CaptureDialog::initialize()
{
	mCalc = Calc::Instance();
	QString currentPath = QDir::currentPath();
	//cur_path = cur_path.left(cur_path.lastIndexOf("/"));
	currentPath += "/capture";
	ui->led_save_path->setText(currentPath);
	qDebug() << "set call back";
	mCsvFile = new CSVFile();
	depthRadioGroup = new QButtonGroup(this);
	phaseRadioGroup = new QButtonGroup(this);
	pcRadioGroup = new QButtonGroup(this);
	mTempControler = new TempControler();
	mTempControl.temp_timer = new QTimer();
	mCountTimer = new QTimer();
	mSensorDevice->SetDepthCaptureCallback(std::bind(&CaptureDialog::CbSnapDepthData, this, std::placeholders::_1));
	mSensorDevice->SetPcCaptureCallback(std::bind(&CaptureDialog::CbSnapPcData, this, std::placeholders::_1));
	mSensorDevice->SetIRCaptureCallback(std::bind(&CaptureDialog::CbSnapIRData, this, std::placeholders::_1));
	mSensorDevice->SetPhaseCaptureCallback(std::bind(&CaptureDialog::CbSnapPhaseData, this, std::placeholders::_1));
	mSensorDevice->SetColorCaptureCallback(std::bind(&CaptureDialog::CbSnapColorData, this, std::placeholders::_1));
	mSensorDevice->SetCSVCaptureCallback(std::bind(&CaptureDialog::CbSnapCsvForPhase, this, std::placeholders::_1));

	connect(this, SIGNAL(SigInputDepthData()), this, SLOT(slotInputDepthData()));
	connect(this, SIGNAL(SigInputIrData()), this, SLOT(slotInputIrData()));
	connect(this, SIGNAL(SigInputPhaseData()), this, SLOT(slotInputPhaseData()));
	connect(this, SIGNAL(SigInputColorData()), this, SLOT(slotInputColorData()));

	connect(ui->btn_open_save_path, SIGNAL(clicked()), this, SLOT(OnFilePathClick()));
	connect(ui->btn_start_capture, SIGNAL(clicked()), this, SLOT(OnSavePictureClick()));
	connect(ui->btn_ptc_capture, SIGNAL(clicked()), this, SLOT(OnPtcCaptureClick()));
	connect(ui->btn_darkcurrent_capture, SIGNAL(clicked()), this, SLOT(OnDarkCurrentCaptureClick()));

	connect(mCsvFile, SIGNAL(record_finish()), this, SLOT(onRecordFinish()));
	connect(this, SIGNAL(SigShowInfo(QString)), this, SLOT(slotShowInfo(QString)), Qt::QueuedConnection);

	connect(ui->btn_phase_raw, SIGNAL(clicked()), this, SLOT(onButtonStatusClick()));
	connect(ui->btn_phase_png, SIGNAL(clicked()), this, SLOT(onButtonStatusClick()));
	connect(ui->rb_phase_raw, SIGNAL(clicked(bool)), this, SLOT(onPhaseRadioClicked(bool)));
	connect(ui->rb_phase_png, SIGNAL(clicked(bool)), this, SLOT(onPhaseRadioClicked(bool)));

	connect(ui->btn_start_temp_capture, SIGNAL(clicked()), this, SLOT(onTempCaptureClicked()));
	connect(ui->cb_tempe_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(onTemperatureChange(int)));
	connect(ui->btn_clear_log, SIGNAL(clicked(bool)), this, SLOT(onlogClearClicked(bool)));
	connect(this, SIGNAL(setNextTempState(bool, int)), mTempControler, SLOT(on_setNextTempState(bool, int)));
	
	connect(mTempControl.temp_timer, SIGNAL(timeout()), this, SLOT(onTimeoutTempControl()));
	connect(mCountTimer, SIGNAL(timeout()), this, SLOT(onOneTimeFinish()));

	mPCColorEnable = tagPCColor::CUR;
	ui->btn_start_capture->setFocus();
	ui->et_roi->setValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

	phaseRadioGroup->addButton(ui->rb_phase_raw, 0);
	phaseRadioGroup->addButton(ui->rb_phase_png, 1);
	//updateUIState();
}

void CaptureDialog::GetStartStatus()
{

}
void CaptureDialog::OnSavePictureClick() {

	path = ui->led_save_path->text();
	if (!mCurFileDir.exists(path)) {
		mCurFileDir.mkpath(path);
	}
	path += QString("/Picture");
	if (!mCurFileDir.exists(path)) {
		mCurFileDir.mkpath(path);
	}
	if (!mColorThreadFinish || !mPhaseThreadFinish || !mThreadFinish
		|| !nPcThreadFinish || !mDepthThreadFinish) {
		QMessageBox::warning(nullptr, "warning", "please wait capture finish!");
		return;

	}

	//phase
	if (ui->ckb_save_ir_right->isChecked() && mSensorDevice->mPhaseStreamOpen)
	{
		bool ok;
		int total = ui->led_ir_right_total->text().toInt(&ok);
		int step = 0;
		if (total <= 0 && !ok)
		{
			QMessageBox::warning(nullptr, "error", "please check total for phase");
			return;
		}

	}

	//phase
	if (ui->ckb_save_ir_right->isChecked() && mSensorDevice->mPhaseStreamOpen)
	{
		bool ok;
		int total = ui->led_ir_right_total->text().toInt(&ok);
		int step = ui->led_step_phase->text().toInt();
		mSensorDevice->mPhaseStreamInfo->mCaptureFull = false;
		mSensorDevice->StartPhaseCapture(total, step);
		//std::thread phase_save_thread(&CaptureDialog::SavePhaseThread, this);
		//phase_save_thread.detach();
	}
	distance = ui->led_distance->text();
}

void CaptureDialog::OnPtcCaptureClick() {

	int exp_time[] = { 1,100,300,600,900,1200,1500,1800,2100,2400,2700,3000 };
	//int exp_time[] = { 1,100,300,600,900,1200,1500,1800,2100,2700,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000 };
	for (int i = 0; i < sizeof(exp_time)/sizeof(exp_time[0]); i++) {
		bool status = mSensorDevice->setIRExposure(exp_time[i]);
		QThread::msleep(50);
		OnSavePictureClick();
	}
}

void CaptureDialog::OnDarkCurrentCaptureClick() {

	int exp_time[] = { 200000,250000,300000,350000,400000,450000,500000};
	for (int i = 0; i < sizeof(exp_time) / sizeof(exp_time[0]); i++) {
		bool status = mSensorDevice->setIRExposure(exp_time[i]);
		QThread::msleep(500);
		OnSavePictureClick();
	}
}

void CaptureDialog::timerEvent(QTimerEvent *event)
{
	if (event->timerId() == mTimerId)
	{

		bool is_phase_open = mSensorDevice->mPhaseStreamOpen;
		if (false == is_phase_open) {
			ui->ckb_save_ir_right->setChecked(is_phase_open);
		}

		// 没有数据流开启的时候，禁用Capture按钮
		if (!is_phase_open)
		{
			ui->btn_start_capture->setEnabled(false);
			//ui->btn_save_oni->setEnabled(false);
		}
		else
		{
			ui->btn_start_capture->setEnabled(true);
			//ui->btn_save_oni->setEnabled(true);
		}
	}
}

void CaptureDialog::ReadTemp()
{
}

void CaptureDialog::ShowInfo(QString info)
{
	emit SigShowInfo(info);
}
void CaptureDialog::slotShowInfo(QString info)
{
	std::lock_guard<mutex> lg(mMutex);
	ui->textBrowser->insertPlainText(QString("%1 :%2%3").arg(mLogIndex++).arg(info).arg("\n"));
	ui->textBrowser->moveCursor(QTextCursor::End);
}

void CaptureDialog::OnFilePathClick() {

	QString file_path = QFileDialog::getExistingDirectory(this, "Please select a folder to save data.", "./");
	if (!file_path.isEmpty()) {
		ui->led_save_path->setText(file_path);
	}
}

void CaptureDialog::CbSnapCsvForPhase(OniData &data)
{
	if (mTempControl.bFirstReadTempr) {
		mTempControl.bFirstReadTempr = false;
		mTempControl.temp_step = data.mTofExteraline.temperRX;
	}
	mTempControl.mCurrentRxTempr = data.mTofExteraline.temperRX;


	if (mCsvFile->isRecording()) {
		TOFSensorMode outmodel = data.mTofExteraline.getSensorMode();
		int n = mCsvFile->roi_rows_num;
		//mMutexCsv.lock();
		if (!mCsvFile->hasRecordTitle()) {

			QString title;
			title = QString("outputModel:,%1").arg("S5K_4TAP_DUAL_FREQ_4FRAME");
			title.append("\nfreqmode:,SINGLE_FREQ");
			title.append(QString("\nintegration_time:,%1").arg(data.mTofExteraline.getLowIntegration()));
			title.append(QString("\nfrequency:,%1").arg(data.mTofExteraline.getLowFreq()));
			title.append(QString("\nduty_cycle:,%1").arg(data.mTofExteraline.getLowDutyCycle() * 100));

			title.append(QString("\nroi_rows:,%1x%2").arg(n).arg(n));
			title.append("\n");
			mCsvFile->recordTitle(title);

		}
		int frame_index = data.mTofExteraline.groupIndex;
		uint16_t rxTemp = ObUtils::Temperature2FixPoint(data.mTofExteraline.temperRX);
		uint16_t txTemp = ObUtils::Temperature2FixPoint(data.mTofExteraline.temperTX);

		int width = data.mTofExteraline.width;
		int height = data.mTofExteraline.height;
	    //if (outmodel == S5K_4TAP_DUAL_FREQ_4FRAME)
		if (data.mOutMode == OB_TOF_OUT_MODE_A_AND_B)
		{
			if (data.mTofExteraline.frameIndex == 0)
			{
				if (mCsvBean != nullptr) {
					mCsvBean->clear();
					delete mCsvBean;
					mCsvBean = nullptr;
				}
				mCsvBean = new CSVBean();
			}
			int index[2];
			for (int k = 0; k < n; k++) {
				for (int j = 0; j < n; j++) {
					mCsvBean->mType = 0x21;
					mCsvBean->mRoi = n;
					mCsvBean->mRxTemp = rxTemp;
					mCsvBean->mTxTemp = txTemp;
					mCsvBean->mFrameIndex = data.mTofExteraline.frameIndex;

					int16_t* frameData = (int16_t*)data.oData.mDataPtr;
					//width = 3*640;
					int centerIndex = width * (height / 2) + (width / 2);
					//三个点
					int centerIndex_1 = centerIndex;
					int centerIndex_2 = centerIndex + 1;
					//int centerIndex_3 = centerIndex + 2;
					//index[0] = (centerIndex_1 - ((n - 1) / 2) * 2) - (width*(n - 1) / 2) + (k*width) + 2 * j;
					//index[1] = (centerIndex_2 - ((n - 1) / 2) * 2) - (width*(n - 1) / 2) + (k*width) + 2 * j;
					//index[2] = (centerIndex_3 - ((n - 1) / 2) * 3) - (width*(n - 1) / 2) + (k*width) + 3 * j;
					index[0] = centerIndex_1 + (k - (n - 1) / 2) * width + 2 * (j - (n - 1) / 2);
					index[1] = centerIndex_2 + (k - (n - 1) / 2) * width + 2 * (j - (n - 1) / 2);
					mCsvBean->mDataBean.mData[0] = frameData[index[0]];
					mCsvBean->mDataBean.mData[1] = frameData[index[1]];
					//mCsvBean->mDataBean.mData[2] = frameData[index[2]];

					mCsvBean->mDataBean.mTempr[0] = rxTemp;
					mCsvBean->mDataBean.mTempr[1] = txTemp;

					DataBean bean = mCsvBean->mDataBean;
					if (data.mTofExteraline.frameIndex == 0) {
						mCsvBean->mPhaseOne.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 1)
					{
						mCsvBean->mPhaseTwo.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 2)
					{
						mCsvBean->mPhaseThree.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 3)
					{
						mCsvBean->mPhaseFour.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 4)
					{
						mCsvBean->mPhaseFive.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 5)
					{
						mCsvBean->mPhaseSix.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 6)
					{
						mCsvBean->mPhaseSeven.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 7)
					{
						mCsvBean->mPhaseEight.push(bean);
					}
				}
			}
			if (data.mTofExteraline.frameIndex == (data.mPhaseNumber -1)) {
				QString item;
				while (!mCsvBean->mPhaseOne.empty())
				{
					item = "";
					DataBean &dataOne = mCsvBean->mPhaseOne.front();

					item.append(QString("%1").arg(dataOne.mTempr[0]));
					item.append(QString(",%1").arg(dataOne.mTempr[1]));

					item.append(QString(",%1").arg(dataOne.mData[0]));
					item.append(QString(",%1").arg(dataOne.mData[1]));
					//item.append(QString(",%1").arg(dataOne.mData[2]));
					mCsvBean->mPhaseOne.pop();

					DataBean &dataTwo = mCsvBean->mPhaseTwo.front();
					item.append(QString(",%1").arg(dataTwo.mData[0]));
					item.append(QString(",%1").arg(dataTwo.mData[1]));
					//item.append(QString(",%1").arg(dataTwo.mData[2]));
					mCsvBean->mPhaseTwo.pop();

					DataBean &dataThree = mCsvBean->mPhaseThree.front();
					item.append(QString(",%1").arg(dataThree.mData[0]));
					item.append(QString(",%1").arg(dataThree.mData[1]));
					//item.append(QString(",%1").arg(dataThree.mData[2]));
					mCsvBean->mPhaseThree.pop();

					DataBean &dataFour = mCsvBean->mPhaseFour.front();
					item.append(QString(",%1").arg(dataFour.mData[0]));
					item.append(QString(",%1").arg(dataFour.mData[1]));
					//item.append(QString(",%1").arg(dataFour.mData[2]));
					mCsvBean->mPhaseFour.pop();

					if ((data.mPhaseNumber > 4))
					{
						DataBean &dataFive = mCsvBean->mPhaseFive.front();
						item.append(QString(",%1").arg(dataFive.mData[0]));
						item.append(QString(",%1").arg(dataFive.mData[1]));
						mCsvBean->mPhaseFive.pop();

						DataBean &dataSix = mCsvBean->mPhaseSix.front();
						item.append(QString(",%1").arg(dataSix.mData[0]));
						item.append(QString(",%1").arg(dataSix.mData[1]));
						mCsvBean->mPhaseSix.pop();

						DataBean &dataSeven = mCsvBean->mPhaseSeven.front();
						item.append(QString(",%1").arg(dataSeven.mData[0]));
						item.append(QString(",%1").arg(dataSeven.mData[1]));
						mCsvBean->mPhaseSeven.pop();

						DataBean &dataEight = mCsvBean->mPhaseEight.front();
						item.append(QString(",%1").arg(dataEight.mData[0]));
						item.append(QString(",%1").arg(dataEight.mData[1]));
						mCsvBean->mPhaseEight.pop();
					}
					mCsvFile->record(item);
				}
			}
		}
		else if (data.mOutMode == OB_TOF_OUT_MODE_A_B)
		{
			if (data.mTofExteraline.frameIndex == 0)
			{
				if (mCsvBean != nullptr) {
					mCsvBean->clear();
					delete mCsvBean;
					mCsvBean = nullptr;
				}
				mCsvBean = new CSVBean();
			}
			int index[2];
			for (int k = 0; k < n; k++) {
				for (int j = 0; j < n; j++) {
					mCsvBean->mType = 0x21;
					mCsvBean->mRoi = n;
					mCsvBean->mRxTemp = rxTemp;
					mCsvBean->mTxTemp = txTemp;
					mCsvBean->mFrameIndex = data.mTofExteraline.frameIndex;

					int16_t* frameData = (int16_t*)data.oData.mDataPtr;
					//width = 3*640;
					int centerIndex = width * (height / 2) + (width / 2);
					//一个点
					int centerIndex_1 = centerIndex;

					index[0] = centerIndex_1 + (k - (n - 1) / 2) * width + (j - (n - 1) / 2);
					mCsvBean->mDataBean.mData[0] = frameData[index[0]];

					mCsvBean->mDataBean.mTempr[0] = rxTemp;
					mCsvBean->mDataBean.mTempr[1] = txTemp;

					DataBean bean = mCsvBean->mDataBean;
					if (data.mTofExteraline.frameIndex == 0) {
						mCsvBean->mPhaseOne.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 1)
					{
						mCsvBean->mPhaseTwo.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 2)
					{
						mCsvBean->mPhaseThree.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 3)
					{
						mCsvBean->mPhaseFour.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 4)
					{
						mCsvBean->mPhaseFive.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 5)
					{
						mCsvBean->mPhaseSix.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 6)
					{
						mCsvBean->mPhaseSeven.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 7)
					{
						mCsvBean->mPhaseEight.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 8) {
						mCsvBean->mPhaseNine.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 9)
					{
						mCsvBean->mPhaseTen.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 10)
					{
						mCsvBean->mPhaseEleven.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 11)
					{
						mCsvBean->mPhaseTwelve.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 12)
					{
						mCsvBean->mPhaseThirteen.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 13)
					{
						mCsvBean->mPhaseFourteen.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 14)
					{
						mCsvBean->mPhaseFifteen.push(bean);
					}
					else if (data.mTofExteraline.frameIndex == 15)
					{
						mCsvBean->mPhaseSixteen.push(bean);
					}
				}
			}
			if (data.mTofExteraline.frameIndex == (data.mPhaseNumber - 1)) {
				QString item;
				while (!mCsvBean->mPhaseOne.empty())
				{
					item = "";
					DataBean &dataOne = mCsvBean->mPhaseOne.front();

					item.append(QString("%1").arg(dataOne.mTempr[0]));
					item.append(QString(",%1").arg(dataOne.mTempr[1]));

					item.append(QString(",%1").arg(dataOne.mData[0]));
					mCsvBean->mPhaseOne.pop();

					DataBean &dataTwo = mCsvBean->mPhaseTwo.front();
					item.append(QString(",%1").arg(dataTwo.mData[0]));
					mCsvBean->mPhaseTwo.pop();

					DataBean &dataThree = mCsvBean->mPhaseThree.front();
					item.append(QString(",%1").arg(dataThree.mData[0]));
					mCsvBean->mPhaseThree.pop();

					DataBean &dataFour = mCsvBean->mPhaseFour.front();
					item.append(QString(",%1").arg(dataFour.mData[0]));
					mCsvBean->mPhaseFour.pop();

					if (data.mPhaseNumber > 4)
					{
						DataBean &dataFive = mCsvBean->mPhaseFive.front();
						item.append(QString(",%1").arg(dataFive.mData[0]));
						mCsvBean->mPhaseFive.pop();

						DataBean &dataSix = mCsvBean->mPhaseSix.front();
						item.append(QString(",%1").arg(dataSix.mData[0]));
						mCsvBean->mPhaseSix.pop();

						DataBean &dataSeven = mCsvBean->mPhaseSeven.front();
						item.append(QString(",%1").arg(dataSeven.mData[0]));
						mCsvBean->mPhaseSeven.pop();

						DataBean &dataEight = mCsvBean->mPhaseEight.front();
						item.append(QString(",%1").arg(dataEight.mData[0]));
						mCsvBean->mPhaseEight.pop();

						if (data.mPhaseNumber > 8)
						{
							DataBean &dataNine = mCsvBean->mPhaseNine.front();
							item.append(QString(",%1").arg(dataNine.mData[0]));
							mCsvBean->mPhaseNine.pop();

							DataBean &dataTen = mCsvBean->mPhaseTen.front();
							item.append(QString(",%1").arg(dataTen.mData[0]));
							mCsvBean->mPhaseTen.pop();

							DataBean &dataEleven = mCsvBean->mPhaseEleven.front();
							item.append(QString(",%1").arg(dataEleven.mData[0]));
							mCsvBean->mPhaseEleven.pop();

							DataBean &dataTwelve = mCsvBean->mPhaseTwelve.front();
							item.append(QString(",%1").arg(dataTwelve.mData[0]));
							mCsvBean->mPhaseTwelve.pop();

							DataBean &dataThirteen = mCsvBean->mPhaseThirteen.front();
							item.append(QString(",%1").arg(dataThirteen.mData[0]));
							mCsvBean->mPhaseThirteen.pop();

							DataBean &dataFourteen = mCsvBean->mPhaseFourteen.front();
							item.append(QString(",%1").arg(dataFourteen.mData[0]));
							mCsvBean->mPhaseFourteen.pop();

							DataBean &dataFifteen = mCsvBean->mPhaseFifteen.front();
							item.append(QString(",%1").arg(dataFifteen.mData[0]));
							mCsvBean->mPhaseFifteen.pop();

							DataBean &dataSixteen = mCsvBean->mPhaseSixteen.front();
							item.append(QString(",%1").arg(dataSixteen.mData[0]));
							mCsvBean->mPhaseSixteen.pop();
						}
					}
					mCsvFile->record(item);
				}
			}
		}
	}
	emit SigInputCSVData();
}
void CaptureDialog::onRecordFinish()
{
	ShowInfo("Record CSV finished!!");
	mSensorDevice->StartCSVCapture(0);
	CsvThreadFinish = true;
	ui->btn_start_temp_capture->setEnabled(true);
}
void CaptureDialog::onlogClearClicked(bool)
{
	onCearLogReleased();
}
void CaptureDialog::onPcRadioClicked(bool checked)
{
	if (pcRadioGroup->checkedId() == 0) {
		qDebug() << "pc ply";
		pointCloudFormat = PicSaveFormat::PLY;
	}
	else {
		qDebug() << "pc xyz";
		pointCloudFormat = PicSaveFormat::XYZ;
	}
}
void CaptureDialog::onTempCaptureClicked() {

	if (!mSensorDevice->mPhaseStreamOpen) {
		ShowInfo("Please open phase stream first!");
		return;
	}

	QString path = ui->led_save_path->text();
	mCsvFile->setFileDir(path + "/csv/");
	QDateTime date_time;
	int64_t  time_sec = QDateTime::currentMSecsSinceEpoch();
	QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");

	//path = ui->led_save_path->text();
	if (!mCurFileDir.exists(path)) {
		mCurFileDir.mkpath(path);
	}
	path += QString("/Picture");
	if (!mCurFileDir.exists(path)) {
		mCurFileDir.mkpath(path);
	}

	recordTempCsv();

	ShowInfo("Start record csv ...");
	//save_csv = true;
	mSensorDevice->StartCSVCapture(-1);
	ui->btn_start_temp_capture->setEnabled(false);
	mTempControl.bFirstReadTempr = true;
}

void CaptureDialog::recordTempCsv()
{
	bool ok = false;
	int value = 0;
	int timeUp = 0;
	float tempMax = 0;
	float tempMin = 0;
	float tempInterval = 0;
	float tempValue = 0;

	if (!mTempControl.isTempControl) {
		value = ui->et_record_time->text().toInt(&ok);
		if (ok) {
			timeUp = value;
		}
		if (timeUp <= 0) {
			QMessageBox::warning(this, tr("警告"), "请输入正确的采集时间!", QMessageBox::Close);
			return;
		}
	}
	else {

		tempValue = ui->et_record_temp_min->text().toFloat(&ok);
		if (ok) {
			tempMin = tempValue;
		}
		else {
			QMessageBox::warning(this, tr("警告"), tr("请输入正确的温度!"), QMessageBox::Close);
			return;
		}

		tempValue = ui->et_record_temp_max->text().toFloat(&ok);
		if (ok) {
			tempMax = tempValue;
		}
		else {
			QMessageBox::warning(this, tr("警告"), tr("请输入正确的温度!"), QMessageBox::Close);
			return;
		}

		tempValue = ui->et_record_temp_interval->text().toFloat(&ok);
		if (ok) {
			tempInterval = tempValue;
		}
		else {

			QMessageBox::warning(this, tr("警告"), tr("请输入正确的温度间隔!"), QMessageBox::Close);
			return;
		}
	}
	ok = false;
	int roi_num = 0;
	value = ui->et_roi->text().toInt(&ok);
	if (ok) {
		roi_num = value;
	}

	if (roi_num <= 0 || (roi_num - 1) % 2 != 0) {
		QMessageBox::warning(this, tr("警告"), "请输入正确的矩阵列数!", QMessageBox::Close);
		return;
	}
	mCsvFile->roi_rows_num = roi_num;

	if (!mTempControl.isTempControl) {
		int64_t  current_time = QDateTime::currentMSecsSinceEpoch();
		mCsvFile->startRecord(QString("%1").arg(current_time), timeUp);
	}
	else {
		mTempControl.temp_max = tempMax;
		mTempControl.temp_min = tempMin;
		mTempControl.temp_interval = tempInterval;


		QSettings tempCfg(TEMPERATURE_DRIFT_CAP, QSettings::IniFormat);
		//别忘记传入采图中
		mTempControl.frameTotal = tempCfg.value("TempControlCapture/frameTotal", 1).toInt();
		mTempControl.frameStep = tempCfg.value("TempControlCapture/frameStep", 0).toInt();
		mTempControl.distance = tempCfg.value("TempControlCapture/distance", 100).toInt();

		mTempControl.temp_judge_interval = tempCfg.value("TempControlJudge/temp_judge_interval", 0.5).toFloat();
		mTempControl.timer_interval = tempCfg.value("TempControlTimer/timer_interval", 500).toInt();

		if (!mTempControl.isAutoTempControl) {

			if (!mTempControl.temp_timer->isActive()) {
				mTempControl.temp_timer->start(mTempControl.timer_interval);
				qDebug() << "start temp timer.";
			}
		}
		else {
			if (!mTempControler->init()) {
				QMessageBox::warning(this, tr("error"), tr("温控平台初始化失败!"), QMessageBox::Close);
				return;
			}
			mTempControler->startTempControl(tempMax, tempMin, tempInterval);
		}
	}
}
void CaptureDialog::onTemperatureChange(int index) {
	switch (index) {
	case 0:
		qDebug() << "关闭温控";
		mTempControl.isTempControl = false;
		if (!mTempControl.isAutoTempControl)
		{
			if (mCsvFile->isRecording()) {
				mCsvFile->timeUp();
				if (mTempControl.temp_timer->isActive())
				{
					mTempControl.temp_timer->stop();
					qDebug() << "stop temp timer.";
				}
			}
		}
		else {
			if (mCsvFile->isRecording()) {
				mCsvFile->timeUp();
			}
			mTempControler->stopTempControl();
			qDebug() << "stop auto TempControl.";
		}

		break;
	case 1:
		mTempControl.isTempControl = true;
		mTempControl.isAutoTempControl = false;
		break;

	default:
		break;
	}
	updateUIState();
}

/**
* 更新ui状态
*/
void CaptureDialog::updateUIState()
{

	ui->btn_start_temp_capture->setEnabled(
		!mCsvFile->isRecording() &&
		!mTempControl.temp_timer->isActive());
	//ui->et_record_time->setEnabled(!mTempControl.isTempControl);
	ui->et_record_temp_min->setEnabled(mTempControl.isTempControl);
	ui->et_record_temp_max->setEnabled(mTempControl.isTempControl);
	ui->et_record_temp_interval->setEnabled(mTempControl.isTempControl);

	ui->cb_tempe_mode->setEnabled(true);


}
void CaptureDialog::onPhaseRadioClicked(bool checked)
{
	if (phaseRadioGroup->checkedId() == 0) {
		qDebug() << "phase raw";
		phaseFormat = PicSaveFormat::RAW;
	}
	else {
		qDebug() << "phase png";
		phaseFormat = PicSaveFormat::PNG;
	}
}
void CaptureDialog::onButtonStatusClick() {
	QString btn_name = sender()->objectName();
	if (0 == btn_name.compare(ui->btn_phase_raw->objectName()))
	{
		ui->rb_phase_raw->setChecked(true);
		ui->rb_phase_png->setChecked(false);
		phaseFormat = PicSaveFormat::RAW;
	}
	else if (0 == btn_name.compare(ui->btn_phase_png->objectName()))
	{
		ui->rb_phase_raw->setChecked(false);
		ui->rb_phase_png->setChecked(true);
		phaseFormat = PicSaveFormat::PNG;
	}
}
void CaptureDialog::onDepthRadioClicked(bool checked)
{
	if (depthRadioGroup->checkedId() == 0) {
		qDebug() << "depth raw";
		depthFormat = PicSaveFormat::RAW;
	}
	else {
		qDebug() << "depth png";
		depthFormat = PicSaveFormat::PNG;
	}
}
void CaptureDialog::CbSnapDepthData(OniData & data)
{

	OniData* tmp = new OniData();
	tmp->copyTofExtInfo(&data.mTofExteraline, &tmp->mTofExteraline);
	*tmp = data;
	mMutexDepth.lock();
	mDepthQue.push_back(*tmp);
	mMutexDepth.unlock();
	emit SigInputDepthData();
}

void CaptureDialog::CbSnapPcData(OniData & data)
{
	OniData* tmp = new OniData();
	(*tmp).copyPointCloud(data);
	mPcQue.push_back(*tmp);

	emit SigInputPcData();

}

void CaptureDialog::CbSnapIRData(OniData &data)
{

	OniData* tmp = new OniData();
	tmp->copyTofExtInfo(&data.mTofExteraline, &tmp->mTofExteraline);
	*tmp = data;
	mMutexIR.lock();
	mIrQue.push_back(*tmp);
	mMutexIR.unlock();
	emit SigInputIrData();
}
void CaptureDialog::CbSnapPhaseData(OniData &data)
{
	OniData* tmp = new OniData();
	*tmp = data;
	tmp->copyTofExtInfo(&data.mTofExteraline, &tmp->mTofExteraline);
	mMutexPhase.lock();
	mPhaseQue.push_back(*tmp);
	mMutexPhase.unlock();
		
	emit SigInputPhaseData();
}
void CaptureDialog::CbSnapColorData(OniData & data)
{
	if (mColorQue.size() > 200)
	{
		data.isCaptureSuccess = false;
		return;

	}
	OniData* tmp = new OniData();
	tmp->mFrameTimeStamp = data.mFrameTimeStamp;
	*tmp = data;
	mColorQue.push(*tmp);
	data.isCaptureSuccess = true;
}
void CaptureDialog::slotInputDepthData()
{
	if (mDepthThreadFinish)
	{
		std::thread depth_save_thread(&CaptureDialog::SaveDepthThread, this);
		depth_save_thread.detach();
	}
}

void CaptureDialog::slotInputPcData()
{
	if (nPcThreadFinish)
	{
		std::thread pc_save_thread(&CaptureDialog::SavePcThread, this);
		pc_save_thread.detach();
	}
}

void CaptureDialog::slotInputIrData()
{
	if (mThreadFinish)
	{
		std::thread ir_save_thread(&CaptureDialog::SaveIrThread, this);
		ir_save_thread.detach();
	}
}
void CaptureDialog::slotInputPhaseData()
{
	if (mPhaseThreadFinish)
	{
		std::thread phase_save_thread(&CaptureDialog::SavePhaseThread, this);
		phase_save_thread.detach();
	}
}
/**
* 子线程中保存深度数据
*
*/
void CaptureDialog::SaveDepthThread()
{
	mDepthThreadFinish = false;


	ObPng ob_png;
	while (!mDepthQue.empty())
	{
		mMutexDepth.lock();
		if (mDepthQue.size() > (mCaptureDequeSize - 1))
		{
			mSensorDevice->mDepthStreamInfo->mCaptureFull = true;
		}
		else if (mDepthQue.size() < (mCaptureDequeSize - 5))
		{
			mSensorDevice->mDepthStreamInfo->mCaptureFull = false;
		}
		OniData &cur_data = mDepthQue.front();
		mMutexDepth.unlock();
		QDateTime date_time;
		int64_t  time_sec = cur_data.mCustomTimeStamp;//= QDateTime::currentMSecsSinceEpoch();
		QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
		QString phase_type;
		if (cur_data.mTofExteraline.isShuffle()) {
			phase_type = "SHUFFLE";

		}
		else {
			phase_type = "NON_SHUFFLE";
		}
		QString frequency_str = "MHZ";
		if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33D) {
			frequency_str = "MHZ";
		}
		else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
			frequency_str = "NS";
		}
		// save depth png
		if (depthFormat == PicSaveFormat::PNG)
		{
			QString frequency_str = "MHZ";
			if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33d) {
				frequency_str = "MHZ";
			}
			else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
				frequency_str = "NS";
			}
			QString frameType = "normal";
			if (cur_data.mTofExteraline.getFrameType() == 5)
			{
				if (cur_data.mTofExteraline.getSensorMode() == 0)
				{
					frameType = "laser";
				}
				else
				{
					frameType = "flood";
				}
			}
			QString full_name_png = QString("%1/%2_%3_Depth_%4x%5_F%6%7_%8mm_%9%10_%11_%12_%13.png")
				.arg(path).arg(timestamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0')).arg(cur_data.mWidth).arg(cur_data.mHeight)
				.arg(cur_data.mTofExteraline.groupIndex, 3, 10, QLatin1Char('0')).arg(cur_data.mTofExteraline.frameIndex, 3, 10, QLatin1Char('0'))
				.arg(distance).arg(cur_data.mTofExteraline.getLowFreq()).arg(frequency_str).arg(cur_data.mTofExteraline.getLowDutyCycle())
				.arg(phase_type).arg(frameType);
			int distance_phase = distance.toInt();
			createPNGForPhase(cur_data, full_name_png, distance_phase, DEPTH_MAP);
		}
		// 保存 RAW
		else if (depthFormat == PicSaveFormat::RAW)
		{
			QString frequency_str = "MHZ";
			if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33D) {
				frequency_str = "MHZ";
			}
			else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
				frequency_str = "NS";
			}
			QString frameType = "normal";
			if (cur_data.mTofExteraline.getFrameType() == 5)
			{
				if (cur_data.mTofExteraline.getSensorMode() == 0)
				{
					frameType = "laser";
				}
				else
				{
					frameType = "flood";
				}
			}
			//qDebug()<<"howard cur_data.frame_time_stamp="<<cur_data.frame_time_stamp;
			QString full_name_raw = QString("%1/%2_%3_Depth_%4x%5_F%6%7_%8mm_%9%10_%11_%12_%13.raw")
				.arg(path).arg(timestamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0')).arg(cur_data.mWidth).arg(cur_data.mHeight)
				.arg(cur_data.mTofExteraline.getGroupIndex(), 3, 10, QLatin1Char('0')).arg(cur_data.mTofExteraline.getFrameIndex(), 3, 10, QLatin1Char('0'))
				.arg(distance).arg(cur_data.mTofExteraline.getLowFreq()).arg(frequency_str).arg(cur_data.mTofExteraline.getLowDutyCycle())
				.arg(phase_type).arg(frameType);
		 	QByteArray ba = full_name_raw.toLocal8Bit();
			char* c_full_name_raw = ba.data();
			WriteRawData(c_full_name_raw, cur_data.oData.mDataPtr, cur_data.mDataSizeBytes);
			ShowInfo(QString("depth raw save success "));
		}
		cur_data.Rst();
		if (mDepthQue.front().oData.mDataPtr != nullptr)
		{
			delete[] mDepthQue.front().oData.mDataPtr;
			mDepthQue.front().oData.mDataPtr = nullptr;
		}
		// 会执行析构
		mMutexDepth.lock();
		mDepthQue.pop_front();
		mMutexDepth.unlock();
	}
	mMutexDepth.lock();
	mSensorDevice->mDepthStreamInfo->mCaptureFull = false;
	mMutexDepth.unlock();
	mDepthThreadFinish = true;
}

void CaptureDialog::SavePcThread()
{
	nPcThreadFinish = false;

	while (!mPcQue.empty())
	{
		OniData &cur_data = mPcQue.front();
		QDateTime date_time;
		int64_t  time_sec = cur_data.mCustomTimeStamp;//= QDateTime::currentMSecsSinceEpoch();
		QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
		QString full_name_png;
		if (pointCloudFormat == PicSaveFormat::XYZ) {
			full_name_png = QString("%1/%2_%3_PointCloud_%4x%5_%6mm_%7.xyz")
				.arg(path).arg(timestamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0'))
				.arg(cur_data.mWidth).arg(cur_data.mHeight).arg(distance).arg(cur_data.mFrameTimeStamp);
			QByteArray ba = full_name_png.toLocal8Bit();
			char* c_full_name_png = ba.data();

			mCalc->writePly(c_full_name_png, cur_data.oData.mDataPointCouldPtr, cur_data.oData.SizeOfPointCloud(), false, cur_data.mWidth, cur_data.mHeight);
			ShowInfo(QString("pointCloud save success"));
		}
		else if (pointCloudFormat == PicSaveFormat::PLY) {
			full_name_png = QString("%1/%2_%3_PointCloud_%4x%5_%6mm_%7.ply")
				.arg(path).arg(timestamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0'))
				.arg(cur_data.mWidth).arg(cur_data.mHeight).arg(distance).arg(cur_data.mFrameTimeStamp);
			QByteArray ba = full_name_png.toLocal8Bit();
			char* c_full_name_png = ba.data();

			mCalc->writePly(c_full_name_png, cur_data.oData.mDataPointCouldPtr, cur_data.oData.SizeOfPointCloud(), true, cur_data.mWidth, cur_data.mHeight);
			ShowInfo(QString("pointCloud save success"));
		}
		cur_data.Rst();
		if (cur_data.oData.mDataPtr != nullptr)
		{
			delete[] cur_data.oData.mDataPtr;
			cur_data.oData.mDataPtr = nullptr;
		}
		if (cur_data.oData.mDataPointCouldPtr != nullptr)
		{
			delete[] cur_data.oData.mDataPointCouldPtr;
			cur_data.oData.mDataPointCouldPtr = nullptr;
		}
		mPcQue.pop_front();
	}
	nPcThreadFinish = true;
}

void CaptureDialog::SaveIrThread()
{
	mThreadFinish = false;

	ObPng ob_png;
	while (!mIrQue.empty())
	{
		mMutexIR.lock();
		if (mIrQue.size() > (mCaptureDequeSize - 1))
		{
			mSensorDevice->mIrStreamInfo->mCaptureFull = true;
		}
		else if (mIrQue.size() > (mCaptureDequeSize - 5))
		{
			mSensorDevice->mIrStreamInfo->mCaptureFull = false;
		}

		OniData &cur_data = mIrQue.front();
		mMutexIR.unlock();
		QDateTime date_time;
		int64_t  time_sec = cur_data.mCustomTimeStamp;//= QDateTime::currentMSecsSinceEpoch();
		QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
		QString phase_type;
		if (cur_data.mTofExteraline.isShuffle()) {
			phase_type = "SHUFFLE";

		}
		else {
			phase_type = "NON_SHUFFLE";
		}
		// 保存PNG
		if (irFormat == PicSaveFormat::PNG && 2 == cur_data.mPixelInBytes)
		{
			QString frequency_str = "MHZ";
			if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33d) {
				frequency_str = "MHZ";
			}
			else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
				frequency_str = "NS";
			}
			QString full_name_png = QString("%1/%2_%3_IR_%4x%5_F%6%7_%8mm_%9%10_%11_%12_%13.png")
				.arg(path).arg(timestamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0')).arg(cur_data.mWidth).arg(cur_data.mHeight)
				.arg(cur_data.mTofExteraline.groupIndex, 3, 10, QLatin1Char('0')).arg(cur_data.mTofExteraline.frameIndex, 3, 10, QLatin1Char('0'))
				.arg(distance).arg(cur_data.mTofExteraline.getLowFreq()).arg(frequency_str).arg(cur_data.mTofExteraline.getLowDutyCycle())
				.arg(phase_type).arg(cur_data.mFrameTimeStamp);
			int distance_phase = distance.toInt();
			createPNGForPhase(cur_data, full_name_png, distance_phase, IR_DATA_MAP);
		}
		mMutexIR.lock();
		if (mIrQue.front().oData.mDataPtr != nullptr)
		{
			delete[] mIrQue.front().oData.mDataPtr;
			mIrQue.front().oData.mDataPtr = nullptr;
		}
		cur_data.Rst();

		mIrQue.pop_front();
		mMutexIR.unlock();
	}

	mSensorDevice->mIrStreamInfo->mCaptureFull = false;
	mThreadFinish = true;
}

void CaptureDialog::SavePhaseThread()
{
	mPhaseThreadFinish = false;

	//ObPng ob_png;
	while (!mPhaseQue.empty())
	{
		mMutexPhase.lock();
		if (mPhaseQue.size() > mCaptureDequeSize)
		{
			mSensorDevice->mPhaseStreamInfo->mCaptureFull = true;
		}
		else if (mPhaseQue.size() < (mCaptureDequeSize - 5))
		{
			mSensorDevice->mPhaseStreamInfo->mCaptureFull = false;
		}
		OniData &cur_data = mPhaseQue.front();
		mMutexPhase.unlock();
		QDateTime date_time;
		int64_t  time_sec = cur_data.mCustomTimeStamp;// QDateTime::currentMSecsSinceEpoch();

		if (cur_data.mTofExteraline.groupIndex != phaseGroupIndex) {
			phaseGroupIndex = cur_data.mTofExteraline.groupIndex;
			phaseTimestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
		}
		QString phase_type;
		if (cur_data.mTofExteraline.isShuffle()) {
			phase_type = "SHUFFLE";

		}
		else {
			phase_type = "NON_SHUFFLE";
		}
		if (phaseFormat == PicSaveFormat::RAW)
		{
			QString frequency_str = "MHZ";
			if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33D) {
				frequency_str = "MHZ";
			}
			else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
				frequency_str = "NS";
			}
			QString frameType = "normal";
			if (cur_data.mTofExteraline.getSensorMode() == 5)
			{
				if (cur_data.mTofExteraline.getFrameType() == 0)
				{
					frameType = "flood";
				}
				else
				{
					frameType = "laser";
				}
			}
			//qDebug()<<"howard cur_data.frame_time_stamp="<<cur_data.frame_time_stamp;
			/*QString full_name_raw = QString("%1/%2_%3_IR_%4x%5_F%6%7_%8mm_%9%10_%11_%12_%13.raw")
				.arg(path).arg(cur_data.mFrameTimeStamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0')).arg(cur_data.mWidth).arg(cur_data.mHeight)
				.arg(cur_data.mTofExteraline.getGroupIndex(), 3, 10, QLatin1Char('0')).arg(cur_data.mTofExteraline.getFrameIndex(), 3, 10, QLatin1Char('0'))
				.arg(distance).arg(cur_data.mTofExteraline.getLowFreq()).arg(frequency_str).arg(cur_data.mTofExteraline.getLowDutyCycle())
				.arg(phase_type).arg(frameType);*/
			QString full_name_raw = QString("%1/%2.raw")
				.arg(path).arg(cur_data.mFrameTimeStamp);
			QByteArray ba = full_name_raw.toLocal8Bit();
			char* c_full_name_raw = ba.data();


			WriteRawData(c_full_name_raw, cur_data.oData.mDataPtr, cur_data.mDataSizeBytes);
			ShowInfo(QString("phase raw save success "));
		}
		else if (phaseFormat == PicSaveFormat::PNG && 2 == cur_data.mPixelInBytes)
		{
			QString frequency_str = "MHZ";
			if (cur_data.mTofExteraline.getSensorID() == 0x303d || cur_data.mTofExteraline.getSensorID() == 0x33d) {
				frequency_str = "MHZ";
			}
			else if (cur_data.mTofExteraline.getSensorID() == 0x5A || cur_data.mTofExteraline.getSensorID() == 0x0123) {
				frequency_str = "NS";
			}
			QString frameType = "normal";
			if (cur_data.mTofExteraline.getSensorMode() == 5)
			{
				if (cur_data.mTofExteraline.getFrameType() == 0)
				{
					frameType = "flood";
				}
				else
				{
					frameType = "laser";
				}
			}
			/*QString full_name_png = QString("%1/%2_%3_IR_%4x%5_F%6%7_%8mm_%9%10_%11_%12_%13.png")
				.arg(path).arg(cur_data.mFrameTimeStamp).arg(cur_data.mLogicId, 6, 10, QLatin1Char('0')).arg(cur_data.mWidth).arg(cur_data.mHeight)
				.arg(cur_data.mTofExteraline.groupIndex, 3, 10, QLatin1Char('0')).arg(cur_data.mTofExteraline.frameIndex, 3, 10, QLatin1Char('0'))
				.arg(distance).arg(cur_data.mTofExteraline.getLowFreq()).arg(frequency_str).arg(cur_data.mTofExteraline.getLowDutyCycle())
				.arg(phase_type).arg(frameType);*/
			QString full_name_png = QString("%1/%2.png")
				.arg(path).arg(cur_data.mFrameTimeStamp);
			int distance_phase = distance.toInt();
			/*OniData *phase_data = new OniData();
			phase_data->copy(cur_data);
			phase_data->copyTofExtInfo(&cur_data.tof_exteraline, &phase_data->tof_exteraline);*/
			createPNGForPhase(cur_data, full_name_png, distance_phase, PHASE_MAP);

		}
		mMutexPhase.lock();
		if (mPhaseQue.front().oData.mDataPtr != nullptr)
		{
			delete[] mPhaseQue.front().oData.mDataPtr;
			mPhaseQue.front().oData.mDataPtr = nullptr;
		}
		cur_data.Rst();
		mPhaseQue.pop_front();

		mMutexPhase.unlock();
	}
	mSensorDevice->mPhaseStreamInfo->mCaptureFull = false;
	mPhaseThreadFinish = true;
}
void CaptureDialog::SaveColorThread(int thread_id)
{
	//	std::lock_guard<mutex> lg(mutex_color);
	mColorThreadFinish = false;

	qDebug() << "start thread: " << thread_id;

	ObPng ob_png;
	while (!mColorQue.empty() || mSensorDevice->getColorCaptureTotal() > 0)
	{
		mMutexColor.lock();
		if (mColorQue.empty())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			mMutexColor.unlock();
			continue;
		}
		OniData &temp = mColorQue.front();
		OniData *cur_data = new OniData();
		cur_data->mFrameTimeStamp = cur_data->mFrameTimeStamp;
		*cur_data = temp;
		mColorQue.pop();

		mMutexColor.unlock();

		QDateTime date_time;
		int64_t  time_sec = cur_data->mCustomTimeStamp;//= QDateTime::currentMSecsSinceEpoch();
		QString timestamp = date_time.fromMSecsSinceEpoch(time_sec).toString("yyyyMMddhhmmsszzz");
		//QString distance = ui->led_color_distance->text();
		/// 淇濆瓨PNG
		if (colorFormat == PicSaveFormat::PNG)
		{
			QString full_name_png = QString("%1/%2_%3_%4_%5x%6_%7mm_%8.png")
				.arg(path).arg(timestamp).arg(cur_data->mLogicId, 6, 10, QLatin1Char('0'))
				.arg("COLOR").arg(cur_data->mWidth).arg(cur_data->mHeight)
				.arg(distance).arg(cur_data->mFrameTimeStamp);
			QByteArray ba = full_name_png.toLocal8Bit();
			char* c_full_name_png = ba.data();

			auto size = cur_data->mWidth * cur_data->mHeight * 3;

			stbi_write_png(c_full_name_png, cur_data->mWidth, cur_data->mHeight, 3, cur_data->oData.mDataPtr, cur_data->mWidth * 3);
			ShowInfo(QString("rgb png save success"));
		}
		if (cur_data->oData.mDataPtr != nullptr)
		{
			//qDebug() << "color delete. "<< thread_id;
			delete[] cur_data->oData.mDataPtr;
			cur_data->oData.mDataPtr = nullptr;
		}
		cur_data->Rst();

	}

	mColorThreadFinish = true;
	qDebug() << "thread end, " << thread_id;
}
void CaptureDialog::WriteRawData(const char * raw_name, const int8_t * data, uint32_t byte_size)
{
	FILE *fp = nullptr;
	fopen_s(&fp, raw_name, "wb");

	fwrite(data, sizeof(char), byte_size, fp);

	fclose(fp);
}

void CaptureDialog::WriteInteriParam2Raw(const char * raw_name, const raw_parm & rp)
{
	FILE *raw_file = NULL;
	fopen_s(&raw_file, raw_name, "a");
	if (raw_file == NULL) {
		return;
	}

	char* mack = new char[3 * sizeof(char)]();
	memset(mack, 0xAB, 3);
	shared_ptr<char> sp_mack = shared_ptr<char>(mack);

	const int iParamSize = sizeof(raw_parm);
	char* cParams = new char[sizeof(char) * iParamSize]();

	::memcpy(cParams, &rp, iParamSize);

	fwrite(mack, sizeof(char), 3, raw_file);
	fwrite(cParams, sizeof(char), iParamSize, raw_file);
	//写入标准 AB AB AB
	fwrite(mack, sizeof(char), 3, raw_file);

	fclose(raw_file);
}
void CaptureDialog::onCearLogReleased()
{
	std::lock_guard<mutex> lg(mMutex);
	ui->textBrowser->clear();
	mLogIndex = 0;
}
QString CaptureDialog::getCsvFileName(QString name)
{
	QString csv_name = name.append("_").append(QDateTime::currentDateTime().toString("yyyy-MM-dd")).append(".csv");
	return csv_name;
}
void CaptureDialog::saveCSVData(QString csv_name, QString file_path) {
	m_qMutex.lock();
	mSaveCsvPath.append("/").append(csv_name);
	infoFile = new QFile(mSaveCsvPath);
	if (!infoFile->exists()) {
		infoFile->open(QIODevice::Append | QIODevice::Text | QIODevice::ReadWrite);
		QString title = "picture Path,Time,\n";
		infoFile->write(title.toUtf8().data());
		infoFile->close();
	}
	QDateTime* data_time = new QDateTime(QDateTime::currentDateTime());
	QString timestamp = data_time->toString("MMddhhmmsszzz");
	infoFile->open(QIODevice::Append | QIODevice::Text | QIODevice::ReadWrite);
	infoFile->write(QString("%1%2").arg(file_path).arg(",").toUtf8().data());
	infoFile->write(QString("%1%2").arg(timestamp).arg(",").toUtf8().data());

	infoFile->write("\n");
	infoFile->close();
	m_qMutex.unlock();
}

int CaptureDialog::createPNGForPhase(OniData &cur_data, QString fileName, int distance, TOF_DATA_TYPE dataType) {
	int true_roi_m, true_roi_n;

	QByteArray ba = fileName.toLocal8Bit();
	const char* save_png_file = ba.data();

	FILE* fb = fopen(save_png_file, "wb");
	if (fb == NULL) {
		qWarning() << "PngCreater run: open file failed:" << fileName;
		return -3;
	}

	png_struct_ptr png_ptr = NULL;
	png_info_ptr   info_ptr = NULL;
	//TODO
	PngHolderInfo pngInfo_phase;
	memset(&pngInfo_phase, 0, sizeof(PngHolderInfo));
	pngInfo_phase.channels = 1;
	pngInfo_phase.compression_level = 0;
	pngInfo_phase.data.data_bits = 16;
	pngInfo_phase.dev.model_type = MODEL_TOF;

	char toolName[50];
	sprintf(toolName, "%s%s", "OrbbecSensor", "0.0.0.12");
	pngInfo_phase.ob_info.tool_name.length = strlen(toolName);
	sprintf((char*)pngInfo_phase.ob_info.tool_name.name, "%s", toolName);
	//[Device]
	pngInfo_phase.dev.device.length = strlen(mDeviceName.data());
	sprintf((char*)pngInfo_phase.dev.device.name, "%s", mDeviceName.data());

	pngInfo_phase.dev.ir_model.length = strlen(mRrModelName.data());
	sprintf((char*)pngInfo_phase.dev.ir_model.name, "%s", mRrModelName.data());

	pngInfo_phase.dev.ir_sensor.length = strlen(mIrSensorName.data());
	sprintf((char*)pngInfo_phase.dev.ir_sensor.name, "%s", mIrSensorName.data());

	pngInfo_phase.dev.dev_serial.length = strlen(mDeviceSerial.data());
	sprintf((char*)pngInfo_phase.dev.dev_serial.name, "%s", mDeviceSerial.data());
	pngInfo_phase.dev.ir_serial.length = strlen(mIRSerial.data());
	sprintf((char*)pngInfo_phase.dev.ir_serial.name, "%s", mIRSerial.data());
	pngInfo_phase.dev.ldmp_serial.length = strlen(mLdmpSerial.data());
	sprintf((char*)pngInfo_phase.dev.ldmp_serial.name, "%s", mLdmpSerial.data());
	pngInfo_phase.dev.depth_engine.length = strlen(mDepthEngine.data());
	sprintf((char*)pngInfo_phase.dev.depth_engine.name, "%s", mDepthEngine.data());
	pngInfo_phase.drv.driver_version.length = strlen(mDriverVersion.data());
	sprintf((char*)pngInfo_phase.drv.driver_version.name, "%s", mDriverVersion.data());

	pngInfo_phase.dev.ldmp_model.length = strlen(mVcselName.data());
	sprintf((char*)pngInfo_phase.dev.ldmp_model.name, "%s", mVcselName.data());
	pngInfo_phase.groupIndex = cur_data.mTofExteraline.getFrameIndex();
	pngInfo_phase.frameIndex = cur_data.mTofExteraline.getGroupIndex();
	pngInfo_phase.data.width = cur_data.mTofExteraline.getWidth();
	pngInfo_phase.data.height = cur_data.mTofExteraline.getHeight();
	pngInfo_phase.data.data_type = dataType;
	pngInfo_phase.sensorId = cur_data.mTofExteraline.getSensorID();
	pngInfo_phase.env.vcsel_temp = ObUtils::Temperature2FixPoint(cur_data.mTofExteraline.getTemperTX());
	pngInfo_phase.env.ir_temp = ObUtils::Temperature2FixPoint(cur_data.mTofExteraline.getTemperRX());
	pngInfo_phase.env.dll_temp = 0;
	pngInfo_phase.env.distance = distance;

	pngInfo_phase.drv.integration_time = cur_data.mTofExteraline.getLowIntegration();
	pngInfo_phase.drv.output_mode = cur_data.mTofExteraline.getSensorMode();
	pngInfo_phase.drv.frequency = cur_data.mTofExteraline.getLowFreq();
	pngInfo_phase.drv.duty_cycle = cur_data.mTofExteraline.getLowDutyCycle() * 100;
	pngInfo_phase.drv.phase = cur_data.mTofExteraline.getSensorMode();

	png_ptr = create_writepng_struct();
	if (png_ptr == NULL) {
		qWarning() << "PngCreater run: create_writepng_struct failed:";
		fclose(fb);
		remove(save_png_file);
		return -4;
	}
	if (pngInfo_phase.sensorId == openni::SENSOR_ID_S5K33D || pngInfo_phase.sensorId == 0x303d) {
		true_roi_m = pngInfo_phase.data.width;
		true_roi_n = pngInfo_phase.data.height;
	}
	else if (pngInfo_phase.sensorId == 0x5A || pngInfo_phase.sensorId == 0x0123) {
		//true_roi_m = 1920;//m*2;
		//true_roi_n = 480;// n * 2;
		true_roi_m = pngInfo_phase.data.width;
		true_roi_n = pngInfo_phase.data.height;
	}
	else
	{
		true_roi_m = pngInfo_phase.data.width;
		true_roi_n = pngInfo_phase.data.height;
	}

	info_ptr = create_writepng_info(fb, png_ptr, true_roi_m, true_roi_n, pngInfo_phase.data.data_bits, pngInfo_phase.channels, pngInfo_phase.compression_level);
	if (info_ptr == NULL) {
		qWarning() << "PngCreater run: create_writepng_info failed:";
		fclose(fb);
		remove(save_png_file);
		return -5;
	}
	int ret = write_png_image(png_ptr, info_ptr, true_roi_m, true_roi_n, (char*)cur_data.oData.mDataPtr);
	pngInfo_phase.data.height = true_roi_n;
	pngInfo_phase.data.width = true_roi_m;

	ret = add_chunk_info(png_ptr, CHUNK_DEVICE, &pngInfo_phase.dev, sizeof(pngInfo_phase.dev));
	ret = add_chunk_info(png_ptr, CHUNK_TOF_DRIVER, &pngInfo_phase.drv, sizeof(pngInfo_phase.drv));
	ret = add_chunk_info(png_ptr, CHUNK_TOF_ENV, &pngInfo_phase.env, sizeof(pngInfo_phase.env));
	ret = add_chunk_info(png_ptr, CHUNK_TOF_DATA, &pngInfo_phase.data, sizeof(pngInfo_phase.data));
	ret = add_chunk_info(png_ptr, CHUNK_TOF_IRSENSOR, &pngInfo_phase.irSensor, sizeof(pngInfo_phase.irSensor));
	ret = add_chunk_info(png_ptr, CHUNK_OBPNGINFO, &pngInfo_phase.ob_info, sizeof(pngInfo_phase.ob_info));

	ret = write_chunk_end(png_ptr, info_ptr);
	ShowInfo(QString("png save success"));
	pngwrite_release(&png_ptr, &info_ptr);

	fclose(fb);
	return ret;
}
void CaptureDialog::csvDataArray(OniData &data, uint16_t *array, int num) {
	int index[4];
	int16_t* frameData = (int16_t*)data.oData.mDataPtr;
	int width = data.mTofExteraline.width;
	int height = data.mTofExteraline.height;
	//csv_item = QString("%1, %2").arg(rxTemp).arg(txTemp);
	for (int k = 0; k < num; k++) {
		for (int j = 0; j < num; j++) {
			int centerIndex_1 = width * (height / 2) + (width / 2);
			int centerIndex_2 = centerIndex_1 + width;
			int centerIndex_3 = centerIndex_2 + 1;
			int centerIndex_4 = centerIndex_1 + 1;

			index[0] = (centerIndex_1 - (num - 1) - ((num - 1)*width)) + (width)*k * 2 + 2 * j;
			index[1] = (centerIndex_2 - (num - 1) - ((num - 1)*width)) + (width)*k * 2 + 2 * j;
			index[2] = (centerIndex_3 - (num - 1) - ((num - 1)*width)) + (width)*k * 2 + 2 * j;
			index[3] = (centerIndex_4 - (num - 1) - ((num - 1)*width)) + (width)*k * 2 + 2 * j;
			array[0] = frameData[index[0]];
			array[1] = frameData[index[1]];
			array[2] = frameData[index[2]];
			array[3] = frameData[index[3]];
		}
	}
}

void CaptureDialog::closeEvent(QCloseEvent *event) {
	killTimer(mTimerId);
	mTimerId = -1;
	if (mCountTimer->isActive())
	{
		mCountTimer->stop();
	}
}

void CaptureDialog::onTimeoutTempControl()
{
	if (mTempControl.bFirstReadTempr)
	{
		return;
	}
	float rxT = mTempControl.mCurrentRxTempr;

	if ((rxT >= mTempControl.temp_min) && (rxT <= mTempControl.temp_max)) {
		//记录csv和采图
		if (!mCsvFile->isRecording())
		{
			int64_t  current_time = QDateTime::currentMSecsSinceEpoch();
			mCsvFile->startRecord(QString("%1").arg(current_time), -1);
			qDebug() << "start record csv. rxTemp: " << rxT;
		}
		if (rxT > (mTempControl.temp_step - mTempControl.temp_judge_interval) && rxT < (mTempControl.temp_step + mTempControl.temp_judge_interval)) {
			//采图
			QString capture_path = "./capture/Picture/temperature_" + QString::number(mTempControl.temp_step);
			if (!mCurFileDir.exists(capture_path)) {
				mCurFileDir.mkpath(capture_path);
			}
			path = capture_path;
			qDebug() << "start capture rxTemp:" << rxT << ", temp_step: " << mTempControl.temp_step;
			mSensorDevice->StartPhaseCapture(mTempControl.frameTotal, 0);
			//std::thread phase_save_thread(&CaptureDialog::SavePhaseThread, this);
			//phase_save_thread.detach();
			slotInputPhaseData();
			mTempControl.temp_step += mTempControl.temp_interval;
		}
	}
	else {
		if (mCsvFile->isRecording() &&
			(((mTempControl.temp_step == (mTempControl.temp_max + mTempControl.temp_interval)) && mTempControl.temp_interval > 0) ||
			((mTempControl.temp_step == (mTempControl.temp_min + mTempControl.temp_interval)) && mTempControl.temp_interval < 0))) {
			qDebug() << "stop record csv, rxt:" << rxT;
			mCsvFile->timeUp();
			if (mTempControl.temp_timer->isActive())
			{
				mTempControl.temp_timer->stop();
				qDebug() << "stop temp timer.";
			}

		}
	}

}
void CaptureDialog::onOneTimeFinish()
{
	mLogIndex = 0;
	path = ui->led_save_path->text();
	//qDebug() << "mCurrentCaptureTime " << mCurrentCaptureTime;
}