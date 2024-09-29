#include "pnginfowindow.h"
#include "ui_pnginfowindow.h"

#include <QDebug>
#include <QFileInfo>
#include <QStandardItemModel>
/** \class CaptureDialog
*
* png图片显示窗口类
*
*/

PngInfoWindow::PngInfoWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::PngInfoWindow)
{
	ui->setupUi(this);
	setWindowTitle(title);

	pngParser = new PngParser(this);
	connect(pngParser, SIGNAL(pngParseFinish(int)), this, SLOT(onPngParseFinish(int)));


	itemModel = new QStandardItemModel(0, 2);
	ui->tableView->setModel(itemModel);
	ui->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

	//    QStringList column;
	//    column << "字段" << "值" ;
	//    itemModel->setHorizontalHeaderLabels(column);
	ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

	ui->tableView->horizontalHeader()->hide();                       // 隐藏水平表头
	ui->tableView->verticalHeader()->hide();

	//    ui->tableView->setShowGrid(false);                               // 隐藏网格线
	//    ui->tableView->setFocusPolicy(Qt::NoFocus);                      // 去除当前Cell周边虚线框
	ui->tableView->setAlternatingRowColors(true);                    // 开启隔行异色
}

PngInfoWindow::~PngInfoWindow()
{
	delete ui;
	if (pngParser != NULL) {
		delete pngParser;
		pngParser = NULL;
	}
}


void PngInfoWindow::setPngFile(const QString &value)
{
	pngFile = value;

	QFileInfo file(pngFile);
	if (file.isFile()) {
		itemModel->clear();
		title = file.fileName();
		pngParser->parsePng(pngFile);
		setWindowTitle(title);
	}
}

void PngInfoWindow::onPngParseFinish(int error)
{
	if (error >= 0) {
		PNG_FILE* png_file = pngParser->getPng_file();

		TOF_DATA_INFO dataInfo;
		int ret = get_chunk_info(png_file, CHUNK_TOF_DATA, &dataInfo, sizeof(dataInfo));
		if (ret != 0) {
			return;
		}
		else {
			mDataType = dataInfo.data_type;
		}

		showDeviceInfo(png_file);
		showDataInof(png_file);
		showDriverInfo(png_file);
		showEnvInfo(png_file);

		for (int i = 0; i < itemModel->rowCount(); i++) {
			itemModel->item(i, 1)->setTextAlignment(Qt::AlignCenter);
		}

		showImage(png_file);

	}
	else {
		qDebug() << "on_pngParseFinish-----error:" << error;
	}
}

void PngInfoWindow::showDeviceInfo(PNG_FILE* png_file)
{

	DEVICE_INFO devInfo;
	int ret = get_chunk_info(png_file, CHUNK_DEVICE, &devInfo, sizeof(devInfo));
	if (ret == 0) {
		QList<QStandardItem *> lsi;
		lsi.append(new QStandardItem("device"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.device.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("ir_model"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.ir_model.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("ir_sensor"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.ir_sensor.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("ldmp_model"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.ldmp_model.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("dev_serial"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.dev_serial.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("ir_serial"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.ir_serial.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("ldmp_serial"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.ldmp_serial.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("depth_engine"));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(devInfo.depth_engine.name))));
		itemModel->appendRow(lsi);


		QString model_type;
		if (devInfo.model_type == MODEL_TOF) {
			model_type = "MODEL_TOF";
		}
		else if (devInfo.model_type == MODEL_STEREO_PASSIVE) {
			model_type = "MODEL_STEREO_PASSIVE";
		}
		else if (devInfo.model_type == MODEL_STEREO_ACTIVE) {
			model_type = "MODEL_STEREO_ACTIVE";
		}
		else if (devInfo.model_type == MODEL_MONO_STRUCTURE_LIGHT) {
			model_type = "MODEL_MONO_STRUCTURE_LIGHT";
		}

		lsi.clear();
		lsi.append(new QStandardItem("model_type"));
		lsi.append(new QStandardItem(model_type));
		itemModel->appendRow(lsi);
	}
}

void PngInfoWindow::showDriverInfo(PNG_FILE* png_file)
{
	TOF_DRIVER_INFO drvInfo;
	int ret = get_chunk_info(png_file, CHUNK_TOF_DRIVER, &drvInfo, sizeof(drvInfo));
	if (ret == 0) {
		QList<QStandardItem *> lsi;
		lsi.append(new QStandardItem(tr("驱动库版本")));
		lsi.append(new QStandardItem(QString::fromUtf8(reinterpret_cast<char *>(drvInfo.driver_version.name))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("驱动IC")));
		QString itemStr;
		//4016:cxa4016    5016:PHX3D
		if (drvInfo.vcsel_driver_ic == 4016) {
			itemStr = "CXA4016";
		}
		else if (drvInfo.vcsel_driver_ic == 5016) {
			itemStr = "PHX3D_3021_AA";
		}
		else if (drvInfo.vcsel_driver_ic == 5017) {
			itemStr = "PHX3D_3021_CB";
		}
		else {
			itemStr = QString("%1").arg(drvInfo.vcsel_driver_ic);
		}
		lsi.append(new QStandardItem(itemStr));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("电流(mA)")));
		lsi.append(new QStandardItem(QString("%1").arg(drvInfo.illum_power)));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("激光工作模式")));

		if (drvInfo.tx_a_b_power == 0) {
			itemStr = "单区";
		}
		else if (drvInfo.tx_a_b_power == 1) {
			itemStr = "A区";
		}
		else if (drvInfo.tx_a_b_power == 2) {
			itemStr = "B区";
		}
		else if (drvInfo.tx_a_b_power == 3) {
			itemStr = "A+B区";
		}
		lsi.append(new QStandardItem(itemStr));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("输出模式")));
		if (drvInfo.output_mode == 1) {
			lsi.append(new QStandardItem(QString("S5K_SINGLE_1FRAME")));

		}
		else if (drvInfo.output_mode == 9) {
			lsi.append(new QStandardItem(QString("S5K_DUAL_2FRAME")));

		}
		else if (drvInfo.output_mode == 100) {
			lsi.append(new QStandardItem(QString("PLECO_SINGLE_3FRAME")));

		}
		else if (drvInfo.output_mode == 101) {
			lsi.append(new QStandardItem(QString("PLECO_DUAL_6FRAME")));

		}
		else {
			lsi.append(new QStandardItem(QString("%1").arg(drvInfo.output_mode)));
		}
		itemModel->appendRow(lsi);


		if (mDataType == PHASE_MAP || mDataType == AMPLITUDE_MAP) {

			lsi.clear();
			lsi.append(new QStandardItem(tr("频率(MHZ)")));
			lsi.append(new QStandardItem(QString("%1").arg(drvInfo.frequency)));
			itemModel->appendRow(lsi);
		}

		if (mDataType == PHASE_MAP) {

			lsi.clear();
			lsi.append(new QStandardItem(tr("相位")));
			if (drvInfo.phase == 0x1B) {
				lsi.append(new QStandardItem(QString("SHUFFLE")));

			}
			else if (drvInfo.phase == 0x10) {
				lsi.append(new QStandardItem(QString("NON-SHUFFLE")));

			}
			else {
				lsi.append(new QStandardItem(QString("%1").arg(drvInfo.phase)));
			}
			itemModel->appendRow(lsi);


			lsi.clear();
			lsi.append(new QStandardItem(tr("占空比")));
			lsi.append(new QStandardItem(QString("%1%").arg(drvInfo.duty_cycle)));
			itemModel->appendRow(lsi);

			lsi.clear();
			lsi.append(new QStandardItem(tr("积分时间(us)")));
			lsi.append(new QStandardItem(QString("%1").arg(drvInfo.integration_time)));
			itemModel->appendRow(lsi);

			lsi.clear();
			lsi.append(new QStandardItem(tr("激光器数量")));
			lsi.append(new QStandardItem(QString("%1").arg(drvInfo.vcsel_count)));
			itemModel->appendRow(lsi);
		}
	}
}

void PngInfoWindow::showEnvInfo(PNG_FILE* png_file)
{
	TOF_ENV_INFO envInfo;
	int ret = get_chunk_info(png_file, CHUNK_TOF_ENV, &envInfo, sizeof(envInfo));
	if (ret == 0) {
		QList<QStandardItem *> lsi;
		lsi.append(new QStandardItem(tr("ir温度")));
		lsi.append(new QStandardItem(QString("%1(%2℃)").arg(envInfo.ir_temp).arg(ObUtils::FixPoint2Temperature(envInfo.ir_temp))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("vcsel温度")));
		lsi.append(new QStandardItem(QString("%1(%2℃)").arg(envInfo.vcsel_temp).arg(ObUtils::FixPoint2Temperature(envInfo.vcsel_temp))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("dll温度")));
		lsi.append(new QStandardItem(QString("%1(%2℃)").arg(envInfo.dll_temp).arg(ObUtils::FixPoint2Temperature(envInfo.dll_temp))));
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem("distance(mm)"));
		lsi.append(new QStandardItem(QString("%1").arg(envInfo.distance)));
		itemModel->appendRow(lsi);
	}
}

void PngInfoWindow::showDataInof(PNG_FILE* png_file)
{
	TOF_DATA_INFO dataInfo;
	int ret = get_chunk_info(png_file, CHUNK_TOF_DATA, &dataInfo, sizeof(dataInfo));
	if (ret == 0) {
		QList<QStandardItem *> lsi;

		lsi.append(new QStandardItem(tr("数据类型")));
		QStandardItem *dataTypeItem;
		if (dataInfo.data_type == PHASE_MAP) {
			dataTypeItem = new QStandardItem(QString(tr("相位图")));

		}
		else if (dataInfo.data_type == DEPTH_MAP) {
			dataTypeItem = new QStandardItem(QString(tr("深度图")));


		}
		else if (dataInfo.data_type == AMPLITUDE_MAP) {
			dataTypeItem = new QStandardItem(QString(tr("幅值图")));


		}
		else if (dataInfo.data_type == IR_MAP) {
			dataTypeItem = new QStandardItem(QString(tr("ir图")));


		}
		else {
			dataTypeItem = new QStandardItem(QString("unknow"));

		}
		lsi.append(dataTypeItem);
		itemModel->appendRow(lsi);

		lsi.clear();
		lsi.append(new QStandardItem(tr("分辨率")));
		lsi.append(new QStandardItem(QString("%1x%2").arg(dataInfo.width).arg(dataInfo.height)));
		itemModel->appendRow(lsi);

		//        lsi.clear();
		//        lsi.append(new QStandardItem("数据比特数"));
		//        lsi.append(new QStandardItem(QString("%1").arg(dataInfo.data_bits)));
		//        itemModel->appendRow(lsi);

		//        if(data_type == DEPTH_MAP){
		//            lsi.clear();
		//            lsi.append(new QStandardItem("深度单位"));
		//            lsi.append(new QStandardItem(QString("%1um").arg(dataInfo.depth_unit)));
		//            itemModel->appendRow(lsi);
		//        }

	}
}

void PngInfoWindow::showImage(PNG_FILE* png_file)
{
	PNG_IMAGE* png_image = &png_file->image;
	if (png_image != NULL && png_image->data != NULL) {

		TOF_DATA_INFO tofData;
		get_chunk_info(png_file, CHUNK_TOF_DATA, &tofData, sizeof(tofData));


		TOF_DRIVER_INFO drvInfo;
		get_chunk_info(png_file, CHUNK_TOF_DRIVER, &drvInfo, sizeof(drvInfo));

		int raw16_type = 0;
		/*  if(drvInfo.output_mode == OB_TOF_OUT_MODE_A_B){
			  raw16_type = 1;
		  }*/


		BufBean *rgbBean = NULL;
		if (tofData.data_type == PHASE_MAP || tofData.data_type == AMPLITUDE_MAP || tofData.data_type == IR_MAP) {
			imageUtils.grayRGB((uint16_t *)png_image->data, png_image->width, png_image->height, 2);
			rgbBean = imageUtils.getRgbBean();

		}
		else if (tofData.data_type == DEPTH_MAP) {
			imageUtils.histogramRGB((uint16_t *)png_image->data, png_image->width, png_image->height, 0);
			rgbBean = imageUtils.getRgbBean();

		}

		if (rgbBean != NULL && rgbBean->valid) {
			/*  ui->renderer->updateTextureBufferData((uint8_t *)rgbBean->data, rgbBean->w, rgbBean->h, rgbBean->pointSize);
			  ui->renderer->setRaw16((uint16_t *)png_image->data, rgbBean->w * rgbBean->h * 2, raw16_type);
			  ui->renderer->update();*/
			QString mouse_info;
			QPoint mp = ui->renderer->GetMousePos();
			if (mp.x() >= 0 && mp.y() >= 0 && mp.x() < png_image->width && mp.y() < png_image->height)
			{
				uint32_t index = mp.y() * png_image->width + mp.x();


				uint16_t IR_8_bit = 0;
				uint16_t IR_16_bit = 0;
				if (tofData.data_type == DEPTH_MAP) {
					IR_16_bit = reinterpret_cast<uint16_t*>(png_image->data)[index];
				}
				else {
					if (nullptr != rgbBean->data) {
						IR_8_bit = (rgbBean->data)[index * 3];
					}

				}



				//mouse_info = QString("IR(x:%1 y:%2) 8bit:%3 16bit:%4 resolution %5x%6").arg(mp.x()).arg(mp.y()).arg(IR_8_bit).arg(IR_16_bit).arg(data->frame_index).arg(data->frame_time_stamp);
				mouse_info = QString("IR(x:%1 y:%2) 8bit:%3 16bit:%4 resolution %5x%6").arg(mp.x()).arg(mp.y()).arg(IR_8_bit).arg(IR_16_bit).arg(png_image->width).arg(png_image->height);
			}


			TextInfo ti_mouse_info;
			ti_mouse_info.penColor = Qt::green;
			ti_mouse_info.text = mouse_info;
			ui->renderer->updateTextureBufferData((int8_t*)rgbBean->data, png_image->width, png_image->height, 3 * png_image->width* png_image->height);
			try
			{
				ui->renderer->UpdateMouseInfo(ti_mouse_info);
			}
			catch (const std::exception&)
			{
				qDebug() << "howard ir UpdateMouseInfo error";
			}




		}
#if 0
		if (isSaveRaw) {
			QByteArray ba = (pngFile.replace(".png", ".raw")).toLocal8Bit();
			const char* save_png_file = ba.data();

			FILE* fb = fopen(save_png_file, "wb");
			if (fb == NULL) {
				qWarning() << "PngCreater run: open file failed:" << (pngFile + ".raw");
				//return -3;
			}
			fwrite(png_image->data, sizeof(char), (2 * png_image->width*png_image->height), fb);
			fclose(fb);
		}
#endif
	}
}

void PngInfoWindow::closeEvent(QCloseEvent *event)
{
	emit pngInfoWindowClose(this);
}
