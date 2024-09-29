/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef PNGINFOWINDOW_H
#define PNGINFOWINDOW_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <src/device/imageutils.h>
#include <src/device/pngparser.h>
#include <src/device/obutils.h>

namespace Ui {
	class PngInfoWindow;
}

class PngInfoWindow : public QMainWindow
{
	Q_OBJECT
private:
	Ui::PngInfoWindow *ui;
	uint8_t mDataType{ 0 };

	QString title{ "Png Viewer" };

	QStandardItemModel* itemModel;

	QString pngFile{ "" };
	PngParser* pngParser;
	ImageUtils imageUtils;
	bool isSaveRaw{ false };

public:
	explicit PngInfoWindow(QWidget *parent = 0);
	~PngInfoWindow();


	void setPngFile(const QString &value);

private:
	void showDeviceInfo(PNG_FILE* png_file);
	void showDriverInfo(PNG_FILE* png_file);
	void showEnvInfo(PNG_FILE* png_file);
	void showDataInof(PNG_FILE* png_file);

	void showImage(PNG_FILE* png_file);

signals:
	void pngInfoWindowClose(void *handle);
	private slots:
	void onPngParseFinish(int error);

	// QWidget interface
protected:
	void closeEvent(QCloseEvent *event);
};

#endif // PNGINFOWINDOW_H
