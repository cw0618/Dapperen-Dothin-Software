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
#ifndef UVCCOLORWIDGET_H
#define UVCCOLORWIDGET_H

#include <QWidget>

namespace Ui {
class UVCColorWidget;
}

class UVCColorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit UVCColorWidget(QWidget *parent = 0);
    ~UVCColorWidget();
    const int kResolutionChange = 1;
    const int kDevicesChange = 2;
    void addDeviceList(QString device);
    void clearDeviceList();
    void addResolution(QString value);
    void resolutionClear();
    void setCurrentResolution(QString resolution);
    void setResolutionIndex(int index);
    int getCurrentResolution();
    QString getCurrentResolutionStr();
    int getCurrentDevices();
    void setResolutionChange(bool change);
    void setDevicesChange(bool change);
private slots:
    void OnUVCDevicesChange(int index);
    void OnResolutionChange(int index);
	
signals:
    void colorResolutionCallback(int type, QString value);
    void deviceChangeCallback(int type, QString value);
private:
    Ui::UVCColorWidget *ui;
    bool devices_change=true;
    bool resolution_change=true;
};

#endif // UVCCOLORWIDGET_H
