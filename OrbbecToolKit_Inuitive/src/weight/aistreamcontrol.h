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
#ifndef AISTREAMCONTROL_H
#define AISTREAMCONTROL_H
#include <QWidget>

namespace Ui {
class AIStreamControl;
}

class AIStreamControl : public QWidget
{
    Q_OBJECT

public:
    explicit AIStreamControl(QWidget *parent = 0);
    ~AIStreamControl();
    const int kAiTypeChange = 1;
    const int kAiHeightChange = 2;
    const int kAi2DSkeleton = 0;
    const int kAi3DSkeleton = 1;
    const int kAiBodyMask = 2;
    const int kAiFloorInfo = 3;
    const int kAiBodyShape = 4;
	bool mWidgetExist = false;
    void addAIType(QString value);
    void AITypeClear();
    void setCurrentAIType(QString resolution);
    void setAITypeIndex(int index);
    int getCurrentAIType();
    void setAIStatueChange(bool change);
    void setBodyHeight(QString height);
    void setPlaneCenter(QString value);
    void setPlaneVector(QString value);

private slots:
    void OnAITypeChange(int index);
    void OnBodyHeightChange();
signals:
    void aiChangeCallback(int type, QString value);
private:
    Ui::AIStreamControl *ui;
    bool mAIStatus = true;
};

#endif // AISTREAMCONTROL_H
