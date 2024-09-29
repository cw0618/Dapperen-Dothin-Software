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
#ifndef SRCEMPCONTROLEREMPCONTROLER_H
#define SRCEMPCONTROLEREMPCONTROLER_H

#include <QObject>
#include <QThread>
#include "3rd/tempcontrolsdk/include/TemperatureControl.h"

class TempControler : public QThread
{
    Q_OBJECT
public:
    TempControler();
    ~TempControler();

    bool init();
    bool openControler();
    bool closeControler();


    bool setTemp(float temp,int flag);//flag 1 ldmp, 2 ir , 3 water cool
    bool getTemp(float &temp,int flag);
  //  float getCurrentTemp(){return currentTemp;}

    void startTempControl(float tMax, float tMin, float tInterval);
    void stopTempControl();

signals:
    void tempSetFinished();
    void showLog(const char*);
    void getCurrentTemp(float temp, int flag);

public slots:
    void on_setNextTempState(bool needSet,int offset);


protected:
    void run();

private:
    TemperatureControl *doubleControler{nullptr}; // 双通道
    TemperatureControl *waterCoolControler{nullptr};

    float currentTemp;
    bool isAlive{false};
    float tempMax;
    float tempMin;
    float tempinterval;
    float dstTemp;
    float offsetTemp{0.0f};

    bool isNeedSet{false};

};

#endif // SRCEMPCONTROLEREMPCONTROLER_H
