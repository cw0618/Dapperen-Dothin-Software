#include "src\tempcontroler\tempcontroler.h"
#include <QDebug>
/** \class TempControler
*
* 温漂逻辑控制类
*
*/
TempControler::TempControler()
{
    doubleControler = new TemperatureControl(0);
    waterCoolControler = new TemperatureControl(1);

}

TempControler::~TempControler()
{
     stopTempControl();
     delete doubleControler;
     delete waterCoolControler;
}

bool TempControler::init()
{
    bool ret =false;
    SerialParams params;
//    params.portName = "COM7";
//    params.baudRate = 115200;

//    ret = doubleControler->initSerialPort(params);

    params.portName = "COM7";
    params.baudRate = 115200;

    ret = waterCoolControler->initSerialPort(params);

    if(!openControler()){
        qDebug() << "使能温控仪失败";
        ret = false;
    }

    return ret;

}

bool TempControler::openControler()
{
    bool ret =false;

     ret = waterCoolControler->startTempControl();

     return ret;
}

bool TempControler::closeControler()
{
    bool ret =false;

     ret = waterCoolControler->stopTempControl();

     return ret;
}

bool TempControler::setTemp(float temp,int flag)
{
    bool ret =false;
    if(flag != 3){
         ret = doubleControler->setTemperature(temp,flag);
    }else{
         ret = waterCoolControler->setTemperature(temp,1);
    }




    return ret;
}

bool TempControler::getTemp(float &temp,int flag)
{
    bool ret =false;
    if(flag !=3){
        ret =  doubleControler->getTemperature(temp,flag);
    }else{
        ret =  waterCoolControler->getTemperature(temp,1);
    }



    return ret;
}

void TempControler::startTempControl(float tMax, float tMin, float tInterval)
{
    tempMax = tMax;
    tempMin  = tMin;
    tempinterval = tInterval;
    dstTemp = tInterval > 0 ? tempMin : tempMax;
    isNeedSet =true;

    if(!isAlive){
        isAlive = true;
        start();

    }

}

void TempControler::stopTempControl()
{
    
	if (isAlive)
	{
		closeControler();
		if (isRunning()) {
			requestInterruption();
		}
		quit();
		wait();
		isAlive = false;
	}
}

void TempControler::on_setNextTempState(bool needSet, int offset)
{
    isNeedSet = needSet;
    offsetTemp = offset;
    if(isNeedSet){
        dstTemp += tempinterval;
        offsetTemp = 0;
    }
}

void TempControler::run()
{
    bool controlRet = false;
    qDebug() << "start TempControler run";
    while(isAlive && !isInterruptionRequested()){

        if(isNeedSet ||  qAbs(offsetTemp) > 0.00001f)
        {
          controlRet = waterCoolControler->setTemperature(dstTemp + offsetTemp,1);
          isNeedSet = false;
          offsetTemp = 0;
          msleep(500);
        }else{
            controlRet = true;
        }

        if(controlRet){
            controlRet = waterCoolControler->getTemperature(currentTemp,1);
        }
        if(controlRet){
             emit getCurrentTemp(currentTemp,0);
            if(currentTemp >= (dstTemp + offsetTemp - 0.5f) && currentTemp <= (dstTemp + offsetTemp + 0.5f))
            {

                qDebug() << (QString("set dst temp finished,dstTemp: %1,offsetTemp:%2, currentTemp:%3")
                             .arg(dstTemp).arg(offsetTemp).arg(currentTemp).toLocal8Bit());
                 emit tempSetFinished();
            }
        }
        msleep(100);
    }
     qDebug() << "TempControler run end";
}
